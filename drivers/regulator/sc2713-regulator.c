/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/adi.h>
#include <mach/adc.h>

/*
#define CONFIG_REGULATOR_CAL_DEBUG
#define CONFIG_REGULATOR_ADC_DEBUG
*/

#undef debug
#define debug(format, arg...) pr_info("regu: " "@@@%s: " format, __func__, ## arg)
#define debug0(format, arg...) pr_debug("regu: " "@@@%s: " format, __func__, ## arg)
#define debug2(format, arg...) pr_debug("regu: " "@@@%s: " format, __func__, ## arg)

#ifndef	ANA_REG_OR
#define	ANA_REG_OR(_r, _b)	sci_adi_write(_r, _b, 0)
#endif

#ifndef	ANA_REG_BIC
#define	ANA_REG_BIC(_r, _b)	sci_adi_write(_r, 0, _b)
#endif

#ifndef	ANA_REG_GET
#define	ANA_REG_GET(_r)		sci_adi_read(_r)
#endif

#ifndef	ANA_REG_SET
#define	ANA_REG_SET(_r, _v, _m)	sci_adi_write(_r, _v, _m)
#endif

struct sci_regulator_regs {
	int typ;
	u32 pd_set, pd_set_bit;
	u32 pd_rst, pd_rst_bit;
	u32 slp_ctl, slp_ctl_bit;
	u32 vol_trm, vol_trm_bits;
	u32 cal_ctl, cal_ctl_bits;
	u32 vol_def;
	u32 vol_ctl, vol_ctl_bits;
	u32 vol_sel_cnt, vol_sel[];
};

/**
 * struct sci_regulator_ops - sci regulator operations.
 *
 * @trimming:
 *
 * This struct describes regulator operations which can be implemented by
 * regulator chip drivers.
 */
struct sci_regulator_ops {
	int (*init_trimming) (struct regulator_dev * rdev);
	int (*is_trimming) (struct regulator_dev * rdev);
	int (*get_trimming_step) (struct regulator_dev * rdev, int);
	int (*set_trimming) (struct regulator_dev * rdev, int, int);
	int (*calibrate) (struct regulator_dev * rdev, int, int);
};

struct sci_regulator_data {
	struct delayed_work dwork;
	struct regulator_dev *rdev;
};

struct sci_regulator_desc {
	struct regulator_desc desc;
	struct sci_regulator_ops *ops;
	const struct sci_regulator_regs *regs;
	struct sci_regulator_data data;	/* FIXME: dynamic */
#if defined(CONFIG_DEBUG_FS)
	struct dentry *debugfs;
#endif
};

enum {
	VDD_TYP_LDO = 0,
	VDD_TYP_LDO_D = 1,
	VDD_TYP_DCDC = 2,
};

static int __regu_calibrate(struct regulator_dev *, int, int);

#define SCI_REGU_REG(VDD, TYP, PD_SET, SET_BIT, PD_RST, RST_BIT, SLP_CTL, SLP_CTL_BIT, \
                     VOL_TRM, VOL_TRM_BITS, CAL_CTL, CAL_CTL_BITS, VOL_DEF, \
                     VOL_CTL, VOL_CTL_BITS, VOL_SEL_CNT, ...)   \
do { 														\
	static const struct sci_regulator_regs REGS_##VDD = {	\
		.typ		= TYP,									\
		.pd_set = PD_SET,                           		\
		.pd_set_bit = SET_BIT,                      		\
		.pd_rst = PD_RST,                           		\
		.pd_rst_bit = RST_BIT,                      		\
		.slp_ctl = SLP_CTL,                         		\
		.slp_ctl_bit = SLP_CTL_BIT,                 		\
		.vol_trm = VOL_TRM,                                 \
		.vol_trm_bits = VOL_TRM_BITS,                       \
		.cal_ctl = CAL_CTL, 								\
		.cal_ctl_bits = CAL_CTL_BITS,						\
		.vol_def = VOL_DEF,									\
		.vol_ctl = VOL_CTL,                         		\
		.vol_ctl_bits = VOL_CTL_BITS,               		\
		.vol_sel_cnt = VOL_SEL_CNT,                 		\
		.vol_sel = {__VA_ARGS__},                   		\
	};														\
	static struct sci_regulator_desc DESC_##VDD = {			\
		.desc.name = #VDD,									\
		.desc.id = 0,										\
		.desc.ops = 0,										\
		.desc.type = REGULATOR_VOLTAGE,						\
		.desc.owner = THIS_MODULE,							\
		.regs = &REGS_##VDD,								\
	};														\
	sci_regulator_register(pdev, &DESC_##VDD);				\
} while (0)

static struct sci_regulator_desc *__get_desc(struct regulator_dev *rdev)
{
	return (struct sci_regulator_desc *)rdev->desc;
}

/* standard ldo ops*/
static int ldo_turn_on(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;

	debug0("regu %p (%s), set %08x[%d], rst %08x[%d]\n", regs,
	       desc->desc.name, regs->pd_set, __ffs(regs->pd_set_bit),
	       regs->pd_rst, __ffs(regs->pd_rst_bit));

	if (regs->pd_rst)
		ANA_REG_OR(regs->pd_rst, regs->pd_rst_bit);

	if (regs->pd_set)
		ANA_REG_BIC(regs->pd_set, regs->pd_set_bit);

	debug2("regu %p (%s), turn on\n", regs, desc->desc.name);
	/* ldo trimming when first turn on */
	if (regs->vol_trm && !desc->ops->is_trimming(rdev)) {
		__regu_calibrate(rdev, 0, 0);
	}
	return 0;
}

static int ldo_turn_off(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;

	debug0("regu %p (%s), set %08x[%d], rst %08x[%d]\n", regs,
	       desc->desc.name, regs->pd_set, __ffs(regs->pd_set_bit),
	       regs->pd_rst, __ffs(regs->pd_rst_bit));
#if !defined(CONFIG_REGULATOR_CAL_DEBUG)
	if (regs->pd_set)
		ANA_REG_OR(regs->pd_set, regs->pd_set_bit);

	if (regs->pd_rst)
		ANA_REG_BIC(regs->pd_rst, regs->pd_rst_bit);
#endif
	debug2("regu %p (%s), turn off\n", regs, desc->desc.name);
	return 0;
}

static int ldo_is_on(struct regulator_dev *rdev)
{
	int ret = -EINVAL;
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;

	debug0("regu %p (%s), set %08x[%d], rst %08x[%d]\n", regs,
	       desc->desc.name, regs->pd_set, __ffs(regs->pd_set_bit),
	       regs->pd_rst, __ffs(regs->pd_rst_bit));

	if (regs->pd_rst && regs->pd_set) {
		ret = ! !(ANA_REG_GET(regs->pd_rst) & regs->pd_rst_bit);
		if (ret == ! !(ANA_REG_GET(regs->pd_set) & regs->pd_set_bit))
			ret = -EINVAL;
	} else if (regs->pd_set) {	/* new feature */
		ret = !(ANA_REG_GET(regs->pd_set) & regs->pd_set_bit);
	}

	debug2("regu %p (%s) return %d\n", regs, desc->desc.name, ret);
	return ret;
}

static int ldo_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;

	debug("regu %p (%s), slp %08x[%d] mode %x\n", regs, desc->desc.name,
	      regs->slp_ctl, regs->slp_ctl_bit, mode);

	if (!regs->slp_ctl)
		return -EINVAL;

	if (mode == REGULATOR_MODE_STANDBY) {	/* disable auto slp */
		ANA_REG_BIC(regs->slp_ctl, regs->slp_ctl_bit);
	} else {
		ANA_REG_OR(regs->slp_ctl, regs->slp_ctl_bit);
	}
	return 0;
}

static int ldo_set_voltage(struct regulator_dev *rdev, int min_uV,
			   int max_uV, unsigned *selector)
{
	static const int vol_bits[4] = { 0xa, 0x9, 0x6, 0x5 };
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;
	int mv = min_uV / 1000;
	int ret = -EINVAL;
	int i, shft = __ffs(regs->vol_ctl_bits);
	BUG_ON(regs->vol_sel_cnt > 4);
	debug("regu %p (%s) %d %d\n", regs, desc->desc.name, min_uV, max_uV);

	if (!regs->vol_ctl)
		return -EACCES;
	for (i = 0; i < regs->vol_sel_cnt; i++) {
		if (regs->vol_sel[i] == mv) {
			ANA_REG_SET(regs->vol_ctl, i << shft, regs->vol_ctl_bits);
			/*clear_bit(desc->desc.id, trimming_state); */
			ret = 0;
			break;
		}
	}

	WARN(0 != ret,
	     "warning: regulator (%s) not support %dmV\n", desc->desc.name, mv);
	return ret;
}

static int ldo_get_voltage(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;
	u32 vol, vol_bits;
	int i, shft = __ffs(regs->vol_ctl_bits);

	debug0("regu %p (%s), vol ctl %08x, shft %d, mask %08x\n",
	       regs, desc->desc.name, regs->vol_ctl, shft, regs->vol_ctl_bits);

	if (!regs->vol_ctl)
		return -EACCES;

	BUG_ON(regs->vol_sel_cnt != 4);
	i = vol_bits = ((ANA_REG_GET(regs->vol_ctl) & regs->vol_ctl_bits) >> shft);
	vol = regs->vol_sel[i];
	debug2("regu %p (%s), voltage %d\n", regs, desc->desc.name, vol);
	return vol * 1000;

	if ((vol_bits & BIT(0)) ^ (vol_bits & BIT(1))
	    && (vol_bits & BIT(2)) ^ (vol_bits & BIT(3))) {
		i = (vol_bits & BIT(0)) | ((vol_bits >> 1) & BIT(1));
		vol = regs->vol_sel[i];
		debug2("regu %p (%s), voltage %d\n", regs, desc->desc.name,
		       vol);
		return vol * 1000;
	}
	return -EFAULT;
}

static unsigned long trimming_state[2] = { 0, 0 };	/* max 64 bits */

static int __is_trimming(struct regulator_dev *rdev)
{
	int id;
	BUG_ON(!rdev);
	id = rdev->desc->id;
	BUG_ON(!(id > 0 && id < sizeof(trimming_state) * 8));
	return test_bit(id, trimming_state);
}

static int ldo_init_trimming(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;
	int ret = -EINVAL;
	int shft = __ffs(regs->vol_trm_bits);
	u32 trim;

	if (!regs->vol_trm)
		goto exit;

	trim = (ANA_REG_GET(regs->vol_trm) & regs->vol_trm_bits) >> shft;
	if (trim != 0x10 /* 100 % */ ) {
		debug("regu %p (%s) trimming ok\n", regs, desc->desc.name);
		set_bit(desc->desc.id, trimming_state);
		ret = trim;
	} else if (1 || 1 == ldo_is_on(rdev)) {	/* some LDOs had been turned in uboot-spl */
		//ret = ldo_turn_on(rdev);
		__regu_calibrate(rdev, 0, 0);
	}

exit:
	return ret;
}

/**
 * ldo trimming step about 0.7%, range 90% ~ 110%. that all maps as follow.
	0x00 : 90.000
	0x01 : 90.625
	0x02 : 91.250
	0x03 : 91.875
	0x04 : 92.500
	0x05 : 93.125
	0x06 : 93.750
	0x07 : 94.375
	0x08 : 95.000
	0x09 : 95.625
	0x0A : 96.250
	0x0B : 96.875
	0x0C : 97.500
	0x0D : 98.125
	0x0E : 98.750
	0x0F : 99.375
	0x10 : 100.000
	0x11 : 100.625
	0x12 : 101.250
	0x13 : 101.875
	0x14 : 102.500
	0x15 : 103.125
	0x16 : 103.750
	0x17 : 104.375
	0x18 : 105.000
	0x19 : 105.625
	0x1A : 106.250
	0x1B : 106.875
	0x1C : 107.500
	0x1D : 108.125
	0x1E : 108.750
	0x1F : 109.375

	0x20 : 110.000
 */
static int ldo_set_trimming(struct regulator_dev *rdev, int ctl_vol, int to_vol)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;
	int ret = -EINVAL, cal_vol;

	ctl_vol /= 1000;
	to_vol /= 1000;

	cal_vol = ctl_vol - to_vol * 90 / 100;	/* cal range 90% ~ 110% */
	if (!regs->vol_trm || cal_vol < 0 || cal_vol >= to_vol * 20 / 100)
		goto exit;

	/* always update voltage ctrl bits */
	ret =
	    rdev->desc->ops->set_voltage(rdev, to_vol * 1000, to_vol * 1000, 0);
	if (IS_ERR_VALUE(ret) && regs->vol_ctl)
		goto exit;

	else {
		u32 trim =	/* assert 5 valid trim bits */
		    (cal_vol * 100 * 32) / (to_vol * 20) & 0x1f;
		debug
		    ("regu %p (%s) trimming %u = %u %+dmv, got [%02X] %u.%03u%%\n",
		     regs, desc->desc.name, ctl_vol, to_vol,
		     (cal_vol - to_vol / 10), trim, ctl_vol * 100 / to_vol,
		     (ctl_vol * 100 * 1000 / to_vol) % 1000);
#if !defined(CONFIG_REGULATOR_CAL_DEBUG)
		ANA_REG_SET(regs->vol_trm, trim << __ffs(regs->vol_trm_bits),
			    regs->vol_trm_bits);
		ret = 0;
#endif
	}

exit:
	return ret;
}

static int ldo_get_trimming_step(struct regulator_dev *rdev, int def_vol)
{
	return def_vol * 7 / 1000;	/*uV */
}

/* standard dcdc ops*/
static int dcdc_get_trimming_step(struct regulator_dev *rdev, int def_vol)
{
	return 1000 * 100 / 32;	/*uV */
}

static int dcdc_set_trimming(struct regulator_dev *rdev, int ctl_vol,
			     int to_vol)
{
	return rdev->desc->ops->set_voltage(rdev, ctl_vol, ctl_vol, 0);
}

static int __match_dcdc_vol(const struct sci_regulator_regs *regs, u32 vol)
{
	int i, j = -1;
	int ds, min_ds = 100;	/* mV, the max range of small voltage */
	for (i = 0; i < regs->vol_sel_cnt; i++) {
		ds = vol - regs->vol_sel[i];
		if (ds >= 0 && ds < min_ds) {
			min_ds = ds;
			j = i;
		}
	}
	return j;
}

static int dcdc_set_voltage(struct regulator_dev *rdev, int min_uV,
			    int max_uV, unsigned *selector)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;
	int i, mv = min_uV / 1000;

	debug0("regu %p (%s) %d %d\n", regs, desc->desc.name, min_uV, max_uV);

	BUG_ON(0 != __ffs(regs->vol_trm_bits));
	BUG_ON(regs->vol_sel_cnt > 8);

	if (!regs->vol_ctl)
		return -EACCES;

	/* found the closely vol ctrl bits */
	i = __match_dcdc_vol(regs, mv);
	if (i < 0)
		return -EINVAL;

	debug("regu %p (%s) %d = %d %+dmv\n", regs, desc->desc.name,
	      mv, regs->vol_sel[i], mv - regs->vol_sel[i]);

#if !defined(CONFIG_REGULATOR_CAL_DEBUG)
	/* dcdc calibration control bits (default 00000),
	 * small adjust voltage: 100/32mv ~= 3.125mv
	 */
	{
		int shft = __ffs(regs->vol_ctl_bits);
		int max = regs->vol_ctl_bits >> shft;
		int j = ((mv - regs->vol_sel[i]) * 32) / (100) % 32;

		if (regs->vol_trm == regs->vol_ctl) {	/* new feature */
			ANA_REG_SET(regs->vol_ctl, j | (i << shft),
				    regs->vol_trm_bits | regs->vol_ctl_bits);
		} else {
			if (regs->vol_trm) {	/* small adjust first */
#define BITS_DCDC_CAL_RST(_x_)     ( (_x_) << 5 & (BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)) )
#define BITS_DCDC_CAL(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)) )
				ANA_REG_SET(regs->vol_trm,
					    BITS_DCDC_CAL(j) |
					    BITS_DCDC_CAL_RST(BITS_DCDC_CAL(-1)
							      - j), -1);
			}

			ANA_REG_SET(regs->vol_ctl, i | (max - i) << 4, -1);
		}
	}
#endif

	return 0;
}

static int dcdc_get_voltage(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc =
	    (struct sci_regulator_desc *)rdev->desc;
	const struct sci_regulator_regs *regs = desc->regs;
	u32 mv;
	int cal = 0;		/* mV */
	int i, shft = __ffs(regs->vol_ctl_bits);

	debug0("regu %p (%s), vol ctl %08x, shft %d, mask %08x, sel %d\n",
	       regs, desc->desc.name, regs->vol_ctl,
	       shft, regs->vol_ctl_bits, regs->vol_sel_cnt);

	if (!regs->vol_ctl)
		return -EINVAL;

	BUG_ON(0 != __ffs(regs->vol_trm_bits));
	BUG_ON(regs->vol_sel_cnt > 8);

	i = (ANA_REG_GET(regs->vol_ctl) & regs->vol_ctl_bits) >> shft;

	mv = regs->vol_sel[i];

	if (regs->vol_trm && regs->vol_trm != regs->vol_ctl) {
		/*check the reset relative bit of vol ctl */
		u32 vol_bits =
		    (~ANA_REG_GET(regs->vol_ctl) & (regs->vol_ctl_bits << 4)) >>
		    4;

		if (i != vol_bits)
			return -EFAULT;

		cal = (ANA_REG_GET(regs->vol_trm) & regs->vol_trm_bits)
		    * desc->ops->get_trimming_step(rdev, mv) / 1000;
	}

	debug("regu %p (%s) %d +%dmv\n", regs, desc->desc.name, mv, cal);
	return (mv + cal) * 1000;
}

static int dcdc_init_trimming(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;
	int ret = -EINVAL;

	if (!regs->vol_trm || !regs->vol_def)
		goto exit;

	__regu_calibrate(rdev, 0, 0);
	return 0;

exit:
	return ret;
}

static short adc_data[2][2]
#if 0
    = {
	{4100, 3196},		/* same as nv adc_t */
	{3633, 2807},
}
#endif
;

static int __is_valid_adc_cal(void)
{
	return 0 != adc_data[0][0];
}

static int __init __adc_cal_setup(char *str)
{
	u32 *p = (u32 *) adc_data;
	*p = simple_strtoul(str, &str, 0);
	if (*p++ && *++str) {
		*p = simple_strtoul(str, &str, 0);
		if (*p) {
			/* update adc data from kernel parameter */
			debug("%d : %d -- %d : %d\n",
			      (int)adc_data[0][0], (int)adc_data[0][1],
			      (int)adc_data[1][0], (int)adc_data[1][1]);
		}
	}
	return 1;
}

__setup("adc_cal=", __adc_cal_setup);

static int __adc2vbat(int adc_res)
{
	int t = adc_data[0][0] - adc_data[1][0];
	t *= (adc_res - adc_data[0][1]);
	t /= (adc_data[0][1] - adc_data[1][1]);
	t += adc_data[0][0];
	return t;
}

#define MEASURE_TIMES	(15)

static void __dump_adc_result(u32 adc_val[])
{
#if defined(CONFIG_REGULATOR_ADC_DEBUG)
	int i;
	for (i = 0; i < MEASURE_TIMES; i++) {
		printk("%d ", adc_val[i]);
	}
	printk("\n");
#endif
}

static int cmp_val(const void *a, const void *b)
{
	return *(int *)a - *(int *)b;
}

/**
 * __adc_voltage - get regulator output voltage through auxadc
 * @regulator: regulator source
 *
 * This returns the current regulator voltage in uV.
 *
 * NOTE: If the regulator is disabled it will return the voltage value. This
 * function should not be used to determine regulator state.
 */
static int regu_adc_voltage(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;

	int ret, adc_chan = regs->cal_ctl_bits >> 16;
	u16 ldo_cal_sel = regs->cal_ctl_bits & 0xFFFF;
	u32 adc_res, adc_val[MEASURE_TIMES];
	u32 chan_numerators = 1, chan_denominators = 1;
	u32 bat_numerators, bat_denominators;

	struct adc_sample_data adc_data = {
		.channel_id = adc_chan,
		.channel_type = 0,	/*sw */
		.hw_channel_delay = 0,	/*reserved */
		.scale = 1,	/*big scale */
		.pbuf = &adc_val[0],
		.sample_num = MEASURE_TIMES,
		.sample_bits = 1,	/*12bits mode */
		.sample_speed = 0,	/*quick mode */
		.signal_mode = 0,	/*resistance path */
	};

	if (!__is_valid_adc_cal())
		return -EACCES;

	if (!regs->cal_ctl)
		return -EINVAL;

	/* enable ldo cal before adc sampling and ldo calibration */
	if (0 == regs->typ) {
		ANA_REG_OR(regs->cal_ctl, ldo_cal_sel);
		debug0("%s adc channel %d : %04x\n",
		       desc->desc.name, adc_data.channel_id, ldo_cal_sel);
	}

	ret = sci_adc_get_values(&adc_data);
	BUG_ON(0 != ret);

	__dump_adc_result(adc_val);

	/* close ldo cal */
	if (0 == regs->typ) {
		ANA_REG_BIC(regs->cal_ctl, ldo_cal_sel);
	}
	sort(adc_val, MEASURE_TIMES, sizeof(u32), cmp_val, 0);

	__dump_adc_result(adc_val);

	sci_adc_get_vol_ratio(adc_data.channel_id, adc_data.scale,
			      &chan_numerators, &chan_denominators);

	sci_adc_get_vol_ratio(ADC_CHANNEL_VBAT, 0, &bat_numerators,
			      &bat_denominators);

	adc_res = adc_val[MEASURE_TIMES / 2];
	debug("%s adc result value %d, chan (%d/%d)\n",
	      desc->desc.name, adc_res, chan_numerators, chan_denominators);

	if (adc_res == 0)
		return -EAGAIN;
	else
		return __adc2vbat(adc_res)
		    * (bat_numerators * chan_denominators)
		    / (bat_denominators * chan_numerators);
}

static void do_regu_work(struct work_struct *w)
{
	struct sci_regulator_data *data =
	    container_of(w, struct sci_regulator_data, dwork.work);
	struct sci_regulator_desc *desc = __get_desc(data->rdev);
	debug0("%s\n", desc->desc.name);
	if (!desc->ops->is_trimming(data->rdev))
		desc->ops->calibrate(data->rdev, 0, 0);
}

int __regu_calibrate(struct regulator_dev *rdev, int def_vol, int to_vol)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;
	int in_calibration(void);
	if (in_calibration() || !__is_valid_adc_cal()
	    || !regs->cal_ctl || !regs->vol_ctl || !regs->vol_trm) {
		/* bypass if in CFT or not adc cal or no cal ctl */
		return -EACCES;
	}

	schedule_delayed_work(&desc->data.dwork, msecs_to_jiffies(100));
	return 0;
}

/*
 * ASSERT dcdc/ldo is enabled
 */
static int regu_calibrate(struct regulator_dev *rdev, int def_vol, int to_vol)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	const struct sci_regulator_regs *regs = desc->regs;

	int ret = 0;
	int acc_vol, adc_vol = 0, ctl_vol, cal_vol = 0;

	ctl_vol = rdev->desc->ops->get_voltage(rdev);
	if (IS_ERR_VALUE(ctl_vol)) {
		debug0("no valid %s vol ctrl bits\n", desc->desc.name);
	} else			/* dcdc/ldo maybe had been adjusted or opened in uboot-spl */
		ctl_vol /= 1000;

	if (!def_vol)
		def_vol = (IS_ERR_VALUE(ctl_vol)) ? regs->vol_def : ctl_vol;

	if (!to_vol)
		to_vol = (IS_ERR_VALUE(ctl_vol)) ? regs->vol_def : ctl_vol;

	adc_vol = regu_adc_voltage(rdev);
	if (adc_vol <= 0) {
		debug("%s default %dmv, maybe not enable\n", desc->desc.name,
		      def_vol);
		goto exit;
	}
	cal_vol = abs(adc_vol - to_vol);

	debug("%s default %dmv, from %dmv to %dmv, bias %c%d.%02d%%\n",
	      desc->desc.name,
	      def_vol, adc_vol, to_vol,
	      (adc_vol > to_vol) ? '+' : '-',
	      to_vol ? cal_vol * 100 / to_vol : 0,
	      to_vol ? cal_vol * 100 * 100 / to_vol % 100 : 0);

	if (!def_vol || !to_vol || adc_vol <= 0)
		goto exit;

	if (cal_vol > to_vol / 10)	/* adjust limit 10% */
		goto exit;
	else if (cal_vol < to_vol / 100) {	/* bias 1% */
		goto verify;
	}

	acc_vol = desc->ops->get_trimming_step(rdev, to_vol * 1000);

	/* always set valid vol ctrl bits */
	def_vol = ctl_vol =
	    DIV_ROUND_UP(def_vol * to_vol, adc_vol) + acc_vol / 1000;
	ret = desc->ops->set_trimming(rdev, ctl_vol * 1000, to_vol * 1000);
	if (IS_ERR_VALUE(ret))
		goto exit;

	set_bit(desc->desc.id, trimming_state);	/*force set before verify */
	msleep(1);		/* wait a moment before cal verify */

verify:
	adc_vol = regu_adc_voltage(rdev);
	cal_vol = abs(adc_vol - to_vol);

	debug("%s default %dmv, from %dmv to %dmv, bias %c%d.%02d%%\n",
	      desc->desc.name,
	      def_vol, adc_vol, to_vol,
	      (adc_vol > to_vol) ? '+' : '-',
	      cal_vol * 100 / to_vol, cal_vol * 100 * 100 / to_vol % 100);

	if (cal_vol < to_vol / 100) {	/* bias 1% */
		set_bit(desc->desc.id, trimming_state);
		debug("%s is ok\n", desc->desc.name);
	}

	return ctl_vol;

exit:
	debug("%s failure\n", desc->desc.name);
	return -1;
}

static struct regulator_ops ldo_ops = {
	.enable = ldo_turn_on,
	.disable = ldo_turn_off,
	.is_enabled = ldo_is_on,
	.set_voltage = ldo_set_voltage,
	.get_voltage = ldo_get_voltage,
	.set_mode = ldo_set_mode,
};

static struct regulator_ops usbd_ops = {
	.enable = 0,		/* reserved d-die ldo */
	.disable = 0,		/* reserved d-die ldo */
	.is_enabled = 0,	/* reserved d-die ldo */
};

static struct regulator_ops dcdc_ops = {
	.enable = ldo_turn_on,
	.disable = ldo_turn_off,
	.is_enabled = ldo_is_on,
	.set_voltage = dcdc_set_voltage,
	.get_voltage = dcdc_get_voltage,
};

static struct sci_regulator_ops sci_ldo_ops = {
	.init_trimming = ldo_init_trimming,
	.is_trimming = __is_trimming,
	.get_trimming_step = ldo_get_trimming_step,
	.set_trimming = ldo_set_trimming,
	.calibrate = regu_calibrate,
};

static struct sci_regulator_ops sci_dcdc_ops = {
	.init_trimming = dcdc_init_trimming,
	.is_trimming = __is_trimming,
	.get_trimming_step = dcdc_get_trimming_step,
	.set_trimming = dcdc_set_trimming,
	.calibrate = regu_calibrate,
};

/*
 * Consider the following machine :-
 *
 *   Regulator-1 -+-> [Consumer A @ 1.8V]
 *                |
 *                +-> [Consumer B @ 1.8V]
 *
 *   Regulator-2 ---> [Consumer C @ 3.3V]
 *
 * The drivers for consumers A & B must be mapped to the correct regulator in
 * order to control their power supply. This mapping can be achieved in board/machine
 * initialisation code by creating a struct regulator_consumer_supply for each regulator.
 * Alternatively, we built a regulator supply-consumers map, the format is as follow:
 *
 *      supply source-1, consumer A, consumer B, ..., NULL
 *      supply source-2, consumer C, ..., NULL
 *      ...
 *      NULL
 *
 */
static struct regulator_consumer_supply *set_supply_map(struct device *dev,
							const char *supply_name,
							int *num)
{
	char **map = (char **)dev_get_platdata(dev);
	int i, n;
	struct regulator_consumer_supply *consumer_supplies = NULL;

	if (!supply_name || !(map && map[0]))
		return NULL;

	for (i = 0; map[i] || map[i + 1]; i++) {
		if (map[i] && 0 == strcmp(map[i], supply_name))
			break;
	}

	/* i++; *//* Do not skip supply name */

	for (n = 0; map[i + n]; n++) ;

	if (n) {
		debug0("supply %s consumers %d - %d\n", supply_name, i, n);
		consumer_supplies =
		    kzalloc(n * sizeof(*consumer_supplies), GFP_KERNEL);
		BUG_ON(!consumer_supplies);
		for (n = 0; map[i]; i++, n++) {
			consumer_supplies[n].supply = map[i];
		}
		if (num)
			*num = n;
	}
	return consumer_supplies;
}

#if defined(CONFIG_DEBUG_FS)
static struct dentry *debugfs_root = NULL;

static int adc_chan = 5 /*VBAT*/;
static int debugfs_adc_chan_get(void *data, u64 * val)
{
	int i, ret;
	u32 adc_res, adc_val[MEASURE_TIMES];
	struct adc_sample_data adc_data = {
		.channel_id = adc_chan,
		.channel_type = 0,	/*sw */
		.hw_channel_delay = 0,	/*reserved */
		.scale = 1,	/*big scale */
		.pbuf = &adc_val[0],
		.sample_num = MEASURE_TIMES,
		.sample_bits = 1,	/*12bits mode */
		.sample_speed = 0,	/*quick mode */
		.signal_mode = 0,	/*resistance path */
	};

	ret = sci_adc_get_values(&adc_data);
	BUG_ON(0 != ret);

	for (i = 0; i < MEASURE_TIMES; i++) {
		printk("%d ", adc_val[i]);
	}
	printk("\n");

	sort(adc_val, MEASURE_TIMES, sizeof(u32), cmp_val, 0);
	adc_res = adc_val[MEASURE_TIMES / 2];
	pr_info("adc chan %d, result value %d, vbat %d\n",
		adc_data.channel_id, adc_res, __adc2vbat(adc_res));
	*val = adc_res;
	return 0;
}

static int debugfs_adc_chan_set(void *data, u64 val)
{
	adc_chan = val;
	return 0;
}

static int debugfs_enable_get(void *data, u64 * val)
{
	struct regulator_dev *rdev = data;
	if (rdev && rdev->desc->ops->is_enabled)
		*val = rdev->desc->ops->is_enabled(rdev);
	else
		*val = -1;
	return 0;
}

static int debugfs_enable_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;
	if (rdev && rdev->desc->ops->enable)
		(val) ? rdev->desc->ops->enable(rdev)
		    : rdev->desc->ops->disable(rdev);
	return 0;
}

static int debugfs_voltage_get(void *data, u64 * val)
{
	struct regulator_dev *rdev = data;
	if (rdev)
		*val = regu_adc_voltage(rdev);
	else
		*val = -1;
	return 0;
}

static int debugfs_ldo_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;
	if (rdev && rdev->desc->ops->set_voltage)
		rdev->desc->ops->set_voltage(rdev, val * 1000, val * 1000, 0);
	return 0;
}

static int debugfs_dcdc_set(void *data, u64 val)
{
	struct regulator_dev *rdev = data;
	int to_vol = (int)val;
	if (rdev)
		regu_calibrate(rdev, 0, to_vol);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_adc_chan,
			debugfs_adc_chan_get, debugfs_adc_chan_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_enable,
			debugfs_enable_get, debugfs_enable_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_ldo,
			debugfs_voltage_get, debugfs_ldo_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_dcdc,
			debugfs_voltage_get, debugfs_dcdc_set, "%llu\n");

static void rdev_init_debugfs(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc = __get_desc(rdev);
	desc->debugfs = debugfs_create_dir(rdev->desc->name, debugfs_root);
	if (IS_ERR(rdev->debugfs) || !rdev->debugfs) {
		pr_warn("Failed to create debugfs directory\n");
		rdev->debugfs = NULL;
		return;
	}

	debugfs_create_file("enable", S_IRUGO | S_IWUSR,
			    desc->debugfs, rdev, &fops_enable);

	debugfs_create_file("voltage", S_IRUGO | S_IWUSR,
			    desc->debugfs, rdev,
			    (0 == desc->regs->typ) ? &fops_ldo : &fops_dcdc);
}
#else
static void rdev_init_debugfs(struct regulator_dev *rdev)
{
}
#endif

void *__devinit sci_regulator_register(struct platform_device *pdev,
				       struct sci_regulator_desc *desc)
{
	static atomic_t __devinitdata idx = ATOMIC_INIT(1);	/* 0: dummy */
	struct regulator_dev *rdev;
	struct regulator_ops *__regs_ops[] = {
		&ldo_ops, &usbd_ops, &dcdc_ops, 0,
	};
	struct sci_regulator_ops *__sci_regs_ops[] = {
		&sci_ldo_ops, 0, &sci_dcdc_ops, 0,
	};
	struct regulator_consumer_supply consumer_supplies_default[] = {
		[0] = {
//                     .dev = 0,
		       .dev_name = 0,
		       .supply = desc->desc.name,
		       }
	};
	struct regulator_init_data init_data = {
		.supply_regulator = 0,
		.constraints = {
				.min_uV = 0,
				.max_uV = 4200 * 1000,
				.valid_modes_mask =
				REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
				.valid_ops_mask =
				REGULATOR_CHANGE_STATUS |
				REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_MODE,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = consumer_supplies_default,
		.regulator_init = 0,
		.driver_data = 0,
	};

	desc->desc.id = atomic_inc_return(&idx) - 1;

	BUG_ON(desc->regs->pd_set == desc->regs->pd_rst
	       && desc->regs->pd_set_bit == desc->regs->pd_rst_bit);

	BUG_ON(desc->regs->typ >= sizeof(__regs_ops));
	if (!desc->ops)
		desc->ops = __sci_regs_ops[desc->regs->typ];

	if (!desc->desc.ops)
		desc->desc.ops = __regs_ops[desc->regs->typ];

	init_data.consumer_supplies =
	    set_supply_map(&pdev->dev, desc->desc.name,
			   &init_data.num_consumer_supplies);

	if (!init_data.consumer_supplies)
		init_data.consumer_supplies = consumer_supplies_default;

	debug0("regu %p (%s)\n", desc->regs, desc->desc.name);
	rdev = regulator_register(&desc->desc, &pdev->dev, &init_data, 0, 0);
	if (init_data.consumer_supplies != consumer_supplies_default)
		kfree(init_data.consumer_supplies);

	if (!IS_ERR(rdev)) {
		rdev->reg_data = rdev;
		INIT_DELAYED_WORK(&desc->data.dwork, do_regu_work);
		desc->data.rdev = rdev;
		desc->ops->init_trimming(rdev);
		rdev_init_debugfs(rdev);
	}
	return rdev;
}

/**
 * IMPORTANT!!!
 * spreadtrum power regulators is intergrated on the chip, include LDOs and DCDCs.
 * so i autogen all regulators non-variable description in plat or mach directory,
 * which named __xxxx_regulator_map.h, BUT register all in regulator driver probe func,
 * just like other regulator vendor drivers.
 */
static int __devinit sci_regulator_probe(struct platform_device *pdev)
{
	debug0("platform device %p\n", pdev);
#include CONFIG_REGULATOR_SPRD_MAP
	return 0;
}

static struct platform_driver sci_regulator_driver = {
	.driver = {
		   .name = "sc2713-regulator",
		   .owner = THIS_MODULE,
		   },
	.probe = sci_regulator_probe,
};

static int __init regu_driver_init(void)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_root =
	    debugfs_create_dir(sci_regulator_driver.driver.name, NULL);
	if (IS_ERR(debugfs_root) || !debugfs_root) {
		WARN(!debugfs_root,
		     "%s: Failed to create debugfs directory\n",
		     sci_regulator_driver.driver.name);
		debugfs_root = NULL;
	}
	debugfs_create_file("adc_chan", S_IRUGO | S_IWUSR,
			    debugfs_root, &adc_chan, &fops_adc_chan);
#endif

	pr_info("%s chip id: (%08x)\n", sci_regulator_driver.driver.name,
		ANA_REG_GET(ANA_REG_GLB_CHIP_ID_HIGH) << 16
		| ANA_REG_GET(ANA_REG_GLB_CHIP_ID_LOW));
	return platform_driver_register(&sci_regulator_driver);
}

int __init sci_regulator_init(void)
{
	static struct platform_device regulator_device = {
		.name = "sc2713-regulator",
		.id = -1,
	};
	return platform_device_register(&regulator_device);
}

subsys_initcall(regu_driver_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum voltage regulator driver");
MODULE_AUTHOR("robot <zhulin.lian@spreadtrum.com>");
