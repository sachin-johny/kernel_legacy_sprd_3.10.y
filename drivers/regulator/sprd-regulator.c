/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/err.h>
#include <linux/platform_device.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/regs_glb.h>
#include <mach/regs_ana_glb.h>
#include <mach/adi.h>

#undef debug
#define debug(format, arg...) pr_info("regu: " "@@@%s: " format, __func__, ## arg)

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
	u32 vol_ctl, vol_ctl_bits;
	u32 vol_sel_cnt, vol_sel[];
};

struct sci_regulator_desc {
	struct regulator_desc desc;
	const struct sci_regulator_regs *regs;
};

enum {
	VDD_TYP_LDO = 0,
	VDD_TYP_LDO_D = 1,
	VDD_TYP_DCDC = 2,
};

enum {
	VDD_IS_ON = 0,
	VDD_ON,
	VDD_OFF,
	VOL_SET,
	VOL_GET,
};

#define SCI_REGU_REG(VDD, TYP, PD_SET, SET_BIT, PD_RST, RST_BIT, SLP_CTL, SLP_CTL_BIT, VOL_CTL, VOL_CTL_BITS, VOL_SEL_CNT, ...) \
do { 														\
	static const struct sci_regulator_regs REGS_##VDD = {	\
		.typ		= TYP,									\
		.pd_set = PD_SET,                           		\
		.pd_set_bit = SET_BIT,                      		\
		.pd_rst = PD_RST,                           		\
		.pd_rst_bit = RST_BIT,                      		\
		.slp_ctl = SLP_CTL,                         		\
		.slp_ctl_bit = SLP_CTL_BIT,                 		\
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

/* standard ldo ops*/
int sci_ldo_op(const struct sci_regulator_regs *regs, int op)
{
	int ret = 0;

	debug("regu %p op(%d), set %08x[%d], rst %08x[%d]\n", regs, op,
	      regs->pd_set, __ffs(regs->pd_set_bit), regs->pd_rst,
	      __ffs(regs->pd_rst_bit));

	if (!regs->pd_rst || !regs->pd_set)
		return -EACCES;

	switch (op) {
	case VDD_ON:
		ANA_REG_OR(regs->pd_rst, regs->pd_rst_bit);
		ANA_REG_BIC(regs->pd_set, regs->pd_set_bit);
		break;
	case VDD_OFF:
		ANA_REG_OR(regs->pd_set, regs->pd_set_bit);
		ANA_REG_BIC(regs->pd_rst, regs->pd_rst_bit);
		break;
	case VDD_IS_ON:
		ret = ! !(ANA_REG_GET(regs->pd_rst) & regs->pd_rst_bit);
		//sci_assert(ret ^ ! !(ANA_REG_GET(regs->pd_set) & regs->pd_set_bit));
		break;
	default:
		break;
	}
	return ret;
}

static int ldo_turn_on(struct regulator_dev *rdev)
{
	sci_ldo_op(((struct sci_regulator_desc *)(rdev->desc))->regs, VDD_ON);
	return 0;
}

static int ldo_turn_off(struct regulator_dev *rdev)
{
	sci_ldo_op(((struct sci_regulator_desc *)(rdev->desc))->regs, VDD_OFF);
	return 0;
}

static int ldo_is_on(struct regulator_dev *rdev)
{
	sci_ldo_op(((struct sci_regulator_desc *)(rdev->desc))->regs,
		   VDD_IS_ON);
	return 0;
}

static int ldo_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct sci_regulator_desc *desc =
	    (struct sci_regulator_desc *)rdev->desc;
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
	struct sci_regulator_desc *desc =
	    (struct sci_regulator_desc *)rdev->desc;
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
			ANA_REG_SET(regs->vol_ctl, vol_bits[i] << shft,
				    regs->vol_ctl_bits);
			ret = 0;
			break;
		}
	}

	return ret;
}

static int ldo_get_voltage(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc =
	    (struct sci_regulator_desc *)rdev->desc;
	const struct sci_regulator_regs *regs = desc->regs;
	u32 vol, vol_bits;
	int i, shft = __ffs(regs->vol_ctl_bits);

	debug("regu %p (%s), vol ctl %08x, shft %d, mask %08x\n",
	      regs, desc->desc.name, regs->vol_ctl, shft, regs->vol_ctl_bits);

	if (!regs->vol_ctl)
		return -EACCES;

	BUG_ON(regs->vol_sel_cnt != 4);
	vol_bits = ((ANA_REG_GET(regs->vol_ctl) & regs->vol_ctl_bits) >> shft);

	if ((vol_bits & BIT(0)) ^ (vol_bits & BIT(1))
	    && (vol_bits & BIT(2)) ^ (vol_bits & BIT(3))) {
		i = (vol_bits & BIT(0)) | ((vol_bits >> 1) & BIT(1));
		vol = regs->vol_sel[i];
		return vol * 1000;
	}
	return -EFAULT;
}

/* standard dcdc ops*/
static int dcdc_set_voltage(struct regulator_dev *rdev, int min_uV,
			    int max_uV, unsigned *selector)
{
	struct sci_regulator_desc *desc =
	    (struct sci_regulator_desc *)rdev->desc;
	const struct sci_regulator_regs *regs = desc->regs;
	int mv = min_uV / 1000;
	int ret = -EINVAL;
	int i, shft = __ffs(regs->vol_ctl_bits);
	int max = regs->vol_ctl_bits >> shft;

	BUG_ON(shft != 0);
	debug("regu %p (%s) %d %d\n", regs, desc->desc.name, min_uV, max_uV);
	return -EACCES;

	if (!regs->vol_ctl)
		return -EACCES;

	for (i = 0; i < regs->vol_sel_cnt; i++) {
		if (regs->vol_sel[i] == mv) {
			ANA_REG_SET(regs->vol_ctl, i | (max - i) << 4, -1);
			/*FIXME: small adjust later */
			ret = 0;
			break;
		}
	}

	return ret;
}

static int dcdc_get_voltage(struct regulator_dev *rdev)
{
	struct sci_regulator_desc *desc =
	    (struct sci_regulator_desc *)rdev->desc;
	const struct sci_regulator_regs *regs = desc->regs;
	debug("regu %p (%s)\n", regs, desc->desc.name);
	return -EACCES;
}

/* standard ldo-D-Die ops*/
static int usbd_turn_on(struct regulator_dev *rdev)
{
	const struct sci_regulator_regs *regs =
	    ((struct sci_regulator_desc *)(rdev->desc))->regs;
	sci_glb_set(regs->pd_set, regs->pd_set_bit);
	return 0;
}

static int usbd_turn_off(struct regulator_dev *rdev)
{
	const struct sci_regulator_regs *regs =
	    ((struct sci_regulator_desc *)(rdev->desc))->regs;
	sci_glb_clr(regs->pd_set, regs->pd_set_bit);
	return 0;
}

static int usbd_is_on(struct regulator_dev *rdev)
{
	const struct sci_regulator_regs *regs =
	    ((struct sci_regulator_desc *)(rdev->desc))->regs;
	return ! !sci_glb_read(regs->pd_set, regs->pd_set_bit);
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
	.enable = usbd_turn_on,
	.disable = usbd_turn_off,
	.is_enabled = usbd_is_on,
};

static struct regulator_ops dcdc_ops = {
	.enable = ldo_turn_on,
	.disable = ldo_turn_off,
	.is_enabled = ldo_is_on,
	.set_voltage = dcdc_set_voltage,
	.get_voltage = dcdc_get_voltage,
};

void *__devinit sci_regulator_register(struct platform_device *pdev,
				       struct sci_regulator_desc *desc)
{
	static int __devinitdata idx = 0;
	struct regulator_ops *__regs_ops[] = {
		&ldo_ops, &usbd_ops, &dcdc_ops, 0,
	};
	struct regulator_consumer_supply consumer_supplies[] = {
		[0] = {
		       .dev = 0,
		       .dev_name = 0,
		       .supply = desc->desc.name,
		       }
	};
	struct regulator_init_data init_data = {
		.supply_regulator = 0,
		.constraints = {
				.min_uV = 0,
				.max_uV = 4200 * 1000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask =
				REGULATOR_CHANGE_STATUS |
				REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_MODE,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = consumer_supplies,
		.regulator_init = 0,
		.driver_data = 0,
	};

	BUG_ON(desc->regs->typ > 3);
	if (!desc->desc.ops)
		desc->desc.ops = __regs_ops[desc->regs->typ];

	desc->desc.id = idx++;
	debug("regu %p (%s)\n", desc->regs, desc->desc.name);
	return regulator_register(&desc->desc, &pdev->dev, &init_data, 0);
}

/**
 * IMPORTANT!!!
 * spreadtrum power regulators is intergrated on the chip, include LDOs and DCDCs.
 * so i autogen all regulators invariable description in platform directory,
 * which named __regulator_map.h, BUT register all in regulator driver probe func,
 * just like other regulator drivers.
 */
static int __devinit sci_regulator_probe(struct platform_device *pdev)
{
	debug("platform device %p\n", pdev);
#include "mach/__regulator_map.h"
	return 0;
}

static struct platform_driver sci_regulator_driver = {
	.driver = {
		   .name = "sprd-regulator",
		   .owner = THIS_MODULE,
		   },
	.probe = sci_regulator_probe,
};

static int __init sci_regulator_init(void)
{
	return platform_driver_register(&sci_regulator_driver);
}

subsys_initcall(sci_regulator_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum voltage regulator driver");
MODULE_AUTHOR("robot <zhulin.lian@spreadtrum.com>");
