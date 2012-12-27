#include <linux/bug.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <mach/hardware.h>
#include <mach/adi_hal_internal.h>
#include <mach/adc_drvapi.h>
#include <mach/regs_global.h>
#include <mach/regs_ana.h>
#include <mach/regs_ahb.h>

/* FIXME */
#define REG_SYST_VALUE                  (SPRD_SYSCNT_BASE + 0x0004)
#define debug(format, arg...) pr_info("dcdc: " "@@@" format, ## arg)
#define info(format, arg...) pr_info("dcdc: " "@@@" format, ## arg)

extern u32 sprd_glb_get_chipid(void);
uint16_t CHGMNG_AdcvalueToVoltage(uint16_t adcvalue);
int sprd_get_adc_cal_type(void);
void sci_efuse_poweron(void);
int sci_efuse_read(unsigned blk);
void sci_efuse_poweroff(void);

#define CALIBRATE_TO	(60 * 1)	/*one minute */
#define MEASURE_TIMES	(128)
const int dcdc_ctl_vol[] = {
	650, 700, 800, 900, 1000, 1100, 1200, 1300, 1400,
};

int dcdc_calibrate(int adc_chan, int def_vol, int to_vol)
{
	int i;
	u32 val[MEASURE_TIMES], sum = 0, adc_vol, ctl_vol, cal_vol;
	for (i = 0; i < ARRAY_SIZE(val); i++) {
		sum += val[i] = ADC_GetValue(adc_chan, false);
	}
	sum /= ARRAY_SIZE(val);	//get average value
	info("adc chan %d, value %d\n", adc_chan, sum);
	adc_vol = CHGMNG_AdcvalueToVoltage(sum) * (8 * 5) / (30 * 4);

	if (!def_vol) {
		switch (adc_chan) {
		case ADC_CHANNEL_DCDC:
			def_vol = 1100;
			cal_vol = ANA_REG_GET(ANA_DCDC_CTRL_CAL) & 0x1f;
			i = ANA_REG_GET(ANA_DCDC_CTRL) & 0x07;
			break;
		case ADC_CHANNEL_DCDCARM:
			def_vol = 1200;
			cal_vol = ANA_REG_GET(ANA_DCDCARM_CTRL_CAL) & 0x1f;
			i = ANA_REG_GET(ANA_DCDCARM_CTRL) & 0x07;
			break;
		default:
			goto exit;
		}
		if (0 != i /* + cal_vol */ )
			def_vol = dcdc_ctl_vol[i];
		def_vol += cal_vol * 100 / 32;
#if 0
		if (0 != i + cal_vol) {	/* dcdc had been adjusted in uboot-spl */
			debug("%s default %dmv, from %dmv to %dmv\n",
			     __FUNCTION__, def_vol, adc_vol, to_vol);
			goto exit;
		}
#endif
	}
	info("%s default %dmv, from %dmv to %dmv\n", __FUNCTION__, def_vol,
	     adc_vol, to_vol);

	cal_vol = abs(adc_vol - to_vol);
	if (cal_vol > 200 /* mv */ )
		goto exit;
	else if (cal_vol < to_vol / 100) {
		info("%s is ok\n", __FUNCTION__);
		return 0;
	}

	ctl_vol = def_vol * to_vol / adc_vol;
	for (i = 0; i < ARRAY_SIZE(dcdc_ctl_vol) - 1; i++) {
		if (ctl_vol < dcdc_ctl_vol[i + 1])
			break;
	}
	if (i >= ARRAY_SIZE(dcdc_ctl_vol) - 1)
		goto exit;

	cal_vol = ((ctl_vol - dcdc_ctl_vol[i]) * 32 / 100) % 32;
	debug("%s cal_vol %dmv: %d, 0x%02x\n", __FUNCTION__,
	      dcdc_ctl_vol[i] + cal_vol * 100 / 32, i, cal_vol);
	switch (adc_chan) {
	case ADC_CHANNEL_DCDC:
		ANA_REG_SET(ANA_DCDC_CTRL_CAL, cal_vol | (0x1f - cal_vol) << 8);
		ANA_REG_SET(ANA_DCDC_CTRL, i | (0x07 - i) << 4);
		break;
	case ADC_CHANNEL_DCDCARM:
		ANA_REG_SET(ANA_DCDCARM_CTRL_CAL,
			    cal_vol | (0x1f - cal_vol) << 8);
		ANA_REG_SET(ANA_DCDCARM_CTRL, i | (0x07 - i) << 4);
		break;
	default:
		break;
	}

	return dcdc_ctl_vol[i] + cal_vol * 100 / 32;
      exit:
	info("%s failure\n", __FUNCTION__);
	return -1;
}

int mpll_calibrate(int cpu_freq)
{
	u32 val = 0;
	unsigned long flags;
//	BUG_ON(cpu_freq != 1200);	/* only upgrade 1.2G */
	cpu_freq /= 4;
	hw_local_irq_save(flags);
	val = __raw_readl(GR_MPLL_MN);
	if ((val & 0x7ff) == cpu_freq)
		goto exit;
	val = (val & ~0x7ff) | cpu_freq;
	__raw_writel(__raw_readl(GR_GEN1) | BIT(9), GR_GEN1);	/* mpll unlock */
	__raw_writel(val, GR_MPLL_MN);
	__raw_writel(__raw_readl(GR_GEN1) & ~BIT(9), GR_GEN1);
exit:
	hw_local_irq_restore(flags);
	debug("%s 0x%08x\n", __FUNCTION__, val);
	return 0;
}

struct dcdc_delayed_work {
	struct delayed_work work;
	u32 uptime;
	int cal_typ;
};

static struct dcdc_delayed_work dcdc_work = {
	.work.work.func = NULL,
	.uptime = 0,
	.cal_typ = 0,
};

static u32 sci_syst_read(void)
{
	u32 t = __raw_readl(REG_SYST_VALUE);
	while (t != __raw_readl(REG_SYST_VALUE))
		t = __raw_readl(REG_SYST_VALUE);
	return t;
}

static void do_dcdc_work(struct work_struct *work)
{
	int ret, cnt = CALIBRATE_TO;
	int dcdc_to_vol = 1100;	/* vddcore */
	int dcdcarm_to_vol = 1200;	/* vddarm */
	int cpu_freq = 1000;	/* Mega */
	u32 val = 0;

	/* debug("%s %d\n", __FUNCTION__, sprd_get_adc_cal_type()); */
	if (dcdc_work.cal_typ == sprd_get_adc_cal_type())
		goto exit;	/* no change, set next delayed work */

	sci_efuse_poweron();
	val = sci_efuse_read(5);
	sci_efuse_poweroff();
	debug("%s efuse flag 0x%08x, mpll %08x\n", __FUNCTION__, val,
	      __raw_readl(GR_MPLL_MN));

	if (val & BIT(16) /*1.2G flag */ ) {
		dcdc_to_vol = 1200;
		dcdcarm_to_vol = 1250;
		cpu_freq = 1200;
	}

	if (sprd_glb_get_chipid() == CHIP_ID_8810S) {	/*SMIC CHIP*/
		dcdcarm_to_vol += 100;
		dcdc_to_vol += 100;
	}

	dcdc_work.cal_typ = sprd_get_adc_cal_type();
	debug("%s %d %d\n", __FUNCTION__, dcdc_work.cal_typ, cnt);

	ret = dcdc_calibrate(ADC_CHANNEL_DCDC, 0, dcdc_to_vol);
	if (ret > 0)
		dcdc_calibrate(ADC_CHANNEL_DCDC, ret, dcdc_to_vol);

	ret = dcdc_calibrate(ADC_CHANNEL_DCDCARM, 0, dcdcarm_to_vol);
	if (ret > 0)
		dcdc_calibrate(ADC_CHANNEL_DCDCARM, ret, dcdcarm_to_vol);

      exit:
	if (sci_syst_read() - dcdc_work.uptime < CALIBRATE_TO * 1000) {
		schedule_delayed_work(&dcdc_work.work, msecs_to_jiffies(1000));
	} else {
		info("%s end\n", __FUNCTION__);
	}

	if (cpu_freq == 1200) {
		msleep(100);
		mpll_calibrate(cpu_freq);
	}
	return;
}

void dcdc_calibrate_callback(void *data)
{
	if (!dcdc_work.work.work.func) {
		INIT_DELAYED_WORK(&dcdc_work.work, do_dcdc_work);
		dcdc_work.uptime = sci_syst_read();
	}
	schedule_delayed_work(&dcdc_work.work, msecs_to_jiffies(10));
}

static int __init dcdc_init(void)
{
	dcdc_calibrate_callback(0);
	return 0;
}

EXPORT_SYMBOL(dcdc_calibrate);
EXPORT_SYMBOL(mpll_calibrate);
late_initcall(dcdc_init);
