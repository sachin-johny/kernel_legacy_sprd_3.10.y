#include <linux/bug.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <mach/hardware.h>

#include <linux/irqflags.h>
#include <linux/io.h>
#include <linux/spinlock.h>

#include <mach/hardware.h>
#include <mach/regs_glb.h>
#include <mach/regs_ana_glb.h>
#include <mach/sci.h>
#include <mach/adi.h>
#include <mach/adc.h>
#include <mach/efuse.h>

/* FIXME */
#define GR_GEN1							REG_GLB_GEN1
#define GR_MPLL_MN						REG_GLB_M_PLL_CTL0
#define ANA_DCDC_CTRL        			ANA_REG_GLB_DCDC_CTRL0
#define ANA_DCDC_CTRL_CAL    			ANA_REG_GLB_DCDC_CTRL_CAL
#define ANA_DCDCARM_CTRL     			ANA_REG_GLB_DCDCARM_CTRL0
#define ANA_DCDCARM_CTRL_CAL 			ANA_REG_GLB_DCDCARM_CTRL_CAL

#define REG_SYST_VALUE                  (SPRD_SYSCNT_BASE + 0x0004)

#define debug(format, arg...) pr_info("dcdc: " "@@@" format, ## arg)
#define info(format, arg...) pr_info("dcdc: " "@@@" format, ## arg)

int sprd_get_adc_cal_type(void);
uint16_t sprd_get_adc_to_vol(uint16_t data);

#define CALIBRATE_TO	(60 * 1)	/* one minute */
#define MEASURE_TIMES	(128)
const int dcdc_ctl_vol[] = {
	650, 700, 800, 900, 1000, 1100, 1200, 1300, 1400,
};

int dcdc_calibrate(int adc_chan, int def_vol, int to_vol)
{
	int i;
	u32 val[MEASURE_TIMES], sum = 0, adc_vol, ctl_vol, cal_vol;
	for (i = 0; i < ARRAY_SIZE(val); i++) {
		sum += val[i] = sci_adc_get_value(adc_chan, true);
	}
	sum /= ARRAY_SIZE(val);	/* get average value */
	info("adc chan %d, value %d\n", adc_chan, sum);
	adc_vol = sprd_get_adc_to_vol(sum) * (8 * 5) / (30 * 4);
	if (!def_vol) {
		switch (adc_chan) {
		case ADC_CHANNEL_DCDCCORE:
			def_vol = 1100;
			cal_vol = sci_adi_read(ANA_DCDC_CTRL_CAL) & 0x1f;
			i = sci_adi_read(ANA_DCDC_CTRL) & 0x07;
			break;
		case ADC_CHANNEL_DCDCARM:
			def_vol = 1200;
			cal_vol = sci_adi_read(ANA_DCDCARM_CTRL_CAL) & 0x1f;
			i = sci_adi_read(ANA_DCDCARM_CTRL) & 0x07;
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
	case ADC_CHANNEL_DCDCCORE:
		sci_adi_raw_write(ANA_DCDC_CTRL_CAL,
				  cal_vol | (0x1f - cal_vol) << 8);
		sci_adi_raw_write(ANA_DCDC_CTRL, i | (0x07 - i) << 4);
		break;
	case ADC_CHANNEL_DCDCARM:
		sci_adi_raw_write(ANA_DCDCARM_CTRL_CAL,
				  cal_vol | (0x1f - cal_vol) << 8);
		sci_adi_raw_write(ANA_DCDCARM_CTRL, i | (0x07 - i) << 4);
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
	//BUG_ON(cpu_freq != 1200);	/* only upgrade 1.2G */
	cpu_freq /= 4;
	flags = hw_local_irq_save();
	val = sci_glb_raw_read(GR_MPLL_MN);
	if ((val & MASK_MPLL_N) == cpu_freq)
		goto exit;
	val = (val & ~MASK_MPLL_N) | cpu_freq;
	sci_glb_set(GR_GEN1, BIT(9));	/* mpll unlock */
	sci_glb_write(GR_MPLL_MN, val, MASK_MPLL_N);
	sci_glb_clr(GR_GEN1, BIT(9));
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

	val = sci_efuse_get(5);
	debug("%s efuse flag 0x%08x, mpll %08x\n", __FUNCTION__, val,
	      __raw_readl(GR_MPLL_MN));

	if (val & BIT(16) /*1.2G flag */ ) {
		dcdc_to_vol = 1200;
		dcdcarm_to_vol = 1250;
		cpu_freq = 1200;
	}

	dcdc_work.cal_typ = sprd_get_adc_cal_type();
	debug("%s %d %d\n", __FUNCTION__, dcdc_work.cal_typ, cnt);

	ret = dcdc_calibrate(ADC_CHANNEL_DCDCCORE, 0, dcdc_to_vol);
	if (ret > 0)
		dcdc_calibrate(ADC_CHANNEL_DCDCCORE, ret, dcdc_to_vol);

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
