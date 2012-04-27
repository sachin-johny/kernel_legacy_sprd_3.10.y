#include <linux/bug.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/adi_hal_internal.h>
#include <mach/adc_drvapi.h>
#include <mach/regs_global.h>
#include <mach/regs_ana.h>

#define debug(format, arg...) pr_info("dcdc: " "@@@" format, ## arg)
#define info(format, arg...) pr_info("dcdc: " "@@@" format, ## arg)

uint16_t CHGMNG_AdcvalueToVoltage(uint16_t adcvalue);
int sprd_get_adc_cal_type(void);

#define MEASURE_TIMES	(128)
const int dcdc_ctl_vol[] = {
	650, 700, 800, 900, 1000, 1100, 1200, 1300, 1400,
};

static int dcdc_calibrate(int adc_chan, int def_vol, int to_vol)
{
	int i;
	u32 val[MEASURE_TIMES], sum = 0, adc_vol, ctl_vol, cal_vol;
	for (i = 0; i < ARRAY_SIZE(val); i++) {
		sum += val[i] = ADC_GetValue(adc_chan, false);
	}
	sum /= ARRAY_SIZE(val);	//get average value
	adc_vol = CHGMNG_AdcvalueToVoltage(sum) * (8 * 5) / (30 * 4);
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

int do_dcdc_init(void *data)
{
	int ret, cnt = 60 * 3;	//three minutes
	int dcdc_def_vol = 1100;	//FIXME: how to read dcdc value?
	debug("%s %d\n", __FUNCTION__, sprd_get_adc_cal_type());

#if 0				//cal test
	ANA_REG_SET(ANA_DCDC_CTRL_CAL, 0x10);
	dcdc_def_vol += (ANA_REG_GET(ANA_DCDC_CTRL_CAL) & 0x1f) * 100 / 32;
#endif

	do {
		msleep(1000);
	} while (0 == sprd_get_adc_cal_type() && --cnt);	//wait for user app setup battery calibrate params
	debug("%s %d\n", __FUNCTION__, sprd_get_adc_cal_type());

	if (0 == cnt || 0 == sprd_get_adc_cal_type()) {
		info("%s maybe timeout\n", __FUNCTION__);
		return 0;
	}

	ret = dcdc_calibrate(ADC_CHANNEL_DCDC, dcdc_def_vol, 1100);
	if (ret > 0)		//verify
		dcdc_calibrate(ADC_CHANNEL_DCDC, ret, 1100);

	ret = dcdc_calibrate(ADC_CHANNEL_DCDCARM, 1200, 1200);
	if (ret > 0)		//verify
		dcdc_calibrate(ADC_CHANNEL_DCDCARM, ret, 1200);
	return 0;
}

static int __init dcdc_init(void)
{
	int ret;
	ret = kernel_thread(do_dcdc_init, 0, 0);
	if (ret < 0) {
		debug("Can't create dcdc thread!\n");
	}
	return 0;
}

late_initcall(dcdc_init);
