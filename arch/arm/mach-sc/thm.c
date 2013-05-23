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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/adi.h>
#include <mach/thm.h>

//#define SPRD_THM_DEBUG
#ifdef SPRD_THM_DEBUG
#define THM_DEBUG(format, arg...) printk("sprd thm: " "@@@" format, ## arg)
#else
#define THM_DEBUG(format, arg...)
#endif

#define THMA_BASE	((unsigned int)SPRD_ADI_BASE + 0x8280)

#define THM_CTRL            (0x0000)
#define SENSOR_CTRL         (0x0020)
#define SENSOR_TEMPER0	(0x0058)

static const short temp_search_high_40nm[] =
    { -41, -14, 14, 41, 68, 95, 122, 150 };
static const short temp_search_low_40nm[] =
    { 0, 2, 4, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24, 26 };
static const short temp_search_high_152nm[] =
    { -45, -19, 7, 33, 59, 85, 111, 137 };
static const short temp_search_low_152nm[] =
    { 0, 2, 3, 5, 6, 8, 10, 11, 13, 14, 16, 18, 19, 21, 22, 24 };

static int arm_sen_cal_offset = 0;
static int pmic_sen_cal_offset = 0;

int sprd_thm_temp_read(u32 sensor)
{
	u32 temp = 0;

	if (SPRD_ARM_SENSOR == sensor) {
		temp = __raw_readl((SPRD_THM_BASE + SENSOR_TEMPER0));
		THM_DEBUG("thm sensor id:%d, raw temp:0x%x", sensor, temp);
		return (temp_search_high_40nm[(temp >> 4) & 0x07] +
			temp_search_low_40nm[temp & 0x0F] + arm_sen_cal_offset);
	} else if (SPRD_PMIC_SENSOR == sensor) {
		temp = sci_adi_read(THMA_BASE + SENSOR_TEMPER0);
		THM_DEBUG("thm sensor id:%d, raw temp:0x%x", sensor, temp);
		return (temp_search_high_152nm[(temp >> 4) & 0x07] +
			temp_search_low_152nm[temp & 0x0F] +
			pmic_sen_cal_offset);
	} else {
		printk(KERN_ERR "error thm sensor id:%d \n", sensor);
	}
	return 0;
}

static void sprd_thm_hw_init(void)
{
	sci_glb_set(REG_AON_APB_APB_EB1, BIT_THM_EB);
	sci_glb_set(REG_AON_APB_APB_RTC_EB,
		    (BIT_THM_RTC_EB | BIT_ARM_THMA_RTC_EB |
		     BIT_ARM_THMA_RTC_AUTO_EN));
	__raw_writel(0x01, (SPRD_THM_BASE + THM_CTRL));	//enable ddie sensor
	__raw_writel((__raw_readl((SPRD_THM_BASE + SENSOR_CTRL)) | 0x9), (SPRD_THM_BASE + SENSOR_CTRL));	//sensor0 ready

	//adie thm en
	sci_adi_set(ANA_REG_GLB_ARM_MODULE_EN, BIT_ANA_THM_EN);
	sci_adi_set(ANA_REG_GLB_RTC_CLK_EN,
		    BIT_RTC_THMA_AUTO_EN | BIT_RTC_THMA_EN | BIT_RTC_THM_EN);
	sci_adi_set((THMA_BASE + THM_CTRL), 0x1);
	sci_adi_set((THMA_BASE + SENSOR_CTRL), 0x9);
}

static int __init sprd_thm_init(void)
{
	sprd_thm_hw_init();
	arm_sen_cal_offset = 0;
	pmic_sen_cal_offset = 0;
	return 0;
}

late_initcall(sprd_thm_init);
