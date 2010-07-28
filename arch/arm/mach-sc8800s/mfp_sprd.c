/*
 *  linux/arch/arm/mach-sc8800s/mfp-sprd.c
 *
 *  Spreadtrum SoC multi-function pin configuration support
 *
 *  The GPIOs on SoC can be configured as one of many alternate
 *  functions, 
 *
 *  Author:	Yingchun Li(yingchun.li@spreadtrum.com)
 *  Created:	March 10, 2010
 *  Copyright:	Spreadtrum Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <asm/io.h>
#include <mach/mfp.h>
#include <mach/regs_cpc.h>

/*
NOTE: pin to gpio's map, you should check it from your chip's spec 
	carefully.
	in the map table, pin as the index, for mostly we use 
	MFP_PIN_TO_GPIO to get the gpio from the pin
*/
const static unsigned long pin_gpio_map[MFP_PIN_MAX] = {
	[MFP_PIN_SD1_CLK] = 9,
	[MFP_PIN_LCD_EN] = 103,
	[MFP_PIN_MAX - 1] = 0xffff
};

unsigned long mfp_to_gpio(int pin)
{
	return pin_gpio_map[pin];
}

static  int __mfp_validate(unsigned long c)
{
	int pin = MFP_CFG_TO_PIN(c);

	if (pin > MFP_PIN_MAX) {
		pr_warning("%s: pin %d is invalid pin\n", __func__, pin);
		return -1;
	}

	return pin;
}

static int __mfp_config_pin(int pin, unsigned long c)
{
	//get chip pin select register base 
	unsigned long flags;
	unsigned long pin_reg = (unsigned long )PIN_CTL_BASE + (pin * 4);
	unsigned long pin_cfg;

	//pr_info("register is :0x%x, old config is %x", (int)pin_reg, (int)pin_cfg);
	
	local_irq_save(flags);
	pin_cfg =__raw_readl(pin_reg);
	if (c & MFP_IO_SET) {
		pin_cfg = (pin_cfg & ~MFP_IO_MASK) | (c & MFP_IO_MASK); 
	}

	if (c & MFP_PULL_SET) {
		pin_cfg = (pin_cfg & ~MFP_PULL_MASK) | (c & MFP_PULL_MASK); 
	}

	if (c & MFP_DS_SET) {
		pin_cfg = (pin_cfg & ~MFP_DS_MASK) | (c & MFP_DS_MASK); 
	}

	if (c & MFP_AF_SET) {
		pin_cfg = (pin_cfg & ~MFP_AF_MASK) | (c & MFP_AF_MASK); 
	}
	__raw_writel(pin_cfg, pin_reg);	
	local_irq_restore(flags);
	
	//pr_info("new config is :%x", (int)pin_cfg);
	
	return 0;
}

void sprd_mfp_config(unsigned long *mfp_cfgs, int num)
{
	unsigned long flags;
	unsigned long *c;
	int i, pin;

	for (i = 0, c = mfp_cfgs; i < num; i++, c++) {

		pin = __mfp_validate((*c));
		if (pin < 0)
			continue;

		local_irq_save(flags);

		__mfp_config_pin(pin, *c);

		local_irq_restore(flags);
	}
}

