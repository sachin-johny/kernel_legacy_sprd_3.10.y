/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/irqflags.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/adi.h>
#include <mach/ctl_adc.h>
#include <mach/ana_ctl_glb.h>

#define ANA_CTL_GLB_BASE				SCI_ADDRESS(SPRD_MISC_BASE, 0x0600)
#define ANA_CTL_ADC_BASE				SCI_ADDRESS(SPRD_MISC_BASE, 0x0300)

int sci_adc_enable(void)
{
	SCI_A_SET(ANA_REG_GLB_APB_CLK_EN,
		  BIT_ADC_EB | BIT_CLK_AUXADC_EN | BIT_CLK_AUXAD_EN);
	SCI_A_SET(ANA_REG_ADC_CTRL, BIT_ADC_EN);
//      SCI_A_SET(ANA_REG_GLB_APB_ARM_RST, BIT(4));
	udelay(1);
//      SCI_A_CLR(ANA_REG_GLB_APB_ARM_RST, BIT(4));
	return 0;
}

int sci_adc_disable(void)
{
	return 0;
}

#if defined(CONFIG_SUPPORT_ADC_LOCK)
int sci_adc_lock(void)
{
	return 0;
}

int sci_adc_unlock(void)
{
	return 0;
}

#else
#define sci_adc_lock()					\
		unsigned long flags;			\
		local_irq_save(flags);			\

#define sci_adc_unlock() local_irq_restore(flags);

#endif

int sci_adc_get_value(unsigned chan, int scale)
{
	int ret;
	u32 cnt = 12;
	sci_adc_lock();

	BUG_ON(chan >= (MASK_ADC_CS >> SHIFT_ADC_CS));

	//sci_adc_enable();

	//clear int
	sci_adi_set(ANA_REG_ADC_IC, BIT_ADC_IC);

	//choose channel and set scale
	sci_adi_write(ANA_REG_ADC_CS, BITS_ADC_CS(chan), MASK_ADC_DAT);
	//udelay(100);
	if (scale)
		sci_adi_set(ANA_REG_ADC_CS, BIT_ADC_SCALE);
	else
		sci_adi_clr(ANA_REG_ADC_CS, BIT_ADC_SCALE);

	//turn on sw channel
	sci_adi_set(ANA_REG_ADC_CTRL, BIT_SW_CH_ON);

	//wait adc complete
	while (!(SCI_A(ANA_REG_ADC_ISRC) & BIT_ADC_RIS) && cnt--) {
		udelay(50);
	}

	WARN_ON(!cnt);

	ret = SCI_A(ANA_REG_ADC_DAT) & MASK_ADC_DAT;

	//turn off sw channel
	sci_adi_clr(ANA_REG_ADC_CTRL, BIT_SW_CH_ON);

	//clear int
	sci_adi_set(ANA_REG_ADC_IC, BIT_ADC_IC);

	sci_adc_unlock();
	return ret;
}

/*
int adc_test(void)
{
	sci_adc_enable();
	for(;;)
		sci_adc_get_value(5, 0);
	return 0;
}
*/
