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

#include <mach/hardware.h>
#include <mach/adi.h>
#include <mach/ctl_adc.h>

#define ANA_CTL_ADC_BASE				( SPRD_MISC_BASE + 0x0300 )

int sci_adc_enable(void)
{
	//BUGBUG: enable ana global ADC
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
	sci_adc_lock();

	BUG_ON(chan >= (MASK_ADC_CS >> SHIFT_ADC_CS));

	//choose channel and set scale
	sci_adi_raw_write(ANA_REG_ADC_CS, BITS_ADC_CS(chan));
	if (unlikely(scale))
		sci_adi_set(ANA_REG_ADC_CS, BIT_ADC_SCALE);

	//turn on sw channel
	sci_adi_set(ANA_REG_ADC_CTRL, BIT_SW_CH_ON);

	//wait adc complete
	do {
		;		//BUGBUG: how about timeout?
	} while (!(SCI_A(ANA_REG_ADC_ISRC) & BIT_ADC_RIS));

	ret = SCI_A(ANA_REG_ADC_DAT) & MASK_ADC_CS;

	//turn off sw channel
	sci_adi_clr(ANA_REG_ADC_CTRL, BIT_SW_CH_ON);

	//clear int
	sci_adi_set(ANA_REG_ADC_IC, BIT_ADC_IC);

	sci_adc_unlock();
	return ret;
}
