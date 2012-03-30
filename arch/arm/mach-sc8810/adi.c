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
#include <asm/delay.h>

#include <mach/hardware.h>
#include <mach/adi.h>
#include <mach/ctl_glb.h>

#define CTL_GLB_BASE					( SPRD_GREG_BASE )
#define CTL_ADI_BASE					( SPRD_MISC_BASE )
#define SCI_ADDRESS(_b_, _o_)			( (u32)(_b_) + (_o_) )

#define hw_local_irq_save local_irq_save
#define hw_local_irq_restore local_irq_restore

static int sci_adi_internal_reset(void)
{
	//BUGBUG: reset ADI module (global)
	SCI_D(REG_GLB_SOFT_RST) |= BIT_ADI_SOFT_RST;
	udelay(2);		//wait 2 ticks
	SCI_D(REG_GLB_SOFT_RST) &= ~BIT_ADI_SOFT_RST;
	return 0;
}

static int sci_adi_chan_priority(void)
{
	SCI_D(REG_ADI_CTRL0) &= ~BIT_ARM_SCLK_EN;
	SCI_D(REG_ADI_CHNL_PRI) =
	    BITS_PD_WR_PRI(1) | BITS_RFT_WR_PRI(1) |
	    BITS_DSP_RD_PRI(0) | BITS_DSP_WR_PRI(0) |
	    BITS_ARM_RD_PRI(0) | BITS_ARM_WR_PRI(0) | BITS_STC_WR_PRI(1) |
	    BITS_INT_STEAL_PRI(0);
	//BUGBUG: How about CMMB_WR_PRI?
	return 0;
}

int sci_adi_enable(void)
{
	//BUGBUG: enable ADI_ACC (global) to put the adi master to normal operation mode
	SCI_D(REG_GLB_GEN0) |= BIT_ADI_EB;
	return 0;
}

int sci_adi_disable(void)
{
	//BUGBUG: disable ADI_ACC (global)
	return 0;
}

#if 0
int sci_adi_lock(void)
{
	return 0;
}

int sci_adi_unlock(void)
{
	return 0;
}

#else
#define sci_adi_lock()					\
		unsigned long flags;			\
		hw_local_irq_save(flags);		\

#define sci_adi_unlock() hw_local_irq_restore(flags);

#endif

int sci_adi_ready(void)
{
	int cnt = 1000;
	do {
		udelay(1);
	} while (!(SCI_D(REG_ADI_FIFO_STS) & BIT_FIFO_EMPTY) && cnt--);
	return 0;
}

int sci_adi_read(u32 reg)
{
	unsigned long val;
	sci_adi_lock();

	sci_adi_ready();

	SCI_D(REG_ADI_RD_CMD) = ANA_V2P(reg);	//BUGBUG: phy addr or virt?

	//wait read operation complete, RD_data[31] will be cleared after the read operation complete
	do {
		val = SCI_D(REG_ADI_RD_DATA);
	} while (val & BIT_RD_CMD_BUSY);	//BUGBUG: need timeout?

	//val high part should be the address of the last read operation
	BUG_ON((val & MASK_RD_ADDR) != BITS_RD_ADDR(reg));

	sci_adi_unlock();
	return (val & MASK_RD_VALU) >> SHIFT_RD_VALU;
}

int sci_adi_raw_write(u32 reg, u16 val)
{

	sci_adi_lock();

	do {
		;		//BUGBUG: need timeout?
	} while (!(SCI_D(REG_ADI_FIFO_STS) & BIT_FIFO_EMPTY));

	SCI_D(reg) = (u32) val;

	sci_adi_unlock();
	return 0;
}

int sci_adi_write(u32 reg, u16 val, u16 msk)
{
	sci_adi_lock();

	sci_adi_raw_write(reg, (sci_adi_read(reg) & ~msk) | val);

	sci_adi_unlock();
	return 0;
}

int sci_adi_set(u32 reg, u16 bits)
{
	sci_adi_lock();

	sci_adi_raw_write(reg, (u16) sci_adi_read(reg) | bits);

	sci_adi_unlock();
	return 0;
}

int sci_adi_clr(u32 reg, u16 bits)
{
	sci_adi_lock();

	sci_adi_raw_write(reg, (u16) sci_adi_read(reg) & ~bits);

	sci_adi_unlock();
	return 0;
}

static int __init adi_init(void)
{
	sci_adi_enable();
	sci_adi_internal_reset();
	sci_adi_chan_priority();
	return 0;
}

//EXPORT_SYMBOL(sci_adi_lock);
//EXPORT_SYMBOL(sci_adi_unlock);
//EXPORT_SYMBOL(sci_adi_enable);
//EXPORT_SYMBOL(sci_adi_disable);
EXPORT_SYMBOL(sci_adi_read);
EXPORT_SYMBOL(sci_adi_raw_write);
EXPORT_SYMBOL(sci_adi_write);
EXPORT_SYMBOL(sci_adi_set);
EXPORT_SYMBOL(sci_adi_clr);

arch_initcall(adi_init);
