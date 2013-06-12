/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 * Copyright (C) 2012 steve zhan
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/irqflags.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/hwspinlock.h>
#include <asm/delay.h>

#include <mach/hardware.h>
#include <mach/globalregs.h>
#include <mach/adi.h>
#include <mach/irqs.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/arch_misc.h>


static u32 chip_id __read_mostly;
u32 sci_get_chip_id(void)
{
	return chip_id;
}

void __init sc_init_chip_id(void)
{
#if defined(CONFIG_ARCH_SC8825)
	chip_id = __raw_readl(REG_AHB_CHIP_ID);
#endif
#if defined(CONFIG_ARCH_SCX35)
	chip_id = __raw_readl(REG_AON_APB_CHIP_ID);
	if (chip_id == 0)
		chip_id = SCX35_ALPHA_TAPOUT;//alpha Tapout.
#endif
}


