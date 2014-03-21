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
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>
#include <mach/board.h>

#include "dcam_drv.h"

#ifdef CONFIG_OF
uint32_t		dcam_regbase;
#endif

void   parse_baseaddress(struct device_node	*dn)
{
	struct resource  r;

#ifdef CONFIG_OF
	of_address_to_resource(dn, 0,&r);
	printk("DCAM BASE=0x%x \n",r.start);
	dcam_regbase = r.start;
#endif
}

uint32_t   parse_irq(struct device_node *dn)
{
#ifdef CONFIG_OF
	return irq_of_parse_and_map(dn, 0);
#else
	return DCAM_IRQ;
#endif
}

struct clk  * parse_clk(struct device_node *dn, char *clkname)
{
#if CONFIG_OF
	return of_clk_get_by_name(dn, clkname);
#else
	return clk_get(NULL, clkname);
#endif
}
