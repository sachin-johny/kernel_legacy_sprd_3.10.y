/* linux/arch/arm/mach-sc8800s/board-hisense.c
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/initrd.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/board.h>
#include <mach/hardware.h>

#include <asm/io.h>
#include <asm/delay.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mach/gpio.h>

static struct resource example_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C004400,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 44,
		.end	= 44,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device example_device = {
	.name           = "example",
	.id	            = 0,
	.num_resources  = ARRAY_SIZE(example_resources),
	.resource       = example_resources,
};

static struct platform_device *devices[] __initdata = {
	&example_device,
};

extern struct sys_timer sprd_timer;

static void __init hisense_init_irq(void)
{
	sprd_init_irq();
}

static void __init hisense_init(void)
{
	platform_add_devices(devices, ARRAY_SIZE(devices));
	sprd_add_devices();
	sprd_gpio_init();
	sprd_add_sdio_device();
	sprd_gadget_init();
}

static void __init hisense_map_io(void)
{
	sprd_map_common_io();
}

extern unsigned long phys_initrd_start;
extern unsigned long phys_initrd_size;

static void __init
hisense_fixup(struct machine_desc *desc, struct tag *tag,
	    char **cmdline, struct meminfo *mi)
{
#ifdef CONFIG_BLK_DEV_INITRD

	/*
	phys_initrd_start = 0x3000000;
 	phys_initrd_size = 4*1024*1024;
	*/
    
#endif
}

MACHINE_START(HISENSE, "SPRDHS")
/* UART for LL DEBUG */
	.phys_io        = SPRD_SERIAL1_PHYS,
	.io_pg_offst    = ((SPRD_SERIAL1_BASE) >> 18) & 0xfffc,

	.map_io         = hisense_map_io,
	.init_irq       = hisense_init_irq,
	.init_machine   = hisense_init,
	.timer          = &sprd_timer,
	.fixup          = hisense_fixup,
MACHINE_END
