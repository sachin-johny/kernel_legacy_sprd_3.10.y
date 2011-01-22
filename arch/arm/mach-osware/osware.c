/*
 *  linux/arch/arm/mach-osware/osware.c
 *
 *  Copyright (C) 2006 Jaluna SA.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/memory.h>
#include <asm/setup.h>
#include <asm/io.h>

#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>

extern void nk_ddi_init(void);

/*
 * Generic OSware timer.
 */
#ifdef CONFIG_GENERIC_CLOCKEVENTS
extern struct sys_timer nk_vtick_timer;
#else
extern struct sys_timer osware_timer;
#endif

/*
 * IRQ Initialization.
 */
static void __init osware_init_irq(void)
{
    nk_ddi_init();
}

#ifdef CONFIG_VBATTERY_FRONTEND
static struct platform_device vbattery_fe_device = {
	.name		= "vbattery-fe",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
};
#endif

/*
 * Initialization call.
 */
static void __init osware_init(void)
{
#ifdef CONFIG_VBATTERY_FRONTEND
	platform_device_register(&vbattery_fe_device);
#endif
}

/*
 * Generic OSware machine description.
 */
MACHINE_START(ARM_OSWARE, "Generic ARM OSware Machine")
	.init_irq	= osware_init_irq,
#ifdef CONFIG_GENERIC_CLOCKEVENTS
	.timer		= &nk_vtick_timer,
#else
	.timer		= &osware_timer,
#endif
	.init_machine	= osware_init,
MACHINE_END
