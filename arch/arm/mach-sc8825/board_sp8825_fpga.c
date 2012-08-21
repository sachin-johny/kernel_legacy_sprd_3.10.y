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
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/globalregs.h>
#include <mach/board.h>
#include "devices.h"

extern void __init sc8825_reserve(void);
extern void __init sc8825_map_io(void);
extern void __init sc8825_init_irq(void);
extern void __init regulator_add_devices(void);
extern void __init sc8825_clock_init(void);

static struct platform_device *devices[] __initdata = {
	&sprd_serial_device0,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_serial_device3,
};

static void __init sc8825_init_machine(void)
{
	int clk;
	platform_add_devices(devices, ARRAY_SIZE(devices));
	clk=48000000;
	platform_device_add_data(&sprd_serial_device0,(const void*)&clk,sizeof(int));
	/* clk=26000000; */
	platform_device_add_data(&sprd_serial_device1,(const void*)&clk,sizeof(int));
	platform_device_add_data(&sprd_serial_device2,(const void*)&clk,sizeof(int));
}

static void __init sc8825_fixup(struct machine_desc *desc, struct tag *tag,
		char **cmdline, struct meminfo *mi)
{
}

static struct sys_timer sc8825_timer = {
		/* .init = sc8825_timer_init, */
};

static void __init sc8825_init_early(void)
{
	/* earlier init request than irq and timer */
	/* sc8825_clock_init(); */

}

MACHINE_START(SC8825OPENPHONE, "SP8825")
	/* .reserve	= sc8825_reserve, */
	.map_io		= sc8825_map_io,
	/* .init_irq	= sc8825_init_irq, */
	.timer		= &sc8825_timer,
	.init_machine	= sc8825_init_machine,
	.fixup		= sc8825_fixup,
	.init_early	= sc8825_init_early,
MACHINE_END
