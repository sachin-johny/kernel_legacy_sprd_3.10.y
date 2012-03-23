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

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/time.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>

#include "devices.h"

extern void __init sc8810_map_io(void);
extern void __init sc8810_init_irq(void);
extern void __init sc8810_timer_init(void);
extern void __init regulator_add_devices(void);

static struct platform_device *devices[] __initdata = {
	&sprd_device_example,
	&sprd_serial_device1,
	&sprd_serial_device2,
	&sprd_serial_device3,
	&sprd_device_rtc,
	&sprd_nand_device,
	&sprd_lcd_device,
	&sprd_backlight_device,
	&sprd_i2c_device0,
	&sprd_i2c_device1,
	&sprd_i2c_device2,
};

static struct sys_timer sc8810_timer = {
	        .init = sc8810_timer_init,
};

static int calibration_mode = false;
static int __init calibration_start(char *str)
{
        if(str)
                pr_info("modem calibartion:%s\n", str);
        calibration_mode = true;
        return 1;
}
__setup("calibration=", calibration_start);

int in_calibration(void)
{
     return (calibration_mode == true);
}

EXPORT_SYMBOL(in_calibration);

static void __init sprd_add_otg_device(void)
{
	/*
	 * if in calibrtaion mode, we do nothing, modem will handle everything
	 */
	if (calibration_mode)
		return;
	platform_device_register(&sprd_otg_device);
}

static void __init sc8810_init_machine(void)
{
	regulator_add_devices();
	sprd_add_otg_device();
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

static void __init sc8810_fixup(struct machine_desc *desc, struct tag *tag,
		char **cmdline, struct meminfo *mi)
{
}

MACHINE_START(SC8810OPENPHONE, "SP8810")
	.map_io		= sc8810_map_io,
	.init_irq	= sc8810_init_irq,
	.timer		= &sc8810_timer,
	.init_machine	= sc8810_init_machine,
	.fixup		= sc8810_fixup,
MACHINE_END
