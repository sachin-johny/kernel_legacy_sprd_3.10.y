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

#include <asm/io.h>
#include <asm/page.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>

#define SPRD_DEVICE(name) { \
	.virtual = SPRD_##name##_BASE, \
	.pfn = __phys_to_pfn(SPRD_##name##_PHYS), \
	.length = SPRD_##name##_SIZE, \
	.type = MT_DEVICE_NONSHARED, \
	}

#define SPRD_IRAM(name) { \
	.virtual = SPRD_##name##_BASE, \
	.pfn = __phys_to_pfn(SPRD_##name##_PHYS), \
	.length = SPRD_##name##_SIZE, \
	.type = MT_MEMORY, \
	}

static struct map_desc sprd_io_desc[] __initdata = {
	SPRD_DEVICE(UART0),
	SPRD_DEVICE(UART1),
	SPRD_DEVICE(UART2),
	SPRD_DEVICE(UART3),
};

void __init sc8825_map_io(void)
{
	iotable_init(sprd_io_desc, ARRAY_SIZE(sprd_io_desc));
}
