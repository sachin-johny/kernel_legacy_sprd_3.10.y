/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 * Author: steve.zhan <steve.zhan@spreadtrum.com>
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
#include <linux/bug.h>

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

#define ARCH_SC_SOC_IO_MAP
static struct map_desc sprd_io_desc[] __initdata = {	
#if defined(CONFIG_ARCH_SC8830) 
#include "mach/__io-sc8830.h"
#elif defined(CONFIG_ARCH_SC8825)
#include "mach/__io-sc8825.h"
#else
#error "Unknown architecture specification"
#endif
};

void __init sci_map_io(void)
{
	iotable_init(sprd_io_desc, ARRAY_SIZE(sprd_io_desc));
}

