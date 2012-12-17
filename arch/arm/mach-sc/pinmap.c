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
#include <linux/io.h>
#include <linux/module.h>

#include <mach/hardware.h>
#include <mach/pinmap.h>

typedef struct {
	uint32_t reg;
	uint32_t val;
} pinmap_t;

pinmap_t __initconst pinmap[] = {
#if defined (CONFIG_MACH_SP8830EA) || defined (CONFIG_MACH_SP8830FPGA)
#include "mach/__pinmap-sp8830ea.h"
#elif defined (CONFIG_MACH_SP8825EA)
#include "mach/__pinmap-sp8825ea.h"
#endif
};

static int __init pin_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pinmap); i++) {
		__raw_writel(pinmap[i].val, CTL_PIN_BASE + pinmap[i].reg);
	}
	return 0;
}

arch_initcall(pin_init);
