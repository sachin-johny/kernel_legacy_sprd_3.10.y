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
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include "devices.h"

/* example platform device & its resource */
static struct resource resources_example[] = {
	[0] = {
		.start	= IRQ_SOFT_TRIG_INT,
		.end	= IRQ_SOFT_TRIG_INT,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= SPRD_BUSM1_BASE,
		.end	= SPRD_BUSM1_BASE + SPRD_BUSM1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device sprd_device_example = {
	.name		= "sprd_example",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_example),
	.resource	= resources_example,
};
