/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#if defined(CONFIG_ARCH_SC7710)
int ldo_trimming_callback(void *data)
{
	return 0;
}
#endif

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Spreadtrum dcdc/ldo add-on modules");
MODULE_AUTHOR("robot <zhulin.lian@spreadtrum.com>");
