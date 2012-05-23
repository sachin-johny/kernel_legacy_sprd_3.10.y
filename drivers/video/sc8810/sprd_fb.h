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

#ifndef _SPRDFB_H_
#define _SPRDFB_H_

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/lcd.h>
#include <linux/clk.h>
#include <linux/earlysuspend.h>

#if CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#endif


#define FB_NORMAL 0
#define FB_NO_REFRESH 1

struct sprdfb_device {
	struct fb_info  *fb;
	struct ops_mcu  *ops;
	struct lcd_spec *panel;

	int 	open;

	int	id;
	uint32_t width;
	uint32_t height;
	uint32_t bpp;
	uint32_t vsync_waiter;
	
	uint32_t device_id;
	uint32_t timing[2];
	uint32_t reserved[2];	
	uint32_t run;
	void (*update_lcm)(struct sprdfb_device *dev);
	uint32_t fb_state;
	uint32_t need_reinit;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

#if CONFIG_CPU_FREQ
	struct notifier_block freq_transition;
	struct notifier_block freq_policy;
#endif
};

#endif
