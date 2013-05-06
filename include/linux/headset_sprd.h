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

#ifndef __HEADSET_SPRD_H__
#define __HEADSET_SPRD_H__
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/platform_device.h>

struct headset_button {
	int adc_min;
	int adc_max;
	int code;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
};

struct sprd_headset_buttons_platform_data {
	/* Configuration parameters */
	struct headset_button *headset_button;
	int nbuttons;
	int active_low;
	const char *vdd_name;
	const char *desc;
};

struct sprd_headset_detect_platform_data {
	int switch_gpio;
	const char *vdd_name;
	int active_low;
};

struct headset_button_data {
	struct sprd_headset_buttons_platform_data *platform_data;
	struct input_dev *input;
	struct work_struct work;
	struct timer_list timer;
	int active_low;
	int irq;
};

struct headset_detect_data {
	struct sprd_headset_detect_platform_data *platform_data;
	struct switch_dev sdev;
	struct work_struct work;
	struct timer_list timer;
	struct timer_list irq_timer;
	int headphone;
	int irq;
	int type;
};
enum {
	BIT_HEADSET_OUT = 0,
	BIT_HEADSET_MIC = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};
struct sprd_headset {
	struct headset_detect_data detect;
	struct headset_button_data button;
};

#endif
