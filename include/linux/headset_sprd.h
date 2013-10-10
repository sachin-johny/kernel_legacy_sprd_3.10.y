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

//Bug 185497, step 0: Button state
enum {
	HEADSET_BUTTON_STATE_RELASED = 0,
	HEADSET_BUTTON_STATE_PRESSED = 1,
};

struct headset_button {
	int adc_min;
	int adc_max;
	int code;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int state;          /* Bug 185497, step 0: Record Headset button state, 0: RELASED, 1:PRESSED */
};

struct sprd_headset_buttons_platform_data {
	/* Configuration parameters */
	struct headset_button *headset_button;
	int nbuttons;
	const char *vdd_name;
	const char *desc;
};

struct sprd_headset_detect_platform_data {
	int switch_gpio;
	int detect_gpio;
	int button_gpio;
	const char *vdd_name;
	int detect_active_low;
	int button_active_low;
};

struct headset_button_data {
	struct sprd_headset_buttons_platform_data *platform_data;
	struct input_dev *input_dev;
	struct work_struct work;
	struct timer_list timer;
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

extern int register_headset_plug_notifier(struct notifier_block *nb);
extern int unregister_headset_plug_notifier(struct notifier_block *nb);

#endif
