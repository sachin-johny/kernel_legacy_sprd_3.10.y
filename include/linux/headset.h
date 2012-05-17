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

#ifndef __HEADSET_H__
#define __HEADSET_H__
enum {
	BIT_HEADSET_OUT = 0,
	BIT_HEADSET_MIC = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

struct _headset_gpio {
	int active_low;
	int gpio;
	int irq;
	unsigned int irq_type_active;
	unsigned int irq_type_inactive;
	int debounce;
	int debounce_sw;
	int holded;
	int actived;
	int actived_count;
	int pstatus;
	int irq_enabled;
	const char *desc;
	struct _headset *parent;
	unsigned int timeout_ms;
	struct hrtimer timer;
};

struct _headset {
	struct switch_dev sdev;
	struct input_dev *input;
	struct _headset_gpio detect;
	struct _headset_gpio button;
	int headphone;
};
#endif
