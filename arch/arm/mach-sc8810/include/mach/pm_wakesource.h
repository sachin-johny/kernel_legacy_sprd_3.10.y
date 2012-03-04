/* include/arch/arm/mach_../pm_wakesource.h
 *
 * Copyright (C) 2001-2011 Spreadtrum, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _PM_WAKE_SOURCE_H
#define _PM_WAKE_SOURCE_H

#include <linux/list.h>

struct wake_source {
	struct list_head link;
	void* param;
	void (*set)(struct wake_source *h);
	void (*clear)(struct wake_source *h);
};

void register_wake_source(struct wake_source *handler);
void unregister_wake_source(struct wake_source *handler);
void wake_source_set(void);
void wake_source_clr(void);

#endif

