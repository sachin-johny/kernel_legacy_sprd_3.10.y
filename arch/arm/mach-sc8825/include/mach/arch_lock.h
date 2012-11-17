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

#ifndef __ARCH_LOCK_H
#define __ARCH_LOCK_H

#include <linux/hwspinlock.h>

#define SHIFT_ID(_X_) (1<<(_X_))

#define	arch_get_hwlock(_ID_)	(hwlocks[_ID_])
extern struct hwspinlock *hwlocks[];

//Configs lock id
#define HWSPINLOCK_WRITE_KEY	(0x11111111)	/*processor specific write lock id */

#define HWLOCK_ADI	(0)
#define HWLOCK_GLB	(1)
#define HWLOCK_AGPIO	(2)
#define HWLOCK_AEIC	(3)
#define HWLOCK_ADC	(4)

static inline int arch_hwlocks_implemented(void)
{
	return (SHIFT_ID(HWLOCK_ADI) | SHIFT_ID(HWLOCK_GLB) |
		SHIFT_ID(HWLOCK_AGPIO) | SHIFT_ID(HWLOCK_AEIC) |
		SHIFT_ID(HWLOCK_ADC));
}

#define HWSPINLOCK_ID_TOTAL_NUMS	(32)

#endif
