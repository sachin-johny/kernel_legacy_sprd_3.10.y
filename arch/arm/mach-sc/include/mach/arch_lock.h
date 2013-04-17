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

#ifndef __ARCH_LOCK_H
#define __ARCH_LOCK_H

#include <linux/hwspinlock.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/regs_ahb.h>
#include <mach/sci.h>


extern struct hwspinlock *hwlocks[];
extern unsigned char hwlocks_implemented[];

#define	arch_get_hwlock(_ID_)	(hwlocks[_ID_])
#define FILL_HWLOCKS(_X_)	do {hwlocks_implemented[(_X_)] = 1;} while(0)

#if	defined (CONFIG_ARCH_SC8825)
#define HWLOCK_ADDR(_X_)	(SPRD_HWLOCK_BASE + (0x80 + 0x4*(_X_)))
static __inline __init int __hwspinlock_init(void)
{
	sci_glb_set(REG_AHB_AHB_CTL0, BIT_SPINLOCK_EB);
	return 0;
}

#else

#include <mach/regs_sc8830_ap_ahb.h>
#include <mach/regs_sc8830_aon_apb.h>

static __inline unsigned long HWLOCK_ADDR(unsigned int id)
{
	BUG_ON(id > 63);
	if (id < 31)
		return SPRD_HWLOCK1_BASE + (0x800 + 0x4*(id));
	else
		return SPRD_HWLOCK0_BASE + (0x800 + 0x4*(id));
}

static __inline __init int __hwspinlock_init(void)
{
	sci_glb_set(REG_AP_AHB_AHB_EB, BIT_SPINLOCK_EB);
	sci_glb_set(REG_AON_APB_APB_EB0, BIT_SPLK_EB);
	return 0;
}
#endif

//Configs lock id
#define HWSPINLOCK_WRITE_KEY	(0x1)	/*processor specific write lock id */

#define HWSPINLOCK_NOTTAKEN_V0		(0x524c534c)	/*free: RLSL */
#define HWSPINLOCK_NOTTAKEN_V1		(0x55aa10c5)	/*free: RLSL */

#define HWSPINLOCK_ID_TOTAL_NUMS	(64)
#define HWLOCK_ADI	(0)
#define HWLOCK_GLB	(1)
#define HWLOCK_AGPIO	(2)
#define HWLOCK_AEIC	(3)
#define HWLOCK_ADC	(4)

static inline void arch_hwlocks_implemented(void)
{
	FILL_HWLOCKS(HWLOCK_ADI);
	FILL_HWLOCKS(HWLOCK_GLB);
	FILL_HWLOCKS(HWLOCK_AGPIO);
	FILL_HWLOCKS(HWLOCK_AEIC);
	FILL_HWLOCKS(HWLOCK_AGPIO);
	FILL_HWLOCKS(HWLOCK_ADC);
}

#if	defined (CONFIG_ARCH_SC8825)
static inline int arch_hwlock_fast_trylock(unsigned int lock_id)
{
	unsigned long addr = HWLOCK_ADDR(lock_id);
	sci_glb_set(REG_AHB_AHB_CTL0, BIT_SPINLOCK_EB);
	if (HWSPINLOCK_NOTTAKEN_V0 == __raw_readl(addr)) {
		__raw_writel(HWSPINLOCK_WRITE_KEY, addr);
		if (HWSPINLOCK_WRITE_KEY == __raw_readl(addr)) {
			dsb();
			return 1;
		}
	}
	return 0;
}

static inline void arch_hwlock_fast_unlock(unsigned int lock_id)
{
	unsigned long addr = HWLOCK_ADDR(lock_id);
	dsb();
	__raw_writel(HWSPINLOCK_NOTTAKEN_V0, addr);
}
#else
static inline int arch_hwlock_fast_trylock(unsigned int lock_id)
{
	unsigned long addr = HWLOCK_ADDR(lock_id);
	sci_glb_set(REG_AHB_AHB_CTL0, BIT_SPINLOCK_EB);

	if (!readl(addr)) {
		dsb();
		return 1;
	} else
		return 0;
}

static inline void arch_hwlock_fast_unlock(unsigned int lock_id)
{
	unsigned long addr = HWLOCK_ADDR(lock_id);
	dsb();
	__raw_writel(HWSPINLOCK_NOTTAKEN_V1, addr);
}
#endif

#define arch_hwlock_fast(_LOCK_ID_) do { \
	while (!arch_hwlock_fast_trylock(_LOCK_ID_)) \
	cpu_relax();} while (0)

#define arch_hwunlock_fast(_LOCK_ID_) do { \
				arch_hwlock_fast_unlock(_LOCK_ID_);} while (0)

#endif
