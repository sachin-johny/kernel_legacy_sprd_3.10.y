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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>

#include <mach/sci.h>
#include <mach/hardware.h>
#include <mach/regs_glb.h>

static DEFINE_SPINLOCK(glb_lock);
static struct hwspinlock *glb_hwlock = NULL;

#ifdef CONFIG_NKERNEL
#define sci_glb_lock()				\
		spin_lock_irqsave(&glb_lock, flags);\
		if (glb_hwlock) WARN_ON(IS_ERR_VALUE(hwspin_lock_timeout(glb_hwlock, -1)));\
		hw_flags = hw_local_irq_save()
#define sci_glb_unlock()			\
		hw_local_irq_restore(hw_flags);		\
		if (glb_hwlock) hwspin_unlock(glb_hwlock);	\
		spin_unlock_irqrestore(&glb_lock, flags)
#else
#define sci_glb_lock() 		do {spin_lock_irqsave(&glb_lock,flags);} while(0)
#define sci_glb_unlock() 	do {spin_unlock_irqrestore(&glb_lock,flags);} while(0)
#endif

u32 sci_glb_read(u32 reg, u32 msk)
{
	return __raw_readl(reg) & msk;
}

int sci_glb_write(u32 reg, u32 val, u32 msk)
{
	unsigned long flags, hw_flags;
	sci_glb_lock();
	__raw_writel((__raw_readl(reg) & ~msk) | val, reg);
	sci_glb_unlock();
	return 0;
}

int sci_glb_set(u32 reg, u32 bit)
{
	__raw_writel(bit, REG_GLB_SET(reg));
	return 0;
}

int sci_glb_clr(u32 reg, u32 bit)
{
	__raw_writel(bit, REG_GLB_CLR(reg));
	return 0;
}

EXPORT_SYMBOL(sci_glb_read);
EXPORT_SYMBOL(sci_glb_write);
EXPORT_SYMBOL(sci_glb_set);
EXPORT_SYMBOL(sci_glb_clr);

static int __init glb_hwlock_init(void)
{
	glb_hwlock = hwspin_lock_request_specific(1);
	if (WARN_ON(IS_ERR_OR_NULL(glb_hwlock)))
		glb_hwlock = NULL;
	else
		pr_info("glb hwspinlock id %d\n", hwspin_lock_get_id(glb_hwlock));
	return 0;
}

arch_initcall_sync(glb_hwlock_init);

