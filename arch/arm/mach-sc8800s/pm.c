/*
 * sc8800s Power Management Routines
 * * 
 * Copyright (c) 2010 Spreadtrum, Inc.
 *
 * created for sc8800s, 2010-09-07
 * Wang Liwei.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>

/* #include <mach/pm.h> */

#include <asm/io.h>
#include <mach/regs_ahb.h>

extern void sc8800s_cpu_standby(void);
#ifdef        CONFIG_DEBUG_LL
extern void printascii(char *);
#endif


int sc8800s_pm_enter(suspend_state_t state)
{
	/* *** go zzz *** */
	local_fiq_disable();
    sc8800s_cpu_standby();
	local_fiq_enable();
	return 0;
}

EXPORT_SYMBOL_GPL(sc8800s_pm_enter);

unsigned long sleep_phys_sp(void *sp)
{
	return virt_to_phys(sp);
}

static int sc8800s_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM || state == PM_SUSPEND_STANDBY;
}

int sc8800s_pm_prepare(void)
{
	int ret = 0;
	return ret;
}

void sc8800s_pm_finish(void)
{

}

static struct platform_suspend_ops sc8800s_pm_ops = {
	.valid		= sc8800s_pm_valid,
	.enter		= sc8800s_pm_enter,
	.prepare	= sc8800s_pm_prepare,
	.finish		= sc8800s_pm_finish,
};

static int __init sc8800s_pm_init(void)
{
#ifdef CONFIG_SUSPEND
	suspend_set_ops(&sc8800s_pm_ops);
#endif /* CONFIG_SUSPEND */
	return 0;
}

device_initcall(sc8800s_pm_init);
