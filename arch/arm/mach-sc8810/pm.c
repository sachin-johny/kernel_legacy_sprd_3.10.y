/*
 * sc8810 Power Management Routines
 * * 
 * Copyright (c) 2011 Spreadtrum, Inc.
 *
 * created for sc8810, 2011
 * steve.zhan
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <mach/pm.h>

#include <asm/io.h>
#include <mach/regs_ahb.h>
extern int sp_pm_collapse(void);
extern void sp_pm_collapse_exit(void);
static uint32_t *sp_pm_reset_vector = NULL;

static uint32_t saved_vector[2];
void sc8810_enter_dormant(void)
{
	uint32_t isDormantExit = 0;
	if (!sp_pm_reset_vector) {
		sp_pm_reset_vector = ioremap(0xffff0000, PAGE_SIZE);
		if (sp_pm_reset_vector == NULL) {
			printk(KERN_ERR "sp_pm_init: failed to map reset vector\n");
			return 0;
		}
	}
	printk(KERN_INFO "sc8810_enter_dormant\n");

	raw_local_irq_disable();
	local_fiq_disable();

	saved_vector[0] = sp_pm_reset_vector[0];
	saved_vector[1] = sp_pm_reset_vector[1];
	sp_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
	sp_pm_reset_vector[1] = virt_to_phys(sp_pm_collapse_exit);
	isDormantExit = sp_pm_collapse();
	sp_pm_reset_vector[0] = saved_vector[0];
	sp_pm_reset_vector[1] = saved_vector[1];
	if (isDormantExit) {
		cpu_init();
		local_fiq_enable();
		raw_local_irq_enable();
		printk(KERN_INFO "wakeup1\n");
	}
	else {
		local_fiq_enable();
		raw_local_irq_enable();
	}
	printk(KERN_INFO "wakeup2\n");

}

extern void sc8800g_cpu_standby(void);
#ifdef        CONFIG_DEBUG_LL
extern void printascii(char *);
#endif


int sc8800g_pm_enter(suspend_state_t state)
{
	/* *** go zzz *** */
#if 0
	local_fiq_disable();
	sc8800g_cpu_standby();
	local_fiq_enable();
#else
	static int entercnt = 0;
	if (entercnt ++ > 30)
		sc8810_enter_dormant();
#endif
	return 0;
}

EXPORT_SYMBOL_GPL(sc8800g_pm_enter);


static int sc8800g_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM || state == PM_SUSPEND_STANDBY;
}

int sc8800g_pm_prepare(void)
{
	int ret = 0;
	return ret;
}

void sc8800g_pm_finish(void)
{

}

static struct platform_suspend_ops sc8800g_pm_ops = {
	.valid		= sc8800g_pm_valid,
	.enter		= sc8800g_pm_enter,
	.prepare		= sc8800g_pm_prepare,
	.finish		= sc8800g_pm_finish,
};


/* APIs for cpuIdle. */

int sc8800g_enter_sleep(sc88xx_state_t state)
{
	sc8810_enter_dormant();
	return 0;
}

EXPORT_SYMBOL_GPL(sc8800g_enter_sleep);

static int __init sc8800g_pm_init(void)
{
#ifdef CONFIG_SUSPEND
//	suspend_set_ops(&sc8800g_pm_ops);
#endif /* CONFIG_SUSPEND */
	return 0;
}

device_initcall(sc8800g_pm_init);
