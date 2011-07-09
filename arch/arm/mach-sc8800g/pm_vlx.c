/*
 * sc8800s Power Management Routines
 * * 
 * Copyright (c) 2010 Spreadtrum, Inc.
 *
 * created for sc8800g, 2010-09-07
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
#include <linux/wakelock.h>
#include <mach/pm.h>

#include <asm/io.h>
#include <mach/regs_ahb.h>
#include <mach/test.h>


/*
#define CONFIG_PM_SC8800G_TEST_MODE 0
*/

extern void sc8800g_cpu_standby(void);

extern int prepare_deep_sleep(void);


extern int prepare_deep_sleep(void);
extern int sc8800g_prepare_deep_sleep(void);
extern int sc8800g_enter_deepsleep(int);

int sc8800g_pm_enter(suspend_state_t state)
{
	int ret_val = 0;
	u32 suspend_start, suspend_end, suspend_time;
	unsigned long flags;

	suspend_start = suspend_end = get_sys_cnt();
	suspend_time = suspend_end - suspend_start;
	/*
	irq has been disabled,
	so we check irq status here safely.
	 */
	while((0 == sprd_suspend_interval) ||
		  (suspend_time < sprd_suspend_interval)) {
		hw_local_irq_disable();

		local_irq_save(flags);
		if (raw_local_irq_pending()) {
			/*
			printk("*******: pm_enter(), irq pending! *****\n");
			*/
			local_irq_restore(flags);
			hw_local_irq_enable();
			break;
		}
		local_irq_restore(flags);
		WARN_ONCE(!irqs_disabled(),
			"#####: Interrupts enabled in pm_enter()!\n");
	
		ret_val = os_ctx->idle(os_ctx);
		if (0 == ret_val) {
			sc8800g_enter_deepsleep(0);
		}
		suspend_end = get_sys_cnt();
		suspend_time = suspend_end - suspend_start;

		hw_local_irq_enable();
	}
	return ret_val;
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
	.prepare	= sc8800g_pm_prepare,
	.finish		= sc8800g_pm_finish,
};


/* APIs for cpuIdle. */

int sc8800g_enter_sleep(sc88xx_state_t state)
{
	return 0;
}

EXPORT_SYMBOL_GPL(sc8800g_enter_sleep);

static int __init sc8800g_pm_init(void)
{
/* prepare for deep sleep */
    sc8800g_prepare_deep_sleep();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&sc8800g_pm_ops);
#endif /* CONFIG_SUSPEND */

    return 0;
}

device_initcall(sc8800g_pm_init);
