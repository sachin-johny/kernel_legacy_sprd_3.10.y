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

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <asm/io.h>

static void sc8810_sleep(void){
	cpu_do_idle();
}

/* FIXME: need more implementation for deep sleep */
static int sc8810_deep_sleep(void){
	cpu_do_idle();
	return 0;
}

static int sc8810_pm_enter(suspend_state_t state)
{
	int rval = 0;

	pr_debug("pm_enter: %d\n", state);
	switch (state) {
		case PM_SUSPEND_STANDBY:
			sc8810_sleep();
			break;
		case PM_SUSPEND_MEM:
			rval = sc8810_deep_sleep();
			break;
		default:
			break;
	}

	return rval;
}

static int sc8810_pm_valid(suspend_state_t state)
{
	pr_debug("pm_valid: %d\n", state);
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;
		default:
			return 0;
	}
}

static int sc8810_pm_prepare(void)
{
	pr_debug("enter %s\n", __func__);
	return 0;
}

static void sc8810_pm_finish(void)
{
	pr_debug("enter %s\n", __func__);
}

static struct platform_suspend_ops sc8810_pm_ops = {
	.valid		= sc8810_pm_valid,
	.enter		= sc8810_pm_enter,
	.prepare	= sc8810_pm_prepare,
	.finish		= sc8810_pm_finish,
};

void sc8810_idle(void)
{
	cpu_do_idle();
}

static int __init sc8810_pm_init(void)
{
	/* FIXME: need more initialization for deep sleep */

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&sc8810_pm_ops);
#endif

	return 0;
}

device_initcall(sc8810_pm_init);
