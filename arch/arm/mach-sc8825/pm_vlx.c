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
#include <linux/io.h>
#include <asm/irqflags.h>
#include <mach/pm_debug.h>
#include <mach/globalregs.h>
#include <mach/regs_ahb.h>
#include <mach/sci.h>
#include <mach/hardware.h>
#include <linux/delay.h>

#define CORE1_RUN		(1)


extern void l2x0_suspend(void);
extern void l2x0_resume(int collapsed);
extern int sprd_cpu_deep_sleep(unsigned int cpu);
extern void sc8825_pm_init(void);
extern void check_ldo(void);
extern void check_pd(void);

/*for battery*/
#define BATTERY_CHECK_INTERVAL 30000
extern int battery_updata(void);
extern void battery_sleep(void);

static int sprd_check_battery(void)
{
	int ret_val = 0;
	if (battery_updata()) {
		ret_val = 1;
	}
	return ret_val;
}

static void sprd_pm_standby(void){
	cpu_do_idle();
}

static int sprd_pm_deepsleep(suspend_state_t state)
{
	int ret_val = 0;
	unsigned long hwflags, flags;
	u32 holding_pen, battery_time, cur_time;
	unsigned int cpu;

	battery_time = cur_time = get_sys_cnt();
	cpu = smp_processor_id();

	/* add for debug & statisic*/
	clr_sleep_mode();
	time_statisic_begin();

#ifndef CONFIG_NKERNEL_PM_MASTER
	hwflags = hw_local_irq_save();
	os_ctx->suspend_to_memory(os_ctx);
	hw_local_irq_restore(hwflags);
#else	/* PM MASTER */

	/*
	* when we get here, only boot cpu still alive
	*/
	if (smp_processor_id()) {
		__WARN();
		goto enter_exit;
	}

#ifdef CONFIG_CACHE_L2X0
	/* guarantee cpu1 is in wfi */
	mdelay(10);
	printk("PM: l2 suspending...\n");
	hwflags = hw_local_irq_save();
	holding_pen = sci_glb_read(REG_AHB_HOLDING_PEN, -1UL);
	while(holding_pen & CORE1_RUN ){
		holding_pen = sci_glb_read(REG_AHB_HOLDING_PEN, -1UL);
		continue;
	}
	l2x0_suspend();
	hw_local_irq_restore(hwflags);
#endif

	while(1){
		hwflags = hw_local_irq_save();
		local_fiq_disable();
		local_irq_save(flags);

		if (arch_local_irq_pending()) {
			/* add for debug & statisic*/
			irq_wakeup_set();

			local_irq_restore(flags);
			hw_local_irq_restore(hwflags);
			break;
		}else{
			local_irq_restore(flags);
			WARN_ONCE(!irqs_disabled(), "#####: Interrupts enabled in pm_enter()!\n");
			/*
			* return value 0 means that other guest OS  are all idle
			*/
			ret_val = os_ctx->idle(os_ctx);
			if (0 == ret_val) {
				os_ctx->smp_cpu_stop(0);
				ret_val = sprd_cpu_deep_sleep(cpu);
				os_ctx->smp_cpu_start(0, 0);/* the 2nd parameter is meaningless*/
			} else {
				printk("******** os_ctx->idle return %d ********\n", ret_val);
			}
			hw_local_irq_restore(hwflags);
		}

		print_hard_irq_inloop(ret_val);

		battery_sleep();
		cur_time = get_sys_cnt();
		if ((cur_time -  battery_time) > BATTERY_CHECK_INTERVAL) {
			battery_time = cur_time;
			if (sprd_check_battery()) {
				printk("###: battery low!\n");
				break;
			}
		}
	}/*end while*/

#ifdef CONFIG_CACHE_L2X0
	printk("PM: l2 resuming...\n");
	hwflags = hw_local_irq_save();
	l2x0_resume(1);
	hw_local_irq_restore(hwflags);
#endif
	time_statisic_end();

#endif

enter_exit:
	return ret_val;
}


static int sprd_pm_enter(suspend_state_t state)
{
	int rval = 0;
	switch (state) {
		case PM_SUSPEND_STANDBY:
			sprd_pm_standby( );
			break;
		case PM_SUSPEND_MEM:
			rval = sprd_pm_deepsleep(state);
			break;
		default:
			break;
	}

	return rval;
}

static int sprd_pm_valid(suspend_state_t state)
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

static int sprd_pm_prepare(void)
{
	pr_debug("enter %s\n", __func__);
	check_ldo();
	check_pd();
	return 0;
}

static void sprd_pm_finish(void)
{
	pr_debug("enter %s\n", __func__);
	printk("enter %s\n", __func__);
	print_statisic();
}

static struct platform_suspend_ops sprd_pm_ops = {
	.valid		= sprd_pm_valid,
	.enter		= sprd_pm_enter,
	.prepare	= sprd_pm_prepare,
	.prepare_late 	= NULL,
	.finish		= sprd_pm_finish,
};

static int __init pm_init(void)
{
	sc8825_pm_init();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&sprd_pm_ops);
#endif

	return 0;
}

device_initcall(pm_init);
