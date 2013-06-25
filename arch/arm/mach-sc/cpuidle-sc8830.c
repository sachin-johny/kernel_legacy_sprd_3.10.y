/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
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

#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/export.h>
#include <linux/clockchips.h>
#include <linux/suspend.h>
#include <asm/proc-fns.h>
#include <mach/sci_glb_regs.h>


/*#define SC_IDLE_DEBUG 1*/

#ifdef SC_IDLE_DEBUG
unsigned int idle_debug_state[NR_CPUS];
#endif

#define SC_CPUIDLE_STATE_NUM		ARRAY_SIZE(cpuidle_params_table)
#define WAIT_WFI_TIMEOUT		(20)

static unsigned int idle_disabled_by_suspend;

/* Machine specific information to be recorded in the C-state driver_data */
struct sc_idle_statedata {
	u32 cpu_state;
	u32 mpu_logic_state;
	u32 mpu_state;
	u8 valid;
};

/*
 * cpuidle mach specific parameters
 * Can board code can override the default C-states definition???
 */
struct cpuidle_params {
	u32 exit_latency;	/* exit_latency = sleep + wake-up latencies */
	u32 target_residency;
	u32 power_usage;
	u8 valid;
};

/*
* TODO: chip parameters
*/
static struct cpuidle_params cpuidle_params_table[] = {
	/* WFI */
	{.exit_latency = 1 , .target_residency = 1, .valid = 1},
	/* LIGHT SLEEP */
	{.exit_latency = 50 , .target_residency = 100, .valid = 1},
	/* CPU CORE POWER DOWN */
	{.exit_latency = 2000 , .target_residency = 5000, .valid = 1},
};


struct sc_idle_statedata sc8830_idle_data[SC_CPUIDLE_STATE_NUM];
char* sc_cpuidle_desc[ SC_CPUIDLE_STATE_NUM] = {
	"standby",
	"l_sleep",
	"core_pd",
};

enum {
	STANDBY = 0,    /* wfi */
	L_SLEEP,	/* light sleep */
	CORE_PD,	/* cpu core power down except cpu0 */
};

static void set_cpu_pd(void *data)
{
	int cpu_id = *((int*)data);
	int on_cpu = smp_processor_id();
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(WAIT_WFI_TIMEOUT);

	if(on_cpu != 0)
		panic(" this function only can excute on cpu0 \n");
	if(cpu_id == 0)
		panic(" cpu0 can not power down \n");

	/*
	 * TODO: set cpu power down
	while (!(__raw_readl(REG_AP_AHB_CA7_STANDBY_STATUS) & (1<<cpu_id))) {
		if (time_after(jiffies, timeout)) {
			printk(" waiting for cpu%d wfi time out \n", cpu_id);
			return;
		}
	}
	printk(" set cpu%d power down\n", cpu_id);
	*/

	return;
}

/*
* TODO:  light sleep enable
*/
static void sc_cpuidle_light_sleep_en(void)
{
	return;
}

/*
* TODO: light sleep disable, light sleep must be disabled before deep-sleep
*/
static void sc_cpuidle_light_sleep_dis(void)
{
	return;
}

/**
 * sc_enter_idle - Programs arm core to enter the specified state
 * @dev: cpuidle device
 * @drv: cpuidle driver
 * @index: the index of state to be entered
 *
 * Called from the CPUidle framework to program the device to the
 * specified low power state selected by the governor.
 * Returns the index of  power state.
 */
static int sc_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv,
			int index)
{
	int cpu_id = smp_processor_id();

	local_irq_disable();
	local_fiq_disable();
#ifdef SC_IDLE_DEBUG
	idle_debug_state[cpu_id] =  index;
	if(index)
		printk("cpu:%d, index:%d \n", cpu_id, index );
#endif

	switch(index){
	case STANDBY:
		cpu_do_idle();
		break;
	case L_SLEEP:
		/*
		*   TODO: enter light sleep
		*/
		sc_cpuidle_light_sleep_en();
		cpu_do_idle();
		sc_cpuidle_light_sleep_dis();
		break;
	case CORE_PD:
		if(cpu_id != 0){
			clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER, &cpu_id);
			/*
			 * TODO:
			 *    set irq affinity to cpu0,  so irq can be handled on cpu0
			 gic_affinity_to_cpu0(cpu_id);
			 smp_call_function_single(0, set_cpu_pd, &cpu_id, 0);
			 */
			cpu_do_idle();
			/*
			 gic_affinity_restore(cpu_id);
			 */
			clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT, &cpu_id);
		}else{
			sc_cpuidle_light_sleep_en();
			cpu_do_idle();
			sc_cpuidle_light_sleep_dis();
		}
		break;
	default:
		cpu_do_idle();
		WARN(1, "CPUIDLE: NO THIS LEVEL!!!");
	}

	local_fiq_enable();
	local_irq_enable();

	return index;
}

DEFINE_PER_CPU(struct cpuidle_device, sc8830_idle_dev);

struct cpuidle_driver sc8830_idle_driver = {
	.name		= "sc_cpuidle",
	.owner		= THIS_MODULE,
};


/*
* sc_fill_cstate: initialize cpu idle status
* @drv: cpuidle_driver
* @idx: cpuidle status index
*/
static inline void sc_fill_cstate(struct cpuidle_driver *drv, struct cpuidle_device *dev, int idx)
{
	char *descr = sc_cpuidle_desc[idx];
	struct cpuidle_state *state = &drv->states[idx];
	struct cpuidle_state_usage *state_usage = &dev->states_usage[idx];

	sprintf(state->name, "C%d", idx + 1);
	strncpy(state->desc, descr, CPUIDLE_DESC_LEN);
	state->flags		= CPUIDLE_FLAG_TIME_VALID;
	state->exit_latency	= cpuidle_params_table[idx].exit_latency;
	/*TODO*/
	/*state->power_usage = cpuidle_params_table[idx].power_usage;*/
	state->target_residency	= cpuidle_params_table[idx].target_residency;
	state->enter		= sc_enter_idle;

	/*TODO*/
	/*state_usage->driver_data =  ;*/
}

static int sc_cpuidle_register_device(struct cpuidle_driver *drv, unsigned int cpu)
{
	struct cpuidle_device *dev;
	int state_idx;

	dev =  &per_cpu(sc8830_idle_dev, cpu);
	dev->cpu = cpu;
	pr_info("%s, cpu:%d \n", __func__, cpu);
	for(state_idx=0; state_idx<SC_CPUIDLE_STATE_NUM; state_idx++){
		sc_fill_cstate(drv, dev, state_idx);
	}
	dev->state_count = state_idx;

	if (cpuidle_register_device(dev)) {
		pr_err("CPU%u: failed to register idle device\n", cpu);
		return -EIO;
	}

	return 0;
}

static int sc_cpuidle_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
#ifdef CONFIG_PM_SLEEP
	if (event == PM_SUSPEND_PREPARE)
		idle_disabled_by_suspend = true;
	else if (event == PM_POST_SUSPEND)
		idle_disabled_by_suspend = false;
#endif

	return NOTIFY_OK;
}

static struct notifier_block sc_cpuidle_pm_notifier = {
	.notifier_call = sc_cpuidle_pm_notify,
};

int __init sc_cpuidle_init(void)
{
	unsigned int cpu_id = 0;
	struct cpuidle_driver *drv = &sc8830_idle_driver;
	drv->safe_state_index = 0;
	drv->state_count = SC_CPUIDLE_STATE_NUM;
	pr_info("%s, enter,  drv->state_count:%d \n", __func__, drv->state_count );

	cpuidle_register_driver(drv);

	for_each_possible_cpu(cpu_id) {
		if (sc_cpuidle_register_device(drv, cpu_id))
			pr_err("CPU%u: error initializing idle loop\n", cpu_id);
	}

	register_pm_notifier(&sc_cpuidle_pm_notifier);

	return 0;

}
