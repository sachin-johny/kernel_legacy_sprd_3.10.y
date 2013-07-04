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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/regulator/consumer.h>
#include <asm/system.h>
#include <trace/events/power.h>

#include <mach/hardware.h>
#include <mach/regulator.h>
#include <mach/adi.h>
#include <mach/sci.h>
#include <mach/sci_glb_regs.h>



#if defined(CONFIG_ARCH_SC8825)

#define MHz                     (1000000)
#define GR_MPLL_REFIN_2M        (2 * MHz)
#define GR_MPLL_REFIN_4M        (4 * MHz)
#define GR_MPLL_REFIN_13M       (13 * MHz)
#define GR_MPLL_REFIN_SHIFT     16
#define GR_MPLL_REFIN_MASK      (0x3)
#define GR_MPLL_N_MASK          (0x7ff)
#define GR_MPLL_MN				(REG_GLB_M_PLL_CTL0)
#define GR_GEN1					(REG_GLB_GEN1)


static void set_mcu_clk_freq(u32 mcu_freq)
{
	u32 val, rate, arm_clk_div, gr_gen1;

	rate = mcu_freq / MHz;
	switch(1000 / rate)
	{
		case 1:
			arm_clk_div = 0;
			break;
		case 2:
			arm_clk_div = 1;
			break;
		default:
			panic("set_mcu_clk_freq fault\n");
			break;
	}
	pr_debug("%s --- before, AHB_ARM_CLK: %08x, rate = %d, div = %d\n",
		__FUNCTION__, __raw_readl(REG_AHB_ARM_CLK), rate, arm_clk_div);

	gr_gen1 =  __raw_readl(GR_GEN1);
	gr_gen1 |= BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

	val = __raw_readl(REG_AHB_ARM_CLK);
	val &= 0xfffffff8;
	val |= arm_clk_div;
	__raw_writel(val, REG_AHB_ARM_CLK);

	gr_gen1 &= ~BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

	pr_debug("%s --- after, AHB_ARM_CLK: %08x, rate = %d, div = %d\n",
		__FUNCTION__, __raw_readl(REG_AHB_ARM_CLK), rate, arm_clk_div);

	return;
}

static unsigned int get_mcu_clk_freq(void)
{
	u32 mpll_refin, mpll_n, mpll_cfg = 0, rate, val;

	mpll_cfg = __raw_readl(GR_MPLL_MN);

	mpll_refin = (mpll_cfg >> GR_MPLL_REFIN_SHIFT) & GR_MPLL_REFIN_MASK;
	switch(mpll_refin){
		case 0:
			mpll_refin = GR_MPLL_REFIN_2M;
			break;
		case 1:
		case 2:
			mpll_refin = GR_MPLL_REFIN_4M;
			break;
		case 3:
			mpll_refin = GR_MPLL_REFIN_13M;
			break;
		default:
			pr_err("%s --- ERROR mpll_refin: %d\n", __FUNCTION__, mpll_refin);
	}
	mpll_n = mpll_cfg & GR_MPLL_N_MASK;
	rate = mpll_refin * mpll_n;

	/*find div */
	val = __raw_readl(REG_AHB_ARM_CLK) & 0x7;
	val += 1;
	return rate / val;
}

#endif


#define FREQ_TABLE_SIZE 	10

struct cpufreq_conf {
	struct clk 					*clk;
	struct clk 					*mpllclk;
	struct clk 					*tdpllclk;
	struct regulator 				*regulator;
	unsigned int					orignal_freq;
	struct cpufreq_frequency_table	freq_tbl[FREQ_TABLE_SIZE];
	unsigned int					vddarm_mv[FREQ_TABLE_SIZE];
};

struct cpufreq_status {
	unsigned int	percpu_target[CONFIG_NR_CPUS];
	int		is_suspend;
};


struct cpufreq_conf sc8825_cpufreq_conf = {
	.clk = NULL,
	.regulator = NULL,
	.freq_tbl =	{
		{0, 1000000},
		{1, 500000},
		{2, CPUFREQ_TABLE_END}
	},
	.vddarm_mv = {
		0
	},
};

/* khz */
#define SHARK_TOP_FREQUENCY	(1000000)
#define SHARK_TDPLL_FREQUENCY	(768000)
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
struct cpufreq_conf sc8830_cpufreq_conf = {
	.clk = NULL,
	.mpllclk = NULL,
	.tdpllclk = NULL,
	.regulator = NULL,
	.freq_tbl = {
		{0, SHARK_TOP_FREQUENCY},
		{1, SHARK_TDPLL_FREQUENCY},
		{2, 50000},	//fake freq means we need to unplug one cpu
		{3, CPUFREQ_TABLE_END}
	},
	.vddarm_mv = {
		1250000,
		1200000,
		1000000,
		1000000,
	},
};
#else
struct cpufreq_conf sc8830_cpufreq_conf = {
	.clk = NULL,
	.mpllclk = NULL,
	.tdpllclk = NULL,
	.regulator = NULL,
	.freq_tbl = {
		{0, SHARK_TOP_FREQUENCY},
		{1, SHARK_TDPLL_FREQUENCY},
		{2, CPUFREQ_TABLE_END}
	},
	.vddarm_mv = {
		1250000,
		1200000,
		1000000,
	},
};
#endif
/* ns */
#define TRANSITION_LATENCY	(500 * 1000)

struct cpufreq_conf *sprd_cpufreq_conf = NULL;
struct cpufreq_status sprd_cpufreq_status = {0};
struct cpufreq_freqs global_freqs;
static DEFINE_MUTEX(freq_lock);

#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
static unsigned int bottom_freq = SHARK_TDPLL_FREQUENCY;
struct unplug_work_info {
	unsigned int cpuid;
	int need_unplug;
	struct delayed_work unplug_work;
};
/* milliseconds */
static int unplug_delay = 1000;
//we enable dynamic cpu hotplug by default
static int enabled_dhp = 1;

static DEFINE_PER_CPU(struct unplug_work_info, uwi);
static void sprd_unplug_one_cpu(struct work_struct *work)
{
	struct unplug_work_info *puwi = container_of(work,
		struct unplug_work_info, unplug_work.work);
	if (puwi->need_unplug) {
		pr_debug("### we gonna unplug cpu%d\n", puwi->cpuid);
		cpu_down(puwi->cpuid);
	} else {
		pr_debug("### ok do nonthing for cpu%d ###\n", puwi->cpuid);
	}
	return;
}

static ssize_t show_enabled(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", enabled_dhp);
}

static ssize_t store_enabled(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	enabled_dhp = val;
	return n;
}

static ssize_t show_unplug_delay_time
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", unplug_delay);
}

static ssize_t store_unplug_delay_time(struct kobject *kobj, struct attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	unplug_delay = val;
	return n;
}

define_one_global_rw(enabled);
define_one_global_rw(unplug_delay_time);

static struct attribute *dynamic_hp_attributes[] = {
	&enabled.attr,
	&unplug_delay_time.attr,
	NULL
};

static struct attribute_group dynamic_hp_attr_group = {
	.attrs = dynamic_hp_attributes,
	.name = "dynamic_hotplug",
};
static int sprd_auto_hotplug_init(void)
{
	int rc;
	unsigned int i;
	struct unplug_work_info *puwi;

	rc = sysfs_create_group(cpufreq_global_kobject, &dynamic_hp_attr_group);
	if (rc)
		goto sysfs_err;

	for_each_possible_cpu(i) {
		puwi = &per_cpu(uwi, i);
		puwi->cpuid = i;
		puwi->need_unplug = 0;
		INIT_DELAYED_WORK(&puwi->unplug_work, sprd_unplug_one_cpu);
	}

	return 0;
sysfs_err:
	return rc;
}
static void sprd_auto_hotplug_exit(void)
{
	sysfs_remove_group(cpufreq_global_kobject, &dynamic_hp_attr_group);
	return;
}
#endif

static void sprd_raw_set_cpufreq(struct cpufreq_freqs freq, int index)
{
#if defined(CONFIG_ARCH_SCX35)
	int ret;

#define CPUFREQ_SET_VOLTAGE() \
	do { \
	    ret = regulator_set_voltage(sprd_cpufreq_conf->regulator, \
			sprd_cpufreq_conf->vddarm_mv[index], \
			sprd_cpufreq_conf->vddarm_mv[index]); \
		if (ret) \
			pr_err("cpufreq: Failed to set vdd to %d mv\n", \
				sprd_cpufreq_conf->vddarm_mv[index]); \
	} while (0)
#define CPUFREQ_SET_CLOCK() \
	do { \
		if (freq.new == SHARK_TDPLL_FREQUENCY) { \
			ret = clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->tdpllclk); \
			if (ret) \
				pr_err("cpufreq: Failed to set cpu parent to tdpll\n"); \
		} else { \
			if (clk_get_parent(sprd_cpufreq_conf->clk) != sprd_cpufreq_conf->tdpllclk) { \
				ret = clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->tdpllclk); \
				if (ret) \
					pr_err("cpufreq: Failed to set cpu parent to tdpll\n"); \
			} \
			ret = clk_set_rate(sprd_cpufreq_conf->mpllclk, (freq.new * 1000)); \
			if (ret) \
				pr_err("cpufreq: Failed to set mpll rate\n"); \
			ret = clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->mpllclk); \
			if (ret) \
				pr_err("cpufreq: Failed to set cpu parent to mpll\n"); \
		} \
	} while (0)
	trace_cpu_frequency(freq.new, freq.cpu);

	if (freq.new > freq.old) {
		CPUFREQ_SET_VOLTAGE();
		CPUFREQ_SET_CLOCK();
	} else {
		CPUFREQ_SET_CLOCK();
		CPUFREQ_SET_VOLTAGE();
	}

#undef CPUFREQ_SET_VOLTAGE
#undef CPUFREQ_SET_CLOCK

#elif defined(CONFIG_ARCH_SC8825)
	set_mcu_clk_freq(freq.new * 1000);
#endif
	return;
}

static unsigned int sprd_raw_get_cpufreq(void)
{
#if defined(CONFIG_ARCH_SCX35)
	return clk_get_rate(sprd_cpufreq_conf->clk) / 1000;
#elif defined(CONFIG_ARCH_SC8825)
	return get_mcu_clk_freq() / 1000;
#endif
}

static void sprd_real_set_cpufreq(unsigned int new_speed, int index)
{

	pr_debug("$$$ sprd_real_set_cpufreq %u khz on cpu%d\n",
		new_speed, smp_processor_id());
	mutex_lock(&freq_lock);

	if (global_freqs.old == new_speed) {
		mutex_unlock(&freq_lock);
		return;
	}
	global_freqs.new = new_speed;

	for_each_online_cpu(global_freqs.cpu)
		cpufreq_notify_transition(&global_freqs, CPUFREQ_PRECHANGE);

	sprd_raw_set_cpufreq(global_freqs, index);

	for_each_online_cpu(global_freqs.cpu)
		cpufreq_notify_transition(&global_freqs, CPUFREQ_POSTCHANGE);

	global_freqs.old = global_freqs.new;

	mutex_unlock(&freq_lock);
	return;
}

static int sprd_update_cpu_speed(int cpu,
	unsigned int target_speed, int index)
{
	int i;
	unsigned int new_speed = 0;
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	unsigned int min_speed = SHARK_TOP_FREQUENCY, cpuid;
#endif

	/*
	 * CONFIG_NR_CPUS cores are always in the same voltage, at the same
	 * frequency. But, cpu load is calculated individual in each cores,
	 * So we remeber the original target frequency and voltage of core0,
	 * and use the higher one
	 */

	for_each_online_cpu(i) {
		new_speed = max(new_speed, sprd_cpufreq_status.percpu_target[i]);
	}
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	for_each_online_cpu(i) {
		min_speed = min(min_speed, sprd_cpufreq_status.percpu_target[i]);
	}

	if ((new_speed == min_speed) && (min_speed == SHARK_TOP_FREQUENCY)) {
		if (num_online_cpus() < nr_cpu_ids) {
			cpuid = cpumask_next_zero(0, cpu_online_mask);
			if (enabled_dhp) {
				pr_debug("# we gonna plug cpu%d\n", cpuid);
				cpu_up(cpuid);
			}
		}
	}

	if (new_speed < bottom_freq)
		new_speed = bottom_freq;
#endif

	sprd_real_set_cpufreq(new_speed, index);
	return 0;
}

static int sprd_cpufreq_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	int i;

	/* in suspend and hibernation process, we need set frequency to the orignal
	 * one to make sure all things go right */
	if (event == PM_SUSPEND_PREPARE || event == PM_HIBERNATION_PREPARE) {
		sprd_cpufreq_status.is_suspend = true;

		for_each_online_cpu(i) {
			sprd_cpufreq_status.percpu_target[i] =
				sprd_cpufreq_conf->orignal_freq;
		}

		for (i = 0; i < FREQ_TABLE_SIZE; i++) {
			if (CPUFREQ_TABLE_END == sprd_cpufreq_conf->freq_tbl[i].frequency)
				break;
			if (sprd_cpufreq_conf->freq_tbl[i].frequency ==
				sprd_cpufreq_conf->orignal_freq)
				break;
		}

		if (FREQ_TABLE_SIZE == i ||
				CPUFREQ_TABLE_END == sprd_cpufreq_conf->freq_tbl[i].frequency) {
			pr_err("cpufreq: Failed to find orignal cpu frequency in table\n");
		} else
			sprd_update_cpu_speed(0, sprd_cpufreq_conf->orignal_freq, i);
	} else if (event == PM_POST_SUSPEND || event == PM_POST_HIBERNATION)
		sprd_cpufreq_status.is_suspend = false;

	return NOTIFY_OK;
}

static struct notifier_block sprd_cpufreq_pm_notifier = {
	.notifier_call = sprd_cpufreq_pm_notify,
};


static int sprd_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu > CONFIG_NR_CPUS) {
		pr_err("%s --- no such cpu id %d\n", __FUNCTION__, policy->cpu);
		return -EINVAL;
	}

	return cpufreq_frequency_table_verify(policy, sprd_cpufreq_conf->freq_tbl);
}

static int sprd_cpufreq_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int ret = -EFAULT;
	int index;
	unsigned int new_speed;
	struct cpufreq_frequency_table *table;
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	struct unplug_work_info *puwi;
#endif
	if (true == sprd_cpufreq_status.is_suspend)
		return 0;

	table = cpufreq_frequency_get_table(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table,
					target_freq, relation, &index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		return -EINVAL;
	}

	pr_debug("CPU_%d target %d relation %d (%d-%d) selected %d\n",
			policy->cpu, target_freq, relation,
			policy->min, policy->max, table[index].frequency);

	new_speed = table[index].frequency;

	sprd_cpufreq_status.percpu_target[policy->cpu] = new_speed;
	pr_debug("## %s cpu:%d %u on cpu%d\n", __func__, policy->cpu, new_speed, smp_processor_id());

#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	puwi = &per_cpu(uwi, policy->cpu);
	if (new_speed < bottom_freq) {
		if (policy->cpu) {
			puwi->need_unplug = 1;
			pr_debug("we set need unplug here cpu%d\n", policy->cpu);
			if (enabled_dhp)
				schedule_delayed_work_on(0, &puwi->unplug_work, msecs_to_jiffies(unplug_delay));
			return 0;
		}
	} else {
		pr_debug("we ununuset need unplug here cpu%d\n", policy->cpu);
		puwi->need_unplug = 0;
	}
#endif

	ret = sprd_update_cpu_speed(policy->cpu, new_speed, index);

	return ret;
}

static unsigned int sprd_cpufreq_getspeed(unsigned int cpu)
{
	if (cpu > CONFIG_NR_CPUS) {
		pr_err("%s --- no such cpu id %d\n", __FUNCTION__, cpu);
		return -EINVAL;
	}

	return sprd_raw_get_cpufreq();
}

static int sprd_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	cpufreq_frequency_table_cpuinfo(policy, sprd_cpufreq_conf->freq_tbl);
	policy->cur = sprd_raw_get_cpufreq(); /* current cpu frequency: KHz*/
	 /*
	  * transition_latency 5us is enough now
	  * but sampling too often, unbalance and irregular on each online cpu
	  * so we set 500us here.
	  */
	policy->cpuinfo.transition_latency = TRANSITION_LATENCY;
	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	cpufreq_frequency_table_get_attr(sprd_cpufreq_conf->freq_tbl, policy->cpu);

	sprd_cpufreq_status.percpu_target[policy->cpu] = policy->cur;

	ret = cpufreq_frequency_table_cpuinfo(policy, sprd_cpufreq_conf->freq_tbl);
	if (ret != 0)
		pr_err("%s --- Failed to config freq table: %d\n", __FUNCTION__, ret);

	if (policy->cpu == 0)
		register_pm_notifier(&sprd_cpufreq_pm_notifier);

	pr_err("sprd_cpufreq_driver_init policy->cpu = %d, policy->cur = %u, cpu = %d, ret = %d\n",
		policy->cpu, policy->cur, smp_processor_id(), ret);

	return ret;
}

static int sprd_cpufreq_exit(struct cpufreq_policy *policy)
{
	if (policy->cpu == 0)
		unregister_pm_notifier(&sprd_cpufreq_pm_notifier);

	return 0;
}

static struct freq_attr *sprd_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sprd_cpufreq_driver = {
	.verify		= sprd_cpufreq_verify_speed,
	.target		= sprd_cpufreq_target,
	.get		= sprd_cpufreq_getspeed,
	.init		= sprd_cpufreq_init,
	.exit		= sprd_cpufreq_exit,
	.name		= "sprd",
	.attr		= sprd_cpufreq_attr,
};



static int sprd_cpufreq_policy_notifier(
	struct notifier_block *nb, unsigned long event, void *data)
{
	return NOTIFY_OK;
}

static struct notifier_block sprd_cpufreq_policy_nb = {
	.notifier_call = sprd_cpufreq_policy_notifier,
};


static int __init sprd_cpufreq_modinit(void)
{
	int ret;

#if defined(CONFIG_ARCH_SCX35)
	sprd_cpufreq_conf = &sc8830_cpufreq_conf;
#elif defined(CONFIG_ARCH_SC8825)
	sprd_cpufreq_conf = &sc8825_cpufreq_conf;
#endif

#if defined(CONFIG_ARCH_SCX35)
	sprd_cpufreq_conf->clk = clk_get_sys(NULL, "clk_mcu");
	if (IS_ERR_OR_NULL(sprd_cpufreq_conf->clk))
		return PTR_ERR(sprd_cpufreq_conf->clk);

	sprd_cpufreq_conf->mpllclk = clk_get_sys(NULL, "clk_mpll");
	if (IS_ERR_OR_NULL(sprd_cpufreq_conf->mpllclk))
		return PTR_ERR(sprd_cpufreq_conf->mpllclk);

	sprd_cpufreq_conf->tdpllclk = clk_get_sys(NULL, "clk_tdpll");
	if (IS_ERR_OR_NULL(sprd_cpufreq_conf->tdpllclk))
		return PTR_ERR(sprd_cpufreq_conf->tdpllclk);

	sprd_cpufreq_conf->regulator = regulator_get(NULL, "vddarm");
	if (IS_ERR_OR_NULL(sprd_cpufreq_conf->regulator))
		return PTR_ERR(sprd_cpufreq_conf->regulator);

	/* set max voltage first */
	regulator_set_voltage(sprd_cpufreq_conf->regulator,
		sprd_cpufreq_conf->vddarm_mv[0],
		sprd_cpufreq_conf->vddarm_mv[0]);
	clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->tdpllclk);
	clk_set_rate(sprd_cpufreq_conf->mpllclk, (SHARK_TOP_FREQUENCY * 1000));
	clk_set_parent(sprd_cpufreq_conf->clk, sprd_cpufreq_conf->mpllclk);

	sprd_cpufreq_conf->orignal_freq = sprd_raw_get_cpufreq();
	global_freqs.old = sprd_cpufreq_conf->orignal_freq;
	sprd_cpufreq_status.is_suspend = false;

#endif
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	sprd_auto_hotplug_init();
#endif
	ret = cpufreq_register_notifier(
		&sprd_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;

	ret = cpufreq_register_driver(&sprd_cpufreq_driver);

	return ret;
}

static void __exit sprd_cpufreq_modexit(void)
{
#if defined(CONFIG_ARCH_SCX35)
	if (!IS_ERR_OR_NULL(sprd_cpufreq_conf->regulator))
		regulator_put(sprd_cpufreq_conf->regulator);
#endif
#if defined(CONFIG_SPRD_CPU_DYNAMIC_HOTPLUG)
	sprd_auto_hotplug_exit();
#endif
	cpufreq_unregister_driver(&sprd_cpufreq_driver);
	cpufreq_unregister_notifier(
		&sprd_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	return;
}

module_init(sprd_cpufreq_modinit);
module_exit(sprd_cpufreq_modexit);

MODULE_AUTHOR("Jianjun.He <jianjun.he@spreadtrum.com>");
MODULE_DESCRIPTION("cpufreq driver for Spreadtrum");
MODULE_LICENSE("GPL");
