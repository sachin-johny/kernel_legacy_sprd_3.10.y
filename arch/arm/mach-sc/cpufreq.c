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
	struct regulator 				*regulator;
	unsigned int					orignal_freq;
	struct cpufreq_frequency_table	freq_tbl[FREQ_TABLE_SIZE];
	unsigned int					vddarm_mv[FREQ_TABLE_SIZE];
};

struct cpufreq_status {
	unsigned int	real_global;
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

struct cpufreq_conf sc8830_cpufreq_conf = {
	.clk = NULL,
	.regulator = NULL,
	.freq_tbl = {
		{0, 800000},
		{1, 400000},
		{2, 266000},
		{3, CPUFREQ_TABLE_END}
	},
	.vddarm_mv = {
		1000000,
		1000000,
		1000000,
		1000000,
	},
};

struct cpufreq_conf *sprd_cpufreq_conf = NULL;
struct cpufreq_status sprd_cpufreq_status = {0};

void sprd_auto_hotplug_init(void)
{
	return;
}
void sprd_auto_hotplug_exit(void)
{
	return;
}
void tegra_auto_hotplug_governor(void)
{
	return;
}

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
		ret = clk_set_rate(sprd_cpufreq_conf->clk, freq.new * 1000); \
		if (ret) \
			pr_err("cpufreq: Failed to set cpu frequency to %d kHz\n", \
				freq.new); \
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

static int sprd_update_cpu_speed(int cpu,
	unsigned int target_speed, int index)
{
	int i;
	unsigned int new_speed = 0;
	struct cpufreq_freqs freqs;

	/*
	 * CONFIG_NR_CPUS cores are always in the same voltage, at the same
	 * frequency. But, cpu load is calculated individual in each cores,
	 * So we remeber the original target frequency and voltage of core0,
	 * and use the higher one
	 */

	for_each_online_cpu(i) {
		new_speed = max(new_speed, sprd_cpufreq_status.percpu_target[i]);
	}

	if (sprd_cpufreq_status.real_global == new_speed)
		return 0;

	freqs.old = sprd_cpufreq_status.real_global;
	freqs.new = new_speed;

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	sprd_raw_set_cpufreq(freqs, index);

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	sprd_cpufreq_status.real_global = new_speed;

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
	if (policy->cpu != 0)
		return 0;

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
	int i, ret;

#if defined(CONFIG_ARCH_SCX35)
	if (!sprd_cpufreq_conf->clk) {
		sprd_cpufreq_conf->clk = clk_get_sys(NULL, "clk_mcu");
		if (IS_ERR_OR_NULL(sprd_cpufreq_conf->clk))
			return PTR_ERR(sprd_cpufreq_conf->clk);
	}
	if (!sprd_cpufreq_conf->regulator) {
		sprd_cpufreq_conf->regulator = regulator_get(NULL, "vddarm");
			if (IS_ERR_OR_NULL(sprd_cpufreq_conf->regulator))
				return PTR_ERR(sprd_cpufreq_conf->regulator);
	}
#endif

	sprd_cpufreq_conf->orignal_freq = sprd_raw_get_cpufreq();

	cpufreq_frequency_table_cpuinfo(policy, sprd_cpufreq_conf->freq_tbl);
	policy->cur = sprd_raw_get_cpufreq(); /* current cpu frequency: KHz*/
	 /*
	  * transition_latency 5us is enough now
	  * but sampling too often, unbalance and irregular on each online cpu
	  * so we set 500us here.
	  */
	policy->cpuinfo.transition_latency = 500 * 1000;
	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	cpufreq_frequency_table_get_attr(sprd_cpufreq_conf->freq_tbl, policy->cpu);

	sprd_cpufreq_status.real_global = policy->cur;
	for_each_online_cpu(i) {
		sprd_cpufreq_status.percpu_target[i] = policy->cur;
	}
	sprd_cpufreq_status.is_suspend = false;

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
	memset(&sprd_cpufreq_status, 0, sizeof(sprd_cpufreq_status));

#if defined(CONFIG_ARCH_SCX35)
	if (!IS_ERR_OR_NULL(sprd_cpufreq_conf->clk))
		clk_put(sprd_cpufreq_conf->clk);

	if (!IS_ERR_OR_NULL(sprd_cpufreq_conf->regulator))
		regulator_put(sprd_cpufreq_conf->regulator);
#endif

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

	sprd_auto_hotplug_init();
	ret = cpufreq_register_notifier(
		&sprd_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;

	ret = cpufreq_register_driver(&sprd_cpufreq_driver);

	return ret;
}

static void __exit sprd_cpufreq_modexit(void)
{
	sprd_auto_hotplug_exit();
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
