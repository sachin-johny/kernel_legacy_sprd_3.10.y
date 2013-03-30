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

#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/regulator.h>
#include <mach/regs_ana_glb.h>
#include <mach/regs_ahb.h>
#include <mach/regs_glb.h>
#include <mach/adi.h>
#include <mach/sci.h>


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



#define FREQ_TABLE_SIZE 	3

struct cpufreq_conf {
#if 0
	struct clk						*clk[CONFIG_NR_CPUS];
	struct regulator				*vdd[CONFIG_NR_CPUS];
#endif
	unsigned int					orignal_freq;
	struct cpufreq_frequency_table	freq_tbl[FREQ_TABLE_SIZE];
#if 0
	unsigned long					vdd_mcu_mv[FREQ_TABLE_SIZE];
#endif
};

struct cpufreq_status {
	unsigned int	real_global;
	unsigned int	percpu_target[CONFIG_NR_CPUS];
	int				is_suspend;
};


struct cpufreq_conf sc8825_cpufreq_conf = {
	.freq_tbl =		{
		{0, 1000000}, 	{1, 500000}, 	{2, CPUFREQ_TABLE_END}
	},
#if 0
	.vdd_mcu_mv =	{
		1100000,		1100000,		0
	},
#endif
};

struct cpufreq_status sc8825_cpufreq_status = {0};


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


static int sprd_update_cpu_speed(int cpu,
	unsigned int target_speed)
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
	sc8825_cpufreq_status.percpu_target[cpu] = target_speed;
	for_each_online_cpu(i) {
		new_speed = max(new_speed, sc8825_cpufreq_status.percpu_target[i]);
	}

	if (sc8825_cpufreq_status.real_global == new_speed)
		return 0;

	freqs.old = sc8825_cpufreq_status.real_global;
	freqs.new = new_speed;

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	set_mcu_clk_freq(new_speed * 1000);

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	sc8825_cpufreq_status.real_global = new_speed;

	return 0;
}


static int sc8825_cpufreq_table_init(int cpu)
{
	int i;

	if (cpu > CONFIG_NR_CPUS) {
		pr_err("%s --- no such cpu id %d\n", __FUNCTION__, cpu);
		return -EINVAL;
	}
#if 0
	sc8825_cpufreq_conf.clk[cpu] = clk_get(NULL, "clk_mcu");
	if (IS_ERR_OR_NULL(sc8825_cpufreq_conf.clk[cpu])) {
		pr_err("%s --- cpu_%d get clk error: 0x%p\n", __FUNCTION__, cpu,
			sc8825_cpufreq_conf.clk[cpu]);
		return -ENXIO;
	}

	sc8825_cpufreq_conf.vdd[cpu] = regulator_get(NULL, "vddarm");
	if (IS_ERR_OR_NULL(sc8825_cpufreq_conf.vdd[cpu])) {
		pr_err("%s --- cpu_%d get vdd error: 0x%p\n", __FUNCTION__, cpu,
			sc8825_cpufreq_conf.vdd[cpu]);
		return -ENXIO;
	}
#endif
	for (i = 0; i < ARRAY_SIZE(sc8825_cpufreq_conf.freq_tbl); i++) {
		pr_debug("%s --- sc8825_cpufreq_conf cpu_%d index:%d, freq:%d\n",
			__FUNCTION__, cpu, i, sc8825_cpufreq_conf.freq_tbl[i].frequency);
	}

	return 0;
}

static int sprd_cpufreq_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	int i;

	/* in suspend and hibernation process, we need set frequency to the orignal
	 * one to make sure all things go right */
	if (event == PM_SUSPEND_PREPARE || event == PM_HIBERNATION_PREPARE) {
		sc8825_cpufreq_status.is_suspend = true;

		for_each_online_cpu(i) {
			sc8825_cpufreq_status.percpu_target[i] =
				sc8825_cpufreq_conf.orignal_freq;
		}

		sprd_update_cpu_speed(0, sc8825_cpufreq_conf.orignal_freq);
	} else if (event == PM_POST_SUSPEND || event == PM_POST_HIBERNATION)
		sc8825_cpufreq_status.is_suspend = false;

	return NOTIFY_OK;
}

static struct notifier_block sprd_cpufreq_pm_notifier = {
	.notifier_call = sprd_cpufreq_pm_notify,
};


int sprd_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu > CONFIG_NR_CPUS) {
		pr_err("%s --- no such cpu id %d\n", __FUNCTION__, policy->cpu);
		return -EINVAL;
	}

	return cpufreq_frequency_table_verify(policy, sc8825_cpufreq_conf.freq_tbl);
}

static int sprd_cpufreq_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int ret = -EFAULT;
	int index;
	unsigned int new_speed;
	struct cpufreq_frequency_table *table;

	if (true == sc8825_cpufreq_status.is_suspend)
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
	ret = sprd_update_cpu_speed(policy->cpu, new_speed);

	tegra_auto_hotplug_governor();

	return ret;
}

unsigned int sprd_cpufreq_getspeed(unsigned int cpu)
{
	if (cpu > CONFIG_NR_CPUS) {
		pr_err("%s --- no such cpu id %d\n", __FUNCTION__, cpu);
		return -EINVAL;
	}

	return get_mcu_clk_freq() / 1000;
}

static int sprd_cpufreq_init(struct cpufreq_policy *policy)
{
	int i, ret;

	ret = sc8825_cpufreq_table_init(policy->cpu);
	if(ret)
		return -ENODEV;

	sc8825_cpufreq_conf.orignal_freq = get_mcu_clk_freq() / 1000;

	cpufreq_frequency_table_cpuinfo(policy, sc8825_cpufreq_conf.freq_tbl);
	policy->cur = get_mcu_clk_freq() / 1000; /* current cpu frequency: KHz*/
	policy->cpuinfo.transition_latency = 1 * 1000 * 1000; /* why this value? */
	policy->shared_type = CPUFREQ_SHARED_TYPE_ALL;
	cpumask_copy(policy->related_cpus, cpu_possible_mask);

	cpufreq_frequency_table_get_attr(sc8825_cpufreq_conf.freq_tbl, policy->cpu);

	sc8825_cpufreq_status.real_global = policy->cur;
	for_each_online_cpu(i) {
		sc8825_cpufreq_status.percpu_target[i] = policy->cur;
	}
	sc8825_cpufreq_status.is_suspend = false;

	ret = cpufreq_frequency_table_cpuinfo(policy, sc8825_cpufreq_conf.freq_tbl);
	if (ret != 0)
		pr_err("%s --- Failed to config freq table: %d\n", __FUNCTION__, ret);

	if (policy->cpu == 0)
		register_pm_notifier(&sprd_cpufreq_pm_notifier);

	pr_err("sprd_cpufreq_driver_init policy->cpu = %d, cpu = %d, ret = %d\n",
		policy->cpu, smp_processor_id(), ret);

	return ret;
}

static int sprd_cpufreq_exit(struct cpufreq_policy *policy)
{
	memset(&sc8825_cpufreq_status, 0, sizeof(sc8825_cpufreq_status));

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
