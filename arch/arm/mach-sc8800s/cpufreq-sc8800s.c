/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
//#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <mach/regs_ahb.h>
#include <mach/regs_global.h>

#define REG_AHB_AHB_ARM_CLK				(*((volatile unsigned int *)(AHB_AHB_ARM_CLK)))
#define REG_GR_MPLL_MN					(*((volatile unsigned int *)(GR_MPLL_MN)))
#define REG_GR_GEN1					(*((volatile unsigned int *)(GR_GEN1)))
#define REG_GR_LDO_CTL0					(*((volatile unsigned int *)(GR_LDO_CTL0)))
#define REG_GR_LDO_CTL2					(*((volatile unsigned int *)(GR_LDO_CTL2)))
#define REG_GR_LDO_CTL3					(*((volatile unsigned int *)(GR_LDO_CTL3)))
#define REG_CHIP_TYPE					(*((volatile unsigned int *)(CHIP_TYPE)))
#define REG_GR_DCDC_CTL					(*((volatile unsigned int *)(GR_DCDC_CTL)))

#define MIN_FREQ					(50000)
#define MAX_FREQ					(600000)
#define FREQ_STEP					(50000)
#define FREQ_TABLE_ENTRY				(((MAX_FREQ - MIN_FREQ) / FREQ_STEP) + 1 + 1) 

typedef enum
{
	CORE_VOLTAGE_1800MV = 0,
	CORE_VOLTAGE_1950MV,
	CORE_VOLTAGE_1500MV,
	CORE_VOLTAGE_1650MV,
} armcore_voltage_e;

#ifdef CONFIG_ARCH_SC8800S
struct sc8800s_dvfs {
	unsigned int vddarm;
	unsigned int ldoarm;
};

static struct sc8800s_dvfs sc8800s_dvfs_table[] = {
	[0] = { 1800 , CORE_VOLTAGE_1800MV }, /* 1.80v */
	[1] = { 1950 , CORE_VOLTAGE_1950MV }, /* 1.95v */
	[2] = { 1500 , CORE_VOLTAGE_1500MV }, /* 1.50v */
	[3] = { 1650 , CORE_VOLTAGE_1650MV }, /* 1.65v */
};

static struct cpufreq_frequency_table sc8800s_freq_table[FREQ_TABLE_ENTRY];

#endif

/*
 * Standard clock functions defined in include/linux/clk.h
 */
#if 0
struct clk *clk_get(struct device *dev, const char *id)
{
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	return -ENOSYS;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	return ERR_PTR(-ENOSYS);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
}
EXPORT_SYMBOL(clk_set_flags);
#endif

static unsigned long get_mpll_clk(void)
{
	unsigned long ext_26m_clk, reg_val, M, N, mpll;
	
	ext_26m_clk = (REG_GR_GEN1 >> 15) & 0x1;
	reg_val = REG_GR_MPLL_MN;
	M = reg_val & 0x0fff;
	N = (reg_val & 0x0fff0000) >> 16;
	
	if (ext_26m_clk == 1)
		mpll = 26 * N / M;
	else
		mpll = 13 * N / M;
	
	return mpll * 1000 * 1000;
}

/* return is arm clock */
static unsigned long clk_get_rate(void)
{
	unsigned long arm_div, arm_clk;
	
	arm_div = REG_AHB_AHB_ARM_CLK & 0x1f;
	arm_clk = get_mpll_clk() / (arm_div + 1);
	return arm_clk;
}

/* rate is arm clock */
static int clk_set_rate(unsigned long rate)
{
	unsigned long i, arm_div, ahb_div, emc_div, xahb_div, mpll_set = 0;

	arm_div = 2;
	ahb_div = 4;
	emc_div = 2;
	mpll_set = ((rate / 1000000) << 16) | 0xd;

	REG_GR_GEN1 |= BIT_9;
	REG_GR_MPLL_MN = mpll_set;
	for (i = 0; i < 100; i++);
	REG_GR_GEN1 &= ~BIT_9;
	xahb_div = ahb_div / arm_div;
	REG_AHB_AHB_ARM_CLK = (arm_div-1) | ((ahb_div-1) << 5) | ((emc_div-1) << 10) | ((xahb_div-1) << 15);
	for (i = 0; i < 100; i++);
	
	return rate;
}

static void set_armcore_voltage(armcore_voltage_e core_voltage)
{
	unsigned long pwr;	

	pwr = REG_GR_DCDC_CTL & 0xfffffffc;
	pwr |= (BIT_8 | core_voltage);
	REG_GR_DCDC_CTL = pwr;
	REG_GR_DCDC_CTL &= ~BIT_8;
}

static int sc8800s_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, sc8800s_freq_table);
}

static unsigned int sc8800s_cpufreq_get_speed(unsigned int cpu)
{
	if (cpu != 0)
		return 0;
	return clk_get_rate() / 1000;
}

static int sc8800s_cpufreq_set_target(struct cpufreq_policy *policy,
				      unsigned int target_freq,
				      unsigned int relation)
{
	unsigned long ret;
	unsigned int i;
	struct cpufreq_freqs freqs;
	struct sc8800s_dvfs *dvfs;
	ret = cpufreq_frequency_table_target(policy, sc8800s_freq_table,
					     target_freq, relation, &i);
	if (ret != 0)
		return ret;
	//printk("target_freq = %d  i = %d\n", target_freq, i);
	freqs.cpu = 0;
	freqs.old = clk_get_rate() / 1000;
	freqs.new = sc8800s_freq_table[i].frequency;
	freqs.flags = 0;
	dvfs = &sc8800s_dvfs_table[sc8800s_freq_table[i].index];
	if (freqs.old == freqs.new)
		return 0;
	//printk("cpufreq: Transition %d-%dkHz\n", freqs.old, freqs.new);

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	/*if (freqs.new > freqs.old) {
		ret = sc8800s_freq_table[i].index;
		//printk("idx = %d  vddarm = %d  ldoarm = %d\n", ret, sc8800s_dvfs_table[ret].vddarm, sc8800s_dvfs_table[ret].ldoarm);
		set_armcore_voltage(sc8800s_dvfs_table[ret].ldoarm);
	}*/

	/* perhaps, we need change lcd and emc pll etc. */
	ret = clk_set_rate(freqs.new * 1000);

	/*if (freqs.new < freqs.old) {
		ret = sc8800s_freq_table[i].index;
		//printk("idx = %d  vddarm = %d  ldoarm = %d\n", ret, sc8800s_dvfs_table[ret].vddarm, sc8800s_dvfs_table[ret].ldoarm);
		set_armcore_voltage(sc8800s_dvfs_table[ret].ldoarm);
	}*/

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	//printk("cpufreq: Set actual frequency %lukHz\n", clk_get_rate() / 1000);

	return 0;
}

static int __init sc8800s_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;

	if (policy->cpu != 0)
		return -EINVAL;

	if (sc8800s_freq_table == NULL) {
		pr_err("cpufreq: No frequency information for this CPU\n");
		return -ENODEV;
	}

	policy->cur = clk_get_rate() / 1000;
	policy->cpuinfo.transition_latency = 1 * 1000 * 1000;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(sc8800s_freq_table, policy->cpu);
#endif

	ret = cpufreq_frequency_table_cpuinfo(policy, sc8800s_freq_table);
	if (ret != 0) {
		pr_err("cpufreq: Failed to configure frequency table: %d\n", ret);
	}
	
	return ret;
}

static struct freq_attr *sc8800s_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sc8800s_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags          = 0,
	.verify		= sc8800s_cpufreq_verify_speed,
	.target		= sc8800s_cpufreq_set_target,
	.get		= sc8800s_cpufreq_get_speed,
	.init		= sc8800s_cpufreq_driver_init,
	.name		= "sc8800s",
	.attr		= sc8800s_cpufreq_attr,
};

static int __init sc8800s_cpufreq_init(void)
{
#if 0
	unsigned long chipid = 1;

	chipid = REG_CHIP_TYPE;
	printk("REG_AHB_AHB_ARM_CLK = 0x%08x\n", REG_AHB_AHB_ARM_CLK);
	printk("REG_GR_MPLL_MN = 0x%08x\n", REG_GR_MPLL_MN);
	printk("REG_GR_GEN1 = 0x%08x\n", REG_GR_GEN1);
	printk("REG_GR_LDO_CTL0	 = 0x%08x\n", REG_GR_LDO_CTL0);
	printk("REG_GR_LDO_CTL2	 = 0x%08x\n", REG_GR_LDO_CTL2);
	printk("REG_GR_LDO_CTL3	 = 0x%08x\n", REG_GR_LDO_CTL3);
	printk("chipid = 0x%08x\n", chipid);
	printk("REG_GR_DCDC_CTL = 0x%08x\n", REG_GR_DCDC_CTL);	

	if (chipid <= 0x8850A003) {
	} else if (0x8850A007 == chipid) {
	} else {
	}

	printk("\nset arm core voltage : 1800mv\n");
	set_armcore_voltage(CORE_VOLTAGE_1800MV);
	printk("REG_GR_DCDC_CTL = 0x%08x\n", REG_GR_DCDC_CTL);
	mdelay(8000);

	printk("\nset arm core voltage : 1950mv\n");
	set_armcore_voltage(CORE_VOLTAGE_1950MV);
	printk("REG_GR_DCDC_CTL = 0x%08x\n", REG_GR_DCDC_CTL);
	mdelay(8000);

	printk("\nset arm core voltage : 1500mv\n");
	set_armcore_voltage(CORE_VOLTAGE_1500MV);
	printk("REG_GR_DCDC_CTL = 0x%08x\n", REG_GR_DCDC_CTL);
	mdelay(8000);

	printk("\nset arm core voltage : 1650mv\n");
	set_armcore_voltage(CORE_VOLTAGE_1650MV);
	printk("REG_GR_DCDC_CTL = 0x%08x\n", REG_GR_DCDC_CTL);
	mdelay(8000);
	return 0;
#endif
	unsigned int cnt, clk, arm_clk;

	arm_clk = clk_get_rate() / 1000;
	for (cnt = 0; cnt < FREQ_TABLE_ENTRY; cnt ++) {
		sc8800s_freq_table[cnt].index = 0;
		sc8800s_freq_table[cnt].frequency = CPUFREQ_TABLE_END;
	}

	if (arm_clk <= MIN_FREQ) {
		printk("arm clock is too low, cpufreq is not needed.\n");
		return 0;
	}
	if (arm_clk > MAX_FREQ) {
		printk("arm clock is too big beyond frequency table, please add frequency table entry.\n");
		return 0;
	}

	cnt = 0;
	for (clk = MIN_FREQ; clk <= MAX_FREQ; clk += FREQ_STEP) {
		sc8800s_freq_table[cnt].frequency = clk;
		if ((clk + FREQ_STEP) >= arm_clk) {
			sc8800s_freq_table[cnt + 1].frequency = arm_clk;
			break;
		}
		cnt ++;
	}
	clk = cnt + 1;
	for (cnt = 0; cnt <= clk; cnt ++)
		printk("sc8800s_freq_table[%d].frequency = %d\n", cnt, sc8800s_freq_table[cnt].frequency);

	return cpufreq_register_driver(&sc8800s_cpufreq_driver);
}
module_init(sc8800s_cpufreq_init);
