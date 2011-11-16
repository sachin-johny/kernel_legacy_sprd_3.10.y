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
#include <mach/regs_ana.h>
#include <mach/regs_global.h>
#include <mach/adi_hal_internal.h>

#define USE_FAKE_CPUFREQ		1

#ifdef USE_FAKE_CPUFREQ
static unsigned long cpu_main_freq = 0;
#define REG_AHB_ARM_CLK					(*((volatile unsigned int *)(AHB_ARM_CLK)))
#define REG_CHIP_ID					(*((volatile unsigned int *)(CHIP_ID)))
#define REG_GR_GEN1					(*((volatile unsigned int *)(GR_GEN1)))
#define REG_GR_MPLL_MN					(*((volatile unsigned int *)(GR_MPLL_MN)))
#define MIN_FREQ					(26000)
#define MAX_FREQ					(800000)
#else
#define MIN_FREQ					(26000)
#define MAX_FREQ					(400000)
#endif

#define FREQ_TABLE_ENTRY				(6)

typedef enum
{
	CORE_VOLTAGE_1800MV = 0,
	CORE_VOLTAGE_1950MV,
	CORE_VOLTAGE_1500MV,
	CORE_VOLTAGE_1650MV,
} armcore_voltage_e;

#ifdef CONFIG_ARCH_SC8800G
struct sc8800g_dvfs {
	unsigned int vddarm;
	unsigned int ldoarm;
};

static struct sc8800g_dvfs sc8800g_dvfs_table[] = {
	[0] = { 1800 , CORE_VOLTAGE_1800MV }, /* 1.80v */
	[1] = { 1950 , CORE_VOLTAGE_1950MV }, /* 1.95v */
	[2] = { 1500 , CORE_VOLTAGE_1500MV }, /* 1.50v */
	[3] = { 1650 , CORE_VOLTAGE_1650MV }, /* 1.65v */
};

static struct cpufreq_frequency_table sc8800g_freq_table[FREQ_TABLE_ENTRY];

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

/* return is : Hz */
static unsigned long get_mcu_clk(void)
{
#ifdef USE_FAKE_CPUFREQ
	return (cpu_main_freq * 1000);	
#else
	int clk_mcu_sel, clk_mcu;
	clk_mcu_sel = (REG_AHB_ARM_CLK >> 23) & 0x3;
	switch (clk_mcu_sel) {
	case 0:
		clk_mcu = 400000000;
            	break;
        case 1:
            	clk_mcu = 153600000;
            	break;
        case 2:
            	clk_mcu = 64000000;
            	break;
        case 3:
            	clk_mcu = 26000000;
            	break;
        default:
            	break;
    	}

	return clk_mcu;
#endif
}

/* return is : Hz */
static unsigned long get_ahb_clk(void)
{
#ifdef USE_FAKE_CPUFREQ
	return (get_mcu_clk() / 4);
#else
	unsigned long ahb_div = REG_AHB_ARM_CLK;
    	
	ahb_div = (ahb_div >> 4) & 7;
	ahb_div = ahb_div + 1;

    	if (REG_AHB_ARM_CLK & BIT_30)
        	ahb_div = ahb_div << 1;
    	if( REG_AHB_ARM_CLK & BIT_31)
        	ahb_div = ahb_div << 1;

    	return (get_mcu_clk() / ahb_div);
#endif

}

/* return is arm clock : Hz */
static unsigned long clk_get_rate(void)
{
#ifdef USE_FAKE_CPUFREQ
	unsigned long arm_clk;
	
	arm_clk = get_mcu_clk();
	return arm_clk;
#else
	unsigned long arm_div, arm_clk;

    	arm_div = 1;
    	if (REG_AHB_ARM_CLK & BIT_30)
        	arm_div = arm_div << 1;
	
	arm_clk = get_mcu_clk();
	arm_clk = arm_clk / arm_div;

	return arm_clk;
#endif
}

/* rate is arm clock : Hz */
static int clk_set_rate(unsigned long rate)
{
#ifdef USE_FAKE_CPUFREQ
	return rate;
#else
	unsigned long clk, sel, i;
	
	clk = REG_AHB_ARM_CLK;
	if (clk & BIT_30)
        	clk |= BIT_30;
	
	switch (rate) {
	case 400000000:
		sel = 0;
            	break;
        case 153600000:
            	sel = 1;
            	break;
        case 64000000:
            	sel = 2;
            	break;
        case 26000000:
            	sel = 3;
            	break;
        default:
		//printk("\nInvalid clock\n");
            	break;
    	}
	
	clk = (clk & ~(BIT_23 | BIT_24)) | (sel << 23);
	REG_AHB_ARM_CLK = clk;
	//printk("clk = 0x%08x   REG_AHB_ARM_CLK = 0x%08x\n", clk, REG_AHB_ARM_CLK);
	for (i = 0; i < 100; i++);
	
	return rate;
#endif
}

/*static void set_armcore_voltage(armcore_voltage_e core_voltage)
{
	unsigned long pwr;	

	pwr = REG_GR_DCDC_CTL & 0xfffffffc;
	pwr |= (BIT_8 | core_voltage);
	REG_GR_DCDC_CTL = pwr;
	REG_GR_DCDC_CTL &= ~BIT_8;
}*/

static int sc8800g_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, sc8800g_freq_table);
}

static unsigned int sc8800g_cpufreq_get_speed(unsigned int cpu)
{
	if (cpu != 0)
		return 0;
	return clk_get_rate() / 1000;
}

static int sc8800g_cpufreq_set_target(struct cpufreq_policy *policy,
				      unsigned int target_freq,
				      unsigned int relation)
{
	unsigned long ret;
	unsigned int i;
	struct cpufreq_freqs freqs;
	struct sc8800g_dvfs *dvfs;
	ret = cpufreq_frequency_table_target(policy, sc8800g_freq_table,
					     target_freq, relation, &i);
	if (ret != 0)
		return ret;
	//printk("target_freq = %d  i = %d\n", target_freq, i);
	freqs.cpu = 0;
	freqs.old = clk_get_rate() / 1000;
	freqs.new = sc8800g_freq_table[i].frequency;
	freqs.flags = 0;
	dvfs = &sc8800g_dvfs_table[sc8800g_freq_table[i].index];
	if (freqs.old == freqs.new)
		return 0;
	printk("cpufreq: Transition %d-%dkHz\n", freqs.old, freqs.new);

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

	printk("cpufreq: Set actual frequency %lukHz\n", clk_get_rate() / 1000);

	return 0;
}

static int __init sc8800g_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;

	if (policy->cpu != 0)
		return -EINVAL;

	if (sc8800g_freq_table == NULL) {
		pr_err("cpufreq: No frequency information for this CPU\n");
		return -ENODEV;
	}

	policy->cur = clk_get_rate() / 1000; /* current cpu frequency : KHz*/
	policy->cpuinfo.transition_latency = 1 * 1000 * 1000;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(sc8800g_freq_table, policy->cpu);
#endif

	ret = cpufreq_frequency_table_cpuinfo(policy, sc8800g_freq_table);
	if (ret != 0) {
		pr_err("cpufreq: Failed to configure frequency table: %d\n", ret);
	}
	
	return ret;
}

static struct freq_attr *sc8800g_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sc8800g_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags          = 0,
	.verify		= sc8800g_cpufreq_verify_speed,
	.target		= sc8800g_cpufreq_set_target,
	.get		= sc8800g_cpufreq_get_speed,
	.init		= sc8800g_cpufreq_driver_init,
	.name		= "sc8800g",
	.attr		= sc8800g_cpufreq_attr,
};

static int __init sc8800g_cpufreq_init(void)
{
	unsigned int cnt, clk, arm_clk;

#ifdef USE_FAKE_CPUFREQ
	/*printk("\nREG_CHIP_ID = 0x%08x\n", REG_CHIP_ID);
	printk("REG_AHB_ARM_CLK = 0x%08x\n", REG_AHB_ARM_CLK);
	printk("REG_GR_MPLL_MN = 0x%08x\n", REG_GR_MPLL_MN);
	printk("REG_GR_GEN1 = 0x%08x\n", REG_GR_GEN1);
	printk("ANA_DCDC_CTL = 0x%08x\n", ANA_REG_GET(ANA_DCDC_CTL));*/

	switch (REG_GR_MPLL_MN & 0xfff) {
	case 0xe1:
		cpu_main_freq = 450 * 1000; /* KHz */
		break;
	case 0x12c:
		cpu_main_freq = 600 * 1000;
		break;
	case 0x113:
		cpu_main_freq = 550 * 1000;
		break;
	case 0xfa:
		cpu_main_freq = 500 * 1000;
		break;
	case 0x104:
		cpu_main_freq = 520 * 1000;
		break;
	case 0x145:
		cpu_main_freq = 650 * 1000;
		break;
	}
#endif

	arm_clk = clk_get_rate() / 1000;
	for (cnt = 0; cnt < FREQ_TABLE_ENTRY; cnt ++) {
		sc8800g_freq_table[cnt].index = 0;
		sc8800g_freq_table[cnt].frequency = CPUFREQ_TABLE_END;
	}

	if (arm_clk < MIN_FREQ) {
		printk("arm clock is too low, cpufreq is not needed.\n");
		return 0;
	}
	if (arm_clk > MAX_FREQ) {
		printk("arm clock is too big beyond frequency table, please add frequency table entry.\n");
		return 0;
	}

	clk = 0;
#ifdef USE_FAKE_CPUFREQ
	sc8800g_freq_table[clk].frequency = cpu_main_freq;
	clk ++;
	sc8800g_freq_table[clk].frequency = cpu_main_freq;
	clk ++;
	sc8800g_freq_table[clk].frequency = cpu_main_freq;
	clk ++;
	sc8800g_freq_table[clk].frequency = cpu_main_freq;
	clk ++;
#else
	sc8800g_freq_table[clk].frequency = 400000;
	clk ++;
	sc8800g_freq_table[clk].frequency = 153600;
	clk ++;
	sc8800g_freq_table[clk].frequency = 64000;
	clk ++;
	sc8800g_freq_table[clk].frequency = 26000;
	clk ++;
#endif

	for (cnt = 0; cnt < clk; cnt ++)
		printk("sc8800g_freq_table[%d].frequency = %d\n", cnt, sc8800g_freq_table[cnt].frequency);

	return cpufreq_register_driver(&sc8800g_cpufreq_driver);
}
module_init(sc8800g_cpufreq_init);
