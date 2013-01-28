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

#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/bitops.h>
#include <mach/hardware.h>
#include <mach/regulator.h>
#include <mach/regs_ana_glb.h>
#include <mach/regs_ahb.h>
#include <mach/regs_glb.h>
#include <mach/adi.h>
#include <mach/sci.h>

#define DELTA 				msecs_to_jiffies(500)
#define FREQ_TABLE_ENTRY		(3)

static int sprd_cpufreq_set_rate(struct cpufreq_policy *policy, int index);
static unsigned int get_mcu_clk_freq(u32 cpu);

/*
 *   Cpu freqency is not be scaled yet, because of reasons of stablily.
 *   But we still define CONFIG_CPU_FREQ for some APKs, they will
 *display BogoMIPS instead of the real cpu frequency if CONFIG_CPU_FREQ
 *is not be defined
 */
static int cpufreq_bypass = 0;


struct sprd_dvfs_table {
	unsigned long  clk_mcu_mhz;
	unsigned long  vdd_mcu_mv;
};

static struct sprd_dvfs_table sc8825g_dvfs_table[] = {
	[0] = { 1000000 , 1100000 }, /* 1000,000KHz,  1100mv */
	[1] = { 500000 , 1100000 },  /* 500,000KHz,  1100mv */
};


static struct cpufreq_frequency_table sc8825g_freq_table[FREQ_TABLE_ENTRY];

enum scalable_cpus {
	CPU0 = 0,
	CPU1 = 1,
};

struct scalable {
	int 				cpu;
	struct clk			*clk;
	struct regulator		*vdd;
	struct cpufreq_frequency_table	*freq_tbl;
	struct sprd_dvfs_table		*dvfs_tbl;
};

struct scalable scalable_sc8825[] = {
	[CPU0] = {
		.cpu		= CPU0,
		.freq_tbl	= sc8825g_freq_table,
		.dvfs_tbl	= sc8825g_dvfs_table,
	},
	[CPU1] = {
		.cpu            = CPU1,
		.freq_tbl       = sc8825g_freq_table,
		.dvfs_tbl       = sc8825g_dvfs_table,
	},

};

struct sprd_dvfs_table current_cfg[] = {
	[CPU0] = {
		.clk_mcu_mhz = 0,
		.vdd_mcu_mv = 0,
	},
	[CPU1] = {
		.clk_mcu_mhz = 0,
		.vdd_mcu_mv = 0,
	},

};

struct clock_state {
	struct sprd_dvfs_table  current_para;
	struct mutex			lock;
}drv_state;

#ifdef CONFIG_SMP
struct cpufreq_work_struct {
	struct work_struct work;
	struct cpufreq_policy *policy;
	struct completion complete;
	unsigned int index;
	int frequency;
	int status;
};

static DEFINE_PER_CPU(struct cpufreq_work_struct, cpufreq_work);
static struct workqueue_struct *sprd_cpufreq_wq;
#endif

struct cpufreq_suspend_t {
	struct mutex suspend_mutex;
	int device_suspended;
};

static DEFINE_PER_CPU(struct cpufreq_suspend_t, cpufreq_suspend);


#ifdef CONFIG_SMP
static void set_cpu_work(struct work_struct *work)
{
	struct cpufreq_work_struct *cpu_work =
		container_of(work, struct cpufreq_work_struct, work);

	//cpu_work->status = set_cpu_freq(cpu_work->policy, cpu_work->frequency);
	cpu_work->status  = sprd_cpufreq_set_rate(cpu_work->policy, cpu_work->index);
	complete(&cpu_work->complete);
}
#endif

/*@return: Hz*/
static unsigned long cpu_clk_get_rate(int cpu){
	struct clk *mcu_clk = NULL;
	unsigned long clk_rate = 0;
	
	mcu_clk = scalable_sc8825[cpu].clk;
	clk_rate = get_mcu_clk_freq(cpu);

	if(clk_rate < 0){
		pr_err("!!!%s cpu%u frequency is %lu\n", __func__, cpu, clk_rate);
	}
	return clk_rate;
}

#define MHz                     (1000000)
#define GR_MPLL_REFIN_2M        (2*MHz)
#define GR_MPLL_REFIN_4M        (4*MHz)
#define GR_MPLL_REFIN_13M       (13*MHz)
#define GR_MPLL_REFIN_SHIFT     16
#define GR_MPLL_REFIN_MASK      (0x3)
#define GR_MPLL_N_MASK          (0x7ff)
#define GR_MPLL_MN		(REG_GLB_M_PLL_CTL0)
#define GR_GEN1			(REG_GLB_GEN1)


#define WAIT_US			200
#if 0
static void sc8825_wait_mpll(void)
{
	udelay(WAIT_US);
}
#endif

static void set_mcu_clk_freq_div(u32 cpu, u32 mcu_freq)
{
	u32 val, rate, arm_clk_div, gr_gen1;
	printk("***** %s, mcu_freq:%d ******\n", __func__, mcu_freq);
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
			panic("fault\n");
			break;
	}
	pr_debug("before, AHB_ARM_CLK:%08x, rate =%d, div = %d\n", __raw_readl(REG_AHB_ARM_CLK), rate, arm_clk_div);

	gr_gen1 =  __raw_readl(GR_GEN1);
	gr_gen1 |= BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);
	
	val = __raw_readl(REG_AHB_ARM_CLK);
	val &= 0xfffffff8;
	val |= arm_clk_div;
	__raw_writel(val, REG_AHB_ARM_CLK);
	
	gr_gen1 &= ~BIT(9);
	__raw_writel(gr_gen1, GR_GEN1);

	pr_debug("before, AHB_ARM_CLK:%08x, rate =%d, div = %d\n", __raw_readl(REG_AHB_ARM_CLK), rate, arm_clk_div);
}

static unsigned int get_mcu_clk_freq(u32 cpu)
{
	u32 mpll_refin, mpll_n, mpll_cfg = 0, rate, val;
	switch(cpu){
	case CPU0:
	case CPU1:
		mpll_cfg = __raw_readl(GR_MPLL_MN);
		break;
	default :
		pr_err("!!! no more cups !!!\n");
	}


	mpll_refin = (mpll_cfg>>GR_MPLL_REFIN_SHIFT) & GR_MPLL_REFIN_MASK;
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
			printk("%s ERROR mpll_refin:%d\n", __func__, mpll_refin);
	}
	mpll_n = mpll_cfg & GR_MPLL_N_MASK;
	rate = mpll_refin * mpll_n;

	/*find div */
	val = __raw_readl(REG_AHB_ARM_CLK) & 0x7;
	val += 1;
	return rate/val;
}

static int set_mcu_freq(int cpu, unsigned long mcu_freq_hz){
	unsigned long freq_mcu_mhz = mcu_freq_hz * 1000;
	set_mcu_clk_freq_div(cpu, freq_mcu_mhz);
	return 0;
}

static int cpu_set_freq_vdd(struct cpufreq_policy *policy, unsigned long mcu_clk, unsigned long mcu_vdd)
{
	unsigned int ret;
	unsigned long cpu = policy->cpu;
	unsigned long cur_clk = current_cfg[cpu].clk_mcu_mhz;

	ret = set_mcu_freq(cpu, mcu_clk);

	current_cfg[cpu].clk_mcu_mhz = mcu_clk;
	current_cfg[cpu].vdd_mcu_mv  = mcu_vdd;
	pr_info("*** %s, cpu:%lu, cur_clk:%lu, current_cfg[cpu].clk_mcu_mhz:%lu, current_cfg[cpu].vdd_mcu_mv:%lu ****\n",
			 __func__, cpu, cur_clk, current_cfg[cpu].clk_mcu_mhz, current_cfg[cpu].vdd_mcu_mv);

	return ret;
}

static int cpu_target_freq[NR_CPUS], cpu_target_vdd[NR_CPUS];
static int is_scaling = 0;

static void get_freq_vdd(struct cpufreq_policy * policy, unsigned int * newfreq, unsigned int * newvdd)
{
	int cpu = 0;
	unsigned int vdd = 0, freq =0;
	pr_debug("cpu[%d] = %d\n", cpu, *newfreq);
	cpu_target_freq[policy->cpu] =* newfreq;
	cpu_target_vdd[policy->cpu] = *newvdd;
	for_each_cpu(cpu, cpu_online_mask){
		if(cpu >= NR_CPUS || cpu < 0)
			continue;
		pr_debug("cpu_target_freq[%d] = %d\n", cpu, cpu_target_freq[cpu]);
		if(freq < cpu_target_freq[cpu] ){
			freq = cpu_target_freq[cpu];
			vdd = cpu_target_vdd[cpu];
		}
	}
	*newfreq = freq;
	*newvdd = vdd;
}

static DEFINE_SPINLOCK(cpufreq_lock);
static int sprd_cpufreq_set_rate(struct cpufreq_policy *policy, int index)
{
	int ret = 0, cpu;
	struct cpufreq_freqs freqs;
	struct sprd_dvfs_table *dvfs_tbl = scalable_sc8825[policy->cpu].dvfs_tbl;
	unsigned int new_freq = dvfs_tbl[index].clk_mcu_mhz;
	unsigned int new_vdd = dvfs_tbl[index].vdd_mcu_mv;
	struct cpufreq_policy * policy_temp = NULL;

	if(cpufreq_bypass)
		return ret;
	/*
	*  2 cores are always in the same voltage, at the same frequency. But,
	* cpu load is calculated individual in each cores, So we remeber the 
	* original target frequency and voltage of core0, and use the higher one
	*/
	if(policy->cpu >= NR_CPUS || policy->cpu < 0){
		printk("wrong cpu id = %d\n", policy->cpu);
		return -1;
	}

	spin_lock(&cpufreq_lock);
	get_freq_vdd(policy, &new_freq, &new_vdd);

	if(new_freq == current_cfg[policy->cpu].clk_mcu_mhz || is_scaling){
		spin_unlock(&cpufreq_lock);
		pr_debug("new_freq = %u  current_cfg[policy->cpu].clk_mcu_mhz=%lu is_scaling=%d\n", 
			new_freq, current_cfg[policy->cpu].clk_mcu_mhz, is_scaling);
		return 0;
	}
	is_scaling = 1;
	spin_unlock(&cpufreq_lock);
	pr_debug("new_freq + %d  cur_freq = %lu\n", new_freq, current_cfg[policy->cpu].clk_mcu_mhz);

	freqs.old = policy->cur;
	freqs.new = new_freq;
	for_each_cpu(cpu, cpu_online_mask){
		freqs.cpu = cpu;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}
	ret = cpu_set_freq_vdd(policy, new_freq, new_vdd);
	if (!ret){
		for_each_cpu(cpu, cpu_online_mask){
			freqs.cpu = cpu;
			cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
			policy_temp = cpufreq_cpu_get(cpu);
			policy_temp->cur = new_freq;
			current_cfg[cpu].clk_mcu_mhz = new_freq;
			cpufreq_cpu_put(policy_temp);
		}
		pr_debug("%s, new_freq:%lu KHz, new_vdd:%lu uv \n", __func__,
				current_cfg[policy->cpu].clk_mcu_mhz, current_cfg[policy->cpu].vdd_mcu_mv);
	}
	spin_lock(&cpufreq_lock);
	is_scaling = 0;
	spin_unlock(&cpufreq_lock);
	return ret;
}

static int sc8825_cpufreq_table_init(int cpu){
	int cnt;

	scalable_sc8825[cpu].cpu = cpu;
	scalable_sc8825[cpu].clk = clk_get(NULL, "clk_mcu");
	scalable_sc8825[cpu].vdd = regulator_get(NULL, "vddarm");
	if( scalable_sc8825[cpu].clk == NULL ||
		scalable_sc8825[cpu].vdd == NULL ){
		printk("%s, cpu:%d, clk:%p, vdd:%p\n", __func__, cpu,
				scalable_sc8825[cpu].clk, scalable_sc8825[cpu].vdd);
		return -1;
	}

	if (sc8825g_freq_table == NULL) {
		printk(" cpufreq: No frequency information for this CPU\n");
		return -1;
	}

	for (cnt = 0; cnt < FREQ_TABLE_ENTRY-1; cnt++) {
		sc8825g_freq_table[cnt].index = cnt;
		sc8825g_freq_table[cnt].frequency = sc8825g_dvfs_table[cnt].clk_mcu_mhz;
	}
	sc8825g_freq_table[cnt].index = cnt;
	sc8825g_freq_table[cnt].frequency = CPUFREQ_TABLE_END;
	scalable_sc8825[cpu].freq_tbl = sc8825g_freq_table;

	for (cnt = 0; cnt < FREQ_TABLE_ENTRY; cnt++) {
		printk("%s, scalable_sc8825[cpu].freq_tbl[%d].index:%d\n", __func__, cnt,
				scalable_sc8825[cpu].freq_tbl[cnt].index);
		printk("%s, scalable_sc8825[cpu].freq_tbl[%d].frequency:%d\n", __func__, cnt,
				scalable_sc8825[cpu].freq_tbl[cnt].frequency);
	}
	return 0;
}

static int sprd_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu <  0 ||policy->cpu >  1 )
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, scalable_sc8825[policy->cpu].freq_tbl);
}

/*@return: KHz*/
static unsigned int sprd_cpufreq_get_speed(unsigned int cpu)
{
	return cpu_clk_get_rate(cpu) / 1000;
}

static int sprd_cpufreq_set_target(struct cpufreq_policy *policy,
				unsigned int target_freq,
				unsigned int relation)
{
	int ret = -EFAULT;
	int index;
	struct cpufreq_frequency_table *table;
#ifdef CONFIG_SMP
	struct cpufreq_work_struct *cpu_work = NULL;
	cpumask_var_t mask;

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	if (!cpu_active(policy->cpu)) {
		pr_info("cpufreq: cpu %d is not active.\n", policy->cpu);
		return -ENODEV;
	}
#endif
	mutex_lock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);

	if (per_cpu(cpufreq_suspend, policy->cpu).device_suspended) {
		printk("cpufreq: cpu%d scheduling frequency change "
				"in suspend.\n", policy->cpu);
		ret = -EFAULT;
		goto done;
	}

	table = cpufreq_frequency_get_table(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
				&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto done;
	}

//#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_debug("CPU[%d] target %d relation %d (%d-%d) selected %d\n",
			policy->cpu, target_freq, relation,
			policy->min, policy->max, table[index].frequency);
//#endif

#ifdef CONFIG_SMP
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	cpu_work->policy = policy;
	cpu_work->frequency = table[index].frequency;
	cpu_work->status = -ENODEV;
	cpu_work->index = index;

	cpumask_clear(mask);
	cpumask_set_cpu(policy->cpu, mask);
	if (cpumask_equal(mask, &current->cpus_allowed)) {
/*		pr_debug("******** %s, mask:0x%x **********\n", __func__, mask); */
		pr_debug(" cpumask_equal(mask, &current->cpus_allowed)\n ");
		ret = sprd_cpufreq_set_rate(cpu_work->policy, cpu_work->index);
		goto done;
	} else {
/*		pr_debug("******** %s, mask:0x%x **********\n", __func__, mask); */
		pr_debug(" !!!!!cpumask_equal(mask, &current->cpus_allowed)\n ");
		cancel_work_sync(&cpu_work->work);
		INIT_COMPLETION(cpu_work->complete);
		queue_work_on(policy->cpu, sprd_cpufreq_wq, &cpu_work->work);
		wait_for_completion(&cpu_work->complete);
	}

	free_cpumask_var(mask);
	ret = cpu_work->status;
	pr_debug("******** %s, cpu_work->status:%d **********\n", __func__, ret);
#else
	ret = sprd_cpufreq_set_rate(policy, index);
#endif

done:
	mutex_unlock(&per_cpu(cpufreq_suspend, policy->cpu).suspend_mutex);
	return ret;

}

void update_policy_user(struct cpufreq_policy * policy);
static int sprd_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;
#ifdef CONFIG_SMP
	struct cpufreq_work_struct *cpu_work = NULL;
#endif
	printk("sprd_cpufreq_driver_init cpu = %d\n", smp_processor_id());
	ret = sc8825_cpufreq_table_init(policy->cpu);
	if(ret)
		return -ENODEV;

	policy->cur = cpu_clk_get_rate(policy->cpu) / 1000; /* current cpu frequency : KHz*/
	policy->cpuinfo.transition_latency =1* 1000* 1000;//why this value??
	current_cfg[policy->cpu].clk_mcu_mhz = policy->cur;	
	cpu_target_freq[policy->cpu] =  policy->cur;

#ifdef CONFIG_CPU_FREQ_STAT_DETAILS
	cpufreq_frequency_table_get_attr(scalable_sc8825[policy->cpu].freq_tbl, policy->cpu);
#endif

	ret = cpufreq_frequency_table_cpuinfo(policy, scalable_sc8825[policy->cpu].freq_tbl);
	update_policy_user( policy);
	if (ret != 0) {
		pr_err("cpufreq: Failed to configure frequency table: %d\n", ret);
	}

#ifdef CONFIG_SMP
	cpu_work = &per_cpu(cpufreq_work, policy->cpu);
	INIT_WORK(&cpu_work->work, set_cpu_work);
	init_completion(&cpu_work->complete);
#endif
	printk("sprd_cpufreq_driver_init cpu = %d ret = %d end\n", smp_processor_id(), ret);
	return ret;
}

static struct freq_attr *sprd_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver sprd_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags      	= CPUFREQ_STICKY,
	.verify		= sprd_cpufreq_verify_speed,
	.target		= sprd_cpufreq_set_target,
	.get		= sprd_cpufreq_get_speed,
	.init		= sprd_cpufreq_driver_init,
	.name		= "sprd_cpufreq",
	.attr		= sprd_cpufreq_attr,
};

static int sprd_cpufreq_suspend(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		mutex_lock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
		per_cpu(cpufreq_suspend, cpu).device_suspended = 1;
		mutex_unlock(&per_cpu(cpufreq_suspend, cpu).suspend_mutex);
	}

	return NOTIFY_DONE;
}

static int sprd_cpufreq_resume(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

	return NOTIFY_DONE;
}

static int sprd_cpufreq_pm_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	switch (event) {
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		return sprd_cpufreq_resume();
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		return sprd_cpufreq_suspend();
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block sprd_cpufreq_pm_notifier = {
	.notifier_call = sprd_cpufreq_pm_event,
};

static int __init sprd_cpufreq_register(void){
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		mutex_init(&(per_cpu(cpufreq_suspend, cpu).suspend_mutex));
		per_cpu(cpufreq_suspend, cpu).device_suspended = 0;
	}

#ifdef CONFIG_SMP
	sprd_cpufreq_wq = create_workqueue("sprd-cpufreq");
#endif

	register_pm_notifier(&sprd_cpufreq_pm_notifier);
	ret = cpufreq_register_driver(&sprd_cpufreq_driver);
	return ret;
}

late_initcall(sprd_cpufreq_register);

