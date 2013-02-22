/* linux/arch/arm/mach-sc8830/platsmp.c
 * 
 * Copyright (c) 2010-2012 Spreadtrum Co., Ltd.
 *		http://www.spreadtrum.com
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Cloned from linux/arch/arm/mach-vexpress/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/smp_scu.h>
#include <asm/unified.h>

#include <mach/hardware.h>

extern void sci_secondary_startup(void);

#if (defined CONFIG_ARCH_SC8825)
static int __cpuinit boot_secondary_cpus(int cpu_id, u32 paddr)
{
	if (cpu_id != 1)
		return -1;
	writel(paddr,CPU_JUMP_VADDR);
	writel(1 << (cpu_id * 4),HOLDING_PEN_VADDR);
	return 0;
}

#elif (defined CONFIG_ARCH_SC8830)

static int __cpuinit boot_secondary_cpus(int cpu_id, u32 paddr)
{
	int _cpu_id = readl(HOLDING_PEN_VADDR);
	if (cpu_id < 1 || cpu_id > 3)
		return -1;
	writel(paddr,(CPU_JUMP_VADDR + (cpu_id << 2)));
	writel(_cpu_id | (1 << cpu_id),HOLDING_PEN_VADDR);
	return 0;
}

#endif

/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
volatile int pen_release = -1;

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void __cpuinit write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	/*
	 * if any interrupts are already enabled for the primary
	 * core (e.g. timer irq), then they will not have been enabled
	 * for us: do so
	 */
	gic_secondary_init(0);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */

	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;
	int ret;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 */
	ret = boot_secondary_cpus(cpu, virt_to_phys(sci_secondary_startup));
	if (ret < 0)
		pr_warn("SMP: boot_secondary(%u) error\n", cpu);

	write_pen_release(cpu_logical_map(cpu));

	dsb_sev();
	timeout = jiffies + (1 * HZ);

	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

#ifdef CONFIG_HAVE_ARM_SCU
static unsigned int __get_core_num(void)
{
	void __iomem *scu_base = (void __iomem *)(SPRD_CORE_BASE);
	return (scu_base ? scu_get_core_count(scu_base) : 1);
}
#else
static unsigned int __get_core_num(void)
{
	return 4;
}
#endif

static unsigned int sci_get_core_num(void)
{
	return __get_core_num();
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores;

	ncores = sci_get_core_num();

	/* sanity check */
	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	set_smp_cross_call(gic_raise_softirq);
}

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
#ifdef CONFIG_HAVE_ARM_SCU
	scu_enable((void __iomem *)(SPRD_CORE_BASE));
#endif
}

