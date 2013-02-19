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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/clocksource.h>

#include <asm/sched_clock.h>
#include <asm/smp_twd.h>
#include <asm/localtimer.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/regs_glb.h>
#include <mach/sci.h>

static __iomem void *base_gptimer = (__iomem void *)SPRD_GPTIMER_BASE;
#define	TIMER_LOAD(id)	(base_gptimer + 0x20 * (id) + 0x0000)
#define	TIMER_VALUE(id)	(base_gptimer + 0x20 * (id) + 0x0004)
#define	TIMER_CTL(id)	(base_gptimer + 0x20 * (id) + 0x0008)
#define	TIMER_INT(id)	(base_gptimer + 0x20 * (id) + 0x000C)

#define	ONETIME_MODE	(0 << 6)
#define	PERIOD_MODE	(1 << 6)

#define	TIMER_DISABLE	(0 << 7)
#define	TIMER_ENABLE	(1 << 7)

#define	TIMER_INT_EN	(1 << 0)
#define	TIMER_INT_CLR	(1 << 3)
#define	TIMER_INT_BUSY	(1 << 4)

/**
 * timer0 is reserved by modem side,
 * timer1 is used as clockevent,
 * timer2 is used as clocksource
 */
#define	EVENT_TIMER	1
#define	SOURCE_TIMER	2

static __iomem void *base_syscnt = (__iomem void *)SPRD_SYSCNT_BASE;
#define	SYSCNT_COUNT	(base_syscnt + 0x0004)
#define	SYSCNT_CTL	(base_syscnt + 0x0008)
#define	SYSCNT_SHADOW_CNT	(base_syscnt + 0x000C)

static inline void __gptimer_ctl(int timer_id, int enable, int mode)
{
	__raw_writel(enable | mode, TIMER_CTL(timer_id));
}

static int __gptimer_set_next_event(unsigned long cycles,
				    struct clock_event_device *c)
{
	while(TIMER_INT_BUSY & __raw_readl(TIMER_INT(EVENT_TIMER)));
	__gptimer_ctl(EVENT_TIMER, TIMER_DISABLE, ONETIME_MODE);
	__raw_writel(cycles, TIMER_LOAD(EVENT_TIMER));
	__gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, ONETIME_MODE);
	return 0;
}

static void __gptimer_set_mode(enum clock_event_mode mode,
			       struct clock_event_device *c)
{
	unsigned int saved;
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		__gptimer_ctl(EVENT_TIMER, TIMER_DISABLE, PERIOD_MODE);
		__raw_writel(LATCH, TIMER_LOAD(EVENT_TIMER));
		__gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, PERIOD_MODE);
		__raw_writel(TIMER_INT_EN, TIMER_INT(EVENT_TIMER));
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		__gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, ONETIME_MODE);
		__raw_writel(TIMER_INT_EN, TIMER_INT(EVENT_TIMER));
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		saved = __raw_readl(TIMER_CTL(EVENT_TIMER)) & PERIOD_MODE;
		__gptimer_ctl(EVENT_TIMER, TIMER_DISABLE, saved);
		break;
	case CLOCK_EVT_MODE_RESUME:
		saved = __raw_readl(TIMER_CTL(EVENT_TIMER)) & PERIOD_MODE;
		__gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, saved);
		break;
	}
}

static struct clock_event_device gptimer_event = {
	.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.shift = 32,
	.rating = 200,
	.set_next_event = __gptimer_set_next_event,
	.set_mode = __gptimer_set_mode,
};

static irqreturn_t __gptimer_interrupt(int irq, void *dev_id)
{
	unsigned int value;
	struct clock_event_device *evt = dev_id;

	value = __raw_readl(TIMER_INT(EVENT_TIMER));
	value |= TIMER_INT_CLR;
	__raw_writel(value, TIMER_INT(EVENT_TIMER));

	if (evt->event_handler)
		evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction gptimer_irq = {
	.name = "gptimer1",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = __gptimer_interrupt,
	.dev_id = &gptimer_event,
};

static void sprd_gptimer_clockevent_init(unsigned int irq, const char *name,
					 unsigned long hz)
{
	struct clock_event_device *evt = &gptimer_event;
	int ret = 0;

	__raw_writel(TIMER_DISABLE, TIMER_CTL(EVENT_TIMER));
	__raw_writel(TIMER_INT_CLR, TIMER_INT(EVENT_TIMER));

	evt->name = name;
	evt->irq = irq;
	evt->mult = div_sc(hz, NSEC_PER_SEC, evt->shift);
	evt->max_delta_ns = clockevent_delta2ns(ULONG_MAX, evt);
	evt->min_delta_ns = clockevent_delta2ns(2, evt);
	evt->cpumask = cpu_all_mask;

	ret = setup_irq(irq, &gptimer_irq);
	if (ret)
		BUG_ON(1);
	clockevents_register_device(evt);
}

/* ****************************************************************** */
static void __gptimer_clocksource_init(const char *name, unsigned long hz)
{
	/* disalbe irq since it's just a read source */
	__raw_writel(0, TIMER_INT(SOURCE_TIMER));

	__gptimer_ctl(SOURCE_TIMER, TIMER_DISABLE, PERIOD_MODE);
	__raw_writel(ULONG_MAX, TIMER_LOAD(SOURCE_TIMER));
	__gptimer_ctl(SOURCE_TIMER, TIMER_ENABLE, PERIOD_MODE);

	clocksource_mmio_init(TIMER_VALUE(SOURCE_TIMER), name,
			      hz, 300, 32, clocksource_mmio_readl_down);
}

static void __syscnt_clocksource_init(const char *name, unsigned long hz)
{
	/* disable irq for syscnt */
	__raw_writel(0, SYSCNT_CTL);

	clocksource_mmio_init(SYSCNT_SHADOW_CNT, name,
			      hz, 200, 32, clocksource_mmio_readw_up);
}

/* ****************************************************************** */
static u32 notrace __update_sched_clock(void)
{
	return ~(readl_relaxed(TIMER_VALUE(SOURCE_TIMER)));
}

static void __init __sched_clock_init(unsigned long rate)
{
	setup_sched_clock(__update_sched_clock, 32, rate);
}

#ifdef CONFIG_HAVE_ARM_TWD
static DEFINE_TWD_LOCAL_TIMER(twd_local_timer,
			      SC_LOCAL_TIMER_PA,
			      IRQ_LOCALTIMER);

static void __init __twd_init(void)
{
	int err = twd_local_timer_register(&twd_local_timer);
	if (err)
		pr_err("twd_local_timer_register failed %d\n", err);
}
#else
#define __twd_init()	do { } while(0)
#endif

void __init sci_enable_timer_early(void)
{
	/* enable timer & syscnt in global regs */
	sci_glb_set(REG_GLB_GEN0, BIT_TMR_EB | BIT_SYST0_EB);
#if !defined (CONFIG_ARM_ARCH_TIMER)
	__sched_clock_init(26000000);
#endif
}
#ifdef CONFIG_ARM_ARCH_TIMER
static void __init __arch_timer_init(void)
{
	struct arch_timer at;
	at.res[0].start = 29;
	at.res[0].flags = IORESOURCE_IRQ;

	at.res[1].start = 0;
	at.res[1].flags = IORESOURCE_IRQ;

	arch_timer_register(&at);
	if (arch_timer_sched_clock_init() != 0)
		printk("arch_timer_sched_clock_init return error\n");

}
#else
#define __arch_timer_init()   do { } while(0)
#endif
void __init sci_timer_init(void)
{
	/* setup timer2 and syscnt as clocksource */
	__gptimer_clocksource_init("gptimer2", 26000000);
	__syscnt_clocksource_init("syscnt", 1000);

	__twd_init();

	__arch_timer_init();
	/* setup timer1 as clockevent. */
	sprd_gptimer_clockevent_init(IRQ_TIMER1_INT, "gptimer1", 32768);
	printk("sci_timer_init\n");
}
