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

#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/irqs.h>

#define GREG_GEN0	(SPRD_GREG_BASE + 0x0008)

/* timer0/1 is trigged by RTC, 32KHZ */
#define GPTIMER_FREQ	32768

/* timer2 is trigged by PCLK, 26MHZ */
#define	PCLKTIMER_FREQ	26000000

#define	TIMER_LOAD(id)	(SPRD_TIMER_BASE + 0x20 * (id) + 0x0000)
#define	TIMER_VALUE(id)	(SPRD_TIMER_BASE + 0x20 * (id) + 0x0004)
#define	TIMER_CTL(id)	(SPRD_TIMER_BASE + 0x20 * (id) + 0x0008)
#define	TIMER_INT(id)	(SPRD_TIMER_BASE + 0x20 * (id) + 0x000C)

#define	ONETIME_MODE	(0 << 6)
#define	PERIOD_MODE	(1 << 6)

#define	TIMER_DISABLE	(0 << 7)
#define	TIMER_ENABLE	(1 << 7)

#define	TIMER_INT_EN	(1 << 0)
#define	TIMER_INT_CLR	(1 << 3)
#define	TIMER_INT_BUSY	(1 << 4)

/* timer0 is reserved by modem side,
 * timer1 is used as clockevent,
 * timer2 is used as clocksource
 */
#define	EVENT_TIMER	1
#define	SOURCE_TIMER	2

/* syscnt is trigged by RTC, 1000HZ */
#define	SYSCNT_FREQ	1000

#define	SYSCNT_COUNT	(SPRD_SYSCNT_BASE + 0x0004)
#define	SYSCNT_CTL	(SPRD_SYSCNT_BASE + 0x0008)

static inline void sprd_gptimer_ctl(int timer_id, int enable, int mode)
{
	__raw_writel(enable | mode, TIMER_CTL(timer_id));
}

static int sprd_gptimer_set_next_event(unsigned long cycles, struct clock_event_device *c)
{
	sprd_gptimer_ctl(EVENT_TIMER, TIMER_DISABLE, ONETIME_MODE);
	__raw_writel(cycles, TIMER_LOAD(EVENT_TIMER));
	sprd_gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, ONETIME_MODE);

	return 0;
}

static void sprd_gptimer_set_mode(enum clock_event_mode mode, struct clock_event_device *c)
{
	unsigned int saved;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		sprd_gptimer_ctl(EVENT_TIMER, TIMER_DISABLE, PERIOD_MODE);
		__raw_writel(LATCH, TIMER_LOAD(EVENT_TIMER));
		sprd_gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, PERIOD_MODE);
		__raw_writel(TIMER_INT_EN, TIMER_INT(EVENT_TIMER));
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		sprd_gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, ONETIME_MODE);
		__raw_writel(TIMER_INT_EN, TIMER_INT(EVENT_TIMER));
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		saved = __raw_readl(TIMER_CTL(EVENT_TIMER)) & PERIOD_MODE;
		sprd_gptimer_ctl(EVENT_TIMER, TIMER_DISABLE, saved);
		break;
	case CLOCK_EVT_MODE_RESUME:
		saved = __raw_readl(TIMER_CTL(EVENT_TIMER)) & PERIOD_MODE;
		sprd_gptimer_ctl(EVENT_TIMER, TIMER_ENABLE, saved);
		break;
	}
}

static struct clock_event_device sprd_gptimer_event = {
	.name		= "gptimer1",
	.features	= CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.shift		= 32,
	.rating		= 200,
	.set_next_event	= sprd_gptimer_set_next_event,
	.set_mode	= sprd_gptimer_set_mode,
};

static irqreturn_t sprd_gptimer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	unsigned int value;

	/* clear interrupt */
	value = __raw_readl(TIMER_INT(EVENT_TIMER));
	value |= TIMER_INT_CLR;
	__raw_writel(value, TIMER_INT(EVENT_TIMER));

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction sprd_gptimer_irq = {
	.name		= "gptimer1",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= sprd_gptimer_interrupt,
	.dev_id		= &sprd_gptimer_event,
};

static void sprd_gptimer_clockevent_init(void)
{
	__raw_writel(TIMER_DISABLE, TIMER_CTL(EVENT_TIMER));
	__raw_writel(TIMER_INT_CLR, TIMER_INT(EVENT_TIMER));

	setup_irq(IRQ_TIMER1_INT, &sprd_gptimer_irq);

	sprd_gptimer_event.mult =
		div_sc(GPTIMER_FREQ, NSEC_PER_SEC, sprd_gptimer_event.shift);
	sprd_gptimer_event.max_delta_ns =
		clockevent_delta2ns(ULONG_MAX, &sprd_gptimer_event);
	sprd_gptimer_event.min_delta_ns =
		clockevent_delta2ns(2, &sprd_gptimer_event);
	sprd_gptimer_event.cpumask = cpumask_of(0);

	clockevents_register_device(&sprd_gptimer_event);
}

/* ****************************************************************** */

static cycle_t sprd_gptimer_read(struct clocksource *cs)
{
	spinlock_t lock;
	unsigned int val1, val2;
	unsigned long flags;

	/* read multiple times in case of boundary issue */
	spin_lock_irqsave(&lock, flags);
	val1 = __raw_readl(TIMER_VALUE(SOURCE_TIMER));
	val2 = __raw_readl(TIMER_VALUE(SOURCE_TIMER));
	/* NOTE: register access is slower than 26Mhz, need more cycles */
	while((int)(val1 - val2) & ~15) {
		val1 = val2;
		val2 = __raw_readl(TIMER_VALUE(SOURCE_TIMER));
	}
	spin_unlock_irqrestore(&lock, flags);

	return (ULONG_MAX - val2);
}

static struct clocksource sprd_gptimer_src = {
	.name		= "gptimer2",
	.rating		= 300,
	.read		= sprd_gptimer_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void sprd_gptimer_clocksource_init(void)
{
	/* disalbe irq since it's just a read source */
	__raw_writel(0, TIMER_INT(SOURCE_TIMER));

	/* set timer load as maximal */
	sprd_gptimer_ctl(SOURCE_TIMER, TIMER_DISABLE, PERIOD_MODE);
	__raw_writel(ULONG_MAX, TIMER_LOAD(SOURCE_TIMER));
	sprd_gptimer_ctl(SOURCE_TIMER, TIMER_ENABLE, PERIOD_MODE);

	sprd_gptimer_src.mult =
		clocksource_hz2mult(PCLKTIMER_FREQ, sprd_gptimer_src.shift);
	clocksource_register(&sprd_gptimer_src);
}

/* ****************************************************************** */

static cycle_t sprd_syscnt_read(struct clocksource *cs)
{
	spinlock_t lock;
	unsigned int val1, val2;
	unsigned long flags;

	/* read multiple times in case of boundary issue */
	spin_lock_irqsave(&lock, flags);
	val1 = __raw_readl(SYSCNT_COUNT);
	val2 = __raw_readl(SYSCNT_COUNT);
	while((int)(val2 - val1) >> 1) {
		val1 = val2;
		val2 = __raw_readl(SYSCNT_COUNT);
	}
	spin_unlock_irqrestore(&lock, flags);

	return val2;
}

static struct clocksource sprd_syscnt = {
	.name		= "syscnt",
	.rating		= 200,
	.read		= sprd_syscnt_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 10,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void sprd_syscnt_clocksource_init(void)
{
	/* disable irq for syscnt */
	__raw_writel(0, SYSCNT_CTL);

	sprd_syscnt.mult =
		clocksource_hz2mult(SYSCNT_FREQ, sprd_syscnt.shift);
	clocksource_register(&sprd_syscnt);
}

/* ****************************************************************** */

unsigned long long sched_clock(void)
{
	/* use timer2 as sched clock tick */
	return clocksource_cyc2ns(sprd_gptimer_src.read(&sprd_gptimer_src),
			sprd_gptimer_src.mult, sprd_gptimer_src.shift);
}

void __init sc8810_timer_init(void)
{
	unsigned int value;

	/* enable timer & syscnt in global regs */
	value = __raw_readl(GREG_GEN0);
	value |= (1 << 2) | (1 << 19);
	__raw_writel(value, GREG_GEN0);

	/* setup timer2 and syscnt as clocksource */
	sprd_gptimer_clocksource_init();
	sprd_syscnt_clocksource_init();

	/* setup timer1 as clockevent.  */
	sprd_gptimer_clockevent_init();
}
