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
#include <linux/io.h>
#include <linux/sched.h>

#include <asm/smp_twd.h>
#include <asm/localtimer.h>
#include <asm/mach/time.h>
#include <asm/sched_clock.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/globalregs.h>

/* timer0/1 is trigged by RTC, 32KHZ */
#define GPTIMER_FREQ	(32768)

/* timer2 is trigged by PCLK, 26MHZ */
#define	PCLKTIMER_FREQ	(26000000)

#define GEN0_TIMER_EN		BIT(2)
#define GEN0_SYST_EN		BIT(19)
#define GR_GEN0             0x0008

#define	TIMER_LOAD(id)	(SPRD_RTC_BASE + 0x20 * (id) + 0x0000)
#define	TIMER_VALUE(id)	(SPRD_RTC_BASE + 0x20 * (id) + 0x0004)
#define	TIMER_CTL(id)	(SPRD_RTC_BASE + 0x20 * (id) + 0x0008)
#define	TIMER_INT(id)	(SPRD_RTC_BASE + 0x20 * (id) + 0x000C)

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

static DEFINE_CLOCK_DATA(cd);
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
	unsigned int value;
	struct clock_event_device *evt = dev_id;
	if (evt->event_handler == NULL)
		return IRQ_HANDLED;

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

	setup_irq(IRQ_TIMER1, &sprd_gptimer_irq);//TODO

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
	unsigned int val;

	/* read multiple times in case of boundary issue */
	val = __raw_readl(TIMER_VALUE(SOURCE_TIMER));
	/* NOTE: register access is slower than 26Mhz, need more cycles */

	return (ULONG_MAX - val);
}

static struct clocksource sprd_gptimer_src = {
	.name		= "gptimer2",
	.rating		= 300,
	.read		= sprd_gptimer_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 26,
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


static cycle_t sprd_syscnt_read(struct clocksource *cs)
{
	unsigned int val1;
	val1 = __raw_readl(SYSCNT_COUNT);
	return val1;
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

/*
 * Constants generated by clocks_calc_mult_shift(m, s, 26MHz, NSEC_PER_SEC, 60).
 * This gives a resolution of about 41ns and a wrap period of about 178s.
 */
#define SC_MULT		2581110154u
#define SC_SHIFT	26

unsigned long long notrace sched_clock(void)
{
		u32 cyc = sprd_gptimer_read(NULL);
		return cyc_to_fixed_sched_clock(&cd, cyc, (u32)~0,
						SC_MULT, SC_SHIFT);
}

static void notrace tiger_update_sched_clock(void)
{
	u32 cyc = sprd_gptimer_read(NULL);
	update_sched_clock(&cd, cyc, (u32)~0);
}

static void __init tiger_sched_clock_init(unsigned long rate)
{
	init_fixed_sched_clock(&cd, tiger_update_sched_clock,
			       32, rate, SC_MULT, SC_SHIFT);
}


static void sprd_greg_set_bits1(uint32_t bits, uint32_t reg_offset)
{
        unsigned int value;
        value = __raw_readl(SPRD_GREG_BASE + reg_offset);
        value |= bits;
        __raw_writel(value, SPRD_GREG_BASE + reg_offset);
}

void tiger_enable_timer_early(void)
{
	/* enable timer & syscnt in global regs */
	sprd_greg_set_bits1(GEN0_TIMER_EN | GEN0_SYST_EN, GR_GEN0);
	tiger_sched_clock_init(26000000);
}
#if !defined(CONFIG_NKERNEL) || defined(CONFIG_NATIVE_LOCAL_TIMER)
#ifdef CONFIG_LOCAL_TIMERS
/*
 * Setup the local clock events for a CPU.
 */
int __cpuinit local_timer_setup(struct clock_event_device *evt)
{
	evt->irq = IRQ_LOCALTIMER;
	twd_timer_setup(evt);
	return 0;
}

#endif /* CONFIG_LOCAL_TIMERS */
#endif

void __init tiger_timer_init(void)
{
#if !defined(CONFIG_NKERNEL) || defined(CONFIG_NATIVE_LOCAL_TIMER)
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = (void __iomem *)TIGER_VA_PRIVATE_TIMER;
#endif
#endif
	/* setup timer2 and syscnt as clocksource */
	sprd_gptimer_clocksource_init();
	sprd_syscnt_clocksource_init();

	/* setup timer1 as clockevent.  */
	sprd_gptimer_clockevent_init();
}

