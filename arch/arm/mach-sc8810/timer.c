/* linux/arch/arm/mach-sc8800g/timer.c
 *
 * timer support
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timex.h>
#include <linux/clockchips.h>


#include <linux/io.h>

#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
    
#define TIMER_REG(off) (SPRD_TIMER_BASE + (off))
#define TIMER0_LOAD     TIMER_REG(0x0000)
#define TIMER0_VALUE    TIMER_REG(0x0004)
#define TIMER0_CONTROL  TIMER_REG(0x0008)
#define TIMER0_CLEAR    TIMER_REG(0x000C)
#define TIMER1_LOAD     TIMER_REG(0x0020)
#define TIMER1_VALUE    TIMER_REG(0x0024)
#define TIMER1_CONTROL  TIMER_REG(0x0028)
#define TIMER1_CLEAR    TIMER_REG(0x002C)

#define SYSCNT_REG(off) (SPRD_SYSCNT_BASE + (off))
#define SYSCNT_COUNT    SYSCNT_REG(0x0004)
#define SYSCNT_CTL      SYSCNT_REG(0x0008)

#define GREG_REG(off)   (SPRD_GREG_BASE + (off))
#define GREG_GEN0       GREG_REG(0x0008)
#define GREG_GEN1       GREG_REG(0x0018)

#define GPTIMER_FREQ    32768
#define SYSCNT_FREQ	1000

#define GPTIMER_MAX_DELTA 0x7fffff /* 23-bit counter */
#define GPTIMER_MIN_DELTA 33        /* about 122us */ //sword
#ifndef CONFIG_NKERNEL
/*
 * we use one of the 2 general-purpose timers as the clock event device. 
 * timer1 is chosen as default.
 */
static irqreturn_t sprd_gptimer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	/* clear interrupt */
	__raw_bits_or((1<<3), TIMER0_CLEAR);

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static int
sprd_gptimer_set_next_event(unsigned long cycles, struct clock_event_device *c)
{
	//pr_info("cycle :%d\r\n", cycles);

	/*busy wait for timer loading finished*/
	while(__raw_readl(TIMER0_CLEAR) & (1<<4));

	__raw_writel(0, TIMER0_CONTROL);	

	//__raw_writel(0x7fffff, TIMER0_CLEAR);
	
	__raw_writel((1<<7), TIMER0_CONTROL); //sword

	__raw_writel(1, TIMER0_CLEAR);
	
	__raw_writel(cycles, TIMER0_LOAD);

	return cycles <= GPTIMER_MIN_DELTA ? -ETIME : 0;
}

static void
sprd_gptimer_set_mode(enum clock_event_mode mode, struct clock_event_device *c)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
		//__raw_writel((1<<7), TIMER0_CONTROL);
		//__raw_bits_and(~(1 << 6), TIMER0_CONTROL);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		__raw_writel(0, TIMER0_CONTROL);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		//__raw_bits_or((1 << 6), TIMER0_CONTROL);
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_UNUSED:
		break;
	}
}

static struct clock_event_device sprd_gptimer = {
	.name		= "timer0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.rating		= 200,
	/*.cpumask	= CPU_MASK_CPU0, */
	.set_next_event	= sprd_gptimer_set_next_event,
	.set_mode	= sprd_gptimer_set_mode,
};

static struct irqaction sprd_gptimer_irq = {
	.name		= "timer0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= sprd_gptimer_interrupt,
	.dev_id		= &sprd_gptimer,
};

/*
 * we use the system counter as the clock source.
 */
static cycle_t sprd_syscnt_read(struct clocksource *cs)
{
	return __raw_readl(SYSCNT_COUNT);
}

static struct clocksource sprd_syscnt = {
	.name		= "syscnt",
	.rating		= 200,
	.read		= sprd_syscnt_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 8,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

unsigned long long sched_clock(void)
{
	return clocksource_cyc2ns(sprd_syscnt.read(&sprd_syscnt),
				  sprd_syscnt.mult, sprd_syscnt.shift);
}

static void __init sprd_timer_init(void)
{
	/* enable timer0 */
	__raw_bits_or((1 << 2), GREG_GEN0);
        
	/* init timer0 */
	__raw_writel((1<<3), TIMER0_CLEAR);
	__raw_writel(0, TIMER0_CONTROL); //sword
	//tmp = __raw_readl(TIMER0_CLEAR);
	//tmp |= 1;
	//__raw_writel(tmp,TIMER0_CLEAR); //sword

	/* enable system counter */
	__raw_bits_or((1 << 19), GREG_GEN0); /* ? */

	sprd_gptimer.mult =
		div_sc(GPTIMER_FREQ, NSEC_PER_SEC, sprd_gptimer.shift);
	sprd_gptimer.max_delta_ns =
		clockevent_delta2ns(GPTIMER_MAX_DELTA, &sprd_gptimer);
	sprd_gptimer.min_delta_ns =
		clockevent_delta2ns(GPTIMER_MIN_DELTA, &sprd_gptimer) + 1;
	sprd_gptimer.cpumask = cpumask_of(0);

	sprd_syscnt.mult =
		clocksource_hz2mult(SYSCNT_FREQ, sprd_syscnt.shift);

	pr_info("min_delta_ns:%d\n", sprd_gptimer.min_delta_ns);
	clocksource_register(&sprd_syscnt);
	clockevents_register_device(&sprd_gptimer);
	setup_irq(IRQ_TIMER0_INT, &sprd_gptimer_irq);
}
#else

#include <mach/test.h>

/*
 * we use one of the 2 general-purpose timers as the clock event device. 
 * timer1 is chosen as default.
 */
static irqreturn_t sprd_gptimer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;
	/*
	if (get_sys_cnt() > 150000) {
		printk("##: timer interrupt!\n");
	}
	*/

	/* clear interrupt */
	__raw_bits_or((1<<3), TIMER1_CLEAR);

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static int
sprd_gptimer_set_next_event(unsigned long cycles, struct clock_event_device *c)
{
	/*
	if (get_sys_cnt() > 150000) {
		printk("cycles = %d\n", cycles);
	}
	*/

	/*busy wait for timer loading finished*/
	while(__raw_readl(TIMER1_CLEAR) & (1<<4));

	__raw_writel(0, TIMER1_CONTROL);	

	//__raw_writel(0x7fffff, TIMER1_CLEAR);
	
	__raw_writel((1<<7), TIMER1_CONTROL); //sword

	__raw_writel(1, TIMER1_CLEAR);
	
	__raw_writel(cycles, TIMER1_LOAD);

	return cycles <= GPTIMER_MIN_DELTA ? -ETIME : 0;
}



static void sprd_gptimer_set_mode(enum clock_event_mode mode, struct clock_event_device *c);

static struct clock_event_device sprd_gptimer = {
	.name		= "timer1",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.rating		= 200,
	/*.cpumask	= CPU_MASK_CPU0, */
	.set_next_event	= sprd_gptimer_set_next_event,
	.set_mode	= sprd_gptimer_set_mode,
};

static void
sprd_gptimer_set_mode(enum clock_event_mode mode, struct clock_event_device *c)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
		//__raw_writel((1<<7), TIMER1_CONTROL);
		//__raw_bits_and(~(1 << 6), TIMER1_CONTROL);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		__raw_writel(0, TIMER1_CONTROL);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		//__raw_bits_or((1 << 6), TIMER1_CONTROL);
	case CLOCK_EVT_MODE_RESUME:
		/*
		printk("## timer resume!\n");
		sprd_gptimer_set_next_event(1, &sprd_gptimer);
		udelay(2);
		break;
		*/
	case CLOCK_EVT_MODE_UNUSED:
		break;
	}
}


static struct irqaction sprd_gptimer_irq = {
	.name		= "timer1",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= sprd_gptimer_interrupt,
	.dev_id		= &sprd_gptimer,
};

#ifdef CONFIG_LOW_RESOLUTION_CLKSRC
static u32 saved_syscnt = 0, same_cnt_num = 0;
#endif

/*
 * we use the system counter as the clock source.
 */
static cycle_t sprd_syscnt_read(struct clocksource *cs)
{
	u32 val1, val2;
	val1 = __raw_readl(SYSCNT_COUNT);
	val2 = __raw_readl(SYSCNT_COUNT);
	while(val2 != val1) {
		val1 = val2;
		val2 = __raw_readl(SYSCNT_COUNT);
	}
#ifdef CONFIG_LOW_RESOLUTION_CLKSRC
	/* FIXME: fix the bug that low resolution timer will cause cts failed, use a high one in nexttime pls */
	if(saved_syscnt == val2)
		same_cnt_num++;
	else {
		same_cnt_num = 0;
		saved_syscnt = val2;
	}
#endif
	return val2;
}

static struct clocksource sprd_syscnt = {
	.name		= "syscnt",
	.rating		= 200,
	.read		= sprd_syscnt_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 8,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

unsigned long long sched_clock(void)
{
#ifdef CONFIG_LOW_RESOLUTION_CLKSRC
	/* FIXME: fix the bug that low resolution timer will cause cts failed, use a high one in nexttime pls */
	return clocksource_cyc2ns(sprd_syscnt.read(&sprd_syscnt),
				  sprd_syscnt.mult, sprd_syscnt.shift) + same_cnt_num;
#else
	return clocksource_cyc2ns(sprd_syscnt.read(&sprd_syscnt),
				  sprd_syscnt.mult, sprd_syscnt.shift);
#endif
}

void read_persistent_clock(struct timespec *ts)
{

	u32 msecs = sprd_syscnt_read(&sprd_syscnt);
	ts->tv_sec = msecs / 1000;
	ts->tv_nsec = (msecs % 1000) * 1000 * 1000;
	//printk("##read_persistent_clock(): ts = %lld\n", timespec_to_ns(ts));
}


static void __init sprd_timer_init(void)
{
	/* enable timer1 */
	__raw_bits_or((1 << 2), GREG_GEN0);
        
	/* init timer1 */
	__raw_writel((1<<3), TIMER1_CLEAR);
	__raw_writel(0, TIMER1_CONTROL); //sword
	//tmp = __raw_readl(TIMER1_CLEAR);
	//tmp |= 1;
	//__raw_writel(tmp,TIMER1_CLEAR); //sword

	/* enable system counter */
	__raw_bits_or((1 << 19), GREG_GEN0); /* ? */

	sprd_gptimer.mult =
		div_sc(GPTIMER_FREQ, NSEC_PER_SEC, sprd_gptimer.shift);
	sprd_gptimer.max_delta_ns =
		clockevent_delta2ns(GPTIMER_MAX_DELTA, &sprd_gptimer);
	sprd_gptimer.min_delta_ns =
		clockevent_delta2ns(GPTIMER_MIN_DELTA, &sprd_gptimer) + 1;
	sprd_gptimer.cpumask = cpumask_of(0);

	sprd_syscnt.mult =
		clocksource_hz2mult(SYSCNT_FREQ, sprd_syscnt.shift);

	pr_info("min_delta_ns:%d\n", sprd_gptimer.min_delta_ns);
	clocksource_register(&sprd_syscnt);
	clockevents_register_device(&sprd_gptimer);
	setup_irq(IRQ_TIMER1_INT, &sprd_gptimer_irq);
}
#endif

#ifdef CONFIG_PM
/* TODO: ... */
static void sprd_timer_suspend(void)
{

}

static void sprd_timer_resume(void)
{

}
#else
#define sprd_timer_suspend NULL
#define sprd_timer_resume NULL
#endif

struct sys_timer sprd_timer = {
	.init		= sprd_timer_init,
	.suspend	= sprd_timer_suspend,
	.resume		= sprd_timer_resume,
};

