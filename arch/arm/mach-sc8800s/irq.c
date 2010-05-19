/* linux/arch/arm/mach-sc8800s/irq.c
 *
 * supporting stuff for interrupt handling 
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
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/timer.h>

#include <linux/irq.h>
#include <mach/hardware.h>

#include <asm/io.h>

#define INTCV_REG(off) (SPRD_INTCV_BASE + (off))

#define INTCV_FIQ_STS     INTCV_REG(0x0000)
#define INTCV_IRQ_STS     INTCV_REG(0x0004)
#define INTCV_INT_RAW     INTCV_REG(0x0008)
#define INTCV_INT_SEL     INTCV_REG(0x000C) /* 1: FIQ, 0: IRQ */
#define INTCV_INT_EN      INTCV_REG(0x0010) /* 1: enable, 0: disable */
#define INTCV_INT_EN_CLR  INTCV_REG(0x0014)
#define INTCV_PROTECT     INTCV_REG(0x0020)

static void sprd_irq_ack(unsigned int irq)
{
	/* nothing to do... */
}

static void sprd_irq_mask(unsigned int irq)
{
	__raw_writel(1 << (irq & 31), INTCV_INT_EN_CLR);
}

static void sprd_irq_unmask(unsigned int irq)
{
	__raw_writel(1 << (irq & 31), INTCV_INT_EN);
}

static int sprd_irq_set_wake(unsigned int irq, unsigned int on)
{
	return -EINVAL;
}

static int sprd_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	/* TODO: make sure our INTCV really has nothing to do with
	   type/polarity */
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		set_irq_handler(irq, handle_edge_irq);
	}
	if (flow_type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW)) {
		set_irq_handler(irq, handle_level_irq);
	}
	return 0;
}

static struct irq_chip sprd_irq_chip = {
	.name      = "sprd",
	.ack       = sprd_irq_ack,
	.mask      = sprd_irq_mask,
	.unmask    = sprd_irq_unmask,
	.set_wake  = sprd_irq_set_wake,
	.set_type  = sprd_irq_set_type,
};

void __init sprd_init_irq(void)
{
	unsigned n;

	/* enable INTCV write operation */
	__raw_writel(1, INTCV_PROTECT);

	for (n = 0; n < NR_SPRD_IRQS; n++) {
		set_irq_chip(n, &sprd_irq_chip);
		set_irq_handler(n, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}
}
