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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

/* general interrupt registers */
#define	INTCV_REG(off)        (SPRD_INTCV_BASE + (off))
#define	INTCV_IRQ_STS         INTCV_REG(0x0000)
#define	INTCV_IRQ_RAW         INTCV_REG(0x0004)
#define	INTCV_IRQ_EN          INTCV_REG(0x0008)
#define	INTCV_IRQ_DIS         INTCV_REG(0x000C)
#define	INTCV_IRQ_SOFT        INTCV_REG(0x0010)

#ifdef CONFIG_ARCH_SC7710

#define	INTC1_REG(off)		(SPRD_INTCV1_BASE + (off))
#define	INTCV1_IRQ_MSKSTS	INTC1_REG(0x0000)
#define	INTCV1_IRQ_RAW		INTC1_REG(0x0004)
#define	INTCV1_IRQ_EN		INTC1_REG(0x0008)
#define	INTCV1_IRQ_DIS		INTC1_REG(0x000C)
#define	INTCV1_IRQ_SOFT		INTC1_REG(0x0010)

#define INTC1_IRQ_NUM_MIN	(32)
#define INTC_NUM_MAX		(61)

#define __irq_msk_intc1(irq) do { \
	__raw_writel(1 << (irq - INTC1_IRQ_NUM_MIN),INTCV1_IRQ_DIS); \
} while (0)
#define __irq_unmsk_intc1(irq) do { \
	__raw_writel(1 << (irq - INTC1_IRQ_NUM_MIN),INTCV1_IRQ_EN); \
} while (0)

#else
#define INTC1_IRQ_NUM_MIN	(32)
#define INTC_NUM_MAX		(31)

#define __irq_msk_intc1(irq) do {} while(0)
#define __irq_unmsk_intc1(irq) do {} while(0)
#endif

static void sprd_irq_ack(struct irq_data *data)
{
	/* nothing to do... */
}

static void sprd_irq_mask(struct irq_data *data)
{
	unsigned int irq = data->irq;
	if (irq <= INTC_NUM_MAX) {
		if (irq < INTC1_IRQ_NUM_MIN) {
			__raw_writel(1 << irq, INTCV_IRQ_DIS);
		} else {
			__irq_msk_intc1(irq);
		}
	}
}

static void sprd_irq_unmask(struct irq_data *data)
{
	unsigned int irq = data->irq;
	if (irq <= INTC_NUM_MAX) {
		if (irq < INTC1_IRQ_NUM_MIN) {
			__raw_writel(1 << irq, INTCV_IRQ_EN);
		} else {
			__irq_unmsk_intc1(irq);
		}
	}
}

static int sprd_irq_set_wake(struct irq_data *data, unsigned int on)
{
	return 0;
}

static int sprd_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	/* TODO: make sure our INTCV really has nothing to do with type/polarity */
	if (flow_type & (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)) {
		irq_set_handler(data->irq, handle_edge_irq);
	}
	if (flow_type & (IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW)) {
		irq_set_handler(data->irq, handle_level_irq);
	}
	return 0;
}

static struct irq_chip sprd_irq_chip = {
	.name = "irq-sprd",
	.irq_ack = sprd_irq_ack,
	.irq_mask = sprd_irq_mask,
	.irq_unmask = sprd_irq_unmask,
	.irq_set_wake = sprd_irq_set_wake,
	.irq_set_type = sprd_irq_set_type,
};

void __init sc8810_init_irq(void)
{
	int n;
	for (n = 0; n < NR_SPRD_IRQS; n++) {
		irq_set_chip_and_handler(n, &sprd_irq_chip, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}
}
