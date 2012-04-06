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

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/adi.h>
#include <mach/ana_ctl_int.h>
#include "adi_internal.h"

/* general interrupt registers */
#define	INTCV_REG(off)        (SPRD_INTCV_BASE + (off))
#define	INTCV_IRQ_STS         INTCV_REG(0x0000)
#define	INTCV_IRQ_RAW         INTCV_REG(0x0004)
#define	INTCV_IRQ_EN          INTCV_REG(0x0008)
#define	INTCV_IRQ_DIS         INTCV_REG(0x000C)
#define	INTCV_IRQ_SOFT        INTCV_REG(0x0010)

static void sprd_irq_ack(struct irq_data *data)
{
	/* nothing to do... */
}

static void sprd_irq_mask(struct irq_data *data)
{
	__raw_writel(1 << (data->irq & 31), INTCV_IRQ_DIS);
}

static void sprd_irq_unmask(struct irq_data *data)
{
	__raw_writel(1 << (data->irq & 31), INTCV_IRQ_EN);
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
	.name = "sprd",
	.irq_ack = sprd_irq_ack,
	.irq_mask = sprd_irq_mask,
	.irq_unmask = sprd_irq_unmask,
	.irq_set_wake = sprd_irq_set_wake,
	.irq_set_type = sprd_irq_set_type,
};

/* ****************************************************************** */

/* Analog Die interrupt registers */
#define ANA_CTL_INT_BASE				( SPRD_MISC_BASE + 0x380 )

void sprd_ack_ana_irq(struct irq_data *data)
{
	/* nothing to do... */
}

static void sprd_mask_ana_irq(struct irq_data *data)
{
	int offset = data->irq - IRQ_ANA_INT_START;
	sci_adi_clr(ANA_REG_INT_EN, BIT(offset & MASK_ANA_INT));
}

static void sprd_unmask_ana_irq(struct irq_data *data)
{
	int offset = data->irq - IRQ_ANA_INT_START;
	sci_adi_set(ANA_REG_INT_EN, BIT(offset & MASK_ANA_INT));
}

/* WARN: disable/enable is the same with umask/mask. */
static void sprd_disable_ana_irq(struct irq_data *data)
{
	sprd_mask_ana_irq(data);
}

static void sprd_enable_ana_irq(struct irq_data *data)
{
	sprd_unmask_ana_irq(data);
}

static struct irq_chip sprd_muxed_ana_chip = {
	.name = "ANA",
	.irq_ack = sprd_ack_ana_irq,
	.irq_mask = sprd_mask_ana_irq,
	.irq_unmask = sprd_unmask_ana_irq,
	.irq_disable = sprd_disable_ana_irq,
	.irq_enable = sprd_enable_ana_irq,
};

static void sprd_ana_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	uint32_t irq_ana;
	uint32_t status;
	int i;

	desc->irq_data.chip->irq_mask(&desc->irq_data);
	if (desc->irq_data.chip->irq_ack)
		desc->irq_data.chip->irq_ack(&desc->irq_data);
	/* TODO IF gpu has interrupt coming
	   if (gpu_has_interrupt)
	   generic_handle_irq(gpu_interrpt_id);
	 */
	status = sci_adi_read(ANA_REG_INT_MASK_STATUS);

	for (i = 0; i < NR_ANA_IRQS; ++i) {
		if ((status >> i) & 0x1) {
			irq_ana = IRQ_ANA_INT_START + i;
			generic_handle_irq(irq_ana);
		}
	}
	if (desc->irq_data.chip->irq_unmask)
		desc->irq_data.chip->irq_unmask(&desc->irq_data);
}

void __init sc8810_init_irq(void)
{
	int n;
	for (n = 0; n < NR_SPRD_IRQS; n++) {
		irq_set_chip_and_handler(n, &sprd_irq_chip, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}

	irq_set_chained_handler(IRQ_ANA_INT, sprd_ana_demux_handler);
	for (n = IRQ_ANA_INT_START; n < IRQ_ANA_INT_START + NR_ANA_IRQS; n++) {
		irq_set_chip_and_handler(n, &sprd_muxed_ana_chip,
					 handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}
}
