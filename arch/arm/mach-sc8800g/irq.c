/* linux/arch/arm/mach-sc8800g/irq.c
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
#include <asm/io.h>

#include <mach/hardware.h>
#include <mach/bits.h>
#include <mach/adi_hal_internal.h>

#define INTCV_REG(off) (SPRD_INTCV_BASE + (off))

//#define INTCV_FIQ_STS     INTCV_REG(0x0000)
#define INTCV_IRQ_STS     INTCV_REG(0x0000)
#define INTCV_INT_RAW     INTCV_REG(0x0004)
//#define INTCV_INT_SEL     INTCV_REG(0x0018) /* 1: FIQ, 0: IRQ */
#define INTCV_INT_EN      INTCV_REG(0x0008) /* 1: enable, 0: disable */
#define INTCV_INT_EN_CLR  INTCV_REG(0x000C)
//#define INTCV_PROTECT     INTCV_REG(0x0020)

/*Analog Die interrupt register*/
#define ANA_INT_STATUS             (SPRD_MISC_BASE +0x380+ 0x00)
#define ANA_INT_RAW                  (SPRD_MISC_BASE + 0x380 + 0x04)
#define ANA_INT_EN                	(SPRD_MISC_BASE + 0x380 + 0x08)
#define ANA_INT_STATUS_SYNC      (SPRD_MISC_BASE + 0x380 + 0x0C)

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

void sprd_ack_ana_irq(unsigned int irq)
{
	/* nothing to do... */
}
EXPORT_SYMBOL(sprd_ack_ana_irq);

void sprd_mask_ana_irq(unsigned int irq)
{
	ANA_REG_AND(ANA_INT_EN,~(1 << (irq % 32)));
}

EXPORT_SYMBOL(sprd_mask_ana_irq);

void sprd_unmask_ana_irq(unsigned int irq)
{
	ANA_REG_OR(ANA_INT_EN,1 << (irq % 32));
}

EXPORT_SYMBOL(sprd_unmask_ana_irq);

static void sprd_disable_ana_irq(unsigned int irq)
{
	sprd_mask_ana_irq(irq);
}

static void sprd_enable_ana_irq(unsigned int irq)
{
	sprd_unmask_ana_irq(irq);
}

static struct irq_chip sprd_muxed_ana_chip = {
	.name		= "ANA",
	.ack		= sprd_ack_ana_irq,
	.mask		= sprd_mask_ana_irq,
	.unmask		= sprd_unmask_ana_irq,
	.disable	= sprd_disable_ana_irq,
	.enable     = sprd_enable_ana_irq,
};

static void sprd_ana_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	uint32_t irq_ana;
	uint32_t status;
	int i;

	status=ANA_REG_GET(ANA_INT_STATUS);

	for (i = 0; i < NR_ANA_IRQS; i++) {
		if ((status >> i) & 0x1) {
			irq_ana = IRQ_ANA_INT_START + i;
			generic_handle_irq(irq_ana);
		}
	}

}

void __init sprd_init_irq(void)
{
	unsigned n;

	/* enable INTCV write operation */
	//__raw_writel(1, INTCV_PROTECT);

	for (n = 0; n < NR_SPRD_IRQS; n++) {
		set_irq_chip(n, &sprd_irq_chip);
		set_irq_handler(n, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}

	for (n = IRQ_ANA_ADC_INT; n < IRQ_ANA_ADC_INT+ NR_ANA_IRQS; n++) {
		set_irq_chip(n, &sprd_muxed_ana_chip);
		set_irq_flags(n, IRQF_VALID );//| IRQF_PROBE);
		set_irq_handler(n,handle_level_irq);
	}

	set_irq_chained_handler(IRQ_ANA_INT, sprd_ana_demux_handler);
}
