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

#include <asm/hardware/gic.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

#ifdef CONFIG_NKERNEL
#include <nk/nkern.h>
#define CONFIG_NKERNEL_NO_SHARED_IRQ
#endif

/* general interrupt registers */
#define	INTC0_REG(off)		(SPRD_INTC0_BASE + (off))
#define	INTCV0_IRQ_MSKSTS	INTC0_REG(0x0000)
#define	INTCV0_IRQ_RAW		INTC0_REG(0x0004)
#define	INTCV0_IRQ_EN		INTC0_REG(0x0008)
#define	INTCV0_IRQ_DIS		INTC0_REG(0x000C)
#define	INTCV0_IRQ_SOFT		INTC0_REG(0x0010)

#define	INTC1_REG(off)		(SPRD_INTC0_BASE + 0x1000 + (off))
#define	INTCV1_IRQ_MSKSTS	INTC1_REG(0x0000)
#define	INTCV1_IRQ_RAW		INTC1_REG(0x0004)
#define	INTCV1_IRQ_EN		INTC1_REG(0x0008)
#define	INTCV1_IRQ_DIS		INTC1_REG(0x000C)
#define	INTCV1_IRQ_SOFT		INTC1_REG(0x0010)

#define INTC1_IRQ_NUM_MIN	(32)
#define INTC_NUM_MAX		(61)

#ifndef CONFIG_NKERNEL
static void sci_irq_eoi(struct irq_data *data)
{
	/* nothing to do... */
}

#ifdef CONFIG_PM
static int sci_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}

#else
#define sci_set_wake	NULL
#endif

static void sci_irq_mask(struct irq_data *data)
{
	unsigned int irq = SCI_GET_INTC_IRQ(data->irq);
	if (irq <= INTC_NUM_MAX) {
		if (irq >= INTC1_IRQ_NUM_MIN) {
			__raw_writel(1 << (irq - INTC1_IRQ_NUM_MIN),
				     INTCV1_IRQ_DIS);
		} else {
			__raw_writel(1 << irq, INTCV0_IRQ_DIS);
		}
	}
}

static void sci_irq_unmask(struct irq_data *data)
{
	unsigned int irq = SCI_GET_INTC_IRQ(data->irq);
	if (irq <= INTC_NUM_MAX) {
		if (irq >= INTC1_IRQ_NUM_MIN) {
			__raw_writel(1 << (irq - INTC1_IRQ_NUM_MIN),
				     INTCV1_IRQ_EN);
		} else {
			__raw_writel(1 << irq, INTCV0_IRQ_EN);
		}
	}
}

void __init sc8825_init_irq(void)
{
	gic_init(0, 29, (void __iomem *)SC8825_VA_GIC_DIS,
		 (void __iomem *)SC8825_VA_GIC_CPU);
	gic_arch_extn.irq_eoi = sci_irq_eoi;
	gic_arch_extn.irq_mask = sci_irq_mask;
	gic_arch_extn.irq_unmask = sci_irq_unmask;
	gic_arch_extn.irq_set_wake = sci_set_wake;
	ana_init_irq();
}

#else /* CONFIG_NKERNEL */

extern NkDevXPic *nkxpic;	/* virtual XPIC device */
extern NkOsId nkxpic_owner;	/* owner of the virtual XPIC device */
extern NkOsMask nkosmask;	/* my OS mask */

extern void nk_ddi_init(void);
extern void __nk_xirq_startup(struct irq_data *d);
extern void __nk_xirq_shutdown(struct irq_data *d);

static unsigned int nk_startup_irq(struct irq_data *data)
{
	__nk_xirq_startup(data);
#ifdef CONFIG_NKERNEL_NO_SHARED_IRQ
	nkxpic->irq[data->irq].os_enabled = nkosmask;
#else
	nkxpic->irq[data->irq].os_enabled |= nkosmask;
#endif
	nkops.nk_xirq_trigger(nkxpic->xirq, nkxpic_owner);

	return 0;
}

static void nk_shutdown_irq(struct irq_data *data)
{
	__nk_xirq_shutdown(data);
#ifdef CONFIG_NKERNEL_NO_SHARED_IRQ
	nkxpic->irq[data->irq].os_enabled = 0;
#else
	nkxpic->irq[irq].os_enabled &= ~nkosmask;
#endif
	nkops.nk_xirq_trigger(nkxpic->xirq, nkxpic_owner);
}

static void nk_sprd_mask_ack_irq(struct irq_data *data)
{
	/*
	 * mask_ack() is called only from handle_level_irq.
	 * in our case this job is already done by vpic
	 *
	 * we do not define mask(), because it is called
	 * only from interrupt migration code. No migration
	 * for us because we do not have set_affinity().
	 */
}

static void nk_sprd_ack_irq(struct irq_data *data)
{
	/*
	 * ack might be called by some stupid drivers
	 * for cascaded interrupt controllers
	 */
}

static void nk_sprd_unmask_irq(struct irq_data *data)
{
#ifdef CONFIG_NKERNEL_NO_SHARED_IRQ
	if (data->irq > 31)
		printk("nk_sprd_unmask_irq, irq error = 0x%x\n", data->irq);
	__raw_writel(1 << (data->irq & 31), INTCV0_IRQ_EN);
#else
	nkops.nk_xirq_trigger(data->irq, nkxpic_owner);
#endif
}

static struct irq_chip nk_sprd_irq_chip = {
	.name = "irq-sprd",
	.irq_mask_ack = nk_sprd_mask_ack_irq,
	.irq_ack = nk_sprd_ack_irq,
	.irq_unmask = nk_sprd_unmask_irq,
	.irq_startup = nk_startup_irq,
	.irq_shutdown = nk_shutdown_irq,
};

void __init sc8825_init_irq(void)
{
	nk_ddi_init();
	gic_init(0, 29, SC8825_VA_GIC_DIS, SC8825_VA_GIC_CPU);
	ana_init_irq();
}
#endif /* CONFIG_NKERNEL */
