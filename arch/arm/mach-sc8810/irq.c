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

#ifdef CONFIG_NKERNEL

#include <nk/nkern.h>

#define CONFIG_NKERNEL_NO_SHARED_IRQ

extern void nk_ddi_init(void);
static struct irq_chip nk_sprd_irq_chip;

#endif

#define INTCV_REG(off) (SPRD_INTCV_BASE + (off))

#define INTCV_IRQ_STS     INTCV_REG(0x0000)
#define INTCV_INT_RAW     INTCV_REG(0x0004)
//#define INTCV_INT_SEL     INTCV_REG(0x0018) /* 1: FIQ, 0: IRQ */
#define INTCV_INT_EN      INTCV_REG(0x0008)	/* 1: enable, 0: disable */
#define INTCV_INT_EN_CLR  INTCV_REG(0x000C)
//#define INTCV_PROTECT     INTCV_REG(0x0020)

/*Analog Die interrupt register*/
#define ANA_INT_STATUS             (SPRD_MISC_BASE +0x380+ 0x00)
#define ANA_INT_RAW                  (SPRD_MISC_BASE + 0x380 + 0x04)
#define ANA_INT_EN                	(SPRD_MISC_BASE + 0x380 + 0x08)
#define ANA_INT_STATUS_SYNC      (SPRD_MISC_BASE + 0x380 + 0x0C)

#ifndef CONFIG_NKERNEL

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
	.name = "sprd",
	.ack = sprd_irq_ack,
	.mask = sprd_irq_mask,
	.unmask = sprd_irq_unmask,
	.set_wake = sprd_irq_set_wake,
	.set_type = sprd_irq_set_type,
};

void sprd_ack_ana_irq(unsigned int irq)
{
	/* nothing to do... */
}

EXPORT_SYMBOL(sprd_ack_ana_irq);

void sprd_mask_ana_irq(unsigned int irq)
{
	ANA_REG_AND(ANA_INT_EN, ~(1 << (irq % 32)));
}

EXPORT_SYMBOL(sprd_mask_ana_irq);

void sprd_unmask_ana_irq(unsigned int irq)
{
	ANA_REG_OR(ANA_INT_EN, 1 << (irq % 32));
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
	.name = "ANA",
	.ack = sprd_ack_ana_irq,
	.mask = sprd_mask_ana_irq,
	.unmask = sprd_unmask_ana_irq,
	.disable = sprd_disable_ana_irq,
	.enable = sprd_enable_ana_irq,
};

static void sprd_ana_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	uint32_t irq_ana;
	uint32_t status;
	int i;

	desc->chip->mask(irq);
	if (desc->chip->ack)
		desc->chip->ack(irq);
/*TODO IF gpu has interrupt coming 
	if (gpu_has_interrupt)
		generic_handle_irq(gpu_interrpt_id);
*/	
	status = ANA_REG_GET(ANA_INT_STATUS);

	for (i = 0; i < NR_ANA_IRQS; ++i) {
		if ((status >> i) & 0x1) {
			irq_ana = IRQ_ANA_INT_START + i;
			//pr_info("ana irq\r\n");
			generic_handle_irq(irq_ana);
		}
	}
	if (desc->chip->unmask)
		desc->chip->unmask(irq);

}

#else /* CONFIG_NKERNEL */

extern NkDevXPic*   nkxpic;		/* virtual XPIC device */
extern NkOsId       nkxpic_owner;	/* owner of the virtual XPIC device */
extern NkOsMask     nkosmask;		/* my OS mask */

extern void __nk_xirq_startup  (NkXIrq xirq);
extern void __nk_xirq_shutdown (NkXIrq xirq);

    static unsigned int
nk_startup_irq (unsigned int irq)
{
	__nk_xirq_startup(irq);
#ifdef CONFIG_NKERNEL_NO_SHARED_IRQ
	nkxpic->irq[irq].os_enabled  = nkosmask;
#else
	nkxpic->irq[irq].os_enabled |= nkosmask;
#endif
	nkops.nk_xirq_trigger(nkxpic->xirq, nkxpic_owner);

	return 0;
}

    static void
nk_shutdown_irq (unsigned int irq)
{
	__nk_xirq_shutdown(irq);
#ifdef CONFIG_NKERNEL_NO_SHARED_IRQ
	nkxpic->irq[irq].os_enabled  = 0;
#else
	nkxpic->irq[irq].os_enabled &= ~nkosmask;
#endif
	nkops.nk_xirq_trigger(nkxpic->xirq, nkxpic_owner);
}

    static void
nk_sprd_mask_ack_irq (unsigned int irq)
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

    static void
nk_sprd_ack_irq (unsigned int irq)
{
    /*
     * ack might be called by some stupid drivers
     * for cascaded interrupt controllers
     */
}

    static void
nk_sprd_unmask_irq (unsigned int irq)
{
	if (irq >31 )
		return ;
#ifdef CONFIG_NKERNEL_NO_SHARED_IRQ
	__raw_writel(1 << (irq & 31), INTCV_INT_EN);
#else
	nkops.nk_xirq_trigger(irq, nkxpic_owner);
#endif
}

static struct irq_chip nk_sprd_irq_chip = {
	.name		= "sprd",
	.mask_ack	= nk_sprd_mask_ack_irq,
	.ack		= nk_sprd_ack_irq,
	.unmask		= nk_sprd_unmask_irq,
	.startup	= nk_startup_irq,
	.shutdown	= nk_shutdown_irq,
};

    static void
nk_sprd_ack_ana_irq(unsigned int irq)
{
	/* nothing to do... */
}

    static void
nk_sprd_mask_ana_irq(unsigned int irq)
{
	/* nothing to do... */
}

    static void
nk_sprd_unmask_ana_irq(unsigned int irq)
{
	int value = *(volatile*)(INTCV_INT_EN);
	printk("linux nk_sprd_unmask_ana_irq int_enable = 0x%x\n",value );
	nkops.nk_xirq_trigger(irq, nkxpic_owner);	
}

static struct irq_chip nk_sprd_muxed_ana_chip = {
	.name		= "ANA",
	.ack		= nk_sprd_ack_ana_irq,
	.mask		= nk_sprd_mask_ana_irq,
	.unmask		= nk_sprd_unmask_ana_irq,
        .startup        = nk_startup_irq,
        .shutdown       = nk_shutdown_irq,
};

#endif /* CONFIG_NKERNEL */

void( *eic_mux_handler)(unsigned int irq, struct irq_desc *desc) = NULL;
void( *gpio_mux_handler)(unsigned int irq, struct irq_desc *desc) = NULL;

EXPORT_SYMBOL(eic_mux_handler);
EXPORT_SYMBOL(gpio_mux_handler);


static void eic_gpio_mux_handler(unsigned int irq, struct irq_desc *desc)
{
	if (eic_mux_handler)
		eic_mux_handler(irq, desc);
	if (gpio_mux_handler)
		gpio_mux_handler(irq,desc);
}

#ifdef CONFIG_NKERNEL
void sprd_enable_ana_irq(void)
{
	__raw_writel(1 << (IRQ_ANA_INT), INTCV_INT_EN);
}
EXPORT_SYMBOL(sprd_enable_ana_irq);

#endif
void __init sprd_init_irq(void)
{
	unsigned int n;

#ifndef CONFIG_NKERNEL

	for (n = 0; n < NR_SPRD_IRQS; n++) {
		set_irq_chip(n, &sprd_irq_chip);
		set_irq_handler(n, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}

	for (n = IRQ_ANA_ADC_INT; n < IRQ_ANA_ADC_INT + NR_ANA_IRQS; ++n) {
		set_irq_chip(n, &sprd_muxed_ana_chip);
		set_irq_flags(n, IRQF_VALID);
		set_irq_handler(n, handle_level_irq);
	}

	set_irq_chained_handler(IRQ_ANA_INT, sprd_ana_demux_handler);//Shared
#else /* CONFIG_NKERNEL */

	nk_ddi_init();
	for (n = 0; n < NR_SPRD_IRQS; ++n) {
		if (n != IRQ_ANA_INT) {
			set_irq_chip(n, &nk_sprd_irq_chip);
			set_irq_handler(n, handle_level_irq);
			set_irq_flags(n, IRQF_VALID);
		}
	}

	for (n = IRQ_ANA_ADC_INT; n < IRQ_ANA_ADC_INT+ NR_ANA_IRQS; ++n) {
		set_irq_chip(n, &nk_sprd_muxed_ana_chip);
		set_irq_flags(n, IRQF_VALID );//| IRQF_PROBE);
		set_irq_handler(n,handle_level_irq);
	}
#endif /* CONFIG_NKERNEL */

	set_irq_chained_handler(IRQ_GPIO_EIC_INT, eic_gpio_mux_handler);
}
