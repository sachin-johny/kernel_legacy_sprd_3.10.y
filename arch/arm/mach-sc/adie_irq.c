/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 * Author: steve.zhan <steve.zhan@spreadtrum.com>
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/irqflags.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <asm/delay.h>

#include <mach/hardware.h>
#include <mach/globalregs.h>
#include <mach/adi.h>
#include <mach/irqs.h>

/* registers definitions for controller ANA_CTL_INT */
#define ANA_REG_INT_MASK_STATUS         (ANA_CTL_INT_BASE + 0x0000)
#define ANA_REG_INT_RAW_STATUS          (ANA_CTL_INT_BASE + 0x0004)
#define ANA_REG_INT_EN                  (ANA_CTL_INT_BASE + 0x0008)
#define ANA_REG_INT_MASK_STATUS_SYNC    (ANA_CTL_INT_BASE + 0x000c)

/* bits definitions for register REG_INT_MASK_STATUS */
#if defined(CONFIG_ARCH_SC8825)
/* vars definitions for controller ANA_CTL_INT */
#define MASK_ANA_INT                    ( 0xFF )
#elif defined(CONFIG_ARCH_SCX15)
/* vars definitions for controller ANA_CTL_INT */
#define MASK_ANA_INT                    ( 0x3FF )
#elif defined(CONFIG_ARCH_SCX35)
/* vars definitions for controller ANA_CTL_INT */
#define MASK_ANA_INT                    ( 0x7FF )
#else
#error "pls define interrupt number mask value"
#endif


static struct irq_domain *irq_domain;
void sprd_ack_ana_irq(struct irq_data *data)
{
	/* nothing to do... */
}

static void sprd_mask_ana_irq(struct irq_data *data)
{
	int offset = data->irq - IRQ_ANA_INT_START;
	pr_debug("%s %d\n", __FUNCTION__, data->irq);
	sci_adi_write(ANA_REG_INT_EN, 0, BIT(offset) & MASK_ANA_INT);
}

static void sprd_unmask_ana_irq(struct irq_data *data)
{
	int offset = data->irq - IRQ_ANA_INT_START;
	pr_debug("%s %d\n", __FUNCTION__, data->irq);
	sci_adi_write(ANA_REG_INT_EN, BIT(offset) & MASK_ANA_INT, 0);
}

static struct irq_chip sprd_muxed_ana_chip = {
	.name = "irq-ANA",
	.irq_ack = sprd_ack_ana_irq,
	.irq_mask = sprd_mask_ana_irq,
	.irq_unmask = sprd_unmask_ana_irq,
};

static irqreturn_t sprd_muxed_ana_handler(int irq, void *dev_id)
{
	uint32_t irq_ana, status;
	int i;

	status = sci_adi_read(ANA_REG_INT_MASK_STATUS) & MASK_ANA_INT;
	pr_debug("%s %d 0x%08x\n", __FUNCTION__, irq, status);
	while (status) {
		i = __ffs(status);
		status &= ~(1 << i);
#ifndef CONFIG_OF
		irq_ana = IRQ_ANA_INT_START + i;
#else
		irq_ana = irq_find_mapping(irq_domain, i);
#endif
		pr_debug("%s generic_handle_irq %d\n", __FUNCTION__, irq_ana);
		generic_handle_irq(irq_ana);
	}
	return IRQ_HANDLED;
}

static struct irqaction __adie_mux_irq = {
	.name		= "adie_mux",
	.flags		= IRQF_DISABLED | IRQF_NO_SUSPEND,
	.handler	= sprd_muxed_ana_handler,
};

void __init ana_init_irq(void)
{
	int n;

	for (n = IRQ_ANA_INT_START; n < IRQ_ANA_INT_START + NR_ANA_IRQS; n++) {
		irq_set_chip_and_handler(n, &sprd_muxed_ana_chip,
					 handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}
	setup_irq(IRQ_ANA_INT, &__adie_mux_irq);

}

#ifdef CONFIG_OF
#define IRQCHIP_DECLARE(name,compstr,fn)				\
	static const struct of_device_id irqchip_of_match_##name	\
	__used __section(__irqchip_of_table)				\
	= { .compatible = compstr, .data = fn }
int __init adi_of_init(struct device_node *node, struct device_node *parent)
{
	void __iomem *v_base;
	uint32_t reg_size, irqnums, i;
	struct resource res;
	int irq;

	if (WARN_ON(!node))
		return -ENODEV;

	if(of_address_to_resource(node, 0, &res)){
		pr_err("no reg of property specified for adi\n");
		return -ENODEV;
	}
	v_base = res.start;
	reg_size = resource_size(&res);

	if (of_property_read_u32(node, "sprd,irqnums", &irqnums)){
		pr_err("no sprd,irqnums of property specified");
		BUG();
	}

	irq_domain = irq_domain_add_linear(node, irqnums,
			&irq_domain_simple_ops, NULL);
	BUG_ON(!irq_domain);

	if (parent) {
		irq = irq_of_parse_and_map(node, 0);
		setup_irq(irq, &__adie_mux_irq);
	}
	for(i = 0; i < irqnums; i++){
		irq = irq_create_mapping(irq_domain, i);
		irq_set_chip_and_handler(irq, &sprd_muxed_ana_chip,
					 handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}
	return 0;
}
IRQCHIP_DECLARE(sprd_adi, "sprd,adi-bus", adi_of_init);

#endif
