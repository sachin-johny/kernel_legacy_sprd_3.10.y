/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * Contact: steve.zhan <steve.zhan@spreadtrum.com>
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

/* general interrupt registers */
#define	INTC_IRQ_MSKSTS		(0x0000)
#define	INTC_IRQ_RAW		(0x0004)
#define	INTC_IRQ_EN		(0x0008)
#define	INTC_IRQ_DIS		(0x000C)
#define	INTC_IRQ_SOFT		(0x0010)

struct intc {
	u32 intc_reg_base;
	u32 min_int_num;/*global logical number max value in intc*/
	u32 max_min_num;/*global logical number min value in intc */
	u32 min_bit_offset;/*It is equal to "min_int_num - actual bit offset"*/
};

struct intc_mux_irq {/*multi irqs mapping to one bit*/
	int irq_number;
	int intc_base;
	int bits;
};

#if defined(CONFIG_ARCH_SC8830)
/*_intc and mux_irq is separately, we just simply enable/disable it at the same time
   When system deepsleep, only mux_irq(AON) can wakeup system(intc0,1,2,3,gic,armcore)
   intc0/1/2/3 will lose if system in deepsleep, _mux can wakeup. intc0/1/2/3 need SW retention.
   gic clock will stop if all of core has been wfi.intc0/1/2/3 can wakeup.
*/
static const struct intc _intc[] = {
		{SPRD_INTC0_BASE, 2, 31, 0},
		{SPRD_INTC1_BASE, 34, 63, 32},
		{SPRD_INTC2_BASE, 66, 95, 64},
		{SPRD_INTC3_BASE, 98, 121, 96},
};
static const struct intc_mux_irq _mux[] = {
		{30,SPRD_INT_BASE,2},{31,SPRD_INT_BASE,2},{28,SPRD_INT_BASE,2},
		{121,SPRD_INT_BASE,2},{120,SPRD_INT_BASE,2},{119,SPRD_INT_BASE,2},
		{118,SPRD_INT_BASE,2},{29,SPRD_INT_BASE,2},
		{21,SPRD_INT_BASE,3},{22,SPRD_INT_BASE,3},{23,SPRD_INT_BASE,3},{24,SPRD_INT_BASE,3},
		{35,SPRD_INT_BASE,4},{36,SPRD_INT_BASE,4},{38,SPRD_INT_BASE,4},{25,SPRD_INT_BASE,4},
		{27,SPRD_INT_BASE,4},{20,SPRD_INT_BASE,4},{34,SPRD_INT_BASE,4},
		{41,SPRD_INT_BASE,5},{40,SPRD_INT_BASE,5},{42,SPRD_INT_BASE,5},{44,SPRD_INT_BASE,5},
		{45,SPRD_INT_BASE,5},{43,SPRD_INT_BASE,5},
		{39,SPRD_INT_BASE,6},
		{85,SPRD_INT_BASE,7},{84,SPRD_INT_BASE,7},{83,SPRD_INT_BASE,7},
		{67,SPRD_INT_BASE,8},{68,SPRD_INT_BASE,8},{69,SPRD_INT_BASE,8},
		{70,SPRD_INT_BASE,9},{71,SPRD_INT_BASE,9},{72,SPRD_INT_BASE,9},
		{73,SPRD_INT_BASE,10},{74,SPRD_INT_BASE,10},
		{123,SPRD_INT_BASE,11},{124,SPRD_INT_BASE,11},
		{26,SPRD_INT_BASE,12},{122,SPRD_INT_BASE,12},
		{86,SPRD_INT_BASE,13},
		{37,SPRD_INT_BASE,14},
};

#define LEGACY_FIQ_BIT	(32)
#define LEGACY_IRQ_BIT	(29)

static __init void __irq_init(void){}

#else

static const struct intc _intc[] = {
		{SPRD_INTC0_BASE, 0, 31, 0},
		{SPRD_INTC1_BASE, 32, 61, 32},
};

static const struct intc_mux_irq _mux[] = {};

#define LEGACY_FIQ_BIT	(31)
#define LEGACY_IRQ_BIT	(28)

static __init void __irq_init(void)
{
	/*
	 * gic clock will be stopped after 2 cores enter standby in the same time,
	 * dsp assert if IRQ_DSP0_INT and IRQ_DSP1_INT are disabled. so enable IRQ_DSP0_INT
	 * and IRQ_DSP1_INT in INTC0 here.
	 */
	u32 val = __raw_readl(SPRD_INTC0_BASE + INTC_IRQ_EN);
	val |= (SCI_INTC_IRQ_BIT(IRQ_DSP0_INT) | SCI_INTC_IRQ_BIT(IRQ_DSP1_INT) | SCI_INTC_IRQ_BIT(IRQ_EPT_INT));
	val |= (SCI_INTC_IRQ_BIT(IRQ_SIM0_INT) | SCI_INTC_IRQ_BIT(IRQ_SIM1_INT));
	val |= (SCI_INTC_IRQ_BIT(IRQ_TIMER0_INT));
	__raw_writel(val, SPRD_INTC0_BASE + INTC_IRQ_EN);
}
#endif

static inline int __irq_mux_irq_find(u32 irq, u32 *base, u32 *bit)
{
	int s = ARRAY_SIZE(_mux);
	while (s--)
		if (_mux[s].irq_number == irq) {
			*base = _mux[s].intc_base;
			*bit = _mux[s].bits;
			return 0;
		}
	return -ENXIO;
}

static inline int __irq_find_base(u32 irq, u32 *base, u32 *bit)
{
	int s = ARRAY_SIZE(_intc);
	while (s--)
		if (irq >= _intc[s].min_int_num && irq <= _intc[s].max_min_num) {
			*base = _intc[s].intc_reg_base;
			*bit = irq - _intc[s].min_bit_offset;
			return 0;
		}
	return -ENXIO;
}

static void sci_irq_mask(struct irq_data *data)
{
	unsigned int irq = SCI_GET_INTC_IRQ(data->irq);
	u32 base;
	u32 bit;
	if (!__irq_mux_irq_find(irq, &base, &bit))
		__raw_writel(1 << bit, base + INTC_IRQ_DIS);
	if (!__irq_find_base(irq, &base, &bit))
		__raw_writel(1 << bit, base + INTC_IRQ_DIS);
}

static void sci_irq_unmask(struct irq_data *data)
{
	unsigned int irq = SCI_GET_INTC_IRQ(data->irq);
	u32 base;
	u32 bit;
	if (!__irq_mux_irq_find(irq, &base, &bit))
		__raw_writel(1 << bit, base + INTC_IRQ_EN);
	if (!__irq_find_base(irq, &base, &bit))
		__raw_writel(1 << bit, base + INTC_IRQ_EN);
}

static int sci_set_wake(struct irq_data *d, unsigned int on)
{
	/*we don't add mask/unmask in this now,  this is not mean mask interrupt*/
	return 0;
}

void __init sci_init_irq(void)
{
	gic_init(0, 29, (void __iomem *)CORE_GIC_DIS_VA,
		 (void __iomem *)CORE_GIC_CPU_VA);
	gic_arch_extn.irq_mask = sci_irq_mask;
	gic_arch_extn.irq_unmask = sci_irq_unmask;
	gic_arch_extn.irq_set_wake = sci_set_wake;
	ana_init_irq();

	/*disable legacy interrupt*/
	__raw_writel(1<<LEGACY_FIQ_BIT, CORE_GIC_DIS_VA + GIC_DIST_ENABLE_CLEAR);
	__raw_writel(1<<LEGACY_IRQ_BIT, CORE_GIC_DIS_VA + GIC_DIST_ENABLE_CLEAR);

	__irq_init();
}

