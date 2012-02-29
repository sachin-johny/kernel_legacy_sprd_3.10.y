/*
 *  linux/arch/arm/mach-sc8810/eic.c
 *
 *  Spreadtrum eic unit
 *
 *
 *  Author:	steve.zhan@spreadtrum.com
 *  Created:	Wed jul 20, 2011
 *  Copyright:	Spreadtrum Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/bug.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <mach/regs_global.h>
#include <mach/regs_ana.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/adi_hal_internal.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>

#include <mach/eic.h>

#define EIC_TYPE	(0x1<<0)
#define SIC_TYPE	(0x0<<0)
#define EIC_SIC_MASK	(0x1<<0)

#define A_DIE		(0x1<<1)
#define D_DIE		(0X0<<1)
#define A_D_DIE_MASK	(0X1<<1)

#define INTER_ID(x)	((x)<<15)
#define GET_INTER_ID(x) (((x)>>15)&0xff)	//internal_id from 0-7

struct eic_config {
	enum EIC_TYPE_E eic_export_id;
	unsigned int type;
	// unsigned int internal_id;   /*find bit or find control register */
	unsigned int base_addr;
};

//config begin
#define EIC_BASE_ADDR_D_V	(SPRD_EIC_BASE + 0x0)	//v addr
#define EIC_BASE_ADDR_A_V	(SPRD_MISC_BASE + 0x700)	//v addr.
#define EIC_BASE_ADDR_V	(SPRD_MISC_BASE + 0x600)

#define EIC_SOFTRESET_ADDR_V (SPRD_MISC_BASE + 0x604)

#define SIC_BASE_ADDR_D_P	(0x8a001080)	//Phy.addr
#define SIC_BASE_ADDR_D_V	(SPRD_EIC_BASE + 0x80)	//Phy.addr

//
#define EIC_DATA_OFFSET	(0x00)	//EICDATA sync data input with 2cycles of rtcdiv5_clk
#define EIC_DMASK_OFFSET	(0x04)
#define EIC_IEV_OFFSET		(0x14)
#define EIC_IE_OFFSET		(0x18)
#define EIC_RIS_OFFSET		(0x1C)	//RO
#define EIC_MIS_OFFSET		(0x20)	//RO
#define EIC_IC_OFFSET		(0x24)	//WO
#define EIC_TRIG_OFFSET		(0x28)
#define AUX_EIC_MASK_BITS	(x)	((x)& 0xff)

//
#define EIC_CTRL(x)		(0x40 + x*0x4)
static int eic_ctrl_reg[] =
    { EIC_CTRL(0), EIC_CTRL(1), EIC_CTRL(2), EIC_CTRL(3),
	EIC_CTRL(4), EIC_CTRL(5), EIC_CTRL(6), EIC_CTRL(7)
};

#define AUX_FORCE_CLK_DBNC(x)		((x)<<15)
#define AUX_DBNC_EN(x)		((x)<<14)
#define AUX_DBNC_CNT(x)		(((x)<<0) & 0XFFF)

//for sic register map
#define SCI_BASE_ADDR_0	(0x8a001080)	//Phy addr
#define SIC_INT_EN		(0x00)
#define SIC_INT_RAW		(0X04)
#define SIC_INT_MSK		(0X08)
#define SIC_INT_CLR		(0X0C)
#define SIC_INT_POL		(0x10)
#define SIC_INT_MODE		(0x14)

#define AUX_SIC_MASK_BITS(x)	((x)& 0xff)

#define EIC_SOFTRESET_EN_D (BIT_29)
#define EIC_SOFTRESET_EN_A (BIT_6)

#define EIC_APB_EN_A (BIT_3)
#define EIC_APB_RTC_EIC_A (BIT_11)

static struct wake_lock anaeic_wake_lock;

static struct eic_config config_data[] = {
	{EIC_ID_0, (D_DIE | EIC_TYPE | INTER_ID(0)), EIC_BASE_ADDR_D_V},
	{EIC_ID_1, (D_DIE | EIC_TYPE | INTER_ID(1)), EIC_BASE_ADDR_D_V},
	{EIC_ID_2, (D_DIE | EIC_TYPE | INTER_ID(2)), EIC_BASE_ADDR_D_V},
	{EIC_ID_3, (D_DIE | EIC_TYPE | INTER_ID(3)), EIC_BASE_ADDR_D_V},
	{EIC_ID_4, (D_DIE | EIC_TYPE | INTER_ID(4)), EIC_BASE_ADDR_D_V},
	{EIC_ID_5, (D_DIE | EIC_TYPE | INTER_ID(5)), EIC_BASE_ADDR_D_V},
	{EIC_ID_6, (D_DIE | EIC_TYPE | INTER_ID(6)), EIC_BASE_ADDR_D_V},
	{EIC_ID_7, (D_DIE | EIC_TYPE | INTER_ID(7)), EIC_BASE_ADDR_D_V},
	{EIC_ID_8, (A_DIE | EIC_TYPE | INTER_ID(0)), EIC_BASE_ADDR_A_V},
	{EIC_ID_9, (A_DIE | EIC_TYPE | INTER_ID(1)), EIC_BASE_ADDR_A_V},
	{EIC_ID_10, (A_DIE | EIC_TYPE | INTER_ID(2)), EIC_BASE_ADDR_A_V},
	{EIC_ID_11, (A_DIE | EIC_TYPE | INTER_ID(3)), EIC_BASE_ADDR_A_V},
	{EIC_ID_12, (A_DIE | EIC_TYPE | INTER_ID(4)), EIC_BASE_ADDR_A_V},
	{EIC_ID_13, (A_DIE | EIC_TYPE | INTER_ID(5)), EIC_BASE_ADDR_A_V},
	{EIC_ID_14, (A_DIE | EIC_TYPE | INTER_ID(6)), EIC_BASE_ADDR_A_V},
	{EIC_ID_15, (A_DIE | EIC_TYPE | INTER_ID(7)), EIC_BASE_ADDR_A_V},
	{EIC_ID_16, (D_DIE | SIC_TYPE | INTER_ID(0)), SIC_BASE_ADDR_D_V},
	{EIC_ID_17, (D_DIE | SIC_TYPE | INTER_ID(1)), SIC_BASE_ADDR_D_V},
	{EIC_ID_18, (D_DIE | SIC_TYPE | INTER_ID(2)), SIC_BASE_ADDR_D_V},
	{EIC_ID_19, (D_DIE | SIC_TYPE | INTER_ID(3)), SIC_BASE_ADDR_D_V},
	{EIC_ID_20, (D_DIE | SIC_TYPE | INTER_ID(4)), SIC_BASE_ADDR_D_V},
	{EIC_ID_21, (D_DIE | SIC_TYPE | INTER_ID(5)), SIC_BASE_ADDR_D_V},
	{EIC_ID_22, (D_DIE | SIC_TYPE | INTER_ID(6)), SIC_BASE_ADDR_D_V},
	{EIC_ID_23, (D_DIE | SIC_TYPE | INTER_ID(7)), SIC_BASE_ADDR_D_V},
};

static DEFINE_SPINLOCK(eic_lock);

static int __get_config_index(enum EIC_TYPE_E num)
{
	int index = ARRAY_SIZE(config_data) - 1;

	do {
		if (config_data[index].eic_export_id == num)
			break;
	} while ((index--) >= 0);

	return index;
}

//config end

#define EIC_RO (1)
#define EIC_WO (2)
#define EIC_RW (EIC_RO | EIC_WO)
/*
#define DEBUG
*/
#ifdef DEBUG
#define EIC_DBG(fmt...) printk(fmt)
#else
#define EIC_DBG(fmt...)
#endif

//function begin
static unsigned int __eic_read_data(unsigned int config_index);	//called by eic_get_data
static unsigned int eic_get_dmask(unsigned int config_index);	//called by eic_get_data
static void eic_set_dmask(unsigned int config_index, unsigned int value);	//called by eic_get_data
static int eic_get_data(unsigned int config_index);
static int eic_set_iev(unsigned int config_index, unsigned int value);
static int eic_set_interrupt(unsigned int config_index, unsigned int value);
static int eic_get_raw_interrupt(unsigned int config_index);
static int eic_get_masked_interrupt(unsigned int config_index);
static int eic_clear_interrupt(unsigned int config_index);
static int eic_start_trig(unsigned int config_index);	//for the debounce bypass mode, arm can receive int without need of trig plus.
void eic_ctrl(unsigned int config_index, unsigned int isDebounceEnable,
		     unsigned int debounceMs);

static void sic_set_interrupt(unsigned int config_index, unsigned int enable);
static int sic_get_interrupt_masked(unsigned int config_index);
static int sic_interrupt_clear(unsigned int config_index);
static int sic_set_interrupt_pol(unsigned int config_index, unsigned pol);
static int sic_set_interrupt_mode(unsigned int config_index, unsigned int mode);
//function end

static void eic_set(struct eic_config *p, unsigned int value,
		    unsigned int offset)
{
	if ((p->type & A_D_DIE_MASK) == D_DIE) {
		EIC_DBG("eic_set is ddie\n");
		__raw_writel(value, p->base_addr + offset);
	} else {
		EIC_DBG("eic_set is adie, add  = 0x%x, value = 0x%x\n",
			p->base_addr + offset, value);
		ANA_REG_SET(p->base_addr + offset, value);
	}
}

static unsigned int eic_get(struct eic_config *p, unsigned int offset)
{
	if ((p->type & A_D_DIE_MASK) == D_DIE) {
		EIC_DBG("eic_get is ddie\n");
		return __raw_readl(p->base_addr + offset);
	} else {
		EIC_DBG("eic_get is adie, add  = 0x%x\n",
			p->base_addr + offset);
		return ANA_REG_GET(p->base_addr + offset);
	}
}

static unsigned int __eic_read_data(unsigned int config_index)
{
	return eic_get(&config_data[config_index], EIC_DATA_OFFSET);
}

static unsigned int eic_get_dmask(unsigned int config_index)
{
	return eic_get(&config_data[config_index], EIC_DMASK_OFFSET);
}

static void eic_set_dmask(unsigned int config_index, unsigned int value)
{
	eic_set(&config_data[config_index], value, EIC_DMASK_OFFSET);
}

static int eic_get_data(unsigned int config_index)
{
	unsigned int mask_value = 0;
	unsigned int eic_bit = 0;

	eic_bit = GET_INTER_ID(config_data[config_index].type);
	mask_value = eic_get_dmask(config_index);
	if (!((mask_value >> eic_bit) & 0x1)) {
		mask_value |= (0x1 << eic_bit);
		eic_set_dmask(config_index, mask_value);
	}
	return (! !(__eic_read_data(config_index) & (0x1 << eic_bit)));
}

/**
	Write: set 1
	EIC_RO:get the eic status value of the register.
*/
static int eic_aux(unsigned int config_index, unsigned reg_offset, unsigned op,
		   unsigned set_value)
{
	int old = -1;
	unsigned int eic_bit = GET_INTER_ID(config_data[config_index].type);
	unsigned int _value = 0;

	if (op & EIC_RO) {
		_value = eic_get(&config_data[config_index], reg_offset);
		EIC_DBG("%s, _value = 0x%x\n", __FUNCTION__, _value);
		old = ! !(_value & (0x1 << eic_bit));
	}

	if (op & EIC_WO) {
		_value &= ~(1 << eic_bit);
		set_value = _value | ((! !set_value) << eic_bit);
		EIC_DBG("%s,value = 0x%x, offset = 0x%x\n", __FUNCTION__,
			set_value, reg_offset);
		eic_set(&config_data[config_index], set_value, reg_offset);
	}
	return old;
}

/**
	"1" high level trigger interrupt.
	"0" low level trigger interrupt.
*/
static int eic_set_iev(unsigned int config_index, unsigned int value)
{
	return eic_aux(config_index, EIC_IEV_OFFSET, EIC_RW, value);
}

static int eic_set_interrupt(unsigned int config_index, unsigned int value)
{
	return eic_aux(config_index, EIC_IE_OFFSET, EIC_RW, value);
}

static int eic_get_raw_interrupt(unsigned int config_index)
{
	return eic_aux(config_index, EIC_RIS_OFFSET, EIC_RO, 0);
}

static int eic_get_masked_interrupt(unsigned int config_index)
{
	return eic_aux(config_index, EIC_MIS_OFFSET, EIC_RO, 0);
}

static int eic_clear_interrupt(unsigned int config_index)
{
	return eic_aux(config_index, EIC_IC_OFFSET, EIC_WO, 1);
}

static int eic_start_trig(unsigned int config_index)
{
	unsigned int value = 0;
	unsigned int eic_bit = 0;

	eic_bit = GET_INTER_ID(config_data[config_index].type);
	value = eic_get(&config_data[config_index], EIC_IE_OFFSET);
	if (value & (1 << eic_bit))
		return eic_aux(config_index, EIC_TRIG_OFFSET, EIC_WO, 1);
	return (-1);
}

void eic_ctrl(enum EIC_TYPE_E num, unsigned int isDebounceEnable,
		     unsigned int debounceMs)
{
	unsigned int value = 0;
	unsigned int eic_id = 0;
	unsigned int config_index = __get_config_index(num);

	eic_id = GET_INTER_ID(config_data[config_index].type);
	value |= (1 << 15);	//clock of dbnc forced open
	if (debounceMs > 0xfff)
		debounceMs = 0xFFF;
	if (debounceMs < 1)
		debounceMs = 1;
	if (isDebounceEnable)
		value |= (1 << 14);
	value |= debounceMs;
	eic_set(&config_data[config_index], value, eic_ctrl_reg[eic_id]);
}
EXPORT_SYMBOL(eic_ctrl);

static void sic_set_interrupt(unsigned int config_index, unsigned int enable)
{
	eic_aux(config_index, SIC_INT_EN, EIC_RW, enable);
}

static int sic_get_interrupt_masked(unsigned int config_index)
{
	return eic_aux(config_index, SIC_INT_MSK, EIC_RO, 0);	//msk = sicinten & sicintraw
}

static int sic_interrupt_clear(unsigned int config_index)
{
	return eic_aux(config_index, SIC_INT_CLR, EIC_WO, 1);
}

static int sic_set_interrupt_pol(unsigned int config_index, unsigned pol)
{
	return eic_aux(config_index, SIC_INT_POL, EIC_RW, pol);
}

static int sic_set_interrupt_mode(unsigned int config_index, unsigned int mode)
{
	return eic_aux(config_index, SIC_INT_MODE, EIC_RW, mode);
}

struct eic_irq_map {
	enum EIC_TYPE_E eic_id;
	int irq_num;
};

static struct eic_irq_map eic_irq_table[NR_EIC_ALL_IRQS];

static void sprd_ack_eic_irq(unsigned int irq)
{
	enum EIC_TYPE_E eic;
	unsigned int index = 0;
	struct eic_irq_map *map = get_irq_chip_data(irq);

	eic = map->eic_id;
	if (eic <= EIC_ID_MIN || eic >= EIC_ID_MAX)
		goto Err;

	EIC_DBG("ack irq eic %d  irq %d", eic, irq);

	index = __get_config_index(eic);
	if ((config_data[index].type & EIC_SIC_MASK) == EIC_TYPE) {
		eic_clear_interrupt(index);
	} else if ((config_data[index].type & EIC_SIC_MASK) == SIC_TYPE) {
		sic_interrupt_clear(index);
	} else
		goto Err;

	return;
Err:
	pr_warning(" [%s] error eic %d\n", __FUNCTION__, eic);
}

static void sprd_mask_eic_irq(unsigned int irq)
{
	enum EIC_TYPE_E eic;
	unsigned int config_index = 0;
	struct eic_irq_map *map = get_irq_chip_data(irq);

	eic = map->eic_id;
	if (eic <= EIC_ID_MIN || eic >= EIC_ID_MAX)
		goto Err;

	EIC_DBG("mask eic %d  irq %d", eic, irq);

	config_index = __get_config_index(eic);
	if ((config_data[config_index].type & EIC_SIC_MASK) == EIC_TYPE) {
		eic_set_interrupt(config_index, 0);
	} else if ((config_data[config_index].type & EIC_SIC_MASK) == SIC_TYPE) {
		sic_set_interrupt(config_index, 0);
	} else
		goto Err;

	return;
Err:
	pr_warning(" [%s] error eic %d\n", __func__, eic);
}

static void sprd_unmask_eic_irq(unsigned int irq)
{
	enum EIC_TYPE_E eic;
	unsigned int config_index = 0;
	struct eic_irq_map *map = get_irq_chip_data(irq);

	eic = map->eic_id;
	if (eic <= EIC_ID_MIN || eic >= EIC_ID_MAX)
		goto Err;

	config_index = __get_config_index(eic);

	EIC_DBG("unmask eic %d  irq %d, config_index %d", eic, irq,
		config_index);
	if ((config_data[config_index].type & EIC_SIC_MASK) == EIC_TYPE) {
		eic_set_interrupt(config_index, 1);
		eic_start_trig(config_index);	//TODO: the interval of two EIC trigger needs be longer than 2ms
	} else if ((config_data[config_index].type & EIC_SIC_MASK) == SIC_TYPE) {
		sic_set_interrupt(config_index, 1);
	} else
		goto Err;
//TODO:
#ifdef CONFIG_NKERNEL
	sprd_enable_ana_irq();
#endif
	return;
Err:
	pr_warning(" [%s] error eic %d\n", __FUNCTION__, eic);
}

static int sprd_eic_irq_type(unsigned int irq, unsigned int type)
{
	enum EIC_TYPE_E eic;
	struct eic_irq_map *map = get_irq_chip_data(irq);
	unsigned int config_index = 0;
	unsigned long flags;

	eic = map->eic_id;
	if (eic <= EIC_ID_MIN || eic >= EIC_ID_MAX) {
		pr_warning(" [%s] error eic %d\n", __FUNCTION__, eic);
		return -1;
	}

	spin_lock_irqsave(&eic_lock, flags);

	config_index = __get_config_index(eic);
	if ((config_data[config_index].type & EIC_SIC_MASK) == EIC_TYPE) {
		if (IRQ_TYPE_LEVEL_HIGH == type)
			eic_set_iev(config_index, 1);
		else
			eic_set_iev(config_index, 0);
	} else if ((config_data[config_index].type & EIC_SIC_MASK) == SIC_TYPE) {
		if (IRQ_TYPE_LEVEL_HIGH == type)
			sic_set_interrupt_pol(config_index, 1);
		else
			sic_set_interrupt_pol(config_index, 0);
	} else
		pr_warning(" [%s] error eic type %d\n", __FUNCTION__, eic);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__set_irq_handler_unlocked(irq, handle_level_irq);

	spin_unlock_irqrestore(&eic_lock, flags);

	return 0;
}

static void sprd_disable_eic_irq(unsigned int irq)
{
	sprd_mask_eic_irq(irq);
}

static struct irq_chip sprd_muxed_eic_chip = {
	.name = "eic",
	.ack = sprd_ack_eic_irq,
	.mask = sprd_mask_eic_irq,
	.unmask = sprd_unmask_eic_irq,
	.set_type = sprd_eic_irq_type,
	.disable = sprd_disable_eic_irq,
};

static void eic_handler(unsigned int irq, struct irq_desc *desc)
{
	int i;
	int config_index = 0;
	int interrupt_status = 0;

	wake_lock_timeout(&anaeic_wake_lock, 2 * HZ);
	//Debug INT
	printk("eic_handler\n");
#if 0
	//clear INT register: use eic_clear_interrupt() and eic_get_masked_interrupt() function.
	ANA_REG_SET(EIC_BASE_ADDR_A_V + EIC_IC_OFFSET, 0xff);
	ANA_REG_SET(EIC_BASE_ADDR_A_V + EIC_MIS_OFFSET, 0xff);
	__raw_writel(0xff, EIC_BASE_ADDR_D_V + EIC_IC_OFFSET);
#endif

	for (i = 0; i < ARRAY_SIZE(eic_irq_table); ++i) {
		if (eic_irq_table[i].eic_id == EIC_ID_MAX)
			continue;
		config_index = __get_config_index(eic_irq_table[i].eic_id);
		//get the EICMIS register bit value,interrupt active or not.
		if ((config_data[config_index].type & EIC_SIC_MASK) == EIC_TYPE) {
			interrupt_status =
			    eic_get_masked_interrupt(config_index);
		} else if ((config_data[config_index].type & EIC_SIC_MASK) ==
			   SIC_TYPE) {
			interrupt_status =
			    sic_get_interrupt_masked(config_index);
		}
		if (interrupt_status == 1) {
			generic_handle_irq(eic_irq_table[i].irq_num);	//run the irq_desc->handle_irq.
		}
	}
	desc->chip->unmask(irq);
}

extern void (*eic_mux_handler) (unsigned int irq, struct irq_desc * desc);
static void eic_irq_init(void)
{
	int irq;
	int i;

	for (i = 0; i < ARRAY_SIZE(eic_irq_table); ++i)
		eic_irq_table[i].eic_id = EIC_ID_MAX;

	for (irq = EIC_IRQ_START; irq < (EIC_IRQ_START + NR_EIC_ALL_IRQS);
	     ++irq) {
		set_irq_chip(irq, &sprd_muxed_eic_chip);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	//register two common interrupt function for EIC and EICA.
	eic_mux_handler = eic_handler;
	set_irq_chained_handler(IRQ_ANA_EIC_INT, eic_handler);
}

#define EIC_SOFTRESET_EN_D (BIT_29)
#define EIC_SOFTRESET_EN_A (BIT_6)

static void eic_phy_softreset(void)
{
	volatile int i = 0;
	//D-die reset.
	__raw_bits_or(EIC_SOFTRESET_EN_D, GR_SOFT_RST);
	for (i = 0; i < 100; ++i) ;
	__raw_bits_and(~EIC_SOFTRESET_EN_D, GR_SOFT_RST);

	//A-die reset.
	ANA_REG_OR(EIC_SOFTRESET_ADDR_V, EIC_SOFTRESET_EN_A);
	for (i = 0; i < 100; ++i) ;
	ANA_REG_AND(EIC_SOFTRESET_ADDR_V, ~EIC_SOFTRESET_EN_A);
}

#define EIC_APB_EN_A (BIT_3)
#define EIC_APB_RTC_EIC_A (BIT_11)

//add eic_enable().
static void eic_phy_enable(void)
{
	//enable pin and adi.
	__raw_bits_or(GEN0_PIN_EN | GEN0_ADI_EN, GR_GEN0);

	//D-die.
	__raw_bits_or(GEN0_GPIO_RTC_EN, GR_GEN0);
	__raw_bits_or(GEN0_EIC_EN, GR_GEN0);

	//A-die.
	ANA_REG_OR(EIC_BASE_ADDR_V, EIC_APB_RTC_EIC_A);
	ANA_REG_OR(EIC_BASE_ADDR_V, EIC_APB_EN_A);

	eic_phy_softreset();
}

int sprd_get_eic_data(enum EIC_TYPE_E eic_id)
{
	int config_index = __get_config_index(eic_id);
	return eic_get_data(config_index);
}
EXPORT_SYMBOL(sprd_get_eic_data);

/*
	allocate an irq for eic
*/
__must_check int sprd_alloc_eic_irq(enum EIC_TYPE_E eic_id)
{
	int irq;
	int i;
	unsigned long flags;
	unsigned int size_table = ARRAY_SIZE(eic_irq_table);

	for (i = 0; i < size_table; ++i) {
		if (eic_irq_table[i].eic_id == eic_id) {
			pr_warning("irq for eic_%d has been alloc !\n", eic_id);
			goto Err;
		}
	}

	for (i = 0; i < size_table; ++i) {
		if (eic_irq_table[i].eic_id == EIC_ID_MAX)
			break;
	}

	if (i >= size_table)
		goto Err;

	local_irq_save(flags);
	irq = EIC_IRQ_START + i;
	eic_irq_table[i].eic_id = eic_id;
	eic_irq_table[i].irq_num = irq;
#if 1
	printk(KERN_ALERT "eic_irq_table eic_id:%d,irq_num:%d\n", eic_id, irq);
#endif
	set_irq_chip_data(irq, &eic_irq_table[i]);
	local_irq_restore(flags);

	return irq;
Err:
	printk(KERN_ALERT "sprd_alloc_eic_irq, return -1\n");
	return -1;
}

EXPORT_SYMBOL(sprd_alloc_eic_irq);

void sprd_free_eic_irq(int irq)
{
	int i;
	unsigned long flags;

	local_irq_save(flags);
	for (i = 0; i < ARRAY_SIZE(eic_irq_table); ++i) {
		if (eic_irq_table[i].irq_num == irq) {
			set_irq_chip_data(irq, NULL);
			eic_irq_table[i].eic_id = EIC_ID_MAX;
			break;
		}
	}
	local_irq_restore(flags);
}

EXPORT_SYMBOL(sprd_free_eic_irq);

void __init eic_init(void)
{
	eic_phy_enable();
	eic_irq_init();
	wake_lock_init(&anaeic_wake_lock, WAKE_LOCK_SUSPEND, "anaeic_work");
}
