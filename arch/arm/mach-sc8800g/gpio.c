/*
 *  linux/arch/arm/mach-sprd/gpio.c
 *
 *  Generic SPRD GPIO handling
 *
 *  Author:	Yingchun Li(yingchun.li@spreadtrum.com)
 *  Created:	March 10, 2010
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
#include <asm/gpio.h>

#include <mach/regs_global.h>
#include <mach/regs_gpio.h>
#include <mach/regs_ana.h>
#include "gpio_phy.h"

#ifndef GPO_TRI
#define GPO_TRI            0xFFFF
#endif

#ifndef GPO_DATA
#define GPO_DATA           0xFFFF
#endif


struct gpio_info{
    enum gpio_section_type gpio_type;
    u32 baseAddr;
    u16 bit_num;
};


void GPIO_PHY_GetBaseInfo (u32 gpio_id, struct gpio_info *pGpio_info)
{
    int i = 0;
    u32 table_size = 0;

    GPIO_SECTION_T  *p_gpio_section_table = (GPIO_SECTION_T *) Gpio_GetCfgSectionTable (&table_size);

    BUG_ON(!(gpio_id < GPIO_MAX_PIN_NUM && table_size > 0));


    pGpio_info->baseAddr = GpioCfg_GetBaseAddr (gpio_id);
    pGpio_info->bit_num  = GpioCfg_GetBitNum (gpio_id);

    for (i = 0; i < table_size; ++i)
    {
        if (p_gpio_section_table[i].gpxx_pagex_base == pGpio_info->baseAddr)
        {
            if (p_gpio_section_table[i].gpxx_pagex_size > pGpio_info->bit_num)
            {
                pGpio_info->gpio_type = p_gpio_section_table[i].gpxx_section_type;
                return;
            }

            break;
        }
    }

    pGpio_info->gpio_type = GPIO_SECTION_INVALID;

    return;
}


 void GPIO_PHY_SetDirection (struct gpio_info *pGpio_info, int directions)
{
    int value = (directions ? true : false);
    u32 reg_addr = 0;
	
    reg_addr = pGpio_info->baseAddr;

    switch (pGpio_info->gpio_type)
    {
        case GPIO_SECTION_GPI:

            if (directions)
            {
                pr_err("[GPIO_DRV]GPIO_SetDirection error");
                WARN_ON(1);
            }

            return;

        case GPIO_SECTION_GPO:

            if (!directions)
            {
                pr_err("[GPIO_DRV]GPIO_SetDirection error");
                WARN_ON(1);
            }

            return;

        case GPIO_SECTION_GPIO:
            reg_addr += GPIO_DIR;
            break;
        case GPIO_SECTION_INVALID:
            pr_err("[GPIO_DRV]the GPIO_ID is Invalid in this chip");
            BUG_ON(1);
            return;

        default:
             BUG_ON(1);
            break;
    }

    GPIO_REG_SET (reg_addr, ( (GPIO_REG32 (reg_addr) & ~ (1<<pGpio_info->bit_num)) | (value<<pGpio_info->bit_num)));

}


static int sprd_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	unsigned gpio_id = offset;
	struct gpio_info gpio_info;
	
       GPIO_PHY_GetBaseInfo (gpio_id, &gpio_info);
	GPIO_PHY_SetDirection(&gpio_info, 0);
	return 0;
}

static int sprd_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	unsigned gpio_id = offset;
	struct gpio_info gpio_info;

	 GPIO_PHY_GetBaseInfo (gpio_id, &gpio_info);
	GPIO_PHY_SetDirection(&gpio_info, 1);
	return 0;
}

static int _GPIO_GetGpioDataRegAddr (struct gpio_info *pGpio_info, u32 *pOffsetAddr)
{
    switch (pGpio_info->gpio_type)
    {
        case GPIO_SECTION_GPI:
            *pOffsetAddr = GPI_DATA;
            break;
        case GPIO_SECTION_GPO:
            *pOffsetAddr = GPO_DATA;
            break;
        case GPIO_SECTION_GPIO:
            *pOffsetAddr = GPIO_DATA;
            break;
        case GPIO_SECTION_INVALID:
	 default:
            pr_err("[GPIO_DRV]the GPIO_ID is Invalid in this chip");
            BUG_ON(1);
            return false;
      }
    return true;
}

int GPIO_PHY_GetPinData (struct gpio_info *pGpio_info)
{
    u32 offsetAddr = 0;
    u32 reg_addr = 0;

    reg_addr = pGpio_info->baseAddr;

    if (_GPIO_GetGpioDataRegAddr (pGpio_info, &offsetAddr))
    {
        reg_addr += offsetAddr;
        return ( (GPIO_REG32 (reg_addr) & (1<<pGpio_info->bit_num)) ? true : false);
    }

    return false;
}
int _GPIO_GetGpioDataMaskRegAddr (struct gpio_info *pGpio_info, u32 *pOffsetAddr)
{
    switch (pGpio_info->gpio_type)
    {
        case GPIO_SECTION_GPI:
            *pOffsetAddr = GPI_DMSK;
            break;
        case GPIO_SECTION_GPO:
            *pOffsetAddr = GPO_TRI;
            break;
        case GPIO_SECTION_GPIO:
            *pOffsetAddr = GPIO_DMSK;
            break;
        case GPIO_SECTION_INVALID:
            pr_err("[GPIO_DRV]the GPIO_ID is Invalid in this chip");
            BUG_ON(1);
            return false;
        default:
            pr_err ("[GPIO_DRV]the GPIO_ID is Invalid in this chip");
            BUG_ON(1);
            return false; /*lint !e527 comfirmed by xuepeng*/
    }

    return true;
}

int GPIO_PHY_GetDataMask (struct gpio_info *pGpio_info)
{
    u32 offsetAddr = 0;
    u32 reg_addr = 0;

    reg_addr = pGpio_info->baseAddr;

    if (_GPIO_GetGpioDataMaskRegAddr (pGpio_info, &offsetAddr))
    {
        reg_addr += offsetAddr;

	//pr_info("gpio_addr %x data mask :%x\r\n", pGpio_info->baseAddr, __raw_readl(reg_addr));
        return ( (GPIO_REG32 (reg_addr) & (1<<pGpio_info->bit_num)) ? true : false);
    }

    return false;
}

int GPIO_PHY_GetDirection (struct gpio_info *pGpio_info)
{
    u32 reg_addr = 0;
    reg_addr = pGpio_info->baseAddr;

    switch (pGpio_info->gpio_type)
    {
        case GPIO_SECTION_GPI:
            return false;

        case GPIO_SECTION_GPO:
            return true;

        case GPIO_SECTION_GPIO:
            reg_addr += GPIO_DIR;
            break;

        case GPIO_SECTION_INVALID:
            pr_info("[GPIO_DRV]the GPIO_ID is Invalid in this chip");
            BUG_ON(1);
            return false;

        default:
            BUG_ON(1);
            break;
    }

    return ( (GPIO_REG32 (reg_addr) & (1<<pGpio_info->bit_num)) ? true : false);
}

void GPIO_PHY_SetPinData (struct gpio_info *pGpio_info ,int b_on)
{
    u32 offsetAddr = 0;
    u32 reg_addr = 0;
    int value = (b_on ? true : false);

    reg_addr = pGpio_info->baseAddr;

    _GPIO_GetGpioDataRegAddr (pGpio_info, &offsetAddr);
    reg_addr += offsetAddr;

    GPIO_REG_SET (reg_addr, ( (GPIO_REG32 (reg_addr) & ~ (1<<pGpio_info->bit_num)) |
                              (value<<pGpio_info->bit_num)));

}

/*
 * Return GPIO level
 */
static int sprd_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	u32 bit_num;
	u32 gpio_pg_base;
	u32 value;
	unsigned gpio_id = offset;

	 struct gpio_info gpio_info;
    GPIO_PHY_GetBaseInfo (gpio_id, &gpio_info);

    if (!GPIO_PHY_GetDataMask (&gpio_info))
    {
        WARN(1, "[GPIO_DRV]GPIO_GetValue: GPIO_%d data mask hasn't been opened!\n", gpio_id);
    }

    if (GPIO_PHY_GetDirection (&gpio_info))
    {
        WARN(1, "[GPIO_DRV]GPIO_GetValue: GPIO_%d should be input port!\n", gpio_id);
    }

    return GPIO_PHY_GetPinData (&gpio_info);
	
}


/*
 * Set output GPIO level
 */
static void sprd_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_info gpio_info;
	u32 gpio_id = offset;
	
    BUG_ON (gpio_id >= GPIO_MAX_PIN_NUM);

    GPIO_PHY_GetBaseInfo (gpio_id, &gpio_info);

    if (!GPIO_PHY_GetDataMask (&gpio_info))
    {
        WARN(1, "[GPIO_DRV]GPIO_%d data mask no opened!",  gpio_id);
    }

    if (!GPIO_PHY_GetDirection (&gpio_info))
    {
        WARN(1, "[GPIO_DRV]GPIO_%d dir wrong!", gpio_id);
    }

    GPIO_PHY_SetPinData (&gpio_info, value);
/*
	pr_info("gpio_%d setting :%x \n", gpio_id,\
		__raw_readl(gpio_pg_base + GPIO_DATA));
*/
}

void GPIO_PHY_SetDataMask (struct gpio_info *pGpio_info, int b_on)
{
    int value = (b_on ? true : false);
    u32 reg_addr = 0;
    u32 offsetAddr = 0;

    reg_addr = pGpio_info->baseAddr;

    if (_GPIO_GetGpioDataMaskRegAddr (pGpio_info, &offsetAddr))
    {
        reg_addr += offsetAddr;
        GPIO_REG_SET ( (reg_addr), ( (GPIO_REG32 (reg_addr) & ~ (1<<pGpio_info->bit_num)) |
                                     (value<<pGpio_info->bit_num)));
	//pr_info("After setting gpio_addr %x data mask :%x\r\n", reg_addr, 
	//	__raw_readl(reg_addr));
    }

    return;
}

void GPIO_Enable (u32 gpio_id)
{
    struct gpio_info gpio_info;
    GPIO_PHY_GetBaseInfo (gpio_id, &gpio_info);

	//pr_info("gpio info, pin is :%d, base addr :%p, bit num :%d, type :%d\r\n", gpio_id, 
		//gpio_info.baseAddr, gpio_info.bit_num, gpio_info.gpio_type);
    GPIO_PHY_SetDataMask (&gpio_info, true);
}


void GPIO_Disable (u32 gpio_id)
{
    struct gpio_info gpio_info;
    GPIO_PHY_GetBaseInfo (gpio_id, &gpio_info);

    GPIO_PHY_SetDataMask (&gpio_info, false);
}

static int sprd_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	//this enable the gpio
	unsigned long flags;
	u32 bit_num;
	u32 gpio_pg_base;
	unsigned gpio_id = offset;
	int reg_value;
	
	GPIO_Enable(gpio_id);
	//pr_info("new data mask is %x\r\n", __raw_readl(gpio_pg_base + GPIO_DMSK));
	return 0;
}

static void sprd_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	unsigned int  bit_num;
	unsigned int gpio_pg_base;
	unsigned gpio_id = offset;
	int reg_value;
	unsigned long flags;
	
	if (gpio_id > GPIO_MAX_PIN_NUM) {
		WARN(1, KERN_WARNING"gpio number is too larger:%d\r\n", gpio_id);
		return;
	}

	
	GPIO_Disable(gpio_id);
	return;
}
static int sprd_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	//get the irq for the gpio.
	return 0;
}

static struct gpio_chip sprd_gpio_chip = {
	.label		  = "sc8800s-gpio",
	.direction_input  = sprd_gpio_direction_input,
	.direction_output = sprd_gpio_direction_output,
	.get		  = sprd_gpio_get,
	.set		  = sprd_gpio_set,
	.request 	  = sprd_gpio_request,
	.free		  = sprd_gpio_free,
	.to_irq	= sprd_gpio_to_irq,
	.base		  = 0,	
	.ngpio		  = GPIO_MAX_PIN_NUM,
};

struct gpio_irq_map {
	int gpio_id;
	unsigned int irq_num;
};

#define GPIO_INVALID_ID 0xffff
#define GPIO_MAX_IRQ_NUM (NR_GPIO_IRQS)
static struct gpio_irq_map gpio_irq_table[GPIO_MAX_IRQ_NUM];

static u32  __get_gpio_page(unsigned gpio_id)
{
	u32 page;
	u32  gpio_pg_base;

	page = gpio_id >>4;
	gpio_pg_base = page * 0x80 + GPIO_PG0_BASE;
	return gpio_pg_base;
}

void sprd_ack_gpio_irq(unsigned int irq)
{
	int gpio;
	int bit_num;
	unsigned int gpio_page;
 	struct gpio_irq_map *map= get_irq_chip_data(irq);

	gpio = map->gpio_id;
	if (gpio >= GPIO_MAX_PIN_NUM ) {
		pr_warning(" [%s] error gpio %d\n", __FUNCTION__, gpio);
		return;
	}
	//pr_info("ack gpio %d  irq %d", gpio, irq);
    	gpio_page = __get_gpio_page(gpio);
	bit_num = gpio & 0x0f;
	//Interrupt clear, "1" clears edge detection interrupt. "0" has no effect.
	__raw_bits_or((1 << bit_num), (gpio_page + GPIO_IC));
}
EXPORT_SYMBOL(sprd_ack_gpio_irq);

void sprd_mask_gpio_irq(unsigned int irq)
{
	int gpio;
	int bit_num;
	unsigned int gpio_page;
	struct gpio_irq_map *map= get_irq_chip_data(irq);

	gpio = map->gpio_id;
	if (gpio >= GPIO_MAX_PIN_NUM ) {
		pr_warning(" [%s] error gpio %d\n", __FUNCTION__, gpio);
		return;
	}
	//pr_info("mask gpio %d  irq %d", gpio, irq);
  	gpio_page = __get_gpio_page(gpio);
	bit_num = gpio & 0x0f;
	//Clear Interrupt mask register
	__raw_bits_and( ~(1 << bit_num), (gpio_page + GPIO_IE));
}

EXPORT_SYMBOL(sprd_mask_gpio_irq);

void sprd_unmask_gpio_irq(unsigned int irq)
{
	int gpio;
	int bit_num;
	unsigned int gpio_page;
	struct gpio_irq_map *map= get_irq_chip_data(irq);

	gpio = map->gpio_id;
	if (gpio >= GPIO_MAX_PIN_NUM ) {
		pr_warning(" [%s] error gpio %d\n", __FUNCTION__, gpio);
		return;
	}
	//pr_info("unmask gpio %d  irq %d", gpio, irq);
  	gpio_page = __get_gpio_page(gpio);
	bit_num = gpio & 0x0f;
	//Set Interrupt mask register
   	__raw_bits_or( (1 << bit_num), (gpio_page + GPIO_IE));
}


static int sprd_gpio_irq_type(unsigned int irq, unsigned int type)
{
	int gpio;
	int bit_num;
	unsigned int gpio_pg_base;
	struct gpio_irq_map *map= get_irq_chip_data(irq);

	gpio = map->gpio_id;
	if (gpio >= GPIO_MAX_PIN_NUM ) {
		pr_warning(" [%s] error gpio %d\n", __FUNCTION__, gpio);
		return -1;
	}
  	gpio_pg_base = __get_gpio_page(gpio);
	bit_num = gpio & 0x0f;

	//set irq type
	switch(type) {
	case IRQ_TYPE_LEVEL_HIGH: 
		__raw_bits_or(0x1 << bit_num, (gpio_pg_base + GPIO_IS));
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IBE));
		__raw_bits_or(0x1 << bit_num, (gpio_pg_base + GPIO_IEV));
		break;

	case IRQ_TYPE_LEVEL_LOW:
		__raw_bits_or(0x1 << bit_num, (gpio_pg_base + GPIO_IS));
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IBE));
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IEV));
		break;

	case IRQ_TYPE_EDGE_BOTH:
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IS));
		__raw_bits_or(0x1 << bit_num, (gpio_pg_base + GPIO_IBE));
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IEV));
		break;

	case IRQ_TYPE_EDGE_RISING:
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IS));
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IBE));
		__raw_bits_or(0x1 << bit_num, (gpio_pg_base + GPIO_IEV));
		break;

	case IRQ_TYPE_EDGE_FALLING:
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IS));
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IBE));
		__raw_bits_and(~(0x1 << bit_num), (gpio_pg_base + GPIO_IEV));
		break;

	default:
		pr_warning("error irq type %d\n", type);
		return -1;
	}

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__set_irq_handler_unlocked(irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__set_irq_handler_unlocked(irq, handle_edge_irq);
	return 0;
}

static void sprd_disable_gpio_irq(unsigned int irq)
{
	sprd_mask_gpio_irq(irq);	
}
static struct irq_chip sprd_muxed_gpio_chip = {
	.name		= "GPIO",
	.ack		= sprd_ack_gpio_irq,
	.mask		= sprd_mask_gpio_irq,
	.unmask		= sprd_unmask_gpio_irq,
	.set_type	= sprd_gpio_irq_type,
	.disable	= sprd_disable_gpio_irq,
};

static  int __get_gpio_int_state(int gpio)
{
	int bit_num;
	unsigned int gpio_pg_base;
	unsigned int value;
    
  	gpio_pg_base = __get_gpio_page(gpio);
	bit_num = gpio & 0x0f;
	value = __raw_readl(gpio_pg_base + GPIO_MIS);

	if (value & (1<<bit_num)) 
		return 1;
	else
       	return 0;
}


static void sprd_gpio_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	int i;

	for (i = 0; i < GPIO_MAX_IRQ_NUM; i++) {
		if (gpio_irq_table[i].gpio_id == GPIO_INVALID_ID) {
		            continue;
        	}
		if (__get_gpio_int_state(gpio_irq_table[i].gpio_id)) {
		  	generic_handle_irq(gpio_irq_table[i].irq_num);
		}
	}
}
static void sprd_gpio_irq_init(void)
{
	int irq;
	int i;

	for(i = 0; i < GPIO_MAX_IRQ_NUM; i++) {
		gpio_irq_table[i].gpio_id= GPIO_INVALID_ID;
	}
	for (irq = GPIO_IRQ_START; irq < (GPIO_IRQ_START + NR_GPIO_IRQS); irq++) {
		set_irq_chip(irq, &sprd_muxed_gpio_chip);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}
	
	set_irq_chained_handler(IRQ_GPIO_INT, sprd_gpio_demux_handler);
}

/*
	add a gpio irq into the map,and setup the irq
*/
__must_check int sprd_gpio_irq_register(int gpio_id, unsigned int irq)
{
	int i;

	   // find a free record
	for (i = 0; i< GPIO_MAX_IRQ_NUM; i++) {
		if (gpio_irq_table[i].gpio_id == gpio_id) {
			pr_warning("GPIO_AddToIntTable: GPIO_%d has been added !\n", gpio_id); 
		    return -1;
		}
	}

	for(i = 0; i < GPIO_MAX_IRQ_NUM; i++) {
		if (gpio_irq_table[i].gpio_id == GPIO_INVALID_ID) 
			break;
	}

	if (i >= GPIO_MAX_IRQ_NUM) {
		// No free item in the table.
		return -1;
	}
	 gpio_irq_table[i].gpio_id = gpio_id;
	 gpio_irq_table[i].irq_num = irq;
	 
	set_irq_chip_data(irq, &gpio_irq_table[i]);
	return 0;
}

EXPORT_SYMBOL(sprd_gpio_irq_register);

__init void sprd_gpio_init(void)
{
	//enable gpio bank 0~10, that is, all 176 gpio 
	//__raw_writel(0x7fff, GR_GEN2);
	  //CHIP_REG_OR ( (GR_GEN0), (GEN0_GPIO_EN | GEN0_GPIO_RTC_EN));
	  __raw_bits_or (GEN0_GPIO_EN | GEN0_GPIO_RTC_EN, GR_GEN0);
    ANA_REG_OR (ANA_AGEN,AGEN_RTC_GPIO_EN);
    msleep(5);
    ANA_REG_OR (ANA_AGEN,AGEN_GPIO_EN);
	
	gpiochip_add(&sprd_gpio_chip);

	sprd_gpio_irq_init();
}
