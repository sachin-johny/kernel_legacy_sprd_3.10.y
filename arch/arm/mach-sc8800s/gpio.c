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
#include <asm/io.h>
#include <asm/gpio.h>

#include <mach/regs_global.h>
#include <mach/regs_gpio.h>


static u32  __get_gpio_page(unsigned gpio_id)
{
	u32 page;
	u32  gpio_pg_base;

	page = gpio_id >>4;
	gpio_pg_base = page * 0x80 + GPIO_PG0_BASE;
	return gpio_pg_base;
}

/*
	get the gpio direction
	0, INPUT
	none zero, OUTPUT
*/
static u32 __get_gpio_dir(unsigned gpio_id)
{
	u32 bit_num;
	u32 gpio_pg_base;

	gpio_pg_base = __get_gpio_page(gpio_id);
	bit_num = gpio_id & 0x0f;
	return (__raw_readl(gpio_pg_base + GPIO_DIR) & (1<<bit_num));
}
static int sprd_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	unsigned long        flags;
	u32 bit_num;
	u32 gpio_pg_base;
	u32 dir;
	unsigned gpio_id = offset;
	
	if (gpio_id > GPIO_MAX_PIN_NUM) {
		WARN(1, KERN_WARNING"gpio number is too larger:%d\r\n", gpio_id);
		return -1;
	}

	gpio_pg_base = __get_gpio_page(gpio_id);
	bit_num = gpio_id & 0x0f;
	local_irq_save(flags);
	dir = __raw_readl(gpio_pg_base + GPIO_DIR);
	__raw_writel(dir &  ~(1<<bit_num), (gpio_pg_base + GPIO_DIR));
	local_irq_restore(flags);
	
	return 0;
}

static int sprd_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	unsigned long        flags;
	u32 bit_num;
	u32 gpio_pg_base;
	u32 reg_value;
	unsigned gpio_id = offset;
	
	if (gpio_id > GPIO_MAX_PIN_NUM) {
		WARN(1, KERN_WARNING"gpio number is too larger:%d\r\n", gpio_id);
		return -1;
	}

	gpio_pg_base = __get_gpio_page(gpio_id);
	pr_warning("gpio_%d base: %x output :%d!\n", gpio_id, gpio_pg_base, value);
	bit_num = gpio_id & 0x0f;
	local_irq_save(flags);
	//set direction
	reg_value = __raw_readl(gpio_pg_base + GPIO_DIR);
	__raw_writel(reg_value |(1<<bit_num), (gpio_pg_base + GPIO_DIR));

	//set value;
	reg_value = __raw_readl(gpio_pg_base + GPIO_DATA);
	if (value) {
		reg_value = reg_value | (1<<bit_num);
	} else {
		reg_value = reg_value & ~(1<<bit_num);
	}
	__raw_writel(reg_value, gpio_pg_base + GPIO_DATA);
	
	local_irq_restore(flags);

	return 0;
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

	gpio_pg_base = __get_gpio_page(gpio_id);
	bit_num = gpio_id & 0x0f;
	value = __raw_readl(gpio_pg_base + GPIO_DATA);
	
	return value & (1<<bit_num);
}

/*
 * Set output GPIO level
 */
static void sprd_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	unsigned long flags;
	u32 bit_num;
	u32 gpio_pg_base;
	unsigned gpio_id = offset;
	int reg_value;

	gpio_pg_base = __get_gpio_page(gpio_id);
	
	pr_info("gpio_%d base :%x \n", gpio_id, (int)gpio_pg_base);
	if(!__get_gpio_dir(gpio_id))
	{
		pr_warning("gpio_%d should be output port!\n", gpio_id);
		WARN(1, "gpio_%d dir wrong!DataReg:0x%x,DataMask:0x%x,DirReg:0x%x\n", 
		    gpio_id, 
		    __raw_readl(gpio_pg_base + GPIO_DATA),
		    __raw_readl(gpio_pg_base + GPIO_DMSK), 
		    __raw_readl(gpio_pg_base + GPIO_DIR) 
		    );
	}
	
	bit_num = gpio_id & 0x0f;

	local_irq_save(flags);
	reg_value = __raw_readl(gpio_pg_base + GPIO_DATA);
	if (value) {
		reg_value = reg_value | (1<<bit_num);
	} else {
		reg_value = reg_value & ~(1<<bit_num);
	}
	__raw_writel(reg_value, gpio_pg_base + GPIO_DATA);
	local_irq_restore(flags);
	pr_info("gpio_%d setting :%x \n", gpio_id,\
		__raw_readl(gpio_pg_base + GPIO_DATA));
}

static int sprd_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	//this enable the gpio
	unsigned long flags;
	u32 bit_num;
	u32 gpio_pg_base;
	unsigned gpio_id = offset;
	int reg_value;
	
	gpio_pg_base = __get_gpio_page(gpio_id);
	bit_num = gpio_id & 0x0f;
	local_irq_save(flags);
	reg_value =  __raw_readl(gpio_pg_base + GPIO_DMSK);
	__raw_writel(reg_value | (1 << bit_num), (gpio_pg_base + GPIO_DMSK));
	local_irq_restore(flags);
	//pr_info("gpio %d data mask is %x\r\n", bit_num,  __raw_readl(gpio_pg_base + GPIO_DMSK));
	__raw_bits_or(1<<bit_num, (gpio_pg_base + GPIO_DMSK));
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

	gpio_pg_base = __get_gpio_page(gpio_id);
	bit_num = gpio_id & 0x0f;

	local_irq_save(flags);
	reg_value =  __raw_readl(gpio_pg_base + GPIO_DMSK);
	__raw_writel(reg_value & ~(1 << bit_num), (gpio_pg_base + GPIO_DMSK));
	local_irq_restore(flags);
	
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
		return;
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
	__raw_writel(0x7fff, GR_GEN2);
	gpiochip_add(&sprd_gpio_chip);

	sprd_gpio_irq_init();
}
