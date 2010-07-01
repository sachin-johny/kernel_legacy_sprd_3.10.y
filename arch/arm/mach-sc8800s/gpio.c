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
	
	if (gpio_id >= GPIO_MAX_PIN_NUM) {
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
	
	if (gpio_id >= GPIO_MAX_PIN_NUM) {
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
	
	if (gpio_id >= GPIO_MAX_PIN_NUM) {
		WARN(1, KERN_WARNING"gpio number is too larger:%d\r\n", gpio_id);
		return -1;
	}

	gpio_pg_base = __get_gpio_page(gpio_id);
	bit_num = gpio_id & 0x0f;
	local_irq_save(flags);
	reg_value =  __raw_readl(gpio_pg_base + GPIO_DMSK);
	__raw_writel(reg_value | (1 << bit_num), (gpio_pg_base + GPIO_DMSK));
	local_irq_restore(flags);
	pr_info("gpio %d data mask is %x\r\n", bit_num,  __raw_readl(gpio_pg_base + GPIO_DMSK));
	__raw_bits_or(1<<bit_num, (gpio_pg_base + GPIO_DMSK));
	pr_info("new data mask is %x\r\n", __raw_readl(gpio_pg_base + GPIO_DMSK));
	return 0;
}

static void sprd_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	unsigned int  bit_num;
	unsigned int gpio_pg_base;
	unsigned gpio_id = offset;
	int reg_value;
	unsigned long flags;
	
	if (gpio_id >= GPIO_MAX_PIN_NUM) {
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

void __init sprd_gpio_init(void)
{
	//enable gpio bank 0~10, that is, all 176 gpio 
	__raw_writel(0x7fff, GR_GEN2);
	gpiochip_add(&sprd_gpio_chip);
}
#if 0
void sprd_ack_gpio_irq(unsigned int irq)
{
	int gpio = irq - IRQ_GPIO(2) + 2;
	GEDR(gpio) = GPIO_bit(gpio);
}
EXPORT_SYMBOL(sprd_ack_gpio_irq);

void sprd_mask_gpio_irq(unsigned int irq)
{
	int gpio = irq - IRQ_GPIO(2) + 2;
	gpio = get_irq_chip_data(irq);
	
	__clear_bit(gpio, GPIO_IRQ_mask);
	GRER(gpio) &= ~GPIO_bit(gpio);
	GFER(gpio) &= ~GPIO_bit(gpio);
}

EXPORT_SYMBOL(sprd_mask_gpio_irq);

static void sprd_unmask_gpio_irq(unsigned int irq)
{
	int gpio = irq - IRQ_GPIO(2) + 2;
	int idx = gpio >> 5;
	__set_bit(gpio, GPIO_IRQ_mask);
	GRER(gpio) = GPIO_IRQ_rising_edge[idx] & GPIO_IRQ_mask[idx];
	GFER(gpio) = GPIO_IRQ_falling_edge[idx] & GPIO_IRQ_mask[idx];
}

EXPORT_SYMBOL(sprd_unmask_muxed_gpio);

static int sprd_gpio_irq_type(unsigned int irq, unsigned int type)
{
	//set irq type
}
static struct irq_chip sprd_muxed_gpio_chip = {
	.name		= "GPIO",
	.ack		= sprd_ack_gpio_irq,
	.mask		= sprd_mask_gpio_irq,
	.unmask		= sprd_unmask_gpio_irq,
	.set_type	= sprd_gpio_irq_type,
};

static void sprd_gpio_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	//if dev id is not null, check if it is struct gpio_deshake
	if (irq_desc->action->dev_id) {
		//get deshaking interval
		struct gpio_deshake *shake = dev_id;
		//ack irq
		//start a hrtimer with the interval
	} else {
		//generic handler_irq
		//ack irq
		//mask irq
		//get the interrupted gpio, and its irq number
		generic_handle_irq();
		//unmask irq
	}
}
static void sprd_gpio_irq_init(void)
{
	//set irq chip from IRQ_GPIO1 to GPIO_MAX_IRQ_NUM
	//if the irq have to be deshaked, install the handle_simple_irq
		set_irq_chip(IRQ_headset, &sprd_muxed_gpio_chip);
		set_irq_handler(IRQ_charger, handle_simple_irq);
	//else install handler according trigger

		set_irq_chained_handler(IRQ_GPIO, sprd_gpio_demux_handler);
}

struct gpio_irq_map {
	unsigned gpio_id;
	struct sprd_gpio_irq_desc gpio_irq_desc;
};

static struct gpio_irq_map gpio_irq_map[GPIO_MAX_IRQ_NUM];

/*
	add a gpio irq into the map,and setup the irq
*/
int sprd_gpio_irq_setup(unsigned gpio_id, struct sprd_gpio_irq_desc * gpio_desc)
{
	//add the gpio and  irq into the map

	//if the map is full, reture fail

	//set the gpio num to the irq's private data
	set_irq_chip_data(irq, &gpio_irq_map[].gpio_id);
}

EXPORT_SYMBOL(sprd_gpio_irq_add);


//implement in the platform drivers
static __init int sprd_gpio_init_irqs(void)
{
	//request the gpio
	//set gpio's direction interrupt sense
	//add the gpio irqs, e.g. headset, charger...
}
#endif
