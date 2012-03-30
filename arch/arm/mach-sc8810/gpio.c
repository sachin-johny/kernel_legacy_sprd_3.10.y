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

/* #define DEBUG */
#define debug(format, arg...) pr_info("gpio: " "+++" format, ## arg)

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <asm/gpio.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/adi.h>
#include <mach/gpio.h>
#include <mach/ctl_gpio.h>
#include <mach/ctl_eic.h>

/*
 * SC8810 GPIO bank and number summary:
 *
 * Bank	From	To		GPIOs	Style
 * 1	0~		15		16		EIC
 * 2	16~		159		144		GPIO
 * 3	160~	175		16		ANA EIC
 * 4	176~	207		32		ANA GPIO
 *								SIC
 */

#define CTL_GPIO_BASE					( SPRD_GPIO_BASE )
#define ANA_CTL_EIC_BASE				( SPRD_MISC_BASE + 0x0700)
#define ANA_CTL_GPIO_BASE				( SPRD_MISC_BASE + 0x0480 )

typedef struct {
	u32 reg, val;
} gpiomap_t;

struct sci_gpio_chip {
	struct gpio_chip chip;
	u32 base_addr;
	int adie;
};

#define GPIO_INFO(_v_)					( ((struct sci_gpio_chip *)chip)->_v_ )

//16 gpios for one address
#define GPIO_OFF2REG(_b_)				( (_b_) + ((offset >> 4) << 7) )
#define GPIO_OFF2REG2(_b_)				( GPIO_OFF2REG(_b_) - CTL_GPIO_BASE + base_addr )
#define GPIO_OFF2REG3(_b_)				( GPIO_OFF2REG(_b_) - CTL_EIC_BASE + base_addr )
#define GPIO_OFF2SHIFT					( offset & 0x0f )

//16 ana gpios for one address
#define GPIO_OFF2ANAREG(_b_)			( (_b_) + ((offset >> 4) << 6) )

//#define CONFIG_SUPPORT_GPIO_LOCK      0

int sci_gpio_enable(void)
{
	//BUGBUG: enable D-Die and A-Die gpio and eic controller and enable clock rtc/div5 for eic
	return 0;
}

int sci_gpio_disable(void)
{
	return 0;
}

#if defined(CONFIG_SUPPORT_GPIO_LOCK)
int sci_gpio_lock(void)
{
	return 0;
}

int sci_gpio_unlock(void)
{
	return 0;
}

#else
#define sci_gpio_lock()					\
		unsigned long flags;			\
		local_irq_save(flags)			\

#define sci_gpio_unlock() local_irq_restore(flags)

#endif

int sci_gpio_read(struct gpio_chip *chip, unsigned offset, u32 reg)
{
	u32 base_addr = GPIO_INFO(base_addr);
	int adie = GPIO_INFO(adie);
	u32 addr = GPIO_OFF2REG2(reg);
	return SCI_REG_GET(addr);
}

int sci_gpio_write(struct gpio_chip *chip, unsigned offset, u32 reg, int value)
{
	u32 base_addr = GPIO_INFO(base_addr);
	int adie = GPIO_INFO(adie);
	u32 addr = GPIO_OFF2REG2(reg);
	sci_gpio_lock();
	(value) ? SCI_REG_SET(addr, 1 << GPIO_OFF2SHIFT) :
	    SCI_REG_CLR(addr, 1 << GPIO_OFF2SHIFT);
	sci_gpio_unlock();
	return 0;
}

int sci_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	debug("%s %d\n", __FUNCTION__, offset);
	return sci_gpio_write(chip, offset, REG_GPIO_DMSK, 1);
}

void sci_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	debug("%s %d\n", __FUNCTION__, offset);
	sci_gpio_write(chip, offset, REG_GPIO_DMSK, 0);
}

int sci_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	sci_gpio_write(chip, offset, REG_GPIO_DIR, 0);
	debug("%s %d\n", __FUNCTION__, offset);
	return sci_gpio_write(chip, offset, REG_GPIO_INEN, 1);
}

static int sci_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
				     int value)
{
	sci_gpio_write(chip, offset, REG_GPIO_DIR, 1);
	sci_gpio_write(chip, offset, REG_GPIO_INEN, 0);
	debug("%s %d %d\n", __FUNCTION__, offset, value);
	return sci_gpio_write(chip, offset, REG_GPIO_DATA, value);
}

int sci_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	int adie = GPIO_INFO(adie);
	WARN(!
	     (sci_gpio_read(chip, offset, REG_GPIO_DMSK) &
	      (1 << GPIO_OFF2SHIFT)), "%sGPIO%u data mask hasn't been opened\n",
	     (adie) ? "ANA " : "", chip->base + offset);
	debug("%s %d\n", __FUNCTION__, offset);
	return ! !(sci_gpio_read(chip, offset, REG_GPIO_DATA) &
		   (1 >> GPIO_OFF2SHIFT));
}

static void sci_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	int adie = GPIO_INFO(adie);
	sci_gpio_lock();
	WARN(!
	     (sci_gpio_read(chip, offset, REG_GPIO_DIR) &
	      (1 << GPIO_OFF2SHIFT)),
	     "%sGPIO%u dir wrong!can't set input value\n", (adie) ? "ANA " : "",
	     chip->base + offset);
	sci_gpio_write(chip, offset, REG_GPIO_DATA, value);
	sci_gpio_unlock();
}

static int sci_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
				 unsigned debounce)
{
	WARN(1, "GPIO%u not support hw debounce\n", chip->base + offset);
	return 0;
}

int sci_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return chip->base + offset + GPIO_IRQ_START;
}

int sci_irq_to_gpio(struct gpio_chip *chip, unsigned irq)
{
	return irq - GPIO_IRQ_START - chip->base;
}

static struct sci_gpio_chip sc8810_gpio_chip = {
	.chip.label = "sprd-gpio",
	.chip.request = sci_gpio_request,
	.chip.free = sci_gpio_free,
	.chip.direction_input = sci_gpio_direction_input,
	.chip.get = sci_gpio_get,
	.chip.direction_output = sci_gpio_direction_output,
	.chip.set = sci_gpio_set,
	.chip.set_debounce = sci_gpio_set_debounce,
	.chip.to_irq = sci_gpio_to_irq,
	.chip.base = 16,
	.chip.ngpio = 144,
	.base_addr = CTL_GPIO_BASE,
	.adie = 0,
};

static struct sci_gpio_chip sc8810_ana_gpio_chip = {
	.chip.label = "sprd-ana-gpio",
	.chip.request = sci_gpio_request,
	.chip.free = sci_gpio_free,
	.chip.direction_input = sci_gpio_direction_input,
	.chip.get = sci_gpio_get,
	.chip.direction_output = sci_gpio_direction_output,
	.chip.set = sci_gpio_set,
	.chip.set_debounce = sci_gpio_set_debounce,
	.chip.to_irq = sci_gpio_to_irq,
	.chip.base = 176,
	.chip.ngpio = 32,
	.base_addr = ANA_CTL_GPIO_BASE,
	.adie = 1,
};

////////////////////////////////////////////////////////////
void sci_gpio_irq_enable(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sci_irq_to_gpio(chip, data->irq);
	debug("%s %d\n", __FUNCTION__, offset);
	sci_gpio_write(chip, offset, REG_GPIO_IE, 1);
}

void sci_gpio_irq_disable(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sci_irq_to_gpio(chip, data->irq);
	debug("%s %d\n", __FUNCTION__, offset);
	sci_gpio_write(chip, offset, REG_GPIO_IE, 0);
}

void sci_gpio_irq_ack(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sci_irq_to_gpio(chip, data->irq);
	debug("%s %d\n", __FUNCTION__, offset);
	sci_gpio_write(chip, offset, REG_GPIO_IC, 1);
}

void sci_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sci_irq_to_gpio(chip, data->irq);
	debug("%s %d\n", __FUNCTION__, offset);
	sci_gpio_write(chip, offset, REG_GPIO_MIS, 0);
}

void sci_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	int offset = sci_irq_to_gpio(chip, data->irq);
	debug("%s %d\n", __FUNCTION__, offset);
	sci_gpio_write(chip, offset, REG_GPIO_MIS, 1);
}

int sci_gpio_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	int offset;
	struct gpio_chip *chip = (struct gpio_chip *)data->chip_data;
	sci_gpio_lock();
	offset = sci_irq_to_gpio(chip, data->irq);
	debug("%s %d %d\n", __FUNCTION__, offset, flow_type);
	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		sci_gpio_write(chip, offset, REG_GPIO_IS, 0);
		sci_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sci_gpio_write(chip, offset, REG_GPIO_IEV, 1);
		__irq_set_handler_locked(data->irq, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		sci_gpio_write(chip, offset, REG_GPIO_IS, 0);
		sci_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sci_gpio_write(chip, offset, REG_GPIO_IEV, 0);
		__irq_set_handler_locked(data->irq, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		sci_gpio_write(chip, offset, REG_GPIO_IS, 0);
		sci_gpio_write(chip, offset, REG_GPIO_IBE, 1);
		__irq_set_handler_locked(data->irq, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		sci_gpio_write(chip, offset, REG_GPIO_IS, 1);
		sci_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sci_gpio_write(chip, offset, REG_GPIO_IEV, 1);
		__irq_set_handler_locked(data->irq, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		sci_gpio_write(chip, offset, REG_GPIO_IS, 1);
		sci_gpio_write(chip, offset, REG_GPIO_IBE, 0);
		sci_gpio_write(chip, offset, REG_GPIO_IEV, 0);
		__irq_set_handler_locked(data->irq, handle_level_irq);
		break;
	default:
		WARN(1, "IRQ#%u unknown type %08x\n", data->irq, flow_type);
		break;
	}
	sci_gpio_unlock();
	return 0;
}

int sci_gpio_irq_set_wake(struct irq_data *data, unsigned int on)
{
	return 0;
}

void sci_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct gpio_chip *chip =
	    (struct gpio_chip *)desc->irq_data.handler_data;
	int offset;
	for (offset = 0; offset <= chip->ngpio; offset += 16) {
		u16 tmp =
		    sci_gpio_read(chip, offset, REG_GPIO_MIS) & BITS_GPIO_MASK;
		while (tmp) {
			irq = __ffs(tmp);	//Count Leading Zeros
			tmp &= ~(1 << irq);	//clear
			irq = chip->to_irq(chip, offset + irq);
			debug("%s %d\n", __FUNCTION__, irq);
			generic_handle_irq(irq);
		}
	}
}

static struct irq_chip sc8810_gpio_irq_chip = {
	.name = "sprd-gpio",
	.irq_enable = sci_gpio_irq_enable,
	.irq_disable = sci_gpio_irq_disable,
	.irq_ack = sci_gpio_irq_ack,
	.irq_mask = sci_gpio_irq_mask,
	.irq_unmask = sci_gpio_irq_unmask,
	.irq_set_wake = sci_gpio_irq_set_wake,
	.irq_set_type = sci_gpio_irq_set_type,
};

static struct irq_chip sc8810_ana_gpio_irq_chip = {
	.name = "sprd-ana-gpio",
	.irq_enable = sci_gpio_irq_enable,
	.irq_disable = sci_gpio_irq_disable,
	.irq_ack = sci_gpio_irq_ack,
	.irq_mask = sci_gpio_irq_mask,
	.irq_unmask = sci_gpio_irq_unmask,
	.irq_set_wake = sci_gpio_irq_set_wake,
	.irq_set_type = sci_gpio_irq_set_type,
};

void __init gpio_irq_init(unsigned int irq, irq_flow_handler_t handler,
			  struct gpio_chip *gpiochip, struct irq_chip *irqchip)
{
	int offset;

	/* setup the cascade irq handlers */
	irq_set_chained_handler(irq, handler);
	irq_set_handler_data(irq, gpiochip);
	for (offset = 0; offset < gpiochip->ngpio; offset++) {
		int irqno = gpiochip->to_irq(gpiochip, offset);
		irq_set_chip_and_handler(irqno, irqchip, handle_level_irq);
		irq_set_chip_data(irqno, gpiochip);
		set_irq_flags(irqno, IRQF_VALID);
	}
}

static gpiomap_t __initconst gpiomap[] = {
//#include "__gpiomap.c"
};

static int __init gpio_init(void)
{
	int i;
	sci_gpio_enable();
	for (i = 0; i < sizeof(gpiomap) / sizeof(gpiomap_t); i++) {
		if (SCI_REG_IS_ADIE(gpiomap[i].reg))
			sci_adi_raw_write(gpiomap[i].reg, gpiomap[i].val);
		else
			SCI_D(gpiomap[i].reg) = gpiomap[i].val;
	}
	gpiochip_add((struct gpio_chip *)&sc8810_gpio_chip);
	gpiochip_add((struct gpio_chip *)&sc8810_ana_gpio_chip);
	gpio_irq_init(IRQ_GPIO_INT, sci_gpio_irq_handler,
		      (struct gpio_chip *)&sc8810_gpio_chip,
		      &sc8810_gpio_irq_chip);
	gpio_irq_init(IRQ_ANA_GPIO_INT, sci_gpio_irq_handler,
		      (struct gpio_chip *)&sc8810_ana_gpio_chip,
		      &sc8810_ana_gpio_irq_chip);
	return 0;
}

/*
#define IRQ_ANA_PB		(163)

static irqreturn_t sprd_pint_isr(int irq, void *dev_id)
{
	u16 data;
	data = SCI_A(ANA_REG_EIC_DATA);
	printk("%s %d %x\n", __FUNCTION__, irq, (u32)data);

	if (data & 8) {//not hold
		irq_set_irq_type(irq, IRQF_TRIGGER_LOW);
	}
	else {
		irq_set_irq_type(irq, IRQF_TRIGGER_HIGH);
	}
	gpio_direction_input(IRQ_ANA_PB);//reset eic triger
	return IRQ_HANDLED;
}

static int __init gpio_test(void)
{
	gpio_request(IRQ_ANA_PB, "");
	gpio_direction_input(IRQ_ANA_PB);
	printk("++++++++++++++++++++++++++++\n");
	return request_irq(gpio_to_irq(IRQ_ANA_PB), sprd_pint_isr, IRQF_TRIGGER_LOW, "powerkey", (void*)0);
}

late_initcall(gpio_test);

*/

arch_initcall(gpio_init);
