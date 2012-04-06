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
#include <linux/module.h>
#include <asm/gpio.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/adi.h>
#include <mach/gpio.h>
#include <mach/ctl_eic.h>
#include <mach/ana_ctl_glb.h>
#include "adi_internal.h"

/*
#undef pr_debug
#define pr_debug pr_info
*/
#define debug(format, arg...) pr_debug("gpio: " "+++" format, ## arg)

#define CTL_EIC_BASE					( SPRD_EIC_BASE )
#define ANA_CTL_EIC_BASE				( SPRD_MISC_BASE + 0x0700)
#define ANA_CTL_GLB_BASE				SCI_ADDRESS(SPRD_MISC_BASE, 0x0600)

int sci_gpio_read(struct gpio_chip *chip, unsigned offset, u32 reg);
int sci_gpio_write(struct gpio_chip *chip, unsigned offset, u32 reg, int value);
int sci_gpio_request(struct gpio_chip *chip, unsigned offset);
void sci_gpio_free(struct gpio_chip *chip, unsigned offset);
int sci_gpio_direction_input(struct gpio_chip *chip, unsigned offset);
int sci_gpio_get(struct gpio_chip *chip, unsigned offset);
int sci_gpio_to_irq(struct gpio_chip *chip, unsigned offset);
int sci_irq_to_gpio(struct gpio_chip *chip, unsigned irq);

void sci_gpio_irq_enable(struct irq_data *data);
void sci_gpio_irq_disable(struct irq_data *data);
void sci_gpio_irq_ack(struct irq_data *data);
void sci_gpio_irq_mask(struct irq_data *data);
void sci_gpio_irq_unmask(struct irq_data *data);
int sci_gpio_irq_set_type(struct irq_data *data, unsigned int flow_type);
int sci_gpio_irq_set_wake(struct irq_data *data, unsigned int on);

void sci_gpio_irq_handler(unsigned int irq, struct irq_desc *desc);
irqreturn_t sci_gpio_irq_act_handler(int irq, void *dev_id);
void __init gpio_irq_init(unsigned int irq, irq_flow_handler_t handler,
			  struct gpio_chip *gpiochip, struct irq_chip *irqchip);
void __init gpio_demux_irq_init(unsigned int irq, struct irqaction *action,
				struct gpio_chip *gpiochip,
				struct irq_chip *irqchip);

struct sci_eic_chip {
	struct gpio_chip chip;
	u32 base_addr;
	int adie;
};

#define EIC_INFO(_v_)					( ((struct sci_eic_chip *)chip)->_v_ )

//16 gpios for one address
#define EIC_OFF2REG(_b_)				( (_b_) + ((offset >> 4) << 7) )
#define EIC_OFF2REG2(_b_)				( EIC_OFF2REG(_b_) - CTL_GPIO_BASE + base_addr )
#define EIC_OFF2REG3(_b_)				( EIC_OFF2REG(_b_) - CTL_EIC_BASE + base_addr )
#define EIC_OFF2SHIFT					( offset & 0x0f )

//16 ana gpios for one address
#define EIC_OFF2ANAREG(_b_)			( (_b_) + ((offset >> 4) << 6) )

//#define CONFIG_SUPPORT_EIC_LOCK       0

int sci_eic_enable(void)
{
	SCI_A_SET(ANA_REG_GLB_APB_CLK_EN, BIT_EIC_EB | BIT_RTC_EIC_EB);
	return 0;
}

int sci_eic_disable(void)
{
	return 0;
}

#if defined(CONFIG_SUPPORT_EIC_LOCK)
int sci_eic_lock(void)
{
	return 0;
}

int sci_eic_unlock(void)
{
	return 0;
}

#else
#define sci_eic_lock()					\
		unsigned long flags;			\
		local_irq_save(flags)			\

#define sci_eic_unlock() local_irq_restore(flags)

#endif

static int sci_eic_direction_output(struct gpio_chip *chip, unsigned offset,
				    int value)
{
	WARN(1, "EIC%u not support direction output\n", chip->base + offset);
	return -1;
}

static int sci_eic_set_debounce(struct gpio_chip *chip, unsigned offset,
				unsigned debounce)
{
	u32 base_addr = EIC_INFO(base_addr);
	int adie = EIC_INFO(adie);
	u32 reg = EIC_OFF2REG3(REG_EIC_0CTRL) + (EIC_OFF2SHIFT << 2);
	u32 tmp = SCI_REG_GET(reg);
	debug("%s %d+%d\n", __FUNCTION__, chip->base, offset);
	BUG_ON(offset >= 8);	//only lower 8 eics is valid
	tmp |= BIT_FORCE_CLK_DBNC;	//BUGBUG: why force?

	tmp &= ~MASK_EIC_DBNC_CNT;	//clear
	tmp |= BIT_EIC_DBNC_EN | BITS_EIC_DBNC_CNT(debounce);
	SCI_REG_WRITE(reg, tmp);
	return 0;
}

static struct sci_eic_chip sc8810_eic_chip = {
	.chip.label = "sc8810-eic",
	.chip.request = sci_gpio_request,
	.chip.free = sci_gpio_free,
	.chip.direction_input = sci_gpio_direction_input,
	.chip.get = sci_gpio_get,
	.chip.direction_output = sci_eic_direction_output,
	.chip.set = 0,
	.chip.set_debounce = sci_eic_set_debounce,
	.chip.to_irq = sci_gpio_to_irq,
	.chip.base = 0,
	.chip.ngpio = 16,
	.base_addr = CTL_EIC_BASE,
	.adie = 0,
};

static struct sci_eic_chip sc8810_ana_eic_chip = {
	.chip.label = "sc8810-ana-eic",
	.chip.request = sci_gpio_request,
	.chip.free = sci_gpio_free,
	.chip.direction_input = sci_gpio_direction_input,
	.chip.get = sci_gpio_get,
	.chip.direction_output = sci_eic_direction_output,
	.chip.set = 0,
	.chip.set_debounce = sci_eic_set_debounce,
	.chip.to_irq = sci_gpio_to_irq,
	.chip.base = 160,
	.chip.ngpio = 16,
	.base_addr = ANA_CTL_EIC_BASE,
	.adie = 1,
};

int sci_eic_irq_set_wake(struct irq_data *data, unsigned int on)
{
	return 0;
}

static struct irq_chip sc8810_eic_irq_chip = {
	.name = "sprd-eic",
	.irq_ack = sci_gpio_irq_ack,
	.irq_mask = sci_gpio_irq_mask,
	.irq_unmask = sci_gpio_irq_unmask,
	.irq_set_wake = sci_eic_irq_set_wake,
	.irq_set_type = sci_gpio_irq_set_type,
};

static struct irq_chip sc8810_ana_eic_irq_chip = {
	.name = "sprd-ana-eic",
	.irq_ack = sci_gpio_irq_ack,
	.irq_mask = sci_gpio_irq_mask,
	.irq_unmask = sci_gpio_irq_unmask,
	.irq_set_wake = sci_eic_irq_set_wake,
	.irq_set_type = sci_gpio_irq_set_type,
};

static struct irqaction sc8810_eic_irq_action = {
	.name = "sprd-eic-act",
	.flags = IRQF_SHARED,
	.handler = sci_gpio_irq_act_handler,
	.dev_id = &sc8810_eic_chip,
};

static int __init eic_init(void)
{
	sci_eic_enable();
	gpiochip_add((struct gpio_chip *)&sc8810_eic_chip);
	gpiochip_add((struct gpio_chip *)&sc8810_ana_eic_chip);
#define IRQ_EIC_INT		IRQ_GPIO_INT
	gpio_demux_irq_init(IRQ_EIC_INT, &sc8810_eic_irq_action,
			    (struct gpio_chip *)&sc8810_eic_chip,
			    &sc8810_eic_irq_chip);
	gpio_irq_init(IRQ_ANA_EIC_INT, sci_gpio_irq_handler,
		      (struct gpio_chip *)&sc8810_ana_eic_chip,
		      &sc8810_ana_eic_irq_chip);
	return 0;
}

arch_initcall(eic_init);
