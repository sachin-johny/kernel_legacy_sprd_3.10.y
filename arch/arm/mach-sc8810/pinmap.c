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
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/pinmap.h>

pinmap_t __initconst pinmap[] = {
#include <pinmap-board.h>
};

pinmap_t __initconst rfctl_pinmap[] = {
#include <rfctl-pinmap-board.h>
};

static int __init pull_down_rfctl_nongpio_pins(void);

/*
 * For all non-rfctl pins, initialize them as the default configuration
 * defined in pinmap-board.h
 *
 * For rfctl pins defined in rfctl-pinmap-board.h, we have a special initialization.
 * @see: pull_down_rfctl_nongpio_pins() defined in this file
 */
static int __init pin_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pinmap); i++) {
		__raw_writel(pinmap[i].val, CTL_PIN_BASE + pinmap[i].reg);
	}
	return pull_down_rfctl_nongpio_pins();
}

/*
 * All rfctl pins that are not configured as gpio functionality must be
 * pulled down before modem become alive, otherwise the unstable
 * electrical level may lead to a leackage of electricity.
 *
 * Also, we have to recover the default configuration defined in
 * rfctl-pinmap-board.h after modem become alive.
 * @see: cp_alive_gpio_handle() defined in
 *       drivers/char/modem_interface/modem_gpio_drv.c
 */
static int __init pull_down_rfctl_nongpio_pins(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(rfctl_pinmap); i++) {
		if ((rfctl_pinmap[i].val & BITS_PIN_AF_MASK) != BITS_PIN_AF_GPIO) {
			/* If a rfctl pin is configured as a non-gpip, set the
			   pin as a output gpio temporarily and pull it down. */
			__raw_writel((rfctl_pinmap[i].val & BITS_PIN_DS_MASK)
				| BITS_PIN_AF_GPIO | BIT_PIN_WPD
				| BIT_PIN_SLP_WPD | BIT_PIN_SLP_OE,
				CTL_PIN_BASE + rfctl_pinmap[i].reg);
		} else {
			/* If a rfctl pin is configured as a gpio, use the default
			   configuration defined in rfctl-pinmap-board.h. */
			__raw_writel(rfctl_pinmap[i].val, CTL_PIN_BASE
				+ rfctl_pinmap[i].reg);
		}
	}
	return 0;
}

#ifdef CONFIG_ARCH_SC7710
void enable_cp_jtag(void)
{
	uint32_t val;
	val = __raw_readl(CTL_PIN_BASE + REG_PIN_MTCK) & ~ BITS_PIN_AF(-1);
	__raw_writel(val|BITS_PIN_AF(1), CTL_PIN_BASE + REG_PIN_MTCK);

	val = __raw_readl(CTL_PIN_BASE + REG_PIN_MTDI) & ~ BITS_PIN_AF(-1);
	__raw_writel(val|BITS_PIN_AF(1), CTL_PIN_BASE + REG_PIN_MTDI);

	val = __raw_readl(CTL_PIN_BASE + REG_PIN_MTDO) & ~ BITS_PIN_AF(-1);
	__raw_writel(val|BITS_PIN_AF(1), CTL_PIN_BASE + REG_PIN_MTDO);

	val = __raw_readl(CTL_PIN_BASE + REG_PIN_MTMS) & ~ BITS_PIN_AF(-1);
	__raw_writel(val|BITS_PIN_AF(1), CTL_PIN_BASE + REG_PIN_MTMS);

	val = __raw_readl(CTL_PIN_BASE + REG_PIN_MTRST_N) & ~ BITS_PIN_AF(-1);
	__raw_writel(val|BITS_PIN_AF(1), CTL_PIN_BASE + REG_PIN_MTRST_N);

	val = __raw_readl(SPRD_GREG_BASE + 0xB0) & ~ (BIT(19)|BIT(20)|BIT(21)|BIT(22));
	__raw_writel(val, SPRD_GREG_BASE + 0xB0);
}
#endif
arch_initcall(pin_init);
