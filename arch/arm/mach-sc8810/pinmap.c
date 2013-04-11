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

typedef struct {
	uint32_t reg;
	uint32_t val;
} pinmap_t;

pinmap_t __initconst pinmap[] = {
#include <pinmap-board.h>
};

static int __init pin_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(pinmap); i++) {
		__raw_writel(pinmap[i].val, CTL_PIN_BASE + pinmap[i].reg);
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
