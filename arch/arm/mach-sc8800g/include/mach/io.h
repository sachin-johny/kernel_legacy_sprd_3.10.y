/* arch/arm/mach-sc8800g/include/mach/io.h
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <asm/system.h>

#define IO_SPACE_LIMIT 0xffffffff

static inline void __iomem *__io(unsigned long addr)
{
	return (void __iomem *)addr;
}
#define __io(a)         __io(a)
#define __mem_pci(a)    (a)

#ifndef CONFIG_NKERNEL

static inline void __raw_bits_and(unsigned int v, unsigned int a)
{
	unsigned long flags;

	raw_local_irq_save(flags);
	__raw_writel((__raw_readl(a) & v), a);
	raw_local_irq_restore(flags);
}

static inline void __raw_bits_or(unsigned int v, unsigned int a)
{
	unsigned long flags;

	raw_local_irq_save(flags);
	__raw_writel((__raw_readl(a) | v), a);
	raw_local_irq_restore(flags);
}

static inline void __raw_bits_xor(unsigned int v, unsigned int a)
{
	unsigned long flags;

	raw_local_irq_save(flags);
	__raw_writel((__raw_readl(a) ^ v), a);
	raw_local_irq_restore(flags);
}

#else /* CONFIG_NKERNEL */

static inline void __raw_bits_and(unsigned int v, unsigned int a)
{
	unsigned long flags;

	hw_local_irq_save(flags);
	__raw_writel((__raw_readl(a) & v), a);
	hw_local_irq_restore(flags);
}

static inline void __raw_bits_or(unsigned int v, unsigned int a)
{
	unsigned long flags;

	hw_local_irq_save(flags);
	__raw_writel((__raw_readl(a) | v), a);
	hw_local_irq_restore(flags);
}

static inline void __raw_bits_xor(unsigned int v, unsigned int a)
{
	unsigned long flags;

	hw_local_irq_save(flags);
	__raw_writel((__raw_readl(a) ^ v), a);
	hw_local_irq_restore(flags);
}

#endif /* CONFIG_NKERNEL */

#endif
