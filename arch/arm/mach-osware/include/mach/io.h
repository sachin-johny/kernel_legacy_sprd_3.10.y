/*
 *  linux/include/asm-arm/arch-osware/io.h
 *
 *  Copyright (C) 2006 Jaluna SA.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_ARCH_OSWARE_IO_H
#define __ASM_ARCH_OSWARE_IO_H

#define IO_SPACE_LIMIT   0xffffffff

#define __io(a)          ((void __iomem *)(a))
#define __mem_pci(a)	 (a)
#define __mem_isa(a)	 (a)

#endif /* __ASM_ARCH_OSWARE_IO_H */
