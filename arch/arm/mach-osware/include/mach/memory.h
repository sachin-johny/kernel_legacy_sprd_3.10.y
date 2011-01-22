/*
 *  linux/include/asm-arm-nk/arch-osware/memory.h
 *
 *  Copyright (C) 2006 Jaluna SA.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_ARCH_OSWARE_MEMORY_H
#define __ASM_ARCH_OSWARE_MEMORY_H

/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET         CONFIG_RAM_PHYS

/*
 * Bus address is physical address.
 */
#define __virt_to_bus(x)    __virt_to_phys(x)
#define __bus_to_virt(x)    __phys_to_virt(x)

#endif /* __ASM_ARCH_OSWARE_MEMORY_H */
