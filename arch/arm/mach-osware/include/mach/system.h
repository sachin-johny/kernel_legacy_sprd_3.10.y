/*
 *  linux/include/asm-arm/arch-osware/system.h
 *
 *  Copyright (C) 2006 Jaluna SA.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_ARCH_OSWARE_SYSTEM_H
#define __ASM_ARCH_OSWARE_SYSTEM_H

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char* cmd)
{
}

#endif /* __ASM_ARCH_OSWARE_SYSTEM_H */
