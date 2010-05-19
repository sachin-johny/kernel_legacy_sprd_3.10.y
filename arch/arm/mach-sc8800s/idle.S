/* arch/arm/mach-sc8800s/idle.S
 *
 * Idle processing for SC8800
 *
 * Copyright (c) 2010 Spreadtrum
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
		
#include <linux/linkage.h>
#include <asm/assembler.h>

ENTRY(arch_idle)
#ifdef CONFIG_SC8800S_IDLE

#endif
	mov     pc, lr
