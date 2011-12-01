/* arch/arm/mach-sc8800g/idle.S
 *
 * Idle processing for SC8800G
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

ENTRY(cpu_idle_asm)
	mov     pc, lr
