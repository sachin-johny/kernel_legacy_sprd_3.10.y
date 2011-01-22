/*
 * include/asm-arm/arch-osware/entry-macro.S
 *
 * Low-level IRQ helper macros for generic OSWARE-based platforms
 *
 *  Copyright (C) 2007 Jaluna SA.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

		.macro	disable_fiq
		.endm

		.macro	get_irqnr_preamble, base, tmp
		.endm

		.macro	arch_ret_to_user, tmp1, tmp2
		.endm

		.macro	irq_prio_table
		.endm
