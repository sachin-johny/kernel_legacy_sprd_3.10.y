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

#ifndef __ASM_ARM_ARCH_GLOBALREGS_H
#define __ASM_ARM_ARCH_GLOBALREGS_H
#include <mach/hardware.h>



/* global register types */
enum {
	REG_TYPE_GLOBAL = 0,
	REG_TYPE_AHB_GLOBAL,
	REG_TYPE_MAX
};

int32_t sprd_greg_read(uint32_t type, uint32_t reg_offset);
void sprd_greg_write(uint32_t type, uint32_t value, uint32_t reg_offset);
void sprd_greg_set_bits(uint32_t type, uint32_t bits, uint32_t reg_offset);
void sprd_greg_clear_bits(uint32_t type, uint32_t bits, uint32_t reg_offset);

#endif

