/* * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#ifndef __ADI_H__
#define __ADI_H__

#include <mach/hardware.h>
#include <mach/ctl_adi.h>

#define ANA_VIRT_BASE					( SPRD_MISC_BASE )
#define ANA_PHYS_BASE					( SPRD_MISC_PHYS )
#define ANA_V2P(reg) 					( (reg) - ANA_VIRT_BASE + ANA_PHYS_BASE )
#define ANA_P2V(reg) 					( (reg) - ANA_PHYS_BASE + ANA_VIRT_BASE )

#define SCI_ADDRESS(_b_, _o_)			( (u32)(_b_) + (_o_) )

#define SCI_D(reg)						( *(volatile u32 *)(reg) )
#define SCI_A(reg)						( sci_adi_read(reg) )

#define SCI_D_SET(reg, bits)			( SCI_D(reg) |= (bits) )
#define SCI_D_CLR(reg, bits)			( SCI_D(reg) &=~(bits) )
#define SCI_A_SET(reg, bits)			( sci_adi_set(reg, bits) )
#define SCI_A_CLR(reg, bits)			( sci_adi_clr(reg, bits) )

#define SCI_REG_GET(reg)				( (adie)?SCI_A(reg):SCI_D(reg) )
#define SCI_REG_SET(reg, bits)			( (adie)?SCI_A_SET(reg, bits):SCI_D_SET(reg, bits) )
#define SCI_REG_CLR(reg, bits)			( (adie)?SCI_A_CLR(reg, bits):SCI_D_CLR(reg, bits) )
#define SCI_REG_IS_ADIE(reg)			( reg >= SPRD_MISC_BASE + 0x0040 && reg < SPRD_MISC_BASE + 0x1000 )

#define SCI_REG_READ(reg)				( SCI_REG_GET(reg) )
#define SCI_REG_WRITE(reg, val)			( (adie)?sci_adi_raw_write(reg, val):(SCI_D(reg) = val) )

int sci_adi_read(u32 reg);
int sci_adi_raw_write(u32 reg, u16 val);
int sci_adi_write(u32 reg, u16 val, u16 msk);
int sci_adi_set(u32 reg, u16 bits);
int sci_adi_clr(u32 reg, u16 bits);

#endif //__ADI_H__
