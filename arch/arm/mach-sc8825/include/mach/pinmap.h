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

#ifndef __ASM_ARM_ARCH_PINMAP_H
#define __ASM_ARM_ARCH_PINMAP_H

#define CTL_PIN_BASE			( SPRD_PIN_BASE )

/* registers definitions for controller CTL_PIN
   this is offset address, the real address should be REG_PIN_* + CTRL_PIN_BASE.
 */
#define REG_PIN_SIMCLK0                 ( 0x0018 )
#define REG_PIN_U0RTS                   ( 0x0134 )

/* bits definitions for register REG_PIN_XXX */
#define BITS_PIN_DS(_x_)              	( (_x_) << 8 & (BIT(8)|BIT(9)) )
#define BITS_PIN_DS_3(_x_)              ( (_x_) << 8 & (BIT(8)|BIT(9)|BIT(10)) )
#define BIT_PIN_WPU                     ( BIT(7) )
#define BIT_PIN_WPD                     ( BIT(6) )
#define BITS_PIN_AF(_x_)                ( (_x_) << 4 & (BIT(4)|BIT(5)) )
#define BIT_PIN_SLP_WPU                 ( BIT(3) )
#define BIT_PIN_SLP_WPD                 ( BIT(2) )
#define BIT_PIN_SLP_IE                  ( BIT(1) )
#define BIT_PIN_SLP_OE                  ( BIT(0) )

/* vars definitions for controller CTL_PIN */
#define BIT_PIN_NUL                     ( 0 )
#define BIT_PIN_SLP_NUL                 ( 0 )
#define BIT_PIN_SLP_Z                   ( 0 )

#endif
