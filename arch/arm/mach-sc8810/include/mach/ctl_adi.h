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
 *
 ************************************************
 * Automatically generated C config: don't edit *
 ************************************************
 */

#ifndef __CTL_ADI_H__
#define __CTL_ADI_H__

#define CTL_ADI

/* registers definitions for controller CTL_ADI */
#define REG_ADI_CTRL0                   SCI_ADDRESS(CTL_ADI_BASE, 0x0004)
#define REG_ADI_CHNL_PRI                SCI_ADDRESS(CTL_ADI_BASE, 0x0008)
#define REG_ADI_RD_CMD                  SCI_ADDRESS(CTL_ADI_BASE, 0x0024)
#define REG_ADI_RD_DATA                 SCI_ADDRESS(CTL_ADI_BASE, 0x0028)
#define REG_ADI_FIFO_STS                SCI_ADDRESS(CTL_ADI_BASE, 0x002c)

/* bits definitions for register REG_ADI_CTRL0 */
#define BIT_ARM_SCLK_EN                 ( BIT(1) )

/* bits definitions for register REG_ADI_CHNL_PRI */
#define BITS_PD_WR_PRI(_x_)             ( (_x_) << 14 & (BIT(14)|BIT(15)) )
#define BITS_RFT_WR_PRI(_x_)            ( (_x_) << 12 & (BIT(12)|BIT(13)) )
#define BITS_DSP_RD_PRI(_x_)            ( (_x_) << 10 & (BIT(10)|BIT(11)) )
#define BITS_DSP_WR_PRI(_x_)            ( (_x_) << 8 & (BIT(8)|BIT(9)) )
#define BITS_ARM_RD_PRI(_x_)            ( (_x_) << 6 & (BIT(6)|BIT(7)) )
#define BITS_ARM_WR_PRI(_x_)            ( (_x_) << 4 & (BIT(4)|BIT(5)) )
#define BITS_STC_WR_PRI(_x_)            ( (_x_) << 2 & (BIT(2)|BIT(3)) )
#define BITS_INT_STEAL_PRI(_x_)         ( (_x_) << 0 & (BIT(0)|BIT(1)) )

/* bits definitions for register REG_ADI_RD_CMD */
#define BIT_RD_CMD_BUSY                 ( BIT(31) )

/* bits definitions for register REG_ADI_RD_DATA */
#define BITS_RD_ADDR(_x_)               ( (_x_) << 16 & (BIT(16)|BIT(17)|BIT(18)|BIT(19)|BIT(20)|BIT(21)|BIT(22)|BIT(23)|BIT(24)|BIT(25)|BIT(26)) )
#define BITS_RD_VALU(_x_)               ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )

#define SHIFT_RD_ADDR                   ( 16 )
#define MASK_RD_ADDR                    ( BIT(16)|BIT(17)|BIT(18)|BIT(19)|BIT(20)|BIT(21)|BIT(22)|BIT(23)|BIT(24)|BIT(25)|BIT(26) )

#define SHIFT_RD_VALU                   ( 0 )
#define MASK_RD_VALU                    ( BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15) )

/* bits definitions for register REG_ADI_FIFO_STS */
#define BIT_FIFO_FULL                   ( BIT(11) )
#define BIT_FIFO_EMPTY                  ( BIT(10) )

/* vars definitions for controller CTL_ADI */
#define ADI_CHNL_PRI_LOWEST             ( 0 )
#define ADI_CHNL_PRI_LOWER              ( 1 )
#define ADI_CHNL_PRI_HIGHER             ( 2 )
#define ADI_CHNL_PRI_HIGHEST            ( 3 )

#endif //__CTL_ADI_H__
