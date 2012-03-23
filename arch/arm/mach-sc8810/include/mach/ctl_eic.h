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

#ifndef __CTL_EIC_H__
#define __CTL_EIC_H__

#define CTL_EIC
#define ANA_CTL_EIC

/* registers definitions for controller CTL_EIC */
#define REG_EIC_DATA                    SCI_ADDRESS(CTL_EIC_BASE, 0x0000)
#define REG_EIC_DMSK                    SCI_ADDRESS(CTL_EIC_BASE, 0x0004)
#define REG_EIC_IEV                     SCI_ADDRESS(CTL_EIC_BASE, 0x0014)
#define REG_EIC_IE                      SCI_ADDRESS(CTL_EIC_BASE, 0x0018)
#define REG_EIC_RIS                     SCI_ADDRESS(CTL_EIC_BASE, 0x001c)
#define REG_EIC_MIS                     SCI_ADDRESS(CTL_EIC_BASE, 0x0020)
#define REG_EIC_IC                      SCI_ADDRESS(CTL_EIC_BASE, 0x0024)
#define REG_EIC_TRIG                    SCI_ADDRESS(CTL_EIC_BASE, 0x0028)
#define REG_EIC_0CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x0040)
#define REG_EIC_1CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x0044)
#define REG_EIC_2CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x0048)
#define REG_EIC_3CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x004c)
#define REG_EIC_4CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x0050)
#define REG_EIC_5CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x0054)
#define REG_EIC_6CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x0058)
#define REG_EIC_7CTRL                   SCI_ADDRESS(CTL_EIC_BASE, 0x005c)
#define REG_EIC_DUMMYCTRL               SCI_ADDRESS(CTL_EIC_BASE, 0x0000)
/* registers definitions for controller ANA_CTL_EIC */
#define ANA_REG_EIC_DATA                SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0000)
#define ANA_REG_EIC_DMSK                SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0004)
#define ANA_REG_EIC_IEV                 SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0014)
#define ANA_REG_EIC_IE                  SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0018)
#define ANA_REG_EIC_RIS                 SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x001c)
#define ANA_REG_EIC_MIS                 SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0020)
#define ANA_REG_EIC_IC                  SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0024)
#define ANA_REG_EIC_TRIG                SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0028)
#define ANA_REG_EIC_0CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0040)
#define ANA_REG_EIC_1CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0044)
#define ANA_REG_EIC_2CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0048)
#define ANA_REG_EIC_3CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x004c)
#define ANA_REG_EIC_4CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0050)
#define ANA_REG_EIC_5CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0054)
#define ANA_REG_EIC_6CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0058)
#define ANA_REG_EIC_7CTRL               SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x005c)
#define ANA_REG_EIC_DUMMYCTRL           SCI_ADDRESS(ANA_CTL_EIC_BASE, 0x0000)

/* bits definitions for register REG_EIC_DUMMYCTRL */
#define BIT_FORCE_CLK_DBNC              ( BIT(15) )
#define BIT_EIC_DBNC_EN                 ( BIT(14) )
#define BITS_EIC_DBNC_CNT(_x_)          ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11)) )

#define SHIFT_EIC_DBNC_CNT              ( 0 )
#define MASK_EIC_DBNC_CNT               ( BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)|BIT(10)|BIT(11) )

/* vars definitions for controller CTL_EIC */
#define BITS_EIC_MASK                   ( 0xff )

#endif //__CTL_EIC_H__
