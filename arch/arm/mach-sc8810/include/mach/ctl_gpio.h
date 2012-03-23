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

#ifndef __CTL_GPIO_H__
#define __CTL_GPIO_H__

#define CTL_GPIO
#define ANA_CTL_GPIO

/* registers definitions for controller CTL_GPIO */
#define REG_GPIO_DATA                   SCI_ADDRESS(CTL_GPIO_BASE, 0x0000)
#define REG_GPIO_DMSK                   SCI_ADDRESS(CTL_GPIO_BASE, 0x0004)
#define REG_GPIO_DIR                    SCI_ADDRESS(CTL_GPIO_BASE, 0x0008)
#define REG_GPIO_IS                     SCI_ADDRESS(CTL_GPIO_BASE, 0x000c)
#define REG_GPIO_IBE                    SCI_ADDRESS(CTL_GPIO_BASE, 0x0010)
#define REG_GPIO_IEV                    SCI_ADDRESS(CTL_GPIO_BASE, 0x0014)
#define REG_GPIO_IE                     SCI_ADDRESS(CTL_GPIO_BASE, 0x0018)
#define REG_GPIO_RIS                    SCI_ADDRESS(CTL_GPIO_BASE, 0x001c)
#define REG_GPIO_MIS                    SCI_ADDRESS(CTL_GPIO_BASE, 0x0020)
#define REG_GPIO_IC                     SCI_ADDRESS(CTL_GPIO_BASE, 0x0024)
#define REG_GPIO_INEN                   SCI_ADDRESS(CTL_GPIO_BASE, 0x0028)
/* registers definitions for controller ANA_CTL_GPIO */
#define ANA_REG_GPIO_DATA               SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0000)
#define ANA_REG_GPIO_DMSK               SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0004)
#define ANA_REG_GPIO_DIR                SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0008)
#define ANA_REG_GPIO_IS                 SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x000c)
#define ANA_REG_GPIO_IBE                SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0010)
#define ANA_REG_GPIO_IEV                SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0014)
#define ANA_REG_GPIO_IE                 SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0018)
#define ANA_REG_GPIO_RIS                SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x001c)
#define ANA_REG_GPIO_MIS                SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0020)
#define ANA_REG_GPIO_IC                 SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0024)
#define ANA_REG_GPIO_INEN               SCI_ADDRESS(ANA_CTL_GPIO_BASE, 0x0028)

/* vars definitions for controller CTL_GPIO */
#define BITS_GPIO_MASK                  ( 0xffff )

#endif //__CTL_GPIO_H__
