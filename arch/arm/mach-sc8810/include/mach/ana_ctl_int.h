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

#ifndef __ANA_CTL_INT_H__
#define __ANA_CTL_INT_H__

#define ANA_CTL_INT
#define ANA_CTL_INT

/* registers definitions for controller ANA_CTL_INT */
#define ANA_REG_INT_MASK_STATUS         SCI_ADDRESS(ANA_CTL_INT_BASE, 0x0000)
#define ANA_REG_INT_RAW_STATUS          SCI_ADDRESS(ANA_CTL_INT_BASE, 0x0004)
#define ANA_REG_INT_EN                  SCI_ADDRESS(ANA_CTL_INT_BASE, 0x0008)
#define ANA_REG_INT_MASK_STATUS_SYNC    SCI_ADDRESS(ANA_CTL_INT_BASE, 0x000c)

/* bits definitions for register REG_INT_MASK_STATUS */
#define BIT_ANA_CHGRWDG_INT             ( BIT(6) )
#define BIT_ANA_EIC_INT                 ( BIT(5) )
#define BIT_ANA_TPC_INT                 ( BIT(4) )
#define BIT_ANA_WDG_INT                 ( BIT(3) )
#define BIT_ANA_RTC_INT                 ( BIT(2) )
#define BIT_ANA_GPIO_INT                ( BIT(1) )
#define BIT_ANA_ADC_INT                 ( BIT(0) )

/* vars definitions for controller ANA_CTL_INT */
#define MASK_ANA_INT                    ( BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6) )

#endif //__ANA_CTL_INT_H__
