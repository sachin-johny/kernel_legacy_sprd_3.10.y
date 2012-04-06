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

#ifndef __CTL_ADC_H__
#define __CTL_ADC_H__

#define CTL_ADC
#define ANA_CTL_ADC

/* registers definitions for controller ANA_CTL_ADC */
#define ANA_REG_ADC_CTRL                SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x0000)
#define ANA_REG_ADC_CS                  SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x0004)
#define ANA_REG_ADC_TPC_CH_CTRL         SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x0008)
#define ANA_REG_ADC_DAT                 SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x000c)
#define ANA_REG_ADC_IE                  SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x0010)
#define ANA_REG_ADC_IC                  SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x0014)
#define ANA_REG_ADC_ISTS                SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x0018)
#define ANA_REG_ADC_ISRC                SCI_ADDRESS(ANA_CTL_ADC_BASE, 0x001c)

/* bits definitions for register REG_ADC_CTRL */
#define BIT_ADC_STATUS                  ( BIT(4) )
#define BIT_HW_INT_EN                   ( BIT(3) )
#define BIT_TPC_CH_ON                   ( BIT(2) )
#define BIT_SW_CH_ON                    ( BIT(1) )
#define BIT_ADC_EN                      ( BIT(0) )

/* bits definitions for register REG_ADC_CS */
#define BIT_ADC_SLOW                    ( BIT(5) )
#define BIT_ADC_SCALE                   ( BIT(4) )
#define BITS_ADC_CS(_x_)                ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)) )

#define SHIFT_ADC_CS                    ( 0 )
#define MASK_ADC_CS                     ( BIT(0)|BIT(1)|BIT(2)|BIT(3) )

/* bits definitions for register REG_ADC_TPC_CH_CTRL */
#define BITS_TPC_CH_DELAY(_x_)          ( (_x_) << 8 & (BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )
#define BITS_TPC_Y_CH(_x_)              ( (_x_) << 4 & (BIT(4)|BIT(5)|BIT(6)|BIT(7)) )
#define BITS_TPC_X_CH(_x_)              ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)) )

/* bits definitions for register REG_ADC_DAT */
#define BITS_ADC_DAT(_x_)               ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9)) )

#define SHIFT_ADC_DAT                   ( 0 )
#define MASK_ADC_DAT                    ( BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)|BIT(8)|BIT(9) )

/* bits definitions for register REG_ADC_IE */
#define BIT_ADC_IE                      ( BIT(0) )

/* bits definitions for register REG_ADC_IC */
#define BIT_ADC_IC                      ( BIT(0) )

/* bits definitions for register REG_ADC_ISTS */
#define BIT_ADC_MIS                     ( BIT(0) )

/* bits definitions for register REG_ADC_ISRC */
#define BIT_ADC_RIS                     ( BIT(0) )

/* vars definitions for controller CTL_ADC */

/* adc channel definition */
#define ADC_CHANNEL_INVALID  0xffff

enum adc_channel {
	ADIN_0 = 0,
	ADC_CHANNEL_TEMP = 1,
	ADIN_2 = 2,
	ADIN_3 = 3,
	ADC_CHANNEL_PROG = 4,
	ADC_CHANNEL_VBAT = 5,
	ADC_CHANNEL_VCHG = 6,
	ADIN_7 = 7,
	ADIN_8 = 8,
	ADIN_9 = ADC_CHANNEL_INVALID,
	ADIN_10 = ADC_CHANNEL_INVALID,
	ADIN_11 = ADC_CHANNEL_INVALID,
	ADIN_12 = ADC_CHANNEL_INVALID,
	ADIN_13 = ADC_CHANNEL_INVALID,
	ADIN_14 = 14,
	ADIN_15 = 15,
	ADC_MAX = 16,
};

/* adc scale definition */
enum adc_scale {
	ADC_SCALE_3V = 0,
	ADC_SCALE_1V2 = 1,
};

#endif //__CTL_ADC_H__
