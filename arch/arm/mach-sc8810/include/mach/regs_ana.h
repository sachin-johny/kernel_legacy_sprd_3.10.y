/*
 *  linux/arch/arm/mach-sc8810/include\mach
 *
 *  Spreadtrum ldo control
 *
 *
 *  Author:	steve.zhan@spreadtrum.com
 *  Created:	Mon Jun 27, 2011
 *  Copyright:	Spreadtrum Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
*   the first version, only modified and checked ana ldo register.
*/


/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/


#ifndef _ANALOG_REG_V3_H_
#define _ANALOG_REG_V3_H_

#include <mach/hardware.h>
#include <mach/bits.h>
/**----------------------------------------------------------------------------*
**                               Micro Define                                 **
**----------------------------------------------------------------------------*/
///
//Analog die register define            for 8810 fixed begin
#define 	SPRD_ANA_BASE 	(SPRD_MISC_BASE + 0x600)
#define   ANA_REG_BASE         SPRD_ANA_BASE   //  0x82000600

#define   ANA_AGEN              (ANA_REG_BASE + 0x00)
#define   ANA_MODULE_ARM_RST    (ANA_REG_BASE + 0x04)
#define   ANA_LDO_PD_SET        (ANA_REG_BASE+ 0x08)
#define   ANA_LDO_PD_RST        (ANA_REG_BASE + 0x0C)
#define   ANA_LDO_PD_CTL0        (ANA_REG_BASE + 0x10)
#define   ANA_LDO_PD_CTL1        (ANA_REG_BASE + 0x14)
#define   ANA_LDO_VCTL0         (ANA_REG_BASE + 0x18)
#define   ANA_LDO_VCTL1         (ANA_REG_BASE + 0x1C)
#define   ANA_LDO_VCTL2         (ANA_REG_BASE + 0x20)
#define   ANA_LDO_VCTL3         (ANA_REG_BASE + 0x24)
#define   ANA_LDO_VCTL4         (ANA_REG_BASE + 0x28)
#define   ANA_LDO_SLP0           (ANA_REG_BASE + 0x2C)
#define   ANA_LDO_SLP1           (ANA_REG_BASE + 0x30)
#define   ANA_LDO_SLP2           (ANA_REG_BASE + 0x34)
#define   ANA_DCDC_CTRL             (ANA_REG_BASE + 0x38)
#define   ANA_DCDC_CTRL_DS           (ANA_REG_BASE + 0x3C)
#define   ANA_CTRL_CAL           (ANA_REG_BASE + 0x40)
#define   ANA_DCDCARM_CTRL            (ANA_REG_BASE + 0x44)
#define   ANA_DCDCARM_CTRL_CAL      (ANA_REG_BASE + 0x48)
#define   ANA_PLL_CTRL         (ANA_REG_BASE + 0x4C)
#define   ANA_APLLMN               (ANA_REG_BASE + 0x50)
#define   ANA_APLLWAIT         (ANA_REG_BASE + 0x54)
#define   ANA_RTC_CTRL         (ANA_REG_BASE + 0x58)
#define   ANA_TRF_CTRL          (ANA_REG_BASE + 0x5C)
#define   ANA_CHGR_CTRL0         (ANA_REG_BASE + 0X60)
#define   ANA_CHGR_CTRL1          (ANA_REG_BASE + 0X64)
#define   ANA_LED_CTRL          (ANA_REG_BASE + 0X68)
#define   ANA_VIBRATOR_CTRL0          (ANA_REG_BASE + 0X6C)
#define   ANA_VIBRATOR_CTRL1          (ANA_REG_BASE + 0X70)
#define   ANA_AUDIO_CTRL          (ANA_REG_BASE + 0X74)
#define   ANA_AUDIO_PA_CTRL0                   (ANA_REG_BASE + 0X78)
#define   ANA_AUDIO_PA_CTRL1          (ANA_REG_BASE + 0X7C)
#define   ANA_MIXED_CTRL          (ANA_REG_BASE + 0X80)
#define   ANA_STATUS          (ANA_REG_BASE + 0X84)
#define   ANA_RST_STATUS          (ANA_REG_BASE + 0X88)
#define   ANA_MCU_WR_PROT          (ANA_REG_BASE + 0X8C)
#define   ANA_VIBR_WR_PROT          (ANA_REG_BASE + 0X90)
#define   ANA_INT_CPI_DEBUG          (ANA_REG_BASE + 0X94)
#define   ANA_HWRST_RTC          (ANA_REG_BASE + 0X98)
#define   ANA_IF_SPR_CTRL          (ANA_REG_BASE + 0X9C)
#define   ANA_CHIP_ID_LOW          (ANA_REG_BASE + 0XF8)
#define   ANA_CHIP_ID_HIGH          (ANA_REG_BASE + 0XFC)

#define ANA_CHGR_CTL0 ANA_CHGR_CTRL0
#define ANA_CHGR_CTL1 ANA_CHGR_CTRL1
//fixed end






/*
  the AGEN register bit
*/
/*
//Reserved BIT_14-31
#define AGEN_PLL_FORCE_PD_EN    BIT_13
#define AGEN_RTC_ARCH_EN        BIT_12
#define AGEN_RTC_WDG_EN         BIT_11
#define AGEN_RTC_RTC_EN         BIT_10
#define AGEN_RTC_TPC_EN         BIT_9
#define AGEN_RTC_GPIO_EN        BIT_8
#define AGEN_APB_ARCH_EN        BIT_7
#define AGEN_TPC_EN             BIT_6
//Reserved BIT_5
#define AGEN_WDG_EN             BIT_4
#define AGEN_PINREG_EN          BIT_3
#define AGEN_ADC_EN             BIT_2
#define AGEN_RTC_EN             BIT_1
#define AGEN_GPIO_EN            BIT_0
*/
#define AGEN_APB_ARCH_EN	  	BIT_0
#define AGEN_RTC_EN              	BIT_1
#define AGEN_WDG_EN             	BIT_2
#define AGEN_EIC_EN				BIT_3
#define AGEN_TPC_EN              	BIT_4
#define AGEN_ADC_EN             		BIT_5
#define AGEN_GPIO_EN            	BIT_6
#define AGEN_PINREG_EN        	BIT_7
#define AGEN_RTC_ARCH_EN    	BIT_8
#define AGEN_RTC_RTC_EN       	BIT_9
#define AGEN_RTC_WDG_EN         	BIT_10
#define AGEN_RTC_EIC_EN		BIT_11
#define AGEN_RTC_TPC_EN         	BIT_12
#define AGEN_CLK_AUXADC_EN	BIT_13
#define AGEN_CLK_AUXAD_EN		BIT_14
#define AGEN_CHGRWDG_EN		BIT_15
//Reserved BIT_16-31

///ANA_CLK_CTL BIT map
/*
#define ACLK_CTL_AUXAD_EN       BIT_4
#define ACLK_CTL_AUXADC_EN  BIT_0
#define VBMCLK_ARM_EN           BIT_1
#define VBCTL_SEL               BIT_2
*/
#define ACLK_CTL_AUXAD_EN		AGEN_CLK_AUXAD_EN
#define ACLK_CTL_AUXADC_EN	AGEN_CLK_AUXADC_EN
#define VBMCLK_ARM_EN           	BIT_0
#define VBMCLK_SRC_SEL			BIT_2
#define VBCTL_SEL				VBMCLK_SRC_SEL

///ANA_LDO_PD_SET
#define ANA_LDO_PD_SET_MSK  0x3FF
///ANA_LDO_PD_CTL
#define ANA_LDO_PD_CTL_MSK  0x5555
///ANA_LDO_RST_MSK
#define ANA_LDO_PD_RST_MSK  0x0000

///ANA_VIBRATOR_CTRL0 BIT map
#define VIBR_STABLE_V_SHIFT	12
#define VIBR_STABLE_V_MSK	(0x0F << VIBR_STABLE_V_SHIFT)
#define VIBR_INIT_V_SHIFT	8
#define VIBR_INIT_V_MSK		(0x0F << VIBR_INIT_V_SHIFT)
#define VIBR_V_BP_SHIFT         4
#define VIBR_V_BP_MSK           (0x0F << VIBR_V_BP_SHIFT)
#define VIBR_PD_RST             BIT_3
#define VIBR_PD_SET             BIT_2
#define VIBR_BP_EN		BIT_1
#define VIBR_RTC_EN		BIT_0

///ANA_CHGR_CTL0
#define CHGR_ADAPTER_EN		BIT_0
#define CHGR_ADAPTER_EN_RST	BIT_1
#define CHGR_USB_500MA_EN		BIT_2
#define CHGR_USB_500MA_EN_RST	BIT_3

#define CHGR_USB_CHG_SHIFT              4
#define CHGR_USB_CHG_MSK                (3 << CHGR_USB_CHG_SHIFT)
#define CHGR_ADAPTER_CHG_SHIFT          6
#define CHGR_ADAPTER_CHG_MSK            (3 << CHGR_ADAPTER_CHG_SHIFT)
#define CHGR_PD_BIT                     			BIT_8
#define PA_LDO_EN_RST					BIT_9
#define CHGR_RECHG_BIT                  		BIT_12
#define CHGR_ADATPER_EN_BIT             	BIT_0
#define CHGR_ADATPER_EN_RST_BIT       	BIT_1
#define CHGR_USB_500MA_EN_BIT           	BIT_2
#define CHGR_USB_500MA_EN_RST_BIT       BIT_3
#define CHAR_ADAPTER_MODE_MSK           	(BIT_0|BIT_1|BIT_2|BIT_3)

///ANA_CHGR_CTL1
#define CHAR_SW_POINT_SHIFT     	0
#define CHAR_SW_POINT_MSK       		(0x1F << CHAR_SW_POINT_SHIFT)

///ANA_LED_CTL BIT map
/*
#define KPLED_CTL               ANA_LED_CTRL
#define KPLED_PD_SET            BIT_7
#define KPLED_PD_RST            BIT_8
#define KPLED_V_SHIFT           9
#define KPLED_V_MSK             (0x07 << KPLED_V_SHIFT)
*/
#define KPLED_CTL               ANA_LED_CTRL
#define KPLED_PD_SET            BIT_11
#define KPLED_PD_RST            BIT_12
#define KPLED_V_SHIFT           7
#define KPLED_V_MSK             (0x07 << KPLED_V_SHIFT)

/*
#define WHTLED_CTL              ANA_LED_CTRL
#define WHTLED_PD_SET           BIT_0
#define WHTLED_PD_RST           BIT_1
#define WHTLED_V_SHIFT          2
#define WHTLED_V_MSK            (0x1F << WHTLED_V_SHIFT)
*/
#define WHTLED_CTL              ANA_LED_CTRL
#define WHTLED_PD_SET           BIT_5
#define WHTLED_PD_RST           BIT_6
#define WHTLED_V_SHIFT          0
#define WHTLED_V_MSK            (0x1F << WHTLED_V_SHIFT)

///ANA_PA_CTL
#define LDO_PA_SET              BIT_6
#define LDO_PA_RST              BIT_7

///ANA_ADIE_CHIP_ID
#define ANA_G1_CHIP_ID          ((unsigned short)0)
#define ANA_G2_CHIP_ID          ((unsigned short)1)


/****************************************
  * add bit definitions used by power management.
  *
  *  Wang liwei. 2011-01-18
  *
  ****************************************/

/* bit definitions for register ANA_LDO_SLP */
#define	FSM_RF0_BP_EN		(0x1UL << 0)
#define	FSM_RF1_BP_EN		(0x1UL << 1)
#define	FSM_LDOPA_BP_EN		(0x1UL << 15)

/* ANA_LDO_PD_CTL0 */
#define LDO_USB_CTL		BIT_1
#define LDO_SDIO0_CTL	BIT_3
#define LDO_SIM0_CTL	BIT_5
#define LDO_SIM1_CTL	BIT_7
#define LDO_BPCAMD0_CTL		BIT_9
#define LDO_BPCAMD1_CTL		BIT_11
#define LDO_BPCAMA_CTL		BIT_13
#define LDO_BPVB_CTL		BIT_15

/* ANA_LDO_PD_CTL1 */
#define LDO_SDIO1_CTL	BIT_1
#define LDO_BPWIF0_CTL	BIT_3
#define LDO_BPWIF1_CTL	BIT_5
#define LDO_SIM2_CTL	BIT_7
#define LDO_SIM3_CTL	BIT_9


/* ANA_AUDIO_PA_CTRL0 */
#define AUDIO_PA_ENABLE		BIT_0
#define AUDIO_PA_ENABLE_RST		BIT_1


/* ANA_AUDIO_PA_CTRL1 */
#define AUDIO_PA_LDO_ENABLE		BIT_8
#define AUDIO_PA_LDO_ENABLE_RST		BIT_9

#endif //_ANALOG_REG_V3_H_

