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

#ifndef __ANA_CTL_GLB_H__
#define __ANA_CTL_GLB_H__

#define ANA_CTL_GLB
#define ANA_CTL_GLB

/* registers definitions for controller ANA_CTL_GLB */
#define ANA_REG_GLB_APB_CLK_EN          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0000)
#define ANA_REG_GLB_APB_ARM_RST         SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0004)
#define ANA_REG_GLB_LDO_PD_SET          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0008)
#define ANA_REG_GLB_LDO_PD_RST          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x000c)
#define ANA_REG_GLB_LDO_PD_CTRL0        SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0010)
#define ANA_REG_GLB_LDO_PD_CTRL1        SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0014)
#define ANA_REG_GLB_LDO_VCTRL0          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0018)
#define ANA_REG_GLB_LDO_VCTRL1          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x001c)
#define ANA_REG_GLB_LDO_VCTRL2          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0020)
#define ANA_REG_GLB_LDO_VCTRL3          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0024)
#define ANA_REG_GLB_LDO_VCTRL4          SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0028)
#define ANA_REG_GLB_LDO_SLP_CTRL0       SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x002c)
#define ANA_REG_GLB_LDO_SLP_CTRL1       SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0030)
#define ANA_REG_GLB_LDO_SLP_CTRL2       SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0034)
#define ANA_REG_GLB_DCDC_CTRL           SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0038)
#define ANA_REG_GLB_DCDC_CTRL_DS        SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x003c)
#define ANA_REG_GLB_DCDC_CTRL_CAL       SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0040)
#define ANA_REG_GLB_DCDCARM_CTRL        SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0044)
#define ANA_REG_GLB_DCDCARM_CTRL_CAL    SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0048)
#define ANA_REG_GLB_PLL_CTRL            SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x004c)
#define ANA_REG_GLB_APLLMN              SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0050)
#define ANA_REG_GLB_APLLWAIT            SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0054)
#define ANA_REG_GLB_RTC_CTRL            SCI_ADDRESS(ANA_CTL_GLB_BASE, 0x0058)

/* bits definitions for register REG_GLB_LDO_PD_SET */
#define BIT_CHGRWDG_EB                  ( BIT(15) )
#define BIT_CLK_AUXAD_EN                ( BIT(14) )
#define BIT_CLK_AUXADC_EN               ( BIT(13) )
#define BIT_RTC_TPC_EB                  ( BIT(12) )
#define BIT_RTC_EIC_EB                  ( BIT(11) )
#define BIT_RTC_WDG_EB                  ( BIT(10) )
#define BIT_RTC_RTC_EB                  ( BIT(9) )
#define BIT_RTC_ARCH_EB                 ( BIT(8) )
#define BIT_PINREG_EB                   ( BIT(7) )
#define BIT_GPIO_EB                     ( BIT(6) )
#define BIT_ADC_EB                      ( BIT(5) )
#define BIT_TPC_EB                      ( BIT(4) )
#define BIT_EIC_EB                      ( BIT(3) )
#define BIT_WDG_EB                      ( BIT(2) )
#define BIT_RTC_EB                      ( BIT(1) )
#define BIT_APB_ARCH_EB                 ( BIT(0) )

/* vars definitions for controller ANA_CTL_GLB */

#endif //__ANA_CTL_GLB_H__
