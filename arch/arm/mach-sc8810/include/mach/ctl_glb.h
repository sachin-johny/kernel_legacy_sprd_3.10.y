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

#ifndef __CTL_GLB_H__
#define __CTL_GLB_H__

#define CTL_GLB

/* registers definitions for controller CTL_GLB */
#define REG_GLB_RESERVED0               SCI_ADDRESS(CTL_GLB_BASE, 0x0000)
#define REG_GLB_RESERVED1               SCI_ADDRESS(CTL_GLB_BASE, 0x0004)
#define REG_GLB_GEN0                    SCI_ADDRESS(CTL_GLB_BASE, 0x0008)
/* Peripheral control */
#define REG_GLB_PCTRL                   SCI_ADDRESS(CTL_GLB_BASE, 0x000c)
#define REG_GLB_INT_CTRL                SCI_ADDRESS(CTL_GLB_BASE, 0x0010)
#define REG_GLB_INT_CLR                 SCI_ADDRESS(CTL_GLB_BASE, 0x0014)
#define REG_GLB_GEN1                    SCI_ADDRESS(CTL_GLB_BASE, 0x0018)
#define REG_GLB_GEN3                    SCI_ADDRESS(CTL_GLB_BASE, 0x001c)
#define REG_GLB_BOOT_FLG                SCI_ADDRESS(CTL_GLB_BASE, 0x0020)
#define REG_GLB_MPLL_CTRL0              SCI_ADDRESS(CTL_GLB_BASE, 0x0024)
#define REG_GLB_PIN_CTRL                SCI_ADDRESS(CTL_GLB_BASE, 0x0028)
#define REG_GLB_GEN2                    SCI_ADDRESS(CTL_GLB_BASE, 0x002c)
#define REG_GLB_ARM_BOOT_ADDR           SCI_ADDRESS(CTL_GLB_BASE, 0x0030)
#define REG_GLB_STC_ST                  SCI_ADDRESS(CTL_GLB_BASE, 0x0034)
#define REG_GLB_MPLL_CTRL1              SCI_ADDRESS(CTL_GLB_BASE, 0x0038)
#define REG_GLB_TDPLL_CTRL              SCI_ADDRESS(CTL_GLB_BASE, 0x003c)
#define REG_GLB_DPLL_CTRL               SCI_ADDRESS(CTL_GLB_BASE, 0x0040)
#define REG_GLB_BUSCLK                  SCI_ADDRESS(CTL_GLB_BASE, 0x0044)
#define REG_GLB_ARCH_CTRL               SCI_ADDRESS(CTL_GLB_BASE, 0x0048)
#define REG_GLB_SOFT_RST                SCI_ADDRESS(CTL_GLB_BASE, 0x004c)
#define REG_GLB_RESERVED2               SCI_ADDRESS(CTL_GLB_BASE, 0x0050)
#define REG_GLB_RESERVED3               SCI_ADDRESS(CTL_GLB_BASE, 0x0054)
#define REG_GLB_NFC_DLY_CTRL            SCI_ADDRESS(CTL_GLB_BASE, 0x0058)
#define REG_GLB_CLK_DLY_CTRL            SCI_ADDRESS(CTL_GLB_BASE, 0x005c)
#define REG_GLB_GEN4                    SCI_ADDRESS(CTL_GLB_BASE, 0x0060)
#define REG_GLB_RESERVED4               SCI_ADDRESS(CTL_GLB_BASE, 0x0064)
#define REG_GLB_PWR_CTRL0               SCI_ADDRESS(CTL_GLB_BASE, 0x0068)
#define REG_GLB_PWR_CTRL1               SCI_ADDRESS(CTL_GLB_BASE, 0x006c)
#define REG_GLB_PLL_SCR                 SCI_ADDRESS(CTL_GLB_BASE, 0x0070)
#define REG_GLB_CLK_EN                  SCI_ADDRESS(CTL_GLB_BASE, 0x0074)
#define REG_GLB_CLK26_ANA_CTRL          SCI_ADDRESS(CTL_GLB_BASE, 0x0078)
#define REG_GLB_CLK_GEN5                SCI_ADDRESS(CTL_GLB_BASE, 0x007c)
#define REG_GLB_RESERVED5               SCI_ADDRESS(CTL_GLB_BASE, 0x0080)
#define REG_GLB_MM_PWR_CTRL             SCI_ADDRESS(CTL_GLB_BASE, 0x0084)
#define REG_GLB_CEVA_RAM_TH_PWR_CTRL    SCI_ADDRESS(CTL_GLB_BASE, 0x0088)
#define REG_GLB_GSM_PWR_CTRL            SCI_ADDRESS(CTL_GLB_BASE, 0x008c)
#define REG_GLB_TD_PWR_CTRL             SCI_ADDRESS(CTL_GLB_BASE, 0x0090)
#define REG_GLB_PERI_PWR_CTRL           SCI_ADDRESS(CTL_GLB_BASE, 0x0094)
#define REG_GLB_CEVA_RAM_BH_PWR_CTRL    SCI_ADDRESS(CTL_GLB_BASE, 0x0098)
#define REG_GLB_ARM_SYS_PWR_CTRL        SCI_ADDRESS(CTL_GLB_BASE, 0x009c)
#define REG_GLB_G3D_PWR_CTRL            SCI_ADDRESS(CTL_GLB_BASE, 0x00a0)

/* bits definitions for register REG_GLB_GEN0 */
#define BIT_IC3_EB                      ( BIT(31) )
#define BIT_IC2_EB                      ( BIT(30) )
#define BIT_IC1_EB                      ( BIT(29) )
#define BIT_RTC_TMR_EB                  ( BIT(28) )
#define BIT_RTC_SYST0_EB                ( BIT(27) )
#define BIT_RTC_KPD_EB                  ( BIT(26) )
#define BIT_IIS1_EB                     ( BIT(25) )
#define BIT_RTC_EIC_EB                  ( BIT(24) )
#define BIT_VB_EB                       ( BIT(23) )
#define BIT_UART2_EB                    ( BIT(22) )
#define BIT_UART1_EB                    ( BIT(21) )
#define BIT_UART0_EB                    ( BIT(20) )
#define BIT_SYST0_EB                    ( BIT(19) )
#define BIT_SPI1_EB                     ( BIT(18) )
#define BIT_SPI0_EB                     ( BIT(17) )
#define BIT_SIM1_EB                     ( BIT(16) )
#define BIT_EPT_EB                      ( BIT(15) )
#define BIT_CCIR_MCLK_EN                ( BIT(14) )
#define BIT_PINREG_EB                   ( BIT(13) )
#define BIT_IIS0_EB                     ( BIT(12) )
/* MCU soft reset the whole MCU sub-system, processor core, AHB and APB.
it will be self-cleard to zero after set.
 */
#define BIT_MCU_SOFT_RST                ( BIT(11) )
/* MCU soft reset DSP Z-bus Accelerators. it will be self-cleared to zero after set */
#define BIT_MCU_DSP_RST                 ( BIT(10) )
#define BIT_EIC_EB                      ( BIT(9) )
#define BIT_KPD_EB                      ( BIT(8) )
#define BIT_EFUSE_EB                    ( BIT(7) )
#define BIT_ADI_EB                      ( BIT(6) )
#define BIT_GPIO_EB                     ( BIT(5) )
#define BIT_I2C0_EB                     ( BIT(4) )
#define BIT_SIM0_EB                     ( BIT(3) )
#define BIT_TMR_EB                      ( BIT(2) )
#define BITS_RESERVED_EB(_x_)           ( (_x_) << 0 & (BIT(0)|BIT(1)) )

/* bits definitions for register REG_GLB_PCTRL */
#define BIT_IIS0_SEL                    ( BIT(31) )
#define BIT_IIS1_SEL                    ( BIT(30) )
#define BITS_CLK_AUX1_DIV(_x_)          ( (_x_) << 22 & (BIT(22)|BIT(23)|BIT(24)|BIT(25)|BIT(26)|BIT(27)|BIT(28)|BIT(29)) )
#define BIT_ROM_CLK_EN                  ( BIT(10) )
/* All clock gatings will be invalid ,ad then all clock enable, for debug use */
#define BIT_ALL_CLK_EN                  ( BIT(9) )
/* Owner selection for UART1, <0: ARM control, 1: DSP control> */
#define BIT_UART1_SEL                   ( BIT(8) )
/* Flag indicating MPLL is stable, active high. Only for SW debug. */
#define BIT_MPLL_CNT_DONE               ( BIT(7) )
#define BIT_TDPLL_CNT_DONE              ( BIT(6) )
#define BIT_DPLL_CNT_DONE               ( BIT(5) )
#define BIT_ARM_JTAG_EN                 ( BIT(4) )
#define BIT_MCU_DPLL_EN                 ( BIT(3) )
#define BIT_MCU_TDPLL_EN                ( BIT(2) )
#define BIT_MCU_MPLL_EN                 ( BIT(1) )
/* MCU force deepsleep. for debug use. */
#define BIT_MCU_FORECE_DEEP_SLEEP       ( BIT(0) )

/* bits definitions for register REG_GLB_GEN1 */
#define BIT_RTC_ARCH_EB                 ( BIT(18) )
#define BIT_CLK_AUX1_EN                 ( BIT(11) )
#define BIT_CLK_AUX0_EN                 ( BIT(10) )
#define BIT_M_PLL_CTRL_WE               ( BIT(9) )
#define BITS_CLK_AUX0_DIV(_x_)          ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)) )

/* bits definitions for register REG_GLB_GEN3 */
#define BITS_CCIR_MCLK_DIV(_x_)         ( (_x_) << 24 & (BIT(24)|BIT(25)) )
#define BIT_JTAG_DAISY_EN               ( BIT(23) )
#define BITS_CLK_IIS1_DIV(_x_)          ( (_x_) << 8 & (BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )
#define BITS_BOND_OPT(_x_)              ( (_x_) << 0 & (BIT(0)|BIT(1)|BIT(2)) )

/* bits definitions for register REG_GLB_BOOT_FLG */
/* Function-undefined register for chip boot use */
#define BITS_BOOT_FLAG(_x_)             ( (_x_) << 8 & (BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)) )

/* bits definitions for register REG_GLB_GEN2 */
#define BITS_CLK_IIS0_DIV(_x_)          ( (_x_) << 24 & (BIT(24)|BIT(25)|BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)|BIT(31)) )
#define BITS_CLK_SPI0_DIV(_x_)          ( (_x_) << 21 & (BIT(21)|BIT(22)|BIT(23)) )
#define BITS_CLK_GPU_2X_DIV(_x_)        ( (_x_) << 17 & (BIT(17)|BIT(18)|BIT(19)) )
#define BITS_CLK_GPU_AXI_DIV(_x_)       ( (_x_) << 14 & (BIT(14)|BIT(15)|BIT(16)) )
#define BITS_CLK_SPI1_DIV(_x_)          ( (_x_) << 11 & (BIT(11)|BIT(12)|BIT(13)) )
#define BITS_CLK_NFC_DIV(_x_)           ( (_x_) << 6 & (BIT(6)|BIT(7)|BIT(8)) )
#define BITS_CLK_NFC_SEL(_x_)           ( (_x_) << 4 & (BIT(4)|BIT(5)) )
#define BITS_CLK_GPU_2X_SEL(_x_)        ( (_x_) << 2 & (BIT(2)|BIT(3)) )
#define BITS_CLK_GPU_AXI_SEL(_x_)       ( (_x_) << 0 & (BIT(0)|BIT(1)) )

/* bits definitions for register REG_GLB_STC_ST */
#define BIT_FRC_WAKE_EN                 ( BIT(5) )
#define BIT_TMR_WAKE_AFC                ( BIT(4) )
#define BIT_SCH_SLEEP                   ( BIT(3) )
#define BIT_DSP_STOP                    ( BIT(2) )
#define BIT_MCU_STOP                    ( BIT(1) )
#define BIT_EMC_STOP                    ( BIT(0) )

/* bits definitions for register REG_GLB_SOFT_RST */
#define BIT_EIC_SOFT_RST                ( BIT(29) )
#define BIT_EFUSE_SOFT_RST              ( BIT(28) )
#define BIT_PWM3_SOFT_RST               ( BIT(27) )
#define BIT_PWM2_SOFT_RST               ( BIT(26) )
#define BIT_PWM1_SOFT_RST               ( BIT(25) )
#define BIT_PWM0_SOFT_RST               ( BIT(24) )
#define BIT_VBC_SOFT_RST                ( BIT(23) )
#define BIT_ADI_SOFT_RST                ( BIT(22) )
#define BIT_GPIO_SOFT_RST               ( BIT(21) )
#define BIT_PINREG_SOFT_RST             ( BIT(20) )
#define BIT_SYST0_SOFT_RST              ( BIT(19) )
#define BIT_RESERVED18                  ( BIT(18) )
#define BIT_IIS1_SOFT_RST               ( BIT(17) )
#define BIT_IIS0_SOFT_RST               ( BIT(16) )
#define BIT_SPI1_SOFT_RST               ( BIT(15) )
#define BIT_SPI0_SOFT_RST               ( BIT(14) )
#define BIT_UART2_SOFT_RST              ( BIT(13) )
#define BIT_UART1_SOFT_RST              ( BIT(12) )
#define BIT_UART0_SOFT_RST              ( BIT(11) )
#define BIT_EPT_SOFT_RST                ( BIT(10) )
#define BIT_RESERVED09                  ( BIT(9) )
#define BIT_TMR_SOFT_RST                ( BIT(8) )
#define BIT_RESERVED07                  ( BIT(7) )
#define BIT_SIM1_SOFT_RST               ( BIT(6) )
#define BIT_SIM0_SOFT_RST               ( BIT(5) )
#define BIT_I2C3_SOFT_RST               ( BIT(4) )
#define BIT_I2C2_SOFT_RST               ( BIT(3) )
#define BIT_I2C1_SOFT_RST               ( BIT(2) )
#define BIT_KPD_SOFT_RST                ( BIT(1) )
#define BIT_I2C0_SOFT_RST               ( BIT(0) )

/* vars definitions for controller CTL_GLB */

#endif //__CTL_GLB_H__
