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

#ifndef __ASM_ARM_ARCH_GLOBALREGS_H
#define __ASM_ARM_ARCH_GLOBALREGS_H

#include <mach/hardware.h>
#include <mach/bits.h>

/* general global register offset */
#define GR_GEN0			0x0008
#define GR_PCTL			0x000C
#define GR_IRQ			0x0010
#define GR_ICLR			0x0014
#define GR_GEN1			0x0018
#define GR_GEN3			0x001C
#define GR_BOOT_FLAG		0x0020  /* GR_HWRST */
#define GR_MPLL_MN		0x0024
#define GR_PIN_CTL		0x0028
#define GR_GEN2			0x002C
#define GR_ARM_BOOT_ADDR	0x0030
#define GR_STC_STATE		0x0034
#define GR_DPLL_CTRL		0x0040
#define GR_BUSCLK		0x0044  /* GR_BUSCLK_ALM */
#define GR_ARCH_CTL		0x0048
#define GR_SOFT_RST		0x004C  /* GR_SOFT_RST */
#define GR_NFC_MEM_DLY		0x0058
#define GR_CLK_DLY		0x005C
#define GR_GEN4			0x0060
#define GR_POWCTL0		0x0068
#define GR_POWCTL1		0x006C
#define GR_PLL_SCR		0x0070
#define GR_CLK_EN		0x0074
#define GR_CLK_GEN5		0x007C
#define GR_GPU_PWR_CTRL		0x0080
#define GR_MM_PWR_CTRL		0x0084
#define GR_CEVA_RAM_TH_PWR_CTRL	0x0088
#define GR_GSM_PWR_CTRL		0x008C
#define GR_TD_PWR_CTRL		0x0090
#define GR_PERI_PWR_CTRL	0x0094
#define GR_CEVA_RAM_BH_PWR_CTRL	0x0098
#define GR_ARM_SYS_PWR_CTRL	0x009C
#define GR_G3D_PWR_CTRL		0x00A0

#define GR_SWRST		GR_SOFT_RST
#define GR_BUSCLK_ALM		GR_BUSCLK

/* the GEN0 register bit */
#define GEN0_TIMER_EN		BIT_2
#define GEN0_SIM0_EN		BIT_3
#define GEN0_I2C_EN		BIT_4
#define GEN0_I2C0_EN		GEN0_I2C_EN
#define GEN0_GPIO_EN		BIT_5
#define GEN0_ADI_EN		BIT_6
#define GEN0_EFUSE_EN		BIT_7
#define GEN0_KPD_EN		BIT_8
#define GEN0_EIC_EN		BIT_9
#define GEN0_MCU_DSP_RST	BIT_10
#define GEN0_MCU_SOFT_RST	BIT_11
#define GEN0_I2S_EN		BIT_12
#define GEN0_I2S0_EN		GEN0_I2S_EN
#define GEN0_PIN_EN		BIT_13
#define GEN0_CCIR_MCLK_EN	BIT_14
#define GEN0_EPT_EN		BIT_15
#define GEN0_SIM1_EN		BIT_16
#define GEN0_SPI_EN		BIT_17
#define GEN0_SPI0_EN		GEN0_SPI_EN
#define GEN0_SPI1_EN		BIT_18
#define GEN0_SYST_EN		BIT_19
#define GEN0_UART0_EN		BIT_20
#define GEN0_UART1_EN		BIT_21
#define GEN0_UART2_EN		BIT_22
#define GEN0_VB_EN		BIT_23
#define GEN0_GPIO_RTC_EN	BIT_24
#define GEN0_EIC_RTC_EN		GEN0_GPIO_RTC_EN
#define GEN0_I2S1_EN		BIT_25
#define GEN0_KPD_RTC_EN		BIT_26
#define GEN0_SYST_RTC_EN	BIT_27
#define GEN0_TMR_RTC_EN		BIT_28
#define GEN0_I2C1_EN		BIT_29
#define GEN0_I2C2_EN		BIT_30
#define GEN0_I2C3_EN		BIT_31

/* GR_PCTL */
#define MCU_MPLL_EN		BIT_1

/* GR_GEN1 */
#define GEN1_MPLL_MN_EN		BIT_9
#define GEN1_CLK_AUX0_EN	BIT_10
#define GEN1_CLK_AUX1_EN	BIT_11
#define GEN1_RTC_ARCH_EN	BIT_18

/* the APB Soft Reset register bit */
#define SWRST_I2C_RST		BIT_0
#define SWRST_KPD_RST		BIT_1
#define SWRST_SIM0_RST		BIT_5
#define SWRST_SIM1_RST		BIT_6
#define SWRST_TIMER_RST		BIT_8
#define SWRST_EPT_RST		BIT_10
#define SWRST_UART0_RST		BIT_11
#define SWRST_UART1_RST		BIT_12
#define SWRST_UART2_RST		BIT_13
#define SWRST_SPI_RST		BIT_14
#define SWRST_IIS_RST		BIT_16
#define SWRST_SYST_RST		BIT_19
#define SWRST_PINREG_RST	BIT_20
#define SWRST_GPIO_RST		BIT_21
#define ADI_SOFT_RST		BIT_22
#define SWRST_VBC_RST		BIT_23
#define SWRST_PWM0_RST		BIT_24
#define SWRST_PWM1_RST		BIT_25
#define SWRST_PWM2_RST		BIT_26
#define SWRST_PWM3_RST		BIT_27
#define SWRST_EFUSE_RST		BIT_28

/* the ARM VB CTRL register bit */
#define ARM_VB_IIS_SEL		BIT_0
#define ARM_VB_MCLKON		BIT_1
#define ARM_VB_DA0ON		BIT_2
#define ARM_VB_DA1ON		BIT_3
#define ARM_VB_AD0ON		BIT_4
#define ARM_VB_AD1ON		BIT_5
#define ARM_VB_ANAON		BIT_6
#define ARM_VB_ACC		BIT_7
#define ARM_VB_ADCON		ARM_VB_AD0ON

/* the Interrupt control register bit */
#define IRQ_MCU_IRQ0		BIT_0
#define IRQ_MCU_FRQ0		BIT_1
#define IRQ_MCU_IRQ1		BIT_2
#define IRQ_MCU_FRQ1		BIT_3
#define IRQ_VBCAD_IRQ		BIT_5
#define IRQ_VBCDA_IRQ		BIT_6
#define IRQ_RFT_INT		BIT_12

/* the Interrupt clear register bit */
#define ICLR_DSP_IRQ0_CLR	BIT_0
#define ICLR_DSP_FRQ0_CLR	BIT_1
#define ICLR_DSP_IRQ1_CLR	BIT_2
#define ICLR_DSP_FIQ1_CLR	BIT_3
#define ICLR_VBCAD_IRQ_CLR	BIT_5
#define ICLR_VBCDA_IRQ_CLR	BIT_6
#define ICLR_RFT_INT_CLR	BIT_12

/* the Clock enable register bit */
#define CLK_PWM0_EN		BIT_21
#define CLK_PWM1_EN		BIT_22
#define CLK_PWM2_EN		BIT_23
#define CLK_PWM3_EN		BIT_24
#define CLK_PWM0_SEL		BIT_25
#define CLK_PWM1_SEL		BIT_26
#define CLK_PWM2_SEL		BIT_27
#define CLK_PWM3_SEL		BIT_28

/* POWER CTL1 */
#define POWCTL1_CONFIG		0x0423F91E  /* isolation number 1ms:30cycles */

/* bits definition for CLK_EN. */
#define	MCU_XTLEN_AUTOPD_EN	BIT_18
#define	APB_PERI_FRC_CLP	BIT_19

/* bits definition for GR_STC_STATE. */

#define	GR_EMC_STOP		BIT_0
#define	GR_MCU_STOP		BIT_1
#define	GR_DSP_STOP		BIT_2

/* bits definition for GR_CLK_DLY. */
#define	GR_EMC_STOP_CH5		BIT_4
#define	GR_EMC_STOP_CH4		BIT_5
#define	GR_EMC_STOP_CH3		BIT_6
#define	DSP_DEEP_STOP		BIT_9
#define	DSP_SYS_STOP		BIT_10
#define	DSP_AHB_STOP		BIT_11
#define	DSP_MTX_STOP		BIT_12
#define	DSP_CORE_STOP		BIT_13

/* ****************************************************************** */

/* AHB register offset */
#define AHB_CTL0		0x00
#define AHB_CTL1		0x04
#define AHB_CTL2		0x08
#define AHB_CTL3		0x0C
#define AHB_SOFT_RST		0x10
#define AHB_PAUSE		0x14
#define AHB_REMAP		0x18
#define AHB_ARM_CLK		0x24
#define AHB_SDIO_CTL		0x28
#define AHB_CTL4		0x2C
#define AHB_ENDIAN_SEL		0x30
#define AHB_STS			0x34
#define AHB_CA5_CFG		0x38
#define AHB_DSP_BOOT_EN		0x84
#define AHB_DSP_BOOT_VECTOR	0x88
#define AHB_DSP_RESET		0x8C
#define AHB_ENDIAN_EN		0x90
#define USB_PHY_CTRL		0xA0

/* AHB_CTL0 bits */
#define AHB_CTL0_DCAM_EN	BIT_1
#define AHB_CTL0_CCIR_EN	BIT_2
#define AHB_CTL0_LCDC_EN	BIT_3
#define AHB_CTL0_SDIO_EN	BIT_4
#define AHB_CTL0_SDIO0_EN	AHB_CTL0_SDIO_EN
#define AHB_CTL0_USBD_EN	BIT_5
#define AHB_CTL0_DMA_EN		BIT_6
#define AHB_CTL0_BM0_EN		BIT_7
#define AHB_CTL0_NFC_EN		BIT_8
#define AHB_CTL0_BM1_EN		BIT_11
#define AHB_CTL0_VSP_EN		BIT_13
#define AHB_CTL0_ROT_EN		BIT_14
#define AHB_CTL0_DRM_EN		BIT_18
#define AHB_CTL0_SDIO1_EN	BIT_19
#define AHB_CTL0_G2D_EN		BIT_20
#define AHB_CTL0_G3D_EN		BIT_21
#define AHB_CTL0_AHB_ARCH_EB	BIT_15
#define AHB_CTL0_EMC_EN		BIT_28
#define AHB_CTL0_AXIBUSMON0_EN	BIT_29
#define AHB_CTL0_AXIBUSMON1_EN	BIT_30

/* AHB_CTRL1 bits */
#define AHB_CTRL1_EMC_AUTO_GATE_EN 	BIT_8
#define AHB_CTRL1_EMC_CH_AUTO_GATE_EN	BIT_9
#define AHB_CTRL1_ARM_AUTO_GATE_EN	BIT_11
#define AHB_CTRL1_AHB_AUTO_GATE_EN	BIT_12
#define AHB_CTRL1_MCU_AUTO_GATE_EN	BIT_13
#define AHB_CTRL1_MSTMTX_AUTO_GATE_EN	BIT_14
#define AHB_CTRL1_ARMMTX_AUTO_GATE_EN	BIT_15
#define AHB_CTRL1_ARM_DAHB_SLEEP_EN	BIT_16

/* USB_PHY_CTRL bits */
#define USB_DM_PULLUP_BIT	BIT_19
#define USB_DP_PULLDOWN_BIT	BIT_20
#define USB_DM_PULLDOWN_BIT	BIT_21

/* bit definitions for register DSP_BOOT_EN */
#define	DSP_BOOT_ENABLE		BIT_0

/* bit definitions for register DSP_RST */
#define	DSP_RESET		BIT_0

/* bit definitions for register AHB_PAUSE */
#define	MCU_CORE_SLEEP		BIT_0
#define	MCU_SYS_SLEEP_EN	BIT_1
#define	MCU_DEEP_SLEEP_EN	BIT_2
#define	APB_SLEEP		BIT_3


/* bit definitions for register AHB_STS */
#define	EMC_STOP_CH0		BIT_0
#define	EMC_STOP_CH1		BIT_1
#define	EMC_STOP_CH2		BIT_2
#define	EMC_STOP_CH3		BIT_3
#define	EMC_STOP_CH4		BIT_4
#define	EMC_STOP_CH5		BIT_5
#define	EMC_STOP_CH6		BIT_6
#define	EMC_STOP_CH7		BIT_7
#define	EMC_STOP_CH8		BIT_8
#define	ARMMTX_STOP_CH0		BIT_12
#define	ARMMTX_STOP_CH1		BIT_13
#define	ARMMTX_STOP_CH2		BIT_14
#define	AHB_STS_EMC_STOP	BIT_16
#define	AHB_STS_EMC_SLEEP	BIT_17
#define	DMA_BUSY		BIT_18
#define	DSP_MAHB_SLEEP_EN	BIT_19
#define	APB_PRI_EN		BIT_20

/* ****************************************************************** */

/* global register types */
enum {
	REG_TYPE_GLOBAL = 0,
	REG_TYPE_AHB_GLOBAL,
	REG_TYPE_MAX
};

int32_t sprd_greg_read(uint32_t type, uint32_t reg_offset);
void sprd_greg_write(uint32_t type, uint32_t value, uint32_t reg_offset);
void sprd_greg_set_bits(uint32_t type, uint32_t bits, uint32_t reg_offset);
void sprd_greg_clear_bits(uint32_t type, uint32_t bits, uint32_t reg_offset);

#endif

