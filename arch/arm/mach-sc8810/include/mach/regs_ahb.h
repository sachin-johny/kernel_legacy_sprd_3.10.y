/*
 * arch/arm/mach-sc8800s/include/mach/regs_ahb.h
 *
 * Chip AHB  registers Definitions
 *
 * Copyright (C) 2010 Spreadtrum International Ltd.
 *
 * 2010-05-25: yingchun li <yingchun.li@spreadtrum.com>
 *            initial version
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _SC8800H_REG_AHB_H_
#define _SC8800H_REG_AHB_H_

#include <mach/hardware.h>
#include <mach/bits.h>

#define AHB_REG_BASE                (SPRD_AHB_BASE+0x200)
#define CHIP_TYPE                   (SPRD_AHB_BASE + 0x03FC)    //0x209003FC

#define AHB_CTL0                (AHB_REG_BASE + 0x00)
#define AHB_CTL1                (AHB_REG_BASE + 0x04)
#define AHB_CTL2                (AHB_REG_BASE + 0x08)
#define AHB_CTL3                (AHB_REG_BASE + 0x0C)
#define AHB_SOFT_RST            (AHB_REG_BASE + 0x10)
#define AHB_PAUSE               (AHB_REG_BASE + 0x14)
#define AHB_REMAP               (AHB_REG_BASE + 0x18)
#define AHB_ARM_CLK             (AHB_REG_BASE + 0x24)
#define AHB_SDIO_CTL            (AHB_REG_BASE + 0x28)
#define AHB_CTL4                (AHB_REG_BASE + 0x2C)
#define AHB_ENDIAN_SEL          (AHB_REG_BASE + 0x30)
#define AHB_STS                 (AHB_REG_BASE + 0x34)
#define AHB_CA5_CFG	(AHB_REG_BASE + 0x38)
#define DSP_BOOT_EN             (AHB_REG_BASE + 0x84)
#define DSP_BOOT_VEC            (AHB_REG_BASE + 0x88)
#define DSP_RST                 (AHB_REG_BASE + 0x8C)
#define AHB_ENDIAN_EN           (AHB_REG_BASE + 0x90)
#define USB_PHY_CTRL            (AHB_REG_BASE + 0xA0)
#define USB_SPR_REG             (AHB_REG_BASE + 0xC0)

#define CHIP_ID                 (AHB_REG_BASE + 0x1FC)

#define AHB_DSP_BOOT_EN             (AHB_REG_BASE + 0x84)
#define AHB_DSP_BOOT_VECTOR         (AHB_REG_BASE + 0x88)
#define AHB_DSP_RESET               (AHB_REG_BASE + 0x8C)
#define AHB_BIGEND_PROT_REG    (AHB_REG_BASE + 0x90)

#define AHB_CTL0_DCAM_EN   BIT_1
#define AHB_CTL0_CCIR_EN    BIT_2
#define AHB_CTL0_LCDC_EN    BIT_3
#define AHB_CTL0_SDIO_EN    BIT_4
#define AHB_CTL0_SDIO0_EN    AHB_CTL0_SDIO_EN
#define AHB_CTL0_USBD_EN    BIT_5
#define AHB_CTL0_DMA_EN     BIT_6
#define AHB_CTL0_BM0_EN     (BIT_7)
#define AHB_CTL0_NFC_EN      BIT_8
#define AHB_CTL0_BM1_EN     (BIT_11)
#define AHB_CTL0_VSP_EN      BIT_13
#define AHB_CTL0_ROT_EN      BIT_14
#define AHB_CTL0_DRM_EN     BIT_18
#define AHB_CTL0_SDIO1_EN    BIT_19
#define AHB_CTL0_G2D_EN    BIT_20
#define AHB_CTL0_G3D_EN    BIT_21
#define AHB_CTL0_AHB_ARCH_EB   BIT_15
#define AHB_CTL0_EMC_EN   BIT_28
#define AHB_CTL0_AXIBUSMON0_EN   BIT_29
#define AHB_CTL0_AXIBUSMON1_EN   BIT_30

#define AHB_BIGENDIAN_DMA           BIT_0
#define AHB_BIGENDIAN_NFC						BIT_1
#define AHB_BIGENDIAN_LCDC					BIT_2
#define AHB_BIGENDIAN_SDIO					BIT_3
#define AHB_BIGENDIAN_DCAM					BIT_4
#define AHB_BIGENDIAN_VSP						BIT_5
#define AHB_BIGENDIAN_ROT						BIT_6
#define AHB_BIGENDIAN_BM0						BIT_7
#define AHB_BIGENDIAN_BM1						BIT_8
#define AHB_BIGENDIAN_SHARM					BIT_9

#define AHB_ENDIAN_OPEN             0xC3D4
// Bit define AHB_CTRL1
#define AHB_CTRL1_EMC_AUTO_GATE_EN BIT_8
#define AHB_CTRL1_EMC_CH_AUTO_GATE_EN  BIT_9
#define AHB_CTRL1_ARM_AUTO_GATE_EN  BIT_11
#define AHB_CTRL1_AHB_AUTO_GATE_EN  BIT_12
#define AHB_CTRL1_MCU_AUTO_GATE_EN  BIT_13
#define AHB_CTRL1_MSTMTX_AUTO_GATE_EN  BIT_14
#define AHB_CTRL1_ARMMTX_AUTO_GATE_EN  BIT_15
#define AHB_CTRL1_ARM_DAHB_SLEEP_EN  BIT_16

///USB_PHY_CTRL
#define USB_DM_PULLUP_BIT       BIT_19
#define USB_DP_PULLDOWN_BIT     BIT_20
#define USB_DM_PULLDOWN_BIT     BIT_21


/**************************************** 
  * add bit definitions used by power management. 
  *
  *  Wang liwei. 2011-01-18
  *
  ****************************************/

/* bit definitions for register DSP_BOOT_EN */
#define	DSP_BOOT_ENABLE		(0x1UL << 0)

/* bit definitions for register DSP_RST */
#define	DSP_RESET		(0x1UL << 0)

/* bit definitions for register AHB_PAUSE */
#define	MCU_CORE_SLEEP		(0x1UL << 0)
#define	MCU_SYS_SLEEP_EN		(0x1UL << 1)
#define	MCU_DEEP_SLEEP_EN		(0x1UL << 2)
#define	APB_SLEEP		(0x1UL << 3)


/* bit definitions for register AHB_STS */
#define	EMC_STOP_CH0			(0x1UL << 0)
#define	EMC_STOP_CH1			(0x1UL << 1)
#define	EMC_STOP_CH2			(0x1UL << 2)
#define	EMC_STOP_CH3			(0x1UL << 3)
#define	EMC_STOP_CH4			(0x1UL << 4)
#define	EMC_STOP_CH5			(0x1UL << 5)
#define	EMC_STOP_CH6			(0x1UL << 6)
#define	EMC_STOP_CH7			(0x1UL << 7)
#define	EMC_STOP_CH8			(0x1UL << 8)

#define	ARMMTX_STOP_CH0		(0x1UL << 12)
#define	ARMMTX_STOP_CH1		(0x1UL << 13)
#define	ARMMTX_STOP_CH2		(0x1UL << 14)

#define	AHB_STS_EMC_STOP		(0x1UL << 16)
#define	AHB_STS_EMC_SLEEP		(0x1UL << 17)
#define	DMA_BUSY		(0x1UL << 18)
#define	DSP_MAHB_SLEEP_EN		(0x1UL << 19)
#define	APB_PRI_EN		(0x1UL << 20)


/* chip id metafixed for register CHIP_ID or CHIP_TYPE */
#define CHIP_ID_VER_0		(0x88100000UL)
#define CHIP_ID_VER_MF		(0x88100040UL)

/*******************************************
  *
  *******************************************/

#endif 

