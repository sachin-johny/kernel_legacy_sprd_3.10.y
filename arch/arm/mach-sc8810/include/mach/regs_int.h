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

#ifndef _SC8800H_REG_INT_H_
#define _SC8800H_REG_INT_H_

#include <mach/hardware.h>
#include <mach/bits.h>

//#define INT_REG_BASE            		0x20a00000
#define INT_REG_BASE        SPRD_INTCV_BASE

#define INT_IRQ_BASE                    (INT_REG_BASE + 0x0000)
#define INT_IRQ_STS                     (INT_REG_BASE + 0x0000) //Interrupt status after masked by irq_enable.
#define INT_IRQ_RAW_STS                 (INT_REG_BASE + 0x0004) //Interrupt status from different interrupt source.
#define INT_IRQ_EN                      (INT_REG_BASE + 0x0008) //Enable bits  for the corresponding interrupt sources. 
#define INT_IRQ_DISABLE                 (INT_REG_BASE + 0x000C)
#define INT_IRQ_SOFT                    (INT_REG_BASE + 0x0010)
#define INT_IRQ_TEST_SRC                (INT_REG_BASE + 0x0014)
#define INT_IRQ_TEST_SEL                (INT_REG_BASE + 0x0018)
#define INT_IRQ_UINT_STS                (INT_REG_BASE + 0x001C)
#define INT_FIQ_STS                     (INT_REG_BASE + 0x0020)
#define INT_FIQ_RAW_STS                 (INT_REG_BASE + 0x0024)
#define INT_FIQ_EN                      (INT_REG_BASE + 0x0028)
#define INT_FIQ_DISABLE                 (INT_REG_BASE + 0x002C)
#define INT_FIQ_SOFT                    (INT_REG_BASE + 0x0030)
#define INT_FIQ_TEST_SRC                (INT_REG_BASE + 0x0034)
#define INT_FIQ_TEST_SEL                (INT_REG_BASE + 0x0038)
#define INT_UINT_CTL                    (INT_REG_BASE + 0x003C)

#define INTCTL_SPECIAL_LATCH_IRQ            BIT_0
#define INTCTL_SOFT_IRQ                     BIT_1
#define INTCTL_UART0_IRQ                    BIT_2
#define INTCTL_UART1_IRQ                    BIT_3
#define INTCTL_UART2_IRQ                    BIT_4
#define INTCTL_TIMER0_IRQ                   BIT_5
#define INTCTL_TIMER1_IRQ                   BIT_6
#define INTCTL_TIMER2_IRQ                   BIT_7
#define INTCTL_COMMTX                       BIT_7
#define INTCTL_GPIO_IRQ                     BIT_8
#define INTCTL_SPI_IRQ                      BIT_9
#define INTCTL_KPD_IRQ                      BIT_10
#define INTCTL_I2C_IRQ                      BIT_11
#define INTCTL_SIM0_IRQ                     BIT_12
#define INTCTL_SIM1_IRQ                     BIT_12
#define INTCTL_PIU_SER_INT_IRQ              BIT_13
#define INTCTL_PIU_CR_HINT_IRQ              BIT_14
#define INTCTL_DSP_IRQ0                     BIT_15
#define INTCTL_DSP_IRQ1                     BIT_16
#define INTCTL_SYST_IRQ                     BIT_17
#define INTCTL_EPT_IRQ                      BIT_18
#define INTCTL_IIS_IRQ                      BIT_19
#define INTCTL_DSP_INT_OR_IRQ               BIT_20
#define INTCTL_DMA_IRQ                      BIT_21
#define INTCTL_VBC_IRQ                      BIT_22
#define INTCTL_VSP_IRQ                      BIT_23
#define INTCTL_ANA_DIE_IRQ                  BIT_24
#define INTCTL_ADI_IRQ                      BIT_25
#define INTCTL_USB_IRQ                      BIT_26
#define INTCTL_DCAM_IRQ                     BIT_27
#define INTCTL_NFC_IRQ                      BIT_28
#define INTCTL_LCDC_IRQ                     BIT_29
#define INTCTL_DRM_IRQ                      BIT_30
#define INTCTL_SDIO_IRQ                     BIT_30
#define INTCTL_BUS_MON_IRQ                  BIT_31
#define INTCTL_BUS_MON0_IRQ                 BIT_31
#define INTCTL_BUS_MON1_IRQ                 BIT_31
#define INTCTL_COMMRX_IRQ                   BIT_31

#define INTCTL_PCM_IRQ                      INTCTL_IIS_IRQ
#define INTCTL_ICLR_ALL                     0xFFFFFFFF

#endif
