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

#define INT_IRQ_BASE            		(INT_REG_BASE + 0x0000)
#define INT_FIQ_STS             		(INT_REG_BASE + 0x0000)
#define INT_IRQ_STS             		(INT_REG_BASE + 0x0004)
#define INT_IRQ_RAW_STS         		(INT_REG_BASE + 0x0008)
#define INT_IRQ_SEL         			(INT_REG_BASE + 0x000c)
#define INT_IRQ_EN              		(INT_REG_BASE + 0x0010)
#define INT_IRQ_DISABLE         		(INT_REG_BASE + 0x0014) 
#define INT_IRQ_SOFT            		(INT_REG_BASE + 0x0018)
#define INT_IRQ_SOFT_CLR           		(INT_REG_BASE + 0x001C)
#define INT_IRQ_PROTECT            		(INT_REG_BASE + 0x0020)
#define INT_VADDR                       (INT_REG_BASE + 0x0024)
#define INT_DEFVADDR                    (INT_REG_BASE + 0x0028)
#define INT_VCTL_BASE                   (INT_REG_BASE + 0x0030)
#define INT_VADDR_BASE                  (INT_REG_BASE + 0x006C)
#define INT_IRQFIQ_UARTSTS              (INT_REG_BASE + 0x00AC)
#define INT_PMASK                       (INT_REG_BASE + 0x00B0)
#define INT_VADDR2                      (INT_REG_BASE + 0x00B4)
#define INT_UART_INT_CFG                (INT_REG_BASE + 0x00B8)
#define INT_INTEN_DSP                   (INT_REG_BASE + 0x00BC)


//The corresponding bit of all INT_CTL registers.
#define INTCTL_UART_SLEEP_IRQ			(1 << 0)
#define INTCTL_SDIO_IRQ         		(1 << 1)
#define INTCTL_COMMRX           		(1 << 2)
#define INTCTL_COMMTX           		(1 << 3)
#define INTCTL_CNT1_IRQ         		(1 << 4)
#define INTCTL_CNT2_IRQ         		(1 << 5)
#define INTCTL_GPIO_IRQ         		(1 << 6)
#define INTCTL_RTC_IRQ          		(1 << 7)
#define INTCTL_KPD_IRQ          		(1 << 8)
#define INTCTL_I2C_IRQ          		(1 << 9)
#define INTCTL_SIM_IRQ          		(1 << 10)
#define INTCTL_CX_SEM_HINT_IRQ          (1 << 11)
#define INTCTL_CX_CR_HINT_IRQ        	(1 << 12)
#define INTCTL_DSP_IRQ          		(1 << 13)
#define INTCTL_ADC_IRQ          		(1 << 14)
#define INTCTL_GEA_POOL_IRQ     		(1 << 15)
#define INTCTL_SYST_IRQ         		(1 << 16)
#define INTCTL_RFT_IRQ         			(1 << 17)
#define INTCTL_UART2_3_IRQ        		(1 << 18)
#define INTCTL_UART2_IRQ        		(1 << 18)
#define INTCTL_UART3_IRQ        		(1 << 18)
#define INTCTL_DSP_INT_OR_IRQ        	(1 << 19)
#define INTCTL_DMA_IRQ        			(1 << 20)
#define INTCTL_VBC_IRQ        			(1 << 21)
#define INTCTL_MEA_IRQ        			(1 << 22)
#define INTCTL_DCT_IRQ        			(1 << 23)
#define INTCTL_UART0_1_IRQ        		(1 << 24)
#define INTCTL_UART0_IRQ        		(1 << 24)
#define INTCTL_UART1_IRQ        		(1 << 24)
#define INTCTL_USBD_IRQ        			(1 << 25)
#define INTCTL_ISP_IRQ        			(1 << 26)
#define INTCTL_NLC_IRQ        			(1 << 27)
#define INTCTL_LCDC_IRQ        			(1 << 28)
#define INTCTL_PCM_IRQ        			(1 << 29)
#define INTCTL_DRM_IRQ        			(1 << 30)
#define INTCTL_PBINT_IRQ       			(1 << 31)
#define INTCTL_ICLR_ALL         		0xFFFFFFFF

#endif
