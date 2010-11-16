/* arch/arm/mach-sc8800g/include/mach/hardware.h
 *
 * Copyright (C) 2010 Spreadtrum
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
 */

#ifndef __ASM_ARCH_SPRD_HARDWARE_H
#define __ASM_ARCH_SPRD_HARDWARE_H

/*
 * SC8800S internal I/O mappings
 *
 * We have the following mapping:
 *      phys            virt
 *      20000000        E0000000
 *      60000000        E0010000
 *      80000000        E0020000/E0030000
 */

/* INTCV control registers */
#define SPRD_INTCV_BASE          0xE0000000
#define SPRD_INTCV_PHYS          0x80003000
#define SPRD_INTCV_SIZE          SZ_4K

/* DMA control registers */
#define SPRD_DMA_BASE            0xE0001000
#define SPRD_DMA_PHYS            0x20100000
#define SPRD_DMA_SIZE            SZ_4K

/* ISP control registers */
#define SPRD_ISP_BASE            0xE0002000
#define SPRD_ISP_PHYS            0x20200000
#define SPRD_ISP_SIZE            SZ_4K

/* USB device space */
//#define SPRD_USB_BASE            0xE0003000
//#define SPRD_USB_PHYS            0x20300000
//#define SPRD_USB_SIZE            SZ_4K

/* DCT control registers */
#define SPRD_DCT_BASE            0xE0004000
#define SPRD_DCT_PHYS            0x20400000
#define SPRD_DCT_SIZE            SZ_4K

/* SDIO control registers */
#define SPRD_SDIO_BASE           0xE0005000
#define SPRD_SDIO_PHYS           0x20500000
#define SPRD_SDIO_SIZE           SZ_4K

/* LCD control registers */
#define SPRD_LCDC_BASE        0xE0008000
#define SPRD_LCDC_PHYS        0x20700000
#define SPRD_LCDC_SIZE        SZ_4K

/* rotation  control registers*/
#define SPRD_ROTO_BASE           0xE0009000
#define SPRD_ROTO_PHYS           0x20800000
#define SPRD_ROTO_SIZE           SZ_4K

/* AHB control registers 
   NOTE: the real AHB phisical address base is 0x20900100, but here
	 we can	only map the address aligned by page size, so the
	 defination of AHB registers should be SPRD_AHB_BASE + 0x100
	 + register offset.
*/

#define SPRD_AHB_BASE            0xE000A000
#define SPRD_AHB_PHYS            0x20900000
#define SPRD_AHB_SIZE            SZ_4K

/* EMC control registers */
#define SPRD_EMC_BASE            0xE000B000
#define SPRD_EMC_PHYS            0x20000000
#define SPRD_EMC_SIZE            SZ_4K

/* DRM control registers */
#define SPRD_DRM_BASE            0xE000C000
#define SPRD_DRM_PHYS            0x20B00000
#define SPRD_DRM_SIZE            SZ_4K

/* MEA control registers */
#define SPRD_MEA_BASE            0xE000D000
#define SPRD_MEA_PHYS            0x20C00000
#define SPRD_MEA_SIZE            SZ_4K

/* NAND & LCM0 */
#define SPRD_NAND_BASE           0xE0010000
#define SPRD_NAND_PHYS           0x60000000
#define SPRD_NAND_SIZE           SZ_8K

/* PCMCIA */
#define SPRD_PCMCIA_BASE         0xE0020000
#define SPRD_PCMCIA_PHYS         0x80000000
#define SPRD_PCMCIA_SIZE         SZ_4K

/* Ceva access ARM */
#define SPRD_ASHB_BASE           0xE0021000
#define SPRD_ASHB_PHYS           0x80000000
#define SPRD_ASHB_SIZE           SZ_4K

/* general RTC timers */
#define SPRD_TIMER_BASE          0xE0022000
#define SPRD_TIMER_PHYS          0x81000000
#define SPRD_TIMER_SIZE          SZ_4K

/* voice band */
#define SPRD_VB_BASE             0xE0023000
#define SPRD_VB_PHYS             0x82003000
#define SPRD_VB_SIZE             SZ_4K

/* serial 0 */
#define SPRD_SERIAL0_BASE        0xE0024000
#define SPRD_SERIAL0_PHYS        0x83000000
#define SPRD_SERIAL0_SIZE        SZ_4K

/* serial 1 */
#define SPRD_SERIAL1_BASE        0xE0025000
#define SPRD_SERIAL1_PHYS        0x84000000
#define SPRD_SERIAL1_SIZE        SZ_4K

/* SIM Card 0 */
#define SPRD_SIM0_BASE           0xE0026000
#define SPRD_SIM0_PHYS           0x85000000
#define SPRD_SIM0_SIZE           SZ_4K

/* SIM Card 1 */
#define SPRD_SIM1_BASE           0xE0027000
#define SPRD_SIM1_PHYS           0x85003000
#define SPRD_SIM1_SIZE           SZ_4K

/* I2C */
#define SPRD_I2C_BASE            0xE0028000
#define SPRD_I2C_PHYS            0x86000000
#define SPRD_I2C_SIZE            SZ_4K

/* keypad and system counter share the same base */
#define SPRD_KPD_BASE            0xE0029000
#define SPRD_KPD_PHYS            0x87000000
#define SPRD_KPD_SIZE            SZ_4K
//#define SPRD_SYSCNT_BASE         SPRD_KPD_BASE

#define SPRD_SYSCNT_BASE            0xE002A000
#define SPRD_SYSCNT_PHYS            0x87003000
#define SPRD_SYSCNT_SIZE            SZ_4K

/* PWM */
#define SPRD_PWM_BASE            0xE002B000
#define SPRD_PWM_PHYS            0x88000000
#define SPRD_PWM_SIZE            SZ_4K

/* real time clock */
#define SPRD_RTC_BASE            0xE002C000
#define SPRD_RTC_PHYS            0x89000000
#define SPRD_RTC_SIZE            SZ_4K

/* watchdog */
#define SPRD_WDG_BASE            0xE002D000
#define SPRD_WDG_PHYS            0x82000040
#define SPRD_WDG_SIZE            SZ_4K

/* GPIO */
#define SPRD_GPIO_BASE           0xE002E000
#define SPRD_GPIO_PHYS           0x8A000000
#define SPRD_GPIO_SIZE           SZ_4K

/* global regsiters */
#define SPRD_GREG_BASE           0xE002F000
#define SPRD_GREG_PHYS           0x8B000000
#define SPRD_GREG_SIZE           SZ_4K

/* chip pin control registers */
#define SPRD_CPC_BASE            0xE0030000
#define SPRD_CPC_PHYS            0x8C000000
#define SPRD_CPC_SIZE            SZ_4K

/* EPT */
#define SPRD_EPT_BASE            0xE0031000
#define SPRD_EPT_PHYS            0x8D000000
#define SPRD_EPT_SIZE            SZ_4K

/* serial 2 */
#define SPRD_SERIAL2_BASE        0xE0032000
#define SPRD_SERIAL2_PHYS        0x8E000000
#define SPRD_SERIAL2_SIZE        SZ_4K

/* serial 3 */
//#define SPRD_SERIAL3_BASE        0xE0032000
//#define SPRD_SERIAL3_PHYS        0x8F000000
//#define SPRD_SERIAL3_SIZE        SZ_4K

/* registers for watchdog ,RTC, touch panel, aux adc, analog die... */
#define SPRD_MISC_BASE          0xE0033000
#define SPRD_MISC_PHYS          0x82000000
#define SPRD_MISC_SIZE          SZ_4K


/* USB device space */
#define SPRD_USB_BASE            0xE0036000
#define SPRD_USB_PHYS            0x20300000
#define SPRD_USB_SIZE            SZ_1M

#endif
