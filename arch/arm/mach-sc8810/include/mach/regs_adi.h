/*
 *  linux/arch/arm/mach-sc8810/include\mach\regs_adi.h
 *
 *  Spreadtrum regs for adi
 *
 *
 *  Author:	steve.zhan@spreadtrum.com
 *  Created:	Thu AUG 4, 2011
 *  Copyright:	Spreadtrum Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
*   the first version, only modified and checked ana ldo register.
*/
#ifndef _ADI_REG_V3_H_
#define _ADI_REG_V3_H_

#include <mach/hardware.h>
#include <mach/bits.h>
#include <mach/regs_global.h>

#define SPRD_ADI_BASE	SPRD_MISC_BASE
#define SPRD_ADI_PHYS   SPRD_MISC_PHYS
#define  ADI_BASE            SPRD_ADI_BASE  //0x82000000

#define  ADI_BASE_ADDR          ADI_BASE
#define  ADI_CLK_DIV            (ADI_BASE + 0x0 )
#define  ADI_CTL_REG            (ADI_BASE + 0x4 )
#define  ADI_CHANNEL_PRI        (ADI_BASE + 0x8 )
#define  ADI_INT_EN             (ADI_BASE + 0xC )
#define  ADI_INT_RAW_STS        (ADI_BASE + 0x10)
#define  ADI_INT_MASK_STS       (ADI_BASE + 0x14)
#define  ADI_INT_CLR            (ADI_BASE_ADDR + 0x18)
//#define  RESERVED             (ADI_BASE_ADDR + 0x1C)
//#define  RESERVED             (ADI_BASE_ADDR + 0x20)
#define  ADI_ARM_RD_CMD         (ADI_BASE + 0x24)
#define  ADI_RD_DATA            (ADI_BASE + 0x28)
#define  ADI_FIFO_STS           (ADI_BASE + 0x2C)
#define  ADI_STS                (ADI_BASE + 0x30)
#define  ADI_REQ_STS            (ADI_BASE + 0x34)

//ADI_CTL_REG
#define ANA_INT_STEAL_EN        BIT_0
#define ARM_SERCLK_EN           BIT_1
#define DSP_SERCLK_EN           BIT_2

//ADI_FIFO_STS
#define   ADI_FIFO_EMPTY        BIT_10
#define   ADI_FIFO_FULL         BIT_11



//ADI_CHANNEL_PRI bit define
#define    INT_STEAL_PRI        0
#define    STC_WR_PRI           2
#define    ARM_WR_PRI           4
#define    ARM_RD_PRI           6
#define    DSP_WR_PRI           8
#define    DSP_RD_PRI           10
#define    RFT_WR_PRI           12
#define    PD_WR_PRI            14

#define ANA_REG_ADDR_START      (SPRD_ADI_BASE + 0x40) //0x82000040
#define ANA_REG_ADDR_END        (SPRD_ADI_BASE + 0x780) //0x82000780

#endif  //_ADI_REG_V3_H_


