/*
 * arch/arm/mach-sc8800s/include/mach/regs_cpc.h
 *
 * Chip Pin Control registers Definitions
 *
 * Copyright (C) 2010 Spreadtrum International Ltd.
 *
 * 2010-03-05: yingchun li <yingchun.li@spreadtrum.com>
 *            initial version
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _SC8800H_REG_CPC_H_
#define _SC8800H_REG_CPC_H_
#include <mach/hardware.h>

#define PIN_CTL_BASE            		(SPRD_CPC_BASE)

#define PIN_CTL_REG					(SPRD_CPC_BASE + 0x0000)

#define ANA_CPC_BASE			(SPRD_MISC_BASE + 0x180)
#define ANA_PIN_CTL_BASE		(ANA_CPC_BASE	+ 0x8C)
#define PM_INVALID_VAL        0xffffffff
#define PM_INVALID_SHORT_VAL  0xffff

typedef  unsigned int  uint32;
typedef  unsigned short uint16;

typedef struct PM_PINFUNC_tag
{
    uint32 addr;
    uint32 value;
} PM_PINFUNC_T;

typedef enum
{
    PM_OUTPUT,
    PM_INPUT,
    PM_INVALID_DIR
} PM_DIR_E;

typedef enum
{
    PM_LEVEL,
    PM_RISING_EDGE,
    PM_FALLING_EDGE,
    PM_BOTH_EDGE,
    PM_NO_INT,
    PM_INVALID_INT
} PM_IS_E;

typedef struct GPIO_CTL_tag
{
    uint16   gpio_num;
    uint16   default_val;
    PM_DIR_E dir;
    PM_IS_E  int_sense;
} PM_GPIO_CTL_T;

extern const PM_PINFUNC_T   pm_func[];
extern const PM_PINFUNC_T   pm_default_global_map[];
extern const PM_GPIO_CTL_T  pm_gpio_default_map[];

#define CHIP_REG_OR(reg_addr, value)   do{ \
                                             unsigned long flags;   \
                                             hw_local_irq_save(flags);  \
                                             (*(volatile unsigned int *)(reg_addr) |= (unsigned int)(value));   \
                                             hw_local_irq_restore(flags);   \
                                       }while(0)       
#define CHIP_REG_AND(reg_addr, value)   do{ \
                                             unsigned long flags;   \
                                             hw_local_irq_save(flags);  \
                                             (*(volatile unsigned int *)(reg_addr) &= (unsigned int)(value)); \
                                             hw_local_irq_restore(flags);   \
                                       }while(0)
#define CHIP_REG_GET(reg_addr)          (*(volatile unsigned int *)(reg_addr))
#define CHIP_REG_SET(reg_addr, value)   do{ \
                                             unsigned long flags;   \
                                             hw_local_irq_save(flags);  \
                                             (*(volatile unsigned int *)(reg_addr)  = (unsigned int)(value)) ;   \
                                             hw_local_irq_restore(flags);   \
                                       }while(0)

#define SCI_ASSERT(condition) BUG_ON(!(condition))  
#define SCI_PASSERT(condition, format...)  \
	do {		\
		if(!(condition)) { \
			pr_err("function :%s\r\n", __FUNCTION__);\
			BUG();	\
		} \
	}while(0)

#endif

