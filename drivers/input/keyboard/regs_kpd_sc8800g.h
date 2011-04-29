/*
 * arch/arm/mach-sc8800g/include/mach/regs_kpd.h
 *
 * Chip keypad controller registers Definitions
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

#ifndef _SC8800S_REG_KPD_H_
#define _SC8800S_REG_KPD_H_

#include <mach/hardware.h>
#include <mach/regs_global.h>
#include <mach/bits.h>

#define KPD_REG_BASE                    SPRD_KPD_BASE

#define KPD_CTRL                        (KPD_REG_BASE + 0x0000)
#define KPD_INT_EN                      (KPD_REG_BASE + 0x0004)
#define KPD_INT_RAW_STATUS              (KPD_REG_BASE + 0x0008)
#define KPD_INT_MASK_STATUS             (KPD_REG_BASE + 0x000C)

#define KPD_INT_CLR                     (KPD_REG_BASE + 0x0010)
#define KPD_POLARITY                    (KPD_REG_BASE + 0x0018)
#define KPD_DEBOUNCE_CNT                (KPD_REG_BASE + 0x001C)
#define KPD_LONG_KEY_CNT                (KPD_REG_BASE + 0x0020)

#define KPD_SLEEP_CNT                   (KPD_REG_BASE + 0x0024)
#define KPD_CLK_DIV_CNT                 (KPD_REG_BASE + 0x0028)
#define KPD_KEY_STATUS                	(KPD_REG_BASE + 0x002C)
#define KPD_SLEEP_STATUS                (KPD_REG_BASE + 0x0030)

#define KPD_DEBUG_STATUS1               (KPD_REG_BASE + 0x0034)
#define KPD_DEBUG_STATUS2               (KPD_REG_BASE + 0x0038)

#define REG_GR_SOFT_RST			(*((volatile unsigned int *)(GR_SOFT_RST)))

#define REG_KPD_CTRL                    (*((volatile unsigned int *)(KPD_CTRL)))
#define REG_KPD_INT_EN                  (*((volatile unsigned int *)(KPD_INT_EN)))
#define REG_KPD_INT_RAW_STATUS          (*((volatile unsigned int *)(KPD_INT_RAW_STATUS)))
#define REG_KPD_INT_MASK_STATUS         (*((volatile unsigned int *)(KPD_INT_MASK_STATUS)))

#define REG_KPD_INT_CLR         	(*((volatile unsigned int *)(KPD_INT_CLR)))
#define REG_KPD_POLARITY         	(*((volatile unsigned int *)(KPD_POLARITY)))
#define REG_KPD_DEBOUNCE_CNT         	(*((volatile unsigned int *)(KPD_DEBOUNCE_CNT)))
#define REG_KPD_LONG_KEY_CNT         	(*((volatile unsigned int *)(KPD_LONG_KEY_CNT)))

#define REG_KPD_SLEEP_CNT         	(*((volatile unsigned int *)(KPD_SLEEP_CNT)))
#define REG_KPD_CLK_DIV_CNT         	(*((volatile unsigned int *)(KPD_CLK_DIV_CNT)))
#define REG_KPD_KEY_STATUS         	(*((volatile unsigned int *)(KPD_KEY_STATUS)))
#define REG_KPD_SLEEP_STATUS         	(*((volatile unsigned int *)(KPD_SLEEP_STATUS)))

#define REG_KPD_DEBUG_STATUS1         	(*((volatile unsigned int *)(KPD_DEBUG_STATUS1)))
#define REG_KPD_DEBUG_STATUS2         	(*((volatile unsigned int *)(KPD_DEBUG_STATUS2)))

#endif
