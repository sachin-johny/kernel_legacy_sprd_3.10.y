/*
 * arch/arm/mach-sc8800s/include/mach/regs_kpd.h
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

#ifndef _SC8800H_REG_KPD_H_
#define _SC8800H_REG_KPD_H_

#include <mach/hardware.h>
#include <mach/bits.h>

#define KPD_REG_BASE                    SPRD_KPD_BASE

#define KPD_STS                         (KPD_REG_BASE + 0x0000)
#define KPD_CTL                         (KPD_REG_BASE + 0x0004)
#define KPD_ICLR                        (KPD_REG_BASE + 0x0008)
#define KPD_POLARITY                    (KPD_REG_BASE + 0x000C)

#define KPD_CLKDIV                      (KPD_REG_BASE + 0x0010)
#define KPD_TOUTCNT                     (KPD_REG_BASE + 0x0014)
#define KPD_INT_MSK                     (KPD_REG_BASE + 0x0018)
#define KPD_PBINT_CTL                   (KPD_REG_BASE + 0x0028)
#define KPD_PBINT_CNT                   (KPD_REG_BASE + 0x002C)
#define KPD_PBINT_LATCNT                (KPD_REG_BASE + 0x0030)

#define REG_KPD_STS                     (*((volatile unsigned int *)(KPD_STS)))
#define REG_KPD_CTL                     (*((volatile unsigned int *)(KPD_CTL)))
#define REG_KPD_ICLR                    (*((volatile unsigned int *)(KPD_ICLR)))
#define REG_KPD_POLARITY                (*((volatile unsigned int *)(KPD_POLARITY)))
#define REG_KPD_CLKDIV                  (*((volatile unsigned int *)(KPD_CLKDIV)))
#define REG_KPD_TOUTCNT                 (*((volatile unsigned int *)(KPD_TOUTCNT)))
#define REG_KPD_INT_MSK                 (*((volatile unsigned int *)(KPD_INT_MSK)))
#define REG_KPD_PBINT_CTL               (*((volatile unsigned int *)(KPD_PBINT_CTL)))
#define REG_KPD_PBINT_CNT               (*((volatile unsigned int *)(KPD_PBINT_CNT)))
#define REG_KPD_PBINT_LATCNT            (*((volatile unsigned int *)(KPD_PBINT_LATCNT)))

#endif
