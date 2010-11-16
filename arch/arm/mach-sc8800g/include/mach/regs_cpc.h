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

#define PIN_CTL_BASE            		(SPRD_CPC_BASE + 0x8C)

#define PIN_CTL_REG					(SPRD_CPC_BASE + 0x0000)

#define ANA_CPC_BASE			(SPRD_MISC_BASE + 0x180)
#define ANA_PIN_CTL_BASE		(ANA_CPC_BASE	+ 0x8C)
#endif

