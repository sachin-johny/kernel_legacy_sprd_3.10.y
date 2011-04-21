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

#ifndef _SC8800H_REG_EMC_H_
#define _SC8800H_REG_EMC_H_

#include <mach/hardware.h>
#include <mach/bits.h>


#define EMC_CFG0                				(SPRD_EMC_BASE + 0x0000)
#define EMC_CFG0_CHANNELS_BASE            	(SPRD_EMC_BASE + 0x0030)
#define EMC_DCFG2				              (SPRD_EMC_BASE + 0x0148)


#define EXT_MEM_STS2                    (SPRD_EMC_BASE + 0x0078)
#define EXT_MEM_STS3                    (SPRD_EMC_BASE + 0x007c)

/* bit definitions for register EMC_CFG0. */
#define	RF_AUTO_SLEEP_ENABLE			(0x1UL << 10)


/* bit definitions for register EMC_CFG0_CHX. */
#define	RF_AUTO_SLEEP_ENABLE_CHX		(0x1UL << 17)

/* bit definitions for register EMC_DCFG2. */
#define	DRF_AUTO_SLEEP_MODE		(0x1UL << 22)
#define	DRF_REF_CNT_RST		(0x1UL << 15)



#endif 

