/*
 * arch/arm/mach-sprd/include/mach/pm.h
 *
 * Pin Map Definitions
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

#ifndef __ASM_ARCH_MFP_H
#define __ASM_ARCH_MFP_H

#include <mach/bits.h>
#include <mach/regs_cpc.h>
/*
	This is identical to the order of pin's register map.
	NOTE: if the pin register is not countinus, you should
	intest a RESEVERED pin between them;
*/

//The pin is in A die
#define A_DIE_PIN	BIT_31

#define MFP_PIN(x)	((((x##_REG)-PIN_CTL_BASE) & 0xffff) << 16)

#define MFP_ANA_PIN(x) (((((x##_REG)-ANA_PIN_CTL_BASE) & 0xffff) << 16) | A_DIE_PIN)

#define MFP_CFG_TO_REG_OFFS(x)  ((x) >> 16)

//special bit for setting, for the default value of is not same
//with all registers
#define MFP_IO_SET				(0x1 << 15)
#define MFP_S_PULL_SET 	(0x1 << 14)
#define MFP_AF_SET   			(0x1 << 13)
#define MFP_F_PULL_SET 		(0x1 << 12)
#define MFP_DS_SET 				(0x1 << 11)

/* Pinmap ctrl register Bit field value
--------------------------------------------------------------------------------------------------------------------------
|                 |                 |            |            |              |       |       |            |              |
| Reserved[31:10] | Drv str sel[9:8]| func PU[7] | func PD[6] | func sel[5:4]| PU[3] | PD[2] | input En[1]| output En[0] |
|                 |                 |            |            |              |       |       |            |              |
--------------------------------------------------------------------------------------------------------------------------
*/

/*
pin output/input enable.
NOTE, this is not applied to GPIO pins, GPIO pin's input/output direction have specific
	registers and specfic bits.
	BIT 0, 1
*/
#define MFP_IO_NONE  (0x0 << 0)
#define MFP_IO_Z		MFP_IO_NONE
#define MFP_IO_OE		(0x1	<< 0)
#define MFP_IO_IE		(0x2 << 0)
#define MFP_IO_BOTH   (0x3 << 0)
#define MFP_IO_MASK  MFP_IO_BOTH

/*
	pin weak pull up/down in sleep mode
	BIT 2, 3
*/
#define MFP_S_PULL_NONE	(0x0  << 2)
#define MFP_S_PULL_DOWN   	(0x1 << 2)
#define MFP_S_PULL_UP		(0x2  << 2)
#define MFP_S_PULL_BOTH	(0x3  << 2)
#define MFP_S_PULL_MASK 	MFP_S_PULL_BOTH

/*
	pin alternate function
	BIT 4, 5
*/
#define MFP_AF0			(0x0 << 4)
#define MFP_AF1			(0x1 << 4)
#define MFP_AF2			(0x2 << 4)
#define MFP_AF3			(0x3 << 4)
#define MFP_AF_MASK		(0x3 << 4)
#define MFP_GPIO  MFP_AF3

/*
	pin weak pull up/down in function mode
	BIT 6, 7
*/
#define MFP_F_PULL_NONE	(0x0  << 6)
#define MFP_F_PULL_DOWN   	(0x1 << 6)
#define MFP_F_PULL_UP		(0x2  << 6)
#define MFP_F_PULL_BOTH	(0x3  << 6)
#define MFP_F_PULL_MASK 	MFP_F_PULL_BOTH

/*
	pin driver strenth
	BIT 8, 9
*/

#define MFP_DS0		(0x0 << 8)
#define MFP_DS1		(0x1 << 8)
#define MFP_DS2		(0x2 << 8)
#define MFP_DS3		(0x3 << 8)
#define MFP_DS_MASK	(0x3 << 8)


#define MFP_CFG(pin, af)		\
	(MFP_AF_SET |\
	 (MFP_PIN(PIN_##pin) | MFP_##af))

#define MFP_CFG_DRV(pin, af, drv)	\
	((MFP_AF_SET |MFP_DS_SET) |\
	 (MFP_PIN(PIN_##pin) | MFP_##af | MFP_##drv))

#define MFP_CFG_SLEEP_UPDOWN(pin, af, updown)	\
	((MFP_AF_SET | MFP_S_PULL_SET) |\
	 (MFP_PIN(PIN_##pin) | MFP_##af | MFP_##updown))

#define MFP_CFG_IOE(pin, af, io)	\
	((MFP_AF_SET |MFP_IO_SET)  |\
	 (MFP_PIN(PIN_##pin) | MFP_##af | MFP_##io))

#define MFP_SET_ALL	\
	(MFP_AF_SET |MFP_IO_SET | MFP_S_PULL_SET | MFP_DS_SET | \
	MFP_F_PULL_SET)

#define MFP_CFG_X(pin, af, drv, func_updown, sleep_updown, io)	\
	(MFP_SET_ALL |\
	 (MFP_PIN(PIN_##pin) | MFP_##af | MFP_##drv |\
	 MFP_##func_updown  | MFP_##sleep_updown| MFP_##io))

#define MFP_ANA_CFG_X(pin, drv, func_updown, af, sleep_updown, io)	\
	(MFP_SET_ALL |\
	 (MFP_ANA_PIN(ANA_PIN_##pin) |  MFP_##af | MFP_##drv |\
	 MFP_##func_updown | MFP_##sleep_updown| MFP_##io))

extern unsigned long mfp_to_gpio(int pin);
extern void sprd_mfp_config(unsigned long *mfp_cfgs, int num);
#endif

