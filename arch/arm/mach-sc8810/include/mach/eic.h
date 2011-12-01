/*
 *  linux/arch/arm/mach-sprd/gpio.h
 *
 *  Generic SPRD GPIO handling
 *
 *  Author:	Ryan Liao(Ryan Liao@spreadtrum.com)
 *  Created:	Nov 22, 2011
 *  Copyright:	Spreadtrum Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef _EIC_HEAD_H
#define _EIC_HEAD_H

enum EIC_TYPE_E {
	EIC_ID_MIN,
//eic
	EIC_ID_0,
	EIC_ID_1,
	EIC_ID_2,
	EIC_ID_3,
	EIC_ID_4,
	EIC_ID_5,
	EIC_ID_6,
	EIC_ID_7,

	EIC_ID_8,
	EIC_ID_9,
	EIC_ID_10,
	EIC_ID_11,
	EIC_ID_12,
	EIC_ID_13,
	EIC_ID_14,
	EIC_ID_15,
//sic
	EIC_ID_16,
	EIC_ID_17,
	EIC_ID_18,
	EIC_ID_19,
	EIC_ID_20,
	EIC_ID_21,
	EIC_ID_22,
	EIC_ID_23,

	EIC_ID_MAX
};
//api end

extern int sprd_get_eic_data(enum EIC_TYPE_E eic_id);
extern int sprd_alloc_eic_irq(enum EIC_TYPE_E eic_id);
#endif
