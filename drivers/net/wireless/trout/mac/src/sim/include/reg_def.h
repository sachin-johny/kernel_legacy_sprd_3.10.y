/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: reg_def.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _REG_DEF_H
#define _REG_DEF_H
#include <linux/types.h>

#define REG_RW	0
#define REG_RO	1
#define REG_WO	2


typedef struct dv_reg_s{
	u8 valid;       //0: invalid, 1: valid;
	u8 log;         //0: log; 1: not care;
	u8 rw;          //0: read/write; 1: read only; 2: write only;
	u8 simCare;     //hwSim should be care this register: 0 not care; 1: read; 2: write; 3: both.
	u8 goal;        //de set, hwSim clear: 0 no goal ; 1: read; 2: write; 3: both.

	u8 res0;
	u16 res1;

	u16 rdCount;    //read count from DRV.
	u16 wtCount;    //write count from DRV.
	u32 addr;
	u32 data;
}DV_REG_T;


#endif /* #ifndef _REG_DEF_H */

/*
 *$Log$
 */

