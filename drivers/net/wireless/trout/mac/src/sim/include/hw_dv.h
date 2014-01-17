/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: hw_dv.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _HW_DV_H
#define _HW_DV_H
#include <linux/types.h>
#include "dv_ioctl.h"
#include "normal_reg.h"
#include "data_hook_space.h"

static inline u8 hwDvRegRead8(u32 addr)
{
	return dv_read_reg(addr);
}

static inline u16 hwDvRegRead16(u32 addr)
{
	return dv_read_reg(addr);
}

static inline u32 hwDvRegRead32(u32 addr)
{
	return dv_read_reg(addr);
}

static inline void hwDvRegWrite8(u8 data, u32 addr)
{
	dv_write_reg(data, addr);
}

static inline void hwDvRegWrite16(u16 data, u32 addr)
{
	dv_write_reg(data, addr);
}

static inline void hwDvRegWrite32(u32 data, u32 addr)
{
	dv_write_reg(data, addr);
}

static inline int hwDvDataHookRead(char *pBuf, int bufLen, int mode)
{
	//return data_hook_out(pBuf, bufLen, mode);
	return 0;
}

int hwDvDataRxInWrite(DV_IOCTL_T *ioctl, char *pBuf);
int hwDvDataTxoutRead(struct file *filp, DV_IOCTL_T *ioctl, char *pBuf);

int hwDvRegReset(void);

int hwDvRegReadAll(char *pBuf, int bufLen, int kernMode);

int hwDvSimIsDone(int timeout);
int hwDvUsrDefineOp(DV_IOCTL_T *ioctl);

int get_kern_addr(DV_IOCTL_T *ioctl);
int get_kern_data(DV_IOCTL_T *ioctl);
int set_kern_data(DV_IOCTL_T *ioctl);


int hwDvSimInit(void);		//initialize hw registers, data mem, and so on.
void hwDvSimExit(void);


#endif /* #ifndef _HW_DV_H */

/*
 *$Log$
 */

