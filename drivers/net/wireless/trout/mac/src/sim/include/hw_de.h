/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: hw_de.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _HW_DE_H
#define _HW_DE_H
#include <linux/types.h>
#include "kern_comm.h"
#include "reg_space.h"
#include "data_hook_space.h"


static inline u8 hwDeRegRead8(const volatile void *addr)
{
	return de_read_reg(addr);
}

static inline u16 hwDeRegRead16(const volatile void *addr)
{
	return de_read_reg(addr);
}

static inline u32 hwDeRegRead32(const volatile void *addr)
{
	return de_read_reg(addr);
}

static inline void hwDeRegWrite8(u8 data, volatile void *addr)
{
	de_write_reg(data, addr);
}

static inline void hwDeRegWrite16(u16 data, volatile void *addr)
{
	de_write_reg(data, addr);
}

static inline void hwDeRegWrite32(u32 data, volatile void *addr)
{
	de_write_reg(data, addr);
}

/*
u8  hwDeFifoRead8(const volatile void *addr);
u16 hwDeFifoRead16(const volatile void *addr);
u32 hwDeFifoRead32(const volatile void *addr);

void hwDeFifoWrite8(u8 data, const volatile void *addr);
void hwDeFifoWrite16(u16 data, const volatile void *addr);
void hwDeFifoWrite32(u32 data, const volatile void *addr);
*/

int hwDeDataHookCopyIn(char *pBuf, int bufLen, int kernMode);
int deIrqInfoRegister(unsigned int irq,
                          irq_handler_t handler,
                          unsigned long flags,
                          const char *name,
                          void *dev);

void hwDeLogEndMsg(void);


#endif /* #ifndef _HW_DE_H */

/*
 *$Log$
 */

