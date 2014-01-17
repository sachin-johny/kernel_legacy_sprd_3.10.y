/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: hw_sim.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/10, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _HW_SIM_H
#define _HW_SIM_H

#include "kern_comm.h"
#include "reg_def.h"
#include "types.h"


typedef struct hw_de_irq_info_s{
    unsigned int irq;
    irq_handler_t handler;
    unsigned long flags;
    const char *name;
    void *dev;
}HW_DE_IRQ_INFO_T;

#define DATA_SRC_SPACE_SIZE 4096
#define DATA_DEST_SPACE_SIZE 4096
#define DATA_HOOK_SPACE_SIZE 4096

#define HW_SIM_STATE_UNINIT 0
#define HW_SIM_STATE_IDLE   1
#define HW_SIM_STATE_RX     2
#define HW_SIM_STATE_TX     3
#define HW_SIM_STATE_HALT   4




typedef struct hw_sim_s{
	void *pRegBaseAddr;
	int regSpaceLen;
    struct semaphore regLock;

	void *pDataSrcAddr;
	int dataSrcLen;                 //actual len
	int dataSrcSpaceLen;            //max len
    int srcCurrOffset;
    struct semaphore dataSrcLock;

	void *pDataDestAddr;
	int dataDestLen;                //actual len
	u32 dataDestSpaceLen;           //max len
    int destCurrOffset;
    struct semaphore dataDestLock;

	void *pDataHookAddr;
	int dataHookLen;                //actual len
	u32 dataHookSpaceLen;           //max len
    int hookCurrOffset;
    struct semaphore dataHookLock;

	void *pDePrivData;
	struct semaphore simIsDone;
    
    struct work_struct dvDataWork;
    struct work_struct deDataWork;
    
    struct workqueue_struct *pHwSimWq;
	int state;				//sim state machine: idle, tx busy, rx busy, uninit, inited.
	int errCode;
} HW_SIM_T;



DV_REG_T *regLookup(const volatile void *addr);
int hwErrCodeGet(void);

HW_DE_IRQ_INFO_T *hwDeIrqInfoGet(void);
inline u8 hwRegRead8(const volatile void *addr);
inline u16 hwRegRead16(const volatile void *addr);
inline u32 hwRegRead32(const volatile void *addr);
inline void hwRegWrite8(u8 data, volatile void *addr);
inline void hwRegWrite16(u16 data, volatile void *addr);
inline void hwRegWrite32(u32 data, volatile void *addr);

u8 hwFifoRead8(const volatile void *addr);
u16 hwFifoRead16(const volatile void *addr);
u32 hwFifoRead32(const volatile void *addr);
void hwFifoWrite8(u8 data, const volatile void *addr);
void hwFifoWrite16(u16 data, const volatile void *addr);
void hwFifoWrite32(u32 data, const volatile void *addr);

int hwRegSpaceCopyOut(char *pBuf, int bufLen, int kernMode);
int hwRegSpaceCopyIn(char *pBuf, int bufLen, int kernMode);
int hwDataHookCopyIn(char *pBuf, int bufLen, int kernMode);
int hwDataHookCopyOut(char *pBuf, int bufLen, int kernMode);
int hwDataDestCopyOut(char *pBuf, int bufLen, int kernMode);
int hwDataDestCopyIn(char *pBuf, int bufLen, int kernMode);
int hwDataRxIn(void *pBuf, int bufLen, int mode);

int hwErrCodeGet(void);
int hwSimIsDone(int timeout);
int hwSimRegReset(void);
int hwSimUsrDefineOp(char *pBuf, int bufLen, int mode);

int hwSimInit(void);		//initialize hw registers, data mem, and so on.
void hwSimExit(void);		

#endif /* #ifndef _HW_SIM_H */

/*
 *$Log$
 */

