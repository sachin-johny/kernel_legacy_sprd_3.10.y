/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: hw_sim.c
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/
#include <linux/types.h>
#include <linux/workqueue.h>
#include "../include/sim.h"
/*
#include "../include/kern_comm.h"
#include "../include/reg_def.h"
#include "../include/debug.h"
#include "../include/hw_sim.h"
#include "../include/error.h" */

HW_DE_IRQ_INFO_T hwDeIrqInfo;
HW_SIM_T hwSimInfo;

#define DATA_TX_ADDR_REG  0xB01C0100
#define DATA_TX_LEN_REG   0xB01C0104        //bit0~16 is packet len, bit31 set to 1 to enable tx.

#define DATA_RX_ADDR_REG  0xB01C0200
#define DATA_RX_LEN_REG   0xB01C0204        //bit0~16 is buffer len, bit31 set to 1 to indicate rx.


static DV_REG_T hwRegGroup[] = {
{.valid = 1, .log = 1, .rw = 0, .simCare = 0, .goal = 0, .res0 = 0, .res1 = 0, .rdCount = 0, .wtCount = 0, .addr = 0xB01C000C, .data = 0},
{.valid = 1, .log = 1, .rw = 0, .simCare = 0, .goal = 0, .res0 = 0, .res1 = 0, .rdCount = 0, .wtCount = 0, .addr = 0xB01C0014, .data = 0},

{.valid = 1, .log = 1, .rw = 0, .simCare = 0, .goal = 0, .res0 = 0, .res1 = 0, .rdCount = 0, .wtCount = 0, .addr = DATA_TX_ADDR_REG, .data = 0},
{.valid = 1, .log = 1, .rw = 0, .simCare = 2, .goal = 0, .res0 = 0, .res1 = 0, .rdCount = 0, .wtCount = 0, .addr = DATA_TX_LEN_REG, .data = 0},

{.valid = 1, .log = 1, .rw = 0, .simCare = 0, .goal = 0, .res0 = 0, .res1 = 0, .rdCount = 0, .wtCount = 0, .addr = DATA_RX_ADDR_REG, .data = 0},
{.valid = 1, .log = 1, .rw = 0, .simCare = 2, .goal = 0, .res0 = 0, .res1 = 0, .rdCount = 0, .wtCount = 0, .addr = DATA_RX_LEN_REG, .data = 0},


//....
 
//end
{.valid = 0, .log = 0, .rw = 0, .simCare = 0, .goal = 0, .res0 = 0, .res1 = 0, .rdCount = 0, .wtCount = 0, .addr = 0x00000000, .data = 0},
};





//函数功能: 在寄存器空间中查找对应的寄存器。
//函数参数: addr - 寄存器地址。
//返回值:   寄存器对应的信息。
DV_REG_T *regLookup(const volatile void *addr)
{
	DV_REG_T *pCell = (DV_REG_T *)hwSimInfo.pRegBaseAddr;

    if(pCell != NULL)
    {
    	while(pCell->valid && ((u32)(pCell->addr) != (u32)addr))
    	{
    		pCell++;
    	}

    	if(pCell->valid)
    		return pCell;
    }

    return NULL;
}

//函数功能: 读取1个字节类型的寄存器
//函数参数: addr - 寄存器地址
//返回值:   寄存器值。
inline u8 hwRegRead8(const volatile void *addr)
{
	DV_REG_T *pCell = regLookup(addr);
	
	if(pCell != NULL)
	{
		return pCell->data;
	}
	else
	{
		DV_DBG("%s: invalid addr: 0x%p\n", __FUNCTION__, addr);
        BUG();
		return 0;
	}
}

//函数功能: 读取1个短整形类型的寄存器
//函数参数: addr - 寄存器地址
//返回值:   寄存器值。
inline u16 hwRegRead16(const volatile void *addr)
{
	DV_REG_T *pCell = regLookup(addr);
	
	if(pCell != NULL)
	{
		return pCell->data;
	}
	else
	{
		DV_DBG("%s: invalid addr: 0x%p\n", __FUNCTION__, addr);
        BUG();
		return 0;
	}
}

//函数功能: 读取1个整形类型的寄存器
//函数参数: addr - 寄存器地址
//返回值:   寄存器值。
inline u32 hwRegRead32(const volatile void *addr)
{
	DV_REG_T *pCell = regLookup(addr);
	
	if(pCell != NULL)
	{
		return pCell->data;
	}
	else
	{
		DV_DBG("%s: invalid addr: 0x%p\n", __FUNCTION__, addr);
        BUG();
		return 0;
	}
}


//函数功能: 写1个字节类型的寄存器
//函数参数: addr - 寄存器地址
inline void hwRegWrite8(u8 data, volatile void *addr)
{
	DV_REG_T *pCell = regLookup(addr);
	
	if(pCell != NULL)
	{
		pCell->data = data;
	}
	else
	{
		DV_DBG("%s: invalid addr: 0x%p\n", __FUNCTION__, addr);
        BUG();
	}
}

//函数功能: 写1个短整形类型的寄存器
//函数参数: addr - 寄存器地址
inline void hwRegWrite16(u16 data, volatile void *addr)
{
	DV_REG_T *pCell = regLookup(addr);
	
	if(pCell != NULL)
	{
		pCell->data = data;
	}
	else
	{
		DV_DBG("%s: invalid addr: 0x%p\n", __FUNCTION__, addr);
        BUG();
	}
}

//函数功能: 写1个整形类型的寄存器
//函数参数: addr - 寄存器地址
inline void hwRegWrite32(u32 data, volatile void *addr)
{
	DV_REG_T *pCell = regLookup(addr);
	
	if(pCell != NULL)
	{
		pCell->data = data;
	}
	else
	{
		DV_DBG("%s: invalid addr: 0x%p\n", __FUNCTION__, addr);
        BUG();
	}
}

//hw FIFO operation

#define HW_FIFO_READX(addr, tmp, bits) do{  \
    register u##bits *pData = (u##bits *)hwSimInfo.pDataSrcAddr;  \
    register u32 totalLen = hwSimInfo.dataSrcLen;  \
    register u32 offset = hwSimInfo.srcCurrOffset;  \
        \
    if(pData != NULL && offset < totalLen)  \
    {   \
        tmp = pData[offset];    \
    }   \
        \
    offset += bits >> 3;    \
    if(++offset >= totalLen)    \
    {   \
        offset = 0; \
        hwSimInfo.dataSrcSpaceLen = 0;  \
    }   \
    hwSimInfo.srcCurrOffset = offset;   \
}while(0)

#define HW_FIFO_WRITEX(addr, tmp, bits) do{  \
        register u##bits *pData = (u##bits *)hwSimInfo.pDataDestAddr;  \
        register u32 totalLen = hwSimInfo.dataDestLen;  \
        register u32 offset = hwSimInfo.destCurrOffset;  \
            \
        if(pData != NULL && offset < totalLen)  \
        {   \
            pData[offset] = tmp;    \
        }   \
            \
        offset += bits >> 3;    \
        if(++offset >= totalLen)    \
        {   \
            offset = 0; \
            hwSimInfo.dataSrcSpaceLen = 0;  \
        }   \
        hwSimInfo.srcCurrOffset = offset;   \
    }while(0)


#if 1
u8 hwFifoRead8(const volatile void *addr)
{
    u8 tmp = 0;

    HW_FIFO_READX(addr, tmp, 8);
    
    return tmp;
}

u16 hwFifoRead16(const volatile void *addr)
{
    u16 tmp = 0;

    HW_FIFO_READX(addr, tmp, 16);
    
    return tmp;
}

u32 hwFifoRead32(const volatile void *addr)
{
    u32 tmp = 0;

    HW_FIFO_READX(addr, tmp, 32);
    
    return tmp;
}
#else
u8 hwFifoRead8(u32 addr)
{
    register u8 *pChar = (u8 *)hwSimInfo.pDataSrcAddr;
    register u32 totalLen = hwSimInfo.dataSrcSpaceLen;
    register u32 offset = hwSimInfo.srcCurrOffset;
    u8 tmp = 0;

    if(pChar != NULL && offset < totalLen)
    {
        tmp = pChar[offset];
    }

    //update offset
    if(++offset >= totalLen)
    {
        offset = 0;
        hwSimInfo.dataSrcSpaceLen = 0;
    }
    
    hwSimInfo.srcCurrOffset = offset;
    
    return tmp;
}

u16 hwFifoRead16(u32 addr)
{
    register u16 *pShort = (u16 *)hwSimInfo.pDataSrcAddr;
    register u32 totalLen = hwSimInfo.dataSrcSpaceLen;
    register u32 offset = hwSimInfo.srcCurrOffset;
    u16 tmp = 0;

    if(pShort != NULL && offset < totalLen)
    {
        tmp = pShort[offset];
    }

    //update offset
    offset += 2;
    if(offset >= totalLen)
    {
        offset = 0;
        hwSimInfo.dataSrcSpaceLen = 0;
    }
    
    hwSimInfo.srcCurrOffset = offset;
    
    return tmp;
}

u32 hwFifoRead32(u32 addr)
{
    register u32 *pInt = (u32 *)hwSimInfo.pDataSrcAddr;
    register u32 totalLen = hwSimInfo.dataSrcSpaceLen;
    register u32 offset = hwSimInfo.srcCurrOffset;
    u32 tmp = 0;

    if(pInt != NULL && offset < totalLen)
    {
        tmp = pInt[offset];
    }

    //update offset
    offset += 4;
    if(offset >= totalLen)
    {
        offset = 0;
        hwSimInfo.dataSrcSpaceLen = 0;
    }
    
    hwSimInfo.srcCurrOffset = offset;
    
    return tmp;
}
#endif

void hwFifoWrite8(u8 data, const volatile void *addr)
{
    HW_FIFO_WRITEX(addr, data, 8);
}

void hwFifoWrite16(u16 data, const volatile void *addr)
{
    HW_FIFO_WRITEX(addr, data, 16);
}

void hwFifoWrite32(u32 data, const volatile void *addr)
{
    HW_FIFO_WRITEX(addr, data, 32);
}

void hwRegGoalSet(volatile void *addr, int val)
{
	DV_REG_T *pReg = regLookup(addr);
	if(pReg != NULL)
	{
		pReg->goal = val;
	}
	else
	{
		DV_DBG("%s: invalid addr: 0x%p\n", __FUNCTION__, addr);
    }
}

int hwRegSpaceCopyOut(char *pBuf, int bufLen, int kernMode)
{
    char *regBaseAddr = hwSimInfo.pRegBaseAddr;
    int ret = DV_ERROR;

    if(down_interruptible(&hwSimInfo.regLock) == 0)
    {
        if(hwSimInfo.regSpaceLen <= bufLen)
        {            
            ret = hwSimInfo.regSpaceLen;
            if(kernMode)
                memcpy(pBuf, regBaseAddr, ret);
            else
            {
                if(copy_to_user(pBuf, regBaseAddr, ret))
                { 
                    ret = DV_ER_INTERNAL; 
                }
            }
        }
        else
            ret = DV_ER_NOMEM;

        up(&hwSimInfo.regLock);
    }
    
    return ret;
}


int hwRegSpaceCopyIn(char *pBuf, int bufLen, int kernMode)
{
    char *regBaseAddr = hwSimInfo.pRegBaseAddr;
    int ret = DV_ERROR;

    if(down_interruptible(&hwSimInfo.regLock) == 0)
    {
        if(bufLen <= hwSimInfo.regSpaceLen)
        {            
            ret = bufLen;
            if(kernMode)
                memcpy(regBaseAddr, pBuf, ret);
            else
            {
                if(copy_from_user(regBaseAddr, pBuf, ret))
                { 
                    ret = DV_ER_INTERNAL; 
                }
            }
        }
        else
            ret = DV_ER_NOMEM;

        up(&hwSimInfo.regLock);
    }
    
    return ret;
}

//copy data from hwSim dest data to DV or DE.
//kernelMode -0: pBuf is in user mem space; else pBuf is in kernel mem space.
int hwDataSrcCopyOut(char *pBuf, int bufLen, int kernMode)
{
    char *pChar = hwSimInfo.pDataSrcAddr;
    int ret = hwSimInfo.dataSrcLen;

    if(ret <= bufLen)
    {            
        if(down_interruptible(&hwSimInfo.dataSrcLock) == 0)
        {
            if(kernMode)
                memcpy(pBuf, pChar, ret);
            else
            {
                if(copy_to_user(pBuf, pChar, ret))
                { 
                    ret = DV_ER_INTERNAL;                     
                    hwSimInfo.dataDestLen = 0;
                }
            }
            
            up(&hwSimInfo.dataSrcLock);
        }
        else
            ret = DV_ERROR;
    }
    else
    {
        ret = DV_ER_NOMEM;
        DV_DBG("%s: no mem!\n", __FUNCTION__);
    }
    
    return ret;
}


//copy data from hwSim src data from DV or DE.
//kernelMode -0: pBuf is in user mem space; else pBuf is in kernel mem space.
int hwDataSrcCopyIn(char *pBuf, int bufLen, int kernMode)
{
    char *pChar = hwSimInfo.pDataSrcAddr;
    int ret = bufLen;

    if(bufLen <= hwSimInfo.dataSrcSpaceLen)
    {            
        if(down_interruptible(&hwSimInfo.dataSrcLock) == 0)
        {
            hwSimInfo.dataSrcLen = bufLen;
            if(kernMode)
                memcpy(pChar, pBuf, bufLen);
            else
            {
                if(copy_from_user(pChar, pBuf, bufLen))
                { 
                    ret = DV_ER_INTERNAL;                     
                    hwSimInfo.dataSrcLen = 0;
                }
            }
            
            up(&hwSimInfo.dataSrcLock);
        }
        else
            ret = DV_ERROR;
    }
    else
    {
        ret = DV_ER_NOMEM;
        DV_DBG("%s: no mem!\n", __FUNCTION__);
    }
    
    return ret;
}

//copy data from hwSim dest data to DV or DE.
//kernelMode -0: pBuf is in user mem space; else pBuf is in kernel mem space.
int hwDataDestCopyOut(char *pBuf, int bufLen, int kernMode)
{
    char *pChar = hwSimInfo.pDataDestAddr;
    int ret = hwSimInfo.dataDestLen;

    DV_DBG("%s ret=%d len=%d \n",__FUNCTION__,ret,bufLen);
    if(ret <= bufLen)
    {            
        if(down_interruptible(&hwSimInfo.dataDestLock) == 0)
        {
            if(kernMode)
                memcpy(pBuf, pChar, ret);
            else
            {
                if(copy_to_user(pBuf, pChar, ret))
                { 
                    ret = DV_ER_INTERNAL;                     
                    hwSimInfo.dataDestLen = 0;
                }
            }
            
            up(&hwSimInfo.dataDestLock);
        }
        else
            ret = DV_ERROR;
    }
    else
    {
        ret = DV_ER_NOMEM;
        DV_DBG("%s: no mem!\n", __FUNCTION__);
    }
    
    return ret;
}


//copy data to hwSim dest data from DV or DE.
//kernelMode -0: pBuf is in user mem space; else pBuf is in kernel mem space.
int hwDataDestCopyIn(char *pBuf, int bufLen, int kernMode)
{
    char *pChar = hwSimInfo.pDataDestAddr;
    int ret = bufLen;

    if(bufLen <= hwSimInfo.dataDestSpaceLen)
    {   
        DV_DBG("%s bufLen=%d \n",__FUNCTION__,bufLen);
        if(down_interruptible(&hwSimInfo.dataDestLock) == 0)
        {
            hwSimInfo.dataDestLen = bufLen;
            if(kernMode)
                memcpy(pChar, pBuf, bufLen);
            else
            {
                if(copy_from_user(pChar, pBuf, bufLen))
                { 
                    ret = DV_ER_INTERNAL;                     
                    hwSimInfo.dataDestLen = 0;
                }
            }
            DV_DBG("%s hwSimInfo.dataDestLen=%d \n",__FUNCTION__,hwSimInfo.dataDestLen);
            up(&hwSimInfo.dataDestLock);
        }
        else
            ret = DV_ERROR;
    }
    else
    {
        ret = DV_ER_NOMEM;
        DV_DBG("%s: no mem!\n", __FUNCTION__);
    }
    
    return ret;
}

//copy data from hwSim src data from DVor DE.
//kernelMode -0: pBuf is in user mem space; else pBuf is in kernel mem space.
int hwDataHookCopyIn(char *pBuf, int bufLen, int kernMode)
{
    char *pChar = hwSimInfo.pDataHookAddr;
    int ret = bufLen;

    if(bufLen <= hwSimInfo.dataHookSpaceLen)
    {
        if(down_interruptible(&hwSimInfo.dataHookLock) == 0)
        {
            hwSimInfo.dataHookLen = bufLen;
            if(kernMode)
                memcpy(pChar, pBuf, bufLen);
            else
            {
                if(copy_from_user(pChar, pBuf, bufLen))
                { 
                    ret = DV_ER_INTERNAL;                     
                    hwSimInfo.dataHookLen = 0;
                }
            }
            
            up(&hwSimInfo.dataHookLock);
        }
        else
            ret = DV_ERROR;
    }
    else
    {
        ret = DV_ER_NOMEM;
        DV_DBG("%s: no mem!\n", __FUNCTION__);
    }
    
    return ret;
}


//copy data from hwSim dest data to DVor DE.
//kernelMode -0: pBuf is in user mem space; else pBuf is in kernel mem space.
int hwDataHookCopyOut(char *pBuf, int bufLen, int kernMode)
{
    char *pChar = hwSimInfo.pDataDestAddr;
    int ret = hwSimInfo.dataHookLen;

    if(ret <= bufLen)
    {            
        if(down_interruptible(&hwSimInfo.dataHookLock) == 0)
        {
            if(kernMode)
                memcpy(pBuf, pChar, ret);
            else
            {
                if(copy_to_user(pBuf, pChar, ret))
                { 
                    ret = DV_ER_INTERNAL;                     
                    hwSimInfo.dataHookLen = 0;
                }
            }
            
            up(&hwSimInfo.dataHookLock);
        }
        else
            ret = DV_ERROR;
    }
    else
    {
        ret = DV_ER_NOMEM;
        DV_DBG("%s: no mem!\n", __FUNCTION__); 
    }
    
    return ret;
}


inline void *hwSimRegBaseAddrGet(void)
{
    return hwSimInfo.pRegBaseAddr;
}

inline u32 hwSimRegSpaceLenGet(void)
{
    return hwSimInfo.regSpaceLen;
}

inline void *hwSimDataSrcAddrGet(void)
{
    return hwSimInfo.pDataSrcAddr;
}

inline u32 hwSimDataSrcSpaceLenGet(void)
{
    return hwSimInfo.dataSrcSpaceLen;
}


inline void *hwSimDataDestAddrGet(void)
{
    return hwSimInfo.pDataDestAddr;
}

inline u32 hwSimDataDestSpaceLenGet(void)
{
    return hwSimInfo.dataDestSpaceLen;
}


//process src data from hwSimInfo.pDataSrcAddr.
//put error into hwSimInfo.errCode.
static void hwDataRxInProcess(struct work_struct *work)
{
    HW_SIM_T *pHwSim = container_of(work, HW_SIM_T, dvDataWork);

    //process src data.
    char *pBuf = (char *)hwRegRead32((const volatile void *)DATA_RX_ADDR_REG);
    int bufLen = hwRegRead32((const volatile void *)DATA_RX_LEN_REG);

    pHwSim->errCode = DV_OK;
    pHwSim->state = HW_SIM_STATE_RX;

    if((pBuf != NULL) && !(bufLen & (1U << 31)))
    {
        if((bufLen=hwDataSrcCopyOut(pBuf, DATA_SRC_SPACE_SIZE, 1)) < 0)
        {
            DV_DBG(" hwDataSrcCopyOut failed.\n");
//            bufLen |= 1UL << 30;
            pHwSim->errCode = DV_ER_NOMEM;
            up(&pHwSim->simIsDone);
            return;
        }
    }
    else
    {
        pHwSim->errCode = DV_ER_EXCPT;
//        bufLen |= 1UL << 29;        
        up(&pHwSim->simIsDone);
        return;
    }

    bufLen |= 1UL << 31;

    //set register associate bit.
    hwRegWrite32(bufLen, (volatile void *)DATA_RX_LEN_REG);
    
    //call dut process routine, sample:
    if(hwDeIrqInfo.handler != NULL)
    {
        hwDeIrqInfo.handler(hwDeIrqInfo.irq, hwDeIrqInfo.dev);
    }
    else
    {
        hwSimInfo.errCode = DV_ER_NOTFOUND;
    }
    
    //wake up test task.
//    up(&pHwSim->simIsDone);
    
//    pHwSim->state = HW_SIM_STATE_IDLE;
}

int hwDataRxIn(void *pBuf, int bufLen, int mode)
{
    if(hwDataSrcCopyIn(pBuf, bufLen, mode) != bufLen)
    {
        DV_DBG(" hwDataSrcCopyIn failed.\n");
        return DV_ERROR;
    }

    queue_work_on(0, hwSimInfo.pHwSimWq, &hwSimInfo.dvDataWork);

    return DV_OK;
}

//process Tx data from de.
//put error into hwSimInfo.errCode.
static void hwDataTxOutProcess(struct work_struct *work)
{
    char *pBuf;
    u32 bufLen;
    u32 dataLen;
    HW_SIM_T *pHwSim = container_of(work, HW_SIM_T, deDataWork);

    //check Rx over flag, wake up waiting task.
    if(pHwSim->state == HW_SIM_STATE_RX)
    {
        bufLen = hwRegRead32((const void *)DATA_RX_LEN_REG);
        if((bufLen & (1U << 31)) == 0)
        {
            up(&pHwSim->simIsDone);
            pHwSim->state = HW_SIM_STATE_IDLE;
            return;
        }
    }
    
    //check registers, copy data from de driver to dest space if Tx.
    pBuf = (char *)hwRegRead32((const void *)DATA_TX_ADDR_REG);
    bufLen = hwRegRead32((const void *)DATA_TX_LEN_REG);
    dataLen = bufLen & 0xFFFF;

    pHwSim->errCode = DV_OK;
    pHwSim->state = HW_SIM_STATE_TX;
    if((pBuf != NULL) && (bufLen & (1U << 31)) && dataLen)
    {
        if(hwDataDestCopyIn(pBuf, dataLen, 1) != dataLen)
        {
            DV_DBG(" hwDataDestCopyIn failed.\n");
            //return DV_ERROR;
            bufLen |= 1UL << 30;
            pHwSim->errCode = DV_ER_NOMEM;
        }
    }
    else
    {
        pHwSim->errCode = DV_ER_EXCPT;
        bufLen |= 1UL << 29;
    }

    //set/clear hw register bit.
    bufLen &= ~(1UL << 31);     //clear bit
    hwRegWrite32(bufLen, (volatile void *)DATA_TX_LEN_REG);

    //call dut process routine, sample:
    if(hwDeIrqInfo.handler != NULL)
    {
        hwDeIrqInfo.handler(hwDeIrqInfo.irq, hwDeIrqInfo.dev);
    }
    else
    {
        hwSimInfo.errCode = DV_ER_NOTFOUND;
    }
    
    //wake up test task.
    hwRegGoalSet((volatile void *)DATA_TX_LEN_REG, 0);
    up(&pHwSim->simIsDone);

    pHwSim->state = HW_SIM_STATE_IDLE;
}

int hwSimRegReset(void)
{
	if(hwSimInfo.pRegBaseAddr == NULL)
	{
		//malloc register space and init with default data.
		hwSimInfo.pRegBaseAddr = kmalloc(sizeof(hwRegGroup), GFP_KERNEL);
		if(hwSimInfo.pRegBaseAddr == NULL)
		{
			DV_DBG("%s: no mem for regSpace!\n", __FUNCTION__);
			return DV_ER_NOMEM;
		}
	}

	memcpy(hwSimInfo.pRegBaseAddr, hwRegGroup, sizeof(hwRegGroup));
	hwSimInfo.regSpaceLen = sizeof(hwRegGroup);

	return DV_OK;
}

int hwSimIsDone(int timeout)
{
	long jeffies;

	if(timeout == -1)
	{
	if(down_interruptible(&hwSimInfo.simIsDone) == 0)
		return DV_OK;
	else
		return DV_ERROR;
	}

	jeffies = msecs_to_jiffies(timeout);
	if(down_timeout(&hwSimInfo.simIsDone, jeffies) == 0)
		return DV_OK;
	else
		return DV_ERROR;
}

int hwErrCodeGet(void)
{
	return hwSimInfo.errCode;
}

int hwSimUsrDefineOp(char *pBuf, int bufLen, int mode)
{
	//add user operation
	return DV_ERROR;
}

inline u32 hwDeRegRead32(const volatile void *addr);

int hwSimInit(void)		//initialize hw registers, data mem, and so on.
{
    memset(&hwSimInfo, 0, sizeof(hwSimInfo));
    memset(&hwDeIrqInfo, 0, sizeof(hwDeIrqInfo));
    
    //malloc register space and init with default data.
    hwSimInfo.pRegBaseAddr = kmalloc(sizeof(hwRegGroup), GFP_KERNEL);
    if(hwSimInfo.pRegBaseAddr == NULL)
    {
        DV_DBG("%s: no mem for regSpace!\n", __FUNCTION__);
        return DV_ER_NOMEM;
    }
    
    memcpy(hwSimInfo.pRegBaseAddr, hwRegGroup, sizeof(hwRegGroup));
    hwSimInfo.regSpaceLen = sizeof(hwRegGroup);

    //malloc dats src space and zeroed it.
    hwSimInfo.pDataSrcAddr = kmalloc(DATA_SRC_SPACE_SIZE, GFP_KERNEL | __GFP_ZERO);
    if(hwSimInfo.pDataSrcAddr == NULL)
    {
        DV_DBG("%s: no mem for dataDestSpace!\n", __FUNCTION__);
        kfree(hwSimInfo.pRegBaseAddr);
        hwSimInfo.pRegBaseAddr = NULL;
        
        return DV_ER_NOMEM;
    }
    hwSimInfo.dataSrcSpaceLen = DATA_SRC_SPACE_SIZE;
    
    //malloc dats dest space and zeroed it.
    hwSimInfo.pDataDestAddr = kmalloc(DATA_DEST_SPACE_SIZE, GFP_KERNEL | __GFP_ZERO);
    if(hwSimInfo.pDataDestAddr == NULL)
    {
        DV_DBG("%s: no mem for dataDestSpace!\n", __FUNCTION__);
        kfree(hwSimInfo.pDataSrcAddr);
        hwSimInfo.pDataSrcAddr = NULL;
        kfree(hwSimInfo.pRegBaseAddr);
        hwSimInfo.pRegBaseAddr = NULL;
        
        return DV_ER_NOMEM;
    }
     hwSimInfo.dataDestSpaceLen = DATA_DEST_SPACE_SIZE;
     
    //malloc dats dest space and zeroed it.
    hwSimInfo.pDataHookAddr = kmalloc(DATA_HOOK_SPACE_SIZE, GFP_KERNEL | __GFP_ZERO);
    if(hwSimInfo.pDataHookAddr == NULL)
    {
        DV_DBG("%s: no mem for dataDestSpace!\n", __FUNCTION__);
        kfree(hwSimInfo.pDataDestAddr);
        hwSimInfo.pDataDestAddr = NULL;
        
        kfree(hwSimInfo.pDataSrcAddr);
        hwSimInfo.pDataSrcAddr = NULL;
        
        kfree(hwSimInfo.pRegBaseAddr);
        hwSimInfo.pRegBaseAddr = NULL;
        
        return DV_ER_NOMEM;
    }
     hwSimInfo.dataHookSpaceLen = DATA_HOOK_SPACE_SIZE;
    //printk("read32: %x\n", hwDeRegRead32(0xB01C000C));

    init_MUTEX(&hwSimInfo.regLock);
    init_MUTEX(&hwSimInfo.dataSrcLock);
    init_MUTEX(&hwSimInfo.dataDestLock);
    init_MUTEX(&hwSimInfo.dataHookLock);
    init_MUTEX_LOCKED(&hwSimInfo.simIsDone);

    hwSimInfo.pHwSimWq = create_workqueue("hwSimWq");
    INIT_WORK(&hwSimInfo.dvDataWork, hwDataRxInProcess);
    INIT_WORK(&hwSimInfo.deDataWork, hwDataTxOutProcess);
    
    hwSimInfo.state = HW_SIM_STATE_IDLE;
    
    return DV_OK;
}

void hwSimExit(void)
{
    destroy_workqueue(hwSimInfo.pHwSimWq);
    memset(&hwDeIrqInfo, 0, sizeof(hwDeIrqInfo));

    if(hwSimInfo.pDataHookAddr != NULL)
        kfree(hwSimInfo.pDataHookAddr);

    if(hwSimInfo.pDataDestAddr != NULL)
        kfree(hwSimInfo.pDataDestAddr);
    
    if(hwSimInfo.pDataSrcAddr != NULL)
        kfree(hwSimInfo.pDataSrcAddr);
    
    if(hwSimInfo.pRegBaseAddr != NULL)
        kfree(hwSimInfo.pRegBaseAddr);
    
    memset(&hwSimInfo, 0, sizeof(hwSimInfo));
}

/*
 *$Log$
 */

