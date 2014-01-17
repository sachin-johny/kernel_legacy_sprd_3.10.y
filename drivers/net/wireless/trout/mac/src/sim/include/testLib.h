/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: testLib.h
 * Abstract: 
 *     
 * Author:   Lucy
 * Version:  1.00
 * Revison Log:
 *     2012/03/17, Lucy: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _TESTLIB_H
#define _TESTLIB_H

// define  device driver type
typedef enum
{
    DUT_TYPE_DEVCHAR = 1,
    DUT_TYPE_DEVBLOCK,
    DUT_TYPE_DEVNET
}DUT_TYPE;

#define MAX_BUF_LEN  0x10000   
#define DV_DATA_POOL_MAX_SIZE 0x20000  //128k
#define MAX_PACKET_LEN 2000

#define MAX_CFG_VALUE_LEN  64

//struct of device operation
typedef struct de_interface_op_s{
    int (*deDrvOpen)(const char *pFileName);
    int (*deDrvRead)(int fd, char *pBuf, int bufLen);  
    int (*deDrvWrite)(int fd, char *pBuf, int bufLen); 
    int (*deDrvIoctl)(int fd, int cmd,int data);
    void (*deDrvClose)(int fd);
}DE_INTERFACE_OP_T;

//struct of check rule
typedef struct dv_check_rule_s{
    u8  valid;   //0: invalid ;1:valid
    u8  type;
    u8  op;
    u8  res;
    u32 mask;
    u32 addr;
    u32 data;
        
}DV_CHECK_RULE_T;

typedef struct dv_transaction_s{
    
    DV_DB_CELL_T *pCaseCell;   //å
    DV_DB_CELL_T *pResultCell;
    DV_CHECK_RULE_T *pCheckRule;

}DV_TRANSACTION_T;

int dvRegRead8(u32 addr);			//read 8bit register
int dvRegRead16(u32 addr);          //read 16bit register
int dvRegRead32(u32 addr);          //read 32bit register

int dvRegWrite8(u32 addr, u8 data);			//write 8bit register
int dvRegWrite16(u32 addr, u16 data);       //write 16bit register
int dvRegWrite32(u32 addr, u32 data);       //write 32bit register

int dvDataTx(DV_DB_CELL_T *pDbCell);		//send data to device 
int dvDataReadTxHook(DV_DB_CELL_T *pDbCell);//get data form tx stream at the hook point
int dvDataReadTxOut(DV_DB_CELL_T *pDbCell); //get device output data

int dvDataRx(DV_DB_CELL_T *pDbCellBuf, int bufLen); //receive data from device
int dvDataReadRxHook(DV_DB_CELL_T *pDbCell); //get data from input stream at the hook point
int dvDataWriteRxIn(DV_DB_CELL_T *pDbCell);//set device input data for simulator


int dvIsDone(int timeout);						//check if SIM complete.
int dvRegReadAll(DV_REG_T *pRegBuf, int bufLen); //read all registers value
int dvRegReset(void);  //reset all registers to default value
int dvUsrDefineOp(char *pBuf, int bufLen);

int dvInit(const char *pDvFileName,const char *pDeFileName,DUT_TYPE dutType);
void dvExit(void);



#endif /* #ifndef _TESTLIB_H */

/*
 *$Log$
 */

