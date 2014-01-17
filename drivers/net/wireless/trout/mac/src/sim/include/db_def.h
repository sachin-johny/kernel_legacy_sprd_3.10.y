/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: db_def.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _DB_DEF_H
#define _DB_DEF_H
#include "types.h"

enum {

        DV_DB_TYPE_REG_RD8 = 1,
        DV_DB_TYPE_REG_RD16 ,
        DV_DB_TYPE_REG_RD32,
        
        DV_DB_TYPE_REG_WT8,
        DV_DB_TYPE_REG_WT16,
        DV_DB_TYPE_REG_WT32,

        DV_DB_TYPE_DATA_RX, //from DE drv to testcase
        DV_DB_TYPE_DATA_TX,		//from testcase to DE drv
        DV_DB_TYPE_DATA_RXIN,    //from testcase to DV drv
        DV_DB_TYPE_DATA_TXOUT,	//from DV drv to  testcase
        DV_DB_TYPE_DATA_RXHOOK,   //from DE drv to  kernel
        DV_DB_TYPE_DATA_TXHOOK,   //from kernel to DE drv  
};


typedef struct dv_db_cell_s{
	u8 valid; 		//0: invalid, 1: valid;
	u8 type;		//0: cpu to device; 1: device to cpu; 2: kernel to device; 3:device to kernel
	u16 res;			//reserved.
	u32 checkId;     //the index of testcase,if a testcase contains several cells,those cells have same id.
	                 //checkId is used for get right check data from another database.
	u32 dataLen;		//byte length.
	u8 data[0];
}DV_DB_CELL_T;

//interface functions:
int dbOpen(const char *pFileName);
int dbCellRead(int fd,DV_DB_CELL_T *dbaseCell);
int dbCellReadByIndex(int fd, int index, DV_DB_CELL_T *dbaseCell);
void dbClose(int fd);

#endif /* #ifndef _DB_DEF_H */

/*
 *$Log$
 */

