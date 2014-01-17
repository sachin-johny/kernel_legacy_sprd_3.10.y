/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: dv_log.h
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
#ifndef _DV_LOG_H
#define _DV_LOG_H

//define log type
#define DV_LOG_TYPE_REG_RD  0
#define DV_LOG_TYPE_REG_WT  1

#define DV_LOG_TYPE_DATA_RX 2		//from DE drv to testcase
#define DV_LOG_TYPE_DATA_TX 3		//from testcase to DE drv
#define DV_LOG_TYPE_DATA_RXIN  4    //from testcase to DV drv
#define DV_LOG_TYPE_DATA_TXOUT 5	//from DV drv to  testcase
#define DV_LOG_TYPE_DATA_RXHOOK 6   //from DE drv to  kernel
#define DV_LOG_TYPE_DATA_TXHOOK 7   //from kernel to DE drv  

#define DV_LOG_TYPE_GLOBAL_VAR  8   //for global variable debug

#define DV_LOG_TYPE_DONE  9

#define DV_LOG_MAX_SIZE 0x4000  //16k


typedef struct dv_log_cell_s{
	u8 valid;
	u8 type;			//0: register read
	u32 timeStamp;
	u32 dataLen;
	u8 data[0];
}DV_LOG_CELL_T;

int dvLog(DV_LOG_CELL_T *logCell,u32 *data);  //get log message

#endif /* #ifndef _DV_LOG_H */

/*
 *$Log$
 */

