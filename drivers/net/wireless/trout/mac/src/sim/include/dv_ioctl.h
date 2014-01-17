/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: dv_ioctl.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _DV_IOCTL_H
#define _DV_IOCTL_H

#include <linux/types.h>
#include <asm-generic/ioctl.h>
//#include "types.h"
//#include "db_def.h"
//#include "reg_def.h"
#include "dv_log.h"

struct rx_info {
	u8 is_amsdu;
	u8 status;
	u8 sub_msdus_of_buf;
	u8 bufs_of_amsdu;
	u8 buf_idx;
	u8 hdr_len;
};

struct tx_info {
	int is_amsdu;
	u8 status;
	u8 sub_msdus_of_buf;
	u8 sub_msdu_idx;
};

typedef struct dv_ioctl_s{
	int addr;
	int dataLen;
	int data;
	union {
		struct rx_info rx_info;
		struct tx_info tx_info;
	};
}DV_IOCTL_T;


//ioctl command
#define DV_MAGIC 'T'
#define DV_IOCGETREG8 _IOR(DV_MAGIC,1,DV_IOCTL_T)   //read 8bit register 
#define DV_IOCGETREG16 _IOR(DV_MAGIC,2,DV_IOCTL_T)  //read 16bit register  
#define DV_IOCGETREG32 _IOR(DV_MAGIC,3,DV_IOCTL_T)  //read 32bit register  

#define DV_IOCSETREG8 _IOW(DV_MAGIC,4,DV_IOCTL_T)  //write 8bit register  
#define DV_IOCSETREG16 _IOW(DV_MAGIC,5,DV_IOCTL_T)  //write 16bit register  
#define DV_IOCSETREG32 _IOW(DV_MAGIC,6,DV_IOCTL_T)   //write 32bit register  

#define DV_IOCGETDATA_TXHOOK _IOR(DV_MAGIC,7,DV_IOCTL_T) //read tx stream at hook point 
#define DV_IOCGETDATA_RXHOOK _IOR(DV_MAGIC,8,DV_IOCTL_T) //read rx stream at hook point
#define DV_IOCGETDATA_TXOUT _IOR(DV_MAGIC,9,DV_IOCTL_T)  //read device output data
#define DV_IOCSETDATA _IOW(DV_MAGIC,10,DV_IOCTL_T)      //set device input data

#define DV_IOCSETREG_RESET _IOW(DV_MAGIC,11,DV_IOCTL_T)  //reset all  registers to default status

#define DV_IOCGETREG_ALL _IOR(DV_MAGIC,12,DV_IOCTL_T)    //read all registers value

#define DV_IOCGET_SIMISDONE _IO(DV_MAGIC,13)             //get the flag which indicate the process is completed and you can get some information

#define DV_IOCOP_USERDEF _IOWR(DV_MAGIC,14,DV_IOCTL_T)   // user define operation

#define DV_GET_KERN_ADDR _IOWR(DV_MAGIC,15,DV_IOCTL_T)

#define DV_GET_KERN_DATA _IOWR(DV_MAGIC,16,DV_IOCTL_T)

#define DV_SET_KERN_DATA _IOWR(DV_MAGIC,17,DV_IOCTL_T)




// user op_code define
#define USER_INT_OPT 	0
#define USER_INT_STATUS 1
#define USER_TX_STATUS 	2
#define USER_RX_STATUS 	3

#endif /* #ifndef _DV_IOCTL_H */

/*
 *$Log$
 */

