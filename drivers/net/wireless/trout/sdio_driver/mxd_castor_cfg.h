/*!
* \file    <mxd_castor_cfg.h>
* \brief
* \par    Include files
* \par    (C) 2013 Maxscend Technologies Inc.
* \version
* Revision of last commit: $Rev: 66 $
* Author of last commit  : $Author: maxscend\zhenlin.liu $
* Date of last commit    : $Date: 2013-09-28 17:09:04 +0800 (÷‹¡˘, 28 æ≈‘¬ 2013) $
*
*/
#include "mxd_castor_reg.h"
#include "mxd_castor_sdk.h"
#ifndef _CASTOR_CFG_H_
#define _CASTOR_CFG_H_
/*
//#define _MXD_DEV_     //mxdtesttool
//#define _SPRD_DEV_     //mini code
//#define _MXD_DEV_MATLIB_DLL_   //matlab dll
//#define _MXD_ANDROID_          //android for 6820/
*/
//#define _MXD_DEV_

#ifdef __GNUC__
#define _MXD_ANDROID_
#elif  WIN32
//#define _MXD_DEV_MATLIB_DLL_
#define _MXD_DEV_
#else
#define _SPRD_DEV_
#endif

//#define NEWCHIPID 0xCA   //chip id old is 0xc7 , new is 0xca 
#define _CHIP_ID_C7 0xc7
#define _CHIP_ID_C8 0xc8
#define _CHIP_ID_C9 0xc9
#define _CHIP_ID_CA 0xca

#ifndef _MXD_DEV_
typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned long int uint32;
typedef unsigned long int  uint64;
#define VOID void
#define MXD_API
#define NULL 0
#define IN
#define OUT
#define CONST const
#endif
#define TROUT2_RF_BASE_ADDR 0x1000
#define RF_Clbr_Data_LENGTH 258

#ifdef _MXD_DEV_
#include <windows.h>
#include <stdlib.h>
#include "..\usb2spi\mxd_porting_silicon.h"
//#include "..\mxd_porting_silicon.h"
void * ghspi=NULL;
VOID MXD_API OAL_DebugPrint(IN CONST MXD_CHAR* dbgOutput, ...);
#define mxdDbgInfo OAL_DebugPrint
#define mxdDbgInfo1       //
#define mxdDbgInfo2       //

#define rf_reg_read(regAddr,regVal)  MxdSpiReadTroutReg(ghspi,regAddr+TROUT2_RF_BASE_ADDR,(MXD_U32 *)regVal)
#define rf_reg_write(regAddr,regVal)  MxdSpiWriteTroutReg(ghspi,regAddr+TROUT2_RF_BASE_ADDR,regVal)
#define trout_reg_read(regAddr,regVal)  MxdSpiReadTroutReg(ghspi,regAddr,regVal)
#define trout_reg_write(regAddr,regVal)  MxdSpiWriteTroutReg(ghspi,regAddr,regVal)
#if 0
#define MxdSleep(n) do{\
	MXD_U32 iiii=0;\
	if (n<500)	{\
	/*for(iiii=0; iiii++;iiii<(n*750));;*/ for(iiii=0;iiii<(n*0x003fffff); iiii++);}\
	else	{\
	Sleep(n);	}\
}while (0)
#else
#define MxdSleep(n) do{\
	MXD_U32 iiii=0;\
	if (n<500)	{\
	for(iiii=0;iiii<(n*750); iiii++);}\
	else	{\
	Sleep(n);	}\
}while (0)
#endif

#endif

#ifdef _MXD_DEV_MATLIB_DLL_
//#include <stdafx.h>
#include "..\f320api.h"
#include "..\usb2spilib.h"
void OAL_DebugPrint(IN CONST MXD_CHAR* dbgOutput, ...);
#define mxdDbgInfo(a) //OAL_DebugPrint(a)
#define mxdDbgInfo1       //
#define mxdDbgInfo2       //
#define rf_reg_read(regAddr,regVal)     Mxd_SpiReadCastorReg(regAddr,(MXD_U32 *)regVal)
#define rf_reg_write(regAddr,regVal)    Mxd_SpiWriteCastorReg(regAddr,regVal)
#define trout_reg_read(regAddr,regVal)      Mxd_SpiReadTroutReg(regAddr,regVal)
#define trout_reg_write(regAddr,regVal)     Mxd_SpiWriteTroutReg(regAddr,regVal)
#endif

#ifdef _MXD_ANDROID_
#include <linux/delay.h>
extern unsigned int host_read_trout_reg(unsigned int addr);
extern unsigned int host_write_trout_reg(unsigned int val,unsigned int addr);
static void mxd_reg_read(unsigned int reg_addr, unsigned int *reg_data)
{
    *reg_data = host_read_trout_reg(reg_addr<<2);
}
static void  mxd_reg_write(unsigned int reg_addr, unsigned int reg_data)
{
    host_write_trout_reg(reg_data, (reg_addr<<2));
}

#define rf_reg_readbase(regAddr,regVal)   do{\
    MXD_U32 nAddr=(MXD_U32)regAddr;\
    MXD_U32 val=0;\
    mxd_reg_read(nAddr+TROUT2_RF_BASE_ADDR,&val);\
    *regVal = val; \
}while (0)

#define rf_reg_write(regAddr,regVal)    do{\
    MXD_U32 nAddr=(MXD_U32)regAddr;\
    MXD_U32 nVal=(MXD_U32)regVal;\
    mxd_reg_write(nAddr+TROUT2_RF_BASE_ADDR,nVal);\
}while (0)
#define rf_reg_read(regAddr,regVal)   do{\
    MXD_U32 nAddr=(MXD_U32)regAddr;\
    MXD_U32 val=0;\
    if (nAddr<0x600)\
    {\
             mxd_reg_read(nAddr+TROUT2_RF_BASE_ADDR,&val);\
    }\
    else\
    {\
    	rf_reg_write(rfmux_mem_read_addr_reg,nAddr);\
	rf_reg_write(rfmux_mem_read_trig,0x01);\
       mxd_reg_read(rfmux_mem_read_data_reg+TROUT2_RF_BASE_ADDR,&val);\
    }\
    *regVal = val; \
}while (0)

#define trout_reg_read(regAddr,regVal)  do{\
    MXD_U32 nAddr=(MXD_U32)regAddr;\
    MXD_U32 pVal=0;\
    mxd_reg_read(nAddr,&pVal);\
    *regVal = pVal; \
}while (0)

#define trout_reg_write(regAddr,regVal)  do{\
    MXD_U32 nAddr=(MXD_U32)regAddr;\
    MXD_U32 nVal=(MXD_U32)regVal;\
    mxd_reg_write(nAddr,nVal);\
}while (0)

#define mxdDbgInfo
#define mxdDbgInfo1       //
#define mxdDbgInfo2       //
//#define loftMxdDbgInfo	//
//delay us
//#define MxdSleep     udelay
#define MxdSleep(n)     udelay(n*100)
#endif

// for minicode
#ifdef _SPRD_DEV_
extern void             SCI_Sleep(MXD_U32 ms);
extern void             rf_reg_read(MXD_U32 reg_addr, MXD_U32 *reg_data);
extern void             rf_reg_write(MXD_U32 reg_addr, MXD_U32 reg_data);
extern MXD_U32     trout_read_reg(MXD_U32 reg_addr, MXD_U32 *reg_data);
extern MXD_U32     trout_write_reg(MXD_U32 reg_addr, MXD_U32 reg_data);
extern MXD_U32     SCI_TraceLow(const char *x_format, ...);
#define trout_reg_write trout_write_reg


//debug control
#define mxdDbgInfo      SCI_TraceLow
#define mxdDbgInfo1       SCI_TraceLow
#define mxdDbgInfo2       //
#define _wfLoftMxdDbgInfo	//
#define _btLoftMxdDbgInfo	//
#define _wfTxImbMxdDbgInfo //
#define _btTxImbMxdDbgInfo	//

#define MxdSleep           SCI_Sleep

#define trout_reg_read trout_read_reg
#define trout_reg_write trout_write_reg

#endif

/* config Macro */

#define rf_mem_read(regAddr,regVal) rf_reg_write(rfmux_mem_read_addr_reg,regAddr);\
    rf_reg_write(rfmux_mem_read_trig,0x01);\
    rf_reg_read(rfmux_mem_read_data_reg,regVal)

#define rf_mem_write(regAddr,regVal) rf_reg_write(regAddr,regVal)

/*
#ifdef USB2SPILIB_EXPORTS
#define MXD_API_C __declspec(dllexport)
#else
#define MXD_API_C
#endif
*/



#if 0
#define _TROUT_REG_SPI_CFG 0x21   /* trout */
#else
#define _TROUT_REG_SPI_CFG 0x44   /* trout2 */
#endif
#define _TROUT_REG_ADI_CFG 0x4012
//
#if 0
#define _HW_SWITCH_13317
#else
//#define _HW_SWITCH_6603_NEW
//_HW_SWITCH_6603_NEW //_HW_SWITCH_6603_OLD
	//_HW_SWITCH_8205_V1_3_0 	//switch8250_lna8250_Pa_8250
	//_HW_SWITCH_8205_V1_3_1	//no used
	//#define _HW_SWITCH_8205_V1_3_0
	

//_HW_SWITCH_13317   //No_Pa,No_Lna
//_HW_TroutII_SWTRT6627_LNART6627_PANO_
//_HW_TroutII_SWTFM8205_LNAFM8205_PAFM8205_
//_HW_TroutII_SWT_XM2400_LNA_XM2400_PA_NONE_
//_HW_TroutII_SWT_XM2400_LNA_XM2400_PA_2574_
#define _HW_TroutII_SWT_XM2400_LNA_XM2400_PA_NONE_


//#define _HW_TroutII_2_SWITCH6627_LNA6627_PANO_

//#define _HW_TroutII_2_SWITCH6627_LNA6627_PA6642_
#endif

#define _TROUT_ANT_SEL_REG_   0x58
 #define _TROUT_M_CLK_CTRL1 0x44
  #define _TROUT_M_XTLEN_REG 0x05


//#define _HW_SWITCH_13317
//#define _HW_SWITCH_6603_OLD
//#define _HW_SWITCH_6603_NEW
#ifdef  _HW_SWITCH_13317
// switch13331:: 0off 1on
// sel0: Tx 001b    sel1:Rx  010b
#define _HW_LNA_NONE
#define _HW_PA_NONE
#define _SET_TROUT2_ANT_SEL_INIT     trout_reg_write(_TROUT_ANT_SEL_REG_,0x250);\
    mxdDbgInfo("[mxd]   _SET_TROUT2_ANT_SEL_INIT0x250!\r\n")

#elif defined( _HW_SWITCH_6603_OLD )
#define _HW_LNA_NONE
#define _HW_PA_NONE
#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x264);\
    mxdDbgInfo("[mxd] V2xEVB::  _SET_TROUT2_ANT_SEL_INIT 0x264 \r\n");

#elif defined( _HW_SWITCH_6603_NEW )
#define _HW_LNA_NONE
#define _HW_PA_NONE
#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x488);\
    mxdDbgInfo("[mxd] V2xMobile::  _SET_TROUT2_ANT_SEL_INIT  0x488 \r\n");
#elif defined( _HW_SWITCH_8205_V1_3_0 )
#define _HW_PA_8205
#define _HW_LNA_8205
#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x850);\
	mxdDbgInfo("[mxd] _HW_SWITCH_8205_V1_3_0:  ANT_SEL_INIT  0x850 \r\n");

#elif defined( _HW_SWITCH_8205_V1_3_1 )

#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x850);\
	mxdDbgInfo("[mxd] _HW_SWITCH_8205_V1_3_1::  ANT_SEL_INIT:    0x850 \r\n");

#define _HW_LNA_8205
#define _HW_PA_8205

#elif defined( _HW_TroutII_2_SWITCH6627_LNA6627_PANO_ )

#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x850);\
	mxdDbgInfo("[mxd] _HW_TroutII_2_SWITCH6627_LNA6627_PANO_::  ANT_SEL_INIT:    0x850 \r\n");

#define _HW_LNA_8205
#define _HW_PA_8205
	//#define _HW_PA_NONE

#elif defined( _HW_TroutII_SWT_XM2400_LNA_XM2400_PA_NONE_ )
#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x250);\
	mxdDbgInfo("[mxd] _HW_TroutII_SWT_XM2400_LNA_XM2400_PA_NONE_::  ANT_SEL_INIT:    0x250 \r\n");

#define _HW_LNA_XM2400
#define _HW_PA_NONE

#elif defined( _HW_TroutII_SWT_XM2400_LNA_XM2400_PA_2574_ )

#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x850);\
	mxdDbgInfo("[mxd] _HW_TroutII_SWT_XM2400_LNA_XM2400_PA_NONE_::  ANT_SEL_INIT:    0x850 \r\n");

#define _HW_LNA_XM2400
#define _HW_PA_2574

#elif defined( _HW_TroutII_SWTRT6627_LNART6627_PANO_ )
#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x250);\
	mxdDbgInfo("[mxd] _HW_TroutII_MURATA_SWTRT6627_LNART6627_PANO_::  ANT_SEL_INIT:    0x250 \r\n");

#define _HW_LNA_RT6627
#define _HW_PA_NONE

#elif defined( _HW_TroutII_SWTFM8205_LNAFM8205_PAFM8205_ )
#define _SET_TROUT2_ANT_SEL_INIT      trout_reg_write(_TROUT_ANT_SEL_REG_,0x850);\
	mxdDbgInfo("[mxd] _HW_TroutII_MURATA_SWTFM8205_LNAFM8205_PAFM8205_::  ANT_SEL_INIT:    0x850 \r\n");

#define _HW_LNA_FM8205
#define _HW_PA_FM8205

#endif
#define _SET_TROUT2_ANT_SEL_INIT_FOR_SWTCH_LNA_OFF      gSwitchLnaOffBit5_3=0x240;\
	mxdDbgInfo("[mxd] _SEL_INIT_FOR_SWTCH_LNA_OFF::  gSwitchLnaOffBit5_3%x\r\n",gSwitchLnaOffBit5_3);
#define _LNA_ID_NONE  		0
#define _LNA_ID_FM8205  	1
#define _LNA_ID_RT6627		2
#define _LNA_ID_XM2400		3

#ifdef _HW_LNA_NONE 
#define _SET_TROUT2_LNA_INIT _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT7, _BIT0, _LNA_ID_NONE);\
	mxdDbgInfo("[mxd $] V2xMobile::  _SET_TROUT2_LNA_INIT  _HW_LNA_NONE \r\n");

#elif defined ( _HW_LNA_FM8205 ) 
#define _SET_TROUT2_LNA_INIT   _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT7, _BIT0, _LNA_ID_FM8205);\
	mxdDbgInfo("[mxd $] V2xMobile::  _SET_TROUT2_LNA_INIT  _HW_PA_FM8205 \r\n");

#elif defined( _HW_LNA_RT6627 )
#define _SET_TROUT2_LNA_INIT   _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT7, _BIT0, _LNA_ID_RT6627);\
	mxdDbgInfo("[mxd $] V2xMobile::  _SET_TROUT2_LNA_INIT  _HW_LNA_RT6627 \r\n");

#elif defined( _HW_LNA_XM2400 )
#define _SET_TROUT2_LNA_INIT   _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT7, _BIT0, _LNA_ID_XM2400);\
	mxdDbgInfo("[mxd $] V2xMobile::  _SET_TROUT2_LNA_INIT  _LNA_ID_XM2400 \r\n");

#endif

#define _PA_ID_NONE  0
#define _PA_ID_FM8205  1
#define _PA_ID_2574  2

#ifdef  _HW_PA_NONE 
#define _SET_TROUT2_PA_INIT _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT15, _BIT8, _PA_ID_NONE);\
	mxdDbgInfo("[mxd $] V2xMobile::  _SET_TROUT2_PA_INIT  _PA_ID_NONE \r\n");

#elif defined(_HW_PA_FM8205)
#define _SET_TROUT2_PA_INIT _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT15, _BIT8, _PA_ID_FM8205);\
	mxdDbgInfo("[mxd $] V2xMobile::  _SET_TROUT2_PA_INIT  _LNA_ID_8205 \r\n");

#elif defined( _HW_PA_2574 )
#define _SET_TROUT2_PA_INIT _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT15, _BIT8, _PA_ID_2574);\
	mxdDbgInfo("[mxd $] V2xMobile::  _SET_TROUT2_PA_INIT  _PA_ID_2574 \r\n");
#endif

#ifndef _wfLoftMxdDbgInfo
#define _wfLoftMxdDbgInfo	//
#endif
#ifndef _btLoftMxdDbgInfo
#define _btLoftMxdDbgInfo	//
#endif

#ifndef _wfTxImbMxdDbgInfo
#define _wfTxImbMxdDbgInfo	//
#endif

#ifndef _wfTxPoutMxdDbgInfo
#define _wfTxPoutMxdDbgInfo	mxdDbgInfo
#endif
#ifndef _btTxImbMxdDbgInfo
#define _btTxImbMxdDbgInfo	//
#endif
#endif /*end  <mxd_castor_cfg.h>*/
