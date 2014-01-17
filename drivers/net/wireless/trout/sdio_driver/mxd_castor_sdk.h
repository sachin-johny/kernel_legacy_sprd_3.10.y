/*!
* \file    <mxd_castor_sdk.h>
* \brief
* \par    Include files
* \par    (C) 2013 Maxscend Technologies Inc.
* \version
* Revision of last commit: $Rev: 66 $
* Author of last commit  : $Author: maxscend\zhenlin.liu $
* Date of last commit    : $Date: 2013-09-28 17:09:04 +0800 (??, 28 ?? 2013) $
*
*/

#ifndef _CASTOR_SDK_H_
#define _CASTOR_SDK_H_


#ifdef __cplusplus
extern "C"{
#endif

#ifndef __USE_32BIT_PROCESSOR__
#define __USE_32BIT_PROCESSOR__
typedef unsigned char MXD_U8, *PMXD_U8;
typedef unsigned short MXD_U16, *PMXD_U16;
typedef unsigned int MXD_U32, *PMXD_U32;
typedef signed char MXD_S8, *PMXD_S8;
typedef signed short MXD_S16, *PMXD_S16;
typedef signed int MXD_S32, *PMXD_S32;
typedef signed char MXD_BOOL;
typedef char MXD_CHAR;
#endif

#ifdef USB2SPILIB_EXPORTS
#define MXD_API_C __declspec(dllexport)
#else
#define MXD_API_C
#endif

typedef struct adjust_reg_t 
{ 
    unsigned int updatedNvFlag;     // 0x5A: rfchip calibrate; 0xA5: use parameter in current structure.
    unsigned int len;
    unsigned int nvVer;   			// NV version
    unsigned int paId; 
    unsigned int lnaId; 
    unsigned int wifiImb[2]; 		//sel clbr
    unsigned int wifiLoft[4]; 		//sel clbr
    unsigned int dacScaleCh[14]; 	
    unsigned int wifiPaTbl[32]; 	//sel clbr
    unsigned int wifiTxGain[2]; 	
    unsigned int wifiPadTbl[32]; 	//sel clbr
    unsigned int btLoft[4]; 		//sel clbr
    unsigned int poutSelfChk[18];	
    unsigned int reserved[8]; //user costom		  //gIdxWfTxPwrShift =reserved[5];
                                                                       //gWfTxScaleCfg =reserved[6];	    
    unsigned int btImb[2];			//sel clbr
    unsigned int btPaTbl[6]; 		//sel clbr
    unsigned int btPadTbl[6]; 		//sel clbr
    unsigned int wifiDcoc[32*2];	//sel clbr
    unsigned int btDcoc[32];		//sel clbr
    unsigned int reserved2[50];		//0926 //reserved2[0~11] txlevel 1,2,3,4 for b g n
      								//zl@0926  reserved2[12] for phy23 config bit[7:0]=goodch  bit[15:8]deepCh
      								//zl@0926  reserved2[13] for phy44 donfig bit[7:0]=goodch  bit[15:8]deepCh
} ADJUST_REG_T, *P_ADJUST_REG_T, CLBR_PARA_S, *P_CLBR_PARA_S;
typedef struct cfgFromNv_t 
{ 
    unsigned int updatedNvFlag;     // 0: rfchip calaberate; 1: use parameter in current structure.
    unsigned int len;
    unsigned int nvVer;   			// NV version
    unsigned int paId; 
    unsigned int lnaId; 
    unsigned int dacScaleCh[14]; 	
    unsigned int poutSelfChk[18];
    int               poutShiftIdx; //user costom		        //gIdxWfTxPwrShift =reserved[5];
    unsigned int poutDacScale; //user costom	 //gWfTxScaleCfg =reserved[6];
    unsigned int phy;
    unsigned int reserved2[50];		//0926 //reserved2[0~11] txlevel 1,2,3,4 for b g n
} CFG_FROM_NV_T, *P_CFG_FROM_NV_T;
// function
int MxdRfInit(void * pClbrRslt);
int MxdRfSetBtMode(unsigned short flagWorkMode) ;
int MxdRfSetFmMode(unsigned short flagWorkMode) ;
int MxdRfSetWifiMode(unsigned short flagWorkMode);
void castorLcPllInit(void);
int  MxdRfSetFreqHKhzFm(unsigned int freqHKHz);
int  MxdRfSetFreqWifiCh(unsigned int chNo);
int  MxdRfSetFreqMHzBt(unsigned int freqMzh);
int MxdRfGetClbrPara(P_CLBR_PARA_S pRegVal) ;
int MxdRfSetClbrPara(P_CLBR_PARA_S pRegVal);
void MxdRfPulse(MXD_S32  sIntRssiDb, MXD_U32 rate1M ,MXD_U32 rate2_0_5M);


#ifndef _BIT0
#define _BIT0  0
#define _BIT1  1
#define _BIT2  2
#define _BIT3  3
#define _BIT4  4
#define _BIT5  5
#define _BIT6  6
#define _BIT7  7
#define _BIT8  8
#define _BIT9  9
#define _BIT10 10
#define _BIT11 11
#define _BIT12 12
#define _BIT13 13
#define _BIT14 14
#define _BIT15 15
#define _BIT16 16
#define _BIT17 17
#define _BIT18 18
#define _BIT19 19
#define _BIT20 20
#define _BIT21 21
#define _BIT22 22
#define _BIT23 23
#define _BIT24 24
#define _BIT25 25
#define _BIT26 26
#define _BIT27 27
#define _BIT28 28
#define _BIT29 29
#define _BIT30 30
#define _BIT31 31
#endif

// MAX macro
#define _MAX(a,b) ((a)>(b)?(a):(b))
// MIN macro
#define _MIN(a,b) ((a)<(b)?(a):(b))
// ABS macro
#define _ABS(a) ( (a)<0?((a)*(-1)):(a))
// Bit Set
#define _BS(a,b)  (a) = (a) | (1<<(b))
// Bit Clear
#define _BC(a,b)  (a) = (a) & ( ~(1<<(b)) )
// BitS Clear
#define _BSC(x,hb,lb) (x) =  (x) &  (  ~( (2<<(hb))-(1<<(lb)) )  )

// Bits Get
#define _BSG(x,hb,lb) ( (   (x)&( ((2<<(hb)) -(1<<(lb) )    )  ))           >>(lb))
// BitS Set
#define _BSS(x,hb,lb,val)  _BSC(x,hb,lb);\
    (x) = (x) | (  ( (2<<(hb))-(1<<(lb)) ) & ((val&((1<<(hb-lb+1))-1))<<(lb))  )
#define _REGBS(reg,bn)      \
    do{ \
    MXD_U32 regVal = 0; \
    rf_reg_read( reg,  &regVal ); \
    _BS(regVal,bn); \
    rf_reg_write(reg, regVal ); \
    /* mxdDbgInfo("[mxd] ::  _REGBS [%x,%x] \r\n",reg,regVal); */\
    }while(0)

// RF_REG Bit clean
#define _REGBC(reg,bn)      \
    do{ \
    MXD_U32 regVal = 0; \
    rf_reg_read( reg,  &regVal ); \
    _BC(regVal,bn); \
    rf_reg_write(reg, regVal ); \
    /*mxdDbgInfo("[mxd] ::  _REGBC [%x,%x] \r\n",reg,regVal); */ \
    }while(0)

// RF_REG Bits set
#define _REGBSS(reg_x,hb,lb,val) \
    do{ \
    MXD_U32 regVal = 0; \
    rf_reg_read( reg_x,  &regVal ); \
    _BSS(regVal,hb,lb,val); \
    rf_reg_write(reg_x, regVal ); \
    }while(0)

//wait until (reg.bitn==1)
#define _REG_CHK(reg,bitn) \
{\
    MXD_U32 regVal = 0; \
    do{ \
    rf_reg_read( reg,  &regVal ); \
    }while(0==(regVal&(1<<bitn)));\
}
#define _REG_CHK2(reg,bitn) \
{\
    MXD_U32 regVal = 0; \
    MXD_U32 maxCheck = 0;\
    do{ \
    rf_reg_read( reg,  &regVal ); \
    }while((0==(regVal&(1<<bitn)))&&((maxCheck++)<100));\
    if(maxCheck==100) mxdDbgInfo("[mxd] [%s]_REG_CHK2[%x].bit%d check=%d \r\n",__FUNCTION__,regVal,bitn, maxCheck);\
}
#define _REG_CHANGE(reg,val) \
{\
    MXD_U32 regVal = 0, tmp=val;\
	rf_reg_read( reg,  &regVal ); \
	rf_reg_write( reg,  val ); \
    mxdDbgInfo("[mxd] _REG_CHK2[%x] =0x%04x -> %04x\r\n",reg,regVal,tmp );\
}
#define _FLAG_MODE_BT 0
#define _FLAG_MODE_WIFI 1
#define _FLAG_FREQ_500K 0
#define _FLAG_FREQ_1M   1

#define _TROUT_ANT_SEL_REG_   0x58

#define   _rfStdStart        rf_reg_write(frxp_rcstd_bgn,0|(1<<0));\
do{ \
  MXD_U32  regVal=0; \
  rf_reg_read(1, &regVal);\
 }while(0);
//ctrl_fm_adc_ctrl_reg.bit11=0     0: convert true code to complementary code    1: use fm adc output data directly
#define   _rfFmAdcOutCmpl     _REGBC(ctrl_fm_adc_ctrl_reg,_BIT11)
#define   _rfFmAdcOutDrct      _REGBS(ctrl_fm_adc_ctrl_reg,_BIT11)

/* read sysReg0x58 */
#define _GET_TROUT2_ANT_SEL_  \
 do{ \
  MXD_U32  regVal=0; \
  trout_reg_read(_TROUT_ANT_SEL_REG_, &regVal); \
  rf_reg_write(_RFREG_SW_ANTSEL_BK, regVal);\
  mxdDbgInfo("[mxd] V2xMobile::  _GET_TROUT2_ANT_SEL_  %x \r\n",regVal);\
 }while(0);

#define _SET_TROUT2_ANT_SEL_NORMAL     \
 do{ \
  MXD_U32  regVal=0; \
  rf_reg_read(_RFREG_SW_ANTSEL_BK, &regVal);\
  trout_reg_write(_TROUT_ANT_SEL_REG_, regVal); \
  mxdDbgInfo("[mxd] V2xMobile::  _SET_TROUT2_ANT_SEL_NORMAL  %x \r\n",regVal);\
 }while(0);

/*bit[5:3]] antiSelRX*/
#define _SET_TROUT2_ANT_SEL_DCOC      \
 do{ \
  MXD_U32  regVal=0; \
  rf_reg_read(_RFREG_SW_ANTSEL_BK, &regVal);\
  regVal = (regVal>>3)&0x07;\
  trout_reg_write(_TROUT_ANT_SEL_REG_, ( regVal<<9)|( regVal<<6)| ( regVal<<3)  |regVal );\
  mxdDbgInfo("[mxd] V2xMobile::  _SET_TROUT2_ANT_SEL_DCOC  [all Rx] %x \r\n",( (regVal<<9)|( regVal<<6)|( regVal<<3)|regVal ));\
 }while(0);

/*bit[11:9]] antiSel_wifi_TX*/
//for SWITCH_PA_LNA_8025
#if 1
#define	_BIT_WIFI_LOFT_POSI  (9)
#else
/*no pa , so we use bt txPath*/
#define 	_BIT_WIFI_LOFT_POSI  (6)
#endif  

#define 	_BIT_BT_LOFT_POSI  (6)
 /*When clbr tx_wf without PA */
#define _SET_TROUT2_ANT_SEL_LOFT_WIFI     \
 do{ \
  MXD_U32  regVal=0; \
  rf_reg_read(_RFREG_SW_ANTSEL_BK, &regVal);\
  regVal = (regVal>>_BIT_WIFI_LOFT_POSI)&0x07; \
  trout_reg_write(_TROUT_ANT_SEL_REG_, ( regVal<<9)|( regVal<<6)| ( regVal<<3)  |regVal );\
  mxdDbgInfo("[mxd] ::  _SET_TROUT2_ANT_SEL_LOFTWifi [all wifiTx]  %x \r\n",( (regVal<<9)|( regVal<<6)| ( regVal<<3) |regVal )); \
 }while(0);

/*bit[8:6]] antiSel_wifi_TX*/
#define _SET_TROUT2_ANT_SEL_LOFT_BT     \
 do{ \
  MXD_U32  regVal=0; \
  rf_reg_read(_RFREG_SW_ANTSEL_BK, &regVal);\
  regVal = (regVal>>_BIT_BT_LOFT_POSI)&0x07; \
  trout_reg_write(_TROUT_ANT_SEL_REG_, ( regVal<<9)|( regVal<<6)| ( regVal<<3)  |regVal );\
  mxdDbgInfo("[mxd] ::  _SET_TROUT2_ANT_SEL_LOFT [all btTx]  %x \r\n",( (regVal<<9)|( regVal<<6)| ( regVal<<3) |regVal )); \
 }while(0); 

 /*bit[2:0]] antiSel_wifi_TX*/
#define _SET_TROUT2_ANT_SEL_SWT_OFF     \
 do{ \
  MXD_U32  regVal=0; \
  rf_reg_read(_RFREG_SW_ANTSEL_BK, &regVal);\
  regVal = (regVal)&0x07; \
  trout_reg_write(_TROUT_ANT_SEL_REG_, ( regVal<<9)|( regVal<<6)| ( regVal<<3)  |regVal );\
  mxdDbgInfo("[mxd] _SET_TROUT2_ANT_SEL_SWT_OFF::  _SET_TROUT2_ANT_SEL_SWT_OFF [all off]  %x \r\n",( (regVal<<9)|( regVal<<6)| ( regVal<<3) |regVal )); \
 }while(0); 

#ifdef __cplusplus
   }
#endif

#endif


/* _CASTOR_REG_H_ */
