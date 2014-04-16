/*!
* \file    <mxd_castor_sdk.c>
* \brief
* \par    Include files
* \par    (C) 2013 Maxscend Technologies Inc.
* \version
* Revision of last commit: $Rev: 67 $
* Author of last commit  : $Author: maxscend\zhenlin.liu $
* Date of last commit    : $Date: 2013-09-28 17:11:59 +0800 (ÖÜÁù, 28 ¾ÅÔÂ 2013) $
V1.8F
	1)Pout update 13.11.18
	update add WIFI_SHIFTIDX_CHIPIDC8 (5)
V1.8D
1) Pout update 13.11.13
//V1.8B
    1) throughput  config update 
        dcr: reg24 ->12             [chipc7 enable]
        agc: update for c7 goodch.
        phy: reg27 e->f

//V1.8 A
//chip_c7 dcr disable
    chip-c8 dcr enable
    chipId ==c8 ; update wfPaPAD [enable]\
    
//V1.8Pre
   1. tx11b Mask config!    miniCode only
   2. cfg update 
       a) workModeInit, 0x207 config :: 0x119->0xe119   ;;for mini code  debug. Host  mode 
       b)                      DCR 24   ON/OFF
       c) new chip id config
   
   4. FEM update to V1.2?  LNA same cfg??
   5. MxdRfSetClkReqLow

//0929
 	1. added new interface:: MxdRfSwtchLnaCfg
 			int  MxdRfSwtchLnaCfg(MXD_U32 flagOnOff) // 0:off  1: On
			       //if(wifiOnly)  config switch based on flagOnoff   and  return 0              
			       //if (bt/ON) config switch to default and  return 1
 	2. 11b mask txGain config shift

//0928
SDK V1.7E release to SPRD:
	1 nv control phyreg_23 phyReg44 
	2 nv support wfTxPower control 
	3 Pout auto- Caliberate 
	4 Nv support rx Lna close setting 
	5 Fm anti-spur V1.0
	6 auto config based on chipId. 
	7 Dcoc(0x1c~1f) use interpolation 
	8 power-on if NV config error protecttion.
//0909
   added fm spurch cfg

//0903 0907 pout debug
  1) added  castorRfClbrPoutWf
  2) disable freq chang txDacScale

// 0902v1.5.0
   1) wf/bt txImb phitbl update
   2) wifi DCOC update
   3) aci supprot nv.
   4) fem config v.1.1 update

//0830 v1.5.0
 1)add new Dcoc (gain28,29,30,31)
 2)updata TxImb
//0819
 1) update minicode  sync android for aci config
 2) restore wifi rx gain idx
 3) bttx filter: fd_bttx_lock_offset_reg(rfreg0x55) 0x40

//0815
 add aci config funciton
 add interface: MxdRfPulse()

//0814 v1.4.0
 1)update wifi rx gain idx
 2)update RT6603 and MX2400 Lna config (FEM CONFIG_V1.1)

// 0809 V1.3B
txImb
// 0726
    1) support  murata xm2400 switch+lna
    2) support  murata xm2400 +Pa/nonePa
    3) GDY modify: wifi PaPad tbl
//0714
 // added new ldo config.
 // update arm code config.
//0708 V1.2.B
 1) added anti-deep ch
 2) added wifi_ch_dcoc
//0626
 1) added lna/pa setting
 2) added file mxd_castor_cfg.h

//0613
 1) castorLcPll_OnOff: 0x220 ctrl

//0606
 1. fmdiable:: rfmux_fm_cfg_reg=0
 2. wfdcoc:: lna update

//0527
 1. wifi Rx gain  planB, map planB

//0524
 1) added pll control
 2) update dac control
 3) macro for antisel
 4) config V2x
//0523
  1) loft id 11b 10b 01b 00b    idx 00,10,20,30

//0512
 1) for chip V2X
//0510
 1) colisten modify

// 0507
 1) 0x409,0x40c,0x40f, modify hold
 2) Modify interface  MxdRfInit(MXD_U16 * pClbrRslt)
 3) Added interface  MxdWifiTxPowerCfg
 4) bt tx power use 5~11 [11.6,9.9,6.4,2.7,-2.8,-6.6,-10.1]

//0506:
 0x040f, 0x01c7, //@05/06, 0x00c7 // wtco tbl h
//0504
 // bt tx power

//0503:
 1. comInit:
 0x409->0x1c3        //c1
 0x40c->0x1c5        //c5
 0x209->0xc0c1       //0c1

//0502
 1. fm imb enable

// 0426
 1.   0x222,0xc230,// // bt_lomixer_cfg 0x210,0xc230 ???
      0x223,0xc230,     // wifi_lomixer_cfg 0x210 0xc230 ???
 2. MxdRfSetClbrRslt
 3. MxdRfGetClbrPara
 4. modify power control

//ZL @2013-04-18
 1. wifi dcoc update PGA_PGA
 2. loft changed.
 3. wifi agc tbl update
//ZL @2013-04-15
 1. MxdRfSetFmMode update
 2. added test functions.
 3. wifi dcoc update,agc tbl update
 4. update ldo control

// ZL@2013-04-13
 1. added BT search agc:    0x0640, 0x1bc3,     //0411 for bt search. 11 00000
 2. bt/wifi use rfReg0x41 = 0x1983
 3. bt use trout2 sysReg0x4f.bit0=1 trout2     entry[3:0] :wifiTx, btTx ,wifiRx/btRx/colisten, init

2013-04-08
 1. DCOC open lna
 2. ComInit update
 020d d27f
 020e 89cd

2013-04-08
 1. change fm 86.4Mhz
 2. change force 26Mhz
*/


//#ifdef _MXD_DEV_
//#include <windows.h>
//#endif


//#include <stdlib.h>
//#include "..\usb2spi\mxd_porting_silicon.h"

#include "mxd_castor_reg.h"
#include "mxd_castor_sdk.h"
#include "mxd_castor_cfg.h"


//ZL @@
#define _MXD_DEBUG_DCOC_SYS
//#define TRACE_DCOC_LOG
//#define _MXD_CLBR_OPTIMIZE
#ifndef _MXD_DEBUG_DCOC_SYS
#define _MXD_DEBUG_DCOC_CASTOR
#endif

#define _MXD_DEBUG_
// read 2^N times
#define _DCOC_READ_N_ 3

//zl@0816
//LNA_SEL_0: [0~0x10]   LNA_SEL_3: [0x11~1f]//
#define  GINDX_WF_LNA_CHANGE_M    0x11
//lna idx  0~9          a~12            13~1f
// zl@0605 modify
#define  GINDX_WF_LNA_CHANGE_1    0x0
#define  GINDX_WF_LNA_CHANGE_2    0xa
#define  GINDX_WF_LNA_CHANGE_3    0x13
#define  LNA_SEL_0  0
#define  LNA_SEL_2  2
#define  LNA_SEL_3  4


#define MXD_SLEEP_MS      5

#define _TBL_DCOC_ROW_LEN_  4
//clbr pga    InPgaTestPort
//clbr Tia    InTiaTestPort
#define _WIFI_DCOC_PGAPGA
//#define _WIFI_DCOC_TIAPGA

#define _MXD_WFBT_POUT_   
#define _MXD_WF_DCOC_INSERT_
#define _MXD_POUT_INDEX_VOL  (0x28)
#define _MXD_POUT_INDEX28_SUBSCRIPT  (15)		/* index28 array gWifiTxGaincfg  subscript */

#define WIFI_SHIFTIDX_CHIPIDC8	 (6)
#define  _WF_CFG_DEEP_CH_  0
#define  _WF_CFG_GOOD_CH_  1
#ifdef _MXD_ANDROID_
#define _MXD_NV_SUPPORT_
extern void write_dot11_phy_reg(unsigned char ra, unsigned int rd);
extern void read_dot11_phy_reg(unsigned char ra, unsigned int * rd); //by lihua
#define trout_write_phy_reg(ra,rd) write_dot11_phy_reg(ra,rd)
#endif

unsigned int *pNvBuf=NULL;
unsigned char g_tempr_en= 0; 
#ifdef _MXD_DEV_
#define trout_write_phy_reg(ra,rd) //
#endif

#define NOT_UPDATED 0x5A		// need to equal to the same definition in trout2_interface.c
#define ALREADY_UPDATED 0xA5	// need to equal to the same definition in trout2_interface.c
#define _MXD_POUT_REF   (29000)
#define _MXD_POUT_REF_RANGE (0x10)
#define _MXD_POUT_REF_140_MAX       (27000)
#define _MXD_POUT_REF_140_MIN        (25000)

MXD_U32 gWfTxScaleCfg =0x200;
MXD_U32 gIdxWfTxPwrShift = 0;
MXD_U32    gPoutMode =0;       //bit7:     [1:0]     [en/dis] 
MXD_S32    gPoutPcbSwtchRecover = 0; // for pcb, switch.
MXD_S32    gPoutOffsetC7C8 = 0; // for pcb, switch.
MXD_U32    gPoutRef140Max = _MXD_POUT_REF_140_MAX;
MXD_U32    gPoutRef140Min = _MXD_POUT_REF_140_MIN;
MXD_U32    gWfTxPoutRefVal = _MXD_POUT_REF;
MXD_U32    gWfTxPoutRefRange = _MXD_POUT_REF_RANGE;
MXD_U32 gDacScaleCh[16]={0,};
// gDacScaleCh[15] save the pout clbr result ininit.
//MXD_U32 gChannelNO = 0;

MXD_U32 gPhy23Cfg[2]={0x1c,0x1c};// deepch  goodch
MXD_U32 gPhy44Cfg[2]={0x1c,0x1c};//deepch  goodch
MXD_U32 gSwitchLnaOffBit5_3 = 0;

MXD_U32 gWfChFromAp = 0;//by lihua
extern ADJUST_REG_T cal_regs;
int last_tempr_ref_num=-1;
MXD_U32  castorReadTmpr(void);
void castorFmInit(void);
void castorHostBtTxRxInit(void);
void castorHostBtRxInit(void);
void castorHostBtTxInit(void);
int castorSetFreqWifiCh(MXD_U32 freqChin);
int castorSetFreqMhzBt(MXD_U32 freqMHz);
int castorSetFreqHKhzFm(MXD_U32 freqHKhz);
void castorHostWifiTxRxInit(void);
void castorHostWifiTxInit(void);
MXD_API_C void castorHostWifiRxInit(void);
//MXD_API_C void castorClbrDcocWifi(void);
MXD_API_C void castorClbrDcocWifi(unsigned int flagCh);
void castorClbrDcocBt(void);
int  castorClbrLoftBt(void);
MXD_API_C int  castorClbrLoftWifi(void);
void castorClbrTximbWifi(void);
void castorClbrPoutWifi(void);
void cfgTrout2Dac(void);
void castorSetRfEntry(int flag); // 1: bt only 2:wifi only 3:colisten
void castorDbgGetRfInfo(void);
MXD_U16 regBitsGet(MXD_U16 regAddr,MXD_U8 bitH, MXD_U8 bitL);
void MxdRfCfgForAci(int  flag);
void rfClbrShiftWftxgain(MXD_U32 minIdx, MXD_U32 maxIdx,  MXD_S32 wfTxGIdxDelta);
void rfClbrShiftBttxgain(MXD_U32 minIdx, MXD_U32 maxIdx,  MXD_S32 btTxGIdxDelta);
void castorGetWfChDcocAll(void);
int  MxdRfSwtchLnaCfg(MXD_U32 flagOnOff);
void castorGetRfRegInfo(void);
void update_for_deepChCfg_phy(unsigned int * buf, unsigned char phyRegLen);
void update_for_goodChCfg_phy(unsigned int * buf, unsigned char phyRegLen);

void troutSpiSetup(void)
{
    MXD_U32 regVal=0;
    /*   trout_reg_read(_TROUT_REG_SPI_CFG,&regVal);
    mxdDbgInfo("troutSpiSetup::PowerON, SysReg44 =  0x%x",regVal);
    trout_reg_read(_TROUT_REG_ADI_CFG,&regVal);

    trout_reg_write(_TROUT_REG_SPI_CFG, 0x40114);
    mxdDbgInfo("troutSpiSetup OK, chip id =  0x%x",regVal);
    */
    trout_reg_write(_TROUT_REG_ADI_CFG, 0x93);

    rf_reg_read(ctrl_chip_id_reg,&regVal);
    mxdDbgInfo("troutSpiSetup OK, chip id =  0x%x",regVal);
    rf_reg_write(ctrl_soft_reset_reg,0);
    rf_reg_write(ctrl_soft_reset_reg,0xffff);
}


void castorFdFsmInit(void)
{

    // auto tune      TUNING_EN
    _REGBS(rfmux_mode_cfg_ctrl0_reg, _BIT11);                //0331
    _REGBS(rf_test_ctrl1_reg, _BIT4);  //enable fd FD analog
    _REGBS(fd_tune_sel, _BIT0);         // Filter control bit

    _REGBC(ctrl_soft_reset_reg,_BIT5);  //  Reset FD
    _REGBS(ctrl_soft_reset_reg,_BIT5);

    //0x003[1]: FD tune finish sync, 1: FD cal done status (read only)
    _REG_CHK2(ctrl_tuner_status_reg,_BIT1);  //Check FD tune finish sync
}
/*
void castorReadTmpr(void)
{
    MXD_U32 regVal =0;
    int i=0;
    // enable Temperature
    _REGBS(rf_test_ctrl1_reg,_BIT5);
    _REGBSS(rf_ts_ctrl_reg,_BIT7,_BIT0,0x80);
    for (i=7;i>=0;i--)
    {
        _REGBS(rf_ts_ctrl_reg,i);
        rf_reg_read(afc_syn_status_reg,&regVal);
        rf_reg_read(afc_syn_status_reg,&regVal);
        if (0!=(afc_syn_status_reg&temp_sense1_out_bit))
        {
            _REGBC(rf_ts_ctrl_reg,i);
        }
    }
    _REGBC(rf_test_ctrl1_reg,_BIT5);
}
*/
void castorLcPllInit(void)
{
    MXD_U32 rfRegVal;
    MXD_U8 vco_dc_test = 0; // 0/1/2/3
    MXD_U8 vco_ib = 3;      // 0/1/2/3
    MXD_U8 vco_var = 3;     // 0: agnd; 1: avdd12; 2: noConn; 3: vctrl
    MXD_U8 vctrl_bias_cfg = 1; // 0:bypass; 1:2ns; 2:4ns; 3:8ns;
    MXD_U16 lpPllTune=0x80;
    int i=0;
    MXD_U16 lcpllCfgVal2=0;
    MXD_U16 regCfgInit[]=
    {
        0x062,0x0,      // sdm_setup_reg
        0x063,0x37,      // afc_setup_reg: tw_fm=3, tw_bt=1, tw_wifi=3
        0x221,0x7ff7,    // Icp for lcpll/wifi/bt/fm
        0x222,0xc230,// // bt_lomixer_cfg 0x210,0xc230 ???
        0x223,0xc230,     // wifi_lomixer_cfg 0x210 0xc230 ???
        0x224,0x0f,      // vco_cfg[5:0] for lcpll: [dctest ib var]
        0x226,0x30c,    // vco_cfg[11:0] for bt/wifi: [dctest ib var]
        0x227,0xf,       // vco_cfg[5:0] for fm: [dctest ib var]
    };

    rf_reg_write(ctrl_soft_reset_reg,0xffff); //reset all
    rf_reg_write(0x201,0xffff);    //ldo all
    //check rf id OK
    rf_reg_read(ctrl_chip_id_reg, &rfRegVal);
    mxdDbgInfo("castorLcPllInit $$$$ chipId =0x%x !\r\n",rfRegVal);

    _REGBSS(ctrl_soft_reset_reg,_BIT4,_BIT1,0x0f);  // soft reset sdm/afc
    for (i=0;i<sizeof(regCfgInit)/sizeof(MXD_U16)/2;i++)
    {
        rf_reg_write(regCfgInit[2*i],regCfgInit[2*i+1]);
    }

    //LDO ON  0331
    _REGBS(rf_ldo_ctrl_reg,_BIT11);   //0331
    rf_reg_write(rf_syn_clk_cfg_reg, 0x06);  // !!!enable !44!/48/80MHz clock [bit0=0 enablev44Mhz] bit2:1=0x03 enable

    // find tune
    vctrl_bias_cfg = 1; // 0:bypass; 1:2ns; 2:4ns; 3:8ns;
    lpPllTune=0;


    for (i=7;i>=0;i--)
    {
        lpPllTune |= (0x01<<(i));
        lcpllCfgVal2=((vctrl_bias_cfg&0x03)<<8)|lpPllTune;
        rf_reg_write(rf_syn_lcpll_cfg_reg, lcpllCfgVal2);

        //Delay 30us  !!Line-210--
        rfRegVal=0;
        rf_reg_read(afc_syn_status_reg, &rfRegVal);
        rf_reg_read(afc_syn_status_reg, &rfRegVal);

        if (0==(rfRegVal&vco_slow_lcpll_bit))
        {
            lpPllTune &=  ~(0x01<<i);
        }
    }

    //delay 30us
    rf_reg_read(afc_syn_status_reg, &rfRegVal);

    do
    {
        rf_reg_read(afc_syn_status_reg, &rfRegVal);
        rf_reg_read(afc_syn_status_reg, &rfRegVal);
        rfRegVal &=      vco_lock_lcpll_bit;
    }
    while (0);//(rfRegVal!= vco_lock_lcpll_bit );

    if ((rfRegVal!= vco_lock_lcpll_bit))
    {
        mxdDbgInfo("LCPLL UnLock!\n");
    }
    else
    {
        mxdDbgInfo("LCPLL Locked!\n");
    }
    _REGBSS(rf_syn_lcpll_cfg_reg,_BIT9,_BIT8,0x01);


    rf_reg_read(ctrl_cgf_ctrl_reg, &rfRegVal);
    rfRegVal &=  ~(gfclk40m_sel_26m_bit|gfclk44m_sel_26m_bit|gfclk48m_sel_26m_bit|gfclk80m_sel_26m_bit) ;
    //rfRegVal |=   (gfclk40m_en_bit|gfclk44m_en_bit|gfclk48m_en_bit|gfclk80m_en_bit);
    rf_reg_write(ctrl_cgf_ctrl_reg, rfRegVal);

}

//add for
void castorLcPllctrl(int on)
{
    MXD_U32 rfRegVal;
    mxdDbgInfo("castorLcPllctrl on=%d!\n",on);
    if (1==on)
    {
        _REGBS(rf_ldo_ctrl_reg,_BIT11);   //power on
        rf_reg_read(ctrl_cgf_ctrl_reg, &rfRegVal);
        rfRegVal   &=  ~(gfclk40m_sel_26m_bit|gfclk44m_sel_26m_bit|gfclk48m_sel_26m_bit|gfclk80m_sel_26m_bit) ;
        rf_reg_write(ctrl_cgf_ctrl_reg, rfRegVal);
        //rf_reg_write(0x220, 0); // zl@0613
        rf_reg_write(rf_syn_clk_cfg_reg, 0x06);  // !!!enable !44!/48/80MHz clock [bit0=0 enablev44Mhz] bit2:1=0x03 enable
    }
    else
    {
        rf_reg_read(ctrl_cgf_ctrl_reg, &rfRegVal);
        rfRegVal  |= (gfclk40m_sel_26m_bit|gfclk44m_sel_26m_bit|gfclk48m_sel_26m_bit|gfclk80m_sel_26m_bit) ;
        rf_reg_write(ctrl_cgf_ctrl_reg, rfRegVal);
        _REGBC(rf_ldo_ctrl_reg,_BIT11);   // power off
        rf_reg_write(rf_syn_clk_cfg_reg, 0x01);  // !!!enable !44!/48/80MHz clock [bit0=1 disablev44Mhz] bit2:1=0x03 enable
        //rf_reg_write(0x220, 0); // zl@0613
    }

}


void rfClbrStdEnable(int flagMode, int flagModeFreq)
{

    MXD_U16  val[]=    //cos  sin        // frxp_frxp_rcstd_ctrl_reg.bit3=1 4096.
    {
        0x3fCE,0x500,   //500kBT
        0x3e21,0xf5c,   //1M   BT
        0x4000,0x0,     //Dc  bt

        0x3fCE,0x500,   //500k WIFI
        0x3e38,0x9f9,   //1M  Wifi
        0x4000,0x0,     //Dc  wifi
    };
    if (flagMode>1)
    {
        mxdDbgInfo("rfClbrStdEnable:: error! flagMode\n");
        flagMode=0;
    }
    if (flagModeFreq>1)
    {
        mxdDbgInfo("rfClbrStdEnable:: input error  flagModeFreq!\n");
        flagModeFreq=0;
    }

    //FmRxSingleToneDetect STD_enable    defaultVal|bit4=1
    rf_reg_write(frxp_frxp_rcstd_ctrl_reg,(1<<4)|(1<<3)|(4));
    // SetSTD freq
    rf_reg_write(frxp_frxp_rcstd_cos_reg,val[flagMode*6+flagModeFreq*2]);
    rf_reg_write(frxp_frxp_rcstd_sin_reg,val[flagMode*6+flagModeFreq*2+1]);
}




void rfClbrTxToneEn_SetNco(int flagMode, int flagModeFreq)
{
    MXD_U16 val[]=    //ncoH  ncoL
    {
        0x000c,0xc000,   //500kbti
        0x0019,0x999a,   //1M  bt
        0x000c,0xc000,   //500kWifi
        0x0019,0x999a,   //1M  Wif
    };

    if (flagMode>1)
    {
        mxdDbgInfo("rfClbrTxToneEn_SetNco:: error! flagMode\r\n");
        flagMode=0;
    }
    if (flagModeFreq>1)
    {
        mxdDbgInfo("rfClbrTxToneEn_SetNco:: error!   flagModeFreq\r\n");

        flagModeFreq=0;
    }

    //txsale
    _REGBS(txp_txp_ctrl_reg,_BIT4);//enable txTone
    _REGBC(txp_txp_ctrl_reg,_BIT3);

    rf_reg_write(txp_txp_nco1_freq_h_reg,val[flagMode*4+flagModeFreq*2]); // set Nco Freq
    rf_reg_write(txp_txp_nco1_freq_l_reg,val[flagMode*4+flagModeFreq*2+1]); // 500k?
}

int  castorComInit(void)
{
    int i=0 , chipid = 0;
    int lna=0;
    for (i=0;i<sizeof(gCastorRegComInit)/sizeof(gCastorRegComInit[0])/2;i++)
    {
        rf_reg_write(gCastorRegComInit[2*i], gCastorRegComInit[2*i+1]);
    }
    //rf_reg_read(ctrl_chip_id_reg, &chipid);
    //if (chipid != _CHIP_ID_C7)
    //{
    //    rf_reg_write(0x402 , 0x0003);
    //    rf_reg_write(0x405 , 0x8003);
    //    rf_reg_write(0x408 , 0x8003);
    //}
    mxdDbgInfo("\r\r castorComInit:: regMap size = %d \r\n",i*2);
    return 0;
}


void   castorInitForLnaPa()
{
    MXD_U32 regVal=0,idLna=0,idPa=0;
    int i=0 , ChipId = 0;
    MXD_U16 NewChipForWifiRxLna[] =    //sunzl@0913  for new chip form xiaomei
    {
        0x6d8 ,0x1ba8,
        0x6d4 ,0x1ba0,
        0x6d0 ,0x1b98,
        0x6cc ,0x1b90,
        0x6c8 ,0x1b88,
        0x6c4 ,0x1b80,
        0x6c0 ,0x1b80,
    };
	MXD_U16  wfTxPaPadChipC8[]=
	{
		0X0494, 0x97ff,
		0X0495, 0xfcd8,
		0x0498, 0x97ff,
		0x0499, 0xfcd8,
		0x049C, 0x97ff,
		0x049D, 0xfcd8,
		0x04A0, 0x97ff,
		0x04A1, 0xfcd8,
		0x04A4, 0x97ff,
		0x04A5, 0xfcd8,
		0x04A8, 0x97ff,
		0x04A9, 0xfcd8,
		0x04AC, 0x97ff,
		0x04AD, 0xfcd8,
		0x04B0, 0x97ff,
		0x04B1, 0xfcd8,
		0x04B4, 0x4758,
		0x04B5, 0x5aa1,
		0x04B8, 0x4758,
		0x04B9, 0x5aa1,
		0x04BC, 0x4758,
		0x04BD, 0x5aa1,
		0x04C0, 0x4758,
		0x04C1, 0x5aa1,
		0x04C4, 0x4758,
		0x04C5, 0x5aa1,
		0x04C8, 0x4758,
		0x04C9, 0x5aa1,
		0x04CC, 0x4758,
		0x04CD, 0x5aa1,
		0x04D0, 0x4758,
		0x04D1, 0x5aa1,	
    };
    rf_reg_read(_RFREG_SW_CFG_PA_LNA,&regVal);
    idLna = regVal&0xff;
    idPa = (regVal>>8)&0xff;
    // idPa=1;//for test

    if (_LNA_ID_NONE==idLna)
    {    // idlna 0: noLNA
    }
    else if ((_LNA_ID_FM8205==idLna) || (_LNA_ID_RT6627==idLna) ||(_LNA_ID_XM2400 ==idLna))
    {  //for mobile V1.3.0   switch-pa-lna  8205
        for (i=0;i<sizeof(gCastorRegInitForWifiRxLna)/sizeof(gCastorRegInitForWifiRxLna[0])/2;i++)
        {
            rf_reg_write(gCastorRegInitForWifiRxLna[2*i], gCastorRegInitForWifiRxLna[2*i+1]);
        }
        rf_reg_read(ctrl_chip_id_reg, &ChipId);
        if (ChipId != _CHIP_ID_C7)
        {
            for (i=0;i<sizeof(NewChipForWifiRxLna)/sizeof(NewChipForWifiRxLna[0])/2;i++)
            {
                rf_reg_write(NewChipForWifiRxLna[2*i], NewChipForWifiRxLna[2*i+1]);
            }
        }
    }

    if (_PA_ID_NONE==idPa)
    { // idpa 0: noPA  gCastorRegInitNoPa
    }
    else if ((_PA_ID_FM8205==idPa) || (_PA_ID_2574==idPa))
    {
    }


        for (i=0;i<sizeof(gCastorRegInitForWifiTxGain)/sizeof(gCastorRegInitForWifiTxGain[0])/2;i++)
        {
            rf_reg_write(gCastorRegInitForWifiTxGain[2*i], gCastorRegInitForWifiTxGain[2*i+1]);
        }
#if 1
       rf_reg_read(ctrl_chip_id_reg, &ChipId);
	if (ChipId == _CHIP_ID_C8)
	{
		for (i=0;i<(sizeof(wfTxPaPadChipC8)/sizeof(wfTxPaPadChipC8[0]))/2;i++)
		{
			rf_reg_write(wfTxPaPadChipC8[2*i], wfTxPaPadChipC8[2*i+1]);
    }
	}
#endif
}



// flagDeepCh =1 : deep ch
// flagDeepCh =0 : normal
void   castorWfCfgForCh(int chNo)
{
    int i=0;
    int chipid = 0;
    MXD_U8 flagRfCfgCh =   _WF_CFG_DEEP_CH_;      //0=deep  1=good
    MXD_U8 flagPhyCfgCh = _WF_CFG_DEEP_CH_;   //0=deep  1=good	
    MXD_U16  rfCfgForChDeep[]=
    {
        // deep channel
        0x700,0x06c2,
        0x6fc,0x06c9,
        0x6f8,0x06c1,
        0x43f,0x7150,
        0x43e,0x4f4e,
        0x209,0xc0a1,
        0x484,0xf4f4,
    };

    MXD_U16  rfCfgForChGood[]=
    {
        // good channel
        0x700,    0x06c2,		 //0x1bcb,
        0x6fc,    0x06c9,		//0x1bc3,
        0x6f8,0x1bca,
        0x43f,    0x7150,			// 0x7110, //rxAgc
        0x43e,    0x4f0e,			//   0x0f0e,
        0x209,    0xc0a1,			//   0xc0c1,****
        0x484,    0xf4f4, 			//   0xa4a4,
    };
    MXD_U16  rfCfgForChipC8[]=   //0917 add xiaomei for new chip
    {
        0x700 ,0x06c2,
        0x6fc ,0x06c9,
        0x43f ,0x7150,
        0x43e ,0x4f0e,
        0x209 ,0xc0a1,
        0x484 ,0xf4f4,
    };

 MXD_U16  phyCfgCom[]=
    {
        0xff , 0x00 ,          0x00,   0x00,  0x00,
        0x22 , 0x57 ,          0x5a,   0x5a,  0x5a, 
        0x25 , 0x02 ,          0x02,   0x02,  0x02,
        0x27 , 0x0f ,          0x0f,   0x0f,  0x0f,  //0x0e,   0x0e,  0x0e, // xiao & weiKe @ 2011-11-020x0f
        0x2a , 0x0a ,          0x0a,   0x0a,  0x0a,
        0x2b , 0x80 ,          0x9a,   0x9a,  0x9a,
        0x2e , 0x09 ,          0x07,   0x07,  0x07,
        0x2f , 0x1b ,          0x1a,   0x1a,  0x1a,
        0x31 , 0x55 ,          0xa5,   0xa5,  0xa5,
        0x4d , 0x20 ,          0x14,   0x14,  0x14, 
  	 0x23 , 0x1f,           0x1d,   0x1d,  0x1c, 
        0x44 , 0x1f,           0x1d,   0x1d,  0x1c, 
        0x57 , 0x11,           0x0b,   0x0b,  0x07, 
    };
  MXD_U16  phyCfgForChGood[]=                                 
    {
         0xff, 0x00 ,           0x00,   0x00,  0x00,                                                            
      	  0x23, 0x1f,            0x1d,   0x1d,  0x1c, 
         0x44, 0x1f,            0x1d,   0x1d,  0x1c, 
         0x57, 0x11,            0x0b,   0x0b,  0x07,                                                         
     };     
     
  MXD_U16  phyCfgForChDeep[]=                                 
     {                                                  
         0xff, 0x00 ,           0x00,   0x00,  0x00,                                                            
      	  0x23, 0x1f,            0x1c,   0x1c,  0x1c, 
         0x44, 0x1f,            0x1c,   0x1c,  0x1c, 
         0x57, 0x11,            0x05,   0x05,  0x04,                                                         
     };                      
  MXD_U16  phyCfgForChipC8[]=                                 
     {                                                  
         0xff, 0x00 ,           0x00,   0x00,  0x00,                                                            
      	  0x23, 0x1f,            0x1d,   0x1d,  0x1d, 
         0x44, 0x1f,            0x1d,   0x1d,  0x1d, 
         0x57, 0x11,            0x0b,   0x0b,  0x0b,  
         0x25 , 0x02 ,          0x03,   0x03,  0x03,
    };
    MXD_U32 regVal=0;
    MXD_U32 idLna=0,idPa=0;

    rf_reg_read(ctrl_chip_id_reg,&chipid);
	if ((chNo>=5)&& (chNo<=8))
	{
		flagRfCfgCh =_WF_CFG_DEEP_CH_;
		flagPhyCfgCh =_WF_CFG_DEEP_CH_;
	}
	else
	{
		flagRfCfgCh =_WF_CFG_GOOD_CH_;
		flagPhyCfgCh =_WF_CFG_GOOD_CH_;
	}
    if (chipid != _CHIP_ID_C7)  //new chip 0913
    {
	 flagPhyCfgCh =_WF_CFG_GOOD_CH_;
    }

    rf_reg_read(_RFREG_SW_CFG_PA_LNA,&regVal);
    idLna = regVal&0xff;
    idPa = (regVal>>8)&0xff;
    mxdDbgInfo("[castorWfCfgForCh  test]-Line =%d----[lna] = %d, [pa] = %d ... !\r\n",__LINE__,idLna,idPa);
//config phy 
	mxdDbgInfo("[castorWfCfgForCh]  idLna = %d [0:none][1:fm8205][2:rt6627][3:xm2400]!!!!\n",idLna);
	if (idLna >0x03) idLna=3; //protect;
    	for (i=0;i<sizeof(phyCfgCom)/sizeof(phyCfgCom[0])/5;i++)
        {
			 trout_write_phy_reg(phyCfgCom[5*i], phyCfgCom[5*i+idLna+1]);        
        }

	 if ( _CHIP_ID_C7==chipid)  
    {
		
		if (_WF_CFG_DEEP_CH_==flagPhyCfgCh)
        { //deep chanel
			for (i=0;i<sizeof(phyCfgForChDeep)/sizeof(phyCfgForChDeep[0])/5;i++)
            {
				 trout_write_phy_reg(phyCfgForChDeep[5*i], phyCfgForChDeep[5*i+idLna+1]);        
            }
        }
        else
        { //ok channel
            //  wifi_phy_regbank1_switch();

			for (i=0;i<sizeof(phyCfgForChGood)/sizeof(phyCfgForChGood[0])/5;i++)
            {
				 trout_write_phy_reg(phyCfgForChGood[5*i], phyCfgForChGood[5*i+idLna+1]);        
            }
        }

        //trout_write_phy_reg(0x2b , 0x80);//80  trout_write_phy_reg(0x2b , 0x9a);//80
    }

        else
        { //ok channel
 		for (i=0;i<sizeof(phyCfgForChipC8)/sizeof(phyCfgForChipC8[0])/5;i++)
            {
			 trout_write_phy_reg(phyCfgForChipC8[5*i], phyCfgForChipC8[5*i+idLna+1]);        
            }
        }
	//config RF
	//0917 xiaomei modify  c8 need now ???

	
	if (chipid == _CHIP_ID_C7)
    {
		if (_WF_CFG_DEEP_CH_==flagRfCfgCh)
        { //deep chanel
		     for (i=0;i<sizeof(rfCfgForChDeep)/sizeof(rfCfgForChDeep[0])/2;i++)
            {
	                rf_reg_write(rfCfgForChDeep[2*i], rfCfgForChDeep[2*i+1]);
            }
        }
        else
        { //ok channel
		     for (i=0;i<sizeof(rfCfgForChGood)/sizeof(rfCfgForChGood[0])/2;i++)
            {
	                rf_reg_write(rfCfgForChGood[2*i], rfCfgForChGood[2*i+1]);
            }
        }

    }
	
    //0917 xiaomei modify
    if (chipid != _CHIP_ID_C7)
    {
		for (i=0;i<sizeof(rfCfgForChipC8)/sizeof(rfCfgForChipC8[0])/2;i++)
        {
			rf_reg_write(rfCfgForChipC8[2*i], rfCfgForChipC8[2*i+1]);
        }
    }
    if (idPa==_PA_ID_FM8205)
    {
    }

#ifdef _MXD_WF_DCOC_INSERT_
    rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+28* 4+2,gWfChDcoc[(chNo-1)*5+1]);
    rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+29* 4+2,gWfChDcoc[(chNo-1)*5+2]);
    rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+30* 4+2,gWfChDcoc[(chNo-1)*5+3]);
    rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+31* 4+2,gWfChDcoc[(chNo-1)*5+4]);
    mxdDbgInfo("[mxd]dcocidx28:: ch16_2380=0x%x  ch17_2508=0x%x:  nowCh%d_%d=0x%x \r\n",gWfChDcoc[(16-1)*5+1],gWfChDcoc[(17-1)*5+1],chNo,gWfChDcoc[(chNo-1)*5],gWfChDcoc[(chNo-1)*5+1]);
    mxdDbgInfo("[mxd]dcocidx29:: ch16_2380=0x%x  ch17_2508=0x%x:  nowCh%d_%d=0x%x \r\n",gWfChDcoc[(16-1)*5+2],gWfChDcoc[(17-1)*5+2],chNo,gWfChDcoc[(chNo-1)*5],gWfChDcoc[(chNo-1)*5+2]);
    mxdDbgInfo("[mxd]dcocidx30:: ch16_2380=0x%x  ch17_2508=0x%x:  nowCh%d_%d=0x%x \r\n",gWfChDcoc[(16-1)*5+3],gWfChDcoc[(17-1)*5+3],chNo,gWfChDcoc[(chNo-1)*5],gWfChDcoc[(chNo-1)*5+3]);
    mxdDbgInfo("[mxd]dcocidx31:: ch16_2380=0x%x  ch17_2508=0x%x:  nowCh%d_%d=0x%x \r\n",gWfChDcoc[(16-1)*5+4],gWfChDcoc[(17-1)*5+4],chNo,gWfChDcoc[(chNo-1)*5],gWfChDcoc[(chNo-1)*5+4]);
#endif

#ifdef   _MXD_NV_SUPPORT_
    if ((chNo>=5)&& (chNo<=8))
    { //deep chanel
        trout_write_phy_reg(0xff, 0);
        if(0!= gPhy23Cfg[0])  trout_write_phy_reg(0x23 , gPhy23Cfg[0]);
        if(0!= gPhy44Cfg[0])  trout_write_phy_reg(0x44 , gPhy44Cfg[0]);
    }
    else

    {
//ok channel
        trout_write_phy_reg(0xff, 0);
        if(0!= gPhy23Cfg[1]) trout_write_phy_reg(0x23 ,  gPhy23Cfg[1]);
        if(0!= gPhy44Cfg[1]) trout_write_phy_reg(0x44 ,  gPhy44Cfg[1]);
    }
#endif
//by lihua for tempr compensation
    if (g_tempr_en == 1)
    {
	if (gWfChFromAp == 0)
		update_for_deepChCfg_phy(pNvBuf, 3);
	else if (gWfChFromAp == 1)
		update_for_goodChCfg_phy(pNvBuf,3);
    }

}


void  castorWorkInit(void)
{
    int i=0;
    MXD_U32 regVal=0,idLna=0,idPa=0,chipId=0;
    mxdDbgInfo("[mxd test]castorWorkInit... !\r\n");
    rf_reg_write(rfmux_mode_sel_reg, 0x0008);
    rf_reg_write(rfmux_sprd_mode_sel_reg, 0x0000);
    rf_reg_write(rf_lobuffer_cfg_reg, 0x0101);
    rf_reg_write(rfmux_wb_tx_gain_reg, 0x0101);

    //    mxdDbgInfo("[mxd test]castorWorkInit... 2013-04-10 !\r\n");
    rf_reg_write(rfmux_wb_tx_gain_reg, 0x7bd);//0410

    // config 0xc8 in wifiPout zl@0903
    // rf_reg_write(rfmux_pout_tx_scale_reg , 0x200);//gdy@0801: 0x190->0x200   gdy@0524 0x200->0x190
    // rf_reg_write(rfmux_pout_tx_scale_reg , 0x280);//0410
    //rf_reg_write(rfmux_pout_tx_scale_reg ,   gDacScaleCh[15]);  //zl@0911 pout clbr
    rf_reg_write(rfmux_pout_tx_scale_reg ,   gWfTxScaleCfg);

    _REGBSS(txp_txp_ctrl_reg,_BIT12,_BIT11,0x03);   //Txe_Sel: 00:bt   01:wifi 1* SPRD
    _REGBSS(txp_txp_ctrl_reg,_BIT9,_BIT7,0x03);     //DAC 12:3
    _REGBSS(txp_txp_ctrl_reg,_BIT4,_BIT3,0x0);      //disable txtone        bit4: en/dis txtone    bit3: en/dis dualTone
    _REGBS(txp_txp_ctrl_reg,_BIT1);                 //tx 2x2 Enable
    _REGBS(txp_txp_ctrl_reg,_BIT0); //0410 enable   //   _REGBC(txp_txp_ctrl_reg,_BIT0);                 //disable tx iq scale   sunzl

    _REGBSS(rf_ts_ctrl_reg,_BIT15,_BIT14,0x2); //bit15: ANT_SEL_1_EN bit14:testPinEn
    _REGBS(rfmux_gain_table_sel_reg,_BIT7); //LnaGainSel:   0:Host  1:tbl
    _REGBS(rfmux_gain_table_sel_reg,_BIT6); //PA_sel    0:host 1:mxd
    _REGBS(rfmux_gain_table_sel_reg,_BIT5); //btRxGain      0:Host  1:tbl
    _REGBS(rfmux_gain_table_sel_reg,_BIT4);   //btTxGainSel:  0:Host  1:tbl
    _REGBSS(rfmux_gain_table_sel_reg,_BIT3,_BIT2,0x03);   //wifiRxGain Sel:       00:Host  01:mxd 10:reserved 11:sprd
    // _REGBSS(rfmux_gain_table_sel_reg,_BIT1,_BIT0,01);        //wifitxGain Sel:       00:Host  01:mxd 10:reserved 11:sprd
    _REGBSS(rfmux_gain_table_sel_reg,_BIT1,_BIT0,03);        //wifitxGain Sel:       00:Host  01:mxd 10:reserved 11:sprd
    _REGBS(rfmux_gain_table_sel_reg,_BIT8); //colisten priotity:  0:btRxLnaLbl  1:wifiRxLnaTbl
    _REGBSS(rfmux_loft_pout_cfg_reg,_BIT7,_BIT6,0x3);        //bit7: btTxLoftSel 0 host 1.tbl   // bit6 wifiTxLoftSel
    castorSetRfEntry(0);
    rf_reg_write(rf_ldo_ctrl_reg,0x2801); // LDO: DAB_En PA_Dis PLL_EN CPwf/bt/fm_Dis Lowf/bt/fm_Dis fm_dis rx/Tx dis Adc_Dis bg_ana_En
    rf_reg_write(_RFREG_SW_CFG_LDO,0x2801); // LDO: DAB_En PA_Dis PLL_EN CPwf/bt/fm_Dis Lowf/bt/fm_Dis fm_dis rx/Tx dis Adc_Dis bg_ana_En

    rf_reg_write(_RFREG_SW_RF_STATUS_,0x0);// clear all module flag_workStatus

    rf_reg_write(rfmux_wftx_gidx_reg,0x30);   //wifi tx init
    rf_reg_write(rfmux_bttx_gidx_reg,0x09); //Bt tx init@0504modify  9:[-2.8db,16.6mA]   8[2.7,29.1ma] 1/2:[-1db, 17.5mA]
    _REGBS(rfmux_wfrx_gidx_reg,_BIT7);  //enable map

    {
        rf_reg_read(_RFREG_SW_CFG_PA_LNA,&regVal);
        idLna = regVal&0xff;
        idPa = (regVal>>8)&0xff;
	 rf_reg_read(ctrl_chip_id_reg, &chipId);
	 if (_CHIP_ID_C8 == chipId)
	 {
		//rf_reg_write(0x24 ,0x6);   //SPRD! DCR Disable
             rf_reg_write(wrxp_wrxp_dcr_ctrl_reg ,0x11);   //zl@0926 !!!wifi dcr enable, dcr para =6.   ,11b_1M 11g_6M
	 }
	 if (_CHIP_ID_C7 == chipId)	
	 {
	 	rf_reg_write(wrxp_wrxp_dcr_ctrl_reg ,0x6);   //SPRD! DCR Disable
	 }
	 rf_reg_write(wrxp_wrxp_dcr_ctrl_reg ,0x12);   //from wk&meirui @2011-11-2
        //wrxp_wrxp_dcr_ctrl_reg
        if (0==idPa)
        { // idpa 0: noPA
        }
        else if (1==idPa)
        { //fro mobile V1.3.0   switch-pa-lna  8205
        }
    }

    //
    rf_reg_write(fd_bttx_lock_offset_reg,0x040);             //improve  Bt Tx  evm 0815

    //improve packet miss
    rf_reg_write(fd_wifitx_lock_offset_reg ,0x78);   //  rf_reg_write(0x57 ,0x78); 
    rf_reg_write(rfmux_mode_cfg_ctrl2_reg, 0xe119); //!bit15:13-> 111b           (0x207, 0x119); 
    // _REGBC(rfmux_fm_cfg_reg,_BIT4);   //disable fm adc
    rf_reg_write(rfmux_fm_cfg_reg, 0x00); //disable fm adc

    _REGBS(0x200,_BIT1);  //zl@0613 disable SIDO
    _REGBS(0x21e,_BIT4);  //zl@0613 LPF tuning disable enable map
    //rf_reg_write(0x209, 0xa1);   //c0a1 update in [castorComInit] ??????   MR@0607 modify for packet error 54M/48M

    _rfFmAdcOutCmpl;
    _SET_TROUT2_ANT_SEL_NORMAL;
    _wfTxPoutMxdDbgInfo("[castorGetRfRegInfo]:: Step4   workModeInit !=========>>>>>>>>>>\r\n");
    castorGetRfRegInfo();
}

void  castorCfgLdo()
{
    MXD_U32 cfgNow=0;
    MXD_U32 flagStatusFmWifiBt;
    rf_reg_read(_RFREG_SW_RF_STATUS_,&flagStatusFmWifiBt);
    flagStatusFmWifiBt &= ((1<<_FLAG_B_WIFI)|(1<<_FLAG_B_BT)|(1<<_FLAG_B_FM));
    if (0==flagStatusFmWifiBt)
    {
        //  castorLcPllctrl(0); //force 26Mhz
        cfgNow=0x2801;  //idle
    }
    /*
        else if ((1<<_FLAG_B_FM)==flagStatusFmWifiBt)
        {
         //  castorLcPllctrl(0); //force 26Mhz
       cfgNow |= 0x2073;     //->2073 // fm only work  zl@0923 for fm spur
         }
    */
    else
    {
        // castorLcPllctrl(1); // !!!enable !44!/48/80MHz
        if (0!= (flagStatusFmWifiBt&(1<<_FLAG_B_WIFI)))
        {
            cfgNow |= 0x3f0f; // wifi work
        }
        if (0!=(flagStatusFmWifiBt&(1<<_FLAG_B_BT)))
        {
            cfgNow |= 0x3d8f; // bt work
        }
        if (0 !=(flagStatusFmWifiBt&(1<<_FLAG_B_FM)))
        {
            cfgNow |= 0x2873; //// fm work  zl@0923 for fm spur
        }
    }
    rf_reg_write(rf_ldo_ctrl_reg,cfgNow);
    rf_reg_write(_RFREG_SW_CFG_LDO,cfgNow);
}

void  castorChkLdo()
{
    MXD_U32 cfgNow=0,valRegLdoCtrl=0;
    //MXD_U32 flagStatusFmWifiBt;
    rf_reg_read(rf_ldo_ctrl_reg,&valRegLdoCtrl);
    rf_reg_read(_RFREG_SW_CFG_LDO,&cfgNow);
    if (valRegLdoCtrl!=cfgNow)
    {
        castorCfgLdo()  ;
    }
}


MXD_API_C int  castorClbrLoftWifi(void)
{

    MXD_U16 regVal=0;
    MXD_U32 chipid = 0;
    MXD_U16 minMag=0xffff;
    int flagMode=0;
    int n;
    MXD_U16 iLoftDac=0;
    MXD_U16 qLoftDac=0;
    MXD_U32 gainTx;
    MXD_U16 bestTxLoftCtrl=0,bestTxLoftCtrlQ=0,bestTxLoftCtrlI=0;
    MXD_U16 regAddrTxGidx[2]={0xc1,0xc3};    //regAddr btTxIndex   wifiTxIndex
    MXD_U16 txLoftDacVal[2]= {0x21a,0x21b};   //BtTxLoftDacCtrl     wifiTxLoftDacCtrl
    MXD_U16 txLoftTblAddr[2]={0x4d4,0x4d8};   //bt/wifi entry0  txLoftDacCtrlTbl
    MXD_U16 valCtrlLoftEn[2]={0x0032,0x0031};
    MXD_U32 mag=0;
    MXD_U16 tblClbrTxIdx [2][4]={        {0x1,0x10,0x20,0x2e},  {0x1,0x10,0x20,0x2e},  };
    int flagModePllLdoEn[]={  0,1,0, //fmPllDis btPllEn  wifiPllDis
                              0,0,1  //fmPllDis btPllDis wifiPllEn
                           };
    MXD_U32     wifiTxPga4bit, txGm3bit ;
    MXD_U16 wifiTxGainIdx=0;
    //  MXD_U32 mxd_wf_dcoc_ch=mxd_set_wf_loft_ch();
    flagMode=1;


    _SET_TROUT2_ANT_SEL_LOFT_WIFI;
    //_REGBSS(0x04,7,4,0x0f);//26Mhz force.
    _REGBSS(ctrl_txp_clk_force_ctrl_reg,_BIT1,_BIT0,0x02);//txe 26Mhz force disable.

    if (flagMode==1)
    {
        //initClbrLoft();
        castorHostWifiTxInit();

        _REGBSS(rf_ldo_ctrl_reg,_BIT10,_BIT8,0x7);
        _REGBS(rf_lobuffer_cfg_reg,_BIT2);

        _REGBC(rf_ldo_ctrl_reg,_BIT7);// disable bt lo
        _REGBC(rf_lobuffer_cfg_reg,_BIT1); //disable bt Lo mixer
        castorSetFreqWifiCh(16);  //  castorSetFreqWifiCh(9);
    }    
    //set rfcali mode  // bit[6:5]:00b reserved// bit[4:3]  10b rfcali mux-> enable dcoc// bit[2:0]
    // 01 10  010          [LoftCl] [rfclaiMode] [btTxmdoe]
    rf_reg_write(rfmux_mode_sel_reg, 0x0031); //loftCl_WifiTx

    //regbf       bit4=1: btTxGainSel tbl   bit[1:0]=1 wifi bt indxTxGainTbl from reg.
    //rf_reg_write(rfmux_gain_table_sel_reg, (1<<6)|(1<<4)|1); //
    _REGBSS(rfmux_gain_table_sel_reg,_BIT1,_BIT0, 0x0);//wifi tx gain sel: 00 host 01mxd 10:reserved 11:sprd tbl
    _REGBC(rfmux_gain_table_sel_reg,_BIT4);     // bt txgain sel: 0: host 1: table
    _REGBC(rfmux_gain_table_sel_reg,_BIT6);     // wf/bt pa sel : 0: host 1: table
    //  rf_reg_write(rfmux_gain_table_sel_reg,0);

    rf_reg_write(rfmux_mode_sel_reg, valCtrlLoftEn[flagMode]); //LoftCl_BtTx
    _rfFmAdcOutCmpl;
    //TxloftSel bit7=0:BTfrom Reg21a,bit6=0:WifiFrom Reg21b
    _REGBSS(rfmux_loft_pout_cfg_reg,_BIT7,_BIT6,0);    //TxloftSel bit7=0:BTfromReg21a,bit6=0:WifiFromReg21b
    minMag=0xffff;

    /*   int flagModePllLdoEn={  0,1,0 ;0,0,1 };     */
    for (;flagMode<2;flagMode++)  //flag = 0: bt   1: wifi
    {
        {
            //  rfClbrSetLdoPll(0,0,1);// LdoPll: fmDis btDis wifiEn
            _REGBS(rf_ldo_ctrl_reg,_BIT1);
            _REGBS(rf_ldo_ctrl_reg,_BIT4);
            _REGBS(rfmux_fm_cfg_reg,_BIT1);
            _REGBS(rfmux_fm_cfg_reg,_BIT4);
            _REGBS(rfmux_cal_cfg_reg,_BIT0);
            _REGBS(rfmux_cal_cfg_reg,_BIT1);
            _REGBS(rfmux_mode_cfg_ctrl0_reg,_BIT11);
            _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT8);
        }
        //rf_reg_read(ctrl_chip_id_reg, &chipid);
        //if (chipid != _CHIP_ID_C7)
        {
            _REGBSS(rfmux_mode_cfg_ctrl2_reg, _BIT15, _BIT13, 7);
        }
        rf_reg_write(rfmux_mode_sel_reg, valCtrlLoftEn[flagMode]); //[ClbrMode.Loft,btTx/wfTx]
        rfClbrTxToneEn_SetNco(flagMode,_FLAG_FREQ_500K);
        rfClbrStdEnable(flagMode,_FLAG_FREQ_500K);

        /*       MXD_U16 idxModeTxGidx [2][4]={ {0,5,6,31},{0,5,6,31},};*/
        for (n=0;n<4;n++)   //txGain 0:3
        {
            //  regAddrTxGidx[2]={rfmux_bttx_gidx_reg, rfmux_wftx_gidx_reg};
            wifiTxGainIdx= tblClbrTxIdx[flagMode][n];
            rf_reg_write(regAddrTxGidx[flagMode],wifiTxGainIdx);
            rf_mem_read(WIFI_TX_GAIN_TBL_START_ADDR+wifiTxGainIdx*2,&gainTx);

            //   mxdDbgInfo("castorClbrLoftWifi:: getTxGainTbl  memaddr= 0x%x, val=0x%x\r\n ",WIFI_TX_GAIN_TBL_START_ADDR+(gindx*16*2), gainTx);
            wifiTxPga4bit = _BSG(gainTx,_BIT6,_BIT3);
            txGm3bit = _BSG(gainTx,_BIT2,_BIT0);
            _REGBSS(rfmux_wb_tx_gain_reg, _BIT10,_BIT7, wifiTxPga4bit);
            _REGBSS(rfmux_wb_tx_gain_reg, _BIT2,_BIT0, txGm3bit);

            minMag = 0xffff;
            bestTxLoftCtrl = 0x8080;
            for (iLoftDac = (0x80-LOFT_DAC_I_RNG); iLoftDac<(0x80+LOFT_DAC_I_RNG); iLoftDac+=2)
            {
                //  txLoftDacVal[2]= {0x21a,0x21b};
                rf_reg_write(txLoftDacVal[flagMode], (iLoftDac<<8)|0x80);  //21a:bt  21b:wifi
                _rfStdStart;
                _REG_CHK(ctrl_tuner_status_reg,_BIT5);

                rf_reg_read(frxp_frxp_rcstd_mag_status_reg, &mag);
                MxdSleep(MXD_SLEEP_MS);

                _wfLoftMxdDbgInfo("castorClbrLoft:: SearchIIII iLoftDac= 0x%x, mag=0x%x\r\n ",iLoftDac, mag);

                if (mag<minMag)
                {
                    minMag=mag;
                    bestTxLoftCtrl= (iLoftDac<<8)|0x80;

                    _wfLoftMxdDbgInfo("castorClbrLoft:: SearchIIII minMag= 0x%x, bestTxLoftCtrl=0x%x\r\n ",minMag, bestTxLoftCtrl);

#ifdef _MXD_CLBR_OPTIMIZE
                    if (minMag<=2)
                    {
                        _wfLoftMxdDbgInfo("castorClbrLoft[IIII]:: BREAKNOW!  \r\n");
                        break;
                    }
#endif
                }

            }// end for idac

            rf_reg_write(txLoftDacVal[flagMode], bestTxLoftCtrl);  //21a:bt  21b:wifi
            //bestTxLoftCtrlI= (bestTxLoftCtrl&0xff00)>>8;
            //   minMag=0xffff;

            for (qLoftDac = 0x80-LOFT_DAC_Q_RNG; qLoftDac<0x80+LOFT_DAC_Q_RNG; qLoftDac+=2)
            {
                rf_reg_write(txLoftDacVal[flagMode], (bestTxLoftCtrl&0xff00)|qLoftDac);  //21a:bt  21b:wifi
                _rfStdStart;
                _REG_CHK(ctrl_tuner_status_reg,_BIT5);
                MxdSleep(MXD_SLEEP_MS);
                _wfLoftMxdDbgInfo("castorClbrLoft::  QLoftDac= 0x%x, mag=0x%x\r\n ",qLoftDac, mag);

                rf_reg_read(frxp_frxp_rcstd_mag_status_reg, &mag);
                if (mag<minMag)
                {
                    minMag=mag;
                    bestTxLoftCtrl= (bestTxLoftCtrl&0xff00)|qLoftDac;
                    _wfLoftMxdDbgInfo("castorClbrLoft:: SearchQQQQ minMag= %x, bestTxLoftCtrl=%x\r\n ",minMag, bestTxLoftCtrl);
#ifdef _MXD_CLBR_OPTIMIZE
                    if (minMag<=2)
                    {
                        mxdDbgInfo("castorClbrLoft[QQQQ]:: BREAK NOW!  \r\n");
                        break;
                    }
#endif
                }
            }//end for qdac
            // now get the best LoftDac for this gain
            rf_reg_write(txLoftDacVal[flagMode], bestTxLoftCtrl);

            //zl@0523 00 10 20 30-> loft id 11b 10b 01b,00b
            //rf_reg_write(txLoftTblAddr[flagMode]+gindx, bestTxLoftCtrl);
            rf_reg_write(txLoftTblAddr[flagMode]+3-n, bestTxLoftCtrl);
            mxdDbgInfo("[mxd  wifi] [mode: %d    0:bt 1:wifi]wifiTxGainIdx=[0x%x]castorClbrLoft:: SearchIQ minMag=%x, bestTxLoftCtrl[%x]=%x  to RfReg[0x%x]  gainTx=%x loftId=0x%x \r\n ",flagMode,wifiTxGainIdx,minMag, bestTxLoftCtrl,txLoftDacVal[flagMode], txLoftTblAddr[flagMode]+n,gainTx,(3-n));
        }//end for txGidx
  } // end for mode
   
    castorClbrTximbWifi();
    castorClbrPoutWifi();
    _REGBSS(ctrl_txp_clk_force_ctrl_reg,_BIT1,_BIT0,0x0);//txe 26Mhz force disable.
    _SET_TROUT2_ANT_SEL_NORMAL;
    return 0;
}

// val tmpr
// ...189 = -6, 190 = 0, 191 = 6
// tmpr = (val-190)*6
MXD_U32 castorReadTmpr(void)
{
	MXD_U32 regVal =0;
	int i=0;
	MXD_U32 tmprCur =0,tmprVal =0;
	
	//Enable temperature
	_REGBS(rf_test_ctrl1_reg, _BIT5);
	_REGBSS(rf_ts_ctrl_reg, _BIT7,_BIT0,0x80);
	
	for(i=7;i>=0;i--)
	{
		_REGBS(rf_ts_ctrl_reg, i);
		rf_reg_read(afc_syn_status_reg,&regVal);
		if(0 != (regVal&temp_sense1_out_bit))
		{
			_REGBC(rf_ts_ctrl_reg, i);		
		}	
	}	
	_REGBC(rf_test_ctrl1_reg, _BIT5);
	tmprVal = regBitsGet(rf_ts_ctrl_reg, _BIT7, _BIT0);
	//tmprCur = (0x42 - tmprVal)*6;
	
	tmprVal = (0x100-tmprVal);
	tmprCur = (tmprVal-190)*6;
	mxdDbgInfo("castorReadTmpr tmprCur =%d tmprVal =%d[0x%x] regVal = 0x%x!\r\n", tmprCur,tmprVal,tmprVal,(0x100-tmprVal));
	return tmprVal;
	
	
}
MXD_U32 MxdRfReadTempreture(void)
{
    // tmpr = (val-190)*6
    return castorReadTmpr();
}


#if 0    //add by lihua
#define myptf mxdDbgInfo
#else
#define myptf
#endif
void update_for_GoodDeepChCfg_rf(unsigned int *buf, unsigned char len)
{
		int i;
		unsigned short rf_reg,rf_val,temp;
		
		for (i=0;i<len;i++)
		{
			rf_reg = (unsigned short)((buf[i]>>16)&0xffff);
			rf_val = (unsigned short)((buf[i])&0xffff);
			myptf("lihua:: rf_reg=0x%x\trf_val=0x%x\r\n", rf_reg, rf_val);
			rf_reg_read(0xff, &temp);
			rf_reg_write(rf_reg, rf_val);
			myptf("lihua::reg(0xff) = %d\r\n", temp);
			rf_reg_read(rf_reg, &temp);
			myptf("lihua:: readback:rf_reg=0x%x\trf_val=0x%x\r\n", rf_reg, temp);
		}
}

void update_for_deepChCfg_phy(unsigned int *buf, unsigned char phyRegLen)
{
		int i;
		unsigned char phy_reg,phy_val,temp;
		
		for (i=0;i<phyRegLen;i++)
		{
			phy_reg = (unsigned char)((buf[i]>>8)&0xff);
			phy_val = (unsigned char)((buf[i])&0xff);
			myptf("lihua:: phy_reg=0x%x\tphy_val=0x%x\r\n", phy_reg, phy_val);
			read_dot11_phy_reg(0xff, &temp);
			myptf("lihua::reg(0xff) = %d\r\n", temp);
			write_dot11_phy_reg(phy_reg, phy_val);
			myptf("lihua::reg(0xff) = %d\r\n", temp);
			read_dot11_phy_reg(phy_reg, &temp);
			myptf("lihua:: readback:phy_reg=0x%x\tphy_val=0x%x\r\n", phy_reg, temp);
		}
		
}

void update_for_goodChCfg_phy(unsigned int *buf, unsigned char phyRegLen)
{
		int i;
		unsigned char phy_reg,phy_val,temp;
		
		for (i=0;i<phyRegLen;i++)
		{
			phy_reg = (unsigned char)((buf[i]>>24)&0xff);
			phy_val = (unsigned char)((buf[i]>>16)&0xff);
			myptf("lihua:: phy_reg=0x%x\tphy_val=0x%x\r\n", phy_reg, phy_val);
			read_dot11_phy_reg(0xff, &temp);
			myptf("lihua::reg(0xff) = %d\r\n", temp);
			write_dot11_phy_reg(phy_reg, phy_val);
			myptf("lihua::reg(0xff) = %d\r\n", temp);
			read_dot11_phy_reg(phy_reg, &temp);
			myptf("lihua:: readback:phy_reg=0x%x\tphy_val=0x%x\r\n", phy_reg, temp);
		}
		
}

int update_for_goodChCfg(P_CLBR_PARA_S pParaIn)
{
		unsigned int reg32_val=0;
		unsigned int *buf;
		unsigned char phy_reg,phy_val,phy_0x57_val;
		unsigned short rf_reg,rf_val; 
		unsigned char phyRegLen=0;
		unsigned char rfRegLen_Deep=0;
		unsigned char rfRegLen_Good=0;
		
		reg32_val = pParaIn->reserved2[16];
		phyRegLen = (unsigned char)(reg32_val & 0xff);
		rfRegLen_Good = (unsigned char)((reg32_val>>24)&0xff);
		rfRegLen_Deep = (unsigned char)((reg32_val>>16)&0xff);
		myptf("lihua:: phyRegLen=0x%x\rfRegLen_Good=0x%x\trfRegLen_Deep=0x%x\r\n", phyRegLen, rfRegLen_Good,rfRegLen_Deep);
		
		buf = pParaIn->reserved2 +17;
		update_for_goodChCfg_phy(buf, phyRegLen);
		
		buf = pParaIn->reserved2 + 17 + phyRegLen + rfRegLen_Deep;
		update_for_GoodDeepChCfg_rf(buf, rfRegLen_Good);

		//buf = pParaIn->reserved2 + 15; //for reg 0x57
		//g_ori_0x57 = (buf[0]>>16)&0xff;
		phy_0x57_val = (unsigned char)((pParaIn->reserved2[15]>>8)&0xff);
		if (phy_0x57_val != 0)
		{
			write_dot11_phy_reg(0x57, phy_0x57_val);		
		}
		
		return 0;	
}

int update_for_deepChCfg(P_CLBR_PARA_S pParaIn)
{
		unsigned int reg32_val=0;
		unsigned int *buf;
		unsigned char phy_reg,phy_val,phy_0x57_val;
		unsigned short rf_reg,rf_val; 
		unsigned char phyRegLen=0;
		unsigned char rfRegLen_Deep=0;
		
		reg32_val = pParaIn->reserved2[16];
		phyRegLen = (unsigned char)(reg32_val & 0xff);
		rfRegLen_Deep = (unsigned char)((reg32_val>>16)&0xff);
		
		buf = pParaIn->reserved2 +17;
		update_for_deepChCfg_phy(buf, phyRegLen);
		
		buf = pParaIn->reserved2 + 17 + phyRegLen;
		update_for_GoodDeepChCfg_rf(buf, rfRegLen_Deep);

		//buf = pParaIn->reserved2 + 15; //for reg 0x57
		//g_ori_0x57 = buf[0]&0xff;
		phy_0x57_val = (unsigned char)((pParaIn->reserved2[15])&0xff);
		if (phy_0x57_val != 0)
		{
			write_dot11_phy_reg(0x57, phy_0x57_val);		
		}
		
		return 0;	
}

unsigned int check_tpc_switch_from_nv(void)
{
	unsigned int reg32_val;
	P_CLBR_PARA_S pUpdatePara = &cal_regs;

	reg32_val = pUpdatePara->reserved2[49];
	if (reg32_val & 0x01)
		return 1;
	else 
		return 0;
}

int get_cur_tmpr_ref_num(int cur_tempr)
{
	int tempr_ref0,tempr_ref1,tempr_ref2,tempr_ref3;
	P_CLBR_PARA_S pUpdatePara = &cal_regs;

	tempr_ref0 = (pUpdatePara->reserved2[30])&0xff;		
	tempr_ref1 = (pUpdatePara->reserved2[30]>>8)&0xff;
	tempr_ref2 = (pUpdatePara->reserved2[30]>>16)&0xff;
	tempr_ref3 = (pUpdatePara->reserved2[30]>>24)&0xff;
	myptf("lihua:: tempr_ref0=%d\ttempr_ref1=%d\ttempr_ref2=%d\ttempr_ref3=%d\r\n", tempr_ref0,tempr_ref1,tempr_ref2,tempr_ref3);

	if (cur_tempr <= tempr_ref0)
	{
		return 0;			
	}
	else if (cur_tempr <= tempr_ref1)
	{
		return 1;	
	}
	else if (cur_tempr <= tempr_ref2)
	{
		return 2;
	}
	else if (cur_tempr <= tempr_ref3)
	{
		return 3;
	}
	else
	{
		return 4;
	}
}

void tempr_compensated(void )
{
	int cur_tempr,cur_tmpr_ref_num=0;
	unsigned int reg32_val;
	unsigned int *buf = NULL;
	P_CLBR_PARA_S pUpdatePara = &cal_regs;
	unsigned int flagDeepGood=0;
	
	myptf("lihua:: tempr_compensated!\r\n");
	reg32_val = pUpdatePara->reserved2[49];
	myptf("Reserved[49]=%d\r\n", reg32_val);
	if ((reg32_val&0xc) == 0) //if disable
	{
		myptf("tempr compensated Disable!\r\n");
		g_tempr_en = 0;
		return;
	}
	g_tempr_en = 1;	
	flagDeepGood = gWfChFromAp; 	
	cur_tempr = MxdRfReadTempreture();	// 190:0   191:6   189:-6
	cur_tmpr_ref_num =get_cur_tmpr_ref_num(cur_tempr);
	myptf("lihua::cur_tmpr_ref_num=%d\r\n",cur_tmpr_ref_num);
	if (cur_tmpr_ref_num == last_tempr_ref_num)
	{
		myptf("lihua::the same tempr!\r\n");
		return;
	}

	last_tempr_ref_num = cur_tmpr_ref_num;
	
	buf = pUpdatePara->reserved2 + 31+cur_tmpr_ref_num*3;
	pNvBuf = buf;
	if (flagDeepGood == 0)
		update_for_deepChCfg_phy(buf, 3);
	else if (flagDeepGood == 1)
		update_for_goodChCfg_phy(buf,3);
	else
		myptf("lihua::flagDeepGood error!\r\n");
	return;
	
}

//EXPORT_SYMBOL(tempr_compensated);

int update_cfg_front_nv(void)
{
	unsigned int reg32_val=0;
	unsigned int flagDeepGood=0;
	P_CLBR_PARA_S pUpdatePara = &cal_regs;
	
	myptf("lihua::update_cfg_front_nv!\r\n");
	/*reg32_val = pUpdatePara->reserved2[49];
	if (reg32_val & 0x01)							//TBD
		return 0;*/
	flagDeepGood = gWfChFromAp;
	
	if (flagDeepGood == 0)//for deepChCfg
	{
		update_for_deepChCfg(pUpdatePara);
	}
	else if (flagDeepGood == 1) //for goodChCfg
	{
		update_for_goodChCfg(pUpdatePara);
	}	
	else 
	{
		;
	}	
	return 1;
}

void  MxdRfSetClkReqLow(void)
{
  // base on liYuan[SPRD email 2013-09-29 ]
    MXD_U32 regVal =0;
    mxdDbgInfo("MxdRfSetClkReqLow:: ....\r\n");
    trout_reg_read(_TROUT_M_CLK_CTRL1,&regVal);
    mxdDbgInfo("MxdRfSetClkReqLow::  _TROUT_M_CLK_CTRL1= 0x%04x    \r\n", regVal);	

    regVal |= (0x0b<<_BIT10);
    regVal &=~(0x01<<_BIT12);
    regVal |= (0x01<<_BIT16);	  
    trout_reg_write(_TROUT_M_CLK_CTRL1,regVal);
    mxdDbgInfo("MxdRfSetClkReqLow:: ...   _TROUT_M_CLK_CTRL1= 0x%04x\r\n",regVal);

    trout_reg_read(_TROUT_M_CLK_CTRL1,&regVal);
    mxdDbgInfo("MxdRfSetClkReqLow:: done!   _TROUT_M_CLK_CTRL1= 0x%04x\r\n",regVal);
    return;
}
castorGetParaFromNv(void *  pParaIn)
{
    //**********************PoutDebug****************
    // btPadTbl[0]  pout dectMode  0: disable     1:Pin        2: PaD Detct
    // btPadTbl[1]  shift posi for : : pout ref2Val
    // btPadTbl[2]  pout ref1Val   // clbr gIdxWfTxPwrShift
    // btPadTbl[3]  pout ref2Val   // clbr dac scale 
    
	
    P_CLBR_PARA_S pClbrPara = (P_CLBR_PARA_S)pParaIn;
    int i=0,val=0;
	MXD_S16 tmpS16=0;
    if (NULL==pClbrPara)
    {
            mxdDbgInfo("castorGetParaFromNv:: break???? NV ERROR  pClbrPara==NULL!! \r\n");
            return;
    }   
    if (0==pClbrPara->len)
    {
        mxdDbgInfo("castorGetParaFromNv:: break???? NV ERROR  pClbrPara->len==%d!! \r\n",pClbrPara->len);
    }
    for (i = 0; i < 14; i ++)
    {
        if (0==pClbrPara->dacScaleCh[i])
        {
            mxdDbgInfo("castorGetParaFromNv:: break! NV ERROR! Trout not ready!!!  dacScaleCh[%d]=%d!! \r\n",i,pClbrPara->dacScaleCh[i]);
            return;
        }
    }
    gPoutMode =(MXD_U32) pClbrPara->btPadTbl[0];
	if (pClbrPara->reserved2[49]& (1<<_BIT5))
	{
		_BS(gPoutMode,_BIT7);
	}
    else
	{
		_BC(gPoutMode,_BIT7);		
	}
    pClbrPara->btPadTbl[0]=gPoutMode ;
    tmpS16 = (MXD_S16)(pClbrPara->btPadTbl[1]&0xffff);
    gPoutPcbSwtchRecover = (MXD_S32) tmpS16;
    tmpS16 = (MXD_S16)( (pClbrPara->btPadTbl[1]>>_BIT16) &0xffff);
    gPoutOffsetC7C8=(MXD_S32) tmpS16;
    gPoutRef140Max = (MXD_U32)   (pClbrPara->btPadTbl[2]&0xffff);
    gPoutRef140Min = (MXD_U32)   (pClbrPara->btPadTbl[3]&0xffff);
    gWfTxPoutRefVal = (MXD_U32)   (pClbrPara->btPadTbl[4]&0xffff);	
    gWfTxPoutRefRange= (MXD_U32)   (pClbrPara->btPadTbl[5]&0xffff);
    mxdDbgInfo("[mxd $] castorGetParaFromNv: gPoutMode=0x%04x, gPoutPcbSwtchRecover=%d  gPoutOffsetC7C8=%d \r\n",gPoutMode,gPoutPcbSwtchRecover,gPoutOffsetC7C8);
    mxdDbgInfo("[mxd $] castorGetParaFromNv: gPoutRef140Max=0x%x[d%d]     gPoutRef140Min=0x%x[d%d]\r\n",gPoutRef140Max,gPoutRef140Max, gPoutRef140Min,gPoutRef140Min);
    mxdDbgInfo("[mxd $] castorGetParaFromNv: gWfTxPoutRefVal=0x%04x [d%d]  ,    gWfTxPoutRefRange=0x%x[d%d] \r\n",gWfTxPoutRefVal, gWfTxPoutRefVal,gWfTxPoutRefRange,gWfTxPoutRefRange);
	
    //mxdDbgInfo("[mxd $]    from NV : gPhy23Cfg[deepCh]=0x%x   [goodCh]=0x%x  \r\n",gPhy23Cfg[0],gPhy23Cfg[1]);
    //mxdDbgInfo("[mxd $]    from NV : gPhy44Cfg[deepCh]=0x%x   [goodCh]=0x%x  \r\n",gPhy44Cfg[0],gPhy44Cfg[1]);
    //mxdDbgInfo("[mxd $]    from NV : pClbrPara->reserved2[12]=0x%x \r\n",pClbrPara->reserved2[12]);
    //mxdDbgInfo("[mxd $]    from NV : pClbrPara->reserved2[13]=0x%x \r\n",pClbrPara->reserved2[13]);

}
//MXD_U16 mxd_clbr_para[268];
//int mxd_wf_dcoc_ch=9;

int MxdRfInit(void *  pParaIn)
{
    int chipid=0;
    int dac=1;
    int wf=1;
    int bt=1;
    P_CLBR_PARA_S pClbrPara = (P_CLBR_PARA_S)pParaIn;

    mxdDbgInfo("MXD_SDK version V1.8F %s \r\n","$Rev: 69 $");
    mxdDbgInfo("MXD_SDK build time   %s - %s \r\n",__DATE__,__TIME__);
	rf_reg_read(ctrl_chip_id_reg, &chipid);
	mxdDbgInfo("[mxd]MXD_SDK ChipId = 0x%04x \r\n", chipid);
#ifndef _MXD_ANDROID_
    castorLcPllInit();
    mxdDbgInfo("MxdRfInit:: castorLcPllInit done!\r\n");
#endif
    castorComInit();
    mxdDbgInfo("MxdRfInit:: castorComInit done!\r\n");
#ifdef _MXD_ANDROID_
    if (NULL!=pClbrPara)
    {
        int i=0,val=0;
        if (0==pClbrPara->len)
        {
            mxdDbgInfo("MxdRfInit:: break???? NV ERROR  pClbrPara->len==%d!! \r\n",pClbrPara->len);
            //return;
        }
        for (i = 0; i < 14; i ++)
        {
            if (0==pClbrPara->dacScaleCh[i])
            {
                mxdDbgInfo("MxdRfInit:: break! NV ERROR! Trout not ready!!!  dacScaleCh[%d]=%d!! \r\n",i,pClbrPara->dacScaleCh[i]);
                return;
            }
        }
        trout_reg_read(0x58,&val);
        mxdDbgInfo("[mxd] MxdRFinit::   swtich setting  0x58=0x%x\r\n",val);
        _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT7, _BIT0, (pClbrPara->lnaId & 0xff));
        mxdDbgInfo("[mxd] MxdRFinit:: fromNV   lna_id=0x%x\r\n",pClbrPara->lnaId);
        _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT15, _BIT8, (pClbrPara->paId & 0xff));
        mxdDbgInfo("[mxd] MxdRFinit:: fromNV   paId=0x%x\r\n",pClbrPara->paId);
        castorGetParaFromNv(pClbrPara);
    }
#else
    /*mini code need. And now android need !!*/
    _SET_TROUT2_LNA_INIT;
    _SET_TROUT2_PA_INIT;
    _SET_TROUT2_ANT_SEL_INIT;
    //_SET_TROUT2_ANT_SEL_INIT_FOR_SWTCH_LNA_OFF;
	//gPoutMode= 0x85;//  switch on   antenna 
	//gPoutMode= 0x80;// switch off   PAD
	//gPoutMode= 0x84;// switch on   PAD
     gPoutMode= 0x86;//  switch on   antenna 
#endif
    castorInitForLnaPa();
    mxdDbgInfo("MxdRfInit:: castorInitForLnaPa  done!\r\n");
    castorFdFsmInit();
    mxdDbgInfo("MxdRfInit:: castorFdFsmInit done! \r\n");

    _GET_TROUT2_ANT_SEL_;
    _SET_TROUT2_ANT_SEL_NORMAL;
    mxdDbgInfo("lihua::pClbrPara->updatedNvFlag = %d\r\n", pClbrPara->updatedNvFlag);
    if ((NULL==pClbrPara) || ((NULL!=pClbrPara) && ((ALREADY_UPDATED != pClbrPara->updatedNvFlag) && (0xFF != pClbrPara->updatedNvFlag))))
    {
        castorClbrDcocWifi(16);
        mxdDbgInfo("MxdRfInit:: castorClbrDcocWifi done\r\n");
        castorClbrDcocBt();
        mxdDbgInfo("MxdRfInit:: castorClbrDcocBt done\r\n");
        castorClbrLoftWifi();
        mxdDbgInfo("MxdRfInit:: castorClbrLoftWifi done\r\n");
        castorClbrLoftBt();
        mxdDbgInfo("MxdRfInit:: castorClbrLoftBt done\r\n");
        if ((NULL!=pClbrPara)&&((ALREADY_UPDATED != pClbrPara->updatedNvFlag) && (0xFF != pClbrPara->updatedNvFlag)))
        {
            MxdRfGetClbrPara(pClbrPara);
            mxdDbgInfo("MxdRfInit:: MxdRfGetClbrPara done\r\n");
        }
    }
    else if ((NULL!=pClbrPara)&&(ALREADY_UPDATED==pClbrPara->updatedNvFlag))
    {
        MxdRfSetClbrPara(pClbrPara);
        mxdDbgInfo("MxdRfInit:: MxdRfSetClbrPara done \r\n");
    }
    castorWorkInit();
    mxdDbgInfo("MxdRfInit:: castorWorkInit done\r\n");
    cfgTrout2Dac();
    mxdDbgInfo("MxdRfInit:: cfgTrout2Dac done\r\n");

    mxdDbgInfo("MxdRfInit:: finished! \r\n");
    castorDbgGetRfInfo();
      mxdDbgInfo("MxdRfInit:: test  MxdRfReadTempreture! \r\n");
	MxdRfReadTempreture();
    return 0;
}

//Loft result:
//0x4d8~0x4db   tbl_wftx_loft_0_cw      RW      [15:0]  wftx loft dac control word table, entry 0~3
//TxImb result:
//0x046~0x049   txp_bttx_txe1   RW      [11:0]  bttx_txe1, (0,2,10)
//Pout result:
//0x4de ~ 0x4ed,   pout_cfg table
// pRegVal      24x16bits.
int MxdRfGetClbrPara(P_CLBR_PARA_S pClbrPara)
{
    MXD_U32 i=0;
    MXD_U32 tiaDcoc=0, pgaDcoc=0;

    for (i = 0; i < 14; i ++)
    {
        gDacScaleCh[i]=pClbrPara->dacScaleCh[i];
        mxdDbgInfo("[mxd $] MxdRfGetClbrPara::  read gDacScaleCh from NV : gDacScaleCh[%d]=0x%x\r\n",i,gDacScaleCh[i]);
        if (gDacScaleCh[i]<0x140) gDacScaleCh[i]=0x140;
        if (gDacScaleCh[i]>0x2c0) gDacScaleCh[i]=0x2c0;
    }
    gDacScaleCh[14] =0x200;
    gDacScaleCh[15] =0x200;
    gSwitchLnaOffBit5_3 = pClbrPara->reserved2[14]&0x38;
    mxdDbgInfo("[mxd $]    from NV : gSwitchLnaOffBit5_3=0x%x   rsv2[14]=0x%x  \r\n",i,gSwitchLnaOffBit5_3,pClbrPara->reserved2[14]);

    gPhy23Cfg[0] =  pClbrPara->reserved2[12]&0xff;	//deepCh
    gPhy23Cfg[1] = (pClbrPara->reserved2[12]>>8) &0xff;	//goodch
    gPhy44Cfg[0] =  pClbrPara->reserved2[13]&0xff;  //deepCh
    gPhy44Cfg[1] = (pClbrPara->reserved2[13]>>8) &0xff; //goodch
    mxdDbgInfo("[mxd $]    from NV : gPhy23Cfg[deepCh]=0x%x   [goodCh]=0x%x  \r\n",gPhy23Cfg[0],gPhy23Cfg[1]);
    mxdDbgInfo("[mxd $]    from NV : gPhy44Cfg[deepCh]=0x%x   [goodCh]=0x%x  \r\n",gPhy44Cfg[0],gPhy44Cfg[1]);
    mxdDbgInfo("[mxd $]    from NV : pClbrPara->reserved2[12]=0x%x \r\n",pClbrPara->reserved2[12]);
    mxdDbgInfo("[mxd $]    from NV : pClbrPara->reserved2[13]=0x%x \r\n",pClbrPara->reserved2[13]);

    mxdDbgInfo("[mxd $] MxdRfGetClbrPara from chip. Prepare save to nv::    nvVer=0x%x\r\n",pClbrPara->nvVer);
   
    // bt tx imb
    rf_reg_read(txp_txp_bttx_txe1_reg, &(pClbrPara->btImb[0]));
    rf_reg_read(txp_txp_bttx_txe2_reg, &(pClbrPara->btImb[1]));
    // wf tx imb
    rf_reg_read(txp_txp_wftx_txe1_reg, &(pClbrPara->wifiImb[0]));
    rf_reg_read(txp_txp_wftx_txe2_reg, &(pClbrPara->wifiImb[1]));
    mxdDbgInfo("[mxd $] MxdRfGetClbrPara:: btImb[%x--%x]wifiImb[%x--%x]\r\n",pClbrPara->btImb[0],pClbrPara->btImb[1],pClbrPara->wifiImb[0],pClbrPara->wifiImb[1]);

    // bt loft/ wifi loft
    for (i=0;i<4;i++)
    {
        rf_reg_read(rfmux_tbl_bttx_loft_0_cw_reg+i,&(pClbrPara->btLoft[i]));
        rf_reg_read(rfmux_tbl_wftx_loft_0_cw_reg+i,&(pClbrPara->wifiLoft[i]));
        mxdDbgInfo("[mxd $] MxdRfGetClbrPara:[%d]: btLoft[%x]wifiLoft[%x]\r\n",i,pClbrPara->btLoft[i],pClbrPara->wifiLoft[i]);
    }
    /*
        //wifi tx pa/pad tbl
        for (i=0;i<32;i++)
        {
            rf_reg_read(rfmux_tbl_pa_band0_wfbh_h_reg + i, &(pClbrPara->wifiPaTbl[i]) );
            rf_reg_read(rfmux_tbl_pad_band0_wfbh_h_reg + i, &(pClbrPara->wifiPadTbl[i]) );
        }

        // bt tx pa/pad tbl
        for (i=0;i<6;i++)
        {
            rf_reg_read(rfmux_tx_pa_bth_h_cfg + i,  &(pClbrPara->btPaTbl[i]));
            rf_reg_read(rfmux_tx_pad_bth_h_cfg + i, &(pClbrPara->btPadTbl[i]));
        }
    */
    /*
     for (i = 0; i < 14; i ++)
     {
      pClbrPara->dacScaleCh[i]=gDacScaleCh[i];
      mxdDbgInfo("[mxd $] MxdRFinit::  gDacScaleCh[%d]=0x%x\r\n",i,gDacScaleCh[i]);
     }
    */
    for (i = 0; i < 32; i ++)
    {  //wifi dcoc
        rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR +i*4 +1, &pgaDcoc);    //wifi dcoc Pga
        rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR +i*4 +2, &tiaDcoc);     //wifi dcoc tia
        pgaDcoc &=0xffff;
        tiaDcoc &=0xffff;
        pClbrPara->wifiDcoc[i]= (tiaDcoc<<16) | pgaDcoc;
        mxdDbgInfo("[mxd $] MxdRfGetClbrPara:[%d]:WF pgaDcoc[%x]tiaDcoc[%x]=0x%x\r\n",i,pgaDcoc,tiaDcoc,pClbrPara->wifiDcoc[i]);

        //bt dcoc
        rf_mem_read(BT_RX_GAIN_TBL_START_ADDR+ i*4 +1, &pgaDcoc);    //btPga
        rf_mem_read(BT_RX_GAIN_TBL_START_ADDR+ i*4 +2, &tiaDcoc);  //btTia
        pgaDcoc &=0xffff;
        tiaDcoc &=0xffff;
        pClbrPara->btDcoc[i]= (tiaDcoc<<16) | pgaDcoc;
        mxdDbgInfo("[mxd $] MxdRfGetClbrPara:[%d]:BT pgaDcoc[%x]tiaDcoc[%x]=0x%x\r\n",i,pgaDcoc,tiaDcoc,pClbrPara->btDcoc[i]);
    }


    for (i = 0x1c; i <=0x1f; i ++)
    {  //wifi dcoc
        //rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR +i*4 +1, &pgaDcoc );   //wifi dcoc Pga
        //rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR +i*4 +2, &tiaDcoc);     //wifi dcoc tia
        pgaDcoc =0x8080;
        tiaDcoc =gWfChDcoc[(16-1)*5+1+i-0x1c];  //ch16 2308MHz gIdx=i
        pClbrPara->wifiDcoc[i]= (tiaDcoc<<16) | pgaDcoc;
        mxdDbgInfo("[mxd $] MxdRfGetClbrPara:[%d]:2308MHz:WF pgaDcoc[%x]tiaDcoc[%x]=0x%x\r\n",i,pgaDcoc,tiaDcoc,pClbrPara->wifiDcoc[i]);

        pgaDcoc =0x8080;
        tiaDcoc =gWfChDcoc[(17-1)*5+1+i-0x1c];  //ch17 2508MHz  gIdx=i
        pClbrPara->wifiDcoc[i+32]= (tiaDcoc<<16) | pgaDcoc;
        mxdDbgInfo("[mxd $] MxdRfGetClbrPara:[%d]:2508MHz:WF pgaDcoc[%x]tiaDcoc[%x]=0x%x\r\n",i+32,pgaDcoc,tiaDcoc,pClbrPara->wifiDcoc[i+32]);

    }

    pClbrPara->reserved[5] = gIdxWfTxPwrShift;
    pClbrPara->reserved[6] = gWfTxScaleCfg;
    pClbrPara->updatedNvFlag = ALREADY_UPDATED;

    return 0;
}

int MxdRfSetClbrPara(P_CLBR_PARA_S pClbrPara)
{
    //MXD_U32 clbtParaRegVal[256]= {0};
    //MXD_U16 * pBuf=NULL;
    //MXD_U32 paraMap[]=      // start addr   len
    //{
    //    0x46,  4,       //txImb bttxe1,bttxe2,wifitxe1,wifiTxe2
    //    0x4d4, 8,       //btLoftRslt: 16bit x4   wifiLoftRslt: 16bit x4
    //};
    //int idx=0;
    //int regVall=0;
    MXD_U32 i=0;
    MXD_U32 val=0;


    //if (pClbrPara->updatedNvFlag==1)
    mxdDbgInfo("[mxd $] MxdRfSetClbrPara:: Get para from NV@@@!   Config to chip Now@@@!\r\n");

    _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT7, _BIT0, (pClbrPara->lnaId & 0xff));
    mxdDbgInfo("[mxd $] MxdRfSetClbrPara::    lna_id=0x%x\r\n",pClbrPara->lnaId);
    _REGBSS(_RFREG_SW_CFG_PA_LNA, _BIT15, _BIT8, (pClbrPara->paId & 0xff));
    mxdDbgInfo("[mxd $] MxdRfSetClbrPara::    paId=0x%x\r\n",pClbrPara->paId);

    // bt tx imb
    rf_reg_write(txp_txp_bttx_txe1_reg, pClbrPara->btImb[0]);
    rf_reg_write(txp_txp_bttx_txe2_reg, pClbrPara->btImb[1]);
    // wf tx imb
    rf_reg_write(txp_txp_wftx_txe1_reg, pClbrPara->wifiImb[0]);
    rf_reg_write(txp_txp_wftx_txe2_reg, pClbrPara->wifiImb[1]);
    mxdDbgInfo("[mxd ] MxdRfSetClbrPara:: btImb[%x--%x]wifiImb[%x--%x]\r\n",pClbrPara->btImb[0],pClbrPara->btImb[1],pClbrPara->wifiImb[0],pClbrPara->wifiImb[1]);
// zl@0904

    // wf dac scale
    for (i = 0; i < 14; i ++)
    {
        gDacScaleCh[i]=pClbrPara->dacScaleCh[i];
        mxdDbgInfo("[mxd $] MxdRFinit::  gDacScaleCh[%d]=0x%x\r\n",i,gDacScaleCh[i]);
        if (gDacScaleCh[i]<0x140) gDacScaleCh[i]=0x140;
        if (gDacScaleCh[i]>0x2c0) gDacScaleCh[i]=0x2c0;
    }
    gDacScaleCh[14] =0x200;
    gDacScaleCh[15] =0x200;
    gSwitchLnaOffBit5_3 = pClbrPara->reserved2[14]&0x38;
    mxdDbgInfo("[mxd $]    from NV : gSwitchLnaOffBit5_3=0x%x   rsv2[14]=0x%x  \r\n",i,gSwitchLnaOffBit5_3,pClbrPara->reserved2[14]);

    gPhy23Cfg[0] =  pClbrPara->reserved2[12]&0xff;
    gPhy23Cfg[1] = (pClbrPara->reserved2[12]>>8) &0xff;
    gPhy44Cfg[0] =  pClbrPara->reserved2[13]&0xff;
    gPhy44Cfg[1] = (pClbrPara->reserved2[13]>>8) &0xff;
    mxdDbgInfo("[mxd $]    from NV : gPhy23Cfg[deepCh]=0x%x   [goodCh]=0x%x  \r\n",gPhy23Cfg[0],gPhy23Cfg[1]);
    mxdDbgInfo("[mxd $]    from NV : gPhy44Cfg[deepCh]=0x%x   [goodCh]=0x%x  \r\n",gPhy44Cfg[0],gPhy44Cfg[1]);
    mxdDbgInfo("[mxd $]    from NV : pClbrPara->reserved2[12]=0x%x \r\n",pClbrPara->reserved2[12]);
    mxdDbgInfo("[mxd $]    from NV : pClbrPara->reserved2[13]=0x%x \r\n",pClbrPara->reserved2[13]);

    //rf_reg_write(rfmux_pout_tx_scale_reg, gDacScaleCh[gChannelNO]);
    //mxdDbgInfo("[mxd]wifi freqChin=%d [1~15] gDacScaleCh=0x%x\n",gChannelNO,gDacScaleCh[gChannelNO]);

    //rf_reg_read(rfmux_pout_tx_scale_reg, &val);
    //mxdDbgInfo("2[mxd]wifi freqChin=%d [1~15] gDacScaleCh=0x%x, reg=%#x\n",gChannelNO,gDacScaleCh[gChannelNO],val);

    // bt loft/ wifi loft
    for (i=0;i<4;i++)
    {
        rf_reg_write(rfmux_tbl_bttx_loft_0_cw_reg+i,pClbrPara->btLoft[i]);
        rf_reg_write(rfmux_tbl_wftx_loft_0_cw_reg+i,pClbrPara->wifiLoft[i]);
        mxdDbgInfo("[mxd] MxdRfSetClbrPara:[%d]: btLoft[%x]wifiLoft[%x]\r\n",i,pClbrPara->btLoft[i],pClbrPara->wifiLoft[i]);
    }
    /*
        //wifi tx pa/pad tbl
        for (i=0;i<32;i++)
        {
            rf_reg_write(rfmux_tbl_pa_band0_wfbh_h_reg + i, pClbrPara->wifiPaTbl[i]);
            rf_reg_write(rfmux_tbl_pad_band0_wfbh_h_reg + i, pClbrPara->wifiPadTbl[i]);
        }

        // bt tx pa/pad tbl
        for (i=0;i<6;i++)
        {
            rf_reg_write(rfmux_tx_pa_bth_h_cfg + i,  pClbrPara->btPaTbl[i]);
            rf_reg_write(rfmux_tx_pad_bth_h_cfg + i, pClbrPara->btPadTbl[i]);
        }
    */
    for (i = 0; i < 32; i ++)
    {  //wifi dcoc
        rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR +i*4 +1, (pClbrPara->wifiDcoc[i])&0xffff);     //wifi dcoc Pga
        rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR +i*4 +2, (pClbrPara->wifiDcoc[i])>>16);   //wifi dcoc tia
        mxdDbgInfo("[mxd] MxdRfSetClbrPara:[%d]:WF =0x%x\r\n",i,pClbrPara->wifiDcoc[i]);

        rf_reg_write(BT_RX_GAIN_TBL_START_ADDR+ i*4 +1, (pClbrPara->btDcoc[i])&0xffff);    //btPga
        rf_reg_write(BT_RX_GAIN_TBL_START_ADDR+ i*4 +2, (pClbrPara->btDcoc[i])>>16);     //btTia
        mxdDbgInfo("[mxd] MxdRfSetClbrPara:DCOC[%d]:BT  =0x%x     [%d]:WF =0x%x\r\n",i,pClbrPara->btDcoc[i],pClbrPara->wifiDcoc[i]);
    }

    for (i = 0x1c; i <=0x1f; i ++)
    {  //wifi dcoc
        gWfChDcoc[(16-1)*5+1+i-0x1c]= pClbrPara->wifiDcoc[i]>>16; //ch16 2308MHz gIdx=i
        gWfChDcoc[(17-1)*5+1+i-0x1c]= pClbrPara->wifiDcoc[i+32]>>16; //ch17 2508MHz  gIdx=i
        mxdDbgInfo("[mxd ] ch16 2308MHz MxdRfSetClbrPara:[wfRxIdx=%d]: tiaDcoc[%x]=0x%x\r\n",i,pClbrPara->wifiDcoc[i]>>16);
        mxdDbgInfo("[mxd ] ch17 2508MHz MxdRfSetClbrPara:[wfRxIdx=%d]: tiaDcoc[%x]=0x%x\r\n",i,pClbrPara->wifiDcoc[i+32]>>16);
    }
#ifdef  _MXD_WF_DCOC_INSERT_
    castorGetWfChDcocAll();
#endif

    gIdxWfTxPwrShift = pClbrPara->reserved[5];
    gWfTxScaleCfg = pClbrPara->reserved[6];
    mxdDbgInfo("[mxd ]  MxdRfSetClbrPara::    pClbrPara->reserved[5,6] = 0x%04x , 0x%04x  \r\n",pClbrPara->reserved[5],pClbrPara->reserved[6]);
    mxdDbgInfo("[mxd ]  MxdRfSetClbrPara::   gWfTxScaleCfg = 0x%04x  \r\n",gWfTxScaleCfg);
    mxdDbgInfo("[mxd ]  MxdRfSetClbrPara::   gIdxWfTxPwrShift= 0x%04x \r\n",gIdxWfTxPwrShift);
#ifdef _MXD_WFBT_POUT_
    rfClbrShiftWftxgain(0x01,0x0f, gIdxWfTxPwrShift-4);
    rfClbrShiftWftxgain(0x10,0x1f, gIdxWfTxPwrShift-4);
    rfClbrShiftWftxgain(0x20,0x2f, gIdxWfTxPwrShift);
    rfClbrShiftWftxgain(0x30,0x3f, gIdxWfTxPwrShift-4);	
    rfClbrShiftBttxgain (0x7,  0xb, (gIdxWfTxPwrShift+2)/4-1);	
#endif
    rf_reg_write(rfmux_pout_tx_scale_reg ,   gWfTxScaleCfg);
    mxdDbgInfo("[mxd ] MxdRfSetClbrPara   gIdxWfTxPwrShift  =  %x  rfmux_pout_tx_scale_reg = %x  !!!! \n",gIdxWfTxPwrShift,gWfTxScaleCfg);

    return 0;
}

int MxdRfSetBtMode(unsigned short flagWorkMode)    // flag =1 work  0:idle
{
//    MXD_U16 rfRegVal;
    MXD_U32 flagStatusFmWifiBt;

    rf_reg_read(_RFREG_SW_RF_STATUS_,&flagStatusFmWifiBt);
    if (0==flagWorkMode)
    {    //idle
        mxdDbgInfo("[mxd]BT Mode disable!\n ");
        _REGBC(_RFREG_SW_RF_STATUS_,_FLAG_B_BT);        //Bit is bt workstatus
        //_REGBC(rf_ldo_ctrl_reg,_BIT7);
        castorCfgLdo();

        _REGBC(rf_lobuffer_cfg_reg,_BIT1);
        // disable bt lo
        if (0==(flagStatusFmWifiBt&(1<<_FLAG_B_WIFI)))
        {
            // _REGBC(rf_ldo_ctrl_reg,_BIT10); //idle
            castorSetRfEntry(0);   //
        }
        else
        {
            mxdDbgInfo("[mxd]BT Mode disable!!  and  wifi enable! >>Wifi Only \n ");
            //rf_reg_write(rfmux_sprd_mode_sel_reg, (_RF_MODE_WIFI_TX<<9)|(_RF_MODE_INIT<<6)|(_RF_MODE_WIFI_RX<<3)|_RF_MODE_INIT);//set sprd entry
            castorSetRfEntry(2);//wifionly
        }
    }
    else
    {     //Enable bt
        //  rf_reg_write(0x207, 0x119);
	MxdRfSwtchLnaCfg(1);//set switch lna setting to default
        mxdDbgInfo("[mxd]BT Mode enable!\n ");
        _REGBS(_RFREG_SW_RF_STATUS_,_FLAG_B_BT); //Bit is bt workstatus
        castorCfgLdo();
        _REGBS(rf_lobuffer_cfg_reg, _BIT1);        // bt lo

        if (0==(flagStatusFmWifiBt&(1<<_FLAG_B_WIFI)))
        {
            mxdDbgInfo("[mxd]BT enable!!  and  wifi disable! >>Mxd BT only  \n");
            castorSetRfEntry(1);//bt only
        }
        else
        {
            mxdDbgInfo("[mxd]BT enable!!  and  wifi enable!\n >>Mxd Co-Listen \n");
            castorSetRfEntry(3);//colisten
        }
        rf_reg_write(rfmux_mode_sel_reg, 0x0018);
    }

    return 0;
}



int MxdRfSetFmMode(unsigned short flagWorkMode)    // flag =1 work  0:idle
{
    int i;
    MXD_U32 rfRegVal,rfRegVal2;        //4sunzl add 0411
    MXD_U32 fmPga3bit=0,fmTia3bit=0;
    MXD_U32 flagStatusFmWifiBt=0;
    MXD_U8 gainTblFm[20] =
    {
        0x24, //0x226
        0x24, //0x224
        0x24, //0x222
        0x24, //0x220
        0x1c, //0x21e
        0x1c,
        0x14,
        0x14,
        0x0c,
        0x0c,
        0x04,
        0x03,
        0x53,
        0x4b,
        0x4a,
        0x42,
        0x41,
        0x40,
        0x40,
        0x40,
    };

    if (0==flagWorkMode)
    {   //idle
        _REGBC(_RFREG_SW_RF_STATUS_,_FLAG_B_FM);        //fm idle : status udpate
        mxdDbgInfo("[mxd]FMMode disable!\n ");
        castorCfgLdo();
        rf_reg_write(rfmux_fm_cfg_reg,0x0);
    }
    else
    {  //work
        castorFmInit();
        mxdDbgInfo("[mxd]FMMode enable!\n ");
        _REGBS(_RFREG_SW_RF_STATUS_,_FLAG_B_FM);        //fm work : status udpate
        castorCfgLdo();
        rf_reg_write(frxp_fm_dcr_ctrl_reg,0x17);    //fm DCR en@0502
        rf_reg_write(frxp_fm_imb_ctrl_reg,0x2ce);    //fm imbEn@0502

        for (i = 0; i < 20; i++)
        {
            fmTia3bit= _BSG(gainTblFm[i], _BIT5, _BIT3);
            fmPga3bit = _BSG(gainTblFm[i], _BIT2, _BIT0);
            rfRegVal2 = (fmPga3bit << _BIT5) | (fmTia3bit<<_BIT2)|((gainTblFm[i]>>_BIT6)&0x1);
            rf_reg_write(0x626 - (2 * i), rfRegVal2);
            rf_reg_write(0x626 - (2 * i+1), 0x8080);
            mxdDbgInfo("[mxd]FM agcTbl [index=%d]  gain=0x%x  AGCTBL regAddr=0x%x regVal=0x%x    !\r\n ",i, gainTblFm[i], 0x626 - (2 * i),rfRegVal2);
        }
    }
    _REGBSS(fmagc_setup_reg, _BIT1, _BIT0, 0x03);//    _REGBSS(0x6d, _BIT1, _BIT0, 0x03);
    return 0;
}


// zl@2014-01-09 
void  MxdRfGetRfMode(unsigned int * pFlagFmBit2BtBit1WfBit0 )
{	
    	MXD_U32 flagStatusFmWifiBt;
    	rf_reg_read(_RFREG_SW_RF_STATUS_,&flagStatusFmWifiBt);
	*pFlagFmBit2BtBit1WfBit0= (unsigned int)flagStatusFmWifiBt;	
	mxdDbgInfo("[mxd]MxdRfGetRfMode! flagStatusFmWifiBt =0x%x!\n ");
}

int MxdRfSetWifiMode(unsigned short flagWorkMode)  // flag =1 wifi-rf enable   0:wifi-rf disable
{
    //MXD_U16 rfRegVal;
    MXD_U32 flagStatusFmWifiBt;
    rf_reg_read(_RFREG_SW_RF_STATUS_,&flagStatusFmWifiBt);
    if (0==flagWorkMode)
    {
        mxdDbgInfo("[mxd]WifiMode disable!\n ");
        _REGBC(_RFREG_SW_RF_STATUS_,_FLAG_B_WIFI);      //clear wifi workstatus
        _REGBC(rf_lobuffer_cfg_reg,_BIT2);
        //_REGBC(rf_ldo_ctrl_reg,_BIT9);                //sunzl
        castorCfgLdo();

        //        if (0==regBitsGet(_RFREG_SW_RF_STATUS_,_FLAG_B_BT,_FLAG_B_BT))
        if (0== (flagStatusFmWifiBt&(1<<_FLAG_B_BT)))
        {
            mxdDbgInfo("[mxd]Wifi Mode disable!!  and  Bt disable! >>>mxd Idle!\n ");
            // rf_reg_write(rfmux_sprd_mode_sel_reg, (_RF_MODE_INIT<<9)|(_RF_MODE_INIT<<6)|(_RF_MODE_INIT<<3)|_RF_MODE_INIT);//set sprd entry
            castorSetRfEntry(0); //ilde-> disable PA
        }
        else
        {
            castorSetRfEntry(1); //bt only
            mxdDbgInfo("[mxd]WIFI Mode disable!!  and  bt enable! coWork>>>BT-Only,Need to update sprd entry. \n ");
        }
    }
    else
    {  //Enable
        //         rf_reg_write(0x207, 0x139);
        mxdDbgInfo("[mxd]WifiMode enable!\n ");
        _REGBS(_RFREG_SW_RF_STATUS_,_FLAG_B_WIFI);      //Set wifi flag = work
        castorCfgLdo();
        _REGBS(rf_lobuffer_cfg_reg,_BIT2);
        //        if (0==regBitsGet(_RFREG_SW_RF_STATUS_,_FLAG_B_BT,_FLAG_B_BT))
        if (0== (flagStatusFmWifiBt&(1<<_FLAG_B_BT)))
        {
            castorSetRfEntry(2);  //wifi only
            mxdDbgInfo("[mxd]Wifi Mode Enable!!  and  Bt disalbe! >>>Wifi Only!\n ");
        }
        else
        {
            mxdDbgInfo(">> into Mxd Co-Listen need to config entry \n ");
            castorSetRfEntry(3);   //co listen
            mxdDbgInfo("[mxd]WIFI Mode Enable!!  and  bt enable! >>> into Mxd Co-Listen\n ");
        }
        rf_reg_write(rfmux_mode_sel_reg, 0x0018);               //enable sprd mode   sunzl
    }
    return 0;
}


int castorSetFreqWifiCh(MXD_U32 freqChin)
{
    //
    MXD_U32 flagRegAfc=0;
    MXD_U32 freqCh=freqChin;

    if (freqCh>=15)
    {
        mxdDbgInfo("[mxd]wifi input channel error@castorSetFreqWifiCh!freqChin=%d [1~15]\n",freqChin);
    }
    if (freqCh>0) freqCh--;

    rf_reg_write(afc_fvco_wifi_config, gFreqWifiCh[freqCh*3+2]);
    rf_reg_write(sdm_syn_divn_cfg5_reg,gFreqWifiCh[freqCh*3+0]);
    rf_reg_write(sdm_syn_divn_cfg6_reg,gFreqWifiCh[freqCh*3+1]);
    MxdSleep(1);
    _REG_CHK2(afc_syn_status_reg, _BIT1);
    do
    {
        rf_reg_read(afc_syn_status_reg,&flagRegAfc);         //#define vco_lock_wifi_bit (1<<1)
        //rf_reg_read(afc_syn_status_reg,&flagRegAfc);         //#define vco_lock_wifi_bit (1<<1)
    }
    while (0);//(0==(flagRegAfc&vco_lock_wifi_bit));
    if (0==(flagRegAfc&vco_lock_wifi_bit))
    {
        // rf_reg_read(afc_syn_status_reg,&flagRegAfc);         //#define vco_lock_wifi_bit (1<<1)

        mxdDbgInfo("[mxd]wifi UnLock!freqChin=%d indx=%d flagRegAfc=%x \r\n",freqChin,freqCh,flagRegAfc);
    }
    else
    {
        if (freqChin==14)
        {
            mxdDbgInfo("[mxd]wifi Locked!  freqChin=%d indx=%d TstLO=%d FreqMHz = 2484Mhz   \r\n ",freqChin,freqCh,2484*4/3);
        }
        else if (freqChin>=15)
        {
            mxdDbgInfo("[mxd]wifi Locked!  test freqChin=%d  \r\n ",freqChin);
        }
        else
        {
            mxdDbgInfo("[mxd]wifi Locked!  freqChin=%d indx=%d TstLO=%d  FreqMHz = %dMhz \r\n ",freqChin,freqCh,(2412+5*freqCh)*4/3, 2412+5*freqCh);
        }
    }
    return 0;
}


int castorSetFreqMhzBt(MXD_U32 freqMHz)
{
    MXD_U32 flagRegAfc=0;
    MXD_U16 freqCh=freqMHz-2400;

    //    freqCh=40;
    if ((freqMHz>2483)||(freqMHz<2400))
    {
        mxdDbgInfo("[mxd]BT input error@castorSetFreqMhzBt! freqMHz=%d  freqCh=%d  tstLo= %dMhz\n", freqMHz, freqCh,freqMHz*4/3);
        return 1;
    }

    rf_reg_write(afc_fvco_bt_config, gFreqMhzBt[freqCh*3+2]);      //#define vco_lock_bt_bit (1<<2)
    rf_reg_write(sdm_syn_divn_cfg3_reg,gFreqMhzBt[freqCh*3+0]);
    rf_reg_write(sdm_syn_divn_cfg4_reg,gFreqMhzBt[freqCh*3+1]);

    MxdSleep(1);
    do
    {
        rf_reg_read(0x6b,&flagRegAfc);
        rf_reg_read(afc_syn_status_reg,&flagRegAfc);
    }
    while (0);//(0==(flagRegAfc&vco_lock_bt_bit));

    if ((0==(flagRegAfc&vco_lock_bt_bit)))
    {
        mxdDbgInfo("[mxd]BT UnLock! freqMHz=%d freqCh=%d tstLo= %dMhz \r\n", freqMHz,freqCh,freqMHz*4/3);
    }
    else
    {
        mxdDbgInfo("[mxd]BT Locked! v freqMHz=%d  freqCh=%d  tstLo= %dMhz\r\n", freqMHz, freqCh,freqMHz*4/3);
    }
    return 0;
}



//
int castorSetFreqHKhzFm(MXD_U32 freqHKhz)
{
    MXD_U32 flagRegAfc=0;
    MXD_U16 fmCh=freqHKhz-700;
    MXD_U16 i=0;

    if ((freqHKhz>1080)||(freqHKhz<700))//70.0MHz~108MHz
    {
        mxdDbgInfo("[mxd]FM inputError@ castorSetFreqHKhzFm! freq=%d00kHz\n", freqHKhz);
        return 1;
    }

    _REGBC(ctrl_fm_adc_ctrl_reg, _BIT12);
/*	
    for (i=0; i<(sizeof(gFmFreqHKhzForSpur)/sizeof(gFmFreqHKhzForSpur[0]));i++)
    {
        if (freqHKhz==gFmFreqHKhzForSpur[i])
        {
            fmCh=freqHKhz-700+2;
			freqHKhz+=2;
            _REGBS(ctrl_fm_adc_ctrl_reg, _BIT12);
            break;
        }
    }
*/
    if (freqHKhz>864)//86.4Mhz
    {
        _REGBS(rf_vco_cfg_fm_reg,_BIT6);
    }
    else
    {
        _REGBC(rf_vco_cfg_fm_reg,_BIT6);
    }
    rf_reg_write(afc_fvco_fm_config, gFmFreqHKhz[fmCh*3+2]);       //#define vco_lock_fm_bit (1<<3)
    rf_reg_write(sdm_syn_divn_cfg1_reg,gFmFreqHKhz[fmCh*3+0]);
    rf_reg_write(sdm_syn_divn_cfg2_reg,gFmFreqHKhz[fmCh*3+1]);
    //    mxdDbgInfo(" %s :: Fm set freq : %dHKhz  %d  %d  %d \n",__func__,freqHKhz,gFmFreqHKhz[fmCh*3+0],gFmFreqHKhz[fmCh*3+1],gFmFreqHKhz[fmCh*3+2]);
    MxdSleep(1);
    _REGBSS(rfmux_fm_cfg_reg,_BIT9,_BIT8,0x02);
    _REGBS(rfmux_fm_cfg_reg,_BIT8);
    do
    {
        rf_reg_read(afc_tune_fm_status,&flagRegAfc);//delay 20us
        rf_reg_read(afc_syn_status_reg,&flagRegAfc);
    }
    while (0);//(0==(flagRegAfc&vco_lock_fm_bit));
    if (0==(flagRegAfc&vco_lock_fm_bit))
    {
        mxdDbgInfo("[mxd]FM Lo UnLock! freq=%d00kHz\n", freqHKhz);
    }
    else
    {
        mxdDbgInfo("[mxd]FM Locked!  freq=%d00kHz\n", freqHKhz);
    }
    return 0;
}

int  MxdRfSwtchLnaCfg(MXD_U32 flagOnOff) // 0:off  1: On
// if(wifiOnly)  config switch based on flagOnoff   and  return 0              
// if (bt/ON) config switch to default and  return 1
{
	  MXD_U32 flagStatusFmWifiBt=0;
	  MXD_U32 regVal=0;
	  MXD_U32 antSelRegCur =0;
	  MXD_U32 antSetBit5_3Default =0x0;
	  trout_reg_read(_TROUT_ANT_SEL_REG_,&antSelRegCur);
	  antSelRegCur &= 0xfc7;
	//gSwitchLnaOffBit5_3
	rf_reg_read(_RFREG_SW_ANTSEL_BK, &antSetBit5_3Default);
	antSetBit5_3Default &= 0x0038;
		 
       rf_reg_read(_RFREG_SW_RF_STATUS_,&flagStatusFmWifiBt);	  
	if (1==(flagStatusFmWifiBt&(1<<_FLAG_B_BT)))
	{

		 antSelRegCur |= antSetBit5_3Default;
           	 trout_reg_write(_TROUT_ANT_SEL_REG_, antSelRegCur); 
		return 1;
	}
	else
	{
		if (flagOnOff==0)
		{
			antSelRegCur |= (gSwitchLnaOffBit5_3&0x38);
           	       trout_reg_write(_TROUT_ANT_SEL_REG_, antSelRegCur); 
		}
		else
	       {
	  	       antSelRegCur |= antSetBit5_3Default;
           	       trout_reg_write(_TROUT_ANT_SEL_REG_, antSelRegCur);      	
		}				
	}
	return 0;
}


int  MxdRfSetFreqHKhzFm(unsigned int freqHKHz)
{
    castorChkLdo();
    castorSetFreqHKhzFm(freqHKHz);
    return 0;
}

int  MxdRfSetFreqWifiCh(unsigned int chNo)
{ //zl@0701 for spur CH
    MXD_U32 freqCh=chNo;
    MXD_U32 val = 0;
    MXD_U32 scaletemp = 0;
    if ((chNo >=5) && (chNo<=8))
    	gWfChFromAp = 0;
   else
   		gWfChFromAp = 1;
    castorChkLdo();
    castorWfCfgForCh(chNo);

#ifdef _MXD_NV_SUPPORT_
    if (freqCh>0) freqCh--;
    //gChannelNO = freqCh;
    scaletemp = gWfTxScaleCfg +0x200- gDacScaleCh[freqCh];
    if ((0!=gDacScaleCh[freqCh]) && (freqCh<14))
    {
        rf_reg_read(rfmux_pout_tx_scale_reg, &val);
        if (scaletemp < 0x110)
        {
            scaletemp =0x110;
        }
        else if (scaletemp > 0x2e0)
        {
            scaletemp = 0x2e0;
        }

        mxdDbgInfo("1[mxd]wifi chin=[1~15] gDacScaleCh[%d]=0x%x, reg=0x%x->0x%x   gWfTxScaleCfg=0x%x\n",freqCh,gDacScaleCh[freqCh],val, scaletemp,gWfTxScaleCfg);
        rf_reg_write(rfmux_pout_tx_scale_reg, scaletemp);
        //rf_reg_read(rfmux_pout_tx_scale_reg, &val);
        //mxdDbgInfo("2[mxd]wifi freqChin=%d [1~15] gDacScaleCh=0x%x, reg=%#x\n",freqCh,gDacScaleCh[freqCh],val);
    }
#else
	//        if(gWfTxScaleCfg < 0x140) gWfTxScaleCfg =0x140;
	//  else if(gWfTxScaleCfg > 0x2c0) gWfTxScaleCfg = 0x2c0;
    rf_reg_write(rfmux_pout_tx_scale_reg, gWfTxScaleCfg);
#endif
    castorSetFreqWifiCh(chNo);
    return 1;
}

int  MxdRfSetFreqMHzBt(unsigned int freqMzh)
{
    castorSetFreqMhzBt(freqMzh);
    return 1;
}


MXD_API_C void castorHostWifiRxInit(void)
{
    int i=0;
    MXD_U32 regMap[]= // fd config
    {
        0x00bd, 0x0003, // Host mode, WFRX
        0x00bf, 0x0000, // Set gain from rbus
        0x0205, 0x6A38, // Enable WF SWBias, Filter, PGA, ADC, DCOC
        0x0206, 0x0028, // Set WF path
        0x0207, 0x0187, //gdy@0907 186->187// Enable LNA, TIA
        0x021F, 0x0105, // Set nolap LO, LO mixer enalbe
    };

    for (i=0;i<sizeof(regMap)/sizeof(regMap[0])/2;i++)
    {
        rf_reg_write(regMap[2*i], regMap[2*i+1]);
    }
}

/* WiFi Force TX Mode config */
void castorHostWifiTxInit(void)
{
    int i=0;
    MXD_U32 regMap[]=
    {
        0x00bd,0x0001,  // Host mode, WFTX
        0x00bF,0x0000,  // Set gain from rbus
        0x0205,0x0900,  // TX DAC enable
        0x0206,0x0018,  // Set wifi path, enable TX Lobuf
        0x0207,0x0F39,  // Enable WF TX Filter, PGA, Modulator, TXGM Boost, TX Bias, LOFT DAC, PA
        0x021F,0x0105,  // Set LO mixer
    };

    for (i=0;i<sizeof(regMap)/sizeof(regMap[0])/2;i++)
    {
        rf_reg_write(regMap[2*i], regMap[2*i+1]);
    }

}

void MxdTstForceHostWifiTx()
{
    MXD_U32 reg_data;
    castorHostWifiTxInit();
    rf_reg_write(0x41, 0x380);
    rf_reg_read(0x229, &reg_data);
    mxdDbgInfo("rf reg [229] = %x\r\n", reg_data);
    mxdDbgInfo("[mxd test]MxdTstForceHostWifiTx done!\r\n");
}
void castorHostWifiTxRxInit(void)
{
    int i=0;
    MXD_U32 regMap[]=
    {
        // fd config
        //0x0201,0x3f0F ,// Enable LCPLL, LDO_WF_CP/WF_LO/RX/TX, DAB, BG
        0x0205, 0x6A38, // Enable WF SWBias, Filter, PGA, ADC, DCOC zl@from wifi RX mode
        //       0x0205, 0x6A38, // Enable WF SWBias, Filter, PGA, ADC, DCOC zl@from wifi RX mode
        0x0206, 0x0028, // Set WF path zl@@ from wifi rxMode
        0x0207, 0x0187, // Enable LNA, TIA zl@from wifi rxmode

        // set0x control source
        0x00c7,0x0000,
        0x021F,0x0105 ,// Set nolap LO, LO mixer enalbe
        0x0209,0x00c1 ,// Config LNA SW bias, CTRL_VB2
        0x020a,0xa4a4 ,// Config LNA VB1_Main, VB1_AUX
        0x020F,0x030d ,// Enable WF gmctrl 11, hgm 11, hvsat 01
        0x0210,0x2b59 ,// LNA 0dB, TIA 32dB, PGA 0dB -38dBm
        0x0210, 0x615B, // LNA 20dB, TIA 18dB, PGA 0dB

        0x0219,0x8080 ,// TIA DCOC Config
        0x0218,0x8080 ,// PGA DCOC Config
        //0x00bF,0x0000 ,// Set gain from rbus
        0x0208,0x022F ,// Enable PA config
        0x0213,0x000f ,// Disable negm, tuning gm output cap
        0x0211,0x003d ,// WF TX pga: 4dB; TxGm 0dB                                                      0031
        // PA,0xconfig
        0x020b,0xa15a,
        0x020c,0x5847 ,// 7f67
        0x020d,0xbc7f ,// bc4f
        0x020e,0x807c ,// 5f00
    };

    for (i=0;i<sizeof(regMap)/sizeof(regMap[0])/2;i++)
    {
        rf_reg_write(regMap[2*i], regMap[2*i+1]);
    }
    mxdDbgInfo("[mxd test]castorHostWifiTxRxInit done!\r\n");

}

void castorFmInit(void)
{
    int i=0;
    MXD_U32 regMap[]=
    {
        0x0203,0x023F,  // Enable LOBuf, Bias, DCOC, TIA, Filter, PGA and ADC
        0x0203,0x033F,  // Reset lo /8 path
        0x021c,0x8080,  // Set FM DCOC DAC
        0x0202,0x0004,  // RF gain 12dB, PGA gain 0dB
    };

    for (i=0;i<sizeof(regMap)/sizeof(regMap[0])/2;i++)
    {
        rf_reg_write(regMap[2*i], regMap[2*i+1]);
    }
    mxdDbgInfo("[mxd test]castorFmInit done!\r\n");
}

void MxdRfPulse(MXD_S32 rssi,MXD_U32 rate1, MXD_U32 rate2) // 0: auto   1: cfgAci 2: cfgSens.
{
    MXD_U32 i = 0;
    MXD_S32 rssiGateArray[]=
    {
        //rate1, rate2,rssi
        6,  0, -83+3, //11g.6M
        9,  0, -80,     //11g.
        12, 0, -80,
        18, 0, -80,
        24, 0, -80,
        36, 0, -80,
        48, 0, -80,
        54, 0, -66-2, //11g54M
        1,  0, -82,   //* 11b
        2,  0, -79-3, //11b.2M
        5,  1, -77,   //11b.5.5M
        11, 0, -75-3, //11b.11M
    };
    MXD_S32 realRssi =0;
    mxdDbgInfo("[mxdRfPulse]   ::  rssi=%d dBm \r\n ",rssi);
    mxdDbgInfo("[mxdRfPulse]   ::  rate=%d.%d M \r\n ", rate1,rate2*5);
    realRssi = rssi+2;  //delta = 2dB
    for (i=0;i<sizeof(rssiGateArray)/sizeof(rssiGateArray[0])/3; i++)
    {
        //if(i<4)
        if (i>7)
        { // 11b config changed.
            if (rssiGateArray[i*3] == rate1)
            {
                if (realRssi< rssiGateArray[i*3+2])
                    MxdRfCfgForAci(2); //sense
                else if (realRssi<rssiGateArray[i*3+2]+5+4)
                    MxdRfCfgForAci(1); //aci
                else
                    MxdRfCfgForAci(0); //default
                break;
            }
        }
        else
        {
            // do nothing.??
            MxdRfCfgForAci(0); //default setting
        }

    }

}
void MxdRfCfgForAci(int  flag) // 0: default   1: cfgAci 2: cfgSens.
{
    MXD_U32 i=0,regValRssi=0,sumRssi =0;
    MXD_U32 rfRegcfgAci[]=
    {
        //ac i new update
        0x485, 0x3,
        0x484, 0x4040,
        //if  reg_78>0xaf
        //aci  config
        0x447, 0x7f7f,
        0x446, 0x7f7e,
        0x445, 0x5f5e,
        0x444, 0x5d5c,
        0x443, 0x5b5a,
        0x442, 0x5958,
        0x441, 0x5756,
        0x440, 0x5554,
        0x43f , 0x5310,
        //'phy
        //0x23, 0x1b,
        //0x44, 0x1b,
    };

    MXD_U32 rfRegcfgSens[]=
    {
        //ac i new update
        0x485, 0x3,
        0x484, 0x4040,
        //el se    // 'sen
        0x447, 0x7f7f,
        0x446, 0x7f7e,
        0x445, 0x7d7c,
        0x444, 0x7b7a,
        0x443, 0x7978,
        0x442, 0x7776,
        0x441, 0x7574,
        0x440, 0x7372,
        0x43f , 0x7110,
        // 'phy
        //0x23, 0x1d ,
        //0x44, 0x1d ,
    };


    MXD_U32 rfRegCfgDft[]=
    {
        //ac i new update

        0x485, 0x0002,
        0x484, 0xa4a4,
        //el se    // 'sen
        0x447, 0x7f7f,
        0x446, 0x7f7e,
        0x445, 0x7d7c,
        0x444, 0x7b7a,
        0x443, 0x7978,
        0x442, 0x7776,
        0x441, 0x7574,
        0x440, 0x7372,
        0x43f , 0x7110,
        // 'phy
        //0x23, 0x1d ,
        //0x44, 0x1d ,

    };

    MXD_U32 regVal=0;

    if ((flag <0)||(flag >2))
        flag =0;

    if (flag ==0)
    {
        mxdDbgInfo("[mxdRfPulse]   ::  update to default config !! \r\n ");
        for (i=0;i<sizeof(rfRegCfgDft)/sizeof(rfRegCfgDft[0])/2;i++)
        {
            rf_reg_write(rfRegCfgDft[2*i], rfRegCfgDft[2*i+1]);
        }
        rf_reg_read(0x43e, &regVal);
        if (regVal==0x4f4e)
        { //deep Ch
            rf_reg_write(0x43f,0x7150);

            rf_reg_write(0x484,0xf4f4);
            mxdDbgInfo("[mxdRfPulse] defaultCfg  ::  deepCh !! \r\n ");
        }
        else if (regVal==0x0f0e)
        { //good ch
            rf_reg_write(0x43f,0x7110);

            rf_reg_write(0x484,0xa4a4);
            mxdDbgInfo("[mxdRfPulse] defaultCfg  ::  goodCh !! \r\n ");
        }

        //XM 2400 default setting
        trout_write_phy_reg(0xff, 0);
        trout_write_phy_reg(0x23 , 0x1c);
        trout_write_phy_reg(0x44 , 0x1c);
        return;
    }
    else if (1==flag)
    {  //aci  config

        //mxdDbgInfo("[mxdRfPulse]  force ACI config!!\r\n");
        mxdDbgInfo("[mxdRfPulse]   ::  update to aci config !! \r\n ");
        for (i=0;i<sizeof(rfRegcfgAci)/sizeof(rfRegcfgAci[0])/2;i++)
        {
            rf_reg_write(rfRegcfgAci[2*i], rfRegcfgAci[2*i+1]);
        }
        trout_write_phy_reg(0xff, 0);
        trout_write_phy_reg(0x23 , 0x1b);
        trout_write_phy_reg(0x44 , 0x1b);
    }
    else if (2==flag)
    { //sense config
        mxdDbgInfo("[mxdRfPulse]   ::  update to sense config !! \r\n ");
        for (i=0;i<sizeof(rfRegcfgSens)/sizeof(rfRegcfgSens[0])/2;i++)
        {
            rf_reg_write(rfRegcfgSens[2*i], rfRegcfgSens[2*i+1]);
        }
        trout_write_phy_reg(0xff, 0);
        trout_write_phy_reg(0x23 , 0x1d);
        trout_write_phy_reg(0x44 , 0x1d);
    }



}
void castorHostBtTxInit(void)
{
    int i=0;


    MXD_U32 regMap[]=
    {
        0x00bd,0x0002,  // Host mode, BTTX
        0x00bF,0x0000,  // Set gain from rbus
        0x0205,0x0907,  // Enable TX DAC, TX Filter, PGA, LOFT DAC
        0x0206,0x0016,  // Set BT path, Enable TX Lobuf, BTTX Enable
        0x0207,0x0119,  // Enable Modulator, PA, PDET
        0x021F,0x0103,  // Enable BT LO mixer
        0x0208,0x022F,  // Enable PA PAD AUX
        0x0213,0x000f,  // Disable negm, tuning gm output cap
        //0x0211,0x053d,  // WF TX pga: 4dB; TxGm 0dB    0331
        0x021a,0x8080,  // WF LOFT DAC config
        0x00c8,0x280, //0x3ff->280 gdy@0911  0200  // 0dB scale
    };


    for (i=0;i<sizeof(regMap)/sizeof(regMap[0])/2;i++)
    {
        rf_reg_write(regMap[2*i], regMap[2*i+1]);
    }
    mxdDbgInfo("[mxd test]castorHostBtTxInit done!");
}


void castorHostBtRxInit(void)
{
    int i=0;
    MXD_U32 regMap[]=
    {
        0x00bd,0x0004,// Host mode, BTRX
        0x00bF,0x0000,// Set gain from rbus
        0x0205,0x9cc0,// Enable BT SWBias, IF, ADC, TIA/IF DCOC
        0x0206,0x0005,// Set BT path
        0x0207,0x0146,// Enable LNA, TIA
        0x021F,0x0103,// Set nolap LO, LO mixer enalbe                          0331
    };

    for (i=0;i<sizeof(regMap)/sizeof(regMap[0])/2;i++)
    {
        rf_reg_write(regMap[2*i], regMap[2*i+1]);
    }
    mxdDbgInfo("[mxd test]castorHostBtRxInit done!\r\n");
}




MXD_API_C void getBestDcocIq(MXD_U16 flagMode)
{
    /*indx=0,1,2,3,     btRxTiaI    btRxTiaQ     btRxPgaI     btRxPgaQ
    4,5,6,7      wifiRxTiaI  wifiRxTiaQ   wifiRxPgaI   wifiRxPgaQ
    8,9          fmI,fmQ
    */
    //MXD_U16 AvgNum = 10; // Average Num
    int dcocMode =flagMode;
    int i;
    MXD_U16 dcocCtlMap[]=
    {
        dcocsw_bt_rx_tia_i_bit,       REG_BT_DCOC_TIA_IQ_CTRL,   BIT_I_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_bt_rx_tia_q_bit,       REG_BT_DCOC_TIA_IQ_CTRL,   BIT_Q_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_bt_rx_pga_i_bit,       REG_BT_DCOC_PGA_IQ_CTRL,   BIT_I_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_bt_rx_pga_q_bit,       REG_BT_DCOC_PGA_IQ_CTRL,   BIT_Q_H ,  REG_RFCALI_FM_DCOC_IDAT,
//#if 1     /* */
#ifdef _WIFI_DCOC_TIAPGA
        dcocsw_wifi_rx_tia_i_bit,     REG_WIFI_DCOC_TIA_IQ_CTRL, BIT_I_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_wifi_rx_tia_q_bit,     REG_WIFI_DCOC_TIA_IQ_CTRL, BIT_Q_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_wifi_rx_pga_i_bit,     REG_WIFI_DCOC_PGA_IQ_CTRL, BIT_I_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_wifi_rx_pga_q_bit,     REG_WIFI_DCOC_PGA_IQ_CTRL, BIT_Q_H ,  REG_RFCALI_FM_DCOC_IDAT,
#else   /*0409 test*/
        dcocsw_wifi_rx_pga_i_bit,     REG_WIFI_DCOC_TIA_IQ_CTRL, BIT_I_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_wifi_rx_pga_q_bit,     REG_WIFI_DCOC_TIA_IQ_CTRL, BIT_Q_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_wifi_rx_pga_i_bit,     REG_WIFI_DCOC_PGA_IQ_CTRL, BIT_I_H ,  REG_RFCALI_FM_DCOC_IDAT,
        dcocsw_wifi_rx_pga_q_bit,     REG_WIFI_DCOC_PGA_IQ_CTRL, BIT_Q_H ,  REG_RFCALI_FM_DCOC_IDAT,
#endif

        0x00,                         REG_FM_DCOC_IQ_CTRL, BIT_I_H ,REG_RFCALI_FM_DCOC_IDAT,
        0x00,                         REG_FM_DCOC_IQ_CTRL, BIT_Q_H ,REG_RFCALI_FM_DCOC_QDAT,
    };
    MXD_U16 regSetDcocVal,bitH_DcocVal,regGetDcocVal;
    MXD_U32 fmAdcValI=0;
    MXD_U16 tttdbg=0;
    MXD_U32 sumVal=0;
//    MXD_U32 regValT;
    int j;
    MXD_U32 deltaNow=0, deltaMin=0, delta80=0;
    MXD_U32 minCfg=0;
    mxdDbgInfo2("[getBestDcocIq]-------------------->>> flagMode=0x%04x !!\r\n",flagMode);
    if (flagMode<10)
    {
        // set clbr dcoc mode
        rf_reg_write(dcoc_dcoc_path_switch_reg, dcocCtlMap[dcocMode*_TBL_DCOC_ROW_LEN_+0]);
        // set regDcocVal & high bit posi config
        regSetDcocVal=dcocCtlMap[dcocMode*_TBL_DCOC_ROW_LEN_+1];
        bitH_DcocVal =dcocCtlMap[dcocMode*_TBL_DCOC_ROW_LEN_+2];
        regGetDcocVal= dcocCtlMap[dcocMode*_TBL_DCOC_ROW_LEN_+3];
        if ((flagMode==0)||(flagMode==1))
        {                                   // when clbr tia
            _REGBC(0x205,_BIT7);    //Disable btIf:: PGA &filter!
        }
        if ((flagMode==2)||(flagMode==3))
        {                                   // when clbr pga
            _REGBS(0x205,_BIT7);   //Enable btIf ;;  PGA &filter!
        }
#ifdef _WIFI_DCOC_TIAPGA
        if ((flagMode==4)||(flagMode==5))
        {                                   // when clbr tia
            _REGBSS(0x205, _BIT5, _BIT4, 0x00);   // disable wifi  PGA &filter!
        }
#else
        if ((flagMode==4)||(flagMode==5))
        {                                   // when clbr tia
            _REGBSS(0x205, _BIT5, _BIT4, 0x03);   // Enable wifi  PGA &filter!
        }
#endif
        if ((flagMode==6)||(flagMode==7))
        {                                   // when clbr pga
            _REGBSS(0x205,_BIT5,_BIT4,0x03);   //Enable wifi  PGA &filter!
        }
        deltaMin = 0xffff;
        rf_reg_read(regSetDcocVal,  &minCfg);
        mxdDbgInfo2("\t\t\t!![mxd] deltaMin=%d ,deltaNow= %d   regSetDcocVal reg%x=0x%x\r\n",deltaMin,deltaNow,regSetDcocVal,minCfg);
        for (i=0;i<8;i++)//8 bit search
        {
//           mxdDbgInfo("[read fmAdcValI] n=%d   \r\n",1<<_DCOC_READ_N_);
            // Set the current bit to 1
            _REGBS(regSetDcocVal,bitH_DcocVal-i);
            sumVal=0;
            for (j=0;j<(1<<_DCOC_READ_N_);j++)    // read 8 times for average
            {
#ifdef _MXD_DEBUG_DCOC_CASTOR
                rf_reg_read(ctrl_chip_id_reg,&regValT); //delay 32 clk
                rf_reg_read(ctrl_chip_id_reg,&regValT);

                _REGBS(0x002,_BIT1);
                _REG_CHK(0x03,_BIT0);
                rf_reg_read(regGetDcocVal,&fmAdcValI);
                sumVal+=fmAdcValI;
#ifdef TRACE_DCOC_LOG
                mxdDbgInfo("ZL notice Here!!0x7b fmAdcValIt=0x%08x\t\n",fmAdcValI);
#endif

#endif
#ifdef _MXD_DEBUG_DCOC_SYS
                // Sleep(1);
                //rf_reg_read(ctrl_chip_id_reg,&regValT); //delay 32 clk
                //rf_reg_read(ctrl_chip_id_reg,&regValT);
                trout_reg_read(0x4074,&fmAdcValI);
                //  mxdDbgInfo("ZL notice Here!! fmAdcValI.H10bit=0x%04x  L10bit=0x%04x\t\n",(fmAdcValI>>10),(fmAdcValI&0x3ff));
#ifdef TRACE_DCOC_LOG
                mxdDbgInfo("\t\t read time[bit%d n=%d] fmAdcValIt=  %6d, 0x%04x,      0x%04x\r\n",i,j,fmAdcValI & 0x3ff,fmAdcValI & 0x3ff,fmAdcValI);
                //mxdDbgInfo("read time[n=%d] fmAdcValIt=0x%08x\r\n",j,fmAdcValI & 0x3ff);
#endif
                //sumVal+=(fmAdcValI>>10)&0x3ff;
                sumVal+=(fmAdcValI&0x3ff);
#endif
            }
            fmAdcValI=(sumVal>>_DCOC_READ_N_);
//           mxdDbgInfo("\t avg(fmAdcValI) =  %6d, 0x%04x, \r\n",fmAdcValI ,fmAdcValI );
            tttdbg=(MXD_U16)fmAdcValI;
#ifdef TRACE_DCOC_LOG
            mxdDbgInfo("\t\t\t!! fmAdcValI=0x%x  tttdbg=0x%4x bitn = %d\t\n",fmAdcValI,tttdbg,bitH_DcocVal-i);
#endif
            if (fmAdcValI>511)
            {
                deltaNow =  fmAdcValI-511;
            }
            else
            {
                deltaNow =511-fmAdcValI;
            }
            if (deltaNow<deltaMin)
            {
                rf_reg_read(regSetDcocVal,  &minCfg);
                mxdDbgInfo2("\t\t\t!![mxd] deltaMin=%d ,deltaNow= %d minCfg=0x%04x \r\n",deltaMin,deltaNow,minCfg);
                deltaMin=deltaNow;
            }
            if ((fmAdcValI>511-1) &&(fmAdcValI<511+1))
            {
#ifdef  _MXD_CLBR_OPTIMIZE
#ifdef TRACE_DCOC_LOG
                mxdDbgInfo("\t Break Now??? now!! fmAdcValI=0x%x  \t\n",fmAdcValI);
#endif
                break;
#endif
            }
            if (fmAdcValI<511)
            {
                _REGBC(regSetDcocVal,bitH_DcocVal-i);
            }
        }

        // if (deltaNow>deltaMin)
        {
            mxdDbgInfo2("\t\t\t!![mxd]zl@0601 write the best config deltaMin=%4d ,deltaNow= %4d minCfg= 0x%04x  flagMode= 0x%04x\r\n",deltaMin,deltaNow,minCfg,flagMode);
            rf_reg_write(regSetDcocVal,  minCfg);
        }
#ifdef TRACE_DCOC_LOG

        mxdDbgInfo("[getBestDcocIQ]======================! flagMode=0x%04x done!!\r\n",flagMode);
#endif
    }
    // close switch
    rf_reg_write(dcoc_dcoc_path_switch_reg, 0);
    return;
}

int FloatToInt(int V_min, int V_max, int NumberAll, int Number)
{
    int Ret = 0;
    Ret = (((2 * V_min * NumberAll) + (2 * Number * (V_max - V_min))  + NumberAll) / (2 *  NumberAll));
    return Ret;
}
// ch16: 2380   ch17:2508
//N= 2508-2380-1=127
//freqCh= 2407+chNo*5

void castorGetWfChDcocAll()
{
    //MXD_U32 tiaNew=0, i=0, chSub2380=0,ChannelMax=127;
    MXD_U32 j=0,ch=0,freqCurMhz=0;
    MXD_U32 tiaH8b2380MHz,tiaL8b2380MHz,tiaH8b2508MHz,tiaL8b2508MHz;

    MXD_U32 valH8b=0, valL8b=0, val=0;

    for (j=0;j<4;j++)
    {
        //gIdx = 28+j;
        // current GainIndex
        tiaH8b2380MHz=(gWfChDcoc[(16-1)*5+j+1]>>8) &0xff;
        tiaL8b2380MHz= gWfChDcoc[(16-1)*5+j+1]    &0xff;
        tiaH8b2508MHz=(gWfChDcoc[(17-1)*5+j+1]>>8) &0xff;
        tiaL8b2508MHz= gWfChDcoc[(17-1)*5+j+1]    &0xff;

        for (ch=1;ch<=17;ch++)//
        {
            if (ch>=1 && ch<= 14)   //ch1-ch14 ;if channel=2380-2508Mhz, chSub2380=0-127
            {
                freqCurMhz= gWfChDcoc[(ch-1)*5];
                valH8b = FloatToInt(tiaH8b2380MHz,tiaH8b2508MHz,(2508-2380-1),freqCurMhz-2380);
                valL8b = FloatToInt(tiaL8b2380MHz,tiaL8b2508MHz,(2508-2380-1),freqCurMhz-2380);
                gWfChDcoc[(ch-1)*5+j+1] = (valH8b << 8) + valL8b;
            }
            //tiaNew = (tiaNew >> 8) + ((tiaNew & 0xff) << 8);
        }
    }
    for (ch=1;ch<=17;ch++)
    {
        freqCurMhz=gWfChDcoc[(ch-1)*5];
        mxdDbgInfo("\t  [mxd] freqMhz = %dMhz :: ",freqCurMhz);
        for (j=0;j<4;j++)
        {
            mxdDbgInfo(" tiaDcocGain[%x]= %4X\t  ",28+j,gWfChDcoc[(ch-1)*5+j+1]);
        }
        mxdDbgInfo("\t\n");
    }

}

void castorGetWfGainFromMap()
{
    MXD_U16 wfRxGainTbl[32]={0,};
    MXD_U32 regVal=0;
    MXD_U16 i=0;

    for (i=0;i<16;i++)
    {
        rf_reg_read(0x0447+i, &regVal);
        wfRxGainTbl[2*i]= regVal&0xff;
    }
}



//MXD_API_C void castorClbrDcocWifi(void)
MXD_API_C void castorClbrDcocWifi(unsigned int flagCh)
{
    int ii;
    MXD_U32 regValT=0;
    int gindx=0;
    MXD_U32 wfPgaDcocI8b=0,wfPgaDcocQ8b=0,wfTiaDcocI8b=0,wfTiaDcocQ8b=0;
    MXD_U32 gainVal=0,gainL16=0;
    // MXD_U32 wifiGainTbl[]={1};
    MXD_U16    wfTia3bit,wfPga4bit,wfGmCtrl2bit,wfGmId4bit ;
    int i=0;

    //  MXD_U32 fmDcocI8b,fmDcocQ8b;
    // MXD_U16  fmTia3bit,fmPga3bit,fmGmSel1bit;
//mxd_wf_dcoc_ch = mxd_set_wf_dcod_ch();
#ifdef _MXD_DEBUG_DCOC_CASTOR
    for (i    = 0;i < 0x42f-0x418+1; i ++)
    {
        rf_reg_write(0x418+i,0);
    }
#endif

    {
        _REGBSS(rf_ldo_ctrl_reg,_BIT10,_BIT8,0x7); //Enable wifi ldo
        _REGBS(rf_lobuffer_cfg_reg,_BIT2);  //enable wifi Lo mixer
        _REGBC(rf_ldo_ctrl_reg,_BIT7);// disable bt lo
        _REGBC(rf_lobuffer_cfg_reg,_BIT1); //disable bt Lo mixer
    }

    castorHostWifiRxInit();
    castorSetFreqWifiCh(flagCh);//(mxd_wf_dcoc_ch);
    _SET_TROUT2_ANT_SEL_DCOC;
    _rfFmAdcOutDrct;
    //disable LNA/SW      0331
#if 0
    _REGBC(rfmux_mode_cfg_ctrl2_reg,_BIT1);
    _REGBC(rfmux_mode_cfg_ctrl2_reg,_BIT2);
#else
    _REGBSS(rfmux_mode_cfg_ctrl2_reg,_BIT2,_BIT1,0x3);
#endif
    //FM ADC Enable
    _REGBS(0x203,_BIT1);
    _REGBS(0x203,_BIT4);
    //ADC/FM LDO enable
    _REGBS(0x201,_BIT1);
    _REGBS(0x201,_BIT4);
#ifdef  _MXD_DEBUG_DCOC_SYS
    _REGBSS(wrxp_oc_sel_reg, _BIT3,_BIT2, 0x1);
    //_REGBSS(wrxp_oc_sel_reg, _BIT3,_BIT2, 0x0); //0409 debug wifi
    _REGBSS(rfmux_mode_cfg_ctrl0_reg,_BIT10,_BIT9,0);
    trout_reg_write(0x4071, 0x05);
#endif
    for (gindx = 0;gindx < 32; gindx ++)
//    for( gindx = 31;gindx < 32; gindx ++)
    {
        //gainVal = wifiGainTbl[gindx];
        rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+gindx*4+0,&gainL16);
        //  rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+gindx*4+1,&gainM16);
        //  rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+gindx*4+2,&gainH16);
        if (1) //(gainVal!=gainL16)
        {
            gainVal= gainL16;

            //zl@@
            wfTia3bit =    _BSG(gainVal,_BIT2,_BIT0);
            wfPga4bit =    _BSG(gainVal,_BIT6,_BIT3); // gainVal.bit6:3;
            wfGmCtrl2bit = _BSG(gainVal,_BIT8,_BIT7); // gainVal.bit8:7;
            wfGmId4bit =   _BSG(gainVal,_BIT12,_BIT9); // gainVal.bit12:9;
            if (gindx<GINDX_WF_LNA_CHANGE_M)//LNA_SEL_0: [0~0x10]   LNA_SEL_3: [0x11~1f]
            {
                _REGBSS(rfmux_wb_rx_gain_reg,_BIT2,_BIT0,LNA_SEL_0);
            }
            /*else if(gindx<GINDX_WF_LNA_CHANGE_3)
            {
                _REGBSS(rfmux_wb_rx_gain_reg,_BIT2,_BIT0,LNA_SEL_2);
            }*/
            else //if(gindx<GINDX_WF_LNA_CHANGE_3)
            {
                _REGBSS(rfmux_wb_rx_gain_reg,_BIT2,_BIT0,LNA_SEL_3);
            }

            _REGBSS(rfmux_wb_rx_gain_reg,_BIT11,_BIT9,wfTia3bit);
            _REGBSS(rfmux_wb_rx_gain_reg,_BIT15,_BIT12,wfPga4bit);
            _REGBSS(rfmux_wb_rx_gm_reg,_BIT3,_BIT0,wfGmId4bit);
            _REGBSS(rfmux_wb_rx_gm_reg,_BIT9,_BIT8,wfGmCtrl2bit);
#ifdef TRACE_DCOC_LOG
            mxdDbgInfo("[mxd] Wifi DCOC clbr::wfTia3bit=0x%x wfPga4bit=0x%x wfGmId4bit=0x%x wfGmCtrl2bit=%x !!  \r\n",wfTia3bit,wfPga4bit,wfGmId4bit,wfGmCtrl2bit);
#endif
            /*
                      indx=0,1,2,3,     btRxTiaI    btRxTiaQ     btRxPgaI     btRxPgaQ
                      4,5,6,7      wifiRxTiaI  wifiRxTiaQ   wifiRxPgaI   wifiRxPgaQ
            */
            rf_reg_write(REG_WIFI_DCOC_PGA_IQ_CTRL, 0x8080);
            rf_reg_write(REG_WIFI_DCOC_TIA_IQ_CTRL, 0x8080);

            _REGBC(0x205,_BIT9);            //Disable wifi adc

            for (ii = 4; ii<8 ; ii++)// wifiRx: tia_i,tia_q,pga_q,pga_i
            {
#ifdef _MXD_WF_DCOC_INSERT_
                if ((gindx>27)&&(gindx < 32))//1c,1d,1e,1f
                {
                    if (ii>5) break;
                }
#endif
                getBestDcocIq(ii);  // DCOC cal

            }
            _REGBS(0x205,_BIT9);            //Enable wifi adc

            //from PGA dcocIQ to table
            rf_reg_read(0x218, &regValT);
            wfPgaDcocI8b= _BSG(regValT,15,8);
            wfPgaDcocQ8b= _BSG(regValT,7,0);

            //from TIA dcocIQ to table
            rf_reg_read(0x219, &regValT);
            wfTiaDcocI8b=  _BSG(regValT,15,8);
            wfTiaDcocQ8b=  _BSG(regValT,7,0);
        }
        else
        {
#ifdef TRACR_RF_LOG
            mxdDbgInfo("[mxd] Wifi DCOC clbr SameGain  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
#endif
        }
        // save to gain table
//       mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
        rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+gindx* 4+1,(wfPgaDcocQ8b<<8)|wfPgaDcocI8b);
//       mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!   TIA_QI:0x%04x  Write to reg_0x%x in wifiTbl  \r\n",gindx,(wfTiaDcocQ8b<<8)|wfTiaDcocI8b,WIFI_RX_GAIN_TBL_START_ADDR+gindx* 4+2);
        rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+gindx* 4+2,(wfTiaDcocQ8b<<8)|wfTiaDcocI8b);
#if 1 //def TRACR_RF_LOG
        mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
        mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!   TIA_QI:0x%04x PGA_QI 0x%04x   \r\n",gindx,(wfTiaDcocQ8b<<8)|wfTiaDcocI8b,(wfPgaDcocQ8b<<8)|wfPgaDcocI8b);
        mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!   PGA_QI 0x%04x   Write to reg_0x%x in wifiTbl  \r\n",gindx,(wfPgaDcocQ8b<<8)|wfPgaDcocI8b,WIFI_RX_GAIN_TBL_START_ADDR+gindx* 4+1);
#endif
    }
    mxdDbgInfo("[mxd] Wifi DCOC calibration done !!\r\n");

#ifdef _MXD_WF_DCOC_INSERT_
//if( getDcocflag )
    {
//wyp
        if (flagCh==16)
        {
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+31* 4+2,&gWfChDcoc[(16-1)*5+4]);//ch16 2308MHz gIdx = 0x1f
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+30* 4+2,&gWfChDcoc[(16-1)*5+3]);//2308MHz gIdx = 0x1e
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+29* 4+2,&gWfChDcoc[(16-1)*5+2]);//2308MHz gIdx = 0x1d
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+28* 4+2,&gWfChDcoc[(16-1)*5+1]);//2308MHz gIdx = 0x1c


            castorSetFreqWifiCh(flagCh+1);//(mxd_wf_dcoc_ch);
            for (gindx = 28;gindx < 32; gindx ++)
            {
                rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+gindx*4+0,&gainL16);
                {
                    gainVal= gainL16;

                    //zl@@
                    wfTia3bit =    _BSG(gainVal,_BIT2,_BIT0);
                    wfPga4bit =    _BSG(gainVal,_BIT6,_BIT3); // gainVal.bit6:3;
                    wfGmCtrl2bit = _BSG(gainVal,_BIT8,_BIT7); // gainVal.bit8:7;
                    wfGmId4bit =   _BSG(gainVal,_BIT12,_BIT9); // gainVal.bit12:9;
                    if (gindx<GINDX_WF_LNA_CHANGE_M)//LNA_SEL_0: [0~e]   LNA_SEL_3: [f~1f]
                    {
                        _REGBSS(rfmux_wb_rx_gain_reg,_BIT2,_BIT0,LNA_SEL_0);
                    }
                    /*else if(gindx<GINDX_WF_LNA_CHANGE_3)
                    {
                        _REGBSS(rfmux_wb_rx_gain_reg,_BIT2,_BIT0,LNA_SEL_2);
                    }*/
                    else //if(gindx<GINDX_WF_LNA_CHANGE_3)
                    {
                        _REGBSS(rfmux_wb_rx_gain_reg,_BIT2,_BIT0,LNA_SEL_3);
                    }

                    _REGBSS(rfmux_wb_rx_gain_reg,_BIT11,_BIT9,wfTia3bit);
                    _REGBSS(rfmux_wb_rx_gain_reg,_BIT15,_BIT12,wfPga4bit);
                    _REGBSS(rfmux_wb_rx_gm_reg,_BIT3,_BIT0,wfGmId4bit);
                    _REGBSS(rfmux_wb_rx_gm_reg,_BIT9,_BIT8,wfGmCtrl2bit);
#ifdef TRACE_DCOC_LOG
                    mxdDbgInfo("[mxd] Wifi DCOC clbr::wfTia3bit=0x%x wfPga4bit=0x%x wfGmId4bit=0x%x wfGmCtrl2bit=%x !!  \r\n",wfTia3bit,wfPga4bit,wfGmId4bit,wfGmCtrl2bit);
#endif
                    /*
                              indx=0,1,2,3,     btRxTiaI    btRxTiaQ     btRxPgaI     btRxPgaQ
                              4,5,6,7      wifiRxTiaI  wifiRxTiaQ   wifiRxPgaI   wifiRxPgaQ     */
                    rf_reg_write(REG_WIFI_DCOC_PGA_IQ_CTRL, 0x8080);
                    rf_reg_write(REG_WIFI_DCOC_TIA_IQ_CTRL, 0x8080);

                    _REGBC(0x205,_BIT9);            //Disable wifi adc

                    for (ii = 4; ii<8 ; ii++)// wifiRx: tia_i,tia_q,pga_q,pga_i
                    {
#ifdef _MXD_WF_DCOC_INSERT_
                        if ((gindx>27)&&(gindx < 32))//1c,1d,1e,1f
                        {
                            if (ii>5) break;
                        }
#endif
                        getBestDcocIq(ii);  // DCOC cal
                    }
                    _REGBS(0x205,_BIT9);            //Enable wifi adc

                    //from PGA dcocIQ to table
                    rf_reg_read(0x218, &regValT);
                    wfPgaDcocI8b= _BSG(regValT,15,8);
                    wfPgaDcocQ8b= _BSG(regValT,7,0);

                    //from TIA dcocIQ to table
                    rf_reg_read(0x219, &regValT);
                    wfTiaDcocI8b=  _BSG(regValT,15,8);
                    wfTiaDcocQ8b=  _BSG(regValT,7,0);
                }
          
                // save to gain table
                rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+gindx* 4+1,(wfPgaDcocQ8b<<8)|wfPgaDcocI8b);
                rf_reg_write(WIFI_RX_GAIN_TBL_START_ADDR+gindx* 4+2,(wfTiaDcocQ8b<<8)|wfTiaDcocI8b);
//rf_mem_read

#if 1 //def TRACR_RF_LOG
                mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
                mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!   TIA_QI:0x%04x PGA_QI 0x%04x   \r\n",gindx,(wfTiaDcocQ8b<<8)|wfTiaDcocI8b,(wfPgaDcocQ8b<<8)|wfPgaDcocI8b);
                mxdDbgInfo("[mxd] Wifi DCOC clbr done  gindex = %d !!   PGA_QI 0x%04x   Write to reg_0x%x in wifiTbl  \r\n",gindx,(wfPgaDcocQ8b<<8)|wfPgaDcocI8b,WIFI_RX_GAIN_TBL_START_ADDR+gindx* 4+1);
#endif
            }
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+31* 4+2,&gWfChDcoc[(17-1)*5+4]); //ch17 2508MHz
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+30* 4+2,&gWfChDcoc[(17-1)*5+3]);
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+29* 4+2,&gWfChDcoc[(17-1)*5+2]);
            rf_mem_read(WIFI_RX_GAIN_TBL_START_ADDR+28* 4+2,&gWfChDcoc[(17-1)*5+1]);

        }

        castorGetWfChDcocAll();
    }
#endif

#ifdef  _MXD_DEBUG_DCOC_SYS
    trout_reg_write(0x4071, 0x00);//read adc val from 4074;
#endif
    //Zl@@???
    _REGBSS(wrxp_oc_sel_reg, _BIT3,_BIT2, 0x0);
    //enable LNA/SW      0331
    _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT1);
    _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT2);
    //rf_reg_write(dcoc_dcoc_path_switch_reg, 0);
    _SET_TROUT2_ANT_SEL_NORMAL;
}

MXD_API_C void castorClbrDcocBt(void)
{
    int ii;
    MXD_U32 regValT=0;
    int gindx=0;
    MXD_U16 btTia3bit,btPga3bit,btTiaRes1bit,btGmCtrl2bit,btGmId4bit;
    MXD_U32 btPgaDcocI8b=0,btPgaDcocQ8b=0,btTiaDcocI8b=0,btTiaDcocQ8b=0;
    MXD_U32 gainVal=0,gainL16=0;
#ifdef _MXD_DEBUG_DCOC_CASTOR
    int i=0;
    for (i    = 0;i < 0x42f-0x418+1; i ++)
    {
        rf_reg_write(0x418+i,0);
    }
#endif
    {
        _REGBS(rf_ldo_ctrl_reg,_BIT10);
        _REGBSS(rf_ldo_ctrl_reg,_BIT8,_BIT7,0x3);
        _REGBS(rf_lobuffer_cfg_reg,_BIT1);

        _REGBC(rf_ldo_ctrl_reg,_BIT9);  //disable wifi lo
        _REGBC(rf_lobuffer_cfg_reg,_BIT2);  //disable wifi loMixer
    }
    _SET_TROUT2_ANT_SEL_DCOC;
    castorHostBtRxInit();
    castorSetFreqMhzBt(2412);

    // rfmux_MODE_CFG_CTRL2_reg
    _rfFmAdcOutDrct;

    //disable LNA/SW      //0331
    //_REGBC(rfmux_mode_cfg_ctrl2_reg,_BIT1);
    //_REGBC(rfmux_mode_cfg_ctrl2_reg,_BIT2);
    //Enable LNA/SW 
    _REGBSS(rfmux_mode_cfg_ctrl2_reg,_BIT2,_BIT1,0x3);

    //FM ADC Enable
    _REGBS(0x203,_BIT1);
    _REGBS(0x203,_BIT4);
    //ADC/FM LDO enable
    _REGBS(0x201,_BIT1);
    _REGBS(0x201,_BIT4);
    _REGBC(0x200,_BIT2);
#ifdef  _MXD_DEBUG_DCOC_SYS
    rf_reg_write(0x31, 0x4);
    trout_reg_write(0x4071, 0x05);
    _REGBSS(rfmux_mode_cfg_ctrl0_reg,_BIT10,_BIT9,0);
#endif
    for (gindx = 0;gindx < 32; gindx ++)
    {

        rf_mem_read(BT_RX_GAIN_TBL_START_ADDR+gindx*4+0,&gainL16);
        if (gainVal!= gainL16)
        {
            gainVal= gainL16;

            //zl@@
            btTia3bit =    _BSG(gainVal,_BIT2,_BIT0);
            btPga3bit =    _BSG(gainVal,_BIT5,_BIT3); // gainVal.bit6:3;
            btTiaRes1bit = (gainVal>>_BIT6) & 0x01;
            btGmCtrl2bit = _BSG(gainVal,_BIT8,_BIT7); // gainVal.bit8:7;
            btGmId4bit =   _BSG(gainVal,_BIT12,_BIT9); // gainVal.bit12:9;

            _REGBSS(rfmux_wb_rx_gain_reg,5,3,btTia3bit);
            _REGBSS(rfmux_wb_rx_gain_reg,8,6,btPga3bit);
            _REGBSS(rfmux_wb_rx_gm_reg,7,4,btGmId4bit);
            _REGBSS(rfmux_wb_rx_gm_reg,11,10,btGmCtrl2bit);
#ifdef TRACE_DCOC_LOG
            mxdDbgInfo("[mxd] BT DCOC clbr::btTia3bit=0x%x btPga3bit=0x%x btGmId4bit=0x%x btGmCtrl2bit=%x btTiaRes1bit=%x!!  \r\n",btTia3bit,btPga3bit,btGmId4bit,btGmCtrl2bit,btTiaRes1bit);
#endif
            if (btTiaRes1bit==0)
            {
                _REGBC(0x212,_BIT5);
            }
            else
            {
                _REGBS(0x212,_BIT5);
            }

            rf_reg_write(REG_BT_DCOC_PGA_IQ_CTRL, 0x8080);
            rf_reg_write(REG_BT_DCOC_TIA_IQ_CTRL, 0x8080);

            _REGBC(rfmux_mode_cfg_ctrl0_reg,_BIT10);            //Disable bt adc
            for (ii = 0; ii<4 ; ii++)// btRx: tia_i,tia_q,pga_q,pga_i
            {
                getBestDcocIq(ii);  // DCOC cal
            }
            _REGBS(rfmux_mode_cfg_ctrl0_reg,_BIT10);            //Disable bt adc
            //get PGA dcocIQ to table
            rf_reg_read(REG_BT_DCOC_PGA_IQ_CTRL, &regValT);
            btPgaDcocI8b= _BSG(regValT,15,8);
            btPgaDcocQ8b= _BSG(regValT,7,0);

            //get TIA dcocIQ to table
            rf_reg_read(REG_BT_DCOC_TIA_IQ_CTRL, &regValT);
            btTiaDcocI8b=  _BSG(regValT,15,8);
            btTiaDcocQ8b=  _BSG(regValT,7,0);
        }
        else
        {
#ifdef TRACE_DCOC_LOG
            mxdDbgInfo("[mxd] BT DCOC clbr SameGain  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
#endif
        }

        // write gain table
        // mxdDbgInfo("[mxd] BT DCOC clbr done  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
        rf_reg_write(BT_RX_GAIN_TBL_START_ADDR+gindx*4+1,(btPgaDcocQ8b<<8)|btPgaDcocI8b);
        // mxdDbgInfo("[mxd] BT DCOC clbr done  gindex = %d !!   PGA_QI:0x%04x  Write to reg_%x in bttbl\r\n",gindx,(btPgaDcocQ8b<<8)|btPgaDcocI8b,(BT_RX_GAIN_TBL_START_ADDR+gindx*4+1));
        rf_reg_write(BT_RX_GAIN_TBL_START_ADDR+gindx*4+2,(btTiaDcocQ8b<<8)|btTiaDcocI8b);
#ifndef TRACR_RF_LOG
        mxdDbgInfo("[mxd] BT DCOC clbr done  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
        mxdDbgInfo("[mxd] BT DCOC clbr done  gindex = %d !!   TIA_QI:0x%04x PGA_QI 0x%04x   \r\n",gindx,(btTiaDcocQ8b<<8)|btTiaDcocI8b,(btPgaDcocQ8b<<8)|btPgaDcocI8b);
#endif
    }
    mxdDbgInfo("[mxd] BT DCOC calibration done !!\r\n");
#ifdef  _MXD_DEBUG_DCOC_SYS
    trout_reg_write(0x4071, 0x00);//read adc val from 4074;
#endif
    //Zl@@???
    _REGBSS(0x31, _BIT3,_BIT2, 0x0);
    _REGBS(0x200,_BIT2);
    rf_reg_write(dcoc_dcoc_path_switch_reg, 0);
    //enable LNA/SW      0331
    _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT1);
    _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT2);
    _SET_TROUT2_ANT_SEL_NORMAL;
//     _REGBSS(0x04,7,4,0x00);//26Mhz force disable

}
MXD_API_C void castorClbrDcocFm(void)
{
    int ii;
    MXD_U32 regValT=0;
    int gindx=0;
    MXD_U32 gainVal=0,gainL16=0;
    MXD_U32 fmDcocI8b,fmDcocQ8b;
    MXD_U16  fmTia3bit,fmPga3bit,fmGmSel1bit;
#ifdef _MXD_DEBUG_DCOC_CASTOR
    int i=0;
    for (i    = 0;i < 0x42f-0x418+1; i ++)
    {
        rf_reg_write(0x418+i,0);
    }
#endif
    {
        //Enable FM
        _REGBSS(rf_ldo_ctrl_reg,_BIT6,_BIT4,0x7);
    }
    mxdDbgInfo("[mxd] FM DCOC calibration start... !!\r\n");
    castorFmInit();
    castorSetFreqHKhzFm(910);
    _rfFmAdcOutDrct;
    _REGBS(0x203,_BIT1);
    _REGBS(0x203,_BIT4);
    _REGBS(rf_ldo_ctrl_reg,_BIT1);
#ifdef  _MXD_DEBUG_DCOC_SYS
    trout_reg_write(0x4071, 0x05);
#endif
    for (gindx = 0;gindx < 20; gindx ++)
    {
        rf_mem_read(FM_RX_GAIN_TBL_START_ADDR+gindx*2+0,&gainL16);
        if ((gainVal!=gainL16)||(gainL16==0))
        {
            gainVal= gainL16;
            fmTia3bit =    _BSG(gainVal,_BIT4,_BIT2);
            fmPga3bit =    _BSG(gainVal,_BIT7,_BIT5);
            fmGmSel1bit= (gainVal&0x01)>>_BIT0;
            rf_reg_write(rfmux_fm_gain_reg,(fmGmSel1bit <<_BIT6)|(fmTia3bit<<_BIT3)| (fmGmSel1bit<<_BIT0));
            rf_reg_write(rfmux_fm_rx_dcoc_ctrl_reg, 0x8080);
            for (ii = 8; ii<10 ; ii++)// : tia_i,tia_q,pga_q,pga_i
            {
                getBestDcocIq(ii);  // DCOC cal
            }
            rf_reg_read(rfmux_fm_rx_dcoc_ctrl_reg, &regValT);
            fmDcocI8b= _BSG(regValT,15,8);
            fmDcocQ8b= _BSG(regValT,7,0);
            rf_reg_write(FM_RX_GAIN_TBL_START_ADDR+gindx*2+1,(fmDcocQ8b<<8)|fmDcocI8b);
        }
        else
        {
            mxdDbgInfo("[mxd] FM DCOC clbr SameGain  gindex = %d !!  gainL16 = 0x%04x \r\n",gindx,gainL16);
            rf_reg_write(FM_RX_GAIN_TBL_START_ADDR+gindx*2+1,(fmDcocQ8b<<8)|fmDcocI8b);
        }
    }
    mxdDbgInfo("[mxd] FM DCOC calibration done !!\r\n");
#ifdef  _MXD_DEBUG_DCOC_SYS
    trout_reg_write(0x4071, 0x00);//read adc val from 4074;
#endif
    _REGBSS(wrxp_oc_sel_reg, _BIT3,_BIT2, 0x0);
    _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT1);
    _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT2);
}

void castorHostBtTxRxInit(void)
{
    int i=0;
    MXD_U32 regMap[]= // set contreol source
    {
        0x00c7, 0x0000,  // sprd mode
        0x021F, 0x0103,  // Set nolap LO, 0xLO mixer enalbe
        0x0209, 0x00c1,  // Config LNA SW bias, 0xCTRL_VB2
        0x020a, 0xa4a4,  // Config LNA VB1_Main, 0xVB1_AUX
        0x020F, 0x0edd,  // Enable WF gmctrl 11, 0xhgm 11, 0xhvsat 01
        0x0210, 0xAD0A,  // LNA 0dB, 0xTIA 32dB, 0xPGA 0dB -38dBm
        0x0216, 0x8080,  // TIA DCOC Config
        0x0217, 0x8080,  // PGA DCOC Config
        0x00bF, 0x0000,  // Set gain from rbus
        0x0208, 0x022F,  // Enable PA config
        0x0213, 0x0004,  // Disable negm, 0xtuning gm output cap
        0x0211, 0x003d,  // WF TX pga: 4dB; TxGm 0dB     //0331
        0x021b, 0x8080,  // WF LOFT DAC config
        0x00bd, 0x0018,  // sprd mode
        //00be 029c b & sprd mode definiton 10/00 bt t/r
        // PA configA
        0x020b, 0xa15a,
        0x020c, 0x5847,  // 7f67
        0x020d, 0xbc7f,  // bc4f
        0x020e, 0x807c,  // 5f00
    };
    for (i=0;i < sizeof(regMap)/sizeof(regMap[0])/2;i++)
    {
        rf_reg_write(regMap[2*i],regMap[2*i + 1]);

    }
    mxdDbgInfo("\r\r castorHostBtTxRxInit:: regMap size = %d \r\n",i*2);
    //BtRx: rfreg0xbd = 0c      :: 01 100
    //BtTx: rfregbd0xbd = 0a    :: 01 010
}


void castorClbrTximbBt(void)
{

    MXD_U16      k=128,i2 = 0,i3 = 0,ii = 0,ib = 0,minMag = 0,Rng = 0;
    MXD_U32    i=0,Mag = 0,minVal = 0,flg = 0,step = 0,Rreg = 0,kk = 0,rval=0;
    MXD_U16 ampTbl[] =
    {
        0x32D ,0x340 ,0x354  ,0x368 ,0x37C ,0x391 ,0x3A6 ,
        0x3BC ,0x3D2 ,0x3E9 ,0x400 ,0x418 ,0x430 ,0x449  ,
        0x463 ,0x47D ,0x498 ,0x4B3 ,0x4CF ,0x4EC ,0x509
    };//[21]
    MXD_U16 Phi_tbl[] =
    {
        0xf1a, 0xf1e, 0xf22, 0xf26, 0xf2a, 0xf2e, 0xf32,//@0826
        0xf36, 0xf3a, 0xf3e, 0xf42, 0xf46, 0xf4a, 0xf4e,
        0xf52, 0xf56, 0xf5a, 0xf5e, 0xf62, 0xf66, 0xf6a,//
        0xf6e, 0xf72 ,0xf76 , 0xf7a, 0xf7e , 0xf82,0xf86,
        0xf8a, 0xf8e ,0xf92 , 0xf96, 0xf9a , 0xf9e,0xfa2, //new tbl gdy@0809
        0xFa7 ,0xfaa ,0xfae  ,0xfb1 ,0xfb5 ,0xfb9 ,0xfbc ,
        0xfc0 ,0xfc3 ,0xfc7 ,0xfca ,0xfce ,0xfd2 ,0xfd5  ,
        0xfd9 ,0xfdc ,0xfe0 ,0xfe3 ,0xfe7 ,0xfeb ,0xfee  ,
        0xff2 ,0xff5 ,0xff9 ,0xffc  ,0x0 ,0x4  ,0x7 ,0xb ,
        0xe ,0x12 ,0x15  ,0x19  ,0x1d  ,0x20 ,0x24,0x27  ,
        0x2b  ,0x2e  ,0x32  ,0x36  ,0x39  ,0x3d  ,0x40   ,
        0x44  ,0x47  ,0x4b  ,0x4f  ,0x52  ,0x56  ,0x59
    };//[51]


    // SetSTD freq
    rf_reg_write(frxp_frxp_rcstd_cos_reg,0x3f38);  // 3f88
    rf_reg_write(frxp_frxp_rcstd_sin_reg,0x9f9);                   // 7bd   sunzl err

    rf_reg_write(txp_txp_bttx_txe2_reg, 0x0);
    rf_reg_write(txp_txp_bttx_txe1_reg,0x400);

    _REGBS(txp_txp_ctrl_reg, _BIT1);         //  BT
    _REGBSS(txp_txp_ctrl_reg,_BIT12,_BIT11,0x00);     //txe1/e2 sel::     00:bttx    01:wftx  1?:sprdPinMode                             //

    // Amp
    rf_reg_write(txp_txp_bttx_txe2_reg, 0x0);
    minMag = 0xffff;
    for (kk = 0; kk <sizeof(ampTbl)/sizeof(ampTbl[0]);kk++)
    {
        //Sleep(500);
        rf_reg_write(txp_txp_bttx_txe1_reg,ampTbl[kk]);         //?: ?FPGA RBUS, RF IPtest RBUS
        rf_reg_write(frxp_rcstd_bgn,0x1);                                       // rf_reg_write(0x20, 1);  // start STD
        do
        {
            MxdSleep(MXD_SLEEP_MS);
            rval=0;
            rf_reg_read(ctrl_tuner_status_reg,&rval);
        }
        while ((rval & 0x20)==0);
        rf_reg_read(frxp_frxp_rcstd_mag_status_reg,&Mag);
        //if( (minMag==0)||( Mag < minMag))
        if ((Mag < minMag))
        {
            minMag = Mag;
            minVal = ampTbl[kk];
            // mxdDbgInfo("%s:%d\twrite : txp_txp_bttx_txe1_reg = %x rval=%x \n",__FUNCTION__,__LINE__,minVal, rval);
        }
    }
//#ifdef TRACR_RF_LOG
    mxdDbgInfo("%s:%d\t write : txp_txp_bttx_txe1_reg(0x%x) = %x\n",__FUNCTION__,__LINE__,txp_txp_bttx_txe1_reg,minVal);
//#endif
    rf_reg_write(txp_txp_bttx_txe1_reg,minVal); //Amp?

    // Phi
    minMag = 0xffff;
    for (kk = 0; kk <sizeof(Phi_tbl)/sizeof(Phi_tbl[0]); kk++)
    {

        rf_reg_write(txp_txp_bttx_txe2_reg, Phi_tbl[kk]);       //?:  FPGA RBUS, RF IPtest???
        rf_reg_write(frxp_rcstd_bgn,0x1);                                       // rf_reg_write(0x20, 1);  // start STD
        do
        {
            MxdSleep(MXD_SLEEP_MS);
            rval=0;
            rf_reg_read(ctrl_tuner_status_reg,&rval);
        }
        while (!(rval & 0x20));
        rf_reg_read(frxp_frxp_rcstd_mag_status_reg,&Mag);

        //     if ( (minMag==0)|| Mag < minMag)
        if (Mag < minMag)
        {
            minMag = Mag;
            minVal = Phi_tbl[kk];
#ifdef TRACE_DCOC_LOG
            mxdDbgInfo("write : txp_txp_bttx_txe2_reg = %x\n",minVal);
#endif
        }
#ifdef TRACE_DCOC_LOG
        mxdDbgInfo("write : txp_txp_bttx_txe2_reg = %x Phi_tbl[%d]\n",minVal,Phi_tbl[kk]);
#endif
    }
    mxdDbgInfo("[Func::%s]   line[%d]\twrite : 0x%04x = %x\n",__FUNCTION__,__LINE__,txp_txp_bttx_txe2_reg,minVal);
    rf_reg_write(txp_txp_bttx_txe2_reg,minVal); // ?Pphi
}

void castorClbrTximbWifi(void)
{

    MXD_U16      k=128,i2 = 0,i3 = 0,ii = 0,ib = 0,minMag = 0,Rng = 0;
    MXD_U32    i=0,Mag = 0,minVal = 0,flg = 0,step = 0,reg = 0,kk = 0,rval=0;
    MXD_U16 ampTbl[] =
    {
        0x32D ,0x340 ,0x354  ,0x368 ,0x37C ,0x391 ,0x3A6 ,
        0x3BC ,0x3D2 ,0x3E9 ,0x400 ,0x418 ,0x430 ,0x449  ,
        0x463 ,0x47D ,0x498 ,0x4B3 ,0x4CF ,0x4EC ,0x509
    };//[21]
    MXD_U16 phiTbl[] =
    {
        0xf1a, 0xf1e, 0xf22, 0xf26, 0xf2a, 0xf2e, 0xf32,//@wyp0826
        0xf36, 0xf3a, 0xf3e, 0xf42, 0xf46, 0xf4a, 0xf4e,
        0xf52, 0xf56, 0xf5a, 0xf5e, 0xf62, 0xf66, 0xf6a,//
        0xf6e, 0xf72 ,0xf76 , 0xf7a, 0xf7e , 0xf82,0xf86,
        0xf8a, 0xf8e ,0xf92 , 0xf96, 0xf9a , 0xf9e,0xfa2, //new tbl gdy@0809
        0xFa7 ,0xfaa ,0xfae  ,0xfb1 ,0xfb5 ,0xfb9 ,0xfbc ,
        0xfc0 ,0xfc3 ,0xfc7 ,0xfca ,0xfce ,0xfd2 ,0xfd5  ,
        0xfd9 ,0xfdc ,0xfe0 ,0xfe3 ,0xfe7 ,0xfeb ,0xfee  ,
        0xff2 ,0xff5 ,0xff9 ,0xffc  ,0x0 ,0x4  ,0x7 ,0xb ,
        0xe ,0x12 ,0x15  ,0x19  ,0x1d  ,0x20 ,0x24,0x27  ,
        0x2b  ,0x2e  ,0x32  ,0x36  ,0x39  ,0x3d  ,0x40   ,
        0x44  ,0x47  ,0x4b  ,0x4f  ,0x52  ,0x56  ,0x59
    };//[51]

    rf_reg_write(txp_txp_wftx_txe2_reg, 0x0);
    rf_reg_write(txp_txp_wftx_txe1_reg, 0x400);
    // SetSTD freq
    rf_reg_write(frxp_frxp_rcstd_cos_reg,0x3f38);  // 3f88
    rf_reg_write(frxp_frxp_rcstd_sin_reg,0x9f9);                   // 7bd   sunzl err

    _REGBS(txp_txp_ctrl_reg, _BIT1);                                  //
    _REGBSS(txp_txp_ctrl_reg,_BIT12,_BIT11,0x01);     //txe1/e2 sel::     00:bttx    01:wftx  1?:sprdPinMode

    // Amp
    rf_reg_write(txp_txp_wftx_txe2_reg, 0x0);
    minMag = 0xffff;
    for (kk = 0; kk <sizeof(ampTbl)/sizeof(ampTbl[0]);kk++)
    {
        //Sleep(200);
        rf_reg_write(txp_txp_wftx_txe1_reg,ampTbl[kk]);         //?: ?FPGA RBUS, RF IPtest???RBUS
        rf_reg_write(frxp_rcstd_bgn,0x1);                                       // rf_reg_write(0x20, 1);  // start STD
        do
        {
            MxdSleep(MXD_SLEEP_MS);
            rval=0;
            rf_reg_read(ctrl_tuner_status_reg,&rval);
        }
        while ((rval & 0x20)==0);
        rf_reg_read(frxp_frxp_rcstd_mag_status_reg,&Mag);
        //if( (minMag==0)||( Mag < minMag))
        if ((Mag < minMag))
        {
            minMag = Mag;
            minVal = ampTbl[kk];
//#ifdef TRACE_DCOC_LOG
            _wfTxImbMxdDbgInfo("[Func::%s]   line[%d]\twrite : 0x%04x = %x\n",__FUNCTION__,__LINE__,txp_txp_wftx_txe1_reg,minVal);
//#endif
        }
    }
//#ifdef TRACR_RF_LOG
    mxdDbgInfo("[Func::%s]   line[%d]\twrite : 0x%04x = %x \r\n",__FUNCTION__,__LINE__,txp_txp_wftx_txe1_reg,minVal);
//#endif
    rf_reg_write(txp_txp_wftx_txe1_reg,minVal); // ??Amp?

    // Phi
    minMag = 0xffff;
    for (kk = 0; kk <sizeof(phiTbl)/sizeof(phiTbl[0]); kk++)
    {

        rf_reg_write(txp_txp_wftx_txe2_reg, phiTbl[kk]);       //?:  FPGA RBUS, RF IPtest???
        rf_reg_write(frxp_rcstd_bgn,0x1);                                       // rf_reg_write(0x20, 1);  // start STD
        do
        {
            MxdSleep(MXD_SLEEP_MS);
            rval=0;
            rf_reg_read(ctrl_tuner_status_reg,&rval);
            //Sleep(200);
        }
        while ((rval & 0x20)==0);
        rf_reg_read(frxp_frxp_rcstd_mag_status_reg,&Mag);

//        if ( (minMag==0)|| Mag < minMag)
        if (Mag < minMag)
        {
            minMag = Mag;
            minVal = phiTbl[kk];
//#ifdef TRACE_DCOC_LOG
            _wfTxImbMxdDbgInfo("[Func::%s]   line[%d]\twrite : 0x%04x = %x\n",__FUNCTION__,__LINE__,txp_txp_wftx_txe2_reg,minVal);
//#endif
        }
    }
//#ifdef TRACR_RF_LOG
    mxdDbgInfo("[Func::%s]   line[%d]\twrite : 0x%04x = %x \r\n",__FUNCTION__,__LINE__,txp_txp_wftx_txe2_reg,minVal);
//#endif
    rf_reg_write(txp_txp_wftx_txe2_reg,minVal); // Phi
}

void rfClbrReadMag(MXD_U32 * pMag)
{
    MXD_U32 testVal1=0, testValMax=0, testValMin=0xffff,testSum=0,i=0,mag=0;
    testSum=0;
    testValMax=0;
    testValMin=0xffff;
    // get msg avg for
    for (i=0;i<5;i++)
    {
        _REGBC(rfmux_mode_cfg_ctrl2_reg, _BIT3); //tx  disable
        MxdSleep(1);
        _REGBS(rfmux_mode_cfg_ctrl2_reg, _BIT3);    //tx Enable
        rf_reg_write(frxp_rcstd_bgn,0x1);                 // start STD
        _REG_CHK2(ctrl_tuner_status_reg,_BIT5);
        rf_reg_read(frxp_frxp_rcstd_mag_status_reg,&mag);
        if (testValMax<mag)testValMax=mag;//wyp
        if (testValMin>mag)testValMin=mag;
        testSum+=mag;
    }
    testSum=testSum/i; //avg mag
    if (NULL != pMag)  *pMag =testSum;
}

void rfClbrSetWfTxGain(MXD_U32 wfTxGIdx)
{
    MXD_U32     wifiTxPga4bit, txGm3bit ;
     MXD_U32   gainTx,i;
     i = _MXD_POUT_INDEX28_SUBSCRIPT +(wfTxGIdx-_MXD_POUT_INDEX_VOL);//  _MXD_POUT_INDEX28_SUBSCRIPT is gWifiTxGaincfg index28 ;wfTxGIdx-28 is delta
    gainTx = gWifiTxGaincfg[i];
    rf_reg_write(rfmux_wftx_gidx_reg,_MXD_POUT_INDEX_VOL);
    //rf_reg_write(WIFI_TX_GAIN_TBL_START_ADDR+_MXD_POUT_INDEX_VOL*2,gainTx);
    wifiTxPga4bit = _BSG(gainTx,_BIT6,_BIT3);
    txGm3bit = _BSG(gainTx,_BIT2,_BIT0);
    _REGBSS(rfmux_wb_tx_gain_reg, _BIT10,_BIT7, wifiTxPga4bit);
    _REGBSS(rfmux_wb_tx_gain_reg, _BIT2,_BIT0, txGm3bit);
}


//wfTxGIdxDelta>0  Power increase
//wfTxGIdxDelta<0; Power Decrease//
void rfClbrShiftBttxgain(MXD_U32 minIdx, MXD_U32 maxIdx,  MXD_S32 btTxGIdxDelta)
{
    MXD_U32 gainTx=0;
    MXD_S32 i=0;
	MXD_S32 txindexArray[16]={0,};
	MXD_U32 chipId =0;
	 rf_reg_read(ctrl_chip_id_reg, &chipId);
	 _wfTxPoutMxdDbgInfo("[mxd][%s  chip%x]::   gPoutPcbSwtchRecover=%d   gPoutOffsetC7C8=%d  wfTxGIdxDelta=%d\r\n", __FUNCTION__, chipId,gPoutPcbSwtchRecover,gPoutOffsetC7C8,btTxGIdxDelta);	
	if (chipId == _CHIP_ID_C7)
	{	
		btTxGIdxDelta  +=(gPoutPcbSwtchRecover+2)/4;
	}  
	else if (chipId == _CHIP_ID_C8)
	{
		btTxGIdxDelta  += ((gPoutPcbSwtchRecover+gPoutOffsetC7C8+4)+2)/4;  
	}	
    _wfTxPoutMxdDbgInfo("[mxd after clbr autoc7c8][%s  chip%x]::   wfTxGIdxDelta=%d\r\n", __FUNCTION__, chipId,btTxGIdxDelta);	
	_wfTxPoutMxdDbgInfo("[%s BT   Tx] ::before shift  --------btdelta=%d----------\r\n", __FUNCTION__,btTxGIdxDelta);
    for (i=minIdx;i<=maxIdx;i++)
    {
		rf_mem_read(BT_TX_GAIN_TBL_START_ADDR+i,&gainTx);
		_wfTxPoutMxdDbgInfo("[mxd][%s bt TxTbl] ::BT TxIdx%x   %x   %x  \r\n", __FUNCTION__,i,BT_TX_GAIN_TBL_START_ADDR+i,gainTx);

    }
	for (i=0;i<=0xf;i++)
    {
		rf_mem_read(BT_TX_GAIN_TBL_START_ADDR+i,&txindexArray[i]);
    }
	for (i=0;i<=0xf;i++)
    {
		MXD_S32 idx=0;
		idx=i-btTxGIdxDelta;
		if(idx<0){
			idx=0;
		}
		else if(idx>=16)
        {
			idx=15;
		}
		rf_reg_write(BT_TX_GAIN_TBL_START_ADDR+i,txindexArray[idx]);
		_wfTxPoutMxdDbgInfo("[%s]::  readaddr1=%x writeaddr2=%x \t  gaintx =%x  \r\n", __FUNCTION__,BT_TX_GAIN_TBL_START_ADDR+i,BT_TX_GAIN_TBL_START_ADDR+idx, txindexArray[idx]);

        }
	_wfTxPoutMxdDbgInfo("[%s BT   Tx] ::after shift  ------------------\r\n", __FUNCTION__);
	for (i=0;i<=0xf;i++)//(i=minIdx;i<=maxIdx;i++)
        {
		rf_mem_read(BT_TX_GAIN_TBL_START_ADDR+i,&gainTx);
		_wfTxPoutMxdDbgInfo("[%s Bt TxTbl] ::btTxIdx%x   %x   %x  \r\n", __FUNCTION__,i,BT_TX_GAIN_TBL_START_ADDR+(i)*2,gainTx);
	}
}
//wfTxGIdxDelta>0  Power increase
//wfTxGIdxDelta<0; Power Decrease//
#if 0   //wyp
#else
void rfClbrShiftWftxgain(MXD_U32 minIdx, MXD_U32 maxIdx,  MXD_S32 wfTxGIdxDelta)
{
    MXD_U32   wifiTxPga4bit, txGm3bit ;
    MXD_U32 gainTx=0;
    MXD_S32 i=0;
    MXD_S32 txindextemp=0;
    MXD_S32 maskDeltagain = 0;	//11b mask pass delta
    MXD_S32 txindexarray[16]={0,};
    MXD_S32 j=0;
       MXD_U32 chipId=0;

       rf_reg_read(ctrl_chip_id_reg, &chipId);
	_wfTxPoutMxdDbgInfo("[mxd][%s  chip%x]::   gPoutPcbSwtchRecover=%d   gPoutOffsetC7C8=%d  wfTxGIdxDelta=%d\r\n", __FUNCTION__, chipId,gPoutPcbSwtchRecover,gPoutOffsetC7C8,wfTxGIdxDelta);	
	if (chipId == _CHIP_ID_C7)
	{	
		wfTxGIdxDelta  += gPoutPcbSwtchRecover;
	}  
	else if (chipId == _CHIP_ID_C8)
	{
		wfTxGIdxDelta  += gPoutPcbSwtchRecover+gPoutOffsetC7C8+WIFI_SHIFTIDX_CHIPIDC8;  
	}	
    _wfTxPoutMxdDbgInfo("[mxd after clbr autoc7c8][%s  chip%x]::   wfTxGIdxDelta=%d\r\n", __FUNCTION__, chipId,wfTxGIdxDelta);	
    _wfTxPoutMxdDbgInfo("[%s wfTxTbl] ::before shift  **************************\r\n", __FUNCTION__);
    if (minIdx == maxIdx) 
    {	//11B_mask 
   	 maskDeltagain = 10;		//11b mask pass delta
	 _wfTxPoutMxdDbgInfo("[%s wfTxTbl] ::before shift for 11B_mask  ^&^&^&^&^&^   \r\n", __FUNCTION__);
	rf_mem_read(WIFI_TX_GAIN_TBL_START_ADDR,&gainTx);
	_wfTxPoutMxdDbgInfo("[%s wfTxTbl] ::wfTxIdx  oldCfgGain 0 (mask)   %x   %x  \r\n", __FUNCTION__,WIFI_TX_GAIN_TBL_START_ADDR,gainTx);
	wifiTxPga4bit = _BSG(gainTx,_BIT6,_BIT3);
	wifiTxPga4bit = wifiTxPga4bit+wfTxGIdxDelta-maskDeltagain;
	_BSS(gainTx, _BIT6,_BIT3, wifiTxPga4bit);
	rf_reg_write(WIFI_TX_GAIN_TBL_START_ADDR,gainTx);
	_wfTxPoutMxdDbgInfo("[%s]::wfTxIdx   newCfgGain %x -> 0 (mask)  \r\n", __FUNCTION__,gainTx);
	return;
}

    for (i=minIdx;i<=maxIdx;i++)  //ergodic gain table
    {
        rf_mem_read(WIFI_TX_GAIN_TBL_START_ADDR+(i)*2,&gainTx);
        _wfTxPoutMxdDbgInfo("[%s wfTxTbl] ::wfTxIdx%x   %x   %x  \r\n", __FUNCTION__,i,WIFI_TX_GAIN_TBL_START_ADDR+(i)*2,gainTx);
	txindexarray[i-minIdx] = gainTx;
    }
    for (i=0;i < sizeof(gWifiTxGaincfg)/sizeof(gWifiTxGaincfg[0]);i++)//new gain table
    {
	if (txindexarray[maxIdx-minIdx] == gWifiTxGaincfg[i])
	{
		for(j=maxIdx;j>=minIdx;j--,i--)
		{
			if((i+wfTxGIdxDelta) <0)
				rf_reg_write(WIFI_TX_GAIN_TBL_START_ADDR+(j)*2,gWifiTxGaincfg[0]);
			else if((i+wfTxGIdxDelta) >= (sizeof(gWifiTxGaincfg)/sizeof(gWifiTxGaincfg[0]) ))
				rf_reg_write(WIFI_TX_GAIN_TBL_START_ADDR+(j)*2,gWifiTxGaincfg[sizeof(gWifiTxGaincfg)/sizeof(gWifiTxGaincfg[0])-1]);
			else
				rf_reg_write(WIFI_TX_GAIN_TBL_START_ADDR+(j)*2,gWifiTxGaincfg[i+wfTxGIdxDelta]);
		}
		break;
	}
    }
	/*
    for (i=minIdx;i<=maxIdx;i++)
    {
        rf_mem_read(WIFI_TX_GAIN_TBL_START_ADDR+(i)*2,&gainTx);
        _wfTxPoutMxdDbgInfo("[%s wfTxTbl] ::----wfTxIdx%x   %x   %x  \r\n", __FUNCTION__,i,WIFI_TX_GAIN_TBL_START_ADDR+(i)*2,gainTx);
    }
	
    for (i=minIdx;i<=maxIdx;i++)
    {
        rf_mem_read(WIFI_TX_GAIN_TBL_START_ADDR+(i)*2,&gainTx);
		if(gainTx == 0x03)
		{
			//read
			for(txindextemp = i-3,j=0; txindextemp>minIdx;txindextemp--,j++)
			{
				rf_mem_read(WIFI_TX_GAIN_TBL_START_ADDR+(txindextemp+2)*2,&txindexarray[j]);
			}
			//write
			for(txindextemp = i-3,j=0; txindextemp>minIdx;txindextemp--,j++)
			{
          			rf_reg_write(WIFI_TX_GAIN_TBL_START_ADDR+(txindextemp-1)*2,txindexarray[j]);
				_wfTxPoutMxdDbgInfo("[%s]::  readaddr1=%x writeaddr2=%x \t  gaintx =%x  \r\n", __FUNCTION__,WIFI_TX_GAIN_TBL_START_ADDR+(txindextemp+2)*2,WIFI_TX_GAIN_TBL_START_ADDR+(txindextemp-1)*2,txindexarray[j]);
			}
			for(txindextemp = i; txindextemp>i-3;txindextemp--)
			{
				if( txindextemp>minIdx )
				rf_reg_write(WIFI_TX_GAIN_TBL_START_ADDR+(txindextemp-1)*2,0x72);
			}
		}	
    }
*/
    _wfTxPoutMxdDbgInfo("[%s wfTxTbl] ::after shift  **************************\r\n", __FUNCTION__);
    for (i=minIdx;i<=maxIdx;i++)
    {
        rf_mem_read(WIFI_TX_GAIN_TBL_START_ADDR+(i)*2,&gainTx);
        _wfTxPoutMxdDbgInfo("[%s wfTxTbl] ::wfTxIdx%x   %x   %x  \r\n", __FUNCTION__,i,WIFI_TX_GAIN_TBL_START_ADDR+(i)*2,gainTx);
    }
}
#endif


void castorClbrPoutWifi(void)

{

//   MXD_U16    k=128, i2 = 0,i3 = 0, ii = 0, ib = 0, Rng = 0;
    MXD_U32    xj=0,i=0,mag = 0, minVal = 0,  kk = 0, rval=0,minMag=0,maxMag=0,refVal=0,delta=0;
    //MXD_U16  txSclTbl[]={0x200;0x1f0;0x210;0x1e0};
    MXD_U32 testVal1=0, testValMax=0, testValMin=0xffff,testSum=0;
    MXD_U32 testTbl[200]={0};

    MXD_U32 minCfg=0,maxCfg=0,curCfg =0x200;
    MXD_U16 valxx[10][10]={0},j=0;
    MXD_S16 deltaIdx = 0;
    MXD_U32 curWfTxIdx=0;
    MXD_U16 modeDctPosi =0,cfgSwtch =0 ;
    MXD_U16 flagClbrPoutFromNv =0;
	MXD_U32 regVal=0,chipid=0;
	MXD_U32 iiii=0;
	MXD_U32 testReslt[1024]={0,};
	flagClbrPoutFromNv= gPoutMode&(0x01<<_BIT7);
      _wfTxPoutMxdDbgInfo("[mxd] [castorClbrPoutWifi]:: gPoutModeFromNV= 0x%04x  \r\n",gPoutMode );
	if(flagClbrPoutFromNv==0)
	{
	      _wfTxPoutMxdDbgInfo("[mxd] [castorClbrPoutWifi]:: Pout disable!!! \r\n" );
	      return;
	}
	rf_reg_read(ctrl_chip_id_reg, &chipid);
    cfgSwtch= _BSG(gPoutMode, _BIT3,_BIT2);
    modeDctPosi= _BSG(gPoutMode,_BIT1,_BIT0);
	if(cfgSwtch== 0)	
	{
    _SET_TROUT2_ANT_SEL_SWT_OFF;
	}
	else if(cfgSwtch== 1)	
	{
		_SET_TROUT2_ANT_SEL_LOFT_WIFI;
	}
	if(0==modeDctPosi)//0: detect PAD    1: detect antenna
	{
		if (chipid == _CHIP_ID_C7)
		{			
			rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0xd27f);
			rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0x89cd);
		}  
		else if (chipid == _CHIP_ID_C8)
		{
			rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0xd8fc);
			rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0xff97);
		}			
		_REGBC(rfmux_cal_cfg_reg,_BIT0);
	       _wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: PDET_POSI   detect PAD !!! \r\n" );
	}
	else if(1==modeDctPosi)//  1: detect antenna
   	{
		_REGBS(rfmux_cal_cfg_reg,_BIT0);
		_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: PDET_POSI   detect antenna !!! \r\n" );
   	}
	else if(2==modeDctPosi)//  2: detect PAD & PA-off  
	{
		_REGBC(rfmux_cal_cfg_reg,_BIT0);
		rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0x0);
		rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0x0);
		_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: PDET_POSI   detect PAD &&&&& PA-off !!! \r\n" );
	}
	else if(3==modeDctPosi)//  3: detect PAD & PA-off  
	{
		_REGBC(rfmux_cal_cfg_reg,_BIT0);
		rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0x0);
		rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0x0);
		_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: PDET_POSI   detect PAD &&&&& PA-off !!! \r\n" );
	}
    rf_reg_write(0xbd,0x01);

    _REGBSS(ctrl_fm_adc_ctrl_reg,_BIT9,_BIT0,0);

    // SetSTD freq
    rf_reg_write(frxp_frxp_rcstd_cos_reg,0x4000);  // check the Lo
    rf_reg_write(frxp_frxp_rcstd_sin_reg,0x0);     //

    _REGBS(txp_txp_ctrl_reg, _BIT1);
    _REGBSS(txp_txp_ctrl_reg,_BIT12,_BIT11,0x01);     //txe1/e2 sel wifi::     00:bttx    01:wftx  1?:sprdPinMode

#if 0
	//debug info


	_wfTxPoutMxdDbgInfo("[Func::%s]  : **************************************  \n",__FUNCTION__);
	//for(curWfTxIdx=0x24;curWfTxIdx<0x30;curWfTxIdx+=4)
	for(curWfTxIdx=0x2a;curWfTxIdx<0x2c;curWfTxIdx+=4)
	{
		refVal=0xc0;

		rfClbrSetWfTxGain(curWfTxIdx);

		for (kk = 0; kk <(0x2c-0xc+1);kk+=4)
		{
			_REGBS(rfmux_cal_cfg_reg,_BIT0);
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: PDET_POSI   detect antenna !!! \r\n" );


			rf_reg_read(ctrl_chip_id_reg, &chipid);
			if (chipid == _CHIP_ID_C7)
			{			
				rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0xd27f);
				rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0x89cd);

			}  
			else if (chipid == _CHIP_ID_C8)
			{
				rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0xd8fc);
				rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0xff97);

			}
			testReslt[kk*4+0]=refVal+kk*16;


			rf_reg_write(rfmux_pout_tx_scale_reg,refVal+kk*16);
			MxdSleep(MXD_SLEEP_MS);
			testValMax=0;
			testValMin=0xffff;
			testSum=0;
			for (xj=0;xj<30;xj++)
			{           
				rfClbrReadMag(&mag);		
				if (testValMax<mag)testValMax=mag;
				if (testValMin>mag)testValMin=mag;
				testSum+=mag;
			}
			for(iiii=0;iiii<(5*0x0fffffff); iiii++);
			//for(iiii=0;iiii<(5*0x0fffffff); iiii++);
			rf_reg_read( rfmux_cal_cfg_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]::  rfreg200 = 0x%04x!!! \r\n", regVal);
			rf_reg_read( rfmux_pa_bias_ctrl1_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: rfreg20d = 0x%04x!!! \r\n", regVal);
			rf_reg_read( rfmux_pa_bias_ctrl2_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: rfreg20e = 0x%04x!!! \r\n", regVal);

			_wfTxPoutMxdDbgInfo("[%sPAD_PA]::[g=%x] magavg = %d %d %d, delta= %4d ]  refTxScalVal[%3d] = 0x%04x    xj=%d\n", __FUNCTION__,curWfTxIdx,testSum/(xj), testValMin,testValMax,  testValMax-testValMin, kk, refVal+kk*16 , xj);
			testReslt[kk*4+1]=testSum/(xj);

			_REGBC(rfmux_cal_cfg_reg,_BIT0);
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: PDET_POSI   detect PAD !!! \r\n" );	
			rf_reg_write(rfmux_pout_tx_scale_reg,refVal+kk*16);
			MxdSleep(MXD_SLEEP_MS);
			testValMax=0;
			testValMin=0xffff;
			testSum=0;
			for (xj=0;xj<30;xj++)
			{           
				rfClbrReadMag(&mag);		
				if (testValMax<mag)testValMax=mag;
				if (testValMin>mag)testValMin=mag;
				testSum+=mag;
			}
			for(iiii=0;iiii<(5*0x0fffffff); iiii++);	
			//for(iiii=0;iiii<(5*0x0fffffff); iiii++);
			rf_reg_read( rfmux_cal_cfg_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]::  rfreg200 = 0x%04x!!! \r\n", regVal);
			rf_reg_read( rfmux_pa_bias_ctrl1_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: rfreg20d = 0x%04x!!! \r\n", regVal);
			rf_reg_read( rfmux_pa_bias_ctrl2_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: rfreg20e = 0x%04x!!! \r\n", regVal);
			_wfTxPoutMxdDbgInfo("[%sPAD_D]::[g=%x] magavg = %d %d %d, delta= %4d ]  refTxScalVal[%3d] = 0x%04x    xj=%d\n", __FUNCTION__,curWfTxIdx,testSum/(xj), testValMin,testValMax,  testValMax-testValMin, kk, refVal+kk*16 , xj);
			testReslt[kk*4+2]=testSum/(xj);

			mxdDbgInfo("[mxd ][castorClbrPoutWifi]::  PA off\n");   
			rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0x0);
			rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0x0);

			//rf_reg_write(0x20d, 0xd27f);
			//rf_reg_write(0x20e, 0x89cd);

			rf_reg_write(rfmux_pout_tx_scale_reg,refVal+kk*16);
			MxdSleep(MXD_SLEEP_MS);
			testValMax=0;
			testValMin=0xffff;
			testSum=0;
			for (xj=0;xj<30;xj++)
			{           
				rfClbrReadMag(&mag);		
				if (testValMax<mag)testValMax=mag;
				if (testValMin>mag)testValMin=mag;
				testSum+=mag;
			}
			for(iiii=0;iiii<(5*0x0fffffff); iiii++);
			//for(iiii=0;iiii<(5*0x0fffffff); iiii++);
			rf_reg_read( rfmux_cal_cfg_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]::  rfreg200 = 0x%04x!!! \r\n", regVal);
			rf_reg_read( rfmux_pa_bias_ctrl1_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: rfreg20d = 0x%04x!!! \r\n", regVal);
			rf_reg_read( rfmux_pa_bias_ctrl2_reg,  &regVal );
			_wfTxPoutMxdDbgInfo("[mxd ][castorClbrPoutWifi]:: rfreg20e = 0x%04x!!! \r\n", regVal);
			_wfTxPoutMxdDbgInfo("[%s Paoff-pad]::[g=%x] magavg = %d %d %d, delta= %4d ]  refTxScalVal[%3d] = 0x%04x    xj=%d\n", __FUNCTION__,curWfTxIdx,testSum/(xj), testValMin,testValMax,  testValMax-testValMin, kk, refVal+kk*16 , xj);

			testReslt[kk*4+3]=testSum/(xj);


		}
		_wfTxPoutMxdDbgInfo("ValC8	PA_dtct	PAD_detect	PAD_dtct_PAoff	pow1	pow2	pow3	chipId=%x+++++++++++++++++++++++++++++++>>>>>\n",chipid);

		for (kk = 0; kk <(0x2c-0xc+1);kk+=4)
		{
			_wfTxPoutMxdDbgInfo("id=%02x		%04x	%d	%d	%d		\n",chipid,testReslt[kk*4+0], testReslt[kk*4+1],testReslt[kk*4+2],testReslt[kk*4+3]);
		}
		_wfTxPoutMxdDbgInfo("================================================================================\n");

	}

	return;
#endif

#if 1
// clbr pout here

    _wfTxPoutMxdDbgInfo("[castorGetRfRegInfo]:: step 0 : clbr Pout before  =========>>>>>>>>>>\r\n");
    castorGetRfRegInfo();
    rf_reg_write(rfmux_pout_tx_scale_reg,0x140);
    curWfTxIdx=0x28;
    deltaIdx = 0;
    for (kk=0;kk<10;kk++)
    {
        rfClbrSetWfTxGain(curWfTxIdx+deltaIdx);
        rfClbrReadMag(&mag);
        _wfTxPoutMxdDbgInfo("[%s]:: Searching ... wfTxIdx= %x  mag = %d  \r\n", __FUNCTION__, curWfTxIdx+deltaIdx, mag);
        if (mag>gPoutRef140Max)
        {
            deltaIdx--;
        }
        else if (mag<gPoutRef140Min)
        {
            deltaIdx++;
        }
        else
        {
            break;
        }
    }
    gIdxWfTxPwrShift = deltaIdx;
    _wfTxPoutMxdDbgInfo("[%s]:: SearchEnd wfTxIdx= %x mag = %d deltaIdx= %d  \r\n", __FUNCTION__, curWfTxIdx+deltaIdx, mag,deltaIdx);
#ifdef _MXD_WFBT_POUT_
    //rfClbrShiftWftxgain(0x0,  0x0,   gIdxWfTxPwrShift);
    rfClbrShiftWftxgain(0x01,0x0f, gIdxWfTxPwrShift-4);
    rfClbrShiftWftxgain(0x10,0x1f, gIdxWfTxPwrShift-4);
    rfClbrShiftWftxgain(0x20,0x2f, gIdxWfTxPwrShift);
    rfClbrShiftWftxgain(0x30,0x3f, gIdxWfTxPwrShift-4);
    rfClbrShiftBttxgain (0x7,0xb, (gIdxWfTxPwrShift+2)/4-1);	
#endif
    _wfTxPoutMxdDbgInfo("[castorGetRfRegInfo]:: Step2   shift tbl OK=========>>>>>>>>>>\r\n");
    castorGetRfRegInfo();
    //
    //_wfTxPoutMxdDbgInfo("[%s]::start Search txScal ... ...\n", __FUNCTION__ );

    maxCfg = 0x2e0;
    minCfg = 0x110;
    curCfg =0x200;
    for (kk = 0;kk<10;kk++)
    {
        rf_reg_write(rfmux_pout_tx_scale_reg,curCfg);
        MxdSleep(MXD_SLEEP_MS);
        testSum=0;
        testValMax=0;
        testValMin=0xffff;
        // get msg avg for
        for (i=0;i<5;i++)
        {
            rfClbrReadMag(&mag);
#if 0
            _REGBC(0x207, _BIT3);
            MxdSleep(1);
            _REGBS(0x207, _BIT3);
            rf_reg_write(frxp_rcstd_bgn,0x1);                       // rf_reg_write(0x20, 1);  // start STD
            _REG_CHK2(ctrl_tuner_status_reg,_BIT5);
            rf_reg_read(frxp_frxp_rcstd_mag_status_reg,&mag);
#endif     
            if (testValMax<mag)testValMax=mag;//wyp
            if (testValMin>mag)testValMin=mag;
            testSum+=mag;
            valxx[kk][i]= mag;
            //_wfTxPoutMxdDbgInfo("[%s] :<i=%d>  mag[0x%04x] = %d  0x%04x    refTxScalVal[%d] = 0x%04x \n",__FUNCTION__,i,frxp_frxp_rcstd_mag_status_reg,mag,mag,kk,refVal+kk*16);
        }
        testSum=testSum/i; //avg mag
        //_wfTxPoutMxdDbgInfo("[%s]::Searching ... ... curCfg = 0x%04x  mag =[%d  ,0x%04x]\n", __FUNCTION__,curCfg ,testSum, testSum);

        valxx[kk][i+0] = testSum;
        valxx[kk][i+1]= testValMin;
        valxx[kk][i+2]= testValMax;
        valxx[kk][i+3]= testValMax-testValMin;
        valxx[kk][i+4]= curCfg;
        if (    (testSum>(gWfTxPoutRefVal-gWfTxPoutRefRange))
	     &&(testSum<(gWfTxPoutRefVal+gWfTxPoutRefRange)))
        {
            break;
        }

        if (testSum > (gWfTxPoutRefVal+gWfTxPoutRefRange))
        {
            maxCfg= curCfg;
        }
        else
        {
            minCfg= curCfg;
        }
        curCfg= (maxCfg+minCfg)/2;
        if ((maxCfg-minCfg)<0x04)
            break;
    }
    _wfTxPoutMxdDbgInfo("[%s]::magavg = %d %d %d %d  0x%04x %d %d %x %x] \t\n", __FUNCTION__,testSum, testValMin,testValMax,  testValMax-testValMin,curCfg, curCfg,testSum,minCfg,maxCfg);
    rf_reg_write(rfmux_pout_tx_scale_reg,curCfg);
    _wfTxPoutMxdDbgInfo("[%s]::Search End!!! curCfg = 0x%04x %d %d \r\n", __FUNCTION__,curCfg, curCfg,testSum);
    for (j = 0;j<kk+1;j++)
    {
        testSum=valxx[j][i+0];
        testValMin=valxx[j][i+1];
        testValMax=valxx[j][i+2];
        _wfTxPoutMxdDbgInfo("[j=%d]::magavg = %d %d %d, delta= %d ]   [%d %d %d %d %d ] [c8= %4x %d]\n", j,testSum, testValMin,testValMax,  testValMax-testValMin,valxx[j][0],valxx[j][1],valxx[j][2],valxx[j][3],valxx[j][4], valxx[j][i+4],valxx[j][i+4]);
    }


    //for (kk=0;kk<100;kk++)MxdSleep(50);

#endif
#if 0
    rf_reg_write(rfmux_pout_tx_scale_reg,refVal+kk*16);
    MxdSleep(MXD_SLEEP_MS);
    //MxdSleep(1);
    testValMax=0;
    testValMin=0xffff;
    testSum=0;
    for (xj=0;xj<50;xj++)
    {
        for (i=0;i<100;i++)
        {
            rfClbrReadMag(&mag);
#if 0
            _REGBC(0x207, _BIT4);
            MxdSleep(1);
            rf_reg_write(frxp_rcstd_bgn,0x1);                       // rf_reg_write(0x20, 1);  // start STD
            do
            {
                rval=0;
                rf_reg_read(ctrl_tuner_status_reg,&rval);
            }
            while ((rval & 0x20)==0);
            rf_reg_read(frxp_frxp_rcstd_mag_status_reg,&mag);
#endif		 
            testTbl[i]= mag;
            if (testValMax<mag)testValMax=mag;
            if (testValMin>mag)testValMin=mag;
            testSum+=mag;
        }
    }
    _wfTxPoutMxdDbgInfo("[%s pout disable]::magavg = %d %d %d, delta= %d ]  refTxScalVal[%d] = 0x%04x    xj=%d\n", __FUNCTION__,testSum/(i*xj), testValMin,testValMax,  testValMax-testValMin, kk, refVal+kk*16 , xj);

//clbr End
#endif
    gWfTxScaleCfg = curCfg;
    // init
    // wf dac scale
    for (i = 0; i < sizeof(gDacScaleCh)/sizeof(gDacScaleCh[0]); i ++)
    {
        gDacScaleCh[i]=0x200;
        mxdDbgInfo("[mxd $] :: castorClbrPoutWifi:: init gDacScaleCh[%d]=0x%x\r\n",i,gDacScaleCh[i]);
    }
    _REGBSS(ctrl_fm_adc_ctrl_reg,_BIT9,_BIT0,0x200);
	rf_reg_read(ctrl_chip_id_reg, &chipid);
	if (chipid == _CHIP_ID_C7)
	{			
		rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0xd27f);
		rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0x89cd);
	}  
	else if (chipid == _CHIP_ID_C8)
	{
		rf_reg_write(rfmux_pa_bias_ctrl1_reg, 0xd8fc);
		rf_reg_write(rfmux_pa_bias_ctrl2_reg, 0xff97);
	}			
	    _wfTxPoutMxdDbgInfo("[castorGetRfRegInfo]:: Step3   Pout End!=========>>>>>>>>>>\r\n");
           castorGetRfRegInfo();
}


void cfgTrout2Dac()
{
    MXD_U32 sysRegMap[]=
    {
        0x802000, 0xA02000, 0x802000,
        0x810002, 0xA10002, 0x810002,
        0x820004, 0xA20004, 0x820004,
        0x830008, 0xA30008, 0x830008,
        0x840010, 0xA40010, 0x840010,
        0x850020, 0xA50020, 0x850020,
        0x860040, 0xA60040, 0x860040,
        0x870080, 0xA70080, 0x870080,
        0x880100, 0xA80100, 0x880100,
        0x890200, 0xA90200, 0x890200,
        0x8A0400, 0xAA0400, 0x8A0400,
        0x8B0800, 0xAB0800, 0x8B0800,
        0x8C1000, 0xAC1000, 0x8C1000,
        0x8D2000, 0xAD2000, 0x8D2000,
        0x8E4000, 0xAE4000, 0x8E4000,
        0x8F8000, 0xAF8000, 0x8F8000,
        0x902000, 0xB02000, 0x902000,
        0x910002, 0xB10002, 0x910002,
        0x920004, 0xB20004, 0x920004,
        0x930008, 0xB30008, 0x930008,
        0x940010, 0xB40010, 0x940010,
        0x950020, 0xB50020, 0x950020,
        0x960040, 0xB60040, 0x960040,
        0x970080, 0xB70080, 0x970080,
        0x980100, 0xB80100, 0x980100,
        0x990200, 0xB90200, 0x990200,
        0x9A0400, 0xBA0400, 0x9A0400,
        0x9B0800, 0xBB0800, 0x9B0800,
        0x9C1000, 0xBC1000, 0x9C1000,
        0x9D2000, 0xBD2000, 0x9D2000,
        0x9E4000, 0xBE4000, 0x9E4000,
        0x9F8000, 0xBF8000, 0x9F8000,

    };
    int i=0;
    MXD_U32 regVal=0;
    MXD_U32  regValForPA=0;
    MXD_U32 idPa=0;
    for (i=0;i<sizeof(sysRegMap)/sizeof(sysRegMap[0]);i++)
    {
        trout_reg_write(0x70,sysRegMap[i]);
#ifdef TRACE_DCOC_LOG
        mxdDbgInfo("write sysreg: 0x70 = %x--i=%d\n",sysRegMap[i],i);
#endif
    }
    trout_reg_write(0x70, 0x0);

    mxdDbgInfo("cfgTrout2Dac new config    0523!!!\r\n",sysRegMap[i],i);
    for (i=0;i<sizeof(sysRegMap)/sizeof(sysRegMap[0])/3;i++)
    {
        trout_reg_write(0x70,(sysRegMap[(i*3 + 1)]&0x9FFFFF));
        //  trout_reg_write(0x70,sysRegMap[i*3]);
        trout_reg_read(0x71,&regVal);
        if (regVal  != (sysRegMap[3*i+1]&0x0ffff))
            mxdDbgInfo("DAC read sysreg: 0x70 = %x-should be %x-i=%d\n",regVal,sysRegMap[3*i+1]&0x0ffff,i);
    }
//trout_reg_write(0x70,sysRegMap[i]);
    trout_reg_read(0x4f,&regVal);
    regVal|=0x01;
    trout_reg_write(0x4f,regVal);
    mxdDbgInfo("PLS config in sys!!!    Trout2write::  troutreg(4f.bit0=1):  = 0x%x-\n",regVal);

#ifdef _MXD_ANDROID_
    trout_reg_read(0x55,&regVal);
    regVal|=0x01;
    trout_reg_write(0x55,regVal);
    mxdDbgInfo("Trout2write::  troutreg(55.bit0=1):  = 0x%x-\n",regVal);
#endif
#ifndef _MXD_ANDROID_
    // Configure TX Power
    rf_reg_read(_RFREG_SW_CFG_PA_LNA,&regValForPA);
    idPa = (regValForPA>>8)&0xff;
    if (_PA_ID_2574 == idPa)
    {
        trout_write_phy_reg(0xff, 0);
        trout_write_phy_reg(0x70 , 0x3b);  // 11n,11g 0x30 rTXPOWER11A      3f,30,20
        trout_write_phy_reg(0x71 , 0x1b);  // 11b 0x10 rTXPOWER11B    00,10,1
    }
    else if (_PA_ID_NONE == idPa)
    {
        trout_write_phy_reg(0xff, 0);
        trout_write_phy_reg(0x70 , 0x2a);  // 11n,11g  rTXPOWER11A      
        trout_write_phy_reg(0x71 , 0x05);  // 11b  rTXPOWER11B    
    }
    else if (_PA_ID_FM8205== idPa)
    {
        trout_write_phy_reg(0xff, 0);
        trout_write_phy_reg(0x70 , 0x39);  // 11n,11g 0x30 rTXPOWER11A      3f,30,20
        trout_write_phy_reg(0x71 , 0x19);  // 11b 0x10 rTXPOWER11B    00,10,1
    }
    // Configure TX Power
    trout_write_phy_reg(0x6e , 0x00);  // zlf@0524
	trout_write_phy_reg(0xa1 , 0xd7);  // Pls refer to ChengGuang.Hu@SPRD 
	trout_write_phy_reg(0xa2 , 0x13);  
	trout_write_phy_reg(0xa1 , 0x3d);   
	trout_write_phy_reg(0xa2 , 0x14);
	trout_write_phy_reg(0xa1 , 0xf3);  
	trout_write_phy_reg(0xa2 , 0x18);  
#endif
}




int  castorClbrLoftBt()
{
    //MXD_U16 regVal=0;
    MXD_U16 minMag=0xffff;
    int flagMode=0;
//    int i;
    MXD_U16 iLoftDac=0;
    MXD_U16 qLoftDac=0;
//    MXD_U32 flag;
    MXD_U16 bestTxLoftCtrl=0,bestTxLoftCtrlQ=0,bestTxLoftCtrlI=0;
    MXD_U16 regAddrTxGidx[2]={0xc1,0xc3};    //regAddr btTxIndex   wifiTxIndex
    MXD_U16 txLoftDacVal[2]= {0x21a,0x21b};   //BtTxLoftDacCtrl     wifiTxLoftDacCtrl
    MXD_U16 txLoftTblAddr[2]={0x4d4,0x4d8};   //bt/wifi entry0  txLoftDacCtrlTbl
    MXD_U16 valCtrlLoftEn[2]={0x0032,0x0031};
    MXD_U32 mag=0;
    //MXD_U16 tblClbrTxIdx [2][4]=  {{4,4,4,4}, {4,4,4,4},};
    //for test
    MXD_U16 tblClbrTxIdx [2][8]=
    {
        {8,8,8,8,8,8,8,8},
        {8,8,8,8,8,8,8,8},
    };
    MXD_U32 gainTx=0,btTxPga4bit=0,txGm3bit=0;
    int btTxGainIdx;
    int flagModePllLdoEn[]={  0,1,0, //fmPllDis btPllEn  wifiPllDis
                              0,0,1  //fmPllDis btPllDis wifiPllEn
                           };
    int n =0;
    int xx=0;
    MXD_U32 tstIQ=0;

    flagMode=0;//bt clbr

    //_REGBSS(0x04,7,4,0x0f);//26Mhz force.

    _REGBSS(ctrl_txp_clk_force_ctrl_reg,_BIT1,_BIT0,0x02);//26Mhz force.
    _SET_TROUT2_ANT_SEL_LOFT_BT;   
    if (flagMode==0)
    {
        castorHostBtTxInit();
        _REGBS(rf_ldo_ctrl_reg,_BIT10);
        _REGBSS(rf_ldo_ctrl_reg,_BIT8,_BIT7,0x3);
        _REGBS(rf_lobuffer_cfg_reg,_BIT1);

        _REGBC(rf_ldo_ctrl_reg,_BIT9);//diable wifi lo
        _REGBC(rf_lobuffer_cfg_reg,_BIT2);

        castorSetFreqMhzBt(2412);  //0331
    }

    _REGBSS(rfmux_gain_table_sel_reg,_BIT1,_BIT0, 0x0);//wifi tx gain sel: 00 host 01mxd 10:reserved 11:sprd tbl
    _REGBC(rfmux_gain_table_sel_reg,_BIT4);     // bt txgain sel: 0: host 1: table
    _REGBC(rfmux_gain_table_sel_reg,_BIT6);     // wf/bt pa sel : 0: host 1: table

    _rfFmAdcOutCmpl;
    //TxloftSel bit7=0:BTfrom Reg21a,bit6=0:WifiFrom Reg21b
    _REGBSS(rfmux_loft_pout_cfg_reg,7,6,0);    //TxloftSel bit7=0:BTfromReg21a,bit6=0:WifiFromReg21b
    minMag=0xffff;

    /*   int flagModePllLdoEn={  0,1,0 ;0,0,1 };     */
    //for (flagMode=0;flagMode<1;flagMode++)  //flag = 0: bt   1: wifi
    {
        if (flagMode==0)
        {
            // rfClbrSetLdoPll(0,1,0);// LdoPll: fmDis btEn wifiDis
            _REGBS(rf_ldo_ctrl_reg,_BIT1);
            _REGBS(rf_ldo_ctrl_reg,_BIT4);
            _REGBS(rfmux_fm_cfg_reg,_BIT1);
            _REGBS(rfmux_fm_cfg_reg,_BIT4);
            _REGBS(rfmux_cal_cfg_reg,_BIT0);
            _REGBS(rfmux_cal_cfg_reg,_BIT1);
            _REGBS(rfmux_mode_cfg_ctrl0_reg,_BIT11);
            _REGBS(rfmux_mode_cfg_ctrl2_reg,_BIT8);
        } 
        {
            _REGBSS(rfmux_mode_cfg_ctrl2_reg, _BIT15, _BIT13, 7);
        } 

        // rf_reg_write(rfmux_mode_sel_reg, valCtrlLoftEn[flagMode]); //LoftCl_BtTx
        rfClbrTxToneEn_SetNco(flagMode,_FLAG_FREQ_500K);
        rfClbrStdEnable(flagMode,_FLAG_FREQ_500K);

//        for(gindx=0;gindx<4;gindx++)    //txGain 0:3
        for (n=0;n<4;n++)    //txGain 0:3
        {
            btTxGainIdx= tblClbrTxIdx[flagMode][n];
            //  regAddrTxGidx[2]={rfmux_bttx_gidx_reg, rfmux_wftx_gidx_reg};
            rf_reg_write(regAddrTxGidx[flagMode],btTxGainIdx);
            rf_mem_read(BT_TX_GAIN_TBL_START_ADDR+btTxGainIdx,&gainTx);
            //rf_mem_read(BT_TX_GAIN_TBL_START_ADDR+(n*2*8),&gainTx);
            btTxPga4bit = _BSG(gainTx,_BIT6,_BIT3);
            txGm3bit = _BSG(gainTx,_BIT2,_BIT0);

            _REGBSS(rfmux_wb_tx_gain_reg, _BIT6,_BIT3, btTxPga4bit);
            _REGBSS(rfmux_wb_tx_gain_reg, _BIT2,_BIT0, txGm3bit);
            minMag = 0xffff;
            bestTxLoftCtrl = 0x8080;
            for (iLoftDac = (0x80-LOFT_DAC_I_RNG); iLoftDac<(0x80+LOFT_DAC_I_RNG); iLoftDac+=2)
            {
                //  txLoftDacVal[2]= {0x21a,0x21b};
                rf_reg_write(txLoftDacVal[flagMode], (iLoftDac<<8)|0x80);  //21a:bt  21b:wifi
                _rfStdStart;
                MxdSleep(MXD_SLEEP_MS);
                _REG_CHK(ctrl_tuner_status_reg,_BIT5);
                rf_reg_read(frxp_frxp_rcstd_mag_status_reg, &mag);

                if (mag<minMag)
                {
                    minMag=mag;
                    bestTxLoftCtrl= (iLoftDac<<8)|0x80;
                    _btLoftMxdDbgInfo("castorClbrLoft:: SearchIIII minMag= 0x%x, bestTxLoftCtrl=0x%x\r\n ",minMag, bestTxLoftCtrl);
#ifdef _MXD_CLBR_OPTIMIZE
                    if (minMag<=2)
                    {
                        _btLoftMxdDbgInfo("castorClbrLoft[IIII]:: BREAKNOW!  iLoftDac=%x\r\n",iLoftDac);
                        break;
                    }
#endif
                }
            }

            rf_reg_write(txLoftDacVal[flagMode], bestTxLoftCtrl);  //21a:bt  21b:wifi
            //bestTxLoftCtrlI= (bestTxLoftCtrl&0xff00)>>8;
            minMag=0xffff;

            for (qLoftDac = 0x80-LOFT_DAC_Q_RNG; qLoftDac<0x80+LOFT_DAC_Q_RNG; qLoftDac+=2)
            {
                rf_reg_write(txLoftDacVal[flagMode], (bestTxLoftCtrl&0xff00)|qLoftDac);  //21a:bt  21b:wifi
                _rfStdStart;
                MxdSleep(1);
                _REG_CHK(ctrl_tuner_status_reg,_BIT5);
                rf_reg_read(frxp_frxp_rcstd_mag_status_reg, &mag);

                if (mag<minMag)
                {
                    minMag=mag;
                    bestTxLoftCtrl= (bestTxLoftCtrl&0xff00)|qLoftDac;
                    _btLoftMxdDbgInfo("castorClbrLoft:: SearchQQQQ minMag= %x, bestTxLoftCtrl=%x\r\n ",minMag, bestTxLoftCtrl);

#ifdef _MXD_CLBR_OPTIMIZE
                    if (minMag<=2)
                    {
                        _btLoftMxdDbgInfo("castorClbrLoft[QQQQ]:: BREAK NOW! qLoftDac=%x \r\n",qLoftDac);
                        break;
                    }
#endif
                }
            }
            rf_reg_write(txLoftDacVal[flagMode], bestTxLoftCtrl);
            // now get the best LoftDac for this gain
            rf_reg_write(txLoftTblAddr[flagMode]+n, bestTxLoftCtrl);
            mxdDbgInfo("[mxd][mode = %d  ::0bt,1wifi]btTxGainIdx=[%d]castorClbrLoft:: SearchIQ minMag= %x, bestTxLoftCtrl=%x Write reg_host_0x%x  reg_tbl_0x%x loftIdx= %x    gainTx=0x%x\r\n ",flagMode,btTxGainIdx,minMag, bestTxLoftCtrl,txLoftDacVal[flagMode],txLoftTblAddr[flagMode]+n,n,gainTx);

            //_btLoftMxdDbgInfo("[mxd][mode = %d  ::0bt,1wifi]btTxGainIdx=[%d]castorClbrLoft:: SearchIQ minMag= %x, bestTxLoftCtrl=%x Write reg_host_0x%x  reg_tbl_0x%x  loftIdx= %x\r\n ",flagMode,btTxGainIdx,minMag, bestTxLoftCtrl,txLoftDacVal[flagMode],txLoftTblAddr[flagMode]+n,n);

        }

        //DisableLo(flagMode);
    } // end for mode

    castorClbrTximbBt();
    _REGBSS(ctrl_txp_clk_force_ctrl_reg,_BIT1,_BIT0,0x0);//txe 26Mhz force disable.
    _SET_TROUT2_ANT_SEL_NORMAL;
    return 0;
}



MXD_U16 regBitsGet(MXD_U16 regAddr,MXD_U8 bitH, MXD_U8 bitL)
{
    MXD_U32 regVal=0;
    rf_reg_read(regAddr,  &regVal);

    return (0xffff&_BSG(regVal,bitH,bitL));
}
void castorSetGainCo(int flag)  // 1: colisten    0: wifionly
{
    
}


void castorSetRfEntry(int flag) // 1: bt only 2:wifi only 3:colisten 0:all idle
{
    mxdDbgInfo("castorSetRfEntry:: input flagmode= %d  !!![1: bt 2:wifi 3: bt&wirfi]    \r\n",flag);
    if (flag==1)
    { // BT Only
        //0x2a0 sprd entry 3: [bit11:9],entry 2[bit8:6], entry1[bit5:3], entry0[bit2:0]
        castorSetGainCo(0);
        rf_reg_write(0x40c,0x1c5);               // tbl4 ->btrx tbl h
        rf_reg_write(0x040d,0x0066);             // btrx tbl m
        rf_reg_write(0x040e,0xb400);             // btrx tbl l
        rf_reg_write(rfmux_sprd_mode_sel_reg, (_RF_MODE_WIFI_TX<<9)|(_RF_MODE_BT_TX<<6)|(_RF_MODE_BT_RX<<3)|_RF_MODE_INIT);

    }
    else if (2==flag)
    { // WF Only
        castorSetGainCo(0);
        rf_reg_write(0x0409,0x1c3);      //table3-> wifiRx          //  , 0x00c3->1c3 // wfrx tbl h
        rf_reg_write(0x040a,0x0389);                                // wfrx tbl m
        rf_reg_write(0x040b,0xca00);                                // wfrx tbl l
        rf_reg_write(rfmux_sprd_mode_sel_reg, (_RF_MODE_WIFI_TX<<9)|(_RF_MODE_BT_TX<<6)|(_RF_MODE_WIFI_RX<<3)|_RF_MODE_INIT);

    }
    else if (3==flag)
    { // WF &BT co-work
        castorSetGainCo(1);
        rf_reg_write(0x0409,0x1c7);              //0x0409, 0x1c7,   // tbl3 ->bwco tbl h
        rf_reg_write(0x040a,0x03ef);            //0x040a, 0x03ef,  // wtco tbl m
        rf_reg_write(0x040b,0xfe00);            //0x040b, 0xfe00,  // wtco tbl l
        rf_reg_write(0x40c,0x1c7);                  //0x0409, 0x1c7,   // tbl4 ->bwco tbl h
        rf_reg_write(0x040d,0x03ef);            //0x040a, 0x03ef,  // wtco tbl m
        rf_reg_write(0x040e,0xfe00);            //0x040b, 0xfe00,  // wtco tbl l
        rf_reg_write(rfmux_sprd_mode_sel_reg, (_RF_MODE_WIFI_TX<<9)|(_RF_MODE_BT_TX<<6)|(_RF_MODE_RX_COL<<3)|_RF_MODE_INIT);
        //zl@0730
        rf_reg_write(rfmux_btrx_gidx_reg, 0x001a);
        rf_reg_write(rfmux_mode_sel_reg, 0x0c);
        //rf_reg_write(rfmux_mode_sel_reg, 0x0018);
        // rf_reg_write(fd_bttx_lock_offset_reg,0x040);             // Bt Tx  0815
    }
    else if (0==flag)
    { // wifi bt/ildle
        castorSetGainCo(0);
        rf_reg_write(rfmux_sprd_mode_sel_reg, (_RF_MODE_INIT<<9)|(_RF_MODE_INIT<<6)|(_RF_MODE_INIT<<3)|_RF_MODE_INIT);
    }
    else
    {
        mxdDbgInfo("castorSetRfEntry:: input flagmode error !!!                 \r\n");
    }
}

#define _TST_PIN_SEL_IP   1
#define _TST_PIN_SEL_IN   2
#define _TST_PIN_SEL_QP   3
#define _TST_PIN_SEL_QN   4
#define _TST_PIN_DISABLE    0
#define _TST_PIN_SEL_BT     1
#define _TST_PIN_SEL_WIFI   2
#define _TST_PIN_SEL_FM     3
#define _TST_PIN_SEL_RX   0
#define _TST_PIN_SEL_TX   1
#define _TST_PIN_SEL_LO   2
void castorSetTstPin(int flagMode, int flagTxRxLo, int flagIQ) // flagMode[1:BT,2:wifi,3,fm,0:dis]   flagTxRxLo=[0:Rx,1:Tx,2:Lo]  flagIQ=[1-4:IP,IN,QP,QN]
{
//    int iqSel;
    int bitSel=0;
    mxdDbgInfo("castorSetTstPin:: input flagMode= %d!  [1:BT, 2:wifi: 0 disable]    \r\n",flagMode);
    mxdDbgInfo("castorSetTstPin:: input flagTxRxLo= %d!  [1:Tx, 2:Rx, 3:Lo]    \r\n",flagTxRxLo);
    mxdDbgInfo("castorSetTstPin:: input flagIQ= %d!    [1-4:IP,IN,QP,QN]    \r\n",flagIQ);
    if (flagMode==_TST_PIN_DISABLE)
    {  //disable
        _REGBSS(rf_ts_ctrl_reg,_BIT15,_BIT14,0x2);  // bit15 =1: ant_sel_1 en   bit14=0:diable TEST_OUT_EN
        _REGBSS(rf_syn_lotest_reg,_BIT1,_BIT0,0x0);//disablelo test   bit1=0:bt   bit0=0: fm
        _REGBC(rf_syn_lotest_reg,_BIT7);           //disable wifiLo test bit7=0
        rf_reg_write(rf_test_ctrl0_reg, 0x0);       // disable test output
        return;
    }
    if (flagTxRxLo==_TST_PIN_SEL_LO)
    {   //Lo to test
        _REGBSS(rf_ts_ctrl_reg,_BIT15,_BIT14,0x01);
        switch (flagMode)
        {
            case _TST_PIN_SEL_WIFI :
                _REGBS(rf_syn_lotest_reg,_BIT7);           //enable wifiLo test bit7=0
                break;
            case _TST_PIN_SEL_BT :
                _REGBS(rf_syn_lotest_reg,_BIT1);         //enable lo test   bit1=0:bt   bit0=0: fm
                break;
            case _TST_PIN_SEL_FM :
                _REGBS(rf_syn_lotest_reg,_BIT0);           //enable lo test   bit1=0:bt   bit0=0: fm
                break;
        }
        return;
    }
    if (flagMode==_TST_PIN_SEL_FM)
    {
        _REGBSS(rf_ts_ctrl_reg,_BIT15,_BIT14,0x1);  // bit15 =0: ant_sel_1 dis   bit14=1:En TEST_OUT_EN
        rf_reg_write(rf_test_ctrl0_reg, 0x0);       // disable test output
        return;
    }
    if (flagMode==_TST_PIN_SEL_FM)
    {
        _REGBSS(rf_ts_ctrl_reg,_BIT15,_BIT14,0x1);                 // bit15 =0: ant_sel_1 dis   bit14=1:En TEST_OUT_EN
        rf_reg_write(rf_test_ctrl0_reg, (flagIQ&0x03)<<_BIT12);    //enable fm rx
        return;
    }
    if ((_TST_PIN_SEL_BT ==flagMode)|| (_TST_PIN_SEL_WIFI==flagMode))
    {
        _REGBSS(rf_ts_ctrl_reg,_BIT15,_BIT14,0x1);                 // bit15 =0: ant_sel_1 dis   bit14=1:En TEST_OUT_EN
        flagIQ += (flagMode-1)*4; // valBt: 1,2,3,4 wifiVal:5,6,7,8
        flagIQ &= 0xf; //@@zl@0621
        bitSel= (flagTxRxLo&0x01)*4;  //rfReg0x21d    Rx[3:0] Tx[7:4]
        rf_reg_write(rf_test_ctrl0_reg, (flagIQ&0x0f)<<bitSel);    // enable BT/Wifi ,RX/TX, IQ
        return;
    }
    mxdDbgInfo("castorSetTstPin:: Error!   \r\n");
    return;
}
void castorDbgSetWorkMode(int flagCfg, int  flagRfMode)
{
    mxdDbgInfo("castorDbgSetWorkMode:: flagCfg=%d[00:Host 01:Mxd 10:clbr 11:sprd]!   \r\n",flagCfg);
    _REGBSS(rfmux_mode_sel_reg,_BIT4,_BIT3,flagCfg);
    mxdDbgInfo("castorDbgSetWorkMode:: flagRfMode=%d[0:init, 1:wftx, 2:bttx, 3:wfrx, 4:btrx 5:rxco]!   \r\n",flagRfMode);
    _REGBSS(rfmux_mode_sel_reg,_BIT2,_BIT0,flagRfMode);
}
void castorDbgGetWorkMode(int *pFlagCfg, int *pFlagRfMode)
{
    *pFlagCfg = regBitsGet(rfmux_mode_sel_reg,_BIT4,_BIT3);
    * pFlagRfMode = regBitsGet(rfmux_mode_sel_reg,_BIT2,_BIT0);
    mxdDbgInfo("castorDbgGetWorkMode:: flagCfg=%d[00:Host 01:Mxd 10:clbr 11:sprd]!   \r\n", *pFlagCfg);
    mxdDbgInfo("castorDbgGetWorkMode:: flagRfMode=%d[0:init, 1:wftx, 2:bttx, 3:wfrx, 4:btrx 5:rxco]!   \r\n",*pFlagRfMode);
}
void castorDbgSetGainModeBtTx(int  flagGainMode)
{
    mxdDbgInfo("castorDbgSetGainModeBtTx:: flagGainMode=%d[0:Hot 01:Tbl]!   \r\n", flagGainMode);
    if (0==flagGainMode)
    {   //BtTX
        //_REGBS(rfmux_gain_table_sel_reg,_BIT7);                 //LnaGainSel:   0:Host  1:tbl
        //_REGBC(rfmux_gain_table_sel_reg,_BIT6);                 //PA_sel    0:host 1:mxd
        //_REGBS(rfmux_gain_table_sel_reg,_BIT5);                 //btRxGain      0:Host  1:tbl
        //_REGBSS(rfmux_gain_table_sel_reg,_BIT3,_BIT2,0x03);     //wifiRxGain Sel:       00:Host  01:mxd 10:reserved 11:sprd
        //_REGBSS(rfmux_gain_table_sel_reg,_BIT1,_BIT0,0);        //wifitxGain Sel:       00:Host  01:mxd 10:reserved 11:sprd
        //_REGBS(rfmux_gain_table_sel_reg,_BIT8); //colisten priotity:  0:btRxLnaLbl  1:wifiRxLnaTbl
        _REGBC(rfmux_gain_table_sel_reg,_BIT4);                 //btTxGainSel:  0:Host  1:tbl
        _REGBC(rfmux_gain_table_sel_reg,_BIT6);                 //PA_sel    0:host 1:mxd
        return;
    }
    if (1==flagGainMode)
    {   //BtTX
        _REGBS(rfmux_gain_table_sel_reg,_BIT4);                 //btTxGainSel:  0:Host  1:tbl
        _REGBS(rfmux_gain_table_sel_reg,_BIT6);                 //PA_sel    0:host 1:mxd
        return;
    }
}
void castorDbgSetGainModeBtRx(int  flagGainMode)
{
    mxdDbgInfo("castorDbgSetGainModeBtRx:: flagGainMode=%d[0:Host 01:Tbl]!   \r\n", flagGainMode);
    if (0==flagGainMode)
    {   //Bt Rx Host
        _REGBC(rfmux_gain_table_sel_reg,_BIT7);                 //LnaGainSel:   0:Host  1:tbl
        _REGBC(rfmux_gain_table_sel_reg,_BIT5);                 //btRxGain      0:Host  1:tbl
        return;
    }
    if (1==flagGainMode)
    {   //BtRx tbl
        _REGBS(rfmux_gain_table_sel_reg,_BIT7);                 //LnaGainSel:   0:Host  1:tbl
        _REGBS(rfmux_gain_table_sel_reg,_BIT5);                 //btRxGain      0:Host  1:tbl
        return;
    }
}
void castorDbgSetGainModeWifiTx(int  flagGainMode)
{
    mxdDbgInfo("castorDbgSetGainModeWifiTx:: flagGainMode=%d[00:Host  01:mxd 10:reserved 11:sprd ]!   \r\n", flagGainMode);
    _REGBSS(rfmux_gain_table_sel_reg,_BIT1,_BIT0,flagGainMode);        //wifitxGain Sel:       00:Host  01:mxd 10:reserved 11:sprd
    if (0==flagGainMode)
    {   // Rx Host
        _REGBC(rfmux_gain_table_sel_reg,_BIT6);                 //PA_sel    0:host 1:mxd
        return;
    }
    else
    {   //Rx tbl
        _REGBS(rfmux_gain_table_sel_reg,_BIT6);                 //PA_sel    0:host 1:mxd
        return;
    }
}
void castorDbgSetGainModeWifiRx(int  flagGainMode)
{
    mxdDbgInfo("castorDbgSetGainModeWifiRx:: flagGainMode=%d[00:Host  01:mxd 10:reserved 11:sprd ]!   \r\n", flagGainMode);
    _REGBSS(rfmux_gain_table_sel_reg,_BIT3,_BIT2,flagGainMode);     //wifiRxGain Sel:       00:Host  01:mxd 10:reserved 11:sprd
    if (0==flagGainMode)
    {   // Rx Host
        _REGBC(rfmux_gain_table_sel_reg,_BIT7);                 //LnaGainSel:   0:Host  1:tbl
        //_REGBC(rfmux_gain_table_sel_reg,_BIT6);                 //PA_sel    0:host 1:mxd
        return;
    }
    else
    {   //Rx tbl
        _REGBS(rfmux_gain_table_sel_reg,_BIT7);                 //LnaGainSel:   0:Host  1:tbl
        //_REGBS(rfmux_gain_table_sel_reg,_BIT6);                 //PA_sel    0:host 1:mxd
        return;
    }
}

void castorDbgGetRfInfo()
{
    MXD_U32 regVal =0;
    int i;
    MXD_U32 regMap[]=
    {
        ctrl_chip_id_reg,
        rf_ldo_ctrl_reg,
	 rfmux_mode_cfg_ctrl2_reg,    //bit15:13     000b->111b
        ctrl_soft_reset_reg,
        rfmux_mode_sel_reg,
        rfmux_sprd_mode_sel_reg,
        rfmux_gain_table_sel_reg,
        rfmux_btrx_gidx_reg,
        rfmux_bttx_gidx_reg,
        rfmux_wfrx_gidx_reg,
        rfmux_wftx_gidx_reg,
        rfmux_sprd_wifi_gidx_reg,
        rfmux_wifi_actual_gidx_reg,
        rfmux_pout_tx_scale_reg,
        txp_txp_ctrl_reg,
        rf_ts_ctrl_reg,
        rfmux_wb_tx_gain_reg,
        _RFREG_SW_RF_STATUS_,//sw
        _RFREG_SW_CFG_LDO,
        _RFREG_SW_CFG_PA_LNA,
        _RFREG_SW_ANTSEL_BK,
        ctrl_cgf_ctrl_reg,
        txp_txp_bttx_txe1_reg,
        txp_txp_bttx_txe2_reg,
        txp_txp_wftx_txe1_reg,
        txp_txp_wftx_txe2_reg,
        0x402,	// 0000->0003
        0x405,   // 8000->8003
        0x408,  //  8000->8003
    };


    for (i=0;i<sizeof(regMap)/sizeof(regMap[0]);i++)
    {
        rf_reg_read(regMap[i], &regVal);
        mxdDbgInfo("[mxd castorDbgGetRfInfo] [0x%04x,  0x%04x]   done!\n",regMap[i], regVal);
    }

}
void castorGetRfRegInfo()
{
    MXD_U32 regVal =0;
    int i;
    return;
    for (i=0;i<0x7ff;i++)
    {
        rf_reg_read(i, &regVal);
        mxdDbgInfo("[mxd castorGetRfRegInfo] 			[0x%04x,  0x%04x]   done!\n",i, regVal);
    }	
}
void tstNco()
{
    rf_reg_write(txp_txp_nco1_freq_h_reg,0x0010); // set Nco Freq
    rf_reg_write(txp_txp_nco1_freq_l_reg,0x0000); // 500k?
}

void tstSST()
{
    rf_reg_write(txp_txp_nco1_freq_h_reg,0x0010); // set Nco Freq
    rf_reg_write(txp_txp_nco1_freq_l_reg,0x0000); // 500k?
    //    rf_reg_write(0x41,0x0191);
    rf_reg_write(0xc7,0x0000);
    rf_reg_write(0xc8,0x0200);
}


