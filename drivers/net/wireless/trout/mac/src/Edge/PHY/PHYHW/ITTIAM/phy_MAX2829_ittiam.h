/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : phy_MAX2829_ittiam.h                                 */
/*                                                                           */
/*  Description       : This file contains PHY register values for Ittiam    */
/*                      PHY to be used with Maxim MAX2829 RF chipset         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PHY_MAX2829_ITTIAM_H
#define PHY_MAX2829_ITTIAM_H

#ifdef  ITTIAM_PHY
#ifdef MAX2829

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
//#include "phy_ittiam.h"	//marked by chengwg, 0813


/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/*****************************************************************************/
/*  PHY - 11G register initialization values definitions                     */
/*****************************************************************************/

#define PHYIMPLEMENTATIONREG                        0x00
#define PHYMSBVERNUMREG                             0x00
#define PHYLSBVERNUMREG                             0x00
#define PHYCNTRLREG                                 0x20
#define DIGHPFCOEFFLSB                              0xE8
#define DIGHPFCOEFFMSB                              0x00

/*****************************************************************************/
/*  PHY - 11B register initialization values definitions                     */
/*****************************************************************************/

#define PHYBTXCNTRL                                 0x90
#define PHYBTXLENH                                  0x00
#define PHYBTXLENL                                  0xFF
#define PHYBRXCNTRL                                 0x60
#define PHYBRXSTA                                   0x00
#define PHYBRXLENH                                  0x00
#define PHYBRXLENL                                  0x00
#define PHYBACQTHRESH                               0xd4
#define PHYBACQTIMEOUT                              0x29
#define PHYBSNRUPTHRESHRSW                          0xE6
#define PHYBSNRLOTHRESHRSW                          0xE6
#define PHYBMPRTHRESHRSW                            0x00
#define PHYBTSTMODCTRL1                             0x00

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYBTSTMODCTRL2                             0x8C //0x00 //wuranqing 0825
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYBTSTMODCTRL2                             0x8C //0x00 //wuranqing 0825
#endif

#ifdef TROUT2_WIFI_IC
#define PHYBTSTMODCTRL2                             0x40 //0x8C //0x00 //wuranqing 0825
#endif

#ifdef TROUT_WIFI_EVB
#define PHYBTSTMODCTRL2                             0x00 //wuranqing 1102
#endif

#define PHYBRXPLCPDLY                               0x40
#define PHYCSBARKERTH1REG1                          0x2A
#define PHYCSBARKERTH1REG2                          0xBE
#define PHYCSBARKERTH1REG3                          0xE6
#define PHYCSBARKERTH1REG4                          0x1C
#define PHYCSBARKERTH1REG5                          0x84
#define PHYCSBARKERTH1REG6                          0x0C
#define PHYCSBARKERTH2REG1                          0x70
#define PHYCSBARKERTH2REG2                          0x35
#define PHYCSBARKERTH2REG3                          0x49
#define PHYCSBARKERTH2REG4                          0xD2
#define PHYCSBARKERTH2REG5                          0xB4
#define PHYCSBARKERTH2REG6                          0x28
#define PHYCSBARKERTH2REG7                          0x1A
#define PHYRAKEESTDELAY                             0x00
#define PHYBHDRPLCPDLY                              0x02
#define PHYBANTIPHASEHANGUP                         0x05
#define PHYBDIGHPFCOEFFLSB                          0xFF

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYBTXSCALEFACT                             0x3F
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYBTXSCALEFACT                             0x3F
#endif

#ifdef TROUT2_WIFI_IC
#define PHYBTXSCALEFACT                             0x3F
#endif

#ifdef TROUT_WIFI_EVB
#define PHYBTXSCALEFACT                             0x7F //0x3F
#endif

#define DSSSTXPREFILTCOEFFLSB                       0x00
#define DSSSTXPREFILTCOEFFMSB                       0x00
#ifdef PHY_11B_ANT_DIV
#define ANTDIVERSTYCTRL                             0x50
#else /* PHY_11B_ANT_DIV */
#define ANTDIVERSTYCTRL                             0x10
#endif /* PHY_11B_ANT_DIV */
#define ANTSWITCHDUR                                0x50
#define ANTIDLESTATEWAIT                            0x01
#define PHY11BMODECTRL                              0x00
#define ADRSSICOMPFACTOR                            0x51
#define RAKESWTMULTIPATHCNT                         0x11

/*****************************************************************************/
/*  PHY - 11A register initialization values definitions                     */
/*****************************************************************************/

#define PHYATXCON                                   0x00
#define PHYATXLENH                                  0x00
#define PHYATXLENL                                  0xFF
#define PHYATXSERFLD                                0x00
#define PHYATXSAMPLESCALEFACTOR                     0xFF
#define PHYARXCTL                                   0x7C
#define PHYARXSTS                                   0x00
#define PHYARXSERFLD                                0x00
#define PHYARXLENH                                  0x00
#define PHYARXLENL                                  0x00
#define PHYAPKTDETTHRESH                            0x12
#define PHYAPKTDETCNT1                              0x06
#define PHYAPKTDETCNT2                              0xB0
#define PHYALPDETTH1                                0x53
#define PHYALPDETTH2                                0x22
#define PHYALPDETCNT                                0xB3
#define PHYASNRTH1                                  0x00
#define PHYASNRTH2                                  0x00
#define PHYAUNLOCKTIMECORRCTRL                      0x08
#define PHYATSTMODCTRL1                             0x00//wuranqing add 0X48 //0x04 Hugh// 0x00
#define PHYATSTMODCTRL2                             0x00
#define PHYATSTMODCTRL3                             0x00
#define PHYARXPLCPDLY                               0xB5
#define PHYALOCKCONVFACTLSB                         0xB7
#define PHYALOCKCONVFACTMSB                         0x02
#define PHYAMAXRXLENGTHMSB                          0x0F
#define PHYAMAXRXLENGTHLSB                          0xFF
#define UNLOCKSAMPREG1                              0xD1
#define UNLOCKSAMPREG2                              0xA4
#define UNLOCKSAMPREG3                              0x3E
#define CFEBIASPOS                                  0x00
#define CFEBIASNEG                                  0x00
#define FASTCSWIN                                   0x08
#define PHYATXSCALEFACT                             0x7F
#define PHYADIGHPFCOEFFLSB                          0xE8
#define PHYADIGHPFCOEFFMSB                          0x00

#define PHYRADARCNTRLREG1                           0x00 //0xB7
#define PHYRADARCNTRLREG2                           0xBE
#define PHYRADARCNTRLREG3                           0x37
#define PHYRADARCNTRLREG4                           0xDA
#define PHYRADARCNTRLREG5                           0x78
#define PHYRADARCNTRLREG6                           0x10
#define PHYRADARCNTRLREG7                           0xA8
#define PHYRADARCNTRLREG8                           0xBC
#define PHYRADARCNTRLREG9                           0x52
#define PHYRADARCNTRLREG10                          0x78

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!! HACK ALERT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/* CIR Estimation scheme is "Averaging" as per design but                    */
/* But "HT Coefficient" is used in AP/STA mode for CFE drift issue with      */
/* Atheros devices. Need to be fixed in PHY                                  */
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
//#ifdef MAC_HW_UNIT_TEST_MODE
//#define RXCNTRLREG11N                               0x00
//#else /* MAC_HW_UNIT_TEST_MODE */
//#define RXCNTRLREG11N                               0x01
//#endif /* MAC_HW_UNIT_TEST_MODE */

#define RXCNTRLREG11N                               0x65 // 0x01 //0x65 //0x00
#define PREFFTDELAY                                 0x0A
#define FFTDELAY                                    0x64
/*****************************************************************************/
/*  PHY - RF register initialization values definitions                      */
/*****************************************************************************/

#ifdef LEFT_ANTENNA_SELECTED
#define PHYRFCNTRL_START_FREQ_5                     0xC7  /* 5GHz */
#define PHYRFCNTRL_START_FREQ_2                     0xC3  /* 2.4GHz */
#else /* LEFT_ANTENNA_SELECTED */
#define PHYRFCNTRL_START_FREQ_5                     0x47 /*  5GHz */
#define PHYRFCNTRL_START_FREQ_2                     0x43 /*  2.4 GHz */
#endif /* LEFT_ANTENNA_SELECTED */

/* Power up and power down time for Rf Tx and Rx control signals */

#ifdef AUTORATE_PING
#define PHYTXPWUPTIME1                              0x26 //ping.jiang modify for throughput 0x00->0x26 2014-01-10 
#else
#define PHYTXPWUPTIME1                              0x00
#endif /* AUTORATE_PING */

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYTXPWUPTIME2                              0x2a  // tuned //0x50  // 0x68
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYTXPWUPTIME2                              0x2a  // tuned //0x50  // 0x68
#endif

#ifdef TROUT2_WIFI_IC
#define PHYTXPWUPTIME2                              0x2a  // tuned //0x50  // 0x68
#endif

#ifdef TROUT_WIFI_EVB
#define PHYTXPWUPTIME2                              0x8F  //0x2a  // tuned //0x50  // 0x68
#endif

#define PHYTXPWUPTIME3                              0x24  // tuned //0x51  // 0x79
#define PHYTXPWUPTIME4                              0x33  // tuned //0x83  // 0x9B
#define PHYTXPWUPTIME5                              0x00
#define PHYRXPWUPTIME1                              0x00
#define PHYRXPWUPTIME2                              0x00
#define PHYRXPWUPTIME3                              0x14
#define PHYRXPWUPTIME4                              0x00
#define PHYRXPWUPTIME5                              0x00
#define PHYTXPWDNTIME1                              0x00
#define PHYTXPWDNTIME2                              0x00
#define PHYTXPWDNTIME3                              0x08
#define PHYTXPWDNTIME4                              0x08
#define PHYTXPWDNTIME5                              0x00
#define PHYRXPWDNTIME1                              0x00
#define PHYRXPWDNTIME2                              0x00
#define PHYRXPWDNTIME3                              0x00
#define PHYRXPWDNTIME4                              0x00
#define PHYRXPWDNTIME5                              0x00


/* Polarity of the RF control signals */
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYTXPOLCNTRL                               0x0D//0x81 //0x01 //wuranqing 0825
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYTXPOLCNTRL                               0x0D//0x81
#endif

#ifdef TROUT2_WIFI_IC
#define PHYTXPOLCNTRL                               0x0D//0x81
#endif

#ifdef TROUT_WIFI_EVB
#define PHYTXPOLCNTRL                               0x0D//0x81 //dumy add 0123

#endif

#define PHYRXPOLCNTRL                               0xC1

#define PHYAGCTIMEOUT                               0xD0

/* AGC related register values */
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYAGCCNTRL                                 0x98
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYAGCCNTRL                                 0x98
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCNTRL					                0x68 //SINGLE_LINE_AGC by wlh 20130329
#endif

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define PHYAGCCNTRL                                 0x68 //2013-02-05 wanglihuai //0xB8
#else
#define PHYAGCCNTRL        							0xB8
#endif
#endif

#define PHYAGCADDTNGAIN                             0x00

#ifdef TROUT2_WIFI_IC
#define PHYAGCINITWAITPKTSETTIME                    0x57 
#else
#define PHYAGCINITWAITPKTSETTIME                    0xA5
#endif

#define PHYAGCMAXVGAGAINUPTHR                       0x1C //0x1F

#define PHYAGCMAXVGAGAIN_START_FREQ_5               0x19 //0x1A /* 5GHz   */

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYAGCMAXVGAGAIN_START_FREQ_2               0x1A //0x1C /* 2.4GHz */
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYAGCMAXVGAGAIN_START_FREQ_2               0x1A //0x1C /* 2.4GHz */
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCMAXVGAGAIN_START_FREQ_2               0x1B  /* 2.4GHz */
#endif

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define PHYAGCMAXVGAGAIN_START_FREQ_2               0x19// 2013-02-05 wanglihuai //0x14 //0x19 /* 2.4GHz */
#else
#define PHYAGCMAXVGAGAIN_START_FREQ_2               0x11 /* 2.4GHz */
#endif
#endif

#define PHYAGCMAXVGAGAINLOWTHR                      0x16

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYAGCLNAGAIN                               0x1b
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYAGCLNAGAIN                               0x1b
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCLNAGAIN						        0x34
#endif

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define PHYAGCLNAGAIN                               0x34 //2013-02-05 wanglihuai //0x00
#else
#define PHYAGCLNAGAIN                               0x00
#endif
#endif

#ifndef ANALOG_RSSI
#define PHYAGCPKTDETCNTRL1                          0x01 //0x07  
#define PHYAGCPKTDETCNTRL2                          0x24
#else /* ANALOG_RSSI */
#define PHYAGCPKTDETCNTRL1                          0x04
#define PHYAGCPKTDETCNTRL2                          0x12
#endif /* ANALOG_RSSI */

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCVGAGAINAFTERPKTDET                    0x12
#else
#define PHYAGCVGAGAINAFTERPKTDET                    0x0F
#endif/*TROUT2_WIFI_FPGA_RF55*/
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCVGAGAINAFTERPKTDET				    0x0F
#endif


#ifdef TROUT_WIFI_EVB
#if  defined(TROUT_WIFI_EVB_MF)
#define PHYAGCVGAGAINAFTERPKTDET                    0x0A //2013-02-05 wanglihuai
#elif defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCVGAGAINAFTERPKTDET                    0x12
#else
#define PHYAGCVGAGAINAFTERPKTDET                    0x0F
#endif
#endif

#ifdef TROUT_WIFI_FPGA
#define PHYAGCVGAGAINAFTERPKTDET                    0x0F
#endif/*TROUT2_WIFI_FPGA*/

#define PHYAGCLNAHITOLOVGACOMP_START_FREQ_5         0x11 /* 5GHz */

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCLNAHITOLOVGACOMP_START_FREQ_2         0x0F//0x10 /* 2.4GHz */
#else
#define PHYAGCLNAHITOLOVGACOMP_START_FREQ_2         0x10 /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA_RF55*/
#endif

#if defined(TROUT_WIFI_EVB) && defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCLNAHITOLOVGACOMP_START_FREQ_2         0x0F//0x10 /* 2.4GHz */
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCLNAHITOLOVGACOMP_START_FREQ_2         0x0F//0x10 /* 2.4GHz */
#else
#define PHYAGCLNAHITOLOVGACOMP_START_FREQ_2         0x10 /* 2.4GHz */
#endif

#define PHYAGCLNAHITOMIDVGACOMP_START_FREQ_5        0x09 /* 5GHz */

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCLNAHITOMIDVGACOMP_START_FREQ_2        0x05 /* 2.4GHz */
#else
#define PHYAGCLNAHITOMIDVGACOMP_START_FREQ_2        0x08 /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA_RF55*/
#endif

#if defined(TROUT_WIFI_EVB) && defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCLNAHITOMIDVGACOMP_START_FREQ_2        0x05 /* 2.4GHz */
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCLNAHITOMIDVGACOMP_START_FREQ_2        0x05 /* 2.4GHz */
#else
#define PHYAGCLNAHITOMIDVGACOMP_START_FREQ_2        0x08 /* 2.4GHz */
#endif

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define PHYAGCCORS1UPWINCTRL                        0x1F //2013-02-05 wanglihuai
#else
#define PHYAGCCORS1UPWINCTRL                        0x0D
#endif
#endif

#ifdef  TROUT2_WIFI_IC
#define PHYAGCCORS1UPWINCTRL                        0xA // weihua 0425 by wlh change //0x1F(ranqing)//0x0F (old config)
#else
#define PHYAGCCORS1UPWINCTRL                        0x0D
#endif

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define PHYAGCCORS1LOWINCTRL                        0x80 //2013-02-05 wanglihuai
#else
#define PHYAGCCORS1LOWINCTRL                        0x8D
#endif
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS1LOWINCTRL						0x80 //0x4C(old) change by wlh 0425
#else
#define PHYAGCCORS1LOWINCTRL                        0x8D
#endif

#define PHYAGCCORS1LNAHITOLOCTRL                    0x1F
#define PHYAGCCORS1LNAHITOMIDCTRL                   0x1F

#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_5        0x01 /* 5GHz */

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2        0x05 /* 2.4GHz */
#else/*TROUT2_WIFI_FPGA_RF55*/
#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2        0x01 /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA_RF2829*/
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2        0x09 /* 2.4GHz */
#endif

#ifdef TROUT_WIFI_EVB
#if defined(TROUT_WIFI_EVB_MF)
#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2        0x04 /* 2.4GHz */
#elif defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2        0x05 /* 2.4GHz */
#else
#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2        0x01 /* 2.4GHz */
#endif
#endif

#ifdef TROUT_WIFI_FPGA
#define PHYAGCCORS1UPWINVGAGAIN_START_FREQ_2        0x01 /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA*/

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCCORS1LOWINVGAGAIN                     0x14
#else/*TROUT2_WIFI_FPGA_RF55*/
#define PHYAGCCORS1LOWINVGAGAIN                     0x1A
#endif/*TROUT2_WIFI_FPGA_RF2829*/
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS1LOWINVGAGAIN                     0x1B
#endif

#ifdef TROUT_WIFI_EVB
#if defined(TROUT_WIFI_EVB_MF)
#define PHYAGCCORS1LOWINVGAGAIN                     0x12
#elif  defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCCORS1LOWINVGAGAIN                     0x14
#else
#define PHYAGCCORS1LOWINVGAGAIN                     0x1A
#endif
#endif

#ifdef TROUT_WIFI_FPGA
#define PHYAGCCORS1LOWINVGAGAIN                     0x1A
#endif/*TROUT_WIFI_FPGA*/

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS1SETTIMECTRL1 					0x55
#define PHYAGCCORS1SETTIMECTRL2 					0x55
#else
#define PHYAGCCORS1SETTIMECTRL1                     0xF5
#define PHYAGCCORS1SETTIMECTRL2                     0xD0
#endif/*TROUT2_WIFI_IC*/

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define PHYAGCCORS1NXTSTATECTRL                     0x33
#else
#define PHYAGCCORS1NXTSTATECTRL                     0x32
#endif
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS1NXTSTATECTRL 					0x33
#else
#define PHYAGCCORS1NXTSTATECTRL                     0x32
#endif/*TROUT2_WIFI_IC*/

#define PHYAGCCORSNXTSTATECTRL                      0x33

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORSLNAGAINCTRL                       0x00
#else
#define PHYAGCCORSLNAGAINCTRL                       0x07
#endif/*TROUT2_WIFI_IC*/

#define PHYAGCCORS2UPWINCTRL_START_FREQ_5           0x16 /* 5GHz */

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCCORS2UPWINCTRL_START_FREQ_2           0x0D /* 2.4GHz */
#else
#define PHYAGCCORS2UPWINCTRL_START_FREQ_2           0x0D /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA_RF55*/
#endif

#if defined(TROUT_WIFI_EVB) && defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCCORS2UPWINCTRL_START_FREQ_2           0x0D /* 2.4GHz */
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS2UPWINCTRL_START_FREQ_2           0x13//0x0D /* 2.4GHz */
#else
#define PHYAGCCORS2UPWINCTRL_START_FREQ_2           0x0D /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA*/

#define PHYAGCCORS2LOWINCTRL                        0x00

#define PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_5       0x16 /* 5GHz */

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_2       0x0D /* 2.4GHz */
#else
#define PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_2       0x0D /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA_RF55*/
#endif

#if defined(TROUT_WIFI_EVB) && defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_2       0x0D /* 2.4GHz */
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_2       0x2d //0x0D /* 2.4GHz */
#else 
#define PHYAGCCORS2LNAHITOLOCTRL_START_FREQ_2       0x0D /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA*/

#define PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_5      0x7C /* 5GHz */

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_2      0x1C /* 2.4GHz */
#else
#define PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_2      0x7C /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA_RF55*/
#endif

#if defined(TROUT_WIFI_EVB) && defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_2      0x1C /* 2.4GHz */
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_2      0x7c//0x1C /* 2.4GHz */
#else
#define PHYAGCCORS2LNAHITOMIDCTRL_START_FREQ_2      0x7C /* 2.4GHz */
#endif/*TROUT2_WIFI_FPGA*/

#define PHYAGCCORS2UPWINVGAGAIN_START_FREQ_5        0x0A /* 5GHz */
#define PHYAGCCORS2UPWINVGAGAIN_START_FREQ_2        0x08 /* 2.4GHz */


#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF55
#define PHYAGCCORS2LOWINVGAGAIN                     0x14
#else
#define PHYAGCCORS2LOWINVGAGAIN                     0x00
#endif/*TROUT2_WIFI_FPGA_RF55*/
#endif

#if defined(TROUT_WIFI_EVB) && defined(TROUT_WIFI_EVB_MF_RF55)
#define PHYAGCCORS2LOWINVGAGAIN                     0x14
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORS2LOWINVGAGAIN                     0x00 //0x14
#else
#define PHYAGCCORS2LOWINVGAGAIN                     0x00
#endif/*TROUT2_WIFI_IC*/

#define PHYAGCCORS2SETTIMECTRL1                     0x0D
#define PHYAGCCORS2SETTIMECTRL2                     0xDD

#define PHYAGCCORS2NXTSTATECTRL                     0x33
#define PHYAGCCORSFINEAVGWINCTRL1                   0x53

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYAGCCORSFINEAVGWINCTRL2                   0x00

#define PHYAGCFINEUPWINCTRL                         0x19

#define PHYAGCFINE1CTRL                             0x47
#define PHYAGCFINE2CTRL                             0x00
#define PHYAGCFINE3CTRL                             0x00

#define PHYAGCFINALVGAGAINTH_START_FREQ_5           0x17 //0x19 /* 5GHz   */
#define PHYAGCFINALVGAGAINTH_START_FREQ_2           0x19 //0x1B /* 2.4GHz */

#define AGCLNAMIDLOWCTRL1                           0x0D //0x00
#define AGCLNAMIDLOWCTRL2                           0x0C
#define AGCLNAMIDLOWCTRL3                           0x08
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYAGCCORSFINEAVGWINCTRL2                   0x00

#define PHYAGCFINEUPWINCTRL                         0x19

#define PHYAGCFINE1CTRL                             0x47
#define PHYAGCFINE2CTRL                             0x00
#define PHYAGCFINE3CTRL                             0x00

#define PHYAGCFINALVGAGAINTH_START_FREQ_5           0x17 //0x19 /* 5GHz   */
#define PHYAGCFINALVGAGAINTH_START_FREQ_2           0x19 //0x1B /* 2.4GHz */

#define AGCLNAMIDLOWCTRL1                           0x0D //0x00
#define AGCLNAMIDLOWCTRL2                           0x0C
#define AGCLNAMIDLOWCTRL3                           0x08
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCCORSFINEAVGWINCTRL2                   0x55
#define PHYAGCFINEUPWINCTRL                         0x1f
#define PHYAGCFINE1CTRL                             0x3A  //0x47 last modify on 1012.11.5
#define PHYAGCFINE2CTRL                             0x4A//0x37  //0x00 last modify on 1012.11.5
#define PHYAGCFINE3CTRL                             0x47  //0x00 last modify on 1012.11.5
#define AGCLNAMIDLOWCTRL3                           0x0a  //0x08 last modify on 1012.11.5

#define PHYAGCFINALVGAGAINTH_START_FREQ_5           0x17 //0x19 /* 5GHz   */
#define PHYAGCFINALVGAGAINTH_START_FREQ_2           0x1B  /* 2.4GHz */

#define AGCLNAMIDLOWCTRL1                           0x0D //0x00
#define AGCLNAMIDLOWCTRL2                           0x0C
#endif/*TROUT2_WIFI_IC*/

#ifdef TROUT_WIFI_EVB
#define PHYAGCCORSFINEAVGWINCTRL2                   0x55

#define PHYAGCFINEUPWINCTRL                         0x1F

#define PHYAGCFINE1CTRL                             0x37

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define PHYAGCFINE2CTRL                             0x47
#else
#define PHYAGCFINE2CTRL                             0x37
#endif

#else
#define PHYAGCFINE2CTRL                             0x37
#endif

#define PHYAGCFINE3CTRL                             0x47

#define PHYAGCFINALVGAGAINTH_START_FREQ_5           0x17 //0x19 /* 5GHz   */
#define PHYAGCFINALVGAGAINTH_START_FREQ_2           0x19 //0x1B /* 2.4GHz */

#define AGCLNAMIDLOWCTRL1                           0x1C //0x00
#define AGCLNAMIDLOWCTRL2                           0x0C
#ifdef TROUT_WIFI_EVB_MF
#define AGCLNAMIDLOWCTRL3                           0x0A
#else
#define AGCLNAMIDLOWCTRL3                           0x11 // 0x14 //dumy add for rx gain 1108
#endif
#endif

#define AGCLNAMIDLOWAVGWINCTRL1                     0x54
#define AGCLNAMIDLOWAVGWINCTRL2                     0x00
#define AGCMIDCOARSE1UPPERWINCTRL                   0x13
#define AGCMIDCOARSE1LOWERWINCTRL                   0x8D
#define AGCMIDCOARSE1LNAMIDTOLOWCTRL                0x13
#define AGCMIDCOARSE1UPPERWINVGAGAIN                0x01
#define AGCMIDCOARSE1LOWERWINVGAGAIN                0x1A
#define AGCMIDCOARSELNAGAINCTRL                     0x05
#define AGCMIDCOARSE1SETTLETIMECTRL1                0xF5
#define AGCMIDCOARSE1SETTLETIMECTRL2                0xD0
#define AGCMIDCOARSE1NXTSTATECTRL                   0x32
#define AGCMIDCOARSENXTSTATECTRL                    0x33
#define AGCMIDCOARSE2UPPERWINCTRL                   0x1F
#define AGCMIDCOARSE2LOWERWINCTRL                   0x8D
#define AGCMIDCOARSE2LNAMIDTOLOWCTRL                0x1F
#define AGCMIDCOARSE2UPPERWINVGAGAIN                0x01
#define AGCMIDCOARSE2LOWERWINVGAGAIN                0x08
#define AGCMIDCOARSE2SETTLETIMECTRL1                0xF5
#define AGCMIDCOARSE2SETTLETIMECTRL2                0xD0
#define AGCMIDCOARSE2NXTSTATECTRL                   0x33
#define AGCLOWFINE1CTRL                             0x47
#define AGCLOWFINE2CTRL                             0x00
#define AGCLOWFINE3CTRL                             0x00

#define AGCRELOCKUPMARGIN                           0x05
#define AGCRELOCKLOWMARGIN                          0x05
#define AGCRELOCKCHECKTIME                          0x50 //0x4A

#define TXREADYDELAY                                0x00
#define RXREADY11ADELAY                             0x5c // Tuned 0x6E
#define RXREADY11BDELAY                             0x28 // Tuned 0x24
#define SWRESETCTRL                                 0x00


#define PHYAGCULCTRL1                               0x6D //0x65
#define PHYAGCULCTRL2                               0x62
#define PHYAGCULUPTH                                0x10

#ifndef ANALOG_RSSI
#define PHYAGCULLOTH                                0x02 //0x0A
#else /* ANALOG_RSSI */
#define PHYAGCULLOTH                                0xFF
#endif /* ANALOG_RSSI */

#define PHYAGCULSETTIME                             0xA5
#define PHYAGCMAXANDFINETOCORSTIME                  0x5A

#define PHYAGCVGAGAINBACKOFF                        0x00
#define PHYAGCABSULUPTH                             0x70 //0xFF

//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define PHYAGCABSULLOTH                             0x78 //0x00
#endif

#ifdef TROUT2_WIFI_FPGA
#define PHYAGCABSULLOTH                             0x78 //0x00
#endif

#ifdef TROUT2_WIFI_IC
#define PHYAGCABSULLOTH                             0x20 //0x00
#endif

#ifdef TROUT_WIFI_EVB
#define PHYAGCABSULLOTH                             0x20 //0x00
#endif

/* AGC Test mode register */
#define PHYTMAGCGAIN                                0x00
#define PHYTMAGCGAINSETTIME                         0x00

/* CCA related registers */
#define PHYCCACONTROL                               0x47
#define PHYCCACONSTVGAMC1                           0x10 //0x10 //dumy add for debug 0820 ED&CS detect
#define PHYCCACONSTVGAMC2                           0x00
#define PHYCCACONSTVGAMC3                           0x00

#define PHYCCAMACTH1_START_FREQ_5                   0xAE /* 5GHz */
#define PHYCCAMACTH1_START_FREQ_2                   0xB4 /* 2.4GHz */

#define PHYCCAMACTH2_START_FREQ_5                   0xC2 /* 5GHz */
#define PHYCCAMACTH2_START_FREQ_2                   0xB4 /* 2.4GHz */

#define PHYCCAPHYTH                                 0xA1

#define PHYCCAINSERTIONVAL_START_FREQ_5             0xE8 /* 5GHz */
#define PHYCCAINSERTIONVAL_START_FREQ_2             0x14  //0xE7  2.4GHz  0425 weihua

#define PHYCCAERMARGIN1                             0xFE
#define PHYCCAERMARGIN2                             0xFE
#define PHYCCASTATUS                                0x00

#define PHYUNLOCKCKECKCNTR                          0x18

/* Scale factor in the 11b path */
#define PHYRXSCALEFACTMSB                           0x05
#define PHYRXSCALEFACTLSB                           0xBD

/* Power save  and Analog RSSI register */
#define PHYPOWDNCNTRL                               0x32

/* AGC SP RAM regs are initialised later */
#define AGCSPRAMLSB                                 0x00
#define AGCSPRAMMSB                                 0x00

#ifdef ANALOG_RSSI
#define ERSSI_EN                                    0x53
#else /* ANALOG_RSSI */
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define ERSSI_EN                                    0x42
#endif

#ifdef TROUT2_WIFI_FPGA
#define ERSSI_EN                                    0x42
#endif

#ifdef TROUT2_WIFI_IC
#define ERSSI_EN                                    0x42
#endif/*TROUT2_WIFI_IC*/

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF
#define ERSSI_EN                                    0x42 //2013-02-05 wanglihuai //0x43
#else
#define ERSSI_EN                                    0x43
#endif
#endif

#endif /* ANALOG_RSSI */

#define ERSSI_LTH                                   0x18
#define ERSSI_HTH                                   0x1D

#ifdef ANALOG_RSSI
#define ERSSI_AGC_EN                                0xC1
#else /* ANALOG_RSSI */
#define ERSSI_AGC_EN                                0x41
#endif /* ANALOG_RSSI */

#define ERSSI_PKT_DET_TIM_CTRL1                     0x55
#define ERSSI_PKT_DET_TIM_CTRL2                     0x05

#define ERSSI_SAT_NXT_ST_LNA                        0x82
#define ERSSI_SAT_TH                                0x1F
#define ERSSI_MC1                                   0x19
#define ERSSI_MC2                                   0x81
#define ERSSI_MC3                                   0xA8
#define ERSSI_OFSET_ML1                             0x80
#define ERSSI_OFSET_ML2                             0x00
#define ERSSI_OFSET_ML3                             0x00 //0x08 //modify0525
#define ERSSI_PKT_DET_ITR_2_CTRL                    0x03

//////  802.11n >6.5M no work
//#define TXPOWER11A                                  0x31		//802.11g -65dm
//#define TXPOWER11B                                  0x27

//modify by liusy on 2012/07/12, new  ----802.11n work
//#define TXPOWER11A                                  0x6F
//#define TXPOWER11B                                  0x4F

//modify by liusy on 2012/07/12, new -----test 802.11b
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define TXPOWER11A                                 0x29// 0x3F//0x29 //dumy add 0803 from wuranqing
#define TXPOWER11B                                 0x30// 0x3F//0x30 //dumy add 0803 from wuranqing
#endif

#ifdef TROUT2_WIFI_FPGA
#define TXPOWER11A                                 0x29// 0x3F//0x29 //dumy add 0803 from wuranqing
#define TXPOWER11B                                 0x30// 0x3F//0x30 //dumy add 0803 from wuranqing
#endif

#ifdef TROUT2_WIFI_IC
#define TXPOWER11A                                 0x30 //0x29// 0x3F//0x29 //dumy add 0803 from wuranqing
#define TXPOWER11B                                 0x00 //0x30// 0x3F//0x30 //dumy add 0803 from wuranqing
#endif

#ifdef TROUT_WIFI_EVB
#ifdef TROUT_WIFI_EVB_MF //dumy add 0123
#define TXPOWER11A                                 0x29 
#define TXPOWER11B                                 0x30
#else
#define TXPOWER11A                                 0x7F
#define TXPOWER11B                                 0x7F
#endif
#endif

/* DC offset related register */
#define TXDCOI                                      0x00
#define TXDCOQ                                      0x00
#define TXPOSTFILTCOEFFLSB                          0x00
#define TXPOSTFILTCOEFFMSB                          0x00

#define RXHPSWITCHDLY                               0x28

#define RXDIGHPFCNTRL                               0x01 //0x03 (0x03 -> 11a radar)

#define RSSIMSB                                     0x00
#define RSSILSB                                     0x00
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_FPGA
#define TXPOWCALCNTRL                               0x01 //0x09//wuranqing 0825
#endif

#ifdef TROUT2_WIFI_FPGA
#define TXPOWCALCNTRL                               0x01 //0x09//wuranqing 0825
#endif

#ifdef TROUT2_WIFI_IC
//#define TXPOWCALCNTRL                               0x01 //0x09//wuranqing 0825
#define TX_POW_SEL_HDR                             (0UL << 3) // set TX_POW_SEL_HDR to 1.  zhongli.wang 20130905
#define TXPOWCALCNTRL                               (0x01 | TX_POW_SEL_HDR) // 

#endif/*TROUT2_WIFI_IC*/

#ifdef TROUT_WIFI_EVB
#define TXPOWCALCNTRL                               0x01 //0x09//wuranqing 1104
#endif

#define TXPOWCALDLY                                 0x21

#define ADCDACTSTMODE                               0xC8

#define POWDET11A                                   0x00
#define POWDET11B                                   0x00
#define TXPOWERCALAVGWIN                            0x03


#define PHYRFCNTRL3                                 0x09 //0x01
#define AGCTRIGSTATLSB                              0x20  //0x00
#define AGCTRIGSTATMSB                              0xa4 //0x00
#define AGCSETTLEDVGAGAIN                           0x1d //0x00
#define AGCSETTLEDLNAGAIN                           0x00

#define RXDCOI                                      0x00
#define RXDCOQ                                      0x00

#define TXIQIMBREG1                                 0x00
#define TXIQIMBREG2                                 0x7F

#define RXIQIMBREG1                                 0x00
#define RXIQIMBREG2                                 0x00
#define RXIQIMBREG3                                 0x00

#define SPIDATA                                     0x00
#define SPICTRL                                     0x00
#define MACIFCTRL                                   0x006e6e06 //Tuned 0x008c8c06
#define MACIFCTRL2                                  0x00fd7e7e

#define MONITORSEL                                  0x00


/*****************************************************************************/
/* Test setting for measuring sensitivity of lower data rates.               */
/* NOT to be used over the air.                                              */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

/* test setting for measuring sensitivity of lower data rates
not to be used over the air */
#undef PHYAGCMAXVGAGAIN_START_FREQ_5
#undef PHYAGCMAXVGAGAIN_START_FREQ_2
#undef PHYAGCFINALVGAGAINTH_START_FREQ_5
#undef PHYAGCFINALVGAGAINTH_START_FREQ_2

#define PHYAGCMAXVGAGAIN_START_FREQ_5               0x1A /* 5GHz */
#define PHYAGCMAXVGAGAIN_START_FREQ_2               0x1C /* 2.4GHz */

#define PHYAGCFINALVGAGAINTH_START_FREQ_5           0x19 /* 5GHz */
#define PHYAGCFINALVGAGAINTH_START_FREQ_2           0x1B /* 2.4GHz */

#ifdef ANALOG_RSSI
#undef PHYAGCPKTDETCNTRL1
#define PHYAGCPKTDETCNTRL1                          0x02
#endif /* ANALOG_RSSI */

#endif /* MAC_HW_UNIT_TEST_MODE */

#endif /* MAX2829 */
#endif /* ITTIAM_PHY */
#endif /* PHY_MAX2829_ITTIAM_H */
