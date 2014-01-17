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
/*  File Name         : phy_MAX2830_32_ittiam.h                              */
/*                                                                           */
/*  Description       : This file contains PHY register values for Ittiam    */
/*                      PHY to be used with Maxim MAXIM 2830-32 RF chipset   */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PHY_MAX2830_32_ITTIAM_H
#define PHY_MAX2830_32_ITTIAM_H

#ifdef ITTIAM_PHY
#ifdef MAX2830_32

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "phy_ittiam.h"

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

#ifdef MXFE_22MHZ
    #define DIGHPFCOEFFLSB                          0xFF
#elif MXFE_44MHZ
    #define DIGHPFCOEFFLSB                          0xFF
#else /* MXFE_44MHZ */
    #define DIGHPFCOEFFLSB                          0xE8
#endif /* MXFE_22MHZ */

#define DIGHPFCOEFFMSB                              0x00

/*****************************************************************************/
/*  PHY - 11B register initialization values definitions                     */
/*****************************************************************************/

#ifndef LOCKED
    #ifdef MXFE_22MHZ
        #define PHYBTXCNTRL                         0x00
    #elif MXFE_44MHZ
        #define PHYBTXCNTRL                         0x00
    #else /* MXFE_44MHZ */
        #define PHYBTXCNTRL                         0x80
    #endif /* MXFE_22MHZ */
#else /* LOCKED */
    #ifdef MXFE_22MHZ
        #define PHYBTXCNTRL                         0x10
    #elif MXFE_44MHZ
        #define PHYBTXCNTRL                         0x10
    #else /* MXFE_44MHZ */
        #define PHYBTXCNTRL                         0x90
    #endif /* MXFE_22MHZ */
#endif /* LOCKED */

#define PHYBTXLENH                                  0x00
#define PHYBTXLENL                                  0xFF

#ifndef LOCKED
#define PHYBRXCNTRL                                 0x64
#else /* LOCKED */
#define PHYBRXCNTRL                                 0x60
#endif /* LOCKED */

#define PHYBRXSTA                                   0x00
#define PHYBRXLENH                                  0x00
#define PHYBRXLENL                                  0x00
#define PHYBACQTHRESH                               0x04
#define PHYBACQTIMEOUT                              0x29

#ifdef MXFE_22MHZ
#define PHYBSNRUPTHRESHRSW                          0xED
#define PHYBSNRLOTHRESHRSW                          0xED
#elif MXFE_44MHZ
#define PHYBSNRUPTHRESHRSW                          0xEC
#define PHYBSNRLOTHRESHRSW                          0xEC
#else /* MXFE_44MHZ */
#define PHYBSNRUPTHRESHRSW                          0xE3
#define PHYBSNRLOTHRESHRSW                          0xE3
#endif /* MXFE_22MHZ */

#define PHYBMPRTHRESHRSW                            0x1A
#define PHYBTSTMODCTRL1                             0x00
#define PHYBTSTMODCTRL2                             0x40

#ifdef MXFE_22MHZ
    #define PHYBRXPLCPDLY                           0x24
#elif MXFE_44MHZ
    #define PHYBRXPLCPDLY                           0x24
#else /* MXFE_44MHZ */
    #define PHYBRXPLCPDLY                           0x40
#endif /* MXFE_22MHZ */

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
#define PHYRAKEESTDELAY                             0x06
#define PHYBHDRPLCPDLY                              0x02
#define PHYBANTIPHASEHANGUP                         0x05
#define PHYBDIGHPFCOEFFLSB                          0xFF
#define PHYBTXSCALEFACT                             0x3F
#define DSSSTXPREFILTCOEFFLSB                       0x00
#define DSSSTXPREFILTCOEFFMSB                       0x00
#ifdef PHY_11B_ANT_DIV
#define ANTDIVERSTYCTRL                             0x50
#else /* PHY_11B_ANT_DIV */
#define ANTDIVERSTYCTRL                             0x10
#endif /* PHY_11B_ANT_DIV */

#ifdef MXFE_22MHZ
#define ANTSWITCHDUR                                0x2C
#define ANTIDLESTATEWAIT                            0x01
#elif MXFE_44MHZ
#define ANTSWITCHDUR                                0x2C
#define ANTIDLESTATEWAIT                            0x01
#else /* MXFE_44MHZ */
#define ANTSWITCHDUR                                0x50
#define ANTIDLESTATEWAIT                            0x01
#endif /* MXFE_22MHZ */

#define PHY11BMODECTRL                              0x00
#define ADRSSICOMPFACTOR                            0x51


/*****************************************************************************/
/*  PHY - 11A register initialization values definitions                     */
/*****************************************************************************/

#define PHYATXCON                                   0x00
#define PHYATXLENH                                  0x00
#define PHYATXLENL                                  0xFF
#define PHYATXSERFLD                                0x00
#define PHYATXSAMPLESCALEFACTOR                     0xFF

#ifndef LOCKED
#define PHYARXCTL                                   0xFC
#else /* LOCKED */
#define PHYARXCTL                                   0x7C
#endif /* LOCKED */

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
#define PHYATSTMODCTRL1                             0x00
#define PHYATSTMODCTRL2                             0x00
#define PHYATSTMODCTRL3                             0x00
#define PHYARXPLCPDLY                               0xA8
#define PHYALOCKCONVFACTLSB                         0xB7
#define PHYALOCKCONVFACTMSB                         0x02
#define PHYAMAXRXLENGTHMSB                          0x06
#define PHYAMAXRXLENGTHLSB                          0x40
#define UNLOCKSAMPREG1                              0xD1
#define UNLOCKSAMPREG2                              0xA4
#define UNLOCKSAMPREG3                              0x3E
#define CFEBIASPOS                                  0xF4
#define CFEBIASNEG                                  0xE4
#define FASTCSWIN                                   0x08
#define PHYATXSCALEFACT                             0x7F
#define PHYADIGHPFCOEFFLSB                          0xE8
#define PHYADIGHPFCOEFFMSB                          0x00
#define PHYRADARCNTRLREG1                           0xFC
#define PHYRADARCNTRLREG2                           0x51
#define PHYRADARCNTRLREG3                           0x6A
#define PHYRADARCNTRLREG4                           0x00 //0xD0 for enabling radar detection
#define PHYRADARCNTRLREG5                           0xBA
#define PHYRADARCNTRLREG6                           0x00

/*****************************************************************************/
/*  PHY - RF register initialization values definitions                      */
/*****************************************************************************/

#define PHYRFCNTRL                                  0x23

/* Power up and power down time for Rf Tx and Rx control signals */
#ifdef MXFE_22MHZ

#define PHYTXPWUPTIME1                              0x00
#define PHYTXPWUPTIME2                              0x42
#define PHYTXPWUPTIME3                              0x00
#define PHYTXPWUPTIME4                              0x00
#define PHYTXPWUPTIME5                              0x00
#define PHYRXPWUPTIME1                              0x00
#define PHYRXPWUPTIME2                              0x00
#define PHYRXPWUPTIME3                              0x0B
#define PHYRXPWUPTIME4                              0x00
#define PHYRXPWUPTIME5                              0x00
#define PHYTXPWDNTIME1                              0x00
#define PHYTXPWDNTIME2                              0x00
#define PHYTXPWDNTIME3                              0x05
#define PHYTXPWDNTIME4                              0x05
#define PHYTXPWDNTIME5                              0x00
#define PHYRXPWDNTIME1                              0x00
#define PHYRXPWDNTIME2                              0x00
#define PHYRXPWDNTIME3                              0x00
#define PHYRXPWDNTIME4                              0x00
#define PHYRXPWDNTIME5                              0x00

#elif MXFE_44MHZ

#define PHYTXPWUPTIME1                              0x00
#define PHYTXPWUPTIME2                              0x42
#define PHYTXPWUPTIME3                              0x00
#define PHYTXPWUPTIME4                              0x00
#define PHYTXPWUPTIME5                              0x00
#define PHYRXPWUPTIME1                              0x00
#define PHYRXPWUPTIME2                              0x00
#define PHYRXPWUPTIME3                              0x0B
#define PHYRXPWUPTIME4                              0x00
#define PHYRXPWUPTIME5                              0x00
#define PHYTXPWDNTIME1                              0x00
#define PHYTXPWDNTIME2                              0x00
#define PHYTXPWDNTIME3                              0x05
#define PHYTXPWDNTIME4                              0x05
#define PHYTXPWDNTIME5                              0x00
#define PHYRXPWDNTIME1                              0x00
#define PHYRXPWDNTIME2                              0x00
#define PHYRXPWDNTIME3                              0x00
#define PHYRXPWDNTIME4                              0x00
#define PHYRXPWDNTIME5                              0x00

#else /* MXFE_44MHZ */

#define PHYTXPWUPTIME1                              0x00
#define PHYTXPWUPTIME2                              0x78
#define PHYTXPWUPTIME3                              0x00
#define PHYTXPWUPTIME4                              0x00
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

#endif /* MXFE_22MHZ */

/* Polarity of the RF control signals */
#define PHYTXPOLCNTRL                               0x81
#define PHYRXPOLCNTRL                               0xC1

#define PHYAGCTIMEOUT                               0x8A

/* AGC related register values */
#define PHYAGCCNTRL                                 0x98
#define PHYAGCADDTNGAIN                             0x00

#ifdef MXFE_22MHZ
    #define PHYAGCINITWAITPKTSETTIME                0x63
#elif MXFE_44MHZ
    #define PHYAGCINITWAITPKTSETTIME                0x63
#else /* MXFE_44MHZ */
    #define PHYAGCINITWAITPKTSETTIME                0xA5
#endif /* MXFE_22MHZ */


#define PHYAGCMAXVGAGAINUPTHR                       0x1F
#define PHYAGCMAXVGAGAIN                            0x16
#define PHYAGCMAXVGAGAINLOWTHR                      0x16

#define PHYAGCLNAGAIN                               0x1b

#ifndef ANALOG_RSSI
    #define PHYAGCPKTDETCNTRL1                      0x07
    #define PHYAGCPKTDETCNTRL2                      0x24
#else /* ANALOG_RSSI */
    #define PHYAGCPKTDETCNTRL1                      0x04
    #define PHYAGCPKTDETCNTRL2                      0x13
#endif /* ANALOG_RSSI */

#define PHYAGCVGAGAINAFTERPKTDET                    0x0F
#define PHYAGCLNAHITOLOVGACOMP                      0x10
#define PHYAGCLNAHITOMIDVGACOMP                     0x08

#define PHYAGCCORS1UPWINCTRL                        0x0D
#define PHYAGCCORS1LOWINCTRL                        0x8D
#define PHYAGCCORS1LNAHITOLOCTRL                    0x1F
#define PHYAGCCORS1LNAHITOMIDCTRL                   0x1F
#define PHYAGCCORS1UPWINVGAGAIN                     0x01
#define PHYAGCCORS1LOWINVGAGAIN                     0x1A

#ifdef MXFE_22MHZ
    #define PHYAGCCORS1SETTIMECTRL1                 0x93
    #define PHYAGCCORS1SETTIMECTRL2                 0x80
#elif MXFE_44MHZ
    #define PHYAGCCORS1SETTIMECTRL1                 0x93
    #define PHYAGCCORS1SETTIMECTRL2                 0x80
#else /* MXFE_44MHZ */
    #define PHYAGCCORS1SETTIMECTRL1                 0xF5
    #define PHYAGCCORS1SETTIMECTRL2                 0xD0
#endif /* MXFE_22MHZ */

#define PHYAGCCORS1NXTSTATECTRL                     0x32

#define PHYAGCCORSNXTSTATECTRL                      0x33
#define PHYAGCCORSLNAGAINCTRL                       0x07

#define PHYAGCCORS2UPWINCTRL                        0x0D
#define PHYAGCCORS2LOWINCTRL                        0x00
#define PHYAGCCORS2LNAHITOLOCTRL                    0x0D
#define PHYAGCCORS2LNAHITOMIDCTRL                   0x7C
#define PHYAGCCORS2UPWINVGAGAIN                     0x07
#define PHYAGCCORS2LOWINVGAGAIN                     0x00

#ifdef MXFE_22MHZ
    #define PHYAGCCORS2SETTIMECTRL1                 0x08
    #define PHYAGCCORS2SETTIMECTRL2                 0x88
#elif MXFE_44MHZ
    #define PHYAGCCORS2SETTIMECTRL1                 0x08
    #define PHYAGCCORS2SETTIMECTRL2                 0x88
#else /* MXFE_44MHZ */
    #define PHYAGCCORS2SETTIMECTRL1                 0x0D
    #define PHYAGCCORS2SETTIMECTRL2                 0xDD
#endif /* MXFE_22MHZ */

#define PHYAGCCORS2NXTSTATECTRL                     0x33


#define PHYAGCCORSFINEAVGWINCTRL1                   0x53
#define PHYAGCCORSFINEAVGWINCTRL2                   0x44

#define PHYAGCFINEUPWINCTRL                         0x19

#ifdef MXFE_22MHZ
    #define PHYAGCFINE1CTRL                         0x44
    #define PHYAGCFINE2CTRL                         0x00
    #define PHYAGCFINE3CTRL                         0x00
#elif MXFE_44MHZ
    #define PHYAGCFINE1CTRL                         0x44
    #define PHYAGCFINE2CTRL                         0x00
    #define PHYAGCFINE3CTRL                         0x00
#else /* MXFE_44MHZ */
    #define PHYAGCFINE1CTRL                         0x47
    #define PHYAGCFINE2CTRL                         0x00
    #define PHYAGCFINE3CTRL                         0x00
#endif /* MXFE_22MHZ */

#define PHYAGCFINALVGAGAINTH                        0x18

#define PHYAGCULCTRL1                               0x65
#define PHYAGCULCTRL2                               0x62

#define PHYAGCULUPTH                                0x10
#define PHYAGCULLOTH                                0x0A

#ifdef MXFE_22MHZ
#define PHYAGCULSETTIME                             0x63
#define PHYAGCMAXANDFINETOCORSTIME                  0x36
#elif MXFE_44MHZ
#define PHYAGCULSETTIME                             0x63
#define PHYAGCMAXANDFINETOCORSTIME                  0x36
#else /* MXFE_44MHZ */
#define PHYAGCULSETTIME                             0xA5
#define PHYAGCMAXANDFINETOCORSTIME                  0x5A
#endif /* MXFE_22MHZ */

#define PHYAGCVGAGAINBACKOFF                        0x00
#define PHYAGCABSULUPTH                             0xFF
#define PHYAGCABSULLOTH                             0x00

/* AGC Test mode register */
#define PHYTMAGCGAIN                                0x00
#define PHYTMAGCGAINSETTIME                         0x00

/* CCA related registers */
#define PHYCCACONTROL                               0x47

#define PHYCCACONSTVGAMC1                           0x10
#define PHYCCACONSTVGAMC2                           0x00
#define PHYCCACONSTVGAMC3                           0x00
#define PHYCCAMACTH1                                0xB4
#define PHYCCAMACTH2                                0xB4
#define PHYCCAPHYTH                                 0xA1
#define PHYCCAINSERTIONVAL                          0xE7
#define PHYCCAERMARGIN1                             0xFA
#define PHYCCAERMARGIN2                             0xFA
#define PHYCCASTATUS                                0x00

#define PHYUNLOCKCKECKCNTR                          0x18

/* Scale factor in the 11b path */
#define PHYRXSCALEFACTMSB                           0x01
#define PHYRXSCALEFACTLSB                           0xBD

/* Power save  and Analog RSSI register */
#define PHYPOWDNCNTRL                               0x62

/* AGC SP RAM regs are initialised later */
#define AGCSPRAMLSB                                 0x00
#define AGCSPRAMMSB                                 0x00

#ifdef ANALOG_RSSI
#define ERSSI_EN                                    0x13
#else /* ANALOG_RSSI */
#define ERSSI_EN                                    0x02
#endif /* ANALOG_RSSI */

#define ERSSI_LTH                                   0x17
#define ERSSI_HTH                                   0x1D

#define ERSSI_AGC_EN                                0x01

#ifdef MXFE_22MHZ
    #define ERSSI_PKT_DET_TIM_CTRL1                 0x33
    #define ERSSI_PKT_DET_TIM_CTRL2                 0x03
#elif MXFE_44MHZ
    #define ERSSI_PKT_DET_TIM_CTRL1                 0x33
    #define ERSSI_PKT_DET_TIM_CTRL2                 0x03
#else /* MXFE_44MHZ */
    #define ERSSI_PKT_DET_TIM_CTRL1                 0x55
    #define ERSSI_PKT_DET_TIM_CTRL2                 0x05
#endif /* MXFE_22MHZ */

#define ERSSI_SAT_NXT_ST_LNA                        0x82
#define ERSSI_SAT_TH                                0x1F
#define ERSSI_MC1                                   0x19
#define ERSSI_MC2                                   0x21
#define ERSSI_MC3                                   0x2C
#define ERSSI_OFSET_ML1                             0x80
#define ERSSI_OFSET_ML2                             0x00
#define ERSSI_OFSET_ML3                             0x08
#define ERSSI_PKT_DET_ITR_2_CTRL                    0x03

#define TXPOWER11A                                  0x31
#define TXPOWER11B                                  0x27

/* DC offset related register */
#define TXDCOI                                      0x00
#define TXDCOQ                                      0x00
#define TXPOSTFILTCOEFFLSB                          0x00
#define TXPOSTFILTCOEFFMSB                          0x00

#ifdef MXFE_22MHZ
    #define RXHPSWITCHDLY                           0x00
#elif MXFE_44MHZ
    #define RXHPSWITCHDLY                           0x00
#else /* MXFE_44MHZ */
    #define RXHPSWITCHDLY                           0x28
#endif /* MXFE_22MHZ */

#ifdef MXFE_22MHZ
#define RXDIGHPFCNTRL                               0x00
#else /* MXFE_22MHZ */
#define RXDIGHPFCNTRL                               0x01
#endif /* MXFE_22MHZ */

#define RSSIMSB                                     0x00
#define RSSILSB                                     0x00

#ifdef MXFE_22MHZ
#define TXPOWCALCNTRL                               0x04
#elif MXFE_44MHZ
#define TXPOWCALCNTRL                               0x04
#else /* MXFE_44MHZ */
#define TXPOWCALCNTRL                               0x00
#endif /* MXFE_22MHZ */

#define TXPOWCALDLY                                 0x32

    #ifdef MXFE_22MHZ
        #define ADCDACTSTMODE                       0x4C
    #elif MXFE_44MHZ
        #define ADCDACTSTMODE                       0x4C
    #else /* MXFE_44MHZ */
        #define ADCDACTSTMODE                       0xCC
    #endif /* MXFE_22MHZ */

#define POWDET11A                                   0x00
#define POWDET11B                                   0x00
#define TXPOWERCALAVGWIN                            0x05

/*****************************************************************************/
/* Test setting for measuring sensitivity of lower data rates.               */
/* NOT to be used over the air.                                              */
/*****************************************************************************/

#ifdef RX_SENS_TEST
/******************************************************************************/
/* Test setting                                                               */
/******************************************************************************/

/* test setting for measuring sensitivity of lower data rates
not to be used over the air */
#define PHYAGCMAXVGAGAIN_RX_TST                     0x1A


#ifdef MXFE_22MHZ
    #define PHYAGCFINALVGAGAINTH_RX_TST             0x1D
#elif MXFE_44MHZ
    #define PHYAGCFINALVGAGAINTH_RX_TST             0x1D
#else /* MXFE_44MHZ */
    #define PHYAGCFINALVGAGAINTH_RX_TST             0x18
#endif /* MXFE_22MHZ */

#ifdef ANALOG_RSSI
#undef PHYAGCPKTDETCNTRL1
#define PHYAGCPKTDETCNTRL1                          0x02
#endif /* ANALOG_RSSI */

#endif /* RX_SENS_TEST */


#endif /* MAX2830_32 */
#endif /* ITTIAM_PHY */
#endif /* PHY_MAX2830_32_ITTIAM_H */
