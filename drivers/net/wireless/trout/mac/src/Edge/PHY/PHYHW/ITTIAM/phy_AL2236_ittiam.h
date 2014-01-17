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
/*  File Name         : phy_AL2236_ittiam.h                                  */
/*                                                                           */
/*  Description       : This file contains PHY register values for Ittiam    */
/*                      PHY to be used with Airoha AL2236 RF chipset         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PHY_AL2236_ITTIAM_H
#define PHY_AL2236_ITTIAM_H

#ifdef ITTIAM_PHY
#ifdef AL2236

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "phy_ittiam.h"

/*****************************************************************************/
/******************************  A L 2 2 3 6 *********************************/
/*****************************************************************************/

/*****************************************************************************/
/*  NOTE : To add a new phy register                                         */
/*  1. Add the register address definition in phy.h prefix with a 'r'        */
/*     This list is maintained so that every register can be acessed         */
/*     individually when required                                            */
/*  2. Add the value to be written in the new register in all the            */
/*     phy_<rf>.h header file.                                               */
/*  3. Add the same macro in "phy_reg_val_table" in phy.c . NOTE that the    */
/*     MACRO HAS TO BE PLACED AT THE EXACT LOCATION ONLY in this             */
/*     table.                                                                */
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
#define PHYBSNRUPTHRESHRSW                          0xEA
#define PHYBSNRLOTHRESHRSW                          0xEA
#define PHYBMPRTHRESHRSW                            0x00
#define PHYBTSTMODCTRL1                             0x00
#define PHYBTSTMODCTRL2                             0x40
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
#define PHYBTXSCALEFACT                             0x3F
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
#define PHYATSTMODCTRL1                             0x04 // 0x00
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

#define PHYRADARCNTRLREG1                           0x00
#define PHYRADARCNTRLREG2                           0x00
#define PHYRADARCNTRLREG3                           0x00
#define PHYRADARCNTRLREG4                           0x00
#define PHYRADARCNTRLREG5                           0x00
#define PHYRADARCNTRLREG6                           0x00
#define PHYRADARCNTRLREG7                           0x00
#define PHYRADARCNTRLREG8                           0x00
#define PHYRADARCNTRLREG9                           0x00
#define PHYRADARCNTRLREG10                          0x00

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

#define RXCNTRLREG11N                               0x00 //0x65 //0x01 //0x00
#define PREFFTDELAY                                 0x0A
#define FFTDELAY                                    0x64
/*****************************************************************************/
/*  PHY - RF register initialization values definitions                      */
/*****************************************************************************/
#define PHYRFCNTRL                                  0xC3 // 0xC1

/* Power up and power down time for Rf Tx and Rx control signals */

#define PHYTXPWUPTIME1                              0x00 // Tuned //0x02
#define PHYTXPWUPTIME2                              0x2a // Tuned //0x54
#define PHYTXPWUPTIME3                              0x24 // Tuned //0x2A
#define PHYTXPWUPTIME4                              0x33 // Tuned //0x2A
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
#define PHYTXPOLCNTRL                               0x0D
#define PHYRXPOLCNTRL                               0x81

#define PHYAGCTIMEOUT                               0xD0

/* AGC related register values */
#define PHYAGCCNTRL                                 0x98
#define PHYAGCADDTNGAIN                             0x00

#define PHYAGCINITWAITPKTSETTIME                    0xA5

#define PHYAGCMAXVGAGAINUPTHR                       0x1F
#define PHYAGCMAXVGAGAIN                            0x1A
#define PHYAGCMAXVGAGAINLOWTHR                      0x16

#define PHYAGCLNAGAIN                               0x1b
#define PHYAGCPKTDETCNTRL1                          0x07
#define PHYAGCPKTDETCNTRL2                          0x24
#define PHYAGCVGAGAINAFTERPKTDET                    0x0F
#define PHYAGCLNAHITOLOVGACOMP                      0x13
#define PHYAGCLNAHITOMIDVGACOMP                     0x0B

#define PHYAGCCORS1UPWINCTRL                        0x0D
#define PHYAGCCORS1LOWINCTRL                        0x8D
#define PHYAGCCORS1LNAHITOLOCTRL                    0x1F
#define PHYAGCCORS1LNAHITOMIDCTRL                   0x1F
#define PHYAGCCORS1UPWINVGAGAIN                     0x00
#define PHYAGCCORS1LOWINVGAGAIN                     0x1A

#define PHYAGCCORS1SETTIMECTRL1                     0xF5
#define PHYAGCCORS1SETTIMECTRL2                     0xD0

#define PHYAGCCORS1NXTSTATECTRL                     0x32

#define PHYAGCCORSNXTSTATECTRL                      0x33
#define PHYAGCCORSLNAGAINCTRL                       0x07

#define PHYAGCCORS2UPWINCTRL                        0x0D
#define PHYAGCCORS2LOWINCTRL                        0x00
#define PHYAGCCORS2LNAHITOLOCTRL                    0x0D

#define PHYAGCCORS2LNAHITOMIDCTRL                   0x4E
#define PHYAGCCORS2UPWINVGAGAIN                     0x08
#define PHYAGCCORS2LOWINVGAGAIN                     0x00

#define PHYAGCCORS2SETTIMECTRL1                     0x0D
#define PHYAGCCORS2SETTIMECTRL2                     0xDD

#define PHYAGCCORS2NXTSTATECTRL                     0x33

#define PHYAGCCORSFINEAVGWINCTRL1                   0x53
#define PHYAGCCORSFINEAVGWINCTRL2                   0x44
#define PHYAGCFINEUPWINCTRL                         0x19

#define PHYAGCFINE1CTRL                             0x47
#define PHYAGCFINE2CTRL                             0x00
#define PHYAGCFINE3CTRL                             0x00

#define PHYAGCFINALVGAGAINTH                        0x14

#define AGCLNAMIDLOWCTRL1                           0x0D //0x00
#define AGCLNAMIDLOWCTRL2                           0x0C
#define AGCLNAMIDLOWCTRL3                           0x08
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
#define RXREADY11ADELAY                             0x5c //0x6E // Tuned trivikram 10/5 0x8C //0x46 //0x13
#define RXREADY11BDELAY                             0x28 // Tuned trivikram 10/5 0x24 //0x12
#define SWRESETCTRL                                 0x00
#define PHYAGCULCTRL1                               0x65
#define PHYAGCULCTRL2                               0x62
#define PHYAGCULUPTH                                0x10
#define PHYAGCULLOTH                                0x02 //0x0A

#define PHYAGCULSETTIME                             0xA5
#define PHYAGCMAXANDFINETOCORSTIME                  0x5A

#define PHYAGCVGAGAINBACKOFF                        0x00
#define PHYAGCABSULUPTH                             0xFF
#define PHYAGCABSULLOTH                             0x00

// AGC Test mode register
#define PHYTMAGCGAIN                                0x00
#define PHYTMAGCGAINSETTIME                         0x00

// CCA related registers
#define PHYCCACONTROL                               0x47

#define PHYCCACONSTVGAMC1                           0x10
#define PHYCCACONSTVGAMC2                           0x00
#define PHYCCACONSTVGAMC3                           0x00
#define PHYCCAMACTH1                                0xB4
#define PHYCCAMACTH2                                0xB4
#define PHYCCAPHYTH                                 0xA1
#define PHYCCAINSERTIONVAL                          0xEA
#define PHYCCAERMARGIN1                             0xFB
#define PHYCCAERMARGIN2                             0xFB
#define PHYCCASTATUS                                0x00

#define PHYUNLOCKCKECKCNTR                          0x18

/* Scale factor in the 11b path */
#define PHYRXSCALEFACTMSB                           0x05 //01

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

#define ERSSI_LTH                                   0x16
#define ERSSI_HTH                                   0x1A
#define ERSSI_AGC_EN                                0x01

#define ERSSI_PKT_DET_TIM_CTRL1                     0x55
#define ERSSI_PKT_DET_TIM_CTRL2                     0x05

#define ERSSI_SAT_NXT_ST_LNA                        0x82
#define ERSSI_SAT_TH                                0x1F
#define ERSSI_MC1                                   0x19
#define ERSSI_MC2                                   0x21
#define ERSSI_MC3                                   0x2C
#define ERSSI_OFSET_ML1                             0x80
#define ERSSI_OFSET_ML2                             0x00
#define ERSSI_OFSET_ML3                             0x08
#define ERSSI_PKT_DET_ITR_2_CTRL                    0x04

#define TXPOWER11A                                  0x31
#define TXPOWER11B                                  0x27

/* DC offset related register */
#define TXDCOI                                      0x00
#define TXDCOQ                                      0x00
#define TXPOSTFILTCOEFFLSB                          0x00
#define TXPOSTFILTCOEFFMSB                          0x00
#define RXHPSWITCHDLY                               0x28 //00

#define RXDIGHPFCNTRL                               0x01

#define RSSIMSB                                     0x00
#define RSSILSB                                     0x00

#define TXPOWCALCNTRL                               0x01
#define TXPOWCALDLY                                 0x21
#define ADCDACTSTMODE                               0x54 //D4
#define POWDET11A                                   0x00
#define POWDET11B                                   0x00
#define TXPOWERCALAVGWIN                            0x06

#define PHYRFCNTRL3                                 0x00
#define AGCTRIGSTATLSB                              0x00
#define AGCTRIGSTATMSB                              0x00
#define AGCSETTLEDVGAGAIN                           0x00
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

#define MONITORSEL                                  0x00


/*****************************************************************************/
/* Test setting for measuring sensitivity of lower data rates.               */
/* NOT to be used over the air.                                              */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE
/******************************************************************************/
/* Test setting                                                               */
/******************************************************************************/

/* test setting for measuring sensitivity of lower data rates
not to be used over the air */
#undef PHYAGCMAXVGAGAIN
#undef PHYAGCFINALVGAGAINTH

#define PHYAGCMAXVGAGAIN                            0x1A
#define PHYAGCFINALVGAGAINTH                        0x18

#endif /* MAC_HW_UNIT_TEST_MODE */


#endif /* AL2236 */
#endif /* ITTIAM_PHY */
#endif /* PHY_AL2236_ITTIAM_H */
