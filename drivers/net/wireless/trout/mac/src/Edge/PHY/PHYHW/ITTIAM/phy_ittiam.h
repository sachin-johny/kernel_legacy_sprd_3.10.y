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
/*  File Name         : phy_ittiam.h                                         */
/*                                                                           */
/*  Description       : This file contains register address definition       */
/*                      and common settings for Ittiam PHY.                  */
/*                                                                           */
/*  List of Functions : write_mxfe_reg                                       */
/*                      read_mxfe_reg                                        */
/*                      write_RF_reg_ittiam                                  */
/*                      clear_phy_radar_status_bit_ittiam                    */
/*                      is_valid_phy_reg_ittiam                              */
/*                      update_phy_ittiam_on_select_channel                  */
/*                      get_phy_rx_sense_ittiam                              */
/*                      set_phy_rx_sense_ittiam                              */
/*                      get_phy_cca_threshold_ittiam                         */
/*                      set_phy_cca_threshold_ittiam                         */
/*                      get_phy_dr_ittiam                                    */
/*                      update_ittiam_reg_val_table                          */
/*                      compute_rssi_db_ittiam                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PHY_ITTIAM_H
#define PHY_ITTIAM_H

#ifdef ITTIAM_PHY

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "itypes.h"
#include "common.h"

#include "spi_interface.h" //dumy add for RF 20120618

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define PHY_ACTIVE                  0x02
#define PHY_SHUTDOWN                0x00

#define MAX_MCS_SUPPORTED           7

#define RX_SENS_TEST  	/* This flags enables full Rx sensitivty */

//#define ANALOG_RSSI

#ifdef MAX2829		//movied by chengwg.
#include "phy_MAX2829_ittiam.h"
#endif /* MAX2829 */

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef MAX2830_32
#include "phy_MAX2830_32_ittiam.h"
#endif /* MAX2830_32 */

#ifdef AL2236
#include "phy_AL2236_ittiam.h"
#endif /* AL2236 */

#ifdef AL7230
#include "phy_AL7230_ittiam.h"
#endif /* AL2236 */
#endif

#define TROUT_RF_SPI_MODE

/*****************************************************************************/
/*  PHY Register Addresses Definitions                                       */
/*****************************************************************************/

/*****************************************************************************/
/*  PHY - 11G register address definitions                                   */
/*****************************************************************************/

#define rPHYIMPLEMENTATIONREG                       0x00
#define rPHYMSBVERNUMREG                            0x01
#define rPHYLSBVERNUMREG                            0x02
#define rPHYCNTRLREG                                0x04

#define rDIGHPFCOEFFLSB                             0x06
#define rDIGHPFCOEFFMSB                             0x07

/*****************************************************************************/
/*  PHY - RF register address definitions                                    */
/*****************************************************************************/

/* RF control */
#define rPHYRFCNTRL                                 0x08

/* Power up and power down time for Rf Tx and Rx control signals */
#define rPHYTXPWUPTIME1                             0x09
#define rPHYTXPWUPTIME2                             0x0A
#define rPHYTXPWUPTIME3                             0x0B
#define rPHYTXPWUPTIME4                             0x0C
#define rPHYTXPWUPTIME5                             0x0D
#define rPHYRXPWUPTIME1                             0x0E
#define rPHYRXPWUPTIME2                             0x0F
#define rPHYRXPWUPTIME3                             0x10
#define rPHYRXPWUPTIME4                             0x11
#define rPHYRXPWUPTIME5                             0x12
#define rPHYTXPWDNTIME1                             0x13
#define rPHYTXPWDNTIME2                             0x14
#define rPHYTXPWDNTIME3                             0x15
#define rPHYTXPWDNTIME4                             0x16
#define rPHYTXPWDNTIME5                             0x17
#define rPHYRXPWDNTIME1                             0x18
#define rPHYRXPWDNTIME2                             0x19
#define rPHYRXPWDNTIME3                             0x1A
#define rPHYRXPWDNTIME4                             0x1B
#define rPHYRXPWDNTIME5                             0x1C

/* Polarity of the RF control signals */
#define rPHYTXPOLCNTRL                              0x1D
#define rPHYRXPOLCNTRL                              0x1E

/* Rx RSSI */
#define rPHYAGCTIMEOUT                              0x1F

/* AGC related registers */
#define rPHYAGCCNTRL                                0x20
#define rPHYAGCADDTNGAIN                            0x21
#define rPHYAGCINITWAITPKTSETTIME                   0x22
#define rPHYAGCMAXVGAGAIN                           0x23
#define rPHYAGCLNAGAIN                              0x24
#define rPHYAGCPKTDETCNTRL1                         0x25
#define rPHYAGCPKTDETCNTRL2                         0x26
#define rPHYAGCVGAGAINAFTERPKTDET                   0x27
#define rPHYAGCLNAHITOLOVGACOMP                     0x28
#define rPHYAGCLNAHITOMIDVGACOMP                    0x29
#define rPHYAGCCORS1UPWINCTRL                       0x2A
#define rPHYAGCCORS1LOWINCTRL                       0x2B
#define rPHYAGCCORS1LNAHITOLOCTRL                   0x2C
#define rPHYAGCCORS1LNAHITOMIDCTRL                  0x2D
#define rPHYAGCCORS1UPWINVGAGAIN                    0x2E
#define rPHYAGCCORS1LOWINVGAGAIN                    0x2F
#define rPHYAGCCORS1SETTIMECTRL1                    0x30
#define rPHYAGCCORS1SETTIMECTRL2                    0x31
#define rPHYAGCCORS1NXTSTATECTRL                    0x32
#define rPHYAGCCORSNXTSTATECTRL                     0x33
#define rPHYAGCCORSLNAGAINCTRL                      0x34

#define rPHYAGCCORS2UPWINCTRL                       0x35
#define rPHYAGCCORS2LOWINCTRL                       0x36
#define rPHYAGCCORS2LNAHITOLOCTRL                   0x37
#define rPHYAGCCORS2LNAHITOMIDCTRL                  0x38
#define rPHYAGCCORS2UPWINVGAGAIN                    0x39
#define rPHYAGCCORS2LOWINVGAGAIN                    0x3A
#define rPHYAGCCORS2SETTIMECTRL1                    0x3B
#define rPHYAGCCORS2SETTIMECTRL2                    0x3C
#define rPHYAGCCORS2NXTSTATECTRL                    0x3D

#define rPHYAGCCORSFINEAVGWINCTRL1                  0x3E
#define rPHYAGCCORSFINEAVGWINCTRL2                  0x3F
#define rPHYAGCFINEUPWINCTRL                        0x40
#define rPHYAGCFINE1CTRL                            0x41
#define rPHYAGCFINE2CTRL                            0x42
#define rPHYAGCFINE3CTRL                            0x43
#define rPHYAGCFINALVGAGAINTH                       0x44
#define rPHYAGCULCTRL1                              0x45
#define rPHYAGCULCTRL2                              0x46
#define rPHYAGCULUPTH                               0x47
#define rPHYAGCULLOTH                               0x48
#define rPHYAGCULSETTIME                            0x49
#define rPHYAGCMAXANDFINETOCORSTIME                 0x4A
#define rPHYAGCVGAGAINBACKOFF                       0x4B
#define rPHYAGCABSULUPTH                            0x4C
#define rPHYAGCABSULLOTH                            0x4D

/* Test mode register */
#define rPHYTMAGCGAIN                               0x4E
#define rPHYTMAGCGAINSETTIME                        0x4F

/* CCA related registers */
#define rPHYCCACONTROL                              0x50
#define rPHYCCACONSTVGAMC1                          0x51
#define rPHYCCACONSTVGAMC2                          0x52
#define rPHYCCACONSTVGAMC3                          0x53
#define rPHYCCAMACTH1                               0x54
#define rPHYCCAMACTH2                               0x55
#define rPHYCCAPHYTH                                0x56
#define rPHYCCAINSERTIONVAL                         0x57
#define rPHYCCAERMARGIN1                            0x58
#define rPHYCCAERMARGIN2                            0x59
#define rPHYCCASTATUS                               0x5A
#define rPHYUNLOCKCKECKCNTR                         0x5B

/* Scale factor in the 11b path */
#define rPHYRXSCALEFACTMSB                          0x5C
#define rPHYRXSCALEFACTLSB                          0x5D

#define rPHYPOWDNCNTRL                              0x5E
#define rAGCSPRAMLSB                                0x5F
#define rAGCSPRAMMSB                                0x60

/* Analog (Ext) RSSI related Registers */
#define rERSSI_EN                                   0x61
#define rERSSI_LTH                                  0x62
#define rERSSI_HTH                                  0x63
#define rERSSI_AGC_EN                               0x64
#define rERSSI_PKT_DET_TIM_CTRL1                    0x65
#define rERSSI_SAT_NXT_ST_LNA                       0x66
#define rERSSI_SAT_TH                               0x67
#define rERSSI_PKT_DET_TIM_CTRL2                    0x68
#define rERSSI_MC1                                  0x69
#define rERSSI_MC2                                  0x6A
#define rERSSI_MC3                                  0x6B
#define rERSSI_OFSET_ML1                            0x6C
#define rERSSI_OFSET_ML2                            0x6D
#define rERSSI_OFSET_ML3                            0x6E
#define rERSSI_PKT_DET_ITR_2_CTRL                   0x6F
#define rTXPOWER11A                                 0x70
#define rTXPOWER11B                                 0x71
#define rTXDCOI                                     0x72
#define rTXDCOQ                                     0x73
#define rTXPOSTFILTCOEFFLSB                         0x74
#define rTXPOSTFILTCOEFFMSB                         0x75
#define rRXHPSWITCHDLY                              0x76
#define rRXDIGHPFCNTRL                              0x77
#define rRSSIMSB                                    0x78
#define rRSSILSB                                    0x79
#define rTXPOWCALCNTRL                              0x7A
#define rTXPOWCALDLY                                0x7B
#define rADCDACTSTMODE                              0x7C
#define rPOWDET11A                                  0x7D
#define rPOWDET11B                                  0x7E
#define rTXPOWERCALAVGWIN                           0x7F

/*****************************************************************************/
/*  PHY - 11B register address definitions                                   */
/*****************************************************************************/

#define rPHYBTXCNTRL                                0x80
#define rPHYBTXLENH                                 0x81
#define rPHYBTXLENL                                 0x82
#define rPHYBRXCNTRL                                0x83
#define rPHYBRXSTA                                  0x84
#define rPHYBRXLENH                                 0x85
#define rPHYBRXLENL                                 0x86
#define rPHYBACQTHRESH                              0x87
#define rPHYBACQTIMEOUT                             0x88
#define rPHYBSNRUPTHRESHRSW                         0x89
#define rPHYBSNRLOTHRESHRSW                         0x8A
#define rPHYBMPRTHRESHRSW                           0x8B
#define rPHYBTSTMODCTRL1                            0x8C
#define rPHYBTSTMODCTRL2                            0x8D
#define rPHYBRXPLCPDLY                              0x8E
#define rPHYCSBARKERTH1REG1                         0x8F
#define rPHYCSBARKERTH1REG2                         0x90
#define rPHYCSBARKERTH1REG3                         0x91
#define rPHYCSBARKERTH1REG4                         0x92
#define rPHYCSBARKERTH1REG5                         0x93
#define rPHYCSBARKERTH1REG6                         0x94
#define rPHYCSBARKERTH2REG1                         0x95
#define rPHYCSBARKERTH2REG2                         0x96
#define rPHYCSBARKERTH2REG3                         0x97
#define rPHYCSBARKERTH2REG4                         0x98
#define rPHYCSBARKERTH2REG5                         0x99
#define rPHYCSBARKERTH2REG6                         0x9A
#define rPHYCSBARKERTH2REG7                         0x9B
#define rPHYRAKEESTDELAY                            0x9C
#define rPHYBHDRPLCPDLY                             0x9D
#define rPHYBANTIPHASEHANGUP                        0x9E
#define rPHYBDIGHPFCOEFFLSB                         0x9F
#define rPHYBTXSCALEFACT                            0xA0
#define rDSSSTXPREFILTCOEFFLSB                      0xA1
#define rDSSSTXPREFILTCOEFFMSB                      0xA2
#define rANTDIVERSTYCTRL                            0xA3
#define rANTSWITCHDUR                               0xA4
#define rANTIDLESTATEWAIT                           0xA5
#define rPHY11BMODECTRL                             0xA6
#define rADRSSICOMPFACTOR                           0xA7
#define rRAKESWTMULTIPATHCNT                        0xA8


/*****************************************************************************/
/*  PHY - RF Register Extended Addresses Definitions                         */
/*****************************************************************************/
#define rPHYRFCNTRL3                                0xB0
#define rAGCTRIGSTATLSB                             0xB1
#define rAGCTRIGSTATMSB                             0xB2
#define rAGCSETTLEDVGAGAIN                          0xB3
#define rAGCSETTLEDLNAGAIN                          0xB4

#define rRXDCOI                                     0xB5
#define rRXDCOQ                                     0xB6

#define rTXIQIMBREG1                                0xB7
#define rTXIQIMBREG2                                0xB8

#define rRXIQIMBREG1                                0xB9
#define rRXIQIMBREG2                                0xBA
#define rRXIQIMBREG3                                0xBB


#define rSPIDATA                                    0xBC
#define rSPICTRL                                    0xBD
#define rMACIFCTRL                                  0xBE

#define rMONITORSEL                                 0xBF

/*****************************************************************************/
/*  PHY - 11A Register Addresses Definitions                                 */
/*****************************************************************************/

#define rPHYATXCON                                  0xC0
#define rPHYATXLENH                                 0xC1
#define rPHYATXLENL                                 0xC2
#define rPHYATXSERFLD                               0xC3
#define rPHYATXSAMPLESCALEFACTOR                    0xC4
#define rPHYARXCTL                                  0xC5
#define rPHYARXSTS                                  0xC6
#define rPHYARXSERFLD                               0xC7
#define rPHYARXLENH                                 0xC8
#define rPHYARXLENL                                 0xC9
#define rPHYAPKTDETTHRESH                           0xCA
#define rPHYAPKTDETCNT1                             0xCB
#define rPHYAPKTDETCNT2                             0xCC
#define rPHYALPDETTH1                               0xCD
#define rPHYALPDETTH2                               0xCE
#define rPHYALPDETCNT                               0xCF
#define rPHYASNRTH1                                 0xD0
#define rPHYASNRTH2                                 0xD1
#define rPHYAUNLOCKTIMECORRCTRL                     0xD2
#define rPHYATSTMODCTRL1                            0xD3
#define rPHYATSTMODCTRL2                            0xD4
#define rPHYATSTMODCTRL3                            0xD5
#define rPHYARXPLCPDLY                              0xD6
#define rPHYALOCKCONVFACTLSB                        0xD7
#define rPHYALOCKCONVFACTMSB                        0xD8
#define rPHYAMAXRXLENGTHMSB                         0xD9
#define rPHYAMAXRXLENGTHLSB                         0xDA
#define rUNLOCKSAMPREG1                             0xDB
#define rUNLOCKSAMPREG2                             0xDC
#define rUNLOCKSAMPREG3                             0xDD
#define rCFEBIASPOS                                 0xE0
#define rCFEBIASNEG                                 0xE1
#define rFASTCSWIN                                  0xE2
#define rPHYATXSCALEFACT                            0xE3
#define rPHYADIGHPFCOEFFLSB                         0xE4
#define rPHYADIGHPFCOEFFMSB                         0xE5

#define rPHYRADARCNTRLREG1                          0xE6
#define rPHYRADARCNTRLREG2                          0xE7
#define rPHYRADARCNTRLREG3                          0xE8
#define rPHYRADARCNTRLREG4                          0xE9
#define rPHYRADARCNTRLREG5                          0xEA
#define rPHYRADARCNTRLREG6                          0xEB
#define rPHYRADARCNTRLREG7                          0xEC
#define rPHYRADARCNTRLREG8                          0xED
#define rPHYRADARCNTRLREG9                          0xEE
#define rPHYRADARCNTRLREG10                         0xEF

#define rRXCNTRLREG11N                              0xF0

/*****************************************************************************/
/*  End of PHY Register Addresses Definitions - REGISTER BANK 1              */
/*****************************************************************************/

/*****************************************************************************/
/*  PHY Register Addresses Definitions  - REGISTER BANK 2                    */
/*****************************************************************************/

/*****************************************************************************/
/*  PHY - RF Register Addresses Definitions                                  */
/*****************************************************************************/

#define rAGCLNAMIDLOWCTRL1                        0x00
#define rAGCLNAMIDLOWCTRL2                        0x01
#define rAGCLNAMIDLOWCTRL3                        0x02
#define rAGCLNAMIDLOWAVGWINCTRL1                  0x03
#define rAGCLNAMIDLOWAVGWINCTRL2                  0x04
#define rAGCMIDCOARSE1UPPERWINCTRL                0x05
#define rAGCMIDCOARSE1LOWERWINCTRL                0x06
#define rAGCMIDCOARSE1LNAMIDTOLOWCTRL             0x07
#define rAGCMIDCOARSE1UPPERWINVGAGAIN             0x08
#define rAGCMIDCOARSE1LOWERWINVGAGAIN             0x09
#define rAGCMIDCOARSELNAGAINCTRL                  0x0A
#define rAGCMIDCOARSE1SETTLETIMECTRL1             0x0B
#define rAGCMIDCOARSE1SETTLETIMECTRL2             0x0C
#define rAGCMIDCOARSE1NXTSTATECTRL                0x0D
#define rAGCMIDCOARSENXTSTATECTRL                 0x0E
#define rAGCMIDCOARSE2UPPERWINCTRL                0x0F
#define rAGCMIDCOARSE2LOWERWINCTRL                0x10
#define rAGCMIDCOARSE2LNAMIDTOLOWCTRL             0x11
#define rAGCMIDCOARSE2UPPERWINVGAGAIN             0x12
#define rAGCMIDCOARSE2LOWERWINVGAGAIN             0x13
#define rAGCMIDCOARSE2SETTLETIMECTRL1             0x14
#define rAGCMIDCOARSE2SETTLETIMECTRL2             0x15
#define rAGCMIDCOARSE2NXTSTATECTRL                0x16
#define rAGCLOWFINE1CTRL                          0x17
#define rAGCLOWFINE2CTRL                          0x18
#define rAGCLOWFINE3CTRL                          0x19
#define rAGCRELOCKUPMARGIN                        0x1A
#define rAGCRELOCKLOWMARGIN                       0x1B
#define rAGCRELOCKCHECKTIME                       0x1C
#define rTXREADYDELAY                             0x1D
#define rRXREADY11ADELAY                          0x1E
#define rRXREADY11BDELAY                          0x1F
#define rSWRESETCTRL                              0x20

/*****************************************************************************/
/*  PHY - 11A Register Addresses Definitions                                  */
/*****************************************************************************/

#define rPREFFTDELAY                              0x40
#define rFFTDELAY                                 0x41

/*****************************************************************************/
/*  End of PHY Register Addresses Definitions - REGISTER BANK 2              */
/*****************************************************************************/

/*****************************************************************************/
/* Last address in PHY register space                                        */
/*****************************************************************************/

#define MAX_PHY_ADDR                                0xF0
#define MAX_PHY_ADDR2                               0x41

/*****************************************************************************/
/* Number of PHY registers initialized differently for 2.4 and 5 GHz bands   */
/*****************************************************************************/

#define NUM_PHY_ADDR_START_FREQ_5                   13

/*****************************************************************************/
/* PHY dependent MAC Register initail value definition for Ittiam-PHY        */
/*****************************************************************************/

#define MAC_TXPLCP_DELAY_INIT_VALUE          0x00000334 // Tuned // 0x00000112
#define MAC_RXPLCP_DELAY_INIT_VALUE          0x00000BB4 // Tuned // 0x00000995
#define MAC_RXTXTURNAROUND_TIME_INIT_VALUE   0x00001E28 // Tuned // 0x00001E3C
#define MAC_PHY_CCA_DELAY_INIT_VALUE         0x00000028
#define MAC_TXPLCP_ADJUST_INIT_VALUE         0x14202000 // Tuned // 0x00000000
#define MAC_RXPLCP_DELAY2_INIT_VALUE         0x00747426 // Tuned // 0x005A5A26

/*****************************************************************************/
/*  Common PHY register values                                               */
/*****************************************************************************/

/* If the address in the space is not used then write 0x00 (Don't Care) */
#define PHY_ADDR_NOT_USED                           0x00

//#ifdef TROUT2_WIFI_FPGA_RF55
#if  defined(TROUT2_WIFI_FPGA_RF55) || defined(TROUT_WIFI_EVB_MF_RF55)
#define TX_UPFILTER_SCALER                           0x0e
#endif

#ifdef TROUT2_WIFI_IC
#define TX_UPFILTER_SCALER                           0x0e
#endif

#ifdef TROUT2_WIFI_FPGA
#define VGA_GAIN_SHIFT_CTRL_LSB_1	0x0
#define VGA_GAIN_SHIFT_CTRL_LSB_2	0x3
#define VGA_GAIN_SHIFT_CTRL_LSB_3	0x6
#define VGA_GAIN_SHIFT_CTRL_LSB_4	0x9
#define VGA_GAIN_SHIFT_CTRL_LSB_5	0xC
#define VGA_GAIN_SHIFT_CTRL_LSB_6	0xF
#define VGA_GAIN_SHIFT_CTRL_LSB_7	0x12
#define VGA_GAIN_SHIFT_CTRL_MSB		0x0
#endif

#ifdef TROUT2_WIFI_IC
#define 	VGA_GAIN_SHIFT_CTRL_LSB_1	0x0
#define 	VGA_GAIN_SHIFT_CTRL_LSB_2	0x2
#define 	VGA_GAIN_SHIFT_CTRL_LSB_3	0x4
#define 	VGA_GAIN_SHIFT_CTRL_LSB_4	0x6
#define 	VGA_GAIN_SHIFT_CTRL_LSB_5	0x8
#define 	VGA_GAIN_SHIFT_CTRL_LSB_6	0xa
#define  	VGA_GAIN_SHIFT_CTRL_LSB_7 	0xc
#define 	VGA_GAIN_SHIFT_CTRL_MSB 	0x0
#endif

/*****************************************************************************/
/* Settings for putting PHY in Continious Tx mode (done for TELEC testing)   */
/*****************************************************************************/

#ifdef PHY_CONTINUOUS_TX_MODE

#define PHYBTSTMODCTRL1_TEST_VAL                    0x58
#define PHYBTSTMODCTRL2_TEST_VAL                    0x0D
#define PHYATSTMODCTRL1_TEST_VAL                    0xC0
#define PHYATSTMODCTRL2_TEST_VAL                    0x40
#define PHYATSTMODCTRL3_TEST_VAL                    0x80

#endif /* PHY_CONTINUOUS_TX_MODE */

/*****************************************************************************/
/* Sensitivity related constants                                             */
/*****************************************************************************/


/*****************************************************************************/
/************************ MxFE Related Constants *****************************/
/*****************************************************************************/
#define NUM_AD9863_REGS             23
#define SPI_DATA_WIDTH_AD9863       16

/* AD9863 Registers */
#define MFE_GEN                     0x0   /* General           */
#define MFE_CM                      0x1   /* Clock Mode        */
#define MFE_PD                      0x2   /* Power Down        */
#define MFE_RAPD                    0x3   /* RX A Power Down   */
#define MFE_RBPD                    0x4   /* RX B Power Down   */
#define MFE_RPD                     0x5   /* RX Power Down     */
#define MFE_RX1                     0x6   /* RX Path           */
#define MFE_RX2                     0x7   /* RX Path           */
#define MFE_RX3                     0x8   /* RX Path           */
#define MFE_RX4                     0x9   /* RX Path           */
#define MFE_RX5                     0xA   /* RX Path           */
#define MFE_TX1                     0xB   /* TX Path           */
#define MFE_TX2                     0xC   /* TX Path           */
#define MFE_TX3                     0xD   /* TX Path           */
#define MFE_TX4                     0xE   /* TX Path           */
#define MFE_TX5                     0xF   /* TX Path           */
#define MFE_TX6                     0x10  /* TX Path           */
#define MFE_TX7                     0x11  /* TX Path           */
#define MFE_TX8                     0x12  /* TX Path           */
#define MFE_IOC1                    0x13  /* I/O Configuration */
#define MFE_IOC2                    0x14  /* I/O Configuration */
#define MFE_CLK1                    0x15  /* Clock             */
#define MFE_CLK2                    0x16  /* Clock             */

//#define AFE_AD9861 //wuranqing 0825
/* Maxim RF Related Configurable Parameters */
/* AD9863 Register Value */
#define MFE_GEN_VAL                 0x80
#ifdef AFE_AD9861
#define MFE_CM_VAL                  0x02
#else  /* AFE_AD9861 */
#define MFE_CM_VAL                  0x00
#endif /* AFE_AD9861 */
#define MFE_PD_VAL                  0x00
#define MFE_RAPD_VAL                0x00//0x40 //wuranqing 0825
#define MFE_RBPD_VAL                0x00//0x40 //wuranqing 0825
#define MFE_RPD_VAL                 0x00
#define MFE_RX1_VAL                 0x00
#define MFE_RX2_VAL                 0x00
#define MFE_RX3_VAL                 0x00
#define MFE_RX4_VAL                 0x00
#define MFE_RX5_VAL                 0x00
#define MFE_TX1_VAL                 0x00
#define MFE_TX2_VAL                 0x00

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF2829
#define MFE_TX3_VAL                 0x40//0x80 //dumy add 0803 from wuranqing
#else
#define MFE_TX3_VAL                 0x80//0x40//0x80 //dumy add 0803 from wuranqing
#endif
#endif

#ifdef TROUT2_WIFI_IC
#define MFE_TX3_VAL                 0x80//0x40//0x80 //dumy add 0803 from wuranqing
#else
#define MFE_TX3_VAL                 0x80//0x40//0x80 //dumy add 0803 from wuranqing
#endif

#define MFE_TX4_VAL                 0x00
#define MFE_TX5_VAL                 0x00

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF2829
#define MFE_TX6_VAL                 0x40//0x80 //dumy add 0803 from wuranqing
#else
#define MFE_TX6_VAL                 0x80//0x40//0x80 //dumy add 0803 from wuranqing
#endif
#endif

#ifdef TROUT2_WIFI_IC
#define MFE_TX6_VAL                 0x80//0x40//0x80 //dumy add 0803 from wuranqing
#else
#define MFE_TX6_VAL                 0x80//0x40//0x80 //dumy add 0803 from wuranqing
#endif

#ifdef TROUT2_WIFI_FPGA
#ifdef TROUT2_WIFI_FPGA_RF2829
#define MFE_TX7_VAL                 0xFF //dumy add 0803 from wuranqing
#else
#define MFE_TX7_VAL                 0x60//0xFF //dumy add 0803 from wuranqing
#endif
#endif

#ifdef TROUT2_WIFI_IC
#define MFE_TX7_VAL                 0x60//0xFF //dumy add 0803 from wuranqing
#endif

#ifdef TROUT_WIFI_FPGA
#define MFE_TX7_VAL                 0xFF//0x60//0xFF //dumy add 0803 from wuranqing
#endif

#ifdef TROUT_WIFI_EVB
#define MFE_TX7_VAL                 0xFF//0x60//0xFF //dumy add 0803 from wuranqing
#endif

#define MFE_TX8_VAL                 0x00
#define MFE_IOC1_VAL                0x00 //0x00  liusy
#define MFE_IOC2_VAL                0x00
#ifdef AFE_AD9861
#define MFE_CLK1_VAL                0x11 
#else  /* AFE_AD9861 */
#define MFE_CLK1_VAL                0x00 //0x80 //0x00 //dumy add 0822 for PLL bypass
#endif /* AFE_AD9861 */
#define MFE_CLK2_VAL                0x00

/*****************************************************************************/
/* RF programming related definitions                                        */
/*****************************************************************************/

/* RF Specific Constants */
#ifdef MAX2829
#define RF_SPI_DATA_WIDTH           18
#endif /* MAX2829 */

// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
#ifdef MAX2830_32
#define RF_SPI_DATA_WIDTH           18
#endif /* MAX2830_32 */

#ifdef AL2236
#define RF_SPI_DATA_WIDTH           20
#endif /* AL2236 */

#ifdef AL7230
#define RF_SPI_DATA_WIDTH           24
#endif /* AL2236 */
#endif

/*****************************************************************************/
/* Extern variable declarations                                              */
/*****************************************************************************/

extern UWORD32 phy_reg_val_table[];
extern UWORD32 phy_reg_val_table_2[];
extern UWORD32 phy_reg_addr_start_freq_5_table[];
extern UWORD32 phy_reg_val_start_freq_5_table[];

/*****************************************************************************/
/* Extern function declarations                                              */
/*****************************************************************************/

/*****************************************************************************/
/* Extern function declarations                                              */
/*****************************************************************************/

extern void   write_dot11_phy_reg(UWORD8 ra, UWORD32 rd);
extern void   root_write_dot11_phy_reg(UWORD8 ra, UWORD32 rd);
extern void   read_dot11_phy_reg(UWORD8 ra, UWORD32 *rd);
extern int GetTxPowerLevel(UWORD32* gnTxPower,UWORD32* bTxPower);
extern int get_tx_pwr_from_nv(UWORD32* tx_pwr_80211b,UWORD32* tx_pwr_80211g,UWORD32* tx_pwr_80211n, UWORD32* delta_0x57);

extern void initialize_phy_ittiam(void);
extern void set_default_tx_power_levels_ittiam(void);
extern void read_phy_hw_version_ittiam(void);
extern void set_phy_tx_power_level_11a_ittiam(UWORD8 val);
extern void set_phy_tx_power_level_11b_ittiam(UWORD8 val);
extern void reset_phy_ittiam(void);
extern void power_up_phy_ittiam(void);
extern void power_down_phy_ittiam(void);
extern void root_power_down_phy_ittiam(void);
extern UWORD8 update_supp_mcs_bitmap_ittiam(UWORD8 *mcs_bmp);
extern UWORD8 get_tssi_11b_ittiam(void);
extern UWORD8 get_tssi_11gn_ittiam(void);
extern void post_radar_detect_ittiam(void);
extern void enable_radar_detection_ittiam(void);
extern void disable_radar_detection_ittiam(void);
extern void reset_ofdmpre_unidentified_bits_ittiam(void);
extern BOOL_T is_unidentified_bit_detected_ittiam(void);
extern BOOL_T is_ofdm_preamble_detected_ittiam(void);
extern UWORD16 get_lock_conv_fact(UWORD8 start_freq, UWORD8 ch_idx);


#ifdef PHY_CONTINUOUS_TX_MODE
extern void set_phy_continuous_tx_mode_ittiam(void);
#endif /* PHY_CONTINUOUS_TX_MODE */


/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/
//chenq add a macro 2012-12-20
#ifdef TROUT_WIFI_EVB
//====================dumy add for Trout IC RF 20121026=======================================
//initial RF SPI mode:
INLINE void trout_ic_rf_spi_init(void)
{
      UWORD32 val;
    
    //printk(KERN_WARNING "trout_ic_rf_spi_init==> ");
    
    /* Set the RF SPI Mode, Select, Clock etc */
    val = 0xd5; 
    
    host_write_trout_reg(val, rCOMM_RF_SPI_CTRL);
}

//--------dumy add 20121227 from wuranqing for Trout1 RF ADI read/write------------------------
INLINE UWORD32 trout_ic_rf_addr_spi2adi(UWORD32 addr)
{
    UWORD32 val;
    val = ((addr>>8)&0x07);
    
    switch(val)
    {
        case 4:
            return((addr&0x7F)|(2<<7));
        case 5:
            return((addr&0x7F)|(3<<7));
        case 6:
            return((addr&0x7F)|(4<<7));
        default:
            return addr;
    } 
}

INLINE void trout_ic_rf_reg_adi_write(UWORD32 addr, UWORD32 data)
{
    //UWORD32 reg_data=0;;
    UWORD32 adi_addr,val;
    
    adi_addr = 0x1000 + trout_ic_rf_addr_spi2adi(addr);

    host_write_trout_reg(data, (adi_addr<<2));
#if 1 
    do {
       val = host_read_trout_reg(rCOMM_INT_STAT);
       
    }while((val & BIT23) == 0);
    
    /* Clear ADI Write INT Stat (BIT23) */
    host_write_trout_reg(BIT23, rCOMM_INT_CLEAR);
#else
    udelay(20);
#endif

}

INLINE void trout_ic_rf_adi_init(void)
{
      UWORD32 val;
    
    /* Set the RF ADI Mode, Select, Clock etc */
    val = 0x95; 
    
    host_write_trout_reg(val, rCOMM_RF_SPI_CTRL);

    trout_ic_rf_reg_adi_write(0x107B, 0);
    
}
INLINE void trout_ic_rf_reg_adi_read(UWORD32 addr, UWORD32 *reg_data)
{
    //UWORD32 reg_data=0;
    UWORD32 adi_addr,val;
    
    adi_addr = 0x1000 + trout_ic_rf_addr_spi2adi(addr);

    host_read_trout_reg(adi_addr<<2);
#if 1 
    do {
       val = host_read_trout_reg(rCOMM_INT_STAT);
       
    }while((val & BIT23) == 0);
    
    /* Clear ADI Write INT Stat (BIT23) */
    host_write_trout_reg(BIT23, rCOMM_INT_CLEAR);
#else
    udelay(20);
#endif
       
       *reg_data =host_read_trout_reg((0x1FFF<<2));
}

//--------------------------------------------------------------------------------------

/* dumy add for Trout IC RF 20121026 */
INLINE void trout_ic_rf_reg_spi_write(UWORD32 address, UWORD32 data)
{
    UWORD32 spi_data;
    //UWORD32 val;
    
    //printk(KERN_WARNING "trout_ic_rf_reg_write==> ");
        
    /* Shift the required SPI data bits to MSB */
    spi_data = (data | (address << 16));

    host_write_trout_reg(spi_data, rCOMM_RF_SPI_DATA);

    //printk(KERN_WARNING "RF=1\n");
#if 0  // 20120730, dumy mod
    /* Wait for the Poll the SPI write done */
    do {
       val = host_read_trout_reg(rCOMM_INT_STAT);
       
    }while((val & BIT23) == 0);
    
    /* Clear ADI Write INT Stat (BIT23) */
    host_write_trout_reg(BIT23, rCOMM_INT_CLEAR);
#else
    udelay(20);
#endif
    //printk(KERN_WARNING "RF=3\n");

}
/* dumy add for Trout IC RF 20121026 */
INLINE void trout_ic_rf_reg_spi_read(UWORD32 reg_addr, UWORD32 *reg_data)
{
	UWORD32 data;
#if 0
	UWORD32 val;
#endif

	data = ((reg_addr << 16) | BIT31);
    
       host_write_trout_reg(data, rCOMM_RF_SPI_DATA);

#if 0  // 20120730, dumy mod
    /* Wait for the Poll the SPI write done */
    do {
       val = host_read_trout_reg(rCOMM_INT_STAT);
       
    }while((val & BIT23) == 0);
    
    /* Clear ADI Write INT Stat (BIT23) */
    host_write_trout_reg(BIT23, rCOMM_INT_CLEAR);
#else
    udelay(20);
#endif

       *reg_data =host_read_trout_reg(rCOMM_RF_SPI_DATA);
}

INLINE void trout_ic_rf_reg_read(UWORD32 reg_addr, UWORD32 *reg_data)
{
#ifdef TROUT_RF_SPI_MODE
    trout_ic_rf_reg_spi_read(reg_addr,reg_data);
#else
    trout_ic_rf_reg_adi_read(reg_addr,reg_data);
#endif
}
INLINE void trout_ic_rf_reg_write(UWORD32 address, UWORD32 data)
{
#ifdef TROUT_RF_SPI_MODE
    trout_ic_rf_reg_spi_write(address, data);
#else
    trout_ic_rf_reg_adi_write(address, data);
#endif
}

/* dumy add for Trout IC RF 20121026 */
INLINE void sc2330_rf_rx_filter_cfg(void)
{
	#if 1 //chenq mod for test 2013-01-06
	 trout_ic_rf_reg_write(0x0, 0x6000);
     trout_ic_rf_reg_write(0x604, 0xf000);
     trout_ic_rf_reg_write(0x672, 0x7);
     trout_ic_rf_reg_write(0x602, 0x5f32); //freq configure
	 udelay(100);
     trout_ic_rf_reg_write(0x66b, 0x2054); 
     trout_ic_rf_reg_write(0x677, 0x40c);
     trout_ic_rf_reg_write(0x675, 0x5c47);    
     trout_ic_rf_reg_write(0x676, 0x447);
	#else //2013-01-06 work ok chenq
    //idle mode
    //trout_ic_rf_reg_write(0x600, 0x0); 

    //init
    //trout_ic_rf_reg_write(0x66b, 0x530a); 

    //start Rx
     trout_ic_rf_reg_write(0x0, 0x6000); 
     trout_ic_rf_reg_write(0x604, 0xf000); 
     trout_ic_rf_reg_write(0x672, 0x7); 
     
    //ranqing add 1103
    trout_ic_rf_reg_write(0x602, 0x5f32); //freq configure
    udelay(100);

     trout_ic_rf_reg_write(0x66b, 0x2054); 
     trout_ic_rf_reg_write(0x677, 0x40c); 
     trout_ic_rf_reg_write(0x675, 0x5c47); //2012-12-04 wurq   0x5c47=>0x5c37   
     trout_ic_rf_reg_write(0x676, 0x447);  //2012-12-04 wurq   0x447=>0x437   
	 #endif
}

INLINE void sc2330_rf_space_sel(void)
{
    trout_ic_rf_reg_write(0x0, 0x6000); 
}

/* dumy add for Trout IC RF 20121026 */
INLINE void sc2330_rf_tx_filter_cfg(void)
{
    //idle mode
    trout_ic_rf_reg_write(0x600, 0x1); //dumy add 0129 for MF BT
    sc2330_rf_space_sel();

	#if 1 //chenq mod 2013-01-07 for test
	trout_ic_rf_reg_write(0x0, 0x6000);
	trout_ic_rf_reg_write(0x607, 0x64);
    trout_ic_rf_reg_write(0x614, 0x2134); 
    trout_ic_rf_reg_write(0x64b, 0x87);    
    //ranqing add 1103, synth tracking enable
    //trout_ic_rf_reg_write(0x66b, 0x530a);  
    trout_ic_rf_reg_write(0x66c, 0x530a); 
    trout_ic_rf_reg_write(0x56a, 0x6809); 
    trout_ic_rf_reg_write(0x606, 0xdfe3);
    trout_ic_rf_reg_write(0x638, 0x4000);
	//wuranqing add 20121121
    trout_ic_rf_reg_write(0x60a, 0x7037);//synth rx en/LO cascode
    trout_ic_rf_reg_write(0x60b, 0x47c7);//synth rx en/LO cascode

#ifdef TROUT_WIFI_EVB_MF
	trout_ic_rf_reg_write(0x77, 0x02);
	trout_ic_rf_reg_write(0x4a, 0xf018);
#endif

	#else //2013-01-06 work ok chenq
    trout_ic_rf_reg_write(0x614, 0x2134); 
    trout_ic_rf_reg_write(0x64b, 0x87);    
    //ranqing add 1103, synth tracking enable
    //trout_ic_rf_reg_write(0x66b, 0x530a);  
    trout_ic_rf_reg_write(0x66c, 0x530a); 

//    trout_ic_rf_reg_write(0x0, 0x6000);  //ranqing mask 1103
    trout_ic_rf_reg_write(0x56a, 0x6809); 
//    trout_ic_rf_reg_write(0x64f, 0x8cf7);     //ranqing mask 1103
    trout_ic_rf_reg_write(0x606, 0xdfe3);
    trout_ic_rf_reg_write(0x638, 0x4000);

	//wuranqing add 20121121
    trout_ic_rf_reg_write(0x60a, 0x7037);//synth rx en/LO cascode
    trout_ic_rf_reg_write(0x60b, 0x47c7);//synth rx en/LO cascode
	#endif
}
/* dumy add for Trout IC RF 20121026 */
INLINE void sc2330_rf_channel_config(UWORD8 rf_channel)
{
    UWORD32 freq=0;

    if((rf_channel >= 0) && (rf_channel < 13))
    {
       freq = (24120 + rf_channel*50);
    }
    else if(rf_channel == 13)
    {
        freq = 24840;
    }
    else
    {
        freq = 24370;
    }

    trout_ic_rf_reg_write(0x602, freq);  

}
/* dumy add for Trout IC RF 20121026 */
INLINE void sc2330_rf_IQ_swap_config(void)
{

    trout_ic_rf_reg_write(0x638, 0x4000);  

	#if 0 //chenq add for test 2013-01-06
	trout_ic_rf_reg_write(0x60A, 0x7037);  
	trout_ic_rf_reg_write(0x60B, 0xC7);
	#endif
}

INLINE void trout_rf_switch(UWORD32 mode_sel, UWORD32 enable_sel)
{
    UWORD32 reg_data;
    
    printk("write (mode_sel, enable_sel) = (%d,%d) rf switch\n", mode_sel,enable_sel);
    switch(mode_sel)
    {
        case 1:
        {
            switch(enable_sel)
            {
                case 1:
                    trout_ic_rf_reg_read(0x64F,&reg_data);
                    reg_data |= (BIT11 | BIT10);
                    trout_ic_rf_reg_write(0x64F,reg_data);
                    printk("write 1,1 rf switch\n");
                    break;
                case 0:
                    trout_ic_rf_reg_read(0x64F,&reg_data);
                    reg_data &= ((~BIT10) & (~BIT11));
                    trout_ic_rf_reg_write(0x64F,reg_data);
                    break;
                default:
                    trout_ic_rf_reg_read(0x64F,&reg_data);
                    reg_data &= ((~BIT10) & (~BIT11));
                    trout_ic_rf_reg_write(0x64F,reg_data);
                    break;
            }
            break;
        }
        case 2:
        {
            switch(enable_sel)
            {
                case 0:
                    trout_ic_rf_reg_read(0x6F,&reg_data);
                    reg_data &= (~BIT15);
                    trout_ic_rf_reg_write(0x6F,reg_data);
                    break;
                case 1:
                    trout_ic_rf_reg_read(0x6F,&reg_data);
                    
                    trout_ic_rf_reg_write(0x6F,0x8000);
                    break;
                default:
                    trout_ic_rf_reg_read(0x6F,&reg_data);
                    reg_data &= (~BIT15);
                    trout_ic_rf_reg_write(0x6F,0);
                    break;
            }
            break;
        }
        case 3:
        {
            switch(enable_sel)
            {
                case 0:
                    trout_ic_rf_reg_read(0x64F,&reg_data);
                    reg_data &= ((~BIT14) & (~BIT15));
                    trout_ic_rf_reg_write(0x64F,reg_data);
                    break;
                case 1:
                    trout_ic_rf_reg_read(0x64F,&reg_data);
                    reg_data |= ((BIT14) | (BIT15));
                    trout_ic_rf_reg_write(0x64F,reg_data);
                    break;
                default:
                    trout_ic_rf_reg_read(0x64F,&reg_data);
                    reg_data &= ((~BIT14) & (~BIT15));
                    trout_ic_rf_reg_write(0x64F,reg_data);
                    break;
            }
            break;
        }
        case 4:
        {
            switch(enable_sel)
            {
                case 0:
                    trout_ic_rf_reg_read(0x7A,&reg_data);
                    reg_data &= (~BIT2);
                    trout_ic_rf_reg_write(0x7A,reg_data);
                    break;
                case 1:
                    trout_ic_rf_reg_read(0x7A,&reg_data);
                    reg_data |= BIT2;
                    trout_ic_rf_reg_write(0x7A,reg_data);
                    break;
                default:
                    trout_ic_rf_reg_read(0x7A,&reg_data);
                    reg_data &= (~BIT2);
                    trout_ic_rf_reg_write(0x7A,reg_data);
                    break;
            }
            break;
        }
        default:
            break;
    }
}

// ADC DAC config:
INLINE void sc2330_initial(void)
{
    host_write_trout_reg(0x2000000, ((UWORD32)0x62)<<2);  // WIFI/BT DAC
    host_write_trout_reg(0x34, ((UWORD32)0x63)<<2);  // WIFI ADC
    host_write_trout_reg(0xFF, ((UWORD32)0x4011)<<2);  //dumy add 0129 for MF BT

	#if 1 //2013-01-06 add for test
	host_write_trout_reg(0x606dfe3,((UWORD32)0xc0<<2));
	host_write_trout_reg(0x606b6c3,((UWORD32)0x100<<2));
	
	host_write_trout_reg(0xe, ((UWORD32)0x50)<<2);
	#else //2013-01-06 work ok chenq
	host_write_trout_reg(0xf, ((UWORD32)0x50)<<2);
	#endif
}

/* dumy add for Trout IC RF 20121026 */
INLINE void sc2330_rf_AGC_config(UWORD32 agc_val)//agc_val:0x0~0x19
{
    UWORD32 reg_val=0;
    reg_val = host_read_trout_reg(((UWORD32)0x44)<<2);

    reg_val = (reg_val & 0xffff01ff); 
        
    agc_val = ((agc_val & 0x7f) << 9); 
    
    host_write_trout_reg((reg_val |agc_val), (((UWORD32)0x44)<<2));
}

//===========================================================
#endif


//#ifdef TROUT2_WIFI_FPGA
#if  defined(TROUT2_WIFI_FPGA) || defined(TROUT_WIFI_EVB)

//#ifdef TROUT2_WIFI_FPGA_RF55
#if  defined(TROUT2_WIFI_FPGA_RF55) || defined(TROUT_WIFI_EVB_MF_RF55)
//for agc
extern UWORD8 gain_table_BT[28];

//for wifi agc
extern UWORD16 gain_table_WIFI[32];

INLINE void  rf_reg_read(UWORD32 reg_addr, UWORD32 *reg_data)
{
    reg_addr = ((reg_addr & 0x0000ffff) | 0x1000);
    
	#if 0 
	host_read_trout_reg(reg_addr<<2);
    
    //*reg_data =host_read_trout_reg((0x10FF<<2));
    *reg_data =host_read_trout_reg((0x1FFF<<2));
	#else // ranqing.wu mod 20130128
    *reg_data = host_read_trout_reg(reg_addr<<2);
    
    //*reg_data =host_read_trout_reg((0x10FF<<2));
    //*reg_data =host_read_trout_reg((0x1FFF<<2));
	#endif
}

INLINE void  rf_reg_write(UWORD32 reg_addr, UWORD32 reg_data)
{
	UWORD32 temp_reg_data;
	UWORD32 temp_reg_addr;

	temp_reg_addr = reg_addr;
	
    reg_addr = ((reg_addr & 0x0000ffff) | 0x1000);

    host_write_trout_reg(reg_data, (reg_addr<<2));

	rf_reg_read(temp_reg_addr,&temp_reg_data);
	if(temp_reg_data != reg_data)
	{
		printk("rf_reg_write err!  reg_addr=%x,reg_data=%x.\n",temp_reg_addr,reg_data);
	}
	
}

//RX Gain Table enable
 INLINE void  table_en_initial(void)
{
	//WiFi TX table enable
	rf_reg_write(0x573, 0x9956);
	rf_reg_write(0x574, 0x1cee);
	rf_reg_write(0x575, 0x2a00);

	//WiFi RX table enable
	rf_reg_write(0x579, 0xa956);
	rf_reg_write(0x57a, 0x1cee);
	rf_reg_write(0x57b, 0x2a00);

//#if 0 //dumy add 0129 for BT Coexist
	//bt rx table enable 
	rf_reg_write(0x057c,0x22b0);
	rf_reg_write(0x057d,0x0315);
	rf_reg_write(0x057e,0x5400);
//#endif 
	//WiFi-BT Coexist table enable
	rf_reg_write(0x057f,0x22d0);
	rf_reg_write(0x0580,0x1f3f);
	rf_reg_write(0x0581,0x7e00);
}

INLINE void host_mode_wifi_rx_init(void)
{
	rf_reg_write(0x0050, 0x0001);
	rf_reg_write(0x0107, 0x0010);
	rf_reg_write(0x00d2, 0xef3f);
	rf_reg_write(0x00d4, 0x0000);
	rf_reg_write(0x00e4, 0x001a);
	rf_reg_write(0x0131, 0x0003);//host mode, wf rx mode. 
	//bw_agc_mode_select( hostMode, wfRxState, 0x181b, 0);

	rf_reg_write(0x00d5, 0x002a);//48 bits rf mode control signal.          
	rf_reg_write(0x00e4, 0x001a);            
	rf_reg_write(0x00d7, 0x7038);//48 bits rf mode control signal.               
	rf_reg_write(0x00d8, 0x0a04);//48 bits rf mode control signal. 
	//host_mode_48bits_ctrl_sigs(0x002a, 0x7038, 0x0a04);

	rf_reg_write(0x00d9, 0x0030);
	rf_reg_write(0x00e2, 0x181b);//rx gain.
	rf_reg_write(0x0016, 0x07fd);
	rf_reg_write(0x0017, 0x6781);
	rf_reg_write(0x0000, 0xfff7);
	rf_reg_write(0x0000, 0xffff);
}
/*
host mode wifi tx initial.
*/
INLINE void host_mode_wifi_tx_init(void)
{
	rf_reg_write(0x0050, 0x0001);
	rf_reg_write(0x0107, 0x0010);
	rf_reg_write(0x00d2, 0xef3f);
	rf_reg_write(0x00d4, 0x0000);

	rf_reg_write(0x0131, 0x0001);//host mode, wf tx mode. 
	//bw_agc_mode_select( hostMode, wfTxState, 0x0005, 0);

	rf_reg_write(0x0044, 0x0004);
	rf_reg_write(0x0045, 0x0400);
	rf_reg_write(0x0047, 0x0055);
	rf_reg_write(0x0048, 0x0066);
	rf_reg_write(0x0040, 0x0004);

	rf_reg_write(0x00d7, 0x2700);//48 bits rf mode control signal.  0x2300
	rf_reg_write(0x00d8, 0x6899);//48 bits rf mode control signal.    0x7899
	//host_mode_48bits_ctrl_sigs(0x0000, 0x2300, 0x7899);

	rf_reg_write(0x003d, 0x0001 ); 

	rf_reg_write(0x00e3, 0x0005);//tx gain.
	rf_reg_write(0x0018, 0x847e);
	rf_reg_write(0x00dd, 0xa15a);
	rf_reg_write(0x00de, 0x0f0f);
	rf_reg_write(0x00df, 0x8c50);
	rf_reg_write(0x00e0, 0x1407);
	rf_reg_write(0x0000, 0xfff7);
	rf_reg_write(0x0000, 0xffff);
	rf_reg_write(0x0044, 0x0004);
	rf_reg_write(0x0045, 0x0400);
	rf_reg_write(0x0047, 0x0066);
	rf_reg_write(0x0040, 0x0004);
	rf_reg_write(0x0009, 0x0200);
}
//%%%%%------------------------------------
INLINE void  castor_set_bt_rx_host(void)  // fd auto calibration
{
	rf_reg_write(0x0050,0x0001);  // fd auto calibration
	rf_reg_write(0x0109,0x0800);  // fd tunig enable

	rf_reg_write(0x0000,0xffff);
	rf_reg_write(0x00d2,0xe33f);  //
	rf_reg_write(0x00d4,0x0000);  // all ldos set 1.3V
	rf_reg_write(0x0131,0x0004);  // set soc to bt rx mode

	rf_reg_write(0x00d5,0x0015);  // enable lo_nolap,bt lobuf,swbias
	rf_reg_write(0x00e4,0x021a);  // set bt tia res 300ohm, wifi tia res 1kohm

	rf_reg_write(0x00d7,0xa0c0);  // enable rx bias, filter,pga, tia/pga dcoc
	rf_reg_write(0x00d8,0x0d44);  // enable Rx enable, tia ,bt_en,
	rf_reg_write(0x00d9,0x0030);  // enable sw

	rf_reg_write(0x00e2,0x2ab3);  // pga 24dB,tia 22dB,lna 24dB
	rf_reg_write(0x0011,0x8080);  // pga dcoc
	rf_reg_write(0x0012,0x8080);  // tia dcoc
	// rbus_write('00db','00c1','6A');  // lna bias/cas voltage
	// rbus_write('00dc','a4a4','6A');  // lna bias
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	rf_reg_write(0x00db,0x00c1);  // lna bias/cas voltage
	rf_reg_write(0x00dc,0x3420);  // lna bias

	rf_reg_write(0x0133,0x0000);  //  set to manual rbus table
}
//dcoc 
// DCOC

INLINE void wifi_mxd_rf_tx_enable(void)
/*
WIFI tx need write WIFI REG 0x55:0xC,we need confirm rf reg:0x133
*/
{  
	rf_reg_write(0x0050, 0x0001);  // 
	rf_reg_write(0x0109, 0x0800);  // fd tunig enable;fd auto calibration
	rf_reg_write(0x0000, 0xffff);  // rest all statemachine
	
	//rf_reg_write(0x0131, 0x0001);  // set soc to wifi tx mode
       rf_reg_write(0x0133, 0x3ffc);// enable MXD wifi tx to host gain mode
   
	rf_reg_write(0x00d2, 0x6c2b);  // enable ldo pa,tx,lcpll,wifi lo,bt cp/lo
	rf_reg_write(0x00d4, 0x0000);  // all ldos set 1.3V
	rf_reg_write(0x00d8, 0x7899);  // enable tx filter,pga,txbias,lobuf,modulator,pa,pdet
	rf_reg_write(0x00e3, 0x0005);  // pga 0dB,modulator 0dB 
	rf_reg_write(0x0018, 0x847e);  // loft dcoc setup
	rf_reg_write(0x00d9, 0x009f);  // pad,pa and sw config
	rf_reg_write(0x00dd, 0xa15a);
	rf_reg_write(0x00de, 0x0f0f);
	rf_reg_write(0x00df, 0x8c50);
	rf_reg_write(0x00e0, 0x1407);
	
	rf_reg_write(0x0018, 0x8080);   // loft cfg
}

INLINE void maxscend_wifi_sprd_table_sel_cfg(void)
{

}

INLINE UWORD32 trout_floor(UWORD32 a)
{  
    return ((a*10)/10);
}

#if 0
/*
WIFI lo cfg
*/
INLINE void  maxscend_wifi_rf_lo_cfg(UWORD32 flo)
{
	UWORD32 tempread;
	UWORD32 vco_lock, vco_slow;
	int i,j;
	UWORD16 tune[131][3];
	UWORD16 afc_mode;
	UWORD16 tune_ok;
	UWORD16 tune_ok_update;
	UWORD32 cnt_lock, bypasssdm, divn_10msb, divn_16lsb, afc_cnt_target;
	#if 1
	UWORD32 divn, divn_castor, divn_err, freq_err_khz;
	#else
	double divn, divn_castor, divn_err, freq_err_khz;
	#endif
	UWORD32 divn_hex,afc_tw;


	bypasssdm = 0;
	//afc_mode = 0;
	printk("castor_wifi_lo_cfg: begin search ...:%d\n", flo);
	// rf_reg_read(0x0d8, &tempread);
	// tempread |= 0x7000;
	//rf_reg_write(0x0d8, tempread); 
	rf_reg_read(0x0d2, &tempread); //reg0d2 = bitand(rbus_read('6A','d2'),61439); //mask = 1110_F_F_F, turn off ring_ldo
	tempread &= 0xecff;
	tempread |= 0x0c00;
	rf_reg_write(0x0d2, tempread);  //rbus_write('d2', dec2hex(bitor(reg0d2,3*2^10)), '6A'); //0x0d2[11:10]: enable LDO for WIFI
	rf_reg_read(0x0d7, &tempread);  //reg0d7 = rbus_read('6A','d7');
	//tempread&= 0xefff;//
	tempread |= 0x1000;
	rf_reg_write(0x0d7, tempread);  //rbus_write('d7', dec2hex(bitor(reg0d7,2^12)), '6A'); //0x0d7[12]: lo_mixer=1
	rf_reg_write(0x0f2, 0x0410);  //410

	//castor_bt_lo_cfg(flo);
	//printk("use bt loop for wifi!\r\n");
#if 1
	rf_reg_write(0x0f3, 0x0000); //rbus_write('f3', '0', '6A'); //disable lo_test


	rf_reg_read(0x0ed, &tempread); //reg0ed = bitand(rbus_read('6A','ed'),hex2dec('F0FF')); // mask = F0FF
	tempread&= 0xf0ff;
	//Icp = 3; // 0~15
	tempread += 0x0300; //reg0ed+Icp*2^8
	rf_reg_write(0x0ed, tempread); //rbus_write('ed', dec2hex(reg0ed+Icp*2^8), '6A'); //


	rf_reg_read(0x0ef, &tempread);  //reg0ef = bitand(rbus_read('6A','ef'),hex2dec('FF00')); // mask = FF00
	tempread&= 0xff00;
	tempread += 16;
	rf_reg_write(0x0ef, tempread); //rbus_write('ef', dec2hex(reg0ef+16*2^0), '6A'); //


	rf_reg_read(0x0f1, &tempread); //reg0f1 = bitand(rbus_read('6A','f1'),hex2dec('1FC0')); //mask = 1_1111_1100_0000
	tempread&= 0x1fc0;
	tempread+=31;
	rf_reg_write(0x0f1, tempread);  //rbus_write('f1', dec2hex(reg0f1+31*2^0), '6A'); //lc_pll_vco_res = '011111'

	if(bypasssdm == 0)
	{
		rf_reg_write(0x065, 0x0000);  //sdm on
#if 1
		divn = (UWORD32)flo/26;	
		//printk("WIFI divn: %.3f  \n", divn);
		divn_hex = trout_floor(divn*(0x01 <<18));
#else
		divn = (double)flo/26;	
		//printk("WIFI divn: %.3f  \n", divn);
		divn_hex = my_floor(divn*(0x01 <<18));		
#endif		
		//printk("WIFI divn_hex: 0x%x \n", divn_hex);
		divn_hex &= 0x0fffffff;
		divn_10msb = (divn_hex >>16)&0x0fff;
		//printk("WIFI divn_hex 60: 0x%x \n", (divn_hex >>16)&0x0fff);
		divn_16lsb = ((divn_hex&0x0000ff00)>>8) |(divn_hex&0x0000ff00 );
		//printk("WIFI divn_hex 61: 0x%x \n", ((divn_hex&0x0000ff00)>>8) |(divn_hex&0x0000ff00 ));
#if 1
		divn_castor= ((UWORD32)((divn_10msb << 16) + divn_16lsb)/(262144));
#else
		divn_castor= ((double)((divn_10msb << 16) + divn_16lsb)/(262144));
#endif
	}
	else
	{
		rf_reg_write(0x065, 0x000f); 
		rf_reg_write(0x0ed, 0x000f); 
		rf_reg_write(0x0ef, 0x000f); 
		#if 1
		divn = (UWORD32)flo/26;
		divn_castor = trout_floor(divn);
		#else
		divn = (double)flo/26;
		divn_castor = my_floor(divn);
		#endif
		divn_10msb = ((UWORD16)(divn_castor*4))&0x0fff;
		divn_16lsb = 0x0000;
	}

        divn_err = divn_castor - divn;
	freq_err_khz = divn_err*26000;
	//printk("flo = %.4f  MHZ, freqErr = %.3f KHZ\r\n",  divn_castor*26, freq_err_khz);
	#if 0
	rf_reg_write(0x060, divn_10msb); 
	rf_reg_write(0x061, divn_16lsb); 
	#else
	rf_reg_write(0x060, 0x176); //dumy add for debug 0117
	rf_reg_write(0x061, 0xECEC); //dumy add for debug 0117
	#endif
	

	//fvco_target
	afc_tw = 3; // 0/1/2/3 ---> 32/64/128/256 clk
	afc_mode = 0; // 0(bypass) 1(fm) 2(bt) 3(wifi)
	rf_reg_write(0x070, afc_tw + afc_mode*4); 
	tempread = (UWORD16)((divn_castor/4)*8*32);
	//printk("tempread:0x%x\r\n", tempread);

	afc_cnt_target = tempread;  //0x178f;  //round(((divn/4)*(0x0001<<afc_tw)*32));  //round((divn/4)*2^afc_tw*32); 
	//printk("afc_cnt_target:0x%x\r\n", afc_cnt_target);
	

	if (afc_cnt_target>=(0x0001<<14)) // 2^14
	{
	    //printk("afc_cnt_target = %d > 14bits!\n",afc_cnt_target);
	}
	#if 0
	rf_reg_write(0x074, afc_cnt_target&0xffff); //rbus_write('74', dec2hex(afc_cnt_target,4), '6A'); //afc_cnt_target
	#else
	rf_reg_write(0x074, 0x176E);//dumy add for debug 0117
	#endif
	rf_reg_read(0x0000, &tempread); 
  printk("reg 0x0000:0x %x \n",tempread);
	tempread|= 0x207;
  printk("reg 0x0000:0x %x \n",tempread);
	rf_reg_write(0x0000, tempread); 


	if (afc_mode == 0)
	{
	    // vco_tune linear search: find the first 1->0 transition
	    for(i = 0; i<131; i++)
	    for(j =0; j<3; j++)
      {
          	tune[i][j]= 1;
      }
	    //vco_slow_lock_list(:,1) = tune;

			for(j =0; j<131; j++)
			{
				tune[j][0]= 55 + j;
			}
			
		
	    tune_ok = 255;
	    tune_ok_update = 0;
	    cnt_lock = 0;
		
	    for (i = 0; i<= (185- 55); i++) // (i = 1:length(tune); i++)
	    {
	        rf_reg_write(0x073, tune[i][0]); //rbus_write('73', dec2hex(tune(i)), '6A'); //wifi_tune
	        udelay(10);//pause(0.01)
	        rf_reg_read(0x0fb, &tempread);  //reg0fb = rbus_read('6A','fb');
	        //vco_slow = bitand(reg0fb,32) == 32; //wifi slow: 0xfb[5] == 1
	        if((tempread &0x20) == 0x20)
	        {
	        	 vco_slow = 1;
	        }
		 else
		 {
			 vco_slow = 0;
		 }
		 //vco_lock =  bitand(reg0fb,2) == 2; //wifi lock: 0xfb[1] == 1
		 if((tempread &0x02) == 0x02)
     {
        vco_lock = 1;
     }
		 else
		 {
			 vco_lock = 0;
		 }
	   //vco_slow_lock_list(i,2:3) = [vco_slow vco_lock];
	   tune[i][1] = vco_slow;
		 tune[i][2] = vco_lock;

		cnt_lock = cnt_lock*vco_lock + vco_lock;
			
		//printk("tune=%d; slow=%d; lock=%d\n", tune[i][0], tune[i][1], tune[i][2]);      //delete, 2013.1.5, Ranqing.Wu 
		
		if ((i>0) && (cnt_lock >= 3) &&(tune_ok_update == 0) && ((tune[i-1][1] + tune[i][1])==2))
		{
				tune_ok_update = 1;
				tune_ok = tune[i][0];
				//printk("tune_ok = %d\n",tune_ok);
		}

			if(tune[i][0] > (tune_ok + 20))
				break;
			}
		
	    rf_reg_write(0x073, tune_ok); //rbus_write('73', dec2hex(tune_ok), '6A'); // 0~ff tune_wifi
	}
	else
	{
	     rf_reg_write(0x002, 0x0002);  //rbus_write('2', '2', '6A'); //start afc

	     // bypass afc
	     udelay(300);//pause(.3)
	     rf_reg_write(0x070, 0x0000);  //rbus_write('70', '0', '6A'); // bypass afc

	     // bt_tune when bypass afc
	     rf_reg_read(0x077, &tempread);
	     rf_reg_write(0x073, tempread); 
	}

	udelay(100);//pause(.1)
	//vco_lock = bitand(rbus_read('6A','fb'),2) == 2;
	rf_reg_read(0x0fb, &tempread);
	if((tempread & 0x0002) == 0x0002)
	{
	 	vco_lock =1;
	}
	else
	{
		 vco_lock = 0;
	}
	//printk("flo = %.4f MHz, freqErr = %.1fKHz\n",divn_castor*26,freq_err_khz);

	rf_reg_read(0x073, &tempread);
	//tune = tempread; //['0x' dec2hex(rbus_read('6A','73'))];

	if (vco_lock == 1)
	{
	    printk("WIFI LO lock to %d MHz ---> OK! (tune=0x%x)\n",divn_castor*26,tempread);
	}
	else
	{
	    printk("WIFI LO lock to %d MHz ---> fail! (tune=0x%x)\n",divn_castor*26,tempread);
	}
#endif
}
#else

// maxscend wifi frequence point table
/*
WIFI lo cfg
*/
INLINE void  maxscend_wifi_rf_lo_cfg_test(UWORD8 rf_channel)
{
	UWORD32 flo=2437;
	UWORD32 tempread;
	UWORD32 vco_lock, vco_slow;
	int i,j;
	UWORD16 tune[131][3];
	UWORD16 afc_mode;
	UWORD16 tune_ok;
	UWORD16 tune_ok_update;
	UWORD32 cnt_lock, bypasssdm, divn_10msb, divn_16lsb, afc_cnt_target;
#if 1
	UWORD32 divn, divn_castor, divn_err, freq_err_khz;
#else
	double divn, divn_castor, divn_err, freq_err_khz;
#endif
	UWORD32 divn_hex,afc_tw;

	UWORD32 reg_60[14] =
	{
		0x173,
		0x173,
		0x174,
		0x175,
		0x176,
		0x176,
		0x177,
		0x178,
		0x179,
		0x17A,
		0x17A,
		0x17B,
		0x17C,
		0x17D
	};

	UWORD32 reg_61[14] =
	{
		0x1313,
		0xD8D8,
		0x9D9D,
		0x6262,
		0x2727,
		0xECEC,
		0xB1B1,
		0x7676,
		0x3B3B,
		0x0,
		0xC4C4,
		0x8989,
		0x4E4E,
		0x1313
	};

	
	UWORD32 reg_74[14] =
	{
		0x1731,
		0x173D,
		0x1749,
		0x1756,
		0x1762,
		0x176E,
		0x177B,
		0x1787,
		0x1793,
		0x17A0,
		0x17AC,
		0x17B8,
		0x17C4,
		0x17D1
	};

	if((rf_channel <= 0) || (rf_channel > 14))
	{
		printk("%s: set channel failed, invalid rf channel = %d\n",
			__FUNCTION__,rf_channel);
		return;
	}

	bypasssdm = 0;
	//afc_mode = 0;
	printk("castor_wifi_lo_cfg: begin search ...:%d\n", flo);
	// rf_reg_read(0x0d8, &tempread);
	// tempread |= 0x7000;
	//rf_reg_write(0x0d8, tempread); 
	rf_reg_read(0x0d2, &tempread); //reg0d2 = bitand(rbus_read('6A','d2'),61439); //mask = 1110_F_F_F, turn off ring_ldo
	tempread &= 0xecff;
	tempread |= 0x0c00;
	rf_reg_write(0x0d2, tempread);	//rbus_write('d2', dec2hex(bitor(reg0d2,3*2^10)), '6A'); //0x0d2[11:10]: enable LDO for WIFI
	rf_reg_read(0x0d7, &tempread);	//reg0d7 = rbus_read('6A','d7');
	//tempread&= 0xefff;//
	tempread |= 0x1000;
	rf_reg_write(0x0d7, tempread);	//rbus_write('d7', dec2hex(bitor(reg0d7,2^12)), '6A'); //0x0d7[12]: lo_mixer=1
	rf_reg_write(0x0f2, 0x0410);  //410

	//castor_bt_lo_cfg(flo);
	//printk("use bt loop for wifi!\r\n");
#if 1
	rf_reg_write(0x0f3, 0x0000); //rbus_write('f3', '0', '6A'); //disable lo_test


	rf_reg_read(0x0ed, &tempread); //reg0ed = bitand(rbus_read('6A','ed'),hex2dec('F0FF')); // mask = F0FF
	tempread&= 0xf0ff;
	//Icp = 3; // 0~15
	tempread += 0x0300; //reg0ed+Icp*2^8
	rf_reg_write(0x0ed, tempread); //rbus_write('ed', dec2hex(reg0ed+Icp*2^8), '6A'); //


	rf_reg_read(0x0ef, &tempread);	//reg0ef = bitand(rbus_read('6A','ef'),hex2dec('FF00')); // mask = FF00
	tempread&= 0xff00;
	tempread += 16;
	rf_reg_write(0x0ef, tempread); //rbus_write('ef', dec2hex(reg0ef+16*2^0), '6A'); //


	rf_reg_read(0x0f1, &tempread); //reg0f1 = bitand(rbus_read('6A','f1'),hex2dec('1FC0')); //mask = 1_1111_1100_0000
	tempread&= 0x1fc0;
	tempread+=31;
	rf_reg_write(0x0f1, tempread);	//rbus_write('f1', dec2hex(reg0f1+31*2^0), '6A'); //lc_pll_vco_res = '011111'

	if(bypasssdm == 0)
	{
		rf_reg_write(0x065, 0x0000);  //sdm on
#if 1
		divn = (UWORD32)flo/26; 
		//printk("WIFI divn: %.3f  \n", divn);
		divn_hex = trout_floor(divn*(0x01 <<18));
#else
		divn = (double)flo/26;	
		//printk("WIFI divn: %.3f  \n", divn);
		divn_hex = my_floor(divn*(0x01 <<18));		
#endif		
		//printk("WIFI divn_hex: 0x%x \n", divn_hex);
		divn_hex &= 0x0fffffff;
		divn_10msb = (divn_hex >>16)&0x0fff;
		//printk("WIFI divn_hex 60: 0x%x \n", (divn_hex >>16)&0x0fff);
		divn_16lsb = ((divn_hex&0x0000ff00)>>8) |(divn_hex&0x0000ff00 );
		//printk("WIFI divn_hex 61: 0x%x \n", ((divn_hex&0x0000ff00)>>8) |(divn_hex&0x0000ff00 ));
#if 1
		divn_castor= ((UWORD32)((divn_10msb << 16) + divn_16lsb)/(262144));
#else
		divn_castor= ((double)((divn_10msb << 16) + divn_16lsb)/(262144));
#endif
	}
	else
	{
		rf_reg_write(0x065, 0x000f); 
		rf_reg_write(0x0ed, 0x000f); 
		rf_reg_write(0x0ef, 0x000f); 
	#if 1
		divn = (UWORD32)flo/26;
		divn_castor = trout_floor(divn);
	#else
		divn = (double)flo/26;
		divn_castor = my_floor(divn);
	#endif
		divn_10msb = ((UWORD16)(divn_castor*4))&0x0fff;
		divn_16lsb = 0x0000;
	}

		divn_err = divn_castor - divn;
	freq_err_khz = divn_err*26000;
	//printk("flo = %.4f	MHZ, freqErr = %.3f KHZ\r\n",  divn_castor*26, freq_err_khz);


#if 0
	rf_reg_write(0x060, divn_10msb); 
	rf_reg_write(0x061, divn_16lsb); 
#else
	rf_reg_write(0x060, reg_60[rf_channel-1]); //dumy add for debug 0117
	rf_reg_write(0x061, reg_61[rf_channel-1]); //dumy add for debug 0117
#endif
	

	//fvco_target
	afc_tw = 3; // 0/1/2/3 ---> 32/64/128/256 clk
	afc_mode = 0; // 0(bypass) 1(fm) 2(bt) 3(wifi)
	rf_reg_write(0x070, afc_tw + afc_mode*4); 
	tempread = (UWORD16)((divn_castor/4)*8*32);
	//printk("tempread:0x%x\r\n", tempread);

	afc_cnt_target = tempread;	//0x178f;  //round(((divn/4)*(0x0001<<afc_tw)*32));  //round((divn/4)*2^afc_tw*32); 
	//printk("afc_cnt_target:0x%x\r\n", afc_cnt_target);
	

	if (afc_cnt_target>=(0x0001<<14)) // 2^14
	{
		//printk("afc_cnt_target = %d > 14bits!\n",afc_cnt_target);
	}
#if 0
	rf_reg_write(0x074, afc_cnt_target&0xffff); //rbus_write('74', dec2hex(afc_cnt_target,4), '6A'); //afc_cnt_target
#else
	rf_reg_write(0x074, reg_74[rf_channel-1]);//dumy add for debug 0117
#endif
	rf_reg_read(0x0000, &tempread); 
  printk("reg 0x0000:0x %x \n",tempread);
	tempread|= 0x207;
  printk("reg 0x0000:0x %x \n",tempread);
	rf_reg_write(0x0000, tempread); 


	if (afc_mode == 0)
	{
		// vco_tune linear search: find the first 1->0 transition
		for(i = 0; i<131; i++)
		for(j =0; j<3; j++)
	  {
			tune[i][j]= 1;
	  }
		//vco_slow_lock_list(:,1) = tune;

			for(j =0; j<131; j++)
			{
				tune[j][0]= 55 + j;
			}
			
		
		tune_ok = 255;
		tune_ok_update = 0;
		cnt_lock = 0;
		
		for (i = 0; i<= (185- 55); i++) // (i = 1:length(tune); i++)
		{
			rf_reg_write(0x073, tune[i][0]); //rbus_write('73', dec2hex(tune(i)), '6A'); //wifi_tune
			udelay(10);//pause(0.01)
			rf_reg_read(0x0fb, &tempread);	//reg0fb = rbus_read('6A','fb');
			//vco_slow = bitand(reg0fb,32) == 32; //wifi slow: 0xfb[5] == 1
			if((tempread &0x20) == 0x20)
			{
				 vco_slow = 1;
			}
		 else
		 {
			 vco_slow = 0;
		 }
		 //vco_lock =  bitand(reg0fb,2) == 2; //wifi lock: 0xfb[1] == 1
		 if((tempread &0x02) == 0x02)
	 {
		vco_lock = 1;
	 }
		 else
		 {
			 vco_lock = 0;
		 }
	   //vco_slow_lock_list(i,2:3) = [vco_slow vco_lock];
	   tune[i][1] = vco_slow;
		 tune[i][2] = vco_lock;

		cnt_lock = cnt_lock*vco_lock + vco_lock;
			
		//printk("tune=%d; slow=%d; lock=%d\n", tune[i][0], tune[i][1], tune[i][2]);	  //delete, 2013.1.5, Ranqing.Wu 
		
		if ((i>0) && (cnt_lock >= 3) &&(tune_ok_update == 0) && ((tune[i-1][1] + tune[i][1])==2))
		{
				tune_ok_update = 1;
				tune_ok = tune[i][0];
				//printk("tune_ok = %d\n",tune_ok);
		}

			if(tune[i][0] > (tune_ok + 20))
				break;
			}
		
		rf_reg_write(0x073, tune_ok); //rbus_write('73', dec2hex(tune_ok), '6A'); // 0~ff tune_wifi
	}
	else
	{
		 rf_reg_write(0x002, 0x0002);  //rbus_write('2', '2', '6A'); //start afc

		 // bypass afc
		 udelay(300);//pause(.3)
		 rf_reg_write(0x070, 0x0000);  //rbus_write('70', '0', '6A'); // bypass afc

		 // bt_tune when bypass afc
		 rf_reg_read(0x077, &tempread);
		 rf_reg_write(0x073, tempread); 
	}

	udelay(100);//pause(.1)
	//vco_lock = bitand(rbus_read('6A','fb'),2) == 2;
	rf_reg_read(0x0fb, &tempread);
	if((tempread & 0x0002) == 0x0002)
	{
		vco_lock =1;
	}
	else
	{
		 vco_lock = 0;
	}
	//printk("flo = %.4f MHz, freqErr = %.1fKHz\n",divn_castor*26,freq_err_khz);

	rf_reg_read(0x073, &tempread);
	//tune = tempread; //['0x' dec2hex(rbus_read('6A','73'))];

	//rf_reg_write(0x070, 0x09); //dumy add 0129 for BT 

	if (vco_lock == 1)
	{
		printk("WIFI LO lock to %d MHz ---> OK! (tune=0x%x)\n",divn_castor*26,tempread);
	}
	else
	{
		printk("WIFI LO lock to %d MHz ---> fail! (tune=0x%x)\n",divn_castor*26,tempread);
	}
#endif
}


#endif

INLINE void castor_dcoc(UWORD16 varargin)
{
	// 0) Selction
	//Tw = 0; // Wait Time(Second)
	UWORD16 AvgNum = 10; // Average Num
	UWORD16 HorL;
	#if 1
	UWORD32 tmp,tmps,tmpstr;
	#else
	UWORD16 tmp,tmps, tmpstr;
	#endif
	UWORD16 TxFlg;
	UWORD16 config_addr;
	UWORD16 BTWifiFlg;
	int k,ll;
	int dc_level_tx,dc_level,dc_level_hist;
	UWORD16 PreValue;
	UWORD16 SEL;
	int signflg;


	//printk("%s\r\n", __func__);
	SEL = varargin;
	// [13] DCOCSW_FM_RX_PGA_I: ¡¡ 
	// [12] DCOCSW_FM_RX_PGA_Q: 
	// [11] DCOCSW_BT_RX_PGA_I: ¡¡ 
	// [10] DCOCSW_BT_RX_PGA_Q: ¡¡ 
	// [9] DCOCSW_BT_RX_TIA_I: ¡¡ 
	// [8] DCOCSW_BT_RX_TIA_Q: ¡¡ 
	// [7] DCOCSW_BT_TX_PGA_I: ¡¡ 
	// [6] DCOCSW_BT_TX_PGA_Q: ¡¡ 
	// [5] DCOCSW_WIFI_RX_PGA_I: ¡¡ 
	// [4] DCOCSW_WIFI_RX_PGA_Q: ¡¡ 
	// [3] DCOCSW_WIFI_RX_TIA_I: ¡¡ 
	// [2] DCOCSW_WIFI_RX_TIA_Q: ¡¡ 
	// [1] DCOCSW_WIFI_TX_PGA_I: ¡¡ 
	// [0] DCOCSW_WIFI_TX_PGA_Q: 

	// 0xd3 = 0x32
	//tmps=rbus_read('6A','0d3');
	rf_reg_read(0x00d3, &tmps);
	//rbus_write('0d3', dec2hex( double( bitor(UWORD16(tmps),UWORD16(hex2dec('32')))) ), '6A');
	rf_reg_write(0x00d3, (tmps| 0x32)); 

	// 0xd2 <2><5>=1
	//tmpstr = dec2bin(rbus_read('6A','0d2'),16);
	rf_reg_read(0x00d2, &tmps);

	//tmpstr(end-5) = '1';
	//tmpstr(end-2) = '1';
	tmps |= 0x0024;
	//rbus_write('0d2', dec2hex(bin2dec(tmpstr)), '6A');
	rf_reg_write(0x00d2, tmps); 

	// 0x07 =0x200
	//rbus_write('007', dec2hex(hex2dec('200')), '6A');
	rf_reg_write(0x0007, 0x0200); 

	TxFlg =0;
	//printk("SEL:0x%x\r\n", SEL);
	#if 1
	tmps = trout_floor(((UWORD32)SEL)/2);
	#else
	tmps = my_floor(((double)SEL)/2);
	#endif
	//printk("tmps:0x%x\r\n", tmps);
	
	switch (tmps)
	{
	case 0: // WIFI TX PGA I/Q
		 config_addr = 0x0018; //'018';
		 TxFlg = 1;
		 BTWifiFlg = 0;
		 break;
	//      tmpstr = dec2bin(rbus_read('6A','0d8'),16);
	//      tmpstr(4) = '1';
	//      rbus_write('0d8', dec2hex(bin2dec(tmpstr)), '6A');
	case 1: // WIFI RX TIA I/Q
		 config_addr = 0x0017; //'017';
	     break;
	//      tmpstr = dec2bin(rbus_read('6A','0d7'),16);
	//      tmpstr(2) = '1';
	//      rbus_write('0d7', dec2hex(bin2dec(tmpstr)), '6A');
	//      
	case 2: // WIFI RX PAG I/Q
		 config_addr = 0x0016; //'016';
	     break;
	//      tmpstr = dec2bin(rbus_read('6A','0d7'),16);
	//      tmpstr(end-3) = '1';
	//      rbus_write('0d7', dec2hex(bin2dec(tmpstr)), '6A');
	case 3: // BT TX PGA I/Q
		 config_addr = 0x0013; //'013';
		 TxFlg = 1;
		 BTWifiFlg = 1;
	     break;
	//      tmpstr = dec2bin(rbus_read('6A','0d7'),16);
	//      tmpstr(end) = '1';
	//      rbus_write('0d7', dec2hex(bin2dec(tmpstr)), '6A');
	//      
	case 4: // BT RX TIA I/Q
		 config_addr = 0x0012; //'012';
		 //tmpstr = dec2bin(rbus_read('6A','0d0'),16);
		 rf_reg_read(0x00d0, &tmpstr);
		 //tmpstr(end-2) = '0';
		 tmpstr &= 0xfffb;
		 //rbus_write('0d0', dec2hex(bin2dec(tmpstr)), '6A');
		 rf_reg_write(0x00d0, tmpstr); 
	     break;
	//      tmpstr = dec2bin(rbus_read('6A','0d7'),16);
	//      tmpstr(1) = '1';
	//      rbus_write('0d7', dec2hex(bin2dec(tmpstr)), '6A');
	//      
	     
	case 5: // BT RX PGA I/Q
		 config_addr = 0x0011; //'011';
		 //tmpstr = dec2bin(rbus_read('6A','0d0'),16);
		 rf_reg_read(0x00d0, &tmpstr);
		 //tmpstr(end-2) = '0';
	     tmpstr &= 0xfffb;
		 //rbus_write('0d0', dec2hex(bin2dec(tmpstr)), '6A');
		  rf_reg_write(0x00d0, tmpstr); 
	     break;
	//      tmpstr = dec2bin(rbus_read('6A','0d7'),16);
	//      tmpstr(end-6) = '1';
	//      rbus_write('0d7', dec2hex(bin2dec(tmpstr)), '6A');

	case 6: // FM RX I/Q
		   config_addr = 0x0015; //'015';  
		   break;
	default:
		break;
	}


	// 1) If TX DCOC 
	if (TxFlg==1)
	{
		// 0x0d1, 0x0d0  switch on
		//tmpstr = dec2bin(rbus_read('6A','0d0'),16);
		//tmpstr(end-1) = '1';
		//rbus_write('0d0', dec2hex(bin2dec(tmpstr)), '6A');
		rf_reg_read(0x00d0, &tmpstr);
	    tmpstr |= 0x0002;
		rf_reg_write(0x00d0, tmpstr); 

		//tmpstr = dec2bin(rbus_read('6A','0d1'),16);
		//tmpstr(end) = '1';
		//rbus_write('0d1', dec2hex(bin2dec(tmpstr)), '6A');
		rf_reg_read(0x00d1, &tmpstr);
	    tmpstr |= 0x0001;
		rf_reg_write(0x00d1, tmpstr); 

		// 0xd7 , 0xd8  switch off
		//tmpstr = dec2bin(rbus_read('6A','0d7'),16);
		//tmpstr(end-2:end) = '000';
		//rbus_write('0d7', dec2hex(bin2dec(tmpstr)), '6A');
		rf_reg_read(0x00d7, &tmpstr);
	    tmpstr &= 0xfff8;
		rf_reg_write(0x00d7, tmpstr); 

		//tmpstr = dec2bin(rbus_read('6A','0d8'),16);
		//tmpstr([2:4]) = '000';
		//rbus_write('0d8', dec2hex(bin2dec(tmpstr)), '6A');
		rf_reg_read(0x00d8, &tmpstr);
	    tmpstr &= 0xfff1;
		rf_reg_write(0x00d8, tmpstr); 
	        
	    
		// read DC level
		dc_level = 0;
		for( k=1;k<=AvgNum; k++)
		{
			//tmp=rbus_read('6A','1e');
			rf_reg_read(0x001e, &tmp);
			if (tmp > 511)
			{
				//dc_level =dc_level + double(tmp) -2^10;
				dc_level =dc_level + tmp - ( 0x0001 << 10);
			}
			else
			{
				//dc_level =dc_level + double(tmp);
				dc_level =dc_level + tmp;
			}
			//pause( Tw );
		}

		dc_level_tx = dc_level/AvgNum;

		// 0x0d1 switch off
		//tmpstr = dec2bin(rbus_read('6A','0d1'),16);
		//tmpstr(end) = '0';
		//rbus_write('0d1', dec2hex(bin2dec(tmpstr)), '6A');
		rf_reg_read(0x00d1, &tmpstr);
	    tmpstr &= 0xfffe;
		rf_reg_write(0x00d1, tmpstr); 
	}
	else
	{
		dc_level_tx = 0;
	}

	if (TxFlg==1)
	{
		if (BTWifiFlg==1) // 0xd7 switch on when  BT 
		{
			   //tmpstr = dec2bin(rbus_read('6A','0d7'),16);
			   //tmpstr(end-2:end) = '111';
			   //rbus_write('0d7', dec2hex(bin2dec(tmpstr)), '6A');
			   rf_reg_read(0x00d7, &tmpstr);
			   tmpstr |= 0x0007;
			   rf_reg_write(0x00d7, tmpstr); 
		}
		else // 0xd8 switch on when Wifi
		{
			   //tmpstr = dec2bin(rbus_read('6A','0d8'),16);
			   //tmpstr([2:4]) = '111';
			   //rbus_write('0d8', dec2hex(bin2dec(tmpstr)), '6A');
			   rf_reg_read(0x00d8, &tmpstr);
			   tmpstr |= 0x7000;
			   rf_reg_write(0x00d8, tmpstr); 
		}
	}

	#if 1
	if( trout_floor((UWORD32)SEL/2) == 6 )
	#else
	if( my_floor((double)SEL/2) == 6 )
	#endif
	{
		//rbus_write('010', dec2hex(0), '6A');
		//printk("come here!/r/n");
		 rf_reg_write(0x0010, 0x0000); 
	}
	else
	{
		//rbus_write('010', dec2hex(2^SEL), '6A');
		//printk("come here1!/r/n");
		rf_reg_write(0x0010, (0x0001<<SEL)); 
	}

	//HorL = mod(SEL,2); // I or Q
	HorL = SEL%2;


	// 2) Start DCOC Loop 
	// 2.1 Set all'0', and Read ADC DC Level For Ref
	PreValue = 0;
	//tmpstr = dec2bin(rbus_read('6A',config_addr),16);
	rf_reg_read(config_addr, &tmpstr);
	//printk("------>>>>>HorL Value :0x%x\r\n", HorL);
	if (HorL==1) // For I
	{
		//tmpstr([1:8])=dec2bin(0,8);
		tmpstr&= 0x00ff;
	}
	else         // For Q
	{
		//tmpstr(8+[1:8])=dec2bin(0,8);
		tmpstr&= 0xff00;
	}
	//rbus_write(config_addr, dec2hex(bin2dec(tmpstr)), '6A');
	rf_reg_write(config_addr, tmpstr); 
	dc_level = 0;

	for(k=1; k<=AvgNum; k++)
	{
		if ( SEL == 12 )
		{
			//tmp=rbus_read('6A','1f');
			rf_reg_read(0x001f, &tmp);
		}
		else
		{
			//tmp=rbus_read('6A','1e');
			rf_reg_read(0x001e, &tmp);
		}
		if (tmp > 511)
		{
			//dc_level =dc_level + double(tmp) -2^10;
			dc_level =dc_level + tmp - (0x0001 << 10);
		}
		else
		{
			//dc_level =dc_level + double(tmp);
			dc_level =dc_level + tmp;
		}
		//printk("DCOC dc_level:0x%x\r\n", dc_level);
		//pause( Tw );
	}

	dc_level = dc_level/AvgNum;
	dc_level_hist = dc_level-dc_level_tx;
	//printk("DCOC dc_level:0x%x, dc_level_hist:0x%x \r\n", dc_level, dc_level_hist);
	signflg = 1;

	for( ll=7;ll >= 0;ll--) //for( ll=7:-1:0)
	{   
		// Set value +- 2^ll
		//tmpstr = dec2bin(rbus_read('6A',config_addr),16);
		//printk("DCOC ll:%d\r\n", ll);
		rf_reg_read(config_addr, &tmpstr);
		//printk("config_addr:0x%x, 0x%x, %d\r\n", config_addr, tmpstr,PreValue);
		//PreValue = PreValue + signflg*2^ll;
		if(signflg < 0)
			PreValue = PreValue -  (0x0001<<ll);
		else
			PreValue = PreValue + (0x0001<<ll);

		
		//printk("PreValue:0x%x\r\n", PreValue);
		if (HorL==1) // For I
		{
		   //tmpstr([1:8])=dec2bin(PreValue,8);
		   tmpstr &= 0x00ff;
		   tmpstr |= ((PreValue &0x00ff)<<8);

		}
		else         // For Q
		{
		   //tmpstr(8+[1:8])=dec2bin(PreValue,8);
		   tmpstr &= 0xff00;
		   tmpstr |= (PreValue &0x00ff);
		}
		//rbus_write(config_addr, dec2hex(bin2dec(tmpstr)), '6A');
		//printk("tmpstr:0x%x\r\n", tmpstr);
		rf_reg_write(config_addr, tmpstr); 
	    
	    
	   // Read DC Level again, and Check If Less than previous value
	   dc_level = 0;
	   for( k=1; k<=AvgNum; k++) //for( k=1:AvgNum)
	   {
	           //printk("DCOC k:0x%x\r\n", k);
		   if ( SEL == 12 )
		   {
			   //tmp=rbus_read('6A','1f');
			   rf_reg_read(0x001f, &tmp);
		   }
		   else
		   {
			   //tmp=rbus_read('6A','1e');
			   rf_reg_read(0x001e, &tmp);
		   }

		   if (tmp > 511)
		   {
			   //dc_level =dc_level + double(tmp) -2^10;
			   dc_level =dc_level + tmp - (0x0001 << 10);
		   }
		   else
		   {
			   //dc_level =dc_level + double(tmp);
			   dc_level =dc_level + tmp;
		   }
		   //pause( Tw );
	   }


	   dc_level = dc_level/AvgNum - dc_level_tx;
	    //printk("DCOC k after\r\n"); 
	   if (ll == 0) // last digit
	   {
		   if ( abs(dc_level) > abs(dc_level_hist) ) // set back if not good
		   {
			   //PreValue = PreValue - signflg*2^ll  11;

			   if(signflg < 0)
			   	  PreValue = PreValue + (0x0001<<ll);
			   else
			   	  PreValue = PreValue - (0x0001<<ll);

			   if (HorL==1) // For I
			   {
				   //tmpstr([1:8])=dec2bin(PreValue,8);
				   tmpstr &= 0x00ff;
				   tmpstr |= ((PreValue &0x00ff)<< 8);
			   }
			   else         // For Q
			   {
				   //tmpstr(8+[1:8])=dec2bin(PreValue,8);
				   tmpstr &= 0xff00;
				   tmpstr |= (PreValue &0x00ff) ;
			   }
		   }
	   }
	   else
	   {
		   if ( ( dc_level*dc_level_hist < 0 ) || ( abs(dc_level) > abs(dc_level_hist) ) )
		   {
			   //signflg = -sign(signflg);
				 signflg = -1*(signflg);
		   }
		   else
		   {
			   //signflg = sign(signflg);
				 signflg = 1*(signflg);
		    }
	   }
	   dc_level_hist = dc_level;

	}
	 //printk("DCOC k after 1\r\n"); 
	 #if 1
       tmpstr = trout_floor((UWORD32)(SEL)/2);
	   #else
	   tmpstr = my_floor((double)(SEL)/2);
	   #endif
	switch(tmpstr)
	{
		case 4: // BT RX TIA I/Q
			 config_addr = 0x0012; //'012';
			 //tmpstr = dec2bin(rbus_read('6A','0d0'),16);
			 //tmpstr(end-2) = '1';
			 //rbus_write('0d0', dec2hex(bin2dec(tmpstr)), '6A');
			 rf_reg_read(0x00d0, &tmpstr);
			 tmpstr |= 0x0004;
			 rf_reg_write(0x00d0, tmpstr); 
			 break;
		case 5: // BT RX PGA I/Q
			 config_addr = 0x0011; //'011';
			 //tmpstr = dec2bin(rbus_read('6A','0d0'),16);
			 //tmpstr(end-2) = '1';
			 //rbus_write('0d0', dec2hex(bin2dec(tmpstr)), '6A');
			 rf_reg_read(0x00d0, &tmpstr);
			 tmpstr |= 0x0004;
			 rf_reg_write(0x00d0, tmpstr); 
			 break;
		default:
			break;
	}

        //printk("DCOC k afte 2\r\n"); 
	//rbus_write('010', dec2hex(0), '6A');
	rf_reg_write(0x0010, 0x0000); 
	//printk("DC calibration done !!\r\n");
	//databack = PreValue;
	//aa=0;
}

INLINE void  bt_table_auto_write(void)
{
    // for each gain value in host mode, cal the DCOC value,
    // write rbus to set gain and DCOC table and switch to mxd mode,
    // then test noise figure, gain and IP3
    //clc;
    UWORD16 tmp = 0;
    UWORD16 gain_table_id;
    int ii;
	UWORD32 tempread;
    UWORD16 gain_table_value,tempValue,gain_value,bt_tia_dcoc_tmp,bt_pga_dcoc_tmp,tia_dcoc,bt_pga_dcoc_l,bt_tia_dcoc_h;
    UWORD16 pga_dcoc,bt_pga_dcoc_h,tempAddr,bt_tia_dcoc_l, talbe_gain;
    //result_file_xls = '.\bt_table_auto_creat.xls';
    //t_tmp = {'%PGA_gain','%TIA_gain','%LNA_gain','%DCOC_tia','%DCOC_PGA','NFmin(dB)','Freqmin(MHz)','Gainmin','NFmax','Freqmax',...
     //   'Gainmax','SinTonex1','SinToney1','SinTonex2','SinToney2','SinTonex3','SinToney3','SinTonex4','SinToney4'};

    //xlswrite(result_file_xls, t_tmp, 'bt_table', 'A1');
    //ss = 0;


    table_en_initial();
	#if 0
//    maxscend_wifi_rf_lo_cfg(2437);

    #else
//	maxscend_wifi_rf_lo_cfg_test(6);
    #endif
    tempAddr = 0x029C;
    for(gain_table_id = 0;gain_table_id < sizeof(gain_table_BT); gain_table_id++)
    {
        tmp = tmp+1;
        castor_set_bt_rx_host();
        gain_table_value = gain_table_BT[gain_table_id];
        //pga_gain_value = gain_table_value(6:8);
        //tia_gain_value = gain_table_value(4:5);
        //lna_gain_value = gain_table_value(1:3);
        
        //%%%%%%%%%%%%%%%%% set to host mode and get the DCOC value %%%%%%%
    	tempValue = 0;
    	tempValue += ((gain_table_value>>5)&0x01);
    	tempValue <<=1;
    	tempValue += ((gain_table_value>>6)&0x01);
    	tempValue <<=1;
    	tempValue += ((gain_table_value>>7)&0x01);
    	tempValue <<=1;

    	tempValue += ((gain_table_value>>3)&0x01);
    	tempValue <<=1;
    	tempValue += ((gain_table_value>>4)&0x01);
    	tempValue <<=1;

    	tempValue += ((gain_table_value)&0x01);
    	tempValue <<=1;
    	tempValue += ((gain_table_value>>1)&0x01);
    	tempValue <<=1;
    	tempValue += ((gain_table_value>>2)&0x01);
    	//tempValue <<=1;
    	printk("tempValue: 0x%x\r\n", tempValue);

        gain_value = tempValue;  //gain_table_value;  //bin2dec([ '00000000' gain_table_value(6:8) gain_table_value(4:5) gain_table_value(1:3)]);

        //rbus_write('e2',dec2hex(gain_value),'6A'); // set gain of host mode
    	rf_reg_write(0x00e2,gain_value); 
        //pause(1.0);
    	udelay(100);

        for(ii = 8; ii <= 11; ii++ ) //wifi rx ,2,3; tia; 4,5
    	{
            castor_dcoc(ii);  // DCOC cal
    	}


    //bt_tia_dcoc_tmp = double(rbus_read('6A','012'));
	rf_reg_read(0x0012, &tempread);
	bt_tia_dcoc_tmp = tempread;
    //bt_pga_dcoc_tmp = double(rbus_read('6A','011'));
	rf_reg_read(0x0011, &tempread);
	bt_pga_dcoc_tmp = tempread;


    //bt_tia_dcoc_l = bitand(bitshift(bt_tia_dcoc_tmp,-8),255);
	bt_tia_dcoc_l = (bt_tia_dcoc_tmp<<8)&0xff;
    //bt_tia_dcoc_h = bitand(bitshift(bt_tia_dcoc_tmp,8),65280);
	bt_tia_dcoc_h = (bt_tia_dcoc_tmp >>8)&0xff00;
    //tia_dcoc = dec2hex(bt_tia_dcoc_h + bt_tia_dcoc_l);
	tia_dcoc = tia_dcoc + bt_tia_dcoc_l;

    //bt_pga_dcoc_l = bitand(bitshift(bt_pga_dcoc_tmp,-8),255);
	bt_pga_dcoc_l = (bt_pga_dcoc_tmp<<8)&0xff;
    //bt_pga_dcoc_h = bitand(bitshift(bt_pga_dcoc_tmp,8),65280);
	bt_pga_dcoc_h = (bt_pga_dcoc_tmp >>8)&0xff00;
    //pga_dcoc = dec2hex(bt_pga_dcoc_h + bt_pga_dcoc_l);
	pga_dcoc = bt_pga_dcoc_h + bt_pga_dcoc_l;

    //%%%%%%%%%%%%%%%%%%% switch to mxd mode and set the table %%%%%%%%%%%%%%%%

    // set the mode config

    //rbus_write('131','00c','6A'); // switch mxd mode
	rf_reg_write(0x0131,0x000c); 
    //rbus_write('133','400','6A'); // switch table mode
	rf_reg_write(0x0133,0x0400); 

    //talbe_gain = bin2dec([ '00000010' gain_table_value(6:8) gain_table_value(4:5) gain_table_value(1:3)]);
	tempValue = 0;
	tempValue += ((gain_table_value>>5)&0x01);
	tempValue <<=1;
	tempValue += ((gain_table_value>>6)&0x01);
	tempValue <<=1;
	tempValue += ((gain_table_value>>7)&0x01);
	tempValue <<=1;

	tempValue += ((gain_table_value>>3)&0x01);
	tempValue <<=1;
	tempValue += ((gain_table_value>>4)&0x01);
	tempValue <<=1;

	tempValue += ((gain_table_value)&0x01);
	tempValue <<=1;
	tempValue += ((gain_table_value>>1)&0x01);
	tempValue <<=1;
	tempValue += ((gain_table_value>>2)&0x01);
	//tempValue <<=1;
	talbe_gain = tempValue | 0x0200;

    
    // set the table's value
    //rbus_write(gain_table{ gain_table_id*2},'3434','6A'); // config mode
	rf_reg_write(tempAddr,0x3434); 
    //rbus_write(dec2hex(hex2dec(gain_table{ gain_table_id*2})+1),tia_dcoc,'6A'); // config tia DCOC table
	rf_reg_write(tempAddr+ 1,tia_dcoc); 
    //rbus_write(dec2hex(hex2dec(gain_table{ gain_table_id*2})+2),pga_dcoc,'6A'); // config pga DCOC table
	rf_reg_write(tempAddr+ 2,pga_dcoc); 
    //rbus_write(dec2hex(hex2dec(gain_table{ gain_table_id*2})+3),dec2hex(talbe_gain),'6A'); // set gain table of mxd mode
	rf_reg_write(tempAddr+ 3,talbe_gain); 

    // set gain table
    //rbus_write('135',dec2hex (bitshift( (length(gain_table)/2 - gain_table_id),5)),'6A'); // set the table idx
	rf_reg_write(0x0135,(sizeof(gain_table_BT) - gain_table_id)>>5); 
	tempAddr -= 0x0004;
    udelay(50); //pause(0.5);
/*
    SinTonex1 = 10;
    SinToney1 = 20;
    SinTonex2 = 30;
    SinToney2 = 40;
    SinTonex3 = 50;
    SinToney3 = 60;
    SinTonex4 = 70;
    SinToney4 = 80;
    NFmin = 0;
    NFmax = 0;
    Freqmax = 0;
    Gainmax = 0;
    Freqmin = 0;
    Gainmin = 0;
*/
    // [NFmin, Freqmin, Gainmin, NFmax, Freqmax, Gainmax] = noise_figure_test( PSA,'1.25','1.5','2437');
    // [SinTonex1, SinToney1, SinTonex2, SinToney2, SinTonex3, SinToney3, SinTonex4, SinToney4] = IIP3_TEST(PSA);

    //t_tmp = {pga_gain_value,tia_gain_value,lna_gain_value,tia_dcoc,pga_dcoc,NFmin,Freqmin,Gainmin,NFmax,Freqmax,Gainmax,...
    //    SinTonex1,SinToney1,SinTonex2,SinToney2,SinTonex3,SinToney3,SinTonex4,SinToney4};

    //printk('%s\t %s\t %s\t %s\t %s\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n',...
    //    pga_gain_value,tia_gain_value,lna_gain_value,tia_dcoc,pga_dcoc,NFmin,Freqmin,Gainmin,NFmax,Freqmax,Gainmax,...
    //    SinTonex1,SinToney1,SinTonex2,SinToney2,SinTonex3,SinToney3,SinTonex4,SinToney4)
    //ss = ss + 1;
    //xlswrite(result_file_xls, t_tmp, 'bt_table', ['A' num2str(ss + 1)]);
}
}

INLINE void  castor_set_wifi_rx_host(void)  // fd auto calibration
{
	rf_reg_write(0x0050,0x0001);  // fd auto calibration
	#ifndef USE_SOC//__USE_SOC__
	rf_reg_write(0x0109,0x0800);    // fd tunig enable 
	#else
	rf_reg_write(0x0107,0x0010);    // fd tunig enable 
	#endif
	// iptest soc trx config
	rf_reg_write(0x0000,0xffff);   // rest all statemachine
	// init config
	rf_reg_write(0x00d2,0xef3f);   // enable lcpll,ringpll,bt cp,lo,Tx dab,wifi cp,lo
	rf_reg_write(0x00d4,0x0000);   // all ldos set 1.3V
	rf_reg_write(0x0131,0x0003);    // set soc to wifi rx mode
	rf_reg_write(0x00e2,0x181b);    // set soc to wifi rx mode
	// rbus_write('00e4','001a','6A');  // set tia res 300ohm
	rf_reg_write(0x00e4,0x021a);  // set tia res 300ohm
	rf_reg_write(0x00d5,0x002a);   // enable lo_nolap,wifi lobuf,swbias
	rf_reg_write(0x00d7,0x7038); // enable rx bias, filter,pga
	rf_reg_write(0x00d8,0x0a04);   // enable tx filter,pga,txbias,lobuf,modulator,pa,pdet
	rf_reg_write(0x00d9,0x0030);
	rf_reg_write(0x0016,0x7f7d);  // wifi pga dcoc
	rf_reg_write(0x0017,0x6781);  // wifi tia dcoc
	rf_reg_write(0x0133,0x0000); // switch mxd mode
	rf_reg_write(0x00db,0x00c1);  // lna bias/cas voltage
	rf_reg_write(0x00dc,0xa4a4);  // lna bias

#ifdef USE_SOC //__USE_SOC__
	rf_reg_write(0x0000,0xfff7);  // lna bias
	rf_reg_write(0x0000,0xffff);  // lna bias
#endif
}


/*
func:
cfg enable tbl,
configure lo clk,
write the AGC gain TBL.
set ctrl mode and work mode.
*/

INLINE void maxscend_wifi_rf_force_rx_cfg(void)
{
		//iptest soc trx config
	// fd config
	printk("castor_wifi_rx_cfg!\r\n");
	rf_reg_write(0x0050, 0x0001);  // fd auto calibration
	rf_reg_write(0x0109, 0x8000);  // fd tunig enable
	// init config
	rf_reg_write(0x0000, 0xffff);  
	rf_reg_write(0x00d2, 0xef3f ); // enable all ldos
	rf_reg_write(0x00d4, 0x0000);  // all ldos set 1.3V
	rf_reg_write(0x0131, 0x0003);  // set soc to wifi rx mode

	rf_reg_write(0x00d5, 0x002a);  // enable lo_nolap,wifi lobuf,swbias
	rf_reg_write(0x00e4, 0x021a);
	rf_reg_write(0x00d7 ,0x7038 ); // enable rx bias, filter,pga
	rf_reg_write(0x00d8 ,0x0a04);  // enable Rx enable, tia 
	rf_reg_write(0x00d9 ,0x0030 ); // enable sw
	
	rf_reg_write(0x00e2 ,0x181b);  // pga 12dB,tia 14dB,lna 17dB
	rf_reg_write(0x00e2 ,0x220b);  // pga 12dB,tia 14dB,lna 17dB
	
	rf_reg_write(0x00db, 0xf0c1 ); 
	rf_reg_write(0x00dc, 0xa4a4 ); 

	//rf_reg_write(0x0008 ,0x0200 );
///	rf_reg_write(0x0000 ,0x7f7);
	//rf_reg_write(0x0000 ,0x7ff);
	printk("RF TX config Done !!\r\n");
}

/*
wifi tx cfg
*/
INLINE void maxscend_wifi_rf_force_tx_cfg(void)
{

	//maxscend_wifi_rf_lo_cfg(2437); //ADD BY MR 20130112
	//printk(" @@@@  new  @@@@ \n");
	//printk(" @@@@  new  @@@@ \r\n");
	//fd config
	printk("wifi_tx_cfg!\r\n");
	rf_reg_write(0x0050 ,0x0001);  // fd auto calibration
	rf_reg_write(0x0107 ,0x0010);  // fd tunig enable
	// iptest soc trx config
	rf_reg_write(0x0000 ,0x7ff ); // rest all statemachine
	rf_reg_write(0x0009 ,0x0200 ); 
	// init config
	rf_reg_write(0x00d2 ,0xfeff ); // enable lcpll,ringpll,bt cp,lo,Tx dab,wifi cp,lo
	rf_reg_write(0x00d4 ,0x0000 ); // all ldos set 1.3V
	rf_reg_write(0x0131 ,0x0001 );// set soc to wifi tx mode
	rf_reg_write(0x00d5 ,0x002a ); // enable lo_nolap,wifi lobuf,swbias
	rf_reg_write(0x00d7 ,0x2700 );
	rf_reg_write(0x00d8 ,0x6899 ); // enable tx filter,pga,txbias,lobuf,modulator,pa,pdet 
	rf_reg_write(0x0009 ,0x0200 ); 

	rf_reg_write(0x003d, 0x0001 ); 

	rf_reg_write(0x0040, 0x0010 ); 
	rf_reg_write(0x0045, 0x0250 ); 
	
	rf_reg_write(0x0000 ,0x7f7 );
	rf_reg_write(0x0000 ,0x7ff ); //reset
	printk("RF RX config Done !!\r\n");

}

INLINE void wifi_mxd_rf_tx_power_reg(void)
{
	rf_reg_write(0x18, 0x8080); 		//tx loft
	rf_reg_write(0xe3, 0x785);		//tx power
}

INLINE void wifi_mxd_rf_ctrl_mode(UWORD32 mode)
{
	//mode 1: RF WiFi pin control mode
	if(mode == 1)
	{
	//switch to WiFi Control Mode
		rf_reg_write(0x131, 0x1b);		//switch to WiFi control mode
		rf_reg_write(0x133, 0xffc); 	//tx gain host mode 
		wifi_mxd_rf_tx_power_reg();
		rf_reg_write(0xdb, 0xffff);
		rf_reg_write(0xde, 0xffff);
		rf_reg_write(0xdf, 0xffff);
		rf_reg_write(0xe0, 0xffff);
	}
	
}

INLINE void trout2_mxd_rf_channel_config(UWORD8 rf_channel)
{
	#if 0
    UWORD32 flo=0;

    if((rf_channel >= 1) && (rf_channel < 14))
    {
       flo = (2412 + (rf_channel-1)*5);
    }
    else if(rf_channel == 14)
    {
        flo = 2484;
    }
    else
    {
        flo = 2437;
    }
	
    maxscend_wifi_rf_lo_cfg(flo); 
	#else
	maxscend_wifi_rf_lo_cfg_test(rf_channel);
	#endif
}

INLINE void wifi_mxd_rf_dc_caribration(void)
{
	UWORD16 tmp = 0;
	int ii;
	UWORD16 gain_table_value, gain_value,gain_table_id, talbe_gain;
	UWORD32 tempread;
	UWORD16 tempAddr,tempValue, wifi_pga_dcoc_l, wifi_pga_dcoc_h;
	UWORD16 wifi_tia_dcoc_tmp,wifi_pga_dcoc_tmp,wifi_tia_dcoc_l,wifi_tia_dcoc_h,tia_dcoc,pga_dcoc;

	//UWORD16 tempAddr1;//dumy add 2013-01-29 for BT
	//tempAddr1 = 0x067c;//dumy add 2013-01-29 for BT


	tempAddr = 0x03aC; //0x039C

        castor_set_wifi_rx_host();
        trout2_mxd_rf_channel_config(6);

	for(gain_table_id = 0;gain_table_id < (sizeof(gain_table_WIFI)/2); gain_table_id ++)
	{
	    //printk("######gain_table_id:%d\r\n", gain_table_id);
	    tmp = tmp+1;
	    //castor_set_wifi_rx_host();
	    rf_reg_write(0x0131,0x0003);
	    rf_reg_write(0x0133,0x0000);
	    gain_table_value = gain_table_WIFI[gain_table_id];
	    //pga_gain_value = gain_table_value(6:9);
	    //tia_gain_value = gain_table_value(4:5);
	    //lna_gain_value = gain_table_value(1:3);

	    //%%%%%%%%%%%%%%%%% set to host mode and get the DCOC value %%%%%%%

	    //gain_value = bin2dec(['00' gain_table_value(6:9) gain_table_value(4:5) '00000' gain_table_value(1:3)]);
		tempValue = 0;
		tempValue += (((gain_table_value<<(5+7))&0x8000)>>15);
		tempValue <<=1;
		tempValue += (((gain_table_value<<(6+7))&0x8000)>>15);
		tempValue <<=1;
		tempValue += (((gain_table_value<<(7+7))&0x8000)>>15);
		tempValue <<=1;
		tempValue += (((gain_table_value<<(8+7))&0x8000)>>15);
		tempValue <<=1;
		tempValue += (((gain_table_value<<(3+7))&0x8000)>>15);
		tempValue <<=1;
		tempValue += (((gain_table_value<<(4+7))&0x8000)>>15);
		
		tempValue <<=5;
		tempValue <<=1;
		tempValue += (((gain_table_value<<7)&0x8000)>>15);
		tempValue <<=1;
		tempValue += (((gain_table_value<<(1+7))&0x8000)>>15);
		tempValue <<=1;
		tempValue += (((gain_table_value<<(2+7))&0x8000)>>15);
		gain_value = tempValue;
		//printk("wifi_table_auto_write:gain_value:0x%x\r\n", gain_value);




	    	//rbus_write('e2',dec2hex(gain_value),'6A'); % set gain of host mode
		rf_reg_write(0x00e2,gain_value);
	    	//pause(1.0);
		udelay(5);

	        for(ii = 2; ii<= 5 ; ii++)// wifi rx ,2,3: tia; 4,5:
		{
	        	castor_dcoc(ii);  // DCOC cal
		}
		
	    	//wifi_tia_dcoc_tmp = double(rbus_read('6A','017'));
		rf_reg_read(0x0017, &tempread);
		wifi_tia_dcoc_tmp = tempread;
	    	//wifi_pga_dcoc_tmp = double(rbus_read('6A','016'));
		rf_reg_read(0x0016, &tempread);
		wifi_pga_dcoc_tmp = tempread;
	       //printk("wifi_tia_dcoc_tmp:0x%x,wifi_pga_dcoc_tmp: 0x%x\r\n", wifi_tia_dcoc_tmp, wifi_pga_dcoc_tmp);
	    	//wifi_tia_dcoc_l = bitand(bitshift(wifi_tia_dcoc_tmp,-8),255);
		wifi_tia_dcoc_l = (wifi_tia_dcoc_tmp>>8)&0x00ff;
	    	//wifi_tia_dcoc_h = bitand(bitshift(wifi_tia_dcoc_tmp,8),65280);
		wifi_tia_dcoc_h = (wifi_tia_dcoc_tmp<<8)&0xff00;
	    	//tia_dcoc = dec2hex(wifi_tia_dcoc_h + wifi_tia_dcoc_l);
		tia_dcoc = wifi_tia_dcoc_l + wifi_tia_dcoc_h;

	    	//wifi_pga_dcoc_l = bitand(bitshift(wifi_pga_dcoc_tmp,-8),255);
		wifi_pga_dcoc_l = (wifi_pga_dcoc_tmp>>8)&0x00ff;
	    	//wifi_pga_dcoc_h = bitand(bitshift(wifi_pga_dcoc_tmp,8),65280);
		wifi_pga_dcoc_h = (wifi_pga_dcoc_tmp<<8)&0xff00;
	    	//pga_dcoc = dec2hex(wifi_pga_dcoc_h + wifi_pga_dcoc_l);
		pga_dcoc = wifi_pga_dcoc_l + wifi_pga_dcoc_h;

	    	//%%%%%%%%%%%%%%%%%%% switch to mxd mode and set the table %%%%%%%%%%%%%%%%

	    	//%     set the mode config
	    	//rbus_write('131','00b','6A'); //% switch mxd mode
		//rf_reg_write(0x0131,0x000b);
	    	//rbus_write('133','010','6A'); //% switch wifi table mode
		//rf_reg_write(0x0133,0x0010);

	    	//talbe_gain = bin2dec([ '0000001' gain_table_value(6:9) gain_table_value(4:5) gain_table_value(1:3)]);
		tempValue = 1;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(5+7))&0x8000)>>15;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(6+7))&0x8000)>>15;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(7+7))&0x8000)>>15;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(8+7))&0x8000)>>15;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(3+7))&0x8000)>>15;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(4+7))&0x8000)>>15;
		tempValue <<=1;

		tempValue += ((gain_table_value<<7)&0x8000)>>15;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(1+7))&0x8000)>>15;
		tempValue <<=1;
		tempValue += ((gain_table_value<<(2+7))&0x8000)>>15;
		talbe_gain = tempValue;

	        //printk("wifi_table_auto_write:talbe_gain:0x%x\r\n", talbe_gain);
		// rf_reg_write(0x0131,0x001b); 
		// rf_reg_write(0x0133,0xffff); 


	    // set the table's value
	    //rbus_write(gain_table{ gain_table_id*2},'a4a4','6A'); % config mode
		rf_reg_write(tempAddr,0xa4a4);
	    //rbus_write(dec2hex(hex2dec(gain_table{ gain_table_id*2})+1),tia_dcoc,'6A'); % config tia DCOC table
		rf_reg_write(tempAddr + 1,tia_dcoc);
	    //rbus_write(dec2hex(hex2dec(gain_table{ gain_table_id*2})+2),pga_dcoc,'6A'); % config pga DCOC table
		rf_reg_write(tempAddr + 2,pga_dcoc);
	    //rbus_write(dec2hex(hex2dec(gain_table{ gain_table_id*2})+3),dec2hex(talbe_gain),'6A'); % set gain table of mxd mode
		rf_reg_write(tempAddr + 3,talbe_gain);

		//rf_reg_write(tempAddr1,0xa4a4);//dumy add 2013-01-29 for BT
		//rf_reg_write(tempAddr1 + 1,tia_dcoc);//dumy add 2013-01-29 for BT
		//rf_reg_write(tempAddr1 + 2,pga_dcoc);//dumy add 2013-01-29 for BT
		//rf_reg_write(tempAddr1 + 3,talbe_gain);//dumy add 2013-01-29 for BT


	    // set gain table
	    //rbus_write('138',dec2hex((length(gain_table)/2 - gain_table_id)),'6A'); % set the table idx
		//rf_reg_write(0x0138,(sizeof(gain_table_WIFI) - gain_table_id));
		tempAddr -= 0x0004;
	    //pause(0.5);
		//udelay(5);

	    //SinTonex1 = 10;
	    //SinToney1 = 20;
	    //SinTonex2 = 30;
	    //SinToney2 = 40;
	    //SinTonex3 = 50;
	    //SinToney3 = 60;
	    //SinTonex4 = 70;
	    //SinToney4 = 80;
	    //NFmin = 0;
	    //NFmax = 0;
	    //Freqmax = 0;
	    //Gainmax = 0;
	    //Freqmin = 0;
	    //Gainmin = 0;

	    //             [NFmin, Freqmin, Gainmin, NFmax, Freqmax, Gainmax] = noise_figure_test( PSA,'1.25','1.5','2437');
	    //             [SinTonex1, SinToney1, SinTonex2, SinToney2, SinTonex3, SinToney3, SinTonex4, SinToney4] = IIP3_TEST(PSA);

	    //t_tmp = {pga_gain_value,tia_gain_value,lna_gain_value,tia_dcoc,pga_dcoc,NFmin,Freqmin,Gainmin,NFmax,Freqmax,Gainmax,...
	    //    SinTonex1,SinToney1,SinTonex2,SinToney2,SinTonex3,SinToney3,SinTonex4,SinToney4};

	    //printk('%s\t %s\t %s\t %s\t %s\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\t %.2f\n',...
	    //    pga_gain_value,tia_gain_value,lna_gain_value,tia_dcoc,pga_dcoc,NFmin,Freqmin,Gainmin,NFmax,Freqmax,Gainmax,...
	    //    SinTonex1,SinToney1,SinTonex2,SinToney2,SinTonex3,SinToney3,SinTonex4,SinToney4)
	    //ss = ss + 1;
	    //xlswrite(result_file_xls, t_tmp, 'wifi_table', ['A' num2str(ss + 1)]);
	}
	printk("DC calibration done !!\r\n");
}

INLINE void mxd_wifi_rf_initial(void)
{
	printk("MXD RF initial start !!\n\n");
	table_en_initial();
	wifi_mxd_rf_dc_caribration();
	wifi_mxd_rf_ctrl_mode(1);
	printk("MXD RF initial done !!\r\n");

	//rf_reg_write(0x131, 0x1d);//dumy add 2013-01-29 for BT
	//rf_reg_write(0x133, 0xf3c);//dumy add 2013-01-29 for BT
}

#if 0
INLINE UWORD32 trout_ic_rf_addr_spi2adi(UWORD32 addr)
{
    UWORD32 val;
    val = ((addr>>8)&0x07);
    
    switch(val)
    {
        case 4:
            return((addr&0x7F)|(2<<7));
        case 5:
            return((addr&0x7F)|(3<<7));
        case 6:
            return((addr&0x7F)|(4<<7));
        default:
            return addr;
    } 
}

INLINE void trout_ic_rf_reg_adi_write(UWORD32 addr, UWORD32 data)
{
    //UWORD32 reg_data=0;;
    UWORD32 adi_addr;
	UWORD32 val;
    
    adi_addr = 0x1000 + trout_ic_rf_addr_spi2adi(addr);

    host_write_trout_reg(data, (adi_addr<<2));
#if 0 
    do {
       val = host_read_trout_reg(rCOMM_INT_STAT);
       
    }while((val & BIT23) == 0);
    
    /* Clear ADI Write INT Stat (BIT23) */
    host_write_trout_reg(BIT23, rCOMM_INT_CLEAR);
#else
    udelay(20);
#endif

}
#endif

INLINE void trout2_fpga_rf55_adi_init(void)
{
      UWORD32 val;
    
    /* Set the RF ADI Mode, Select, Clock etc */
    //val = 0x5;

	//val = 0x93;//chenq mod 2013-01-25 for BT context
	//val = 0x95;//chenq mod 2013-01-26 for BT context
	//val = 0x15;//chenq mod 2013-01-26 for BT context
	//val = 0x3;//chenq mod 2013-01-26 for BT context
	val = 0x13;//ranqing.wu mod 2013-01-28 for BT context
    
    host_write_trout_reg(val, rCOMM_RF_SPI_CTRL);

	//host_write_trout_reg(0x1, (((UWORD32)0x55)<<2));//trout2 FPGA RF55 Wi-Fi IQ switch
	host_write_trout_reg(0x5, (((UWORD32)0x55)<<2));//trout2 FPGA RF55 Wi-Fi IQ switch

   // trout_ic_rf_reg_adi_write(0x107B, 0);
    printk("trout adi init ok...\n");
}
#endif //TROUT2_WIFI_FPGA_RF55

#endif //TROUT2_WIFI_FPGA

#ifdef TROUT2_WIFI_IC

// ADC DAC config:
INLINE void Trout2_ADC_DAC_Init(void)
{
    host_write_trout_reg(0x2000000, ((UWORD32)0x62)<<2);  // WIFI/BT DAC
    host_write_trout_reg(0x34, ((UWORD32)0x63)<<2);  // WIFI ADC
    host_write_trout_reg(0x6, ((UWORD32)0x50)<<2);  //  DAC power 
}
#endif //TROUT2_WIFI_IC

/* This function writes the required value to the specified AD9863 register */
/* address.                                                                  */
INLINE void write_mxfe_reg(UWORD8 address, UWORD8 data)
{
#if 1  //liusysoc
    UWORD32 spi_data;

    /* Shift the required SPI data bits to MSB */
    spi_data = ((address << 8) + data) << (32 - SPI_DATA_WIDTH_AD9863);

    write_dot11_phy_reg(rSPIDATA, spi_data);

    /* Set the SPI enable to program */
    spi_data = (SPI_DATA_WIDTH_AD9863 << 2) | (BIT1 + BIT0);
    write_dot11_phy_reg(rSPICTRL, spi_data);

//#if 1  // 20120730, dumy mod
#ifndef TROUT2_WIFI_IC  //mod by zhangym 0403
    /* Wait for the Poll the SPI write done */
    do {
        read_dot11_phy_reg(rSPICTRL, &spi_data);
    }while(spi_data & BIT0);
#else
    udelay(20);
#endif
    
#endif
}

/* This function reads the current value of the specified AD9863 register */
/* of the mentioned channel (0-3).                                        */
INLINE void read_mxfe_reg(UWORD8 chan, UWORD8 address, UWORD8 *data)
{
    /* Not supported */
}

/* dumy add for RF 20120618 */
INLINE void write_RF_reg_trout(UWORD8 address, UWORD32 data)
{
    UWORD32 spi_data;
    UWORD32 val;
    
    //printk(KERN_WARNING "write_RF_reg_trout==> ");
    
    /* Set the RF SPI Mode, Select, Clock etc */
    val = 0x14E; //0x145; //dumy fix 0816
    
    host_write_trout_reg(val, rCOMM_RF_SPI_CTRL);
        
    /* Shift the required SPI data bits to MSB */
    spi_data = ((data << 4) + address) << (32 - RF_SPI_DATA_WIDTH);

    host_write_trout_reg(spi_data, rCOMM_RF_SPI_DATA);

    //printk(KERN_WARNING "RF=1\n");
#if 0  // 20120730, dumy mod
    /* Wait for the Poll the SPI write done */
    do {
       val = host_read_trout_reg(rCOMM_INT_STAT);
       
    }while((val & BIT23) == 0);
    
    /* Clear ADI Write INT Stat (BIT23) */
    host_write_trout_reg(BIT23, rCOMM_INT_CLEAR);
#else
    udelay(20);
#endif
    //printk(KERN_WARNING "RF=3\n");

}

INLINE void write_RF_reg_ittiam(UWORD8 address, UWORD32 data)
{
   // printk(KERN_WARNING "RF=Null \n");
#if 0   
#if 0//dumy add for RF 20120618
    UWORD32 spi_data;

    /* Shift the required SPI data bits to MSB */
    spi_data = ((data << 4) + address) << (32 - RF_SPI_DATA_WIDTH);

    write_dot11_phy_reg(rSPIDATA, spi_data);

    /* Set the SPI enable to program */
    spi_data = (RF_SPI_DATA_WIDTH << 2) | BIT0;
    write_dot11_phy_reg(rSPICTRL, spi_data);

    /* Wait for the Poll the SPI write done */
    do {
        read_dot11_phy_reg(rSPICTRL, &spi_data);
    }while(spi_data & BIT0);
#else
    write_RF_reg_trout(address, data);
#endif
#endif

//chenq add 2013-02-01 for trout2 fpga 2829
#if  defined(TROUT2_WIFI_FPGA) && defined(TROUT2_WIFI_FPGA_RF2829)
    UWORD32 spi_data;

    /* Shift the required SPI data bits to MSB */
    spi_data = ((data << 4) + address) << (32 - RF_SPI_DATA_WIDTH);

    write_dot11_phy_reg(rSPIDATA, spi_data);

    /* Set the SPI enable to program */
    spi_data = (RF_SPI_DATA_WIDTH << 2) | BIT0;
    write_dot11_phy_reg(rSPICTRL, spi_data);

    /* Wait for the Poll the SPI write done */
    do {
        read_dot11_phy_reg(rSPICTRL, &spi_data);
    }while(spi_data & BIT0);

#else
    write_RF_reg_trout(address, data);
#endif

}

/* This function clears the Radar Detect bit in PHY */
INLINE void clear_phy_radar_status_bit_ittiam(void)
{
    UWORD32 regval = 0;

    read_dot11_phy_reg(rPHYRADARCNTRLREG6, &regval);
    regval &= ~BIT7;
    write_dot11_phy_reg(rPHYRADARCNTRLREG6, regval);
}

/* This function checks whether the PHY address is valid */
INLINE BOOL_T is_valid_phy_reg_ittiam(UWORD32 regaddr)
{
    BOOL_T retval = BFALSE;

    if(regaddr == 0xFF)
        retval = BTRUE;
    else
    {
        UWORD32 regval = 0;

        /* Read the register bank currently selected */
        read_dot11_phy_reg(0xFF, &regval);

        if((regval == 0x00) && (regaddr <= MAX_PHY_ADDR))
            retval = BTRUE;
        else if((regval == 0x01) && (regaddr <= MAX_PHY_ADDR2))
            retval = BTRUE;
    }

    return retval;
}

/* This function updates any required PHY registers based on channel set */
INLINE void update_phy_ittiam_on_select_channel(UWORD8 freq, UWORD8 ch_idx)
{
    UWORD16 val = get_lock_conv_fact(freq, ch_idx);

    /* Set the fs/fc LSB based on the channel */
    write_dot11_phy_reg(rPHYALOCKCONVFACTLSB, (val & 0xFF));
    write_dot11_phy_reg(rPHYALOCKCONVFACTMSB, ((val & 0xFF00) >> 8));
}

/* This function returns the current RX Sensitivity parameter from PHY */
INLINE UWORD16 get_phy_rx_sense_ittiam(void)
{
    UWORD16 ret_val = 0;

    /* ----------------------------------------------- */
    /* Rx sensitivity parameter format (WID view)      */
    /* ----------------------------------------------- */
    /* |   PHYAGCMAXVGAGAIN   | PHYAGCFINALVGAGAINTH | */
    /* ----------------------------------------------- */
    /* | 7                  0 | 15                 8 | */
    /* ----------------------------------------------- */
    ret_val =
          (((UWORD16)phy_reg_val_table[rPHYAGCFINALVGAGAINTH] << 8) & 0xFF00) |
          (((UWORD16)phy_reg_val_table[rPHYAGCMAXVGAGAIN]     << 0) & 0x00FF);

    return ret_val;
}

/* This function sets the RX Sensitivity parameter in PHY */
INLINE void set_phy_rx_sense_ittiam(UWORD16 val)
{
    /* ----------------------------------------------- */
    /* Rx sensitivity parameter format (WID view)      */
    /* ----------------------------------------------- */
    /* |   PHYAGCMAXVGAGAIN   | PHYAGCFINALVGAGAINTH | */
    /* ----------------------------------------------- */
    /* | 7                  0 | 15                 8 | */
    /* ----------------------------------------------- */
    phy_reg_val_table[rPHYAGCFINALVGAGAINTH] = (UWORD8)((val >> 8) & 0x00FF);
    phy_reg_val_table[rPHYAGCMAXVGAGAIN]     = (UWORD8)((val >> 0) & 0x00FF);
}

/* This function returns the current CCA Threshold parameter from PHY */
INLINE UWORD16 get_phy_cca_threshold_ittiam(void)
{
    UWORD16 ret_val = 0;

    ret_val = phy_reg_val_table[rPHYCCAMACTH1];

    return ret_val;
}

/* This function sets the CCA Threshold parameter in PHY */
INLINE void set_phy_cca_threshold_ittiam(UWORD16 input)
{
    phy_reg_val_table[rPHYCCAMACTH1] = (UWORD8)input;
    phy_reg_val_table[rPHYCCAMACTH2] = (UWORD8)input;
}

/* This function gets the data rate of the received frame from PHY Register */
INLINE UWORD8 get_phy_dr_ittiam(void)
{
    UWORD32 phy_dr = 0;

    read_dot11_phy_reg(0x66, &phy_dr);
    phy_dr &= 0xF;

    return (UWORD8)phy_dr;
}

/* This function updates a value in the PHY initialization table */
INLINE void update_ittiam_reg_val_table(UWORD8 reg_idx, UWORD32 val)
{
    UWORD32 preg = 0;

    if(reg_idx == 0xFF)
        return;

    read_dot11_phy_reg(0xFF, &preg);

    if(preg == 0x00)
    {
        phy_reg_val_table[reg_idx] = val;
    }
    else
    {
        phy_reg_val_table_2[reg_idx] = val;
    }
}

/* This function computes the RSSI value in db */
INLINE BOOL_T compute_rssi_db_ittiam(WORD8 rssi, BOOL_T is_ofdm, SWORD8 *rssi_db)
{
    *rssi_db = rssi;
    return BTRUE;
}

#endif /* ITTIAM_PHY */
#endif /* PHY_ITTIAM_H */
