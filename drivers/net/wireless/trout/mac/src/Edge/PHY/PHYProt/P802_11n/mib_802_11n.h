/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : mib_802_11n.h                                        */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related  to PHY MIB as per 802.11n Draft standard    */
/*                      Version 2.00.                                        */
/*                      Access functions to get and set the MIB values       */
/*                      are also provided.                                   */
/*                                                                           */
/*  List of Functions : Access functions for all MIB parameters              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef PHY_802_11n

#ifndef MIB_802_11N_H
#define MIB_802_11N_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib.h"
#include "mib_11n.h"
#include "phy_hw_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define NUM_ANTENNASLISTTABLE             1
#define NUM_TX_POWER_LEVELS               8
#define MAX_NUM_MCS_SUPPORTED             16

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Antenna list sturucture represents the list of antennae. Anantenna can be */
/* marked to be capable of transmitting, receiving, and/or for participation */
/* in receive diversity. It is instantiated as an array in MIB structure.    */
typedef struct
{
    TRUTH_VALUE_T dot11SupportedTxAntenna;
    TRUTH_VALUE_T dot11SupportedRxAntenna;
    TRUTH_VALUE_T dot11DiversitySelectionRx;
} dot11AntennasListEntry_t;

/* PHY MIB definition */
typedef struct
{
    /*************************************************************************/
    /* Phy Operation Table - Dot11PhyOperationEntry                          */
    /*************************************************************************/
    /* PHY level attributes concerned with operation. Implemented as a table */
    /* indexed on ifIndex to allow for multiple instantiations on an Agent.  */
    /* Currently, table is not provided as a single instantiation exists.    */

    UWORD8  dot11PHYType;
    UWORD32 dot11CurrentRegDomain;
    UWORD8  dot11TempType;

    /*************************************************************************/
    /* Phy Antenna Table - Dot11PhyAntennaEntry                              */
    /*************************************************************************/
    /* Group of attributes for PhyAntenna. Implemented as a table indexed on */
    /* ifIndex to allow for multiple instances on an agent.                  */
    /* Currently, table is not provided as a single instantiation exists.    */

    UWORD8        dot11CurrentTxAntenna;
    UWORD8        dot11DiversitySupport;
    UWORD8        dot11CurrentRxAntenna;
    TRUTH_VALUE_T dot11AntennaSelectionOptionImplemented;
    TRUTH_VALUE_T dot11TransmitExplicitCSIFeedbackASOptionImplemented;
    TRUTH_VALUE_T dot11TransmitIndicesFeedbackASOptionImplemented;
    TRUTH_VALUE_T dot11ExplicitCSIFeedbackASOptionImplemented;
    TRUTH_VALUE_T dot11TransmitIndicesComputationFeedbackASOptionImplemented;
    TRUTH_VALUE_T dot11ReceiveAntennaSelectionOptionImplemented;
    TRUTH_VALUE_T dot11TransmitSoundingPPDUOptionImplemented;

    /*************************************************************************/
    /* Phy Tx Power Table - Dot11PhyTxPowerEntry                             */
    /*************************************************************************/
    /* Group of attributes for dot11PhyTxPowerTable. Implemented as a table  */
    /* indexed on STA ID to allow for multiple instances on an Agent.        */
    /* Currently, table is not provided as a single instantiation exists.    */

    UWORD8  dot11NumberSupportedPowerLevels;
    UWORD16 dot11TxPowerLevel[NUM_TX_POWER_LEVELS];
    UWORD8  dot11CurrentTxPowerLevel;

    /*************************************************************************/
    /* Phy FHSS Table - Dot11PhyFHSSEntry                                    */
    /*************************************************************************/
    /* Group of attributes for dot11PhyFHSSTable. Currently unused.          */

    /*************************************************************************/
    /* Phy DHSS Table - Dot11PhyDSSSEntry                                    */
    /*************************************************************************/
    /* Entry of attributes for dot11PhyDSSSEntry. Implemented as a table     */
    /* indexed on ifIndex allow for multiple instances on an Agent.          */
    /* Currently, table is not provided as a single instantiation exists.    */

    UWORD8  dot11CurrentChannel;
    UWORD8  dot11CCAModeSupported;
    UWORD8  dot11CurrentCCAMode;
    UWORD32 dot11EDThreshold;

    /*************************************************************************/
    /* Phy IR Table - Dot11PhyIREntry                                        */
    /*************************************************************************/
    /* Group of attributes for dot11PhyIRTable. Currently unused.            */

    /*************************************************************************/
    /* Reg Domains Supported Table - Dot11RegDomainsSupportEntry             */
    /*************************************************************************/
    /* There are different operational requirements dependent on  regulatory */
    /* domain. This attribute list describes the regulatory domains the PLCP */
    /* and PMD support in this implementation. Currently defined values and  */
    /* their corresponding Regulatory Domains are:                           */
    /* FCC (USA) = X'10', DOC (Canada) = X'20', ETSI (most of Europe) = X'30'*/
    /* Spain = X'31', France = X'32', MKK(Japan) = X'40'.                    */

    UWORD16 dot11RegDomainsSupportValue;

    /*************************************************************************/
    /* Antenna List Table - Dot11AntennasListEntry                           */
    /*************************************************************************/
    /* This table represents the list of antennae. Anantenna can be marked to*/
    /* be capable of transmitting, receiving, and/or for participation in    */
    /* receive diversity. Each entry in this table represents single antenna */
    /* with its properties. The maximum number of antennae that can be       */
    /* contained in this table is 255.                                       */

    dot11AntennasListEntry_t dot11AntennasListTable[NUM_ANTENNASLISTTABLE];

    /*************************************************************************/
    /* Supported Data rate Tx Table - Dot11SupportedDataRatesTxEntry         */
    /*************************************************************************/
    /* The Transmit bit rates supported by the PLCP and PMD, represented by  */
    /* a count from X’02-X’7f, corresponding to data rates in increments of  */
    /* 500Kb/s from 1 Mbit/s to 63.5 Mbit/s subject to limitations of each   */
    /* individual PHY.                                                       */
    /* 802.11b - Table Tx X’02’, X’04’, X’0B’, X’16’                         */
    /* 802.11a - 6, 9, 12, 18, 24, 36, 48, and 54 Mbit/s                     */
    /*           Mandatory rates: 6, 12, and 24                              */

    rate_t dot11SupportedDataRatesTxValue;

    /*************************************************************************/
    /* Supported Data rate Rx Table - Dot11SupportedDataRatesTxEntry         */
    /*************************************************************************/
    /* The receive bit rates supported by the PLCP and PMD, represented by a */
    /* count from X’002-X’7f, corresponding to data rates in increments of   */
    /* 500Kb/s from 1 Mbit/s to 63.5 Mbit/s.                                 */
    /* 802.11b - Table Rx X’02’, X’04’, X’0B’, X’16’                         */
    /* 802.11a - 6, 9, 12, 18, 24, 36, 48, and 54 Mbit/s                     */
    /*           Mandatory rates: 6, 12, and 24                              */

    rate_t dot11SupportedDataRatesRxValue;

    /*************************************************************************/
    /* OFDM Table (11a Attributes)                                           */
    /*************************************************************************/

    UWORD8        dot11CurrentFrequency;
    UWORD32       dot11TIThreshold;
    UWORD8        dot11FrequencyBandsSupported;
    UWORD32       dot11ChannelStartingFactor;
    TRUTH_VALUE_T dot11FiveMHzOperationImplemented;
    TRUTH_VALUE_T dot11TenMHzOperationImplemented;
    TRUTH_VALUE_T dot11TwentyMHzOperationImplemented;
    UWORD8        dot11PhyOFDMChannelWidth;

    /*************************************************************************/
    /* HR-DSS Table (11b Attributes)                                         */
    /*************************************************************************/

    TRUTH_VALUE_T dot11ShortPreambleOptionImplemented;
    TRUTH_VALUE_T dot11PBCCOptionImplemented;
    TRUTH_VALUE_T dot11ChannelAgilityPresent;
    TRUTH_VALUE_T dot11ChannelAgilityEnabled;
    UWORD8        dot11HRCCAModeSupported;

    /*************************************************************************/
    /* ERP Table (11g Attributes)                                            */
    /*************************************************************************/

    TRUTH_VALUE_T dot11ERPBCCOptionImplemented;
    TRUTH_VALUE_T dot11ERPBCCOptionEnabled;
    TRUTH_VALUE_T dot11DSSSOFDMOptionImplemented;
    TRUTH_VALUE_T dot11DSSSOFDMOptionEnabled;
    TRUTH_VALUE_T dot11ShortSlotTimeOptionImplemented;
    TRUTH_VALUE_T dot11ShortSlotOptionEnabled;
    UWORD8        dot11ERPCCAModeSupported;

    /*************************************************************************/
    /* HT Table (11n Attributes)                                             */
    /*************************************************************************/

    TRUTH_VALUE_T dot11FortyMHzOperationImplemented;
    TRUTH_VALUE_T dot11FortyMHzOperationEnabled;
    UWORD32       dot11CurrentPrimaryChannel;
    UWORD32       dot11CurrentSecondaryChannel;
    UWORD8        dot11NumberOfSpatialStreamsImplemented;
    UWORD8        dot11NumberOfSpatialStreamsEnabled;
    TRUTH_VALUE_T dot11GreenfieldOptionImplemented;
    TRUTH_VALUE_T dot11GreenfieldOptionEnabled;
    TRUTH_VALUE_T dot11ShortGIOptionInTwentyImplemented;
    TRUTH_VALUE_T dot11ShortGIOptionInTwentyEnabled;
    TRUTH_VALUE_T dot11ShortGIOptionInFortyImplemented;
    TRUTH_VALUE_T dot11ShortGIOptionInFortyEnabled;
    TRUTH_VALUE_T dot11LDPCCodingOptionImplemented;
    TRUTH_VALUE_T dot11LDPCCodingOptionEnabled;
    TRUTH_VALUE_T dot11TxSTBCOptionImplemented;
    TRUTH_VALUE_T dot11TxSTBCOptionEnabled;
    TRUTH_VALUE_T dot11RxSTBCOptionImplemented;
    TRUTH_VALUE_T dot11RxSTBCOptionEnabled;
    TRUTH_VALUE_T dot11BeamFormingOptionImplemented;
    TRUTH_VALUE_T dot11BeamFormingOptionEnabled;
    UWORD16       dot11HighestSupportedDataRate;
    TRUTH_VALUE_T dot11TxMCSSetDefined;
    TRUTH_VALUE_T dot11TxRxMCSSetNotEqual;
    UWORD8        dot11TxMaximumNumberSpatialStreamsSupported;
    TRUTH_VALUE_T dot11TxUnequalModulationSupported;

    /*************************************************************************/
    /* Supported MCS Tx Table                                                */
    /*************************************************************************/
    /*The Transmit MCS supported by the PLCP and PMD, represented by a count */
    /*from 1 to 127, subject to limitations of each individual PHY.          */

    ht_mcs_t dot11SupportedMCSTxValue;

    /*************************************************************************/
    /* Supported MCS Rx Table                                                */
    /*************************************************************************/
    /*The receive MCS supported by the PLCP and PMD, represented by a count  */
    /*from 1 to 127, subject to limitations of each individual PHY.          */

    ht_mcs_t dot11SupportedMCSRxValue;

    /*************************************************************************/
    /* TX Beamforming Table                                                */
    /*************************************************************************/

    TRUTH_VALUE_T dot11ReceiveStaggerSoundingOptionImplemented;
    TRUTH_VALUE_T dot11TransmitStaggerSoundingOptionImplemented;
    TRUTH_VALUE_T dot11ReceiveNDPOptionImplemented;
    TRUTH_VALUE_T dot11TransmitNDPOptionImplemented;
    TRUTH_VALUE_T dot11ImplicitTxBFOptionImplemented;
    UWORD8        dot11CalibrationOptionImplemented;
    TRUTH_VALUE_T dot11ExplicitCSITxBFOptionImplemented;
    TRUTH_VALUE_T dot11ExplicitNonCompressedbeamformingMatrixOptionImplemented;
    UWORD8        dot11ExplicitBFCSIFeedbackOptionImplemented;
    UWORD8        dot11ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented;
    UWORD8        dot11ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented;
    UWORD8        dot11NumberBeamFormingCSISupportAntenna;
    UWORD8        dot11NumberNonCompressedbeamformingMatrixSupportAntenna;
    UWORD8        dot11NumberCompressedbeamformingMatrixSupportAntenna;
    UWORD32       dot11MaxCSIFeedbackDelay;
} mibp_t;


/* Structure for Private PHY MIB */
typedef struct
{
    TRUTH_VALUE_T  dsss_cck_40mhz_mode;
} p_mibp_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern mibp_t MIBP;
extern p_mibp_t P_MIBP;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_phy_802_11n_mib(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/*                  Phy Operation Table Access Functions                     */
/*****************************************************************************/

/* This is an 8-bit integer value that identifies the PHY type supported by  */
/* attached PLCP and PMD. Currently defined values and their corresponding   */
/* PHY types are:                                                            */
/* FHSS 2.4 GHz = 01 , DSSS 2.4 GHz = 02, IR Baseband = 03,                  */
/* 802.11b - High Rate 2.4 GHz = 05                                          */
/* 802.11a - OFDM 5 GHz        = 04                                          */
/* 802.11g - ERP               = 06                                          */
/* 802.11n - HT                = 07                                          */
INLINE UWORD8 mget_PHYType(void)
{
    return(MIBP.dot11PHYType);
}

INLINE void mset_PHYType(UWORD8 inp)
{
    MIBP.dot11PHYType = inp;
}

/* The current regulatory domain this instance of the PMD is supporting. This*/
/* object corresponds to one of RegDomains listed in dot11RegDomainsSupported*/
INLINE UWORD32 mget_CurrentRegDomain(void)
{
    return(MIBP.dot11CurrentRegDomain);
}

INLINE void mset_CurrentRegDomain(UWORD32 inp)
{
    MIBP.dot11CurrentRegDomain = inp;
}

/* There are different operating temperature requirements dependent on the   */
/* anticipated environmental conditions. This attribute describes the current*/
/* PHY's operating temperature range capability. Currently defined values and*/
/* their corresponding temperature ranges are:                               */
/* Type 1 = X'01'-Commercial range of 0 to 40 degrees C,                     */
/* Type 2 = X'02'-Industrial range of -30 to 70 degrees C.                   */
INLINE UWORD8 mget_TempType(void)
{
    return(MIBP.dot11TempType);
}

INLINE void mset_TempType(UWORD8 inp)
{
    MIBP.dot11TempType = inp;
}

/*****************************************************************************/
/*                 Phy Antenna Table Access Functions                        */
/*****************************************************************************/

/* The current antenna being used to transmit. This value is one of values   */
/* appearing in dot11SupportedTxAntenna. This may be used by a management    */
/* agent to control which antenna is used for transmission.                  */
INLINE UWORD32 mget_CurrentTxAntenna(void)
{
    return(MIBP.dot11CurrentTxAntenna);
}

INLINE void mset_CurrentTxAntenna(UWORD32 inp)
{
    MIBP.dot11CurrentTxAntenna = inp;
}

/* This implementation's support for diversity, encoded as:                  */
/* X'01'-diversity is available and is performed over fixed list of antennas */
/*       defined in dot11DiversitySelectionRx.                               */
/* X'02'-diversity is not supported.                                         */
/* X'03'-diversity is supported and control of diversity is also available,  */
/*       in which case the attribute dot11DiversitySelectionRx can be        */
/*       dynamically modified by the LME.                                    */
INLINE UWORD8 mget_DiversitySupport(void)
{
    return(MIBP.dot11DiversitySupport);
}

INLINE void mset_DiversitySupport(UWORD8 inp)
{
    MIBP.dot11DiversitySupport = inp;
}

/* The current antenna being used to receive, if the dot11 DiversitySupport  */
/* indicates that diversity is not supported. The selected antenna shall be  */
/* one of the antennae marked for receive in the dot11AntennasListTable.     */
INLINE UWORD32 mget_CurrentRxAntenna(void)
{
    return(MIBP.dot11CurrentRxAntenna);
}

INLINE void mset_CurrentRxAntenna(UWORD32 inp)
{
    MIBP.dot11CurrentRxAntenna = inp;
}


INLINE TRUTH_VALUE_T mget_AntennaSelectionOptionImplemented(void)
{
    return(MIBP.dot11AntennaSelectionOptionImplemented);
}

INLINE void mset_AntennaSelectionOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11AntennaSelectionOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_TransmitExplicitCSIFeedbackASOptionImplemented(void)
{
    return(MIBP.dot11TransmitExplicitCSIFeedbackASOptionImplemented);
}

INLINE void mset_TransmitExplicitCSIFeedbackASOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11TransmitExplicitCSIFeedbackASOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_TransmitIndicesFeedbackASOptionImplemented(void)
{
    return(MIBP.dot11TransmitIndicesFeedbackASOptionImplemented);
}

INLINE void mset_TransmitIndicesFeedbackASOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11TransmitIndicesFeedbackASOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_ExplicitCSIFeedbackASOptionImplemented(void)
{
    return(MIBP.dot11ExplicitCSIFeedbackASOptionImplemented);
}

INLINE void mset_ExplicitCSIFeedbackASOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ExplicitCSIFeedbackASOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_TransmitIndicesComputationFeedbackASOptionImplemented(void)
{
    return(MIBP.dot11TransmitIndicesComputationFeedbackASOptionImplemented);
}

INLINE void mset_TransmitIndicesComputationFeedbackASOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11TransmitIndicesComputationFeedbackASOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_ReceiveAntennaSelectionOptionImplemented(void)
{
    return(MIBP.dot11ReceiveAntennaSelectionOptionImplemented);
}

INLINE void mset_ReceiveAntennaSelectionOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ReceiveAntennaSelectionOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T TransmitSoundingPPDUOptionImplemented(void)
{
    return(MIBP.dot11TransmitSoundingPPDUOptionImplemented);
}

INLINE void mset_TransmitSoundingPPDUOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11TransmitSoundingPPDUOptionImplemented = inp;
}

/*****************************************************************************/
/*                 Phy Tx Power Table Access Functions                       */
/*****************************************************************************/

/* The number of power levels supported by the PMD. This attribute can have  */
/* a value of 1 to 8.                                                        */
INLINE UWORD8 mget_NumberSupportedPowerLevels(void)
{
    return(MIBP.dot11NumberSupportedPowerLevels);
}

INLINE void mset_NumberSupportedPowerLevels(UWORD8 inp)
{
    MIBP.dot11NumberSupportedPowerLevels = inp;
}

/* The TxPowerLevel N currently being used to transmit data. Some PHYs also  */
/* use this value to determine the receiver sensitivity requirements for CCA.*/
INLINE UWORD8 mget_CurrentTxPowerLevel(void)
{
    return(MIBP.dot11CurrentTxPowerLevel);
}

INLINE void mset_CurrentTxPowerLevel(UWORD8 inp)
{
    MIBP.dot11CurrentTxPowerLevel = inp;
}

/* These two access functions provide the current transmit power level value */
INLINE UWORD16 mget_CurrentTxPower(void)
{
    return(MIBP.dot11TxPowerLevel[mget_CurrentTxPowerLevel() - 1]);
}

/*****************************************************************************/
/*                 Phy FHSS Table Access Functions                           */
/*****************************************************************************/

/* NOTE: Currently unused and hence no access fucntions are provided         */

/*****************************************************************************/
/*                 Phy DHSS Table Access Functions                           */
/*****************************************************************************/

/* dot11CCAModeSupported is a bit-significant value, representing all of the */
/* CCA modes supported by the PHY. Valid values are:                         */
/* energy detect only (ED_ONLY) = 01, carrier sense only (CS_ONLY) = 02,     */
/* carrier sense and energy detect (ED_and_CS)= 04 or the logical sum of any */
/* of these values.                                                          */
INLINE UWORD8 mget_CCAModeSupported(void)
{
    return(MIBP.dot11CCAModeSupported);
}

INLINE void mset_CCAModeSupported(UWORD8 inp)
{
    MIBP.dot11CCAModeSupported = inp;
}

/* The current CCA method in operation. Valid values are:                    */
/* energy detect only (edonly) = 01, carrier sense only (csonly) = 02,       */
/* carrier sense and energy detect (edandcs)= 04.                            */
INLINE UWORD8 mget_CurrentCCAMode(void)
{
    return(MIBP.dot11CurrentCCAMode);
}

INLINE void mset_CurrentCCAMode(UWORD8 inp)
{
    MIBP.dot11CurrentCCAMode = inp;
}

/* The current Energy Detect Threshold being used by the DSSS PHY. */
INLINE UWORD32 mget_EDThreshold(void)
{
    return(MIBP.dot11EDThreshold);
}

INLINE void mset_EDThreshold(UWORD32 inp)
{
    MIBP.dot11EDThreshold = inp;
}

/*****************************************************************************/
/*                 Phy IR Table Access Functions                             */
/*****************************************************************************/

/* NOTE: Currently unused and hence no access fucntions are provided */

/*****************************************************************************/
/*         Reg Domains Supported Table Access Functions                      */
/*****************************************************************************/

/* There are different operational requirements dependent on the regulatory  */
/* domain. This attribute list describes the regulatory domains the PLCP and */
/* PMD support in this implementation. Currently defined values and their    */
/* corresponding Regulatory Domains are:                                     */
/* FCC (USA) = X'10', DOC (Canada) = X'20', ETSI (most of Europe) = X'30',   */
/* Spain = X'31', France = X'32', MKK (Japan) = X'40'                        */
INLINE UWORD16 mget_RegDomainsSupportValue(void)
{
    return(MIBP.dot11RegDomainsSupportValue);
}

INLINE void mset_RegDomainsSupportValue(UWORD16 inp)
{
    MIBP.dot11RegDomainsSupportValue = inp;
}

/*****************************************************************************/
/*                Antenna List Table Access Functions                        */
/*****************************************************************************/

/* When true, this object indicates that the antenna represented by          */
/* dot11AntennaIndex can be used as a transmit antenna.                      */
INLINE TRUTH_VALUE_T mget_SupportedTxAntenna(UWORD8 idx)
{
    return(MIBP.dot11AntennasListTable[idx].dot11SupportedTxAntenna);
}

INLINE void mset_SupportedTxAntenna(TRUTH_VALUE_T inp, UWORD8 idx)
{
    MIBP.dot11AntennasListTable[idx].dot11SupportedTxAntenna = inp;
}

/* When true, this object indicates that the antenna represented by the      */
/* dot11AntennaIndex can be used as a receive antenna.                       */
INLINE TRUTH_VALUE_T mget_SupportedRxAntenna(UWORD8 idx)
{
    return(MIBP.dot11AntennasListTable[idx].dot11SupportedRxAntenna);
}

INLINE void mset_SupportedRxAntenna(TRUTH_VALUE_T inp, UWORD8 idx)
{
    MIBP.dot11AntennasListTable[idx].dot11SupportedRxAntenna = inp;
}

/* When true, it indicates that antenna represented by dot11AntennaIndex can */
/* be used for receive diversity. This object may only be true if the antenna*/
/* can be used as a receive antenna, as indicated by dot11SupportedRxAntenna.*/
INLINE TRUTH_VALUE_T mget_DiversitySelectionRx(UWORD8 idx)
{
    return(MIBP.dot11AntennasListTable[idx].dot11DiversitySelectionRx);
}

INLINE void mset_DiversitySelectionRx(TRUTH_VALUE_T inp, UWORD8 idx)
{
    MIBP.dot11AntennasListTable[idx].dot11DiversitySelectionRx = inp;
}

/*****************************************************************************/
/*           Supported Data rate Tx Table Access Functions                   */
/*****************************************************************************/

/* The Transmit bit rates supported by the PLCP and PMD, represented by a    */
/* count from X’02-X’7f, corresponding to data rates in increments of 500Kb/s*/
/* from 1 Mbit/s - 63.5 Mbit/s subject to limitations of each individual PHY */
INLINE rate_t mget_SupportedDataRatesTxValue(void)
{
    return(MIBP.dot11SupportedDataRatesTxValue);
}

INLINE void mset_SupportedDataRatesTxValue(rate_t inp)
{
    MIBP.dot11SupportedDataRatesTxValue = inp;
}

/*****************************************************************************/
/*           Supported Data rate Rx Table Access Functions                   */
/*****************************************************************************/

/* The receive bit rates supported by the PLCP and PMD, represented by count */
/* from X’02-X’7f, corresponding to data rates in increments of 500 Kb/s from*/
/* 1 Mbit/s to 63.5 Mbit/s.                                                  */
INLINE rate_t mget_SupportedDataRatesRxValue(void)
{
    return(MIBP.dot11SupportedDataRatesRxValue);
}

INLINE void mset_SupportedDataRatesRxValue(rate_t inp)
{
    MIBP.dot11SupportedDataRatesRxValue = inp;
}

/*****************************************************************************/
/*                     802.11a SPECIFIC MIB Access Functions                 */
/*****************************************************************************/

/* This object gives the valid frequency bands */
INLINE UWORD16 mget_FrequencyBandsSupported(void)
{
    return(MIBP.dot11FrequencyBandsSupported);
}

INLINE void mset_FrequencyBandsSupported(UWORD16 inp)
{
    MIBP.dot11FrequencyBandsSupported = inp;
}

/* This object gives the information regarding the current frequency in use */
INLINE UWORD8 mget_CurrentFrequency(void)
{
    return(MIBP.dot11CurrentFrequency);
}

INLINE void mset_CurrentFrequency(UWORD8 inp)
{
    MIBP.dot11CurrentFrequency = inp;
}

/* This object gives the TI Threshold in use. Currently unused. */
INLINE UWORD8 mget_TIThreshold(void)
{
    return(MIBP.dot11TIThreshold);
}

INLINE void mset_TIThreshold(UWORD8 inp)
{
    MIBP.dot11TIThreshold = inp;
}

/*****************************************************************************/
/*                    802.11b SPECIFIC MIB Access Functions                  */
/*****************************************************************************/

/* This object indicates if the PHY supports the short preamble option */
INLINE TRUTH_VALUE_T mget_ShortPreambleOptionImplemented(void)
{
    return(MIBP.dot11ShortPreambleOptionImplemented);
}

INLINE void mset_ShortPreambleOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ShortPreambleOptionImplemented = inp;
}

/* This object indicates if the PBCC is implemented */
INLINE TRUTH_VALUE_T mget_PBCCOptionImplemented(void)
{
    return(MIBP.dot11PBCCOptionImplemented);
}

INLINE void mset_PBCCOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11PBCCOptionImplemented = inp;
}

/* This objects has information on channel agility being implemented in PHY */
INLINE TRUTH_VALUE_T mget_ChannelAgilityPresent(void)
{
    return(MIBP.dot11ChannelAgilityPresent);
}

INLINE void mset_ChannelAgilityPresent(TRUTH_VALUE_T inp)
{
    MIBP.dot11ChannelAgilityPresent = inp;
}

/* This object, if true indicates thath the channel agility is enabled */
INLINE TRUTH_VALUE_T mget_ChannelAgilityEnabled(void)
{
    return(MIBP.dot11ChannelAgilityEnabled);
}

INLINE void mset_ChannelAgilityEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11ChannelAgilityEnabled = inp;
}

/*****************************************************************************/
/*                    802.11g SPECIFIC MIB Access Functions                  */
/*****************************************************************************/

/* This attribute, when true, shall indicate that ER-PBCC modulation option  */
/* as defined in 19.6 is implemented. The default value of this attribute is */
/* false.                                                                    */
INLINE TRUTH_VALUE_T mget_ERPBCCOptionImplemented(void)
{
    return(MIBP.dot11ERPBCCOptionImplemented);
}

INLINE void mset_ERPBCCOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ERPBCCOptionImplemented = inp;
}

/* This attribute, when true, shall indicate that DSSS-OFDM option as defined*/
/* in 19.7 is implemented. The default value of this attribute is false.     */
INLINE TRUTH_VALUE_T mget_DSSSOFDMOptionImplemented(void)
{
    return(MIBP.dot11DSSSOFDMOptionImplemented);
}

INLINE void mset_DSSSOFDMOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11DSSSOFDMOptionImplemented = inp;
}

/* This attribute, when true, shall indicate that DSSS-OFDM option as defined*/
/* in 19.7 is enabled. The default value of this attribute is false.         */
INLINE TRUTH_VALUE_T mget_DSSSOFDMOptionEnabled(void)
{
    return(MIBP.dot11DSSSOFDMOptionEnabled);
}

INLINE void mset_DSSSOFDMOptionEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11DSSSOFDMOptionEnabled = inp;
}

/* This attribute, when true, shall indicate that the Short Slot Time option */
/* as defined in subclause 7.3.1.4 is implemented. The default value of this */
/* attribute is false.                                                       */
INLINE TRUTH_VALUE_T mget_ShortSlotTimeOptionImplemented(void)
{
    return(MIBP.dot11ShortSlotTimeOptionImplemented);
}

INLINE void mset_ShortSlotTimeOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ShortSlotTimeOptionImplemented = inp;
}

/* This attribute, when true, shall indicate that the Short Slot Time option */
/* as defined in 7.3.1.4 is enabled. Default value of this attribute is false*/
INLINE TRUTH_VALUE_T mget_ShortSlotOptionEnabled(void)
{
    return(MIBP.dot11ShortSlotOptionEnabled);
}

INLINE void mset_ShortSlotOptionEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11ShortSlotOptionEnabled = inp;
}

/* dot11ERPCCAModeSupported is a bit-significant value, representing all of  */
/* CCA modes supported by the ERP. Valid values are:                         */
/* energy detect only (ED_ONLY) = 01,                                        */
/* ERP energy detect and carrier sense (ED_and_CS)= 32,                      */
/* or the logical sum of these values. In the ERP, this attribute shall be   */
/* used in preference to the dot11HRCCAModeSupported attribute.              */
INLINE UWORD8 mget_ERPCCAModeSupported(void)
{
    return(MIBP.dot11ERPCCAModeSupported);
}

INLINE void mset_ERPCCAModeSupported(UWORD8 inp)
{
    MIBP.dot11ERPCCAModeSupported = inp;
}

/*****************************************************************************/
/*          802.11n Specific MIB Access Functions                            */
/*****************************************************************************/
/* This attribute, when TRUE, indicates that the 40 MHz Operation is          */
/* implemented.                  .                                            */

INLINE TRUTH_VALUE_T mget_FortyMHzOperationImplemented(void)
{
    return(MIBP.dot11FortyMHzOperationImplemented);
}

INLINE void mset_FortyMHzOperationImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11FortyMHzOperationImplemented = inp;
}

/* This attribute, when TRUE, indicates that the 40 MHz Operation is enabled.*/
INLINE TRUTH_VALUE_T mget_FortyMHzOperationEnabled(void)
{
    return(MIBP.dot11FortyMHzOperationEnabled);
}

INLINE void mset_FortyMHzOperationEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11FortyMHzOperationEnabled = inp;
}
/* If 20/40 MHz Mode is currently in use then this attribute indicates the  */
/* primary channel.                                                         */
INLINE UWORD32 mget_CurrentPrimaryChannel(void)
{
    return(MIBP.dot11CurrentPrimaryChannel);
}

INLINE void mset_CurrentPrimaryChannel(UWORD32 inp)
{
    MIBP.dot11CurrentPrimaryChannel = inp;
}

/* This attribute indicates the channel number of the secondary channel. If   */
/* 20/40 MHz mode is not currently in use, this attribute value shall be 0.   */
INLINE UWORD32 mget_CurrentSecondaryChannel(void)
{
    return(MIBP.dot11CurrentSecondaryChannel);
}

INLINE void mset_CurrentSecondaryChannel(UWORD32 inp)
{
    MIBP.dot11CurrentSecondaryChannel = inp;
}

/* This attribute indicates the maximum number of spatial streams implemented.*/
INLINE UWORD8 mget_NumberOfSpatialStreamsImplemented(void)
{
    return(MIBP.dot11NumberOfSpatialStreamsImplemented);
}

INLINE void mset_NumberOfSpatialStreamsImplemented(UWORD8 inp)
{
    MIBP.dot11NumberOfSpatialStreamsImplemented = inp;
}
/* This attribute indicates the maximum number of spatial streams (1-4) enabled*/
INLINE UWORD8 mget_NumberOfSpatialStreamsEnabled(void)
{
    return(MIBP.dot11NumberOfSpatialStreamsEnabled);
}

INLINE void mset_NumberOfSpatialStreamsEnabled(UWORD8 inp)
{
    MIBP.dot11NumberOfSpatialStreamsEnabled = inp;
}
/* This attribute, when TRUE, indicates that the HT Greenfield option is implemented */
INLINE TRUTH_VALUE_T mget_GreenfieldOptionImplemented(void)
{
    return(MIBP.dot11GreenfieldOptionImplemented);
}
INLINE void mset_GreenfieldOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11GreenfieldOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the HT Greenfield option is enabled */
INLINE TRUTH_VALUE_T mget_GreenfieldOptionEnabled(void)
{
    return(MIBP.dot11GreenfieldOptionEnabled);
}

INLINE void mset_GreenfieldOptionEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11GreenfieldOptionEnabled = inp;
}

/* This attribute, when TRUE, indicates that  the Short Guard option is */
/* implemented for 20 MHz operation.                                    */
INLINE TRUTH_VALUE_T mget_ShortGIOptionInTwentyImplemented(void)
{
    return(MIBP.dot11ShortGIOptionInTwentyImplemented);
}

INLINE void mset_ShortGIOptionInTwentyImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ShortGIOptionInTwentyImplemented = inp;
}

/* This attribute, when TRUE, indicates that the Short Guard option is enabled*/
/* for 20 MHz operation.                                                      */
INLINE TRUTH_VALUE_T mget_ShortGIOptionInTwentyEnabled(void)
{
    return(MIBP.dot11ShortGIOptionInTwentyEnabled);
}

INLINE void mset_ShortGIOptionInTwentyEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11ShortGIOptionInTwentyEnabled = inp;
}

/* This attribute, when TRUE, indicates that  the Short Guard option is */
/* implemented for 40 MHz operation.                                    */
INLINE TRUTH_VALUE_T mget_ShortGIOptionInFortyImplemented(void)
{
    return(MIBP.dot11ShortGIOptionInFortyImplemented);
}

INLINE void mset_ShortGIOptionInFortyImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ShortGIOptionInFortyImplemented = inp;
}

/* This attribute, when TRUE, indicates that the Short Guard option is enabled */
/* for 40 MHz operation.                                                       */
INLINE TRUTH_VALUE_T mget_ShortGIOptionInFortyEnabled(void)
{
    return(MIBP.dot11ShortGIOptionInFortyEnabled);
}

INLINE void mset_ShortGIOptionInFortyEnabled(TRUTH_VALUE_T inp)
{
	if(TV_TRUE == MIBP.dot11ShortGIOptionInFortyImplemented)
	{
    	MIBP.dot11ShortGIOptionInFortyEnabled = inp;
	}
	else
	{
    	MIBP.dot11ShortGIOptionInFortyEnabled = TV_FALSE;
	}
}

/* This attribute, when TRUE, indicates that the LDPC coding option is implemented */
INLINE TRUTH_VALUE_T mget_LDPCCodingOptionImplemented(void)
{
    return(MIBP.dot11LDPCCodingOptionImplemented);
}

INLINE void mset_LDPCCodingOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11LDPCCodingOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the LDPC coding option is enabled */
INLINE TRUTH_VALUE_T mget_LDPCCodingOptionEnabled(void)
{
    return(MIBP.dot11LDPCCodingOptionEnabled);
}

INLINE void mset_LDPCCodingOptionEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11LDPCCodingOptionEnabled = inp;
}

/* This attribute, when TRUE, indicates that the entity is capable of  */
/* transmitting frames using Space-Time Block Code (STBC) option.      */
INLINE TRUTH_VALUE_T mget_TxSTBCOptionImplemented(void)
{
    return(MIBP.dot11TxSTBCOptionImplemented);
}

INLINE void mset_TxSTBCOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11TxSTBCOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the entity’s capability of */
/* transmitting frames using Space-Time Block Code (STBC) option        */
/* is enabled.                                                          */
INLINE TRUTH_VALUE_T mget_TxSTBCOptionEnabled(void)
{
    return(MIBP.dot11TxSTBCOptionEnabled);
}

INLINE void mset_TxSTBCOptionEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11TxSTBCOptionEnabled = inp;
}

/* This attribute, when TRUE, indicates that the entity is capable of */
/* receiving frames using Space-Time Block Code (STBC) option.        */

INLINE TRUTH_VALUE_T mget_RxSTBCOptionImplemented(void)
{
    return(MIBP.dot11RxSTBCOptionImplemented);
}

INLINE void mset_RxSTBCOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11RxSTBCOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the entity’s capability of */
/* receiving frames using Space-Time Block Code (STBC) option.          */
INLINE TRUTH_VALUE_T mget_RxSTBCOptionEnabled(void)
{
    return(MIBP.dot11RxSTBCOptionEnabled);
}

INLINE void mset_RxSTBCOptionEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11RxSTBCOptionEnabled = inp;
}
/* This attribute, when TRUE, indicates that the Beam Forming option is */
/* implemented.                                                         */
INLINE TRUTH_VALUE_T mget_BeamFormingOptionImplemented(void)
{
    return(MIBP.dot11BeamFormingOptionImplemented);
}

INLINE void mset_BeamFormingOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11BeamFormingOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the Beam Forming option is */
/* enabled.                                                             */
INLINE TRUTH_VALUE_T mget_BeamFormingOptionEnabled(void)
{
    return(MIBP.dot11BeamFormingOptionEnabled);
}

INLINE void mset_BeamFormingOptionEnabled(TRUTH_VALUE_T inp)
{
    MIBP.dot11BeamFormingOptionEnabled = inp;
}

/* This attribute shall specify the Highest Data Rate in Mb/s at which */
/* the station may receive data.                                       */
INLINE UWORD16 mget_HighestSupportedDataRate(void)
{
    return(MIBP.dot11HighestSupportedDataRate);
}

INLINE void mset_HighestSupportedDataRate(UWORD16 inp)
{
    MIBP.dot11HighestSupportedDataRate = inp;
}

/* This attribute, when TRUE, indicates that the Tx MCS set is defined */
INLINE TRUTH_VALUE_T mget_TxMCSSetDefined(void)
{
    return(MIBP.dot11TxMCSSetDefined);
}

INLINE void mset_TxMCSSetDefined(TRUTH_VALUE_T inp)
{
    MIBP.dot11TxMCSSetDefined = inp;
}
/* This attribute, when TRUE, indicates that the supported Tx and Rx MCS sets */
/* are not equal.                                                             */
INLINE TRUTH_VALUE_T mget_TxRxMCSSetNotEqual(void)
{
    return(MIBP.dot11TxRxMCSSetNotEqual);
}

INLINE void mset_TxRxMCSSetNotEqual(TRUTH_VALUE_T inp)
{
    MIBP.dot11TxRxMCSSetNotEqual = inp;
}

/* This attribute indicates the Tx maximum number of spatial streams supported */
INLINE UWORD8 mget_TxMaximumNumberSpatialStreamsSupported(void)
{
    return(MIBP.dot11TxMaximumNumberSpatialStreamsSupported);
}

INLINE void mset_TxMaximumNumberSpatialStreamsSupported(UWORD8 inp)
{
    MIBP.dot11TxMaximumNumberSpatialStreamsSupported = inp;
}

/* This attribute, when TRUE, indicates that Tx unequal modulation is supported */
INLINE TRUTH_VALUE_T mget_TxUnequalModulationSupported(void)
{
    return(MIBP.dot11TxUnequalModulationSupported);
}

INLINE void mset_TxUnequalModulationSupported(TRUTH_VALUE_T inp)
{
    MIBP.dot11TxUnequalModulationSupported = inp;
}

/******************************************************************************/
/*          Supported MCS Tx MIB Access Functions                             */
/******************************************************************************/
/* The Transmit MCS supported by the PLCP and PMD, represented by a count     */
/* from 1 to 127, subject to limitations of each individual PHY.              */
INLINE ht_mcs_t mget_SupportedMCSTxValue(void)
{
    return(MIBP.dot11SupportedMCSTxValue);
}

INLINE void mset_SupportedMCSTxValue(ht_mcs_t inp)
{
    MIBP.dot11SupportedMCSTxValue = inp;
}

/*****************************************************************************/
/*          Supported MCS Rx MIB Access Functions                            */
/*****************************************************************************/
/* The Receive MCS supported by the PLCP and PMD, represented by a count     */
/* from 1 to 127, subject to limitations of each individual PHY.             */
INLINE ht_mcs_t mget_SupportedMCSRxValue(void)
{
    return(MIBP.dot11SupportedMCSRxValue);
}

INLINE void mset_SupportedMCSRxValue(ht_mcs_t inp)
{
    MIBP.dot11SupportedMCSRxValue = inp;
}

/*****************************************************************************/
/*            TX Beamforming MIB Access Functions                            */
/*****************************************************************************/
/* This attribute, when TRUE, indicates that the STA implementation supports */
/* the receiving of staggered sounding frames.                               */
INLINE TRUTH_VALUE_T mget_ReceiveStaggerSoundingOptionImplemented(void)
{
    return(MIBP.dot11ReceiveStaggerSoundingOptionImplemented);
}

INLINE void mset_ReceiveStaggerSoundingOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ReceiveStaggerSoundingOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the STA implementation supports */
/* the transmission of staggered sounding frames.                            */
INLINE TRUTH_VALUE_T mget_TransmitStaggerSoundingOptionImplemented(void)
{
    return(MIBP.dot11TransmitStaggerSoundingOptionImplemented);
}

INLINE void mset_TransmitStaggerSoundingOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11TransmitStaggerSoundingOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the STA implementation is */
/* capable of receiving NDP as sounding frames.                        */
INLINE TRUTH_VALUE_T mget_ReceiveNDPOptionImplemented(void)
{
    return(MIBP.dot11ReceiveNDPOptionImplemented);
}

INLINE void mset_ReceiveNDPOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ReceiveNDPOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that the STA implementation is capable */
/* of transmitting NDP as sounding frames.                                     */
INLINE TRUTH_VALUE_T mget_TransmitNDPOptionImplemented(void)
{
    return(MIBP.dot11TransmitNDPOptionImplemented);
}

INLINE void mset_TransmitNDPOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11TransmitNDPOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that STA implementation is capable of */
/* applying implicit transmit beamforming.                                    */
INLINE TRUTH_VALUE_T mget_ImplicitTxBFOptionImplemented(void)
{
    return(MIBP.dot11ImplicitTxBFOptionImplemented);
}

INLINE void mset_ImplicitTxBFOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ImplicitTxBFOptionImplemented = inp;
}

/* This attribute indicates the level of calibration supported by the STA. */
/* 0=>inCapable; 1=>unableToInitiate; 2=>ableToInitiate; 3=>fullyCapable   */
INLINE UWORD8 mget_CalibrationOptionImplemented(void)
{
    return(MIBP.dot11CalibrationOptionImplemented);
}

INLINE void mset_CalibrationOptionImplemented(UWORD8 inp)
{
    MIBP.dot11CalibrationOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that STA implementation is capable of */
/* applying transmit beamforming using CSI explicit feedback in transmission. */
INLINE TRUTH_VALUE_T mget_ExplicitCSITxBFOptionImplemented(void)
{
    return(MIBP.dot11ExplicitCSITxBFOptionImplemented);
}

INLINE void mset_ExplicitCSITxBFOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ExplicitCSITxBFOptionImplemented = inp;
}

/* This attribute, when TRUE, indicates that STA implementation is capable of */
/* applying transmit beamforming using non-compressed beamforming matrix      */
/* explicit feedback in its transmission.                                     */
INLINE TRUTH_VALUE_T mget_ExplicitNonCompressedbeamformingMatrixOptionImplemented(void)
{
    return(MIBP.dot11ExplicitNonCompressedbeamformingMatrixOptionImplemented);
}

INLINE void mset_ExplicitNonCompressedbeamformingMatrixOptionImplemented(TRUTH_VALUE_T inp)
{
    MIBP.dot11ExplicitNonCompressedbeamformingMatrixOptionImplemented = inp;
}
/* This attribute indicates the level of CSI explicit feedback returned by the STA */
/* 0=>inCapable; 1=>delayed; 2=>immediate; 3=>unsolicitedImmediate; 4=>aggregated; */
/* 5=>delayedAggregated; 6=>immediateAggregated; 7=>unsolicitedImmediateAggregated */
INLINE UWORD8 mget_ExplicitBFCSIFeedbackOptionImplemented(void)
{
    return(MIBP.dot11ExplicitBFCSIFeedbackOptionImplemented);
}

INLINE void mset_ExplicitBFCSIFeedbackOptionImplemented(UWORD8 inp)
{
    MIBP.dot11ExplicitBFCSIFeedbackOptionImplemented = inp;
}

/* This attribute indicates the level of non-compressed beamforming matrix explicit */
/* feedback returned by the STA implementation.                                     */
/* 0=>inCapable; 1=>delayed; 2=>immediate; 3=>unsolicitedImmediate; 4=>aggregated;  */
/* 5=>delayedAggregated; 6=>immediateAggregated; 7=>unsolicitedImmediateAggregated  */
INLINE UWORD8 mget_ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented(void)
{
    return(MIBP.dot11ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented);
}

INLINE void mset_ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented(UWORD8 inp)
{
    MIBP.dot11ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented = inp;
}

/* This attribute indicates the level of non-compressed beamforming matrix explicit */
/* feedback returned by the STA implementation.                                     */
/* 0=>inCapable; 1=>delayed; 2=>immediate; 3=>unsolicitedImmediate; 4=>aggregated;  */
/* 5=>delayedAggregated; 6=>immediateAggregated; 7=>unsolicitedImmediateAggregated  */
INLINE UWORD8 mget_ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented(void)
{
    return(MIBP.dot11ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented);
}

INLINE void mset_ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented(UWORD8 inp)
{
    MIBP.dot11ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented = inp;
}

/* This attribute indicates the maximum number of beamform antennas the */
/* beamformee can support when CSI feedback is required. (1 to 4)       */
INLINE UWORD8 mget_NumberBeamFormingCSISupportAntenna(void)
{
    return(MIBP.dot11NumberBeamFormingCSISupportAntenna);
}

INLINE void mset_NumberBeamFormingCSISupportAntenna(UWORD8 inp)
{
    MIBP.dot11NumberBeamFormingCSISupportAntenna = inp;
}

/* This attribute indicates the maximum number of beamform antennas the   */
/* beamformee can support when non-compressed beamforming matrix feedback */
/* is required. (1 to 4)                                                  */
INLINE UWORD8 mget_NumberNonCompressedbeamformingMatrixSupportAntenna(void)
{
    return(MIBP.dot11NumberNonCompressedbeamformingMatrixSupportAntenna);
}

INLINE void mset_NumberNonCompressedbeamformingMatrixSupportAntenna(UWORD8 inp)
{
    MIBP.dot11NumberNonCompressedbeamformingMatrixSupportAntenna = inp;
}

/* This attribute indicates the maximum number of beamform antennas the  */
/* beamformee can support when compressed beamforming matrix feedback is */
/* required. (1 to 4)                                                    */
INLINE UWORD8 mget_NumberCompressedbeamformingMatrixSupportAntenna(void)
{
    return(MIBP.dot11NumberCompressedbeamformingMatrixSupportAntenna);
}

INLINE void mset_NumberCompressedbeamformingMatrixSupportAntenna(UWORD8 inp)
{
    MIBP.dot11NumberCompressedbeamformingMatrixSupportAntenna = inp;
}

/* This attribute indicates the maximum allowed delay in microseconds between  */
/* the transmission time of a sounding packet and the time a feedback response */
/* arrives (9.17.3 (Explicit feedback beamforming))                            */
INLINE UWORD32 mget_MaxCSIFeedbackDelay(void)
{
    return(MIBP.dot11MaxCSIFeedbackDelay);
}

INLINE void mset_MaxCSIFeedbackDelay(UWORD32 inp)
{
    MIBP.dot11MaxCSIFeedbackDelay = inp;
}

/*****************************************************************************/
/*                 Phy DHSS Table Access Functions                           */
/*****************************************************************************/

/* The current operating frequency channel of the DSSS PHY. Valid channel    */
/* numbers are as defined in 15.4.6.2                                        */
INLINE UWORD8 mget_CurrentChannel(void)
{
    return(MIBP.dot11CurrentChannel);
}

INLINE void mset_CurrentChannel(UWORD8 inp)
{
    UWORD8 ch_num = 0;

    MIBP.dot11CurrentChannel = inp;

    ch_num = get_ch_num_from_idx(get_current_start_freq(), inp);

    mset_CurrentPrimaryChannel(ch_num);
}

/*****************************************************************************/
/* Private PHY-MIB Access Functions                                          */
/*****************************************************************************/

/* This attribute indicates whether the PHY Implementation uses DSSS/CCK in */
/* 40MHz channel bandwidth.                                                 */
INLINE TRUTH_VALUE_T mget_dsss_cck_40mhz_mode(void)
{
    return(P_MIBP.dsss_cck_40mhz_mode);
}

INLINE void mset_dsss_cck_40mhz_mode(TRUTH_VALUE_T inp)
{
    P_MIBP.dsss_cck_40mhz_mode = inp;
}

#endif /* MIB_802_11N_H */

#endif /* PHY_802_11n */
