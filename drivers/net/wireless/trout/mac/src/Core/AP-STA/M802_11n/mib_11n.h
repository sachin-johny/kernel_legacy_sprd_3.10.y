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
/*  File Name         : mib_11n.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB as given in Annex-D of 802.11n        */
/*                      standard. Access functions to get and set the MIB    */
/*                      values are also provided.                            */
/*                                                                           */
/*  List of Functions : Access functions for all 802.11n related MIB         */
/*                      parameters.                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef MAC_802_11N

#ifndef MIB_11N_H
#define MIB_11N_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib1.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MCS_BITMASK_LEN        10
#define UWORD32_MAX            0xFFFFFFFF

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* HT MCS set. This contains the set of MCS a station supports*/
typedef struct
{
    UWORD8 mcs_bmp[MCS_BITMASK_LEN];
    UWORD8 num_mcs;
} ht_mcs_t;

typedef struct
{
    UWORD32 lo_count;
    UWORD32 hi_count;
}counter64_t;

typedef struct
{
    /* Additions to SMT Station Configuration Table */
    /*MIB to indicate whether the entity is HT Capable.*/
    TRUTH_VALUE_T dot11HighThroughputOptionImplemented;

    /*************************************************************************/
    /* HT Station Configuration Table - members of dot11StationConfigEntry   */
    /*************************************************************************/
    ht_mcs_t        dot11HTOperationalMCSSet;
    /*1 => static; 2 => dynamic; 3 => mimo*/
    UWORD8          dot11MIMOPowerSave;
    TRUTH_VALUE_T   dot11NDelayedBlockAckOptionImplemented;
    /*Maximum AMSDU Length: 3839(Default) or 7935*/
    UWORD16         dot11MaxAMSDULength;
    TRUTH_VALUE_T   dot11PSMPOptionImplemented;
    TRUTH_VALUE_T   dot11STBCControlFrameOptionImplemented;
    TRUTH_VALUE_T   dot11LsigTxopProtectionOptionImplemented;
    /*The Maximum Rx A-MPDU length = 2^(13 + dot11MaxRxAMPDUFactor) - 1*/
    UWORD8          dot11MaxRxAMPDUFactor;
    /*The minimum time between the start of adjacent MPDUs within an A-MPDU*/
    /*0 => no restriction; 1 => (1/4)usec; 2 => (1/2) usec; 3 => 1 usec;   */
    /*4 => 2 usec;         5 => 4 usec;    6 => 8 usec;     7 => 16 usec   */
    UWORD8          dot11MinimumMPDUStartSpacing;
    TRUTH_VALUE_T   dot11PCOOptionImplemented;
    /*The minimum transition time within which the STA can switch between   */
    /*20 MHz channel width and 40 MHz channel width.                        */
    /*1 => 400 usec; 2 => 1500 usec; 3 => 5000 usec (Default)               */
    UWORD8          dot11TransitionTime;
    /*The MCS feedback capability of the STA.*/
    /*0 => none; 2 => unsolicited; 3 => both */
    UWORD8          dot11MCSFeedbackOptionImplemented;
    TRUTH_VALUE_T   dot11HTControlFieldSupported;
    TRUTH_VALUE_T   dot11RDResponderOptionImplemented;
    /*************************************************************************/
    /* Operation Table - members of Dot11OperationEntry                      */
    /*************************************************************************/
    /*0 => htPure; 1 => optionalProtection; 2 => mandatoryFortyProtection    */
    /*3 => mandatoryAllProtection                                            */
    UWORD8          dot11HTOperatingMode;
    TRUTH_VALUE_T   dot11RIFSMode;
    TRUTH_VALUE_T   dot11PSMPControlledAccess;
    /*Value of the granularity is (dot11ServiceIntervalGranularity+1)*5 ms.*/
    UWORD8          dot11ServiceIntervalGranularity;
    TRUTH_VALUE_T   dot11DualCTSProtection;
    TRUTH_VALUE_T   dot11LSigTxopFullProtectionEnabled;
    TRUTH_VALUE_T   dot11NonGFEntitiesPresent;
    TRUTH_VALUE_T   dot11PCOActivated;
    UWORD16         dot11PCO40MaxDuration;
    UWORD16         dot11PCO20MaxDuration;
    UWORD16         dot11PCO40MinDuration;
    UWORD16         dot11PCO20MinDuration;
    TRUTH_VALUE_T   dot11FortyMHzIntolerant;
    /*************************************************************************/
    /* Counters Table - members of Dot11CountersEntry                        */
    /*************************************************************************/
    UWORD32         dot11TransmittedAMSDUCount;
    UWORD32         dot11FailedAMSDUCount;
    UWORD32         dot11RetryAMSDUCount;
    UWORD32         dot11MultipleRetryAMSDUCount;
    counter64_t     dot11TransmittedOctetsInAMSDU;
    UWORD32         dot11AMSDUAckFailureCount;
    UWORD32         dot11ReceivedAMSDUCount;
    counter64_t     dot11ReceivedOctesInAMSDUCount;
    UWORD32         dot11TransmittedAMPDUCount;
    UWORD32         dot11TransmittedMPDUsInAMPDUCount;
    counter64_t     dot11TransmittedOctetsInAMPDUCount;
    UWORD32         dot11AMPDUReceivedCount;
    UWORD32         dot11MPDUInReceivedAMPDUCount;
    counter64_t     dot11ReceivedOctetsInAMPDUCount;
    UWORD32         dot11AMPDUDelimiterCRCErrorCount;
    UWORD32         dot11ImplicitBARFailureCount;
    UWORD32         dot11ExplicitBARFailureCount;
    UWORD32         dot11ChannelWidthSwitchCount;
    UWORD32         dot11TwentyMHzTransmittedFrameCount;
    UWORD32         dot11FortyMHzTransmittedFrameCount;
    UWORD32         dot11TwentyMHzReceivedFrameCount;
    UWORD32         dot11FortyMHzReceivedFrameCount;
    UWORD32         dot11PSMPSuccessCount;
    UWORD32         dot11PSMPFailureCount;
    UWORD32         dot11GrantedRDGUsedCount;
    UWORD32         dot11GrantedRDGUnusedCount;
    UWORD32         dot11TransmittedFramesInGrantedRDGCount;
    counter64_t     dot11TransmittedOctetsInGrantedRDG;
    UWORD32         dot11BeamformingFrameCount;
    UWORD32         dot11DualCTSSuccessCount;
    UWORD32         dot11DualCTSFailureCount;
    UWORD32         dot11STBCCTSSuccessCount;
    UWORD32         dot11STBCCTSFailureCount;
    UWORD32         dot11nonSTBCCTSFailureCount;
    UWORD32         dot11RTSLSIGSuccessCount;
    UWORD32         dot11RTSLSIGFailureCount;
}mib_11n_t;

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern mib_11n_t MIB_11N;

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void initialize_mac_mib_11n(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function increments a 64-bit counter by the specified 32-bit value. */
INLINE void incr_counter64(counter64_t *ctr, UWORD32 incr_val)
{
    /* Check for overflow of the lower 32-bits. */
    if((UWORD32_MAX - incr_val) < ctr->lo_count)
        ctr->hi_count++;

    ctr->lo_count += incr_val;
}

INLINE TRUTH_VALUE_T mget_HighThroughputOptionImplemented(void)
{
    return (MIB_11N.dot11HighThroughputOptionImplemented);
}

INLINE void mset_HighThroughputOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11HighThroughputOptionImplemented = inp;
}

INLINE ht_mcs_t mget_HTOperationalMCSSet(void)
{
    return (MIB_11N.dot11HTOperationalMCSSet);
}

INLINE void mset_HTOperationalMCSSet(ht_mcs_t inp)
{
    MIB_11N.dot11HTOperationalMCSSet = inp;
}

INLINE UWORD8 mget_MIMOPowerSave(void)
{
    return (MIB_11N.dot11MIMOPowerSave);
}

INLINE void mset_MIMOPowerSave(UWORD8 inp)
{
    MIB_11N.dot11MIMOPowerSave = inp;
}

INLINE TRUTH_VALUE_T mget_NDelayedBlockAckOptionImplemented(void)
{
    return (MIB_11N.dot11NDelayedBlockAckOptionImplemented);
}

INLINE void mset_NDelayedBlockAckOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11NDelayedBlockAckOptionImplemented = inp;
}

INLINE UWORD16 mget_MaxAMSDULength(void)
{
    return (MIB_11N.dot11MaxAMSDULength);
}

INLINE void mset_MaxAMSDULength(UWORD16 inp)
{
    MIB_11N.dot11MaxAMSDULength = inp;
}

INLINE TRUTH_VALUE_T mget_PSMPOptionImplemented(void)
{
    return (MIB_11N.dot11PSMPOptionImplemented);
}

INLINE void mset_PSMPOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11PSMPOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_STBCControlFrameOptionImplemented(void)
{
    return (MIB_11N.dot11STBCControlFrameOptionImplemented);
}

INLINE void mset_STBCControlFrameOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11STBCControlFrameOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_LsigTxopProtectionOptionImplemented(void)
{
    return (MIB_11N.dot11LsigTxopProtectionOptionImplemented);
}

INLINE void mset_LsigTxopProtectionOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11LsigTxopProtectionOptionImplemented = inp;
}

INLINE UWORD8 mget_MaxRxAMPDUFactor(void)
{
    return (MIB_11N.dot11MaxRxAMPDUFactor);
}

INLINE void mset_MaxRxAMPDUFactor(UWORD8 inp)
{
    MIB_11N.dot11MaxRxAMPDUFactor = inp;
}

INLINE UWORD8 mget_MinimumMPDUStartSpacing(void)
{
    return (MIB_11N.dot11MinimumMPDUStartSpacing);
}

INLINE void mset_MinimumMPDUStartSpacing(UWORD8 inp)
{
    MIB_11N.dot11MinimumMPDUStartSpacing = inp;
}

INLINE TRUTH_VALUE_T mget_PCOOptionImplemented(void)
{
    return (MIB_11N.dot11PCOOptionImplemented);
}

INLINE void mset_PCOOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11PCOOptionImplemented = inp;
}

INLINE UWORD8 mget_TransitionTime(void)
{
    return (MIB_11N.dot11TransitionTime);
}

INLINE void mset_TransitionTime(UWORD8 inp)
{
    MIB_11N.dot11TransitionTime = inp;
}

INLINE UWORD8 mget_MCSFeedbackOptionImplemented(void)
{
    return (MIB_11N.dot11MCSFeedbackOptionImplemented);
}

INLINE void mset_MCSFeedbackOptionImplemented(UWORD8 inp)
{
    MIB_11N.dot11MCSFeedbackOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_HTControlFieldSupported(void)
{
    return (MIB_11N.dot11HTControlFieldSupported);
}

INLINE void mset_HTControlFieldSupported(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11HTControlFieldSupported = inp;
}

INLINE TRUTH_VALUE_T mget_RDResponderOptionImplemented(void)
{
    return (MIB_11N.dot11RDResponderOptionImplemented);
}

INLINE void mset_RDResponderOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11RDResponderOptionImplemented = inp;
}

INLINE UWORD8 mget_HTOperatingMode(void)
{
    return (MIB_11N.dot11HTOperatingMode);
}

INLINE void mset_HTOperatingMode(UWORD8 inp)
{
    MIB_11N.dot11HTOperatingMode = inp;
}

INLINE TRUTH_VALUE_T mget_RIFSMode(void)
{
    return (MIB_11N.dot11RIFSMode);
}

INLINE void mset_RIFSMode(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11RIFSMode = inp;
}

INLINE TRUTH_VALUE_T mget_PSMPControlledAccess(void)
{
    return (MIB_11N.dot11PSMPControlledAccess);
}

INLINE void mset_PSMPControlledAccess(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11PSMPControlledAccess = inp;
}

INLINE UWORD8 mget_ServiceIntervalGranularity(void)
{
    return (MIB_11N.dot11ServiceIntervalGranularity);
}

INLINE void mset_ServiceIntervalGranularity(UWORD8 inp)
{
    MIB_11N.dot11ServiceIntervalGranularity = inp;
}

INLINE TRUTH_VALUE_T mget_DualCTSProtection(void)
{
    return (MIB_11N.dot11DualCTSProtection);
}

INLINE void mset_DualCTSProtection(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11DualCTSProtection = inp;
}

INLINE TRUTH_VALUE_T mget_LSigTxopFullProtectionEnabled(void)
{
    return (MIB_11N.dot11LSigTxopFullProtectionEnabled);
}

INLINE void mset_LSigTxopFullProtectionEnabled(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11LSigTxopFullProtectionEnabled = inp;
}

INLINE TRUTH_VALUE_T mget_NonGFEntitiesPresent(void)
{
    return (MIB_11N.dot11NonGFEntitiesPresent);
}

INLINE void mset_NonGFEntitiesPresent(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11NonGFEntitiesPresent = inp;
}

INLINE TRUTH_VALUE_T mget_PCOActivated(void)
{
    return (MIB_11N.dot11PCOActivated);
}

INLINE void mset_PCOActivated(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11PCOActivated = inp;
}

INLINE UWORD16 mget_PCO40MaxDuration(void)
{
    return (MIB_11N.dot11PCO40MaxDuration);
}

INLINE void mset_PCO40MaxDuration(UWORD16 inp)
{
    MIB_11N.dot11PCO40MaxDuration = inp;
}

INLINE UWORD16 mget_PCO20MaxDuration(void)
{
    return (MIB_11N.dot11PCO20MaxDuration);
}

INLINE void mset_PCO20MaxDuration(UWORD16 inp)
{
    MIB_11N.dot11PCO20MaxDuration = inp;
}

INLINE UWORD16 mget_PCO40MinDuration(void)
{
    return (MIB_11N.dot11PCO40MinDuration);
}

INLINE void mset_PCO40MinDuration(UWORD16 inp)
{
    MIB_11N.dot11PCO40MinDuration = inp;
}

INLINE UWORD16 mget_PCO20MinDuration(void)
{
    return (MIB_11N.dot11PCO20MinDuration);
}

INLINE void mset_PCO20MinDuration(UWORD16 inp)
{
    MIB_11N.dot11PCO20MinDuration = inp;
}

INLINE TRUTH_VALUE_T mget_FortyMHzIntolerant(void)
{
    return (MIB_11N.dot11FortyMHzIntolerant);
}

INLINE void mset_FortyMHzIntolerant(TRUTH_VALUE_T inp)
{
    MIB_11N.dot11FortyMHzIntolerant = inp;
}


INLINE UWORD32 mget_TransmittedAMSDUCount(void)
{
    return (MIB_11N.dot11TransmittedAMSDUCount);
}

INLINE void mset_TransmittedAMSDUCount(UWORD32 inp)
{
    MIB_11N.dot11TransmittedAMSDUCount = inp;
}

INLINE UWORD32 mget_FailedAMSDUCount(void)
{
    return (MIB_11N.dot11FailedAMSDUCount);
}

INLINE void mset_FailedAMSDUCount(UWORD32 inp)
{
    MIB_11N.dot11FailedAMSDUCount = inp;
}

INLINE UWORD32 mget_RetryAMSDUCount(void)
{
    return (MIB_11N.dot11RetryAMSDUCount);
}

INLINE void mset_RetryAMSDUCount(UWORD32 inp)
{
    MIB_11N.dot11RetryAMSDUCount = inp;
}

INLINE UWORD32 mget_MultipleRetryAMSDUCount(void)
{
    return (MIB_11N.dot11MultipleRetryAMSDUCount);
}

INLINE void mset_MultipleRetryAMSDUCount(UWORD32 inp)
{
    MIB_11N.dot11MultipleRetryAMSDUCount = inp;
}

INLINE counter64_t mget_TransmittedOctetsInAMSDU(void)
{
    return (MIB_11N.dot11TransmittedOctetsInAMSDU);
}

INLINE void mset_TransmittedOctetsInAMSDU(counter64_t inp)
{
    MIB_11N.dot11TransmittedOctetsInAMSDU = inp;
}

INLINE void mincr_TransmittedOctetsInAMSDU(UWORD32 inp)
{
    incr_counter64(&(MIB_11N.dot11TransmittedOctetsInAMSDU), inp);
}


INLINE UWORD32 mget_AMSDUAckFailureCount(void)
{
    return (MIB_11N.dot11AMSDUAckFailureCount);
}

INLINE void mset_AMSDUAckFailureCount(UWORD32 inp)
{
    MIB_11N.dot11AMSDUAckFailureCount = inp;
}

INLINE UWORD32 mget_ReceivedAMSDUCount(void)
{
    return (MIB_11N.dot11ReceivedAMSDUCount);
}

INLINE void mset_ReceivedAMSDUCount(UWORD32 inp)
{
    MIB_11N.dot11ReceivedAMSDUCount = inp;
}

INLINE counter64_t mget_ReceivedOctesInAMSDUCount(void)
{
    return (MIB_11N.dot11ReceivedOctesInAMSDUCount);
}

INLINE void mset_ReceivedOctesInAMSDUCount(counter64_t inp)
{
    MIB_11N.dot11ReceivedOctesInAMSDUCount = inp;
}

INLINE void mincr_ReceivedOctesInAMSDUCount(UWORD32 inp)
{
    incr_counter64(&(MIB_11N.dot11ReceivedOctesInAMSDUCount), inp);
}


INLINE UWORD32 mget_TransmittedAMPDUCount(void)
{
    return (MIB_11N.dot11TransmittedAMPDUCount);
}

INLINE void mset_TransmittedAMPDUCount(UWORD32 inp)
{
    MIB_11N.dot11TransmittedAMPDUCount = inp;
}

INLINE UWORD32 mget_TransmittedMPDUsInAMPDUCount(void)
{
    return (MIB_11N.dot11TransmittedMPDUsInAMPDUCount);
}

INLINE void mset_TransmittedMPDUsInAMPDUCount(UWORD32 inp)
{
    MIB_11N.dot11TransmittedMPDUsInAMPDUCount = inp;
}

INLINE counter64_t mget_TransmittedOctetsInAMPDUCount(void)
{
    return (MIB_11N.dot11TransmittedOctetsInAMPDUCount);
}

INLINE void mset_TransmittedOctetsInAMPDUCount(counter64_t inp)
{
    MIB_11N.dot11TransmittedOctetsInAMPDUCount = inp;
}

INLINE void mincr_TransmittedOctetsInAMPDUCount(UWORD32 inp)
{
    incr_counter64(&(MIB_11N.dot11TransmittedOctetsInAMPDUCount), inp);
}

INLINE UWORD32 mget_AMPDUReceivedCount(void)
{
    return (MIB_11N.dot11AMPDUReceivedCount);
}

INLINE void mset_AMPDUReceivedCount(UWORD32 inp)
{
    MIB_11N.dot11AMPDUReceivedCount = inp;
}

INLINE UWORD32 mget_MPDUInReceivedAMPDUCount(void)
{
    return (MIB_11N.dot11MPDUInReceivedAMPDUCount);
}

INLINE void mset_MPDUInReceivedAMPDUCount(UWORD32 inp)
{
    MIB_11N.dot11MPDUInReceivedAMPDUCount = inp;
}

INLINE counter64_t mget_ReceivedOctetsInAMPDUCount(void)
{
    return (MIB_11N.dot11ReceivedOctetsInAMPDUCount);
}

INLINE void mset_ReceivedOctetsInAMPDUCount(counter64_t inp)
{
    MIB_11N.dot11ReceivedOctetsInAMPDUCount = inp;
}

INLINE void mincr_ReceivedOctetsInAMPDUCount(UWORD32 inp)
{
    incr_counter64(&(MIB_11N.dot11ReceivedOctetsInAMPDUCount), inp);
}

INLINE UWORD32 mget_AMPDUDelimiterCRCErrorCount(void)
{
    return (MIB_11N.dot11AMPDUDelimiterCRCErrorCount);
}

INLINE void mset_AMPDUDelimiterCRCErrorCount(UWORD32 inp)
{
    MIB_11N.dot11AMPDUDelimiterCRCErrorCount = inp;
}

INLINE UWORD32 mget_ImplicitBARFailureCount(void)
{
    return (MIB_11N.dot11ImplicitBARFailureCount);
}

INLINE void mset_ImplicitBARFailureCount(UWORD32 inp)
{
    MIB_11N.dot11ImplicitBARFailureCount = inp;
}

INLINE UWORD32 mget_ExplicitBARFailureCount(void)
{
    return (MIB_11N.dot11ExplicitBARFailureCount);
}

INLINE void mset_ExplicitBARFailureCount(UWORD32 inp)
{
    MIB_11N.dot11ExplicitBARFailureCount = inp;
}

INLINE UWORD32 mget_ChannelWidthSwitchCount(void)
{
    return (MIB_11N.dot11ChannelWidthSwitchCount);
}

INLINE void mset_ChannelWidthSwitchCount(UWORD32 inp)
{
    MIB_11N.dot11ChannelWidthSwitchCount = inp;
}

INLINE UWORD32 mget_TwentyMHzTransmittedFrameCount(void)
{
    return (MIB_11N.dot11TwentyMHzTransmittedFrameCount);
}

INLINE void mset_TwentyMHzTransmittedFrameCount(UWORD32 inp)
{
    MIB_11N.dot11TwentyMHzTransmittedFrameCount = inp;
}

INLINE UWORD32 mget_FortyMHzTransmittedFrameCount(void)
{
    return (MIB_11N.dot11FortyMHzTransmittedFrameCount);
}

INLINE void mset_FortyMHzTransmittedFrameCount(UWORD32 inp)
{
    MIB_11N.dot11FortyMHzTransmittedFrameCount = inp;
}

INLINE UWORD32 mget_TwentyMHzReceivedFrameCount(void)
{
    return (MIB_11N.dot11TwentyMHzReceivedFrameCount);
}

INLINE void mset_TwentyMHzReceivedFrameCount(UWORD32 inp)
{
    MIB_11N.dot11TwentyMHzReceivedFrameCount = inp;
}

INLINE UWORD32 mget_FortyMHzReceivedFrameCount(void)
{
    return (MIB_11N.dot11FortyMHzReceivedFrameCount);
}

INLINE void mset_FortyMHzReceivedFrameCount(UWORD32 inp)
{
    MIB_11N.dot11FortyMHzReceivedFrameCount = inp;
}

INLINE UWORD32 mget_PSMPSuccessCount(void)
{
    return (MIB_11N.dot11PSMPSuccessCount);
}

INLINE void mset_PSMPSuccessCount(UWORD32 inp)
{
    MIB_11N.dot11PSMPSuccessCount = inp;
}

INLINE UWORD32 mget_PSMPFailureCount(void)
{
    return (MIB_11N.dot11PSMPFailureCount);
}

INLINE void mset_PSMPFailureCount(UWORD32 inp)
{
    MIB_11N.dot11PSMPFailureCount = inp;
}

INLINE UWORD32 mget_GrantedRDGUsedCount(void)
{
    return (MIB_11N.dot11GrantedRDGUsedCount);
}

INLINE void mset_GrantedRDGUsedCount(UWORD32 inp)
{
    MIB_11N.dot11GrantedRDGUsedCount = inp;
}

INLINE UWORD32 mget_GrantedRDGUnusedCount(void)
{
    return (MIB_11N.dot11GrantedRDGUnusedCount);
}

INLINE void mset_GrantedRDGUnusedCount(UWORD32 inp)
{
    MIB_11N.dot11GrantedRDGUnusedCount = inp;
}

INLINE UWORD32 mget_TransmittedFramesInGrantedRDGCount(void)
{
    return (MIB_11N.dot11TransmittedFramesInGrantedRDGCount);
}

INLINE void mset_TransmittedFramesInGrantedRDGCount(UWORD32 inp)
{
    MIB_11N.dot11TransmittedFramesInGrantedRDGCount = inp;
}

INLINE counter64_t mget_TransmittedOctetsInGrantedRDG(void)
{
    return (MIB_11N.dot11TransmittedOctetsInGrantedRDG);
}

INLINE void mset_TransmittedOctetsInGrantedRDG(counter64_t inp)
{
    MIB_11N.dot11TransmittedOctetsInGrantedRDG = inp;
}

INLINE void mincr_TransmittedOctetsInGrantedRDG(UWORD32 inp)
{
    incr_counter64(&(MIB_11N.dot11TransmittedOctetsInGrantedRDG), inp);
}

INLINE UWORD32 mget_BeamformingFrameCount(void)
{
    return (MIB_11N.dot11BeamformingFrameCount);
}

INLINE void mset_BeamformingFrameCount(UWORD32 inp)
{
    MIB_11N.dot11BeamformingFrameCount = inp;
}

INLINE UWORD32 mget_DualCTSSuccessCount(void)
{
    return (MIB_11N.dot11DualCTSSuccessCount);
}

INLINE void mset_DualCTSSuccessCount(UWORD32 inp)
{
    MIB_11N.dot11DualCTSSuccessCount = inp;
}

INLINE UWORD32 mget_DualCTSFailureCount(void)
{
    return (MIB_11N.dot11DualCTSFailureCount);
}

INLINE void mset_DualCTSFailureCount(UWORD32 inp)
{
    MIB_11N.dot11DualCTSFailureCount = inp;
}

INLINE UWORD32 mget_STBCCTSSuccessCount(void)
{
    return (MIB_11N.dot11STBCCTSSuccessCount);
}

INLINE void mset_STBCCTSSuccessCount(UWORD32 inp)
{
    MIB_11N.dot11STBCCTSSuccessCount = inp;
}

INLINE UWORD32 mget_STBCCTSFailureCount(void)
{
    return (MIB_11N.dot11STBCCTSFailureCount);
}

INLINE void mset_STBCCTSFailureCount(UWORD32 inp)
{
    MIB_11N.dot11STBCCTSFailureCount = inp;
}

INLINE UWORD32 mget_nonSTBCCTSFailureCount(void)
{
    return (MIB_11N.dot11nonSTBCCTSFailureCount);
}

INLINE void mset_nonSTBCCTSFailureCount(UWORD32 inp)
{
    MIB_11N.dot11nonSTBCCTSFailureCount = inp;
}

INLINE UWORD32 mget_RTSLSIGSuccessCount(void)
{
    return (MIB_11N.dot11RTSLSIGSuccessCount);
}

INLINE void mset_RTSLSIGSuccessCount(UWORD32 inp)
{
    MIB_11N.dot11RTSLSIGSuccessCount = inp;
}

INLINE UWORD32 mget_RTSLSIGFailureCount(void)
{
    return (MIB_11N.dot11RTSLSIGFailureCount);
}

INLINE void mset_RTSLSIGFailureCount(UWORD32 inp)
{
    MIB_11N.dot11RTSLSIGFailureCount = inp;
}

#endif /*MIB_11N_H*/

#endif /*MAC_802_11N*/
