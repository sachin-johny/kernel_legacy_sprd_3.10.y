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
/*  File Name         : mib_11e.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions and structures    */
/*                      related to MIB as given in Annex-D of 802.11 e       */
/*                      standard. Access functions to get and set the MIB    */
/*                      values are also provided.                            */
/*                                                                           */
/*  List of Functions : Access functions for all 802.11E related MIB         */
/*                      parameters.                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_WMM

#ifndef MIB_11E_H
#define MIB_11E_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mib1.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define NUM_TIDS 16
#define NUM_AC   4

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Counters used in QoS per TID */
typedef struct
{
    UWORD32 dot11QoSCountersIndex;
    UWORD32 dot11QoSTransmittedFragmentCount;
    UWORD32 dot11QoSFailedCount;
    UWORD32 dot11QoSRetryCount;
    UWORD32 dot11QoSMultipleRetryCount;
    UWORD32 dot11QoSFrameDuplicateCount;
    UWORD32 dot11QoSRTSSuccessCount;
    UWORD32 dot11QoSRTSFailureCount;
    UWORD32 dot11QoSAckFailureCount;
    UWORD32 dot11QoSReceivedFragmentCount;
    UWORD32 dot11QoSTransmittedFrameCount;
    UWORD32 dot11QoSDiscardedFrameCount;
    UWORD32 dot11QoSMPDUsReceivedCount;
    UWORD32 dot11QoSRetriesReceivedCount;
} dot11_qos_counter_t;

/* EDCA Table used for STA */
typedef struct
{
    UWORD32       dot11EDCATableIndex;
    UWORD32       dot11EDCATableCWmin;
    UWORD32       dot11EDCATableCWmax;
    UWORD32       dot11EDCATableAIFSN;
    UWORD32       dot11EDCATableTXOPLimit;
    UWORD32       dot11EDCATableMSDULifetime;
    TRUTH_VALUE_T dot11EDCATableMandatory;
} dot11_edca_table_t;

/* EDCA Table used for QAP */
typedef struct
{
    UWORD32       dot11QAPEDCATableIndex;
    UWORD32       dot11QAPEDCATableCWmin;
    UWORD32       dot11QAPEDCATableCWmax;
    UWORD32       dot11QAPEDCATableAIFSN;
    UWORD32       dot11QAPEDCATableTXOPLimit;
    UWORD32       dot11QAPEDCATableMSDULifetime;
    TRUTH_VALUE_T dot11QAPEDCATableMandatory;
} dot11_qap_edca_table_t;

/* MIB for 11e */
typedef struct
{
    /* Additions to SMT Station Configuration Table */

    /* MIB used to indicate whether QoS is implemented or not */
    TRUTH_VALUE_T dot11QoSOptionImplemented;

    /* Block Ack Option */
    TRUTH_VALUE_T dot11ImmediateBlockAckOptionImplemented;
    TRUTH_VALUE_T dot11DelayedBlockAckOptionImplemented;

    /* DLS Option */
    TRUTH_VALUE_T dot11DirectOptionImplemented;

    /* Advanced Power Management Option */
    TRUTH_VALUE_T dot11APSDOptionImplemented;

    /* Q Ack Option */
    TRUTH_VALUE_T dot11QAckOptionImplemented;

    /* Q BSS Load option */
    TRUTH_VALUE_T dot11QBSSLoadOptionImplemented;

    /* Queue request option */
    TRUTH_VALUE_T dot11QueueRequestOptionImplented;

    /* TXOP Request option */
    TRUTH_VALUE_T dot11TXOPRequestOptionImplemented;

    /* More Data Ack Option */
    TRUTH_VALUE_T dot11MoreDataAckOptionImplemented;

    /* MIB used to tell STA whether to join Non-Q BSS */
    TRUTH_VALUE_T dot11AssociateInNQBSS;

    /* DLS related MIBs */
    TRUTH_VALUE_T dot11DLSAllowedInQBSS;
    TRUTH_VALUE_T dot11DLSAllowed;

    /* Addition to Operation Table */
    UWORD16       dot11CAPLimit;
    UWORD16       dot11HCCWmin;
    UWORD16       dot11HCCWmax;
    UWORD16       dot11HCCAIFSN;
    UWORD16       dot11ADDBAResponseTimeout;
    UWORD16       dot11ADDTSResponseTimeout;
    UWORD16       dot11ChannelUtilzationBeaconInterval;
    UWORD16       dot11ScheduleTimeout;
    UWORD16       dot11DLSResponseTimeout;
    UWORD16       dot11QAPMissingAckRetryLimit;
    UWORD16       dot11EDCAAveragingPeriod;

    /* Additions to Counters Table */
    UWORD32       dot11QoSDiscardedFragmentCount;
    UWORD32       dot11AssociatedStationCount;
    UWORD32       dot11QoSCFPollsReceivedCount;
    UWORD32       dot11QoSCFPollsUnusedCount;
    UWORD32       dot11QoSCFPollsUnusableCount;

    /* EDCA Config Table */
    UWORD8       dot11QoSEDCAParameterSetUpdateCount;
    dot11_edca_table_t EDCATable[NUM_AC];

    /* QoS Counters Table */
    dot11_qos_counter_t QoSCounterTable[NUM_TIDS];

    /* EDCA QAP Config Table */
    dot11_qap_edca_table_t QAPEDCATable[NUM_AC];
} mib_11e_t;

/* Structure for private MIB for WMM */
typedef struct
{
    BOOL_T UAPSD_ap;               /* U-APSD support in AP */
    UWORD8 ac_parameter_set_count; /* Parameter Set Count  */
} p_mib_wmm_t;

/*****************************************************************************/
/* External Global Variables                                                 */
/*****************************************************************************/

extern mib_11e_t   MIB_11E;
extern p_mib_wmm_t P_MIB_WMM;

/*****************************************************************************/
/* Extern Function Declaration                                               */
/*****************************************************************************/

extern void initialize_mac_mib_11e(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Functions for accessing standard 802.11e MIB */
INLINE TRUTH_VALUE_T mget_QoSOptionImplemented(void)
{
    return MIB_11E.dot11QoSOptionImplemented;
}

INLINE void mset_QoSOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11QoSOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_ImmediateBlockAckOptionImplemented(void)
{
    return MIB_11E.dot11ImmediateBlockAckOptionImplemented;
}

INLINE void mset_ImmediateBlockAckOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11ImmediateBlockAckOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_DelayedBlockAckOptionImplemented(void)
{
    return MIB_11E.dot11DelayedBlockAckOptionImplemented;
}

INLINE void mset_DelayedBlockAckOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11DelayedBlockAckOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_DirectOptionImplemented(void)
{
    return MIB_11E.dot11DirectOptionImplemented;
}

INLINE void mset_DirectOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11DirectOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_APSDOptionImplemented(void)
{
    return MIB_11E.dot11APSDOptionImplemented;
}

INLINE void mset_APSDOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11APSDOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_QAckOptionImplemented(void)
{
    return MIB_11E.dot11QAckOptionImplemented;
}

INLINE void mset_QAckOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11QAckOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_QBSSLoadOptionImplemented(void)
{
    return MIB_11E.dot11QBSSLoadOptionImplemented;
}

INLINE void mset_QBSSLoadOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11QBSSLoadOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_QueueRequestOptionImplented(void)
{
    return MIB_11E.dot11QueueRequestOptionImplented;
}

INLINE void mset_QueueRequestOptionImplented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11QueueRequestOptionImplented = inp;
}

INLINE TRUTH_VALUE_T mget_TXOPRequestOptionImplemented(void)
{
    return MIB_11E.dot11TXOPRequestOptionImplemented;
}

INLINE void mset_TXOPRequestOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11TXOPRequestOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_MoreDataAckOptionImplemented(void)
{
    return MIB_11E.dot11MoreDataAckOptionImplemented;
}

INLINE void mset_MoreDataAckOptionImplemented(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11MoreDataAckOptionImplemented = inp;
}

INLINE TRUTH_VALUE_T mget_AssociateInNQBSS(void)
{
    return MIB_11E.dot11AssociateInNQBSS;
}

INLINE void mset_AssociateInNQBSS(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11AssociateInNQBSS = inp;
}

INLINE TRUTH_VALUE_T mget_DLSAllowedInQBSS(void)
{
    return MIB_11E.dot11DLSAllowedInQBSS;
}

INLINE void mset_DLSAllowedInQBSS(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11DLSAllowedInQBSS = inp;
}

INLINE TRUTH_VALUE_T mget_DLSAllowed(void)
{
    return MIB_11E.dot11DLSAllowed;
}

INLINE void mset_DLSAllowed(TRUTH_VALUE_T inp)
{
    MIB_11E.dot11DLSAllowed = inp;
}

INLINE UWORD16 mget_CAPLimit(void)
{
    return MIB_11E.dot11CAPLimit;
}

INLINE void mset_CAPLimit(UWORD16 inp)
{
    MIB_11E.dot11CAPLimit = inp;
}

INLINE UWORD16 mget_HCCWmin(void)
{
    return MIB_11E.dot11HCCWmin;
}

INLINE void mset_HCCWmin(UWORD16 inp)
{
    MIB_11E.dot11HCCWmin = inp;
}

INLINE UWORD16 mget_HCCWmax(void)
{
    return MIB_11E.dot11HCCWmax;
}

INLINE void mset_HCCWmax(UWORD16 inp)
{
    MIB_11E.dot11HCCWmax = inp;
}

INLINE UWORD16 mget_HCCAIFSN(void)
{
    return MIB_11E.dot11HCCAIFSN;
}

INLINE void mset_HCCAIFSN(UWORD16 inp)
{
    MIB_11E.dot11HCCAIFSN = inp;
}

INLINE UWORD16 mget_ADDBAResponseTimeout(void)
{
    return MIB_11E.dot11ADDBAResponseTimeout;
}

INLINE void mset_ADDBAResponseTimeout(UWORD16 inp)
{
    MIB_11E.dot11ADDBAResponseTimeout = inp;
}

INLINE UWORD16 mget_ADDTSResponseTimeout(void)
{
    return MIB_11E.dot11ADDTSResponseTimeout;
}

INLINE void mset_ADDTSResponseTimeout(UWORD16 inp)
{
    MIB_11E.dot11ADDTSResponseTimeout = inp;
}

INLINE UWORD16 mget_ChannelUtilzationBeaconInterval(void)
{
    return MIB_11E.dot11ChannelUtilzationBeaconInterval;
}

INLINE void mset_ChannelUtilzationBeaconInterval(UWORD16 inp)
{
    MIB_11E.dot11ChannelUtilzationBeaconInterval = inp;
}

INLINE UWORD16 mget_ScheduleTimeout(void)
{
    return MIB_11E.dot11ScheduleTimeout;
}

INLINE void mset_ScheduleTimeout(UWORD16 inp)
{
    MIB_11E.dot11ScheduleTimeout = inp;
}

INLINE UWORD16 mget_DLSResponseTimeout(void)
{
    return MIB_11E.dot11DLSResponseTimeout;
}

INLINE void mset_DLSResponseTimeout(UWORD16 inp)
{
    MIB_11E.dot11DLSResponseTimeout = inp;
}

INLINE UWORD16 mget_QAPMissingAckRetryLimit(void)
{
    return MIB_11E.dot11QAPMissingAckRetryLimit;
}

INLINE void mset_QAPMissingAckRetryLimit(UWORD16 inp)
{
    MIB_11E.dot11QAPMissingAckRetryLimit = inp;
}

INLINE UWORD16 mget_EDCAAveragingPeriod(void)
{
    return MIB_11E.dot11EDCAAveragingPeriod;
}

INLINE void mset_EDCAAveragingPeriod(UWORD16 inp)
{
    MIB_11E.dot11EDCAAveragingPeriod = inp;
}

INLINE UWORD32 mget_QoSDiscardedFragmentCount(void)
{
    return MIB_11E.dot11QoSDiscardedFragmentCount;
}

INLINE void mset_QoSDiscardedFragmentCount(UWORD32 inp)
{
    MIB_11E.dot11QoSDiscardedFragmentCount = inp;
}

INLINE UWORD32 mget_AssociatedStationCount(void)
{
    return MIB_11E.dot11AssociatedStationCount;
}

INLINE void mset_AssociatedStationCount(UWORD32 inp)
{
    MIB_11E.dot11AssociatedStationCount = inp;
}

INLINE UWORD32 mget_QoSCFPollsReceivedCount(void)
{
    return MIB_11E.dot11QoSCFPollsReceivedCount;
}

INLINE void mset_QoSCFPollsReceivedCount(UWORD32 inp)
{
    MIB_11E.dot11QoSCFPollsReceivedCount = inp;
}

INLINE UWORD32 mget_QoSCFPollsUnusedCount(void)
{
    return MIB_11E.dot11QoSCFPollsUnusedCount;
}

INLINE void mset_QoSCFPollsUnusedCount(UWORD32 inp)
{
    MIB_11E.dot11QoSCFPollsUnusedCount = inp;
}

INLINE UWORD32 mget_QoSCFPollsUnusableCount(void)
{
    return MIB_11E.dot11QoSCFPollsUnusableCount;
}

INLINE void mset_QoSCFPollsUnusableCount(UWORD32 inp)
{
    MIB_11E.dot11QoSCFPollsUnusableCount = inp;
}

INLINE UWORD8 mget_EDCAParameterSetUpdateCount(void)
{
    return MIB_11E.dot11QoSEDCAParameterSetUpdateCount;
}

INLINE void mset_EDCAParameterSetUpdateCount(UWORD8 inp)
{
    MIB_11E.dot11QoSEDCAParameterSetUpdateCount = inp;
}

INLINE UWORD8 mget_EDCATableIndex(UWORD8 ac)
{
    return MIB_11E.EDCATable[ac].dot11EDCATableIndex;
}

INLINE void mset_EDCATableIndex(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.EDCATable[ac].dot11EDCATableIndex = inp;
}

INLINE UWORD32 mget_EDCATableCWmin(UWORD8 ac)
{
    return MIB_11E.EDCATable[ac].dot11EDCATableCWmin;
}

INLINE void mset_EDCATableCWmin(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.EDCATable[ac].dot11EDCATableCWmin = inp;
}

INLINE UWORD32 mget_EDCATableCWmax(UWORD8 ac)
{
    return MIB_11E.EDCATable[ac].dot11EDCATableCWmax;
}

INLINE void mset_EDCATableCWmax(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.EDCATable[ac].dot11EDCATableCWmax = inp;
}

INLINE UWORD32 mget_EDCATableAIFSN(UWORD8 ac)
{
    return MIB_11E.EDCATable[ac].dot11EDCATableAIFSN;
}

INLINE void mset_EDCATableAIFSN(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.EDCATable[ac].dot11EDCATableAIFSN = inp;
}

INLINE UWORD32 mget_EDCATableTXOPLimit(UWORD8 ac)
{
    return MIB_11E.EDCATable[ac].dot11EDCATableTXOPLimit;
}

INLINE void mset_EDCATableTXOPLimit(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.EDCATable[ac].dot11EDCATableTXOPLimit = inp;
}

INLINE UWORD32 mget_EDCATableMSDULifetime(UWORD8 ac)
{
    return MIB_11E.EDCATable[ac].dot11EDCATableMSDULifetime;
}

INLINE void mset_EDCATableMSDULifetime(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.EDCATable[ac].dot11EDCATableMSDULifetime = inp;
}

INLINE TRUTH_VALUE_T mget_EDCATableMandatory(UWORD8 ac)
{
    return MIB_11E.EDCATable[ac].dot11EDCATableMandatory;
}

INLINE void mset_EDCATableMandatory(TRUTH_VALUE_T inp, UWORD8 ac)
{
    MIB_11E.EDCATable[ac].dot11EDCATableMandatory = inp;
}

INLINE UWORD32 mget_QoSCountersIndex(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSCountersIndex;
}

INLINE void mset_QoSCountersIndex(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSCountersIndex = inp;
}

INLINE UWORD32 mget_QoSTransmittedFragmentCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSTransmittedFragmentCount;
}

INLINE void mset_QoSTransmittedFragmentCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSTransmittedFragmentCount = inp;
}

INLINE UWORD32 mget_QoSFailedCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSFailedCount;
}

INLINE void mset_QoSFailedCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSFailedCount = inp;
}

INLINE UWORD32 mget_QoSRetryCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSRetryCount;
}

INLINE void mset_QoSRetryCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSRetryCount = inp;
}

INLINE UWORD32 mget_QoSMultipleRetryCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSMultipleRetryCount;
}

INLINE void mset_QoSMultipleRetryCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSMultipleRetryCount = inp;
}

INLINE UWORD32 mget_QoSFrameDuplicateCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSFrameDuplicateCount;
}

INLINE void mset_QoSFrameDuplicateCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSFrameDuplicateCount = inp;
}

INLINE UWORD32 mget_QoSRTSSuccessCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSRTSSuccessCount;
}

INLINE void mset_QoSRTSSuccessCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSRTSSuccessCount = inp;
}

INLINE UWORD32 mget_QoSRTSFailureCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSRTSFailureCount;
}

INLINE void mset_QoSRTSFailureCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSRTSFailureCount = inp;
}

INLINE UWORD32 mget_QoSAckFailureCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSAckFailureCount;
}

INLINE void mset_QoSAckFailureCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSAckFailureCount = inp;
}

INLINE UWORD32 mget_QoSReceivedFragmentCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSReceivedFragmentCount;
}

INLINE void mset_QoSReceivedFragmentCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSReceivedFragmentCount = inp;
}

INLINE UWORD32 mget_QoSTransmittedFrameCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSTransmittedFrameCount;
}

INLINE void mset_QoSTransmittedFrameCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSTransmittedFrameCount = inp;
}

INLINE UWORD32 mget_QoSMPDUsReceivedCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSMPDUsReceivedCount;
}

INLINE void mset_QoSMPDUsReceivedCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSMPDUsReceivedCount = inp;
}

INLINE UWORD32 mget_QoSRetriesReceivedCount(UWORD8 tid)
{
    return MIB_11E.QoSCounterTable[tid].dot11QoSRetriesReceivedCount;
}

INLINE void mset_QoSRetriesReceivedCount(UWORD32 inp, UWORD8 tid)
{
    MIB_11E.QoSCounterTable[tid].dot11QoSRetriesReceivedCount = inp;
}

INLINE UWORD32 mget_QAPEDCATableIndex(UWORD8 ac)
{
    return MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableIndex;
}

INLINE void mset_QAPEDCATableIndex(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableIndex = inp;
}

INLINE UWORD32 mget_QAPEDCATableCWmin(UWORD8 ac)
{
    return MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableCWmin;
}

INLINE void mset_QAPEDCATableCWmin(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableCWmin = inp;
}

INLINE UWORD32 mget_QAPEDCATableCWmax(UWORD8 ac)
{
    return MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableCWmax;
}

INLINE void mset_QAPEDCATableCWmax(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableCWmax = inp;
}

INLINE UWORD32 mget_QAPEDCATableAIFSN(UWORD8 ac)
{
    return MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableAIFSN;
}

INLINE void mset_QAPEDCATableAIFSN(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableAIFSN = inp;
}

INLINE UWORD32 mget_QAPEDCATableTXOPLimit(UWORD8 ac)
{
    return MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableTXOPLimit;
}

INLINE void mset_QAPEDCATableTXOPLimit(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableTXOPLimit = inp;
}

INLINE UWORD32 mget_QAPEDCATableMSDULifetime(UWORD8 ac)
{
    return MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableMSDULifetime;
}

INLINE void mset_QAPEDCATableMSDULifetime(UWORD32 inp, UWORD8 ac)
{
    MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableMSDULifetime = inp;
}

INLINE TRUTH_VALUE_T mget_QAPEDCATableMandatory(UWORD8 ac)
{
    return MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableMandatory;
}

INLINE void mset_QAPEDCATableMandatory(TRUTH_VALUE_T inp, UWORD8 ac)
{
    MIB_11E.QAPEDCATable[ac].dot11QAPEDCATableMandatory = inp;
}

/* Functions for accessing WMM private MIB */
INLINE BOOL_T mget_UAPSD_ap(void)
{
    return P_MIB_WMM.UAPSD_ap;
}

INLINE void mset_UAPSD_ap(UWORD8 uap)
{
    /* Input Sanity Check */
    if(uap > 1)
        return;

    P_MIB_WMM.UAPSD_ap = (BOOL_T)uap;
}

INLINE void mincr_ac_parameter_set_count(void)
{
    P_MIB_WMM.ac_parameter_set_count =
        ((P_MIB_WMM.ac_parameter_set_count + 1) % 15);
}

INLINE UWORD8 mget_ac_parameter_set_count(void)
{
    return P_MIB_WMM.ac_parameter_set_count;
}

#endif /* MIB_11E_H */

#endif /* MAC_WMM */
