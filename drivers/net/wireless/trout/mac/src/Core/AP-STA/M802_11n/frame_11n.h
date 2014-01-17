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
/*  File Name         : frame_11n.h                                          */
/*                                                                           */
/*  Description       : This file contains the definitions and inline        */
/*                      functions for the setting/getting various fields of  */
/*                      the MAC frames                                       */
/*                                                                           */
/*  List of Functions : set_amsdu_present_field                              */
/*                      is_amsdu_frame                                       */
/*                      set_supp_mcs_set                                     */
/*                      set_ht_extcap                                        */
/*                      set_txbf_cap                                         */
/*                      set_asel_cap                                         */
/*                      set_ht_opern                                         */
/*                      is_ht_frame                                          */
/*                      set_ht_control                                       */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef MAC_802_11N

#ifndef FRAME_11N_H
#define FRAME_11N_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "mib_11n.h"
#include "mib_802_11n.h"
#include "blockack.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define HT_CTRL_FIELD_OFFSET    26
#define HT_MAC_HDR_LEN          30     /* QoS Present. Non-ESS(No Address4) */
#define HT_CTRL_FIELD_LEN        4     /* Length of HT Control Field        */
#define HT_CAPINFO_FIELD_LEN     2
#define AMPDU_PARAM_LEN          1
#define MCS_SET_FIELD_LEN       16
#define HT_EXTCAP_FIELD_LEN      2
#define TX_BF_FIELD_LEN          4
#define ASEL_FIELD_LEN           1
#define SEC_CHOFF_FIELD_LEN      1
#define BSS_MSHIP_SEL_VAL      127

#define ADDBA_REQ_LEN        (MAC_HDR_LEN + 1 + 1 + 1 + 2 + 2 + 2 + FCS_LEN)
#define ADDBA_RSP_LEN        (MAC_HDR_LEN + 1 + 1 + 1 + 2 + 2 + 2 + FCS_LEN)
#define DELBA_LEN            (MAC_HDR_LEN + 1 + 1 + 2 + 2 + FCS_LEN)
#define BLOCKACK_RSP_LEN     (150)
#define BLOCKACK_REQ_MAX_LEN (86) /* For Multi-TID BlockAckReq  */
#define BLOCKACK_REQ_MIN_LEN (24) /* For Single-TID BlockAckReq */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {STATIC_SMPS  = 0x00,
              DYNAMIC_SMPS = 0x04,
              SM_ENABLED   = 0x0C
} HT_INFO_SMPS_FIELD_T;

typedef enum {STATIC_MODE   = 1,
              DYNAMIC_MODE  = 2,
              MIMO_MODE     = 3,
              NUM_MIMO_POWER_SAVE_MODE
} MIMO_POWER_SAVE_MODE_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 set_ht_cap_11n(UWORD8 *data, UWORD16 index,
                              TYPESUBTYPE_T frame_type);
extern UWORD8 set_ht_opern_11n(UWORD8 *data, UWORD16 index);
extern UWORD8 set_sec_choff_11n(UWORD8 *data, UWORD16 index,
                                TYPESUBTYPE_T frame_type);
extern UWORD16 prepare_addba_req(UWORD8 *data, ba_tx_struct_t *ba, UWORD8 tid);
extern UWORD16 prepare_addba_rsp(UWORD8 *data, ba_rx_struct_t *ba, UWORD8 tid,
                                 UWORD8 status);
extern UWORD16 prepare_delba_req(UWORD8 *data, UWORD8 *addr, UWORD8 tid,
                                 UWORD8 initiator, UWORD8 reason);
extern UWORD8 prepare_blockack_req(UWORD8 *data, ba_tx_struct_t *ba,
                                   UWORD8 tid);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function sets the A-MSDU present bit (BIT7 in the QoS Control field) */
/* in the MAC header.                                                        */
INLINE void set_amsdu_present_bit(UWORD8 *mac_header)
{
    mac_header[QOS_CTRL_FIELD_OFFSET] |= BIT7;
}

/* This function resets the A-MSDU present bit (BIT7 in the QoS Control field) */
/* in the MAC header.                                                         */
INLINE void reset_amsdu_present_bit(UWORD8 *mac_header)
{
    mac_header[QOS_CTRL_FIELD_OFFSET] &= ~BIT7;
}

/* This function checks if the A-MSDU present bit is set in the QoS control  */
/* field of the given MAC header.                                            */
INLINE UWORD8 is_amsdu_bit_set(UWORD8 *mac_header)
{
    if(mac_header[QOS_CTRL_FIELD_OFFSET] & BIT7)
        return 1;

    return 0;
}

/* This function sets the AMPDU Parameters field with the required parameter */
/* values from MIB.                                                          */
INLINE void set_ampdu_params(UWORD8 *data)
{
    /* AMPDU Parameters Field                                              */
    /* |-----------------------------------------------------------------| */
    /* | Maximum Rx AMPDU Factor | Minimum MPDU Start Spacing | Reserved | */
    /* |-----------------------------------------------------------------| */
    /* | B0                   B1 | B2                      B4 | B5     B7| */
    /* |-----------------------------------------------------------------| */

    data[0]  = mget_MaxRxAMPDUFactor();
    data[0] |= mget_MinimumMPDUStartSpacing() << 2;
}

/* This function sets the Supported MCS Set field with the required values   */
/* from MIB.                                                                 */
INLINE void set_supp_mcs_set(UWORD8 *data)
{
    UWORD8    txmcs_params  = 0;
    UWORD16   index         = 0;
    UWORD16   max_supp_rate = 0;
    ht_mcs_t  rxmcs_set     = {{0}, 0};

    /* Supported MCS Set  Field                                                */
    /* |-------------------------------------------------------------------|   */
    /* | Rx MCS Bitmask | Reserved | Highest Supp | Reserved |   Tx MCS    |   */
    /* |                |          | Data Rate    |          | Set Defined |   */
    /* |-------------------------------------------------------------------|   */
    /* | B0         B76 | B77  B79 | B80      B89 | B90  B95 |    B96      |   */
    /* |-------------------------------------------------------------------|   */
    /* | Tx Rx MCS Set  | Tx Max Number     |   Tx Unequal     | Reserved  |   */
    /* |  Not Equal     | Spat Stream Supp  | Modulation Supp  |           |   */
    /* |-------------------------------------------------------------------|   */
    /* |      B97       | B98           B99 |       B100       | B101 B127 |   */
    /* |-------------------------------------------------------------------|   */

    /***************************************************************************/
    /***************************************************************************/
    /* 11NTBD - needs verification, should the bitmask be created here?        */
    /***************************************************************************/
    /***************************************************************************/

    /* When a bit in Rx MCS Bitmask field is set to 1 the corresponding  MCS   */
    /* is supported by the STA on receive.                                     */
    rxmcs_set = mget_SupportedMCSRxValue();

    /* Reset the Rx MCS bitmask (10 bytes) */
    memcpy(data, rxmcs_set.mcs_bmp, MCS_BITMASK_LEN);

    /* Update the index with the length of the Rx MCS bitmask */
    index = MCS_BITMASK_LEN;

    /* Set the Highest Supported Data Rate that the STA is able to receive */
    max_supp_rate = mget_HighestSupportedDataRate();

    data[index++] = max_supp_rate & 0x00FF;
    data[index++] = (max_supp_rate & 0x0300) >> 8;

    /* Based on MIB parameters update the Tx MCS field value */
    if(mget_TxMCSSetDefined() == TV_TRUE)
    {
        /* Set the bit 0 to indicate if Tx MCS Set is defined */
        txmcs_params |= BIT0;

        if(mget_TxRxMCSSetNotEqual() == TV_TRUE)
        {
            /* Set the bit 1 to indicate if Tx Supported MCS Set is equal to */
            /* the Rx Supported MCS Set.                                     */
            txmcs_params |= BIT1;

            /* Set the Tx Maximum Number of Spatial Streams Supported */
            txmcs_params |= (mget_TxMaximumNumberSpatialStreamsSupported()
                             << 2);

            if(mget_TxUnequalModulationSupported() == TV_TRUE)
            {
                /* Set the bit 4 if Tx Unequal Modulation is supported */
                txmcs_params |= BIT4;
            }
        }
    }

    /* Set the Tx MCS parameter in the frame */
    data[index++] = txmcs_params;

    /* Reset the 'Reserved' field to all 0s */
    data[index++] = 0;
    data[index++] = 0;
    data[index++] = 0;
}

/* This function sets the HT Extended Capabilites Field in the given frame.*/
INLINE void set_ht_extcap(UWORD8 *data)
{
    UWORD16 htextcap_field = 0;

    /* HT Extended Capabilities Field                                      */
    /* |-----------------------------------------------------------------| */
    /* | PCO | PCO Trans | Reserved | MCS  |  +HTC   |  RD    | Reserved | */
    /* |     |   Time    |          | Fdbk | Support | Resp   |          | */
    /* |-----------------------------------------------------------------| */
    /* | B0  | B1     B2 | B3    B7 | B8 B9|   B10   |  B11   | B12  B15 | */
    /* |-----------------------------------------------------------------| */

    if(mget_PCOOptionImplemented() == TV_TRUE)
    {
        /* Set bit 0 if Phased Coexistence Operation (PCO) is supported */
        htextcap_field |= BIT0;
    }

    /* Set the transition time for STA to switch between 20 MHz and 40 MHz   */
    /* channel width.                                                        */
    htextcap_field |= (mget_TransitionTime() << 1);

    /* Set the MCS feedback option implemented */
    htextcap_field |= (mget_MCSFeedbackOptionImplemented() << 8);

    if(mget_HTControlFieldSupported() == TV_TRUE)
    {
        /* Set bit 10 if High Throughput Control field is supported */
        htextcap_field |= BIT10;
    }

    if(mget_RDResponderOptionImplemented() == TV_TRUE)
    {
        /* Set bit 11 if the STA can act as a RD responser */
        htextcap_field |= BIT11;
    }

    /* Set the bytes in the frame with the HT extended capability value */
    data[0] = htextcap_field & 0x00FF;
    data[1] = (htextcap_field & 0xFF00) >> 8;

}


/* This function sets the Tx Beamforming Capability Field in the given frame.*/
INLINE void set_txbf_cap(UWORD8 *data)
{
    UWORD32 txbf_cap = 0;

    /* Transmit Beamforming Capability Field                                 */
    /* |-------------------------------------------------------------------| */
    /* | Implicit | Rx Stagg | Tx Stagg  | Rx NDP   | Tx NDP   | Implicit  | */
    /* | TxBF Rx  | Sounding | Sounding  | Capable  | Capable  |    TxBF   | */
    /* | Capable  | Capable  | Capable   |          |          |  Capable  | */
    /* |-------------------------------------------------------------------| */
    /* |    B0    |     B1   |    B2     |   B3     |   B4     |    B5     | */
    /* |-------------------------------------------------------------------| */
    /* |             | Explicit | Explicit Non- |   Explicit    | Explicit | */
    /* | Calibration | CSI TxBF | Compr BF Fdbk | Compr BF Fdbk | TxBF CSI | */
    /* |             | Capable  | Matrix Cap.   |   Matrix Cap. | Feedback | */
    /* |-------------------------------------------------------------------| */
    /* | B6       B7 |   B8     |       B9      |       B10     | B11  B12 | */
    /* |-------------------------------------------------------------------| */
    /* | Explicit Non- | Explicit | Minimal  | CSI Num of | Non-Compr BF   | */
    /* | Compr BF      | Compr BF | Grouping | Beamformer | Fdbk Matrix Num| */
    /* | Fdbk Mat      | Fdbk Mat |          | Ants Supp  | of BF Ants Supp| */
    /* |-------------------------------------------------------------------| */
    /* | B13       B14 | B15  B16 | B17  B18 | B19    B20 | B21        B22 | */
    /* |-------------------------------------------------------------------| */
    /* |    Compr BF       | CSI Max Num of     |   Channel     |          | */
    /* | Fdbk Matrix Num   | Rows Beamformer    | Estimation    | Reserved | */
    /* | of BF Ants Supp   | Supported          | Capability    |          | */
    /* |-------------------------------------------------------------------| */
    /* | B23           B24 | B25            B26 | B27       B28 | B29  B31 | */
    /* |-------------------------------------------------------------------| */

    /*Indicates whether or not this STA can receive TxBF steered frames using*/
    /*implicit feedback                                                      */
    /*************************************************************************/
    /*************************************************************************/
    /* 11NTBD - No MIB exists, not clear what needs to be set                */
    /*************************************************************************/
    /*************************************************************************/

    /* Indicates if this STA can receive staggered sounding frames.*/
    if(mget_ReceiveStaggerSoundingOptionImplemented() == TV_TRUE)
    {
        txbf_cap |= BIT1;
    }

    /* Indicates if this STA can transmit staggered sounding frames.*/
    if(mget_TransmitStaggerSoundingOptionImplemented() == TV_TRUE)
    {
        txbf_cap |= BIT2;
    }

    /* Indicates if this receiver can interpret incoming Null Data Packets   */
    /* as sounding frames.                                                   */
    if(mget_ReceiveNDPOptionImplemented() == TV_TRUE)
    {
        txbf_cap |= BIT3;
    }

    /* Indicates if this STA can transmit Null Data Packets as sounding      */
    /* frames.                                                               */
    if(mget_TransmitNDPOptionImplemented() == TV_TRUE)
    {
        txbf_cap |= BIT4;
    }

    /* Indicates if this STA can apply Implicit transmit beamforming */
    if(mget_ImplicitTxBFOptionImplemented() == TV_TRUE)
    {
        txbf_cap |= BIT5;
    }

    /* Indicates if the STA can participate in a calibration procedure       */
    /* initiated by another STA that is capable of generating an immediate   */
    /* response Sounding PPDU, and can provide a MIMO CSI Matrices Report in */
    /* response to the receipt of a Sounding PPDU.                           */
    txbf_cap |= ((UWORD32)mget_CalibrationOptionImplemented()) << 6;

    /* Indicates if this STA can apply transmit beamforming using CSI        */
    /* explicit feedback in its transmission.                                */
    if(mget_ExplicitCSITxBFOptionImplemented() == TV_TRUE)
    {
        txbf_cap |= BIT8;
    }

    /* Indicates if this STA can apply transmit beamforming using            */
    /* non-compressed beamforming feedback matrix explicit feedback in its   */
    /* transmission.                                                         */
    if(mget_ExplicitNonCompressedbeamformingMatrixOptionImplemented()
      == TV_TRUE)
    {
        txbf_cap |= BIT9;
    }

    /* Indicates if this STA can apply transmit beamforming using compressed */
    /* beamforming feedback matrix explicit feedback in its tranmission.     */
    /*************************************************************************/
    /*************************************************************************/
    /* 11NTBD - No MIB exists, not clear what needs to be set                */
    /*************************************************************************/
    /*************************************************************************/

    /* Indicates if this receiver can return CSI explicit feedback */
    txbf_cap |= ((((UWORD32)mget_ExplicitBFCSIFeedbackOptionImplemented())
                  & 0x3) << 11);

    /* Indicates if this receiver can return non-compressed beamforming      */
    /* feedback matrix explicit feedback.                                    */
    txbf_cap |= ((((UWORD32)
       mget_ExplicitNonCompressedbeamformingMatrixFeedbackOptionImplemented())
                    & 0x3) << 13);

    /* Indicates if this STA can apply transmit beamforming using explicit   */
    /* compressed beamforming feedback matrix.                               */
    txbf_cap |= ((((UWORD32)
       mget_ExplicitCompressedbeamformingMatrixFeedbackOptionImplemented())
                    & 0x3) << 15);

    /*************************************************************************/
    /*************************************************************************/
    /* 11NTBD - For the above 3, MIB value range is different from subfield  */
    /* range. Also value 3 indicates different things in the two cases.      */
    /*************************************************************************/
    /*************************************************************************/

    /* Indicates the minimal grouping used for explicit feedback reports */
    /*************************************************************************/
    /*************************************************************************/
    /* 11NTBD - No MIB exists, not clear what needs to be set                */
    /*************************************************************************/
    /*************************************************************************/

    /* Indicates the maximum number of beamformer antennas the beamformee    */
    /* can support when CSI feedback is required.                            */
    txbf_cap |= (((UWORD32)mget_NumberBeamFormingCSISupportAntenna()) << 19);

    /* Indicates the maximum number of beamformer antennas the beamformee    */
    /* can support when non-compressed beamforming feedback matrix is        */
    /* required                                                              */
    txbf_cap |=
        (((UWORD32)mget_NumberNonCompressedbeamformingMatrixSupportAntenna())
        << 21);

    /* Indicates the maximum number of beamformer antennas the beamformee   */
    /* can support when compressed beamforming feedback matrix is required  */
    txbf_cap |=
        (((UWORD32)mget_NumberCompressedbeamformingMatrixSupportAntenna())
        << 23);

    /* Indicates the maximum number of rows of CSI explicit feedback from    */
    /* beamformee that the beamformer can support when CSI feedback is       */
    /* required                                                              */
    /*************************************************************************/
    /*************************************************************************/
    /* 11NTBD - No MIB exists, not clear what needs to be set                */
    /*************************************************************************/
    /*************************************************************************/


    /* Indicates maximum number of space time streams (columns of the MIMO   */
    /* channel matrix) for which channel dimensions can be simultaneously    */
    /* estimated. When staggered sounding is supported this limit applies    */
    /* independently to both the data portion and to the extension portion   */
    /* of the long training fields.                                          */
    /*************************************************************************/
    /*************************************************************************/
    /* 11NTBD - No MIB exists, not clear what needs to be set                */
    /*************************************************************************/
    /*************************************************************************/


    /* Set the bytes in the frame with the Tx Beamforming value */
    data[0] = (txbf_cap & 0x000000FF);
    data[1] = (txbf_cap & 0x0000FF00) >> 8;
    data[2] = (txbf_cap & 0x00FF0000) >> 16;
    data[3] = (txbf_cap & 0xFF000000) >> 24;

}

/* This function sets the Antenna Selection Capabilities Field in the given */
/* frame.                                                                   */
INLINE void set_asel_cap(UWORD8 *data)
{
    /* First initialize the field to 0 */
    data[0] = 0;

    /* Antenna Selection Capability Field                                    */
    /* |-------------------------------------------------------------------| */
    /* |  Antenna  | Explicit CSI  | Antenna Indices | Explicit | Antenna  | */
    /* | Selection | Fdbk based TX | Fdbk based TX   | CSI Fdbk | Indices  | */
    /* |  Capable  | ASEL Capable  | ASEL Capable    | Capable  | Fdbk Cap.| */
    /* |-------------------------------------------------------------------| */
    /* |    B0     |     B1        |      B2         |    B3    |    B4    | */
    /* |-------------------------------------------------------------------| */
    /*                                                                       */
    /* |------------------------------------|                                */
    /* |  RX ASEL |   Transmit   |          |                                */
    /* |  Capable |   Sounding   | Reserved |                                */
    /* |          | PPDU Capable |          |                                */
    /* |------------------------------------|                                */
    /* |    B5    |     B6       |    B7    |                                */
    /* |------------------------------------|                                */

    /* Indicates whether or not this STA supports Antenna Selection */
    if(mget_AntennaSelectionOptionImplemented() == TV_TRUE)
    {
        data[0] |= BIT0;
    }

    /* Indicates whether or not this STA has TX ASEL capability based */
    /* on explicit CSI feedback                                       */
    if(mget_TransmitExplicitCSIFeedbackASOptionImplemented() == TV_TRUE)
    {
        data[0] |= BIT1;
    }

    /* Indicates whether or not this STA has TX ASEL capability based */
    /* on antenna indices feedback.                                   */
    if(mget_TransmitIndicesFeedbackASOptionImplemented() == TV_TRUE)
    {
        data[0] |= BIT2;
    }

    /* Indicates whether or not this STA can compute CSI and feedback */
    /* in support of Antenna Selection.                               */
    if(mget_ExplicitCSIFeedbackASOptionImplemented() == TV_TRUE)
    {
        data[0] |= BIT3;
    }

    /* Indicates whether or not this STA can conduct antenna indices */
    /* selection computation and feedback the results in support of  */
    /* Antenna Selection.                                            */
    if(mget_TransmitIndicesComputationFeedbackASOptionImplemented() == TV_TRUE)
    {
        data[0] |= BIT4;
    }

    /* Indicates whether or not this STA has RX Antenna Selection capability */
    if(mget_ReceiveAntennaSelectionOptionImplemented() == TV_TRUE)
    {
        data[0] |= BIT5;
    }

    /* Indicates whether or not this STA can transmit sounding PPDUs for */
    /* Antenna Selection training per request.                           */
    if(TransmitSoundingPPDUOptionImplemented() == TV_TRUE)
    {
        data[0] |= BIT6;
    }

}

/* This function sets the HT Operation Field in the given frame */
INLINE void set_ht_opern(UWORD8 *data)
{
    UWORD16 htinfo = 0;
    UWORD32 primary_channel;

/* HT Information Field                                                   */
/* |--------------------------------------------------------------------| */
/* | Primary | Seconday  | STA Ch | RIFS | PSMP STAs | Service Interval | */
/* | Channel | Ch Offset | Width  | Mode | Only      | Granularity      | */
/* |--------------------------------------------------------------------| */
/* |    1    | B0     B1 |   B2   |  B3  |    B4     | B5            B7 | */
/* |--------------------------------------------------------------------| */
/*                                                                        */
/* |---------------------------------------------------------------|      */
/* | Operating | Non-GF STAs | Transmit  | OBSS Non-HT  | Reserved |      */
/* |    Mode   |   Present   | Burst Len | STAs Present |          |      */
/* |---------------------------------------------------------------|      */
/* | B0     B1 |     B2      |    B3     |     B4       | B5   B15 |      */
/* |---------------------------------------------------------------|      */
/*                                                                        */
/* |-------------------------------------------------------------|        */
/* | Reserved |  Dual  |  Dual CTS  | Seconday | LSIG TXOP Protn |        */
/* |          | Beacon | Protection |  Beacon  | Full Support    |        */
/* |-------------------------------------------------------------|        */
/* | B0    B5 |   B6   |     B7     |     B8   |       B9        |        */
/* |-------------------------------------------------------------|        */
/*                                                                        */
/* |---------------------------------------|                              */
/* |  PCO   |  PCO  | Reserved | Basic MCS |                              */
/* | Active | Phase |          |    Set    |                              */
/* |---------------------------------------|                              */
/* |  B10   |  B11  | B12  B15 |    16     |                              */
/* |---------------------------------------|                              */


    /* Indicates the channel number of the primary channel */
    primary_channel = mget_CurrentPrimaryChannel();
    data[0]         = (UWORD8)primary_channel;

    /* Indicates the offset of the secondary channel relative to the */
    /* primary channel.                                              */
    if(mget_FortyMHzOperationEnabled() == TV_TRUE)
    {
	// 20120830 caisf masked, merged ittiam mac v1.3 code
	#if 0
        if(mget_CurrentSecondaryChannel() > primary_channel)
            htinfo |= 1;
        else
            htinfo |= 3;
	#endif
    }
    /*************************************************************************/
    /*Implementation Pending. (Don't Know how to set)                        */
    /*************************************************************************/
    /* Defines the channel widths that may be used to transmit to the STA    */


    /* Indicates whether use of RIFS is permitted within the BSS. */
    if(mget_RIFSMode() == TV_TRUE)
    {
        htinfo |= BIT3;
    }

    /* Indicates whether an AP accepts Association requests only from PSMP */
    /* capable STAs.                                                       */
    if(mget_PSMPControlledAccess() == TV_TRUE)
    {
        htinfo |= BIT4;
    }

    /* Duration of the shortest Service Interval. Used for scheduled PSMP only */
    htinfo |= (mget_ServiceIntervalGranularity() << 5);

    data[1] = (UWORD8)htinfo;

    htinfo = 0;

    /* Indicates the operating mode of the BSS from which protection */
    /* requirements of HT transmissions may be determined.           */
    htinfo |= mget_HTOperatingMode();

    /* Indicates if any HT STAs that are not Greenfield capable have  */
    /* associated. Determines when a non-AP STA should use greenfield */
    /* protection.                                                    */
    if(mget_NonGFEntitiesPresent() == TV_TRUE)
    {
        htinfo |= BIT2;
    }

    /*************************************************************************/
    /*Implementation Pending. (Don't Know how to set)                        */
    /*************************************************************************/
    /* Indicates whether the duration of a transmit burst is limited. For    */
    /* this purpose, the transmit burst is a sequence of one or more PPDUs   */
    /* that are either HT greenfield format, or are preceded by an IFS less  */
    /* than SIFS.                                                            */


    /*************************************************************************/
    /*Implementation Pending. (Don't Know how to set)                        */
    /*************************************************************************/
    /*Indicates if the use of protection for non-HT STAs by overlapping BSSs */
    /*is determined to be desirable.                                         */


    data[2] = htinfo & 0x00FF;
    data[3] = (htinfo & 0xFF00) >> 8;

    htinfo = 0;
    /*************************************************************************/
    /*Implementation Pending. (Don't Know how to set)                        */
    /*************************************************************************/
    /* Indicates whether the AP transmits a a secondary beacon               */


    /*Dual CTS Protection is used by the AP to set a NAV at STAs that do not*/
    /*support STBC and at STAs that can associate solely through the        */
    /*secondary (STBC frame) beacon.                                        */
    if(mget_DualCTSProtection() == TV_TRUE)
    {
        htinfo |= BIT7;
    }

    /*************************************************************************/
    /*Implementation Pending. (Don't Know how to set)                        */
    /*************************************************************************/
    /* Indicates whether the beacon containing this element is a primary or */
    /* a secondary beacon. The secondary beacon has half a beacon period    */
    /* shift relative to the primary beacon.                                */


    /* Indicates whether all HT STA in the BSS support L-SIG TXOP Protection */
    if(mget_LSigTxopFullProtectionEnabled() == TV_TRUE)
    {
        htinfo |= BIT9;
    }

    /* Indicates whether PCO is active in the BSS */
    if(mget_PCOActivated() == TV_TRUE)
    {
        htinfo |= BIT10;
    }

    /*************************************************************************/
    /*Implementation Pending. (Don't Know how to set)                        */
    /*************************************************************************/
    /* Indicates the PCO phase of operation.                                */
    /* Defined only in a Beacon and Probe Response frames when PCO Active   */
    /* is 1. Otherwise reserved.                                            */



    data[4] = htinfo & 0x00FF;
    data[5] = (htinfo & 0xFF00) >> 8;

    /*************************************************************************/
    /*Implementation Pending. (Don't Know how to set)                        */
    /*************************************************************************/
    /*Indicates the MCS values that are supported by all HT STAs in the BSS. */
}

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
/* This function sets the Secondary Channel Offset Field */
INLINE void set_sec_choff_field(UWORD8 *data)
{
    UWORD32 primary_channel   = 0;
    UWORD32 secondary_channel = 0;

    if(mget_FortyMHzOperationEnabled() == TV_FALSE)
    {
        /* Secondary Channel is not present */
        data[0] = 0;
    }
    else
    {
        primary_channel   = mget_CurrentPrimaryChannel();
        secondary_channel = mget_CurrentSecondaryChannel();

        /* Compare the position of the seconday channel relative to that of */
        /* the primary channel                                              */
        if(secondary_channel > primary_channel)
            data[0] = 1;
        else
            data[0] = 3;
    }
}
#else
/* This function sets the Secondary Channel Offset Field */
INLINE void set_sec_choff_field(UWORD8 *data)
{

    if(mget_FortyMHzOperationEnabled() == TV_FALSE)
    {
        /* Secondary Channel is not present */
        data[0] = 0;
    }
    else
    {
    }
}
#endif

/* This function sets the HT control field and indicates its presence by */
/* setting the order field in the Frame Control Field.                   */
INLINE UWORD8 set_ht_control(UWORD8 *header, UWORD8 index)
{
    /* No HT control fields are presently set */
    header[index]     = 0;
    header[index + 1] = 0;
    header[index + 2] = 0;
    header[index + 3] = 0;

    /* Indicate the presence of the HT Control field by seting the order */
    /* field.                                                            */
    set_order_bit(header, 1);

    return HT_CTRL_FIELD_LEN;
}

/* This function check whether the MAC header contains HT control field */
INLINE BOOL_T is_ht_frame(UWORD8 *header)
{
    if((BTRUE == is_qos_bit_set(header)) && (1 == get_order_bit(header)))
        return BTRUE;

    return BFALSE;
}

/* This function checks if the packet is an ADDBA Frame and extracts the  */
/* TID and Start-Sequence number if it is one.                            */
INLINE BOOL_T is_addba_frame(UWORD8 *msa, UWORD8 *tid, UWORD16 *seq_num)
{

    UWORD8 *data   = NULL;

    /* Management frame */
    if((msa[0] & 0xFC) == 0xD0)
    {
        data = msa + MAC_HDR_LEN;
        if((data[0] == 3) && (data[1] == 0))	//ADDBA Request!
        {
            *seq_num = (data[7] >> 4) | (data[8] << 4);
            *tid     = (data[3] & 0x3C) >> 2;
            return BTRUE;
        }
    }

    return BFALSE;
}

#endif /*FRAME_11N_H*/
#endif /*MAC_802_11N*/
