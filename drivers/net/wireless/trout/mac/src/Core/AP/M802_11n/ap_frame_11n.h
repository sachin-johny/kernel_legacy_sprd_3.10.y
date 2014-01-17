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
/*  File Name         : ap_frame_11n.h                                       */
/*                                                                           */
/*  Description       : This file contains the definitions and inline        */
/*                      functions for the setting/getting various fields of  */
/*                      the 11n MAC frames in AP mode of operation.          */
/*                                                                           */
/*  List of Functions : set_ht_capinfo_field_ap                              */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11N

#ifndef AP_FRAME_11N_H
#define AP_FRAME_11N_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "mib_11n.h"
#include "mib_802_11n.h"
#include "frame_11n.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 set_ht_opern_11n_ap(UWORD8 *data, UWORD16 index);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function updates the HT Capability Information Field for AP mode */
INLINE void set_ht_capinfo_field_ap(UWORD8 *data, TYPESUBTYPE_T frame_type)
{
    UWORD16 ht_capinfo = 0;

    /* HT Capability Info Field                                              */
    /* |-------------------------------------------------------------------| */
    /* |  LDPC  | Supp    |  SM   | Green- | Short  | Short  |  Tx  |  Rx  | */
    /* | Coding | Channel | Power | field  | GI for | GI for | STBC | STBC | */
    /* |  Cap   | Wth Set | Save  |        | 20 MHz | 40 MHz |      |      | */
    /* |-------------------------------------------------------------------| */
    /* |   B0   |    B1   |B2   B3|   B4   |   B5   |    B6  |  B7  |B8  B9| */
    /* |-------------------------------------------------------------------| */
    /*                                                                       */
    /*                                                                       */
    /* |------------------------------------------------------------------|  */
    /* |    HT     |  Max   | DSS/CCK | PSMP    | 40 MHz     | L-SIG TXOP |  */
    /* |  Delayed  | AMSDU  | Mode in | Support | Intolerant | Protection |  */
    /* | Block-Ack | Length | 40MHz   |         |            | Support    |  */
    /* |------------------------------------------------------------------|  */
    /* |    B10    |   B11  |   B12   |   B13   |    B14     |    B15     |  */
    /* |------------------------------------------------------------------|  */

    /* Indicates support for receiving LDPC coded packets */
    if(mget_LDPCCodingOptionImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT0;
    }

    /* Indicates which channel widths the STA supports */
    if(mget_FortyMHzOperationImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT1;
    }

    /* Indicates the Spatial Multplexing (SM) Power Save mode */
    if(STATIC_MODE == mget_MIMOPowerSave())
        ht_capinfo |= STATIC_SMPS;
    else if(DYNAMIC_MODE == mget_MIMOPowerSave())
        ht_capinfo |= DYNAMIC_SMPS;
    else if(MIMO_MODE == mget_MIMOPowerSave())
        ht_capinfo |= SM_ENABLED;

    /* Indicates support for reception of PPDUs with HT Greenfield format */
    if(mget_GreenfieldOptionImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT4;
    }

    /* Indicates Short GI support for the reception of 20 MHz packets */
    if(mget_ShortGIOptionInTwentyImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT5;
    }

    /* Indicates Short GI support for the reception of 40 MHz packets */
    if(mget_ShortGIOptionInFortyImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT6;
    }

    /* Indicates support for the transmission of PPDUs using STBC */
    if(mget_TxSTBCOptionImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT7;
    }

    /* Indicates support for the reception of PPDUs using STBC */
    if(mget_RxSTBCOptionImplemented() == TV_TRUE)
    {
        UWORD16 num_spatial_streams = mget_NumberOfSpatialStreamsImplemented();

        /* Max of only 3 Spatial streams can be reported through this subfield*/
        if(4 == num_spatial_streams)
            num_spatial_streams--;

        ht_capinfo |= (num_spatial_streams << 8);
    }

    /* Indicates support for HT-delayed BlockAck operation */
    if(mget_DelayedBlockAckOptionImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT10;
    }

    /*Indicates maximum AMSDU length.*/
    if(mget_MaxAMSDULength() == 7935)
    {
        ht_capinfo |= BIT11;
    }

    /* Indicates use of DSSS/CCK mode in 40MHz capable BSS in 20/40 MHz mode */
    /* Directly derive this from 11g operating mode */
    if((get_running_mode() != G_ONLY_MODE) &&
       (mget_dsss_cck_40mhz_mode() == TV_TRUE))
    {
        ht_capinfo |= BIT12;
    }

    /* BIT-13 is Reserved */

    if(get_current_start_freq() == RC_START_FREQ_2)
    {
        /* Indicates whether 40 MHz transmissions are prohibited by all      */
        /* members of the BSS. Only to be set by user e.g.if a BT is present */
        /* This is valid only for a 2G4 STA                                  */
        if(mget_FortyMHzIntolerant() == TV_TRUE)
        {
            ht_capinfo |= BIT14;
        }
    }

    /* Indicates support for the L-SIG TXOP protection */
    if(mget_LsigTxopProtectionOptionImplemented() == TV_TRUE)
    {
        ht_capinfo |= BIT15;
    }

    data[0] = ht_capinfo & 0x00FF;
    data[1] = (ht_capinfo & 0xFF00) >> 8;
}

#endif /* AP_FRAME_11N_H */
#endif /* MAC_802_11N */
#endif /* BSS_ACCESS_POINT_MODE */
