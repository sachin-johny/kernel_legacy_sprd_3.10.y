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
/*  File Name         : frame_11i.h                                          */
/*                                                                           */
/*  Description       : This file contains the definitions and inline        */
/*                      functions for the setting/getting various fields of  */
/*                      the MAC frames in 802.11i standard.                  */
/*                                                                           */
/*  List of Functions : check_oui                                            */
/*                      append_11i_param                                     */
/*                      adjust_11i_headers                                   */
/*                      copy_11i_trailer                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef FRAME_802_11I_H
#define FRAME_802_11I_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "buff_desc.h"
#include "management_11i.h"
#include "mib_11i.h"
#include "rsna_km.h"
#include "tkip.h"

/*****************************************************************************/
/* Inline Function Declarations                                              */
/*****************************************************************************/

/* This function checks if the received OUI and the configured OUI are same  */
/* Returns TRUE is the OUI matches ; FALSE otherwise                         */
INLINE BOOL_T check_oui(UWORD8* frame, UWORD8 *oui)
{
    if((frame[0] == oui[0]) && (frame[1] == oui[1]) && (frame[2] == oui[2]))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* Append 11i parameters to the out going packet */
INLINE void append_11i_param(CIPHER_T ct, wlan_tx_req_t *wlan_tx_req,
                UWORD8 *tx_info, BOOL_T is_qos)
{
    /* If RSNA is enabled and Cipher type is TKIP add the TKIP MIC */
    if((mget_RSNAEnabled() == TV_TRUE) && (ct == TKIP))
    {
        /* Add the TKIP Mic to the tranmission packet */
        insert_tkip_mic(wlan_tx_req);
    }

    return;
}

/* Adjust length and data offset of the 11i protected received packet */
INLINE void adjust_11i_headers(UWORD8 *msa, CIPHER_T ct,
                                    UWORD16* rx_len, UWORD8* offset)
{
    /* Check if the RSNA is enabled, if RSNA is enabled adjust the */
    /* length and offset according to the encryption               */
    if((mget_RSNAEnabled() == TV_TRUE) && (ct == CCMP))
    {
        /* Data is offset by 8 bytes due the to CCMP/TKIP header */
        *offset = CCMP_HDR_LEN;
        /* Data length is currently passed inclusive of H/W MIC  */
        *rx_len -= CCMP_HDR_LEN + CCMP_MIC_LEN;
    }
    else if((mget_RSNAEnabled() == TV_TRUE) && (ct == TKIP))
    {
        /* Data is offset by 8 bytes due the to CCMP/TKIP header */
        *offset = TKIP_HDR_LEN;
        /* Data length is currently passed inclusive of H/W MIC  */
        *rx_len -= TKIP_HDR_LEN + TKIP_ICV_LEN + TKIP_MIC_LEN;
    }
}

INLINE void copy_11i_trailer(UWORD8 *dst, UWORD8 *src)
{
    memcpy(dst, src, TKIP_MIC_LEN);
}

/* This function performs defragmentation by copying the different fragments */
/* into the first fragment buffer but the other buffers are not freed.       */
/* It is assumed that the first fragment buffer is large enough to           */
/* accomodate the complete MSDU.                                             */
INLINE void merge_1x_frag_buffers(buffer_desc_t  *frame_frags,
                               UWORD8 num_frags)
{
    UWORD8 indx             = 0;
    UWORD8 *next_frag_start = NULL;

    /* The location where the next fragment has to be copied. */
    next_frag_start = frame_frags[0].buff_hdl + frame_frags[0].data_offset +
                           frame_frags[0].data_length;

    for(indx = 1; indx < num_frags; indx++)
    {
        /* Copy the next fragment buffer */
        memcpy(next_frag_start,
               frame_frags[indx].buff_hdl + frame_frags[indx].data_offset,
               frame_frags[indx].data_length);

        /* Update the next-copy location and the length of the first fragment */
        next_frag_start           += frame_frags[indx].data_length;
        frame_frags[0].data_length += frame_frags[indx].data_length;
    }
}

/* Check whether the SNAP implies presence of 802.1x/802.11i header. */
/* If yes, then merge the fragments into a single buffer.            */
INLINE BOOL_T check_11i_frame(UWORD8 *snap_hdr_ptr)
{
    /* Check SNAP header */
    if((snap_hdr_ptr[6] == 0x88) && (snap_hdr_ptr[7] == 0x8E))
        return BTRUE;

    return BFALSE;
}

#endif /* FRAME_802_11I_H */

#endif /* MAC_802_11I */
