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
/*  File Name         : tkip.h                                               */
/*                                                                           */
/*  Description       : This file contains the MAC S/W TKIP related          */
/*                      functions declarations                               */
/*                                                                           */
/*  List of Functions : insert_tkip_mic                                      */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef TKIP_H
#define TKIP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "maccontroller.h"
#include "transmit.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void compute_tkip_mic(UWORD8 *da, UWORD8* sa, UWORD8 priority,
                             buffer_desc_t  *frame_frags, UWORD8 num_frags,
                             UWORD16 frame_size, UWORD8 *mic_key, UWORD8 *mic);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/



/* This function implements function b as specified in 802.11i - 8.3.2.3 */
INLINE void compute_b(UWORD32 *l, UWORD32 *r)
{
    UWORD32 L = 0;
    UWORD32 R = 0;

    L = *l;
    R = *r;

    /* compute_b is defined as follows in the RFC/11i Standard             */
    /* R ^= ROL32(L, 17)                                                   */
    /* L += R                                                              */
    /* R ^= Byteswap(L)                                                    */
    /* L += R                                                              */
    /* R ^= ROL32(L, 3)                                                    */
    /* L += R                                                              */
    /* R ^= ROR32(L, 2)                                                    */
    /* L += R                                                              */

    /* Rotation functions on 32 bit values                                 */
    /* ROL32(A, n) => (((A) << (n)) | (((A)>>(32-n)) & ((1UL << n) - 1 ))) */
    /* ROR32(A, n) => ROL32(A, 32-n)                                       */

    R ^= ((L << 17 ) | (( L >> 15) & (0x1ffff)));
    L += R;
    R ^= ((L & 0xFF00FF00) >> 8) | ((L & 0x00FF00FF) << 8);
    L += R;
    R ^= ((L << 3 ) | (( L >> 29) & (0x7)));
    L += R;
    R ^= ((L << 30 ) | (( L >> 2) & (0x3fffffff)));
    L += R;

    *l = L;
    *r = R;
}

/* Insert the TKIP Mic at the bottom of the out going packet */
INLINE void insert_tkip_mic(wlan_tx_req_t *tx_req)
{
    /* Add the mic length to the data length to be transmitted */
    tx_req->data_len += 8;
}

#endif /* TKIP_H */
#endif /* MAC_802_11I */
