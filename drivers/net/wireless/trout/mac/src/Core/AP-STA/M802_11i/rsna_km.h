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
/*  File Name         : rsna_km.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      RSNA Key Management State Machine.                   */
/*                                                                           */
/*  List of Functions : get_pn_val                                           */
/*                      incr_byte_cnt                                        */
/*                      incr_rply_cnt                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef RSNA_KM_H
#define RSNA_KM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"
#include "management.h"
#include "management_11i.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define NUM_SUPP_STA    MAX_STA_SUPPORTED

#define PMK_SIZE        32 /* In Bytes */
#define PTK_SIZE        64 /* In Bytes */
#define GTK_SIZE        32 /* In Bytes */
#define GMK_SIZE        32 /* In Bytes */
#define NONCE_SIZE      32 /* In Bytes */
#define NUM_GTK         3
#define GTK_BASE_INDEX  1
#define GTK_MAX_INDEX   3

#define PMKID_LEN       16 /* In Bytes */

#define REPLAY_CNT_SIZE 8
#define CCMP_HDR_LEN    8
#define CCMP_MIC_LEN    8
#define TKIP_HDR_LEN    8
#define TKIP_MIC_LEN    8
#define TKIP_ICV_LEN    4

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* 802_11i mode: RSNA or WPA modes */
typedef enum{WPA_802_11I  = 1,
             RSNA_802_11I = 2
}MODE_11I_T;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern BOOL_T   cmp_pn_val(UWORD8 *pn_val_old, UWORD8 *pn_val_rx);
extern void     get_iv_2_pn_val(UWORD32 iv_l, UWORD32 iv_h, UWORD8 cipsuite,
                                UWORD32 *pn_val);
extern CIPHER_T cipsuite_to_ctype(UWORD8 cipsuite);
extern void     incr_cnt(UWORD8 *cntr, UWORD8 size);
extern void   initialize_rsna(void);
extern void   install_psk(void);
extern BOOL_T send_eapol(UWORD8 *addr, UWORD8 *buffer, UWORD16 len,
                    BOOL_T secured);
extern BOOL_T verifyMIC(UWORD8 *eapol_buffer, UWORD16 eapol_length,
                    UWORD8 *key, UWORD8 key_version);
extern void   add_eapol_mic(UWORD8 *eapol_buffer, UWORD16 eapol_length,
                    UWORD8 *key, UWORD8 key_version);
extern void   rsna_send_deauth(UWORD8 *rsna_ptr, UWORD8 supp);
extern void   init_sec_auth(void);
extern void   compute_psk(void);

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_mode_802_11i;
extern BOOL_T g_psk_available;

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* Increment a byte counter of a given size */
INLINE void incr_byte_cnt(UWORD8 *cntr, UWORD8 size)
{
    WORD16 index = 0;

    while(index < size)
    {
        cntr[index]++;
        if(cntr[index] != 0)
        {
            break;
        }
        else
        {
            index ++;
        }
    }
}

/* Increment counter of a size REPLAY_CNT_SIZE byte */
INLINE void incr_rply_cnt(UWORD8 *cntr)
{
    incr_cnt(cntr, REPLAY_CNT_SIZE);
}

/* Utility to extract the PN Value from the received packet */
INLINE void get_pn_val(UWORD8 *msa, UWORD8 cipsuite, UWORD8 *pn_val)
{
    switch(cipsuite)
    {
        case TKIP:
        {
            /* TKIP TSC Value as contained in the IV and extended IV field */
            /* ----------------------------------------------------------- */
            /*    IV / KeyID              |    Extended IV                 */
            /*    (4 octets)              |    (4 octets)                  */
            /* TSC1  WEPSeed  TSC0  FLAGs      TSC2 TSC3 TSC4 TSC5         */
            pn_val[0] = msa[2];
            pn_val[1] = msa[0];
            memcpy(&pn_val[2], &msa[4], 4);
        }
        break;
        case CCMP:
        {
            /* CCMP PN Value as contained in the CCMP Header field   */
            /* ----------------------------------------------------- */
            /*  PN0  PN1  Rsvd  FLAGs  PN2  PN3  PN4  PN5            */
            pn_val[0] = msa[0];
            pn_val[1] = msa[1];
            memcpy(&pn_val[2], &msa[4], 4);
        }
        break;
        default:
        break;
    }
}

/* Utility to extract the PN Value from the received packet */
INLINE UWORD8 get_GTK_ID(UWORD8 *msa)
{
    return ((msa[3] & 0xC0) >> 6);
}

INLINE UWORD8 get_GTK_array_index(UWORD8 gtk_id)
{
    if((gtk_id >= GTK_BASE_INDEX) &&
       (gtk_id <= GTK_MAX_INDEX))
    {
        return (gtk_id - GTK_BASE_INDEX);
    }
    else
    {
        return 0;
    }
}
#endif /* RSNA_KM_H */
#endif /* MAC_802_11I */

