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
/*  File Name         : eapol_key.h                                          */
/*                                                                           */
/*  Description       : This file contains the constant definitions, enums,  */
/*                      structures and inline utility functions required by  */
/*                      the EAPOL Key frame related functions.               */
/*                                                                           */
/*  List of Functions : get_eap_code                                         */
/*                      get_eap_len                                          */
/*                      get_eap_type                                         */
/*                      get_eap_identity                                     */
/*                      get_eap_identity_len                                 */
/*                      prepare_key_info                                     */
/*                      set_mic_field                                        */
/*                      get_rply_cnt                                         */
/*                      get_nonce                                            */
/*                      get_request                                          */
/*                      get_k                                                */
/*                      get_eapol_type                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef EAPOL_KEY_H
#define EAPOL_KEY_H


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "rsna_km.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define KEY_DESCRIPTOR_SIZE 1
#define KEY_INFO_SIZE       2
#define KEY_LENGTH_SIZE     2
#define REPLAY_COUNTER_SIZE 8
#define NONCE_SIZE          32
#define EAPOL_KEY_IV_SIZE   16
#define EAPOL_KEY_IV_OFFSET 45
#define RSC_SIZE            8
#define RSC_OFFSET          61
#define KEY_ID_SIZE         8
#define MIC_SIZE            16
#define MIC_OFFSET          77
#define KEY_DATA_OFFSET     95

#define EAPOL_PKT_MAX_SIZE  200

#define EAPOL_VERSION_01    0x01
#define EAPOL_VERSION_02    0x02/* This needs to be 0x02 as per the standard */
                                /* but to interop with older entities, set   */
                                /* this to 0x01                              */
#define EAPOL_1X_HDR_LEN    12  /* Includes SNAP length and EAPOL Header len */
#define EAPOL_HDR_LEN       4   /* EAPOL Header len                          */

#define PMKID_KDE_SIZE      (PMKID_LEN + KDE_OFFSET)

#define KDE_TYPE            0xDD
#define KDE_OUI             0x000FAC
#define KDE_OFFSET          6

#define MAX_EAPOL_KEY_FRAME_SIZE      2000
#define MAX_EAPOL_KEY_DATA_SIZE       2000
#define RC4_KEY_DESCRIPTOR_TYPE       0x01
#define IEE80211_KEY_DESCRIPTOR_TYPE  0x02
#define WPA_KEY_DESCRIPTOR_TYPE       0xFE

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Key Types */
typedef enum {G = 0,
              P = 1
} KEY_TYPE_T;

/* KDE Data Types */
typedef enum {GTK_KDE      = 1,
              STA_KEY_KDE  = 2,
              MAC_ADDR_KDE = 3,
              PMKID_KDE    = 4
} KDE_DATA_TYPE;

typedef enum {EAP_TYPE_IDENTITY           = 1,
              EAP_TYPE_NOTIFICATION       = 2,
              EAP_TYPE_NAK                = 3,
              EAP_TYPE_MD5_CHALLENGE      = 4,
              EAP_TYPE_ONE_TIME_PWD       = 5,
              EAP_TYPE_GENERIC_TOKEN_CARD = 6,
              EAP_TYPE_TLS                = 13,
              EAP_TYPE_TTLS               = 21,
              EAP_TYPE_PEAP               = 25,
              EAP_TYPE_MS_CHAP_V2         = 29,
} EAP_TYPE;

/* KDE Data Types */
typedef enum {EAP_PACKET   = 0,
              EAPOL_START  = 1,
              EAPOL_LOGOFF = 2,
              EAPOL_KEY    = 3,
              EAPOL_ENC_ASF= 4
} EAPOL_PKT_TYPE;

/* EAP packet type - Code field */
typedef enum {EAP_REQUEST      = 0x01, /* EAP Pkt Type - Request  */
              EAP_RESPONSE     = 0x02, /* EAP Pkt Type - Response */
              EAP_SUCCESS      = 0x03, /* EAP Pkt Type - Success  */
              EAP_FAILURE      = 0x04  /* EAP Pkt Type - Failure  */
} EAP_CODE_T;


/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_mode_802_11i;

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/


/* This function returns the EAP Id present in the EAPOL Packet */
INLINE UWORD8 get_eapol_id(UWORD8 *pkt)
{
    return (*(UWORD8 *)(pkt + 5));
}

/* Get the EAP code packet from the pointer to the start of the EAPOL packet */
INLINE UWORD8 get_eap_code(UWORD8* eapol_hdr)
{
    return *(UWORD8*)(eapol_hdr + EAPOL_HDR_LEN);
}

/* Get the EAP packet length from the pointer to the start of the EAP packet */
INLINE UWORD16 get_eap_len(UWORD8* eap)
{
    UWORD16 len = eap[2];

    len <<= 8;
    len = len | eap[3];

    return len;
}

/* Get the EAP packet type from the pointer to the start of the EAP packet */
INLINE UWORD8 get_eap_type(UWORD8* eap)
{
    return eap[4];
}

/* Get the EAP Identity from the pointer to the start of the EAP ID packet */
INLINE UWORD8* get_eap_identity(UWORD8* eap)
{
    return &(eap[5]);
}

/* Get the EAP Identity length from the pointer to the start of EAP packet */
INLINE UWORD8 get_eap_identity_len(UWORD8* eap)
{
    return (UWORD8)((eap[3] | (eap[2] << 8)) - 5);
}

/* This function sets the MIC value in the EAPOL Key frames. */
INLINE void set_mic_field(UWORD8* eapol_key_frame, UWORD8* mic)
{
    memcpy(&eapol_key_frame[MIC_OFFSET], mic, MIC_SIZE);
}

INLINE UWORD8 *get_rply_cnt(UWORD8* eapol_key_frame)
{
    return (eapol_key_frame + EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE
                + KEY_INFO_SIZE +  KEY_LENGTH_SIZE);
}

/* This function gets the pointer to the Nonce field in the EAPOL Key frame. */
INLINE UWORD8 *get_nonce(UWORD8* eapol_key_frame)
{
    return (eapol_key_frame + EAPOL_HDR_LEN + KEY_DESCRIPTOR_SIZE
                + KEY_INFO_SIZE +  KEY_LENGTH_SIZE  + REPLAY_COUNTER_SIZE);
}

/* This function gets the pointer to the Nonce field in the EAPOL Key frame. */
INLINE UWORD8 get_request(UWORD8* eapol_key_frame)
{
    UWORD8 request = 0;

    if(eapol_key_frame != NULL)
    {
        request = ((UWORD8)(eapol_key_frame
                    [KEY_DESCRIPTOR_SIZE + EAPOL_HDR_LEN] & BIT3)) >> 3;
	}

    return request;
}

/* This function gets the pointer to the key type field in EAPOL Key frame. */
INLINE UWORD8 get_k(UWORD8* eapol_key_frame)
{
    UWORD8 request = 0;

    if(eapol_key_frame != NULL)
    {
        request = ((UWORD8)(eapol_key_frame
                    [KEY_DESCRIPTOR_SIZE + EAPOL_HDR_LEN + 1] & BIT3)) >> 3;
	}

    return request;
}

/* This function gets the EAPOL Packey Type for the EAPOL Key frame. */
INLINE UWORD8 get_eapol_type(UWORD8* eapol_key_frame)
{
    return ((UWORD8)eapol_key_frame[1]);
}

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD16 prepare_eapol_key_frame(UWORD16 key_info, UWORD16 key_length,
                                       UWORD8* key_rsc, UWORD8* nonce,
                                       UWORD8 *key_iv,  UWORD16 key_data_len,
                                       UWORD8* key_data, UWORD8* replay_cnt,
                                       UWORD8* data, UWORD8 mode_802_11i);

extern UWORD16 prepare_key_info(UWORD8 dscr_ver, UWORD8 secure_bit,
                                UWORD8 mic_available, UWORD8 ack_bit,
                                UWORD8 install_flag, KEY_TYPE_T key_type,
                                UWORD8 key_index, UWORD8 encrypted,
                                UWORD8 mode_802_11i);

extern void prepare_1x_hdr(UWORD8* buffer, EAPOL_PKT_TYPE pkt_type,
                           UWORD16 buffer_len, UWORD8 mode_802_11i);
#endif /* EAPOL_KEY_H */

#endif /* MAC_802_11I  */

