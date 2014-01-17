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
/*  File Name         : utils.h                                              */
/*                                                                           */
/*  Description       : This file contains the constant definitions, enums,  */
/*                      structures and inline utility functions required by  */
/*                      RSNA Key Management.                                 */
/*                                                                           */
/*  List of Functions : str_max                                              */
/*                      str_mix                                              */
/*                      get_kck                                              */
/*                      get_kek                                              */
/*                      get_tk                                               */
/*                      get_auth_mic_key                                     */
/*                      get_supp_mic_key                                     */
/*                      get_pmkid                                            */
/*                      get_pmk                                              */
/*                      pbkdf2_sha1                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef UTILS_H
#define UTILS_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "prf.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define PMK_LEN             32
#define GMK_LEN             32
#define PMKID_LEN           16
#define NONCE_LEN           32
#define PTK_PREFIX_LEN      22
#define GTK_PREFIX_LEN      19
#define GTK_DATA_LEN        (NONCE_LEN + MAC_ADDRESS_LEN)
#define PTK_DATA_LEN        (2 * NONCE_LEN + 2 * MAC_ADDRESS_LEN)
#define A_SHA_DIGEST_LEN    20

#define KCK_LENGTH          16
#define KEK_LENGTH          16
#define TEMPORAL_KEY_LENGTH 16
#define MIC_KEY_LENGTH      8


/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void get_ptk(UWORD8* pmk, UWORD8* ptk, UWORD16 ptk_len, UWORD8* aa,
                    UWORD8* spa, UWORD8* anonce, UWORD8* snonce);
extern void get_gtk(UWORD8* gmk, UWORD8* gtk, UWORD16 gtk_len, UWORD8* aa,
                    UWORD8* gnonce);
extern UWORD8 pbkdf2_sha1_f(UWORD8 *key, UWORD8 key_len,
                            UWORD8 *data, UWORD32 data_len,
                   UWORD32 iterations, UWORD32 count, UWORD8 *digest);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function returns the pointer to the maximum of the 2 strings.        */
INLINE UWORD8* str_max(UWORD8* s1, UWORD8* s2, UWORD32 size)
{
    if(memcmp(s1, s2, size) > 0)
        return s1;

    return s2;
}

/* This function returns the pointer to the minimum of the 2 strings.        */
INLINE UWORD8* str_min(UWORD8* s1, UWORD8* s2, UWORD32 size)
{
    if(memcmp(s1, s2, size) < 0)
        return s1;

    return s2;
}

/* This function returns the Key Confirmation Key.                           */
INLINE UWORD8* get_kck(UWORD8* key)
{
    return key;
}

/* This function returns the Key Encryption Key.                             */
INLINE UWORD8* get_kek(UWORD8* key)
{
    return (key + KCK_LENGTH);
}

/* This function returns the Temporal Key.                                   */
INLINE UWORD8* get_tk(UWORD8* key)
{
    return (key + KCK_LENGTH + KEK_LENGTH);
}

/* This function returns the Authenticator TKIP MIC Key                      */
INLINE UWORD8* get_auth_mic_key(UWORD8* key, UWORD8 key_type)
{
    if(key_type == 1)
    {
        /* Derive MIC Key from the 64 byte PTK */
        return (key + KCK_LENGTH + KEK_LENGTH + TEMPORAL_KEY_LENGTH);
    }
    else
    {
        /* Derive MIC Key from the 32 byte GTK */
        return (key + TEMPORAL_KEY_LENGTH);
    }
}

/* This function returns the Supplicatant TKIP MIC Key                       */
INLINE UWORD8* get_supp_mic_key(UWORD8* key, UWORD8 key_type)
{
    if(key_type == 1)
    {
        /* Derive MIC Key from the 64 byte PTK */
        return (key + KCK_LENGTH + KEK_LENGTH +
                        TEMPORAL_KEY_LENGTH + MIC_KEY_LENGTH);
    }
    else
    {
        /* Derive MIC Key from the 32 byte GTK */
        return (key + TEMPORAL_KEY_LENGTH + MIC_KEY_LENGTH);
    }
}


/* This function returns the PMKID.                                          */
/* PMKID = HMAC-SHA1-128(PMK, "PMK Name" || AA || SPA)                       */
INLINE void get_pmkid(UWORD8* pmk, UWORD8* aa, UWORD8* spa,
                                 UWORD8* pmkid)
{
    UWORD8 index = 0;
    UWORD8 data[20];
    UWORD8 digest[HMAC_DIGEST_LENGTH];

    strcpy((WORD8 *)data, "PMK Name");
    index += 8;

    memcpy(data + index, aa, 6);
    index += 6;

    memcpy(data + index, spa, 6);
    index += 6;

    /* Obtain the 160 bits (20 bytes) HMAC digest */
    hmac_sha1(pmk, PMK_LEN, data, index, digest);

    /* Extract first 128 bits (16 bytes) of the digest and save in the input */
    /* PMKID buffer.                                                         */
    memcpy(pmkid, digest, PMKID_LEN);
}

/* This function returns the PMK from the AAA key.                           */
INLINE void get_pmk(UWORD8* aaakey, UWORD8* pmk)
{
    /* Extract first 256 bits (32 bytes) of the AAA key and save in the      */
    /* input PMK buffer.                                                     */
    memcpy(pmk, &aaakey[1], PMK_LEN);
}


/* This function runs the PBKDF2 on the data with a given                    */
/* key. To obtain the PSK from the password, PBKDF2 is                       */
/* run on SSID (data) with password (key)                                    */
INLINE UWORD8 pbkdf2_sha1(UWORD8 *key, UWORD8 key_len,
                          UWORD8 *data, UWORD8 data_len,
                     UWORD8 *digest)
{
    if ((key_len > 64) || (data_len > 32))
        return 0;

    if(pbkdf2_sha1_f(key, key_len, data, data_len, 4096, 1, digest) == 0)
    {
        return 0;
    }
    return pbkdf2_sha1_f(key, key_len, data, data_len,
                            4096, 2, &digest[A_SHA_DIGEST_LEN]);
}

#endif /* UTILS_H */
#endif /* MAC_802_11I */
