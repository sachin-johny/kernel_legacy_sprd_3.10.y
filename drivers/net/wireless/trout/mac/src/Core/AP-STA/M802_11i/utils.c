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
/*  File Name         : utils.c                                              */
/*                                                                           */
/*  Description       : This file contains the function definitions required */
/*                      by RSNA Key Management.                              */
/*                                                                           */
/*  List of Functions : get_ptk                                              */
/*                      get_gtk                                              */
/*                      pbkdf2_sha1_f                                        */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "utils.h"
#include "prf.h"
#include "imem_if.h"

#ifdef UTILS_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_ptk                                               */
/*                                                                           */
/*  Description      : This function derives the PTK from the PMK.           */
/*                                                                           */
/*  Inputs           : 1) Pairwise Master Key (PMK)                          */
/*                     2) Pairwise Transient Key Buffer (PTK)                */
/*                     2) PTK Length                                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function returns the PTK from the PMK.           */
/*                     PTK <- PRF-X(PMK, "Pairwise key expansion",           */
/*                     Min(AA, SPA) || Max(AA, SPA) || Min(ANonce, SNonce)   */
/*                     || Max(ANonce, SNonce)                                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void get_ptk(UWORD8* pmk, UWORD8* ptk, UWORD16 ptk_len, UWORD8* aa,
             UWORD8* spa, UWORD8* anonce, UWORD8* snonce)
{
    UWORD8 data[PTK_DATA_LEN];
    UWORD8 index = 0;

    /* Prepare the data for PTK generation by concatenating Min(AA, SPA),    */
    /* Max(AA, SPA), Min(ANonce, SNonce), Max(ANonce, SNonce)                */

    memcpy(data, str_min(aa, spa, MAC_ADDRESS_LEN), MAC_ADDRESS_LEN);
    index += MAC_ADDRESS_LEN;

    memcpy(data + index, str_max(aa, spa, MAC_ADDRESS_LEN), MAC_ADDRESS_LEN);
    index += MAC_ADDRESS_LEN;

    memcpy(data + index, str_min(anonce, snonce, NONCE_LEN), NONCE_LEN);
    index += NONCE_LEN;

    memcpy(data + index, str_max(anonce, snonce, NONCE_LEN), NONCE_LEN);
    index += NONCE_LEN;

    /* Call PRF to generate the PTK of required size */
    prf(pmk, PMK_LEN, "Pairwise key expansion", PTK_PREFIX_LEN, data,
        PTK_DATA_LEN, ptk_len, ptk);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_gtk                                               */
/*                                                                           */
/*  Description      : This function derives the GTK from the GMK.           */
/*                                                                           */
/*  Inputs           : 1) Group Master Key (GMK)                             */
/*                     2) Group Transient Key Buffer (GTK)                   */
/*                     2) GTK Length                                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function returns the GTK from the GMK.           */
/*                     GTK <- PRF-X(GMK, "Group key expansion", AA || GNonce */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void get_gtk(UWORD8* gmk, UWORD8* gtk, UWORD16 gtk_len, UWORD8* aa,
             UWORD8* gnonce)
{
    UWORD8 data[GTK_DATA_LEN];
    UWORD8 index = 0;

    /* Prepare the data for GTK generation by concatenating AA, GNonce       */

    memcpy(data, aa, MAC_ADDRESS_LEN);
    index += MAC_ADDRESS_LEN;

    memcpy(data + index, gnonce, NONCE_LEN);
    index += NONCE_LEN;

    /* Call PRF to generate the GTK of required size */
    prf(gmk, GMK_LEN, "Group key expansion", GTK_PREFIX_LEN, data,
        GTK_DATA_LEN, gtk_len, gtk);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : pbkdf2_sha1_f                                         */
/*                                                                           */
/*  Description      : This function iterates the hashing of digest as per   */
/*                     the key                                               */
/*                                                                           */
/*  Inputs           : 1) key                                                */
/*                     2) data                                               */
/*                     3) key len                                            */
/*                     4) iterations                                         */
/*                     5) count                                              */
/*                     6) digest                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : F(P, S, c, i) = U1 xor U2 xor ... Uc                  */
/*                     U1 = PRF(P, S || Int(i))                              */
/*                     U2 = PRF(P, U1)                                       */
/*                     Uc = PRF(P, Uc-1)                                     */
/*                                                                           */
/*  Outputs          : Iterated hashed password                              */
/*                                                                           */
/*  Returns          : Returns 0 on failure 1 otherwise returns 1            */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD8 pbkdf2_sha1_f(UWORD8 *key, UWORD8 key_length,
                     UWORD8 *data, UWORD32 data_len,
                   UWORD32 iterations, UWORD32 count, UWORD8 *digest)
{
    UWORD8 *tmp1    = 0; // [36]
    UWORD8 *tmp2    = 0; // [A_SHA_DIGEST_LEN];
    UWORD16 i, j;
    UWORD16 curr_idx = 0;

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    tmp1 = (UWORD8 *)scratch_mem_alloc(36);
    if(tmp1 == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return 0;
    }

    tmp2 = (UWORD8 *)scratch_mem_alloc(A_SHA_DIGEST_LEN);
    if(tmp2 == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return 0;
    }

    for (i = 0; i < key_length ; i++)
    {
        if(key[i] < 32)
        {
            /* Restore the saved scratch memory index */
            restore_scratch_mem_idx(curr_idx);

            return 0;
        }
        else if(key[i] > 126)
        {
            /* Restore the saved scratch memory index */
            restore_scratch_mem_idx(curr_idx);

            return 0;
        }
    }

    /* U1 = PRF(P, S || int(i)) */
    memcpy(tmp1, data, data_len);

    tmp1[data_len]   = (UWORD8)((count>>24) & 0xFF);
    tmp1[data_len+1] = (UWORD8)((count>>16) & 0xFF);
    tmp1[data_len+2] = (UWORD8)((count>>8) & 0xFF);
    tmp1[data_len+3] = (UWORD8)(count & 0xFF);

    hmac_sha1(key, key_length, tmp1, (data_len + 4), tmp2);

    /* digest = U1 */
    memcpy(digest, tmp2, A_SHA_DIGEST_LEN);

    for (i = 1; i < iterations; i++)
    {
        /* Un = PRF(P, Un-1) */
        hmac_sha1(key, key_length, tmp2, A_SHA_DIGEST_LEN, tmp1);

        memcpy(tmp2, tmp1, A_SHA_DIGEST_LEN);

        /* digest = digest xor Un */
        for (j = 0; j < A_SHA_DIGEST_LEN; j++)
        {
            digest[j] ^= tmp1[j];
        }
    }

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);

    return 1;
}

#endif /* UTILS_11I */
#endif /* MAC_802_11I */

