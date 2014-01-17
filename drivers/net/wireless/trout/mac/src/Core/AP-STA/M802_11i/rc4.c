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
/*  File Name         : rc4.c                                                */
/*                                                                           */
/*  Description       : This file contains the RC4 algorithm used by 802.11i */
/*                      and 802.1X state machines                            */
/*                                                                           */
/*  List of Functions : rc4_skip                                             */
/*                      rc4                                                  */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "rc4.h"
#include "imem_if.h"

#ifdef UTILS_11I

/*****************************************************************************/
/* Constants Definitions                                                     */
/*****************************************************************************/

#define S_SWAP(a,b) do { UWORD8 t = S[a]; S[a] = S[b]; S[b] = t; } while(0)

/*****************************************************************************/
/*                                                                           */
/*  Function Name : rc4_skip                                                 */
/*                                                                           */
/*  Description   : This function encrypts the data using the RC4 algorithm  */
/*                                                                           */
/*  Inputs        : 1) The pointer to the RC4 Key                            */
/*                  2) The RC4 key length                                    */
/*                  3) The number of bytes to be skiped in the RC4 stream    */
/*                  4) Pointer to the data to be XOR'ed with RC4 stream      */
/*                  5) The buf length                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Generate RC4 pseudo random stream for the given key,     */
/*                  skip beginning of the stream, and XOR the end result     */
/*                  with the data buffer to perform RC4 encryption. XOR this */
/*                  with the data buffer to perform RC4 encryption           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void rc4_skip(UWORD8 *key, UWORD32 keylen, UWORD32 skip,
              UWORD8 *data, UWORD32 data_len)
{
    UWORD32 i        = 0;
    UWORD32 j        = 0;
    UWORD32 k        = 0;
    WORD32  kpos     = 0;
    UWORD16 curr_idx = 0;
    UWORD8  *pos     = 0;
    UWORD8  *S       = 0;

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    S = (UWORD8 *)scratch_mem_alloc(256);

    if(S == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return ;
    }

    /* Setup RC4 state */
    for (i = 0; i < 256; i++)
        S[i] = i;
    j = 0;
    kpos = 0;
    for (i = 0; i < 256; i++) {
        j = (j + S[i] + key[kpos]) & 0xFF;
        kpos++;
        if (kpos >= keylen)
            kpos = 0;
        S_SWAP(i, j);
    }

    /* Skip the start of the stream */
    i = j = 0;
    for (k = 0; k < skip; k++) {
        i = (i + 1) & 0xFF;
        j = (j + S[i]) & 0xFF;
        S_SWAP(i, j);
    }

    /* Apply RC4 to data */
    pos = data;
    for (k = 0; k < data_len; k++) {
        i = (i + 1) & 0xFF;
        j = (j + S[i]) & 0xFF;
        S_SWAP(i, j);
        *pos++ ^= S[(S[i] + S[j]) & 0xFF];
    }

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : rc4                                                      */
/*                                                                           */
/*  Description   : This function encrypts the data using the RC4 algorithm  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the data to be XOR'ed with RC4 stream      */
/*                  2) The buf length                                        */
/*                  3) The pointer to the RC4 Key                            */
/*                  4) The RC4 key length                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Generate RC4 pseudo random stream for the given key and  */
/*                  XOR this with the data buffer to perform RC4 encryption  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void rc4(UWORD8 *buf, UWORD32 len, UWORD8 *key, UWORD32 key_len)
{
    rc4_skip(key, key_len, 0, buf, len);
}

#endif /* UTILS_11I */
#endif /* MAC_802_11I */
