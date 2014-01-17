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
/*  File Name         : aeskeywrap.c                                         */
/*                                                                           */
/*  Description       : This file contains the functions required for AES    */
/*                      encryption and decryption algorithms.                */
/*                                                                           */
/*  List of Functions : aes_key_wrap                                         */
/*                      aes_key_unwrap                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "aes.h"
#include "imem_if.h"
#include "common.h"

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

#ifdef UTILS_11I
static const UWORD8 g_iv[8] = {0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6, 0xA6};

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : aes_key_wrap                                          */
/*                                                                           */
/*  Description      : This performs AES Key wrap on given data.             */
/*                                                                           */
/*  Inputs           : 1) Pointer to plain text                              */
/*                     2) Key pointer                                        */
/*                     3) Number of 64-bit blocks                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : AES Key wrap as specified in RFC3394 is performed on  */
/*                     the given data in place.                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void aes_key_wrap(UWORD8 *text, UWORD8 *key, UWORD32 n)
{
    UWORD32       t        = 0;
    UWORD32       i        = 0;
    UWORD32       j        = 0;
    UWORD32       k        = 0;
    UWORD32       c        = 0;
    UWORD32       idx      = 0;
    UWORD16       curr_idx = 0;
    UWORD8        *A       = 0; //[8];
    UWORD8        *B       = 0; //[16];
    UWORD8        *R[8]    = {0}; //[8][100];
    UWORD8        *Rbuff   = NULL;
    aes_context_t ctx      = {{0},};

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Allocate scratch memory for the temporary variables */
    A = (UWORD8 *)scratch_mem_alloc(8);
    if(A == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    B = (UWORD8 *)scratch_mem_alloc(16);
    if(B == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    Rbuff = (UWORD8 *)scratch_mem_alloc(8*100);
    if(Rbuff == NULL)
        {
            /* Restore the saved scratch memory index */
            restore_scratch_mem_idx(curr_idx);

            return;
        }

    for(i = 0; i < 8; i ++)
    {
        R[i] = Rbuff + i*100;
    }

    /* AES initialization - Key scheduling  */
    ctx.nrounds = NUM_ROUNDS;
    aes_setup(&ctx, 16, key);


    /* 1) Initialize variables */
    /* Set A = IV              */
    /* For i = 1 to n          */
    /* R[i] = P[i]             */
    for(k = 0; k < 8; k++)
        A[k] = g_iv[k];

    for(i = 0, idx = 0; i < n; i++)
        for(k = 0; k < 8; k++)
            R[k][i] = text[idx++];

    /* 2) Compute intermediate values                */
    /* For j = 5 to 0                                */
    /*     For i = 1 to n                            */
    /*         B = AES(K, A | R[i])                  */
    /*         A = MSB(64, B) ^ t where t = (n*j)+i  */
    /*         R[i] = LSB(64, B)                     */
    for(j = 0; j <= 5; j++)
    {
        for(i = 0; i < n; i++)
        {
            UWORD8 temp128[16];

            /* A | R[i] */
            for(k = 0; k < 8; k ++)
                temp128[k] = A[k];
            for(k = 8, c = 0; k < 16 && c < 8; k++, c++)
                temp128[k] = R[c][i];

            /* B = AES(K, A | R[i] */
            aes_encrypt(&ctx, temp128, B);

            /* MSB(64, B) */
            for(k = 0; k < 8; k++)
                A[k] = B[k];

            /* t does not need to be greater than 32 bits for allowed data   */
            /* lengths. Verify this.                                         */
            t = (n * j) + (i + 1);

            /* A = MSB(64, B) ^ t where t = (n*j)+i  */
            for(k = 0; k < 4; k ++)
            {
                UWORD8 ptr = (UWORD8)(t >> (k << 3));
                A[7 - k] = A[7 - k] ^ ptr;
            }

            /* R[i] = LSB(64, B) */
            for(k = 0; k < 8; k++)
                R[k][i] = B[k + 8];
        }
    }

    /* 3) Output Results */
    /* C[0] = A          */
    /* For i = 1 to n    */
    /*      C[i] = R[i]  */
    for(k = 0; k < 8; k ++)
        text[k] = A[k];

    idx = 8;

    for(i = 0; i < n; i++)
        for(k = 0; k < 8; k ++)
            text[idx++] = R[k][i];

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : aes_key_unwrap                                        */
/*                                                                           */
/*  Description      : This performs AES Key unwrap on given data.           */
/*                                                                           */
/*  Inputs           : 1) Pointer to cipher text                             */
/*                     2) Key pointer                                        */
/*                     3) Number of 64-bit blocks                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : AES Key unwrap as specified in RFC3394 is performed   */
/*                     on the given data in place.                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE, if key integrity check passes                  */
/*                     BFALSE, otherwise.                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T aes_key_unwrap(UWORD8 *text, UWORD8 *key, UWORD32 n)
{
    BOOL_T        ret_val  = BFALSE;
    UWORD32       t        = 0;
    UWORD32       i        = 0;
    UWORD32       j        = 0;
    UWORD32       k        = 0;
    UWORD32       c        = 0;
    UWORD32       idx      = 0;
    UWORD16       curr_idx = 0;
    UWORD8        *A       = 0; //[8];
    UWORD8        *B       = 0; //[16];
    UWORD8        *R[8]    = {0}; //[8][100];
    UWORD8        *Rbuff   = NULL;
    aes_context_t ctx      = {{0},};

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Allocate scratch memory for the temporary variables */
    A = (UWORD8 *)scratch_mem_alloc(8);
    if(A == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return ret_val;
    }

    B = (UWORD8 *)scratch_mem_alloc(16);
    if(B == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return ret_val;
    }

    Rbuff = (UWORD8 *)scratch_mem_alloc(8*100);
    if(Rbuff == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return ret_val;
    }

    for(i = 0; i < 8; i++)
    {
        R[i] = Rbuff + i*100;
    }

    /* AES initialization - Key scheduling */
    ctx.nrounds = NUM_ROUNDS;
    aes_setup(&ctx, 16, key);

    /* 1) Initialize variables  */
    /* Set A = C[0]             */
    /* For i = 1 to n           */
    /* R[i] = C[i]              */

    idx = 0;

    for(k = 0; k < 8; k++)
        A[k] = text[idx++];

    for(i = 0; i < n; i++)
        for(k = 0; k < 8; k++)
            R[k][i] = text[idx++];

    /* 2) Compute intermediate values                       */
    /* For j = 5 to 0                                       */
    /*     For i = n to 1                                   */
    /*         B = AES-1(K, (A ^ t) | R[i]) where t = n*j+i */
    /*         A = MSB(64, B)                               */
    /*         R[i] = LSB(64, B)                            */
    for(j = 6; j > 0; j--)
    {
        for(i = n; i > 0; i--)
        {
            UWORD8 temp128[16];

            /* t does not need to be greater than 32 bits for allowed data */
            /* lengths. Verify this.                                       */
            t = (n * (j - 1)) + i;

            /* (A ^ t) */
            for(k = 0; k < 4; k ++)
            {
                UWORD8 ptr = (UWORD8)(t >> (k << 3));
                A[7 - k] = A[7 - k] ^ ptr;
            }

            /* (A ^ t) | R[i] */
            for(k = 0; k < 8; k ++)
                temp128[k] = A[k];
            for(k = 8, c = 0; k < 16 && c < 8; k++, c++)
                temp128[k] = R[c][i - 1];

            /* B = AES-1(K, (A ^ t) | R[i]) where t = n*j+i */
            aes_decrypt(&ctx, temp128, B);

            /* A = MSB(64, B) */
            for(k = 0; k < 8; k++)
                A[k] = B[k];

            /* R[i] = LSB(64, B) */
            for(k = 0; k < 8; k++)
                R[k][i - 1] = B[k + 8];
        }

    }

    /* 3) Output Results                       */
    /* If A is an appropriate initial value,   */
    /* Then                                    */
    /*      For i = 1 to n                     */
    /*          P[i] = R[i]                    */
    /* Else                                    */
    /*     Return an error                     */
    if(memcmp(g_iv, A, 8) == 0)
    {
        idx = 0;

        for(i = 1; i <= n; i++)
            for(k = 0; k < 8; k ++)
                text[idx++] = R[k][i - 1];

        ret_val = BTRUE;
    }
    else
    {
        ret_val = BFALSE;
    }

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);

    return ret_val;
}

#endif /* UTILS_11I */
#endif /* MAC_802_11I */

