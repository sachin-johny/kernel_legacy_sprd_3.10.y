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
/*  File Name         : prf.c                                                */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various features of eCos used by RSNA Key management.*/
/*                                                                           */
/*  List of Functions : prf_ft                                               */
/*                      prf_k                                                */
/*                      rotl                                                 */
/*                      sha1                                                 */
/*                      hmac_sha1                                            */
/*                      prf                                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "csl_if.h"
#include "imem_if.h"
#include "prf.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

#ifdef UTILS_11I
static UWORD32 prf_ft(UWORD32 t, UWORD32 x, UWORD32 y, UWORD32 z);
static UWORD32 prf_k(UWORD32 t);
static UWORD32 rotl(UWORD32 bits, UWORD32 a);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prf_ft                                                */
/*                                                                           */
/*  Description      : This function implements the ft function used by the  */
/*                     SHA1 algorithm.                                       */
/*                                                                           */
/*  Inputs           : 1) Iteration number                                   */
/*                     2) 3 variables                                        */
/*                                                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 prf_ft(UWORD32 t, UWORD32 x,UWORD32 y,UWORD32 z)
{
    UWORD32 a = 0;
    UWORD32 b = 0;
    UWORD32 c = 0;

    if (t < 20)
    {
        a = x & y;
        b = (~x) & z;
        c = a ^ b;
    }
    else if (t < 40)
    {
        c = x ^ y ^ z;
    }
    else if (t < 60)
    {
        a = x & y;
        b = a ^ (x & z);
        c = b ^ (y & z);
    }
    else if (t < 80)
    {
        c = (x ^ y) ^ z;
    }

    return c;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prf_k                                                 */
/*                                                                           */
/*  Description      : This function implements the k function used by the   */
/*                     SHA1 algorithm.                                       */
/*                                                                           */
/*  Inputs           : 1) Iteration number                                   */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 prf_k(UWORD32 t)
{
    UWORD32 c = 0;

    if (t < 20)
    {
        c = 0x5A827999;
    }
    else if (t < 40)
    {
        c = 0x6ED9EBA1;
    }
    else if (t < 60)
    {
        c = 0x8F1BBCDC;
    }
    else if (t < 80)
    {
        c = 0xCA62C1D6;
    }

    return c;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rotl                                                  */
/*                                                                           */
/*  Description      : This function implements the circular shift function  */
/*                     used by the SHA1 algorithm.                           */
/*                                                                           */
/*  Inputs           : 1) Number of bits                                     */
/*                     2) Message                                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 rotl(UWORD32 bits, UWORD32 a)
{
    UWORD32 c = 0;
    UWORD32 d = 0;
    UWORD32 e = 0;
    UWORD32 f = 0;
    UWORD32 g = 0;

    c = (0x0001 << (32-bits)) - 1;
    d = ~c;

    e = (a & c) << bits;
    f = (a & d) >> (32 - bits);

    g = e | f;

    return (g & 0xFFFFFFFF);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : sha1                                                  */
/*                                                                           */
/*  Description      : This function implements the NIST SHA-1 algorithm.    */
/*                                                                           */
/*  Inputs           : 1) Message string                                     */
/*                     2) Message length                                     */
/*                     3) Digest                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The details of this algorithm is given in RFC 3174.   */
/*                                                                           */
/*  Outputs          : The digest contains the output.                       */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/


void sha1(UWORD8 *message, UWORD32 message_length,UWORD8 *digest)
{
    UWORD32 num_blocks      = 0;
    UWORD32 num_bytes       = 0;
    UWORD32 block_remainder = 0;
    UWORD32 padded_length   = 0;
    UWORD32 temp            = 0;
    UWORD32 i               = 0;
    UWORD32 l               = 0;
    UWORD32 t               = 0;
    UWORD32 a               = 0;
    UWORD32 b               = 0;
    UWORD32 c               = 0;
    UWORD32 d               = 0;
    UWORD32 e               = 0;
    UWORD32 block_index     = 0;
    UWORD16 curr_idx        = 0;

    UWORD32 h[5];
    UWORD32 *w;  //80

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* allocating memory for the local variable w */
    w = (UWORD32 *)scratch_mem_alloc((80 * sizeof(UWORD32)));
    if(w == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }


    /* Calculate the number of 512 bit blocks */

    /* 8 bits are added for the length field and  1 bit is added for the     */
    /* 0x01 postfix.                                                         */
    padded_length = message_length + 8;
    padded_length = padded_length + 1;

    /* Compute the message length in bits */
    l = message_length * 8;

    /* Compute the total number of blocks */
    num_blocks = padded_length / SHA1_BLOCK_LENGTH;
    block_remainder = padded_length % SHA1_BLOCK_LENGTH;

    if (block_remainder > 0)
    {
        num_blocks++;
    }

    /* Compute total number of bytes */
    num_bytes = num_blocks * SHA1_BLOCK_LENGTH;

    /* Compute the number of padding zeroes required */
    padded_length = padded_length + (SHA1_BLOCK_LENGTH - block_remainder);

     /* Pad with zeroes */
    for (i = message_length; i < num_bytes; i++)
    {
        message[i] = 0x00;
    }

    /* Insert b1 padding bit */
    message[message_length] = 0x80;

    /* Insert message length */
    message[num_bytes - 1] = (UWORD8)( l        & 0xFF);
    message[num_bytes - 2] = (UWORD8)((l >> 8)  & 0xFF);
    message[num_bytes - 3] = (UWORD8)((l >> 16) & 0xFF);
    message[num_bytes - 4] = (UWORD8)((l >> 24) & 0xFF);

    /* Set initial hash state */
    h[0] = 0x67452301;
    h[1] = 0xEFCDAB89;
    h[2] = 0x98BADCFE;
    h[3] = 0x10325476;
    h[4] = 0xC3d2E1F0;

    /* Process the blocks */
    for (i = 0; i < num_blocks; i++)
    {
        /* Prepare the message schedule */
        for (t=0; t < 80; t++)
        {
            if (t < 16)
            {
                block_index = (i * SHA1_BLOCK_LENGTH) + (t*4);

                w[t]  = message[block_index]     << 24;
                w[t] |= message[block_index + 1] << 16;
                w[t] |= message[block_index + 2] << 8;
                w[t] |= message[block_index + 3];
            }
            else if (t < 80)
            {
                w[t] = rotl(1,(w[t-3] ^ w[t-8] ^ w[t-14] ^ w[t-16]));
            }
        }

        /* Initialize the five working variables */
        a = h[0];
        b = h[1];
        c = h[2];
        d = h[3];
        e = h[4];

        /* Iterate a-e 80 times */
        for (t = 0; t < 80; t++)
        {
            temp = (rotl(5,a) + prf_ft(t,b,c,d));
            temp = (temp + e);
            temp = (temp + prf_k(t));
            temp = (temp + w[t]);

            /* Set the values for next iteration */
            e = d;
            d = c;
            c = rotl(30,b);
            b = a;
            a = temp;
        }

        /* Compute the ith intermediate hash value */
        h[0] = (a + h[0]);
        h[1] = (b + h[1]);
        h[2] = (c + h[2]);
        h[3] = (d + h[3]);
        h[4] = (e + h[4]);
    }

    /* Final formation of the digest */
    digest[3]  = (UWORD8) ( h[0]        & 0xFF);
    digest[2]  = (UWORD8) ((h[0] >> 8)  & 0xFF);
    digest[1]  = (UWORD8) ((h[0] >> 16) & 0xFF);
    digest[0]  = (UWORD8) ((h[0] >> 24) & 0xFF);

    digest[7]  = (UWORD8) ( h[1]        & 0xFF);
    digest[6]  = (UWORD8) ((h[1] >> 8)  & 0xFF);
    digest[5]  = (UWORD8) ((h[1] >> 16) & 0xFF);
    digest[4]  = (UWORD8) ((h[1] >> 24) & 0xFF);

    digest[11]  = (UWORD8) ( h[2]        & 0xFF);
    digest[10]  = (UWORD8) ((h[2] >> 8)  & 0xFF);
    digest[9]   = (UWORD8) ((h[2] >> 16) & 0xFF);
    digest[8]   = (UWORD8) ((h[2] >> 24) & 0xFF);

    digest[15]  = (UWORD8) ( h[3]        & 0xFF);
    digest[14]  = (UWORD8) ((h[3] >> 8)  & 0xFF);
    digest[13]  = (UWORD8) ((h[3] >> 16) & 0xFF);
    digest[12]  = (UWORD8) ((h[3] >> 24) & 0xFF);

    digest[19]  = (UWORD8) ( h[4]        & 0xFF);
    digest[18]  = (UWORD8) ((h[4] >> 8)  & 0xFF);
    digest[17]  = (UWORD8) ((h[4] >> 16) & 0xFF);
    digest[16]  = (UWORD8) ((h[4] >> 24) & 0xFF);

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : hmac-sha1                                             */
/*                                                                           */
/*  Description      : This function implements the HMAC-SHA1 keyed secure   */
/*                     hash algorithm.                                       */
/*                                                                           */
/*  Inputs           : 1) Key                                                */
/*                     2) Key length                                         */
/*                     3) Data                                               */
/*                     4) Data length                                        */
/*                     5) Digest                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The details of this algorithm is given in RFC 2104.   */
/*                                                                           */
/*  Outputs          : The digest contains the output.                       */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void hmac_sha1(UWORD8 *key, UWORD32 key_length, UWORD8 *data,
               UWORD32 data_length,UWORD8 *digest)

{
    UWORD32 i           = 0;
    UWORD16 curr_idx    = 0;
    UWORD8  *k0         = 0;  //64
    UWORD8  *k0xorIpad  = 0;  //64
    UWORD8  *step7data  = 0;  //64
    UWORD8  *step5data  = 0;  //MAX_MESSAGE_LENGTH+128
    UWORD8  *step8data  = 0;  //64+20+128

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Allocating memory for temporary variables */
    k0 = (UWORD8 *)scratch_mem_alloc(64);
    if(k0 == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    k0xorIpad = (UWORD8 *)scratch_mem_alloc(64);
    if(k0xorIpad == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    step7data = (UWORD8 *)scratch_mem_alloc(64);
    if(step7data == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    step5data = (UWORD8 *)scratch_mem_alloc(512);//(256 + data_length));
    if(step5data == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    step8data = (UWORD8 *)scratch_mem_alloc((128 + 64 + 20));
    if(step8data == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    /* Initialize the block to all zeroes */
    for (i = 0; i < HMAC_SHA1_BLOCK_LENGTH; i++)
    {
        k0[i] = 0x00;
    }

    /* Initialize the block with key values padded as required */
    if (key_length != HMAC_SHA1_BLOCK_LENGTH) /* Step 1 */
    {
        if (key_length > HMAC_SHA1_BLOCK_LENGTH) /* Step 2 */
        {
            sha1(key, key_length, digest);

            for (i = 0;i < HMAC_DIGEST_LENGTH; i++)
                k0[i] = digest[i];
        }
        else if (key_length < HMAC_SHA1_BLOCK_LENGTH) /* Step 3 */
        {
            for (i = 0; i < key_length; i++)
                k0[i] = key[i];
        }
    }
    else
    {
        for (i = 0;i < HMAC_SHA1_BLOCK_LENGTH; i++)
            k0[i] = key[i];
    }

    /* Step 4 */
    for (i = 0; i < HMAC_SHA1_BLOCK_LENGTH; i++)
        k0xorIpad[i] = k0[i] ^ IPAD;

    /* Step 5 */
    for (i = 0; i < HMAC_SHA1_BLOCK_LENGTH; i++)
        step5data[i] = k0xorIpad[i];

    for (i = 0;i < data_length; i++)
    {
        step5data[i + HMAC_SHA1_BLOCK_LENGTH] = data[i];
    }

    /* Step 6 */
    sha1(step5data,(UWORD32)(data_length + HMAC_SHA1_BLOCK_LENGTH), digest);

    /* Step 7 */
    for (i = 0; i < HMAC_SHA1_BLOCK_LENGTH; i++)
        step7data[i] = k0[i] ^ OPAD;

    /* Step 8 */
    for (i = 0; i < HMAC_SHA1_BLOCK_LENGTH; i++)
        step8data[i] = step7data[i];

    for (i = 0;i < HMAC_DIGEST_LENGTH; i++)
        step8data[i + HMAC_SHA1_BLOCK_LENGTH] = digest[i];

    /* Step 9 */
    sha1(step8data,
            (UWORD32)(HMAC_SHA1_BLOCK_LENGTH + HMAC_DIGEST_LENGTH), digest);

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);

    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : prf                                                   */
/*                                                                           */
/*  Description      : This function calculates the PRF of a key, prefix and */
/*                     data as specified in the 802.11i standard.            */
/*                                                                           */
/*  Inputs           : 1) Key                                                */
/*                     2) Key length                                         */
/*                     3) Prefix                                             */
/*                     4) Prefix length                                      */
/*                     5) Data                                               */
/*                     6) Data length                                        */
/*                     7) Desired result length                              */
/*                     4) Result                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The details of this algorithm is given in Section H.3 */
/*                     of IEEE 802.11i (D10.0) standard.                     */
/*                                                                           */
/*  Outputs          : The result contains the output.                       */
/*                                                                           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void prf(UWORD8 *key, UWORD32 key_length, UWORD8 *prefix,
         UWORD32 prefix_length, UWORD8 *data, UWORD32 data_length,
         UWORD32 result_length, UWORD8 *result)
{
    UWORD32 r_length          = 0;
    UWORD32 r_length_blocks   = 0;
    UWORD32 input_data_length = 0;
    UWORD32 i                 = 0;
    UWORD32 j                 = 0;
    UWORD32 length_in_octets  = 0;
    UWORD16 curr_idx          = 0;

    UWORD8  *r               = 0;//[MAX_MESSAGE_LENGTH*2+128];
    UWORD8  *hmac_sha_result = 0;//[HMAC_DIGEST_LENGTH];
    UWORD8  *input_data      = 0;//[MAX_MESSAGE_LENGTH*2+128];

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Allocating memory for temporary variables */
    r = (UWORD8 *)scratch_mem_alloc(640);//(128 + (data_length * 2)));
    if(r == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    hmac_sha_result = (UWORD8 *)scratch_mem_alloc(HMAC_DIGEST_LENGTH);
    if(hmac_sha_result == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }

    input_data = (UWORD8 *)scratch_mem_alloc(640);//(128 + (data_length * 2)));
    if(input_data == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return;
    }


    r_length = 0;
    r_length_blocks = ((result_length + (HMAC_SHA_RESULT_LENGTH - 1))/
                       HMAC_SHA_RESULT_LENGTH);

    for (i = 0; i < r_length_blocks; i++)
    {
        /* Prepare the chunk of data with prefix, zeroes and data */
        for (j = 0;j < prefix_length; j++)
            input_data[j] = prefix[j];

        input_data[prefix_length] = 0x00;

        for (j = 0;j < data_length; j++)
            input_data[prefix_length+j+1] = data[j];

        input_data_length = prefix_length + data_length + 1;

        input_data[input_data_length] = (UWORD8)i;
        input_data_length++;
        input_data[input_data_length] = 0x00;

        /* Perform HMAC-SHA1 to obtain the HMAC digest */
        hmac_sha1(key, key_length,
            input_data, input_data_length, hmac_sha_result);

        /* Save the 160 bits of HMAC digest in the key */
        for (j = 0; j < HMAC_DIGEST_LENGTH; j++)
            r[r_length + j] = hmac_sha_result[j];

        /* Increment the result index */
        r_length = r_length + HMAC_DIGEST_LENGTH;
    }

    /* Copy the result to the given buffer */
    length_in_octets = result_length / 8;
    for (i = 0;i < length_in_octets; i++)
        result[i] = r[i];

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);
}

#endif /* UTILS_11I */
#endif /* MAC_802_11I */
