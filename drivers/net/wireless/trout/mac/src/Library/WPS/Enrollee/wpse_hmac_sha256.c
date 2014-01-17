/*****************************************************************************/
/*                                                                           */
/*                     Ittiam WPS Supplicant SOFTWARE                        */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wpse_hmac_sha256.c                                   */
/*                                                                           */
/*  Description       : This file contains the functions to implement the    */
/*                      HMAC-SHA-256 key hash algorithm and SHA-256 hash     */
/*                      algorithm as  defined in RFC-2104 and FIPS 180-2     */
/*                                                                           */
/*  List of Functions : hmac_sha256                                          */
/*                      sha256                                               */
/*                      process_sha256_block                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         11 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_ENR_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_hmac_sha256.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* SHA-256 Constants: Sixty-four 32-bit words,K array                        */
/*****************************************************************************/
static const UWORD32 K[SHA256_BLOCK_SIZE] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static void process_sha256_block(UWORD32 *state, UWORD8 *buf);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : hmac_sha256                                           */
/*                                                                           */
/*  Description      : This function implements implement the HAMC-SHA-256   */
/*                     key hash algorithm as defined in FIPS 180-2 & RFC-2104*/
/*                                                                           */
/*  Inputs           : 1) Pointer to the key used by HMAC algorithm          */
/*                     2) Input key length                                   */
/*                     3) Array of pointers pointing to fragmented messages  */
/*                        to be hashed                                       */
/*                     4) Array storing length of the input messages         */
/*                     5) Number of input message buffers                    */
/*                     6) Pointer to the location where output hash of the   */
/*                        messages is to be stored. This array can overlap   */
/*                        with the input message array or key                */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function takes key and maximum MAX_HMAC_FRAG_CNT */
/*                     numbers of fragment messages and perform the          */
/*                     HMAC-SHA256 key hash algorithm on messages using the  */
/*                     key. Output is stored at the buffer pointed by the    */
/*                     pointer passed as input.                              */
/*                     As per RFC-2104 and FIPS 180-2, HMAC SHA256 key hash  */
/*                     algorithm  is as follows:                             */
/*                     If(key length > SHA_256_HASH_LEN) then                */
/*                       key = SHA256(Key) and key_length = SHA_256_HASH_LEN */
/*                     Key hash output =                                     */
/*                        SHA256(Key XOR opad, SHA256(Key XOR ipad, message))*/
/*                     where ipad and opad are 8-bit constants repeated      */
/*                     SHA256_BLOCK_SIZE times                               */
/*                                                                           */
/*  Outputs          : HMAC-SHA256 key hash of the input fragmented message  */
/*  Returns          : Status of the function processing                     */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         11 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
//#pragma Otime
STATUS_T hmac_sha256(UWORD8 *key, UWORD16 key_length, UWORD8 *msg[],
                     UWORD16 *msg_len, UWORD8 frag_cnt, UWORD8 *mac)
{
    UWORD8 k_xor[SHA256_BLOCK_SIZE];
    UWORD8 *sha256_msg[MAX_HMAC_FRAG_CNT+1];
    UWORD16 sha256_msg_len[MAX_HMAC_FRAG_CNT+1];
    UWORD8 cnt;

    /*************************************************************************/
    /* If more than the expected fragments then return Failure               */
    /*************************************************************************/
    if(MAX_HMAC_FRAG_CNT < frag_cnt)
    {
        return FAILURE;
    }

    /*************************************************************************/
    /* If(key length > SHA_256_HASH_LEN) then key = SHA256(Key) and          */
    /*      key_length = SHA_256_HASH_LEN                                    */
    /* k_xor = Key XOR ipad, where Key is assumed to be padded till          */
    /*         SHA256_BLOCK_SIZE                                             */
    /*************************************************************************/
    memset(k_xor, IPAD, SHA256_BLOCK_SIZE);
    if (key_length > SHA256_BLOCK_SIZE)
    {
        sha256(&key,&key_length, 1,k_xor);
        key_length = SHA_256_HASH_LEN;
        for (cnt = 0; cnt < SHA_256_HASH_LEN; cnt++)
        {
            k_xor[cnt] ^= IPAD;
        }
    }
    else
    {
        for (cnt = 0; cnt < key_length; cnt++)
        {
            k_xor[cnt] ^= *key++;
        }
    }

    /*************************************************************************/
    /* Intermediate Key hash output = SHA256(k_xor, message)                 */
    /* Append k_xor address and k_xor length to the input fragment messages  */
    /* address and length array                                              */
    /*************************************************************************/
    sha256_msg[0] = k_xor;
    sha256_msg_len[0] = SHA256_BLOCK_SIZE;
    for (cnt = 1; cnt < frag_cnt+1; cnt++) {
        sha256_msg[cnt] = *msg++;
        sha256_msg_len[cnt] = *msg_len++;
    }
    sha256(sha256_msg, sha256_msg_len, cnt,mac);

    /*************************************************************************/
    /* k_xor = Key XOR opad, where Key is assumed to be padded till          */
    /*         SHA256_BLOCK_SIZE                                             */
    /* As k_xor already has zero padded key XORed with ipad, XORing it again */
    /* (ipad XOR opad) will result in values of k_or as mentioned above.     */
    /* (a XOR b) XOR (b XOR c) = (a) XOR (b XOR b) XOR (c)                   */
    /*  = (a) XOR 0 XOR (c) = (a XOR c)                                      */
    /*************************************************************************/
    for (cnt = 0; cnt < SHA256_BLOCK_SIZE; cnt++)
    {
        k_xor[cnt] ^= (IPAD ^ OPAD);
    }

    /*************************************************************************/
    /* output hash = SHA256(Key XOR opad, SHA256(Key XOR ipad, message))     */
    /* mac already has intermediate Key hash = SHA256(Key XOR ipad, message))*/
    /* Therefore  output hash mac = SHA256(k_xor, mac)                       */
    /*************************************************************************/
    sha256_msg[0] = k_xor;
    sha256_msg_len[0] = SHA256_BLOCK_SIZE;
    sha256_msg[1] = mac;
    sha256_msg_len[1] = SHA_256_HASH_LEN;
    sha256(sha256_msg, sha256_msg_len, 2,mac);

    return SUCCESS;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : sha256                                                */
/*                                                                           */
/*  Description      : This function implements implement the SHA-256 hash   */
/*                     algorithm as defined in  FIPS 180-2                   */
/*                                                                           */
/*  Inputs           : 1) Array of pointers pointing to fragmented messages  */
/*                        to be hashed                                       */
/*                     2) Array storing length of the input messages         */
/*                     3) Number of input message buffers                    */
/*                     4) Pointer to the location where output hash of the   */
/*                        messages is to be stored. This array can overlap   */
/*                        with the input message array                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function takes maximum MAX_HMAC_FRAG_CNT numbers */
/*                     of fragment messages and per from the SHA 256 hash    */
/*                     algorithm on the same. Output is stored at the buffer */
/*                     pointed by the pointer passed as input. Proceeding of */
/*                     input messages is done in blocks of length equal to   */
/*                     one SHA-256 block. If in some message fragment if     */
/*                     data remaining is less than the block length then     */
/*                     that data is stored in a local buffer. Data from next */
/*                     message fragment is copied to this local buffer to    */
/*                     form a complete block and then this block is hashed.  */
/*                     When the last message is processed the data in local  */
/*                     buffer is padded and total message length is added    */
/*                     before generating hash of this data. States of        */
/*                     SHA-256 are maintained across different blocks and    */
/*                     message fragments.                                    */
/*                                                                           */
/*  Outputs          : SHA256 hash of the input fragmented message           */
/*  Returns          : Status of the function processing                     */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         09 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
//#pragma Otime
STATUS_T sha256(UWORD8 *msg[], UWORD16 *msg_len, UWORD8 frag_cnt, UWORD8 *mac)
{
    UWORD8 *msg_ptr;
    UWORD32 state[8];
    UWORD32 length;
    UWORD16 temp_len;
    UWORD16 msg_len_rem;
    UWORD8  local_buff[SHA256_BLOCK_SIZE];
    UWORD8  local_buff_len;
    UWORD8  cnt;

    /*************************************************************************/
    /* If more than the expected fragments then return Failure               */
    /*************************************************************************/
    if(MAX_SHA_256_FRAG_CNT < frag_cnt)
    {
        return FAILURE;
    }

    local_buff_len = 0;
    length = 0;
    /*************************************************************************/
    /* Initialize the SHA-256 States                                         */
    /*************************************************************************/
    state[0] = (UWORD32)0x6A09E667;
    state[1] = (UWORD32)0xBB67AE85;
    state[2] = (UWORD32)0x3C6EF372;
    state[3] = (UWORD32)0xA54FF53A;
    state[4] = (UWORD32)0x510E527F;
    state[5] = (UWORD32)0x9B05688C;
    state[6] = (UWORD32)0x1F83D9AB;
    state[7] = (UWORD32)0x5BE0CD19;

    /*************************************************************************/
    /* Process input messages one by one                                     */
    /*************************************************************************/
    for (cnt = 0; cnt < frag_cnt; cnt++)
    {
        msg_ptr = (UWORD8*) msg[cnt];
        msg_len_rem = msg_len[cnt];

        /*********************************************************************/
        /* Process input message if still some data is left                  */
        /*********************************************************************/
        while (msg_len_rem > 0)
        {
            /*****************************************************************/
            /* If local buffer does not have any data that directly start the*/
            /* hash from the input buffer.                                   */
            /* else copy data from the input buffer to the local buffer till */
            /* the local buffer have data equal to one SHA-256 block. If     */
            /* local buffer becomes full (i.e data length == 1 SHA-256 block)*/
            /* then do the hash on the local buffer data.                    */
            /*****************************************************************/
            if ((0 == local_buff_len) && (SHA256_BLOCK_SIZE <= msg_len_rem))
            {
                process_sha256_block(state, (UWORD8 *) msg_ptr);
                length += SHA256_BLOCK_SIZE * 8;
                msg_ptr += SHA256_BLOCK_SIZE;
                msg_len_rem -= SHA256_BLOCK_SIZE;
            }
            else
            {
                temp_len = MIN(msg_len_rem, (UWORD32)(SHA256_BLOCK_SIZE -
                    local_buff_len));
                memcpy(local_buff + local_buff_len, msg_ptr, temp_len);
                local_buff_len += temp_len;
                msg_ptr += temp_len;
                msg_len_rem -= temp_len;
                if (local_buff_len == SHA256_BLOCK_SIZE)
                {
                    process_sha256_block(state, local_buff);
                    length += 8 * SHA256_BLOCK_SIZE;
                    local_buff_len = 0;
                }
            }
        } /* while (msg_len_rem > 0) */
    } /* for (cnt = 0; cnt < frag_cnt; cnt++) */

    /*************************************************************************/
    /* All the messages have been processed now do the processing on the     */
    /* data remaining in the local buffer. Do the padding and length field   */
    /* addition as defined in the SHA-256 Standard.                          */
    /*************************************************************************/
    length += local_buff_len * 8;

    /*************************************************************************/
    /* Append the bit '1' to the end of the message                          */
    /*************************************************************************/
    local_buff[local_buff_len++] = (UWORD8) 0x80;

    /*************************************************************************/
    /* If the length of the data in local buffer is more than                */
    /* SHA256_BLOCK_SIZE-SHA256_LEN_FIELD_SIZE i.e. there is not enough space*/
    /* for the length field than pad the data in local buffer up to          */
    /* SHA256_BLOCK_SIZE and perform Hash operation.                         */
    /*************************************************************************/
    if (local_buff_len > (SHA256_BLOCK_SIZE - SHA256_LEN_FIELD_SIZE))
    {
        while (local_buff_len < SHA256_BLOCK_SIZE)
        {
            local_buff[local_buff_len++] = (UWORD8) 0;
        }
        process_sha256_block(state, local_buff);
        local_buff_len = 0;
    }

    /*************************************************************************/
    /* Pad the data in the local buffer up to SHA256_BLOCK_SIZE -            */
    /* SHA256_LEN_FIELD_SIZE                                                 */
    /*************************************************************************/
    while (local_buff_len < (SHA256_BLOCK_SIZE - SHA256_LEN_FIELD_SIZE))
    {
        local_buff[local_buff_len++] = (UWORD8) 0;
    }

    /*************************************************************************/
    /* Store length filed. It is assumed that maximum length is <2^32, hence */
    /* upper 4 bytes of length field are always zero.                        */
    /*************************************************************************/
    msg_ptr = local_buff + (SHA256_BLOCK_SIZE-SHA256_LEN_FIELD_SIZE);
    *msg_ptr++ =(UWORD8)0;
    *msg_ptr++ =(UWORD8)0;
    *msg_ptr++ =(UWORD8)0;
    *msg_ptr++ =(UWORD8)0;

    *msg_ptr++ =(UWORD8) ((((UWORD32) (length)) >> 24) & 0xff);
    *msg_ptr++ =(UWORD8) ((((UWORD32) (length)) >> 16) & 0xff);
    *msg_ptr++ =(UWORD8) ((((UWORD32) (length)) >> 8) & 0xff);
    *msg_ptr++ =(UWORD8) (((UWORD32) (length)) & 0xff);

    process_sha256_block(state, local_buff);

    /*************************************************************************/
    /* Copy the hash output stored in state variables to the output buffer.  */
    /* Variable 'length' is used as temporary variable to store the data to  */
    /* avoid access to array through pointer and index.                      */
    /*************************************************************************/
    for (cnt = 0; cnt < 8; cnt++)
    {
        length = state[cnt];
        *mac++ =(UWORD8) ((((UWORD32) (length)) >> 24) & 0xff);
        *mac++ =(UWORD8) ((((UWORD32) (length)) >> 16) & 0xff);
        *mac++ =(UWORD8) ((((UWORD32) (length)) >> 8) & 0xff);
        *mac++ =(UWORD8) (((UWORD32) (length)) & 0xff);
    }
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : process_sha256_block                                  */
/*                                                                           */
/*  Description      : This function do performs SHA-256 algorithm on a      */
/*                     algorithm as defined in  FIPS 180-2                   */
/*                                                                           */
/*  Inputs           : 1) Pointer to the state of SHA-256 algorithm          */
/*                     2) Pointer to the input data buffer of length         */
/*                        SHA256_BLOCK_SIZE                                  */
/*                                                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function takes one SHA256_BLOCK_SIZE block of    */
/*                     data and state variables of SHA-256 algorithm and     */
/*                     calculates the hash of input data. State variables are*/
/*                     updated with this hash.                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Updated State variables with the hash of input message*/
/*                     block                                                 */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         09 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
//#pragma Otime
void process_sha256_block(UWORD32 *state, UWORD8 *buf)
{
    UWORD32 work_var[8], W[64], T1, T2;
    UWORD8  cnt;

    /*************************************************************************/
    /* Copy SHA-256 states into working variables                            */
    /*************************************************************************/
    for (cnt = 0; cnt < 8; cnt++)
    {
        work_var[cnt] = state[cnt];
    }

    /*************************************************************************/
    /* Prepare the Message Schedule W(0-15) by copying 64 bytes from input   */
    /* message and W(16-63) by using the equation mentioned in standard.     */
    /*************************************************************************/
    for (cnt = 0; cnt < 16; cnt++)
    {
        W[cnt] = ((((UWORD32) buf[0]) << 24) | (((UWORD32) buf[1]) << 16) | \
             (((UWORD32) buf[2]) << 8) | ((UWORD32) buf[3]));
        buf += 4;
    }
    for (cnt = 16; cnt < 64; cnt++)
    {
        W[cnt] = SIGMA1(W[cnt - 2]) + W[cnt - 7] + SIGMA0(W[cnt - 15]) +
            W[cnt - 16];
    }

    /*************************************************************************/
    /* Perform 64 rounds of Hashing as mentioned in step 3 of "SHA-256 Hash  */
    /* Computation" mentioned in the standard.                               */
    /*************************************************************************/
    for (cnt = 0; cnt < 64; ++cnt)
    {
        RND(work_var[0], work_var[1], work_var[2], work_var[3], work_var[4],
            work_var[5], work_var[6], work_var[7], T1, T2, K, W, cnt);
    }

    /*************************************************************************/
    /* Compute intermediate hash values as mentioned in step 4 of "SHA-256   */
    /* Hash Computation" mentioned in the standard.                          */
    /*************************************************************************/
    for (cnt = 0; cnt < 8; cnt++)
    {
        state[cnt] += work_var[cnt];
    }
}
#endif /* INT_WPS_ENR_SUPP */
