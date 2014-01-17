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
/*  File Name         : wpse_dh_key.c                                        */
/*                                                                           */
/*  Description       : This file contains the functions to implement the    */
/*                      Diffie-Hellman keys generation as defined in WPS     */
/*                      standard  version v.1.2.2                            */
/*                                                                           */
/*  List of Functions : wps_check_msb_pos                                    */
/*                      wps_mult_mod_p                                       */
/*                      wps_dh_core                                          */
/*                      wps_gen_pke                                          */
/*                      wps_gen_ph_dh_key                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_ENR_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_dh_key.h"
/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
/*****************************************************************************/
/* WPS DH Prime: 1536 bit prime. it is assumed that MSB is 1. So if prime is */
/* changed and most significant word is < 0x80000000, the algo will need     */
/* change in calculating x mod P. First word is the least significant word   */
/*****************************************************************************/

static const UWORD32 wps_prime[WPS_DH_PRIME_LEN_WORD32] =
{
    0xFFFFFFFF, 0xFFFFFFFF, 0xCA237327, 0xF1746C08, 0x4ABC9804, 0x670C354E,
    0x7096966D, 0x9ED52907, 0x208552BB, 0x1C62F356, 0xDCA3AD96, 0x83655D23,
    0xFD24CF5F, 0x69163FA8, 0x1C55D39A, 0x98DA4836, 0xA163BF05, 0xC2007CB8,
    0xECE45B3D, 0x49286651, 0x7C4B1FE6, 0xAE9F2411, 0x5A899FA5, 0xEE386BFB,
    0xF406B7ED, 0x0BFF5CB6, 0xA637ED6B, 0xF44C42E9, 0x625E7EC6, 0xE485B576,
    0x6D51C245, 0x4FE1356D, 0xF25F1437, 0x302B0A6D, 0xCD3A431B, 0xEF9519B3,
    0x8E3404DD, 0x514A0879, 0x3B139B22, 0x020BBEA6, 0x8A67CC74, 0x29024E08,
    0x80DC1CD1, 0xC4C6628B, 0x2168C234, 0xC90FDAA2, 0xFFFFFFFF, 0xFFFFFFFF
};
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_check_msb_pos                                     */
/*                                                                           */
/*  Description      : This function check the most significant bit position */
/*                     of a number stored in structure of type               */
/*                     long_num_struct_t                                     */
/*                                                                           */
/*  Inputs           : 1) Pointer to the structure storing the big number    */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This reads the words of the big number starting with  */
/*                     the most significant word till it finds a non zero    */
/*                     word. Then it finds the MSB position within the word  */
/*                     and return the overall MSB based on the word index    */
/*                     & MSB position within the word. the count starts from */
/*                     one i.e. if value of the big number is '1' then MSB   */
/*                     return is 1.                                          */
/*                                                                           */
/*  Outputs          : MSB position of the big number                        */
/*  Returns          : MSB position of the big number                        */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifndef OS_LINUX_CSL_TYPE
#pragma arm
#pragma Otime
#endif /* OS_LINUX_CSL_TYPE */
UWORD16 wps_check_msb_pos(long_num_struct_t* inp)
{
    UWORD16 msb_pos;
    UWORD32 *data_ptr;
    UWORD32 temp;
    UWORD8 bit_pos=0;
    /*************************************************************************/
    /* Find first non zero word. Search starts from most significant word    */
    /*************************************************************************/
    msb_pos = (UWORD16)(inp->length) << (UWORD8) (5);
    data_ptr = inp->val_ptr + inp->length -1;
    while(0 == *data_ptr--)
    {
        msb_pos -= 32;
        if(0 == msb_pos)
        {
            return(msb_pos);
        }
    }

    /*************************************************************************/
    /* Find first non zero bit in the first non zero word                    */
    /*************************************************************************/
    temp = data_ptr[1];
    if (temp & (UWORD32)0xFFFF0000)
    {
        temp >>= 16;
        bit_pos = 16;
    }
    if (temp & 0xFF00)
    {
        temp >>= 8;
        bit_pos |= 8;
    }
    if (temp & 0xF0)
    {
        temp >>= 4;
        bit_pos |= 4;
    }
    if (temp & 0xC)
    {
        temp >>= 2;
        bit_pos |= 2;
    }
    if (temp & 0x2)
    {
        bit_pos |= 1;
    }
    /*************************************************************************/
    /* Based on the word index and bit position of MSB in first non zero word*/
    /* return the MSB position in the big number                             */
    /*************************************************************************/
    return ((UWORD16) msb_pos - 31 + bit_pos);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_mult_mod_p                                        */
/*                                                                           */
/*  Description      : This function multiply two big numbers and then finds */
/*                     mod of the output using the pre-stored prime.         */
/*                                                                           */
/*  Inputs           : 1) Pointer to the structure storing the First big     */
/*                        number of size WPS_DH_PRIME_LEN_WORD32             */
/*                     2) Pointer to the structure storing the Second big    */
/*                        number of size WPS_DH_PRIME_LEN_WORD32             */
/*                     3) Pointer to temporary buffer of size                */
/*                        WPS_DH_PRIME_LEN_WORD32*2                          */
/*                                                                           */
/*  Globals          : wps_prime: Table storing the prime.                   */
/*  Processing       : This function first multiples the two big numbers and */
/*                     store the out put in the temporary buffer passed as   */
/*                     input. It then finds the mod of the output w.r.t. the */
/*                     prime by dividing the multiplied output with the prime*/
/*                     Remainder of the division is copied to the first no   */
/*                                                                           */
/*  Outputs          : (number1 * number2) mod p                             */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifndef OS_LINUX_CSL_TYPE
#pragma Otime
#endif /* OS_LINUX_CSL_TYPE */
void wps_mult_mod_p(long_num_struct_t* num1,
                         long_num_struct_t* num2,
                         UWORD32* temp_mult_buff)
{
    long_num_struct_t prime, temp_mult_long;
    register UWORD64 temp_64bit_1;
    UWORD32 *n1_ptr;
    UWORD32 *n2_ptr;
    UWORD32 *temp_mult_ptr;
    register UWORD32 n2;
#ifdef NO_PROCESS_32BIT_MULT
    register UWORD32 n1;
#endif
    UWORD16 cnt1,cnt2, offset;
    UWORD16 num1_word_len, num2_word_len;
    UWORD8 carry;
    num2_word_len = GET_LONG_WORD_LEN(num2);
    num1_word_len = GET_LONG_WORD_LEN(num1);
    temp_mult_long.val_ptr = temp_mult_buff;
    temp_mult_long.length = WPS_DH_PRIME_LEN_WORD32*2;

    memset((void *) temp_mult_buff, 0,
        temp_mult_long.length * sizeof(UWORD32));

    /*************************************************************************/
    /* Multiply num1 and num2. Each word in num2 is multiplied by each word  */
    /* in num1 and the result shifted by 32 * (num2 word count + num1 word   */
    /* count) is added to the output buffer pointer by temp_mult_buff. This  */
    /* buffer is initialized to zero before starting the multiplication.     */
    /* The outer loop is to get one word at a time from num2 and inner loop  */
    /* is to multiply the chosen word from num2 with each word in num1       */
    /*************************************************************************/
    for(cnt1 = 0; cnt1 < num2_word_len; cnt1++)
    {
        n1_ptr = num1->val_ptr;
        n2 = *(num2->val_ptr + cnt1);
        for(cnt2 = 0; cnt2 < num1_word_len; cnt2++)
        {
            temp_mult_ptr = temp_mult_buff + cnt1 + cnt2;
            /*****************************************************************/
            /* If Processor 32*32 bit multiplication (with 64 bit output) is */
            /* not optimal then do the multiplication using 16 bit numbers   */
            /*****************************************************************/
#ifdef NO_PROCESS_32BIT_MULT
            n1 = *n1_ptr++;
            temp_64bit_1 = (UWORD32)((UWORD32)((UWORD16)n1)*
                (UWORD32)((UWORD16)n2));
            temp_64bit_1 += ((UWORD64)((UWORD32)((UWORD16)n1)*
                (UWORD32)(n2>>16)) << 16);
            n1 = n1 >>16;
            temp_64bit_1 += ((UWORD64)((UWORD32)(n1)*
                (UWORD32)((UWORD16)n2)) << 16);
            temp_64bit_1 += ((UWORD64)((UWORD32)(n1)*
                (UWORD32)(n2>>16)) << 32);
#else
            temp_64bit_1 = (UWORD64)((UWORD64) n2 * (UWORD64) (*n1_ptr++));
#endif
            {
                UWORD32 temp32 = *temp_mult_ptr;
                temp_64bit_1 = temp_64bit_1 + temp32;
                *temp_mult_ptr++ = (UWORD32)temp_64bit_1;
                temp32 = *temp_mult_ptr;
                *temp_mult_ptr = temp32 + (UWORD32)(temp_64bit_1 >>  32);
                if(temp32 > *temp_mult_ptr)
                {
                    temp_mult_ptr++;
                    do
                    {
                        temp32 = *temp_mult_ptr + 1;
                        *temp_mult_ptr++ = temp32;
                    }
                    while(0 == temp32);
                }
            }
        }
    }

    /*************************************************************************/
    /* Prepare long number structure for prime. values are hardcoded to save */
    /* processing                                                            */
    /*************************************************************************/
    prime.val_ptr = (UWORD32 *) wps_prime;
    prime.length = WPS_DH_PRIME_LEN_WORD32; //(sizeof(P)/ sizeof(UWORD32));
    prime.msb_pos = WPS_DH_PRIME_MSB_POS; //wps_check_msb_pos(&prime);

    /*************************************************************************/
    /* Do temp_mult_long = temp_mult_long mod P                              */
    /*************************************************************************/
    temp_mult_long.msb_pos = wps_check_msb_pos(&temp_mult_long);
    while(LONG_NUM_SMALLER != wps_cmp_num(temp_mult_long, prime))
    {
        offset = (temp_mult_long.msb_pos - (WPS_DH_PRIME_MSB_POS - 31)) >> 5;
        n2_ptr = (UWORD32 *) wps_prime;
        n1_ptr = temp_mult_buff + offset;
        /*********************************************************************/
        /* If both numbers have equal number of bits then shift is not       */
        /* required, also subtract only WPS_DH_PRIME_LEN_WORD32 number of    */
        /* words and not WPS_DH_PRIME_LEN_WORD32+1                           */
        /*********************************************************************/
        if(offset)
        {
            offset = (UWORD16) (temp_mult_long.msb_pos & (UWORD16)(0x1F));
            offset = offset ? (offset-1):0x1F;
            n1_ptr--;
        }
        carry = 0;
        temp_64bit_1 = 0;
        /*********************************************************************/
        /* Subtract the prime from remainder of the multiplied output and    */
        /* overwrite the multiplied output. Only upper                       */
        /* WPS_DH_PRIME_LEN_WORD32+ (0/1) words are subtracted               */
        /*********************************************************************/
        if(offset)
        {
            UWORD32 temp32 = 0;
            UWORD8  offset1 = 32 - offset;
            for(cnt1 = 0; cnt1 < WPS_DH_PRIME_LEN_WORD32; cnt1++)
            {
                UWORD32 temp32_1;
                temp32_1 = *n2_ptr++;
                temp32 |= (temp32_1 << offset);
                n2 = *n1_ptr;
                *n1_ptr++ = temp32 = n2 - temp32 - carry;
                carry = (temp32 != n2)? (temp32 > n2) : carry;
                temp32 = temp32_1 >> offset1;

            }
            //if(WPS_DH_PRIME_MSB_POS != temp_mult_long.msb_pos)
            {
                *n1_ptr -= temp32 + carry;
            }

        }
        else
        {
            for(cnt1 = 0; cnt1 < WPS_DH_PRIME_LEN_WORD32; cnt1++)
            {
                UWORD32 temp32;
                n2 = *n1_ptr;
                *n1_ptr++ = temp32 = n2 - *n2_ptr++ - carry;
                carry = (temp32 != n2)? (temp32 > n2) : carry;
            }
            if(WPS_DH_PRIME_MSB_POS != temp_mult_long.msb_pos)
            {
                *n1_ptr -= (UWORD32) carry;
            }
    }
        temp_mult_long.msb_pos = wps_check_msb_pos(&temp_mult_long);

    }
    /*************************************************************************/
    /* Copy the output stored in temp_mult_long to num1                      */
    /*************************************************************************/
    memcpy(num1->val_ptr, temp_mult_buff, WPS_DH_PRIME_LEN_WORD32 * 4);
    num1->msb_pos = temp_mult_long.msb_pos;

    return;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_dh_core                                           */
/*                                                                           */
/*  Description      : This function generates Diffie-Hellman key.           */
/*                                                                           */
/*  Inputs           : 1) Pointer to the structure storing the generator     */
/*                     2) Pointer to the structure storing the exponent      */
/*                     3) Pointer to the structure where DH Key should be    */
/*                        stored                                             */
/*                     4) Pointer to temporary buffer of size                */
/*                        WPS_DH_PRIME_LEN_WORD32*2                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates the DH Key  =                 */
/*                     ((generator ^ exponent) mod prime)                    */
/*                     It implements the above equation using the algorithm  */
/*                     described below                                       */
/*                     G = (g mod p);                                        */
/*                     current_mod_value =0;                                 */
/*                     if(b0 == 1)                                           */
/*                     {                                                     */
/*                       mod_value_assigned = TRUE;                          */
/*                       current_mod_value = G;                              */
/*                     }                                                     */
/*                     for (i=1; i<= m; i++)                                 */
/*                     {                                                     */
/*                       G = (GxG) mod p;                                    */
/*                       if(bi == 1)                                         */
/*                       {                                                   */
/*                         if(mod_value_assigned == TRUE)                    */
/*                         {                                                 */
/*                           current_mod_value =(G* current_mod_value) mod p;*/
/*                         }                                                 */
/*                         else                                              */
/*                         {                                                 */
/*                           mod_value_assigned = TRUE;                      */
/*                           current_mod_value = G;                          */
/*                         }                                                 */
/*                        }                                                  */
/*                     }                                                     */
/*                     ((generator ^ exponent) mod prime)= current_mod_value;*/
/*                     where bx is xth bit in exponent where b0 is the least */
/*                     significant                                           */
/*                     g is generator and p is prime.                        */
/*                                                                           */
/*  Outputs          : DH key = ((generator ^ exponent) mod prime)           */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_dh_core(long_num_struct_t* gen_ptr,
                 long_num_struct_t* exp_ptr,
                 long_num_struct_t* dh_key_ptr,
                 UWORD32* temp_mult_buff)
{
    UWORD32 *a_ptr, *current_mod_value;
    UWORD32 current_word;
    UWORD16 word_cnt, bit_cnt, bit_len;
    BOOL_T  mod_value_assigned = BFALSE;

    current_mod_value = dh_key_ptr->val_ptr;
    /*************************************************************************/
    /* G = (g mod p). In this step finding mode is not necessary as it can be*/
    /* assumed that g < p                                                    */
    /* current_mod_value = 0; This is also not necessary as one of the bit   */
    /* in must be one. If all bits are zero then Dh Key = 1                  */
    /*************************************************************************/
    if(exp_ptr->msb_pos == 0)
    {
        memset((void *) current_mod_value, 0,
            WPS_DH_PRIME_LEN_WORD32 * sizeof(UWORD32));
        *current_mod_value = 1;
        dh_key_ptr->msb_pos = 1;
        return;
    }

    word_cnt = GET_LONG_WORD_LEN(exp_ptr);
    a_ptr = exp_ptr->val_ptr;
    current_word = *a_ptr++;
    /*************************************************************************/
    /* if(b0 == 1)                                                           */
    /* {                                                                     */
    /*      mod_value_assigned = TRUE;                                       */
    /*      current_mod_value = G;                                           */
    /* }                                                                     */
    /*************************************************************************/
    if(current_word & 0x1)
    {
        mod_value_assigned = BTRUE;
        memcpy((void*)current_mod_value, (const void*)gen_ptr->val_ptr,
            WPS_DH_PRIME_LEN_WORD32 * sizeof(UWORD32));
        dh_key_ptr->msb_pos = gen_ptr->msb_pos;
    }
    /*************************************************************************/
    /* First bit is already processed, push it out and also remaining bit in */
    /* current word = minimum (total bits -1, 31).                           */
    /*************************************************************************/
    current_word = current_word >> 1;
    bit_len = exp_ptr->msb_pos;
    bit_cnt = MIN(bit_len-1,31);
    /*************************************************************************/
    /*  for (i=1; i<= total number of valid bits in A nonce; i++)            */
    /*  {                                                                    */
    /*      G = (GxG) mod p;                                                 */
    /*      if(bi == 1)                                                      */
    /*      {                                                                */
    /*          if(mod_value_assigned == TRUE)                               */
    /*          {                                                            */
    /*               current_mod_value = (G* current_mod_value) mod p;       */
    /*          }                                                            */
    /*          else                                                         */
    /*          {                                                            */
    /*              mod_value_assigned = TRUE;                               */
    /*              current_mod_value = G;                                   */
    /*          }                                                            */
    /*      }                                                                */
    /*  }                                                                    */
    /*  ((g^A)mod p) = current_mod_value;                                    */
    /*  Outer loop is word count loop, it is used to avoid accessing from the*/
    /*  memory same word multiple times and then masking the right bit from  */
    /*  it. Word count check is done later to avoid reading from outside the */
    /*  A nonce buffer                                                       */
    /*************************************************************************/
    while(1)
    {
        while(bit_cnt--)
        {
            wps_mult_mod_p(gen_ptr, gen_ptr, temp_mult_buff);
            if(current_word & 0x1)
            {
                if(mod_value_assigned == BTRUE)
                {
                    wps_mult_mod_p(dh_key_ptr, gen_ptr, temp_mult_buff);
                }
                else
                {
                    mod_value_assigned = BTRUE;
                    memcpy((void*)current_mod_value,
                        (const void*)gen_ptr->val_ptr,
                        WPS_DH_PRIME_LEN_WORD32 * sizeof(UWORD32));
                    dh_key_ptr->msb_pos = gen_ptr->msb_pos;
                }
            }
            current_word = current_word >> 1;
        } /* while(bit_cnt--) */
        /*********************************************************************/
        /* Current word is finished, Check if more words are remaining. If   */
        /* so read next word and decrement word count and bit len and        */
        /* initialize the current word bit count                             */
        /*********************************************************************/
        word_cnt--;
        if(0 == word_cnt)
        {
            break;
        }
        current_word = *a_ptr++;
        bit_len -= 32;
        bit_cnt = MIN(bit_len,32);
    } /* while(1) */
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_pke                                           */
/*                                                                           */
/*  Description      : This function generates Diffie-Hellman public and     */
/*                     private key                                           */
/*                                                                           */
/*  Inputs           : 1) Pointer to the structure where DH Public Key       */
/*                        should be stored                                   */
/*                     2) Pointer to the structure where DH Private Key      */
/*                        should be stored                                   */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function first generates the public key using    */
/*                     random nub eras and then generates the public key     */
/*                     = (g^private Key) mod p) where g =2, p  is predefined */
/*                     prime. It then converts the public key in byte format */
/*                     Public key, Private key and Prime are all of  length  */
/*                     WPS_DH_PRIME_LEN_WORD32.                              */
/*                                                                           */
/*  Outputs          : PKE =  of (g^private Key) mod p)                      */
/*  Returns          : Status of the function processing                     */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_gen_pke(UWORD32* pke_ptr,
                      long_num_struct_t* a_nonce_ptr)
{
    UWORD32 *temp_u32_ptr;
    UWORD8* temp_ptr;
    long_num_struct_t gen, temp_pke;
#ifndef BIGEND
    UWORD32 temp_u32;
#endif
    UWORD16 cnt;

    /*************************************************************************/
    /* Allocate temporary buffers required to generate DH key                */
    /*************************************************************************/
    if(NULL == (temp_pke.val_ptr =
        wps_mem_alloc(WPS_DH_PRIME_LEN_WORD32 * sizeof(UWORD32))))
    {
        return FAILURE;
    }
    if(NULL == (temp_u32_ptr =
        wps_mem_alloc(WPS_DH_PRIME_LEN_WORD32*2 * sizeof(UWORD32))))
    {
        wps_mem_free(temp_pke.val_ptr);
        return FAILURE;
    }

    temp_pke.length = WPS_DH_PRIME_LEN_WORD32;

#ifndef OPTIMIZED_PKE_GENERATION
    /*************************************************************************/
    /* Generate private key and store it at a_nonce_ptr->val_ptr.            */
    /*************************************************************************/
    wps_get_rand_byte_array((UWORD8 *) a_nonce_ptr->val_ptr,
        (UWORD16) WPS_PRIVATE_KEY_LEN * 4);
    a_nonce_ptr->msb_pos = wps_check_msb_pos(a_nonce_ptr);
#endif /* OPTIMIZED_PKE_GENERATION */

    /*************************************************************************/
    /* Initialize the generator number. Use PKE buffer to store the generator*/
    /*************************************************************************/
    gen.val_ptr = pke_ptr;
    memset((void *) gen.val_ptr, 0, WPS_DH_PRIME_LEN_WORD32 * sizeof(UWORD32));
    gen.val_ptr[0] = WPS_DH_GENERATOR;
    gen.length = WPS_DH_PRIME_LEN_WORD32;
    gen.msb_pos = wps_check_msb_pos(&gen);
    /*************************************************************************/
    /* Generate PKE using the generator and A nonce                          */
    /*************************************************************************/
    wps_dh_core(&gen,a_nonce_ptr,&temp_pke,temp_u32_ptr);
    /*************************************************************************/
    /* PKE is stored in temp_pke structure in UWORD32 format where lowest    */
    /* word is stored at first location. Do the conversion in bytes and store*/
    /* the PKE in bytes and big Endian format in the buffer pointed by       */
    /* pke_ptr                                                               */
    /*************************************************************************/
    wps_mem_free(temp_u32_ptr);

    temp_u32_ptr = temp_pke.val_ptr + WPS_DH_PRIME_LEN_WORD32 -1;

#ifndef BIGEND
    temp_ptr = (UWORD8 *) pke_ptr;
    for(cnt =0; cnt < WPS_DH_PRIME_LEN_WORD32; cnt++)
    {
        temp_u32 = *temp_u32_ptr--;
        *temp_ptr++ = (UWORD8)((temp_u32>>24) & 0xFF);
        *temp_ptr++ = (UWORD8)((temp_u32>>16) & 0xFF);
        *temp_ptr++ = (UWORD8)((temp_u32>>8) & 0xFF);
        *temp_ptr++ = (UWORD8)(temp_u32 & 0xFF);
    }
#else /* #ifndef BIGEND */
    /*************************************************************************/
    /* In Big Endian, within a word, most significant bytes is stored at     */
    /* first location so no need to do byte order conversion. Direct copy    */
    /* words.                                                                */
    /*************************************************************************/
    for(cnt =0; cnt < WPS_DH_PRIME_LEN_WORD32; cnt++)
    {
        *pke_ptr++ = *temp_u32_ptr--;
    }
#endif /* #ifndef BIGEND */
    wps_mem_free(temp_pke.val_ptr);
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_ph_dh_key                                     */
/*                                                                           */
/*  Description      : This function generates Diffie-Hellman shared secret  */
/*                     key. This key needs to be hashed before it can be used*/
/*                                                                           */
/*  Inputs           : 1) Pointer to the structure where DH remote Public Key*/
/*                        is stored                                          */
/*                     2) Pointer to the structure where DH Private Key  is  */
/*                        stored                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Using the remote public key and local private key this*/
/*                     functions generate the shared secret key  =           */
/*                     = (public key ^private Key) mod p) where g =2, p is   */
/*                     predefined prime. It then  converts the secret key in */
/*                     byte format. Public key, Private key, secret key and  */
/*                     Prime are all of  length  WPS_DH_PRIME_LEN_WORD32.    */
/*                                                                           */
/*  Outputs          : DH Key =  (public key ^private Key) mod p)            */
/*  Returns          : Status of the function processing                     */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_gen_ph_dh_key(UWORD32* pkr_ptr,
                      long_num_struct_t* a_nonce_ptr)
{
    UWORD32 *temp_u32_ptr;
    UWORD8* temp_ptr;
    long_num_struct_t gen, temp_dh_key;
#ifndef BIGEND
    UWORD32 temp_u32;
#endif
    UWORD16 cnt;

    /*************************************************************************/
    /* Allocate temporary buffers required to generate DH key                */
    /*************************************************************************/
    if(NULL == (temp_dh_key.val_ptr =
        wps_mem_alloc(WPS_DH_PRIME_LEN_WORD32 * sizeof(UWORD32))))
    {
        return FAILURE;
    }
    if(NULL == (temp_u32_ptr =
        wps_mem_alloc(WPS_DH_PRIME_LEN_WORD32*2 * sizeof(UWORD32))))
    {
        wps_mem_free(temp_dh_key.val_ptr);
        return FAILURE;
    }
    temp_dh_key.length = WPS_DH_PRIME_LEN_WORD32;

    /*************************************************************************/
    /* Initialize the generator number structure                             */
    /*************************************************************************/
    gen.val_ptr = pkr_ptr;
    gen.length = WPS_DH_PRIME_LEN_WORD32;
    gen.msb_pos = wps_check_msb_pos(&gen);
    /*************************************************************************/
    /* Generate DH_KEY using PKR(gen) and A nonce                            */
    /*************************************************************************/
    wps_dh_core(&gen,a_nonce_ptr,&temp_dh_key,temp_u32_ptr);
    /*************************************************************************/
    /* DH_ Key is stored in temp_dh_key structure in UWORD32 format where    */
    /* lowest word is stored at first location. Do the conversion in bytes   */
    /* and store the DH Key in bytes and big Endian format in the buffer     */
    /* pointed by pkr_ptr                                                    */
    /*************************************************************************/
    wps_mem_free(temp_u32_ptr);

    temp_u32_ptr = temp_dh_key.val_ptr + WPS_DH_PRIME_LEN_WORD32 -1;

#ifndef BIGEND
    temp_ptr = (UWORD8 *) pkr_ptr;
    for(cnt =0; cnt < WPS_DH_PRIME_LEN_WORD32; cnt++)
    {
        temp_u32 = *temp_u32_ptr--;
        *temp_ptr++ = (UWORD8)((temp_u32>>24) & 0xFF);
        *temp_ptr++ = (UWORD8)((temp_u32>>16) & 0xFF);
        *temp_ptr++ = (UWORD8)((temp_u32>>8) & 0xFF);
        *temp_ptr++ = (UWORD8)(temp_u32 & 0xFF);
    }
#else /* #ifndef BIGEND */
    /*************************************************************************/
    /* In Big Endian, within a word, most significant bytes is stored at     */
    /* first location so no need to do byte order conversion. Direct copy    */
    /* words.                                                                */
    /*************************************************************************/
    for(cnt =0; cnt < WPS_DH_PRIME_LEN_WORD32; cnt++)
    {
        *pkr_ptr++ = *temp_u32_ptr--;
    }
#endif /* #ifndef BIGEND */
    wps_mem_free(temp_dh_key.val_ptr);
    return SUCCESS;
}

#endif /* INT_WPS_ENR_SUPP */
