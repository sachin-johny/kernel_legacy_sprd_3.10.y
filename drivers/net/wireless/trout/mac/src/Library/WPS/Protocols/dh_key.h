/*****************************************************************************/
/*                                                                           */
/*                           Ittiam WPS SOFTWARE                             */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : dh_key.h                                             */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      implementing the Diffie-Hellman keys generation as   */
/*                      defined in WPS standard  version v.1.2.2             */
/*                                                                           */
/*  List of Functions : wps_cmp_num                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         25 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_REG_SUPP

#ifndef DH_KEY_H
#define DH_KEY_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "wpsr_reg_if.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/

#define WPS_DH_GENERATOR 2

/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/

#define GET_LONG_WORD_LEN(num) ((num->msb_pos + (UWORD8) 31) >> (UWORD8) 5)

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum
{
    LONG_NUM_SMALLER = -1,
    LONG_NUM_EQUAL   = 0,
    LONG_NUM_GREATER = 1,
} LONG_CMP_RES;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void wps_mult_mod_p (long_num_struct_t* num1,
                            long_num_struct_t* num2,
                            UWORD32* temp_mult_buff);
extern UWORD16 wps_check_msb_pos(long_num_struct_t* inp);
extern void wps_dh_core(long_num_struct_t *gen_ptr,
                        long_num_struct_t *priv_key_ptr,
                        long_num_struct_t *dh_key_ptr,
                        UWORD32           *temp_mult_buff);
extern STATUS_T wps_gen_pub_key(UWORD32* pub_key_ptr,
                                long_num_struct_t* priv_key_ptr);
extern STATUS_T wps_gen_ph_dh_key(UWORD32* pkr_ptr,
                                  long_num_struct_t* exp_ptr);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_cmp_num                                           */
/*                                                                           */
/*  Description      : This function compares two big numbers stored in      */
/*                     structures of type long_num_struct_t and returns the  */
/*                     comparison result.                                    */
/*                                                                           */
/*  Inputs           : 1) Pointer to the structure storing the First big no  */
/*                     2) Pointer to the structure storing the Second big no */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function first checks the MSB position of the two*/
/*                     numbers. If MSB position is same then it checks each  */
/*                     word (starting from Most significant word) till it    */
/*                     finds the bigger number or all words are compared.    */
/*                                                                           */
/*  Outputs          : Result of the comparison of the two big numbers. If   */
/*                     first number > Second number then ret runs            */
/*                     LONG_NUM_GREATER else if first number < Second number */
/*                     then returns LONG_NUM_SMALLER else returns            */
/*                     LONG_NUM_EQUAL                                        */
/*                                                                           */
/*  Returns          : Result of the comparison of the two big numbers       */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
INLINE LONG_CMP_RES wps_cmp_num(long_num_struct_t num1, long_num_struct_t num2)
{
    WORD16 cnt, word_len;
    UWORD32 *n1_ptr, *n2_ptr;

    /* Compare MSB bit positions if bit positions different then return      */
    /* LONG_NUM_GREATER or LONG_NUM_SMALLER appropriately                    */
    if(num1.msb_pos > num2.msb_pos)
    {
        return LONG_NUM_GREATER;
    }
    else if(num1.msb_pos < num2.msb_pos)
    {
        return LONG_NUM_SMALLER;
    }

    /* Starting with most significant word. compare words from two numbers if*/
    /* words differ return LONG_NUM_GREATER or LONG_NUM_SMALLER appropriately*/
    n1_ptr = num1.val_ptr;
    n2_ptr = num2.val_ptr;
    word_len = ((num1.msb_pos + (UWORD8) 31) >> (UWORD8) 5) - 1;
    for(cnt = word_len; cnt >= 0; cnt--)
    {
        if(n1_ptr[cnt] != n2_ptr[cnt])
        {
            if(n1_ptr[cnt] > n2_ptr[cnt])
            {
                return LONG_NUM_GREATER;
            }
            else
            {
                return LONG_NUM_SMALLER;
            }
        }
    }

    /* MSB positions of both words are same and each word in first big number*/
    /* is same as corresponding word in second number. hence both numbers are*/
    /* same. Return LONG_NUM_EQUAL                                           */
    return LONG_NUM_EQUAL;
}
#endif /* DH_KEY_H */
#endif /* INT_WPS_REG_SUPP */
