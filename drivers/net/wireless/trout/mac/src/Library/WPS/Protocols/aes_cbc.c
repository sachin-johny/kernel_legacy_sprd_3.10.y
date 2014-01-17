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
/*  File Name         : aes_cbc.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions related WPS AES-CBC */
/*                      Encryption and Decryption                            */
/*                                                                           */
/*  List of Functions : wps_aes_decrypt                                      */
/*                      wps_aes_encrypt                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_REG_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "wps_prot.h"
#include "aes_cbc.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_aes_decrypt                                       */
/*                                                                           */
/*  Description      : This function dose the AES-CBC encryption as per      */
/*                     FIPS PUB 197                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the data to be decrypted. First         */
/*                        WPS_AES_BYTE_BLOCK_SIZE number of bytes has IV     */
/*                        followed by the data to be decrypted               */
/*                     3) Pointer to the location where decrypted data is to */
/*                        be stored                                          */
/*                     4) Length of the data to be decrypted (without IV     */
/*                        length)                                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the AES-CBC decryption. It is      */
/*                     assumed that IV, of length WPS_AES_BYTE_BLOCK_SIZE    */
/*                     bytes, is stored in the beginning of data followed by */
/*                     the data to be decrypted. 1. In place processing is   */
/*                     not done for decryption as the received packets need  */
/*                     to be stored for Authenticator computation for next   */
/*                     transmit message.                                     */
/*                                                                           */
/*  Outputs          : Decrypted data                                        */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_aes_decrypt(UWORD8 *key_wrap_key,  UWORD8 *data_inp_ptr,
                     UWORD8 *data_out_ptr, UWORD16 data_len)
{
    aes_context_t *aes_context;
    UWORD8* temp_ptr;
    UWORD16 cnt;
    UWORD16 num_blocks = (data_len / WPS_AES_BYTE_BLOCK_SIZE);
    UWORD8 cnt_u8;

    /* Allocate memory for AES Context Structure */
    aes_context = (aes_context_t *)wps_local_mem_alloc(sizeof(aes_context_t));

    if(NULL == aes_context)
    {
        wps_handle_sys_err_reg(NO_LOCAL_MEM);
        return;
    }

    /* Initialize the context struture */
    aes_setup(aes_context, WPS_KEY_WRAP_KEY_LEN, key_wrap_key);

    data_inp_ptr += WPS_AES_BYTE_BLOCK_SIZE;

    /* Perform AES Decryption for each block of data */
    for (cnt =0; cnt < num_blocks; cnt++)
    {
        aes_decrypt(aes_context, data_inp_ptr, data_out_ptr);
        temp_ptr = data_inp_ptr - WPS_AES_BYTE_BLOCK_SIZE;
        for(cnt_u8 = 0; cnt_u8 < WPS_AES_BYTE_BLOCK_SIZE; cnt_u8++)
        {
            *data_out_ptr++  ^= *temp_ptr++;
        }
        data_inp_ptr += WPS_AES_BYTE_BLOCK_SIZE;
    }

    wps_local_mem_free((UWORD8 *)aes_context);
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_aes_encrypt                                       */
/*                                                                           */
/*  Description      : This function dose the AES-CBC encryption as per      */
/*                     FIPS PUB 197                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the data to be encrypted. First         */
/*                        WPS_AES_BYTE_BLOCK_SIZE number of bytes has IV     */
/*                        followed by the data to be encrypted               */
/*                     3) Length of the data to be encrypted (without IV     */
/*                        length)                                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Outputs          : Encrypted data                                        */
/*                                                                           */
/*  Processing       : This function dose the AES-CBC encryption. It is      */
/*                     assumed that IV, of length WPS_AES_BYTE_BLOCK_SIZE    */
/*                     bytes, is stored in the beginning of data followed by */
/*                     the data to be encrypted.                             */
/*                                                                           */
/*  Outputs          : Encrypted data                                        */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_aes_encrypt(UWORD8 *key_wrap_key, UWORD8* data_ptr, UWORD16 data_len)
{
    aes_context_t *aes_context;
    UWORD8* temp_ptr;
    UWORD16 cnt;
    UWORD16 num_blocks = (data_len/ WPS_AES_BYTE_BLOCK_SIZE);
    UWORD8 cnt_u8;

    /* Allocate memory for AES Context Structure */
    aes_context = (aes_context_t *)wps_local_mem_alloc(sizeof(aes_context_t));

    if(NULL == aes_context)
    {
        wps_handle_sys_err_reg(NO_LOCAL_MEM);
        return;
    }

    /* Initialize the context struture */
    aes_setup(aes_context, WPS_KEY_WRAP_KEY_LEN, key_wrap_key);

    /* Perform AES Encryption for each block of data */
    for (cnt =0; cnt < num_blocks; cnt++)
    {
        temp_ptr = data_ptr + WPS_AES_BYTE_BLOCK_SIZE;
        for(cnt_u8 = 0; cnt_u8 < WPS_AES_BYTE_BLOCK_SIZE; cnt_u8++)
        {
            *temp_ptr++  ^= *data_ptr++;
        }
        aes_encrypt(aes_context, data_ptr, data_ptr);
    }

    wps_local_mem_free((UWORD8 *)aes_context);
    return;
}


#endif /* INT_WPS_REG_SUPP */
