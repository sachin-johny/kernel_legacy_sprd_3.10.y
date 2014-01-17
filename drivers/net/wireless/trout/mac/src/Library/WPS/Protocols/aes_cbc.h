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
/*  File Name         : aes_cbc.h                                            */
/*                                                                           */
/*  Description       : This file contains all the data type definitions of  */
/*                      AES Encryption and decryption modules of WPS Library */
/*                                                                           */
/*  List of Functions : None                                                 */
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

#ifndef AES_CBC_H
#define AES_CBC_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpsr_reg_if.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/
/* Structure used for AES Setup */
typedef struct
{
  UWORD32 keys[60];
  UWORD32 ikeys[60];
  UWORD32 nrounds;
} aes_context_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void aes_setup(aes_context_t *ctx, UWORD8 keysize, UWORD8 *key);
extern void aes_encrypt(aes_context_t *ctxt, UWORD8 *ptext, UWORD8 *ctext);
extern void aes_decrypt(aes_context_t *ctxt, UWORD8 *ctext, UWORD8 *ptext);
extern void wps_aes_decrypt(UWORD8 *key_wrap_key,  UWORD8 *data_inp_ptr,
                            UWORD8 *data_out_ptr, UWORD16 data_len);
extern void wps_aes_encrypt(UWORD8 *key_wrap_key, UWORD8* data_ptr,
                            UWORD16 data_len);

#endif /* AES_CBC_H */
#endif /* INT_WPS_REG_SUPP */
