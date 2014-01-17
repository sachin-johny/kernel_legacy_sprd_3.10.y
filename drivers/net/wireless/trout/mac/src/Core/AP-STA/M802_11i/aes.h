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
/*  File Name         : aes.h                                                */
/*                                                                           */
/*  Description       : This file contains the constant definitions, enums,  */
/*                      structures and inline utility functions required by  */
/*                      AES Key wrap algorithm.                              */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef AES_H
#define AES_H


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define NUM_ROUNDS         10
#define RIJNDAEL_BLOCKSIZE 16
#define KEY_SIZE           16

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
  UWORD32 keys[60];
  UWORD32 ikeys[60];
  UWORD32 nrounds;
} aes_context_t;

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#define ROTBYTE(x) (((x) >> 8) | (((x) & 0xFF) << 24))
#define ROTRBYTE(x) (((x) << 8) | (((x) >> 24) & 0xFF))
#define SUBBYTE(x, box) (((box)[((x) & 0xFF)]) | \
                        ((box)[(((x) >> 8) & 0xFF)] << 8) | \
                        ((box)[(((x) >> 16) & 0xFF)] << 16) | \
                        ((box)[(((x) >> 24) & 0xFF)] << 24))

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void aes_setup(aes_context_t *ctx, UWORD8 keysize, UWORD8 *key);
extern void aes_encrypt(aes_context_t *ctxt, UWORD8 *ptext, UWORD8 *ctext);
extern void aes_decrypt(aes_context_t *ctxt, UWORD8 *ctext, UWORD8 *ptext);

extern void aes_key_wrap(UWORD8 *text, UWORD8 *key, UWORD32 n);
extern BOOL_T aes_key_unwrap(UWORD8 *text, UWORD8 *key, UWORD32 n);


#endif /* AES_H */

#endif /* MAC_802_11I */

