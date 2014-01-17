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
/*  File Name         : prf.h                                                */
/*                                                                           */
/*  Description       : This file contains the constant definitions, enums,  */
/*                      structures and inline utility functions required by  */
/*                      the PRF module.                                      */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

#ifndef PRF_H
#define PRF_H


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_MESSAGE_LENGTH      4096
#define SHA1_BLOCK_LENGTH       64
#define HMAC_SHA1_BLOCK_LENGTH  64
#define HMAC_DIGEST_LENGTH      20
#define HMAC_SHA_RESULT_LENGTH  160
#define IPAD  0x36
#define OPAD  0x5C

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void hmac_sha1(UWORD8 *key, UWORD32 key_length, UWORD8 *data,
                      UWORD32 data_length,UWORD8 *digest);
extern void sha1(UWORD8 *message, UWORD32 message_length, UWORD8 *digest);
extern void prf(UWORD8 *key, UWORD32 key_length, UWORD8 *prefix,
                UWORD32 prefix_length, UWORD8 *data, UWORD32 data_length,
                UWORD32 result_length, UWORD8 *result);


#endif /* PRF_H */
#endif  /* MAC_802_11I */

