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
/*  File Name         : wpse_hmac_sha256.h                                   */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      HMAC-SHA-256 and SHA-256 implementation              */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         11 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_ENR_SUPP

#ifndef HAMC_SHA256_H
#define HAMC_SHA256_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define SHA256_BLOCK_SIZE       64
#define SHA256_LEN_FIELD_SIZE   8
#define MAX_HMAC_FRAG_CNT       4
#define MAX_SHA_256_FRAG_CNT    (MAX_HMAC_FRAG_CNT + 1)
#define IPAD                    0x36
#define OPAD                    0x5C
/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

/*****************************************************************************/
/* Various functions used for SHA-256                                        */
/*****************************************************************************/
#define ROTR(x, n)  (UWORD32)(((UWORD32) (x >> (UWORD8) n)) | \
    ((UWORD32)(x << (UWORD8)(32 -n))))
#define SHR(x, n)   (UWORD32)((UWORD32)x >> (UWORD8)n)

/*****************************************************************************/
/* Ch(x,y,z) = (x & y)^(~x & z)                                              */
/*****************************************************************************/
#define Ch(x,y,z)   (z ^ (x & (y ^ z)))
//((x & y)^(~x & z))

/*****************************************************************************/
/* Maj(x,y,z) = (x & y)^( x & z)^( y & z)                                    */
/* Simplifying using K-map or truth table the expression becomes             */
/* Maj(x,y,z) = (x & y)|( x & z)|( y & z) = x&(y|z) | y&z                    */
/*****************************************************************************/
#define Maj(x,y,z)      ((x & (y | z)) | (y & z))

#define SUM0(x)         (ROTR(x, 2)  ^ ROTR(x, 13) ^ ROTR(x, 22))
#define SUM1(x)         (ROTR(x, 6)  ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SIGMA0(x)       (ROTR(x, 7)  ^ ROTR(x, 18) ^ SHR(x, 3))
#define SIGMA1(x)       (ROTR(x, 17) ^ ROTR(x, 19) ^ SHR(x, 10))

/*****************************************************************************/
/* Perform 1 rounds of Hashing as mentioned in step 3 of "SHA-256 Hash       */
/* Computation" mentioned in the standard.                                   */
/*****************************************************************************/
#define RND(a,b,c,d,e,f,g,h,T1, T2, K, W, cnt)          \
    T1 = h + SUM1(e) + Ch(e, f, g) + K[cnt] + W[cnt];   \
    T2 = SUM0(a) + Maj(a, b, c);            \
    h = g;                  \
    g = f;                  \
    f = e;                  \
    e = d+T1;               \
    d = c;                  \
    c = b;                  \
    b = a;                  \
    a  = T1 + T2;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
STATUS_T sha256(UWORD8 *msg[], UWORD16 *msg_len, UWORD8 num_elem, UWORD8 *mac);
STATUS_T hmac_sha256(UWORD8 *key, UWORD16 key_length, UWORD8 *msg[],
                     UWORD16 *msg_len, UWORD8 frag_cnt, UWORD8 *mac);
/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

#endif /* HAMC_SHA256_H */
#endif /* INT_WPS_ENR_SUPP */
