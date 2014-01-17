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
/*  File Name         : md5.h                                                */
/*                                                                           */
/*  Description       : This file contains the constant definitions, enums,  */
/*                      structures and inline utility functions required by  */
/*                      HMAC-MD5 algorithm.                                  */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef  MAC_802_11I

#ifndef HMAC_MD5_H
#define HMAC_MD5_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD32 count[2];
    UWORD32 abcd[4];
    UWORD8 buf[64];
} md5_state_t;

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define T_MASK ((UWORD32)~0)
#define T1 /* 0xD76AA478 */ (T_MASK ^ 0x28955B87)
#define T2 /* 0xE8C7B756 */ (T_MASK ^ 0x173848A9)
#define T3    0x242070DB
#define T4 /* 0xC1BDCEEE */ (T_MASK ^ 0x3E423111)
#define T5 /* 0xF57C0FAF */ (T_MASK ^ 0x0A83F050)
#define T6    0x4787C62A
#define T7 /* 0xA8304613 */ (T_MASK ^ 0x57CFB9EC)
#define T8 /* 0xFD469501 */ (T_MASK ^ 0x02B96AFE)
#define T9    0x698098D8
#define T10 /* 0x8B44F7AF */ (T_MASK ^ 0x74BB0850)
#define T11 /* 0xFFFF5BB1 */ (T_MASK ^ 0x0000A44E)
#define T12 /* 0x895CD7BE */ (T_MASK ^ 0x76A32841)
#define T13    0x6B901122
#define T14 /* 0xFD987193 */ (T_MASK ^ 0x02678E6C)
#define T15 /* 0xA679438E */ (T_MASK ^ 0x5986BC71)
#define T16    0x49B40821
#define T17 /* 0xF61E2562 */ (T_MASK ^ 0x09E1DA9D)
#define T18 /* 0xC040B340 */ (T_MASK ^ 0x3FBF4CBF)
#define T19    0x265E5A51
#define T20 /* 0xE9B6C7AA */ (T_MASK ^ 0x16493855)
#define T21 /* 0xD62F105D */ (T_MASK ^ 0x29D0EFA2)
#define T22    0x02441453
#define T23 /* 0xD8A1E681 */ (T_MASK ^ 0x275E197E)
#define T24 /* 0xE7D3FBC8 */ (T_MASK ^ 0x182C0437)
#define T25    0x21E1CDE6
#define T26 /* 0xC33707D6 */ (T_MASK ^ 0x3CC8F829)
#define T27 /* 0xF4D50D87 */ (T_MASK ^ 0x0B2AF278)
#define T28    0x455A14ED
#define T29 /* 0xA9E3E905 */ (T_MASK ^ 0x561C16FA)
#define T30 /* 0xFCEFA3F8 */ (T_MASK ^ 0x03105C07)
#define T31    0x676F02D9
#define T32 /* 0x8D2A4C8A */ (T_MASK ^ 0x72D5B375)
#define T33 /* 0xFFFA3942 */ (T_MASK ^ 0x0005C6BD)
#define T34 /* 0x8771F681 */ (T_MASK ^ 0x788E097E)
#define T35    0x6D9D6122
#define T36 /* 0xFDE5380C */ (T_MASK ^ 0x021AC7F3)
#define T37 /* 0xA4BEEA44 */ (T_MASK ^ 0x5B4115BB)
#define T38    0x4BDECFA9
#define T39 /* 0xF6BB4B60 */ (T_MASK ^ 0x0944B49F)
#define T40 /* 0xBEBFBC70 */ (T_MASK ^ 0x4140438F)
#define T41    0x289B7EC6
#define T42 /* 0xEAA127FA */ (T_MASK ^ 0x155ED805)
#define T43 /* 0xD4EF3085 */ (T_MASK ^ 0x2B10CF7A)
#define T44    0x04881D05
#define T45 /* 0xD9D4D039 */ (T_MASK ^ 0x262B2FC6)
#define T46 /* 0xE6DB99E5 */ (T_MASK ^ 0x1924661A)
#define T47    0x1FA27CF8
#define T48 /* 0xC4AC5665 */ (T_MASK ^ 0x3B53A99A)
#define T49 /* 0xF4292244 */ (T_MASK ^ 0x0BD6DDBB)
#define T50    0x432AFF97
#define T51 /* 0xAB9423A7 */ (T_MASK ^ 0x546BDC58)
#define T52 /* 0xFC93A039 */ (T_MASK ^ 0x036C5FC6)
#define T53    0x655B59C3
#define T54 /* 0x8F0CCC92 */ (T_MASK ^ 0x70F3336D)
#define T55 /* 0xFFEFF47D */ (T_MASK ^ 0x00100B82)
#define T56 /* 0x85845DD1 */ (T_MASK ^ 0x7A7BA22E)
#define T57    0x6FA87E4F
#define T58 /* 0xFE2CE6E0 */ (T_MASK ^ 0x01D3191F)
#define T59 /* 0xA3014314 */ (T_MASK ^ 0x5CFEBCEB)
#define T60    0x4E0811A1
#define T61 /* 0xF7537E82 */ (T_MASK ^ 0x08AC817D)
#define T62 /* 0xBD3AF235 */ (T_MASK ^ 0x42C50DCA)
#define T63    0x2AD7D2BB
#define T64 /* 0xEB86D391 */ (T_MASK ^ 0x14792C6E)

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#define ROTATE_LEFT(x, n) (((x) << (n)) | ((x) >> (32 - (n))))
#define F(x, y, z) (((x) & (y)) | (~(x) & (z)))
#define SET1(a, b, c, d, k, s, Ti)\
  t = a + F(b,c,d) + X[k] + Ti;\
  a = ROTATE_LEFT(t, s) + b
#define G(x, y, z) (((x) & (z)) | ((y) & ~(z)))
#define SET2(a, b, c, d, k, s, Ti)\
  t = a + G(b,c,d) + X[k] + Ti;\
  a = ROTATE_LEFT(t, s) + b
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define SET3(a, b, c, d, k, s, Ti)\
  t = a + H(b,c,d) + X[k] + Ti;\
  a = ROTATE_LEFT(t, s) + b
#define I(x, y, z) ((y) ^ ((x) | ~(z)))
#define SET4(a, b, c, d, k, s, Ti)\
  t = a + I(b,c,d) + X[k] + Ti;\
  a = ROTATE_LEFT(t, s) + b



/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void md5_init(md5_state_t *pms);
extern void md5_append(md5_state_t *pms, UWORD8 *data, UWORD32 nbytes);
extern void md5_finish(md5_state_t *pms, UWORD8 digest[16]);
extern void hmac_md5(UWORD8* text, UWORD32 text_len, UWORD8* key,
                     UWORD32 key_len, UWORD8* digest);

#endif /* HMAC_MD5_H */
#endif /* MAC_802_11I */

