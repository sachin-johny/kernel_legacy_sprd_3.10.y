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
/*  File Name         : md5.c                                                */
/*                                                                           */
/*  Description       : This file contains the functions required for HMAC-  */
/*                      MD5 algorithms.                                      */
/*                                                                           */
/*  List of Functions : md5_init                                             */
/*                      md5_append                                           */
/*                      md5_finish                                           */
/*                      md5_process                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/


#ifdef MAC_802_11I


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "md5.h"
#include "common.h"

#ifdef UTILS_11I


/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void md5_process(md5_state_t *pms, UWORD8 *data);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : md5_init                                              */
/*                                                                           */
/*  Description      : Initializes the MD5 States                            */
/*                                                                           */
/*  Inputs           : 1) MD5 State                                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void md5_init(md5_state_t *pms)
{
    pms->count[0] = pms->count[1] = 0;
    pms->abcd[0] = 0x67452301;
    pms->abcd[1] = T_MASK ^ 0x10325476;
    pms->abcd[2] = T_MASK ^ 0x67452301;
    pms->abcd[3] = 0x10325476;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : md5_append                                            */
/*                                                                           */
/*  Description      : Performs the Append Function as specified in the MD5  */
/*                     algorithm                                             */
/*                                                                           */
/*  Inputs           : 1) MD5 State                                          */
/*                     2) Data                                               */
/*                     3) Data Length                                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void md5_append(md5_state_t *pms, UWORD8 *data, UWORD32 nbytes)
{
    UWORD32 left   = 0;
    UWORD32 offset = 0;
    UWORD32 nbits  = 0;
    UWORD8  *p     = 0;

    p = data;
    left = nbytes;
    offset = (pms->count[0] >> 3) & 63;
    nbits = (UWORD32)(nbytes << 3);

    /* Exception */
    if (nbytes <= 0)
        return;

    /* Update the message length. */
	#if 0 //caisf mod 0702
    pms->count[1] += nbytes >> 29;
    pms->count[0] += nbits;
	#else
    pms->count[1] = pms->count[1] + (nbytes >> 29);
    pms->count[0] = pms->count[0] + nbits;
	#endif
    if (pms->count[0] < nbits)
        pms->count[1]++;

    /* Process an initial partial block. */
    if(offset != 0)
    {
        UWORD32 copy = 0;
		
#if 0 //caisf mod 0702
        copy = (offset + nbytes > 64 ? 64 - offset : nbytes);
#else
        copy = ((offset + nbytes > 64) ? (64 - offset) : nbytes);
#endif
        memcpy((pms->buf + offset), p, copy);

        if (offset + copy < 64)
            return;
	#if 0 //caisf mod 0702
        p += copy;
        left -= copy;
	#else
        p = p + copy;
        left = left - copy;
	#endif
        md5_process(pms, pms->buf);
    }

    /* Process full blocks. */
#if 0 //caisf mod 0702
    for (; left >= 64; p += 64, left -= 64)
        md5_process(pms, p);
#else
    for (; left >= 64; p = (p + 64), left = (left - 64))
        md5_process(pms, p);
#endif

    /* Process a final partial block. */
    if(left != 0)
        memcpy(pms->buf, p, left);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : md5_finish                                            */
/*                                                                           */
/*  Description      : Performs the MD5 operation                            */
/*                                                                           */
/*  Inputs           : 1) MD5 State                                          */
/*                     2) Digest                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void md5_finish(md5_state_t *pms, UWORD8 *digest)
{
    static UWORD8 pad[64] = {0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0};
    UWORD32 i = 0;
    UWORD8 data[8];

    /* Save the length before padding. */
    for (i = 0; i < 8; ++i)
        data[i] = (UWORD8)(pms->count[i >> 2] >> ((i & 3) << 3));

    /* Pad to 56 bytes mod 64. */
    md5_append(pms, pad, ((55 - (pms->count[0] >> 3)) & 63) + 1);

    /* Append the length. */
    md5_append(pms, data, 8);

    for (i = 0; i < 16; ++i)
        digest[i] = (UWORD8)(pms->abcd[i >> 2] >> ((i & 3) << 3));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : md5_process                                           */
/*                                                                           */
/*  Description      : Performs the MD5 - Process function                   */
/*                                                                           */
/*  Inputs           : 1) MD5 State                                          */
/*                     2) Data                                               */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void md5_process(md5_state_t *pms, UWORD8 *data)
{
    UWORD32 a = 0;
    UWORD32 b = 0;
    UWORD32 c = 0;
    UWORD32 d = 0;
    UWORD32 t = 0;
    UWORD32 *X = 0;
    UWORD32 xbuf[16];
#ifdef MWLAN
    UWORD8 idx = 0;
#endif /* MWLAN */

    a = pms->abcd[0];
    b = pms->abcd[1];
    c = pms->abcd[2];
    d = pms->abcd[3];

#ifdef MWLAN
    	for(idx = 0; idx < 16; idx++)
        xbuf[idx] = SWAP_BYTE_ORDER_WORD(*((UWORD32 *)data + idx));
#else /* MWLAN */
    memcpy(xbuf, data, 64);
#endif /* MWLAN */

    X = xbuf;

    /* Round 1. */
    /* Let [abcd k s i] denote the operation:                                */
    /* a = b + ((a + F(b,c,d) + X[k] + T[i]) <<< s).                         */
    SET1(a, b, c, d,  0,  7,  T1);
    SET1(d, a, b, c,  1, 12,  T2);
    SET1(c, d, a, b,  2, 17,  T3);
    SET1(b, c, d, a,  3, 22,  T4);
    SET1(a, b, c, d,  4,  7,  T5);
    SET1(d, a, b, c,  5, 12,  T6);
    SET1(c, d, a, b,  6, 17,  T7);
    SET1(b, c, d, a,  7, 22,  T8);
    SET1(a, b, c, d,  8,  7,  T9);
    SET1(d, a, b, c,  9, 12, T10);
    SET1(c, d, a, b, 10, 17, T11);
    SET1(b, c, d, a, 11, 22, T12);
    SET1(a, b, c, d, 12,  7, T13);
    SET1(d, a, b, c, 13, 12, T14);
    SET1(c, d, a, b, 14, 17, T15);
    SET1(b, c, d, a, 15, 22, T16);

    /* Round 2. */
    /* Let [abcd k s i] denote the operation:                                */
    /*    a = b + ((a + G(b,c,d) + X[k] + T[i]) <<< s).                      */
    SET2(a, b, c, d,  1,  5, T17);
    SET2(d, a, b, c,  6,  9, T18);
    SET2(c, d, a, b, 11, 14, T19);
    SET2(b, c, d, a,  0, 20, T20);
    SET2(a, b, c, d,  5,  5, T21);
    SET2(d, a, b, c, 10,  9, T22);
    SET2(c, d, a, b, 15, 14, T23);
    SET2(b, c, d, a,  4, 20, T24);
    SET2(a, b, c, d,  9,  5, T25);
    SET2(d, a, b, c, 14,  9, T26);
    SET2(c, d, a, b,  3, 14, T27);
    SET2(b, c, d, a,  8, 20, T28);
    SET2(a, b, c, d, 13,  5, T29);
    SET2(d, a, b, c,  2,  9, T30);
    SET2(c, d, a, b,  7, 14, T31);
    SET2(b, c, d, a, 12, 20, T32);

    /* Round 3. */
    /* Let [abcd k s t] denote the operation:                                */
    /* a = b + ((a + H(b,c,d) + X[k] + T[i]) <<< s).                         */
    SET3(a, b, c, d,  5,  4, T33);
    SET3(d, a, b, c,  8, 11, T34);
    SET3(c, d, a, b, 11, 16, T35);
    SET3(b, c, d, a, 14, 23, T36);
    SET3(a, b, c, d,  1,  4, T37);
    SET3(d, a, b, c,  4, 11, T38);
    SET3(c, d, a, b,  7, 16, T39);
    SET3(b, c, d, a, 10, 23, T40);
    SET3(a, b, c, d, 13,  4, T41);
    SET3(d, a, b, c,  0, 11, T42);
    SET3(c, d, a, b,  3, 16, T43);
    SET3(b, c, d, a,  6, 23, T44);
    SET3(a, b, c, d,  9,  4, T45);
    SET3(d, a, b, c, 12, 11, T46);
    SET3(c, d, a, b, 15, 16, T47);
    SET3(b, c, d, a,  2, 23, T48);

    /* Round 4. */
    /* Let [abcd k s t] denote the operation:                                */
    /* a = b + ((a + I(b,c,d) + X[k] + T[i]) <<< s).                         */
    SET4(a, b, c, d,  0,  6, T49);
    SET4(d, a, b, c,  7, 10, T50);
    SET4(c, d, a, b, 14, 15, T51);
    SET4(b, c, d, a,  5, 21, T52);
    SET4(a, b, c, d, 12,  6, T53);
    SET4(d, a, b, c,  3, 10, T54);
    SET4(c, d, a, b, 10, 15, T55);
    SET4(b, c, d, a,  1, 21, T56);
    SET4(a, b, c, d,  8,  6, T57);
    SET4(d, a, b, c, 15, 10, T58);
    SET4(c, d, a, b,  6, 15, T59);
    SET4(b, c, d, a, 13, 21, T60);
    SET4(a, b, c, d,  4,  6, T61);
    SET4(d, a, b, c, 11, 10, T62);
    SET4(c, d, a, b,  2, 15, T63);
    SET4(b, c, d, a,  9, 21, T64);

    /* Increment each of the four registers by the value it had before this  */
    /* block was started.                                                    */
	#if 0 //caisf mod 0702
    pms->abcd[0] += a;
    pms->abcd[1] += b;
    pms->abcd[2] += c;
    pms->abcd[3] += d;
	#else
    pms->abcd[0] = pms->abcd[0] + a;
    pms->abcd[1] = pms->abcd[1] + b;
    pms->abcd[2] = pms->abcd[2] + c;
    pms->abcd[3] = pms->abcd[3] + d;
	#endif
}

#endif /* UTILS_11I */
#endif /* MAC_802_11I */
