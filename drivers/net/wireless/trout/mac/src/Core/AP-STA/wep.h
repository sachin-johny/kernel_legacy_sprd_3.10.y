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
/*  File Name         : wep.h                                                */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      WEP                                                  */
/*                                                                           */
/*  List of Functions : get_wep_type                                         */
/*                      char_2_hex                                           */
/*                      hex_2_char                                           */
/*                      str_2_hex                                            */
/*                      adjust_wep_headers                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef WEP_H
#define WEP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "csl_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define WEP40_KEY_SIZE           5                     /* Size of Wep 40 Key */
#define WEP104_KEY_SIZE          13
#define MSG_3_TIMEOUT            1000        /* Shared key message 3 timeout */

#define WEP_HDR_LEN              4
#define WEP_IV_LEN               4

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_wep_type;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_wep(void);
extern void init_wep_keys(UWORD8 idx, UWORD8* key, UWORD8 size);
extern void add_wep_entry(UWORD8 index, UWORD8 keyid, UWORD8* addr);
extern void add_wep_key(UWORD8 keyid);

/*****************************************************************************/
/* Inline Function Declarations                                              */
/*****************************************************************************/

/* This function extracts the Cipher type for the given wep key id */
INLINE CIPHER_T get_wep_type(UWORD8 key_id)
{
    CIPHER_T ct = NO_ENCRYP;

    switch(mget_WEPKeySize(key_id))
    {
        /* if Key size is 40, cipher type is WEP 40 */
        case 40:
        default:
            ct = WEP40;
            break;
        case 104:
            ct = WEP104;
            break;
    }
    return ct;
}

/* Convert char to hex */
INLINE UWORD8 char_2_hex(UWORD8 val)
{
	if(val - '0' <= 9)
	{
		return (val - '0');
	}
	else if('f' - val <= 6)
	{
		return (val - 'a' + 10);
	}
	else
	{
		return (val - 'A' + 10);
	}
}

/* Convert char to hex */
INLINE UWORD8 hex_2_char(UWORD8 val)
{
    return (val <= 9)? (val +'0') : ('a' +  val - 10);
}

/* Convert string to hex */
INLINE void str_2_hex(UWORD8 *val, UWORD8 size)
{
    UWORD8 i = 0;
    for(i = 0; i < size; i++)
    {
        val[i] = char_2_hex(val[2*i]) * 16 + char_2_hex(val[2*i + 1]);
    }
}
/* Adjust length and data offset of the WEP protected received packet */
INLINE void adjust_wep_headers(CIPHER_T ct, UWORD16 *rx_len, UWORD8 *offset)
{
    if(mget_PrivacyInvoked() == TV_TRUE)
    {
        /* Data is offset by 4 bytes due the to WEP header */
        *offset = WEP_HDR_LEN;
        /* Data length is currently passed inclusive of H/W IV   */
        /* and WEP Header                                        */
        *rx_len -= WEP_HDR_LEN + WEP_IV_LEN;
    }
}

#endif /* WEP_H */
