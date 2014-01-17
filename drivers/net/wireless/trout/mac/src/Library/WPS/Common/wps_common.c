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
/*  File Name         : common.c                                             */
/*                                                                           */
/*  Description       : This file contains the common functions of WPS       */
/*                      Library                                              */
/*                                                                           */
/*  List of Functions : wps_gen_uuid                                         */
/*                      wps_get_rand_byte_array                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         15 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_REG_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "wpsr_reg_if.h"
#include "wps_common.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/


#if NAMESPACE_ID == NAMESPACE_DNS
/* Name string is a fully-qualified domain name */
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x10, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#elif NAMESPACE_ID == NAMESPACE_URL
/* Name string is a URL */
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x11, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#elif NAMESPACE_ID == NAMESPACE_OID
/* Name string is an ISO OID */
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x12, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#elif NAMESPACE_ID == NAMESPACE_X500
/* Name string is an X.500 DN (in DER or a text output format) */
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x14, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#else
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x52, 0x64, 0x80, 0xf8, 0xc9, 0x9b, 0x4b, 0xe5,
    0xa6, 0x55, 0x58, 0xed, 0x5f, 0x5d, 0x60, 0x84
};
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_uuid                                          */
/*                                                                           */
/*  Description      : This function generate UUID using MAC address         */
/*                                                                           */
/*  Inputs           : 1) Pointer to Mac Address                             */
/*                   : 2) Pointer to buffer where UUID needs to be stored    */
/*                   : 3) Pointer to temporary buffer of length >=           */
/*                        SHA1_DIGEST_LEN                                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function generates named based UUID using        */
/*                     MAC-Address. It uses SHA1 for hashing                 */
/*                                                                           */
/*  Outputs          : UUID                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_gen_uuid(UWORD8 *mac_addr, UWORD8 *uuid, UWORD8 *temp)
{
    memcpy(temp, g_nsid, NSID_LEN);
    memcpy(temp+16, mac_addr, MAC_ADDRESS_LEN);

    sha1(temp, (UWORD32)(MAC_ADDRESS_LEN + NSID_LEN ), temp);
    memcpy(uuid, temp, WPS_UUID_LEN);

    /* Version: 5 = The name-based version specified in RFC 4122 that uses   */
    /* SHA-1 hashing.                                                        */
    uuid[6] = (5 << 4) | (uuid[6] & 0x0f);

    /* Variant specified in RFC 4122 (10 in two MSBS) */
    uuid[8] = 0x80 | (uuid[8] & 0x3f);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_get_rand_byte_array                               */
/*                                                                           */
/*  Description      : This function generate UUID using MAC address         */
/*                                                                           */
/*  Inputs           : 1) Pointer input buffer                               */
/*                   : 2) Length of Random bytes to be generated             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function generates Random bytes of specified     */
/*                     length                                                */
/*                                                                           */
/*  Outputs          : UUID                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_get_rand_byte_array(UWORD8* inp_ptr, UWORD16 arr_len)
{
    UWORD16 cnt;
    for(cnt =0; cnt < arr_len; cnt++)
    {
        *inp_ptr++ = (UWORD8) get_random_byte();
    }
}


#endif /* INT_WPS_REG_SUPP */
