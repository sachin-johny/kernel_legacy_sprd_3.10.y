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
/*  File Name         : wep.c                                                */
/*                                                                           */
/*  Description       : This file contains all the WEP management related    */
/*                      functions.                                           */
/*                                                                           */
/*  List of Functions : init_wep                                             */
/*                      init_wep_keys                                        */
/*                      add_wep_entry                                        */
/*                      msg_3_timeout                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "management.h"
#include "mib.h"
#include "mh.h"
#include "wep.h"
#include "ce_lut.h"

/*****************************************************************************/
/* Global Variable Declarations                                              */
/*****************************************************************************/

UWORD8 g_wep_type = 0;     /* Used to determine the size of the WEP Key used */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_wep                                                 */
/*                                                                           */
/*  Description   : This function initializes the WEP keys                   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the WEP keys                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_wep(void)
{
    UWORD8 key_size = 0;

	TROUT_FUNC_ENTER;
    /* Set the key size based on the key */
    key_size = mget_WEPDefaultKeySize();

    /* Add the configured Keys to the mac hardware */
    init_wep_keys(mget_WEPDefaultKeyID(), mget_WEPDefaultKeyValue(), key_size);

    /* Add an entry in the LUT for the Broadcast and Multicase Packets */
    add_wep_entry(0, mget_WEPDefaultKeyID(), mget_bcst_addr());
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_wep_keys                                            */
/*                                                                           */
/*  Description   : This function sets the given key in the MIB              */
/*                                                                           */
/*  Inputs        : 1) Key ID                                                */
/*                  2) Pointer to WEP Key                                    */
/*                  3) Size of the WEP Key                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the given key in the MIB              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_wep_keys(UWORD8 idx, UWORD8* key, UWORD8 size)
{
    /* Add the key to the MIB */
    mset_WEPKeyValue(idx, key, size);

    /* Add the key to the LUT */
    add_wep_key(idx);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_wep_entry                                            */
/*                                                                           */
/*  Description   : This function adds the station to the WEP list in the    */
/*                  MAC Hardware                                             */
/*                                                                           */
/*  Inputs        : 1) The Station Index                                     */
/*                  2) Key ID                                                */
/*                  3) The pointer to the Station Address                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the MSDU Indicate function with the  */
/*                  required structure as input.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void add_wep_entry(UWORD8 index, UWORD8 keyid, UWORD8* addr)
{
    UWORD8 hkey[16]              = {0};
    UWORD8 *key                  = 0;
    UWORD8 key_size              = 0;
    UWORD8 ct                    = 0;
    UWORD8 i                     = 0;
    UWORD8 mask                  = 0;

    /* Get the Key value and key size value from the MIB */
    key      = mget_WEPKeyValue(keyid);
    key_size = mget_WEPKeySize(keyid);

    mask     = LUT_ADDR_PRGM_MASK;

    switch(key_size)
    {
        case 40:
            /* If Key size is 40, set cipher type to WEP40 */
            /* and key size is updated to 5                */
            ct       = WEP40;
            key_size = 5;
            break;
        case 104:
            /* If Key size is 104, set cipher type to WEP104 */
            /* and key size is updated to 13                 */
            ct       = WEP104;
            key_size = 13;
            break;
        default:
            /* If Key size is unknown, set cipher type to WEP40 */
            /* and key size is updated to 5                     */
            ct       = WEP40;
            key_size = 5;
            break;
    }

    for(i = 0; i < key_size; i++)
    {
        hkey[i] = key[i];
    }

    /* Set the cipher type in RX GTK Cipher Type register */
    if(mac_addr_cmp(addr, mget_bcst_addr()) == BTRUE)
    {
        set_machw_ce_grp_key_type(ct);
    }

    /* Add Sta Key   */
    machw_ce_add_key(keyid, TX_GTK_NUM, index, mask, ct, hkey, addr,
                     AUTH_KEY, NULL);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_wep_key                                              */
/*                                                                           */
/*  Description   : This function adds the WEP key of from the MIB to LUT in */
/*                  MAC Hardware                                             */
/*                                                                           */
/*  Inputs        : 1) Key ID                                                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the MSDU Indicate function with the  */
/*                  required structure as input.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void add_wep_key(UWORD8 keyid)
{
    UWORD8 hkey[16]              = {0};
    UWORD8 addr[6]               = {0};
    UWORD8 *key                  = 0;
    UWORD8 key_size              = 0;
    UWORD8 ct                    = 0;
    UWORD8 i                     = 0;
    UWORD8 mask                  = 0;

    /* Get the Key value and key size value from the MIB */
    key      = mget_WEPKeyValue(keyid);
    key_size = mget_WEPKeySize(keyid);

    mask     = LUT_KEY_PRGM_MASK;

    switch(key_size)
    {
        case 40:
            /* If Key size is 40, set cipher type to WEP40 */
            /* and key size is updated to 5                */
            ct       = WEP40;
            key_size = 5;
            break;
        case 104:
            /* If Key size is 104, set cipher type to WEP104 */
            /* and key size is updated to 13                 */
            ct       = WEP104;
            key_size = 13;
            break;
        default:
            /* If Key size is unknown, set cipher type to WEP40 */
            /* and key size is updated to 5                     */
            ct       = WEP40;
            key_size = 5;
            break;
    }

    /* Programming the hardware key as required */
    for(i = 0; i < key_size; i++)
    {
        hkey[i] = key[i];
    }

    /* Add Sta Key   */
    machw_ce_add_key(keyid, TX_GTK_NUM, 0, mask, ct, hkey, addr,
                     AUTH_KEY, NULL);
}
