/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
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
/*  File Name         : wps_cred_mgmt_ap.c                                   */
/*                                                                           */
/*  Description       : This file contains the all the functions that        */
/*                      implement the Credential Managment functionality of  */
/*                      WPS internal Registrar                               */
/*                                                                           */
/*  List of Functions : wps_get_cur_cred_ap                                  */
/*                      wps_get_num_cred_ap                                  */
/*                      wps_handle_cred_update_ap                            */
/*                      wps_process_wid_write_ap                             */
/*                      wps_update_auth_encr_type                            */
/*                      wps_apply_cred_ap                                    */
/*                      wps_store_cred_eeprom_ap                             */
/*                      wps_get_cred_eeprom_ap                               */
/*                      wps_get_cred_store_ap                                */
/*                      wps_handle_oob_reset_req_ap                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef INT_WPS_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_ap.h"
#include "wps_cred_mgmt_ap.h"
#include "iconfig.h"
#include "imem_if.h"
#include "mac_init.h"

/*****************************************************************************/
/* Global Variable Definitions                                               */
/*****************************************************************************/

BOOL_T g_cred_updated_ap  = BFALSE;
BOOL_T g_wps_wid_restore_in_prog = BFALSE;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void wps_update_auth_encr_type(UWORD16 *wps_auth_type,
                                      UWORD16 *wps_encr_type,
                                      UWORD8  cust_auth_type,
                                      UWORD8  cust_sec_mode);
static void wps_store_cred_eeprom_ap(wps_store_cred_t* store_cred);
static void wps_get_cred_store_ap(wps_store_cred_t *wps_cred);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_get_cur_cred_ap                                   */
/*                                                                           */
/*  Description      : This function gets the current credential of the AP   */
/*                                                                           */
/*  Inputs           : 1) Pointer to Credential Structure                    */
/*                     2) Credential Index                                   */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function updates the input buffer with AP`s      */
/*                     credential at specified input index in wps_cred_t     */
/*                     format. This does the following processing:           */
/*                     1) If the input Credential index is greater than      */
/*                        number of simultaneously credentials supported then*/
/*                        return BFALSE                                      */
/*                     2) Update the input buffer with AP`s Credential at    */
/*                        specified input index in wps_cred_t format and     */
/*                        BTRUE                                              */
/*                                                                           */
/*  Outputs          : Status of Credential availability at specified index  */
/*                                                                           */
/*  Returns          : BTRUE  - If valid Credential Present at given index   */
/*                     BFALSE - Otherwise                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_get_cur_cred_ap(wps_cred_t *wps_cred, UWORD8 cred_index)
{
    UWORD8 auth_type = 0;
    UWORD8 sec_mode  = 0;
    UWORD8 i = 0;
    WORD8  *ssid_temp = 0;
    wps_net_key_t *net_key_info = wps_cred->net_key_info;

    if(MAX_AP_NW_CRED_SUPP <= cred_index)
        return BFALSE;

    /* Note that since AP supports only one Credential at any given time,    */
    /* Current interface functions available for get of parameters are used. */
    /* Ideally these get functions should return value based on the index    */
    /* passed as input, where the index specifies the Credential index for   */
    /* which the value needs to be returned                                  */

    /* Get the authentication and security mode */
    auth_type = get_auth_type();
    sec_mode  = get_802_11I_mode();

    /* Convert the custom format for encoding security and authentication    */
    /* type to the format specified in WPS standard                          */
    wps_update_auth_encr_type(&wps_cred->auth_type, &wps_cred->encr_type,
                              auth_type, sec_mode);

    /* Update the SSID */
    ssid_temp = mget_DesiredSSID();
    strcpy((WORD8 *)wps_cred->ssid, ssid_temp);
    wps_cred->ssid_len = strlen(ssid_temp);

    /* Set the number of Keys to 1. Simultaneous support for multiple keys   */
    /* is not supported                                                      */
    wps_cred->num_key = 1;

    for(i = 0; i < wps_cred->num_key; i++)
    {
        /* Update the MAC address of the STA in the network key information  */
        /* structure                                                         */
        mac_addr_cpy(net_key_info[i].mac_addr, g_wps_config_ptr->mac_addr);

#ifdef WPS_1_0_SEC_SUPP
        /* If the Encryption is WEP then update the Key index, Key length and*/
        /* Network key members of the network key information structure.     */
        if(wps_cred->encr_type & WPS_WEP)
        {
            /* Key Size in Hex Digits */
            UWORD8 key_len = mget_WEPDefaultKeySize() >> 2;
            UWORD8 idx = 0;
            UWORD8 j = 0;
            UWORD8 *dst_key_ptr = net_key_info[i].net_key;
            UWORD8 *src_key_ptr = mget_WEPDefaultKeyValue();

            net_key_info[i].net_key_index = mget_WEPDefaultKeyID();
            net_key_info[i].net_key_len = key_len;

            for(j = 0; j < (key_len >> 1); j++)
            {
                UWORD8 upper_nibble = (src_key_ptr[j] & 0xF0) >> 4;
                UWORD8 lower_nibble = (src_key_ptr[j] & 0x0F);

                dst_key_ptr[idx++] = hex_2_char(upper_nibble);
                dst_key_ptr[idx++] = hex_2_char(lower_nibble);
            }
        }
        else
#endif /* WPS_1_0_SEC_SUPP */
        {
            /* Update the Ley length and PSK */
            net_key_info[i].net_key_index = 0;
            if(wps_cred->encr_type & WPS_NONE)
            {
                net_key_info[i].net_key_len = 0;
            }
            else
            {
                net_key_info[i].net_key_len = mget_RSNAConfigPSKPassPhraseLength();
                memcpy(net_key_info[i].net_key,
                       mget_RSNAConfigPSKPassPhraseValue(),
                       mget_RSNAConfigPSKPassPhraseLength());
            }
        }
    }

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_get_num_cred_ap                                   */
/*                                                                           */
/*  Description      : This function returns the number of simultaneous      */
/*                     credentials supported by the AP                       */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function returns the number of simultaneous      */
/*                     credentials supported by the AP                       */
/*                                                                           */
/*  Outputs          : Number of Credentials Supported                       */
/*                                                                           */
/*  Returns          : UWORD8 - Number of Credentials Supported              */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD8 wps_get_num_cred_ap(void)
{
    return MAX_AP_NW_CRED_SUPP;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_update_auth_encr_type                             */
/*                                                                           */
/*  Description      : This function maps the custom Encryption and          */
/*                     Authentication Type encoding into format specified in */
/*                     WPS standard                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS Authentication Type                 */
/*                     2) Pointer to WPS Encryption Type                     */
/*                     3) Custom Authentication Type                         */
/*                     4) Custom Security Mode                               */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function maps the custom Encryption and          */
/*                     Authentication Type encoding into format specified in */
/*                     WPS standard                                          */
/*                                                                           */
/*  Outputs          : Updates the WPS Authentication and Encryption Type    */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

static void wps_update_auth_encr_type(UWORD16 *wps_auth_type,
                                      UWORD16 *wps_encr_type,
                                      UWORD8  cust_auth_type,
                                      UWORD8  cust_sec_mode)
{
    UWORD8 wep_auth_type = (cust_auth_type & (BIT0|BIT1));
    UWORD8 wpa_auth_type = (cust_auth_type & BIT2) >> 2;
    UWORD8 wpa_type      = (cust_sec_mode & (BIT3|BIT4)) >> 3;
    UWORD8 wpa_encr_type = (cust_sec_mode & (BIT5|BIT6)) >> 5;

    /* Security Mode Bit Encoding                                            */
    /*************************************************************************/
    /* BIT0     : Security Mode (1-> Enabled, 0-> Disabled)                  */
    /* BIT[2:1] : WEP Type (01-> WEP-40, 11->WEP-104, 00/10-> Not valid)     */
    /* BIT[4:3] : 11i Type (01->WPA, 10->WPA2, 11->Mixed)                    */
    /* BIT[6:5] : Encryption Type (01->AES. 10->TKIP, 11->AES+TKIP)          */
    /*************************************************************************/

    /* Authentication Type Encoding                                          */
    /*************************************************************************/
    /* BIT[1:0] : WEP Authentication (01->Open, 10->Shared, 00/11->ANY)      */
    /* BIT2     : 11i Authentication (0->PSK, 1->802.1X)                     */
    /*************************************************************************/

    *wps_auth_type = 0;
    *wps_encr_type = 0;
#ifndef WPS_1_0_SEC_SUPP
    if(cust_sec_mode & BIT0)
    {
        /* WEP is enabled */
        if(cust_sec_mode & BIT1)
        {
            /* Exception WPS_REG_ENCR_EXC_WEP */
            send_wps_status(WPS_REG_ENCR_EXC_WEP, NULL, 0);

            if(AUTH_SHARED == wep_auth_type)
                /* Exception WPS_REG_AUTH_EXC_SHARED */
                send_wps_status(WPS_REG_AUTH_EXC_SHARED, NULL, 0);
        }
        else
        {
            if((ENCR_AES == wpa_encr_type) ||
               (ENCR_AES_TKIP == wpa_encr_type))
                *wps_encr_type = WPS_AES;
            else if(ENCR_TKIP == wpa_encr_type)
                /* Exception WPS_REG_ENCR_EXC_TKIP */
                send_wps_status(WPS_REG_ENCR_EXC_TKIP, NULL, 0);
            else
                *wps_encr_type = WPS_NONE;

            if(TYPE_11I_WPA == wpa_type)
            {
                /* Exception WPS_REG_AUTH_EXC_WPA */
                send_wps_status(WPS_REG_AUTH_EXC_WPA, NULL, 0);
            }
            else if((TYPE_11I_WPA2 == wpa_type) ||
                    (TYPE_11I_MIXED == wpa_type))
            {
                *wps_auth_type = (AUTH_1X == wpa_auth_type) ?
                                   WPS_WPA2 : WPS_WPA2_PSK;
            }
        }
    }
    else
    {
        *wps_encr_type = WPS_NONE;
        *wps_auth_type = WPS_OPEN;
    }
#else  /* WPS_1_0_SEC_SUPP */
    if(cust_sec_mode & BIT0)
    {
        /* WEP is enabled */
        if(cust_sec_mode & BIT1)
        {
            *wps_encr_type = WPS_WEP;

            if(AUTH_SHARED == wep_auth_type)
                *wps_auth_type = WPS_SHARED;
            else
                *wps_auth_type = WPS_OPEN;
        }
        else
        {
            if(ENCR_AES == wpa_encr_type)
                *wps_encr_type = WPS_AES;
            else if(ENCR_TKIP == wpa_encr_type)
                *wps_encr_type = WPS_TKIP;
            else if(ENCR_AES_TKIP == wpa_encr_type)
                *wps_encr_type = WPS_AES_TKIP;
            else
                *wps_encr_type = WPS_NONE;

            if(TYPE_11I_WPA == wpa_type)
            {
                *wps_auth_type = (AUTH_1X == wpa_auth_type) ?
                                   WPS_WPA : WPS_WPA_PSK;
            }
            else if((TYPE_11I_WPA2 == wpa_type) ||
                    (TYPE_11I_MIXED == wpa_type))
            {
                *wps_auth_type = (AUTH_1X == wpa_auth_type) ?
                                   WPS_WPA2 : WPS_WPA2_PSK;
            }
        }
    }
    else
    {
        *wps_encr_type = WPS_NONE;
        *wps_auth_type = WPS_OPEN;
    }
#endif /* WPS_1_0_SEC_SUPP */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_apply_cred_ap                                     */
/*                                                                           */
/*  Description      : This function configures the AP with passed credential*/
/*                                                                           */
/*  Inputs           : Pointer Stored Credential Structure                   */
/*                                                                           */
/*  Globals          : g_wps_reg_init_done                                   */
/*                                                                           */
/*  Processing       : This function checks if a valid credential has been   */
/*                     passed, then applies the credential using the         */
/*                     configuration interface of the system software        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_apply_cred_ap(wps_store_cred_t *store_cred)
{
    UWORD8 ssid_len = strlen((WORD8 *)store_cred->ssid);
    wps_wep_key_t *wep_key = &(store_cred->key.wep_key);

    /* Check if credential is valid, if not then return. SSID Length,        */
    /* Key length, Security mode and  Number of keys are checked             */
    if(((0 == ssid_len) || (MAX_SSID_LEN <= ssid_len)) ||
       (SEC_MODE_MASK < store_cred->sec_mode))
        return;

    switch(store_cred->sec_mode)
    {
        case SEC_MODE_WEP40:
        {
            if((WEP40_KEY_SIZE != store_cred->key_len) ||
               (MAX_NW_KEY_PER_CRED < store_cred->key.wep_key.key_id))
                return;


            mset_WEPDefaultKeyID(wep_key->key_id);

            mset_WEPKeyValue(mget_WEPDefaultKeyID(), wep_key->key, 40);
        }
        break;
        case SEC_MODE_WEP104:
        {
            if((WEP104_KEY_SIZE != store_cred->key_len) ||
               (MAX_NW_KEY_PER_CRED < store_cred->key.wep_key.key_id))
                return;

            mset_WEPDefaultKeyID(wep_key->key_id);

            mset_WEPKeyValue(mget_WEPDefaultKeyID(), wep_key->key, 104);
        }
        break;
        case SEC_MODE_NONE:
            /* Do Nothing */
        break;
        default: /* Non-WEP */
        {
            if((MAX_PSK_PASS_PHRASE_LEN < store_cred->key_len) ||
               (0 == store_cred->key_len))
                return;

            mset_RSNAConfigPSKPassPhrase(store_cred->key.psk,
                                         store_cred->key_len);
        }
        break;
    }

    /* Set SSID */
    mset_DesiredSSID((WORD8 *)store_cred->ssid);

    /* Set Authentication and Encryption Type */
    set_802_11I_mode(store_cred->sec_mode);
    set_auth_type(store_cred->auth_type);

    if(BFALSE == mac_addr_cmp(mget_StationID(), store_cred->bssid))
    {
        mset_StationID(store_cred->bssid);
        mset_bssid(store_cred->bssid);
    }

    /* Restart MAC only if WPS initialization is completed. This is done to  */
    /* prevent recursive calling of this function                            */
    if(BTRUE == g_wps_reg_init_done)
    {
        set_cred_updated_ap(BFALSE);
        restart_mac(&g_mac, 0);
    }

    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_cred_update_ap                             */
/*                                                                           */
/*  Description      : This function handles the updating of credential in   */
/*                     EEPROM when AP`s configuration is updated by the user */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the Credential was updated by */
/*                     the user, if not then returns. If updated then gets   */
/*                     current Credential of the AP in format that is stored */
/*                     in the EEPROM (wps_store_cred_t) and updates the same */
/*                     in the EEPROM.                                        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_handle_cred_update_ap(void)
{
    wps_store_cred_t wps_cred = {0};

    /* If there has been no credential update then return */
    if(BFALSE == get_cred_updated_ap())
        return;

    /* Get the current credential of the AP in wps_cred_store_t format */
    wps_get_cred_store_ap(&wps_cred);

    /* Store the Credential in EEPROM */
    wps_store_cred_eeprom_ap(&wps_cred);

    /* Reset the Credential Update flag */
    set_cred_updated_ap(BFALSE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_get_cred_store_ap                                 */
/*                                                                           */
/*  Description      : This function gets AP`s current credential in         */
/*                     wps_cred_store_t format                               */
/*                                                                           */
/*  Inputs           : 1) Pointer to Store Credential of the AP              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions gets the current credential of the AP  */
/*                     in wps_store_cred_t format that can be stored in the  */
/*                     EEPROM.                                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

static void wps_get_cred_store_ap(wps_store_cred_t *wps_cred)
{
    /* Update the Authentication type and Security Mode */
    wps_cred->auth_type = get_auth_type();
    wps_cred->sec_mode  = get_802_11I_mode();

    /* Depending upon the security mode update the Key ID, Key length, WEP   */
    /* key and PSK                                                           */
    switch(wps_cred->sec_mode)
    {
        case SEC_MODE_WEP40:
        case SEC_MODE_WEP104:
             if(SEC_MODE_WEP40 == wps_cred->sec_mode)
                wps_cred->key_len = WEP40_KEY_SIZE;
             else
                wps_cred->key_len = WEP104_KEY_SIZE;

            wps_cred->key.wep_key.key_id = mget_WEPDefaultKeyID();
            memcpy(wps_cred->key.wep_key.key,
                   mget_WEPKeyValue(mget_WEPDefaultKeyID()),
                   wps_cred->key_len);
        break;
        case SEC_MODE_NONE:
            wps_cred->key_len = 0;
        break;
        default: /* Non-WEP */
            wps_cred->key_len = mget_RSNAConfigPSKPassPhraseLength();
            memcpy(wps_cred->key.psk, mget_RSNAConfigPSKPassPhraseValue(),
                   wps_cred->key_len);
        break;
    }

    /* Update the SSID */
    strcpy((WORD8 *)wps_cred->ssid, mget_DesiredSSID());

    /* Update the BSSID */
    mac_addr_cpy(wps_cred->bssid, mget_bssid());
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_oob_reset_req_ap                           */
/*                                                                           */
/*  Description      : This function handles the OOB reset request from the  */
/*                     user                                                  */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function WPS related OOB reset request from the  */
/*                     user. It sets the current configuration of the AP with*/
/*                     SSID     : Wi-Fi_CUSTOM_AP                            */
/*                     Security : WPA2-AES PSK                               */
/*                     PSK      : Strong@$PSK                                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_handle_oob_reset_req_ap(void)
{
    WORD8 ssid[] = "Wi-Fi_CUSTOM_AP";
    UWORD8 psk[] = "Strong@$PSK";

    /* Set the Authentication Type */
    set_auth_type(OOB_RESET_AUTH_TYPE);

    /* Set the Security Mode */
    set_802_11I_mode(OOB_RESET_SEC_MODE);

    /* Set PSK */
    set_RSNAConfigPSKPassPhrase(psk);

    /* Set SSID */
    mset_DesiredSSID(ssid);

// 20120709 caisf add, merged ittiam mac v1.2 code
    /* Enable Broadcasting of SSID */
    set_bcst_ssid(0);

    /* Set the Credential update flag */
    set_cred_updated_ap(BTRUE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_store_cred_eeprom_ap                                 */
/*                                                                           */
/*  Description   : This function stores a single credential to persistent   */
/*                  memory. Currently nothing is done for MWLAN              */
/*                                                                           */
/*  Inputs        : 1) Pointer to credential to be saved                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function stores a single credential to persistent   */
/*                  memory. There can only be a single credential in the     */
/*                  persistent memory                                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

static void wps_store_cred_eeprom_ap(wps_store_cred_t* store_cred)
{
    /* TBD for MWLAN */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_get_cred_eeprom_ap                                   */
/*                                                                           */
/*  Description   : This function reads a single credential from the         */
/*                  persistent memory.Currently nothing is done for MWLAN    */
/*                                                                           */
/*  Inputs        : 1) Pointer to credential memory where read credential    */
/*                  needs to be stored                                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function reads a single credential from the         */
/*                  persistent memory and writes it in to the buffer pointed */
/*                  by the input pointer.There can only be a single          */
/*                  credential in the persistent memory                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : SUCCESS - Read sucess and valid credential read          */
/*                  FAILURE - Otherwise                                      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_get_cred_eeprom_ap(wps_store_cred_t *store_cred)
{
	STATUS_T retval = FAILURE;

    /* TBD for MWLAN */

	return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_wid_write_ap                              */
/*                                                                           */
/*  Description      : This function handles WID Write request during        */
/*                     registration protocol is in progress                  */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the current WID Write request */
/*                     can be processed further. This function does the      */
/*                     following processing:                                 */
/*                     1) Either if the Registrar is disabled or if flag     */
/*                        indicates WID config writes are allowed then       */
/*                        returns BTRUE                                      */
/*                     2) Parse in input message for WPS Stop request, if    */
/*                        present then return BTRUE, else return BFALSE      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - If WID write request can be processed further*/
/*                     BFALSE - Otherwise                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_process_wid_write_ap(UWORD8 *req, UWORD16 rx_len)
{
    BOOL_T  retval   = BFALSE;
    UWORD16 wid_len  = 0;
    UWORD16 wid_id   = 0;
    UWORD16 wid_type = 0;
    UWORD16 index    = 0;

    /* Check if Registrar is enabled and if config writes are allowed        */
    if((BFALSE == get_wps_reg_enabled()) ||
       (BTRUE  == get_wps_allow_config()))
       return BTRUE;

    /* Parse the input config request for WPS stop request. If found then    */
    /* return BTRUE. Note that WID_WPS_START+WPS_PROT_NONE impluies WPS stop */
    while(index < rx_len)
    {
        wid_id = MAKE_WORD16(req[index], req[index + 1]);
        index += 2;
        wid_type = (wid_id & 0xF000) >> 12;

        if(WID_BIN_DATA == wid_type)
        {
            wid_len = MAKE_WORD16(req[index], req[index + 1]);
            index += 2;
		}
        else
        {
            wid_len = req[index++];
		}

        if(WID_WPS_START == wid_id)
        {
            if(WPS_PROT_NONE == req[index])
            {
                retval = BTRUE;
                break;
            }
        }

        index += wid_len;
    }

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_update_device_info_ap                             */
/*                                                                           */
/*  Description      : This function updates the device specific information */
/*                     in WPS Configuration Structure                        */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : g_wps_config_ptr                                      */
/*                                                                           */
/*  Processing       : This function uses the configuration interface GET    */
/*                     functions to update the WPS device specific           */
/*                     information in WPS Configuration Structure            */
/*                                                                           */
/*  Outputs          : Updated WPS Configuration Structure                   */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_update_device_info_ap(void)
{
    UWORD8 *temp_ptr = NULL;

    if(g_wps_config_ptr == NULL)
        return;


    g_wps_config_ptr->os_version = get_dev_os_version();

    temp_ptr = get_prim_dev_type();
    if(temp_ptr==NULL)
        TROUT_DBG4("bad pointer !!!!!!");

    g_wps_config_ptr->prim_dev_cat_id = &temp_ptr[2];

#ifdef MAC_P2P
    temp_ptr = get_req_dev_type();

    if(temp_ptr==NULL)
        TROUT_DBG4("bad pointer !!!!!!");
	// 20120709 caisf add the if, merged ittiam mac v1.2 code
    if(REQ_DEV_TYPE_LEN == ((temp_ptr[1] << 8) + temp_ptr[0]))
	{
    	g_wps_config_ptr->req_dev_cat_id = &temp_ptr[2];
    }
#endif /* MAC_P2P */

    temp_ptr = get_manufacturer();
    if(temp_ptr==NULL)
        TROUT_DBG4("bad pointer !!!!!!");
    g_wps_config_ptr->manufacturer_len = temp_ptr[0];
    g_wps_config_ptr->manufacturer = &temp_ptr[1];

    temp_ptr = get_model_name();
    if(temp_ptr==NULL)
        TROUT_DBG4("bad pointer !!!!!!");
    g_wps_config_ptr->model_name_len = temp_ptr[0];
    g_wps_config_ptr->model_name = &temp_ptr[1];

    temp_ptr = get_model_num();
    if(temp_ptr==NULL)
        TROUT_DBG4("bad pointer !!!!!!");
    g_wps_config_ptr->model_num_len = temp_ptr[0];
    g_wps_config_ptr->model_num = &temp_ptr[1];

    temp_ptr = get_dev_name();
    if(temp_ptr==NULL)
        TROUT_DBG4("bad pointer !!!!!!");
    g_wps_config_ptr->device_name_len = temp_ptr[0];
    g_wps_config_ptr->device_name = &temp_ptr[1];

    temp_ptr = get_serial_number();
    if(temp_ptr==NULL)
        TROUT_DBG4("bad pointer !!!!!!");
    g_wps_config_ptr->serial_num_len = temp_ptr[0];
    g_wps_config_ptr->serial_num     = &temp_ptr[1];
}

#endif /* INT_WPS_SUPP */
#endif /* BSS_ACCESS_POINT_MODE */
