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
/*  File Name         : rsna_km.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions and definitions     */
/*                      required for the operation of RSNA Key Management.   */
/*                                                                           */
/*  List of Functions : cipsuite_to_ctype                                    */
/*                      incr_cnt                                             */
/*                      get_iv_2_pn_val                                      */
/*                      cmp_pn_val                                           */
/*                      initialize_rsna                                      */
/*                      install_psk                                          */
/*                      verifyMIC                                            */
/*                      add_eapol_mic                                        */
/*                      rsna_send_deauth                                     */
/*                      init_sec_auth                                        */
/*                      compute_psk                                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "core_mode_if.h"
#include "mac_init.h"
#include "rsna_km.h"
#include "ieee_1x.h"
#include "mib_11i.h"
#include "utils.h"
#include "eapol_key.h"
#include "receive.h"
#include "md5.h"
#include "transmit.h"
#include "frame_11i.h"
#include "mh.h"
#include "ce_lut.h"
#include "prot_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

BOOL_T g_psk_available                   = BFALSE;

#ifdef UTILS_11I
static UWORD8 saved_ssid_len                = 0;
static UWORD8 saved_password_len            = 0;
static UWORD8 saved_password[64]            = {0};
static UWORD8 saved_ssid[MAX_SSID_LEN]      = {0};
UWORD8 saved_psk[40]                 = {0};

//chenq add for ap wpa enc 0724
extern UWORD8 g_psk_value[40];

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : cmp_pn_val                                            */
/*                                                                           */
/*  Description      : This function compares the PN value for replay        */
/*                     detection.                                            */
/*                                                                           */
/*  Inputs           : 1) Pointer to old PN value                            */
/*                     2) Pointer to received PN value                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function gets the PN values from the given PN    */
/*                     pointers and compares the obtained values.            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T, Comparision result                            */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T cmp_pn_val(UWORD8 *pn_val_old, UWORD8 *pn_val_rx)
{
    BOOL_T  ret_val      = BFALSE;
    UWORD32 old_val_low  = 0;
    UWORD32 old_val_high = 0;
    UWORD32 rx_val_low   = 0;
    UWORD32 rx_val_high  = 0;


    old_val_low  = pn_val_old[0] | (pn_val_old[1] << 8) |
                   (pn_val_old[2] << 16) | (pn_val_old[3] << 24);
    old_val_high = pn_val_old[4] | (pn_val_old[5] << 8);

    rx_val_low  = pn_val_rx[0] | (pn_val_rx[1] << 8) |
                  (pn_val_rx[2] << 16) | (pn_val_rx[3] << 24);
    rx_val_high = pn_val_rx[4] | (pn_val_rx[5] << 8);

    /* If the received PN Value is newer, that the old value is updated */
    /* and successful comparision is returned                           */
    if(((old_val_high == rx_val_high) && (old_val_low < rx_val_low)) ||
       (old_val_high < rx_val_high))
    {
//yiming.li changed for trout wifi to connect WCN-G300 AP.
#if 0
        if((rx_val_high - old_val_high) >= 0xF)
        {
            ret_val = BFALSE;
        }
        else
#endif
        {
            memcpy(pn_val_old, pn_val_rx, 6);
            ret_val = BTRUE;
    	}
    }

    return ret_val;
}
#endif /* UTILS_11I */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_iv_2_pn_val                                       */
/*                                                                           */
/*  Description      : This function extracts the PN value from the          */
/*                     transmitted IV value.                                 */
/*                                                                           */
/*  Inputs           : 1) IV lower portion                                   */
/*                     2) IV higher portion                                  */
/*                     3) Cipher suite                                       */
/*                     4) Pointer to the PN value                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function extracts the PN value from the given IV */
/*                     field based on the cipher suite type.                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void get_iv_2_pn_val(UWORD32 iv_l, UWORD32 iv_h, UWORD8 cipsuite,
                     UWORD32 *pn_val)
{
    pn_val[0] = 0;
    pn_val[1] = 0;

    switch(cipsuite)
    {
        case TKIP:
        {
            /* TKIP TSC Value as contained in the IV and extended IV field */
            /* ----------------------------------------------------------- */
            /*    IV / KeyID              |    Extended IV                 */
            /*    (4 octets)              |    (4 octets)                  */
            /* TSC1  WEPSeed  TSC0  FLAGs      TSC2 TSC3 TSC4 TSC5         */
            pn_val[0] = ((iv_l & 0x00FF0000) >> 16);
            pn_val[0] |= ((iv_l & 0x000000FF) << 8);
            pn_val[0] |= ((iv_h & 0x000000FF) << 16);
            pn_val[0] |= ((iv_h & 0x0000FF00) << 16);
            pn_val[1] = ((iv_h & 0x00FF0000) >> 16);
            pn_val[1] |= ((iv_h & 0xFF000000) >> 16);
        }
        break;
        case CCMP:
        {
            /* CCMP PN Value as contained in the CCMP Header field   */
            /* ----------------------------------------------------- */
            /*  PN0  PN1  Rsvd  FLAGs  PN2  PN3  PN4  PN5            */
            pn_val[0] =  iv_l & 0x000000FF;
            pn_val[0] |= (iv_l & 0x0000FF00);
            pn_val[0] |= ((iv_h & 0x000000FF) << 16);
            pn_val[0] |= ((iv_h & 0x0000FF00) << 16);
            pn_val[1] =  ((iv_h & 0x00FF0000) >> 16);
            pn_val[1] |= ((iv_h & 0xFF000000) >> 16);
        }
        break;
        default:
        break;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : incr_cnt                                              */
/*                                                                           */
/*  Description      : This function increments the counter of a given size. */
/*                                                                           */
/*  Inputs           : 1) Pointer to the counter buffer                      */
/*                     2) Size of the counter                                */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function increments the array values for the     */
/*                     given size.                                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void incr_cnt(UWORD8 *cntr, UWORD8 size)
{
    WORD16 index = (WORD16)(size - 1);

    while(index >= 0)
    {
        cntr[index]++;
        if(cntr[index] != 0)
        {
            break;
        }
        else
        {
            index --;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : cipsuite_to_ctype                                     */
/*                                                                           */
/*  Description      : This function converts the RSNA 802.11I cipher types  */
/*                     to LUT cipher types.                                  */
/*                                                                           */
/*  Inputs           : 1) Cipher suite                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function maps the given RSNA 802.11I cipher type */
/*                     to the pre-defined LUT cipher type.                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : CIPHER_T, LUT cipher type                             */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

CIPHER_T cipsuite_to_ctype(UWORD8 cipsuite)
{
    CIPHER_T ct = NO_ENCRYP;

    switch(cipsuite)
    {
        case 1:
            ct = WEP40;
            break;
        case 2:
            ct = (CIPHER_T) TKIP;
            break;
        case 4:
            ct = (CIPHER_T) CCMP;
            break;
        case 5:
            ct = WEP104;
            break;
        default:
            ct = NO_ENCRYP;
            break;
    }
    return ct;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : initialize_rsna                                       */
/*                                                                           */
/*  Description      : This function initializes the RSNA KM state machine.  */
/*                     It is called after MAC is enabled.                    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void initialize_rsna(void)
{
	TROUT_FUNC_ENTER;
    /* Initialize the PSK with the value FALSE */
    g_psk_available = BFALSE;

    /* Check if PSK is available and enabled */
    if(check_auth_policy(0x02) == BTRUE)
    {
        g_psk_available = BTRUE;
    }

    /* Before initializing RSNA, first stop it if already started */
    stop_sec_km();

    /* Initialized mode specific RSNA KM FSMs */
    intialize_sec_km();
    TROUT_FUNC_EXIT;
}

#ifdef UTILS_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : install_psk                                           */
/*                                                                           */
/*  Description      : This function initializes the PSK computation and     */
/*                     installs the same in the MIB                          */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void install_psk(void)
{
// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
    /*Compute the PSK again if SSID or Passphrase has changed */
    if(  (saved_ssid_len     != strlen(mget_DesiredSSID()))               ||
         (saved_password_len != mget_RSNAConfigPSKPassPhraseLength())     ||
         (memcmp(saved_ssid, mget_DesiredSSID(),MAX_SSID_LEN))            ||
         (memcmp(saved_password, mget_RSNAConfigPSKPassPhraseValue(),64)))
    {
    /* Copy the password, SSID and ssid len used to calculate the PSK */
        memcpy(saved_password, mget_RSNAConfigPSKPassPhraseValue(),64);
        memcpy(saved_ssid, mget_DesiredSSID(), MAX_SSID_LEN);
        saved_ssid_len     = strlen((const char *)saved_ssid);
        saved_password_len = mget_RSNAConfigPSKPassPhraseLength();

        if(MAX_PSK_PASS_PHRASE_LEN != saved_password_len)
        {
            /* Obtaining PBKDF2 digest for the password as the PSK */
            pbkdf2_sha1(saved_password, saved_password_len, saved_ssid,
                        saved_ssid_len, saved_psk);
        }
        else
        {
            UWORD8 i = 0;
            for(i = 0; i < (MAX_PSK_PASS_PHRASE_LEN/2); i++)
            {
                saved_psk[i] = char_2_hex(saved_password[2*i]) * 16 +
                    char_2_hex(saved_password[2*i + 1]);
            }
        }
    }

    /* Setting the MIB PSK with the psk obtained or saved psk */
    mset_RSNAConfigPSKValue(saved_psk);
#else
    /*Compute the PSK again if SSID or Passphrase has changed */
#ifdef	BSS_ACCESS_POINT_MODE
	return ;
#endif

    if(  (saved_ssid_len     != strlen(mget_DesiredSSID()))               ||
         (saved_password_len != mget_RSNAConfigPSKPassPhraseLength())     ||
         (memcmp(saved_ssid, mget_DesiredSSID(),MAX_SSID_LEN))            ||
         (memcmp(saved_password, mget_RSNAConfigPSKPassPhraseValue(),64)))
    {
    	#ifdef IBSS_BSS_STATION_MODE
		//chenq add 2013-01-10 for inter 11i
		if(g_psk_value[39] == 1)
		{
			/* Setting the MIB PSK with the psk obtained or saved psk */
    		mset_RSNAConfigPSKValue(g_psk_value);
			return;
		}else if(0 == mget_RSNAConfigPSKPassPhraseLength()) {
              //qin.chen, 2013-10-16, add
              printk("%s mget_RSNAConfigPSKPassPhraseLength == 0\n", __FUNCTION__);
              return;
		}
		#endif

	
    	/* Copy the password, SSID and ssid len used to calculate the PSK */
        memcpy(saved_password, mget_RSNAConfigPSKPassPhraseValue(),64);
        memcpy(saved_ssid, mget_DesiredSSID(), MAX_SSID_LEN);
        saved_ssid_len     = strlen((const char *)saved_ssid);
        saved_password_len = mget_RSNAConfigPSKPassPhraseLength();

         /* Check if the device has switched from AP->STA or STA->AP. If the */
         /* device has switched and the PSK is already present then the same */
         /* PSK is used                                                      */
         if((BFALSE == is_switch_in_progress()) ||
            (0 == strlen((const char *)mget_RSNAConfigPSKValue())))
         {
			if(MAX_PSK_PASS_PHRASE_LEN != saved_password_len)
        	{
            	/* Obtaining PBKDF2 digest for the password as the PSK */
            	pbkdf2_sha1(saved_password, saved_password_len, saved_ssid,
                        saved_ssid_len, saved_psk);
        	}
        	else
        	{
            	UWORD8 i = 0;
            	for(i = 0; i < (MAX_PSK_PASS_PHRASE_LEN/2); i++)
            	{
                	saved_psk[i] = char_2_hex(saved_password[2*i]) * 16 +
                    char_2_hex(saved_password[2*i + 1]);
            	}
        	}

    		/* Setting the MIB PSK with the psk obtained or saved psk */
    		mset_RSNAConfigPSKValue(saved_psk);
		}
    }

#endif
	TROUT_FUNC_EXIT;

}
#endif /* UTILS_11I */


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_eapol                                            */
/*                                                                           */
/*  Description      : This function sends an eapol frame                    */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Desntiation address                 */
/*                     2) Buffer                                             */
/*                     3) Buffer Length                                      */
/*                     4) If the port status is to be overriden              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T send_eapol(UWORD8 *addr, UWORD8 *buffer, UWORD16 len, BOOL_T secured)

{
    wlan_tx_req_t   wlan_tx_req = {{0},};

    /* Set the destination address field in the WLAN Tx Request structure. */
    mac_addr_cpy(wlan_tx_req.da, addr);

    /* For AP, the source address should be sent with the frame. */
    mac_addr_cpy(wlan_tx_req.sa, mget_StationID());

    wlan_tx_req.data          = buffer + MAX_MAC_HDR_LEN;

    /* Set the data length parameter to the MAC data length only (does   */
    /* not include headers)                                              */
    wlan_tx_req.data_len      = len;

    /* Set the buffer pointer field in the WLAN Tx Request to the start      */
    /* address of the buffer.                                                */
    wlan_tx_req.buffer_addr   = buffer;

    /* Set the ethernet type to One X */
    wlan_tx_req.eth_type = 0x888E;

    /* Set all fields for the transmit structure */
    /* BUG-ID:IWLANNPLFSW_1 */
    /* Send EAPOL frames with Normal ACK policy */
    wlan_tx_req.service_class = NORMAL_ACK;

#ifdef MAC_WMM
    if(get_wmm_enabled() == BTRUE)
        wlan_tx_req.priority = PRIORITY_3;
    else
        wlan_tx_req.priority = HIGH_PRIORITY;
#else /* MAC_WMM */
    wlan_tx_req.priority = HIGH_PRIORITY;
#endif /* MAC_WMM */

    /* For all the packets originating from this function, necessarily have  */
    /* the RSNA Handles set with necessary attributes. However if the packet */
    /* need to go unencyrpted, the portValid status should be overriden and  */
    /* the packet must be aired unencrypted                                  */
    /* Set security related parameters in WLAN TX Struct                     */
    if(secured == BFALSE)
    {
        wlan_tx_req.ignore_port = BTRUE;
    }
    else
    {
        wlan_tx_req.ignore_port = BFALSE;
    }

    /* EAPOL frames currently have no additional space available for  */
    /* inserting additional frame headers. Hence AMSDU aggregation of */
    /* these frames is not possible.                                  */
    wlan_tx_req.dont_aggr = BTRUE;

    /* Mark the packet as unprocessed */
    wlan_tx_req.added_to_q = BFALSE;

    /* Transmit the EAPOL packet at Min Basic Rate */
    wlan_tx_req.min_basic_rate = BTRUE;

    /* Call the wlan_tx_data() API to transmit the packet. Note that the     */
    /* buffer for the request structure is not freed here. It should be      */
    /* freed after the request has been processed.                           */
    wlan_tx_data(&g_mac, (UWORD8 *)&wlan_tx_req);

    /* Return the status of the packet */
    return (wlan_tx_req.added_to_q);
}


#ifdef UTILS_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : verifyMIC                                             */
/*                                                                           */
/*  Description      : This procedure verifies EAPOL MIC in the rxed frame.  */
/*                                                                           */
/*  Inputs           : 1) Pointer to the EAPOL Packet                        */
/*                     2) Length of the received EAPOL Frame                 */
/*                     3) Pointer to the key in use                          */
/*                     4) The key version in use to indentify cipher type    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This procedure verifies EAPOL MIC in the rxed frame.  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE if the MIC Passes, BFALSE otherwise             */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T verifyMIC(UWORD8 *eapol_buffer, UWORD16 eapol_length, UWORD8 *key,
                    UWORD8 key_version)
{
    UWORD8 *kck      = NULL;
    BOOL_T ret_val   = BFALSE;
    UWORD8 *mic_rcvd = NULL;
    UWORD8 *mic_cmp  = NULL;
    UWORD16 curr_idx = 0;

    if(eapol_buffer == NULL)
    {
        return BFALSE;
    }

    /* Save the current scratch memory index */
    curr_idx = get_scratch_mem_idx();

    /* Allocate memory to store the calculated MIC */
    mic_cmp = (UWORD8 *)scratch_mem_alloc(HMAC_DIGEST_LENGTH);
    if(mic_cmp == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return BFALSE;
    }

    /* Allocate memory to store the received MIC */
    mic_rcvd = (UWORD8 *)scratch_mem_alloc(HMAC_DIGEST_LENGTH);
    if(mic_rcvd == NULL)
    {
        /* Restore the saved scratch memory index */
        restore_scratch_mem_idx(curr_idx);

        return BFALSE;
    }

    /* Copy the received mic in the temporary buffer */
    memcpy(mic_rcvd, eapol_buffer + MIC_OFFSET + EAPOL_HDR_LEN, MIC_SIZE);

    /* Reset the MIC Field in the received EAPOL Buffer */
    mem_set(eapol_buffer + MIC_OFFSET + EAPOL_HDR_LEN, 0, MIC_SIZE);

    /* Obtain the KCK from the PTK. This will be used in the MIC computation.*/
    kck = get_kck(key);

    /* Calculate MIC over the body of the EAPOL-Key frame with Key MIC */
    /* field zeroed. The MIC is computed using the KCK derived from PTK*/
    if(key_version == 1)
    {
        /* MIC is defined as HMAC-MD5 */
        hmac_md5(eapol_buffer, eapol_length, kck, KCK_LENGTH, mic_cmp);
    }
    else if(key_version == 2)
    {
        /* MIC is defined as HMAC-SHA1-128. 160 bits of MIC is computed  */
        /* and placed in the mic buffer. On setting the MIC field in the */
        /* EAPOL Key frame only the first 128 bits get copied.           */
        hmac_sha1(kck, KCK_LENGTH, eapol_buffer, eapol_length, mic_cmp);
    }

    if(memcmp(mic_cmp, mic_rcvd , MIC_SIZE) == 0)
    {
        ret_val = BTRUE;
    }

    /* Restore the saved scratch memory index */
    restore_scratch_mem_idx(curr_idx);

    return ret_val;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name    : add_eapol_mic                                         */
/*                                                                           */
/*  Description      : This procedure adds the EAPOL MIC in the EAPOL frame. */
/*                                                                           */
/*  Inputs           : 1) Pointer to the EAPOL Packet                        */
/*                     2) Length of the received EAPOL Frame                 */
/*                     3) Pointer to the key in use                          */
/*                     4) The key version in use to indentify cipher type    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This procedure adds the EAPOL MIC in the EAPOL frame. */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void add_eapol_mic(UWORD8 *eapol_buffer, UWORD16 eapol_size, UWORD8 *key,
                    UWORD8 key_version)
{
    UWORD8 *kck = NULL;
    UWORD8 mic[HMAC_DIGEST_LENGTH] = {0};

    /* Obtain the KCK from the PTK. This will be used in the MIC computation.*/
    kck = get_kck(key);

    if(key_version == 1)
    {
        /* Key counter is used as the IV, therefore increment the Key */
        /* counter                                                    */

        /* MIC is defined as HMAC-MD5 */
        hmac_md5(eapol_buffer - EAPOL_HDR_LEN, eapol_size + EAPOL_HDR_LEN,
            kck, KCK_LENGTH, mic);
    }
    else if(key_version == 2)
    {
        /* MIC is defined as AES_CBC_MAC. MIC is computed and placed in  */
        /* the mic buffer.                                               */
        hmac_sha1(kck, KCK_LENGTH, eapol_buffer - EAPOL_HDR_LEN,
            eapol_size + EAPOL_HDR_LEN, mic);
    }

    /* Set the MIC field in the EAPOL frame and free the MIC buffer. The key */
    /* data buffer is also freed.                                            */
    set_mic_field(eapol_buffer, mic);
}

#endif /* UTILS_11I */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : rsna_send_deauth                                      */
/*                                                                           */
/*  Description      : This procedure sends Deauth frame for a given RSNA S/M*/
/*                                                                           */
/*  Inputs           : 1) Pointer to the RSNA Handle                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void rsna_send_deauth(UWORD8 *rsna_ptr, UWORD8 supp)
{
    /* An event of type MISC is loaded in the event Q to send Deuth */
    /* Also a Deauth frame is send to the STA                       */
     misc_event_msg_t *misc       = 0;

    /* Create a MISCELLANEOUS event with the pointer to the descriptor and   */
    /* post it to the event queue.                                           */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(misc == NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */

        /* Exception */
        raise_system_error(NO_EVENT_MEM);
        return;
    }

    misc->data = (UWORD8 *)rsna_ptr;
    if(supp != 0)
    {
        misc->name = MISC_SUPP_SND_DEAUTH;
    }
    else
    {
        misc->name = MISC_AUTH_SND_DEAUTH;
    }

    post_event((UWORD8*)misc, MISC_EVENT_QID);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_sec_auth                                         */
/*                                                                           */
/*  Description      : This procedure initializes the authentication policy  */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void init_sec_auth(void)
{
	TROUT_FUNC_ENTER;
    init_sec_auth_policy();
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : compute_psk                                           */
/*                                                                           */
/*  Description      : This procedure initializes and installs the PSK       */
/*                                                                           */
/*  Inputs           : None                                                  */
/*  Globals          : None                                                  */
/*  Processing       : None                                                  */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void compute_psk(void)
{
    compute_install_sec_key();
}

#endif /* MAC_802_11I*/
