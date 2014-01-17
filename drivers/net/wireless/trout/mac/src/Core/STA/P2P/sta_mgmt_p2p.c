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
/*  File Name         : sta_mgmt_p2p.c                                       */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to P2P STA management.                       */
/*                                                                           */
/*  List of Functions : gen_p2p_ssid                                         */
/*                      p2p_match_dev_entry                                  */
/*                      parse_client_info_dscr                               */
/*                      p2p_get_dev                                          */
/*                      handle_scan_itr_to_p2p                               */
/*                      misc_p2p_event_sta                                   */
/*                      p2p_join_complete                                    */
/*                      p2p_end_grp_form                                     */
/*                      update_join_req_params_p2p                           */
/*                      init_p2p_sta_globals                                 */
/*                      store_user_settings_p2p                              */
/*                      restore_user_settings_p2p                            */
/*                      p2p_sta_wait_scan_handle_prb_req                     */
/*                      p2p_config_persist_cred                              */
/*                      print_dev_dscrp                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_P2P

// 20120709 caisf mod, merged ittiam mac v1.2 code
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "common.h"
#include "event_manager.h"
#include "management_sta.h"
#include "fsm_sta.h"
#include "controller_mode_if.h"
#include "p2p_ps_sta.h"
#include "wps_sta.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
/* Variables for group formation                                             */
BOOL_T g_p2p_grp_form_in_prog   = BFALSE; /* Flag to indicate P2P group      */
                                          /* formation is in progress        */
UWORD8 g_p2p_scan_state = 0;
UWORD8 g_p2p_find_phase_itr_cnt = 0;  /* count of the number of times LISTEN */
                                      /* SEARCH state transition happened    */
p2p_join_req_t g_p2p_join_req;        /* Saved join request                  */
UWORD8 g_p2p_join_state         = 0;  /* Current P2P join state              */

UWORD8 g_p2p_go_disc_req_rx     = 0;  /* Flag to indicate a GO discoverabilty*/
                                      /* request has been received by the    */
                                      /* client, to stay awake               */

WORD8 g_p2p_rand_ssid_pre[2]    = {0,}; /* Random chars in P2P SSID */
p2p_user_settings_t g_p2p_user_settings = {0}; /* saved p2p settings */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : gen_rnd_p2p_ssid                                         */
/*                                                                           */
/*  Description   : This function generates 2 random chars for P2P ssid      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_p2p_rand_ssid_pre                                      */
/*                                                                           */
/*  Processing    : This function generated a P2P SSID in the following      */
/*                  format DIRECT-xy where x & y are randomly selected with  */
/*                  a unified distribution from the following character set, */
/*                  upper case letters, lower case letters and numbers       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void gen_p2p_ssid(void)
{
    UWORD8 p2p_ssid[MAX_SSID_LEN] = {0,};
    UWORD8 rem_len                = 0;
    UWORD8 rnd_num                = 0;
    UWORD8 i                      = 0;

    memcpy(p2p_ssid, mget_DesiredSSID(), P2P_WILDCARD_SSID_LEN);

    if(0 == strcmp((WORD8 *)p2p_ssid, P2P_WILDCARD_SSID))
    {   /* The SSID already has a P2P wild card, copy the SSID */
        memcpy(p2p_ssid, mget_DesiredSSID(), MAX_SSID_LEN);
    }
    else
    {   /* No P2P wild card */

        /* Re-generate Random Prefix */
        g_p2p_rand_ssid_pre[0] = 0;

        /* Prefix the P2P wild card to user configured SSID */
        memcpy(p2p_ssid, P2P_WILDCARD_SSID, P2P_WILDCARD_SSID_LEN);
        rem_len = MIN(strlen(mget_DesiredSSID()),
                      MAX_SSID_LEN-1-P2P_WILDCARD_SSID_LEN-2);
        memcpy(p2p_ssid + P2P_WILDCARD_SSID_LEN + 2 , mget_DesiredSSID(),
               rem_len);
        p2p_ssid[P2P_WILDCARD_SSID_LEN + 2 + rem_len] = '\0';
    }

    if(0 == g_p2p_rand_ssid_pre[0])
    {
        /* Random variable space is 52 + 10. Generate a random number from */
        /* 0-63, and map it to 0 - 61                                      */
        for(i = 0; i < 2; i++)
        {
            rnd_num = get_random_byte() >> 2;
            if(rnd_num > 61)
                rnd_num = 61;

            if(rnd_num < 10)
            {   /* choose a digit */
                g_p2p_rand_ssid_pre[i] = '0' + rnd_num;
            }
            else if(rnd_num < 37)
            {   /* choose a Upper case alphabet */
                g_p2p_rand_ssid_pre[i] = 'A' + rnd_num - 10;
            }
            else
            {   /* choose a lower case alphabet */
                g_p2p_rand_ssid_pre[i] = 'a' + rnd_num - 37;
            }
        }
    }

    /* Change the random fields only */
    p2p_ssid[P2P_WILDCARD_SSID_LEN]     =  g_p2p_rand_ssid_pre[0];
    p2p_ssid[P2P_WILDCARD_SSID_LEN + 1] =  g_p2p_rand_ssid_pre[1];

    mset_DesiredSSID((WORD8 *)p2p_ssid);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : gen_random_pass_phrase                                   */
/*                                                                           */
/*  Description   : This function generates 8 byte random Pass Phrase        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_p2p_rand_ssid_pre                                      */
/*                                                                           */
/*  Processing    : This function generates 8 byte random Pass Phrase        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void gen_random_pass_phrase(void)
{
    UWORD8 i                                = 0;
    UWORD8 rnd_num                          = 0;
    UWORD8 pass_phrase[PSK_PASS_PHRASE_LEN] = {0,};

    for(i = 0; i < PSK_PASS_PHRASE_LEN; i++)
    {
		rnd_num = get_random_byte() >> 5;

		pass_phrase[i] = '0' + rnd_num;
	}

    mset_RSNAConfigPSKPassPhrase(pass_phrase, PSK_PASS_PHRASE_LEN);
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_match_dev_entry                                      */
/*                                                                           */
/*  Description   : This function searches the given dev id in the existing  */
/*                  list of device descriptors                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the device address                         */
/*                                                                           */
/*  Globals       : g_p2p_dev_dscr_set                                       */
/*                  g_dev_dscr_set_index                                     */
/*                                                                           */
/*  Processing    : The function matches the given device ID to the device ID*/
/*                  of the existing g_p2p_dev_dscr_set and return BTRUE if   */
/*                  a match is found                                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if match found                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
BOOL_T p2p_match_dev_entry(UWORD8 *dev_addr)
{

    UWORD8 count = 0;

    for(count = 0; count < g_dev_dscr_set_index; count++)
    {
        if(BTRUE == mac_addr_cmp(g_p2p_dev_dscr_set[count].dev_addr, dev_addr))
            return BTRUE;
    }

    return BFALSE;
}


UWORD8* p2p_get_go_dev_address(UWORD8 *grp_bssid)
{

    UWORD8 count   = 0;
    UWORD8 grp_cap = 0;

    for(count = 0; count < g_dev_dscr_set_index; count++)
    {
        grp_cap = g_p2p_dev_dscr_set[count].grp_capability;

        if((BTRUE == mac_addr_cmp(g_p2p_dev_dscr_set[count].grp_bssid,
                                 grp_bssid)) &&
            (CHECK_BIT(grp_cap, P2PCAPBIT_GO)))
            return g_p2p_dev_dscr_set[count].dev_addr;
    }

    return NULL;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : parse_client_info_dscr                                   */
/*                                                                           */
/*  Description   : This function parses the P2P Client Info Descriptors and */
/*                  adds them to the list of device descriptors              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the attribute start                        */
/*                  2) group capability                                      */
/*                  3) Pointer to the bssid                                  */
/*                  4) length of the attribute                               */
/*                                                                           */
/*  Globals       : g_p2p_dev_dscr_set                                       */
/*                  g_dev_dscr_set_index                                     */
/*                                                                           */
/*  Processing    : parse through all the client descriptos                  */
/*                  For each unique P2P device address found update the      */
/*                  device descriptor list. Use the group capability of the  */
/*                  GO for the group capability. Clear the P2P Group Owner   */
/*                  bit to indicate that it is a client                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void parse_client_info_dscr(UWORD8 *attr_start_ptr, UWORD16 ie_len,
                            UWORD8 grp_cap, UWORD8 *bssid, UWORD8 *ssid)
{
    UWORD16 index                 = 0;
    UWORD16 attr_len              = 0;
    UWORD8 *attr_ptr              = NULL;
    p2p_dev_dscr_t *curr_dev_dscr = NULL;

    attr_ptr = p2p_get_attr(P2P_GROUP_INFO, attr_start_ptr, ie_len, &attr_len);

    if(NULL == attr_ptr)
        return;

    /* The group capability of the group remains the same for all    */
    /* devices in a group. The GO BIT (BIT0) is reset                */
    grp_cap &= (~P2PCAPBIT_GO);

    while((index < attr_len) && (g_dev_dscr_set_index < MAX_DEV_FOR_SCAN))
    {
        /****** P2P Client Info descriptor *******/
        /*+------------------------------------+ */
        /*|Field Name                   | Size | */
        /*|-----------------------------|------| */
        /*|Length                       | 1    | */
        /*|P2P Device address           | 6    | */
        /*|P2P Interface address        | 6    | */
        /*|Device Capability Bitmap     | 1    | */
        /*|Config Methods               | 2    | */
        /*|Primary Device Type          | 8    | */
        /*+------------------------------------+ */
        if(BFALSE == p2p_match_dev_entry(attr_ptr+index+1))
        {
            curr_dev_dscr = &g_p2p_dev_dscr_set[g_dev_dscr_set_index];

            mac_addr_cpy(curr_dev_dscr->dev_addr, attr_ptr+index+1);
            mac_addr_cpy(curr_dev_dscr->grp_bssid, bssid);
            strcpy(curr_dev_dscr->grp_ssid, ssid);
            curr_dev_dscr->dev_capability = attr_ptr[index + 13];
            curr_dev_dscr->grp_capability = grp_cap;

            g_dev_dscr_set_index++;
        }

        index += attr_ptr[index] + 1;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_get_dev                                              */
/*                                                                           */
/*  Description   : This function updates the device descriptors with info on*/
/*                  new devices found from management frames                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the mac start address                      */
/*                  2) start index for parsing IE's                          */
/*                  3) Rx length                                             */
/*                                                                           */
/*  Globals       : g_p2p_dev_dscr_set                                       */
/*                  g_dev_dscr_set_index                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       :      BTRUE  if  a) In case of a probe resp from non GO   */
/*                                     P2P device to indicate no further     */
/*                                     processing is required                */
/*                                  b) In case of a probe req to which we    */
/*                                     need to respond with Probe resp       */
/*                                                                           */
/*                       BFALSE, otherwise                                   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
BOOL_T p2p_get_dev(UWORD8 *msa, UWORD16 index, UWORD16 rx_len)
{
    BOOL_T status           = BFALSE;
    TYPESUBTYPE_T frm_type  = get_sub_type(msa);
    UWORD8 dev_name_index   = 0;
    UWORD8 dev_name_len     = 0;
    UWORD8 num_sec_dev      = 0;
    UWORD8 sav_dscr_index   = g_dev_dscr_set_index;
    UWORD16 pref_confg_meth = 0;
    UWORD16 ie_len          = 0;    /* combined Length of all attributes     */
    UWORD16 attr_len        = 0;    /* scratch variable for attribute length */
    UWORD8 *attr_start_ptr  = NULL; /* Start of combined attributes          */
    UWORD8 *attr_ptr        = NULL; /* scratch variable for the current
                                      attribute                              */
    p2p_dev_dscr_t *curr_dev_dscr = NULL; /* pointer to the current dev dscr
                                           struct                            */
	TROUT_FUNC_ENTER;
	
    g_p2p_manageability = 0;
    g_p2p_match_idx     = -1;


    /* Get the pointer to the combined P2P attributes */
    attr_start_ptr = get_p2p_attributes(msa, index, rx_len, &ie_len);

    if(NULL == attr_start_ptr)
    {
		TROUT_FUNC_EXIT;
        return BFALSE;
	}
    /* Get the p2p manageability attribute required by other modules */
    attr_ptr = p2p_get_attr(P2P_MANAGEABILITY, attr_start_ptr, ie_len,
                            &attr_len);
    if(NULL != attr_ptr)
    {
        g_p2p_manageability = *attr_ptr;
    }

    /* Check if we have space for any more device descriptor */
    if(g_dev_dscr_set_index >= MAX_DEV_FOR_SCAN)
    {
 		/* Free the local memory buffer allocated for P2P attributes */
 		mem_free(g_local_mem_handle, attr_start_ptr);
		TROUT_FUNC_EXIT;
        return BFALSE;
	}

    curr_dev_dscr  = &g_p2p_dev_dscr_set[g_dev_dscr_set_index];

    /* Clear the descriptor. It may have some invalid values due to */
    /* incomplete processing in the last frame                      */
    mem_set(curr_dev_dscr, 0, sizeof(p2p_dev_dscr_t));

    /* P2P_CAPABILITY attribute should be present for all frames. Update the */
    /* same and return if it is not present                                  */
    attr_ptr = p2p_get_attr(P2P_CAPABILITY, attr_start_ptr, ie_len,
                                &attr_len);
    if(NULL == attr_ptr)
    {
		/* Free the local memory buffer allocated for P2P attributes */
		mem_free(g_local_mem_handle, attr_start_ptr);
		TROUT_FUNC_EXIT;
        /* INCOMPLETE Information !!!!!!*/
        return BFALSE;
    }
    else
    {
        curr_dev_dscr->dev_capability = attr_ptr[0];
        curr_dev_dscr->grp_capability = attr_ptr[1];
    }

    switch(frm_type)
    {
    case BEACON:
    {
        /*****************************************************************/
        /* In case of a beacon P2P_DEVICE_ID should be present as a      */
        /* unique device ID                                              */
        /*****************************************************************/
        attr_ptr = p2p_get_attr(P2P_DEVICE_ID, attr_start_ptr, ie_len,
                                &attr_len);
        if((NULL  == attr_ptr) || (BTRUE == p2p_match_dev_entry(attr_ptr)))
        {
            break;
        }

        mac_addr_cpy(curr_dev_dscr->dev_addr,attr_ptr);
        get_BSSID(msa, curr_dev_dscr->grp_bssid);
        get_ssid(msa, curr_dev_dscr->grp_ssid);

        g_dev_dscr_set_index++;
    }
    break;
    case ACTION:
    {
        /* Check if the frame is GO negotiation request */
        if(BFALSE == is_p2p_pub_action_frame(msa + MAC_HDR_LEN) ||
           P2P_GO_NEG_REQ != msa [MAC_HDR_LEN + P2P_PUB_ACT_OUI_SUBTYPE_OFF])
        {
            break;
        }

        /* If the frame is a GO Neg request then processing is same as */
        /* PROBE_RSP. Fall through the case and not break              */
    }
    case PROBE_RSP:
    {
        /*****************************************************************/
        /* In case of a probe resp/GO Neg reg P2P_DEVICE_INFO should be  */
        /* present as an unique device ID                                */
        /*****************************************************************/
        attr_ptr = p2p_get_attr(P2P_DEVICE_INFO, attr_start_ptr, ie_len,
                                &attr_len);
        /* Search for an existing entry                                  */
        /* NOTE: The first 6 bytes is the P2P Device address, even though*/
        /* the attribute is longer                                       */
        if((NULL == attr_ptr) || (BTRUE == p2p_match_dev_entry(attr_ptr)))
        {
            break;
        }

        mac_addr_cpy(curr_dev_dscr->dev_addr, attr_ptr);

        /* Get the number of secondary device. This field is present */
        /* at an offset of 16 bytes in the P2P device info attribute */
        /* Ref: Table 29-Device Info attribute format                */
        num_sec_dev  = *(attr_ptr + 16);

        /* Get the pointer to the start of the device name attribute */
        dev_name_index = (17 + num_sec_dev * 8);

        /* The device name is in TLV format (Ref: Table 2, WiFi Simple */
        /* configuration) the attribute length is at offset 2 BYTES and*/
        /* the attribute is at offset of 4 BYTES from the start of the */
        /* attribute                                                   */
        dev_name_len = attr_ptr[dev_name_index + 2];

        /* Copy the device name to the device descriptor */
        memcpy(curr_dev_dscr->dev_name, (attr_ptr + dev_name_index + 4),
                                                 dev_name_len);
        curr_dev_dscr->dev_name[dev_name_len] = '\0';

        g_dev_dscr_set_index++;

        /* If the group capability indicates that the device is a GO */
        /* then further processing is required                       */
        if(PROBE_RSP == frm_type)
        {
            if(CHECK_BIT(curr_dev_dscr->grp_capability, P2PCAPBIT_GO))
            {
                get_BSSID(msa, curr_dev_dscr->grp_bssid);
                get_ssid(msa, curr_dev_dscr->grp_ssid);
                parse_client_info_dscr(attr_start_ptr, ie_len,
                                       curr_dev_dscr->grp_capability,
                                       curr_dev_dscr->grp_bssid,
                                       curr_dev_dscr->grp_ssid);
            }
            else
            {
                /* This probe response was sent by a non GO P2P device */
                status = BTRUE;
            }
        }

        /* Extract the preferred config method advertised by the device */
        p2p_process_prob_rsp_enr_mode(msa, rx_len, &pref_confg_meth);

        memcpy(&(curr_dev_dscr->config_method), &pref_confg_meth, 2);
    }
    break;
    case PROBE_REQ:
    {
        /*****************************************************************/
        /* In case of a probe req there are no unique device ID present. */
        /* Use the TA as the unique device ID                            */
        /*****************************************************************/

        UWORD8 dev_addr[MAC_ADDRESS_LEN];
        get_TA(msa, dev_addr);

        attr_ptr = p2p_get_attr(P2P_DEVICE_ID, attr_start_ptr, ie_len,
                                &attr_len);

        status = BTRUE;
        /* Check if Device ID attribute is present */
        if(attr_ptr != NULL)
        {
            /* If the Device ID attribute is present then match the device   */
            /* address field with the device address of the sta              */
            if (BFALSE == mac_addr_cmp(attr_ptr, mget_p2p_dev_addr()))
                status = BFALSE;
        }

        if(BFALSE == p2p_match_dev_entry(dev_addr))
        {
            mac_addr_cpy(curr_dev_dscr->dev_addr, dev_addr);
            g_dev_dscr_set_index++;
        }
    }
    break;
    default:
    {
        /* No other frame need to be processed in this state. */
    }
    break;
    }

    for(attr_len = sav_dscr_index; attr_len < g_dev_dscr_set_index; attr_len++)
    {
        if(BTRUE == mac_addr_cmp(g_p2p_dev_dscr_set[attr_len].dev_addr,
                                mget_p2p_trgt_dev_id())
#ifdef P2P_AUTO_KLUDGE
                            || (BTRUE ==  is_group(mget_p2p_trgt_dev_id()))
#endif /* P2P_AUTO_KLUDGE */
           )
        {
            g_p2p_match_idx = attr_len;
#ifdef P2P_AUTO_KLUDGE
            mset_p2p_trgt_dev_id(g_p2p_dev_dscr_set[attr_len].dev_addr);
#endif /* P2P_AUTO_KLUDGE */

#ifdef DEBUG_MODE
            print_dev_dscr(attr_len);
#endif /* DEBUG_MODE */
        }
    }

    /* Check if a local mem was used for copying the attributes */
    if(g_local_mem_handle == get_mem_pool_hdl(attr_start_ptr))
    {
        mem_free(g_local_mem_handle, attr_start_ptr);
    }

	/* Free the local memory buffer allocated for P2P attributes */
	mem_free(g_local_mem_handle, attr_start_ptr);

	TROUT_FUNC_EXIT;
    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_scan_itr_to_p2p                                   */
/*                                                                           */
/*  Description   : This function handles a dwell timeout in during scanning */
/*                                                                           */
/*  Inputs        : 1) Pointer to the event message                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Appropriate processing is done based on the P2P MISC     */
/*                  event type.                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
SWORD8 handle_scan_itr_to_p2p(mac_struct_t *mac)
{
    SWORD8 chan2scan = -1;

	TROUT_FUNC_ENTER;
    switch(get_p2p_scan_state())
    {
    case NORMAL_SCAN:
    {
        if (g_channel_index >= g_num_channels)
        {
            /* Regular 802.11 scan done, switch to FIND Phase */
            if((BTRUE == is_p2p_grp_form_in_prog()) ||
               (SEND_INVITE == g_p2p_invit))
            {
                set_mac_state(WAIT_SCAN);
                set_p2p_scan_state(LISTEN_STATE);
                chan2scan = mget_p2p_listen_chan();
                g_p2p_find_phase_itr_cnt = 0;
                TROUT_DBG4("Regular 802.11 scan done, switch to P2P LISTEN_STATE\n\r");
            }
            else
            {
                process_scan_itr_comp(mac);
            }
        }
        else
        {
            /* Scan other channels, channel index will be incremented */
            chan2scan = g_channel_list[g_channel_index];
            g_channel_index++;
        }

    }
        break;
    case LISTEN_STATE:
    {
        if(BFALSE == mget_p2p_listen_mode()) /* Normal Discovery state */
        {
	        /* Listen channel dwell done, switch to SEARCH_STATE */
	        g_channel_index = 0;
	        chan2scan = g_p2p_social_chan[0];
	        set_p2p_scan_state(SEARCH_STATE);
	        TROUT_DBG4("LISTEN_STATE dwell done, switch to SEARCH_STATE\n\r");
        }
        else /* Configured to Listen state only */
        {
            /* Remain in LISTEN_STATE till find timeout */
            if(g_p2p_find_phase_itr_cnt > get_p2p_find_to())
            {
                process_scan_itr_comp(mac);
            }
            else
            {
                chan2scan = mget_p2p_listen_chan();
                g_p2p_find_phase_itr_cnt++;
                TROUT_DBG4("Continuing in LISTEN_STATE\n\r");
            }
        }
    }
    break;
    case SEARCH_STATE:
    {
        g_channel_index++;

        if(g_channel_index >= P2P_NUM_SOCIAL_CHAN)
        {
            if((g_p2p_find_phase_itr_cnt > get_p2p_find_to()) ||
               ((SEND_INVITE == g_p2p_invit) &&
                (g_p2p_find_phase_itr_cnt > P2P_OBSS_FIND_TO)))
            {
                process_scan_itr_comp(mac);
            }
            else
            {
                set_p2p_scan_state(LISTEN_STATE);
                chan2scan = mget_p2p_listen_chan();
                g_p2p_find_phase_itr_cnt++;
                TROUT_DBG4("SEARCH_STATE done, switch to LISTEN_STATE\n\r");
            }
        }
        else
        {
            chan2scan = g_p2p_social_chan[g_channel_index];
        }

	}
    break;
	}

	TROUT_FUNC_EXIT;
    return chan2scan;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : misc_p2p_event_sta                                       */
/*                                                                           */
/*  Description   : This function handles the MISC event for P2P protocol in */
/*                  STA in the ENABLED state.                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the event message                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Appropriate processing is done based on the P2P MISC     */
/*                  event type.                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void misc_p2p_event_sta(UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

	TROUT_FUNC_ENTER;
    switch(misc_event_msg->name)
    {
    case MISC_ABSENT_PERIOD_START:
    {
        handle_p2p_go_absence_period_start();
    }
    break;

    case MISC_ABSENT_PERIOD_END:
    {
        handle_p2p_go_absence_period_end();
    }
    break;

    default:
    {
        /* Do nothing */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_join_complete                                        */
/*                                                                           */
/*  Description   : This function handles completion of P2P Join (or Group   */
/*                  Formation) stage                                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Result of the joining process                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates the join response result and       */
/*                  cancels the management timer. It then changes the fsm    */
/*                  state to JOIN_COMP and sends Join response to MLME       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_join_complete(mac_struct_t *mac, UWORD8 result)
{
    join_rsp_t join_rsp = {0};

	TROUT_FUNC_ENTER;
    /* Prepare a Join response */
    join_rsp.result_code = result;

    /* Cancel the timeout timer */
    cancel_mgmt_timeout_timer();

    /* Set the MAC state to JOIN_COMP */
    set_mac_state(JOIN_COMP);

    /* Send the Join response to MLME */
    send_mlme_rsp_to_host(mac, MLME_JOIN_RSP, (UWORD8 *)(&join_rsp));
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_end_grp_form                                         */
/*                                                                           */
/*  Description   : This function handles completion of P2P Group Formation  */
/*                  The GO/Client roles has been decided                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                                                                           */
/*  Globals       : g_p2p_GO_role                                            */
/*                                                                           */
/*  Processing    : This function updates the WSC password ID as required.   */
/*                  In case we are becoming a client it starts WPS. If we are*/
/*                  becoming a GO it switches the modules also               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_end_grp_form(mac_struct_t *mac)
{
    UWORD16 psswd_id = get_wps_pass_id_enr();
    /* If P2P Group Formation is in progress, this indicates that */
    /* P2P Group formation has completed successfully.            */
    TROUT_FUNC_ENTER;
    set_p2p_grp_form_in_prog(BFALSE);
    store_user_settings_p2p();
    mset_p2p_dev_lmt(BTRUE);

    /* Free the WPS memory handle for a clean start */
    free_wps_handle_mem();

    /* changed as per Table 1 of P2P tech spec v1.1                    */
    /* PASSWD ID for Neg   |PASSWD ID for joining|PASSWD ID for joining*/
    /*                     |as client            |as GO                */
    /* --------------------|---------------------|---------------------*/
    /* DEV_PASS_ID_DEFAULT | Not valid           |Not valid            */
    /* DEV_PASS_ID_USER_SPE| DEV_PASS_ID_REG_SPE |DEV_PASS_ID_DEFAULT  */
    /* DEV_PASS_ID_MACH_SPE| Not valid           |Not valid            */
    /* DEV_PASS_ID_REKEY   | Not valid           |Not valid            */
    /* DEV_PASS_ID_PUSHBUTT| DEV_PASS_ID_PUSHBUTT|DEV_PASS_ID_PUSHBUTT */
    /* DEV_PASS_ID_REG_SPE | DEV_PASS_ID_DEFAULT |DEV_PASS_ID_REG_SPE  */

    if(BTRUE == g_p2p_GO_role)
    {   /* P2P-WPS :                                                    */
        switch(psswd_id)
        {
        case DEV_PASS_ID_USER_SPE:
        {
            set_wps_pass_id_enr(DEV_PASS_ID_DEFAULT);
        }
        break;
        case DEV_PASS_ID_REG_SPE:
        {
           /* Do Nothing */
        }
        break;
        case DEV_PASS_ID_PUSHBUTT:
        {
            /* Do nothing */
        }
        break;
        default:
        {
            TROUT_DBG3("Invalid password ID for joining\n");
        }
        }

        TROUT_DBG4("Becoming a GO\n\r");
        mset_CurrentChannel(mget_p2p_oper_chan());
        if(BTRUE == mget_p2p_persist_grp())
        {
            p2p_update_cred_list();
        }
        initiate_mod_switch(mac);

    }
    else
    {
        switch(psswd_id)
        {
        case DEV_PASS_ID_USER_SPE:
        {
            set_wps_pass_id_enr(DEV_PASS_ID_REG_SPE);
            psswd_id = DEV_PASS_ID_REG_SPE;
        }
        break;
        case DEV_PASS_ID_REG_SPE:
        {
            set_wps_pass_id_enr(DEV_PASS_ID_DEFAULT);
            psswd_id = DEV_PASS_ID_DEFAULT;
        }
        break;
        case DEV_PASS_ID_PUSHBUTT:
        {
            /* Do nothing */
        }
        break;
        default:
        {
            TROUT_DBG3("Invalid password ID for joining\n");
        }
        }

        if(DEV_PASS_ID_PUSHBUTT == psswd_id)
        {
            sys_start_wps(PBC);
        }
        else if((DEV_PASS_ID_DEFAULT == psswd_id) ||
		        (DEV_PASS_ID_REG_SPE == psswd_id))
        {
            sys_start_wps(PIN);
        }

        TROUT_DBG4("caisf set_device_mode");
		/* Change the device mode to P2P Client */
        set_device_mode(P2P_CLIENT);

        /* Update the PS related MAC HW registers */
		set_machw_p2p_enable();
		set_machw_p2p_mode_client();
        set_machw_p2p_rx_beacon_to(MACHW_P2P_RX_BCN_TO);

        TROUT_DBG4("Becoming a client\n\r");
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_join_req_params_p2p                               */
/*                                                                           */
/*  Description   : This function updates the P2P manageability element of   */
/*                  the required bss descriptor                              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the join request                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates the join request parameters for the*/
/*                  P2P protocol. It checks the P2P manageability of the AP  */
/*                  that the STA is trying to join. If the AP supports P2P   */
/*                  P2P manageability then the disables the P2P feature in   */
/*                  the STA                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void update_join_req_params_p2p(join_req_t *join_req)
{
    if((mget_DesiredBSSType() == INFRASTRUCTURE) &&
       (mget_p2p_enable() == BTRUE))
    {
        if((join_req->bss_dscr.p2p_manag & BIT0) != 0)
        {
            mset_p2p_enable(BFALSE);
        }
    }
    return;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_p2p_sta_globals                                     */
/*                                                                           */
/*  Description   : This funtion initializes p2p globals used in STA mode    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This funtion initializes p2p globals used in STA mode    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void init_p2p_sta_globals(void)
{
    g_noa_index              = -1;
    g_p2p_scan_state         = 0;
    g_p2p_find_phase_itr_cnt = 0;
    g_p2p_join_state         = 0;
    g_p2p_dialog_token       = 0;
    g_p2p_go_disc_req_rx     = 0;
    g_p2p_GO_role            = BFALSE;
    mem_set(g_p2p_go_dev_id, 0, MAC_ADDRESS_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : store_user_settings_p2p                                  */
/*                                                                           */
/*  Description   : This funtion stores the user settings which may be       */
/*                  changed by WPS                                           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_p2p_user_settings                                      */
/*                                                                           */
/*  Processing    : None                                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void store_user_settings_p2p(void)
{
    g_p2p_user_settings.auth_type = get_auth_type();
    g_p2p_user_settings.sec_mode = get_802_11I_mode();
    memcpy(g_p2p_user_settings.psk, get_RSNAConfigPSKPassPhrase(),
           MAX_PSK_PASS_PHRASE_LEN);
    memcpy(g_p2p_user_settings.ssid, get_DesiredSSID(), MAX_SSID_LEN);
    memcpy(g_p2p_user_settings.bssid, get_bssid(), MAC_ADDRESS_LEN);
    g_p2p_user_settings.p2p_op_chan = mget_p2p_oper_chan();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : restore_user_settings_p2p                                */
/*                                                                           */
/*  Description   : This funtion restores some user setting after a group    */
/*                  has ended                                                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_p2p_user_settings                                      */
/*                                                                           */
/*  Processing    : None                                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void restore_user_settings_p2p(void)
{
    set_auth_type(g_p2p_user_settings.auth_type);
    set_802_11I_mode(g_p2p_user_settings.sec_mode);
    set_RSNAConfigPSKPassPhrase(g_p2p_user_settings.psk);
    set_DesiredSSID((WORD8 *)(g_p2p_user_settings.ssid));
    set_bssid(g_p2p_user_settings.bssid);
    mset_p2p_oper_chan(g_p2p_user_settings.p2p_op_chan);

    /* Disable the persistent and intra-bss group capability */
	mset_p2p_persist_grp(BFALSE);
    mset_p2p_intra_bss(BFALSE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_sta_wait_scan_handle_prb_req                         */
/*                                                                           */
/*  Description   : This function does the P2P protocol related processing   */
/*                  of the Probe Request frame received in the wait scan     */
/*                  state                                                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the mac start address                      */
/*                  2) Length of the received frame                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : If the device is in listen state and the group formation */
/*                  is in progress then the function to add the device to the*/
/*                  list is called. If the probe request is a broadcast frame*/
/*                  or a directed frame or has a wildcard ssid then a probe  */
/*                  response is sent after doing the WPS related processing  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_sta_wait_scan_handle_prb_req(UWORD8 *msa, UWORD16 rx_len)
{

    UWORD8 *addr3  = 0;

	TROUT_FUNC_ENTER;
    /* Add the device to list and check the returned status */
    if(BTRUE == (p2p_get_dev(msa, MAC_HDR_LEN, rx_len)))
    {

        addr3 = get_address3_ptr(msa);

        if(((BTRUE == mac_addr_cmp(addr3, mget_bcst_addr())) ||
           (BTRUE == mac_addr_cmp(addr3, mget_StationID()))) &&
           (BTRUE == probe_req_ssid_cmp(msa, P2P_WILDCARD_SSID)))
        {
#ifdef INT_WPS_ENR_SUPP
            /* Check if requested device type attribute is present in the WSC*/
            /* IE. If it is present then check if it matches with either the */
            /* primary device type or any of the secondary device type       */
            /* matches with the requested device type                        */
            if(BTRUE == wps_process_p2p_prob_req_enr(g_wps_config_ptr,
                                                     g_wps_enrollee_ptr, msa,
                                                     rx_len))
#endif /* INT_WPS_ENR_SUPP */
            {
                /* Call the function to send probe response */
                send_probe_rsp(msa, BTRUE);
            }
        }
    }
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_config_persist_cred                                  */
/*                                                                           */
/*  Description   : This function configures the device with the credentials */
/*                  stored to reinvoke a persistent group                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_persist_list                                           */
/*                                                                           */
/*  Processing    : This function configures the device with the credentials */
/*                  stored to reinvoke a persistent group                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
STATUS_T p2p_config_persist_cred(void)
{
    UWORD8 ssid_len = 0;

	TROUT_FUNC_ENTER;
    ssid_len = strlen((WORD8 *)g_persist_list.ssid);

    /* Check if credential is valid, if not then return. SSID Length,        */
    /* Key length, Security mode and  Number of keys are checked             */
    if(((0 == ssid_len) || (MAX_SSID_LEN <= ssid_len)) ||
       (SEC_MODE_MASK < g_persist_list.sec_mode))
    {
		TROUT_FUNC_EXIT;
        return FAILURE;
    }

    if((MAX_PSK_PASS_PHRASE_LEN < g_persist_list.key_len) ||
       (0 == g_persist_list.key_len))
    {
		TROUT_FUNC_EXIT;
    	return FAILURE;
    }

    /* Set the PSK */
    mset_RSNAConfigPSKPassPhrase(g_persist_list.psk,
                                  g_persist_list.key_len);


    /* Set SSID */
    mset_DesiredSSID((WORD8 *)g_persist_list.ssid);

    /* Set Authentication and Encryption Type */
    set_802_11I_mode(g_persist_list.sec_mode);
    set_auth_type(g_persist_list.auth_type);

    /* Set P2P_INTRA_BSS  */
    if(CHECK_BIT(g_persist_list.grp_cap, P2PCAPBIT_INTRA_BSS_DIST))
    {
        mset_p2p_intra_bss(BTRUE);
    }
    else
    {
        mset_p2p_intra_bss(BFALSE);
    }

    /* Set P2P_PERSIST_GRP */
    mset_p2p_persist_grp(BTRUE);

    /* Set the current channel to the P2P operating channel */
    mset_CurrentChannel(mget_p2p_oper_chan());

	TROUT_FUNC_EXIT;
    return SUCCESS;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_handle_invit_rsp_sta                                 */
/*                                                                           */
/*  Description   : This function handles the invitation response frame      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the mac start address                      */
/*                  2) Length of the incoming message                        */
/*                  3) Pointer to the MAC structure                          */
/*                                                                           */
/*  Globals       : g_p2p_invit                                              */
/*                                                                           */
/*  Processing    : This function checks if the invitation repsonse is sent  */
/*                  by the device to which it had sent the invitation request*/
/*                  It then cancels the timer and sets the target device id  */
/*                  to NULL and sends the reponse fram to the host           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void p2p_handle_invit_rsp_sta(UWORD8 *msa, UWORD16 rx_len, mac_struct_t *mac)
{
	TROUT_FUNC_ENTER;
    if(WAIT_INVITE_RSP == g_p2p_invit)
    {
        /* Check if the source address of the invitation request frame       */
        /* matches with the target device address                            */
        if(BTRUE == mac_addr_cmp(get_TA_ptr(msa), mget_p2p_trgt_dev_id()))
        {
            cancel_mgmt_timeout_timer();
            handle_inv_req_to(mac);
            send_host_p2p_req(msa, rx_len);
        }
    }
    TROUT_FUNC_EXIT;
}

#ifdef DEBUG_MODE
/*****************************************************************************/
/*                                                                           */
/*  Function Name : print_dev_dscr                                           */
/*                                                                           */
/*  Description   : This function prints the P2P device descriptor           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_p2p_dev_dscr_set                                       */
/*                                                                           */
/*  Processing    : None                                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*****************************************************************************/
void print_dev_dscr(UWORD8 index)
{
    UWORD8 i = 0;
    p2p_dev_dscr_t *curr_dev = &g_p2p_dev_dscr_set[index];

    PRINTD("dev_capability = %0x ", curr_dev->dev_capability);
    PRINTD("grp_capability = %0x ", curr_dev->grp_capability);

    PRINTD("\ntrg_addr=");
    for(i = 0; i < MAC_ADDRESS_LEN; i++)
        PRINTD("%02x ", (mget_p2p_trgt_dev_id())[i]);

        PRINTD("\ndev_addr=");
    for(i = 0; i < MAC_ADDRESS_LEN; i++)
        PRINTD("%02x ", curr_dev->dev_addr[i]);

    PRINTD("\ngrp_bssid=");
    for(i = 0; i < MAC_ADDRESS_LEN; i++)
        PRINTD("%02x ", curr_dev->grp_bssid[i]);

    PRINTD("\ngrp_ssid = %s\n", curr_dev->grp_ssid);
    PRINTD("dev_name = %s\n", curr_dev->dev_name);
}
#endif /* DEBUG_MODE */

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */

