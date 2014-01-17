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
/*  File Name         : sta_frame_p2p.c                                      */
/*                                                                           */
/*  Description       : This file conatins the functions to create P2P frames*/
/*                      in sta mode                                          */
/*                                                                           */
/*  List of Functions : p2p_send_go_neg_req                                  */
/*                      p2p_send_dev_disc_req                                */
/*                      p2p_send_go_neg_cnf                                  */
/*                      p2p_handle_noa                                       */
/*                      add_p2p_ie_probe_req                                 */
/*                      p2p_send_prov_disc_req                               */
/*                      add_asoc_req_p2p_ie                                  */
/*                      p2p_send_go_neg_rsp                                  */
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
#include "imem_if.h"
#include "management.h"
#include "frame_p2p.h"
#include "mh.h"
#include "transmit.h"
#include "p2p_wlan_rx_mgmt_sta.h"
#include "p2p_ps_sta.h"
#include "prot_if.h"
#include "core_mode_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_go_neg_req                                      */
/*                                                                           */
/*  Description   : This function sends a P2P GO negotiation request         */
/*                                                                           */
/*  Inputs        : 1) None                                                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                  g_p2p_dialog_token                                       */
/*                  g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function prepares the P2P GO negotiation            */
/*                  confirmation frame. It adds the MAC header, the action   */
/*                  frame header and the appropriate P2P attributes.         */
/*                  It then calls the function to transmit the frame         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void p2p_send_go_neg_req(void)
{
    UWORD8  *frm_ptr      = 0;
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;

    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
        return;
    }

    TROUT_DBG4("P2P: send a P2P frame...");

    add_p2p_mgmt_frame_hdr(frm_ptr, g_p2p_join_req.dev_dscr.dev_addr);
    g_p2p_dialog_token = get_random_byte();   /* rand dialog token */
    index = add_p2p_pub_act_hdr(frm_ptr, g_p2p_dialog_token, P2P_GO_NEG_REQ);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /*************************************************************************/
    /* The following P2P attributes are added as per the P2P v1.1 spec       */
    /* Table 49—P2P attributes in the GO Negotiation Request frame           */
    /* - P2P Capability                 shall be present                     */
    /* - Group Owner Intent             shall be present                     */
    /* - Configuration Timeout          shall be present                     */
    /* - Listen Channel                 shall be present                     */
    /* - Extended Listen Timing         may   be present                     */
    /* - Intended P2P Interface Address shall be present                     */
    /* - Channel List                   shall be present                     */
    /* - P2P Device Info                shall be present                     */
    /* - Operating Channel              shall be present                     */
    /*************************************************************************/
    index += add_p2p_capability_attr(frm_ptr, index, get_p2p_dev_cap(),
                                     get_p2p_grp_cap());
    index += add_GO_intent_attr(frm_ptr, index);
    index += add_config_timout_attr(frm_ptr, index, get_config_timeout(),
                                    get_config_timeout());
    /* Add channel attribute */
    index += add_listen_chan_attr(frm_ptr, index);
    index += add_int_p2p_if_addr_attr(frm_ptr, index);
    index += add_p2p_chan_list_attr(frm_ptr, index);
    index += add_p2p_device_info_attr(frm_ptr, index);
    index += add_p2p_oper_chan_attr(frm_ptr, index, mget_p2p_oper_chan());

    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Add WSC IE. The following Attributes are required as per P2P V1.1 spec*/
    /* - Version                                                             */
    /* - Device Password ID                                                  */
    index += wps_add_go_neg_frm_wsc_ie(frm_ptr, index,
                                       (WPS_PROT_TYPE_T)(get_wps_prot()),
                                         get_wps_pass_id());

    /* Transmit the management frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_dev_disc_req                                    */
/*                                                                           */
/*  Description   : This function sends a P2P GO device discovery request    */
/*                                                                           */
/*  Inputs        : 1) None                                                  */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                  g_shared_pkt_mem_handle                                  */
/*                  g_p2p_dialog_token                                       */
/*                                                                           */
/*  Processing    : This function prepares the P2P GO device discovery       */
/*                  request  frame. It adds the MAC header, the action frame */
/*                  header and the appropriate P2P attributes.               */
/*                  It then calls the function to transmit the frame         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_send_dev_disc_req(void)
{
    UWORD8  *frm_ptr      = 0;
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;

    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
        return;
    }

    TROUT_DBG4("P2P: send a P2P frame...");
    
    add_p2p_mgmt_frame_hdr(frm_ptr, g_p2p_join_req.dev_dscr.grp_bssid);
    g_p2p_dialog_token = get_random_byte();   /* rand dialog token */
    index = add_p2p_pub_act_hdr(frm_ptr, g_p2p_dialog_token, P2P_DEV_DISC_REQ);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /*************************************************************************/
    /* The following P2P attributes are added as per the P2P v1.1 spec       */
    /* Table 56—P2P attributes in the Device Discoverability Request frame   */
    /* - P2P Device ID  shall be present                                     */
    /* - P2P Group ID   shall be present                                     */
    /*************************************************************************/
    index += add_p2p_device_id_attr(frm_ptr, index,
                                    g_p2p_join_req.dev_dscr.dev_addr);

    index += add_p2p_grp_id_attr(frm_ptr, index,
                                 g_p2p_join_req.dev_dscr.grp_bssid,
                                 g_p2p_join_req.dev_dscr.grp_ssid);

    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Transmit the management frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_go_neg_cnf                                      */
/*                                                                           */
/*  Description   : This function sends a P2P GO negotiation confirmation    */
/*                                                                           */
/*  Inputs        : 1) UWORD8 status to be sent in the STATUS attribute      */
/*                                                                           */
/*  Globals       : g_p2p_join_req                                           */
/*                  g_p2p_dialog_token                                       */
/*                  g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function prepares the P2P GO negotiation            */
/*                  confirmation frame. It adds the MAC header and the       */
/*                  appropriate P2P attributes along with the status code.   */
/*                  It then calls the function to transmit the frame         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_send_go_neg_cnf(UWORD8 status)
{
    UWORD8  grp_cap       = 0;
    UWORD8  *frm_ptr      = 0;
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;

	TROUT_FUNC_ENTER;
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    TROUT_DBG4("P2P: send a P2P frame...");
    
    add_p2p_mgmt_frame_hdr(frm_ptr, g_p2p_join_req.dev_dscr.dev_addr);
    index = add_p2p_pub_act_hdr(frm_ptr, g_p2p_dialog_token, P2P_GO_NEG_CNF);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /*************************************************************************/
    /* The following P2P attributes are added as per the P2P v1.1 spec       */
    /* Table 53—P2P attributes in the GO Negotiation Confirmation frame      */
    /* - Status            shall be present                                  */
    /* - P2P Capability    shall be present                                  */
    /* - Operating Channel shall be present                                  */
    /* - Channel List      shall be present                                  */
    /* - P2P Group ID      shall be present if the P2P Device sending the GO */
    /*                     Negotiation Confirmation frame will become P2P    */
    /*                     Group Owner following Group Owner Negotiation.    */
    /*************************************************************************/
    index += add_status_attr(frm_ptr, index, (P2P_STATUS_CODE_T)status);

    if(BTRUE == g_p2p_GO_role)
    {
        /* If we will be a GO then the grp_cap is our capability else it is  */
        /* reserved                                                          */
        grp_cap = get_p2p_grp_cap();
    }
    index += add_p2p_capability_attr(frm_ptr, index, get_p2p_dev_cap(),
                                     grp_cap);

    index += add_p2p_oper_chan_attr(frm_ptr, index, mget_p2p_oper_chan());
    index += add_p2p_chan_list_attr(frm_ptr, index);

    if(BTRUE == g_p2p_GO_role)
    {
        /* If we will be a GO then the P2P Group ID has to be added  */
        index += add_p2p_grp_id_attr(frm_ptr, index, mget_p2p_dev_addr(),
                                     (UWORD8 *)(mget_DesiredSSID()));
    }


    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Transmit the management frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);

    /* Wait for the confirmation to get transmitted */
    while(BFALSE == is_machw_q_null(HIGH_PRI_Q))
    {
        add_delay(0xFFF);
    }

	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_handle_noa                                           */
/*                                                                           */
/*  Description   : This function handles the NOA frame                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the received NOA frame        */
/*                  2) Length of the recieved frame                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the NOA attribute from the        */
/*                  received frame and if it is present then calls the       */
/*                  function to process it and update the Hw registers       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_handle_noa(UWORD8 *msa, UWORD16 rx_len)
{
    if((BTRUE == is_sta_connected()) && (BTRUE == mget_p2p_enable()))
    {
	    UWORD8  *attr_start_ptr = 0;
	    UWORD16 ie_len          = 0;

        /* Note: a scratch memory need not be allocated because the */
        /* frame is expected to have only one P2P IE  */
	    attr_start_ptr = get_p2p_attributes(msa, MAC_HDR_LEN, rx_len,
	                                                &ie_len);

	    if(NULL != attr_start_ptr)
	    {
	        process_p2p_noa_attr(attr_start_ptr, ie_len);

			/* Free the local memory buffer allocated for P2P attributes */
			mem_free(g_local_mem_handle, attr_start_ptr);
		}
    }
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_prov_disc_req                                   */
/*                                                                           */
/*  Description   : This function prepares the provision discovery request   */
/*                  frame and calls the function to transmit it              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the provision discovery request   */
/*                  frame adds the various P2P IE. It also determines the    */
/*                  config method to be set depending upon the prefered      */
/*                  of the device. It calls the function to add the config   */
/*                  method as WSC IE. It then calls the function to transmit */
/*                  the frame                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       :                                                          */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_send_prov_disc_req(void)
{
    UWORD8  grp_cap       = 0;
    UWORD8  *frm_ptr      = 0;
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;
    UWORD16 config_method = 0;

    TROUT_DBG4("P2P: send a P2P frame...");
    
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
        return;
    }

    add_p2p_mgmt_frame_hdr(frm_ptr, g_p2p_join_req.dev_dscr.dev_addr);
    g_p2p_dialog_token = get_random_byte();   /* rand dialog token */
    index = add_p2p_pub_act_hdr(frm_ptr, g_p2p_dialog_token,
                                P2P_PROV_DISC_REQ);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /*************************************************************************/
    /* The following P2P attributes are added as per the P2P v1.1 spec       */
    /* Table 58—P2P attributes in the Provision Discovery Request frame      */
    /* - P2P Capability    shall be present                                  */
    /* - P2P device info   shall be present                                  */
    /* - P2P Group ID      shall be present                                  */
    /*************************************************************************/

    index += add_p2p_capability_attr(frm_ptr, index, get_p2p_dev_cap(),
                                     grp_cap);

    index += add_p2p_device_info_attr(frm_ptr, index);
    index += add_p2p_grp_id_attr(frm_ptr, index,
                                 g_p2p_join_req.dev_dscr.dev_addr,
                                 g_p2p_join_req.dev_dscr.grp_ssid);
    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Add WSC IE */
    switch(get_wps_pass_id_enr())
    {
    case DEV_PASS_ID_USER_SPE:
    {
        config_method = WPS_CONFIG_METH_DISPLAY;
    }
    break;
    case DEV_PASS_ID_REG_SPE:
    {
        config_method = WPS_CONFIG_METH_KEYPAD;
    }
    break;
    case DEV_PASS_ID_PUSHBUTT:
    {
        config_method = WPS_CONFIG_METH_PUSHBUTTON;
    }
    break;
    default:
    {
        PRINTD("Invalid password ID for joining\n");
    }
    }

    /* Add the WSC IE */
    index += wps_add_config_method_ie(frm_ptr, index, config_method);

    /* Transmit the management frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_asoc_req_p2p_ie                                      */
/*                                                                           */
/*  Description   : This function adds the P2P IE to the assoc request  frame*/
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the probe response frame      */
/*                  2) Index to the start of the P2P IE                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the P2P IE along with the various     */
/*                  attributes required for the assoc request frame and      */
/*                  returns the total length of the P2P IE added             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Total lenght of the P2P IE added                */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 add_asoc_req_p2p_ie(UWORD8 *data, UWORD16 index)
{
    UWORD16 total_len    = 0;
    UWORD16 ie_len_index = 0;

    ie_len_index = index + 1;

    /* Add P2P IE header */
    index += add_p2p_ie_hdr(&data[index]);

    /* Add P2P capability attribute */
    index += add_p2p_capability_attr(data, index, get_p2p_dev_cap(), 0);

    index += add_p2p_device_info_attr(data, index);


    total_len = index - ie_len_index - 1;

    /* Update the length of the IE  */
    data[ie_len_index] =  total_len;

    return (total_len + IE_HDR_LEN);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_go_neg_rsp                                      */
/*                                                                           */
/*  Description   : This function prepares the GO negotiation response frame */
/*                  and calls the function to transmit it                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the GO negotiation req frame  */
/*                  2) Status code to be added                               */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function prepares the GO negotiation response frame,*/
/*                  adds the header and the required P2P attributes and calls*/
/*                  the function to transmit the frame                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_send_go_neg_rsp(UWORD8 *msa, UWORD8 status)
{
    UWORD8  dia_token     = 0;
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;
    UWORD8  *frm_ptr      = 0;
    UWORD8  grp_cap       = 0;
    UWORD16 dev_pass_id   = 0;
    
    TROUT_DBG4("P2P: send a P2P frame...");
    
    /* Allocate memory to prepare the GO negotiation resp frame */
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
        return;
    }

    /* Get the source address of the GO negotiation request frame. This will */
    /* be the DA for the response frame. Call the function to add MAC Header.*/
    add_p2p_mgmt_frame_hdr(frm_ptr, get_SA_ptr(msa));

    /* Extract the dialog token from the GO negotiation request frame */
    dia_token = msa[MAC_HDR_LEN + P2P_PUB_ACT_DIALOG_TOKEN_OFF] ;

    index = add_p2p_pub_act_hdr(frm_ptr, dia_token, P2P_GO_NEG_RSP);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /*************************************************************************/
    /* The following attributes would be present as per v1.1 fo P2P Spec     */
    /* Table 51—P2P attributes in the GO Negotiation Response frame          */
    /* -Status                         shall be present                      */
    /* -P2P Capability                 shall be present                      */
    /* -Group Owner Intent             shall be present                      */
    /* -Configuration Timeout          shall be present                      */
    /* -Operating Channel              may   be present                      */
    /* -Intended P2P Interface Address shall be present                      */
    /* -Channel List                   shall be present                      */
    /* -P2P Device Info                shall be present                      */
    /* -P2P Group ID                   shall be present                      */
    /*                                 if the P2P Device sending the GO      */
    /*                                 Negotiation Response frame will become*/
    /*                                 P2P Group Owner following Group Owner */
    /*                                 Negotiation                           */
    /*************************************************************************/
    index += add_status_attr(frm_ptr, index, (P2P_STATUS_CODE_T)status);

    if(BTRUE == g_p2p_GO_role)
    {
        /* If we will be a GO then the grp_cap is our capability else it is  */
        /* reserved                                                          */
        grp_cap = get_p2p_grp_cap();
    }
    index += add_p2p_capability_attr(frm_ptr, index, get_p2p_dev_cap(),
                                     grp_cap);

    index += add_GO_intent_attr(frm_ptr, index);
    index += add_config_timout_attr(frm_ptr, index, get_config_timeout(),
                                    get_config_timeout());
    index += add_p2p_oper_chan_attr(frm_ptr, index, mget_p2p_oper_chan());
    index += add_int_p2p_if_addr_attr(frm_ptr, index);
    index += add_p2p_chan_list_attr(frm_ptr, index);
    index += add_p2p_device_info_attr(frm_ptr, index);

    if(BTRUE == g_p2p_GO_role)
        index += add_p2p_grp_id_attr(frm_ptr, index, mget_p2p_dev_addr(),
                                     (UWORD8 *)(mget_DesiredSSID()));

    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Change the device password id Table 1 of P2P tech spec v1.1   */
    dev_pass_id = get_wps_pass_id();

    if(dev_pass_id == DEV_PASS_ID_USER_SPE)
        dev_pass_id = DEV_PASS_ID_REG_SPE;
    else if(dev_pass_id == DEV_PASS_ID_REG_SPE)
        dev_pass_id = DEV_PASS_ID_USER_SPE;

    /* Add WSC IE */
    index += wps_add_go_neg_frm_wsc_ie(frm_ptr, index,
                                       (WPS_PROT_TYPE_T)(get_wps_prot()),
                                       dev_pass_id);
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
}

#endif /* MAC_P2P */
#endif /* IBSS_BSS_STATION_MODE */
