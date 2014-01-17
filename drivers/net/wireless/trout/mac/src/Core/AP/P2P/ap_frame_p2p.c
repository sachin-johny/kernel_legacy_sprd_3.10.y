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
/*  File Name         : ap_frame_p2p.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions to send P2P frame   */
/*                      GO mode                                              */
/*                                                                           */
/*  List of Functions : add_asoc_resp_p2p_ie                                 */
/*                      send_p2p_go_disc_req                                 */
/*                      send_p2p_dd_resp                                     */
/*                      send_p2p_presence_resp                               */
/*                      add_p2p_grp_info_attr                                */
/*                      add_p2p_beacon_ie                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_P2P

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "core_mode_if.h"
#include "iconfig.h"
#include "index_util.h"
#include "frame_p2p.h"
#include "wps_reg_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_asoc_resp_p2p_ie                                     */
/*                                                                           */
/*  Description   : This function adds the P2P IE for the association        */
/*                  response frame                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the response frame            */
/*                  2) Index to the start of P2P IE                          */
/*                  3) Status of the association process                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the P2P IE header and the attributes  */
/*                  required for the the association response frame          */
/*                                                                           */
/*  Outputs       : UWORD16, Total length of the P2P IE                      */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 add_asoc_resp_p2p_ie(UWORD8* data, UWORD16 index, UWORD16 status)
{
    UWORD16 total_len    = 0;
    UWORD16 ie_len_index = index + 1;

    P2P_STATUS_CODE_T status_code = P2P_STAT_INFO_UNAVAIL;

    /* Add P2P IE header */
    index += add_p2p_ie_hdr(&data[index]);

    /* Add status code attribute */
    switch(status)
    {
        case SUCCESSFUL_STATUSCODE:
        {
            status_code = P2P_STAT_SUCCESS;
        }
        break;

        case UNSUP_CAP:
        case UNSUPT_ALG:
        case UNSUP_RATE:
        case SHORT_PREAMBLE_UNSUP:
        case PBCC_UNSUP:
        case CHANNEL_AGIL_UNSUP:
        case SHORT_SLOT_UNSUP:
        case OFDM_DSSS_UNSUP:
        case UNSPEC_FAIL:
        {
            status_code = P2P_STAT_INCOMP_PARAM;
        }
        break;

        case AP_FULL:
        {
            status_code = P2P_STAT_LMT_REACHED;
        }
        break;

        case FAIL_OTHER:
        {
            status_code = P2P_STAT_INFO_UNAVAIL;
        }
        break;

        default:
        {
           /* Do nothing. */
        }

    }

    /* Add the status code attribute */
    index += add_status_attr(data, index, (P2P_STATUS_CODE_T)status_code);

    total_len = index - ie_len_index - 1;

    /* Update the length of the IE  */
    data[ie_len_index] =  total_len;

    /* Return the total length of the P2P IE */
    return (total_len + IE_HDR_LEN);

}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_p2p_go_disc_req                                     */
/*                                                                           */
/*  Description   : This function prepares the GO descoverability request    */
/*                  frame and calls the function to transmit it              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the device address of the requesting       */
/*                     device                                                */
/*                  2) Dialog token of the DD request frame                  */
/*                  3) Destination address of the GO discoverability request */
/*                     frame                                                 */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function stores the device address of the P2P device*/
/*                  that had sent the DD request and the dialog token of the */
/*                  DD request frame. It then prepares the GO discoverability*/
/*                  frame to be sent to the requested device and calls the   */
/*                  function to transmit it                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void send_p2p_go_disc_req(UWORD8 *req_dev_addr, UWORD8 dia_token, UWORD8 *da)
{
    UWORD8  *frm_ptr  = 0;
    UWORD16 index     = 0;

	TROUT_FUNC_ENTER;
    /* Allocate memory to prepare the GO Discoverability resquest frame */
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Call the function to add the management frame MAC header */
    add_p2p_mgmt_frame_hdr(frm_ptr, da);

    /* Add the  action frame header with dialog token as NULL  */
    index = add_p2p_gen_act_hdr(frm_ptr, 0, GO_DISC_REQ);

    /* Store the device address of the STA which had sent the device         */
    /* discoverability request frame and the dialog token of that frame,     */
    add_p2p_pending_dd_rsp_elem(dia_token, req_dev_addr, frm_ptr, 
                                index + FCS_LEN);  

    /* Transmit the GO Discoverability resquest frame */
    tx_uc_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_p2p_dd_resp                                         */
/*                                                                           */
/*  Description   : This function prepares the DD response frame and calls   */
/*                  the function to transmit it                              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the destination address                    */
/*                  2) Dialog token to be added                              */
/*                  3) Status of the device discovery process                */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function prepares the DD response frame. It adds the*/
/*                  appropriate headers and the attributes and calls the     */
/*                  function to transmit it                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void send_p2p_dd_resp(UWORD8 *da, UWORD8 dia_token, P2P_STATUS_CODE_T status)
{
    UWORD16 index         = 0;
    UWORD16 ie_len_offset = 0;
    UWORD8  *frm_ptr      = 0;

    /* Allocate memory to prepare the GO negotiation resp frame */
    frm_ptr = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                    MANAGEMENT_FRAME_LEN);

    if(frm_ptr == NULL)
    {
        return;
    }

    /* Call the function to add the management frame MAC header */
    add_p2p_mgmt_frame_hdr(frm_ptr, da);

    /* Add the  action frame header */
    index = add_p2p_pub_act_hdr(frm_ptr, dia_token, P2P_DEV_DISC_RSP);

    /* Store the P2P IE length offset */
    ie_len_offset = MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF + 1;

    /* Add P2P IE */
    index += add_status_attr(frm_ptr, index, (P2P_STATUS_CODE_T)status);

    /* Update the P2P IE length */
    frm_ptr[ie_len_offset] = index - ie_len_offset - 1;

    /* Transmit the management frame */
    tx_mgmt_frame(frm_ptr, index + FCS_LEN, HIGH_PRI_Q, 0);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_p2p_presence_resp                                   */
/*                                                                           */
/*  Description   : This function prepares the P2P presence response frame   */
/*                  to be sent by the GO and calls the function to transmit  */
/*                  it                                                       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming P2P presence request frame    */
/*                  2) Source address of the P2P presence request frame      */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_handle                                  */
/*                                                                           */
/*  Processing    : This function prepares the presence response frame to be */
/*                  sent by the GO. It adds the MAC header and the P2P IE    */
/*                  with status code attribute as success. It calls the      */
/*                  function to transmit the frame                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void send_p2p_presence_resp(UWORD8 *frame, UWORD8 *sa)
{
    UWORD8  dia_tok      = 0;
    UWORD16 index        = 0;
    UWORD16 ie_len_index = 0;
    UWORD8  *resp        = 0;

    /* Allocate memory for p2p presence response frame */
    resp = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                 MANAGEMENT_FRAME_LEN);

    /* If allocation fails then return */
    if(resp == NULL)
    {
        return;
    }

    /* Call the function to add the managemnet frame MAC header */
    add_p2p_mgmt_frame_hdr(resp, sa);

    /* Extract the dialog token from the frame */
    dia_tok = frame[MAC_HDR_LEN + P2P_GEN_ACT_DIALOG_TOKEN_OFF];

    index = add_p2p_gen_act_hdr(resp, dia_tok, P2P_PRESENCE_RESP);

    /* Store the P2P IE length offset */
    ie_len_index = MAC_HDR_LEN + P2P_GEN_ACT_TAG_PARAM_OFF + 1;

    /* Add the P2P attributes */
    index += add_status_attr(resp, index, P2P_STAT_SUCCESS);
    index += add_NOA_attr(resp, index, BTRUE);

    /* Update the P2P IE length */
    resp[ie_len_index] = index - ie_len_index - 1;

    /* Transmit the presence response frame */
    tx_mgmt_frame(resp, index + FCS_LEN, HIGH_PRI_Q, 0);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_grp_info_attr                                    */
/*                                                                           */
/*  Description   : This function adds the Group Info attribute to the frame */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Index to the start of the P2P IE                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the Group Info attribute to the frame */
/*                  and returns the total length of the attribute            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Total length of the attribute                   */
/*  Issues        : None                                                     */
/*****************************************************************************/
UWORD16 add_p2p_grp_info_attr(UWORD8 *ie_ptr, UWORD16 index)
{
    UWORD8 num_sec_device     = 0;
    UWORD8 dev_name_len       = 0;
    UWORD8 num_sta            = 0;
    UWORD16 attr_len          = 0;
    UWORD16 cl_info_len_off   = 0;
    UWORD16 attr_start_offset = index;

    table_elmnt_t *curr_elm   = 0;
    asoc_entry_t  *ae         = 0;

    ie_ptr[index] = P2P_GROUP_INFO;  /* Attribute ID      */
    index += P2P_ATTR_HDR_LEN;

    /* Get the first element of the sta asoc list. */
    curr_elm = (table_elmnt_t *)peek_list(&g_sta_entry_list);

    while(NULL != curr_elm)
    {
        cl_info_len_off = index++;
        ae = (asoc_entry_t *)curr_elm->element;

	   /* Add the client info only if the client is connected to the GO */
	   if(ASOC == ae->state)
	   {
			/* Increment the counter for number of associated STA */
			num_sta++;
	        mac_addr_cpy((ie_ptr + index), ae->p2p_client_dscr.p2p_dev_addr);
	        index += 6;
	
	        mac_addr_cpy((ie_ptr + index), curr_elm->key);
	        index += MAC_ADDRESS_LEN;
	
	        ie_ptr[index++] = ae->p2p_client_dscr.dev_cap_bitmap;
	
	        memcpy((ie_ptr + index), &(ae->p2p_client_dscr.config_method), 2);
	        index += 2;
	
	        memcpy((ie_ptr + index), ae->p2p_client_dscr.prim_dev_typ, 8);
	        index += 8;
	
	        num_sec_device = ae->p2p_client_dscr.num_sec_dev;
	        ie_ptr[index++] =  num_sec_device;
	#ifdef P2P_SEC_DEV_SUPPORT
	        memcpy((ie_ptr + index), ae->p2p_client_dscr.sec_dev_type_list,
	               (num_sec_device * 8));
	        index += num_sec_device * 8;
	#endif /* P2P_SEC_DEV_SUPPORT */
	
	
	        /* The device name should be put in TLV format */
	        /* Ref: Table 2, WiFi Simple Configuration     */
	        PUT_U16_BE((ie_ptr + index), WPS_ATTR_DEV_NAME_P2P);
	        index += 2;
	        dev_name_len  = strlen((WORD8 *)(ae->p2p_client_dscr.dev_name));
	        PUT_U16_BE((ie_ptr + index), dev_name_len);
	        index += 2;
	        memcpy((ie_ptr + index), ae->p2p_client_dscr.dev_name, dev_name_len);
	        index += dev_name_len;
	
	        ie_ptr[cl_info_len_off] = index - (cl_info_len_off + 1);
		}

		curr_elm = (table_elmnt_t *)next_element_list(&g_sta_entry_list,
                                                      curr_elm);
	
    }

    /* Check if ther is atleast one associated STA. Group info attribute is  */
    /* only if there is atleast one associated STA                           */
    if(num_sta != 0)
    {
	    /* Update the length field of the attribute */
	    attr_len = index - (attr_start_offset + P2P_ATTR_HDR_LEN);
	    ie_ptr[attr_start_offset + 1] = attr_len & 0x00FF;
	    ie_ptr[attr_start_offset + 2] = attr_len >> 8;
	
	    return (attr_len + P2P_ATTR_HDR_LEN);
	}
	else
	{
		return 0;
	}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : add_p2p_beacon_ie                                        */
/*                                                                           */
/*  Description   : This function adds the P2P IE to the beacon frame        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the start of the probe response frame      */
/*                  2) Index to the start of the P2P IE                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the P2P IE along with the various     */
/*                  attributes required for the beacon frame and returns the */
/*                  total length of the P2P IE added                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD16; Total lenght of the P2P IE added                */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 add_p2p_beacon_ie(UWORD8* data, UWORD16 index)
{
    UWORD16 total_len    = 0;
    UWORD16 ie_len_index = 0;

    ie_len_index = index + 1;

    /* Add P2P IE header */
    index += add_p2p_ie_hdr(&data[index]);

    /* Add P2P capability attribute */
    index += add_p2p_capability_attr(data, index, get_p2p_dev_cap(),
                                     get_p2p_grp_cap());

    /* Add P2P device ID attribute  */
    index += add_p2p_device_id_attr(data, index, mget_p2p_dev_addr());

#ifdef ENABLE_P2P_GO_TEST_SUPPORT
    /* Add P2P NOA attribute */
    index += add_NOA_attr(data, index, BTRUE);
#endif /* ENABLE_P2P_GO_TEST_SUPPORT */

    total_len = index - ie_len_index - 1;

    /* Update the length of the IE  */
    data[ie_len_index] =  total_len;

    return (total_len + IE_HDR_LEN);
}


#endif /* BSS_ACCESS_POINT_MODE */
#endif /* MAC_P2P */

