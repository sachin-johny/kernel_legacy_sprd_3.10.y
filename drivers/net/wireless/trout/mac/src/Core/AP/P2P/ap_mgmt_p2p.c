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
/*  File Name         : ap_mgmt_p2p.c                                        */
/*                                                                           */
/*  Description       : This file conatins the managemnet functions for      */
/*                      P2P protocol in GO mode                              */
/*                                                                           */
/*  List of Functions : update_p2p_asoc_client_info                          */
/*                      handle_p2p_go_disc_req_com_ap                        */
/*                      find_p2p_client_entry                                */
/*                      p2p_update_persist_cl_list                           */
/*                      p2p_probe_req_dev_id_cmp                             */
/*                      p2p_probe_req_dev_type_cmp                           */
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
#include "ap_mgmt_p2p.h"
#include "ap_frame_p2p.h"
#include "p2p_ps.h"
#include "frame_p2p.h"
#include "management_ap.h"
#include "itypes.h"
#include "mh.h"
#include "mib_p2p.h"
#include "iconfig.h"
#include "mgmt_p2p.h"
#include "qmu_if.h"
#include "management.h"
#include "wps_ap.h"


/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
BOOL_T g_target_p2p_wps_done = BFALSE;


UWORD32 g_invi_scan_chan_list[MAX_SUPP_CHANNEL] = {0};

UWORD8  g_invi_scan_chan_num = 0;
UWORD8  g_invi_scan_chan_idx = 0;
UWORD8  g_num_invi_try       = 0;

UWORD8  g_p2p_switch_chan    = 0;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_p2p_asoc_client_info                              */
/*                                                                           */
/*  Description   : This function updates the associated client info         */
/*                  supported in P2P protocol                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the association entry                      */
/*                  2) Pointer to the start of the P2P attribute list        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The function extracts the device info from the p2p       */
/*                  capability attribute and updates the association tabel   */
/*                  for the P2P client                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void update_p2p_asoc_client_info(void *asoc_entry, UWORD8 *attr_start_ptr,
                                 UWORD16 ie_len)
{
    UWORD16 index         = 0;
    UWORD16 attr_len      = 0;
#ifdef P2P_SEC_DEV_SUPPORT
    UWORD8 num_sec_device = 0;
#endif /* P2P_SEC_DEV_SUPPORT */
    UWORD8 dev_name_len   = 0;
    UWORD8* attr_ptr      = 0;
    asoc_entry_t *ae      = (asoc_entry_t *)asoc_entry;

    /* P2P Capability Attribute body */
    /*******************************************************/
    /* Device Capability  Bitmap | Group Capability Bitmap */
    /* BYTE 0                    | BYTE 1                  */
    /*******************************************************/

    attr_ptr = p2p_get_attr(P2P_CAPABILITY, attr_start_ptr, ie_len, &attr_len);
    if(NULL != attr_ptr)
    {
        ae->p2p_client_dscr.dev_cap_bitmap = attr_ptr[0];
    }

    attr_ptr = 0;
    attr_len = 0;

    /* P2P Device Info Attribute body */
    /********************************************************************/
    /* P2P Device address  | Config Method        | Primary Device type */
    /* BYTE 0 - BYTE 5     | BYTE 6 - BYTE 7      | BYTE 8 - BYTE 15    */
    /*------------------------------------------------------------------*/
    /* Num of Sec device   | Sec Device type list | Device Name         */
    /* BYTE 16             | BYTE 17 - Variable   | Variable            */
    /********************************************************************/

    attr_ptr = p2p_get_attr(P2P_DEVICE_INFO, attr_start_ptr, ie_len,
                            &attr_len);

    if(NULL != attr_ptr)
    {
        mac_addr_cpy(ae->p2p_client_dscr.p2p_dev_addr, attr_ptr);
        index = 6;
        memcpy(&(ae->p2p_client_dscr.config_method), &attr_ptr[index], 2);
        index += 2;
        memcpy(ae->p2p_client_dscr.prim_dev_typ, &attr_ptr[index], 8);
        index += 8;
#ifdef P2P_SEC_DEV_SUPPORT
        num_sec_device = attr_ptr[index];
#endif /* P2P_SEC_DEV_SUPPORT */
        index++;
#ifdef P2P_SEC_DEV_SUPPORT
        ae->p2p_client_dscr->num_sec_dev = MIN(num_sec_device,
                                               P2P_MAX_SEC_DEV);
        memcpy(ae->p2p_client_dscr.sec_dev_type_list,  &attr_ptr[index],
               (ae->p2p_client_dscr.num_sec_dev * 8));
#else/* P2P_SEC_DEV_SUPPORT */
        ae->p2p_client_dscr.num_sec_dev = 0;
#endif /* P2P_SEC_DEV_SUPPORT */

        index += ae->p2p_client_dscr.num_sec_dev * 8;

        /* Copy the device name */
        /* Ref: Table 2, WiFi Simiple Configuration, the attribute length is */
        /* at offset 2 BYTES and the attribute is at offset of 4 BYTES from  */
        /* the start of the attribute                                        */
        dev_name_len = GET_U16_BE(&attr_ptr[index + 2]);
        memcpy(ae->p2p_client_dscr.dev_name, &attr_ptr[index + 4],
               dev_name_len);

        /* Set the last byte to '\0' */
        ae->p2p_client_dscr.dev_name[dev_name_len] = '\0';
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_p2p_go_disc_req_com_ap                            */
/*                                                                           */
/*  Description   : This function does the appropriate processing after the  */
/*                  succcesful delivery of the GO discoverability frame      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Tx descriptor                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the base address of the           */
/*                  transmitted frame and checks if it is a P2P GO           */
/*                  Discoverability frame. If it is a P2P GO Discoverability */
/*                  frame then it calls the function to send the DD response */
/*                  after extracting the destination address and the dialog  */
/*                  token for the frame                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void handle_p2p_go_disc_req_com_ap(UWORD8 *tx_dscr)
{
    UWORD8 sub_type          = 0;
    UWORD8 dialog_token      = 0;
    UWORD8 dest_addr_offset  = P2P_DD_RSP_DST_ADR_OFFSET;
    UWORD8 *base_addr        = NULL;
    UWORD8 *data             = NULL;
    UWORD8 *da               = NULL;
    P2P_STATUS_CODE_T status = P2P_STAT_UNABLE_ACCO_REQ;

    /* Get the buffer start address of the Tx descriptor */
    base_addr = (UWORD8 *)get_tx_dscr_buffer_addr((UWORD32 *)tx_dscr);
    data = base_addr + MAC_HDR_LEN;

    /* Check if the frame is a P2P frame */
    if(BTRUE == is_p2p_gen_action_frame(data))
    {
        sub_type = data[P2P_GEN_ACT_OUI_SUBTYPE_OFF];

        /* Check the frame is a GO DISC REQ frame */
        if(GO_DISC_REQ == sub_type)
    {
        /* Check if the tx was successfull and set the status accordingly */
        if(is_tx_success((UWORD32 *)tx_dscr) == BTRUE)
        {
            status = P2P_STAT_SUCCESS;
        }

            /* Get the pointer to the address of the device to which the     */
            /* Device Discoverability Response has to be sent                */
            da = data + dest_addr_offset;

            /* Get the dialog token for the Device Discoverability Response  */
            /* frame                                                         */
            dialog_token = data[dest_addr_offset + MAC_ADDRESS_LEN];

        /* Send the DD response */
            send_p2p_dd_resp(da, dialog_token, status);
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name :  find_p2p_client_entry                                   */
/*                                                                           */
/*  Description   :  This function searches for a device in the sta entry    */
/*                   list                                                    */
/*                                                                           */
/*  Inputs        :  1) Pointer to the address of the target device          */
/*                   2) Pointer to the memory to store the interface address */
/*                                                                           */
/*  Globals       : g_sta_entry_list                                         */
/*                                                                           */
/*  Processing    :  This function searches if a device with the given       */
/*                   target address is present in the sta entry list of the  */
/*                   GO and if it is present then it returns the interface   */
/*                   address for the device and its association entry        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Direct  : void* Pointer to the association entry         */
/*                  Indirect: UWORD8* Pointer to the interface address       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void *find_p2p_client_entry(UWORD8* trgt_dev_addr, UWORD8* if_addr)
{
    table_elmnt_t *curr_elm   = 0;
    asoc_entry_t  *ae         = 0;

    curr_elm = (table_elmnt_t *)peek_list(&g_sta_entry_list);

    /* Search through the entry table list to find a client with the required*/
    /* P2P device address.                                                   */
    while(NULL != curr_elm)
    {
        ae = (asoc_entry_t *)curr_elm->element;
        
        /* Check if the current element matches with the requested device */
        if(BTRUE == mac_addr_cmp(ae->p2p_client_dscr.p2p_dev_addr,
                                 trgt_dev_addr))
        {
            mac_addr_cpy(if_addr, curr_elm->key);
            return ((void *)ae);
        }

        /* Get the next element in the list */
        curr_elm = (table_elmnt_t *)next_element_list(&g_sta_entry_list,
                                                      curr_elm);
    }

    /* Return NULL if no match is found. */
    return NULL;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_end_ivitation_process                                */
/*                                                                           */
/*  Description   : This function ends the invitation process                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function ends the invitation process. It resets the */
/*                  required globals. It switches back to the operating      */
/*                  channel and resumes all data qs.                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void p2p_end_ivitation_process(void)
{

    /* Reset the globals */
    g_num_invi_try = 0;
    g_invi_scan_chan_idx = 0;
    g_p2p_switch_chan = 0;

    /* Change the channel to the operating channel */
	select_channel_rf(mget_CurrentChannel(), 0);

    /* Resume all transmission */
	resume_all_data_qs();

	handle_inv_req_to(NULL);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_send_invit_ap                                        */
/*                                                                           */
/*  Description   : This function sends the invitation request when the      */
/*                  when the device in a GO                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the mac start address                      */
/*                  2) Rx length                                             */
/*                  3) Pointer to the source address                         */
/*                  4) Start index for parsing IE's                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if an invitation request is pending */
/*                  for the device sending the probe request frame. It also  */
/*                  checks if the device is capable of handling an invitation*/
/*                  request frame and then calls the function to send the    */
/*                  invitation request and starts the management timer       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_send_invit_ap(UWORD8 *msa, UWORD16 rx_len, UWORD8 *sa, UWORD8 index)
{
    UWORD8  *attr_start_ptr = NULL;
    UWORD8  *attr_ptr       = NULL;
    UWORD16 ie_len          = 0;
    UWORD16 attr_len        = 0;

    /* Check if an invitation request is pending for the device */
    if(BTRUE == (mac_addr_cmp(sa, mget_p2p_trgt_dev_id())))
    {
        /* Get the pointer to the combined P2P attributes */
        /* Note: a scratch memory need not be allocated because the */
        /* frame is expected to have only one P2P IE  */
        attr_start_ptr = get_p2p_attributes(msa, index, rx_len, &ie_len);

        if(NULL == attr_start_ptr)
		{
			return;
		}

        /* Get the P2P_CAPABILITY attribute */
        attr_ptr = p2p_get_attr(P2P_CAPABILITY, attr_start_ptr, ie_len,
                                &attr_len);

        if(NULL != attr_ptr)
        {
            /* Check if the device supports invitation procedure */
            if(CHECK_BIT(attr_ptr[0], P2P_INVIT_PROC))
            {
                /* Send the invitation request and set the invitation pending*/
                /* flag to BFALSE                                            */
                p2p_send_inv_req(0, 0, 0, (UWORD8 *)(mget_DesiredSSID()),
                                 mget_p2p_dev_addr(), mget_CurrentChannel());

                g_p2p_invit = WAIT_INVITE_RSP;
                start_mgmt_timeout_timer(RSP_TIMEOUT);
                PRINTD("Invitation Request sent succcesfully\n\r");
            }
            else
            {
                /* Call the function to end the invitatin process */
				p2p_end_ivitation_process();
                PRINTD("Device doesn't support Invitation Procedure\n\r"); 
            }
        }

        /* Free the local memory buffer allocated for P2P attributes */
		mem_free(g_local_mem_handle, attr_start_ptr);
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_update_persist_cl_list                               */
/*                                                                           */
/*  Description   : This function updates the client list in the saved       */
/*                  persistent credentials whenever a client joins the group */
/*                                                                           */
/*  Inputs        : 1) Interface address of the STA                          */
/*                                                                           */
/*  Globals       : g_persist_list                                           */
/*                                                                           */
/*  Processing    : This function gets the mac address of the client from the*/
/*                  association entry and checks if the address is already   */
/*                  present in the client list of the saved persistent       */
/*                  credentials. If not present then it adds the address to  */
/*                  list                                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void p2p_update_persist_cl_list(UWORD8 *if_addr)
{
    UWORD8  cnt       = 0;
    UWORD8 *cl_dev_id = NULL;

    /* Get the association entry of the associated sta */
    asoc_entry_t *ae = (asoc_entry_t *)find_entry(if_addr);

    /* Get the device id of the associated sta */
    cl_dev_id = ae->p2p_client_dscr.p2p_dev_addr;
    if(g_persist_list.num_cl < MAX_PERSIST_NUM_CL)
    {
        for(cnt = 0; cnt < g_persist_list.num_cl; cnt++)
        {
            /* If the device address is already present in the persistent    */
            /* list then return without doing anything                       */
            if(BTRUE == mac_addr_cmp(g_persist_list.cl_list[cnt], cl_dev_id))
                return;
        }

        /* Add the device address of the client to the persistent list */
        mac_addr_cpy(g_persist_list.cl_list[g_persist_list.num_cl++],
                     cl_dev_id);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : p2p_probe_req_dev_id_cmp                              */
/*                                                                           */
/*  Description      : This function extracts the P2P Device ID from the     */
/*                     probe request frame and compares it with that of the  */
/*                     station device address or the device address of any   */
/*                     associated clients.                                   */
/*                                                                           */
/*  Inputs           : 1) Probe Request Frame                                */
/*                     2) Frame Length                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The frame is parsed to extract the P2P attributes and */
/*                     the P2P attributes is parsed to extract the P2P       */
/*                     Device ID attribute. If either are not present, no    */
/*                     checks are required to be done. If the P2P Device ID  */
/*                     is found it is matched with the station device        */
/*                     address. If it does not match,the associated client   */
/*                     list is parsed till a match is found with the device  */
/*                     address of any client. Once a match is found,         */
/*                     no further checks are done.                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T; BFALSE: If Probe response should not be sent  */
/*                                     based on this check                   */
/*                             BTRUE:  Otherwise                             */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T p2p_probe_req_dev_id_cmp(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 comb_attr_len       = 0;
    UWORD16 p2p_dev_id_attr_len = 0;
    UWORD8 *comb_attr_ptr       = NULL;
    UWORD8 *p2p_dev_id_attr     = NULL;

    table_elmnt_t *curr_elm = 0;
    asoc_entry_t  *ae       = 0;

    /* Get the pointer to the combined P2P attributes */
    comb_attr_ptr = get_p2p_attributes(msa, MAC_HDR_LEN, rx_len,
                                       &comb_attr_len);

    /* If no P2P attributes are found, no check is required */
    if(NULL == comb_attr_ptr)
	{
		return BTRUE;
	}

    /* Extract the P2P Device ID attribute */
	p2p_dev_id_attr = p2p_get_attr(P2P_DEVICE_ID, comb_attr_ptr, comb_attr_len,
	                               &p2p_dev_id_attr_len);

	/* If no P2P Device ID attribute is present, no check is required */
	if(p2p_dev_id_attr == NULL)
	{
		/* Free the local memory buffer allocated for P2P attributes */
		mem_free(g_local_mem_handle, comb_attr_ptr);
		return BTRUE;
	}

	/* If the P2P Device ID attribute is present, compare the same with the  */
	/* device address of self. If it matches, the check passes.              */
	if(BTRUE == mac_addr_cmp(p2p_dev_id_attr, mget_p2p_dev_addr()))
	{
		/* Free the local memory buffer allocated for P2P attributes */
		mem_free(g_local_mem_handle, comb_attr_ptr);

		return BTRUE;
	}

	/* If the P2P Device ID attribute is present and does not match the      */
	/* device address of self, compare with all the connected clients by     */
	/* parsing through the association table entries. If any match is found  */
	/* the check passes.                                                     */
    curr_elm = (table_elmnt_t *)peek_list(&g_sta_entry_list);

    while(NULL != curr_elm)
    {
        ae = (asoc_entry_t *)curr_elm->element;

        if(BTRUE == mac_addr_cmp(p2p_dev_id_attr,
                                 ae->p2p_client_dscr.p2p_dev_addr))
		{
			/* Free the local memory buffer allocated for P2P attributes */
			mem_free(g_local_mem_handle, comb_attr_ptr);

			return BTRUE;
		}

        curr_elm = (table_elmnt_t *)next_element_list(&g_sta_entry_list,
                                                      curr_elm);
    }

    /* Free the local memory buffer allocated for P2P attributes */
	mem_free(g_local_mem_handle, comb_attr_ptr);

    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : p2p_probe_req_dev_type_cmp                            */
/*                                                                           */
/*  Description      : This function extracts the Requested Device Type from */
/*                     probe request frame and compares it with that of the  */
/*                     station device type or the device type of any         */
/*                     associated clients.                                   */
/*                                                                           */
/*  Inputs           : 1) Probe Request Frame                                */
/*                     2) Frame Length                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The Requested Device Type is extracted using a WPS    */
/*                     library function. If not present, no checks are to be */
/*                     done. If present and matching the device type of this */
/*                     device the library function returns this status and   */
/*                     nothing is required to be done further. Otherwise,    */
/*                     the associated client list is parsed till a match is  */
/*                     found with the device type of any client. Once match  */
/*                     is found, no further checks are done.                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T; BFALSE: If Probe response should not be sent  */
/*                                     based on this check                   */
/*                             BTRUE:  Otherwise                             */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T p2p_probe_req_dev_type_cmp(UWORD8 *msa, UWORD16 rx_len)
{
    BOOL_T send_prb_rsp = BFALSE;
    UWORD8 req_dev_type[REQ_DEV_TYPE_LEN] = {0};

    table_elmnt_t *curr_elm   = 0;
    asoc_entry_t  *ae         = 0;

    /* Call a WPS library function to process the Probe request and compare  */
    /* the Requested device type. The function should also return the value  */
    /* of the Requested device type attribute.                               */
    send_prb_rsp = wps_process_p2p_prob_req_reg(g_wps_config_ptr, msa,
                                                rx_len, req_dev_type);

    /* If WPS processing indicates that a Probe Response can be sent, no     */
    /* further checks are required                                           */
    if(send_prb_rsp == BTRUE)
    {
		return BTRUE;
	}

	/* Compare the Requested Device Type with all the connected clients by   */
	/* parsing through the association table entries. If any match is found  */
	/* the check passes.                                                     */
    curr_elm = (table_elmnt_t *)peek_list(&g_sta_entry_list);

    while(NULL != curr_elm)
    {
        ae = (asoc_entry_t *)curr_elm->element;

        if(0 == memcmp(req_dev_type, ae->p2p_client_dscr.prim_dev_typ,
                       REQ_DEV_TYPE_LEN))
        {
			return BTRUE;;
		}

#ifdef P2P_SEC_DEV_SUPPORT
        for(i = 0; i < ae->p2p_client_dscr.num_sec_dev; i++)
        {
			UWORD8 *temp = (ae->p2p_client_dscr.sec_dev_type_list) +
			               (i * REQ_DEV_TYPE_LEN);

            if(0 == memcmp(req_dev_type, temp, P2P_PRIM_DEV_TYPE_LEN))
			{
				return BTRUE;;
			}
		}
#endif /* P2P_SEC_DEV_SUPPORT */

        curr_elm = (table_elmnt_t *)next_element_list(&g_sta_entry_list,
                                                      curr_elm);
    }

    return BFALSE;
}


/* This function sets the P2P NOA schedule in AP mode */
void set_p2p_noa_sched_ap(UWORD8 *val)
{
#ifdef ENABLE_P2P_GO_TEST_SUPPORT
    /* --------------------------------------------------------------------- */
    /* Format of NoA schedule message                                        */
    /* --------------------------------------------------------------------- */
    /* | Length | NumNoA | NOA1-Cnt | NOA1-Dur | NOA1-Int | NOA1-StartTime | */
    /* --------------------------------------------------------------------- */
    /* | 1      | 1      | 1        | 4        | 4        | 4              | */
    /* --------------------------------------------------------------------- */
    /* | Present if      | NOA2-Cnt | NOA2-Dur | NOA2-Int | NOA2-StartTime | */
    /* | Num NoA sched   --------------------------------------------------- */
    /* | is 2            | 1        | 4        | 4        | 4              | */
    /* --------------------------------------------------------------------- */
    UWORD8 i     = 0;
    UWORD8 index = 0;
    UWORD8 len   = val[index++];
    UWORD8 num   = val[index++];
    UWORD32 tsf_time          = 0;
    UWORD32 start_time        = 0;
    UWORD32 start_time_offset = 0;
    /* Validity check for length of user input */
    if(len != ((num * P2P_NOA_SCHED_LEN) + 1))
        return;

    /* Validity check for max number of NoA schedule dscr that can be set */
	if (num > NUM_NOA_DSCR)
        num = NUM_NOA_DSCR;

    /* Set the number of NOA schedule descriptors */
    set_num_noa_sched(num);

    set_noa_index(get_noa_index() + 1);

    /* Update the NOA schedule descriptors */
    for(i = 0; i < num; i++)
    {
        g_noa_dscr[i].cnt_typ    = val[index++];

        g_noa_dscr[i].duration   = MAKE_WORD32(MAKE_WORD16(val[index],
                                                           val[index + 1]),
                                               MAKE_WORD16(val[index + 2],
                                                           val[index + 3]));
        index += 4;

        g_noa_dscr[i].interval   = MAKE_WORD32(MAKE_WORD16(val[index],
                                                           val[index + 1]),
                                               MAKE_WORD16(val[index + 2],
                                                           val[index + 3]));
        index += 4;

        /* Get the Start time from the Start time offset given by user */

		tsf_time = get_machw_tsf_timer_lo();
        start_time_offset = MAKE_WORD32(MAKE_WORD16(val[index],
                                                           val[index + 1]),
                                               MAKE_WORD16(val[index + 2],
                                                           val[index + 3]));
        start_time = (tsf_time + start_time_offset);

        g_noa_dscr[i].start_time = start_time;

		PRINTD("NOA%d: TSF: %x, Start Time Offset: %x, Start Time %x 2\n\r",
		       i, tsf_time, start_time_offset, start_time);
        index += 4;
    }

    /* Update the beacon frame with the new NOA schedule information */
    update_beacon_ap();
#endif /* ENABLE_P2P_GO_TEST_SUPPORT */
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : init_invi_scan_chan_list                              */
/*                                                                           */
/*  Description      : This function initializes the array with the list     */
/*                     of channels that has to be scanned                    */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function initializes the array with the list     */
/*                     of channels that has to be scanned                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void init_invi_scan_chan_list(void)
{
    UWORD8 k        = 0;
    UWORD8 i        = 0;
	UWORD8 ch_idx   = 0;
	UWORD8 freq     = get_current_start_freq();
	UWORD8 num_ch   = get_max_num_channel(freq);

	TROUT_FUNC_ENTER;
	/* Prepare the channel list. Scanning is done in the current         */
	/* frequency band set by the user, starting from the channel number  */
	/* configured by the user. The list of all channels in the frequency */
	/* band that are supported by the device and valid for the current   */
	/* regulatory domain are included in the scan request channel list.  */
	k = mget_CurrentChannel();
	for(i = 0; i < num_ch; i++, k++)
	{
		ch_idx = (k % num_ch);

		if(is_ch_idx_supported(freq, ch_idx) == BTRUE)
		{
        #if 0 //caisf add for debug, 20120728
    	    g_invi_scan_chan_list[g_invi_scan_chan_num++] = ch_idx;
        #else
            UWORD8 num_idx = g_invi_scan_chan_num++;
            if(num_idx < MAX_SUPP_CHANNEL)
			    g_invi_scan_chan_list[num_idx] = ch_idx;
            else
                TROUT_DBG2("init_invi_scan_chan_list: ERROR OCCUR! channel idx:%d is out of range!\n",num_idx);
        #endif
		}
	}
	TROUT_FUNC_EXIT;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_p2p_invitation_to                              */
/*                                                                           */
/*  Description      : This function checks if the GO should switch to a new */
/*                     channel and start sending probe request or it should  */
/*                     switch back to the operating channel                  */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function is called when a MISC Timeout happens   */
/*                     and there is an invitation request from user pending  */
/*                     for the GO to handle. It checks if it has to switch   */
/*                     to a new channel or switch back to the operating chan */
/*                     If it has to switch to a new channel then it gets the */
/*                     channel from the channel list and suspends all        */
/*                     transmissions if the new channel is not the operating */
/*                     channel. It then starts a timer to wait in this chan  */
/*                     and calls the function to probe request.              */
/*                     If it is switching back to the operating channel then */
/*                     it resumes all transmissions and changes the channel  */
/*                     If the number of tries to find the devices reaches    */
/*                     the max limit then the search is stoped.              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void handle_p2p_invitation_to(void)
{
    UWORD32 time_out  = 0;
    UWORD8  sc_cnt    = 0;
    UWORD8  chan2scan = 0;
    UWORD16 cnt       = 0;
    UWORD8  freq      = 0;
    UWORD8  tbl_idx   = 0;

    /* Check if the timeout was for starting a scan or for switching back   */
    /* to the operating channel */
    if(1 == g_p2p_switch_chan)
    {
        if(g_num_invi_try <= MAX_NUM_P2P_INVI_TRY)
        {
			time_out = P2P_INV_NON_OP_CHAN_TO;

			if (g_invi_scan_chan_idx >= g_invi_scan_chan_num)
			{
				/* The list is over. Start from the first channel */
				g_invi_scan_chan_idx = 0;
	        }

			chan2scan = g_invi_scan_chan_list[g_invi_scan_chan_idx++];

			freq      = get_current_start_freq();
			tbl_idx   = get_tbl_idx_from_ch_idx(freq, chan2scan);

			if(is_ch_tbl_idx_valid(freq, tbl_idx) == BFALSE)
			{
                return;
		    }

			if(mget_CurrentChannel() != chan2scan)
			{
				/* Suspend all transmission before switching to the chan */
				suspend_all_data_qs();

				/* Wait for the high priority queue to get over */
				while((is_machw_q_null(HIGH_PRI_Q) == BFALSE) &&
					   (cnt < WAIT_HIGH_PRI_Q_EMPTY_CNT))
				{
					add_delay(0xFFF);
					cnt++;
				}

			}


			/* Check if the channel to scan is one of the social channels */
			for(sc_cnt = 0; sc_cnt < P2P_NUM_SOCIAL_CHAN; sc_cnt++)
			{
				if(chan2scan == g_p2p_social_chan[sc_cnt])
				{
					/* The AP should send probe requests for a longer time */
					/* in the social channels                              */
					time_out = P2P_INV_SOCIAL_CHAN_TO;
				}
			}

			/* Set the current channel to chan2scan */
			select_channel_rf(chan2scan, 0);

			/* Increment the count of number of tries to find the device */
			g_num_invi_try++;

			/* Start the timer */
			start_mgmt_timeout_timer(time_out);

			/* Call the function to send probe request in the new channel */
			send_probe_request_ap();
	    }
	    else
	    {
             p2p_end_ivitation_process();
		}

	}
	else if(0 == g_p2p_switch_chan)
	{
        /* Return back to the operating channel */
        select_channel_rf(mget_CurrentChannel(), 0);
        resume_all_data_qs();

	}
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_p2p_prb_req_com_ap                                */
/*                                                                           */
/*  Description   : This function checks if the probe request was sent       */
/*                  and sends another probe requests                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Tx descriptor                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the probe request was sent       */
/*                  and calls the function to send another probe request     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void handle_p2p_prb_req_com_ap(UWORD8 *tx_dscr)
{
	UWORD8 *msa            = 0;
    TYPESUBTYPE_T frm_st   = DATA;

    /* Get the mac header address of the Tx descriptor */
    msa = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)tx_dscr);
    frm_st = get_sub_type(msa);

    /* Check if the frame is Probe Request frame */
    if(PROBE_REQ == frm_st)
    {
		/* Send another probe request */
		send_probe_request_ap();

	}

}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_p2p_inv_req_com_ap                                */
/*                                                                           */
/*  Description   : This function checks if the probe request was sent       */
/*                  and sends another probe requests                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Tx descriptor                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the probe request was sent       */
/*                  and calls the function to send another probe request     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_p2p_inv_req_com_ap(UWORD8 *tx_dscr)
{
    UWORD8 *data             = NULL;
    UWORD8 sub_type          = 0;
    UWORD8 *base_addr        = NULL;
    /* Get the buffer start address of the Tx descriptor */
    base_addr = (UWORD8 *)get_tx_dscr_buffer_addr((UWORD32 *)tx_dscr);
    data = base_addr + MAC_HDR_LEN;

    /* Check if the frame is a P2P frame */
    if(BTRUE == is_p2p_pub_action_frame(data))
    {
        sub_type = data[P2P_PUB_ACT_OUI_SUBTYPE_OFF];


        /* Check the frame is a INVITATION REQ frame */
        if(P2P_INV_REQ == sub_type)
        {
            /*If the invitation request was not sent successfully then      */
            /* change the state of invitation to SEND_INVITE and update the */
            /* scan channel index to the same channel in which the device   */
            /* was found                                                    */
            if(is_tx_success((UWORD32 *)tx_dscr) == BFALSE)
            {

                /* As the invitation was not received by the device change  */
                /* the state of invitation to SEND_INVITE so that the GO    */
                /* retires to send the invitation request frame             */
                g_p2p_invit = SEND_INVITE;

                /* Set the switch channel flag to 0 so that on  timeout the */
                /* GO returns back to its operating channel                 */
                g_p2p_switch_chan = 0;

                /* Decrement the index so that the next scan starts from    */
                /* the same channel                                         */
                g_invi_scan_chan_idx--;

            }

        }
    }

}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_is_dev_found                                         */
/*                                                                           */
/*  Description   : This function processes the p2p ie present in the        */
/*                  probe response frame to find the device for which an     */
/*                  invitation is pending                                    */
/*                                                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the probe response frame                   */
/*                  2) Index to the start of the tagged parameters           */
/*                  3) Length of the probe response frame                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function processes the p2p ie present in the probe  */
/*                  response frame. It checks the P2P Device Info attribute  */
/*                  and P2P Group Info attribute(in case the probe response) */
/*                  is sent by a GO to find the device for which an ivitation*/
/*                  is pending. It retunrns back the result of the search    */
/*                  and the device capability of the device if match is      */
/*                  found                                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
BOOL_T p2p_is_dev_found(UWORD8 *msa, UWORD16 index, UWORD16 rx_len,
                        UWORD8 *dev_cap)
{
	UWORD16 ie_len           = 0;
	UWORD16 attr_len         = 0;
	UWORD8  grp_cap          = 0;
	UWORD8  *attr_start_ptr  = NULL;
	UWORD8  *attr_ptr        = NULL;

	/* Get the pointer to the combined P2P attributes */
	attr_start_ptr = get_p2p_attributes(msa, index, rx_len, &ie_len);

	if(NULL != attr_start_ptr)
	{
		/* Get the P2P Capability attribute */
		attr_ptr = p2p_get_attr(P2P_CAPABILITY, attr_start_ptr, ie_len,
										&attr_len);
		if(NULL == attr_ptr)
		{
			return BFALSE;
		}


		/* Get the device capability */
		*dev_cap = attr_ptr[0];
		/* Get the group capability */
		grp_cap  = attr_ptr[1];



		/* Get the P2P Device Info attribute */
		attr_ptr = p2p_get_attr(P2P_DEVICE_INFO, attr_start_ptr, ie_len,
								&attr_len);

		if(NULL == attr_ptr)
		{
			return BFALSE;
		}

		if(BTRUE == mac_addr_cmp(attr_ptr, mget_p2p_trgt_dev_id()))
		{
			return BTRUE;

		}

		/* Check if the probe response was sent by a P2P GO */
		if(CHECK_BIT(grp_cap, P2PCAPBIT_GO))
		{
			attr_ptr = p2p_get_attr(P2P_GROUP_INFO, attr_start_ptr,
									ie_len, &attr_len);

			if(attr_ptr != NULL)
			{
			    index = 0;

				/* Parse the Group Info attribute to check the client */
				/* device address                                     */
				while(index < attr_len)
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

					if(BTRUE ==
					   mac_addr_cmp(mget_p2p_trgt_dev_id(),
					                attr_ptr+index+1))
					{
						*dev_cap = attr_ptr[index + 13];
						return BTRUE;
					}
					index += attr_ptr[index] + 1;

				}
		    }

		}

		/* Free the local memory buffer allocated for P2P attributes */
		mem_free(g_local_mem_handle, attr_start_ptr);

     }

     return BFALSE;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_handle_probe_rsp                                     */
/*                                                                           */
/*  Description   : This function processes the probe response frame to      */
/*                  find the device for which the probe response is pending  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the probe response frame                   */
/*                  2) Length of the probe response frame                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the incoming probe response      */
/*                  frame is a P2P frame. If it is a P2P frame then it calls */
/*                  the function to find the device for which an invitation  */
/*                  is pending. If the match is found then it checks if the  */
/*                  device supports invitation. If is does then a p2p        */
/*                  invitation frame is sent and a timer is started.         */
/*                  If it doesnot support invitation then all transmissions  */
/*                  are resumed, channel is switched to the operating channel*/
/*                  and function to end the invitation process is called     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void p2p_handle_probe_rsp(UWORD8 *msa, UWORD16 rx_len)
{
    /* Check if the probe response has a P2P IE */
    UWORD8 dev_cap      = 0;
    UWORD16 index       = is_p2p_frame(msa, TAG_PARAM_OFFSET, rx_len);
    BOOL_T  match_found = BFALSE;

    if(index < (rx_len - FCS_LEN))
    {
         match_found = p2p_is_dev_found(msa, TAG_PARAM_OFFSET, rx_len,
                                        &dev_cap);
    }

    if(BTRUE == match_found)
    {

        if(CHECK_BIT(dev_cap, P2P_INVIT_PROC))
        {

         	/* Send the invitation request and set the invitation pending*/
			/* flag to WAIT_INVITE_RSP                                   */
			p2p_send_inv_req(0, 0, 0, mget_DesiredSSID(),
							 mget_p2p_dev_addr(), mget_CurrentChannel());

			g_p2p_invit = WAIT_INVITE_RSP;
			cancel_mgmt_timeout_timer();
		    start_mgmt_timeout_timer(P2P_WAIT_INV_RSP_TO);

	    }
	    else
	    {
			/* Call the function to end the invitation process */
			p2p_end_ivitation_process();
		}

	}
}

#endif /* MAC_P2P */
#endif /* BSS_ACCESS_POINT_MODE */
