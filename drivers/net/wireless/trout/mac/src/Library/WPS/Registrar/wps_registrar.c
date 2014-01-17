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
/*  File Name         : wps_registrar.c                                      */
/*                                                                           */
/*  Description       : This file contains the interface functions of WPS    */
/*                      Registrar Library                                    */
/*                                                                           */
/*  List of Functions : wps_init_registrar                                   */
/*                      wps_start_registrar                                  */
/*                      wps_stop_registrar                                   */
/*                      wps_insert_wsc_ie                                    */
/*                      wps_process_probe_req                                */
/*                      wps_handle_eap_rx                                    */
/*                      wps_handle_event                                     */
/*                      handle_pbc_probe_req                                 */
/*                      wps_start_reg_prot_timer                             */
/*                      wps_start_eap_msg_timer                              */
/*                      wps_handle_reg_prot_timeout                          */
/*                      wps_handle_eap_msg_timeout                           */
/*                      reassemble_wsc_ie_reg                                */
/*                      is_wsc_ie_reassm_req_reg                             */
/*                      check_wps_cap_assoc_req                              */
/*                      wps_set_ver2_cap_sta                                 */
/*                      is_wps_in_prog_reg                                   */
/*                      wps_process_p2p_prob_req_reg                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_REG_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_registrar.h"
#include "wps_eap_rx_reg.h"
#include "wps_eap_tx_reg.h"
#include "trout_trace.h"
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static STATUS_T wps_start_reg_prot_timer(wps_t *wps_ctxt_hdl);
static void handle_pbc_probe_req(wps_t *wps_ctxt_hdl, UWORD8 *mac_addr,
                                 UWORD8 *uuid_e);
static void wps_handle_reg_prot_timeout(wps_t *wps_ctxt_hdl);
static void wps_handle_eap_msg_timeout(wps_t *wps_ctxt_hdl);
static void reassemble_wsc_ie_reg(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                                  UWORD8 *buffer);
static BOOL_T is_wsc_ie_reassm_req_reg(UWORD8 *msa, UWORD16 rx_len,
                                       UWORD16 index,
                                       UWORD16 *total_wsc_ie_len,
                                       UWORD8 **buffer);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_init_registrar                                    */
/*                                                                           */
/*  Description      : This function does WPS Protocol Specific Registrar    */
/*                     initialization                                        */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function is to be called on every MAC-RESET.     */
/*                     This sets members in registrar configuration structure*/
/*                     to default values and generates registrar`s UUID      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : Status of the function processing(SUCCESS/FAILURE)    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_init_registrar(wps_t *wps_ctxt_hdl)
{
    wps_reg_t    *reg_hdl  = wps_ctxt_hdl->wps_reg_hdl;
    wps_priv_t   *priv_hdl = wps_ctxt_hdl->wps_priv_hdl;
    wps_attr_t   *attr_hdl = wps_ctxt_hdl->wps_attr_hdl;
    UWORD8*      temp_ptr  = attr_hdl->reg_pub_key.pk;


    /* Set Selected Registrar attribute to BFALSE */
    attr_hdl->sel_reg = BFALSE;

    /* Set the Association State atribute */
    attr_hdl->assoc_state = WPS_CONNECTION_SUCCESS;

    /* Set Configuration Error attribute to No Error */
    attr_hdl->config_error = WPS_NO_ERROR;

    /* Reset the PBC Enrollee count */
    reg_hdl->pbc_enr_cnt = 0;

    /* Initialize the Private Key Pointer */
    priv_hdl->priv_key_ptr.val_ptr = priv_hdl->olap_priv_key.priv_key;

    /* Generate Registrar`s UUID-R */
    wps_gen_uuid(wps_ctxt_hdl->wps_config_hdl->mac_addr, attr_hdl->reg_uuid,
                 temp_ptr);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_start_registrar                                   */
/*                                                                           */
/*  Description      : This function starts Registration Protocol on the     */
/*                     registrar in desired mode                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the start request type (PIN or*/
/*                     PBC) matches the Device Password ID. It also checks   */
/*                     if there is PBC session overlap in case of PBC mode.  */
/*                     If all the checks pass then the registrar selection is*/
/*                     indicated in the beacon and Registration Protocol     */
/*                     timer is started                                      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - If start request was accepted successfully  */
/*                     FAILURE - If start request processing failed          */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_start_registrar(wps_t *wps_ctxt_hdl)
{
    wps_config_t *config_hdl = wps_ctxt_hdl->wps_config_hdl;

    switch(config_hdl->prot_type)
    {
        /*********************************************************************/
        /* If Protocol type is PIN then check whether device password and    */
        /* device password ID are valid                                      */
        /*********************************************************************/
        case PIN:
            if((NULL == config_hdl->dev_pin)  ||
               (0 == config_hdl->dev_pin_len) ||
               (MAX_WPS_PIN_LEN < config_hdl->dev_pin_len))
            {
                return FAILURE;
            }
            switch(config_hdl->dev_pass_id)
            {
                case PASS_ID_DEFAULT:
                case PASS_ID_USER_SPECIFIED:
                case PASS_ID_MACH_SPECIFIED:
                case PASS_ID_REKEY:
                case PASS_ID_REG_SPECIFIED:
                    break;
                default:
                    return FAILURE;
            }
        break;
        case PBC:
            /* Check if the device password ID is set to PBC */
            if(PASS_ID_PUSHBUTTON != config_hdl->dev_pass_id)
                return FAILURE;

            /* If there is a session overlap then indicate the same to host  */
            if(FAILURE == wps_handle_pbc_start_req(wps_ctxt_hdl))
            {
                send_wps_status(WPS_REG_PBC_SESSION_OVERLAP, NULL, 0);
                return FAILURE;
            }
        break;
        default:
            return FAILURE;
    }

    /* Start Registration Protocol timer for 120 seconds */
    if(SUCCESS != wps_start_reg_prot_timer(wps_ctxt_hdl))
        return FAILURE;

    /* Set start_req flag to BTRUE */
    wps_ctxt_hdl->wps_reg_hdl->start_req  = BTRUE;

    /* Set Selected Registrar attribute to BTRUE indicating the user has     */
    /* activated the registrar for Registration Protocol                     */
    wps_ctxt_hdl->wps_attr_hdl->sel_reg = BTRUE;

    /* Indicate the change in attributes in beacon */
    wps_update_beacon_wsc_ie_ap();

    /* WPS protocol started successfully, display the message */
    send_wps_status(WPS_REG_START_REQ, NULL, 0);

    return SUCCESS;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_stop_registrar                                    */
/*                                                                           */
/*  Description      : This function stops the WPS enrollee protocol         */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Value of selected registrar attribute              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function stops all the running timers and free   */
/*                     all the memory allocated and stops the registration   */
/*                     protocol                                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_stop_registrar(wps_t *wps_ctxt_hdl, BOOL_T start_req)
{
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_reg_t    *reg_hdl    = wps_ctxt_hdl->wps_reg_hdl;

    /* Reset the start request flag */
    reg_hdl->start_req = start_req;
    attr_hdl->sel_reg  = start_req;

    /* Registrar and Registration Protocol States to IDLE */
    reg_hdl->wpsr_state     = WPS_IDLE_ST;
    reg_hdl->reg_prot_state = REG_IDLE_ST;
    reg_hdl->msg_state      = IDLE_ST;

    /* Reflect these changes in Beacon */
    wps_update_beacon_wsc_ie_ap();

    /* Stop and free all the timers */
    wps_delete_timers(wps_ctxt_hdl);

    /* Free all the memories allocated */
    wps_free_all_memory(wps_ctxt_hdl);
    return;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_insert_wsc_ie_probe_req                              */
/*                                                                           */
/*  Description   : This function adds the WSC IE to the frame               */
/*                                                                           */
/*  Inputs        : 1) Pointer to main WPS context structure                 */
/*                  2) Pointer to the Frame                                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the WSC IE to the frame               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD8, Total length of the WSC IE                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf add, merged ittiam mac v1.2 code
UWORD16 p2p_insert_wsc_ie_probe_req(wps_t *wps_ctxt_hdl, UWORD8 *buffer)
{
	UWORD8         sub_elem_cnt = 0;
    UWORD16        index        = 0;
    wps_config_t   *config_hdl  = wps_ctxt_hdl->wps_config_hdl;
    wps_attr_t     *attr_hdl    = wps_ctxt_hdl->wps_attr_hdl;
    wps_sub_elem_t *sub_elem_ptr[MAX_SUB_ELEM_SUPP];

    /* Insert Version Attribute */
    index += put_wsc_version((buffer + index));


    /* Insert Request type */
    index +=  put_wsc_req_type((buffer + index), REQ_REGISTRAR);


    /* Insert Configuration Methods Attribute */
	index += put_wsc_config_meth((buffer + index),
                                  config_hdl->config_methods);

    /* Insert UUID-E Attribute */
    index += put_wsc_uuid_e((buffer + index), attr_hdl->reg_uuid);

    /* Insert Primary Device Type Attribute */
	index += put_wsc_prim_dev_type((buffer + index),
                                    config_hdl->prim_dev_cat_id);


    /* Insert RF Band */
    index += put_wsc_rf_bands((buffer + index), config_hdl->rf_bands);

    /* Insert the assoc state */
    index += put_wsc_assoc_state((buffer + index), attr_hdl->assoc_state);

    index += put_wsc_config_error((buffer + index), WPS_NO_ERROR);


	/* Insert Device password id */
	if(PBC == config_hdl->prot_type)
	{
		index += put_wsc_dev_pass_id((buffer + index),
			(UWORD16) DEV_PASS_ID_PUSHBUTT);
	}
	else
	{
		index += put_wsc_dev_pass_id((buffer + index), config_hdl->dev_pass_id);
    }

    /* Insert Manufacturer Attribute */
	index += put_wsc_manufacturer((buffer + index),
								  config_hdl->manufacturer,
								  config_hdl->manufacturer_len);

	/* Insert Model Name Attribute */
	index += put_wsc_model_name((buffer + index), config_hdl->model_name,
								config_hdl->model_name_len);

	/* Insert Model Number Attribute */
	index += put_wsc_model_num((buffer + index), config_hdl->model_num,
							   config_hdl->model_num_len);


	/* Insert Device Name Attribute */
    index += put_wsc_device_name((buffer + index), config_hdl->device_name,
                                  config_hdl->device_name_len);


#ifdef MAC_P2P
    index += put_wsc_req_dev_type((buffer + index), config_hdl->req_dev_cat_id);
#endif /* MAC_P2P */

	/* Insert WFA Vendor Extension Attribute with desired Sublements */
    index += put_wsc_wfa_vendor((buffer + index), sub_elem_ptr, sub_elem_cnt);


    return index;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_insert_wsc_ie                                     */
/*                                                                           */
/*  Description      : This function inserts WSC-IE in Beacon and Probe      */
/*                     response frame passed                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the Frame                               */
/*                     3) Offset for inserting WSC-E                         */
/*                     4) Type of the Frame (BEACON/PROBE-RESP)              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function inserts WSC-IE in Beacon and Probe      */
/*                     response frame. The attributes inserted are as per    */
/*                     defined in the standard                               */
/*                                                                           */
/*  Outputs          : Length of the inserted WSC-IE                         */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

UWORD16 wps_insert_wsc_ie(wps_t *wps_ctxt_hdl, UWORD8 *frame_ptr,
                          UWORD8 frm_type)
{
    UWORD16 index = 0;

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
    if((BEACON != frm_type) && (PROBE_RSP != frm_type) &&
       (ASSOC_RSP != frm_type) && (REASSOC_RSP != frm_type))
#else
    if((BEACON != frm_type) && (PROBE_RSP != frm_type) &&
       (ASSOC_RSP != frm_type) && (REASSOC_RSP != frm_type) &&
       (PROBE_REQ != frm_type))
#endif
        return 0;

    /* WPS Information Element                                               */
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |     Element ID     |      Length   |      OUI    |     Data        |*/
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |         1 Byte     |       1 Byte  |     4 Byte  |     1-251 Bytes |*/
    /* +--------------------+---------------+-------------+-----------------+*/

    /* Set the Element ID field */
    frame_ptr[index++] = WPS_IE_ID;

    /* Length field is updated later */
    index++;

    /* Update OUI-Field */
    frame_ptr[index++] = WPS_IE_OUI_BYTE0;
    frame_ptr[index++] = WPS_IE_OUI_BYTE1;
    frame_ptr[index++] = WPS_IE_OUI_BYTE2;
    frame_ptr[index++] = WPS_IE_OUI_BYTE3;

// 20120709 caisf add, merged ittiam mac v1.2 code
    if(PROBE_REQ == frm_type)
    {
    	index += p2p_insert_wsc_ie_probe_req(wps_ctxt_hdl, (frame_ptr + index));

    }
    else
    {
    	index += insert_wsc_ie(wps_ctxt_hdl, (frame_ptr + index), frm_type);
	}

    /* Update the Length Field */
    frame_ptr[1] = (index - 2);

    return index;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_probe_req                                 */
/*                                                                           */
/*  Description      : This function processes the received probe request    */
/*                     frame                                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the received probe request frame        */
/*                     3) Pointer to the SA of probe request frame           */
/*                     4) Length of the received probe request frame         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks for WSC-IE. If present then      */
/*                     checks if the Enrollee is in PBC mode. If in PBC mode */
/*                     then the following is done:                           */
/*                      1. If PBC enrollee count is zero then update the     */
/*                         PBC enrollee info structure and exit              */
/*                      2. If the TSF time difference is greater than 120 sec*/
/*                         then update the pbc_info set pbc enrollee count to*/
/*                         1 and exit                                        */
/*                      3. If UUID and MAC-address is same then update the   */
/*                         timestamp in enr_info and exit                    */
/*                      4. Update the UUID and MAC address in enr_info and   */
/*                         increment PBC enrollee count by 1 and if this     */
/*                         count is greater than 1 then set PBC Session      */
/*                         Overlap flag and exit.                            */
/*                                                                           */
/*  Outputs          : None                                                  */
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

void wps_process_probe_req(wps_t *wps_ctxt_hdl, UWORD8 *msa, UWORD8 *sa,
                           UWORD16 rx_len, UWORD8 frm_type)
{
    UWORD16  wsc_ie_len         = 0;
    UWORD16  ie_start_index     = 0;
    wps_rx_attri_t rx_attr      = {0};
    wps_rx_attri_t *rx_attr_ptr = &rx_attr;
    UWORD8 *wsc_ie_buf          = NULL;
    UWORD8 *buffer              = NULL;

    /* Process only Probe request and (Re)Association Request frame */
    if(PROBE_REQ != frm_type)
       return;

    /* Get the IE start index */
    ie_start_index = WPS_MAC_HDR_LEN;

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_reg(msa, rx_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_local_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_reg(msa, rx_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    /* If WSC-IE not present then exit */
    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
        return;

    /* Parse for TLV type WSC attributes */
    if(0 == wps_parse_attri(wsc_ie_buf, &rx_attr_ptr, wsc_ie_len, 1, BTRUE))
    {
        if(NULL != buffer)
            wps_local_mem_free(buffer);

        return;
    }

    /* If the Enrollee is not in PBC mode then exit */
    if(PASS_ID_PUSHBUTTON != get_wsc_dev_pass_id(rx_attr_ptr->dev_pass_id))
    {
        if(NULL != buffer)
            wps_local_mem_free(buffer);

        return;
    }

    /* handle PBC enabled Enrollee`d Probe Request */
    handle_pbc_probe_req(wps_ctxt_hdl, sa, rx_attr_ptr->uuid);

    if(NULL != buffer)
        wps_local_mem_free(buffer);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_rx                                     */
/*                                                                           */
/*  Description      : This function handles reception of EAP RX frame       */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the received frame                      */
/*                     3) Length of the received frame                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following:                     */
/*                     1) Checks for presence of EAPOL SNAP header           */
/*                     2) Checks for compatible 802.1x protocol version      */
/*                     3) Depending on 802.1x packet type further processing */
/*                        functions are called.                              */
/*                     If any of the above checks fail, the status of FAILURE*/
/*                     is returned                                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - If Packet is processed successfully         */
/*                     FAILURE - If Packet is processing fails               */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_handle_eap_rx(wps_t *wps_ctxt_hdl, UWORD8* msa, UWORD16 rx_len)
{
    UWORD8   onex_version = 0;
    UWORD16  onex_pkt_len = 0;
    STATUS_T retval       = SUCCESS;

    /* If not an EAPOL SNAP header then exit */
    if(BFALSE == is_eapol_snap_header(msa))
        return FAILURE;

    /* If 1x version is not 0x01 or 0x02 then exit */
    onex_version = get_1x_version(msa);
    if((WPS_EAP_VERSION_1 != onex_version) &&
       (WPS_EAP_VERSION_2 != onex_version))
        return FAILURE;

    /* Extract 1x packet len */
    onex_pkt_len = get_1x_pkt_len(msa);

    /*************************************************************************/
    /* If the RXed length indicated is less than sum of 1X Header Length and */
    /* EAP Packet Length, then exit                                          */
    /*************************************************************************/
    if(rx_len < (onex_pkt_len + ONE_X_HEADER_LEN))
        return FAILURE;

    switch(get_1x_pkt_type(msa))
    {
        case ONE_X_PKT_TYPE_EAPOL_START:
            retval = wps_handle_eapol_start_rx(wps_ctxt_hdl);
        break;
        case ONE_X_PKT_TYPE_EAP:
            /*****************************************************************/
            /* For EAPOL-Packet, the Packet Length cannot be less than EAP   */
            /* header size                                                   */
            /*****************************************************************/
            if(EAP_HEADER_LEN > onex_pkt_len)
                return FAILURE;

            retval = wps_handle_eapol_pkt_rx(wps_ctxt_hdl, msa, onex_pkt_len);
        break;
        default:
            return FAILURE;
    }

    return retval;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_event                                      */
/*                                                                           */
/*  Description      : This function handles WPS timeout events of the       */
/*                     registrar                                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Timeout data                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following:                     */
/*                     1) If the Timout data is WPS_EAP_MSG_TIMEOUT then     */
/*                        corresponding function to handle EAP TX message    */
/*                        timeout is called                                  */
/*                     2) If the Timout data is WPS_REG_PROT_TIMEOUT then    */
/*                        corresponding function to handle Registration      */
/*                        Protocol timeout is called                         */
/*                                                                           */
/*  Outputs          : None                                                  */
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

void wps_handle_event(wps_t *wps_ctxt_hdl, UWORD32 inp_data)
{
	TROUT_FUNC_ENTER;
    switch(inp_data)
    {
        case WPS_EAP_MSG_TIMEOUT:
            wps_handle_eap_msg_timeout(wps_ctxt_hdl);
        break;
        case WPS_REG_PROT_TIMEOUT:
            wps_handle_reg_prot_timeout(wps_ctxt_hdl);
        break;
		// 20120709 caisf add, merged ittiam mac v1.2 code
        case WPS_DEAUTH_TIMEOUT:
        {
            wps_config_t   *config_hdl = wps_ctxt_hdl->wps_config_hdl;

            sys_stop_wps_reg(UNSPEC_REASON, config_hdl->mac_addr, BFALSE);
	    }
	    break;
        default:
            /* Do Nothing */
        break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : is_wps_in_prog_reg                                    */
/*                                                                           */
/*  Description      : This returns the status of Registration Protocol of   */
/*                     the registrar                                         */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks the WPS Status of the Registrar  */
/*                     and return BTRUE if Registration Protocol is in       */
/*                     progress else return BFALSE                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         11 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_wps_in_prog_reg(wps_t *wps_ctxt_hdl)
{
    BOOL_T retval = BFALSE;

    /*************************************************************************/
    /* Only if the WPS State indicates in Registration Protocol then return  */
    /* BTRUE else return BFALSE                                              */
    /*************************************************************************/
    if(WPS_REG_PROT_ST == wps_ctxt_hdl->wps_reg_hdl->wpsr_state)
        retval = BTRUE;

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_pbc_probe_req                                  */
/*                                                                           */
/*  Description      : This function processes the received PBC probe request*/
/*                     frame                                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the SA of probe request frame           */
/*                     3) Pointer to the UUID_E in WSC-IE of probe request   */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following:                     */
/*                      1. If number of PBC enrollees is Zero then update the*/
/*                         current enrollee at index 0 of the PBC Enrollee   */
/*                         information array and return                      */
/*                      2. If the number of PBC Enrollees detected is 1 then */
/*                         - If the enrollee in the entry is same as the     */
/*                           current enrollee, then update the TSF values at */
/*                           this index and return                           */
/*                         - If the entry is older than MONITOR Window then  */
/*                           replace this with input entry and return        */
/*                         - Update the input entry at index 1 and raise     */
/*                           Session Overlap Flag                            */
/*                           - If PBC Registration in Progres then stop it   */
/*                             and return                                    */
/*                      3. If the number of PBC Enrollee detected is 2 then  */
/*                         - If the input entry is same of either of the     */
/*                           existing entry then update the same and arrage  */
/*                           the etries in increasing order of TSF values and*/
/*                           return                                          */
/*                         - Move entry at index 1 to index 0 and then update*/
/*                           the input entry at index 1 and exit             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static void handle_pbc_probe_req(wps_t *wps_ctxt_hdl, UWORD8 *mac_addr,
                                 UWORD8 *uuid_e)
{
    wps_reg_t *reg_hdl = wps_ctxt_hdl->wps_reg_hdl;
    wps_mon_pbc_t *pbc_enr_info = reg_hdl->pbc_enr_info;
    UWORD32 cur_tsf_hi = 0;
    UWORD32 cur_tsf_lo = 0;

    /* Get the current TSF timer value */
    wps_get_tsf_timestamp(&cur_tsf_hi, &cur_tsf_lo);

    /*************************************************************************/
    /* If this is the first PBC enrollee found then update the PBC           */
    /* information at index 0 and exit                                       */
    /*************************************************************************/
    if(0 == reg_hdl->pbc_enr_cnt)
    {
        update_pbc_enr_info(pbc_enr_info, mac_addr, uuid_e, cur_tsf_hi,
                            cur_tsf_lo, 0);
        reg_hdl->pbc_enr_cnt++;
        return;
    }

    /* If there is one entry in the array */
    if(1 == reg_hdl->pbc_enr_cnt)
    {
        /*********************************************************************/
        /* If the PBC enrollee at this index 0 is same a the current enrollee*/
        /* or if the time difference when this enrollee was detected is      */
        /* greater than MONITOR window then update this entry with           */
        /* information of current enrollee                                   */
        /*********************************************************************/
        if((0 == memcmp(pbc_enr_info[0].mac_addr, mac_addr, MAC_ADDRESS_LEN))||
#ifdef MAC_P2P // 20120709 caisf add the "ifdef", merged ittiam mac v1.2 code
           (0 == memcmp(pbc_enr_info[0].uuid_e, uuid_e, MAC_ADDRESS_LEN))||
#endif /* MAC_P2P */
           (BFALSE == in_mon_window(cur_tsf_hi, cur_tsf_lo,
                                    pbc_enr_info[0].tsf_timer_hi,
                                    pbc_enr_info[0].tsf_timer_lo)))
        {
            update_pbc_enr_info(pbc_enr_info, mac_addr, uuid_e, cur_tsf_hi,
                                cur_tsf_lo, 0);
            return;
        }

        /*********************************************************************/
        /* Update the enrollee info at index 1 and set the session overlap   */
        /* flag                                                              */
        /*********************************************************************/
        update_pbc_enr_info(pbc_enr_info, mac_addr, uuid_e, cur_tsf_hi,
                            cur_tsf_lo, 1);
        reg_hdl->pbc_enr_cnt++;

        /*********************************************************************/
        /* If PBC Registration Protocol is requested then Stop it and        */
        /* indicate SS overlap to user                                       */
        /*********************************************************************/
        if((BTRUE == reg_hdl->start_req) &&
           (PBC == wps_ctxt_hdl->wps_config_hdl->prot_type))
        {
            /* If Registration Protocol is in Progress then Send EAP Failure */
            /* to stop the EAP process                                       */
            if(WPS_REG_PROT_ST == wps_ctxt_hdl->wps_reg_hdl->wpsr_state)
                wps_send_eap_fail(wps_ctxt_hdl, wps_ctxt_hdl->wps_eap_tx_hdl);

            send_wps_status(WPS_REG_PBC_SESSION_OVERLAP, NULL, 0);
            sys_stop_wps_reg(UNSPEC_REASON,
                             wps_ctxt_hdl->wps_config_hdl->mac_addr,
                             BFALSE);
        }
    }
    /*************************************************************************/
    /* If already in session overlap then update the current enrollee at     */
    /* appropriate index                                                     */
    /*************************************************************************/
    else if(2 == reg_hdl->pbc_enr_cnt)
    {
        /* Check if this is from same Enrollee at index 0 */
        if(0 == memcmp(pbc_enr_info[0].mac_addr, mac_addr, MAC_ADDRESS_LEN))
        {
            update_pbc_enr_info(pbc_enr_info, mac_addr, uuid_e, cur_tsf_hi,
                                cur_tsf_lo, 0);
            swap_entries(pbc_enr_info);
            return;
        }

        /* Check if this is from same Enrollee at index 1 */
        if(0 == memcmp(pbc_enr_info[1].mac_addr, mac_addr, MAC_ADDRESS_LEN))
        {
            update_pbc_enr_info(pbc_enr_info, mac_addr, uuid_e, cur_tsf_hi,
                                cur_tsf_lo, 1);
            return;
        }

        /*********************************************************************/
        /* Since this is a new PBC probe request move the entry at index 1 to*/
        /* index 0 and update index 1 with information of this new Enrollee  */
        /*********************************************************************/
        memcpy(&pbc_enr_info[0], &pbc_enr_info[1], sizeof(wps_mon_pbc_t));
        update_pbc_enr_info(pbc_enr_info, mac_addr, uuid_e, cur_tsf_hi,
                            cur_tsf_lo, 1);
    }

    return;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_start_reg_prot_timer                              */
/*                                                                           */
/*  Description      : This function handles the start of Registration       */
/*                     Protocol timer                                        */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function creates a new alarm if not created      */
/*                     already. If the alarm is running then stops it and    */
/*                     starts the alarm with timeout value REG_PROT_TIMEOUT  */
/*                                                                           */
/*  Outputs          : Status of alarm start                                 */
/*                                                                           */
/*  Returns          : SUCCESS - If Alarm is started successfully            */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_start_reg_prot_timer(wps_t *wps_ctxt_hdl)
{
    /* Create alarm if not created */
    if(NULL == wps_ctxt_hdl->reg_prot_timer)
    {
        wps_ctxt_hdl->reg_prot_timer = wps_create_alarm(
                                          wps_ctxt_hdl->wps_timer_cb_fn_ptr,
                                          WPS_REG_PROT_TIMEOUT, NULL);  //Hugh: fixme.

        /* If creation of alarm fails then raise system error and return     */
        /* FAILURE                                                           */
        if(NULL == wps_ctxt_hdl->reg_prot_timer)
        {
            wps_handle_sys_err_reg(SYSTEM_ERROR);
            return FAILURE;
        }
    }
    /* Stop the alarm if running */
    else
        wps_stop_alarm(wps_ctxt_hdl->reg_prot_timer);

    /* Start the alarm */
    if(BFALSE == wps_start_alarm(wps_ctxt_hdl->reg_prot_timer,
                                 REG_PROT_TIMEOUT))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_start_eap_msg_timer                               */
/*                                                                           */
/*  Description      : This function handles the start of EAP Message timer  */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function creates a new alarm if not created      */
/*                     already. If the alarm is running then stops it and    */
/*                     starts the alarm with timeout value EAP_MSG_TIMEOUT   */
/*                                                                           */
/*  Outputs          : Status of alarm start                                 */
/*                                                                           */
/*  Returns          : SUCCESS - If Alarm is started successfully            */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_start_eap_msg_timer(wps_t *wps_ctxt_hdl)
{
    /* Create alarm if not created */
    if(NULL == wps_ctxt_hdl->eap_msg_timer)
    {
        wps_ctxt_hdl->eap_msg_timer = wps_create_alarm(
                                          wps_ctxt_hdl->wps_timer_cb_fn_ptr,
                                          WPS_EAP_MSG_TIMEOUT, NULL);  //Hugh: fixme.

        /* If creation of alarm fails then raise system error and return     */
        /* FAILURE                                                           */
        if(NULL == wps_ctxt_hdl->eap_msg_timer)
        {
            wps_handle_sys_err_reg(SYSTEM_ERROR);
            return FAILURE;
        }
    }
    /* Stop the alarm if running */
    else
        wps_stop_alarm(wps_ctxt_hdl->eap_msg_timer);

    /* Start the alarm */
    if(BFALSE == wps_start_alarm(wps_ctxt_hdl->eap_msg_timer, EAP_MSG_TIMEOUT))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_stop_eap_msg_timer                                */
/*                                                                           */
/*  Description      : This function stops EAP Message timer                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function stops the EAP message timer if running  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_stop_eap_msg_timer(wps_t *wps_ctxt_hdl)
{
    /* Stop the timer only if the alarm handle is valid */
    if(NULL != wps_ctxt_hdl->eap_msg_timer)
        wps_stop_alarm(wps_ctxt_hdl->eap_msg_timer);

    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_reg_prot_timeout                           */
/*                                                                           */
/*  Description      : This function handles Registration Protocol timeout   */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function call sys_stop_wps_reg if                */
/*                     Registration Protocol is in progress                  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static void wps_handle_reg_prot_timeout(wps_t *wps_ctxt_hdl)
{
    /* Indicate Walk Timeout to Host */
    send_wps_status(WPS_REG_REG_PROT_WALK_TIMEOUT, NULL, 0);

    /* Do not proces the timeout if the Registration Protocol is in Progress */
    if(BTRUE == is_wps_in_prog_reg(wps_ctxt_hdl))
        return;

    /* Stop the WPS Registrar and also reset the selected registrar attribute*/
    sys_stop_wps_reg(UNSPEC_REASON, wps_ctxt_hdl->wps_config_hdl->mac_addr,
                     BFALSE);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_msg_timeout                            */
/*                                                                           */
/*  Description      : This function handles EAP Message timeout             */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If not in registration protocol then return        */
/*                     2) Update the retry len appropriately depending on the*/
/*                        frame the registrar has to retry                   */
/*                     3) Call wps_handle_eap_retry to handle retry of the   */
/*                        frames                                             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static void wps_handle_eap_msg_timeout(wps_t *wps_ctxt_hdl)
{
    wps_reg_t *reg_hdl = wps_ctxt_hdl->wps_reg_hdl;
    wps_eap_tx_t *eap_tx_hdl = wps_ctxt_hdl->wps_eap_tx_hdl;
    UWORD16 retry_len = 0;

    /* Registrar must be in Registration Protocol State */
    if(WPS_REG_PROT_ST != reg_hdl->wpsr_state)
    {
        reg_hdl->start_req = BFALSE;
        return;
    }

    /* Set the retry length depending on the frame to be retried */
    if(REG_EAP_ID_ST == reg_hdl->reg_prot_state)
    {
        retry_len = (WPS_EAPOL_1X_HDR_LEN + EAP_HEADER_LEN +
                     EAP_TYPE_FIELD_LEN);
    }
    else if((REG_INIT_ST == reg_hdl->reg_prot_state) ||
            (REG_MSG_ST  == reg_hdl->reg_prot_state))
    {
        retry_len = (eap_tx_hdl->tx_m_msg_len + EAP_WPS_DATA_WO_LEN_OFFSET);
    }

    /* Retry the frame */
    wps_handle_eap_retry(wps_ctxt_hdl, eap_tx_hdl, retry_len);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reassemble_wsc_ie_reg                                 */
/*                                                                           */
/*  Description      : This function gets the WSC-IE in the received         */
/*                     Probe/(Re)Association Response and Beacon frames      */
/*                                                                           */
/*  Inputs           : 1) Pointer to the received frame                      */
/*                     2) Length of the received frame                       */
/*                     3) Index to the start of the IEs                      */
/*                     4) Pointer to reassembly buffer                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function gets the WSC-IE in the received         */
/*                     Probe Response and Beacon frames. This                */
/*                     function does following processing:                   */
/*                      1) If an unexpected frame then return                */
/*                      2) If any of the input pointers are NULL then return */
/*                      3) Search for WSC-IE                                 */
/*                      4) Check for OUI and copy the WSC-IE data (excluding */
/*                         OUI) to reassembly buffer                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         07 07 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static void reassemble_wsc_ie_reg(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                                  UWORD8 *buffer)
{
    UWORD16 total_wsc_ie_len = 0;

    if((NULL == msa) || (NULL == buffer))
        return;

    /*************************************************************************/
    /* WPS Information Element                                               */
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |     Element ID     |      Length   |      OUI    |     Data        |*/
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |         1 Byte     |       1 Byte  |     4 Byte  |     1-251 Bytes |*/
    /* +--------------------+---------------+-------------+-----------------+*/
    /*************************************************************************/

    while(index < (rx_len - FCS_LEN))
    {
        UWORD8 ie_len = msa[index + 1];
        UWORD8 wsc_data_len = 0;

        if(msa[index] == WPS_IE_ID)
        {
            /* Check if OUI is correct */
            if((msa[index + 2] == WPS_IE_OUI_BYTE0) &&
               (msa[index + 3] == WPS_IE_OUI_BYTE1) &&
               (msa[index + 4] == WPS_IE_OUI_BYTE2) &&
               (msa[index + 5] == WPS_IE_OUI_BYTE3))
            {
                wsc_data_len = (ie_len - 4);
                total_wsc_ie_len += wsc_data_len;

                /* Copy the WSC-IE data excluding the OUI */
                memcpy(buffer, (msa + index + 6), wsc_data_len);
                buffer += wsc_data_len;
            }
        }
        index += (ie_len + 2);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : is_wsc_ie_reassm_req_reg                              */
/*                                                                           */
/*  Description      : This function checks if reassembly of WSC-IE is       */
/*                     required. If WSC-IE(s) is present then the total      */
/*                     length of all WSC-IEs is returned                     */
/*                                                                           */
/*  Inputs           : 1) Pointer to the received frame                      */
/*                     2) Length of the received frame                       */
/*                     3) Index to the start of the IEs                      */
/*                     4) Pointer to location where WSC-IE length is returned*/
/*                     5) Pointer to buffer pointer                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if reassembly of WSC-IE is       */
/*                     required. If WSC-IE(s) is present then the total      */
/*                     length of all WSC-IEs is returned                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         07 07 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static BOOL_T is_wsc_ie_reassm_req_reg(UWORD8 *msa, UWORD16 rx_len,
                                       UWORD16 index,
                                       UWORD16 *total_wsc_ie_len,
                                       UWORD8 **buffer)
{
    UWORD8 num_wsc_ies = 0;

    if(NULL == msa)
        return BFALSE;

    /*************************************************************************/
    /* WPS Information Element                                               */
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |     Element ID     |      Length   |      OUI    |     Data        |*/
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |         1 Byte     |       1 Byte  |     4 Byte  |     1-251 Bytes |*/
    /* +--------------------+---------------+-------------+-----------------+*/
    /*************************************************************************/

    while(index < (rx_len - FCS_LEN))
    {
        UWORD8 ie_len = msa[index + 1];

        if(msa[index] == WPS_IE_ID)
        {
            /* Check if OUI is correct */
            if((msa[index + 2] == WPS_IE_OUI_BYTE0) &&
               (msa[index + 3] == WPS_IE_OUI_BYTE1) &&
               (msa[index + 4] == WPS_IE_OUI_BYTE2) &&
               (msa[index + 5] == WPS_IE_OUI_BYTE3))
            {
                *total_wsc_ie_len += (ie_len - 4);
                num_wsc_ies++;

                if(1 == num_wsc_ies)
                    *buffer = (msa + index + 6);
            }
        }
        index += (ie_len + 2);
    }

    /* If more than one WSC-IE is found then return TRUE */
    if(1 < num_wsc_ies)
        return BTRUE;

    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_wps_cap_assoc_req                               */
/*                                                                           */
/*  Description      : This function handles processing of (re)association   */
/*                     request from a WPS capable STA                        */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the received frame                      */
/*                     3) Length of the received frame                       */
/*                     4) Frame Type                                         */
/*                     5) Pointer to indicate WPS capable STA                */
/*                     6) Pointer to indicate Version2.0 capable STA         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) Checks for presence of WSC-IE, if not present      */
/*                        return BFALSE                                      */
/*                     2) Parse the received WSC-IE                          */
/*                     3) If the Registrar is free then indicate STA is WPS  */
/*                        capable. Also set the version2 capable flag if     */
/*                        version2 subelement is present in WSC-IE and       */
/*                        return BTRUE                                       */
/*                     4) Else return BFALSE                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - If STA is WPS capable and can run RP         */
/*                     BFALSE - If STA is not WPS capable or cannot run RP   */
/*                              with STA                                     */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 07 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_wps_cap_assoc_req(wps_t *wps_ctxt_hdl, UWORD8 *msa,
                               UWORD16 rx_len, UWORD8 frm_type,
                               BOOL_T *wps_cap_sta, BOOL_T *wps_ver2_cap_sta)
{
    UWORD16 wsc_ie_len          = 0;
    UWORD16 ie_start_index      = WPS_MAC_HDR_LEN;
    BOOL_T retval               = BFALSE;
    wps_rx_attri_t rx_attr      = {0};
    wps_rx_attri_t *rx_attr_ptr = &rx_attr;
    UWORD8 *wsc_ie_buf          = NULL;
    UWORD8 *buffer              = NULL;

    *wps_ver2_cap_sta = BFALSE;
    *wps_cap_sta      = BFALSE;

    /* Process only Probe request and (Re)Association Request frame */
    if((ASSOC_REQ != frm_type) && (REASSOC_REQ != frm_type))
       return BFALSE;

    /* Get the IE start index depending on the frame type */
    if(ASSOC_REQ == frm_type)
    {
        /* Capability (2) and Listen Interval (2) */
        ie_start_index += (CAP_INFO_LEN + LISTEN_INT_LEN);
    }
    else if(REASSOC_REQ == frm_type)
    {
        /* Capability (2), Listen Interval (2) and Current AP Address (6) */
        ie_start_index += (CAP_INFO_LEN + LISTEN_INT_LEN + WPS_MAC_ADDR_LEN);
    }

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_reg(msa, rx_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_local_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_reg(msa, rx_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    /* If WSC-IE not present then exit */
    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
    {
        return BFALSE;
    }

    /* Parse for TLV type WSC attributes */
    if(0 == wps_parse_attri(wsc_ie_buf, &rx_attr_ptr, wsc_ie_len, 1, BTRUE))
    {
        if(NULL != buffer)
            wps_local_mem_free(buffer);

        return BFALSE;
    }

    /*************************************************************************/
    /* If the Registartion Protocol is not in progress then return BTRUE.    */
    /* Set the WPS capable flag of the STA to BTRUE and set the Version2.0   */
    /* flag to BTRUE if Version2 sub-element is included in the frame        */
    /*************************************************************************/
    if(BFALSE == is_wps_in_prog_reg(wps_ctxt_hdl))
    {
        retval = BTRUE;
        *wps_cap_sta = BTRUE;

        if(NULL != rx_attr_ptr->version2)
            *wps_ver2_cap_sta = BTRUE;
    }

    if(NULL != buffer)
        wps_local_mem_free(buffer);

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_set_ver2_cap_sta                                  */
/*                                                                           */
/*  Description      : This function sets the Version 2 capable flag with    */
/*                     specified input value                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Input value                                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function sets the Version 2 capable flag with    */
/*                     specified input value                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 07 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

void wps_set_ver2_cap_sta(wps_t *wps_ctxt_hdl, BOOL_T val)
{
    wps_ctxt_hdl->wps_reg_hdl->ver2_cap_sta = val;
}

#ifdef MAC_P2P
/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_get_config_method                                    */
/*                                                                           */
/*  Description   : This function gets the config method attribute in WSC IE */
/*                                                                           */
/*  Inputs        : 1) MAC start address of the incoming frame               */
/*                  2) Length of the received frame                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function extracts the config method present in the  */
/*                  WSC IE in the P2P public action frames                   */
/*                                                                           */
/*  Outputs       :                                                          */
/*  Returns       : UWORD16, config method                                   */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD16 wps_get_config_method(UWORD8 *msa, UWORD16 rx_len)
{
    UWORD16 wsc_ie_len             = 0;
    UWORD16 config_method          = 0;
    UWORD16 ie_start_index         = 0;
    wps_rx_attri_t parse_attr      = {0};
    wps_rx_attri_t *parse_attr_ptr = &parse_attr;
    UWORD8 *wsc_ie_buf = NULL;
    UWORD8 *buffer = NULL;

    /* get the IE start index */
    ie_start_index += WPS_MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF;

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_reg(msa, rx_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_local_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_reg(msa, rx_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    /* If WSC-IE not present then exit */
    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
        return 0;

    /* Call the funtion to parse WSC IE and check if one or more */
    /* attrubutes are found                                      */
    if(0 != wps_parse_attri(wsc_ie_buf, &parse_attr_ptr, wsc_ie_len, 1,
                            BTRUE))
    {
        config_method = GET_U16_BE((parse_attr_ptr->config_meth));
    }

    if(NULL != buffer)
        wps_local_mem_free(buffer);

    return config_method;
}




/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_p2p_prob_req_reg                          */
/*                                                                           */
/*  Description      : This funtion checks if there is a requested device    */
/*                     type attribute present in the WSC IE in the probe     */
/*                     request frame. If it is present then it checks if     */
/*                     its primary device type matches with that             */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to received frame                          */
/*                     3) Length of received association response            */
/*                     4) Pointer to store the requested device type present */
/*                        in probe request frame                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the probe request frame has   */
/*                     the WSC IE. It then parses the WSC IE to check if     */
/*                     requested device type attribute is present in it. If  */
/*                     requested device type attribute is present then it    */
/*                     checks if it matches with its primary device type     */
/*                     if it doesnot match then it returns BFALSE else for   */
/*                     other cases it returns BTRUE                          */
/*                                                                           */
/*  Outputs          :                                                       */
/*  Returns          : Direct   : BOOL_T; BTRUE                              */
/*                                BFALSE                                     */
/*                     Indirect : UWORD8 *; Requested device type            */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
BOOL_T wps_process_p2p_prob_req_reg(wps_config_t *config_ptr,UWORD8 *msa,
                                    UWORD16 rx_len, UWORD8 *req_dev_type)
{
    UWORD16 wsc_ie_len             = 0;
    UWORD16 ie_start_index         = 0;
    wps_rx_attri_t parse_attr      = {0};
    wps_rx_attri_t *parse_attr_ptr = &parse_attr;
    UWORD8 *wsc_ie_buf             = NULL;
    UWORD8 *buffer                 = NULL;
    BOOL_T retval                  = BTRUE;

    /* Get the start of the IE */
    /* MAC Header(24) */
    ie_start_index =  WPS_MAC_HDR_LEN;

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_reg(msa, rx_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_local_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_reg(msa, rx_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    /* If WSC-IE not present then exit */
    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
        return retval;

    /* Call the funtion to parse WSC IE and check if one or more         */
    /* attrubutes are found                                              */
    if(0 != wps_parse_attri(wsc_ie_buf, &parse_attr_ptr,
                            wsc_ie_len, 1, BTRUE))
    {
        /* Check if the WSC IE has requested device type attribute */
        if(0 != parse_attr_ptr->req_dev_type)
        {

            /* Copy the requested device type */
            memcpy(req_dev_type, parse_attr_ptr->req_dev_type,
                   WPS_REQ_DEV_TYPE_LEN);

            /* Check if the primary device type matches with the         */
            /* requested device type. If it doesnot match then return    */
            /* BFALSE                                                    */
            /* Note: We match only the Category ID of the primary device */
            /* type i.e the first 2 bytes of the attribute               */
            if(0 != memcmp(config_ptr->prim_dev_cat_id,
                           req_dev_type, 2))
            {
                retval = BFALSE;
            }
        }
    }

    /* Free the allocated buffer */
    if(NULL != buffer)
        wps_local_mem_free(buffer);

    return retval;

}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_add_config_method_ie                                 */
/*                                                                           */
/*  Description   : This function adds the WSC IE with config method         */
/*                                                                           */
/*  Inputs        : 1) Frame pointer                                         */
/*                  2) Index to the start of WSC IE                          */
/*                  3) Config Method to added in the WSC IE                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the WSC IE with config method         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD8, Total length of the WSC IE                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 wps_add_config_method_ie(UWORD8 *frame_ptr, UWORD8 index,
                                     UWORD16 config_meth)
{
    UWORD8 start_index = index;

    /* Set the Element ID field */
    frame_ptr[index++] = WPS_IE_ID;

    /* Length field is updated later */
    index++;

    /* Update OUI-Field */
    frame_ptr[index++] = WPS_IE_OUI_BYTE0;
    frame_ptr[index++] = WPS_IE_OUI_BYTE1;
    frame_ptr[index++] = WPS_IE_OUI_BYTE2;
    frame_ptr[index++] = WPS_IE_OUI_BYTE3;

    index += put_wsc_config_meth(frame_ptr + index, config_meth);

    /* Update the Length Field */
    frame_ptr[start_index + 1] = (index - start_index -  2);

    return (index - start_index);

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_add_go_neg_frm_wsc_ie                                */
/*                                                                           */
/*  Description   : This function adds the WSC IE to the frame               */
/*                                                                           */
/*  Inputs        : 1) Frame pointer                                         */
/*                  2) Index to the start of WSC IE                          */
/*                  3) WPS Protocol Type (PIN/PBC)                           */
/*                  4) Device Password ID                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds the WSC IE to the frame               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : UWORD8, Total length of the WSC IE                       */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 wps_add_go_neg_frm_wsc_ie(UWORD8 *frame_ptr, UWORD16 index,
                                 WPS_PROT_TYPE_T prot_type, UWORD16 dev_pas_id)
{
    UWORD8 start_index = index;

    /* Set the Element ID field */
    frame_ptr[index++] = WPS_IE_ID;

    /* Length field is updated later */
    index++;

    /* Update OUI-Field */
    frame_ptr[index++] = WPS_IE_OUI_BYTE0;
    frame_ptr[index++] = WPS_IE_OUI_BYTE1;
    frame_ptr[index++] = WPS_IE_OUI_BYTE2;
    frame_ptr[index++] = WPS_IE_OUI_BYTE3;

    index += put_wsc_version(frame_ptr + index);

    if(PBC == prot_type)
    {
        index += put_wsc_dev_pass_id((frame_ptr + index),
            (UWORD16) DEV_PASS_ID_PUSHBUTT);
    }
    else
    {
        index += put_wsc_dev_pass_id((frame_ptr + index), dev_pas_id);
    }

    /* Update the Length Field */
    frame_ptr[start_index + 1] = (index - start_index -  2);

    return (index - start_index);

}
#endif /* MAC_P2P */
#endif /* INT_WPS_REG_SUPP */
