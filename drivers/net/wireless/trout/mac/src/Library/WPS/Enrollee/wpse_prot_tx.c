/*****************************************************************************/
/*                                                                           */
/*                     Ittiam WPS Supplicant SOFTWARE                        */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wpse_prot_tx.c                                       */
/*                                                                           */
/*  Description       : This file contains the transmit functions related to */
/*                      the WPS protocol                                     */
/*                                                                           */
/*  List of Functions : wps_gen_secr_keys                                    */
/*                      wps_gen_wsc_ie                                       */
/*                      wps_send_eapol_start                                 */
/*                      wps_send_eap_res_id                                  */
/*                      wps_tx_eap_msg                                       */
/*                      wps_send_wsc_ack                                     */
/*                      wps_send_nack                                        */
/*                      wps_send_nack_m2                                     */
/*                      wps_send_m1                                          */
/*                      wps_send_m3                                          */
/*                      wps_send_m5_or_7                                     */
/*                      wps_send_frag_ack                                    */
/*                      wps_send_done                                        */
/*                      wps_put_key_wrap_auth                                */
/*                      wps_put_auth                                         */
/*                      wps_gen_e_hash                                       */
/*                      wps_aes_encrypt                                      */
/*                      wps_aes_encrypt                                      */
/*                      wps_start_reg_msg_timer                              */
/*                      wps_eap_reg_msg_timeout                              */
/*                      wps_eap_reg_timeout                                  */
/*                      wps_add_discarded_reg_ap                             */
/*                      wps_get_rand_byte_array                              */
/*                      wps_handle_event                                     */
/*                      wps_prep_1x_eap_res_header                           */
/*                      wps_prep_1x_header                                   */
/*                      wps_prep_eap_res_header                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         03 01 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_ENR_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"
#include "wpse_prot_tx.h"
#include "wpse_hmac_sha256.h"
#include "wpse_key.h"
#include "wpse_aes.h"
#include "trout_trace.h"
/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static void wps_prep_1x_eap_res_header(UWORD8* buffer, UWORD16 tx_len,
                                UWORD8 eap_received_id, UWORD8 op_code);
static void wps_prep_1x_header(UWORD8* buffer, UWORD8 one_x_type,
                               UWORD16 tx_len);
static void wps_prep_eap_res_header(UWORD8* buf_ptr, UWORD16 tx_len,
                                    UWORD8 eap_id, UWORD8 op_code);
static UWORD8 wps_gen_e_hash(wps_enrollee_t* wps_enrollee, UWORD8* buf_ptr);

static UWORD8 wps_put_auth(wps_enrollee_t* wps_enrollee, UWORD8* msg_ptr,
                           UWORD16 msg_len);
static UWORD8 wps_put_key_wrap_auth(wps_enrollee_t* wps_enrollee,
                                    UWORD8* msg_ptr, UWORD16 msg_len);

#ifdef MAC_P2P
static UWORD16 wps_add_wsc_ie_p2p_prob_rsp(wps_enrollee_t* wps_enrollee,
                                           UWORD8* buff_ptr);
#endif /* MAC_P2P */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_secr_keys                                     */
/*                                                                           */
/*  Description      : This function generates the secret Keys AuthKey and   */
/*                     KeyWrapKey                                            */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to received PKR                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates the secret Keys AuthKey and   */
/*                     KeyWrapKey using the received PKR and the private key */
/*                     A                                                     */
/*                                                                           */
/*  Outputs          : Auth Key and Key Wrap Key                             */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         15 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_gen_secr_keys(wps_enrollee_t* wps_enrollee, UWORD8* pkr_ptr)
{
    UWORD8 cnt;
    UWORD32* temp32_ptr1;

    temp32_ptr1 = wps_enrollee->pub_key.pkr_u32 + WPS_DH_PRIME_LEN_WORD32 -1;
    for(cnt =0; cnt < WPS_DH_PRIME_LEN_WORD32; cnt++)
    {
        *temp32_ptr1-- = ((UWORD32)((UWORD32) pkr_ptr[0]) << 24) |
            ((UWORD32)((UWORD32) pkr_ptr[1]) << 16) |
            ((UWORD32)((UWORD32) pkr_ptr[2]) << 8) |
            ((UWORD32)((UWORD32) pkr_ptr[3]));
        pkr_ptr += 4;
    }
    pkr_ptr -= WPS_DH_PRIME_LEN_WORD32*4;
    if(FAILURE == wps_gen_keys(wps_enrollee))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    /*************************************************************************/
    /* Copy PKR from pkr_ptr to wps_enrollee->pkr, with most significant byte*/
    /* at the first location. Source is already in desired format.           */
    /*************************************************************************/
    memcpy(wps_enrollee->pub_key.pkr, pkr_ptr, WPS_DH_PRIME_LEN_BYTE);
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_wsc_ie                                        */
/*                                                                           */
/*  Description      : This function generates WSC_IE which can be used in   */
/*                     probe request and (re)association requests            */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Buffer pointer where WSC_IE should be stored       */
/*                     3) Frame Type                                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates WSC_IE which can be used in   */
/*                     probe request and (re)association requests. Only      */
/*                     parameters which are mandatory in probe request WSC_IE*/
/*                     are framed. Same WSC_IE can be used for (re)associati-*/
/*                     -on request                                           */
/*                                                                           */
/*  Outputs          : WSC_IE                                                */
/*  Returns          : Length of WSC_IE generated                            */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         19 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
UWORD16 wps_gen_wsc_ie(wps_enrollee_t* wps_enrollee, UWORD8* buff_ptr,
                       UWORD8 frm_type)
{
    UWORD8* loc_buf_ptr = buff_ptr;
    UWORD8  length;
    wps_enr_config_struct_t* config_ptr = wps_enrollee->config_ptr;
    wps_sub_elem_t *subelem_ptr = &(config_ptr->version2);

    /* WSC-IE is added only to probe request and (re)association request */
    if((PROBE_REQ != frm_type) && (ASSOC_REQ != frm_type) &&
       (REASSOC_REQ != frm_type))
    {
        return 0;
    }

    /*************************************************************************/
    /* Prepare WSC_IE. Only mandatory attributes are framed                  */
    /*************************************************************************/
    /* WPS Information Element                                               */
    /*************************************************************************/
    /* ----------------------------------------------------------------------*/
    /* |     Element ID     |      Length   |      OUI    |     Data        |*/
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |         1 Byte     |       1 Byte  |     4 Byte  |     1-251 Bytes |*/
    /* ----------------------------------------------------------------------*/
    /*************************************************************************/
    *loc_buf_ptr = WPS_IE_ID;
    loc_buf_ptr += 2;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE0;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE1;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE2;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE3;
    loc_buf_ptr = wps_put_wsc_version(loc_buf_ptr);
    loc_buf_ptr = wps_put_req_type(loc_buf_ptr, wps_enrollee->req_type);

    if(PROBE_REQ == frm_type)
    {
        loc_buf_ptr = wps_put_cofig_meth(loc_buf_ptr, config_ptr->config_meth);
        loc_buf_ptr = wps_put_wsc_uuid_e(loc_buf_ptr, wps_enrollee->enr_uuid);
        loc_buf_ptr = wps_put_prim_dev_type(loc_buf_ptr, config_ptr);
        loc_buf_ptr = wps_put_rf_bands(loc_buf_ptr, config_ptr->rf_bands);
        loc_buf_ptr = wps_put_assoc_state(loc_buf_ptr,
                                          wps_enrollee->assoc_state);

        wps_enrollee->config_error = WPS_NO_ERROR;
        loc_buf_ptr = wps_put_config_err(loc_buf_ptr,
                                         wps_enrollee->config_error);

        if(PBC == config_ptr->prot_type)
        {
            loc_buf_ptr = wps_put_dev_pass_id(loc_buf_ptr,
                (UWORD16) DEV_PASS_ID_PUSHBUTT);
        }
        else
        {
            loc_buf_ptr = wps_put_dev_pass_id(loc_buf_ptr,
                                              config_ptr->dev_pass_id);
        }

#if 0 /* Not part of WSC 2.0 specification */
        loc_buf_ptr = wps_put_auth_type_flag(loc_buf_ptr,
                                             config_ptr->auth_type_flags);
        loc_buf_ptr = wps_put_encr_type_flag(loc_buf_ptr,
                                             config_ptr->enc_type_flags);
        loc_buf_ptr = wps_put_cofig_meth(loc_buf_ptr,
                                         config_ptr->config_meth);
#endif /* 0 */
        loc_buf_ptr = wps_put_manufacturer(loc_buf_ptr, config_ptr);
        loc_buf_ptr = wps_put_model_name(loc_buf_ptr, config_ptr);
        loc_buf_ptr = wps_put_model_num(loc_buf_ptr, config_ptr);
        loc_buf_ptr = wps_put_device_name(loc_buf_ptr, config_ptr);
#ifdef MAC_P2P
        loc_buf_ptr = wps_put_req_dev_type(loc_buf_ptr, config_ptr);
#endif /* MAC_P2P */
    }

    loc_buf_ptr = wps_put_wfa_vendor(loc_buf_ptr, &subelem_ptr, 1);

    /*************************************************************************/
    /* Calculate length and add the length to the Length field               */
    /*************************************************************************/
    length = (UWORD8)(loc_buf_ptr - buff_ptr);
    buff_ptr[1] = length - 2;

    return((UWORD16)(length));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_eapol_start                                  */
/*                                                                           */
/*  Description      : This function starts the registration protocol by     */
/*                     sending EAPOL-Start message to the AP/Registrar       */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function allocates a frame buffer and put EAPOL  */
/*                     -Start frame fields in it. After the framing, message */
/*                     is sent to WLAN                                       */
/*                                                                           */
/*  Outputs          : EAPOL-Start frame to peer                             */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         15 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_eapol_start(wps_enrollee_t* wps_enrollee)
{
    UWORD8 *buffer;
    /*************************************************************************/
    /* Fetch memory for the EAPOL-Start Packet to be sent to the Remote Sta  */
    /*************************************************************************/
//    buffer = (UWORD8*) wps_frame_mem_alloc(WPS_EAPOL_START_PCK_LEN);
    buffer = (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);
    if(buffer == NULL)
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    memset(buffer, 0, MAX_WPS_WLAN_FRAME_LEN);

    /*************************************************************************/
    /* Prepare the frame 802.1x header. Length field is zero.                */
    /*************************************************************************/
    wps_prep_1x_header(buffer, ONE_X_PCK_TYPE_EAPOL_START, 0);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    send_eapol(wps_enrollee->sel_ap_info.bssid, buffer, WPS_EAPOL_1X_HDR_LEN,
        BFALSE);
    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_eap_res_id                                   */
/*                                                                           */
/*  Description      : This function prepares and transmits an EAP response/ */
/*                     identity frame with EAP Identity as                   */
/*                     "WFA-SimpleConfig-Enrollee-1-0"                       */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function allocates a frame buffer and put EAP    */
/*                     Response identity frame fields in it. After the       */
/*                     framing, message is sent to WLAN                      */
/*                                                                           */
/*  Outputs          : WPS EAP Response Identity frame to peer               */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         15 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_eap_res_id(wps_enrollee_t* wps_enrollee)
{
    const UWORD8 wps_eap_identity[WPS_EAP_RED_ID_LEN+1] =
        "WFA-SimpleConfig-Enrollee-1-0";
    UWORD8 *buffer, *buf_ptr;
    /*************************************************************************/
    /* Fetch memory for the EAPOL-Response Identity Packet                   */
    /*************************************************************************/
//    buffer = (UWORD8*) wps_frame_mem_alloc(WPS_EAP_RES_ID_MSG_LEN);
    buffer = (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);
    if(buffer == NULL)
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    memset(buffer, 0, MAX_WPS_WLAN_FRAME_LEN);

    /*************************************************************************/
    /* Prepare the frame 802.1x header.                                      */
    /*************************************************************************/
    wps_prep_1x_header(buffer, ONE_X_PCK_TYPE_EAP, WPS_EAP_RED_ID_LEN +5);
    /*************************************************************************/
    /* The EAP Buffer is offseted with mac and 1x header length              */
    /*************************************************************************/
    buf_ptr = buffer + EAP_HEADER_OFFSET+WPS_MAX_MAC_HDR_LEN;

    /*************************************************************************/
    /* EAP Packet                                                            */
    /*  Code          |   Id     |    Length      |           Data           */
    /*  1 Byte        | 1 Byte   |    2 bytes     |         Length - 4 Bytes */
    /*************************************************************************/
    *buf_ptr++ = EAP_CODE_RESPONSE;
    *buf_ptr++ = wps_enrollee->eap_received_id;
    PUT_U16_BE(buf_ptr, (WPS_EAP_RED_ID_LEN + 5));
    buf_ptr += 2;
    *buf_ptr++ = EAP_TYPE_IDENTITY;
    memcpy(buf_ptr, wps_eap_identity, sizeof(wps_eap_identity)-1);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    send_eapol(wps_enrollee->sel_ap_info.bssid, buffer, (WPS_EAPOL_1X_HDR_LEN
        + 0x05 + sizeof(wps_eap_identity) - 1), BFALSE);
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_tx_eap_msg                                        */
/*                                                                           */
/*  Description      : This function prepares a transmit EAP frame from the  */
/*                     frame stored at wps_enrollee->tx_eap_msg and then     */
/*                     sends it to WLAN Device.                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares a transmit EAP frame from the  */
/*                     frame stored at wps_enrollee->tx_eap_msg and then     */
/*                     sends it to WLAN Device.                              */
/*                                                                           */
/*  Outputs          : WSC EAP frame to peer                                 */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         15 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_tx_eap_msg(wps_enrollee_t* wps_enrollee)
{
    UWORD16 tx_len;
    UWORD8  *buffer;
    tx_len = GET_U16_BE(
        (wps_enrollee->tx_eap_msg +EAP_LENGTH_OFFSET-EAP_HEADER_OFFSET));
    /*************************************************************************/
    /* Fetch memory for the EAPOL-Packet to be sent to the Remote STA        */
    /*************************************************************************/
    if(NULL != wps_enrollee->tx_wlan_frm_buff)
    {
        buffer = wps_enrollee->tx_wlan_frm_buff;
    }
    else
    {
        buffer = (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);
        if(NULL == buffer)
        {
            wps_handle_sys_err(wps_enrollee);
            return FAILURE;
        }
    }
    /*************************************************************************/
    /* Prepare the frame 802.1x header.                                      */
    /*************************************************************************/
    wps_prep_1x_header(buffer, ONE_X_PCK_TYPE_EAP, tx_len);
    /*************************************************************************/
    /* The EAP Buffer is offseted with mac and 1x header length              */
    /* Copy EAP frame to the body of the EAPOL frame                         */
    /*************************************************************************/
    memcpy(buffer + EAP_HEADER_OFFSET+WPS_MAX_MAC_HDR_LEN,
        wps_enrollee->tx_eap_msg, tx_len);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    if(BFALSE == send_eapol(wps_enrollee->sel_ap_info.bssid, buffer,
        (UWORD16)(WPS_EAPOL_1X_HDR_LEN + tx_len), BFALSE))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    wps_enrollee->tx_wlan_frm_buff = NULL;
    wps_enrollee->reg_state = MSG_ST;
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_wsc_ack                                      */
/*                                                                           */
/*  Description      : This function prepares and transmits the WSC-EAP      */
/*                     message of type WSC_ACK                               */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares the WSC-EAP message of type    */
/*                     WSC_ACK as described in standard and transmits the    */
/*                     frame. Only mandatory attributes are framed. Fixed    */
/*                     frame length is used hence verify the length if more  */
/*                     attributes are added.                                 */
/*                                                                           */
/*  Outputs          : WSC_ACK frame to peer                                 */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_wsc_ack(wps_enrollee_t* wps_enrollee)
{
    UWORD16 tx_len;
    UWORD8  *buffer, *buf_ptr;
    wps_sub_elem_t *subelem_ptr = &(wps_enrollee->config_ptr->version2);
    /*************************************************************************/
    /* Fetch memory for the WSC_ACK-Packet to be sent to the Remote STA      */
    /*************************************************************************/
//    buffer = (UWORD8*) wps_frame_mem_alloc(WPS_EAP_WSC_ACK_MSG_LEN);
    buffer = (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);
    if(buffer == NULL)
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    memset(buffer, 0, MAX_WPS_WLAN_FRAME_LEN);
    /*************************************************************************/
    /* Prepare WSC_ACK message. Only mandatory attributes (Version, Message  */
    /* Type, Enrollee Nonce and Registrar Nonce  are framed                  */
    /*************************************************************************/
    buf_ptr = buffer + WPS_MAX_MAC_HDR_LEN + EAP_WPS_DATA_WO_LEN_OFFSET;
    buf_ptr = wps_put_wsc_version(buf_ptr);
    buf_ptr = wps_put_wsc_msg_type(buf_ptr, (UWORD8)WPS_MSG_WSC_ACK);
    buf_ptr = wps_put_enr_nonce(buf_ptr, wps_enrollee->enr_nonce);

    /* As a workaround for Ralink AP, send WSC-ACK with zero registrar nonce */
    /* so that the AP terminates registration protocol cleanly with an       */
    /* EAP-Failure.                                                          */
    {
        UWORD8 zero_buffer[WPS_NONCE_LEN] = {0};

        buf_ptr = wps_put_reg_nonce(buf_ptr, zero_buffer);
    }
    buf_ptr = wps_put_wfa_vendor(buf_ptr, &subelem_ptr, 1);
    tx_len  = buf_ptr - (buffer + WPS_MAX_MAC_HDR_LEN + EAP_HEADER_OFFSET);
    /*************************************************************************/
    /* Prepare the frame 802.1x * EAP headers.                               */
    /*************************************************************************/
    wps_prep_1x_eap_res_header(buffer, tx_len,wps_enrollee->eap_received_id,
                               WPS_OP_CODE_WSC_ACK);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    send_eapol(wps_enrollee->sel_ap_info.bssid, buffer,
               (UWORD16)(WPS_EAPOL_1X_HDR_LEN + tx_len), BFALSE);
    wps_enrollee->reg_state = MSG_ST;
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_nack                                         */
/*                                                                           */
/*  Description      : This function prepares and transmits the WSC-EAP      */
/*                     message of type WSC_NACK                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares the WSC-EAP message of type    */
/*                     WSC_NACK as described in standard and stores the      */
/*                     message at wps_enrollee->tx_eap_msg. Then it transmits*/
/*                     the frame. Only mandatory attributes are framed.      */
/*                                                                           */
/*  Outputs          : WSC_NACK frame to peer                                */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_nack(wps_enrollee_t* wps_enrollee)
{
    UWORD16 tx_len;
    UWORD8  *buf_ptr;
    wps_sub_elem_t *subelem_ptr = &(wps_enrollee->config_ptr->version2);

    /*************************************************************************/
    /* Fetch memory from previously stored buffer                            */
    /*************************************************************************/
    buf_ptr = wps_enrollee->tx_m_msg_ptr;
    if((NULL == wps_enrollee->tx_eap_msg)||(NULL == buf_ptr))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    /*************************************************************************/
    /* Prepare WSC_ACK message. Only mandatory attributes (Version, Message  */
    /* Type, Enrollee Nonce, Registrar Nonce, Configuration Error are framed */
    /*************************************************************************/
    buf_ptr = wps_put_wsc_version(buf_ptr);
    buf_ptr = wps_put_wsc_msg_type(buf_ptr, (UWORD8)WPS_MSG_WSC_NACK);
    buf_ptr = wps_put_enr_nonce(buf_ptr, wps_enrollee->enr_nonce);
    buf_ptr = wps_put_reg_nonce(buf_ptr, wps_enrollee->reg_nonce);
    if(WPS_NO_ERROR == wps_enrollee->config_error)
        wps_enrollee->config_error = DEVICE_BUSY;
    buf_ptr = wps_put_config_err(buf_ptr, wps_enrollee->config_error);
    buf_ptr = wps_put_wfa_vendor(buf_ptr, &subelem_ptr, 1);
    tx_len  = buf_ptr - wps_enrollee->tx_eap_msg;
    wps_enrollee->tx_m_msg_len = tx_len -
        (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);
    /*************************************************************************/
    /* Prepare the WPS EAP frame header.                                     */
    /*************************************************************************/
    wps_prep_eap_res_header(wps_enrollee->tx_eap_msg, tx_len,
        wps_enrollee->eap_received_id, WPS_OP_CODE_WSC_NACK);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    wps_enrollee->tx_eap_txed_msg_len =0;
    return (wps_tx_eap_msg(wps_enrollee));
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_nack_m2                                      */
/*                                                                           */
/*  Description      : This function prepares and transmits the WSC-EAP      */
/*                     message of type WSC_NACK for wrong M2 message received*/
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the structure where parsed attributes   */
/*                        are stored                                         */
/*                     3) Configuration Error                                */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares the WSC-EAP message of type    */
/*                     WSC_NACK as described in standard and transmits the   */
/*                     frame. Only mandatory attributes are framed. This     */
/*                     message is not stored in local buffer hence no        */
/*                     re-transmission happens.Fixed frame length is used    */
/*                     hence verify the length if more attributes are added. */
/*                                                                           */
/*  Outputs          : WSC_NACK frame to peer                                */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_nack_m2(wps_enrollee_t* wps_enrollee,
                          wps_rx_attri_t* rx_attr,
                          UWORD16 config_error)
{
    UWORD16 tx_len;
    UWORD8  *buffer, *buf_ptr;
    /*************************************************************************/
    /* Fetch memory for the WSC_NACK-Packet to be sent to the Remote STA     */
    /*************************************************************************/
//    buffer = (UWORD8*) wps_frame_mem_alloc(WPS_EAP_WSC_NACK_MSG_LEN);
    buffer = (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);
    if(buffer == NULL)
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    memset(buffer, 0, MAX_WPS_WLAN_FRAME_LEN);
    /*************************************************************************/
    /* Prepare WSC_ACK message. Only mandatory attributes (Version, Message  */
    /* Type, Enrollee Nonce, Registrar Nonce, Configuration Error are framed */
    /*************************************************************************/
    buf_ptr = buffer + WPS_MAX_MAC_HDR_LEN + EAP_WPS_DATA_WI_LEN_OFFSET;
    buf_ptr = wps_put_wsc_version(buf_ptr);
    buf_ptr = wps_put_wsc_msg_type(buf_ptr, (UWORD8)WPS_MSG_WSC_ACK);
    buf_ptr = wps_put_enr_nonce(buf_ptr, wps_enrollee->enr_nonce);
    buf_ptr = wps_put_reg_nonce(buf_ptr, rx_attr->reg_nonce);
    buf_ptr = wps_put_config_err(buf_ptr, wps_enrollee->config_error);
    tx_len  = buf_ptr - (buffer + EAP_HEADER_OFFSET + WPS_MAX_MAC_HDR_LEN);
    /*************************************************************************/
    /* Prepare the frame 802.1x * EAP headers.                               */
    /*************************************************************************/
    wps_prep_1x_eap_res_header(buffer, tx_len,wps_enrollee->eap_received_id,
        WPS_OP_CODE_WSC_ACK);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    send_eapol(wps_enrollee->sel_ap_info.bssid, buffer,
        (UWORD16) (WPS_EAPOL_1X_HDR_LEN + tx_len), BFALSE);
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m1                                           */
/*                                                                           */
/*  Description      : This function prepares and transmits the EAP message  */
/*                     M1 as described in [WPSSPEC]                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares the WSC-EAP message M1 as      */
/*                     described in standard and stores the message at       */
/*                     wps_enrollee->tx_eap_msg. As M1 is the first frame,   */
/*                     memory allocation for local copy of message to be     */
/*                     transmitted is done here. This function also generates*/
/*                     public and private DH-keys.                           */
/*                                                                           */
/*  Outputs          : M1 message to peer                                    */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_m1(wps_enrollee_t* wps_enrollee)
{
    UWORD16 tx_len;
    UWORD8  *buf_ptr;
    wps_sub_elem_t req_enrol = {0};
    wps_sub_elem_t *subelem_ptr[2];

    /*************************************************************************/
    /* Fetch memory from previously stored buffer                            */
    /*************************************************************************/
    buf_ptr = wps_enrollee->tx_eap_msg;
    /*************************************************************************/
    /* If buffer not present then allocate a new buffer                      */
    /*************************************************************************/
    if(NULL == buf_ptr)
    {
        buf_ptr = (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WSC_EAP_MSG_LEN);
        if(buf_ptr == NULL)
        {
            wps_handle_sys_err(wps_enrollee);
            return FAILURE;
        }
        wps_enrollee->tx_eap_msg = buf_ptr;
        wps_enrollee->tx_m_msg_ptr = buf_ptr + (EAP_WPS_DATA_WO_LEN_OFFSET
            - EAP_HEADER_OFFSET);
        wps_enrollee->tx_m_msg_len =0;
    }
    if(NULL == wps_enrollee->tx_wlan_frm_buff)
    {
        /*********************************************************************/
        /* Fetch memory for the EAPOL-Packet to be sent to the Remote STA    */
        /*********************************************************************/
        wps_enrollee->tx_wlan_frm_buff =
            (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);
        if(NULL == wps_enrollee->tx_wlan_frm_buff)
        {
            wps_handle_sys_err(wps_enrollee);
            return FAILURE;
        }
    }

#ifndef RALINK_AP_WCN_EXT_REG_BUG_FIX
    wps_get_rand_byte_array(wps_enrollee->enr_nonce, WPS_NONCE_LEN);
#endif /* RALINK_AP_WCN_EXT_REG_BUG_FIX */

    wps_enrollee->config_error = WPS_NO_ERROR;

#ifndef OPTIMIZED_PKE_GENERATION
    if(FAILURE ==
       wps_gen_pke((UWORD32 *)wps_enrollee->pke, &(wps_enrollee->a_nonce)))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
#endif /* OPTIMIZED_PKE_GENERATION */
    /*************************************************************************/
    /* Prepare WSC_MSG message M1. Only mandatory attributes are framed      */
    /*************************************************************************/
    buf_ptr = wps_enrollee->tx_m_msg_ptr;
    buf_ptr = wps_put_wsc_version(buf_ptr);
    buf_ptr = wps_put_wsc_msg_type(buf_ptr, (UWORD8)WPS_MSG_M1);
    buf_ptr = wps_put_wsc_uuid_e(buf_ptr, wps_enrollee->enr_uuid);
    buf_ptr = wps_put_wsc_mac_addr(buf_ptr,
        wps_enrollee->config_ptr->mac_address);
    buf_ptr = wps_put_enr_nonce(buf_ptr, wps_enrollee->enr_nonce);
    buf_ptr = wps_put_public_key(buf_ptr, wps_enrollee->pke);
    buf_ptr = wps_put_auth_type_flag(buf_ptr,
        wps_enrollee->config_ptr->auth_type_flags);
    buf_ptr = wps_put_encr_type_flag(buf_ptr,
        wps_enrollee->config_ptr->enc_type_flags);
    buf_ptr = wps_put_conn_type_flag(buf_ptr,
        wps_enrollee->config_ptr->con_type_flag);
    buf_ptr = wps_put_cofig_meth(buf_ptr,
        wps_enrollee->config_ptr->config_meth);
    buf_ptr = wps_put_wps_state(buf_ptr,wps_enrollee->config_ptr->wps_state);
    buf_ptr = wps_put_manufacturer(buf_ptr,wps_enrollee->config_ptr);
    buf_ptr = wps_put_model_name(buf_ptr,wps_enrollee->config_ptr);
    buf_ptr = wps_put_model_num(buf_ptr,wps_enrollee->config_ptr);
    buf_ptr = wps_put_serial_num(buf_ptr,wps_enrollee->config_ptr);
    buf_ptr = wps_put_prim_dev_type(buf_ptr, wps_enrollee->config_ptr);
    buf_ptr = wps_put_device_name(buf_ptr, wps_enrollee->config_ptr);
    buf_ptr = wps_put_rf_bands(buf_ptr, wps_enrollee->config_ptr->rf_bands);
    buf_ptr = wps_put_assoc_state(buf_ptr, wps_enrollee->assoc_state);

    if(PBC == wps_enrollee->config_ptr->prot_type)
    {
        buf_ptr = wps_put_dev_pass_id(buf_ptr,
            (UWORD16) DEV_PASS_ID_PUSHBUTT);
    }
    else
    {
        buf_ptr = wps_put_dev_pass_id(buf_ptr,
            wps_enrollee->config_ptr->dev_pass_id);
    }

    buf_ptr = wps_put_config_err(buf_ptr, wps_enrollee->config_error);
    buf_ptr = wps_put_os_ver(buf_ptr,
                             wps_enrollee->config_ptr->os_version);
    /* Indidicate the desire to Enroll */
    req_enrol.id  = WPS_SUB_ELEM_REQ_2_ENROLL;
    req_enrol.len = 1;
    req_enrol.val[0] = 1;
    subelem_ptr[0] = &(wps_enrollee->config_ptr->version2);
    subelem_ptr[1] = &req_enrol;

    buf_ptr = wps_put_wfa_vendor(buf_ptr, subelem_ptr, 2);

    tx_len  = buf_ptr - wps_enrollee->tx_eap_msg;
    wps_enrollee->tx_m_msg_len = tx_len -
        (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);
    /*************************************************************************/
    /* Prepare the WPS EAP frame header.                                     */
    /*************************************************************************/
    wps_prep_eap_res_header(wps_enrollee->tx_eap_msg, tx_len,
        wps_enrollee->eap_received_id, WPS_OP_CODE_WSC_MSG);
    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    wps_enrollee->tx_eap_txed_msg_len =0;
    return (wps_tx_eap_msg(wps_enrollee));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m3                                           */
/*                                                                           */
/*  Description      : This function prepares and transmits the EAP message  */
/*                     M3 as described in [WPSSPEC]                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares the WSC-EAP message M3 as      */
/*                     described in standard and stores the message at       */
/*                     wps_enrollee->tx_eap_msg.                             */
/*                                                                           */
/*  Outputs          : M3 message to peer                                    */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_m3(wps_enrollee_t* wps_enrollee)
{
    UWORD16 tx_len;
    UWORD8  *buf_ptr;
    wps_sub_elem_t *subelem_ptr = &(wps_enrollee->config_ptr->version2);

    /*************************************************************************/
    /* Fetch memory from previously stored buffer                            */
    /*************************************************************************/
    buf_ptr = wps_enrollee->tx_m_msg_ptr;
    if((NULL == wps_enrollee->tx_eap_msg)||(NULL == buf_ptr))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    buf_ptr = wps_put_wsc_version(buf_ptr);
    buf_ptr = wps_put_wsc_msg_type(buf_ptr, (UWORD8)WPS_MSG_M3);
    buf_ptr = wps_put_reg_nonce(buf_ptr, wps_enrollee->reg_nonce);
    buf_ptr += wps_gen_e_hash(wps_enrollee, buf_ptr);
    buf_ptr = wps_put_wfa_vendor(buf_ptr, &subelem_ptr, 1);
    buf_ptr += wps_put_auth(wps_enrollee, wps_enrollee->tx_m_msg_ptr,
        (UWORD16) (buf_ptr - wps_enrollee->tx_m_msg_ptr));
    tx_len  = buf_ptr - wps_enrollee->tx_eap_msg;
    wps_enrollee->tx_m_msg_len = tx_len -
        (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);
    /*************************************************************************/
    /* Prepare the WPS EAP frame header.                                     */
    /*************************************************************************/
    wps_prep_eap_res_header(wps_enrollee->tx_eap_msg, tx_len,
        wps_enrollee->eap_received_id, WPS_OP_CODE_WSC_MSG);
    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    wps_enrollee->tx_eap_txed_msg_len =0;
    return (wps_tx_eap_msg(wps_enrollee));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m5_or_7                                      */
/*                                                                           */
/*  Description      : This function prepares and transmits the EAP message  */
/*                     M3 as described in [WPSSPEC]                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Message type (0 for M5, 1 for M7) No Error check   */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares the WSC-EAP message M5 or M7 as*/
/*                     described in standard and stores the message at       */
/*                     wps_enrollee->tx_eap_msg.                             */
/*                                                                           */
/*  Outputs          : M5 or M7 message to peer                              */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_m5_or_7(wps_enrollee_t* wps_enrollee,
                                 UWORD8 msg_type)
{
    UWORD16 len;
    UWORD8  *buf_ptr, *iv_ptr;
    wps_sub_elem_t *subelem_ptr = &(wps_enrollee->config_ptr->version2);
    /*************************************************************************/
    /* Fetch memory from previously stored buffer                            */
    /*************************************************************************/
    buf_ptr = wps_enrollee->tx_m_msg_ptr;
    if((NULL == wps_enrollee->tx_eap_msg)||(NULL == buf_ptr))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    buf_ptr = wps_put_wsc_version(buf_ptr);
    buf_ptr = wps_put_wsc_msg_type(buf_ptr,
        (UWORD8)(msg_type?WPS_MSG_M7:WPS_MSG_M5));
    buf_ptr = wps_put_reg_nonce(buf_ptr, wps_enrollee->reg_nonce);

    /*************************************************************************/
    /* Put Encrypted Settings ID and skip length field. The length field will*/
    /* be filled when all the sub-fields of Encrypted Settings have be framed*/
    /* and hence the length of Encrypted Settings is known                   */
    /*************************************************************************/
    /*                        Encrypted Setting Structure                    */
    /*************************************************************************/
    /*        0         |       1     |        2        |     3       |      */
    /*+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+      */
    /*|  Attribute ID   |             |            Length             |      */
    /*+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+      */
    /*|                     IV (1-4)                                  |      */
    /*+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+      */
    /*|                     IV (5-8)                                  |      */
    /*+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+      */
    /*|                     IV (9-12)                                 |      */
    /*+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+      */
    /*|                     IV (13-16)                                |      */
    /*+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+      */
    /*|                     Encrypted data...                                */
    /*+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+                              */
    /*************************************************************************/
    PUT_U16_BE(buf_ptr, WPS_ATTR_ENCRY_SETTINGS);
    buf_ptr += 2;
    iv_ptr = buf_ptr+2;
    /*************************************************************************/
    /* generate IV and store                                                 */
    /*************************************************************************/
    wps_get_rand_byte_array(iv_ptr, WPS_AES_BYTE_BLOCK_SIZE);
    buf_ptr = iv_ptr + WPS_AES_BYTE_BLOCK_SIZE;

    /*************************************************************************/
    /* Based on msg_type, put all sub-attributes for Encrypted settings      */
    /* except Key Wrap Authenticator in to the buffer pointed by             */
    /* encr_sub_attr_ptr and calculate length encr_sub_attr_len of all these */
    /* sub attributes.                                                       */
    /*************************************************************************/
    PUT_U16_BE(buf_ptr, (msg_type?WPS_ATTR_E_SNONCE2:WPS_ATTR_E_SNONCE1));
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_NONCE_LEN);
    buf_ptr += 2;
    memcpy(buf_ptr, (msg_type?
        wps_enrollee->non_olap.nonce_hash.e_snonce2:
        wps_enrollee->non_olap.nonce_hash.e_snonce1),
        WPS_NONCE_LEN);
    buf_ptr += WPS_NONCE_LEN;

    /*************************************************************************/
    /* Put Key Wrap Authenticator attribute at the end of Encrypted Settings */
    /* and calculate pad length. Add PKCS 5.0 pad                            */
    /*************************************************************************/
    buf_ptr += wps_put_key_wrap_auth(wps_enrollee,
        iv_ptr + WPS_AES_BYTE_BLOCK_SIZE,
        (UWORD16) (buf_ptr - (iv_ptr + WPS_AES_BYTE_BLOCK_SIZE)));
    len = WPS_AES_BYTE_BLOCK_SIZE -
        ((buf_ptr - (iv_ptr + WPS_AES_BYTE_BLOCK_SIZE))
        % WPS_AES_BYTE_BLOCK_SIZE);
    memset(buf_ptr, len, len);
    buf_ptr += len;

    /*************************************************************************/
    /* Calculate total length of Encrypted Settings. Do encryption. Only     */
    /* data is encrypted and not IV so pass appropriate length to encryption */
    /* function. Encrypted data will overwrite the original data             */
    /*************************************************************************/
    len = buf_ptr - iv_ptr;
    wps_aes_encrypt(wps_enrollee, iv_ptr,
        (UWORD16)(len-WPS_AES_BYTE_BLOCK_SIZE));

    /*************************************************************************/
    /* Length Encrypted Settings attribute is now know, so put it in the     */
    /* length field of the Encrypted Settings attribute                      */
    /*************************************************************************/
    PUT_U16_BE((iv_ptr-2), len);

    buf_ptr = wps_put_wfa_vendor(buf_ptr, &subelem_ptr, 1);
    /*************************************************************************/
    /* Put Authenticator Attribute and send the Frame for transmission       */
    /*************************************************************************/
    buf_ptr += wps_put_auth(wps_enrollee, wps_enrollee->tx_m_msg_ptr,
        (UWORD16) (buf_ptr - wps_enrollee->tx_m_msg_ptr));
    len  = buf_ptr - wps_enrollee->tx_eap_msg;
    wps_enrollee->tx_m_msg_len = len -
        (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);
    /*************************************************************************/
    /* Prepare the WPS EAP frame header.                                     */
    /*************************************************************************/
    wps_prep_eap_res_header(wps_enrollee->tx_eap_msg, len,
        wps_enrollee->eap_received_id, WPS_OP_CODE_WSC_MSG);
    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    wps_enrollee->tx_eap_txed_msg_len =0;
    return (wps_tx_eap_msg(wps_enrollee));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_frag_ack                                     */
/*                                                                           */
/*  Description      : This function prepares and transmits a WSC EAP frame  */
/*                     of type WSC_FRAG_ACK                                  */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares and transmits a WSC EAP frame  */
/*                     of type WSC_FRAG_ACK                                  */
/*                                                                           */
/*  Outputs          : WSC_FRAG_ACK message to peer                          */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_frag_ack(wps_enrollee_t* wps_enrollee)
{
    UWORD8  *buffer;
    /*************************************************************************/
    /* Fetch memory for the EAPOL-Packet to be sent to the Remote STA        */
    /*************************************************************************/
//    buffer = (UWORD8*) wps_frame_mem_alloc(WPS_EAP_WSC_FRAG_ACK_MSG_LEN);
    buffer = (UWORD8*) wps_frame_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);
    if(buffer == NULL)
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    memset(buffer, 0, MAX_WPS_WLAN_FRAME_LEN);
    /*************************************************************************/
    /* Prepare WSC_FRAG_ACK message.There is no body for this message        */
    /* Prepare the frame 802.1x & EAP headers.                               */
    /*************************************************************************/
    wps_prep_1x_eap_res_header(buffer,
        (EAP_WPS_DATA_WI_LEN_OFFSET - EAP_HEADER_OFFSET),
        wps_enrollee->eap_received_id, WPS_OP_CODE_WSC_FRAG_ACK);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    send_eapol(wps_enrollee->sel_ap_info.bssid, buffer,
        (UWORD16) (WPS_EAPOL_1X_HDR_LEN + EAP_WPS_DATA_WI_LEN_OFFSET
        - EAP_HEADER_OFFSET), BFALSE);
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_done                                         */
/*                                                                           */
/*  Description      : This function prepares the WSC-EAP message of type    */
/*                     WSC_DONE                                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares the WSC-EAP message of type    */
/*                     WSC_DONE as described in standard and stores the      */
/*                     message at wps_enrollee->tx_eap_msg. Then it transmits*/
/*                     the frame. Only mandatory attributes are framed.      */
/*                                                                           */
/*  Outputs          : WSC_DONE frame to peer                                */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_send_done(wps_enrollee_t* wps_enrollee)
{
    UWORD16 tx_len;
    UWORD8  *buf_ptr;
    wps_sub_elem_t *subelem_ptr = &(wps_enrollee->config_ptr->version2);

    /*************************************************************************/
    /* Fetch memory from previously stored buffer                            */
    /*************************************************************************/
    buf_ptr = wps_enrollee->tx_m_msg_ptr;
    if((NULL == wps_enrollee->tx_eap_msg)||(NULL == buf_ptr))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    /*************************************************************************/
    /* Prepare WSC_ACK message. Only mandatory attributes (Version, Message  */
    /* Type, Enrollee Nonce and Registrar Nonce are framed                   */
    /*************************************************************************/
    buf_ptr = wps_put_wsc_version(buf_ptr);
    buf_ptr = wps_put_wsc_msg_type(buf_ptr, (UWORD8)WPS_MSG_WSC_DONE);
    buf_ptr = wps_put_enr_nonce(buf_ptr, wps_enrollee->enr_nonce);
    buf_ptr = wps_put_reg_nonce(buf_ptr, wps_enrollee->reg_nonce);
    buf_ptr = wps_put_wfa_vendor(buf_ptr, &subelem_ptr, 1);
    tx_len  = buf_ptr - wps_enrollee->tx_eap_msg;
    wps_enrollee->tx_m_msg_len = tx_len -
        (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);
    /*************************************************************************/
    /* Prepare the WPS EAP frame header.                                     */
    /*************************************************************************/
    wps_prep_eap_res_header(wps_enrollee->tx_eap_msg, tx_len,
        wps_enrollee->eap_received_id,
        WPS_OP_CODE_WSC_DONE);

    /*************************************************************************/
    /* Schedule the EAPOL frame for transmission                             */
    /*************************************************************************/
    wps_enrollee->tx_eap_txed_msg_len =0;
    return (wps_tx_eap_msg(wps_enrollee));
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_put_key_wrap_auth                                 */
/*                                                                           */
/*  Description      : This function prepares the Key Wrap Authenticator     */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the input message.Key Wrap Authenticator*/
/*                        attribute is put at the end of data                */
/*                     3) Length of input message                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates the Key Wrap Authenticator for*/
/*                     a given message and puts the Key Wrap Authenticator   */
/*                     attribute at the end of the message                   */
/*                                                                           */
/*  Outputs          : Key Wrap Authenticator Attribute                      */
/*  Returns          : Length of Key Wrap Authenticator Attribute            */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
UWORD8 wps_put_key_wrap_auth(wps_enrollee_t* wps_enrollee, UWORD8* msg_ptr,
                             UWORD16 msg_len)
{
    UWORD8 temp_buff[SHA_256_HASH_LEN];
    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, &msg_ptr, &msg_len,
        1, temp_buff);
    msg_ptr += msg_len;
    PUT_U16_BE(msg_ptr, WPS_ATTR_KEY_WRAP_AUTH);
    msg_ptr += 2;
    PUT_U16_BE(msg_ptr, WPS_KEY_WRAP_AUTH_LEN);
    msg_ptr += 2;
    memcpy(msg_ptr, temp_buff, WPS_KEY_WRAP_AUTH_LEN);
    return(WPS_KEY_WRAP_AUTH_LEN+4);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_put_auth                                          */
/*                                                                           */
/*  Description      : This function generates the Authenticator attribute   */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the input message. Authenticator        */
/*                        attribute is put at the end of data                */
/*                     3) Length of input message                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates the Authenticator for a given */
/*                     message and puts the Authenticator attribute at the   */
/*                     end of the message. It also uses the received message */
/*                     to compute the Authenticator value                    */
/*                                                                           */
/*  Outputs          : Authenticator Attribute                               */
/*  Returns          : Length of Authenticator Attribute                     */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
UWORD8 wps_put_auth(wps_enrollee_t* wps_enrollee, UWORD8* msg_ptr,
                    UWORD16 msg_len)
{
    UWORD8  temp_buff[SHA_256_HASH_LEN];
    UWORD8* msg_ptr_arr[2];
    UWORD16 msg_len_arr[2];

    msg_ptr_arr[0] = wps_enrollee->rx_m_msg_ptr;
    msg_ptr_arr[1] = msg_ptr;
    msg_len_arr[0] = wps_enrollee->rx_m_msg_len;
    msg_len_arr[1] = msg_len;

    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr,
        msg_len_arr, 2, temp_buff);

    msg_ptr += msg_len;
    PUT_U16_BE(msg_ptr, WPS_ATTR_AUTHENTICATOR);
    msg_ptr += 2;
    PUT_U16_BE(msg_ptr, WPS_AUTHENTICATOR_LEN);
    msg_ptr += 2;
    memcpy(msg_ptr, temp_buff, WPS_AUTHENTICATOR_LEN);
    return(WPS_AUTHENTICATOR_LEN+4);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_put_auth                                          */
/*                                                                           */
/*  Description      : This function generates the PSK1, PSK2, E-Hash1 and   */
/*                     E-Hash2                                               */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to location where E-hash1 and E-Hash2      */
/*                        attribute should be stored                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates the PSK1, PSK2, E-Hash1 and   */
/*                     E-Hash2 and puts E-Hash1 and E-Hash2 attributes in the*/
/*                     input buffer.                                         */
/*                                                                           */
/*  Outputs          : PSK1, PSK2, E-Hash1, E-Hash2                          */
/*  Returns          : Total length of E-Hash1 and E-Hash2 Attributes        */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
UWORD8 wps_gen_e_hash(wps_enrollee_t* wps_enrollee, UWORD8* buf_ptr)
{

    UWORD8* msg_ptr_arr[4];
    UWORD16 msg_len_arr[4];
    UWORD8  dev_pass[MAX_WPS_PASS_LEN];
    UWORD8  dev_pass_len;

    if(PBC == wps_enrollee->config_ptr->prot_type)
    {
        memset(dev_pass, '0',WPS_PBC_PASS_LEN);
        dev_pass_len = WPS_PBC_PASS_LEN;
    }
    else
    {
        dev_pass_len = wps_enrollee->config_ptr->dev_pass_len;
        memcpy(dev_pass, wps_enrollee->config_ptr->dev_pass, dev_pass_len);
    }
    /*************************************************************************/
    /* PSK1=first 128 bits of HMACAuthKey(1st half of DevicePassword)        */
    /* Use input message buffer for temporary storing hash. it will have     */
    /* enough space for atleast 2 hash ad E-Hash1 and E-hash2 need to be     */
    /* stored there.                                                         */
    /*************************************************************************/
    msg_ptr_arr[0] = dev_pass;
    msg_len_arr[0] = (dev_pass_len + 1) >> 1;
    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr,
        msg_len_arr, 1, buf_ptr);
    memcpy(wps_enrollee->non_olap.nonce_hash.psk1, buf_ptr, WPS_PSK_LEN);

    /*************************************************************************/
    /* PSK2=first 128 bits of HMACAuthKey(2nd half of DevicePassword)        */
    /* Use input message buffer for temporary storing hash. it will have     */
    /* enough space for atleast 2 hash ad E-Hash1 and E-hash2 need to be     */
    /* stored there. msg_len_arr[0] already have first half length of device */
    /* password.                                                             */
    /*************************************************************************/
    msg_ptr_arr[0] = dev_pass + msg_len_arr[0];
    msg_len_arr[0] = (dev_pass_len) >> 1;
    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr,
        msg_len_arr, 1, buf_ptr);
    memcpy(wps_enrollee->non_olap.nonce_hash.psk2, buf_ptr, WPS_PSK_LEN);

    /*************************************************************************/
    /* Generate Secret Nonce E-SNonce1 and E-SNonce2                         */
    /*************************************************************************/
    wps_get_rand_byte_array(wps_enrollee->non_olap.nonce_hash.e_snonce1,
        WPS_NONCE_LEN);
    wps_get_rand_byte_array(wps_enrollee->non_olap.nonce_hash.e_snonce2,
        WPS_NONCE_LEN);

    /*************************************************************************/
    /* Generate E-Hash1 and E-Hash2. Directly put the generated values in the*/
    /* corresponding attributes value fields                                 */
    /*************************************************************************/
    PUT_U16_BE(buf_ptr, WPS_ATTR_E_HASH1);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_HASH_LEN);
    buf_ptr += 2;

    /*************************************************************************/
    /* Generate E-Hash1 = HMACAuthKey(E-S1 || PSK1 || PKE || PKR)            */
    /*************************************************************************/
    msg_ptr_arr[0] = wps_enrollee->non_olap.nonce_hash.e_snonce1;
    msg_ptr_arr[1] = wps_enrollee->non_olap.nonce_hash.psk1;
    msg_ptr_arr[2] = wps_enrollee->pke;
    msg_ptr_arr[3] = wps_enrollee->pub_key.pkr;

    msg_len_arr[0] = WPS_NONCE_LEN;
    msg_len_arr[1] = WPS_PSK_LEN;
    msg_len_arr[2] = WPS_PUBLIC_KEY_LEN;
    msg_len_arr[3] = WPS_PUBLIC_KEY_LEN;
    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr,
        msg_len_arr, 4, buf_ptr);
    buf_ptr += WPS_HASH_LEN;

    /*************************************************************************/
    /* Generate E-Hash2 = HMACAuthKey(E-S2 || PSK2 || PKE || PKR)            */
    /* Only E-S2 and PSK2 pointers needs to be assigned as other fields are  */
    /* same for E-hash1 and E-Hash2                                          */
    /*************************************************************************/
    PUT_U16_BE(buf_ptr, WPS_ATTR_E_HASH2);
    buf_ptr += 2;
    PUT_U16_BE(buf_ptr, WPS_HASH_LEN);
    buf_ptr += 2;
    msg_ptr_arr[0] = wps_enrollee->non_olap.nonce_hash.e_snonce2;
    msg_ptr_arr[1] = wps_enrollee->non_olap.nonce_hash.psk2;
    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, msg_ptr_arr,
        msg_len_arr, 4, buf_ptr);

    return((WPS_HASH_LEN+4)*2);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_aes_encrypt                                       */
/*                                                                           */
/*  Description      : This function dose the AES-CBC encryption as per      */
/*                     FIPS PUB 197                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the data to be encrypted. First         */
/*                        WPS_AES_BYTE_BLOCK_SIZE number of bytes has IV     */
/*                        followed by the data to be encrypted               */
/*                     3) Length of the data to be encrypted (without IV     */
/*                        length)                                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function dose the AES-CBC encryption. It is      */
/*                     assumed that IV, of length WPS_AES_BYTE_BLOCK_SIZE    */
/*                     bytes, is stored in the beginning of data followed by */
/*                     the data to be encrypted.                             */
/*                                                                           */
/*  Outputs          : Encrypted data                                        */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_aes_encrypt(wps_enrollee_t* wps_enrollee, UWORD8* data_ptr,
                     UWORD16 data_len)
{
    aes_context_t *aes_context;
    UWORD8* temp_ptr;
    UWORD16 cnt;
    UWORD16 num_blocks = (data_len/ WPS_AES_BYTE_BLOCK_SIZE);
    UWORD8 cnt_u8;
    if(NULL == (aes_context =
        wps_mem_alloc(sizeof(aes_context_t))))
    {
        wps_handle_sys_err(wps_enrollee);
        return;
    }
    aes_setup(aes_context, WPS_KEY_WRAP_KEY_LEN, wps_enrollee->key_wrap_key);
    for (cnt =0; cnt < num_blocks; cnt++)
    {
        temp_ptr = data_ptr + WPS_AES_BYTE_BLOCK_SIZE;
        for(cnt_u8 = 0; cnt_u8 < WPS_AES_BYTE_BLOCK_SIZE; cnt_u8++)
        {
            *temp_ptr++  ^= *data_ptr++;
        }
        aes_encrypt(aes_context, data_ptr,data_ptr);
    }
    wps_mem_free(aes_context);
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_aes_encrypt                                       */
/*                                                                           */
/*  Description      : This function dose the AES-CBC encryption as per      */
/*                     FIPS PUB 197                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the data to be decrypted. First         */
/*                        WPS_AES_BYTE_BLOCK_SIZE number of bytes has IV     */
/*                        followed by the data to be decrypted               */
/*                     3) Pointer to the location where decrypted data is to */
/*                        be stored                                          */
/*                     4) Length of the data to be decrypted (without IV     */
/*                        length)                                            */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function dose the AES-CBC decryption. It is      */
/*                     assumed that IV, of length WPS_AES_BYTE_BLOCK_SIZE    */
/*                     bytes, is stored in the beginning of data followed by */
/*                     the data to be decrypted. 1. In place processing is   */
/*                     not done for decryption as the received packets need  */
/*                     to be stored for Authenticator computation for next   */
/*                     transmit message.                                     */
/*                                                                           */
/*  Outputs          : Decrypted data                                        */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_aes_decrypt(wps_enrollee_t* wps_enrollee,  UWORD8* data_inp_ptr,
                     UWORD8* data_out_ptr, UWORD16 data_len)
{
    aes_context_t *aes_context;
    UWORD8* temp_ptr;
    UWORD16 cnt;
    UWORD16 num_blocks = (data_len/ WPS_AES_BYTE_BLOCK_SIZE);
    UWORD8 cnt_u8;
    if(NULL == (aes_context =
        wps_mem_alloc(sizeof(aes_context_t))))
    {
        wps_handle_sys_err(wps_enrollee);
        return;
    }
    aes_setup(aes_context, WPS_KEY_WRAP_KEY_LEN, wps_enrollee->key_wrap_key);
    data_inp_ptr += WPS_AES_BYTE_BLOCK_SIZE;
    for (cnt =0; cnt < num_blocks; cnt++)
    {
        aes_decrypt(aes_context, data_inp_ptr, data_out_ptr);
        temp_ptr = data_inp_ptr - WPS_AES_BYTE_BLOCK_SIZE;
        for(cnt_u8 = 0; cnt_u8 < WPS_AES_BYTE_BLOCK_SIZE; cnt_u8++)
        {
            *data_out_ptr++  ^= *temp_ptr++;
        }
        data_inp_ptr += WPS_AES_BYTE_BLOCK_SIZE;
    }
    wps_mem_free(aes_context);
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_start_reg_msg_timer                               */
/*                                                                           */
/*  Description      : This function starts the timer (if already not running*/
/*                     ) for individual message timeout during registration  */
/*                     protocol.                                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function starts the timer (if already not running*/
/*                     ) for individual message timeout during registration  */
/*                     protocol.                                             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_start_reg_msg_timer(wps_enrollee_t* wps_enrollee)
{
    UWORD32 timeout = WPS_REG_EAP_MSG_TIME;
    if(NULL == wps_enrollee->reg_msg_timer)
    {
        if( NULL == (wps_enrollee->reg_msg_timer = wps_create_alarm(
        wps_enrollee->config_ptr->timer_cb_fn_ptr, WPS_REG_MSG_TIMEOUT, NULL))) //Hugh: fixme.
        {
            wps_handle_sys_err(wps_enrollee);
            return FAILURE;
        }
    }
    else
    {
        wps_stop_alarm(wps_enrollee->reg_msg_timer);
    }
    if(EAPOL_START_ST == wps_enrollee->reg_state)
    {
        timeout = WPS_MSG_RETR_TIME;
    }
    else if((MSG_ST ==wps_enrollee->reg_state) &&
        (DONE_ST == wps_enrollee->msg_st))
    {
        timeout = WPS_REG_EAP_FAIL_MSG_TIME;
    }

    wps_start_alarm(wps_enrollee->reg_msg_timer, timeout);
    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_eap_reg_msg_timeout                               */
/*                                                                           */
/*  Description      : This function handles EAP Message timeout             */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function is called when the EAP Message timeout  */
/*                     occurs. If the message retransmission limit has not   */
/*                     reached then the message is retransmitted otherwise   */
/*                     registration protocol is terminated. .                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_eap_reg_msg_timeout(wps_enrollee_t* wps_enrollee)
{
    UWORD8 cnt;
    /*************************************************************************/
    /* If not in registration protocol then its a false alarm, ignore it     */
    /*************************************************************************/
    if(IN_REG_PROT != wps_enrollee->state)
    {
        return;
    }
    /*************************************************************************/
    /* If EAPOL START state and retransmission limit not yet reached then    */
    /* retransmit EAPOL START                                                */
    /*************************************************************************/
    if((EAPOL_START_ST == wps_enrollee->reg_state) &&
        (MAX_EAP_MSG_RETRANS_LIMIT > wps_enrollee->eapol_start_cnt))
    {
        wps_enrollee->eapol_start_cnt++;
        wps_send_eapol_start(wps_enrollee);
        wps_start_reg_msg_timer(wps_enrollee);
        return;
    }

    /*************************************************************************/
    /* Check if already transmitted WSC-Done, if so then assume protocol is  */
    /* successful and complete. This is to fix the issue with Buffalo AP     */
    /*************************************************************************/
    if((MSG_ST ==wps_enrollee->reg_state) && (DONE_ST == wps_enrollee->msg_st))
    {
        wps_stop_enrollee(wps_enrollee);
        end_wps_enrollee(REG_PROT_SUCC_COMP);
        return;
    }

    wps_enrollee->assoc_state = WPS_CONFIGURATION_FAILURE;
    wps_enrollee->config_error = MESSAGE_TIMEOUT;
    /*************************************************************************/
    /* Check if UUID-R is non zero                                           */
    /*************************************************************************/
    for(cnt =0; cnt < WPS_UUID_LEN; cnt++)
    {
        if(0 != wps_enrollee->reg_uuid[cnt])
        {
            cnt = 0;
            break;
        }
    }
    send_wps_status(ERR_REG_MSG_TIMEOUT, NULL, 0);
    /*************************************************************************/
    /* Check if NACK can be transmitted if so then transmit NACK             */
    /*************************************************************************/
    if((MSG_ST ==wps_enrollee->reg_state) &&
        ((M1_ST < wps_enrollee->msg_st) ||
        ((wps_enrollee->msg_st == M1_ST)&&(0 == cnt))))
    {
        wps_enrollee->msg_st = NACK_ST;
        wps_send_nack(wps_enrollee);
        wps_start_reg_msg_timer(wps_enrollee);
        return;
    }
    /*************************************************************************/
    /* If UUID-R present add it to Discarded list. Restart the WPS Protocol  */
    /*************************************************************************/
    if(0 == cnt)
    {
        wps_add_discarded_reg_ap(wps_enrollee,
            wps_enrollee->sel_ap_info.bssid);
    }
    wps_delete_timers(wps_enrollee);
    wps_loc_start_scan(wps_enrollee);
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_eap_reg_timeout                                   */
/*                                                                           */
/*  Description      : This function handles EAP registration protocol       */
/*                     timeouts                                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function is called when the EAP registration     */
/*                     protocol timeout occurs. This function stops the      */
/*                     registration protocol and restarts the scan           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_eap_reg_timeout(wps_enrollee_t* wps_enrollee)
{
    UWORD8 cnt;
    /*************************************************************************/
    /* If not in registration protocol then its a false alarm, ignore it     */
    /*************************************************************************/
    if(IN_REG_PROT != wps_enrollee->state)
    {
        return;
    }
    wps_enrollee->assoc_state = WPS_CONFIGURATION_FAILURE;
    wps_enrollee->config_error = REGISTRATION_SESSION_TIMEOUT;
    send_wps_status(ERR_REG_PROT_TIMEOUT, NULL, 0);
    /*************************************************************************/
    /* Check if NACK can be transmitted if so then transmit NACK(only once)  */
    /*************************************************************************/
    if((MSG_ST == wps_enrollee->reg_state) && (M1_ST < wps_enrollee->msg_st))
    {
        wps_enrollee->msg_st = NACK_ST;
        wps_send_nack(wps_enrollee);
        wps_start_reg_msg_timer(wps_enrollee);
        return;
    }
    /*************************************************************************/
    /* Check if UUID-R is non zero                                           */
    /* If UUID-R present add it to Discarded list. Restart the WPS Protocol  */
    /*************************************************************************/
    for(cnt =0; cnt < WPS_UUID_LEN; cnt++)
    {
        if(0 != wps_enrollee->reg_uuid[cnt])
        {
            wps_add_discarded_reg_ap(wps_enrollee,
                wps_enrollee->sel_ap_info.bssid);
            break;
        }
    }
    wps_delete_timers(wps_enrollee);
    wps_loc_start_scan(wps_enrollee);
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_add_discarded_reg_ap                              */
/*                                                                           */
/*  Description      : This function adds a Registrar to the discarded list. */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the BSSID of the registrar that needs to */
/*                        be added to the discarded list                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function adds a Registrar to the discarded list. */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_add_discarded_reg_ap(wps_enrollee_t* wps_enrollee, UWORD8* bssid)
{
    UWORD8 cnt = wps_enrollee->discarded_reg_cnt;

    if(MAX_DIS_REG_CNT  > cnt)
    {
        wps_enrollee->discarded_reg_cnt++;
    }
    else
    {
        for(cnt =0; cnt < MAX_DIS_REG_CNT - 2; cnt++)
        {
            memcpy(&wps_enrollee->dis_ap_reg_list[cnt],
                &wps_enrollee->dis_ap_reg_list[cnt+1],
                sizeof(wps_dis_ap_reg_info_struct_t));
        }
        wps_enrollee->discarded_reg_cnt = MAX_DIS_REG_CNT;
    }
    memcpy(wps_enrollee->dis_ap_reg_list[cnt].bssid, bssid, WPS_MAC_ADDR_LEN);
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_get_rand_byte_array                               */
/*                                                                           */
/*  Description      : This function fills a byte array with random numbers  */
/*                                                                           */
/*  Inputs           : 1) None                                               */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function uses get_random_byte function to        */
/*                     generate a random byte  and put it in to input array  */
/*                                                                           */
/*  Outputs          : Array of random numbers                               */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
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

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_event                                      */
/*                                                                           */
/*  Description      : This function handles the various timeout events.     */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Input data to identify the event.                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function handles the various timeout events.     */
/*                     based on inp_data, this function identifies the       */
/*                     timeout event and calls the appropriate function.     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_handle_event(wps_enrollee_t* wps_enrollee, UWORD16 inp_data)
{
	TROUT_FUNC_ENTER;
    switch(inp_data)
    {
        case WPS_REG_MSG_TIMEOUT:
            wps_eap_reg_msg_timeout(wps_enrollee);
            break;
        case WPS_REG_TIMEOUT:
            wps_eap_reg_timeout(wps_enrollee);
        case WPS_WALK_TIMEOUT:
            wps_walk_timeout(wps_enrollee);
        default:
            break;
    }
    TROUT_FUNC_EXIT;
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_prep_1x_eap_res_header                            */
/*                                                                           */
/*  Description      : This function prepares 802.1x and WSC EAP header      */
/*                                                                           */
/*  Inputs           : 1) Pointer to the message buffer                      */
/*                     2) EAP frame length                                   */
/*                     3) EAP Identifier                                     */
/*                     4) WSC Op-Code                                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares 802.1x and WSC EAP header      */
/*                     It is assumed that all inputs are correct and hence   */
/*                     no check is done                                      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_prep_1x_eap_res_header(UWORD8* buffer, UWORD16 tx_len, UWORD8 eap_id,
                                UWORD8 op_code)
{
    wps_prep_1x_header(buffer, ONE_X_PCK_TYPE_EAP, tx_len);
    wps_prep_eap_res_header(buffer+WPS_MAX_MAC_HDR_LEN+EAP_HEADER_OFFSET,
        tx_len, eap_id, op_code);
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_prep_1x_header                                    */
/*                                                                           */
/*  Description      : This function prepares 802.1x header                  */
/*                                                                           */
/*  Inputs           : 1) Pointer to the message buffer                      */
/*                     2) Type of 802.1x frame                               */
/*                     3) Length of 802.1x body                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares 802.1x header                  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_prep_1x_header(UWORD8* buffer, UWORD8 one_x_type, UWORD16 tx_len)
{
    /*************************************************************************/
    /* Prepare the frame 802.1x header.                                      */
    /*************************************************************************/
    /* 802.1x header format                                                  */
    /* --------------------------------------------------------------------- */
    /* | SNAP    | Version | Type    | Body Length | Body                  | */
    /* --------------------------------------------------------------------- */
    /* | 8 octet | 1 octet | 1 octet | 2 octet     | (Length - 4) octets   | */
    /* --------------------------------------------------------------------- */
    /*************************************************************************/
    memcpy(buffer+WPS_MAX_MAC_HDR_LEN, wps_1x_header, sizeof(wps_1x_header));
    buffer += WPS_MAX_MAC_HDR_LEN + ONE_X_PCK_TYPE_OFFSET;
    *buffer++ = one_x_type;
    PUT_U16_BE(buffer, tx_len);
    return;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_prep_eap_res_header                               */
/*                                                                           */
/*  Description      : This function prepares WSC EAP header                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to the EAP message buffer                  */
/*                     2) EAP frame length                                   */
/*                     3) EAP Identifier                                     */
/*                     4) WSC Op-Code                                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function prepares WSC EAP header                 */
/*                     It is assumed that all inputs are correct and hence   */
/*                     no check is done                                      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_prep_eap_res_header(UWORD8* buf_ptr, UWORD16 tx_len, UWORD8 eap_id,
                                UWORD8 op_code)
{
    /*************************************************************************/
    /* The EAP Packet format for WPS                                         */
    /*************************************************************************/
    /*         0       |      1        |       2       |       3       |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |     Code      |  Identifier   |            Length             |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |     Type      |               Vendor-Id                       |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |                          Vendor-Type                          |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |  Op-Code      |    Flags      | Message Length                |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |  Message data                                                       */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-                                          */
    /*************************************************************************/
    *buf_ptr++ = EAP_CODE_RESPONSE;
    *buf_ptr++ = eap_id;
    PUT_U16_BE(buf_ptr, tx_len);
    buf_ptr += 2;
    memcpy(buf_ptr, eap_wps_type_vid_vt, EAP_WPS_TYPE_VID_VT_LEN);
    buf_ptr   += EAP_WPS_TYPE_VID_VT_LEN;
    *buf_ptr++ = op_code;
    *buf_ptr++ = 0;//WPS_EAP_MSG_FLAG_LF;
//    tx_len    -= (EAP_WPS_DATA_WI_LEN_OFFSET - EAP_HEADER_OFFSET);
//    PUT_U16_BE(buf_ptr, tx_len);
    return;
}

#ifdef MAC_P2P
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_add_wsc_ie_p2p_prob_rsp                           */
/*                                                                           */
/*  Description      : This function generates WSC_IE which can be used in   */
/*                     probe response frame sent by the enrollee             */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Buffer pointer where WSC_IE should be stored       */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates WSC_IE which can be used in   */
/*                     probe response frame sent by the enrollee. Only       */
/*                     parameters which are mandatory in are framed.         */
/*                                                                           */
/*  Outputs          : WSC_IE                                                */
/*  Returns          : Length of WSC_IE generated                            */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 wps_add_wsc_ie_p2p_prob_rsp(wps_enrollee_t* wps_enrollee,
                                    UWORD8* buff_ptr)
{
    UWORD8* loc_buf_ptr = buff_ptr;
    UWORD16  length      = 0;
    wps_enr_config_struct_t* config_ptr = wps_enrollee->config_ptr;

    /*************************************************************************/
    /* Prepare WSC_IE. Only mandatory attributes are framed                  */
    /*************************************************************************/
    /* WPS Information Element                                               */
    /*************************************************************************/
    /* ----------------------------------------------------------------------*/
    /* |     Element ID     |      Length   |      OUI    |     Data        |*/
    /* +--------------------+---------------+-------------+-----------------+*/
    /* |         1 Byte     |       1 Byte  |     4 Byte  |     1-251 Bytes |*/
    /* ----------------------------------------------------------------------*/
    /*************************************************************************/
    *loc_buf_ptr = WPS_IE_ID;
    loc_buf_ptr += 2;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE0;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE1;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE2;
    *loc_buf_ptr++ = WPS_IE_OUI_BYTE3;


    loc_buf_ptr = wps_put_wsc_version(loc_buf_ptr);

    loc_buf_ptr = wps_put_wps_state(loc_buf_ptr,
                                    wps_enrollee->config_ptr->wps_state);
    loc_buf_ptr = wps_put_sel_reg(loc_buf_ptr, BFALSE);

    loc_buf_ptr = wsc_put_rsp_type(loc_buf_ptr, RSP_ENRO_INFO_ONLY);

    loc_buf_ptr = wps_put_wsc_uuid_e(loc_buf_ptr, wps_enrollee->enr_uuid);
    loc_buf_ptr = wps_put_manufacturer(loc_buf_ptr, config_ptr);
    loc_buf_ptr = wps_put_model_name(loc_buf_ptr, config_ptr);

    loc_buf_ptr = wps_put_model_num(loc_buf_ptr, config_ptr);
    loc_buf_ptr = wps_put_serial_num(loc_buf_ptr, config_ptr);

    loc_buf_ptr = wps_put_prim_dev_type(loc_buf_ptr, config_ptr);
    loc_buf_ptr = wps_put_device_name(loc_buf_ptr, config_ptr);
    loc_buf_ptr = wps_put_cofig_meth(loc_buf_ptr, config_ptr->config_meth);
    loc_buf_ptr = wps_put_rf_bands(loc_buf_ptr, config_ptr->rf_bands);

    /* Calculate length and add the length to the Length field               */
    length = (UWORD8)(loc_buf_ptr - buff_ptr);
    buff_ptr[1] = length - 2;

    return((UWORD16)(length));
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : p2p_add_wsc_ie                                        */
/*                                                                           */
/*  Description      : This function calls the funtion to add the WSC IE     */
/*                     for a P2P frame                                       */
/*                                                                           */
/*  Inputs           : 1) Buffer pointer where WSC_IE should be stored       */
/*                     2) Frame Type                                         */
/*                                                                           */
/*  Globals          : g_wps_enrollee_ptr                                    */
/*                                                                           */
/*  Processing       : This function calls the funtion to add the WSC IE     */
/*                     for a P2P frame                                       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : UWORD16; Total length of the WSC IE                   */
/*  Issues           : None                                                  */
/*****************************************************************************/
UWORD16 p2p_add_wsc_ie(wps_enrollee_t *wps_enrollee_ptr, UWORD8* buff_ptr,
                       UWORD8 frm_type)
{
    UWORD16 ie_len = 0;

    if(PROBE_RSP == frm_type)
    {
        ie_len = wps_add_wsc_ie_p2p_prob_rsp(wps_enrollee_ptr, buff_ptr);
    }
    else
    {
        ie_len =  wps_gen_wsc_ie(wps_enrollee_ptr, buff_ptr, frm_type);
    }
    return ie_len;
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
    UWORD8 *loc_frm_ptr = (frame_ptr + index);
    UWORD8 length = 0;

    /* Set the Element ID field */
    *loc_frm_ptr = WPS_IE_ID;

    /* Length field is updated later */
    loc_frm_ptr += 2;

    /* Update OUI-Field */
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE0;
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE1;
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE2;
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE3;

    loc_frm_ptr = wps_put_cofig_meth(loc_frm_ptr, config_meth);

    /* Calculate length and add the length to the Length field */
    length = loc_frm_ptr - (frame_ptr + index);
    frame_ptr[index + 1] = length - 2;

    return length;

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
    UWORD8 *loc_frm_ptr = (frame_ptr + index);
    UWORD8 length = 0;

    /* Set the Element ID field */
    *loc_frm_ptr = WPS_IE_ID;

    /* Length field is updated later */
    loc_frm_ptr += 2;

    /* Update OUI-Field */
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE0;
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE1;
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE2;
    *loc_frm_ptr++ = WPS_IE_OUI_BYTE3;

    loc_frm_ptr = wps_put_wsc_version(loc_frm_ptr);

    if(PBC == prot_type)
    {
        loc_frm_ptr = wps_put_dev_pass_id(loc_frm_ptr,
            (UWORD16) DEV_PASS_ID_PUSHBUTT);
    }
    else
    {
        loc_frm_ptr = wps_put_dev_pass_id(loc_frm_ptr, dev_pas_id);
    }

    /* Calculate length and add the length to the Length field */
    length = loc_frm_ptr - (frame_ptr + index);
    frame_ptr[index + 1] = length - 2;

    return length;
}


#endif /* MAC_P2P */



#endif /* INT_WPS_ENR_SUPP */
