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
/*  File Name         : wps_eap_tx_reg.c                                     */
/*                                                                           */
/*  Description       : This file contains the all the functions of Registrar*/
/*                      for EAP Transmission handling of Registration        */
/*                      Protocol                                             */
/*                                                                           */
/*  List of Functions : wps_send_eap_req_id                                  */
/*                      wps_send_eap_wsc_start                               */
/*                      wps_send_m2                                          */
/*                      wps_send_m2d                                         */
/*                      wps_send_m4                                          */
/*                      wps_send_m6                                          */
/*                      wps_send_m8                                          */
/*                      wps_send_wsc_nack                                    */
/*                      wps_send_eap_fail                                    */
/*                      wps_handle_eap_retry                                 */
/*                      wps_send_wsc_frag_ack                                */
/*                      wps_tx_eap_wsc_pkt                                   */
/*                      wps_gen_cred_attr                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         24 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_REG_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "aes_cbc.h"
#include "dh_key.h"
#include "wps_eap_tx_reg.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

static BOOL_T g_pkr_computed = BFALSE;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static STATUS_T wps_tx_eap_wsc_pkt(wps_t *wps_ctxt_hdl,
                                   wps_eap_tx_t *eap_tx_hdl, UWORD8 eap_code,
                                   UWORD8 wsc_opcode);
static UWORD16 wps_gen_cred_attr(wps_cred_t *wps_cred, UWORD8 *buffer,
                                 UWORD8 num_cred);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_eap_req_id                                   */
/*                                                                           */
/*  Description      : This function prepares and sends EAP Request/Identity */
/*                     frame to the enrollee                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to EAP TX structure                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function adds the EAPOL-SNAP, 802.1X and EAP     */
/*                     header for an EAP Request/Identity packet and sends   */
/*                     the packet to Enrollee. The EAP Identifier is set to  */
/*                     Zero in the EAP Header.                               */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         24 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_eap_req_id(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl)
{
    UWORD8 *buffer = NULL;

    /* Allocate a buffer for Sending EAP Request/ID */
    eap_tx_hdl->tx_wlan_frm_buff = wps_pkt_mem_alloc(MAX_WPS_WLAN_FRAME_LEN);

    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /* Adjust the offset where the 802.1X packet can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN);

    /* Add 802.1X header */
    wps_prep_1x_header(buffer, ONE_X_PKT_TYPE_EAP,
                       (EAP_HEADER_LEN + EAP_TYPE_FIELD_LEN));

    /* Reset the EAP-Identity */
    eap_tx_hdl->tx_eap_id = 0;

    /* Prepare EAP-Header */
    wps_prep_eap_header(buffer, EAP_CODE_REQUEST, eap_tx_hdl->tx_eap_id,
                        (EAP_HEADER_LEN + EAP_TYPE_FIELD_LEN));

    /* Set the EAP Type in EAP Header */
    set_eap_type(buffer, EAP_TYPE_IDENTITY);

    /* Schedule the EAPOL frame for transmission */
    if(BFALSE == send_eapol(wps_ctxt_hdl->wps_config_hdl->mac_addr,
                            eap_tx_hdl->tx_wlan_frm_buff,
                            (WPS_EAPOL_1X_HDR_LEN + EAP_HEADER_LEN +
                             EAP_TYPE_FIELD_LEN),
                            BFALSE))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_eap_wsc_start                                */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-Start        */
/*                     frame to the enrollee                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to EAP-TX structure                        */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     2) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-Start and send it to Enrollee                  */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         25 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_eap_wsc_start(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl)
{
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t   *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;

    /* If Buffer pointer is NULL then exit */
    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /*************************************************************************/
    /* Set the Message Length to Zero as WSC-Start frame will only have      */
    /* EAP-WSC header                                                        */
    /*************************************************************************/
    eap_tx_hdl->tx_m_msg_len = 0;

    /* Increment EAP Id before transmission */
    eap_tx_hdl->tx_eap_id++;

    /* Generate Registrar Nonce, N2 */
    wps_get_rand_byte_array(attr_hdl->reg_nonce, WPS_NONCE_LEN);

    /*************************************************************************/
    /* Generate Registrar`s Public Key, PKr, only on start request from the  */
    /* user is BTRUE                                                         */
    /*************************************************************************/
    if(BTRUE == wps_ctxt_hdl->wps_reg_hdl->start_req)
    {
        wps_gen_pub_key(attr_hdl->reg_pub_key.pk_u32, &priv_hdl->priv_key_ptr);
        g_pkr_computed = BTRUE;
    }
    else
        g_pkr_computed = BFALSE;

    /*************************************************************************/
    /* Put the 802.1X header, EAP-WSC header for WSC-Start frame and then    */
    /* Send the frame to enrollee                                            */
    /*************************************************************************/
    return wps_tx_eap_wsc_pkt(wps_ctxt_hdl, eap_tx_hdl, EAP_CODE_REQUEST,
                              WPS_OP_CODE_WSC_START);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m2                                           */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-MSG  M2 to   */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX structure                     */
/*                     3) Handle to WPS EAP-RX structure                     */
/*                     4) Pointer to received Enrollee Public Key            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     2) Genrate Registrar Nonce, N2.                       */
/*                     3) Generate Registrar`s Public Key, PKr, by generating*/
/*                        a random registrar private key, B                  */
/*                     4) Compute Authentication Key and Key Wrap Key using  */
/*                        the received Enrolle`s Public Key, PKe.            */
/*                     5) Add all the Mandatory Attributes required for M1   */
/*                        Message. Refer to WPS specification for details    */
/*                     6) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-MSG and send it to Enrollee                    */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         25 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_m2(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                     wps_eap_rx_t *eap_rx_hdl, UWORD8 *enr_pub_key)
{
    UWORD16 index = 0;
    UWORD8  auth_attr[SHA_256_HASH_LEN] = {0};
    UWORD8  *buffer  = NULL;
    wps_config_t *config_hdl = wps_ctxt_hdl->wps_config_hdl;
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t   *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;
    wps_sec_key_t *sec_key_hdl = &(priv_hdl->olap_priv_key.secret_keys);
    wps_sub_elem_t *subelem_ptr = &(wps_ctxt_hdl->wps_config_hdl->version2);

    /* If Buffer pointer is NULL then exit */
    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /* Compute Registrar`s Public Key if not computed yet */
    if(BFALSE == g_pkr_computed)
    {
        wps_gen_pub_key(attr_hdl->reg_pub_key.pk_u32, &priv_hdl->priv_key_ptr);
        g_pkr_computed = BTRUE;
    }

    /* Generate Authentication Key, Key Wrap Key */
    if(FAILURE == wps_gen_secr_keys(enr_pub_key,
                                    attr_hdl->enr_pub_key.pk_u32,
                                    attr_hdl->enr_nonce,
                                    attr_hdl->reg_nonce,
                                    config_hdl->mac_addr,
                                    sec_key_hdl->auth_key,
                                    sec_key_hdl->key_wrap_key,
                                    &priv_hdl->priv_key_ptr))
        return FAILURE;

    /* Set Configuration Error attribute to No Error */
    attr_hdl->config_error = WPS_NO_ERROR;

    /* Adjust the offset where WSC Message can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN +
              EAP_WPS_DATA_WO_LEN_OFFSET);

    /* Put all the Mandatory WSC TLV attributes for M2 Message */
    index += put_wsc_version(buffer + index);
    index += put_wsc_msg_type((buffer + index), WPS_MSG_M2);
    index += put_wsc_enr_nonce((buffer + index), attr_hdl->enr_nonce);
    index += put_wsc_reg_nonce((buffer + index), attr_hdl->reg_nonce);
    index += put_wsc_uuid_r((buffer + index), attr_hdl->reg_uuid);
    index += put_wsc_public_key((buffer + index), attr_hdl->reg_pub_key.pk);
    index += put_wsc_auth_type_flags((buffer + index),
                                     config_hdl->auth_type_flags);
    index += put_wsc_encr_type_flags((buffer + index),
                                     config_hdl->encr_type_flags);
    index += put_wsc_conn_type_flags((buffer + index),
                                     config_hdl->conn_type_flags);
    index += put_wsc_config_meth((buffer + index), config_hdl->config_methods);
    index += put_wsc_manufacturer((buffer + index), config_hdl->manufacturer,
                                  config_hdl->manufacturer_len);
    index += put_wsc_model_name((buffer + index), config_hdl->model_name,
                                config_hdl->model_name_len);
    index += put_wsc_model_num((buffer + index), config_hdl->model_num,
                               config_hdl->model_num_len);
    index += put_wsc_serial_num((buffer + index), config_hdl->serial_num,
                                config_hdl->serial_num_len);
    index += put_wsc_prim_dev_type((buffer + index),
                                   config_hdl->prim_dev_cat_id);
    index += put_wsc_device_name((buffer + index), config_hdl->device_name,
                                 config_hdl->device_name_len);
    index += put_wsc_rf_bands((buffer + index), config_hdl->rf_bands);
    index += put_wsc_assoc_state((buffer + index), attr_hdl->assoc_state);
    index += put_wsc_config_error((buffer + index), attr_hdl->config_error);
    index += put_wsc_dev_pass_id((buffer + index), config_hdl->dev_pass_id);
    index += put_wsc_os_ver((buffer + index), config_hdl->os_version);
    index += put_wsc_wfa_vendor((buffer + index), &subelem_ptr, 1);

    /* Set the Message Start Pointer */
    eap_tx_hdl->tx_m_msg_ptr = buffer;

    /* Compute the authenticator attribute */
    wps_compute_auth_attr(eap_rx_hdl->rx_m_msg_ptr, eap_rx_hdl->rx_m_msg_len,
                          eap_tx_hdl->tx_m_msg_ptr, index,
                          sec_key_hdl->auth_key, auth_attr);

    /* Insert the authenticator attribute at given index */
    index += put_wsc_auth_attr((buffer + index), auth_attr);

    /* Update the Length of WSC Message */
    eap_tx_hdl->tx_m_msg_len = index;

    /* Increment EAP Id before transmission */
    eap_tx_hdl->tx_eap_id++;

    /* Send the WSC EAP Packet to Enrollee */
    return wps_tx_eap_wsc_pkt(wps_ctxt_hdl, eap_tx_hdl, EAP_CODE_REQUEST,
                              WPS_OP_CODE_WSC_MSG);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m2d                                          */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-MSG  M2D to  */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX Structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     2) Genrate Registrar Nonce, N2.                       */
/*                     3) Add all the Mandatory Attributes required for M2D  */
/*                        Message. Refer to WPS specification for details    */
/*                     4) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-MSG and send it to Enrollee                    */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_m2d(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl)
{
    UWORD16 index = 0;
    UWORD8  *buffer  = NULL;
    wps_config_t *config_hdl = wps_ctxt_hdl->wps_config_hdl;
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_sub_elem_t *subelem_ptr = &(wps_ctxt_hdl->wps_config_hdl->version2);

    /* If Buffer pointer is NULL then exit */
    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /* Generate Registrar Nonce, N2 */
    wps_get_rand_byte_array(attr_hdl->reg_nonce, WPS_NONCE_LEN);

    /* Adjust the offset where WSC Message can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN +
              EAP_WPS_DATA_WO_LEN_OFFSET);

    /* Put all the Mandatory WSC TLV attributes for M2D Message */
    index += put_wsc_version(buffer + index);
    index += put_wsc_msg_type((buffer + index), WPS_MSG_M2D);
    index += put_wsc_enr_nonce((buffer + index), attr_hdl->enr_nonce);
    index += put_wsc_reg_nonce((buffer + index), attr_hdl->reg_nonce);
    index += put_wsc_uuid_r((buffer + index), attr_hdl->reg_uuid);
    index += put_wsc_auth_type_flags((buffer + index),
                                     config_hdl->auth_type_flags);
    index += put_wsc_encr_type_flags((buffer + index),
                                     config_hdl->encr_type_flags);
    index += put_wsc_conn_type_flags((buffer + index),
                                     config_hdl->conn_type_flags);
    index += put_wsc_config_meth((buffer + index), config_hdl->config_methods);
    index += put_wsc_manufacturer((buffer + index), config_hdl->manufacturer,
                                  config_hdl->manufacturer_len);
    index += put_wsc_model_name((buffer + index), config_hdl->model_name,
                                config_hdl->model_name_len);
    index += put_wsc_model_num((buffer + index), config_hdl->model_num,
                               config_hdl->model_num_len);
    index += put_wsc_serial_num((buffer + index), config_hdl->serial_num,
                                config_hdl->serial_num_len);
    index += put_wsc_prim_dev_type((buffer + index),
                                   config_hdl->prim_dev_cat_id);
    index += put_wsc_device_name((buffer + index), config_hdl->device_name,
                                 config_hdl->device_name_len);
    index += put_wsc_rf_bands((buffer + index), config_hdl->rf_bands);
    index += put_wsc_assoc_state((buffer + index), attr_hdl->assoc_state);
    index += put_wsc_config_error((buffer + index), attr_hdl->config_error);
    index += put_wsc_os_ver((buffer + index), config_hdl->os_version);
    index += put_wsc_wfa_vendor((buffer + index), &subelem_ptr, 1);

    /* Set the Message Start Pointer and Message Length */
    eap_tx_hdl->tx_m_msg_ptr = buffer;
    eap_tx_hdl->tx_m_msg_len = index;

    /* Increment EAP Id before transmission */
    eap_tx_hdl->tx_eap_id++;

    /* Send the WSC EAP Packet to Enrollee */
    return wps_tx_eap_wsc_pkt(wps_ctxt_hdl, eap_tx_hdl, EAP_CODE_REQUEST,
                              WPS_OP_CODE_WSC_MSG);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m4                                           */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-MSG  M4 to   */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX structure                     */
/*                     3) Handle to WPS EAP-RX structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     2) Compute Registrar PIN hash, R-Hash1 & R-Hash2      */
/*                     3) Add all the Mandatory Attributes required for M4   */
/*                        Message. Refer to WPS specification for details    */
/*                     4) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-MSG and send it to Enrollee                    */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_m4(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                     wps_eap_rx_t *eap_rx_hdl)
{
    UWORD16 index = 0;
    UWORD8  *buffer  = NULL;
    UWORD8  auth_attr[SHA_256_HASH_LEN] = {0};
    wps_config_t *config_hdl = wps_ctxt_hdl->wps_config_hdl;
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t   *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;
    wps_sec_key_t *sec_key_hdl = &(priv_hdl->olap_priv_key.secret_keys);
    wps_sub_elem_t *subelem_ptr = &(wps_ctxt_hdl->wps_config_hdl->version2);

    /* If Buffer pointer is NULL then exit */
    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /* Adjust the offset where WSC Message can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN +
              EAP_WPS_DATA_WO_LEN_OFFSET);

    /* Put all the Mandatory WSC TLV attributes for M4 Message */
    index += put_wsc_version(buffer + index);
    index += put_wsc_msg_type((buffer + index), WPS_MSG_M4);
    index += put_wsc_enr_nonce((buffer + index), attr_hdl->enr_nonce);
    index += wps_gen_pin_hash(config_hdl->prot_type, config_hdl->dev_pin,
                              config_hdl->dev_pin_len, (buffer + index),
                              WPS_ATTR_R_HASH1, WPS_ATTR_R_HASH2,
                              sec_key_hdl->auth_key, sec_key_hdl->psk1,
                              sec_key_hdl->psk2, attr_hdl->snonce1,
                              attr_hdl->snonce2, attr_hdl->reg_pub_key.pk,
                              attr_hdl->enr_pub_key.pk);
    index += wps_gen_encr_settings((buffer + index),
                                   attr_hdl->snonce1,
                                   WPS_ATTR_R_SNONCE1,
                                   WPS_NONCE_LEN,
                                   BTRUE,
                                   sec_key_hdl->auth_key,
                                   sec_key_hdl->key_wrap_key);
    index += put_wsc_wfa_vendor((buffer + index), &subelem_ptr, 1);

    /*************************************************************************/
    /* Set the Message Start Pointer and Message Len before computing the    */
    /* Authenticator attribute                                               */
    /*************************************************************************/
    eap_tx_hdl->tx_m_msg_ptr = buffer;
    eap_tx_hdl->tx_m_msg_len = index;

    /* Compute the authenticator attribute */
    wps_compute_auth_attr(eap_rx_hdl->rx_m_msg_ptr, eap_rx_hdl->rx_m_msg_len,
                          eap_tx_hdl->tx_m_msg_ptr, eap_tx_hdl->tx_m_msg_len,
                          sec_key_hdl->auth_key, auth_attr);

    /* Insert the authenticator attribute at given index */
    index += put_wsc_auth_attr((buffer + index), auth_attr);

    /* Update the Length of WSC Message */
    eap_tx_hdl->tx_m_msg_len = index;

    /* Increment EAP Id before transmission */
    eap_tx_hdl->tx_eap_id++;

    /* Send the WSC EAP Packet to Enrollee */
    return wps_tx_eap_wsc_pkt(wps_ctxt_hdl, eap_tx_hdl, EAP_CODE_REQUEST,
                              WPS_OP_CODE_WSC_MSG);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m6                                           */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-MSG  M6 to   */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX structure                     */
/*                     3) Handle to WPS EAP-RX structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     2) Add all the Mandatory Attributes required for M6   */
/*                        Message. Refer to WPS specification for details    */
/*                     3) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-MSG and send it to Enrollee                    */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_m6(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                     wps_eap_rx_t *eap_rx_hdl)
{
    UWORD16 index = 0;
    UWORD8  *buffer  = NULL;
    UWORD8  auth_attr[SHA_256_HASH_LEN] = {0};
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t   *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;
    wps_sec_key_t *sec_key_hdl = &(priv_hdl->olap_priv_key.secret_keys);
    wps_sub_elem_t *subelem_ptr = &(wps_ctxt_hdl->wps_config_hdl->version2);

    /* If Buffer pointer is NULL then exit */
    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /* Adjust the offset where WSC Message can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN +
              EAP_WPS_DATA_WO_LEN_OFFSET);

    /* Put all the Mandatory WSC TLV attributes for M6 Message */
    index += put_wsc_version(buffer + index);
    index += put_wsc_msg_type((buffer + index), WPS_MSG_M6);
    index += put_wsc_enr_nonce((buffer + index), attr_hdl->enr_nonce);
    index += wps_gen_encr_settings((buffer + index),
                                   attr_hdl->snonce2, WPS_ATTR_R_SNONCE2,
                                   WPS_NONCE_LEN, BTRUE,
                                   sec_key_hdl->auth_key,
                                   sec_key_hdl->key_wrap_key);
    index += put_wsc_wfa_vendor((buffer + index), &subelem_ptr, 1);

    /*************************************************************************/
    /* Set the Message Start Pointer and Message Len before computing the    */
    /* Authenticator attribute                                               */
    /*************************************************************************/
    eap_tx_hdl->tx_m_msg_ptr = buffer;
    eap_tx_hdl->tx_m_msg_len = index;

    /* Compute the authenticator attribute */

    /* Compute the authenticator attribute */
    wps_compute_auth_attr(eap_rx_hdl->rx_m_msg_ptr, eap_rx_hdl->rx_m_msg_len,
                          eap_tx_hdl->tx_m_msg_ptr, eap_tx_hdl->tx_m_msg_len,
                          sec_key_hdl->auth_key, auth_attr);

    /* Insert the authenticator attribute at given index */
    index += put_wsc_auth_attr((buffer + index), auth_attr);

    /* Update the Length of WSC Message */
    eap_tx_hdl->tx_m_msg_len = index;

    /* Increment EAP Id before transmission */
    eap_tx_hdl->tx_eap_id++;

    /* Send the WSC EAP Packet to Enrollee */
    return wps_tx_eap_wsc_pkt(wps_ctxt_hdl, eap_tx_hdl, EAP_CODE_REQUEST,
                              WPS_OP_CODE_WSC_MSG);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_m8                                           */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-MSG  M8 to   */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX structure                     */
/*                     3) Handle to WPS EAP-RX structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     3) Add all the Mandatory Attributes required for M8   */
/*                        Message. Refer to WPS specification for details    */
/*                     4) For Credential Attribute all the valid Credentials */
/*                        of the AP are added                                */
/*                     5) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-MSG and send it to Enrollee                    */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_m8(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                     wps_eap_rx_t *eap_rx_hdl)
{
    UWORD8  cred_index = 0;
    UWORD8  num_ap_cred = 0;
    UWORD16 index = 0;
    UWORD16 cred_len = 0;
    UWORD8  auth_attr[SHA_256_HASH_LEN] = {0};
    wps_cred_t wps_cred = {0};
    UWORD8  *buffer    = NULL;
    UWORD8  *cred_buf  = NULL;
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t   *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;
    wps_sec_key_t *sec_key_hdl = &(priv_hdl->olap_priv_key.secret_keys);
    wps_sub_elem_t *subelem_ptr = &(wps_ctxt_hdl->wps_config_hdl->version2);

    /* If Buffer pointer is NULL then exit */
    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /* Adjust the offset where WSC Message can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN +
              EAP_WPS_DATA_WO_LEN_OFFSET);

    /* Put all the Mandatory WSC TLV attributes for M8 Message */
    index += put_wsc_version(buffer + index);
    index += put_wsc_msg_type((buffer + index), WPS_MSG_M8);
    index += put_wsc_enr_nonce((buffer + index), attr_hdl->enr_nonce);

    /*************************************************************************/
    /* Set the offset for putting the credential such that it puts at Data   */
    /* portion of the Encrypted Settings Attribute                           */
    /*************************************************************************/
    cred_buf = (buffer + index + WPS_ENCR_SET_DATA_OFFSET);

    /* Get the number of credential present in the AP */
    num_ap_cred = wps_get_num_cred_ap();

    /* Add all the valid credentials at specified offset in transmit buffer  */
    for(cred_index = 0; cred_index < num_ap_cred; cred_index++)
    {
        memset(&wps_cred, 0, sizeof(wps_cred_t));

        /* Get the current Credential of the AP */
        if(BTRUE == wps_get_cur_cred_ap(&wps_cred, cred_index))
        {
            /*****************************************************************/
            /* Put the WPS Credential of the AP in transmit buffer only if it*/
            /* is a valid credential                                         */
            /*****************************************************************/
            cred_len += wps_gen_cred_attr(&wps_cred, (cred_buf + cred_len),
                                          num_ap_cred);
        }
    }

    /*************************************************************************/
    /* Generate and put the Encrypted Setting Attribute for the Credential   */
    /* put in the TX buffer                                                  */
    /*************************************************************************/
    index += wps_gen_encr_settings((buffer + index), cred_buf,
                                   WPS_ATTR_CREDENTIAL, cred_len,
                                   BFALSE, sec_key_hdl->auth_key,
                                   sec_key_hdl->key_wrap_key);
    index += put_wsc_wfa_vendor((buffer + index), &subelem_ptr, 1);

    /*************************************************************************/
    /* Set the Message Start Pointer and Message Length before computing the */
    /* Authenticator attribute                                               */
    /*************************************************************************/
    eap_tx_hdl->tx_m_msg_ptr = buffer;
    eap_tx_hdl->tx_m_msg_len = index;

    /* Compute the authenticator attribute */
    wps_compute_auth_attr(eap_rx_hdl->rx_m_msg_ptr, eap_rx_hdl->rx_m_msg_len,
                          eap_tx_hdl->tx_m_msg_ptr, eap_tx_hdl->tx_m_msg_len,
                          sec_key_hdl->auth_key, auth_attr);

    /* Insert the authenticator attribute at given index */
    index += put_wsc_auth_attr((buffer + index), auth_attr);

    /* Update the Length of WSC Message */
    eap_tx_hdl->tx_m_msg_len = index;

    /* Increment EAP Id before transmission */
    eap_tx_hdl->tx_eap_id++;

    /* Send the WSC EAP Packet to Enrollee */
    return wps_tx_eap_wsc_pkt(wps_ctxt_hdl, eap_tx_hdl, EAP_CODE_REQUEST,
                              WPS_OP_CODE_WSC_MSG);

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_wsc_frag_ack                                 */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-FRAG-ACK     */
/*                     frame to the enrollee                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     2) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-FRAG-ACK and send it to Enrollee               */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
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

STATUS_T wps_send_wsc_frag_ack(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl)
{
    UWORD8 *buffer = NULL;
    UWORD8 *tx_wlan_frm_buff = NULL;
    UWORD8  eap_id   = 0;
    UWORD16 eap_len  = 0;

    /* Allocate a buffer for Sending WSC Frag ACK */
    tx_wlan_frm_buff = wps_pkt_mem_alloc(WPS_EAP_WSC_FRAG_ACK_MSG_LEN);

    if(tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Increment EAP Id before transmission */
    eap_tx_hdl->tx_eap_id++;

    /* Adjust the offset where the 802.1X packet can be copied */
    buffer  = (tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN);
    eap_id  = eap_tx_hdl->tx_eap_id;
    eap_len = (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);

    /* Prepare 802.1X EAP WSC header for WSC-MSG */
    wps_prep_1x_eap_wsc_header(buffer, EAP_CODE_REQUEST, eap_id, eap_len,
                               WPS_OP_CODE_WSC_FRAG_ACK, 0, 0);

    /* Schedule the EAPOL frame for transmission */
    if(BFALSE == send_eapol(wps_ctxt_hdl->wps_config_hdl->mac_addr,
                            tx_wlan_frm_buff,
                            (eap_len + WPS_EAPOL_1X_HDR_LEN), BFALSE))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    return SUCCESS;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_wsc_nack                                     */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-NACK Message */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     3) Add all the Mandatory Attributes required for NACK */
/*                        Message. Refer to WPS specification for details    */
/*                     4) Add EAPOL-SNAP, 802.1X and EAP WSC header for      */
/*                        WSC-NACK and send it to Enrollee                   */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_wsc_nack(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl)
{
    UWORD16 index = 0;
    UWORD8 *buffer = NULL;
    wps_attr_t *attr_hdl = wps_ctxt_hdl->wps_attr_hdl;
    wps_sub_elem_t *subelem_ptr = &(wps_ctxt_hdl->wps_config_hdl->version2);

    /* If Buffer pointer is NULL then exit */
    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        wps_handle_sys_err_reg(NO_SHRED_MEM);
        return FAILURE;
    }

    /* Adjust the offset where WSC Message can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN +
              EAP_WPS_DATA_WO_LEN_OFFSET);

    /* WSC NACK cannot be sent with NoError as value for configuration error */
    /* attribute                                                             */
    if(WPS_NO_ERROR == attr_hdl->config_error)
        attr_hdl->config_error = DEVICE_BUSY;

    /* Put all the Mandatory WSC TLV attributes for NACK Message */
    index += put_wsc_version(buffer + index);
    index += put_wsc_msg_type((buffer + index), WPS_MSG_WSC_NACK);
    index += put_wsc_enr_nonce((buffer + index), attr_hdl->enr_nonce);
    index += put_wsc_reg_nonce((buffer + index), attr_hdl->reg_nonce);
    index += put_wsc_config_error((buffer + index), attr_hdl->config_error);
    index += put_wsc_wfa_vendor((buffer + index), &subelem_ptr, 1);

    /* Update the Message Pointer and Message Length */
    eap_tx_hdl->tx_m_msg_ptr = buffer;
    eap_tx_hdl->tx_m_msg_len = index;

    /* Update EAP-Identity Count */
    eap_tx_hdl->tx_eap_id++;

    /* Send the WSC EAP Packet to Enrollee */
    return wps_tx_eap_wsc_pkt(wps_ctxt_hdl, eap_tx_hdl, EAP_CODE_REQUEST,
                              WPS_OP_CODE_WSC_NACK);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_send_eap_fail                                     */
/*                                                                           */
/*  Description      : This function prepares and sends EAP Fail Frame to    */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS EAP-TX structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the WLAN TX buffer is not available then raise  */
/*                        System Error for the same and return FAILURE       */
/*                     2) Add EAPOL-SNAP, 802.1X and EAP header with EAP     */
/*                        Code set to EAP FAILURE and send it to Enrollee    */
/*                                                                           */
/*  Outputs          : Status of TX of the packet                            */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_send_eap_fail(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl)
{
    UWORD8 *buffer = NULL;

    if(eap_tx_hdl->tx_wlan_frm_buff == NULL)
    {
        /* Allocate a buffer for Sending EAP FAIL Frame */
        if(NULL == (eap_tx_hdl->tx_wlan_frm_buff =
                    wps_pkt_mem_alloc(MAX_WPS_WLAN_FRAME_LEN)))
            wps_handle_sys_err_reg(NO_SHRED_MEM);
            return FAILURE;
    }

    /* Adjust the offset where the 802.1X packet can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN);

    /* Add 802.1X header */
    wps_prep_1x_header(buffer, ONE_X_PKT_TYPE_EAP, EAP_HEADER_LEN);

    /* Update EAP-Identity Count */
    eap_tx_hdl->tx_eap_id++;

    /* Prepare EAP-Header */
    wps_prep_eap_header(buffer, EAP_CODE_FAILURE, eap_tx_hdl->tx_eap_id,
                        EAP_HEADER_LEN);

    /* Schedule the EAPOL frame for transmission */
    if(BFALSE == send_eapol(wps_ctxt_hdl->wps_config_hdl->mac_addr,
                            eap_tx_hdl->tx_wlan_frm_buff,
                           (WPS_EAPOL_1X_HDR_LEN + EAP_HEADER_LEN),
                            BFALSE))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_cred_attr                                     */
/*                                                                           */
/*  Description      : This function puts the credentials attribute at given */
/*                     memory location                                       */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS Credential Structure                */
/*                     2) Pointer to memeory location where the attribute has*/
/*                        to be put                                          */
/*                     3) Number of Credentials                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function puts the WPS Credential attribute at    */
/*                     given memory location as per the WPS Specifications   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : Length of the WPS Credential attribute put            */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         28 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static UWORD16 wps_gen_cred_attr(wps_cred_t *wps_cred, UWORD8 *buffer,
                                 UWORD8 num_cred)
{
    UWORD16 index = 0;
    UWORD8  i = 0;
    wps_net_key_t *net_key_info = wps_cred->net_key_info;

    /* If buffer is NULL then return */
    if(NULL == buffer)
        return 0;

    /* Put Credential Attribute ID and skip length field as it is unknown    */
    PUT_U16_BE((buffer + index), WPS_ATTR_CREDENTIAL);
    index += 4;

    /* Put the Network Index Attribute */
    index += put_wsc_net_index((buffer + index), 1);

    /* Put SSID, Authentication Type and Encryption Type attributes          */
    index += put_wsc_ssid((buffer + index), wps_cred->ssid,
                          wps_cred->ssid_len);
    index += put_wsc_auth_type((buffer + index), wps_cred->auth_type);
    index += put_wsc_encr_type((buffer + index), wps_cred->encr_type);

    /*************************************************************************/
    /* Put the Network Key Index, Network Key and MAC Address attributes     */
    /* based on the number of network keys                                   */
    /*************************************************************************/
    while(1)
    {
        /*********************************************************************/
        /* Add Network Key Index Attribute only if the number of keys is     */
        /* greater than 1 or in case of WEP Encryption                       */
        /*********************************************************************/
        if((1 < wps_cred->num_key) || (wps_cred->encr_type & WPS_WEP))
        {
            index += put_wsc_net_key_index((buffer + index),
                                           (net_key_info[i].net_key_index+1));
        }

        index += put_wsc_net_key((buffer + index), net_key_info[i].net_key,
                                 net_key_info[i].net_key_len);
        index += put_wsc_mac_addr((buffer + index), net_key_info[i].mac_addr);
        i++;

        /*********************************************************************/
        /* Break if Max number of keys per credential is reached or if all   */
        /* the required number of keys have been put                         */
        /*********************************************************************/
        if((MAX_NW_KEY_PER_CRED <= i) || (i >= wps_cred->num_key))
            break;
    }

    /* Put the Length of Credential Attribute */
    PUT_U16_BE((buffer + 2), (index - 4));

    return index;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_tx_eap_wsc_pkt                                    */
/*                                                                           */
/*  Description      : This function prepares and sends EAP WSC-MSG    to    */
/*                     the enrollee                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to EAP-TX structure                        */
/*                     3) EAP Code                                           */
/*                     4) WSC Opcode                                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function adds 802.1X and EAP WSC header to packet*/
/*                     to be transmitted and sends it to the Enrollee        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful transmission of the frame     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         25 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_tx_eap_wsc_pkt(wps_t *wps_ctxt_hdl,
                                   wps_eap_tx_t *eap_tx_hdl, UWORD8 eap_code,
                                   UWORD8 wsc_opcode)
{
    UWORD8  eap_id   = 0;
    UWORD16 eap_len  = 0;
    UWORD8  *buffer  = NULL;

    /* If the WLAN buffer is not allocated then exit */
    if(NULL == eap_tx_hdl->tx_wlan_frm_buff)
    {
        /* Handling of System Error TBD */
        return FAILURE;
    }

    /* Adjust the offset where the 802.1X packet can be copied */
    buffer = (eap_tx_hdl->tx_wlan_frm_buff + WPS_MAX_MAC_HDR_LEN);

    eap_id = eap_tx_hdl->tx_eap_id;

    eap_len = eap_tx_hdl->tx_m_msg_len + (EAP_WPS_DATA_WO_LEN_OFFSET -
                                          EAP_HEADER_OFFSET);

    /* Prepare 802.1X EAP WSC header for WSC-MSG */
    wps_prep_1x_eap_wsc_header(buffer, eap_code, eap_id, eap_len, wsc_opcode,
                               0, 0);

    /* Schedule the EAPOL frame for transmission */
    if(BFALSE == send_eapol(wps_ctxt_hdl->wps_config_hdl->mac_addr,
                            eap_tx_hdl->tx_wlan_frm_buff,
                            (eap_len + WPS_EAPOL_1X_HDR_LEN), BFALSE))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_retry                                  */
/*                                                                           */
/*  Description      : This function handles retransmission of EAP WSC frames*/
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) EAP TX handle                                      */
/*                     3) Length of the retry frame                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the retry limit is reached then raise system    */
/*                        error and return FAILURE                           */
/*                     2) If the Transmit buffer is not available then return*/
/*                        FAILURE                                            */
/*                     3) Update the retry cound and send the frame          */
/*                     4) Start the EAP Message timer and exit               */
/*                                                                           */
/*  Outputs          : Status of retransmission                              */
/*                                                                           */
/*  Returns          : SUCCESS - If retry is successful                      */
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

STATUS_T wps_handle_eap_retry(wps_t *wps_ctxt_hdl, wps_eap_tx_t *eap_tx_hdl,
                              UWORD16 length)
{
    /* Check if the EAP Retry Limit has been reached */
    if(MAX_EAP_MSG_RETRY_LIMIT <= eap_tx_hdl->eap_msg_retry_cnt)
    {
        /*********************************************************************/
        /* If WSC_NACK has been transmitted to the Enrollee, then add it to  */
        /* the failure list                                                  */
        /*********************************************************************/
        if(NACK_ST == wps_ctxt_hdl->wps_reg_hdl->msg_state)
            wps_update_failed_enr_list(wps_ctxt_hdl->wps_config_hdl->mac_addr);

        wps_send_eap_fail(wps_ctxt_hdl, eap_tx_hdl);
        send_wps_status(WPS_REG_EAP_MSG_RETRY_LIMIT, NULL, 0);
        sys_stop_wps_reg(AUTH_1X_FAIL,
                         wps_ctxt_hdl->wps_config_hdl->mac_addr,
                         wps_ctxt_hdl->wps_reg_hdl->start_req);

        return FAILURE;
    }

    /* If the transmit buffer is not available then raise system error */
    if(NULL == eap_tx_hdl->tx_wlan_frm_buff)
    {
        send_wps_status(WPS_REG_EAP_BUFFER_NULL, NULL, 0);
        wps_handle_sys_err_reg(NO_SHRED_MEM);

        return FAILURE;
    }

    /* Increment number of users for this buffer as this will not be freed   */
    wps_add_mem_users(eap_tx_hdl->tx_wlan_frm_buff, 1);

    /* Incretment the retry count */
    eap_tx_hdl->eap_msg_retry_cnt++;

    /* Schedule the EAPOL frame for transmission */
    if(BFALSE == send_eapol(wps_ctxt_hdl->wps_config_hdl->mac_addr,
                            eap_tx_hdl->tx_wlan_frm_buff, length, BFALSE))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    /* Restart EAP message timer */
    return wps_start_eap_msg_timer(wps_ctxt_hdl);
}

#endif /* INT_WPS_REG_SUPP */
