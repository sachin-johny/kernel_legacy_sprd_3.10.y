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
/*  File Name         : wpse_prot_rx.c                                       */
/*                                                                           */
/*  Description       : This file contains receiver functions to related to  */
/*                      the WPS protocol                                     */
/*                                                                           */
/*  List of Functions : wps_handle_sys_err                                   */
/*                      wps_loc_start_scan                                   */
/*                      wps_walk_timeout                                     */
/*                      wps_delete_timers                                    */
/*                      wps_free_all_memory                                  */
/*                      wps_eap_reg_start                                    */
/*                      wps_process_rx_frag_frame                            */
/*                      wps_process_eap_msg                                  */
/*                      wps_process_rx_eap_msg                               */
/*                      wps_proc_rx_m_message                                */
/*                      wps_proc_rx_m2                                       */
/*                      wps_proc_rx_m2d                                      */
/*                      wps_proc_rx_m4                                       */
/*                      wps_proc_rx_m6                                       */
/*                      wps_proc_rx_m8                                       */
/*                      wps_handle_msg_failure                               */
/*                      wps_proc_wsc_nack                                    */
/*                      wps_decrypt_encrypted_setting                        */
/*                      wps_verify_key_wrap_auth                             */
/*                      wps_verify_authen                                    */
/*                      wps_process_p2p_prob_rsp_enr                             */
/*                      wps_get_config_method                        */
/*                      wps_process_p2p_prob_req_enr                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_ENR_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"
#include "wpse_prot_rx.h"
#include "wpse_hmac_sha256.h"
#include "trout_trace.h"
/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
/*****************************************************************************/
/* 802.1x header format                                                      */
/* ---------------------------------------------------------------------     */
/* | SNAP    | Version | Type    | Body Length | Body                  |     */
/* ---------------------------------------------------------------------     */
/* | 8 octet | 1 octet | 1 octet | 2 octet     | (Length - 4) octets   |     */
/* ---------------------------------------------------------------------     */
/*****************************************************************************/

const UWORD8 wps_1x_header[ONE_X_PCK_TYPE_OFFSET] =
{
    0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E, EAPOL_VERSION_01
};

const UWORD8 eap_wps_type_vid_vt[EAP_WPS_TYPE_VID_VT_LEN] =
{
    0xFE, 0x00, 0x37, 0x2A, 0x00, 0x00, 0x00, 0x01
};

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
static void wps_handle_msg_failure(wps_enrollee_t* wps_enrollee,
                                  WPS_STATUS_T err_code, UWORD16 config_error);
static STATUS_T wps_process_rx_eap_msg(wps_enrollee_t* wps_enrollee,
                                       UWORD8 op_code);
static STATUS_T wps_proc_rx_m_message(wps_enrollee_t* wps_enrollee);
static STATUS_T wps_proc_wsc_nack(wps_enrollee_t* wps_enrollee);
static STATUS_T wps_proc_rx_m2(wps_enrollee_t* wps_enrollee,
                               wps_rx_attri_t* rx_attr);
static STATUS_T wps_proc_rx_m2d(wps_enrollee_t* wps_enrollee,
                               wps_rx_attri_t* rx_attr);
static STATUS_T wps_proc_rx_m4(wps_enrollee_t* wps_enrollee,
                               wps_rx_attri_t* rx_attr);
static STATUS_T wps_proc_rx_m6(wps_enrollee_t* wps_enrollee,
                               wps_rx_attri_t* rx_attr);
static STATUS_T wps_proc_rx_m8(wps_enrollee_t* wps_enrollee,
                               wps_rx_attri_t* rx_attr);
static STATUS_T wps_verify_authen(wps_enrollee_t* wps_enrollee);
static WORD16  wps_decrypt_encrypted_setting(wps_enrollee_t* wps_enrollee,
                                       UWORD8* encry_data,
                                       UWORD16 encry_data_len,
                                       UWORD8* decry_data);
static STATUS_T wps_verify_key_wrap_auth(wps_enrollee_t* wps_enrollee,
                                       UWORD8* data,
                                       UWORD16 data_len);
static STATUS_T wps_verify_r_snonce(wps_enrollee_t* wps_enrollee,
                                    UWORD8* r_nonce_ptr, UWORD8 nonce_cnt);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_sys_err                                    */
/*                                                                           */
/*  Description      : This function handles the system error in WPS Enrollee*/
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function stops the WPS Enrollee processing and   */
/*                     inform System about the system error                  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_handle_sys_err(wps_enrollee_t* wps_enrollee)
{
        wps_stop_enrollee(wps_enrollee);
        end_wps_enrollee(ERR_SYSTEM);
        return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_loc_start_scan                                    */
/*                                                                           */
/*  Description      : This function starts the MAC Scan in WPS mode         */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function frees all the memory used by WPS        */
/*                     enrollee, issues a scan request                       */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
//#pragma Otime
void wps_loc_start_scan(wps_enrollee_t* wps_enrollee)
{
    /* If WalkTimer has to be processed then stop the Enrollee */
    if(BTRUE == wps_enrollee->process_walk_to)
    {
        wps_stop_enrollee(wps_enrollee);
        end_wps_enrollee(ERR_WALK_TIMEOUT);
        return;
    }

    /*************************************************************************/
    /* Free all the memory allocated                                         */
    /*************************************************************************/
    wps_free_all_memory(wps_enrollee);
    wps_enrollee->state = IN_SCAN;
    wps_enrollee->req_type = ENROLLEE_INFO_ONLY;
    memset(&(wps_enrollee->sel_ap_info), 0, sizeof(wps_enrollee->sel_ap_info));

#ifdef WPS_EXT_REG_SEL_REG_BUG_FIX
    wps_enrollee->ignore_sel_reg_field = BFALSE;
#endif /* WPS_EXT_REG_SEL_REG_BUG_FIX */

    wps_enrollee->scan_cnt = 0;
    start_wps_scan(BTRUE);
    return;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_walk_timeout                                      */
/*                                                                           */
/*  Description      : This function handles the Walk Timeout event          */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function is called if Enrollee couldn't find     */
/*                     any Registrar or AP or it can not completes the WPS   */
/*                     protocol within the timeout                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         02 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_walk_timeout(wps_enrollee_t* wps_enrollee)
{
    /* Do not process Walk Timer event when in WPS registration mode  */
    if(IN_SCAN != wps_enrollee->state)
    {
        wps_enrollee->process_walk_to = BTRUE;
        return;
    }

    wps_stop_enrollee(wps_enrollee);
    end_wps_enrollee(ERR_WALK_TIMEOUT);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_delete_timers                                     */
/*                                                                           */
/*  Description      : This function deletes all the WPS timers except the   */
/*                     walk timer                                            */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function stops and delete all the WPS timers     */
/*                     except the walk timer                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         10 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_delete_timers(wps_enrollee_t* wps_enrollee)
{

    /*************************************************************************/
    /* Stop and free all the timers except the walk timer                    */
    /*************************************************************************/
    if(NULL !=wps_enrollee->reg_msg_timer)
    {
        //wps_stop_alarm(wps_enrollee->reg_msg_timer);
        wps_delete_alarm(&(wps_enrollee->reg_msg_timer));
        wps_enrollee->reg_msg_timer = NULL;
    }
    if(NULL !=wps_enrollee->reg_prot_timer)
    {
        //wps_stop_alarm(wps_enrollee->reg_prot_timer);
        wps_delete_alarm(&(wps_enrollee->reg_prot_timer));
        wps_enrollee->reg_prot_timer = NULL;
    }

    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_free_all_memory                                   */
/*                                                                           */
/*  Description      : This function frees all the dynamically allocated     */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function frees all the dynamically allocated     */
/*                     memory                                                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         10 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_free_all_memory(wps_enrollee_t* wps_enrollee)
{
    /*************************************************************************/
    /* Free all the memory allocated                                         */
    /*************************************************************************/
    /*************************************************************************/
    /*If Buffer is locally allocated then free it                            */
    /*************************************************************************/
    if((wps_enrollee->rx_m_msg_ptr == wps_enrollee->rx_eap_msg) &&
        (NULL != wps_enrollee->rx_m_msg_ptr))
    {
        wps_frame_mem_free(wps_enrollee->rx_eap_msg);
        wps_enrollee->rx_eap_msg = NULL;
    }
    if( NULL != wps_enrollee->tx_eap_msg)
    {
        wps_frame_mem_free(wps_enrollee->tx_eap_msg);
        wps_enrollee->tx_eap_msg = NULL;
    }
    if( NULL != wps_enrollee->tx_wlan_frm_buff)
    {
        wps_frame_mem_free(wps_enrollee->tx_wlan_frm_buff);
        wps_enrollee->tx_eap_msg = NULL;
    }
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_eap_reg_start                                     */
/*                                                                           */
/*  Description      : This function starts the EAP based WPS Enrollee       */
/*                     Registration protocol                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function starts the EAP based WPS Enrollee       */
/*                     Registration protocol. It changes the WPS enrollee    */
/*                     state variable accordingly.                           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         03 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_eap_reg_start(wps_enrollee_t* wps_enrollee)
{
    wps_enrollee->state = IN_REG_PROT;
    wps_enrollee->reg_state = EAPOL_START_ST;
    if( NULL == (wps_enrollee->reg_prot_timer = wps_create_alarm(
        wps_enrollee->config_ptr->timer_cb_fn_ptr, WPS_REG_TIMEOUT, NULL))) //Hugh: fixme.
    {
        wps_handle_sys_err(wps_enrollee);
        return;
    }
    wps_start_alarm(wps_enrollee->reg_prot_timer, WPS_EAP_REG_TIME);
    wps_enrollee->eapol_start_cnt = 0;
    if(FAILURE == wps_send_eapol_start(wps_enrollee))
    {
        return;
    }
    wps_start_reg_msg_timer(wps_enrollee);
    return;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_rx_frag_frame                             */
/*                                                                           */
/*  Description      : This function handles the received fragmented frame   */
/*                     Registration protocol                                 */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to location in the received EAP Packet     */
/*                        where Mx is stored                                 */
/*                     3) Length of the received EAP message                 */
/*                     4) Flags field from the WPS-EAP message received.     */
/*                     5) op_code field from the EAP message received.       */
/*                     6) Length of the current Mx message fragment          */
/*                                                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function does the checks on the received EAP     */
/*                     fragment and if check passes, it appends the received */
/*                     frame to the buffer wps_enrollee->rx_eap_msg.         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_process_rx_frag_frame(wps_enrollee_t* wps_enrollee,
                               UWORD8* msg_pos,
                               UWORD16 message_length,
                               UWORD8 flags,
                               UWORD8 op_code,
                               UWORD16 frag_len)
{
    if(NULL == wps_enrollee->rx_eap_msg)
    {
        if((0 == (WPS_EAP_MSG_FLAG_LF & flags)) ||
            (MAX_WPS_WSC_EAP_MSG_LEN < message_length))
        {
            return (FAILURE);
        }
        wps_enrollee->rx_eap_msg = wps_frame_mem_alloc(message_length);
        if(NULL == wps_enrollee->rx_eap_msg)
        {
            return (FAILURE);
        }
        wps_enrollee->rx_eap_msg_buff_len = message_length;
        wps_enrollee->rx_m_msg_len = message_length;
        wps_enrollee->rx_m_msg_ptr = wps_enrollee->rx_eap_msg;
        memcpy(wps_enrollee->rx_eap_msg, msg_pos, frag_len);
        wps_enrollee->rx_eap_msg_used_len = frag_len;
        wps_enrollee->op_code = op_code;
        return(wps_send_frag_ack(wps_enrollee));
    }
    if((WPS_EAP_MSG_FLAG_LF & flags) ||
        (frag_len > wps_enrollee->rx_m_msg_len  -
        wps_enrollee->rx_eap_msg_used_len) ||
        (op_code != wps_enrollee->op_code))
    {
        return (FAILURE);
    }
    memcpy(wps_enrollee->rx_eap_msg + wps_enrollee->rx_eap_msg_used_len,
        msg_pos, frag_len);
    wps_enrollee->rx_eap_msg_used_len += frag_len;
    if(WPS_EAP_MSG_FLAG_MF & flags)
    {
        return(wps_send_frag_ack(wps_enrollee));
    }
    return (SUCCESS);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_eap_msg                                   */
/*                                                                           */
/*  Description      : This function process the received EAPOL frame        */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to received EAPOL packet buffer            */
/*                     3) Length of the received EAPOL packet                */
/*                                                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function is called by system software whenever it*/
/*                     receives an EAPOL packet (without SNAP header) from   */
/*                     the wireless LAN. Wireless LAN device should check the*/
/*                     SNAP header before calling this function. If WPS      */
/*                     Enrollee is in process of doing EAP based Registration*/
/*                     protocol then based on wps_enrollee->reg_state,       */
/*                     appropriate EAP message processing function is called */
/*                     otherwise packet is dropped..                         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_process_eap_msg(wps_enrollee_t* wps_enrollee,
                               UWORD8* msg,
                               UWORD16 mes_len)
{
    UWORD8* wps_msg_data_ptr;
    UWORD16 length;
    UWORD16 wps_msg_data_len = 0;
    WORD16  frag_len;
    UWORD8  op_code, flags;

    /*************************************************************************/
    /* If not in Registration protocol or if note sufficient length of the   */
    /* the packet. then discard the packet                                   */
    /*************************************************************************/
    if( (IN_REG_PROT != wps_enrollee->state) ||
        ((EAP_HEADER_OFFSET + EAP_HEADER_LEN - 1) >     mes_len))
    {
        return;
    }

    /*************************************************************************/
    /* Check 802.1x header if check fails then discard the packet.Also check */
    /* for expected EAP code and valid EAP length                            */
    /*************************************************************************/
    if(0 != memcmp(msg+ONE_X_SNAP_ETH_TYPE_OFFSET,wps_1x_header,
        ONE_X_SNAP_ETH_TYPE_LEN))
    {
        return;
    }
    length = GET_U16_BE((msg + EAP_LENGTH_OFFSET));
    /*************************************************************************/
    /* As some APs wrongly send the Protocol veraion value as 2, to          */
    /* interoperate with such APs except 2 as valid version value.           */
    /*************************************************************************/
    if(((EAPOL_VERSION_01 != msg[ONE_X_PROT_VER_OFFSET]) &&
        (EAPOL_VERSION_02 != msg[ONE_X_PROT_VER_OFFSET])) ||
        (ONE_X_PCK_TYPE_EAP != msg[ONE_X_PCK_TYPE_OFFSET]) ||
        (EAP_HEADER_LEN > length) ||
        ((length + ONE_X_HEADER_LEN) > mes_len) ||
        ((EAP_CODE_REQUEST != msg[EAP_CODE_OFFSET]) &&
        (EAP_CODE_FAILURE != msg[EAP_CODE_OFFSET])))
    {
        return;
    }

    /*************************************************************************/
    /* Check if EAP FAIL received. If so then if protocol is over, return    */
    /* SUCCESS to MAC else return FAILURE; add the current Registrar to the  */
    /* list of discarded Registrar and continue the protocol                 */
    /*************************************************************************/
    if(EAP_CODE_FAILURE == msg[EAP_CODE_OFFSET])
    {
        if((MSG_ST == wps_enrollee->reg_state) &&
            (DONE_ST == wps_enrollee->msg_st))
        {
            wps_stop_enrollee(wps_enrollee);
            end_wps_enrollee(REG_PROT_SUCC_COMP);
            return;
        }
        wps_enrollee->assoc_state = WPS_CONFIGURATION_FAILURE;
        send_wps_status(ERR_REC_FAIL,NULL,0);
        wps_add_discarded_reg_ap(wps_enrollee,
            wps_enrollee->sel_ap_info.bssid);
        wps_delete_timers(wps_enrollee);
        wps_loc_start_scan(wps_enrollee);
        return;
    }

    if(EAP_HEADER_LEN >= length)
    {
        return;
    }
    /*************************************************************************/
    /* Check if EAP Identity. If so then if its expected then send the       */
    /* Response identity. If Response identity already send but have not yet */
    /* received WSC_START, the retransmit the Response Identity else ignore  */
    /* the message.                                                          */
    /*************************************************************************/
    if(EAP_TYPE_IDENTITY == msg[EAP_TYPE_OFFSET])
    {
        if(EAPOL_START_ST == wps_enrollee->reg_state)
        {
            wps_enrollee->reg_state = RES_ID_ST;
            wps_start_reg_msg_timer(wps_enrollee);
        }
        if(RES_ID_ST == wps_enrollee->reg_state)
        {
            wps_enrollee->eap_received_id =
                msg[EAP_IDENTIFIER_OFFSET];
            wps_send_eap_res_id(wps_enrollee);
        }
        return;
    }
    if((EAP_WPS_MSG_LEN_OFFSET - EAP_HEADER_OFFSET) > length)
    {
        return;
    }

    /*************************************************************************/
    /* Check for WPS EAP method type, Vendor ID and Vendor Type. If check    */
    /* fails ignore the message                                              */
    /*************************************************************************/
    if(0 != memcmp((msg+EAP_TYPE_OFFSET),eap_wps_type_vid_vt,
        EAP_WPS_TYPE_VID_VT_LEN))
    {
        return;
    }

    op_code = msg[EAP_WPS_OP_CODE_OFFSET];
    /*************************************************************************/
    /* Check for expected OP-codes, if check fails ignore message            */
    /*************************************************************************/
    if((WPS_OP_CODE_WSC_START != op_code) &&
        (WPS_OP_CODE_WSC_NACK != op_code) &&
        (WPS_OP_CODE_WSC_MSG != op_code) &&
        (WPS_OP_CODE_WSC_FRAG_ACK != op_code))
    {
        return;
    }
    flags   = msg[EAP_WPS_FLAGS_OFFSET];

    /*************************************************************************/
    /* Get the length of full frame (if first frame) and length of received  */
    /* fragment                                                              */
    /*************************************************************************/
    if(0 == (WPS_EAP_MSG_FLAG_LF & flags))
    {
        frag_len = length - (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);
        wps_msg_data_ptr = msg + EAP_WPS_DATA_WO_LEN_OFFSET;
        wps_msg_data_len = frag_len;
    }
    else
    {
        frag_len = length - (EAP_WPS_DATA_WI_LEN_OFFSET - EAP_HEADER_OFFSET);
        wps_msg_data_ptr = msg + EAP_WPS_DATA_WI_LEN_OFFSET;
        wps_msg_data_len = GET_U16_BE((msg+EAP_WPS_MSG_LEN_OFFSET));
    }
    if(0 > frag_len)
    {
        return;

    }

    /*************************************************************************/
    /* If waiting for WSC_FRAG_ACK and Op code received is not WSC_FRAG_ACK  */
    /* then ignore received messaage(assuming timer based re-transmission)   */
    /* else send the remaining message                                       */
    /*************************************************************************/
    if(WAIT_FACK_ST == wps_enrollee->reg_state)
    {
        if (WPS_OP_CODE_WSC_FRAG_ACK != op_code)
        {
            return;
        }
        wps_tx_eap_msg(wps_enrollee);
        return;
    }

    /*************************************************************************/
    /* Check if WSC_Start is received if so then depending on the            */
    /* registration protocol state do the processing                         */
    /*************************************************************************/
    if(WPS_OP_CODE_WSC_START == op_code)
    {
        /*********************************************************************/
        /* If response Identity was transmitted then transmit M1             */
        /* If M1 already transmitted then retransmit it else ignore message  */
        /*********************************************************************/
        if(RES_ID_ST == wps_enrollee->reg_state)
        {
            /*****************************************************************/
            /* If Buffer is locally allocated then free it                   */
            /*****************************************************************/
            if((wps_enrollee->rx_m_msg_ptr == wps_enrollee->rx_eap_msg) &&
                (NULL != wps_enrollee->rx_m_msg_ptr))
            {
                wps_frame_mem_free(wps_enrollee->rx_eap_msg);
            }
            wps_enrollee->rx_eap_msg = NULL;
            wps_enrollee->reg_state = MSG_ST;
            wps_enrollee->msg_st = M1_ST;
            wps_enrollee->eap_received_id = msg[EAP_IDENTIFIER_OFFSET];
            wps_send_m1(wps_enrollee);
            wps_start_reg_msg_timer(wps_enrollee);
            return;
        }
        else if((MSG_ST == wps_enrollee->reg_state) &&
            (M1_ST == wps_enrollee->msg_st))
        {
            wps_enrollee->tx_eap_txed_msg_len = 0;
            wps_tx_eap_msg(wps_enrollee);
        }
        return;
    } /* if(WPS_OP_CODE_WSC_START == op_code) */
    if(wps_enrollee->reg_state != MSG_ST)
    {
        return;
    }
    wps_enrollee->eap_received_id = msg[EAP_IDENTIFIER_OFFSET];


    /*************************************************************************/
    /* Check if the received frame fragment if so then store in local memory */
    /* if not then assign frame pointers to the received frame directly      */
    /*************************************************************************/
    if((WPS_EAP_MSG_FLAG_MF & flags) || (NULL != wps_enrollee->rx_eap_msg))
    {
        if( FAILURE == wps_process_rx_frag_frame(wps_enrollee,
            wps_msg_data_ptr, wps_msg_data_len, flags, op_code, frag_len))
        {
            return;
        };
        if(WPS_EAP_MSG_FLAG_MF & flags)
        {
            return;
        }
    }
    else
    {
        wps_enrollee->rx_eap_msg = msg + EAP_HEADER_OFFSET;
        wps_enrollee->rx_m_msg_len = wps_msg_data_len;
        wps_enrollee->rx_m_msg_ptr = wps_msg_data_ptr;
    }

    /*************************************************************************/
    /*Process the received EAP Frame                                         */
    /*************************************************************************/
    wps_process_rx_eap_msg(wps_enrollee, op_code);

    /*************************************************************************/
    /*If Buffer is locally allocated then free it                            */
    /*************************************************************************/
    /*************************************************************************/
    /*If Buffer is locally allocated then free it                            */
    /*************************************************************************/
    if((wps_enrollee->rx_m_msg_ptr == wps_enrollee->rx_eap_msg) &&
        (NULL != wps_enrollee->rx_m_msg_ptr))
    {
        wps_frame_mem_free(wps_enrollee->rx_eap_msg);
    }
    wps_enrollee->rx_eap_msg = NULL;
    wps_enrollee->rx_m_msg_ptr = NULL;
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_rx_eap_msg                                */
/*                                                                           */
/*  Description      : This function processes the received EAP frame        */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) op_code field from the EAP message received.       */
/*                                                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function processes the received EAP frame and    */
/*                     based on the Op-Code calls the appropriate function   */
/*                     to do further processing                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_process_rx_eap_msg(wps_enrollee_t* wps_enrollee,
                               UWORD8 op_code)
{
    switch(op_code)
    {
        case WPS_OP_CODE_WSC_MSG:
            return(wps_proc_rx_m_message(wps_enrollee));
        case WPS_OP_CODE_WSC_NACK:
            return(wps_proc_wsc_nack(wps_enrollee));
        default:
            return FAILURE;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_proc_rx_m_message                                 */
/*                                                                           */
/*  Description      : This function processes the received message of Type  */
/*                     WSC_MSG.                                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function parses the received message, do common  */
/*                     checks on the parsed fields and then call appropriate */
/*                     function to do further processing                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_proc_rx_m_message(wps_enrollee_t* wps_enrollee)
{
    /*************************************************************************/
    /*This structure stores all the required attributes                      */
    /*************************************************************************/
    wps_rx_attri_t rx_attr;
    wps_rx_attri_t *rx_attr_ptr_ptr[1];
    rx_attr_ptr_ptr[0] = &rx_attr;

    /*************************************************************************/
    /*Parse the received message and get all the required fields             */
    /*************************************************************************/
    if(1 != wps_parse_attri(wps_enrollee->rx_m_msg_ptr, rx_attr_ptr_ptr,
        wps_enrollee->rx_m_msg_len, 1, BFALSE))
    {
        return FAILURE;
    }
    /*************************************************************************/
    /* Do the checks common to all message i.e. version number, message type */
    /* and Enrollee Nonce check. if check fails return FAILURE               */
    /*************************************************************************/

    if((((NULL != rx_attr.version2)) &&
         (FAILURE == wps_check_ver2_supp(rx_attr.version2))) ||
        (NULL == rx_attr.msg_type) ||
        (NULL == rx_attr.enr_nonce))
    {
        return FAILURE;
    }
    if(0 != memcmp(wps_enrollee->enr_nonce, rx_attr.enr_nonce, WPS_NONCE_LEN))
    {
        return FAILURE;
    }
    /*************************************************************************/
    /* Based on Message type, call appropriate function. If unexpected       */
    /* message type then return failure                                      */
    /*************************************************************************/
    switch(*(rx_attr.msg_type))
    {
        case WPS_MSG_M2:
            return(wps_proc_rx_m2(wps_enrollee, &rx_attr));
        case WPS_MSG_M2D:
            return(wps_proc_rx_m2d(wps_enrollee, &rx_attr));
        case WPS_MSG_M4:
            return(wps_proc_rx_m4(wps_enrollee, &rx_attr));
        case WPS_MSG_M6:
            return(wps_proc_rx_m6(wps_enrollee, &rx_attr));
        case WPS_MSG_M8:
            return(wps_proc_rx_m8(wps_enrollee, &rx_attr));
        default:
            return FAILURE;
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_proc_rx_m2                                        */
/*                                                                           */
/*  Description      : This function processes the received message of type  */
/*                     M2.                                                   */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the parsed attribute memory             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Based on received attributes and current state, this  */
/*                     function either moves to next state (M3), retransmit  */
/*                     M1 or send NACK(in case of multiple Registration      */
/*                     Protocol) or return Failure. If in M1 state, this     */
/*                     function will keep looking for suitable registrar. if */
/*                     it finds one, it moves to next state i.e. M3. No Nack */
/*                     is sent as Enrollee will wait for the suitable        */
/*                     Registrar. If in M3 state, if the M2 received from    */
/*                     a different registrar then NACK with config error as  */
/*                     'Device Busy' is sent else M3 is retransmitted        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_proc_rx_m2(wps_enrollee_t* wps_enrollee, wps_rx_attri_t* rx_attr)
{
    /*************************************************************************/
    /* Check if Registrar Nonce, UUID-R, Public Key, Authentication Type     */
    /* Flags, Encryption Type Flags, Connection Type Flags, Config Methods   */
    /* and Device Password ID are present, if not return FAILURE             */
    /*************************************************************************/
    if((NULL == rx_attr->reg_nonce) ||
        (NULL == rx_attr->uuid_r) ||
        (NULL == rx_attr->pkr) ||
        (NULL == rx_attr->auth_type_flags) ||
        (NULL == rx_attr->encr_type_flags) ||
        (NULL == rx_attr->conn_type_flags) ||
        (NULL == rx_attr->config_meth) ||
        (NULL == rx_attr->dev_pass_id))
    {
        return FAILURE;
    }

    /*************************************************************************/
    /* If already in M3 state, i.e. valid M2 was received and M3 is already  */
    /* sent to a registrar, in that case if this M2 is from a different      */
    /* registrar then send NACK to this registrar with Device Busy as config */
    /* error else retransmit M3.                                             */
    /*************************************************************************/
    if(M3_ST == wps_enrollee->msg_st)
    {
        if(0 != memcmp(rx_attr->uuid_r, wps_enrollee->reg_uuid,WPS_UUID_LEN))
        {
            return(wps_send_nack_m2(wps_enrollee, rx_attr, DEVICE_BUSY));
        }
        else
        {
            wps_enrollee->tx_eap_txed_msg_len = 0;
            wps_tx_eap_msg(wps_enrollee);
            return SUCCESS;
        }
    }
    if(M1_ST != wps_enrollee->msg_st)
    {
        return FAILURE;
    }

    if(BFALSE == wps_enrollee->sel_ap_info.sel_reg)
    {
        wps_enrollee->sel_ap_info.sel_reg = BTRUE;
        if(((PBC == wps_enrollee->config_ptr->prot_type) &&
            (DEV_PASS_ID_PUSHBUTT != (GET_U16_BE(rx_attr->dev_pass_id)))) ||
            ((PIN == wps_enrollee->config_ptr->prot_type) &&
            (wps_enrollee->config_ptr->dev_pass_id !=
            (GET_U16_BE(rx_attr->dev_pass_id)))))
        {
            send_wps_status(ERR_DIFF_PASS_ID_M2, NULL, 0);
            return FAILURE;
        }
    } /* if(BFALSE == wps_enrollee->sel_ap_info.sel_reg) */

    /*************************************************************************/
    /* If Version2 subelement is included and Version2 check passes then set */
    /* version2 capable AP flag                                              */
    /*************************************************************************/
    if((NULL != rx_attr->version2) &&
       (BTRUE == wps_check_ver2_supp(rx_attr->version2)))
    {
        wps_enrollee->ver2_cap_ap = BTRUE;
    }
    else
    {
        wps_enrollee->ver2_cap_ap = BFALSE;
    }

    /*************************************************************************/
    /* Check for common capabilities between received and configured values  */
    /* of "Authentication Type Flags", "Encryption Type Flags" and           */
    /* "Connection Type Flags". If there is no common capability in any one  */
    /* of these then indicate wrong M2 and remove the Registrar from selected*/
    /* registrar list and add it to discarded registrar list                 */
    /*************************************************************************/
    if((FAILURE == wps_find_common_auth_type(
        wps_enrollee->config_ptr->auth_type_flags,
        (GET_U16_BE(rx_attr->auth_type_flags))))
        || (FAILURE == wps_find_common_encry_type(
        wps_enrollee->config_ptr->enc_type_flags,
        (GET_U16_BE(rx_attr->encr_type_flags))))
        || ((FAILURE == wps_find_common_conn_type(
        wps_enrollee->config_ptr->con_type_flag, *rx_attr->conn_type_flags))
#ifdef ALLOW_ZERO_AS_VALID_CONN_TYPE
        &&(0 != *rx_attr->conn_type_flags)
#endif /* ALLOW_ZERO_AS_VALID_CONN_TYPE */
        ))
    {
        send_wps_status(ERR_REC_WRONG_M2, NULL, 0);
        return FAILURE;
    }
#ifdef CHECK_DEV_PASS_ID_M2
    /*************************************************************************/
    /* Even though standard supports different Device Password ID in M2 this */
    /* is currently not supported. Check received Device Password ID with    */
    /* the configured Device Password ID. If this check fails then indicate  */
    /* wrong M2 and remove the Registrar from selected registrar list and add*/
    /* it to discarded registrar list                                        */
    /*************************************************************************/
    if(((PBC == wps_enrollee->config_ptr->prot_type) &&
        (DEV_PASS_ID_PUSHBUTT != (GET_U16_BE(rx_attr->dev_pass_id)))) ||
        ((PIN == wps_enrollee->config_ptr->prot_type) &&
        (wps_enrollee->config_ptr->dev_pass_id !=
        (GET_U16_BE(rx_attr->dev_pass_id)))))
    {
        send_wps_status(ERR_DIFF_PASS_ID_M2, NULL, 0);
        return FAILURE;
    }
#endif
    memcpy(wps_enrollee->reg_uuid, rx_attr->uuid_r, WPS_UUID_LEN);
    memcpy(wps_enrollee->reg_nonce, rx_attr->reg_nonce, WPS_NONCE_LEN);

    /*************************************************************************/
    /* Aloocate frame buffer in advance to avoid running out of buffer due   */
    /* to long processing time of Dh-Key generation                          */
    /*************************************************************************/
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
    /*************************************************************************/
    /* Verify the Authenticator by calling function wps_verify_authen with   */
    /* wps_enrollee as input. If verification fails then indicate wrong M2   */
    /* and remove the Registrar from selected registrar list and add it to   */
    /* discarded registrar list                                              */
    /*************************************************************************/
    wps_gen_secr_keys(wps_enrollee, rx_attr->pkr);
    if(FAILURE == wps_verify_authen(wps_enrollee))
    {
        send_wps_status(ERR_REC_WRONG_M2, NULL, 0);
        wps_enrollee->tx_wlan_frm_buff = NULL;
        return FAILURE;
    }
    wps_enrollee->msg_st = M3_ST;
    if(FAILURE == wps_send_m3(wps_enrollee))
    {
        wps_enrollee->tx_wlan_frm_buff = NULL;
        return FAILURE;
    }
    wps_start_reg_msg_timer(wps_enrollee);
    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_proc_rx_m2d                                       */
/*                                                                           */
/*  Description      : This function processes the received message of type  */
/*                     M2d                                                   */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the parsed attribute memory             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : If in M1 state then this function will send the M2D   */
/*                     info to user/host                                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_proc_rx_m2d(wps_enrollee_t* wps_enrollee, wps_rx_attri_t* rx_attr)
{
    wps_m2d_dis_info_t *m2d_info;
    if(M1_ST != wps_enrollee->msg_st)
    {
        return FAILURE;
    }
    /*************************************************************************/
    /* Check if UUID-R, Manufacturer, Model name, Model Number, Serial       */
    /* Number, Device Name, RF Bands, Association State and Configuration    */
    /* Error are present, if not return FAILURE.                             */
    /*************************************************************************/
    if((NULL == rx_attr->uuid_r) ||
        (NULL == rx_attr->manufacturer) ||
        (NULL == rx_attr->model_name) ||
        (NULL == rx_attr->model_number) ||
        (NULL == rx_attr->serial_number) ||
        (NULL == rx_attr->device_name) ||
        (NULL == rx_attr->rf_bands) ||
        (NULL == rx_attr->assoc_state) ||
        (NULL == rx_attr->config_error))
    {
        return FAILURE;
    }

    /*************************************************************************/
    /* Allocate buffer to send M2d info to Host/user. if allocation fails    */
    /* indicate to system.                                                   */
    /* Copy the required information from M2d to the message buffer that will*/
    /* sent to Host/user                                                     */
    /*************************************************************************/
    if(NULL == (m2d_info = wps_mem_alloc(sizeof(wps_m2d_dis_info_t))))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    memset(m2d_info, 0, sizeof(wps_m2d_dis_info_t));
    memcpy(m2d_info->uuid_r, rx_attr->uuid_r, WPS_UUID_LEN);

    memcpy(m2d_info->manufacturer, rx_attr->manufacturer,
        rx_attr->manufacturer_len);
    m2d_info->manufacturer_len = rx_attr->manufacturer_len;

    memcpy(m2d_info->model_name, rx_attr->model_name, rx_attr->model_name_len);
    m2d_info->model_name_len = rx_attr->model_name_len;

    memcpy(m2d_info->model_number, rx_attr->model_number,
        rx_attr->model_number_len);
    m2d_info->model_number_len = rx_attr->model_number_len;

    memcpy(m2d_info->serial_number, rx_attr->serial_number,
        rx_attr->serial_number_len);
    m2d_info->serial_number_len = rx_attr->serial_number_len;

    memcpy(m2d_info->device_name, rx_attr->device_name,
        rx_attr->device_name_len);
    m2d_info->device_name_len = rx_attr->device_name_len;
    send_wps_status(REC_M2D,(UWORD8*)m2d_info, sizeof(wps_m2d_dis_info_t));
    wps_mem_free(m2d_info);
    memcpy(wps_enrollee->reg_nonce, rx_attr->reg_nonce, WPS_NONCE_LEN);
    if(FAILURE == wps_send_wsc_ack(wps_enrollee))
    {
        return FAILURE;
    }
    wps_start_reg_msg_timer(wps_enrollee);
    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_proc_rx_m4                                        */
/*                                                                           */
/*  Description      : This function processes the received message of type  */
/*                     M4.                                                   */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the parsed attribute memory             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Based on received attributes and current state, this  */
/*                     function either moves to next state (M5), retransmit  */
/*                     M5, send NACK or return Failure.                      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_proc_rx_m4(wps_enrollee_t* wps_enrollee, wps_rx_attri_t* rx_attr)
{
    UWORD8* decry_data;
    WORD16 decry_len;
    wps_rx_attri_t decr_rx_attr;
    wps_rx_attri_t *rx_attr_ptr_ptr[1];
    rx_attr_ptr_ptr[0] = &decr_rx_attr;

    /*************************************************************************/
    /* If in M5 state then retransmit M5. If not in M5 or M3 state then      */
    /* ignore the message and return FAILURE. Also verify Authenticator      */
    /*************************************************************************/
    if(M5_ST == wps_enrollee->msg_st)
    {
        wps_enrollee->tx_eap_txed_msg_len = 0;
        wps_tx_eap_msg(wps_enrollee);
        return SUCCESS;
    }
    if(M3_ST != wps_enrollee->msg_st)
    {
        return FAILURE;
    }

    /*************************************************************************/
    /* Check if required parameters are present, if not then treat it as a   */
    /* message failure                                                       */
    /*************************************************************************/
    if((FAILURE == wps_verify_authen(wps_enrollee)) ||
        (NULL == rx_attr->r_hash1) ||
        (NULL == rx_attr->r_hash2) ||
        (NULL == rx_attr->encrypted_settings))
    {
        send_wps_status(ERR_REC_WRONG_M4, NULL, 0);
        return FAILURE;
    }
    /*************************************************************************/
    /* Copy R-Hash1 and R-Hash2 to the Enrollee structure                    */
    /*************************************************************************/
    memcpy(wps_enrollee->non_olap.nonce_hash.r_hash1, rx_attr->r_hash1,
        SHA_256_HASH_LEN);
    memcpy(wps_enrollee->non_olap.nonce_hash.r_hash2, rx_attr->r_hash2,
        SHA_256_HASH_LEN);

    /*************************************************************************/
    /* Allocate temporary buffer for the decrypted data and decrypt the      */
    /* Encrypted settings.                                                   */
    /*************************************************************************/
    decry_len = (WORD16)(rx_attr->encrypted_settings_len -
        (UWORD16)WPS_AES_BYTE_BLOCK_SIZE);
    if(NULL == (decry_data = wps_mem_alloc(decry_len)))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    if(0 >= (decry_len = wps_decrypt_encrypted_setting(wps_enrollee,
        rx_attr->encrypted_settings,
        rx_attr->encrypted_settings_len,
        decry_data)))
    {
        wps_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M4,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    /*************************************************************************/
    /*Parse the decrypted data and get all the required fields               */
    /*************************************************************************/
    if(1 != wps_parse_attri(decry_data, rx_attr_ptr_ptr,
        (UWORD16)decry_len, 1, BFALSE))
    {
        wps_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M4,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    /*************************************************************************/
    /* Verify the Key Wrap Authenticator and R-SNonce1 parsed from the       */
    /* decrypted settings                                                    */
    /*************************************************************************/
    if((FAILURE == wps_verify_key_wrap_auth(wps_enrollee, decry_data,
        (UWORD16)decry_len)) ||
        (FAILURE == wps_verify_r_snonce(wps_enrollee,
        decr_rx_attr.r_snonce1, 1)))
    {
        wps_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M4,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    wps_mem_free(decry_data);
    wps_enrollee->msg_st = M5_ST;
    if(FAILURE == wps_send_m5_or_7(wps_enrollee, 0))
    {
        return FAILURE;
    }
    wps_start_reg_msg_timer(wps_enrollee);
    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_proc_rx_m6                                        */
/*                                                                           */
/*  Description      : This function processes the received message of type  */
/*                     M6.                                                   */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the parsed attribute memory             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Based on received attributes and current state, this  */
/*                     function either moves to next state (M7), retransmit  */
/*                     M7, send NACK or return Failure.                      */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_proc_rx_m6(wps_enrollee_t* wps_enrollee, wps_rx_attri_t* rx_attr)
{
    UWORD8* decry_data;
    WORD16 decry_len;
    wps_rx_attri_t decr_rx_attr;
    wps_rx_attri_t *rx_attr_ptr_ptr[1];
    rx_attr_ptr_ptr[0] = &decr_rx_attr;

    /*************************************************************************/
    /* If in M7 state then retransmit M7. If not in M7 or M5 state then      */
    /* ignore the message and return FAILURE                                 */
    /*************************************************************************/
    if(M7_ST == wps_enrollee->msg_st)
    {
        wps_enrollee->tx_eap_txed_msg_len = 0;
        wps_tx_eap_msg(wps_enrollee);
        return SUCCESS;
    }
    if(M5_ST != wps_enrollee->msg_st)
    {
        return FAILURE;
    }

    /*************************************************************************/
    /* Check if required parameters are present, if not then treat it is as a*/
    /* message failure                                                       */
    /*************************************************************************/
    if((FAILURE == wps_verify_authen(wps_enrollee)) ||
        (NULL == rx_attr->encrypted_settings))
    {
        send_wps_status(ERR_REC_WRONG_M6, NULL, 0);
        return FAILURE;
    }

    /*************************************************************************/
    /* Allocate temporary buffer for the decrypted data and decrypt the      */
    /* Encrypted settings.                                                   */
    /*************************************************************************/
    decry_len = (WORD16)(rx_attr->encrypted_settings_len -
        (UWORD16)WPS_AES_BYTE_BLOCK_SIZE);
    if(NULL == (decry_data = wps_mem_alloc(decry_len)))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
   }
    if(0 >= (decry_len = wps_decrypt_encrypted_setting(wps_enrollee,
        rx_attr->encrypted_settings,
        rx_attr->encrypted_settings_len,
        decry_data)))
    {
        wps_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M6,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    /*************************************************************************/
    /*Parse the decrypted data and get all the required fields               */
    /*************************************************************************/
    if(1 != wps_parse_attri(decry_data, rx_attr_ptr_ptr,
        (UWORD16)decry_len, 1, BFALSE))
    {
        wps_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M6,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    /*************************************************************************/
    /* Verify the Key Wrap Authenticator and R-SNonce1 parsed from the       */
    /* decrypted settings                                                    */
    /*************************************************************************/
    if((FAILURE == wps_verify_key_wrap_auth(wps_enrollee, decry_data,
        (UWORD16)decry_len)) ||
        (FAILURE == wps_verify_r_snonce(wps_enrollee,
        decr_rx_attr.r_snonce2, 2)))
    {
        wps_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M6,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    wps_mem_free(decry_data);
    wps_enrollee->msg_st = M7_ST;
    if(FAILURE == wps_send_m5_or_7(wps_enrollee, 1))
    {
        return FAILURE;
    }
    wps_start_reg_msg_timer(wps_enrollee);
    return SUCCESS;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_proc_rx_m8                                        */
/*                                                                           */
/*  Description      : This function processes the received message of type  */
/*                     M8.                                                   */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the parsed attribute memory             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Based on received attributes and current state, this  */
/*                     function either moves to next state (Done), retransmit*/
/*                     WSC_Done, send NACK or return Failure. This function  */
/*                     also does the received Credentials processing         */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing (SUCCESS/FAILURE)   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_proc_rx_m8(wps_enrollee_t* wps_enrollee, wps_rx_attri_t* rx_attr)
{
    UWORD8* decry_data;
    WORD16 decry_len;
    wps_rx_attri_t decr_rx_attr;
    wps_rx_attri_t *rx_attr_ptr_ptr[1];
    rx_attr_ptr_ptr[0] = &decr_rx_attr;

    /*************************************************************************/
    /* If in DONE state then retransmit WSC_Done. If not in Done or M7 state */
    /* then ignore the message and return FAILURE                            */
    /*************************************************************************/
    if(DONE_ST == wps_enrollee->msg_st)
    {
        wps_enrollee->tx_eap_txed_msg_len = 0;
        wps_tx_eap_msg(wps_enrollee);
        return SUCCESS;
    }
    if(M7_ST != wps_enrollee->msg_st)
    {
        return FAILURE;
    }

    /*************************************************************************/
    /* Check if required parameters are present, if not then treat it is as a*/
    /* message failure                                                       */
    /*************************************************************************/
    if((FAILURE == wps_verify_authen(wps_enrollee)) ||
        (NULL == rx_attr->encrypted_settings))
    {
        send_wps_status(ERR_REC_WRONG_M8, NULL, 0);
        return FAILURE;
    }

    /*************************************************************************/
    /* Allocate temporary buffer for the decrypted data and decrypt the      */
    /* Encrypted settings. As this buffer can be large, use packet memory    */
    /*************************************************************************/
    decry_len = (WORD16)(rx_attr->encrypted_settings_len -
        (UWORD16)WPS_AES_BYTE_BLOCK_SIZE);
    if(NULL == (decry_data = wps_frame_mem_alloc(decry_len)))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    if(0 >= (decry_len = wps_decrypt_encrypted_setting(wps_enrollee,
        rx_attr->encrypted_settings,
        rx_attr->encrypted_settings_len,
        decry_data)))
    {
        wps_frame_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M8,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    /*************************************************************************/
    /*Parse the decrypted data and get all the required fields               */
    /*************************************************************************/
    if(1 != wps_parse_attri(decry_data, rx_attr_ptr_ptr,
        (UWORD16)decry_len, 1, BFALSE))
    {
        wps_frame_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M8,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    /*************************************************************************/
    /* Verify the Key Wrap Authenticator                                     */
    /*************************************************************************/
    if(FAILURE == wps_verify_key_wrap_auth(wps_enrollee, decry_data,
        (UWORD16)decry_len))
    {
        wps_frame_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M8,
                               DEVICE_PASSWORD_AUTH_FAILURE);
        return FAILURE;
    }
    if(FAILURE == wps_parse_creden(wps_enrollee, decr_rx_attr.credential,
        decr_rx_attr.credential_len, decr_rx_attr.cred_cnt))
    {
        wps_frame_mem_free(decry_data);
        wps_handle_msg_failure(wps_enrollee, ERR_REC_WRONG_M8,
                               ROGUE_ACTIVITY_SUSPECTED);
        return FAILURE;
    }
    wps_frame_mem_free(decry_data);
    wps_enrollee->msg_st = DONE_ST;
    if(FAILURE == wps_send_done(wps_enrollee))
    {
        return FAILURE;
    }
    wps_start_reg_msg_timer(wps_enrollee);
    return SUCCESS;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_msg_failure                                */
/*                                                                           */
/*  Description      : This function handles the failures occurred during the*/
/*                     M4 to M8 received message processing                  */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Message code to be sent to Host/user               */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function send the NACK to the peer entity and    */
/*                     Error message to the host/user                        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         05 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_handle_msg_failure(wps_enrollee_t* wps_enrollee,
                            WPS_STATUS_T err_code, UWORD16 config_error)
{
    send_wps_status(err_code, NULL, 0);
    wps_enrollee->config_error = config_error;
    wps_enrollee->msg_st = NACK_ST;
    if(FAILURE == wps_send_nack(wps_enrollee))
    {
        return;
    }
    wps_start_reg_msg_timer(wps_enrollee);
    return;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_proc_wsc_nack                                     */
/*                                                                           */
/*  Description      : This function processes the received message of Type  */
/*                     WSC_NACK                                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function parses the received WSC_NACK and checks */
/*                     the parsed fields. If fields are correct then it send */
/*                     a new WSC_NACK or retransmits WSC_NACK                */
/*                                                                           */
/*  Outputs          : WSC_NACK info to Host and WSC_NACK to peer            */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         15 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_proc_wsc_nack(wps_enrollee_t* wps_enrollee)
{
    /*************************************************************************/
    /*This structure stores all the required attributes                      */
    /*************************************************************************/
    wps_rx_attri_t rx_attr;
    wps_rx_attri_t *rx_attr_ptr_ptr[1];
    wps_nack_dis_info_t nack_info;
    rx_attr_ptr_ptr[0] = &rx_attr;

    /*************************************************************************/
    /*Parse the received message and get all the required fields             */
    /*************************************************************************/
    if(1 != wps_parse_attri(wps_enrollee->rx_m_msg_ptr, rx_attr_ptr_ptr,
        wps_enrollee->rx_m_msg_len, 1, BFALSE))
    {
        return FAILURE;
    }
    /*************************************************************************/
    /* Do the checks common to all message i.e. version number, message type */
    /* and Enrollee Nonce check. if check fails return FAILURE               */
    /*************************************************************************/
    if((((NULL != rx_attr.version2)) &&
         (FAILURE == wps_check_ver2_supp(rx_attr.version2))) ||
        (NULL == rx_attr.msg_type) ||
        (NULL == rx_attr.enr_nonce)||
        (NULL == rx_attr.reg_nonce)||
        (NULL == rx_attr.config_error))
    {
        return FAILURE;
    }
    if((0 != memcmp(wps_enrollee->enr_nonce, rx_attr.enr_nonce, WPS_NONCE_LEN))
        || (0 != memcmp(wps_enrollee->reg_nonce, rx_attr.reg_nonce,
        WPS_NONCE_LEN)))
    {
        return FAILURE;
    }
    /*************************************************************************/
    /* Ideally, if NACK already sent then it should be resent as AP may not  */
    /* have received it. However, it was observed with Buffalo AP that even  */
    /* AP sends NACK in response of a NACK (not as per standard). To avoid   */
    /* large number of NACKs in such condition, don't send NACK if already   */
    /* sent, instead just stop this protocol                                 */
    /*************************************************************************/
    if(NACK_ST == wps_enrollee->msg_st)
    {
//        wps_enrollee->tx_eap_txed_msg_len = 0;
//        wps_tx_eap_msg(wps_enrollee);
//        return SUCCESS;
        wps_enrollee->assoc_state = WPS_CONFIGURATION_FAILURE;
        send_wps_status(ERR_REC_NACK,NULL,0);
        wps_add_discarded_reg_ap(wps_enrollee,
            wps_enrollee->sel_ap_info.bssid);
        wps_delete_timers(wps_enrollee);
        wps_loc_start_scan(wps_enrollee);
        return FAILURE;
    }
    /*************************************************************************/
    /* Send NACK Info to host                                                */
    /*************************************************************************/
    nack_info.config_error = GET_U16_BE(rx_attr.config_error);
    memcpy(nack_info.uuid_r, wps_enrollee->reg_uuid, WPS_UUID_LEN);
    send_wps_status(ERR_REC_NACK, (UWORD8*)&nack_info,
        sizeof(wps_nack_dis_info_t));
    /*************************************************************************/
    /* Send WSC_NACK to peer                                                 */
    /*************************************************************************/
    wps_start_reg_msg_timer(wps_enrollee);
    wps_enrollee->msg_st = NACK_ST;
    if(FAILURE == wps_send_nack(wps_enrollee))
    {
        return FAILURE;
    }
    return SUCCESS;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_decrypt_encrypted_setting                         */
/*                                                                           */
/*  Description      : This function decrypts the encryption setting         */
/*                     attribute and returns the decrypted data.             */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Array to the encrypted data                        */
/*                     3) Encrypted data length                              */
/*                     4) Pointer to buffer where decrypted data needs to be */
/*                        stored                                             */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Do the encrypted data length check if fails return    */
/*                     error code. Decrypt the data and remove pad. If       */
/*                     invalid pad or pad length received then return error  */
/*                     code else return decrypted data length without pad    */
/*                                                                           */
/*  Outputs          : Decrypted data along with length or error code        */
/*  Returns          : Error code or decrypted data length                   */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         12 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
WORD16  wps_decrypt_encrypted_setting(wps_enrollee_t* wps_enrollee,
                                       UWORD8* encry_data,
                                       UWORD16 encry_data_len,
                                       UWORD8* decry_data)
{
    UWORD8  *data_ptr;
    UWORD16 data_len;
    UWORD8 pad, cnt;
    if ((2*WPS_AES_BYTE_BLOCK_SIZE > encry_data_len ) ||
        (0 != (encry_data_len % WPS_AES_BYTE_BLOCK_SIZE)))
    {
        return ERR_INVALID_ENCR_DATA_LEN;
    }
    data_len = encry_data_len - WPS_AES_BYTE_BLOCK_SIZE;
    wps_aes_decrypt(wps_enrollee, encry_data, decry_data, data_len);
    data_ptr = decry_data + data_len -1;
    pad = *data_ptr--;
    /*************************************************************************/
    /* Pad length should be less than WPS_AES_BYTE_BLOCK_SIZE. As per PKCS#5 */
    /* v2.0, Pad and pad length have same value. Check for pad length and    */
    /* byte of pad. If check fails return Pad error                          */
    /*************************************************************************/
    if(pad > WPS_AES_BYTE_BLOCK_SIZE)
    {
        return ERR_INVALID_PAD;
    }
    for (cnt = 1; cnt < pad; cnt++)
    {
        if (*data_ptr-- != pad)
        {
            return ERR_INVALID_PAD;
        }
    }
    return (data_len - pad);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_verify_key_wrap_auth                              */
/*                                                                           */
/*  Description      : This function verifies the whether the Key Wrap       */
/*                     Authenticator present in the received message is      */
/*                     correct or not                                        */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the decrypted encrypted setting data    */
/*                     3) Length of decrypted encrypted setting data         */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Check if Key Wrap Authenticator is present at the end */
/*                     of the input data. If not then return FAILURE.        */
/*                     Generate Key Warp Authenticator and compare it with   */
/*                     the value received. If comparison fails return        */
/*                     FAILURE else return SUCCESS                           */
/*                                                                           */
/*  Outputs          : Result of Key Way Authenticator check                 */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         12 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_verify_key_wrap_auth(wps_enrollee_t* wps_enrollee,
                                       UWORD8* data,
                                       UWORD16 data_len)
{
    UWORD8 temp_buff[SHA_256_HASH_LEN];
    if(WPS_ATTR_KEY_WRAP_AUTH != GET_U16_BE((data + data_len -
        WPS_KEY_WRAP_AUTH_LEN-4)))
    {
        return FAILURE;
    }
    data_len -= (WPS_KEY_WRAP_AUTH_LEN + 4);
    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, &data, &data_len, 1,
        temp_buff);
    if(0 == memcmp(data + data_len + 4, temp_buff,
        WPS_KEY_WRAP_AUTH_LEN))
    {
        return SUCCESS;
    }
    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_verify_authen                                     */
/*                                                                           */
/*  Description      : This function verifies whether the received Mx message*/
/*                     has correct authenticator or not.                     */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Check if Key Wrap Authenticator is present at the end */
/*                     of the Mx Message. Checks if transmitted message is   */
/*                     present. Generates the Authenticator and compare it   */
/*                     with the value received. If any checks fails return   */
/*                     FAILURE else return SUCCESS                           */
/*                                                                           */
/*  Outputs          : Result of Authenticator check                         */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         12 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_verify_authen(wps_enrollee_t* wps_enrollee)
{
    UWORD8  temp_buff[SHA_256_HASH_LEN];
    UWORD8* msg_ptr[2];
    UWORD16 msg_len[2];

    if((NULL == wps_enrollee->tx_m_msg_ptr) ||
        (0 == wps_enrollee->tx_m_msg_len) ||
        (NULL == wps_enrollee->rx_m_msg_ptr) ||
        ((4 + WPS_AUTHENTICATOR_LEN) > wps_enrollee->rx_m_msg_len))
    {
        return FAILURE;
    }
    if(WPS_ATTR_AUTHENTICATOR !=  GET_U16_BE((wps_enrollee->rx_m_msg_ptr +
        wps_enrollee->rx_m_msg_len - WPS_AUTHENTICATOR_LEN - 4)))
    {
        return FAILURE;
    }
    msg_ptr[0] = wps_enrollee->tx_m_msg_ptr;
    msg_ptr[1] = wps_enrollee->rx_m_msg_ptr;
    msg_len[0] = wps_enrollee->tx_m_msg_len;
    msg_len[1] = wps_enrollee->rx_m_msg_len - 4 - WPS_AUTHENTICATOR_LEN;

    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, msg_ptr, msg_len, 2,
        temp_buff);
    if(0 == memcmp(wps_enrollee->rx_m_msg_ptr + wps_enrollee->rx_m_msg_len -
        WPS_AUTHENTICATOR_LEN, temp_buff, WPS_AUTHENTICATOR_LEN))
    {
        return SUCCESS;
    }
    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_verify_authen                                     */
/*                                                                           */
/*  Description      : This function verifies the received R-SNonce1 or      */
/*                     R-SNonce2                                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the received R-S NonceX                 */
/*                     3) Received R-S Nonce count (1 or 2)(No check done)   */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : Using Received R-S NonceX and other stored parameters */
/*                     generate R-HashX and compare it with received R-hashX */
/*                     If comparison fails return FAILURE else return        */
/*                     SUCCESS                                               */
/*                                                                           */
/*  Outputs          : Result of R-S NonceX verification                     */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         12 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_verify_r_snonce(wps_enrollee_t* wps_enrollee, UWORD8* r_nonce_ptr,
                             UWORD8 nonce_cnt)
{
    UWORD8  temp_buff[SHA_256_HASH_LEN];
    UWORD8* msg_ptr[4];
    UWORD8* hash_ptr;
    UWORD16 msg_len[4];
    if(NULL == r_nonce_ptr)
    {
        return FAILURE;
    }
    msg_ptr[0] = r_nonce_ptr;
    msg_ptr[2] = wps_enrollee->pke;
    msg_ptr[3] = wps_enrollee->pub_key.pkr;
    msg_len[0] = WPS_NONCE_LEN;
    msg_len[1] = WPS_PSK_LEN;
    msg_len[2] = WPS_PUBLIC_KEY_LEN;
    msg_len[3] = WPS_PUBLIC_KEY_LEN;

    if(1 == nonce_cnt)
    {
        hash_ptr = wps_enrollee->non_olap.nonce_hash.r_hash1;
        msg_ptr[1] = wps_enrollee->non_olap.nonce_hash.psk1;
    }
    else
    {
        hash_ptr = wps_enrollee->non_olap.nonce_hash.r_hash2;
        msg_ptr[1] = wps_enrollee->non_olap.nonce_hash.psk2;
    }

    hmac_sha256(wps_enrollee->auth_key, WPS_AUTH_KEY_LEN, msg_ptr, msg_len, 4,
        temp_buff);
    if(0 == memcmp(hash_ptr, temp_buff, WPS_HASH_LEN))
    {
        return SUCCESS;
    }
    return FAILURE;
}

#ifdef MAC_P2P
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_p2p_prob_req_enr                          */
/*                                                                           */
/*  Description      : This funtion checks if there is a requested device    */
/*                     type attribute present in the WSC IE in the probe     */
/*                     request frame. If it is present then it checks if     */
/*                     its primary device type matches with that             */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to received frame                          */
/*                     3) Length of received association response            */
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
/*  Returns          : BOOL_T; BTRUE                                         */
/*                             BFALSE                                        */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T wps_process_p2p_prob_req_enr(wps_enr_config_struct_t *config_ptr,
                                    wps_enrollee_t* wps_enrollee, UWORD8 *msa,
                                    UWORD16 rx_len)
{
    UWORD16 wsc_ie_len             = 0;
    UWORD16 ie_start_index         = 0;
    wps_rx_attri_t parse_attr      = {0};
    wps_rx_attri_t *parse_attr_ptr = &parse_attr;
    UWORD8 *wsc_ie_buf             = NULL;
    UWORD8 *buffer                 = NULL;
    BOOL_T retval                  = BTRUE;

	TROUT_FUNC_ENTER;
    /* Get the start of the IE */
    /* MAC Header(24),  Timestamp (8), Beacon Interval (2) and Capability (2)*/
    ie_start_index =  WPS_MAC_HDR_LEN;

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_enr(msa, rx_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_frame_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_enr(msa, rx_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    /* If WSC-IE not present then exit */
    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
    {
		TROUT_FUNC_EXIT;
        return retval;
	}
    /* Call the funtion to parse WSC IE and check if one or more         */
    /* attrubutes are found                                              */
    if(0 != wps_parse_attri(wsc_ie_buf, &parse_attr_ptr,
                            wsc_ie_len, 1, BTRUE))
    {
        /* Check if the WSC IE has requested device type attribute */
        if(0 != parse_attr_ptr->req_dev_type)
        {
            /* Check if the primary device type matches with the         */
            /* requested device type. If it doesnot match then return    */
            /* BFALSE                                                    */
            /* Note: We match only the Category ID of the primary device */
            /* type i.e the first 2 bytes of the attribute               */
            if(0 != memcmp(config_ptr->prim_dev_cat_id,
                           parse_attr_ptr->req_dev_type, 2))
            {
                retval = BFALSE;
            }
        }
    }

    /* Free the allocated buffer */
    if(NULL != buffer)
        wps_frame_mem_free(buffer);

	TROUT_FUNC_EXIT;
    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_p2p_prob_rsp_enr                          */
/*                                                                           */
/*  Description      : This function processes the probe response            */
/*                                                                           */
/*  Inputs           : 1) MAC start address of the probe response frame      */
/*                     2) Length of received probe response                  */
/*                     3) Pointer to store the config method                 */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function extracts the config method present in   */
/*                     the WSC IE in the probe response frame                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : DIRECT  : None                                        */
/*                     INDIRECT: Config Method                               */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_process_p2p_prob_rsp_enr(UWORD8 *msa, UWORD16 rx_len, UWORD16 *config)
{
    UWORD16 wsc_ie_len             = 0;
    UWORD16 ie_start_index         = 0;
    wps_rx_attri_t parse_attr      = {0};
    wps_rx_attri_t *parse_attr_ptr = &parse_attr;
    UWORD8 *wsc_ie_buf = NULL;
    UWORD8 *buffer = NULL;

	TROUT_FUNC_ENTER;
    /* Get the start of the IE */
    /* MAC Header(24),  Timestamp (8), Beacon Interval (2) and Capability (2)*/
    ie_start_index = (WPS_MAC_HDR_LEN + TIMESTAMP_LEN + BCN_INT_LEN +
                       CAP_INFO_LEN);

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_enr(msa, rx_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_frame_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_enr(msa, rx_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    /* If WSC-IE not present then exit */
    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
    {
		TROUT_FUNC_EXIT;
        return;
	}
    /* Call the funtion to parse WSC IE and check if one or more         */
    /* attrubutes are found                                              */
    if(0 != wps_parse_attri(wsc_ie_buf, &parse_attr_ptr,
                            wsc_ie_len, 1, BTRUE))
    {
        /* Check if the WSC IE has requested device type attribute */
        if(0 != parse_attr_ptr->config_meth)
        {
            /* Get the config method */
            *config = GET_U16_BE((parse_attr_ptr->config_meth));
        }
    }

    /* Free  the allocated buffer */
    if(NULL != buffer)
        wps_frame_mem_free(buffer);

    TROUT_FUNC_EXIT;
}

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
    UWORD16 ie_start_index         = 0;
    UWORD16 config_method          = 0;
    wps_rx_attri_t parse_attr      = {0};
    wps_rx_attri_t *parse_attr_ptr = &parse_attr;
    UWORD8 *wsc_ie_buf = NULL;
    UWORD8 *buffer = NULL;

    /* get the IE start index */
    ie_start_index = WPS_MAC_HDR_LEN + P2P_PUB_ACT_TAG_PARAM_OFF;

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_enr(msa, rx_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_frame_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_enr(msa, rx_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    /* If WSC-IE not present then exit */
    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
        return 0;

    /* Call the funtion to parse WSC IE and check if one or more         */
    /* attrubutes are found                                              */
    if(0 != wps_parse_attri(wsc_ie_buf, &parse_attr_ptr,
                            wsc_ie_len, 1, BTRUE))
    {
        /* Get the config method */
        config_method = GET_U16_BE((parse_attr_ptr->config_meth));
    }

    /* Free  the allocated buffer */
    if(NULL != buffer)
        wps_frame_mem_free(buffer);

    return config_method;
}

#endif /* MAC_P2P */

#endif /* INT_WPS_ENR_SUPP */

