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
/*  File Name         : wps_eap_rx_reg.c                                     */
/*                                                                           */
/*  Description       : This file contains the all functions that handle     */
/*                      EAP message reception during Registration            */
/*                      Protocol of the Registrar                            */
/*                                                                           */
/*  List of Functions : wps_handle_eapol_start_rx                            */
/*                      wps_handle_eapol_pkt_rx                              */
/*                      wps_handle_eap_rsp_id_rx                             */
/*                      wps_handle_eap_wsc_rx                                */
/*                      wps_handle_eap_wsc_defrag                            */
/*                      wps_handle_eap_wsc_msg_rx                            */
/*                      wps_handle_eap_wsc_ack_nack_done_rx                  */
/*                      wps_process_m1_rx                                    */
/*                      wps_process_m3_rx                                    */
/*                      wps_process_m5_rx                                    */
/*                      wps_process_m7_rx                                    */
/*                      wps_handle_msg_failure                               */
/*                      all_req_attr_present                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_REG_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_eap_rx_reg.h"
#include "wps_eap_tx_reg.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static STATUS_T wps_handle_eap_rsp_id_rx(wps_t *wps_ctxt_hdl,
                                         wps_reg_t *reg_hdl,
                                         wps_eap_tx_t *eap_tx_hdl,
                                         wps_eap_rx_t *eap_rx_hdl,
                                         UWORD8 *msa, UWORD16 eap_len);
static STATUS_T wps_handle_eap_wsc_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                      wps_eap_rx_t *eap_rx_hdl,
                                      wps_eap_tx_t *eap_tx_hdl,
                                      UWORD8 *msa, UWORD16 eap_len);
static STATUS_T wps_handle_eap_wsc_defrag(wps_t *wps_ctxt_hdl,
                                          wps_reg_t *reg_hdl,
                                          wps_eap_rx_t *eap_rx_hdl,
                                          wps_eap_tx_t *eap_tx_hdl,
                                          UWORD8 *msg_ptr,
                                          UWORD16 msg_len, UWORD16 frag_len,
                                          UWORD8 flags, UWORD8 opcode);
static STATUS_T wps_handle_eap_wsc_msg_rx(wps_t *wps_ctxt_hdl,
                                          wps_reg_t *reg_hdl,
                                          wps_eap_rx_t *eap_rx_hdl,
                                          wps_eap_tx_t *eap_tx_hdl);
static STATUS_T wps_handle_eap_wsc_ack_nack_done_rx(wps_t *wps_ctxt_hdl,
                                                    wps_reg_t *reg_hdl,
                                                    wps_eap_rx_t *eap_rx_hdl,
                                                    wps_eap_tx_t *eap_tx_hdl,
                                                    UWORD8 opcode);
static STATUS_T wps_process_m1_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr);
static STATUS_T wps_process_m3_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr);
static STATUS_T wps_process_m5_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr);
static STATUS_T wps_process_m7_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr);
static void wps_handle_msg_failure(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                   wps_eap_tx_t *eap_tx_hdl,
                                   WPSR_STATUS_T status);
static BOOL_T all_req_attr_present(WPS_MSG_TYPE_T msg_type,
                                   wps_rx_attri_t *rx_attr);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eapol_start_rx                             */
/*                                                                           */
/*  Description      : This function handles reception of EAPOL-Start frames */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the received frame                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following :                    */
/*                      1) Checks if the Registrar is not IDLE. If not IDLE  */
/*                         then it indicates registrar is busy in            */
/*                         registration protocol                             */
/*                      2) Reset the TX EAP-ID and send EAP Request/Identity */
/*                         to Enrollee                                       */
/*                      3) If the Send is a success then set the WPS State to*/
/*                         WPS_REG_PROT_ST indicating that the Registrar has */
/*                         entered Registration Protocol. Also set the       */
/*                         Registrar`s Registration Protocol State to        */
/*                         REG_EAP_ID_ST indicating Registrar has send       */
/*                         EAP-Request/ID and is waiting for EAP-Response/ID */
/*                         from the Enrollee                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of EAPOL-Start     */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_handle_eapol_start_rx(wps_t *wps_ctxt_hdl)
{
    wps_reg_t *reg_hdl = wps_ctxt_hdl->wps_reg_hdl;
    wps_eap_tx_t *eap_tx_hdl = wps_ctxt_hdl->wps_eap_tx_hdl;
    STATUS_T status = SUCCESS;

    /* If the Registrar`s WPS Protocol State is not IDLE then exit           */
    if(WPS_IDLE_ST != reg_hdl->wpsr_state)
        return FAILURE;

    /* Send EAP-Request/Identidy to Enrollee */
    status = wps_send_eap_req_id(wps_ctxt_hdl, eap_tx_hdl);

    /*************************************************************************/
    /* If the Send is a success then set the WPS State to WPS_REG_PROT_ST    */
    /* and Registration Protocol State to REG_EAP_ID_ST                      */
    /*************************************************************************/
    if(SUCCESS == status)
    {
        reg_hdl->wpsr_state = WPS_REG_PROT_ST;
        reg_hdl->reg_prot_state = REG_EAP_ID_ST;

        /* Reset the EAP Packet retry count */
        eap_tx_hdl->eap_msg_retry_cnt = 0;

        /* Start EAP Message timer */
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;

        /* Indicate the status to host */
        send_wps_status(WPS_REG_REGPROT_START, NULL, 0);
    }

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eapol_pkt_rx                               */
/*                                                                           */
/*  Description      : This function handles reception of frame of 802.1X    */
/*                     type EAPOL-Packet                                     */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Pointer to the received frame                      */
/*                     3) Length of received EAPOL-Packet                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the registrar is in correct   */
/*                     WPS and Registration Protocol States. EAP Code,       */
/*                     Identifier, Length and Type are extracted from EAP    */
/*                     Header and then  appropriate functions to process     */
/*                     further are called                                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of EAPOL-Packet    */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         20 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_handle_eapol_pkt_rx(wps_t *wps_ctxt_hdl, UWORD8 *msa,
                                 UWORD16 pkt_len)
{
    UWORD8  eap_code = 0;
    UWORD8  eap_id   = 0;
    UWORD8  eap_type = 0;
    STATUS_T retval  = FAILURE;
    UWORD16 eap_len  = 0;
    wps_reg_t    *reg_hdl    = wps_ctxt_hdl->wps_reg_hdl;
    wps_eap_rx_t *eap_rx_hdl = wps_ctxt_hdl->wps_eap_rx_hdl;
    wps_eap_tx_t *eap_tx_hdl = wps_ctxt_hdl->wps_eap_tx_hdl;

    /* If Registrar is not in Registration Protocol then exit                */
    if(WPS_REG_PROT_ST != reg_hdl->wpsr_state)
       return FAILURE;

    /* Extract the Length from EAP header */
    eap_len  = get_eap_len(msa);

    /*************************************************************************/
    /* Length indicated in EAP header must be atleast be equal to the length */
    /* indicated in 1X header must be same                                   */
    /*************************************************************************/
    if(eap_len < pkt_len)
        return FAILURE;

    /* Extract EAP Code and Identifier from EAP Header */
    eap_code = get_eap_code(msa);
    eap_id   = get_eap_id(msa);

    /* For EAP Response Process further */
    if(EAP_CODE_RESPONSE != eap_code)
        return FAILURE;

    /* Extract EAP Type from EAP Header */
    eap_type = get_eap_type(msa);

    /* If EAP identifier is not equal to TX EAP ID then do not process */
    if(eap_tx_hdl->tx_eap_id != eap_id)
        return FAILURE;

    /* Handle reception Response Identity/WPS-WSC type */
    if(EAP_TYPE_IDENTITY == eap_type)
    {
        retval = wps_handle_eap_rsp_id_rx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                                          eap_rx_hdl, msa, eap_len);
    }
    else if(EAP_TYPE_WPS_WSC == eap_type)
        retval = wps_handle_eap_wsc_rx(wps_ctxt_hdl, reg_hdl, eap_rx_hdl,
                                       eap_tx_hdl, msa, eap_len);

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_rsp_id_rx                              */
/*                                                                           */
/*  Description      : This function handles reception of EAP Response/ID    */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) Pointer to the received frame                      */
/*                     6) Length of received EAP Response/Identity frame     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the registrar is not in registration protocol   */
/*                        state then return FAILURE                          */
/*                     2) Then check is done to see if the response is valid */
/*                     3) If the response is valid, then if the Registration */
/*                        Protocol State indicates that the registrar is     */
/*                        waiting for reception of M1 then the Enrollee has  */
/*                        not received WSC-Start, hence retry WSC-Start      */
/*                     4) If not in a state where EAP-Response/ID is expected*/
/*                        the return FAILURE                                 */
/*                     5) Send WSC-Start frame to the Enrollee               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         20 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_handle_eap_rsp_id_rx(wps_t *wps_ctxt_hdl,
                                         wps_reg_t *reg_hdl,
                                         wps_eap_tx_t *eap_tx_hdl,
                                         wps_eap_rx_t *eap_rx_hdl,
                                         UWORD8 *msa, UWORD16 eap_len)
{
    UWORD8 msg_str[]   = "WFA-SimpleConfig-Enrollee-1-0";

    /* For EAP Response ID packet must have a minimum length of 34 bytes */
    if((EAP_HEADER_LEN + EAP_TYPE_FIELD_LEN + EAP_WPS_IDENTITY_PKT_LEN) >
       eap_len)
        return FAILURE;

    /* Check if the response indicates an Enrollee */
    if(0 != memcmp(msg_str, (msa + EAP_RESPONSE_ID_OFFSET),
                   EAP_WPS_IDENTITY_PKT_LEN))
        return FAILURE;

    /* If the Registrar is not waiting for a Response/ID then exit */
    if(REG_EAP_ID_ST != reg_hdl->reg_prot_state)
        return FAILURE;

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    if(SUCCESS == wps_send_eap_wsc_start(wps_ctxt_hdl, eap_tx_hdl))
    {
        reg_hdl->reg_prot_state = REG_INIT_ST;

        /* Reset the EAP retry count */
        wps_ctxt_hdl->wps_eap_tx_hdl->eap_msg_retry_cnt = 0;

        /* Start EAP Message timer */
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;

        return SUCCESS;
    }

    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_wsc_rx                                 */
/*                                                                           */
/*  Description      : This function handles reception EAP WSC frames        */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) Pointer to the received frame                      */
/*                     6) Length of received EAP Response/Identity frame     */
/*                                                                           */
/*  Globals          : g_eap_wps_type_vid_vt                                 */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the the registrar is not in a state to receive  */
/*                        EAP WSC type packets then return FAILURE           */
/*                     2) Check the Vendor-ID and Vendor Type                */
/*                     3) Based on the value LF bit in Flags field the       */
/*                        message pointer, entire eap message length and     */
/*                        current fragment length are computed/extracted     */
/*                     4) Process the received fragment if MF bit is set     */
/*                     5) Update the RX handle and based on the WSC opcode   */
/*                        call appropriate functions for further processing  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         20 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_handle_eap_wsc_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                      wps_eap_rx_t *eap_rx_hdl,
                                      wps_eap_tx_t *eap_tx_hdl,
                                      UWORD8 *msa, UWORD16 eap_len)
{
    UWORD8   wsc_flags   = 0;
    UWORD8   wsc_opcode  = 0;
    STATUS_T status      = SUCCESS;
    UWORD16  msg_len     = 0;
    BOOL_T   more_frag   = BFALSE;
    UWORD8   *msg_ptr    = NULL;
    UWORD16  frag_len    = 0;

    /*************************************************************************/
    /* If the Registration Protocol State is not in REG_INIT_ST, i.e in a    */
    /* state to receive WSC type EAP packets, then return FAILURE            */
    /*************************************************************************/
    if(REG_INIT_ST > reg_hdl->reg_prot_state)
        return FAILURE;

    /* Check for Vendor-ID and Vendor-Type */
    if(0 != memcmp((msa + EAP_WPS_VENDOR_ID_OFFSET), g_eap_wps_vid_vt,
                   EAP_WPS_VID_VT_LEN))
        return FAILURE;

    wsc_flags  = get_wsc_flags(msa);
    wsc_opcode = get_wsc_opcode(msa);

    /*************************************************************************/
    /* Based on the LF Bit, update the message pointer, message length and   */
    /* current fragment length                                               */
    /*************************************************************************/
    if(BTRUE == is_wsc_lf_set(wsc_flags))
    {
        msg_ptr  = (msa + EAP_WPS_DATA_WI_LEN_OFFSET);
        msg_len  = get_wsc_msg_len(msa);
        frag_len = eap_len - (EAP_WPS_DATA_WI_LEN_OFFSET - EAP_HEADER_OFFSET);
    }
    else
    {
        msg_ptr  = (msa + EAP_WPS_DATA_WO_LEN_OFFSET);
        msg_len  = eap_len - (EAP_WPS_DATA_WO_LEN_OFFSET - EAP_HEADER_OFFSET);
        frag_len = msg_len;
    }

    more_frag = is_wsc_mf_set(wsc_flags);

    /*************************************************************************/
    /* If MF is set or if defragmentation is in prgress then handle it       */
    /* appropriately                                                         */
    /*************************************************************************/
    if((BTRUE == more_frag) ||
       (BTRUE == eap_rx_hdl->defrag_in_prog))
    {
        eap_rx_hdl->defrag_in_prog = more_frag;
        return wps_handle_eap_wsc_defrag(wps_ctxt_hdl, reg_hdl, eap_rx_hdl,
                                         eap_tx_hdl, msg_ptr, msg_len,
                                         frag_len, wsc_flags, wsc_opcode);
    }

    /* Update the RX handle appropriately */
    eap_rx_hdl->rx_eap_msg     = msa + EAP_HEADER_OFFSET;
    eap_rx_hdl->rx_eap_msg_len = eap_len;
    eap_rx_hdl->rx_m_msg_ptr   = msg_ptr;
    eap_rx_hdl->rx_m_msg_len   = msg_len;

    /* Based on the WSC Op-Code process the packet further */
    switch(wsc_opcode)
    {
        case WPS_OP_CODE_WSC_MSG:
            status = wps_handle_eap_wsc_msg_rx(wps_ctxt_hdl, reg_hdl,
                                               eap_rx_hdl, eap_tx_hdl);
        break;
        case WPS_OP_CODE_WSC_ACK:
        case WPS_OP_CODE_WSC_NACK:
        case WPS_OP_CODE_WSC_DONE:
            status = wps_handle_eap_wsc_ack_nack_done_rx(wps_ctxt_hdl, reg_hdl,
                                                         eap_rx_hdl,
                                                         eap_tx_hdl,
                                                         wsc_opcode);
        break;
        default:
            status = FAILURE;
        break;
    }

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_wsc_defrag                             */
/*                                                                           */
/*  Description      : This function handles defragmentation of received EAP */
/*                     WSC messages                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) Pointer to received message                        */
/*                     6) Length of the entire EAP message                   */
/*                     7) Length of the current EAP fragment                 */
/*                     8) Flags field extracted from EAP-WSC header          */
/*                     9) WSC Opcode of the received WSC message             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If this the first fragment, then do the following: */
/*                        - If the the LF bit is not set then return FAILURE */
/*                        - If the entire msg length is less than current    */
/*                          fragment length then return FAILURE              */
/*                        - Allocate memory for the entire eap packet. If    */
/*                          Memory allocation fails then reset the           */
/*                          defragmentation status flag and return FAILURE   */
/*                     2) If there is a Opcode mismatch or if the length of  */
/*                        total message specified in first fragment is less  */
/*                        that the total length received, then this is an    */
/*                        exception condition or if the LF bit is set for    */
/*                        subsequent fragments, free the allocated memory and*/
/*                        return FAILURE                                     */
/*                     3) Copy the received packet in locally allocated      */
/*                        buffer at appropriate offset                       */
/*                     4) If MF bit is set then send WSC_FRAG_ACK and exit   */
/*                     4) Based on the WSC Op-Code call appropriate function */
/*                        for further processing                             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         22 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_handle_eap_wsc_defrag(wps_t *wps_ctxt_hdl,
                                          wps_reg_t *reg_hdl,
                                          wps_eap_rx_t *eap_rx_hdl,
                                          wps_eap_tx_t *eap_tx_hdl,
                                          UWORD8 *msg_ptr,
                                          UWORD16 msg_len, UWORD16 frag_len,
                                          UWORD8 flags, UWORD8 opcode)
{
    static UWORD8 wsc_opcode = 0;
    BOOL_T first_frag = BFALSE;
    STATUS_T status = SUCCESS;

    /* If EAP RX pointer is NULL, then it indicates fragment message */
    if(NULL == eap_rx_hdl->rx_eap_msg)
    {
        /* Length Bit must be set and total EAP message length cannot be less*/
        /* that current fragment length                                      */
        if((BFALSE == is_wsc_lf_set(flags)) || (msg_len < frag_len))
        {
            eap_rx_hdl->defrag_in_prog = BFALSE;
            return FAILURE;
        }

        /* Allocate a frame memory entire EAP Message */
        eap_rx_hdl->rx_eap_msg = wps_pkt_mem_alloc(msg_len);

        /* If allocation fails then exit by resetting the  */
        if(NULL == eap_rx_hdl->rx_eap_msg)
        {
            eap_rx_hdl->defrag_in_prog = BFALSE;
            wps_handle_sys_err_reg(NO_SHRED_MEM);
            return FAILURE;
        }

        /* Update the total message length, pointer to EAP message and opcode*/
        eap_rx_hdl->rx_eap_msg_len = msg_len;
        eap_rx_hdl->rx_m_msg_ptr   = eap_rx_hdl->rx_eap_msg;
        wsc_opcode = opcode;
        first_frag = BTRUE;
    }

    /*************************************************************************/
    /* If there is a Opcode mismatch or if the length of total message       */
    /* specified in first fragment is less that the total length received,   */
    /* then this is an exception condition or if the LF is set for the non   */
    /* first fragment                                                        */
    /*************************************************************************/
    if((wsc_opcode != opcode) ||
       ((BFALSE == first_frag) && (BTRUE == is_wsc_lf_set(flags))) ||
       ((eap_rx_hdl->rx_m_msg_ptr + frag_len - eap_rx_hdl->rx_eap_msg) >
         eap_rx_hdl->rx_eap_msg_len))
    {
        wps_pkt_mem_free(eap_rx_hdl->rx_eap_msg);
        eap_rx_hdl->rx_m_msg_ptr   = NULL;
        eap_rx_hdl->defrag_in_prog = BFALSE;
        return FAILURE;
    }

    memcpy(eap_rx_hdl->rx_m_msg_ptr, msg_ptr, frag_len);
    eap_rx_hdl->rx_m_msg_ptr += frag_len;

    /*************************************************************************/
    /* If MF bit is set then there are more EAP fragments expected. Send a   */
    /* WSC_FRAG_ACK to the Enrollee                                          */
    /*************************************************************************/
    if(BTRUE == is_wsc_mf_set(flags))
    {
        /* Stop the EAP Message timer before proceeding any further */
        wps_stop_eap_msg_timer(wps_ctxt_hdl);

        if(SUCCESS != wps_send_wsc_frag_ack(wps_ctxt_hdl, eap_tx_hdl))
            return FAILURE;

        /* Reset the EAP retry count */
        eap_tx_hdl->eap_msg_retry_cnt = MAX_EAP_MSG_RETRY_LIMIT;

        /* Start EAP Message timer */
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;

        return SUCCESS;
    }

    /*************************************************************************/
    /* If MF is not set then this os the last fragment, hence process the    */
    /* frame further based on its opcode                                     */
    /*************************************************************************/
    eap_rx_hdl->rx_m_msg_ptr = eap_rx_hdl->rx_eap_msg;
    eap_rx_hdl->rx_m_msg_len = eap_rx_hdl->rx_eap_msg_len;

    switch(opcode)
    {
        case WPS_OP_CODE_WSC_MSG:
            status = wps_handle_eap_wsc_msg_rx(wps_ctxt_hdl, reg_hdl,
                                               eap_rx_hdl, eap_tx_hdl);
        break;
        case WPS_OP_CODE_WSC_ACK:
        case WPS_OP_CODE_WSC_NACK:
        case WPS_OP_CODE_WSC_DONE:
            status = wps_handle_eap_wsc_ack_nack_done_rx(wps_ctxt_hdl,
                                                         reg_hdl, eap_rx_hdl,
                                                         eap_tx_hdl, opcode);
        break;
        default:
            status = FAILURE;
        break;
    }

    /* Free all the locally allocated resources */
    wps_pkt_mem_free(eap_rx_hdl->rx_eap_msg);
    eap_rx_hdl->rx_eap_msg = NULL;
    eap_rx_hdl->rx_m_msg_ptr   = NULL;
    eap_rx_hdl->defrag_in_prog = BFALSE;

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_wsc_msg_rx                             */
/*                                                                           */
/*  Description      : This function handles reception of EAP WSC messages of*/
/*                     Op-Code type WSC_MSG                                  */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If the registrar is not in correct state to receive*/
/*                        a WSC-MSG then return FAILURE                      */
/*                     2) Parse the received WSC TLV attributes              */
/*                     3) Check if the WPS version number is supported, if   */
/*                        not then return FAILURE                            */
/*                     4) Based on the received Message type call appropriate*/
/*                        functions for further processing                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         22 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_handle_eap_wsc_msg_rx(wps_t *wps_ctxt_hdl,
                                          wps_reg_t *reg_hdl,
                                          wps_eap_rx_t *eap_rx_hdl,
                                          wps_eap_tx_t *eap_tx_hdl)
{
    STATUS_T status = SUCCESS;
    wps_rx_attri_t wps_rx_attr = {0};
    wps_rx_attri_t *rx_attr = &wps_rx_attr;

    /*************************************************************************/
    /* The Registration Protocol State of the registrar  must be in a state  */
    /* to receive WSC-MSG                                                    */
    /*************************************************************************/
    if(REG_INIT_ST > reg_hdl->reg_prot_state)
        return FAILURE;

    /*************************************************************************/
    /* Parse the WSC attributes in received message. If the Parsing is not   */
    /* successful then return FAILURE                                        */
    /*************************************************************************/
    if(1 != wps_parse_attri(eap_rx_hdl->rx_m_msg_ptr, &rx_attr,
                            eap_rx_hdl->rx_m_msg_len, 1, BFALSE))
        return FAILURE;

    /* Check for WPS version2 number support */
    if((BTRUE == reg_hdl->ver2_cap_sta) &&
       (BFALSE == wps_check_ver2_supp(wps_rx_attr.version2)))
        return FAILURE;

    /* Call functions to process further based on the message type */
    switch(get_wsc_msg_type(wps_rx_attr.msg_type))
    {
        case WPS_MSG_M1:
            status = wps_process_m1_rx(wps_ctxt_hdl, reg_hdl, eap_rx_hdl,
                                       eap_tx_hdl, rx_attr);
        break;

        case WPS_MSG_M3:
            status = wps_process_m3_rx(wps_ctxt_hdl, reg_hdl, eap_rx_hdl,
                                       eap_tx_hdl, rx_attr);
        break;

        case WPS_MSG_M5:
            status = wps_process_m5_rx(wps_ctxt_hdl, reg_hdl, eap_rx_hdl,
                                       eap_tx_hdl, rx_attr);
        break;

        case WPS_MSG_M7:
            status = wps_process_m7_rx(wps_ctxt_hdl, reg_hdl, eap_rx_hdl,
                                       eap_tx_hdl, rx_attr);
        break;

        default:
            status = FAILURE;
        break;
    }

    return status;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : wps_start_deauth_time                                    */
/*                                                                           */
/*  Description   : This function starts the timer before sending deauth     */
/*                  frame                                                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to main WPS context structure                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function  creats the timer and starts the           */
/*                  alarm to wait before sending the deauth frame to the     */
/*                  enrollee after EAPOL frame has been sent. During this    */
/*                  time the Registrar waits for the Enrollee to send        */
/*                  Deauth/Disassoc/Auth frame. If none of these frames are  */
/*                  received by the Registrar from the Enrollee during this  */
/*                  time then upon timeout the Registrar calls the           */
/*                  sys_stop_wps_reg() function to send deauth frame to the  */
/*                  enrollee and stop WPS protocol                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : STATUS_T; FAILURE                                        */
/*                            SUCCESS                                        */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf add, merged ittiam mac v1.2 code
STATUS_T wps_start_deauth_timer(wps_t *wps_ctxt_hdl)
{
    /* Create alarm if not created */
    if(NULL == wps_ctxt_hdl->deauth_timer)
    {
        wps_ctxt_hdl->deauth_timer = wps_create_alarm(
                                          wps_ctxt_hdl->wps_timer_cb_fn_ptr,
                                          WPS_DEAUTH_TIMEOUT_VALUE, NULL); //Hugh: fixme.

        /* If creation of alarm fails then raise system error and return     */
        /* FAILURE                                                           */
        if(NULL == wps_ctxt_hdl->deauth_timer)
        {
            wps_handle_sys_err_reg(SYSTEM_ERROR);
            return FAILURE;
        }
    }
    /* Stop the alarm if running */
    else
        wps_stop_alarm(wps_ctxt_hdl->deauth_timer);

    /* Start the alarm */
    if(BFALSE == wps_start_alarm(wps_ctxt_hdl->deauth_timer,
                                 WPS_DEAUTH_TIMEOUT_VALUE))
    {
        wps_handle_sys_err_reg(SYSTEM_ERROR);
        return FAILURE;
    }

    return SUCCESS;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_wsc_ack_nack_done_rx                   */
/*                                                                           */
/*  Description      : This function handles reception of EAP WSC messages of*/
/*                     Op-Code type WSC_ACK/WSC_NACK/WSC_DONE                */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) WSC Opcode of the received frame                   */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : None                                                  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         22 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_handle_eap_wsc_ack_nack_done_rx(wps_t *wps_ctxt_hdl,
                                                    wps_reg_t *reg_hdl,
                                                    wps_eap_rx_t *eap_rx_hdl,
                                                    wps_eap_tx_t *eap_tx_hdl,
                                                    UWORD8 opcode)
{
    UWORD8 msg_type = 0;
    wps_rx_attri_t wps_rx_attr = {0};
    wps_rx_attri_t *rx_attr    = &wps_rx_attr;
    wps_attr_t     *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_config_t   *config_hdl = wps_ctxt_hdl->wps_config_hdl;

    /* If not in correct state to process the current packet then exit       */
    if((REG_MSG_ST != reg_hdl->reg_prot_state) ||
       ((WPS_OP_CODE_WSC_ACK == opcode) && (M2D_ST != reg_hdl->msg_state)) ||
       ((WPS_OP_CODE_WSC_NACK == opcode) && (M2_ST > reg_hdl->msg_state))  ||
       ((WPS_OP_CODE_WSC_DONE == opcode) && (M8_ST != reg_hdl->msg_state)))
        return FAILURE;

    /*************************************************************************/
    /* Parse the WSC attributes in received message. If the Parsing is not   */
    /* successful then return FAILURE                                        */
    /*************************************************************************/
    if(0 == wps_parse_attri(eap_rx_hdl->rx_m_msg_ptr, &rx_attr,
                            eap_rx_hdl->rx_m_msg_len, 1, BFALSE))
        return FAILURE;

    /* Check if any mandatory attribute is missing                           */
    if(BFALSE == all_req_attr_present(WPS_MSG_WSC_ACK, rx_attr))
        return FAILURE;

    /* If not supported WPS version then exit */
    if((BTRUE == reg_hdl->ver2_cap_sta) &&
       (BFALSE == wps_check_ver2_supp(wps_rx_attr.version2)))
        return FAILURE;

    msg_type = get_wsc_msg_type(wps_rx_attr.msg_type);

    /* If Message Type is valid then exit */
    if(((WPS_OP_CODE_WSC_ACK  == opcode) && (WPS_MSG_WSC_ACK  != msg_type)) ||
       ((WPS_OP_CODE_WSC_NACK == opcode) && (WPS_MSG_WSC_NACK != msg_type)) ||
       ((WPS_OP_CODE_WSC_DONE == opcode) && (WPS_MSG_WSC_DONE != msg_type)))
        return FAILURE;

    /* Check if the Enrollee Nonce matches */
    if(0 != memcmp(wps_rx_attr.enr_nonce, attr_hdl->enr_nonce,
                   WPS_NONCE_LEN))
        return FAILURE;

    /* Check if the Registrar Nonce matches */
    if(0 != memcmp(wps_rx_attr.reg_nonce, attr_hdl->reg_nonce,
                   WPS_NONCE_LEN))
        return FAILURE;

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    /* Terminate the Registration Protocol by sending an EAP-FAIL to Enrollee*/
    if(SUCCESS == wps_send_eap_fail(wps_ctxt_hdl, eap_tx_hdl))
    {
        UWORD16 reason_code = UNSPEC_REASON;

        /* Send appropriate status to the host */
        if(WPS_OP_CODE_WSC_ACK  == opcode)
        {
            send_wps_status(WPS_REG_M2D_SUCCESS, NULL, 0);

            /******************************************************************/
            /* Stop the Registrar and do not change the selected registrar    */
            /* attribute value                                                */
            /******************************************************************/
            sys_stop_wps_reg(reason_code, config_hdl->mac_addr,
                             reg_hdl->start_req);
        }
        else if(WPS_OP_CODE_WSC_NACK  == opcode)
        {
            send_wps_status(WPS_REG_NACK_RX_FAIL, wps_rx_attr.config_error, 2);

            /******************************************************************/
            /* Indicate to the AP that WPS Protocol with the Enrollee has    */
            /* failed.                                                       */
            /******************************************************************/
            wps_update_failed_enr_list(config_hdl->mac_addr);

            /******************************************************************/
            /* Stop the Registrar and do not change the selected registrar    */
            /* attribute value                                                */
            /******************************************************************/
            reason_code = AUTH_1X_FAIL;
            sys_stop_wps_reg(reason_code, config_hdl->mac_addr,
                             reg_hdl->start_req);
        }
        else
        {
            /*****************************************************************/
            /* On successful completion of PBC protocol delete the Enrollee  */
            /* from PBC Enrollee list                                        */
            /*****************************************************************/
            if(PBC == config_hdl->prot_type)
            {
                if(1 != reg_hdl->pbc_enr_cnt)
                    send_wps_status(WPS_REG_MONITOR_EXC2, NULL, 0);
                else
                    reg_hdl->pbc_enr_cnt = 0;
            }

            send_wps_status(WPS_REG_PROT_COMP_SUCCESS, NULL, 0);

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
            /* Stop the Registrar and reset the selected registrar attribute  */
            sys_stop_wps_reg(reason_code, config_hdl->mac_addr, BFALSE);
#else
            /* Start request is disabled as WPS is completed successfully */
            reg_hdl->start_req = BFALSE;

            /* Start a timer to wait for the enrollee to send DEAUTH/DISASSOC*/
            /*  /AUTH  frame. This is done to avoid sending of Deauth frame  */
            /* to the enrollee immediately after EAPOL faillure is sent      */
            wps_start_deauth_timer(wps_ctxt_hdl);
#endif
        }


        return SUCCESS;
    }

    return FAILURE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_m1_rx                                     */
/*                                                                           */
/*  Description      : This function handles reception of WPS M1 message     */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) Pointer to received attribute structure            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does following processing:              */
/*                     1) Check if the registrar is in correct state to RX M1*/
/*                     2) Check if mandatory attributes are present          */
/*                     3) Check if M2 has to be retried                      */
/*                     4) If there has been no Start Request from the user   */
/*                        then send M2D and return                           */
/*                     5) Check if the capabilities match                    */
/*                     6) Send M2 to Enrollee                                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         22 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_process_m1_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr)
{
    STATUS_T status          = SUCCESS;
    UWORD8  conn_type_flags  = 0;
    UWORD16 config_meth      = 0;
    UWORD16 auth_type_flags  = 0;
    UWORD16 encr_type_flags  = 0;
    wps_config_t *config_hdl = wps_ctxt_hdl->wps_config_hdl;
    wps_attr_t   *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;

    /* Check if all the required attributes are present in the received msg  */
    if(BFALSE == all_req_attr_present(WPS_MSG_M1, rx_attr))
        return FAILURE;

    /* If not Idle state then exit */
    if(IDLE_ST != reg_hdl->msg_state)
        return FAILURE;

    /*************************************************************************/
    /* Check of the UUID-E received in Probe Request and M1 message match for*/
    /* a  WPS 2.0 capable device. This check may not always pass for a WPS   */
    /* 1.0  devices as WSC-IE in management frames is not mandatory          */
    /*************************************************************************/
    if((BTRUE == reg_hdl->ver2_cap_sta) &&
       (BTRUE == reg_hdl->start_req)    &&
       (PBC == config_hdl->prot_type)   &&
       (BFALSE == wps_check_pbc_uuid(reg_hdl->pbc_enr_info,
                                     reg_hdl->pbc_enr_cnt, rx_attr->uuid)))
    {
        attr_hdl->config_error = MULTIPLE_PBC_SESSIONS_DETECTED;
        return wps_handle_m2d_tx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl);
    }

    /* Check for Enrollee`s MAC address */
    if(0 != memcmp(config_hdl->mac_addr, rx_attr->mac_addr, MAC_ADDRESS_LEN))
        return FAILURE;

    /* Copy Enrollee`s Nonce */
    memcpy(attr_hdl->enr_nonce, rx_attr->enr_nonce, WPS_NONCE_LEN);

    /*************************************************************************/
    /* If there has been no start request from the user, then send M2D to the*/
    /* enrollee                                                              */
    /*************************************************************************/
    if(BFALSE == reg_hdl->start_req)
        return wps_handle_m2d_tx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl);

    /*************************************************************************/
    /* Check if the Config Methods of registrar and enrollee match. If there */
    /* is a mismatch indicate the same to host and send M2D to enrollee      */
    /*************************************************************************/
    config_meth = get_wsc_config_meth(rx_attr->config_meth);
    if(BFALSE == is_common_config_meth(config_meth,
                                       config_hdl->config_methods))
    {
        send_wps_status(WPS_REG_CONF_METH_MISMATCH, NULL, 0);
        return wps_handle_m2d_tx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl);
    }

    /*************************************************************************/
    /* Check if the Connection Types of the registrar and enrollee           */
    /* match. If there is a mismatch indicate the same to host and send M2D  */
    /* to enrollee                                                           */
    /*************************************************************************/
    conn_type_flags = get_wsc_conn_type_flags(rx_attr->conn_type_flags);

    /*************************************************************************/
    /* This zero check has been added as a workaround for Intel Wi-Fi test   */
    /* bed STA`s bug. The STA sets the connection type flags attribute to 0. */
    /*************************************************************************/
    if((0 != conn_type_flags) &&
       (BFALSE == is_common_conn_type(conn_type_flags,
                                      config_hdl->conn_type_flags)))
    {
        send_wps_status(WPS_REG_CONN_TYPE_MISMATCH, NULL, 0);
        return wps_handle_m2d_tx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl);
    }

    /*************************************************************************/
    /* Check if the Authentication Types of the registrar and enrollee       */
    /* match. If there is a mismatch indicate the same to host and send M2D  */
    /* to enrollee                                                           */
    /*************************************************************************/
    auth_type_flags = get_wsc_auth_type_flags(rx_attr->auth_type_flags);
    if(BFALSE == is_common_auth_type(auth_type_flags,
                                     config_hdl->auth_type_flags))
    {
        send_wps_status(WPS_REG_AUTH_TYPE_MISMATCH, NULL, 0);
        return wps_handle_m2d_tx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl);
    }

    /*************************************************************************/
    /* Check if the Encryption Types of the registrar and enrollee           */
    /* match. If there is a mismatch indicate the same to host and send M2D  */
    /* to enrollee                                                           */
    /*************************************************************************/
    encr_type_flags = get_wsc_encr_type_flags(rx_attr->encr_type_flags);
    if(BFALSE == is_common_encr_type(encr_type_flags,
                                     config_hdl->encr_type_flags))
    {
        send_wps_status(WPS_REG_ENCR_TYPE_MISMATCH, NULL, 0);
        return wps_handle_m2d_tx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl);
    }

    /*************************************************************************/
    /* Check if the Device Password ID of the registrar and enrollee         */
    /* match. If there is a mismatch indicate the same to host and send M2D  */
    /* to enrollee                                                           */
    /*************************************************************************/
	// 20120709 caisf add, merged ittiam mac v1.2 code
#ifndef MAC_P2P
    if(config_hdl->dev_pass_id != get_wsc_dev_pass_id(rx_attr->dev_pass_id))
#else  /* MAC_P2P */
    if(BFALSE == check_wps_pid_reg(config_hdl->prot_type,
                                   get_wsc_dev_pass_id(rx_attr->dev_pass_id)))
#endif /* MAC_P2P */
    {
        send_wps_status(WPS_REG_PASS_ID_MISMATCH, NULL, 0);
        return wps_handle_m2d_tx(wps_ctxt_hdl, reg_hdl, eap_tx_hdl);
    }

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    /* Extract the Enrollee`s UUID */
    memcpy(attr_hdl->enr_uuid, rx_attr->uuid, WPS_UUID_LEN);

    /* Send M2 message to Enrollee */
    status = wps_send_m2(wps_ctxt_hdl, eap_tx_hdl, eap_rx_hdl,
                         rx_attr->pub_key);

    if(status != FAILURE)
    {
        reg_hdl->reg_prot_state = REG_MSG_ST;
        reg_hdl->msg_state = M2_ST;
        eap_tx_hdl->eap_msg_retry_cnt = 0;

        /* Start EAP Message timer */
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;
    }

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_m3_rx                                     */
/*                                                                           */
/*  Description      : This function handles reception of WPS M3 message     */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) Pointer to received attribute structure            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function processes the received M3 message. It   */
/*                     verifies if all the authenticator attribute is correct*/
/*                     It then saves the Enrollee`s Hash1 and Hash2. Sends M4*/
/*                     to Enrollee and returns                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_process_m3_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr)
{
    STATUS_T       status      = SUCCESS;
    wps_attr_t     *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t     *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;
    wps_sec_key_t  *sec_key_hdl = &(priv_hdl->olap_priv_key.secret_keys);

    /*************************************************************************/
    /* WPS State must indicate that Registrar is in Registration Protocol.   */
    /* Registration Protocol State must indicate the registrar to be in MSG  */
    /* state, indicating that the registrar is in WSC Message Transaction    */
    /* state i.e doing M1-M8 transaction                                     */
    /*************************************************************************/
    if(REG_MSG_ST != reg_hdl->reg_prot_state)
        return FAILURE;

    /*************************************************************************/
    /* Check is all the required attributes are present in the received msg  */
    /* Check if the received registrar nonce is correct                      */
    /*************************************************************************/
    if((BFALSE == all_req_attr_present(WPS_MSG_M3, rx_attr)) ||
       (0 != memcmp(attr_hdl->reg_nonce, rx_attr->reg_nonce, WPS_NONCE_LEN)))
    {
        send_wps_status(WPS_REG_RXED_WRONG_M3, NULL, 0);
        return FAILURE;
    }

    /* If M2 has not yet been transmitted then exit */
    if(M2_ST != reg_hdl->msg_state)
        return FAILURE;

    /* Verify the authenticator attribute */
    if(FAILURE == wps_verify_auth_attr(eap_rx_hdl->rx_m_msg_ptr,
                                       eap_rx_hdl->rx_m_msg_len,
                                       eap_tx_hdl->tx_m_msg_ptr,
                                       eap_tx_hdl->tx_m_msg_len,
                                       sec_key_hdl->auth_key))
    {
        send_wps_status(WPS_REG_RXED_WRONG_M3, NULL, 0);
        return FAILURE;
    }

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    /* Store the Enrollee PIN Hash Values */
    memcpy(reg_hdl->enr_hash1, rx_attr->e_hash1, WPS_HASH_LEN);
    memcpy(reg_hdl->enr_hash2, rx_attr->e_hash2, WPS_HASH_LEN);

    /* Send M4 message to Enrollee */
    status = wps_send_m4(wps_ctxt_hdl, eap_tx_hdl, eap_rx_hdl);

    if(FAILURE != status)
    {
        reg_hdl->msg_state = M4_ST;
        eap_tx_hdl->eap_msg_retry_cnt = 0;

        /* Start EAP Message timer */
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;
    }

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_m5_rx                                     */
/*                                                                           */
/*  Description      : This function handles reception of WPS M5 message     */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) Pointer to received attribute structure            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If not in correct state then return FAILURE        */
/*                     2) Check if all Mandatory attributes are present.     */
/*                        Also check if the registrar nonce value is correct */
/*                     3) Retry M6 if required                               */
/*                     4) Verify the authenticator attribute                 */
/*                     5) Decrypt the encrpypted attribute and verify if the */
/*                        Enrollee Hash1 matches with the one received in M3 */
/*                     6) Send M6 to Enrollee                                */
/*                     7) If any of the above steps fails then return FAILURE*/
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_process_m5_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr)
{
    STATUS_T       status      = SUCCESS;
    UWORD8         *decry_data;
    WORD16         decry_len;
    wps_rx_attri_t decr_attr;
    wps_rx_attri_t *dec_attr_ptr = &decr_attr;
    wps_attr_t     *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t     *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;
    wps_sec_key_t  *sec_key_hdl = &(priv_hdl->olap_priv_key.secret_keys);

    /*************************************************************************/
    /* WPS State must indicate that Registrar is in Registration Protocol.   */
    /* Registration Protocol State must indicate the registrar to be in MSG  */
    /* state, indicating that the registrar is in WSC Message Transaction    */
    /* state i.e doing M1-M8 transaction                                     */
    /*************************************************************************/
    if(REG_MSG_ST != reg_hdl->reg_prot_state)
        return FAILURE;

    /*************************************************************************/
    /* Check is all the required attributes are present in the received msg  */
    /* Check if the received registrar nonce is correct                      */
    /*************************************************************************/
    if((BFALSE == all_req_attr_present(WPS_MSG_M5, rx_attr)) ||
       (0 != memcmp(attr_hdl->reg_nonce, rx_attr->reg_nonce, WPS_NONCE_LEN)))
    {
        send_wps_status(WPS_REG_RXED_WRONG_M5, NULL, 0);
        return FAILURE;
    }

    /* If M4 has not yet been transmitted then exit */
    if(M4_ST != reg_hdl->msg_state)
        return FAILURE;

    /* Verify the authenticator attribute */
    if(FAILURE == wps_verify_auth_attr(eap_rx_hdl->rx_m_msg_ptr,
                                       eap_rx_hdl->rx_m_msg_len,
                                       eap_tx_hdl->tx_m_msg_ptr,
                                       eap_tx_hdl->tx_m_msg_len,
                                       sec_key_hdl->auth_key))
    {
        send_wps_status(WPS_REG_RXED_WRONG_M5, NULL, 0);
        return FAILURE;
    }

    /*************************************************************************/
    /* Allocate temporary buffer for the decrypted data and decrypt the      */
    /* Encrypted settings.                                                   */
    /*************************************************************************/
    decry_len = (WORD16)(rx_attr->encrypted_settings_len -
                         WPS_AES_BYTE_BLOCK_SIZE);

    if(NULL == (decry_data = wps_local_mem_alloc(decry_len)))
    {
        wps_handle_sys_err_reg(NO_LOCAL_MEM);
        return FAILURE;
    }

    decry_len = wps_decrypt_encrypted_setting(sec_key_hdl->key_wrap_key,
                                              rx_attr->encrypted_settings,
                                              rx_attr->encrypted_settings_len,
                                              decry_data);

    if(0 >= decry_len)
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_RXED_WRONG_M5);
        return FAILURE;
    }

    /* Parse the decrypted data and get all the required fields */
    if(1 != wps_parse_attri(decry_data, &dec_attr_ptr, decry_len, 1, BFALSE))
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_RXED_WRONG_M5);
        return FAILURE;
    }

    /* Verify the Key Wrap Authenticator from the decrypted settings */
    if(FAILURE == wps_verify_key_wrap_auth(sec_key_hdl->auth_key,
                                           decry_data, decry_len))
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_RXED_WRONG_KWA_M5);
        return FAILURE;
    }

    /* Verify E-SNonce1 passed in the decrypted settings */
    if(FAILURE == wps_verify_pin_snonce(attr_hdl->enr_pub_key.pk,
                                        attr_hdl->reg_pub_key.pk,
                                        decr_attr.e_snonce1,
                                        reg_hdl->enr_hash1,
                                        sec_key_hdl->psk1,
                                        sec_key_hdl->auth_key))
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_PIN_FAILURE_M5);
        return FAILURE;
    }
    wps_local_mem_free(decry_data);

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    /* Send M6 message to Enrollee */
    status = wps_send_m6(wps_ctxt_hdl, eap_tx_hdl, eap_rx_hdl);

    if(status != FAILURE)
    {
        reg_hdl->msg_state = M6_ST;
        eap_tx_hdl->eap_msg_retry_cnt = 0;

        /* Start EAP Message timer */
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;
    }

    return status;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_m7_rx                                     */
/*                                                                           */
/*  Description      : This function handles reception of WPS M7 message     */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Handle to WPS Registrar structure                  */
/*                     3) Handle to WPS EAP-TX structure                     */
/*                     4) Handle to WPS EAP-RX structure                     */
/*                     5) Pointer to received attribute structure            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If not in correct state then return FAILURE        */
/*                     2) Check if all Mandatory attributes are present.     */
/*                        Also check if the registrar nonce value is correct */
/*                     3) Retry M8 if required                               */
/*                     4) Verify the authenticator attribute                 */
/*                     5) Decrypt the encrpypted attribute and verify if the */
/*                        Enrollee Hash2 matches with the one received in M3 */
/*                     6) Send M6 to Enrollee                                */
/*                     7) If any of the above steps fails then return FAILURE*/
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : SUCCESS - On Successful processing of Packet          */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         23 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static STATUS_T wps_process_m7_rx(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                  wps_eap_rx_t *eap_rx_hdl,
                                  wps_eap_tx_t *eap_tx_hdl,
                                  wps_rx_attri_t *rx_attr)
{
    STATUS_T       status      = SUCCESS;
    UWORD8         *decry_data;
    WORD16         decry_len;
    wps_rx_attri_t decr_attr;
    wps_rx_attri_t *dec_attr_ptr = &decr_attr;
    wps_attr_t     *attr_hdl   = wps_ctxt_hdl->wps_attr_hdl;
    wps_priv_t     *priv_hdl   = wps_ctxt_hdl->wps_priv_hdl;
    wps_sec_key_t  *sec_key_hdl = &(priv_hdl->olap_priv_key.secret_keys);

    /*************************************************************************/
    /* WPS State must indicate that Registrar is in Registration Protocol.   */
    /* Registration Protocol State must indicate the registrar to be in MSG  */
    /* state, indicating that the registrar is in WSC Message Transaction    */
    /* state i.e doing M1-M8 transaction                                     */
    /*************************************************************************/
    if(REG_MSG_ST != reg_hdl->reg_prot_state)
        return FAILURE;

    /*************************************************************************/
    /* Check is all the required attributes are present in the received msg  */
    /* Check if the received registrar nonce is correct                      */
    /*************************************************************************/
    if((BFALSE == all_req_attr_present(WPS_MSG_M7, rx_attr)) ||
       (0 != memcmp(attr_hdl->reg_nonce, rx_attr->reg_nonce, WPS_NONCE_LEN)))
    {
        send_wps_status(WPS_REG_RXED_WRONG_M7, NULL, 0);
        return FAILURE;
    }

    /* If M6 has not yet been transmitted then exit */
    if(M6_ST != reg_hdl->msg_state)
        return FAILURE;

    /* Verify the authenticator attribute */
    if(FAILURE == wps_verify_auth_attr(eap_rx_hdl->rx_m_msg_ptr,
                                       eap_rx_hdl->rx_m_msg_len,
                                       eap_tx_hdl->tx_m_msg_ptr,
                                       eap_tx_hdl->tx_m_msg_len,
                                       sec_key_hdl->auth_key))
    {
        send_wps_status(WPS_REG_RXED_WRONG_M7, NULL, 0);
        return FAILURE;
    }

    /*************************************************************************/
    /* Allocate temporary buffer for the decrypted data and decrypt the      */
    /* Encrypted settings.                                                   */
    /*************************************************************************/
    decry_len = (WORD16)(rx_attr->encrypted_settings_len -
                         WPS_AES_BYTE_BLOCK_SIZE);

    decry_data = wps_local_mem_alloc(decry_len);

    if(NULL == decry_data)
    {
        wps_handle_sys_err_reg(NO_LOCAL_MEM);
        return FAILURE;
    }

    decry_len = wps_decrypt_encrypted_setting(sec_key_hdl->key_wrap_key,
                                              rx_attr->encrypted_settings,
                                              rx_attr->encrypted_settings_len,
                                              decry_data);

    if(0 >= decry_len)
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_RXED_WRONG_M7);
        return FAILURE;
    }

    /* Parse the decrypted data and get all the required fields */
    if(1 != wps_parse_attri(decry_data, &dec_attr_ptr, decry_len, 1, BFALSE))
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_RXED_WRONG_M7);
        return FAILURE;
    }

    /* Verify the Key Wrap Authenticator from the decrypted settings */
    if(FAILURE == wps_verify_key_wrap_auth(sec_key_hdl->auth_key,
                                           decry_data, decry_len))
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_RXED_WRONG_KWA_M7);
        return FAILURE;
    }

    /* Verify E-SNonce2 passed in the decrypted settings */
    if(FAILURE == wps_verify_pin_snonce(attr_hdl->enr_pub_key.pk,
                                        attr_hdl->reg_pub_key.pk,
                                        decr_attr.e_snonce2,
                                        reg_hdl->enr_hash2,
                                        sec_key_hdl->psk2,
                                        sec_key_hdl->auth_key))
    {
        wps_local_mem_free(decry_data);
        wps_handle_msg_failure(wps_ctxt_hdl, reg_hdl, eap_tx_hdl,
                               WPS_REG_PIN_FAILURE_M7);
        return FAILURE;
    }
    wps_local_mem_free(decry_data);

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    /* Send M8 message to Enrollee */
    status = wps_send_m8(wps_ctxt_hdl, eap_tx_hdl, eap_rx_hdl);

    if(status != FAILURE)
    {
        reg_hdl->msg_state = M8_ST;
        eap_tx_hdl->eap_msg_retry_cnt = 0;
        if(SUCCESS != wps_start_eap_msg_timer(wps_ctxt_hdl))
            return FAILURE;
    }

    return status;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_msg_failure                                */
/*                                                                           */
/*  Description      : This function handles EAP Message Failure             */
/*                                                                           */
/*  Inputs           : 1) Pointer to main WPS context structure              */
/*                     2) Error Code                                         */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function is called whenever the registrar has to */
/*                     detected error in received EAP-WSC-MSG frame. It sends*/
/*                     WSC-NACK to the Enrollee and updates the Registrar`s  */
/*                     State NACK_ST and returns                             */
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

static void wps_handle_msg_failure(wps_t *wps_ctxt_hdl, wps_reg_t *reg_hdl,
                                   wps_eap_tx_t *eap_tx_hdl,
                                   WPSR_STATUS_T status)
{
    wps_attr_t *attr_hdl = wps_ctxt_hdl->wps_attr_hdl;

    /* Stop the EAP Message timer before proceeding any further */
    wps_stop_eap_msg_timer(wps_ctxt_hdl);

    /* Send the specified Status to Host */
    send_wps_status(status, NULL, 0);

    /* Set the Configuration Error Attribute */
    attr_hdl->config_error = DEVICE_PASSWORD_AUTH_FAILURE;

    /* Send WSC-NACK frame to Enrollee */
    if(SUCCESS == wps_send_wsc_nack(wps_ctxt_hdl, eap_tx_hdl))
    {
        /* If sending is successful then update the registrar`s state */
        reg_hdl->msg_state = NACK_ST;

        /*********************************************************************/
        /* Update the EAP retry counter to maximum value. NACK frame will not*/
        /* be retried                                                        */
        /*********************************************************************/
        eap_tx_hdl->eap_msg_retry_cnt = MAX_EAP_MSG_RETRY_LIMIT;

        /*********************************************************************/
        /* Start EAP message timer. No check is done if the timer was started*/
        /* correctly                                                         */
        /*********************************************************************/
        wps_start_eap_msg_timer(wps_ctxt_hdl);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : all_req_attr_present                                  */
/*                                                                           */
/*  Description      : This function checks if all the Mandatory WSC         */
/*                     attributes present in given Message                   */
/*                                                                           */
/*  Inputs           : 1) WSC Message type received                          */
/*                     2) Pointer to Parsed RX attribute structure           */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if all the mandatory WSC         */
/*                     attributes are present for a iven input message and   */
/*                     returns BFALSE if any of the attribute is missing     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE  - If all the mandatory attributes are present  */
/*                     BFALSE - Otherwise                                    */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 03 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

static BOOL_T all_req_attr_present(WPS_MSG_TYPE_T msg_type,
                                   wps_rx_attri_t *rx_attr)
{
    BOOL_T retval = BTRUE;

    if((NULL == rx_attr->version) ||
       (NULL == rx_attr->msg_type))
       retval = BFALSE;


    if((WPS_MSG_WSC_ACK == msg_type) || (WPS_MSG_WSC_NACK == msg_type) ||
       (WPS_MSG_WSC_DONE == msg_type))
    {
        if((NULL == rx_attr->reg_nonce) ||
           (NULL == rx_attr->enr_nonce))
           retval = BFALSE;
    }

    if(WPS_MSG_M1 == msg_type)
    {
        if((NULL == rx_attr->uuid)            ||
           (NULL == rx_attr->mac_addr)        ||
           (NULL == rx_attr->enr_nonce)       ||
           (NULL == rx_attr->pub_key)         ||
           (NULL == rx_attr->auth_type_flags) ||
           (NULL == rx_attr->encr_type_flags) ||
           (NULL == rx_attr->conn_type_flags) ||
           (NULL == rx_attr->config_meth)     ||
           (NULL == rx_attr->wps_state)       ||
           (NULL == rx_attr->manufacturer)    ||
           (NULL == rx_attr->model_name)      ||
           (NULL == rx_attr->model_number)    ||
           (NULL == rx_attr->serial_number)   ||
           (NULL == rx_attr->prim_dev_type)   ||
           (NULL == rx_attr->device_name)     ||
           (NULL == rx_attr->rf_bands)        ||
           (NULL == rx_attr->assoc_state)     ||
           (NULL == rx_attr->dev_pass_id)     ||
           (NULL == rx_attr->config_error)    ||
           (NULL == rx_attr->os_version))
            retval = BFALSE;
    }

    if(WPS_MSG_M3 == msg_type)
    {
        if((NULL == rx_attr->reg_nonce) ||
           (NULL == rx_attr->e_hash1)   ||
           (NULL == rx_attr->e_hash2)   ||
           (NULL == rx_attr->authenticator))
            retval = BFALSE;
    }

    if((WPS_MSG_M5 == msg_type) || (WPS_MSG_M7 == msg_type))
    {
        if((NULL == rx_attr->reg_nonce)          ||
           (NULL == rx_attr->encrypted_settings) ||
           (NULL == rx_attr->authenticator))
            retval = BFALSE;
    }

    return retval;
}

#endif /* INT_WPS_REG_SUPP */
