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
/*  File Name         : wps_init.c                                           */
/*                                                                           */
/*  Description       : This file contains the functions to initiate or stop */
/*                      WPS protocol                                         */
/*                                                                           */
/*  List of Functions : wps_start_enrollee                                   */
/*                      wps_stop_enrollee                                    */
/*                      wps_init_enrollee                                    */
/*                      wps_gen_uuid                                         */
/*                      wps_scan_complete                                    */
/*                      wps_assoc_comp                                       */
/*                      wps_process_disconnect                               */
/*                      wps_process_scan_rsp                                 */
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
#include "wpse_prot.h"
#include "wpse_dh_key.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define NAMESPACE_ID NAMESPACE_OID

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

#if NAMESPACE_ID == NAMESPACE_DNS
/*****************************************************************************/
/* Name string is a fully-qualified domain name                              */
/*****************************************************************************/
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x10, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#elif NAMESPACE_ID == NAMESPACE_URL
/*****************************************************************************/
/* Name string is a URL                                                      */
/*****************************************************************************/
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x11, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#elif NAMESPACE_ID == NAMESPACE_OID
/*****************************************************************************/
/* Name string is an ISO OID */
/*****************************************************************************/
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x12, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#elif NAMESPACE_ID == NAMESPACE_X500
/*****************************************************************************/
/* Name string is an X.500 DN (in DER or a text output format)               */
/*****************************************************************************/
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x6b, 0xa7, 0xb8, 0x14, 0x9d, 0xad, 0x11, 0xd1,
    0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8
};

#else
static const UWORD8 g_nsid[NSID_LEN] =
{
    0x52, 0x64, 0x80, 0xf8, 0xc9, 0x9b, 0x4b, 0xe5,
    0xa6, 0x55, 0x58, 0xed, 0x5f, 0x5d, 0x60, 0x84
};
#endif

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/
void wps_gen_uuid(UWORD8 *mac_addr, UWORD8 *uuid, UWORD8 *temp);

#ifndef MAC_P2P
static void wps_init_enrollee(wps_enrollee_t* wps_enrollee,
                              wps_enr_config_struct_t* config_ptr);
void reassemble_wsc_ie_enr(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                           UWORD8 *buffer);
BOOL_T is_wsc_ie_reassm_req_enr(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                                UWORD16 *total_wsc_ie_len, UWORD8 **buffer);
#endif /* MAC_P2P */

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_start_enrollee                                    */
/*                                                                           */
/*  Description      : This function starts the WPS Enrollee protocol        */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the structure where WPS configuration   */
/*                        data is stored                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function shall be called by system software when */
/*                     WPS protocol handshake is desired. This function does */
/*                     the initializations, configures the MAC in WPS scan   */
/*                     mode, sets the WPS Enrollee state variables, fires a  */
/*                     scan timeout timer and returns success/failure based  */
/*                     on whether it is able to successfully carry out all   */
/*                     the intended tasks                                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing(SUCCESS/FAILURE)    */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
//#pragma Otime
STATUS_T wps_start_enrollee(wps_enrollee_t* wps_enrollee,
                            wps_enr_config_struct_t* config_ptr)
{
    switch(config_ptr->prot_type)
    {
        /*********************************************************************/
        /* If Protocol type is PIN then check whether device password and    */
        /* device password ID are valid                                      */
        /*********************************************************************/
        case PIN:
            if((NULL == config_ptr->dev_pass)||(0 == config_ptr->dev_pass_len)
                || (MAX_WPS_PIN_LEN < config_ptr->dev_pass_len))
            {
                return FAILURE;
            }
            switch(config_ptr->dev_pass_id)
            {
                case DEV_PASS_ID_DEFAULT:
                case DEV_PASS_ID_USER_SPE:
                case DEV_PASS_ID_MACH_SPE:
                case DEV_PASS_ID_REKEY:
                case DEV_PASS_ID_PUSHBUTT:
                case DEV_PASS_ID_REG_SPE:
                    break;
                default:
                    return FAILURE;
            }
        case PBC:
            break;
        default:
            return FAILURE;
    } /* switch(config_ptr->prot_type) */

    wps_init_enrollee(wps_enrollee, config_ptr);

    wps_gen_uuid(config_ptr->mac_address, wps_enrollee->enr_uuid,
                 wps_enrollee->pke);

#ifdef RALINK_AP_WCN_EXT_REG_BUG_FIX
    /* Generate Enrollee Nonce now itself */
    wps_get_rand_byte_array(wps_enrollee->enr_nonce, WPS_NONCE_LEN);
#endif /* RALINK_AP_WCN_EXT_REG_BUG_FIX */

#ifdef OPTIMIZED_PKE_GENERATION
    /* Also generate the DH private key and public key as part of            */
    /* optimization                                                          */
    wps_get_rand_byte_array((UWORD8 *) wps_enrollee->a_nonce.val_ptr,
                            (UWORD16) WPS_PRIVATE_KEY_LEN * 4);
    wps_enrollee->a_nonce.msb_pos = wps_check_msb_pos(&(wps_enrollee->a_nonce));

    if(FAILURE ==
       wps_gen_pke((UWORD32 *)wps_enrollee->pke, &(wps_enrollee->a_nonce)))
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
#endif /* OPTIMIZED_PKE_GENERATION */

    start_wps_scan(BFALSE);
    /*************************************************************************/
    /* If Protocol type is PBC then starts the PBC walk timer. If timer      */
    /* allocation fails then stop the WPS protocol. Do it for other modes    */
    /* also as it is better and more secure to do a timebound protocol       */
    /*************************************************************************/
    if( NULL == (wps_enrollee->walk_timer = wps_create_alarm(
        config_ptr->timer_cb_fn_ptr, WPS_WALK_TIMEOUT, NULL)))  //Hugh: fixme.
    {
        wps_handle_sys_err(wps_enrollee);
        return FAILURE;
    }
    wps_start_alarm(wps_enrollee->walk_timer, WPS_WALK_TIME);
    wps_enrollee->process_walk_to = BFALSE;

    /*************************************************************************/
    /* WPS protocol started successfully, display the message                */
    /*************************************************************************/
    send_wps_status(IN_PROGRESS, NULL, 0);

    return SUCCESS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_stop_enrollee                                     */
/*                                                                           */
/*  Description      : This function stops the WPS enrollee protocol         */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function stops all the running timers and free   */
/*                     all the memory allocated                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         26 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
//#pragma Otime
void wps_stop_enrollee(wps_enrollee_t* wps_enrollee)
{
    /*************************************************************************/
    /* Stop and free all the timers                                          */
    /*************************************************************************/
    wps_delete_timers(wps_enrollee);
    if(NULL !=wps_enrollee->walk_timer)
    {
        //wps_stop_alarm(wps_enrollee->walk_timer);
        wps_delete_alarm(&(wps_enrollee->walk_timer));
        wps_enrollee->walk_timer = NULL;
    }

    /*************************************************************************/
    /* Free all the memory allocated                                         */
    /*************************************************************************/
    wps_free_all_memory(wps_enrollee);

    wps_enrollee->process_walk_to = BFALSE;
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_init_enrollee                                     */
/*                                                                           */
/*  Description      : This function initializes the wps enrollee structure  */
/*                     of type wps_enrollee_t to the initial value           */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                     2) Pointer to the structure where WPS configuration   */
/*                        data is stored                                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function initializes the wps enrollee structure  */
/*                     of type wps_enrollee_t to the initial value           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Status of the function processing(SUCCESS/FAILURE)    */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         26 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_init_enrollee(wps_enrollee_t* wps_enrollee,
                       wps_enr_config_struct_t* config_ptr)
{
    /*************************************************************************/
    /* Initialize whole memory to zero                                       */
    /*************************************************************************/
    memset((void *)wps_enrollee, 0, sizeof(wps_enrollee_t));

    wps_enrollee->config_ptr = config_ptr;
    wps_enrollee->req_type = ENROLLEE_INFO_ONLY;
    wps_enrollee->state = IN_SCAN;

    /*************************************************************************/
    /* Initialize private key (A nonce) and local public key (PKR) data      */
    /* structures                                                            */
    /*************************************************************************/
    wps_enrollee->a_nonce.val_ptr = wps_enrollee->non_olap.a_nonce_buff;
    wps_enrollee->a_nonce.length  = WPS_PRIVATE_KEY_LEN_WORD32;
    wps_enrollee->pke_big.val_ptr = (UWORD32 *) wps_enrollee->pke;
    wps_enrollee->pke_big.length  = WPS_DH_PRIME_LEN_WORD32;

    wps_enrollee->process_walk_to = BFALSE;

    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_gen_uuid                                          */
/*                                                                           */
/*  Description      : This function generate UUID using MAC address         */
/*                                                                           */
/*  Inputs           : 1) Pointer to Mac Address                             */
/*                   : 2) Pointer to buffer where UUID needs to be stored    */
/*                   : 3) Pointer to temporary buffer of length >=           */
/*                        SHA1_DIGEST_LEN                                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function generates named based UUID using        */
/*                     MAC-Address. It uses SHA1 for hashing                 */
/*                                                                           */
/*  Outputs          : UUID                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         26 02 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
void wps_gen_uuid(UWORD8 *mac_addr, UWORD8 *uuid, UWORD8 *temp)
{
    memcpy(temp, g_nsid, NSID_LEN);
    memcpy(temp+16, mac_addr, WPS_MAC_ADDR_LEN);

    sha1(temp, (UWORD32)(WPS_MAC_ADDR_LEN + NSID_LEN ), temp);
    memcpy(uuid, temp, WPS_UUID_LEN);

    /*************************************************************************/
    /* Version: 5 = The name-based version specified in RFC 4122 that uses   */
    /* SHA-1 hashing.                                                        */
    /*************************************************************************/
    uuid[6] = (5 << 4) | (uuid[6] & 0x0f);

    /*************************************************************************/
    /* Variant specified in RFC 4122 (10 in two MSBS)                        */
    /*************************************************************************/
    uuid[8] = 0x80 | (uuid[8] & 0x3f);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_scan_complete                                     */
/*                                                                           */
/*  Description      : This function handles the Scan complete  event        */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function is called by System SW or MAC SW once   */
/*                     the scan is completed. If a compatible AP/Registrar is*/
/*                     found then it issues a Join request to MAC else it    */
/*                     issues a Scan request                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Action that MAC needs to take                         */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         02 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
WPS_ACTION_T wps_scan_complete(wps_enrollee_t* wps_enrollee)
{
    /*************************************************************************/
    /* If WPS Enrollee is not in Scan state then MAC should not call this API*/
    /* so check if not in Scan state then stop enrollee and indicate system  */
    /* error                                                                 */
    /*************************************************************************/
    if(IN_SCAN != wps_enrollee->state)
    {
        wps_handle_sys_err(wps_enrollee);
        return(WPS_INVALID);
    }
    if(((1 << (8 * sizeof(wps_enrollee->scan_cnt))) - 1)
        > wps_enrollee->scan_cnt)
    {
        wps_enrollee->scan_cnt++;
    }
    /*************************************************************************/
    /* Check if any AP or registrar was selected, if so then give Join       */
    /* to MAC else give Scan request and make discarded AP/REG list empty    */
    /*************************************************************************/
    if( (PBC ==wps_enrollee->config_ptr->prot_type) &&
        (wps_enrollee->sel_ap_info.reg_num > 0))
    {
        /*************************************************************************/
        /* Scan for PBC_SCN_CNT_SESSION_OVERLAP number of times to ensure that   */
        /* only one registrar is in PBC mode                                     */
        /*************************************************************************/
        if(PBC_SCN_CNT_SESSION_OVERLAP > wps_enrollee->scan_cnt)
        {
            return (CONT_SCAN);
        }
        wps_enrollee->req_type = ENROLLEE_OPEN_802_1X;
        wps_enrollee->state = IN_JOIN;
        return (JOIN_NW);
    }
    wps_enrollee->discarded_reg_cnt = 0;
    memset(wps_enrollee->dis_ap_reg_list, 0,
        sizeof(wps_enrollee->dis_ap_reg_list));
#ifdef WPS_EXT_REG_SEL_REG_BUG_FIX
    if(IGNORE_SEL_REG_FIELD_THESH_CNT > wps_enrollee->scan_cnt)
    {
        wps_enrollee->ignore_sel_reg_field = BTRUE;
        wps_enrollee->scan_cnt = 0;
    }
#endif /* WPS_EXT_REG_SEL_REG_BUG_FIX */
    return (CONT_SCAN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_assoc_comp                                        */
/*                                                                           */
/*  Description      : This function handles the Association complete  event */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function is called by System SW or MAC SW once   */
/*                     the Join request has been completed. If join is       */
/*                     successful then this function starts the WPS          */
/*                     registration protocol else restarts the scan. If in   */
/*                     PBC mode, in case of failure, WPS protocol is stopped */
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
void wps_assoc_comp(wps_enrollee_t* wps_enrollee,
                    WPS_ASSOC_STATUS_T assoc_status)
{
    /*************************************************************************/
    /* If WPS Enrollee is not in Scan state then MAC should not call this API*/
    /* so check if not in Scan state then stop enrollee and indicate system  */
    /* error                                                                 */
    /*************************************************************************/
    if(IN_JOIN != wps_enrollee->state)
    {
        wps_handle_sys_err(wps_enrollee);
        return;
    }
    /*************************************************************************/
    /* If Association not successful then add the AP/Registrar to discarded  */
    /* list and restart the scan                                             */
    /* If Association successful then start the EAP based WPS registration   */
    /* protocol                                                              */
    /*************************************************************************/
    if(WPS_ASSOC_SUCCESS != assoc_status)
    {
        send_wps_status(ERR_ASSOC_FAIL, NULL, 0);
        /*********************************************************************/
        /* Add all the UUIDs associated with current AP to discarded list    */
        /*********************************************************************/
        wps_add_discarded_reg_ap(wps_enrollee,
            wps_enrollee->sel_ap_info.bssid);
        wps_loc_start_scan(wps_enrollee);
    }
    else
    {
        send_wps_status(ASSOC_PASS, NULL, 0);
        wps_enrollee->assoc_state = WPS_NOT_ASSOCIATED;
        wps_eap_reg_start(wps_enrollee);
    }
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reassemble_wsc_ie_enr                                 */
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

void reassemble_wsc_ie_enr(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
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
/*  Function Name    : is_wsc_ie_reassm_req_enr                              */
/*                                                                           */
/*  Description      : This function checks if reassembly of WSC-IE is       */
/*                     required. If WSC-IE(s) is present then the total      */
/*                     length of all WSC-IEs is returned                     */
/*                                                                           */
/*  Inputs           : 1) Pointer to the received frame                      */
/*                     2) Length of the received frame                       */
/*                     3) Frame Type                                         */
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

BOOL_T is_wsc_ie_reassm_req_enr(UWORD8 *msa, UWORD16 rx_len, UWORD16 index,
                                UWORD16 *total_wsc_ie_len, UWORD8 **buffer)
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
/*  Function Name    : wps_process_disconnect                                */
/*                                                                           */
/*  Description      : This function handles the disconnect event            */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function is called by System SW or MAC SW when   */
/*                     it detects a disconect                                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : SUCCESS/FAILURE                                       */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         04 06 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
STATUS_T wps_process_disconnect(wps_enrollee_t* wps_enrollee)
{
    /*************************************************************************/
    /* Add the AP/Registrar to the discarded list and restart the scan       */
    /*************************************************************************/
    send_wps_status(ERR_STA_DISCONNECT, NULL, 0);
    if(IN_SCAN != wps_enrollee->state)
    {
        /*********************************************************************/
        /* Check if Already transmitted WSC-Done, if so then assume protocol */
        /* is successful and complete. This is to fix the issue with Buffalo */
        /* AP. After WSC-DONE, the AP goes off for a few second and link loss*/
        /* happens                                                           */
        /*********************************************************************/
        if((MSG_ST ==wps_enrollee->reg_state) &&
            (DONE_ST == wps_enrollee->msg_st))
        {
            wps_stop_enrollee(wps_enrollee);
            end_wps_enrollee(REG_PROT_SUCC_COMP);
            return SUCCESS;
        }

        wps_add_discarded_reg_ap(wps_enrollee,
            wps_enrollee->sel_ap_info.bssid);
    }
    wps_delete_timers(wps_enrollee);
    wps_loc_start_scan(wps_enrollee);
    return SUCCESS;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_process_scan_rsp                                  */
/*                                                                           */
/*  Description      : This function processes the probe response/ beacon    */
/*                     message received                                      */
/*                                                                           */
/*  Inputs           : 1) Pointer to WPS memory                              */
/*                                                                           */
/*  Globals          : None                                                  */
/*  Processing       : This function parses the probe response/beacon frame  */
/*                     for the fields required for WPS protocol. In PIN mode */
/*                     if Registrar is ready for desired WPS protocol and it */
/*                     is not present in discarded list then start Join      */
/*                     procedure with this registrar/AP. In AP mode, if the  */
/*                     AP/Registrar in PBC mode and this is the first AP/    */
/*                     registrar in PBC mode then add it to selected list, if*/
/*                     the AP/Registrar not the first then indicate Session  */
/*                     overlap.                                              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : Action that MAC needs to take(Keep frame, Skip frame  */
/*                     or join current network)                              */
/*  Issues           : None                                                  */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         03 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
WPS_ACTION_T wps_process_scan_rsp(wps_enrollee_t* wps_enrollee,
                                  UWORD8* msg_ptr, UWORD16 msg_len)
{
    wps_rx_attri_t parse_attr[MAX_REG_PER_AP];
    wps_rx_attri_t *parse_attr_ptr[MAX_REG_PER_AP];
    UWORD16 wsc_ie_len     = 0;
    UWORD16 ie_start_index = 0;
    TYPESUBTYPE_T frm_type;
    BSSTYPE_T bss_type;
    UWORD8    bssid[6];
    UWORD16   dev_pass_id;
    UWORD8    wsc_ie_cnt, reg_num, count, wsc_ie_idx;
    UWORD8 *wsc_ie_buf = NULL;
    UWORD8 *buffer = NULL;

    /*************************************************************************/
    /* If WPS Enrollee is not in Scan state then MAC should not call this API*/
    /* so check if not in Scan state then stop enrollee and indicate system  */
    /* error                                                                 */
    /*************************************************************************/
    if(IN_SCAN != wps_enrollee->state)
    {
        wps_handle_sys_err(wps_enrollee);
        return (WPS_INVALID);
    }

    for(wsc_ie_cnt =0; wsc_ie_cnt < MAX_REG_PER_AP; wsc_ie_cnt++)
    {
        parse_attr_ptr[wsc_ie_cnt] = &parse_attr[wsc_ie_cnt];
    }

    /*************************************************************************/
    /* Check if the WPS Information/Parameter element is present in the frame*/
    /* If WSC IE is absent then return SKIP Frame                            */
    /*************************************************************************/
    frm_type   = get_sub_type(msg_ptr);

    /* Process only Probe Response and Beacon frame */
    if((BEACON != frm_type) && (PROBE_RSP != frm_type))
    {
       return (SKIP_FRAME);
    }

    /*************************************************************************/
    /* Parse the frame to get BSSID, SSID, BSS_TYPE.                         */
    /*************************************************************************/
    bss_type = get_bss_type(get_cap_info(msg_ptr));
    get_BSSID(msg_ptr, bssid);
    if((DISABLED == WPS_IBSS_MODE) & (INDEPENDENT == bss_type))
    {
        return (SKIP_FRAME);
    }

    /* Get the start of the IE */
    /* MAC Header(24),  Timestamp (8), Beacon Interval (2) and Capability (2)*/
    ie_start_index = (WPS_MAC_HDR_LEN + TIMESTAMP_LEN + BCN_INT_LEN +
                       CAP_INFO_LEN);

    /*************************************************************************/
    /* Check if Reassembly of WSC-IE is required, If so then re-assemble the */
    /* WSC-IE                                                                */
    /*************************************************************************/
    if(BTRUE == is_wsc_ie_reassm_req_enr(msg_ptr, msg_len, ie_start_index,
                                         &wsc_ie_len, &wsc_ie_buf))
    {
        buffer = wps_frame_mem_alloc(wsc_ie_len);
        reassemble_wsc_ie_enr(msg_ptr, msg_len, ie_start_index, buffer);
        wsc_ie_buf = buffer;
    }

    if((0 == wsc_ie_len) || (NULL == wsc_ie_buf))
    {
        return (SKIP_FRAME);
    }

    reg_num = 0;
    wsc_ie_cnt = wps_parse_attri(wsc_ie_buf, parse_attr_ptr,
                                 wsc_ie_len, MAX_REG_PER_AP, BTRUE);

    for(wsc_ie_idx =0; wsc_ie_idx < wsc_ie_cnt; wsc_ie_idx++)
    {
        /*********************************************************************/
        /* Check if (Version number is supported) and                        */
        /* ('Selected Registrar' is present and selected) and                */
        /* ('Response Type' is present and is of type AP or Registrar) and   */
        /* ('Device Password ID' is present) and                             */
        /* ('Selected Registrar Config Methods' is present) if not discard   */
        /* current element.                                                  */
        /* Some APs does not show selected registrar field unless WPS        */
        /* protocol is started. So if Ignore Seletced Registrar field is set */
        /* check for only Version number; if registrar is selected the       */
        /* other checks, else start WPS protocol with the AP                 */
        /*********************************************************************/
        if((NULL != parse_attr[wsc_ie_idx].version2) &&
           (BFALSE == wps_check_ver2_supp(parse_attr[wsc_ie_idx].version2)))
        {
            continue;
        }
#if 0 // wxb modify
        if(!wps_check_sel_reg(parse_attr[wsc_ie_idx].sel_reg))
#else
	 if(wps_check_sel_reg(parse_attr[wsc_ie_idx].sel_reg))
#endif
        {
#ifdef WPS_EXT_REG_SEL_REG_BUG_FIX
            if(BFALSE == wps_enrollee->ignore_sel_reg_field)
            {
                continue;
            }
            else
            {
                memcpy(wps_enrollee->sel_ap_info.bssid, bssid,
                    WPS_MAC_ADDR_LEN);
                wps_enrollee->sel_ap_info.sel_reg = BFALSE;
                reg_num++;
                wps_enrollee->sel_ap_info.reg_num = reg_num;
                wps_enrollee->req_type = ENROLLEE_OPEN_802_1X;
                wps_enrollee->state = IN_JOIN;

                if(NULL != buffer)
                    wps_frame_mem_free(buffer);

                return (JOIN_NW);
            }
#else /* WPS_EXT_REG_SEL_REG_BUG_FIX */
            continue;
#endif /* WPS_EXT_REG_SEL_REG_BUG_FIX */
        }

        if((!parse_attr[wsc_ie_idx].dev_pass_id) ||
            (!parse_attr[wsc_ie_idx].sel_reg_config_meth))
        {
            continue;
        }
        if(PROBE_RSP == frm_type)
        {
            if(!wps_check_res_type(parse_attr[wsc_ie_idx].res_type))
            {
                continue;
            }
        }

        /**********************************************************************/
        /* If the Registrar is Version2 Capable, then check for STA`s MAC     */
        /* address in AuthorizedMACs subelement                               */
        /**********************************************************************/
        if((NULL != parse_attr[wsc_ie_idx].version2) &&
           (BTRUE == wps_check_ver2_supp(parse_attr[wsc_ie_idx].version2)))
        {
            if(!wps_check_auth_macs_list(parse_attr[wsc_ie_idx].auth_macs,
                                         parse_attr[wsc_ie_idx].auth_macs_cnt,
                                         wps_enrollee->config_ptr->mac_address))
                continue;
        }

        dev_pass_id = GET_U16_BE((parse_attr[wsc_ie_idx].dev_pass_id));
        if(PBC == wps_enrollee->config_ptr->prot_type)
        {
            /*****************************************************************/
            /* In PBC mode, check if the Registrar is in PBC mode. If not    */
            /* discard the current WSC IE. If registrar is in PBC mode, then */
            /* check if there is already a distinct registrar in PBC mode, if*/
            /* so then indicate Session Overlap error                        */
            /*****************************************************************/
            if(DEV_PASS_ID_PUSHBUTT != dev_pass_id)
            {
                continue;
            }
            /*****************************************************************/
            /* Check whether there is already a registrar in PBC mode        */
            /*****************************************************************/
            if(wps_enrollee->sel_ap_info.reg_num)
            {
                /*************************************************************/
                /* Check if the selected registrar is same as the Registrar  */
                /* in the current WSC IE, if not the indicate Session overlap*/
                /* error. First check the BSSID, if BSSID is same then check */
                /* for UUID                                                  */
                /*************************************************************/
                if(0 != memcmp((const void *)wps_enrollee->sel_ap_info.bssid,
                    (const void *) bssid, WPS_MAC_ADDR_LEN))
                {
                    wps_stop_enrollee(wps_enrollee);
                    end_wps_enrollee(SESSION_OVERLAP_DETECTED);
                    if(NULL != buffer)
                        wps_mem_free(buffer);

                    return (SKIP_FRAME);
                } /* if(0 != memcmp((const void *)wps_enrollee-> .... */
                continue;
            } /* if(wps_enrollee->sel_ap_info.reg_num) */
        } /* if(PBC == wps_enrollee->config_ptr->prot_type) */
        else
        {
            BOOL_T match_found = BFALSE;

            if(dev_pass_id != wps_enrollee->config_ptr->dev_pass_id)
            {
                continue;
            }
            /*****************************************************************/
            /* In PIN mode, Search the discarded Registrar/AP list If match  */
            /* found then discard the current WSC IE                         */
            /*****************************************************************/
            for(count =0; count < wps_enrollee->discarded_reg_cnt; count++)
            {
                if(0== memcmp(wps_enrollee->dis_ap_reg_list[count].bssid,
                    bssid, WPS_MAC_ADDR_LEN))
                {
                    match_found = BTRUE;
                    break;
                } /* if(0== memcmp(wps_enrollee->dis_ap_reg_list.... */
            } /* for(count =0; count < wps_enrollee->discarded_reg_cnt;... */
            /*****************************************************************/
            /* If match found then discard current WSC IE                    */
            /*****************************************************************/
            if(BTRUE == match_found)
            {
                continue;
            }
        } /* else of if(PBC == wps_enrollee->config_ptr->prot_type) */

        /*********************************************************************/
        /* No Match found. So store the AP and registrar information         */
        /*********************************************************************/
        memcpy(wps_enrollee->sel_ap_info.bssid, bssid, WPS_MAC_ADDR_LEN);
        wps_enrollee->sel_ap_info.sel_reg = BTRUE;
        reg_num++;
        wps_enrollee->sel_ap_info.reg_num = reg_num;
    } /* for(wsc_ie_idx =0; wsc_ie_idx < wsc_ie_cnt; wsc_ie_idx++) */

    /*************************************************************************/
    /* Check if any valid registrar was found. If Registrar found then in PBC*/
    /* mode continue the scan(to detect Session overlap). In Pin mode, give  */
    /* join request to MAC                                                   */
    /*************************************************************************/
    if(0 < reg_num)
    {
        if(PBC == wps_enrollee->config_ptr->prot_type)
        {
            if(NULL != buffer)
                wps_mem_free(buffer);

            return (KEEP_FRAME);
        }
        else
        {
            wps_enrollee->req_type = ENROLLEE_OPEN_802_1X;
            wps_enrollee->state = IN_JOIN;
            if(NULL != buffer)
                wps_mem_free(buffer);

            return (JOIN_NW);
        }
    }
    if(NULL != buffer)
        wps_mem_free(buffer);
    return (SKIP_FRAME);
}
#endif /* INT_WPS_ENR_SUPP */
