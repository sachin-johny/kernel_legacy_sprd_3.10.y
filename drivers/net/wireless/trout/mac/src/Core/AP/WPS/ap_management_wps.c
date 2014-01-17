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
/*  File Name         : ap_management_wps.c                                  */
/*                                                                           */
/*  Description       : This file contains the all the functions that        */
/*                      implement the AP functionality of WPS internal       */
/*                      Registrar                                            */
/*                                                                           */
/*  List of Functions : set_wps_wsc_info_element_ap                          */
/*                      wps_handle_probe_req_ap                              */
/*                      wps_handle_eap_rx_ap                                 */
/*                      check_wps_capabilities_ap                            */
/*                      wps_update_beacon_wsc_ie_ap                          */
/*                      wps_update_failed_enr_list                           */
/*                      wps_add_enr_blacklist                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
#ifdef INT_WPS_SUPP

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_reg_if.h"
#include "wps_ap.h"
#include "management.h"
#include "management_11i.h"
#include "ap_management_wps.h"
#include "mh.h"

/*****************************************************************************/
/* Global Variable Definitions                                               */
/*****************************************************************************/

UWORD8 g_fail_enr_cnt = 0;
wps_fail_enr_t g_fail_enr_list[MAX_FAIL_ENR_LIST] = {{0}};
wps_enr_blacklist_t g_enr_blacklist = {0};

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void wps_add_enr_blacklist(UWORD8 *mac_addr);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : set_wps_wsc_info_element_ap                           */
/*                                                                           */
/*  Description      : This function handles the adding of WSC-IE in Beacon  */
/*                     and Probe response frames                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to the transmit packet                     */
/*                     2) Index in the buffer where WSC-IE is to inserted    */
/*                     3) Frame Type                                         */
/*                                                                           */
/*  Globals          : g_wps_ctxt_ptr                                        */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If WPS is disabled then returns 0                  */
/*                     2) Call wps_insert_wsc_ie of the WPS Library to       */
/*                        insert the WSC-IE frame                            */
/*                                                                           */
/*  Outputs          : UWORD16                                               */
/*                                                                           */
/*  Returns          : Length of the WSC_IE inserted                         */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 set_wps_wsc_info_element_ap(UWORD8 *data, UWORD16 index,
                                    TYPESUBTYPE_T frame_type)
{
	asoc_entry_t *ae = NULL;

    if(BFALSE == get_wps_reg_enabled())
        return 0;

    ae = (asoc_entry_t*)find_entry(get_DA_ptr(data));

    /* Add WSC-IE to (re)association response only if STA is WPS capable     */
    if((ASSOC_RSP == frame_type) || (REASSOC_RSP == frame_type))
    {
		if((NULL == ae) || (ae->wps_cap_sta == BFALSE))
			return 0;
	}

    /* Call the appropriate function for inserting of WSC-IE in the frame    */
    if (NULL != g_wps_ctxt_ptr) //wxb add "if-else"
    {
       return wps_insert_wsc_ie(g_wps_ctxt_ptr, (data+index), (UWORD8)frame_type);
    }
    else
    {
	return 0;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_probe_req_ap                               */
/*                                                                           */
/*  Description      : This function handles the reception of Probe Request  */
/*                     to check for any WPS PBC Enrollees                    */
/*                                                                           */
/*  Inputs           : 1) Pointer to the transmit packet                     */
/*                     2) Index in the buffer where WSC-IE is to inserted    */
/*                     3) Frame Type                                         */
/*                                                                           */
/*  Globals          : g_wps_ctxt_ptr                                        */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If WPS is disabled then returns                    */
/*                     2) Call wps_process_probe_req of  the WPS Library for */
/*                        further processing                                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_handle_probe_req_ap(UWORD8 *msa, UWORD8 *sa, UWORD16 rx_len,
                             TYPESUBTYPE_T frm_type)
{
    if(BFALSE == get_wps_reg_enabled())
        return;

    /* Call appropriate library function to process the probe request further*/
    wps_process_probe_req(g_wps_ctxt_ptr, msa, sa, rx_len, frm_type);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_handle_eap_rx_ap                                  */
/*                                                                           */
/*  Description      : This function handles reception of reception of EAP   */
/*                     packets for WPS enabled enrollee                      */
/*                                                                           */
/*  Inputs           : 1) Association Entry of the STA                       */
/*                     2) Pointer to received EAP Packet                     */
/*                     3) Length of the received frame                       */
/*                                                                           */
/*  Globals          : g_wps_ctxt_ptr                                        */
/*                     g_config_ptr                                          */
/*                                                                           */
/*  Processing       : This function does the following processing:          */
/*                     1) If WPS is not enabled then return FAILURE          */
/*                     2) If the STA is not WPS Capable then return          */
/*                     3) If WPS Registration Protocol is not in Progress    */
/*                        then copy the MAC address of the STA in the        */
/*                        configuration structure passed to registrar        */
/*                     4) Else if the MAC address in the configuration       */
/*                        structure is different from the STA`s MAC address  */
/*                        then return FAILURE                                */
/*                     5) Call the function wps_handle_eap_rx of WPS library */
/*                        to process the received EAP frame                  */
/*                                                                           */
/*  Outputs          : Status of Processing of the received frame            */
/*                                                                           */
/*  Returns          : SUCCESS - If processed successfully                   */
/*                     FAILURE - Otherwise                                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

STATUS_T wps_handle_eap_rx_ap(asoc_entry_t *ae, UWORD8 *msa, UWORD16 rx_len)
{
	STATUS_T status = SUCCESS;
	UWORD8 *sta_addr = get_mac_addr_from_sta_id(ae->sta_index);

    /* If WPS Registrar is not enabled then return */
    if(BFALSE == get_wps_reg_enabled())
        return FAILURE;

    /* If the STA is not WPS capable then do not process further */
    if(BFALSE == ae->wps_cap_sta)
        return FAILURE;

    /* If the STA is WPS capable and Registration Protocol is not in prgress */
    /* then copy the MAC address of the STA in the registrar`s configuration */
    /* structure                                                             */
    if(BFALSE == is_wps_in_prog_reg(g_wps_ctxt_ptr))
    {
        mac_addr_cpy(g_wps_config_ptr->mac_addr, sta_addr);

        /* Set Version2 capable flag */
        wps_set_ver2_cap_sta(g_wps_ctxt_ptr, ae->ver2_cap_sta);
    }
    /* If Registration Protocol is not in progress then processes further    */
    /* only the packets from the same Enrollee                               */
    else if(BFALSE == mac_addr_cmp(g_wps_config_ptr->mac_addr, sta_addr))
    {
        /* Send the Status to Host */
        //send_wps_status(WPS_REG_MULTIPLE_REG_ATTEMPT, NULL, 0);
        return FAILURE;
    }

    /* Mask Error Interrupts during WPS Packet Processing */
    mask_machw_error_int();

    /* Call the WPS Library function fo further processing */
    status = wps_handle_eap_rx(g_wps_ctxt_ptr, msa, rx_len);

    /* Unmask Error Interrupts during WPS Packet Processing */
    unmask_machw_error_int();

    return status;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : check_wps_capabilities_ap                             */
/*                                                                           */
/*  Description      : This function handles WPS related checks on reception */
/*                     of association request from the Enrollee              */
/*                                                                           */
/*  Inputs           : 1) Association Entry of the STA                       */
/*                     2) Pointer to received ASOC-REQ                       */
/*                     3) Length of the received frame                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functino does following processing:              */
/*                     1) If the STA includes version 2.0 WSC-IE then, return*/
/*                        WPS_CHECK_SUCCESS                                  */
/*                     2) Check for RSN/SSN IE in the Association Request:   */
/*                        - If present then indicates STA is not WPS capable */
/*                          return WPS_CHECK_FURTHER                         */
/*                        - If not present then it indicates STA is WPS      */
/*                          capable. Do the following :                      */
/*                          - If the Registrar is in Registration Protocol   */
/*                            then return WPS_CHECK_FAILURE                  */
/*                          - If the STA has been Black listed the return    */
/*                            WPS_CHECK_FAILURE                              */
/*                          - Else set the WPS capable flag in assoc entry   */
/*                            and return WPS_CHECK_SUCCESS                   */
/*                                                                           */
/*  Outputs          : Status of WPS Capablility check                       */
/*                                                                           */
/*  Returns          : WPS_CHECK_SUCCESS - WPS Capable, Allow Association    */
/*                     WPS_CHECK_FAILURE - WPS Capable, Do not allow         */
/*                                         association                       */
/*                     WPS_CHECK_FURTHER - Not WPS Capable, Do further       */
/*                                         Security checks                   */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

WPS_ASSOC_CHECK_T check_wps_capabilities_ap(asoc_entry_t *ae, UWORD8 *msa,
                                            UWORD16 rx_len)
{
    WPS_ASSOC_CHECK_T retval = WPS_CHECK_FURTHER;
    UWORD16 index      = MAC_HDR_LEN + 4;
    BOOL_T rsn_found   = BFALSE;
    TYPESUBTYPE_T sub_type = get_sub_type(msa);
    UWORD8 sta_addr[MAC_ADDRESS_LEN] = {0};

    /* If WPS Registrar is disabled then return CHECK_FURTHER */
    if(BFALSE == get_wps_reg_enabled())
        return retval;

    /* Process only if the frame is a (Re)Association Request */
    if((ASSOC_REQ != sub_type) && (REASSOC_REQ != sub_type))
        return retval;

    /* Get the source address */
    get_SA(msa, sta_addr);

    /* If the Enrollee is not black listed then check if it has included     */
    /* WSC-IE. Update the WPS capable flags and return WPS_CHECK_SUCCESS     */
    if((BFALSE == is_enr_blacklisted(sta_addr)) &&
       (BTRUE == check_wps_cap_assoc_req(g_wps_ctxt_ptr, msa, rx_len, sub_type,
                                         &ae->wps_cap_sta, &ae->ver2_cap_sta)))
    {
        return WPS_CHECK_SUCCESS;
    }

    /* For RE-ASSOC-REQ increment the index for parsing by 6 bytes */
    if(REASSOC_REQ == sub_type)
        index += MAC_ADDRESS_LEN;

    /* Set the STA to WPS incapable by default and WPS 2.0 capable to zero */
    ae->wps_cap_sta  = BFALSE;
    ae->ver2_cap_sta = BFALSE;

    while(index < (rx_len - FCS_LEN))
    {
        /* Check if RSNA or WPA IE are present */
        if((msa[index] == IRSNELEMENT) || (is_wpa_ie(msa + index) == BTRUE))
        {
            rsn_found = BTRUE;
            break;
        }

        /* Increment the index by length information & tag header */
        index += msa[index + 1] + 2;
    }

    /* If the RSN/SSN IE is present  then the STA is not WPS capable then    */
    /* return CHECK_FURTHER                                                  */
    if(BTRUE == rsn_found)
    {
        return retval;
    }

    /* If RSN/SSN IE is not presnt then it indicates that STA can be WPS     */
    /* capable, allow association only if Registration Protocol is not in    */
    /* progress or if the STA is not black listed else allow the STA to      */
    /* associate successfully without any security checks                    */
    if((BTRUE == is_enr_blacklisted(sta_addr)) ||
       (BTRUE == is_wps_in_prog_reg(g_wps_ctxt_ptr)))
        retval = WPS_CHECK_FAILURE;
    else
    {
        retval = WPS_CHECK_SUCCESS;
        ae->wps_cap_sta = BTRUE;
    }

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_update_beacon_wsc_ie_ap                           */
/*                                                                           */
/*  Description      : This function handle updating of WSC-IE in beacon     */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function updates call the function which inturn  */
/*                     updates the latest WSC-IE in the beacon               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_update_beacon_wsc_ie_ap(void)
{
    update_beacon_ap();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_update_failed_enr_list                            */
/*                                                                           */
/*  Description      : This function updates the enrollee in the failed      */
/*                     enrollee list                                         */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function checks if the enrollee is already       */
/*                     present in the fail list. If present the updates the  */
/*                     fail count. Everytime the fail count is updated, it is*/
/*                     checked against WPS_BLACKLIST_LIMIT threshold. If     */
/*                     greater then the entry is black listed. If a new entry*/
/*                     then add it to the end of Failed Enrollee list and    */
/*                     retrun                                                */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void wps_update_failed_enr_list(UWORD8 *mac_addr)
{
    UWORD8 i =0;
    BOOL_T enr_found = BFALSE;

    /* Search if the Enrollee is already in the list */
    for(i = 0; i < g_fail_enr_cnt; i++)
    {
        if(BTRUE == mac_addr_cmp(mac_addr, g_fail_enr_list[i].mac_addr))
        {
            enr_found = BTRUE;
            g_fail_enr_list[i].fail_cnt++;

            /* If the Enrollee has failed more than the Blacklist threshold  */
            /* then balcklist the Enrollee and donot allow it to associate   */
            if(WPS_BLACKLIST_LIMIT <= g_fail_enr_list[i].fail_cnt)
            {
                wps_add_enr_blacklist(g_fail_enr_list[i].mac_addr);

                sort_fail_enr_list(i, g_fail_enr_cnt);
                g_fail_enr_cnt--;
            }

            break;
        }
    }

    /* If the Enrollee is not in the failed list then add it */
    if((BFALSE == enr_found) && (MAX_FAIL_ENR_LIST >= g_fail_enr_cnt))
    {
        mac_addr_cpy(g_fail_enr_list[i].mac_addr, mac_addr);
        g_fail_enr_list[i].fail_cnt = 1;
        g_fail_enr_cnt++;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : wps_add_enr_blacklist                                 */
/*                                                                           */
/*  Description      : This function adds the given Enrollee at appropriate  */
/*                     index in the Black List                               */
/*                                                                           */
/*  Inputs           : None                                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function adds the given Enrollee at appropriate  */
/*                     index in the Black List                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

static void wps_add_enr_blacklist(UWORD8 *mac_addr)
{

    /* If already in the list then do nothing */
    if(BTRUE == is_enr_blacklisted(mac_addr))
        return;

    /* Add the Enrollee at appropriate index in the list */
    if(MAX_ENR_BACKLIST >= g_enr_blacklist.blacklist_cnt)
    {
        g_enr_blacklist.blacklist_cnt++;
        mac_addr_cpy(g_enr_blacklist.mac_addr[g_enr_blacklist.blacklist_cnt - 1],
                     mac_addr);
    }
}
#endif /* INT_WPS_SUPP */
#endif /* BSS_ACCESS_POINT_MODE */
