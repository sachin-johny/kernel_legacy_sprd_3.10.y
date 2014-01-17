/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
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
/*  File Name         : management.c                                         */
/*                                                                           */
/*  Description       : This file contains the MAC management related        */
/*                      functions.                                           */
/*                                                                           */
/*  List of Functions : get_bss_type                                         */
/*                      send_probe_rsp                                       */
/*                      probe_req_ssid_cmp                                   */
/*                      mgmt_timeout_alarm_fn                                */
/*                      check_bss_capability_info                            */
/*                      buffer_tx_packet                                     */
/*                      send_deauth_frame                                    */
/*                      update_beacon_erp_info                               */
/*                      is_rate_supp                                         */
/*                      get_mac_addr_from_sta_id                             */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"
#include "management.h"
#include "transmit.h"
#include "prot_if.h"
#include "host_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD16 g_join_start_timeout = JOIN_START_TIMEOUT;

UWORD8  g_bmap[8]               = {1, 2, 4, 8, 16, 32, 64, 128};

/* Transmit queue interface table containing the queue pointer address and   */
/* lock bit value for each queue.                                            */
qif_struct_t g_qif_table[NUM_TX_QUEUE] = {{0,},};

/* Hugh: This function checks if the queue pointers for all queue number are NULL   */
BOOL_T is_all_machw_q_null(void)
{
    UWORD32 i;
    UWORD32 q_hdr_val = 0;

	TROUT_FUNC_ENTER;
    for(i=0; i<NUM_TX_QUEUE; i++)
    {
        while(host_read_trout_reg((UWORD32)rMAC_PA_STAT) & g_qif_table[i].lock_bit)
        {
            q_hdr_val = q_hdr_val;
        }
        
        q_hdr_val = host_read_trout_reg((UWORD32)(g_qif_table[i].addr));
        if(q_hdr_val != 0)
        {
			TROUT_FUNC_EXIT;
            return BFALSE;
        }
    }

    TROUT_FUNC_EXIT;
    return BTRUE;
}

//chenq add 2012-12-29 for fast check machw q null
/* chenq: This function checks if the queue pointers for indecate queue number are NULL   */
BOOL_T is_indecate_machw_q_null(int q_num)
{
    UWORD32 i;
    UWORD32 q_hdr_val = 0;

	TROUT_FUNC_ENTER;

	if( (q_num >=0) && (q_num < NUM_TX_QUEUE) )
	{
		//chenq mod
	    //q_hdr_val = *((volatile UWORD32 *)(g_qif_table[q_num].addr)); 
	    q_hdr_val = host_read_trout_reg( (UWORD32)(g_qif_table[q_num].addr));
	    
	    if(q_hdr_val == 0)
	        return BTRUE;
	    else
		return BFALSE;
	}
	else
	{
	    for(i=0; i<NUM_TX_QUEUE; i++)
	    {   
	        q_hdr_val = host_read_trout_reg((UWORD32)(g_qif_table[i].addr));
	        if(q_hdr_val != 0)
	            return BFALSE;
	    }
	}
    TROUT_FUNC_EXIT;
    return BTRUE;
}

#ifdef TROUT_TRACE_DBG
BOOL_T is_all_machw_q_null_test(void)
{
    UWORD32 i;
    UWORD32 q_hdr_val = 0;
	
    for(i=0; i<NUM_TX_QUEUE; i++)
    {
        while(host_read_trout_reg((UWORD32)rMAC_PA_STAT) & g_qif_table[i].lock_bit)
        {
            q_hdr_val = q_hdr_val;
        }
        
        q_hdr_val = host_read_trout_reg((UWORD32)(g_qif_table[i].addr));
	    printk("txq[%d] addr value: 0x%X\n", i, q_hdr_val);
    }
    
    return BTRUE;
}
#endif	/* TROUT_TRACE_DBG */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_bss_type                                             */
/*                                                                           */
/*  Description   : This function determines the BSS type based on the       */
/*                  capability information value.                            */
/*                                                                           */
/*  Inputs        : 1) Capability Information.                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The BSS Type is determined based on the ESS bit in the   */
/*                  given capability information field.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BSS Type                                                 */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BSSTYPE_T get_bss_type(UWORD16 cap_info)
{
    if((cap_info & ESS) != 0)
        return INFRASTRUCTURE;

    if((cap_info & IBSS) != 0)
        return INDEPENDENT;

    return ANY_BSS;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_probe_rsp                                           */
/*                                                                           */
/*  Description   : This function sends probe response frame for the         */
/*                  received probe request.                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to Probe request frame                        */
/*                  2) If probe request has P2P IE.                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the probe response for the        */
/*                  received probe request (based on mode of operation) and  */
/*                  adds it to the hardware queue for transmission.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void send_probe_rsp(UWORD8* msa, BOOL_T is_p2p)
{
    UWORD8  *probe_rsp    = 0;
    UWORD16 probe_rsp_len = 0;

	TROUT_FUNC_ENTER;
    probe_rsp = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, SHARED_PKT_MEM_BUFFER_SIZE);
    if(probe_rsp == NULL)
    {
		TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif		
        return;
    }

    probe_rsp_len = prepare_probe_rsp(probe_rsp, msa, is_p2p);

    /* Transmit the management frame */
#ifdef ENABLE_MACHW_KLUDGE
    tx_mgmt_frame(probe_rsp, probe_rsp_len, NORMAL_PRI_Q, 0);
#else  /* ENABLE_MACHW_KLUDGE */
    tx_mgmt_frame(probe_rsp, probe_rsp_len, HIGH_PRI_Q, 0);
#endif /* ENABLE_MACHW_KLUDGE */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : probe_req_ssid_cmp                                    */
/*                                                                           */
/*  Description      : This function extracts the SSID from the probe request*/
/*                     frame and compares it with the station desired SSID.  */
/*                                                                           */
/*  Inputs           : 1) Probe Request Frame                                */
/*                     2) SSID                                               */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The SSID in the probe request frame is compared with  */
/*                     the station's SSID byte by byte as the SSID in the    */
/*                     probe request frame SSID is not NULL terminated. Also */
/*                     a probe request with broadcast SSID will be accepted. */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T. Result of the comparison.                     */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T probe_req_ssid_cmp(UWORD8* probe_req, WORD8* ssid)
{
    BOOL_T flag        = BTRUE;
    UWORD8 pr_ssid_len = 0;
    UWORD8 ssid_len    = 0;
    UWORD8 i           = 0;

	TROUT_FUNC_ENTER;
    ssid_len = strlen((WORD8*)ssid);

    /* SSID length of the probe request frame is at an offset of one byte    */
    /* from the MAC header.                                                  */
    pr_ssid_len = probe_req[MAC_HDR_LEN + 1];

    /* Begin comparison only if the lengths match */
    if(pr_ssid_len == ssid_len)
    {
        /* Compare the SSID in the probe request frame. It is at an offset   */
        /* of two bytes from the MAC header. The first two bytes indicate    */
        /* the SSID element name and SSID length.                            */
        for(i = 0; i < pr_ssid_len; i++)
        {
            if(probe_req[MAC_HDR_LEN + 2 + i] != ssid[i])
            {
                flag = BFALSE;
                break;
            }
        }
    }
    else if(pr_ssid_len == 0)
    {
        /* If the SSID length is zero, it denotes a broadcast SSID. Probe    */
        /* requests having broadcast SSID should be accepted and a probe     */
        /* response must be sent. Hence in this case the comparison flag is  */
        /* set to BTRUE.                                                     */
        flag = BTRUE;
    }
    else
    {
        /* Probe request SSID length itself did not match with the station's */
        /* desired SSID length. Also it is not a broadcast SSID.             */
        flag = BFALSE;
    }

	TROUT_FUNC_EXIT;
    return flag;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : mgmt_timeout_alarm_fn                                 */
/*                                                                           */
/*  Description      : This is the completion function called after any      */
/*                     management timout timer expires.                      */
/*                                                                           */
/*  Inputs           : 1) Pointer to the management timeout timer            */
/*                     2) Data for the alarm function (unused)               */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function creates and posts a miscellaneous       */
/*                     timeout event in the event queue.                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void mgmt_timeout_alarm_fn(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void mgmt_timeout_alarm_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    misc_event_msg_t *misc = 0;

	TROUT_FUNC_ENTER;
    /* Create a MISC_TIMEOUT event with no message and post it to the event  */
    /* queue.                                                                */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
    if(misc == NULL)
    {
        /* Exception */
        raise_system_error(NO_EVENT_MEM);
        TROUT_FUNC_EXIT;
        return;
    }

    misc->data = 0;
    misc->info = data; /* Set the info as the callback function input data */
    misc->name = MISC_TIMEOUT;

    post_event((UWORD8*)misc, MISC_EVENT_QID);
    TROUT_FUNC_EXIT;
}


#ifdef IBSS_BSS_STATION_MODE //chenq add for combo scan 2013-03-13
#ifdef COMBO_SCAN
enum hrtimer_restart mgmt_timeout_alarm_fn_combo_scan(struct hrtimer *timer)
{
    misc_event_msg_t *misc = 0;

	TROUT_FUNC_ENTER;
    /* Create a MISC_TIMEOUT event with no message and post it to the event  */
    /* queue.                                                                */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
    if(misc == NULL)
    {
        /* Exception */
        raise_system_error(NO_EVENT_MEM);
        TROUT_FUNC_EXIT;
        return HRTIMER_NORESTART;
    }

    misc->data = 0;
    misc->info = WAIT_SCAN; /* Set the info as the callback function input data */
    misc->name = MISC_TIMEOUT;

    post_event((UWORD8*)misc, MISC_EVENT_QID);
    TROUT_FUNC_EXIT;
	
	return HRTIMER_NORESTART;
}
#endif
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_bss_capability_info                                */
/*                                                                           */
/*  Description   : This function compares the capabiltiy of the STA and the */
/*                  BSS                                                      */
/*                                                                           */
/*  Inputs        : 1) Capability information field                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if all capabilities specific to MAC */
/*                  (like BSS Type, Privacy) match those of the STA.         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE, if all capabilities match                         */
/*                  BFALSE, otherwise.                                       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T check_bss_capability_info(UWORD16 cap_info)
{
    BOOL_T ret_val = BTRUE;

    /* BSS Type */
    if(mget_DesiredBSSType() != get_bss_type(cap_info))
    {
        ret_val = BFALSE;
    }
    else
    {
        ret_val = check_bss_mac_privacy(cap_info);

        if(ret_val == BTRUE)
        {
            ret_val = check_misc_cap_info(cap_info);
        }
    }

    return ret_val;
}



/*****************************************************************************/
/*                                                                           */
/*  Function Name : buffer_tx_packet                                         */
/*                                                                           */
/*  Description   : This function buffers a packet ready for transmission    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit descriptor                    */
/*                  2) Number of the queue to which the packet belongs       */
/*                  3) Destination address                                   */
/*                                                                           */
/*  Globals       : g_mc_q                                                   */
/*                  g_num_mc_bc_pkt                                          */
/*                  g_num_sta_ps                                             */
/*                                                                           */
/*  Processing    : The packet is buffered based on the service class or the */
/*                  power save mode of the receiving STA                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE, if the given packet is buffered in any queue      */
/*                  BFALSE, if the packet is not buffered and can be sent    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T buffer_tx_packet(UWORD8 *entry, UWORD8 *da, UWORD8 priority,
                        UWORD8 q_num, UWORD8 *tx_dscr)
{
	TROUT_FUNC_ENTER;
    if(BTRUE == is_ps_buff_pkt(entry, da, tx_dscr))
    {
#ifdef DEBUG_MODE
        g_mac_stats.psbuff++;
#endif /* DEBUG_MODE */
    }
    else if(BTRUE == is_serv_cls_buff_pkt(entry, q_num, priority, tx_dscr))
    {
#ifdef DEBUG_MODE
        g_mac_stats.bapendingtx++;
#endif /* DEBUG_MODE */
    }
    else
    {
    	TROUT_FUNC_EXIT;
        return BFALSE;
    }

	TROUT_FUNC_EXIT;
    /* If the packet is buffered return BTRUE */
    return BTRUE;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_deauth_frame                                        */
/*                                                                           */
/*  Description   : This function classifies MSDU traffic for PCF.           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Ethernet type of the packet                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function classifies the incoming packet based on    */
/*                  port number.                                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Normal or CF Priority                                    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T send_deauth_frame(UWORD8 *da, UWORD16 error_code)
{
    UWORD16 deauth_len = 0;
    UWORD8  *deauth    = 0;

    TROUT_FUNC_ENTER;
    TROUT_DBG4("send deauth frame to %2x:%2x:%2x:%2x:%2x:%2x, error code: %d\n", da[0], da[1], da[2], da[3], da[4], da[5], error_code);
    /* Sending De-auth for Class2/Class3 Errors is less important    */
    /* So use general buff pool for sending this frame */
    if((error_code == CLASS2_ERR) || (error_code == CLASS3_ERR))
    {
        deauth = (UWORD8 *)pkt_mem_alloc(MEM_PRI_TX);
    }
    else
    {
        deauth = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    }
    if(deauth == NULL)
    {
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif
        if((error_code == CLASS2_ERR) || (error_code == CLASS3_ERR))
        {
            /* Sending De-auth for Class2/Class3 Errors is less important    */
            /* So use general buff pool for sending this frame */
            TROUT_FUNC_EXIT;
            return BFALSE;
        }
        else
        {
            /* Reserved Memory pool tried for high priority deauth frames */
            deauth = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                        MANAGEMENT_FRAME_LEN);
            if(deauth == NULL)
            {
				TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
				g_mac_stats.no_mem_count++;
#endif				
                return BFALSE;
            }
        }
    }

    deauth_len = prepare_deauth(deauth, da, error_code);

    /* MIB variables related to deauthentication are updated */
    mset_DeauthenticateReason(error_code);
    mset_DeauthenticateStation(da);

    /* Buffer this frame in the Memory Queue for transmission */
    tx_mgmt_frame(deauth, deauth_len, HIGH_PRI_Q, 0);
    
	TROUT_FUNC_EXIT;
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_beacon_erp_info                                   */
/*                                                                           */
/*  Description   : This function updates the ERP information field in the   */
/*                  beacon frame and sets the MAC H/w pointer.               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function searches for the ERP inromation field and  */
/*                  updates the same inthe beacon frame. It also updates the */
/*                  MAC H/w beacon pointer.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_beacon_erp_info(void)
{
    UWORD16 idx = TAG_PARAM_OFFSET;

    /* Traverse the beacon till ERP Info element   */
    while(idx < g_beacon_len)
    {
        if(g_beacon_frame[g_beacon_index][idx] == IERPINFO)
        {
            /* Once the position of the ERP in the beacon is found, set the  */
            /* value                                                         */
            set_erp_info_field(g_beacon_frame[g_beacon_index], idx);
            break;
        }
        else
        {
            idx += (IE_HDR_LEN + g_beacon_frame[g_beacon_index][idx + 1]);
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_serv_class                                           */
/*                                                                           */
/*  Description   : This function gets the service class for the required    */
/*                  destination address and TID.                             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The service class is determined from the user setting    */
/*                  and protocol considerations.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_serv_class(void *entry, UWORD8 *da, UWORD8 tid)
{
    UWORD8 service_class = BCAST_NO_ACK;

    if(is_group(da) == BTRUE)
        return BCAST_NO_ACK;
    else
        service_class = g_ack_policy;

    update_serv_class_prot(entry, tid, &service_class);

    return service_class;
}

#ifdef AUTORATE_FEATURE
/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_rate_supp                                             */
/*                                                                           */
/*  Description   : This function gets the service class for the required    */
/*                  destination address and TID.                             */
/*                                                                           */
/*  Inputs        : 1) Rate (in PHY rate format)                             */
/*                  2) Pointer to the entry                                  */
/*                                                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if a rate is supported by the given */
/*                  STA. If the rate is Non-HT, the operational rates table  */
/*                  is searched to determine if the rate is supported. Note  */
/*                  that the rates in the operational rates table is in MAC  */
/*                  rate format. Hence it needs to be converted to PHY rate  */
/*                  format before comparison can be done. If the rate is a   */
/*                  HT rate it is checked if the given MCS index is          */
/*                  supported by the STA.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8. 1, if the rate is supported by the STA           */
/*                          0, if the rate is not supported by the STA       */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 is_rate_supp(UWORD8 rate, void *entry)
{
    UWORD8 rate_is_supp = 0;

    if(IS_RATE_MCS(rate) == BFALSE)
    {
        UWORD8 i         = 0;

        rate_t *op_rates = get_op_rates_entry(entry);

        for(i = 0; i < op_rates->num_rates; i++)
        {
            if(get_phy_rate(get_user_rate(op_rates->rates[i])) == rate)
            {
                rate_is_supp = 1;
                break;
            }
        }
    }
    else
    {
        UWORD8 mcs_index = rate & 0x7F;

        if(is_mcs_supp(entry, mcs_index) == BTRUE)
            rate_is_supp = 1;
    }

    return rate_is_supp;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_rate_allowed                                          */
/*                                                                           */
/*  Description   : This function checks whether the specified rate is       */
/*                  allowed under the restrictions from the protocols and    */
/*                  features currently enabled.  .                           */
/*                                                                           */
/*  Inputs        : 1) Rate (in PHY rate format)                             */
/*                  2) Pointer to the entry                                  */
/*                                                                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks whether for the specified rate, the */
/*                  minimum TXOP fragmentable frame length fits in the       */
/*                  minimum non-zero TXOP limit.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8. 1, if the rate is allowed for the STA            */
/*                          0, if the rate is not allowed for the STA        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 is_rate_allowed(UWORD8 rate ,void *entry)
{
#if 0
    UWORD16 min_txop = 0;
    UWORD16 txop_frag_min_len = 0;
    UWORD8  allow = 1;

    min_txop = get_min_non_zero_txop(entry);

    if(min_txop != 0xFFFF)
    {
        txop_frag_min_len = get_txop_frag_min_len(entry);

        allow = does_frame_fit_in_txop(min_txop, txop_frag_min_len, rate);
    }

    return allow;
#else  /* 0 */
    return 1;
#endif /* 0 */
}

#endif /* AUTORATE_FEATURE */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_mac_addr_from_sta_id                                 */
/*                                                                           */
/*  Description   : This function gets the MAC address of the entry that     */
/*                  corresponds to the given STA index.                      */
/*                                                                           */
/*  Inputs        : 1) STA Index                                             */
/*                                                                           */
/*  Globals       : g_sta_table                                              */
/*                                                                           */
/*  Processing    : This function traverses the entire association/station   */
/*                  table and finds an entry that has a station index that   */
/*                  matches the given value. It then returns the key of this */
/*                  table element. The key of the table element is the MAC   */
/*                  address corresponding to the entry.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8 *,  MAC Address corresponding to the entry with   */
/*                             the given station index. NULL is returned if  */
/*                             no matching entry is found.                   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *get_mac_addr_from_sta_id(UWORD8 sta_id)
{
    UWORD8        i         = 0;
    void          *te       = 0;
    table_elmnt_t *tbl_elm  = 0;
    UWORD8        *mac_addr = NULL;

    /* Traverse entire association/station table and process all non-zero    */
    /* entries to find an entry with STA index matching the given value. If  */
    /* found return the key for this table element. The key is the MAC       */
    /* address.                                                              */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            te = tbl_elm->element;

            if(te == 0)
                break;

            if(sta_id == get_entry_sta_index(te))
            {
                mac_addr = tbl_elm->key;
                break;
            }

            tbl_elm = tbl_elm->next_hash_elmnt;
        } /* end of while loop */
    } /* end of for loop */

    return mac_addr;
}
