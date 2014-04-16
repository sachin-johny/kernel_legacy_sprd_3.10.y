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
/*  File Name         : host_rx_msdu_sta.c                                   */
/*                                                                           */
/*  Description       : This file contains the functions called by the STA   */
/*                      MAC FSM on receiving HOST_RX (MSDU) events.          */
/*                                                                           */
/*  List of Functions : sta_enabled_tx_data                                  */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate_sta.h"
#include "cglobals_sta.h"
#include "index_util.h"
#include "prot_if.h"
#include "sta_prot_if.h"
#include "host_if.h"
#include "qmu_if.h"
#include "pm_sta.h"
#include "amsdu_aggr.h"
#include "core_mode_if.h"
#include "iconfig.h"
#include "qmu_tx.h"

/*****************************************************************************/
/* Global variables                                                          */
/*****************************************************************************/

#ifdef ENABLE_OVERWRITE_SUBTYPE
UWORD8 g_overwrite_frame_type = 0;
#endif /* ENABLE_OVERWRITE_SUBTYPE */

#ifdef WAKE_LOW_POWER_POLICY
UWORD32 g_low_power_tx_data_pkt_cnt = 0;	
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_scan_tx_data                                    */
/*                                                                           */
/*  Description   : This function handles MAC data request from the host when*/
/*                  in scan mode.                                            */
/*                                                                           */
/*  Inputs        : 1) Data request structure                                */
/*                  2) Pointer to the buffer                                 */
/*                                                                           */
/*  Globals       : NONE                                                     */
/*                                                                           */
/*  Processing    : In scan state, if this scan was initiated from data mode */
/*                  then all the incoming packets are processed as if they   */
/*                  came in data(Enabled) mode, else they are dropped        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_scan_tx_data(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_tx_req_t *wlan_tx_req = (wlan_tx_req_t *)msg;

	TROUT_FUNC_ENTER;

	TX_PATH_DBG("%s: state=%d\n", __func__, get_mac_state());
    /* Scan was initiated from Enabled mode */
    if(g_keep_connection == BTRUE)
    {
        sta_enabled_tx_data(mac,msg);
    }
    else
    {
		//caisf add 2013-02-15
        if(g_mac_net_stats)
            g_mac_net_stats->tx_dropped++;
    
        /* If priorty change fails, discard the packet */
        pkt_mem_free(wlan_tx_req->buffer_addr);
    }
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_tx_data                                      */
/*                                                                           */
/*  Description   : This function handles MAC unit data request from the     */
/*                  host and schedules the incoming packet for transmission. */
/*                                                                           */
/*  Inputs        : 1) Data request structure                                */
/*                  2) Pointer to the buffer                                 */
/*                                                                           */
/*  Globals       : g_q_handle                                               */
/*                                                                           */
/*  Processing    : The MAC header contents are set in the given buffer at   */
/*                  the required offset. The transmit descriptor for the     */
/*                  packet is created and the packet is added to the         */
/*                  appropriate H/w queue for transmission. On receiving a   */
/*                  transmit complete interrupt the buffer is freed.         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
//#ifndef AUTORATE_FEATURE
extern UWORD32 g_cmcc_cfg_tx_rate;
//#endif

#ifdef TROUT_WIFI_NPI
extern UWORD8 g_ap_last_rate;

extern UWORD8 g_user_tx_rate;

#endif

void sta_enabled_tx_data(mac_struct_t *mac, UWORD8 *msg)
{
    UWORD8        q_num        = 0;
    UWORD8        mac_hdr_len  = 0;
    UWORD8        tx_rate      = 0;
    UWORD8        pream        = 0;
    BOOL_T        is_qos       = BFALSE;
    BOOL_T        is_htc       = BFALSE;
    UWORD32       phy_tx_mode  = 0;
    UWORD8        *mac_hdr     = 0;
    UWORD8        *tx_dscr     = 0;
    wlan_tx_req_t *wlan_tx_req = 0;
    CIPHER_T      ct           = NO_ENCRYP;
    sta_entry_t   *se          = 0;
    UWORD8        *tx_info     = 0;
    UWORD8        sta_index    = 0;
    BOOL_T        ignore_port  = BFALSE;
    UWORD8        key_type     = 0;
    UWORD8        is_amsdu     = 0;
    buffer_desc_t buffer_desc  = {0};
    UWORD8        to_ds        = 1;
    UWORD8        *ra          = NULL;
    void          *amsdu_ctxt  = 0;
    UWORD32       retry_set[2] = {0};
    LINK_MODE_T   rate_mode;

	TROUT_FUNC_ENTER;
	
#ifdef WAKE_LOW_POWER_POLICY
	g_low_power_flow_ctrl.tx_pkt_num++;	//tx flow detect.

	if(g_wifi_power_mode == WIFI_LOW_POWER_MODE)
	{
		g_low_power_tx_data_pkt_cnt += 1;
		if(g_low_power_tx_data_pkt_cnt >= LOW_POWER_TX_THRESHOLD)
		{
			printk("%s: has %d pkt need send in low power mode!\n", __func__, g_low_power_tx_data_pkt_cnt);
			exit_low_power_mode(BTRUE);
			g_low_power_tx_data_pkt_cnt = 0;
		}	
	}
#endif

    TX_PATH_DBG("%s: enable tx data\n", __func__);

    wlan_tx_req = (wlan_tx_req_t *)msg;

    /* Change the priority of the packet, if required */
    if(change_priority_if(wlan_tx_req) == BFALSE)
    {
		//caisf add 2013-02-15
        if(g_mac_net_stats)
            g_mac_net_stats->tx_dropped++;
    
        /* If priorty change fails, discard the packet */
        pkt_mem_free(wlan_tx_req->buffer_addr);        
        TROUT_FUNC_EXIT;
        return;
    }

    /* Find the queue to which this packet needs to be added */
    q_num = get_txq_num(wlan_tx_req->priority);

    ignore_port = wlan_tx_req->ignore_port;

    if(mget_DesiredBSSType() == INDEPENDENT)
        to_ds = 0;

    /* Get the RA of the MSDU In case of IBSS mode RA is the DA, it is BSSID */
    /* in Infrastructure BSS Mode                                            */
    ra = (1 == to_ds)? mget_bssid() : wlan_tx_req->da;
    
//#ifndef TROUT_WIFI_NPI
    /* This filter is used to check whether the destination is listed */
    /* If its not listed in the STA Table, then drop the packet       */
    if(filter_host_rx_frame_sta(wlan_tx_req, ra, ignore_port,
                    &ct, &sta_index, &key_type, &tx_info, &se) == BTRUE)
    {
		//caisf add 2013-02-15
        if(g_mac_net_stats)
            g_mac_net_stats->tx_dropped++;
    
        /* The buffer is freed inside the function */
        TROUT_FUNC_EXIT;
        return;
    }
//#endif

    /* If the queue for which this packet belongs is full, then return */
    if(is_txq_full(q_num, sta_index) == BTRUE)
    {
		//caisf add 2013-02-15
        if(g_mac_net_stats)
            g_mac_net_stats->tx_dropped++;
    
        pkt_mem_free(wlan_tx_req->buffer_addr);
		TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
		g_mac_stats.tx_queue_full_count++;
#endif
        return;
    }

    /* Set the transmit rate to the required value. To support Multi-Rate    */
    /* the transmit rate is set to the maximum basic rate and the preamble   */
    /* is set to Long.                                                       */
    /* In case of EAPOL packets, send them at Min basic rate in order to     */
    /* increase their reliability                                            */
    if((is_group(ra) == BTRUE) || (wlan_tx_req->min_basic_rate == BTRUE))
    {
        /* For multicast frames send frames in maximum rate */
        tx_rate = get_min_basic_rate();
        pream   = 1;

        /* Update the retry set information for this frame */
        update_retry_rate_set(0, tx_rate, 0, retry_set);
        //update_retry_rate_set2(tx_rate, se, retry_set);
    }
    else
    {
        /* Get the transmit rate for the associated station based on the     */
        /* auto-rate, multi-rate or user-rate settings. The preamble must be */
        /* set accordingly.                                                  */
    	//tx_rate = 0x87;//dumy add for test TX_RATE 0704
        tx_rate = get_tx_rate_to_sta(se);        
        if(g_wifi_bt_coex)	//wifi & bt coexist mode rate protected!
        {
        	rate_mode = cur_rate_mode_sta();
        	if(rate_mode != B_ONLY_RATE_STA)
        	{
				if((rate_mode == N_ONLY_RATE_STA) && (tx_rate < 0x84))
					tx_rate = 0x84;	//n only mode, mini rate set to 26Mbps.
				else if(tx_rate < 12)//pingjiang modify 2014-01-09
					tx_rate = 12;
        	}
	}

#if 0
	 if(g_cmcc_test_mode)
        {
        	rate_mode = cur_rate_mode_sta();
		if((rate_mode == B_ONLY_RATE_STA) && (tx_rate < 11))
			tx_rate = 11;
		else if((rate_mode == N_ONLY_RATE_STA) && (tx_rate < 0x87))
			tx_rate = 0x87;
		else if(tx_rate < 54)
			tx_rate = 54;
	}
#endif
//#ifndef AUTORATE_FEATURE
		if( g_cmcc_cfg_tx_rate != 0 )
			tx_rate = g_cmcc_cfg_tx_rate;
//#endif

#ifdef TROUT_WIFI_NPI
		if(g_user_tx_rate)
		{
			tx_rate = g_user_tx_rate;
		    printk("npi %d: use user tx rate: %u Mb/s\n", __LINE__, tx_rate);
		}
		else
		{
			if(g_ap_last_rate != 0)
			{
				tx_rate = g_ap_last_rate;
		    	printk("npi %d: use ap last tx rate: %u Mb/s\n", __LINE__, tx_rate);
			}
			else
		    	printk("npi %d: use default tx rate: %u Mb/s\n", __LINE__, tx_rate);
		}
#endif
		
        pream = get_preamble(tx_rate);

		//chenq add for cur rx rate 2012-10-17
		set_cur_tx_rate(tx_rate);
        
     	//tx_rate = 0x87;//dumy add for test TX_RATE 0704
        /* Update the retry set information for this frame */
       update_retry_rate_set(1, tx_rate, se, retry_set);
       //update_retry_rate_set2(tx_rate, se, retry_set);
    }

    /* Check whether AMSDU aggregation is possible */
    if(wlan_tx_req->dont_aggr == BFALSE)
    {
        amsdu_ctxt = get_amsdu_handle(ra, wlan_tx_req->priority, q_num,
                                      se, wlan_tx_req->data_len, 1, tx_rate);
    }

    /* Check whether AMSDU option is enabled */
    is_amsdu = (amsdu_ctxt != NULL);
	//printk("sta: is_amsdu=%d\n", is_amsdu);
    if((NULL == amsdu_ctxt) || (NULL == amsdu_get_tx_dscr(amsdu_ctxt)))
    {
        /* Check whether QoS is enabled */
        is_qos = is_qos_required(sta_index);

        /* Check whether receiver is HT capable */
        is_htc  = is_htc_capable(se);

        /* Create the transmit descriptor and set default contents */
        tx_dscr = create_default_tx_dscr(is_qos, wlan_tx_req->priority,
                                         is_amsdu);

        if(tx_dscr == NULL)
        {
            if(is_amsdu == 1)
            {
                /* If allocation of the AMSDU decriptor has failed, then */
                /* attempt transmission of the MSDU as a normal-MSDU.    */
                free_amsdu_handle(amsdu_ctxt);
                is_amsdu   = 0;
                amsdu_ctxt = NULL;
                tx_dscr = create_default_tx_dscr(is_qos, wlan_tx_req->priority,
                                         is_amsdu);
            }

            if(tx_dscr == NULL)
            {
				//caisf add 2013-02-15
                if(g_mac_net_stats)
                    g_mac_net_stats->tx_dropped++;
    
                pkt_mem_free(wlan_tx_req->buffer_addr);
                TROUT_FUNC_EXIT;
                return;
            }
        }

        /* Update the AMSDU handle with the tx descriptor */
        if(1 == is_amsdu)
            amsdu_set_tx_dscr(amsdu_ctxt, tx_dscr);

        /* The MAC header for the MSDU is created and initialized */
#ifdef TX_MACHDR_IN_DSCR_MEM
        mac_hdr = mem_alloc(g_shared_dscr_mem_handle, MAX_MAC_HDR_LEN);
        if(NULL == mac_hdr)
        {
            free_tx_dscr((UWORD32 *)tx_dscr);
			//caisf add 2013-02-15
            if(g_mac_net_stats)
                g_mac_net_stats->tx_dropped++;
				
            pkt_mem_free(wlan_tx_req->buffer_addr);
            free_amsdu_handle(amsdu_ctxt);
            TROUT_FUNC_EXIT;
            return;
        }
#else  /* TX_MACHDR_IN_DSCR_MEM */
        mac_hdr = wlan_tx_req->buffer_addr;
#endif /* TX_MACHDR_IN_DSCR_MEM */

        /* Set the MAC header fields */
        mac_hdr_len = set_mac_hdr_prot(mac_hdr, wlan_tx_req->priority,
                                       wlan_tx_req->service_class, is_qos,
                                       is_htc, is_amsdu);
#ifdef ENABLE_OVERWRITE_SUBTYPE
        if(get_overwrite_frame_type() != 0)
	        set_sub_type(mac_hdr, get_overwrite_frame_type());
#endif /* ENABLE_OVERWRITE_SUBTYPE */

        /* Set WEP Bit in the packet */
        if(ct != NO_ENCRYP)
            set_wep(mac_hdr, 1);

        /* Set the power management bit in the packet */
        if(check_ps_mode(wlan_tx_req->priority) == 1)
            set_pwr_mgt(mac_hdr, 1);

        /* Set the Address fields for based on the mode of STA operation */
        if(0 == to_ds)
        {
            set_to_ds(mac_hdr, 0);

            /* Set Address1 field in the WLAN Header with destination address */
            set_address1(mac_hdr, wlan_tx_req->da);

            /* Set Address2 field in the WLAN Header with the source address */
            set_address2(mac_hdr, mget_StationID());

            /* Set Address3 field in the WLAN Header with the BSSID */
            set_address3(mac_hdr, mget_bssid());
        }
        else /* INFRASTRUCTURE */
        {
            set_to_ds(mac_hdr, 1);

            /* Set Address1 field in the WLAN Header with the BSSID */
            set_address1(mac_hdr, mget_bssid());

	        if( wlan_tx_req->eth_type == LLTD_TYPE )
	        {
	            /* Set Address2 field in the WLAN Header with the ether source address */
	            set_address2(mac_hdr, wlan_tx_req->ethernet_sa_addr );
	        }
	        else
	        {
	            /* Set Address2 field in the WLAN Header with the source address */
	            set_address2(mac_hdr, mget_StationID());
	        }

            /* Set Address3 field in the WLAN Header with destination address */
            set_address3(mac_hdr, wlan_tx_req->da);
       }

        /* Append the security parameters to the packet to be transmitted */
        /******************************************************************/
        /* For AMSDU frames this should be handled separately             */
        /******************************************************************/
        append_sec_param(ct, wlan_tx_req, tx_info, is_qos);

        /* Set the mac header related parameters in the Tx descriptor */
        set_tx_dscr_buffer_addr((UWORD32 *)tx_dscr, (UWORD32)mac_hdr);
        set_tx_dscr_mh_len((UWORD32 *)tx_dscr, mac_hdr_len);
        set_tx_dscr_mh_offset((UWORD32 *)tx_dscr, 0);

        /* Get the PHY transmit mode based on the transmit rate and preamble */
        phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)se);

        printk("[%s] tx_rate:%u\n", __FUNCTION__, tx_rate);
        set_tx_params(tx_dscr, tx_rate, pream, wlan_tx_req->service_class,
                      phy_tx_mode, retry_set);

	//printk("[%s] %#x, %#x, %#x\n", __FUNCTION__,  get_tx_dscr_data_rate_0((UWORD32*)tx_dscr), get_tx_dscr_retry_rate_set1((UWORD32*)tx_dscr), get_tx_dscr_retry_rate_set2((UWORD32*)tx_dscr));
        set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
		

        /* Set Security Parameters for the frame */
        set_tx_security(tx_dscr, ct, key_type, sta_index);

        /* Set the HT power save parameters */
        set_ht_ps_params(tx_dscr, (void *)se, tx_rate);

        /* Set the receiver address LUT index */
        set_ht_ra_lut_index(tx_dscr, get_ht_handle_entry((void *)se),
                            wlan_tx_req->priority, tx_rate);
    }
    else
        tx_dscr = amsdu_get_tx_dscr(amsdu_ctxt);

//chenq add for wapi 2012-09-26
#ifdef MAC_WAPI_SUPP
	if( ( mget_wapi_enable() == TV_TRUE ) && ( ct == NO_ENCRYP ) )
	{
		if((wlan_tx_req->data[6] != 0x88) || (wlan_tx_req->data[7] != 0xb4)) // send data ptk
		{
			if((wlan_tx_req->data[6] == 0x86) || (wlan_tx_req->data[7] == 0xdd))
			{
				printk("86dd ptk ignor\n");
				free_tx_dscr((UWORD32 *)tx_dscr);	
				//pkt_mem_free(wlan_tx_req->buffer_addr);	
				//caisf add 2013-02-15
                if(g_mac_net_stats)
                    g_mac_net_stats->tx_dropped++;
				return;
			}
		
			if(mget_wapi_key_ok() == TV_TRUE) 
			{
				UWORD8 * output_buf = NULL;
				UWORD16 output_data_len = 0;
				
				output_buf = pkt_mem_alloc(MEM_PRI_TX);
				
				
				if(output_buf == NULL)
				{
					free_tx_dscr((UWORD32 *)tx_dscr);
					//pkt_mem_free(wlan_tx_req->buffer_addr);
					//caisf add 2013-02-15
                    if(g_mac_net_stats)
                        g_mac_net_stats->tx_dropped++;
					return;
				}

				set_wep(mac_hdr, 1);

				if( (output_data_len = wlan_rx_wapi_encryption(wlan_tx_req->buffer_addr,
											                  wlan_tx_req->data,wlan_tx_req->data_len,
											                  output_buf) )
					== 0 )
				{
					free_tx_dscr((UWORD32 *)tx_dscr);
					//pkt_mem_free(wlan_tx_req->buffer_addr);
					pkt_mem_free(output_buf);
					//caisf add 2013-02-15
                    if(g_mac_net_stats)
                        g_mac_net_stats->tx_dropped++;
					
					return;	
				}

				memcpy(output_buf,mac_hdr,mac_hdr_len);
				pkt_mem_free(wlan_tx_req->buffer_addr);
				wlan_tx_req->buffer_addr = output_buf;
				wlan_tx_req->data        = output_buf + get_eth_hdr_offset(ETHERNET_HOST_TYPE);
				wlan_tx_req->data_len    = output_data_len;
				set_tx_dscr_buffer_addr((UWORD32 *)tx_dscr, (UWORD32)output_buf);
			}
			else
			{
				printk("wapi key not ready! will not tx data ptk\n");
				free_tx_dscr((UWORD32 *)tx_dscr);
				//pkt_mem_free(wlan_tx_req->buffer_addr);
				//caisf add 2013-02-15
                if(g_mac_net_stats)
                    g_mac_net_stats->tx_dropped++;
				return;
			}
		}	
		else
		{	
			printk(" 88b4 ptk normal send !!!\n");
		}
	}
#endif

	/* Set the sub-MSDU info table in the Tx-Descriptor */
    buffer_desc.buff_hdl    = wlan_tx_req->buffer_addr;
    buffer_desc.data_offset = (wlan_tx_req->data - wlan_tx_req->buffer_addr);
    buffer_desc.data_length = wlan_tx_req->data_len;

    set_submsdu_info(tx_dscr, &buffer_desc, 1, wlan_tx_req->data_len,
                     mac_hdr_len, mget_StationID(), wlan_tx_req->da, amsdu_ctxt);

    /* Update the TSF timestamp */
    //update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);	//chengwg mask, update before send to trout.

    /* Transmit the MSDU */
    if(BTRUE == is_tx_ready(amsdu_ctxt))
    {	
    	TX_PATH_DBG("%s: tx data pkt\n", __func__);
        if(tx_data_packet((UWORD8 *)se, ra, wlan_tx_req->priority, q_num,
                       tx_dscr, amsdu_ctxt) == BTRUE)
                       wlan_tx_req->added_to_q = BTRUE;
    }
    
    TROUT_FUNC_EXIT;
}

#endif /* IBSS_BSS_STATION_MODE */
