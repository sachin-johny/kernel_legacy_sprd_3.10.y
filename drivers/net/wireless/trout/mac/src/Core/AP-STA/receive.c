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
/*  File Name         : receive.c                                            */
/*                                                                           */
/*  Description       : This file contains the MAC receive path related      */
/*                      functions independent of mode.                       */
/*                                                                           */
/*  List of Functions : process_wlan_rx                                      */
/*                      process_rx_frame                                     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "prot_if.h"
#include "receive.h"
#include "qmu_if.h"
#include "core_mode_if.h"
#include "trout_wifi_rx.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_wlan_rx                                          */
/*                                                                           */
/*  Description   : This function processes a WLAN Rx event.                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to MAC structure                              */
/*                  2) Pointer to the WLAN Rx event                          */
/*                                                                           */
/*  Globals       : g_q_handle                                               */
/*                                                                           */
/*  Processing    : This function processes the WLAN Rx event. It checks the */
/*                  sync flag and synchronizes the receive queue if set. In  */
/*                  this process it cleans any orphaned Rx frames. It then   */
/*                  updates the Rx Q by removing all descriptors associated  */
/*                  with the Rx frame of this event. All the associated      */
/*                  descriptors of this Rx frame are then processed. In the  */
/*                  end the descriptor list along with  buffers are freed.   */
/*                  Note that if any buffer is sent for further processing   */
/*                  (sent to host or WLAN or buffered) a user needs to be    */
/*                  added to the buffer. The receive queue is replenished in */
/*                  the end and the MIB is updated for this Rx frame.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifndef TROUT_WIFI
void process_wlan_rx(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_event_msg_t *wlan_rx   = (wlan_rx_event_msg_t*)msg;
    UWORD8              rxq_num    = wlan_rx->rxq_num;
    UWORD8              num_dscr   = wlan_rx->num_dscr;
    UWORD32             *base_dscr = (UWORD32 *)(wlan_rx->base_dscr);
    UWORD8              num_buffs  = 0;
    UWORD8              num_mpdu   = 0;
    UWORD8              rx_dscr_status = 0;

    /* Exception case 0: Base descriptor becomes NULL before the number  */
    /* of descriptors indicated in the interrupt have been processed.    */
    if(base_dscr == NULL)
    {
#ifdef DEBUG_MODE
        PRINTD2("HwEr:NullRxDscr\n\r");
        g_mac_stats.pwrx_null_dscr++;
#endif /* DEBUG_MODE */

        /* Do nothing and return */
        return;
    }

    /* Check if the global flag indicating synchronization loss is set for   */
    /* this receive queue. If set, call a function to synchronize this queue */
    /* with the base descriptor.                                             */
    if(wlan_rx->sync_req == 1)
        qmu_sync_rx_packet(&g_q_handle.rx_handle, rxq_num, base_dscr);

    /* Unlink the total number of descriptors from the receive queue */
    qmu_update_rx_q(&g_q_handle.rx_handle, rxq_num, base_dscr, num_dscr);

    rx_dscr_status = get_rx_dscr_pkt_status(base_dscr);

    /* Exception Case: The status of receive descriptor passed is not updated */
    if(rx_dscr_status == RX_NEW)
    {
        free_rx_dscr_list(base_dscr, num_dscr);
        num_dscr = 0;
#ifdef DEBUG_MODE
            PRINTD2("HwEr:UnexpNewRxDscr\n\r");
            g_mac_stats.pwrx_unexp_newdscr++;
#endif /* DEBUG_MODE */
    }

    /* Loop till number of descriptors remaining to be processed becomes 0 */
    while(num_dscr != 0)
    {
        /* Exception case 1: Base descriptor becomes NULL before the number  */
        /* of descriptors indicated in the interrupt have been processed.    */
        if(base_dscr == NULL)
        {
#ifdef DEBUG_MODE
            PRINTD2("HwEr:RxUnexpEnd\n\r");
            g_mac_stats.pwrx_unexp_end++;
#endif /* DEBUG_MODE */

            /* Do nothing and break */
            break;
        }

        /* Exception case 2: The base descriptor is not the first buffer of  */
        /* an MSDU. Currently this flag is set only for AMSDU frames.        */
        if((is_rx_frame_amsdu(base_dscr) == BTRUE) &&
           (is_rx_frame_start_buf(base_dscr) != BTRUE))
        {
#ifdef DEBUG_MODE
            PRINTD2("HwEr:RxNotStart\n\r");
            g_mac_stats.pwrx_notstart++;
#endif /* DEBUG_MODE */

            /* Free all the remaining descriptors and break */
            free_rx_dscr_list(base_dscr, num_dscr);
            break;
        }

        /* Extract the number of buffers from the base descriptor */
        num_buffs = get_rx_dscr_num_buffers(base_dscr);

        /* Exception case 3: The number of buffers exceeds the remaining     */
        /* number of descriptors                                             */
        if(num_buffs > num_dscr)
        {
#ifdef DEBUG_MODE
            PRINTD2("HwEr:RxNumExc\n\r");
            g_mac_stats.pwrx_numexceed++;
#endif /* DEBUG_MODE */

            /* Free all the remaining descriptors and break */
            free_rx_dscr_list(base_dscr, num_dscr);
            break;
        }

        /* Call function to process the receive frame */
        process_rx_frame(&g_mac, base_dscr, num_buffs);

        /* Update the number of remaining descriptors by decrementing the    */
        /* number of buffers processed                                       */
        num_dscr -= num_buffs;

        /* Increment the number of MPDUs processed */
        num_mpdu++;

        /* Free receive descriptors (and buffers) for this receive frame and */
        /* return the next descriptor in the list.                           */
        /* Important: Note that users need to be added to the receive buffer */
        /* whenever it is sent for further processing (sent to Host, or WLAN */
        /* or buffered) and freed explicitly after such processing is done.  */
        /* No other receive buffers need to be freed outside this function   */
        /* and all buffers for which processing is complete and are not      */
        /* required further will be freed here.                              */
        if(g_mac_reset_done == BFALSE)
            base_dscr = free_rx_dscr_list(base_dscr, num_buffs);
    }

    /* Replenish the receive queue */
    replenish_rx_queue(&g_q_handle, rxq_num);

    /* Update required MIB for the aggregation of frames received */
    update_aggr_rx_mib(num_mpdu);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_rx_frame                                         */
/*                                                                           */
/*  Description   : This function processes one received frame from WLAN.    */
/*                                                                           */
/*  Inputs        : 1) Pointer to MAC structure                              */
/*                  2) Pointer to Rx frame base descriptor                   */
/*                  3) Number of Rx descriptors forming the Rx frame         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC header pointer is extracted from the base        */
/*                  descriptor. Various checks are performed on this header  */
/*                  and the frame is dropped in case any checks fail. A WLAN */
/*                  Rx structure is filled up with required information and  */
/*                  then passed for further processing based on the type of  */
/*                  frame.                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void rx_frame_test(mac_struct_t *mac, UWORD32 *base_dscr, UWORD8 num_dscr)	//chengwg debug.
{
	UWORD8      status      = 0;
	UWORD8      *msa        = 0;
    wlan_rx_t   wlan_rx     = {0};
    UWORD8 *addr = NULL;
    UWORD32     phy_rx_mode = 0;
    BOOL_T      is_rx_ba_sc = BFALSE;

	printk("%s: base=0x%p, num_dscr=%d\n", __func__, base_dscr, num_dscr);
	while(num_dscr > 0)
	{
	    status         = get_rx_dscr_pkt_status(base_dscr);
	    msa            = (UWORD8*)get_rx_dscr_buffer_ptr(base_dscr);
	    wlan_rx.rx_len = get_rx_dscr_frame_len(base_dscr);

	    get_address1(msa, wlan_rx.addr1);	//Destination
	    get_address2(msa, wlan_rx.addr2);	//BSSID
	    get_address3(msa, wlan_rx.addr3);	//Source

	    wlan_rx.type     = get_type(msa);
	    wlan_rx.frm_ds   = get_from_ds(msa);
	    wlan_rx.to_ds    = get_to_ds(msa);
	    wlan_rx.sub_type = get_sub_type(msa);
	    wlan_rx.is_grp_addr = is_group(wlan_rx.addr1);

	    set_SA_DA_BSSID_ptr(&wlan_rx);

	    if(get_protocol_version(msa) != PROTOCOL_VERSION)
	    {
			printk("protocol version error!\n");
			return;
	    }

	    if(check_from_to_ds(&wlan_rx) == BTRUE)
	    {
			printk("to ds frame!\n");
			return;
	    }

	    if(check_bssid_match(mac, &wlan_rx) == BFALSE)
	    {
			printk("bssid not match!\n");
			return;
	    }

	    if(check_source_address(wlan_rx.sa) == BTRUE)
	    {
			printk("source address not match!\n");
			return;
	    }

		wlan_rx.priority_val  = get_priority_value(msa);
	    wlan_rx.service_class = get_ack_policy(msa, wlan_rx.addr1);
	    wlan_rx.sa_entry      = find_entry(wlan_rx.addr2);
	
	    /* Update the base descriptor, number of descriptors and  */
	    /* MAC header start address                               */
	    wlan_rx.base_dscr = base_dscr;
	    wlan_rx.num_dscr  = num_dscr;
	    wlan_rx.msa       = msa;
    
		phy_rx_mode = get_rx_dscr_prm(base_dscr);
    	is_rx_ba_sc = is_rx_ba_service_class(wlan_rx.sa_entry,
                                         		wlan_rx.priority_val);
		if((is_ampdu_rx_prm(phy_rx_mode) == BTRUE) && (is_rx_ba_sc == BFALSE))
	    {
	        if((wlan_rx.service_class == BLOCK_ACK) || (wlan_rx.service_class == NORMAL_ACK))
	        {
				BOOL_T frag, start, amsdu;
				
				frag = is_rx_frame_frag(base_dscr);
				start = is_rx_frame_start_buf(base_dscr);
				amsdu = is_rx_frame_amsdu(base_dscr);
				printk("frag=%d, start=%d, amsdu=%d\n", frag, start, amsdu);
				
		    	addr = wlan_rx.addr1;
				printk("Destination: %02x-%02x-%02x-%02x-%02x-%02x\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		    	addr = wlan_rx.addr2;
				printk("BSSID: %02x-%02x-%02x-%02x-%02x-%02x\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		    	addr = wlan_rx.addr3;
				printk("Source: %02x-%02x-%02x-%02x-%02x-%02x\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

				printk("rx_dscr=0x%p, msa=0x%p, rx_len=%d\n", base_dscr, msa, wlan_rx.rx_len);
				if(wlan_rx.rx_len > 64)
	        		hex_dump("err_pkt2", msa, 64);
	        	else
	        		hex_dump("err_pkt2", msa, wlan_rx.rx_len);
	        }
	    }

		num_dscr--;
	    base_dscr = (UWORD32 *)get_rx_dscr_next_dscr((UWORD32 *)base_dscr);
	    if(num_dscr > 0 && base_dscr == NULL)
	    {
			printk("test not complete, %d!\n", num_dscr);
	    	break;
	    }
	    else if(num_dscr == 0 && base_dscr != NULL)
	    	printk("last dscr is no end flag!\n");
    }
    printk("------exit------\n");
}
#endif	/* TROUT_WIFI */

void process_rx_frame(mac_struct_t *mac, UWORD32 *base_dscr, UWORD8 num_dscr)
{
    CIPHER_T    ct          = NO_ENCRYP;
    UWORD8      status      = 0;
    UWORD8      min_pkt_len = MAC_HDR_LEN + FCS_LEN;
    UWORD8      *msa        = 0;
    wlan_rx_t   wlan_rx     = {0};
    UWORD32     phy_rx_mode = 0;
    BOOL_T      is_rx_ba_sc = BFALSE;

	TROUT_FUNC_ENTER;
    print_log_debug_level_1("\n[DL1][INFO][Rx] {MAC API for WLAN Rx}");

    /* Extract the receive status, pointer to MAC header and receive packet  */
    /* length from the base descriptor                                       */
    status         = get_rx_dscr_pkt_status(base_dscr);
    msa            = (UWORD8*)get_host_rxds_buffer_ptr(base_dscr);
    wlan_rx.rx_len = get_rx_dscr_frame_len(base_dscr);
	
    /* Extract Addresses from the MAC Header */
    get_address1(msa, wlan_rx.addr1);	//Destination
    get_address2(msa, wlan_rx.addr2);	//BSSID
    get_address3(msa, wlan_rx.addr3);	//Source

    wlan_rx.type     = get_type(msa);
    wlan_rx.frm_ds   = get_from_ds(msa);
    wlan_rx.to_ds    = get_to_ds(msa);
    wlan_rx.sub_type = get_sub_type(msa);
    wlan_rx.is_grp_addr = is_group(wlan_rx.addr1);

    /* Update the SA, DA & BSSID pointers to corresponding addr1, addr2 or   */
    /* addr3 fields of wlan_rx structure.                                    */
    set_SA_DA_BSSID_ptr(&wlan_rx);

    /* Update the debug statistics based on the status */
    update_debug_rx_stats(status);

    /* Exception case 1: Maximum length exceeded */
    if(wlan_rx.rx_len > MAX_MSDU_LEN)
    {
#ifdef DEBUG_MODE
        PRINTD2("HwEr:RxMaxLenExc:%d > %d\n\r",wlan_rx.rx_len, MAX_MSDU_LEN);
        g_mac_stats.pwrx_maxlenexc++;
#endif /* DEBUG_MODE */
        /* Do nothing and return */
        TROUT_FUNC_EXIT;
        return;
    }
//#ifndef TROUT_WIFI_NPI
    /* Exception case 2: Protocol Version Match Fail */
    if(get_protocol_version(msa) != PROTOCOL_VERSION)
    {
#ifdef DEBUG_MODE
        PRINTD2("HwEr:RxFrmHdrProtVerFail\n\r");
        g_mac_stats.rxfrmhdrprotverfail++;
#endif /* DEBUG_MODE */

        /* Do nothing and return */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Exception case 3: Address-4 Field Present */
    if(check_from_to_ds(&wlan_rx) == BTRUE)
    {
#ifdef DEBUG_MODE
        PRINTD2("HwEr:RxFrmHdrAddr4Prsnt\n\r");
        g_mac_stats.rxfrmhdraddr4prsnt++;
#endif /* DEBUG_MODE */

        /* Do nothing and return */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Exception case 4: BSSID Match Fail */
    if(check_bssid_match(mac, &wlan_rx) == BFALSE)
    {
#ifdef DEBUG_MODE
#ifdef BSS_ACCESS_POINT_MODE
        if(wlan_rx.is_grp_addr == BFALSE)
#endif /* BSS_ACCESS_POINT_MODE */
        PRINTD2("HwEr:RxFrmHdrBssidChkFail\n\r");
        g_mac_stats.rxfrmhdrbssidchkfail++;
#endif /* DEBUG_MODE */

	//chenq add for check mac state
	#ifdef IBSS_BSS_STATION_MODE
		g_wrong_bssid++;
	#endif
	
        /* Do nothing and return */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Exception case 5: Source Address Match Fail */
    if(check_source_address(wlan_rx.sa) == BTRUE)
    {
#ifdef DEBUG_MODE
        if(wlan_rx.is_grp_addr == BFALSE)
            g_mac_stats.rxfrmhdraddrchkfail++;
#endif /* DEBUG_MODE */
        /* Do nothing and return */
        TROUT_FUNC_EXIT;
        return;
    }
//#endif
    //chenq add for wapi 2012-09-26
#ifdef MAC_WAPI_SUPP
	if( ( mget_wapi_enable() == TV_FALSE ) && (get_wep(msa)) )
#else
    if(get_wep(msa))
#endif	
    {
        /* Extract cipher type from receive descriptor */
        ct = (CIPHER_T)get_rx_dscr_cipher_type(base_dscr);

        /* Exception case 3: Unknown cipher type */
        if(is_valid_cipher_type(ct) == BFALSE)
        {
#ifdef DEBUG_MODE
            PRINTD2("HwEr:InvalidCT=%d\n",ct);
            g_mac_stats.invalidct++;
#endif /* DEBUG_MODE */
            /* Do nothing and return */
            TROUT_FUNC_EXIT;
            return;
        }

        /* Update the minimum packet length based on security header/trailer */
        min_pkt_len += get_sec_header_len(ct) + get_sec_trailer_len(ct) + 1;

        wlan_rx.ct = ct;
    }
    else
    {
        wlan_rx.ct = NO_ENCRYP;

        if(wlan_rx.sub_type == PS_POLL)
            min_pkt_len = PS_POLL_LEN;

#ifdef MAC_802_11N
        if(wlan_rx.sub_type == BLOCKACK_REQ)
            min_pkt_len = BLOCKACK_REQ_MIN_LEN;
#endif /* MAC_802_11N */
    }

    /* Exception case 6: Less than minimum acceptable length */
    if(wlan_rx.rx_len < min_pkt_len)
    {
#ifdef DEBUG_MODE
        PRINTD2("HwEr:RxMinLenExc: %d < %d\n\r", wlan_rx.rx_len, min_pkt_len);
        g_mac_stats.pwrx_minlenexc++;
#endif /* DEBUG_MODE */
        /* Do nothing and return */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Extract required parameters from the frame header */
    wlan_rx.priority_val  = get_priority_value(msa);
    wlan_rx.service_class = get_ack_policy(msa, wlan_rx.addr1);
    wlan_rx.sa_entry      = find_entry(wlan_rx.addr2);

	//print_all_entry();
	
    /* Update the base descriptor, number of descriptors and  */
    /* MAC header start address                               */
    wlan_rx.base_dscr = base_dscr;
    wlan_rx.num_dscr  = num_dscr;
    wlan_rx.msa       = msa;

    /* Extract the PHY Rx mode from the descriptor, update the Rx flag */
    phy_rx_mode = get_rx_dscr_prm(base_dscr);
    is_rx_ba_sc = is_rx_ba_service_class(wlan_rx.sa_entry,
                                         wlan_rx.priority_val);

    /* On Reception of AMPDU with ACK policy set to normal/block ack */
    /* on TID with no active BA session, send DELBA frame            */
#if 0	//masked by chengwg, when rcved AMPDU pkt, this is a bug, not fixed!(2012.12.10)
    if((is_ampdu_rx_prm(phy_rx_mode) == BTRUE) && (is_rx_ba_sc == BFALSE))
    {
        if((wlan_rx.service_class == BLOCK_ACK) || (wlan_rx.service_class == NORMAL_ACK))
        {        	
			printk("amsdu=%d, start=%d\n", is_rx_frame_amsdu(base_dscr), is_rx_frame_start_buf(base_dscr));
			if(wlan_rx.rx_len > 64)
        		hex_dump("err_pkt", msa, 64);
        	else
        		hex_dump("err_pkt", msa, wlan_rx.rx_len);
            handle_non_ba_ampdu_rx(wlan_rx.priority_val, wlan_rx.addr2);
        }
    }
#endif	
    /* Exception case 7: Status not RX_SUCCESS */
    /* For BLOCK ACK case, Replay failure status from Hw is ignored */
    if((status != RX_SUCCESS) &&
       ((is_rx_ba_sc == BFALSE) ||
       ((status != TKIP_REPLAY_FAILURE) && (status != CCMP_REPLAY_FAILURE))))
    {
        /* Update any failure statistics as required and return */

		printk("[wjb]before update_sec_fail_stats, status %d, grp is %d\n",
			status, wlan_rx.is_grp_addr);

        update_sec_fail_stats(ct, status, wlan_rx.ta, wlan_rx.da);

#ifdef DEBUG_MODE
        if(BTRUE == wlan_rx.is_grp_addr)
        {
            PRINTD2("HwEr:BRx%d ",status);
            g_mac_stats.brx_frame_droped++;
        }
        else
        {
            PRINTD2("HwEr:URx%d ",status);
            g_mac_stats.urx_frame_droped++;
        }
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }
    else
        update_sec_success_stats(wlan_rx.sa_entry, ct, status);

    /* Exception case 8: Not a BC/MC frame and not directed */
//#ifndef TROUT_WIFI_NPI
    if((wlan_rx.is_grp_addr == BFALSE) &&
       (mac_addr_cmp(wlan_rx.addr1, mget_StationID()) == BFALSE) &&
       (useful_non_directed(mac, (TYPESUBTYPE_T)wlan_rx.sub_type) == BFALSE))
    {
#ifdef DEBUG_MODE
		{
	        UWORD8 *addr1 = wlan_rx.addr1;
	        PRINTD2("HwEr:RxUnexpFrm:%x:%x:%x:%x:%x:%x\n\r",addr1[0],addr1[1],addr1[2],
	        									addr1[3],addr1[4],addr1[5]);
	        g_mac_stats.rxunexpfrm++;
		}
#endif /* DEBUG_MODE */
        /* Do nothing and return */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Filter the frame based on mode of operation */
    if(filter_wlan_rx_frame(&wlan_rx) == BTRUE)
    {
#ifdef DEBUG_MODE
        g_mac_stats.pewrxft++;
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }
//#endif
#ifdef DISABLE_MACHW_DEAGGR
    /* Check whether fragmented frame or AMSDU frame */
    if((wlan_rx.type == DATA_BASICTYPE) && (is_amsdu_frame(msa) == BTRUE))
    {
#ifdef DEBUG_MODE
        g_mac_stats.num_amsdu_drop++;
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }
#endif /* DISABLE_MACHW_DEAGGR */

#ifdef DISABLE_MACHW_DEFRAG
    /* Extract fragment number and more frag bit from the incoming MPDU */
    {
        UWORD8 frag_num = 0;
        UWORD8 more_frag = 0;

        frag_num  = get_fragment_number(msa);
        more_frag = get_more_frag(msa);

        if((frag_num > 0) || (more_frag == 1))
        {
#ifdef DEBUG_MODE
            g_mac_stats.num_frag_drop++;
#endif /* DEBUG_MODE */
			TROUT_FUNC_EXIT;
            return;
        }
    }
#endif /* DISABLE_MACHW_DEFRAG */

    /* Modify the frame length and get the frame header length depending on  */
    /* the security, QoS and HT features enabled.                            */
    wlan_rx.hdr_len = modify_frame_length((CIPHER_T)wlan_rx.ct, msa,
                                       &(wlan_rx.rx_len), &(wlan_rx.data_len));

    /* Check for the type of frame and process accordingly */
    if((wlan_rx.is_grp_addr == BFALSE) && (wlan_rx.type == CONTROL))
    {
        wlan_rx_control((UWORD8 *)&wlan_rx);
#ifdef DEBUG_MODE
		g_mac_stats.rx_ctrl_frame_count++;
		g_mac_stats.rx_ctrl_frame_len += wlan_rx.data_len;
#endif
    }
    else if(wlan_rx.type == MANAGEMENT)
    {
        /* Update receive MIB counters */
        //mincr_ReceivedFragmentCount(); //dumy mard for test 0726
        wlan_rx_mgmt(mac, (UWORD8 *)&wlan_rx);
#ifdef DEBUG_MODE
		g_mac_stats.rx_mgmt_frame_count++;
		g_mac_stats.rx_mgmt_frame_len += wlan_rx.data_len;
#endif        
    }
    else if(wlan_rx.type == DATA_BASICTYPE)
    {
#ifdef MEASURE_PROCESSING_DELAY
        g_delay_stats.numrxdscr += num_dscr;
#endif /* MEASURE_PROCESSING_DELAY */

        /* Update receive MIB counters */
        if(wlan_rx.is_grp_addr == BTRUE)
        {
            print_log_debug_level_1("\n[DL1][INFO][Rx] {Multicast WLAN Rx}");
            mincr_MulticastReceivedFrameCount();
        }
        else
        {
            print_log_debug_level_1("\n[DL1][INFO][Rx] {Unicast WLAN Rx}");
            mincr_ReceivedFragmentCount();
        }

        update_rx_mib_prot(msa, wlan_rx.data_len);

		//chenq add for wapi 2012-09-26
		#ifdef MAC_WAPI_SUPP
		if( ( mget_wapi_enable() == TV_TRUE )  && ( get_wep(msa) ) )
		{
			wlan_rx_t tmp_wlan_rx;
			UWORD8 * output_buf = NULL;
			UWORD16 output_data_len = 0;

			memcpy((UWORD8 *)&tmp_wlan_rx,(UWORD8 *)&wlan_rx,sizeof(wlan_rx));
			
			output_buf = pkt_mem_alloc(MEM_PRI_TX);
			if(output_buf == NULL)
			{
				return;
			}

			if( (output_data_len = wlan_tx_wapi_decryption(wlan_rx.msa,wlan_rx.hdr_len,
				                                            wlan_rx.data_len,
				                                            (UWORD8 *)(output_buf+wlan_rx.hdr_len)))
				 == 0)
			{
				pkt_mem_free(output_buf);
				return;
			}

			memcpy(output_buf,wlan_rx.msa,wlan_rx.hdr_len);
			wlan_rx.msa      = output_buf;
			wlan_rx.rx_len   -= wlan_rx.data_len;
			wlan_rx.data_len = output_data_len;
			wlan_rx.rx_len   += wlan_rx.data_len;

			wlan_rx.wapi_mode = 1;

			wlan_rx_data(mac, (UWORD8 *)&wlan_rx);

			//memcpy((UWORD8 *)&wlan_rx,(UWORD8 *)&tmp_wlan_rx,sizeof(wlan_rx));
			//pkt_mem_free(output_buf);
		}
		else
		{
			wlan_rx.wapi_mode = 0;
			wlan_rx_data(mac, (UWORD8 *)&wlan_rx);
		}
		#else
        wlan_rx_data(mac, (UWORD8 *)&wlan_rx);
		#endif
    }
    TROUT_FUNC_EXIT;
}
