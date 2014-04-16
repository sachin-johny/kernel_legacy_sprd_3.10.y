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
/*  File Name         : host_rx_msdu_ap.c                                    */
/*                                                                           */
/*  Description       : This file contains the functions called by the AP    */
/*                      MAC FSM on receiving HOST_RX (MSDU) events.          */
/*                                                                           */
/*  List of Functions : ap_enabled_tx_data                                   */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate_ap.h"
#include "pm_ap.h"
#include "ap_prot_if.h"
#include "host_if.h"
#include "qmu_if.h"
#include "buff_desc.h"
#include "core_mode_if.h"

// 20120709 caisf add, merged ittiam mac v1.2 code
extern UWORD16 g_num_mc_bc_qd_pkt;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_tx_data                                       */
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



void ap_enabled_tx_data(mac_struct_t *mac, UWORD8 *msg)
{
    UWORD8        q_num        = 0;
    UWORD8        mac_hdr_len  = 0;
    UWORD8        tx_rate      = 0;
    UWORD8        pream        = 0;
    BOOL_T        is_qos       = BFALSE;
    BOOL_T        is_htc       = BFALSE;
    UWORD32       phy_tx_mode  = 0;
    UWORD8        *mac_hdr     = NULL;
    UWORD8        *tx_dscr     = NULL;
    wlan_tx_req_t *wlan_tx_req = 0;
    CIPHER_T      ct           = NO_ENCRYP;
    UWORD8        sta_index    = 0;
    UWORD8        *tx_info     = NULL;
    UWORD8        key_type     = 0;
    BOOL_T        ignore_port  = BFALSE;
    asoc_entry_t  *ae          = NULL;
    buffer_desc_t buffer_desc  = {0};
    UWORD8        is_amsdu     = 0;
    void          *amsdu_ctxt  = 0;
    UWORD32       retry_set[2] = {0};

    print_log_debug_level_1("\n[DL1][INFO][Tx] {MAC API for WLAN Tx DATA}");

    wlan_tx_req = (wlan_tx_req_t *)msg;
    ignore_port = wlan_tx_req->ignore_port;

    /* This filter is used to check whether the destination is associated,   */
    /* If its not associated, then drop the packet                           */
    if(filter_host_rx_frame_ap(wlan_tx_req, ignore_port, &ct, &sta_index,
                               &key_type, &tx_info, &ae) == BTRUE)
    {
	    //caisf add 2013-02-15
        if(g_mac_net_stats)
            g_mac_net_stats->tx_dropped++;
			
        /* The buffer is freed inside the function */
        return;
    }

    /* Find the queue to which this packet needs to be added */
    q_num = get_txq_num(wlan_tx_req->priority);

    /* If the queue for which this packet belongs is full, then return */
    if(is_txq_full(q_num, sta_index) == BTRUE)
    {
	    //caisf add 2013-02-15
        if(g_mac_net_stats)
            g_mac_net_stats->tx_dropped++;
			
        pkt_mem_free(wlan_tx_req->buffer_addr);

        return;
    }

    /* Set the transmit rate to the required value. To support Multi-Rate    */
    /* the transmit rate is set to the maximum basic rate and the preamble   */
    /* is set to Long.                                                       */
    /* In case of EAPOL packets, send them at Min basic rate in order to     */
    /* increase their reliability                                            */
    if((is_group(wlan_tx_req->da) == BTRUE) ||
       (wlan_tx_req->min_basic_rate == BTRUE))
    {
        /* For multicast frames send frames in maximum rate */
        tx_rate = get_min_basic_rate();
        pream   = 1;

        /* Update the retry set information for this frame */
        update_retry_rate_set(0, tx_rate, 0, retry_set);
    }
    else
    {
        /* Get the transmit rate for the associated station based on the     */
        /* auto-rate, multi-rate or user-rate settings. The preamble must be */
        /* set accordingly.                                                  */
        tx_rate = get_tx_rate_to_sta(ae);

		  //tx_rate = 24;//dumy add for test TX_RATE 0704

		//chenq add for cur tx rate
		set_cur_tx_rate(tx_rate);        
        pream   = get_preamble(tx_rate);

        /* Update the retry set information for this frame */
        update_retry_rate_set(1, tx_rate, ae, retry_set);
    }

    /* Check whether AMSDU aggregation is possible */
    if(wlan_tx_req->dont_aggr == BFALSE)
    {
        amsdu_ctxt = get_amsdu_handle(wlan_tx_req->da, wlan_tx_req->priority,
                                  q_num, ae, wlan_tx_req->data_len, 1,
                                  tx_rate);
    }

    /* Check whether AMSDU option is enabled */
    is_amsdu = (amsdu_ctxt != NULL);
	//printk("is_amsdu = %d\n", is_amsdu);

    /* Check whether AMSDU aggregation is not supported or if this is the */
    /* first sub-MSDU of an AMSDU.                                        */
    if((NULL == amsdu_ctxt) || (NULL == amsdu_get_tx_dscr(amsdu_ctxt)))
    {
        /* Check whether QoS option is implemented */
        is_qos = is_qos_required(sta_index);

        /* Check whether receiver is HT capable */
        is_htc  = is_htc_capable(ae);

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
            return;
        }
#else  /* TX_MACHDR_IN_DSCR_MEM */
        mac_hdr = wlan_tx_req->buffer_addr;
#endif /* TX_MACHDR_IN_DSCR_MEM */

        mac_hdr_len = set_mac_hdr_prot(mac_hdr, wlan_tx_req->priority,
                                       wlan_tx_req->service_class, is_qos,
                                       is_htc, is_amsdu);

        set_from_ds(mac_hdr, 1);

        if(ct != NO_ENCRYP)
            set_wep(mac_hdr, 1);

        /* Set Address1 field in the WLAN Header with destination address */
        set_address1(mac_hdr, wlan_tx_req->da);

        /* Set Address2 field in the WLAN Header with the BSSID */
        set_address2(mac_hdr, mget_bssid());

        /* Set Address2 field in the WLAN Header with the source address */
        set_address3(mac_hdr, wlan_tx_req->sa);

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
        phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)ae);
        printk("[%s] tx_rate:%u\n", __FUNCTION__, tx_rate);
        /* set the parameters related to the frame in the Tx DSCR */
        set_tx_params(tx_dscr, tx_rate, pream, wlan_tx_req->service_class,
                      phy_tx_mode, retry_set);
        set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);

        /* Set Security Parameters for the frame */
        set_tx_security(tx_dscr, ct, key_type, sta_index);

        /* Set the HT power save parameters */
        set_ht_ps_params(tx_dscr, (void *)ae, tx_rate);

        /* Set the receiver address LUT index */
        set_ht_ra_lut_index(tx_dscr, get_ht_handle_entry((void *)ae),
                            wlan_tx_req->priority, tx_rate);
    }
    else
        tx_dscr = amsdu_get_tx_dscr(amsdu_ctxt);

    /* Set the payload related parameters in the Tx descriptor */
    buffer_desc.buff_hdl    = wlan_tx_req->buffer_addr;
    buffer_desc.data_offset = (wlan_tx_req->data - wlan_tx_req->buffer_addr);
    buffer_desc.data_length = wlan_tx_req->data_len;

    set_submsdu_info(tx_dscr, &buffer_desc, 1, wlan_tx_req->data_len,
                     mac_hdr_len, wlan_tx_req->sa, wlan_tx_req->da, amsdu_ctxt);

    /* Update the TSF timestamp */
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    /* Transmit the MSDU */
    if(BTRUE == is_tx_ready(amsdu_ctxt))
    {
        if(tx_data_packet((UWORD8 *)ae, wlan_tx_req->da, wlan_tx_req->priority,
                          q_num, tx_dscr, amsdu_ctxt) == BTRUE)
        {
                          wlan_tx_req->added_to_q = BTRUE;

			// 20120709 caisf add, merged ittiam mac v1.2 code
            /* Increment count indicating packet added to Hw queue */
            /* Check if packet Bcast/Mcast or Unicast */
            if(is_group(wlan_tx_req->da) == BTRUE)
            {
                g_num_mc_bc_qd_pkt++;
            }
        }
    }
}

#endif /* BSS_ACCESS_POINT_MODE */
