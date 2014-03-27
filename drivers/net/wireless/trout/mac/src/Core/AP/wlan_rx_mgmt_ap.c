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
/*  File Name         : wlan_rx_mgmt_ap.c                                    */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the AP MAC FSM on receiving      */
/*                      WLAN_RX (MGMT) events.                               */
/*                                                                           */
/*  List of Functions : ap_enabled_rx_mgmt                                   */
/*                      ap_wait_start_rx_mgmt                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "frame.h"
#include "management_ap.h"
#include "pm_ap.h"
#include "receive.h"
#include "ap_prot_if.h"
#include "core_mode_if.h"
#include "iconfig.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_rx_mgmt                                       */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the ENABLED state.                        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The incoming frame type is checked and appropriate       */
/*                  processing is done based on mode of operation.           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_enabled_rx_mgmt(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_t   *wlan_rx  = (wlan_rx_t *)msg;
    UWORD8      *msa      = wlan_rx->msa;
    UWORD16     rx_len    = wlan_rx->rx_len;
	WORD8       rssi      = get_rx_dscr_rssi_db(wlan_rx->base_dscr);

	/* Validate the RSSI */
	if(rssi == INVALID_RSSI_DB)
		rssi = (WORD8)-100;

    if((mac_addr_cmp(wlan_rx->addr3, mget_bcst_addr()) == BTRUE) &&
       (wlan_rx->sub_type == PROBE_REQ))
    {
        /* Broadcast probe request frames should be handled. */
    }
    else if((mac_addr_cmp(wlan_rx->addr3, mget_bcst_addr()) == BTRUE) &&
    (mac_addr_cmp(wlan_rx->addr1, mget_StationID()) == BTRUE))
    {
        /* Directed public frames to be handled */
    }
    /* Non-directed OBSS frames */
    else if(mac_addr_cmp(wlan_rx->addr3, mget_StationID()) == BFALSE)
    {
	// 20120709 caisf add, merged ittiam mac v1.2 code
#if 0
		/* Process the OBSS beacon/probe response */
        if((wlan_rx->sub_type == BEACON) ||
           (wlan_rx->sub_type == PROBE_RSP))
        {
            process_obss_beacon(msa, rx_len, rssi);
        }

        /* This packet is not directed to the Access Point, return without   */
        /* processing anything.                                              */
#else
		switch(wlan_rx->sub_type)
		{
	        case BEACON:
	        {
	            process_obss_beacon(msa, rx_len, rssi);
	        }
			break;
			case PROBE_RSP:
			{
				 process_obss_beacon(msa, rx_len, rssi);

				 /* Do the protocol related processing of probe response frame */
				 handle_ap_enabled_probe_rsp_prot(msa, rx_len);
			}
			break;
			default:
			{
				handle_mgmt_prot_ap((UWORD8 *)wlan_rx, wlan_rx->sub_type, mac);
			}
	    } /* End of Switch */
#endif
        return;
    }

    switch(wlan_rx->sub_type)
    {
    case PROBE_REQ:
    {
        UWORD8       sa[6]     = {0};
        BOOL_T       is_p2p = BFALSE;
        asoc_entry_t *ae       = 0;

        get_SA(msa, sa);

        ae   = (asoc_entry_t*)find_entry(sa);
		// 20120709 caisf masked, merged ittiam mac v1.2 code
#if 0
        if(ae != 0)
        {
            check_ps_state(ae, (STA_PS_STATE_T)get_pwr_mgt(msa));
        }
#endif /* 0 */

        is_p2p = handle_prob_req_prot_ap(msa, rx_len, sa);
		
// 20120709 caisf add, merged ittiam mac v1.2 code
#if 0
#if 0 /* TBD - P2P_DEBUG */
        if(is_probe_rsp_to_be_sent(msa, rx_len, is_p2p) == BTRUE)
#else /* TBD - P2P_DEBUG */
        if(probe_req_ssid_cmp(msa, mget_DesiredSSID()) == BTRUE)
#endif /* TBD - P2P_DEBUG */
#else
        if(is_probe_rsp_to_be_sent(msa, rx_len, is_p2p) == BTRUE)
#endif
        {
            if(ae == NULL)
                wps_handle_probe_req(msa, sa, rx_len, PROBE_REQ);

            send_probe_rsp(msa, is_p2p);

        }
    }
    break;

    case DEAUTH:
    {
        /* On reception of this message (from other STA) the AP deletes the  */
        /* entry corresponding to that STA in the association table to       */
        /* indicate that the STA is no longer authenticated with this AP.    */
        UWORD8       sa[6] = {0};
        asoc_entry_t *ae   = 0;

        get_SA(msa, sa);
        ae = (asoc_entry_t*)find_entry(sa);

        if(ae != NULL)
        {
            /* Handle any Protocol related STA disconnection */
            handle_disconnect_req_prot_ap(ae, sa);

            /* Send the sta leaving information to host */
            send_join_leave_info_to_host(ae->asoc_id, sa, BFALSE);

            delete_entry(sa);

            PRINTD("Status1: Successfully Deauthenticated STA %x:%x:%x:%x:%x:%x\n\r",
            sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
        }
    }
    break;

    case DISASOC:
    {
        /* On reception of this message (from other STA) the AP changes the  */
        /* STA state to AUTH.                                                */
        asoc_entry_t *ae   = 0;
        UWORD8       sa[6] = {0};

        get_SA(msa, sa);

        ae = (asoc_entry_t*)find_entry(sa);
        if(ae == 0)
        {
            /* Send a De-authentication Frame to the station as DISASOC is a */
            /* Class 2 frame.                                                */
            send_deauth_frame(sa, (UWORD16)CLASS2_ERR);
        }
        else if (ae->state == ASOC)
        {
            /* Handle any Protocol related STA disconnection */
            handle_disconnect_req_prot_ap(ae, sa);

            reset_asoc_entry(ae);
            ae->state = AUTH_COMPLETE;

            /* Send the sta leaving information to host */
            send_join_leave_info_to_host(ae->asoc_id, sa, BFALSE);

            PRINTD("Status: Successfully Disassociated STA %x:%x:%x:%x:%x:%x\n\r",
            sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
        }
    }
    break;

    case AUTH:
    {
        asoc_entry_t *ae     = 0;
        UWORD8  *auth_rsp    = 0;
        UWORD16 auth_rsp_len = 0;
        UWORD8        sa[6]  = {0};

       /* Get the transmitting STA's address */
        get_SA(msa, sa);

		// 20120709 caisf add, merged ittiam mac v1.2 code
        /* Get the association entry */
        ae = (asoc_entry_t*)find_entry(sa);
		if(ae != NULL)
		{
			if (ae->state == ASOC)
		    {
			    /* Handle any Protocol related processing */
			    handle_disconnect_req_prot_ap(ae, sa);
		    }

	    }

        /* On reception of this message (from other STAs) AP prepares the    */
        /* authentication response frame.                                    */
        auth_rsp = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                      MANAGEMENT_FRAME_LEN);
        if(auth_rsp == NULL)
        {
			PRINTD2("No Mem for Auth Rsp\n");
#ifdef DEBUG_MODE
			g_mac_stats.no_mem_count++;
#endif			
            return;
        }

        auth_rsp_len = prepare_auth_rsp(auth_rsp, msa);

        /* MIB variables are updated as required if authentication fails.*/
        {
            UWORD16 status = 0;
            UWORD8 addr1[6] = {0};

            status = get_auth_status(auth_rsp);
                get_address1(auth_rsp, addr1);

            if(status != SUCCESSFUL_STATUSCODE)
            {
                mset_AuthenticateFailStatus(status);
                mset_AuthenticateFailStation(addr1);

                /* ITM_DEBUG */
                PRINTD("Status: Authentication of STA %x:%x:%x:%x:%x:%x Failed(%d)\n",
                addr1[0], addr1[1], addr1[2], addr1[3], addr1[4], addr1[5], status);
            }
            else
            {
#ifdef DEBUG_MODE
                /* ITM_DEBUG */
                UWORD8 auth_type    = 0;
                UWORD8 auth_seq_num = 0;

                auth_type    = get_auth_algo_num(auth_rsp);
                auth_seq_num = get_auth_seq_num(auth_rsp);

                if(((auth_type == OPEN_SYSTEM) && (auth_seq_num == 2)) ||
                   ((auth_type == SHARED_KEY) && (auth_seq_num == 4)))
                {
                PRINTD("Status: Successfully Authenticated STA %x:%x:%x:%x:%x:%x\n\r",
                addr1[0], addr1[1], addr1[2], addr1[3], addr1[4], addr1[5]);
                }
#endif /* DEBUG_MODE */
            }
        }

        /* The Authentication Response Frame is sent to the Transmit task    */
        /* for transmission.                                                 */
        tx_mgmt_frame(auth_rsp, auth_rsp_len, HIGH_PRI_Q, 0);
    }
    break;

    case ASSOC_REQ:
    case REASSOC_REQ:
    {
        /* Check if STA requesting association is authenticated */
        asoc_entry_t *asoc_entry  = 0;
        UWORD8       *data        = 0;
        UWORD16      rsp_len      = 0;
        UWORD8       sa[6]        = {0};
        BOOL_T       is_p2p       = BFALSE;

       /* Get the transmitting STA's address */
        get_SA(msa, sa);

        asoc_entry = (asoc_entry_t*)find_entry(sa);

        if(asoc_entry == 0)
        {
            send_deauth_frame(sa, (UWORD16)ASOC_NOT_AUTH);
            return;
        }
        else
        {
            UWORD16  status = 0;

            if(asoc_entry->state == ASOC)
            {
                /* Check if the new association message is OK */
                status = update_asoc_entry(asoc_entry, msa, rx_len, &is_p2p);
                if(status != SUCCESSFUL_STATUSCODE)
                    asoc_entry->state = AUTH_COMPLETE;
            }
            else if(asoc_entry->state == AUTH_COMPLETE)
            {
                status = update_asoc_entry(asoc_entry, msa, rx_len, &is_p2p);
            }
            else
            {
                status = UNSPEC_FAIL;
            }

            if(status == AP_FULL)
            {
                /* Handle any Protocol related STA disconnection */
                handle_disconnect_req_prot_ap(asoc_entry, sa);

                /* Association table full. Delete the entry. Give unused     */
                /* association Id.                                           */
                delete_entry(sa);
	       PRINTD("Status2(%x): Association of STA %x:%x:%x:%x:%x:%x Failed. \n\r",
                status, sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
            }

            /* Initialize the security parameters for the successful */
            /* association */
            if(status == SUCCESSFUL_STATUSCODE)
            {
                status = init_sec_entry_ap(asoc_entry, sa, msa, rx_len);
            }

            /* The AP sends an association response frame if the sending     */
            /* station is authenticated.                                     */
            data = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle,
                                       MANAGEMENT_FRAME_LEN);
            if(data == NULL)
            {
#ifdef DEBUG_MODE
				g_mac_stats.no_mem_count++;
#endif
                return;
            }

            if(wlan_rx->sub_type == ASSOC_REQ)
            {
                rsp_len = prepare_asoc_rsp(data, msa,
                    asoc_entry->asoc_id, status, ASSOC_RSP, rx_len,
                    asoc_entry->ht_hdl.ht_capable, is_p2p);
            }
            else
            {
                rsp_len = prepare_asoc_rsp(data, msa,
                    asoc_entry->asoc_id, status, REASSOC_RSP, rx_len,
                    asoc_entry->ht_hdl.ht_capable, is_p2p);
            }

            /* The Association Response Frame is sent to the Transmit task   */
            /* for transmission.                                             */
            tx_mgmt_frame(data, rsp_len, HIGH_PRI_Q, 0);

            if(status != SUCCESSFUL_STATUSCODE)
            {
                /* Handle any Protocol related STA disconnection */
                 handle_disconnect_req_prot_ap(asoc_entry, sa);

                delete_entry(sa);

                /* ITM_DEBUG */
                PRINTD("Status3(%x): Association of STA %x:%x:%x:%x:%x:%x Failed\n\r",
                status, sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
            }
            else
            {
                /* Further station entry specific processing is done after */
                /* the Asoc frame is successfully transmitted.             */
                PRINTD("Status(%x): Association of STA %x:%x:%x:%x:%x:%x succeed\n\r",
                status, sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
            }
        }
    }
    break;
	// 20120709 caisf add, merged ittiam mac v1.2 code
    case PROBE_RSP:
	{
		 handle_ap_enabled_probe_rsp_prot(msa, rx_len);
	}
	break;

    default:
    {
        handle_mgmt_prot_ap((UWORD8 *)wlan_rx, wlan_rx->sub_type, mac);
    }
    break;

    } /* end of switch(get_sub_type(msa)) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_wait_start_rx_mgmt                                    */
/*                                                                           */
/*  Description   : This function handles the incoming management frame as   */
/*                  appropriate in the WAIT_START state.                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The AP is in scanning mode while in WAIT_START state.    */
/*                  Hence the relevant information is extracted from the     */
/*                  received Management frames in this function.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_wait_start_rx_mgmt(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_t   *wlan_rx  = (wlan_rx_t *)msg;
    UWORD8      *msa      = wlan_rx->msa;
    UWORD16     rx_len    = wlan_rx->rx_len;

	/* Store the network BSSID of any frame that is received */
	store_network(get_BSSID_ptr(msa));

    switch(wlan_rx->sub_type)
    {
		/* Process beacons as well as, non-directed probe responses */
		case BEACON:
		case PROBE_RSP:
		{
	        UWORD8  freq       = get_current_start_freq();
			UWORD32 *rx_dscr   = wlan_rx->base_dscr;
			WORD8   rssi       = get_rx_dscr_rssi_db(rx_dscr);
			UWORD8  chan_index = get_ch_idx_from_tbl_idx(freq,g_ap_ch_tbl_index);
	        UWORD8  tbl_idx    = INVALID_CH_TBL_INDEX;
	        UWORD16 index      = TAG_PARAM_OFFSET;
	        UWORD8  ch_offset  = SCN;

			/* Validate the RSSI */
			if(rssi == INVALID_RSSI_DB)
				rssi = (WORD8)-100;

			/* Get the channel index from beacon frame */
			get_chan_idx_of_ntwk(msa,index,rx_len,&chan_index,&ch_offset);

            /* Get the table index from the channel index */
	        tbl_idx = get_tbl_idx_from_ch_idx(freq, chan_index);

            /* Update the parameters */
	        if(is_ch_tbl_idx_valid(freq, tbl_idx) == BTRUE)
	        {
	            update_chan_info_prot(msa, rx_len, rssi, tbl_idx, ch_offset);
	        }
		}
		break;
	}
}

#endif /* BSS_ACCESS_POINT_MODE */
