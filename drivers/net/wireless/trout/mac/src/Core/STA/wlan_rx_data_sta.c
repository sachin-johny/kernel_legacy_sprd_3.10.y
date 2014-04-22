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
/*  File Name         : wlan_rx_data_sta.c                                   */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the STA MAC FSM on receiving     */
/*                      WLAN_RX (MSDU) events.                               */
/*                                                                           */
/*  List of Functions : sta_enabled_rx_data                                  */
/*                      msdu_indicate_sta                                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "prot_if.h"
#include "host_if.h"
#include "receive.h"
#include "wep.h"
#include "sta_prot_if.h"
#include "pm_sta.h"
#include "iconfig.h"
#include "trout_wifi_rx.h"
#include "autorate.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void msdu_indicate_sta(msdu_indicate_t *msdu);


#ifdef TROUT_WIFI_NPI
UWORD8 get_rx_rate(UWORD8 phy_rate)
{
    UWORD8 user_data = 0;

    switch(phy_rate)
    {

        case 0: /* Reserved */
            user_data = 0;
            break;
        case 1: /* 2Mbps Short */
            user_data = 2;
            break;
        case 2: /* 5.5Mbps Short */
            user_data = 5;
            break;
        case 3: /* 11Mbps Short */
            user_data = 11;
            break;
        case 4: /* 1Mbps Long */
            user_data = 1;
            break;
        case 5: /* 2Mbps Long */
            user_data = 2;
            break;
        case 6: /* 5.5Mbps Long */
            user_data = 5;
            break;
        case 7: /* 11Mbps Long */
            user_data = 11;
            break;
       case 8: /* 48Mbps */
            user_data = 48;
            break;
        case 9: /* 24Mbps */
            user_data = 24;
            break;
        case 10: /* 12Mbps */
            user_data = 12;
            break;
        case 11: /* 6Mbps */
            user_data = 6;
            break;
        case 12: /* 54Mbps */
            user_data = 54;
            break;
        case 13: /* 36Mbps */
            user_data = 36;
            break;
        case 14: /* 18Mbps */
            user_data = 18;
            break;
        case 15: /* 9Mbps */
            user_data = 9;
            break;

        default:
            user_data = phy_rate;
            break;
    }

    return user_data;
}

UWORD8 g_ap_last_rate = 0;
#endif
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY

#define HS_WAKE_INTERVAL      (2000)

void hs_wake_timer_fn(UWORD32 data)
{
    if(wake_lock_active(&handshake_frame_lock))
    {
        pr_info("%s-%d: release wake_lock %s\n", __func__, __LINE__, 
            handshake_frame_lock.name);
        wake_unlock(&handshake_frame_lock);
    }
    else
        pr_info("%s-%d: Warning: try unlock wake_lock %s failed!\n", __func__, __LINE__, 
            handshake_frame_lock.name);
}

static void start_hs_wake_timer(void)
{
    if(g_hs_wake_timer == 0)
    {
        g_hs_wake_timer = create_alarm(hs_wake_timer_fn, 0, NULL);
    }

    start_alarm(g_hs_wake_timer, HS_WAKE_INTERVAL);
}

static void stop_hs_wake_timer(void)
{
    stop_alarm(g_hs_wake_timer);
}

void del_hs_wake_timer(void)
{
    if(g_hs_wake_timer != 0)
    {
        stop_hs_wake_timer();
        delete_alarm(&g_hs_wake_timer);
        g_hs_wake_timer = 0;
    }
}

#endif
#endif
/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_enabled_rx_data                                      */
/*                                                                           */
/*  Description   : This function handles the incoming DATA frame to send to */
/*                  the host.                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to WLAN receive structure                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the MSDU Indicate function with the  */
/*                  required structure as input.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8  g_ap_last_rate_integer    = 0;
UWORD8  g_ap_last_rate_decimal    = 0;
WORD32  g_total_rssi_value = 0;
WORD32  g_total_rssi_cnt_num   = 0;

void rssi_value_add(void)
{
	g_total_rssi_value += -(0xFF - get_rssi());
	g_total_rssi_cnt_num ++;

	//printk("rssi_value_add %d\n",g_total_rssi_value);
	return;
}

WORD32 get_avg_rssi(void)
{
	WORD32 tmp_avg_rssi = 0;
	
	if(g_total_rssi_cnt_num != 0)
	{
		tmp_avg_rssi = g_total_rssi_value / g_total_rssi_cnt_num;
	}	
	else
		return 277;

	g_total_rssi_value = 0;
	g_total_rssi_cnt_num   = 0;

	//printk("get_avg_rssi %d\n",tmp_avg_rssi);
	return tmp_avg_rssi;
}

typedef struct
{
    UWORD8       rate_index;
    UWORD8       rate;
	UWORD8       point5;
} rate_table_t;

#ifdef NMAC_1X1_MODE
    rate_table_t rate_table_chenq[20] = {
	{0x00,1,0},//"1"
	{0x01,2,0},//"2"
	{0x02,5,1},//"5.5"
	{0x0B,6,0},//"6"
	{0x80,6,1},//"6.5"
	{0x0F,9,0},//"9"
	{0x03,11,0},//"11"
	{0x0A,12,0},//"12"
	{0x81,13,0},//"13"
	{0x0E,18,0},//"18"
	{0x82,19,1},//"19.5"
	{0x09,24,0},//"24"
	{0x83,26,0},//"26"
	{0x0D,36,0},//"36"
	{0x84,39,0},//"39"
	{0x08,48,0},//"48"
	{0x85,52,0},//"52"
	{0x0C,54,0},//"54"
	{0x86,58,1},//"58.5"
	{0x87,65,0}//"65"
	};
#else /* NMAC_1X1_MODE */
    rate_table_t rate_table_chenq[28] = {
	{0x00,1,0},//"1"
	{0x01,2,0},//"2"
	{0x02,5,1},//"5.5"
	{0x0B,6,0},//"6"
	{0x80,6,1},//"6.5"
	{0x0F,9,0},//"9"
	{0x03,11,0},//"11"
	{0x0A,12,0},//"12"
	{0x81,13,0},//"13"
	{0x88,13,0},//"13"
	{0x0E,18,0},//"18"
	{0x82,19,1},//"19.5"
	{0x09,24,0},//"24"
	{0x83,26,0},//"26"
	{0x89,26,0},//"26"
	{0x0D,36,0},//"36"
	{0x84,39,0},//"39"
	{0x8A,39,0},//"39"
	{0x08,48,0},//"48"
	{0x85,52,0},//"52"
	{0x8B,52,0},//"52"
	{0x0C,54,0},//"54"
	{0x86,58,1},//"58.5"
	{0x87,65,0},//"65"
	{0x8C,78,0},//"78"
    {0x8D,104,0},//"104"
    {0x8E,117,0},//"117"
    {0x8F,130,0}//"130"
	};
#endif /* NMAC_1X1_MODE */

void sta_enabled_rx_data(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_t          *wlan_rx    = (wlan_rx_t *)msg;
    UWORD8             priority_rx = wlan_rx->priority_val;
    UWORD8             *msa        = wlan_rx->msa;
    sta_entry_t        *se         = (sta_entry_t *)wlan_rx->sa_entry;
    msdu_desc_t        *frame_desc = 0;
    msdu_indicate_t    msdu        = {{0},};
    msdu_proc_state_t  msdu_state  = {0};
    MSDU_PROC_STATUS_T status      = PROC_ERROR;
    BOOL_T             sta_ps_check_done = BFALSE;

	UWORD32 phy_rate = 0;
	phy_rate = get_rx_dscr_data_rate(wlan_rx->base_dscr);

#if 0
	if(IS_RATE_MCS(phy_rate) == BFALSE)
	{
		g_ap_last_rate_integer = phy_rate;
		g_ap_last_rate_decimal = 0;
	}
	else
#endif		
	{
		int j=0;
		for( j = 0; j < sizeof(rate_table_chenq)/sizeof(rate_table_t); j++ )
		{
			if(rate_table_chenq[j].rate_index == phy_rate)
			{
				g_ap_last_rate_integer = rate_table_chenq[j].rate;
				if(rate_table_chenq[j].point5)
				{
					g_ap_last_rate_decimal = 1;
				}
				else
				{
					g_ap_last_rate_decimal = 0;
				}
				break;
			}
		}

		if( j >= sizeof(rate_table_chenq)/sizeof(rate_table_t) )
		{
			g_ap_last_rate_integer = 0;
			g_ap_last_rate_decimal = 0;
		}
	}
    //printk("chenq: rx rate is: index %02x,%d.%d Mb/s\n",phy_rate, g_ap_last_rate_integer,(g_ap_last_rate_decimal ==0) ? 0 : 5);
	TROUT_FUNC_ENTER;
#ifdef  TROUT_WIFI_NPI
	//UWORD32 data_rate = 0;
    //UWORD8 user_rate = 0;
    UWORD32 rx_len = 0;
    //UWORD32 phy_rate = 0;
    print_log_debug_level_1("\n[DL1][INFO][Rx] {MAC API for WLAN Rx Data}");

    rx_len = get_rx_dscr_frame_len(wlan_rx->base_dscr);
    //printk("rx len is: %u\n", rx_len);
    phy_rate = get_rx_dscr_data_rate(wlan_rx->base_dscr);
   // printk("phy rate is: 0x%x\n", phy_rate);
    g_ap_last_rate = get_rx_rate(phy_rate);
    printk("npi: rx rate is: %u Mb/s\n", g_ap_last_rate);
#endif

#ifdef WAKE_LOW_POWER_POLICY
#if 1
	if(!wlan_rx->is_grp_addr)
		g_low_power_flow_ctrl.rx_pkt_num += wlan_rx->num_dscr;
#endif
#endif

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
    if(NULL_FRAME == wlan_rx->sub_type)
    {
        return;
    }
#else
    /* No NULL frames should be sent to the host */
    if((NULL_FRAME == wlan_rx->sub_type) ||
       (QOS_NULL_FRAME == wlan_rx->sub_type))
    {
        /* Handle any power save related functionality for NULL frame        */
        /* reception                                                         */
        psm_handle_rx_packet_sta(msa, priority_rx);
        TROUT_FUNC_EXIT;
        return;
    }
#endif

    /* make sure we set all filed 0 by zhao */
    memset((void *)&msdu, 0, sizeof(msdu_indicate_t));
    memset((void *)&msdu_state, 0, sizeof(msdu_proc_state_t));
    /* Update the MSDU priority value */
    msdu.priority = priority_rx;

    /* Create the MSDU descriptors for the received frame */
    do
    {
        /* Create the MSDU descriptor */
        status = update_msdu_info(wlan_rx, &msdu, &msdu_state);

        if(PROC_ERROR == status)
            break;

        /* Get the frame descriptor pointer */
        frame_desc = &(msdu.frame_desc);

        /* Before forwarding the packet across to the HOST interface security    */
        /* checks needs to performed on the states of the transmitting station   */
        if(sta_check_sec_tx_sta_state(se, frame_desc,
                                      (CIPHER_T)(wlan_rx->ct)) != BTRUE)
        {
			TROUT_FUNC_EXIT;
            return;
        }

        /* Check for SNAP header at the beginning of the data and set the    */
        /* data pointer and length accordingly.                              */
        if(BTRUE == is_snap_header_present(frame_desc))
        {
            /* If the message is a unicast packet, search for the STA entry */
            if(is_group(msdu.da) == BFALSE)
            {
                /* If the received packet is a security handshake packet     */
                /* process it in the security layer                          */
                if(is_sec_handshake_pkt_sta(se, frame_desc->buffer_addr,
                                            frame_desc->data_offset,
                                            frame_desc->data_len,
                                            (CIPHER_T)(wlan_rx->ct)) == BTRUE)
                {
                    continue;
                }

#ifdef WIFI_SLEEP_POLICY  
                if(check_11i_frame(frame_desc->buffer_addr + frame_desc->data_offset) ||  
                   check_wapi_frame(frame_desc->buffer_addr + frame_desc->data_offset))  
                {  
#if 0
                    if(wake_lock_active(&handshake_frame_lock))  
                    {  
                         pr_info("%s-%d: release wake_lock %s\n", __func__, __LINE__, handshake_frame_lock.name);  
                         wake_unlock(&handshake_frame_lock);  
                    }  
                 
                    wake_lock_timeout(&handshake_frame_lock,msecs_to_jiffies(2000)); /*Keep 2s awake when HK, by caisf 20131004*/  
                    pr_info("%s-%d: acquire wake_lock %s\n", __func__, __LINE__, handshake_frame_lock.name);  
#else
                    if(wake_lock_active(&handshake_frame_lock))
                    {
					    stop_hs_wake_timer();
		                /*Keep 2s awake when HK, by caisf 20131004*/
						start_hs_wake_timer();
						pr_info("%s-%d: refresh timer wake_lock %s\n", __func__,
					                            __LINE__, handshake_frame_lock.name);
                    }
                    else
                    {
						wake_lock(&handshake_frame_lock);
                        /*Keep 2s awake when HK, by caisf 20131004*/
                        start_hs_wake_timer();
						pr_info("%s-%d: acquire wake_lock %s\n", __func__,
                            __LINE__, handshake_frame_lock.name);
                    }
#endif
                } 
#endif  
            }

            /* Adjust the frame to account for the SNAP header */
            adjust_for_snap_header(frame_desc);
        }

        /* Reset the link loss count if a valid frame is received */
        g_link_loss_count = 0;
        //printk("rx_data: link_lost = %d\n", g_link_loss_count);
        update_connection_cnt(se);

		/* Handle packet reception for power save */
		if(sta_ps_check_done == BFALSE)
		{
			sta_ps_check_done = BTRUE;
		    psm_handle_rx_packet_sta(msa, priority_rx);
		}

		//chenq add for wapi 2012-09-29
		#ifdef MAC_WAPI_SUPP
		if( wlan_rx->wapi_mode == 1 )
		{
			msdu.frame_desc.wapi_mode = 1;
		}
		else
		{
			msdu.frame_desc.wapi_mode = 0;
		}
		#endif
		//chenq add for auto set tx rx power 2013-07-29
#ifdef TROUT2_WIFI_IC
		/* Update the RSSI information */
		if(msdu_state.curr_dscr == 0)
		{
			update_rssi(wlan_rx->base_dscr);
		}
		else
        	{
            		update_rssi(msdu_state.curr_dscr);
		}
		
		uptate_rssi4_auto_set_tx_rx_power();
#ifdef AUTORATE_PING
		//ping.jiang add for calculating statistics 2013-10-31
		ar_rssi_value_add();
#endif /* AUTORATE_PING */
#endif
		//chenq add end
		rssi_value_add();  // for set_aci
        /* Call MSDU Indicate API with the MSDU to be sent to the host */
        msdu_indicate_sta(&msdu);

    } while(PROC_MORE_MSDU == status);

    /* Handle packet reception for power save */
	if(sta_ps_check_done == BFALSE)
	{
		sta_ps_check_done = BTRUE;
        psm_handle_rx_packet_sta(msa, priority_rx);
    }
    
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : msdu_indicate_sta                                        */
/*                                                                           */
/*  Description   : This function sends the received MSDU to the host.       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MSDU Indicate message                  */
/*                                                                           */
/*  Globals       : g_ethernet_addr                                          */
/*                                                                           */
/*  Processing    : This function creates and sends the frame to the host    */
/*                  based on the host type.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void msdu_indicate_sta(msdu_indicate_t *msdu)
{
    UWORD8      host_type   = get_host_data_if_type();
    msdu_desc_t *frame_desc = &(msdu->frame_desc);

	TROUT_FUNC_ENTER;
    /* Add one user to the buffer being sent to the host. This ensures that  */
    /* the buffer will remain allocated even after the process rx frame      */
    /* function frees it.                                                    */
    //mem_add_users(g_shared_pkt_mem_handle, frame_desc->buffer_addr, 1);
    
    /* Prepare required frame for the current configured data host interface */
    prepare_host_data_frame(msdu, host_type);

    /* Send the data frame to the required host */
    send_data_frame_to_host_if(frame_desc, host_type);

    print_log_debug_level_1("\n[DL1][INFO][Rx] {MSDU Indicate API}");
    TROUT_FUNC_EXIT;
}

#endif /* IBSS_BSS_STATION_MODE */
