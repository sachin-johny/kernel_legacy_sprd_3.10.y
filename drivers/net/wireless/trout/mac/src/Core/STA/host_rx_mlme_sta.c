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
/*  File Name         : host_rx_mlme_sta.c                                   */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the STA MAC FSM on receiving     */
/*                      HOST_RX (MLME) events.                               */
/*                                                                           */
/*  List of Functions : sta_wait_scan                                        */
/*                      sta_wait_join                                        */
/*                      sta_wait_auth                                        */
/*                      sta_wait_asoc                                        */
/*                      sta_wait_start                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals_sta.h"
#include "frame_sta.h"
#include "fsm_sta.h"
#include "mh.h"
#include "sta_prot_if.h"
#include "qmu_tx.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_scan                                            */
/*                                                                           */
/*  Description   : This function calls a MAC core function to initiate the  */
/*                  scan process.                                            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : g_channel_list                                           */
/*                  g_channel_index                                          */
/*                                                                           */
/*  Processing    : The appropriate MAC core function is called with the     */
/*                  input message and the MAC structure as the input request */
/*                  parameters. State is changed to WAIT_SCAN.               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

extern int itm_scan_flag;

void sta_wait_scan(mac_struct_t *mac, UWORD8 *msg)
{
    scan_req_t *scan_req     = 0;
    UWORD8      temp_addr[6] = {0};

	TROUT_FUNC_ENTER;
	coex_state_switch(COEX_WIFI_ON_SCANNING);
    /* The incoming message is a Scan Request message and is cast to the     */
    /* scan request structure.                                               */
    scan_req = (scan_req_t *)msg;

    /* Change state of the MAC structure to WAIT_SCAN */
    set_mac_state(WAIT_SCAN);

	//chenq move to here 2012-12-22
	/* Update the MIB and globals with the Scan request parameters */
    update_scan_req_params(scan_req);

	//printk("chenq debug g_scan_source = %02x\n",g_scan_source);
	if( g_scan_source == USER_SCAN )
	{
		TROUT_DBG4("%s: set user scan mode!\n", __func__);	
	    /* Start scanning from first channel in the scan request channel list */
	    scan_channel(g_channel_list[g_channel_index]);
	    g_channel_index++;
	}
	else if((g_scan_source == DEFAULT_SCAN) &&  ( strlen(mget_DesiredSSID()) != 0 ))
	{
		TROUT_DBG4("%s: set default scan mode!\n", __func__);
	    /* Start scanning from first channel in the scan request channel list */
	    scan_channel(g_channel_list[g_channel_index]);
	    g_channel_index++;
	}
	else
	{
		/* Change state of the MAC structure to WAIT_SCAN */
		
    	//zhangzhao add 2013-01-12
		set_mac_state(WAIT_SCAN);
		init_scan_limit();
		
		TROUT_DBG4("%s: chenq debug not scan\n", __func__);
	}

	//chenq move to upper
    /* Update the MIB and globals with the Scan request parameters */
    //update_scan_req_params(scan_req);

	//chenq add for use itm in ui 2012-10-19
	//*if user scan ok
	//*if default scan and essid already set ok
	//other no ...
	#if 0
	if( (g_scan_source == USER_SCAN) || 
		 ((g_scan_source == DEFAULT_SCAN) &&  ( strlen(mget_DesiredSSID()) != 0 )) )
	{
		itm_scan_flag = 1;
	    /* Start scanning from first channel in the scan request channel list */
	    scan_channel(g_channel_list[g_channel_index]);
	    g_channel_index++;
	}
	#endif
	
    /* Make sure beacons come for detection of networks */
    disable_machw_beacon_filter();
    disable_machw_obss_mgmt_filter();
#ifndef NON_FC_MACHW_SUPPORT
    disable_machw_non_directed_mgmt_filter();
#endif /* NON_FC_MACHW_SUPPORT */

    /* Disable deauth filter and interrupt. Enable both filter and interrupt */
    /* after successful association                                          */
    disable_machw_deauth_filter();
    mask_machw_deauth_int();

    /* if it is not a user-initiated scan             */
    if(g_scan_source == DEFAULT_SCAN)
    {
        set_machw_bssid(temp_addr);
        mset_bssid(temp_addr);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_join                                            */
/*                                                                           */
/*  Description   : This function calls a MAC core function to initiate the  */
/*                  join process.                                            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The appropriate MAC core funtcion is called with the     */
/*                  input message and the MAC structure as the input request */
/*                  parameters. State is changed to WAIT_JOIN.               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_join(mac_struct_t *mac, UWORD8 *msg)
{
    join_req_t *join_req = 0;
    UWORD32     *trout_beacon_buf = 0, beacon_len = 0;

	TROUT_FUNC_ENTER;
    coex_state_switch(COEX_WIFI_ON_CONNECTING);
    /* Call a function to handle the MLME Join Request based on the protocol */
    /* in use. If handled, return. No further processing is required.        */
    if(BTRUE == sta_handle_join_req_prot(mac, msg))
    {
        /* Enable the hardware filters */
#ifndef NON_FC_MACHW_SUPPORT
		enable_machw_non_directed_mgmt_filter();
#endif /* NON_FC_MACHW_SUPPORT */
        enable_machw_obss_mgmt_filter();
        TROUT_FUNC_EXIT;
        return;
    }

    /* The incoming message is a Join Request message and is cast to the     */
    /* join request structure.                                               */
    join_req = (join_req_t *)msg;

    /* Update the MIB and globals with the Join request parameters */
    update_join_req_params(join_req);

    /* If the BSS Type is INDEPENDENT prepare a beacon frame and add it to   */
    /* the H/w queue with required transmit descriptor.                      */
    if(mget_DesiredBSSType() == INDEPENDENT)
    {
        UWORD8  *beacon_frame = 0;

        /* Allocate 2 global beacon buffers as the beacon for AP may be      */
        /* modified for which 2 buffers are needed.                          */
        g_beacon_frame[0] = (UWORD8 *)pkt_mem_alloc(MEM_PRI_HPTX);
        if(g_beacon_frame[0] == NULL)
        {
            /* Exception - no buffers for beacons */
            /* Error condition: Reset the whole MAC */
            raise_system_error(NO_SHRED_MEM);
            TROUT_FUNC_EXIT;
            return;
        }

        g_beacon_frame[1] = (UWORD8 *)pkt_mem_alloc(MEM_PRI_HPTX);
        if(g_beacon_frame[1] == NULL)
        {
            /* Exception - no buffers for beacons */
            pkt_mem_free(g_beacon_frame[0]);
            /* Error condition: Reset the whole MAC */
            raise_system_error(NO_SHRED_MEM);
            TROUT_FUNC_EXIT;
            return;
        }

        /* Prepare beacon in the first buffer */
        beacon_frame = g_beacon_frame[0] + SPI_SDIO_WRITE_RAM_CMD_WIDTH;
        g_beacon_len = prepare_beacon_sta(beacon_frame);

        /* Copy it to other buffer as well */
        memcpy(g_beacon_frame[1], g_beacon_frame[0], g_beacon_len);

        /* This variable holds the index of the beacon which can be updated by */
        /* MAC S/w. The other copy of the beacon in given to H/w. The beacons  */
        /* are swapped on every TBTT.                                          */
        g_beacon_index = 1;

    	//add by chengwg.
    	beacon_len = g_beacon_len;
    	if(g_beacon_len % 4 != 0)
    		beacon_len += 4 - (g_beacon_len % 4);
    	if(beacon_len > BEACON_MEM_SIZE)
    	{
			printk("%s: beacon mem is too small(%d : %d)!\n", __func__, beacon_len, BEACON_MEM_SIZE);
			return;
    	}
    	trout_beacon_buf = (UWORD32 *)BEACON_MEM_BEGIN;
    	host_write_trout_ram((void *)trout_beacon_buf, g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH, beacon_len);

        set_machw_beacon_pointer((UWORD32)trout_beacon_buf);
        set_machw_beacon_tx_params(g_beacon_len,
                                   get_phy_rate(get_beacon_tx_rate()));
        set_machw_beacon_ptm(get_reg_phy_tx_mode(get_beacon_tx_rate(), 1));
    }
    else
    {
       /* Other BSS frames are not needed in BSS STA mode */
       enable_machw_obss_mgmt_filter();
    }

    /* Start the join timeout timer and change the state of the MAC          */
    /* structure to WAIT_JOIN                                                */
    set_mac_state(WAIT_JOIN);

    start_mgmt_timeout_timer(join_req->join_timeout);

    disable_machw_beacon_filter();
#ifndef NON_FC_MACHW_SUPPORT
    enable_machw_non_directed_mgmt_filter();
#endif /* NON_FC_MACHW_SUPPORT */

#ifdef DV_SIM
    /* GENERATE TBTT ENVENT TO GO TO "JOIN COMMPLETE" STATE */
    if(1)
    {
        misc_event_msg_t *misc = 0;

        /* Create a MISC_TBTT event with no message and post it to the event     */
        /* queue.                                                                */
        misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

        if(misc == NULL)
        {
#ifdef DEBUG_MODE
            g_mac_stats.emiscexc++;
#endif /* DEBUG_MODE */

            return;
        }

        misc->data = 0;
        misc->name = MISC_TBTT;
        post_event((UWORD8*)misc, MISC_EVENT_QID);
    }
#endif
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_auth                                            */
/*                                                                           */
/*  Description   : This function calls a MAC core function to initiate the  */
/*                  authentication process.                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The appropriate MAC core funtcion is called with the     */
/*                  input message and the MAC structure as the input request */
/*                  parameters. State is changed to WAIT_AUTH_SEQ2.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_auth(mac_struct_t *mac, UWORD8 *msg)
{
    auth_req_t *auth_req      = 0;
    UWORD8     *auth_frame    = 0;
    UWORD16    auth_frame_len = 0;

	TROUT_FUNC_ENTER;
    coex_state_switch(COEX_WIFI_ON_CONNECTING);
    auth_frame = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,
                                     MANAGEMENT_FRAME_LEN);
    if(auth_frame == NULL)
    {
        /* Error condition: Reset the whole MAC */
        raise_system_error(NO_SHRED_MEM);
        TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif        
        return;
    }

    /* The incoming message is a Auth Request message and is cast to the     */
    /* auth request structure.                                               */
    auth_req = (auth_req_t *)msg;

    /* Update the MIB and globals with the Auth request parameters */
    update_auth_req_params(auth_req);

    /* Prepare Authentication Request frame and add it to the H/w  queue     */
    /* with required transmit descriptor.                                    */
    if((auth_frame_len = prepare_auth_req(auth_frame)) == 0)
    {
        pkt_mem_free(auth_frame);
        set_mac_state(DISABLED);
    }
    else
    {
        /* Transmit the management frame */
        tx_mgmt_frame(auth_frame, auth_frame_len, HIGH_PRI_Q, 0);

        /* Change state to WAIT_AUTH_SEQ2 and start the auth timeout timer */
        set_mac_state(WAIT_AUTH_SEQ2);
        start_mgmt_timeout_timer(auth_req->auth_timeout);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_asoc                                            */
/*                                                                           */
/*  Description   : This function calls a MAC core function to initiate the  */
/*                  association process.                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The appropriate MAC core funtcion is called with the     */
/*                  input message and the MAC structure as the input request */
/*                  parameters. State is changed to WAIT_ASOC.               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_asoc(mac_struct_t *mac, UWORD8 *msg)
{
    asoc_req_t *asoc_req      = 0;
    UWORD8     *asoc_frame    = 0;
    UWORD16    asoc_frame_len = 0;

	TROUT_FUNC_ENTER;
#ifdef WSC_IE_ASSOC_REQ_EN
    asoc_frame = (UWORD8*)pkt_mem_alloc(MEM_PRI_HPTX);
#else /* WSC_IE_ASSOC_REQ_EN */
    asoc_frame = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle,
                                     MANAGEMENT_FRAME_LEN);
#endif /* WSC_IE_ASSOC_REQ_EN */

    if(asoc_frame == NULL)
    {
        /* Error condition: Reset the whole MAC */
        raise_system_error(NO_SHRED_MEM);
        TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif        
        return;
    }

    /* The incoming message is a Asoc Request message and is cast to the     */
    /* asoc request structure.                                               */
    asoc_req = (asoc_req_t *)msg;

    /* Update the MIB and globals with the Asoc request parameters */
    update_asoc_req_params(asoc_req);

    /* Prepare Association Request frame and add it to the H/w  queue with   */
    /* required transmit descriptor.                                         */
    asoc_frame_len = prepare_asoc_req(asoc_frame);

    /* Transmit the management frame */
    tx_mgmt_frame(asoc_frame, asoc_frame_len, HIGH_PRI_Q, 0);

    /* Change state to WAIT_ASOC and start the asoc timeout timer */
    set_mac_state(WAIT_ASOC);
    start_mgmt_timeout_timer(asoc_req->assoc_timeout);
    
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : sta_wait_start                                           */
/*                                                                           */
/*  Description   : This function calls a MAC core function to initiate the  */
/*                  start process.                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The appropriate MAC core function is called with the     */
/*                  input message and the MAC structure as the input request */
/*                  parameters. State is changed to WAIT_START.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void sta_wait_start(mac_struct_t *mac, UWORD8 *msg)
{
    start_req_t *start_req    = 0;
    UWORD8      *beacon_frame = 0;
    UWORD8      bssid[6]      = {0};
    UWORD32 *trout_beacon_buf = 0, beacon_len = 0;

    /* The incoming message is a Start Request message and is cast to the    */
    /* start request structure.                                              */
    start_req = (start_req_t *)msg;

    /* Update the MIB and globals with the Start request parameters */
    update_start_req_params(start_req);

    /* Generate 46 bit random BSSID */
    generate_bssid(bssid);
    mset_bssid(bssid);

    /* Allocate 2 global beacon buffers as the beacon for AP may be modfied  */
    /* for which 2 buffers are needed.                                       */
    g_beacon_frame[0] = (UWORD8 *)(UWORD8 *)pkt_mem_alloc(MEM_PRI_HPTX);
    if(g_beacon_frame[0] == NULL)
    {
        /* Exception - no buffers for beacons */
        /* Error condition: Reset the whole MAC */
        raise_system_error(NO_SHRED_MEM);
        return;
    }
    g_beacon_frame[1] = (UWORD8 *)pkt_mem_alloc(MEM_PRI_HPTX);
    if(g_beacon_frame[1] == NULL)
    {
        /* Exception - no buffers for beacons */
        pkt_mem_free(g_beacon_frame[0]);
        /* Error condition: Reset the whole MAC */
        raise_system_error(NO_SHRED_MEM);
        return;
    }

    /* Prepare beacon in the first buffer */
    beacon_frame = g_beacon_frame[0] + SPI_SDIO_WRITE_RAM_CMD_WIDTH;
    g_beacon_len = prepare_beacon_sta(beacon_frame);

    /* Copy it to other buffer as well */
    memcpy(g_beacon_frame[1], g_beacon_frame[0], g_beacon_len);

    /* This variable holds the index of the beacon which can be updated by */
    /* MAC S/w. The other copy of the beacon in given to H/w. The beacons  */
    /* are swapped on every TBTT.                                          */
    g_beacon_index = 1;
    
	//add by chengwg.
	beacon_len = g_beacon_len;
	if(g_beacon_len % 4 != 0)
		beacon_len += 4 - (g_beacon_len % 4);
	if(beacon_len > BEACON_MEM_SIZE)
	{
		printk("%s: beacon mem is too small(%d : %d)!\n", __func__, beacon_len, BEACON_MEM_SIZE);
		return;
	}
	trout_beacon_buf = (UWORD32 *)BEACON_MEM_BEGIN;
	host_write_trout_ram((void *)trout_beacon_buf, g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH, beacon_len);

    set_machw_beacon_pointer((UWORD32)trout_beacon_buf);
    set_machw_beacon_tx_params(g_beacon_len,
                               get_phy_rate(get_beacon_tx_rate()));
    set_machw_beacon_ptm(get_reg_phy_tx_mode(get_beacon_tx_rate(), 1));

    /* Write required Hardware registers (BSSID, enable synchronization) and */
    /* wait for TBTT                                                         */
    set_machw_bssid(mget_bssid());
    set_machw_tsf_start();
#ifndef NON_FC_MACHW_SUPPORT
    enable_machw_non_directed_mgmt_filter();
#endif /* NON_FC_MACHW_SUPPORT */

    /* Change the state to WAIT_START and start the start timeout timer */
    set_mac_state(WAIT_START);
    start_mgmt_timeout_timer(start_req->start_timeout);
}

#endif /* IBSS_BSS_STATION_MODE */

