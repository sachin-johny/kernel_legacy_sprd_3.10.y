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
/*  File Name         : host_rx_mlme_ap.c                                    */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the AP MAC FSM on receiving      */
/*                      HOST_RX (MLME) events.                               */
/*                                                                           */
/*  List of Functions : ap_wait_start                                        */
/*                      start_bss_ap                                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "fsm_ap.h"
#include "management_ap.h"
#include "mh.h"
#include "chan_mgmt_ap.h"


/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_bss_ap                                             */
/*                                                                           */
/*  Description   : This function starts a BSS in AP mode.                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming start request message         */
/*                                                                           */
/*  Globals       : g_beacon_frame                                           */
/*                  g_beacon_index                                           */
/*                                                                           */
/*  Processing    : The TSF timer is enabled after setting BSSID and         */
/*                  preparation of beacons is done. Beacon pointer is given  */
/*                  to hardware. It should send it in TBTT. mac state will   */
/*                  be changed to WAIT_START.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_bss_ap(UWORD8 *msg)
{
    start_req_t *start_req    = 0;
    UWORD8      *beacon_frame = 0;
    UWORD32     *trout_beacon_buf = 0, beacon_len = 0;

    /* The incoming message is a Start Request message and is cast to the    */
    /* start request structure.                                              */
    start_req = (start_req_t *)msg;

    /* Update the MIB and globals with the Start request parameters */
    update_start_req_params(start_req);

    /* Generate 46 bit random BSSID which is same as Station ID in AP mode   */
    mset_bssid(mget_StationID());

    /* Prepare a beacon frame and add it to the H/w queue. Also set the      */
    /* parameters like, beacon length, rate etc.                             */

    /* Allocate 2 global beacon buffers as the beacon for AP may be modfied  */
    /* for which 2 buffers are needed.                                       */
    g_beacon_frame[0] = (UWORD8 *)pkt_mem_alloc(MEM_PRI_HPTX);
    if(g_beacon_frame[0] == NULL)
    {
        /* Exception - no buffers for beacons */
        return;
    }

    g_beacon_frame[1] = (UWORD8 *)pkt_mem_alloc(MEM_PRI_HPTX);
    if(g_beacon_frame[1] == NULL)
    {
        /* Exception - no buffers for beacons */
        pkt_mem_free(g_beacon_frame[0]);
        return;
    }
    /* Prepare beacon in the first buffer */
    beacon_frame = g_beacon_frame[0] + SPI_SDIO_WRITE_RAM_CMD_WIDTH;
    g_beacon_len = prepare_beacon_ap(beacon_frame);

    /* Copy it to other buffer as well */
    memcpy(g_beacon_frame[1], g_beacon_frame[0], g_beacon_len + SPI_SDIO_WRITE_RAM_CMD_WIDTH);

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
	host_write_trout_ram((void *)trout_beacon_buf, 
			g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH, beacon_len);

    /* Write required Hardware registers (BSSID, enable synchronization) and */
    /* wait for TBTT                                                         */
    set_machw_beacon_pointer((UWORD32)trout_beacon_buf);
    set_machw_beacon_tx_params(g_beacon_len,
                               get_phy_rate(get_beacon_tx_rate()));
    set_machw_beacon_ptm(get_reg_phy_tx_mode(get_beacon_tx_rate(), 1));
    set_machw_bssid(mget_bssid());
//    set_machw_tsf_start(); //jiangtao.yi

    /* Start the start timeout timer and change the state of the MAC         */
    /* structure to WAIT_START                                               */
    start_mgmt_timeout_timer(start_req->start_timeout);
    set_machw_tsf_start(); //jiangtao.yi
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_wait_start                                            */
/*                                                                           */
/*  Description   : This function calls a MAC core function to initiate the  */
/*                  start process.                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks and performs channel availability   */
/*                  check based on the protocol in use. If not required a    */
/*                  BSS is started. The state is changed to WAIT_START. In   */
/*                  this state it will wait for either the start timeout and */
/*                  TBTT event (in case BSS start has been initiated) or     */
/*                  channel availability check timeout event (otherwise)     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_wait_start(mac_struct_t *mac, UWORD8 *msg)
{
    /* The state is changed to WAIT_START */
    set_mac_state(WAIT_START);

    /* Start the AP after deciding the right channel */
    start_channel_availability_check(msg);
}

#endif /* BSS_ACCESS_POINT_MODE */
