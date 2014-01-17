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
/*  File Name         : controller_mode_if.h                                 */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC controller functions,  */
/*                      which are mode dependent.                            */
/*                                                                           */
/*  List of Functions : process_host_rx_msdu                                 */
/*                      send_mlme_rsp_to_host                                */
/*                      enable_operation                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CONTROLLER_MODE_IF_H
#define CONTROLLER_MODE_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "ce_lut.h"
#include "maccontroller.h"
#include "prot_if.h"
#include "wep.h"
#include "host_if.h"
#include "mac_init.h"

#ifdef IBSS_BSS_STATION_MODE
#include "event_parser_sta.h"
#include "pm_sta.h"
#include "sme_sta.h"
#include "sta_prot_if.h"
#include "channel_sw.h"
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
#include "event_parser_ap.h"
#include "sme_ap.h"
#include "ap_prot_if.h"
#include "chan_mgmt_ap.h"
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
#include "mh_test.h"
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef TX_ABORT_FEATURE
#ifdef TEST_TX_ABORT_FEATURE
#include "mh_test_txabort.h"
#endif /* TEST_TX_ABORT_FEATURE */
#endif /* TX_ABORT_FEATURE */

//zhuyg add
#ifdef TROUT_WIFI_NPI
#include "trout_wifi_npi.h"
#endif

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void create_mac_interrupts(void);
extern void create_mac_alarms(void);
extern void enable_mac_interrupts(void);
extern void disable_mac_interrupts(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function calls the appropriate routines to process a HOST_RX (MSDU)  */
/* event specific to Access Point or Station mode of operation.              */
INLINE void process_host_rx_msdu(mac_struct_t *mac, UWORD8* host_rx)
{
    print_log_debug_level_1("\n[DL1][INFO][Tx] {Host Rx Data Event}");
#ifdef IBSS_BSS_STATION_MODE
    process_host_rx_msdu_sta(mac, host_rx);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    process_host_rx_msdu_ap(mac, host_rx);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* This function calls the appropriate routines to send the MLME response to */
/* the host based on the SME in use.                                         */
INLINE void send_mlme_rsp_to_host(mac_struct_t *mac, UWORD8 type, UWORD8 *msg)
{
#ifdef IBSS_BSS_STATION_MODE
    send_mlme_rsp_to_host_sta(mac, type, msg);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    send_mlme_rsp_to_host_ap(mac, type, msg);
#endif /* BSS_ACCESS_POINT_MODE */
}

/* Call appropriate initialization function depending on mode of operation */
INLINE void enable_operation(mac_struct_t *mac)
{
#ifndef MAC_HW_UNIT_TEST_MODE

    /* Enable Security */
    enable_security();

    /* Initialize protocols */
    init_prot();

    /* Create MAC interrupts on enabling operation */	
#ifdef BSS_ACCESS_POINT_MODE    
    create_beacon_thread();	//modify by chengwg. 2013-03-02
#endif

    create_mac_interrupts();
    create_mac_alarms();
    create_phy_alarms();

    /* Enable MAC interrupts */
    enable_mac_interrupts();

#ifdef BURST_TX_MODE
    /* Start scaning only when burst tx mode is disabled */
    if(g_burst_tx_mode_enabled == BTRUE)
    {
       initiate_burst_tx_mode(mac);
       return;
    }
#endif /* BURST_TX_MODE */

#ifdef IBSS_BSS_STATION_MODE
    /* Create PSM Alarm */
    create_psm_alarms();
    /* Channel switching alarms */
    create_csw_alarms();
    /* Create 11h related alarms */
    create_11h_alarms();

#ifndef TROUT_B2B_TEST_MODE
#ifndef TROUT_WIFI_NPI
    /* Initiate Scan procedure */
    initiate_scan_procedure(mac);
#endif
#endif

#ifdef TROUT_WIFI_NPI
	printk("npi: enable_operation: g_npi_scan_flag :%d\n", g_npi_scan_flag);
	if (g_npi_scan_flag == 1)
	{
		initiate_scan_procedure(mac);
	}
#endif

#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    /* Create Channel Management Alarms */
    create_chan_mgmt_alarms_ap();

    /* Initiate Start procedure */
    initiate_start(mac);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef TX_ABORT_FEATURE
#ifdef TEST_TX_ABORT_FEATURE
        configure_txab_test();
#endif /* TEST_TX_ABORT_FEATURE */
#endif /* TX_ABORT_FEATURE */


#else /* MAC_HW_UNIT_TEST_MODE */

    /* Create all PHY related alarms */
    create_phy_alarms();

    /* Create the MAC interrupts */
    create_testmac_interrupts();

    /* Enable MAC interrupts */
    enable_mac_interrupts();

    g_test_start = BFALSE;

    start_test();
#endif /* MAC_HW_UNIT_TEST_MODE */
}


INLINE BOOL_T process_wps_event(UWORD32 event)
{
    BOOL_T retval = BFALSE;

#ifdef BSS_ACCESS_POINT_MODE
    retval = process_wps_event_ap(event);
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    retval = process_wps_event_sta(event);
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

INLINE void handle_start_scan_req(void)
{
#ifdef IBSS_BSS_STATION_MODE
    handle_start_scan_req_sta();
#endif /* IBSS_BSS_STATION_MODE */
}

#endif /* CONTROLLER_MODE_IF_H */
