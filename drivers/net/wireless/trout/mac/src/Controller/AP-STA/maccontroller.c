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
/*  File Name         : maccontroller.c                                      */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      implementation of the MAC controller (event manager).*/
/*                                                                           */
/*  List of Functions : init_event_manager                                   */
/*                      mac_controller_task                                  */
/*                      process_all_events                                   */
/*                      process_event_queues                                 */
/*                      process_event_q                                      */
/*                      process_host_rx_event                                */
/*                      process_wlan_rx_event                                */
/*                      process_misc_event                                   */
/*                      process_host_tx_event                                */
/*                      is_serious_error                                     */
/*                      handle_system_error                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "controller_mode_if.h"
#include "core_mode_if.h"
#include "event_parser.h"
#include "qmu_if.h"
#include "receive.h"
#include "mac_init.h"
#include "mh.h"
#include "event_manager.h"
#include "iconfig.h"
/*leon liu added for CFG80211*/
#ifdef CONFIG_CFG80211
#include "trout_cfg80211.h"
#endif

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef DEBUG_MODE
#include "host_if_test.h"
#include "trout_trace.h"
#endif /* DEBUG_MODE */
#endif /* MAC_HW_UNIT_TEST_MODE */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

ERROR_CODE_T g_system_error = NO_ERROR;
mac_struct_t g_mac          = {0};
qmu_handle_t g_q_handle     = {0};
void         (*g_process_event[MAX_NUM_EVENT_QUEUES])(UWORD32);

//chenq add 2012-10-30
atomic_t g_event_cnt;

unsigned int counter_tmpr =0; //by lihua for temperator compensation
extern void tempr_compensated(void );
//chenq add 2012-10-30
void mac_time2event_work(UWORD32 data);
void process_event_queues(void);

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

//static void process_event_queues(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_event_manager                                       */
/*                                                                           */
/*  Description   : This function initializes the event manager.             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_process_event                                          */
/*                                                                           */
/*  Processing    : This function initializes all the event queues, globals  */
/*                  and event processing functions used by the event manager */
/*                  run by the MAC controller.                               */
/*                                                                           */
/*  Outputs       : Initializes global event queue array (g_event_q), the    */
/*                  global event processing function array (g_process_event).*/
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_event_manager(void)
{
    UWORD8 i = 0;
    
	TROUT_FUNC_ENTER;
    /* Initialize event queue information for Host TX Config event queues */
    init_host_ctx_event_q_info();

    /* Initialize all event queues */
    for(i = 0; i < get_num_event_q(); i++)
    {
        init_event_q(i);
    }

    /* Initialize all WLAN event processing functions */
    g_process_event[HOST_RX_EVENT_QID] = process_host_rx_event;
    g_process_event[WLAN_RX_EVENT_QID] = process_wlan_rx_event;
    g_process_event[MISC_EVENT_QID]    = process_misc_event;

    /* Initialize all Host event processing functions */
    for(i = HOST_DTX_EVENT_QID; i < get_num_event_q(); i++)
    {
        g_process_event[i] = process_host_tx_event;
    }
    atomic_set(&g_event_cnt, 0);

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_controller_task                                      */
/*                                                                           */
/*  Description   : This is the main function that controls all the MAC      */
/*                  functions.                                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function polls the various event queues serially    */
/*                  for an event. The events in a queue are processed in the */
/*                  order of arrival. The polling continues till all the     */
/*                  queues become empty. The event queues maintained are the */
/*                  HOST_RX, WLAN_RX, MISC queues.                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void mac_controller_task(ADDRWORD_T data)
{
    /* This is the only function running in an infinite loop. The function   */
    /* shall be suspended only when the processor is interrupted and shall   */
    /* be resumed once interrupt processing is over.                         */
    while(1)
    {
        /* process all the pending events */
        process_all_events();

        /* Update the CPU utiltization test count */
        update_cpu_util_count();
    }
}

//add by chenq.
void mac_time2event_work(UWORD32 data)
{
	mac_event_schedule();
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_all_events                                       */
/*                                                                           */
/*  Description   : This function processes all the pending events           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_reset_done                                         */
/*                  g_process_event                                          */
/*                                                                           */
/*  Processing    : This function gets the event at the head of the event    */
/*                  queue and processes it.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
extern struct completion event_complete;

extern void wake_up_rst(void);
#ifdef TROUT_WIFI_NPI
extern void tx_packet_test(void); //zhuyg add for factory test
#endif

void process_all_events(void)
{	
	TROUT_FUNC_ENTER;

	if(reset_mac_trylock() == 0)
	{
		return;
      }
	
	atomic_set(&g_event_cnt, get_total_num_pending_events());
	stop_alarm(g_mac_event_timer);
	
    while(atomic_read(&g_event_cnt) > 0)
    {		     

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if (mutex_is_locked(&suspend_mutex) || (g_wifi_suspend_status != wifi_suspend_nosuspend)) {
		//libing, 20140114, fix fake connection
		pr_info("We can't do %s during suspending, g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
	        reset_mac_unlock();  
                return; 	
	}
#endif
#endif
        /* Process pending events in the event queues */
        process_event_queues();
	if (counter_tmpr++ > 100)//by lihua
	{
		counter_tmpr = 0;
#ifdef WAKE_LOW_POWER_POLICY
			if(g_wifi_power_mode != WIFI_NORMAL_POWER_MODE)
			{
				pr_info("We can't do %s during low power mode, g_wifi_suspend_status = %d\n", __func__, g_wifi_suspend_status);
				break;
			}
#endif
		tempr_compensated();		
	}

#ifndef MAC_HW_UNIT_TEST_MODE
        /* Handle lack of memory for WLAN Rx packets */
//        handle_rxq_replenishment();
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef BSS_ACCESS_POINT_MODE
#ifdef ENABLE_PS_PKT_FLUSH
        /* Find the STA with maximum PS buffers  */
        find_max_ps_ae();
#endif /* ENABLE_PS_PKT_FLUSH */
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef INT_WPS_SUPP //wxb add
#ifdef IBSS_BSS_STATION_MODE
    /* Handle scan request from user/protocol */
    if(g_wps_scan_req_from_user == BTRUE)// caisf add for fix wps scan bug. 1121
    {
        handle_start_scan_req();//may be reset_mac
        g_wps_scan_req_from_user = BFALSE;
    }
#endif
#endif /* ifdef INT_WPS_SUPP  */

        /* Service Watchdog */
        service_wdt();
    }

	//chenq add 2012-11-02
//	if((BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE)	//shield by chengwg, 2013-01-11!
//		return;

    /* Handle system error due to any failure */
	active_netif_queue();	//add by chengwg.
    	handle_system_error();//may be reset_mac
	/* Handle scan request from user/protocol */
	//chenq mask 2012-10-29
	//handle_start_scan_req();
#ifdef INT_WPS_SUPP //wxb add
#ifdef IBSS_BSS_STATION_MODE
    /* Handle scan request from user/protocol */
    if(g_wps_scan_req_from_user == BTRUE)// caisf add for fix wps scan bug. 1121
    {
        handle_start_scan_req(); //the function may be reset_mac
        g_wps_scan_req_from_user = BFALSE;
    }
#endif
#endif /* ifdef INT_WPS_SUPP  */

    /* Service Watchdog */
    service_wdt();
    
    if(get_total_num_pending_events() > 0)
	{    
	    if(g_mac_event_timer == NULL)
        {
            g_mac_event_timer = create_alarm(mac_time2event_work, 0, NULL);
        }
		
        start_alarm(g_mac_event_timer, 100);
	}
     reset_mac_unlock();
    TRACE_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_event_queues                                     */
/*                                                                           */
/*  Description   : This function processes all the event queues.            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function processes all the event queues that are    */
/*                  defined.                                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_event_queues(void)
{
    UWORD8 i     = 0;
    UWORD8 num_q = get_num_event_q();
    
	TROUT_FUNC_ENTER;
    for(i = 0; i < HOST_DTX_EVENT_QID; i++)
    {
        process_wlan_event_q(i);
    }

    for(i = HOST_DTX_EVENT_QID; i < num_q; i++)
    {
        process_host_event_q(i);
    }
        	
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_wlan_event_q                                     */
/*                                                                           */
/*  Description   : This function processes the given WLAN event queue if it */
/*                  is ready for processing. The following event-Qs are      */
/*                  considered to be WLAN-side event-Qs.                     */
/*                  1) Host-RX Event                                         */
/*                  2) WLAN-TX Event                                         */
/*                  3) Misc Event                                            */
/*                  The event handlers for these events can use Scratch      */
/*                  memory and also reset MAC internally. Also, they can     */
/*                  call the Host Event Queue handlers directly.             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_reset_done                                         */
/*                  g_process_event                                          */
/*                                                                           */
/*  Processing    : This function checks if the given event queue is ready   */
/*                  for processing. If it is, the event at the head of the   */
/*                  event queue is removed and processed. If no reset has    */
/*                  occurred due to the event processing the event is freed. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_wlan_event_q(UWORD8 qid)
{
    UWORD32 event = 0;
    
	TROUT_FUNC_ENTER;
    if(is_event_q_ready(qid) == BFALSE)
    {
		TROUT_DBG4("%s: event queue is not ready!\n", __func__);
		TROUT_FUNC_EXIT;
        return;
	}
    /* Get the event from the head of the event queue */
    event = (UWORD32)get_event(qid);

    /* Process the event if it is a valid event */
    if(event != 0)
    {
		process_wlan_event_count++;
		//chenq mod 2012-11-02
		//g_mac_reset_done = BFALSE;
//		if( (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE )	//shield by chengwg, 2013-01-11!
//			goto event_free;

#ifdef MEASURE_PROCESSING_DELAY
        start_delay_measurement(PROC_DELAY, (UWORD32)(qid));
#endif /* MEASURE_PROCESSING_DELAY */

        /* Re-initialize the scratch memory before each event processing */
        reinit_scratch_mem();

#ifndef ENABLE_STACK_ESTIMATION
        /* Call the function to process this event using the global      */
        /* function pointer array.                                       */
        (g_process_event[qid])(event);
#else /* ENABLE_STACK_ESTIMATION */
        process_host_rx_event(event);
        process_wlan_rx_event(event);
        process_misc_event(event);
#endif /* ENABLE_STACK_ESTIMATION */

#ifdef MEASURE_PROCESSING_DELAY
        stop_delay_measurement(PROC_DELAY, (UWORD32)(qid));
#endif /* MEASURE_PROCESSING_DELAY */
        /* Free the event buffer after processing. The buffer is         */
        /* allocated by the ISR posting the event. Note that the event   */
        /* message buffer is not freed here. It should be freed by the   */
        /* MAC core.                                                     */

		//chenq mod 2012-11-02
        //if(g_mac_reset_done == BFALSE)
        //{
            event_mem_free((void *)event, qid);
        //}

		//chenq add 2012-10-30
        atomic_dec(&g_event_cnt);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_host_event_q                                     */
/*                                                                           */
/*  Description   : This function processes the given Host-side event queue  */
/*                  if it is ready for processing. The following event       */
/*                  Queues are considered to be Host-side event queues:      */
/*                  1) Common Data Host-TX Queue                             */
/*                  2) Per-Host I/f Configuration-TX Queue                   */
/*                  The event handlers of these events have the following    */
/*                  restrictions:                                            */
/*                  1) They cannot use Scratch memory directly or call       */
/*                  functions which use scratch memory                       */
/*                  2) They cannot internally reset MAC. Any reset requests  */
/*                  have to be through raising system errors.                */
/*                  3) They cannot call WLAN Event handlers directly.        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_process_event                                          */
/*                                                                           */
/*  Processing    : This function checks if the given event queue is ready   */
/*                  for processing. If it is, the event at the head of the   */
/*                  event queue is removed and processed. If no reset has    */
/*                  occurred due to the event processing the event is freed. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_host_event_q(UWORD8 qid)
{
    UWORD32 event = 0;

	TROUT_FUNC_ENTER;
	
    if(is_event_q_ready(qid) == BFALSE)
    {
		TROUT_DBG4("%s: event queue is not ready!\n", __func__);
		TROUT_FUNC_EXIT;
        return;
	}
    /* Get the event from the head of the event queue */
    event = (UWORD32)get_event(qid);

    /* Process the event if it is a valid event */
    if(event != 0)
    {		
    	process_host_event_count++;
#ifdef MEASURE_PROCESSING_DELAY
        start_delay_measurement(PROC_DELAY, (UWORD32)(qid));
#endif /* MEASURE_PROCESSING_DELAY */

#ifndef ENABLE_STACK_ESTIMATION
        /* Call the function to process this event using the global      */
        /* function pointer array.                                       */
        (g_process_event[qid])(event);
#else /* ENABLE_STACK_ESTIMATION */
        process_host_tx_event(event);
#endif /* ENABLE_STACK_ESTIMATION */

#ifdef MEASURE_PROCESSING_DELAY
        stop_delay_measurement(PROC_DELAY, (UWORD32)(qid));
#endif /* MEASURE_PROCESSING_DELAY */

        /* Free the event buffer after processing. The Host event cannot */
        /* free this event internally. An implication of this is that,   */
        /* the host event cannot reset MAC directly. It has to raise a   */
        /* System-Reset request if it wants to reset MAC.                */
		//chenq add 2012-11-02

        event_mem_free((void *)event, qid);

        atomic_dec(&g_event_cnt);
    }

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_host_rx_event                                    */
/*                                                                           */
/*  Description   : This function processes the HOST_RX event.               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the event                                  */
/*                                                                           */
/*  Globals       : g_mac                                                    */
/*                                                                           */
/*  Processing    : The event type/subtype is checked and the appropriate    */
/*                  interface function is called to process the event (any   */
/*                  required protocol conversion is done) and call the MAC   */
/*                  core API.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_host_rx_event(UWORD32 event)
{
    host_rx_event_msg_t *temp = 0;
    UWORD8 host_type          = 0;

	TROUT_FUNC_ENTER;

#ifdef TROUT_TRACE_DBG
	process_host_rx_event_count++;
	rx_event_count +=1;
#endif	/* TROUT_TRACE_DBG */

    print_log_debug_level_1("\n[DL1][INFO][Tx] {Host Rx Event}");

    temp = (host_rx_event_msg_t *)event;
    host_type = temp->host_if_type;

    check_sec_auth_pkt(((host_rx_event_msg_t *)event)->buffer_addr +
            ((host_rx_event_msg_t *)event)->pkt_ofst,
            ((host_rx_event_msg_t *)event)->rx_pkt_len );

    if(is_config_pkt(temp->buffer_addr + temp->pkt_ofst, host_type) == BTRUE)
    {

#ifdef DEBUG_MODE
        g_mac_stats.pehrxml++;
#endif /* DEBUG_MODE */

        /* Process the incoming host configuration packet */
        process_host_rx_mlme(&g_mac, (UWORD8 *)event);
    }
#ifndef MAC_HW_UNIT_TEST_MODE
    else
    {
        /* Check if the packet has come from the correct host interface */
        if(get_host_data_if_type() != host_type)
        {
            pkt_mem_free(temp->buffer_addr);
            TROUT_FUNC_EXIT;
			//caisf add 2013-02-15
            if(g_mac_net_stats)
                g_mac_net_stats->tx_dropped++;

            return;
        }

#ifdef DEBUG_MODE
        g_mac_stats.pehrxms++;
#endif /* DEBUG_MODE */

#ifndef HOST_LOOP_BACK_MODE
        /* Process the incoming host data packet */
		TX_PATH_DBG("%s: process incoming data pkt\n", __func__);
        process_host_rx_msdu(&g_mac, (UWORD8 *)event);
#else  /* HOST_LOOP_BACK_MODE */
        /* Process incoming data packet & loop it back to the host */
        process_host_rx_loopback((UWORD8 *)event);
#endif /* HOST_LOOP_BACK_MODE */

    }
#else /* MAC_HW_UNIT_TEST_MODE */
    else
    {
        if(BFALSE == check_hut_frame((UWORD8 *)event))
            pkt_mem_free(temp->buffer_addr);
    }
#endif /* MAC_HW_UNIT_TEST_MODE */

	TRACE_FUNC_EXIT;

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_wlan_rx_event                                    */
/*                                                                           */
/*  Description   : This function processes the WLAN_RX event.               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the event                                  */
/*                                                                           */
/*  Globals       : g_q_handle                                               */
/*                  g_mac                                                    */
/*                                                                           */
/*  Processing    : The event type/subtype is checked and the appropriate    */
/*                  MAC core API is called.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void process_wlan_rx_event(UWORD32 event)
{	
    print_log_debug_level_1("\n[DL1][INFO][Rx] {WLAN Rx Event}");
    TROUT_FUNC_ENTER;
    
    process_wlan_rx_event_count++;
#ifdef WAKE_LOW_POWER_POLICY
	if(g_wifi_power_mode != WIFI_NORMAL_POWER_MODE)
	{
		pr_info("We can't do %s during low power mode\n", __func__);
		return;
	}
#endif
    
#ifdef TROUT_B2B_TEST_MODE
    trout_b2b_rx_low(&g_mac, (UWORD8*)event);
#else
    wlan_rx(&g_mac, (UWORD8*)event);
#endif
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_misc_event                                       */
/*                                                                           */
/*  Description   : This function processes the MISC event.                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the event                                  */
/*                                                                           */
/*  Globals       : g_mac                                                    */
/*                                                                           */
/*  Processing    : This function calls the MAC core API to handle MISC      */
/*                  events, regardless of type.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_misc_event(UWORD32 event)
{
	TROUT_FUNC_ENTER;
#ifdef DEBUG_MODE
        g_mac_stats.pewmc++;
#endif /* DEBUG_MODE */
#ifdef WAKE_LOW_POWER_POLICY
	if(g_wifi_power_mode != WIFI_NORMAL_POWER_MODE) 
	{
		pr_info("We can't do %s during low power mode\n", __func__);
		return;
	}
#endif
    /* Check and process the event if it is a WPS event */
    if(process_wps_event(event) == BTRUE)
    {
		TROUT_FUNC_EXIT;
        return;
	}
	process_misc_event_count++;
	
    wlan_misc(&g_mac, (UWORD8 *)event);

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_host_tx_event                                    */
/*                                                                           */
/*  Description   : This function processes the Host TX event.               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the event                                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the MAC core API to handle MISC      */
/*                  events, regardless of type.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_host_tx_event(UWORD32 event)
{
    host_tx_event_msg_t *host_tx = (host_tx_event_msg_t *)event;
	
	TROUT_FUNC_ENTER;
	
	process_host_tx_event_count++;
	
    send_frame_to_host(host_tx->host_type, &(host_tx->fdesc));

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.nummsduhiftxed++;
#endif /* MEASURE_PROCESSING_DELAY */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_serious_error                                         */
/*                                                                           */
/*  Description   : This function checks the severity of the system error.   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : BTURE if serious error.                                  */
/*                  BFALSE otherwise                                         */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_serious_error(ERROR_CODE_T error_code)
{
    /* Check whether it is a serious H/w error */
    if((error_code > 0) && (error_code < MAX_MAC_HW_ERROR_CODES))
        return is_serious_hw_error((UWORD8)error_code & 0x0F);

    return BFALSE;
}

static void reset_mac_task(struct work_struct *work)  
{  
   #ifdef BSS_ACCESS_POINT_MODE  
           restart_mac(&g_mac,0);  
   #else  
           printk("reset_mac_task\n");  
           restart_mac_plus(&g_mac, BTRUE);  
   #endif  
}  
static DECLARE_WORK(reset_mac_work, reset_mac_task);  

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_system_error                                      */
/*                                                                           */
/*  Description   : This function handles a system error                     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_system_error                                           */
/*                                                                           */
/*  Processing    : This function restarts the system incase of an error     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_system_error(void)
{
    BOOL_T is_restart_required = BFALSE;
    int smart_type = MODE_START;
	struct trout_private *tp = netdev_priv(g_mac_dev);
	struct rw_semaphore *sem = &(tp->rst_semaphore);


    TROUT_FUNC_ENTER;
    /* Check whether a MAC reset is allowed. */
    if(allow_mac_reset() == BFALSE)
    {
	 TROUT_FUNC_EXIT;
        return;
    }
	
    if(g_system_error != NO_ERROR)
    {
#ifdef DEBUG_MODE
        switch(g_system_error)
        {
            case NO_LOCAL_MEM:
            {
                g_reset_stats.no_local_mem++;
            }
            break;
            case NO_SHRED_MEM:
            {
                g_reset_stats.no_shred_mem++;
            }
            break;
            case NO_EVENT_MEM:
            {
                g_reset_stats.no_event_mem++;
            }
            break;
            case TX_Q_ERROR:
            {
                g_reset_stats.tx_q_error++;
            }
            break;
            case RX_Q_ERROR:
            {
                g_reset_stats.rx_q_error++;
            }
            break;
            default:
            {
                /* Do nothing */
            }
            break;
        }
#endif /* DEBUG_MODE */

#ifdef DEBUG_MODE
        printk("System-Error = %x\n", g_system_error);
#endif /* DEBUG_MODE */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
        if((g_system_error == DEAUTH_RCD) ||
           (g_system_error == LINK_LOSS) ||
           (g_system_error == CUSTOM_CNTRMSR) || 
           (g_system_error == CE_UPDATE_FAIL)) 
        {

            if(g_wakeup_flag_for_60s)
            {
                if(wake_lock_active(&deauth_err_lock))  
                {
                    pr_info("%s-%d: acquire wake_lock %s\n", __func__, __LINE__,deauth_err_lock.name);   
                    wake_unlock(&deauth_err_lock);  
                }  

                wake_lock_timeout(&deauth_err_lock,msecs_to_jiffies(60000)); /*Keep 60s awake when deauth, by caisf 20130929*/
                g_wakeup_flag_for_60s = 0;
                
                //wake_lock(&deauth_err_lock); /*Keep awake when deauth, by caisf 20130929*/
                pr_info("%s-%d: acquire wake_lock %s\n", __func__, __LINE__, 
                deauth_err_lock.name);
            }
        }
#endif
#endif

        /* Disable any protocols in progress */
        is_restart_required = allow_sys_restart_prot(g_system_error);

        /* Restart the mac */
        if(BTRUE == is_restart_required)
        {
			if(mutex_is_locked(&tp->sm_mutex))
				return;
        	//chenq add 2012-10-31
		set_mac_state(DISABLED);

#ifndef BSS_ACCESS_POINT_MODE
            // Modify by Yiming.Li at 2014-01-07 for fix bug: reconnect
            #ifdef CONFIG_CFG80211
            if(g_system_error == LINK_LOSS) 
		    trout_cfg80211_del_prev_bss(g_mac_dev);
	    #endif
#endif
		schedule_work(&reset_mac_work);
        }

        /* Start a Rejoin timer if required after restarting MAC */
        start_rejoin_timer();

        /* Update the sytem error flag */
        smart_type = critical_section_smart_start(0,1);
        g_system_error = NO_ERROR;
        critical_section_smart_end(smart_type);
    }
    TROUT_FUNC_EXIT;
}
