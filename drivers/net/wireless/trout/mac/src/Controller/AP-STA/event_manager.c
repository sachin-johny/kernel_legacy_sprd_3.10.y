/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : event_manager.c                                      */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      implementation of the Event Manager.                 */
/*                                                                           */
/*  List of Functions : is_event_q_ready                                     */
/*                      post_event                                           */
/*                      event_mem_alloc                                      */
/*                      event_mem_free                                       */
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
#include "metrics.h"
#include "mac_init.h"
#include "mh.h"
#include "event_manager.h"
#include "common.h"
#include "host_if.h"
#include "iconfig.h"
#include "runmode.h"

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef DEBUG_MODE
#include "host_if_test.h"
#endif /* DEBUG_MODE */
#endif /* MAC_HW_UNIT_TEST_MODE */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

event_q_struct_t g_event_q[MAX_NUM_EVENT_QUEUES] = {{0,},};
UWORD8 g_buffer_host_rx_events[TOTAL_MEM_SIZE_HOST_RX_EVENTS]   = {0};
UWORD8 g_buffer_wlan_rx_events[TOTAL_MEM_SIZE_WLAN_RX_EVENTS]   = {0};
UWORD8 g_buffer_misc_events[TOTAL_MEM_SIZE_MISC_EVENTS]         = {0};
UWORD8 g_buffer_host_dtx_events[TOTAL_MEM_SIZE_HOST_DTX_EVENTS] = {0};
UWORD8 *g_buffer_host_ctx_events_ptr[MAX_VAR_NUM_EVENT_QUEUES]  = {0};
UWORD8 g_num_host_ctx_event_q                                   = 0;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_event_q_ready                                         */
/*                                                                           */
/*  Description   : This function checks if an event queue is ready for      */
/*                  processing.                                              */
/*                                                                           */
/*  Inputs        : 1) Event Queue identifier                                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the event queue is ready for     */
/*                  processing. For the Host TX Data and Host TX Config      */
/*                  queues, this is done by checking if the corresponding    */
/*                  host is busy.                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE if ready, BFALSE otherwise                         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_event_q_ready(UWORD8 qid)
{
    if(qid == HOST_DTX_EVENT_QID)
    {
        UWORD8 host_type = get_host_data_if_type();

        if(is_host_busy(host_type) == BTRUE)
        {
#ifdef DEBUG_MODE
        g_mac_stats.hifbusy++;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
        g_delay_stats.hifbusy++;
#endif /* MEASURE_PROCESSING_DELAY */

            return BFALSE;
        }
    }
    else if((qid >= HOST_CTX_EVENT_QID_INIT) && (qid < get_num_event_q()))
    {
        UWORD8 host_type = get_host_type(qid);

        if(is_host_busy(host_type) == BTRUE)
        {
            return BFALSE;
        }
    }

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : post_event                                               */
/*                                                                           */
/*  Description   : This function posts an event to an event queue.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the event buffer                           */
/*                  2) Queue identifier                                      */
/*                                                                           */
/*  Globals       : g_event_q                                                */
/*                                                                           */
/*  Processing    : This function inserts the event buffer to the tail of    */
/*                  the event queue accessed by the queue idenitfier. In     */
/*                  case of Linux OS a tasklet is also scheduled for the     */
/*                  event.                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void post_event(UWORD8 *event, UWORD8 qid)
{
    UWORD16          index     = 0;
	//chenq mod 2012-11-01
    event_q_struct_t *event_q  = NULL;//&g_event_q[qid];
    int smart_type = 0;
    
	//chenq add 2012-11-02
	if( reset_mac_trylock() == 0 ){
		event_mem_free(event,qid);
		return;
	}
	//chenq add 2012-11-01
	//spin_lock(&g_event_spin_lock);
	event_q  = &g_event_q[qid];

    /* Find the index for to update the status of the event */
    index = ((UWORD32)event - (UWORD32)event_q->event_buf) / event_q->event_size;

	smart_type = critical_section_smart_start(1,1);
    //critical_section_start1();
    

    /* Update event status as VALID */
    update_buff_status(event_q->buf_status, index, EVENT_BUFFER_VALID);

    event_q->event_cnt++;

   // critical_section_end1();
	critical_section_smart_end(smart_type);

	//chenq add 2012-11-01
	//spin_unlock(&g_event_spin_lock);
#ifdef DEBUG_MODE
    if(event_q->event_cnt > g_mac_stats.pemax[qid])
        g_mac_stats.pemax[qid] = event_q->event_cnt;
#endif /* DEBUG_MODE */

#ifdef OS_LINUX_CSL_TYPE
//    if(g_event_tasklet->state == 0)
//        tasklet_schedule(g_event_tasklet);
      //queue_work(g_event_wq, &g_event_work);
    mac_event_schedule();
#endif /* OS_LINUX_CSL_TYPE */
	reset_mac_unlock();
}

void event_buf_detail_show(UWORD8 qid)
{
    event_q_struct_t *event_q  = &g_event_q[qid];
    UWORD8 status[64];
    UWORD8 free = 0, alloc = 0, valid = 0;
    int i;

    for(i=0; i<event_q->max_events; i++)
    {
		status[i] = check_buff_status(event_q->buf_status, i);
		switch(status[i])
		{
			case EVENT_BUFFER_FREE:
				free++;
				break;
			case EVENT_BUFFER_ALLOC:
				alloc++;
				break;
			case EVENT_BUFFER_VALID:
				valid++;
				break;
		}
    }
	TROUT_DBG4("event %d status: free=%d, alloc=%d, valid=%d\n", 
						qid, free, alloc, valid);
	
    for(i=0; i<event_q->max_events; i++)
    {
		TROUT_DBG5("%2d: %d ", i, status[i]);
		if((i+1)%10 == 0)
			TROUT_DBG5("\n");
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : event_mem_alloc                                          */
/*                                                                           */
/*  Description   : This function allocates buffer for event masseges.       */
/*                                                                           */
/*  Inputs        :  1) Queue identifier                                     */
/*                                                                           */
/*  Globals       : g_event_q                                                */
/*                                                                           */
/*  Processing    : This function allocates buffer for event messages        */
/*                  from Event queue Ring Buffer.                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to the buffer                                    */
/*                  NULL, in case no buffer could be allocated               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void *_event_mem_alloc(UWORD8 qid)
{
    UWORD16          index            = 0;
    UWORD8           *free_event_buff = NULL;
	//chenq mod 2012-11-01
    event_q_struct_t *event_q         = NULL;//&g_event_q[qid];
    int smart_type = 0;
    
	//chenq add 2012-11-01
	//spin_lock(&g_event_spin_lock);
	event_q         = &g_event_q[qid];

    if(event_q->event_cnt == event_q->max_events)
    {
        /* Event Queue is full. */
        return NULL;
    }

	smart_type = critical_section_smart_start(1,1);
    //critical_section_start1();

    index = INCR_RING_BUFF_INDX(event_q->tail_index, event_q->max_events);

    /* Check for free buffer */
    if(check_buff_status(event_q->buf_status, index) == EVENT_BUFFER_FREE)
    {
        /* Update tail_index of the queue */
        event_q->tail_index = index;

        /* Update the status of the buffer as alloc */
        update_buff_status(event_q->buf_status, index, EVENT_BUFFER_ALLOC);

        /* Find the address of the free buffer element from base */
        /* address of the queue, tail_index and event_size.      */
        free_event_buff = (event_q->event_buf +
                            (index * event_q->event_size));
#ifdef DEBUG_MODE
        g_mac_stats.peadd[qid]++;
#endif /* DEBUG_MODE */
    }
    else
    {
#ifdef DEBUG_MODE
        g_mac_stats.peexc[qid]++;
#endif /* DEBUG_MODE */
    }
	critical_section_smart_end(smart_type);
    //critical_section_end1();
	//chenq add 2012-11-01
	//spin_unlock(&g_event_spin_lock);
    return free_event_buff;
}

void *event_mem_alloc(UWORD8 qid)
{
    UWORD8           *free_event_buff = NULL;
    if(reset_mac_trylock() == 0){
	 return NULL;
    }
    free_event_buff = _event_mem_alloc(qid);
    reset_mac_unlock();
    return free_event_buff;
}
/*****************************************************************************/
/*                                                                           */
/*  Function Name : event_mem_free                                           */
/*                                                                           */
/*  Description   : This function deallocates the given buffer.              */
/*                                                                           */
/*  Inputs        :  1) Pointer to the buffer                                */
/*                   2) Queue identifier                                     */
/*                                                                           */
/*  Globals       : g_event_q                                                */
/*                                                                           */
/*  Processing    : This function frees the given event buffer from the      */
/*                  Event queue ring Buffer.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void event_mem_free(void *event, UWORD8 qid)
{
    UWORD16          index    = 0;
	//chenq mod 2012-11-01
    event_q_struct_t *event_q = NULL;//&g_event_q[qid];
    UWORD8 status;
	int smart_type = 0;
	
	//chenq add 2012-11-01
	//spin_lock(&g_event_spin_lock);
	event_q = &g_event_q[qid];

    /* Find the index for to update the status of the event */
    index = ((UWORD32)event - (UWORD32)event_q->event_buf) /
            event_q->event_size;

	smart_type = critical_section_smart_start(1,1);
    //critical_section_start1();

	status = check_buff_status(event_q->buf_status, index);
    update_buff_status(event_q->buf_status, index, EVENT_BUFFER_FREE);

	//chengwg fix bug for case: only alloced event buffer but has not post event! 
	if(status == EVENT_BUFFER_VALID)
    	event_q->event_cnt--;

    //critical_section_end1();
	critical_section_smart_end(smart_type);

#ifdef DEBUG_MODE
        g_mac_stats.pedel[qid]++;
#endif /* DEBUG_MODE */

	//chenq add 2012-11-01
	//spin_unlock(&g_event_spin_lock);
}
