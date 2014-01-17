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
/*  File Name         : event_manager.h                                      */
/*                                                                           */
/*  Description       : This file contains all the declarations related to   */
/*                      the Event Manager.                                   */
/*                                                                           */
/*  List of Functions : check_buff_status                                    */
/*                      update_buff_status                                   */
/*                      init_static_event_manager_q                          */
/*                      event_q_remove_from_head                             */
/*                      event_q_remove_all_misc_events                       */
/*                      event_q_remove_all_host_tx_events                    */
/*                      set_num_host_ctx_event_q                             */
/*                      get_num_event_q                                      */
/*                      get_num_pending_events_in_q                          */
/*                      get_event                                            */
/*                      init_event_q                                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "itypes.h"
#include "common.h"
#include "qmu.h"
#include "queue.h"
#include "buff_desc.h"
#include "iparams.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/* MAC FSM states and inputs */
#define MAX_MAC_FSM_STATES 11
#define MAX_MAC_FSM_INPUTS 11

/* Minimum 4 Event queues: HOST RX, HOST TX DATA, WLAN RX, WLAN MISC */
#define MIN_NUM_EVENT_QUEUES (4)

/* 1 Event queue for HOST TX CONFIG per Host Interface Type defined. Maximum */
/* number of such event queues is therefore equal to the maximum number of   */
/* host interface types supported.                                           */
#define MAX_VAR_NUM_EVENT_QUEUES (MAX_NUM_HOST_SUPPORTED)

/* The maximum number of event queues that can be present is the sum of the  */
/* minimum number of event queues and the maximum variable number of event   */
/* queues                                                                    */
#define MAX_NUM_EVENT_QUEUES (MIN_NUM_EVENT_QUEUES + MAX_VAR_NUM_EVENT_QUEUES)

/* Event buffer sizes based on the maximum number for each event type */
#define HOST_RX_EVENT_BUF_SIZE  WORD_ALIGN((MAX_NUM_HOST_RX_EVENTS >> 2) + 1)
#define WLAN_RX_EVENT_BUF_SIZE  WORD_ALIGN((MAX_NUM_WLAN_RX_EVENTS >> 2) + 1)
#define MISC_EVENT_BUF_SIZE     WORD_ALIGN((MAX_NUM_MISC_EVENTS >> 2) + 1)
#define HOST_DTX_EVENT_BUF_SIZE WORD_ALIGN((MAX_NUM_HOST_DTX_EVENTS >> 2) + 1)
#define HOST_CTX_EVENT_BUF_SIZE WORD_ALIGN((MAX_NUM_HOST_CTX_EVENTS >> 2) + 1)

/* Event message sizes based on the message structure for each event type */
#define HOST_RX_EVENT_MSG_SIZE WORD_ALIGN(sizeof(host_rx_event_msg_t))
#define WLAN_RX_EVENT_MSG_SIZE WORD_ALIGN(sizeof(wlan_rx_event_msg_t))
#define MISC_EVENT_MSG_SIZE    WORD_ALIGN(sizeof(misc_event_msg_t))
#define HOST_TX_EVENT_MSG_SIZE WORD_ALIGN(sizeof(host_tx_event_msg_t))

/* Total memory size for each event type */
#define TOTAL_MEM_SIZE_HOST_RX_EVENTS (MAX_NUM_HOST_RX_EVENTS * \
                                       HOST_RX_EVENT_MSG_SIZE + \
                                       HOST_RX_EVENT_BUF_SIZE + \
                                       MAX_WORD_ALIGNMENT_BUFFER)

#define TOTAL_MEM_SIZE_WLAN_RX_EVENTS (MAX_NUM_WLAN_RX_EVENTS * \
                                       WLAN_RX_EVENT_MSG_SIZE + \
                                       WLAN_RX_EVENT_BUF_SIZE + \
                                       MAX_WORD_ALIGNMENT_BUFFER)

#define TOTAL_MEM_SIZE_MISC_EVENTS    (MAX_NUM_MISC_EVENTS * \
                                       MISC_EVENT_MSG_SIZE + \
                                       MISC_EVENT_BUF_SIZE + \
                                       MAX_WORD_ALIGNMENT_BUFFER)

#define TOTAL_MEM_SIZE_HOST_DTX_EVENTS (MAX_NUM_HOST_DTX_EVENTS * \
                                        HOST_TX_EVENT_MSG_SIZE +  \
                                        HOST_DTX_EVENT_BUF_SIZE + \
                                        MAX_WORD_ALIGNMENT_BUFFER)

#define TOTAL_MEM_SIZE_HOST_CTX_EVENTS (MAX_NUM_HOST_CTX_EVENTS * \
                                        HOST_TX_EVENT_MSG_SIZE +  \
                                        HOST_CTX_EVENT_BUF_SIZE + \
                                        MAX_WORD_ALIGNMENT_BUFFER)

/* Macro for incrementing the ring buffer index */
#define INCR_RING_BUFF_INDX(indx,max_num) ((((indx) + 1) < (max_num)) ? \
                                            ((indx) + 1) : (0))

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Event Queue Identifier */
typedef enum {HOST_RX_EVENT_QID       = 0,
              WLAN_RX_EVENT_QID       = 1,
              MISC_EVENT_QID          = 2,
              HOST_DTX_EVENT_QID      = 3,
              HOST_CTX_EVENT_QID_INIT = 4
/* HOST_CTX_EVENT_QIDs: 4..., 1 per Host Type supported */
} EVENT_QID_T;


/* Event buffer status */
typedef enum
{
    EVENT_BUFFER_FREE  = 0,
    EVENT_BUFFER_ALLOC = 1,
    EVENT_BUFFER_VALID = 2,
} EVENT_BUFFER_STATUS_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* MAC Library Structure */
typedef struct mac_struct
{
    UWORD8 state;
    void   (*fsm[MAX_MAC_FSM_STATES][MAX_MAC_FSM_INPUTS])
           (struct mac_struct *, UWORD8 *);
} mac_struct_t;

/* Message Structure for MISC event */
typedef struct
{
    UWORD16 name;
    UWORD8  info;
    void    *data;
} misc_event_msg_t;

/* Message Structure for WLAN_RX event */
typedef struct
{
    UWORD8  sync_req;
    UWORD8  rxq_num;
    UWORD8  num_dscr;
    void    *base_dscr;
} wlan_rx_event_msg_t;

/* Message Structure for HOST_RX event */
typedef struct
{
    UWORD8  host_if_type;
    UWORD8  pkt_ofst;
    UWORD16 rx_pkt_len;
    UWORD8  *buffer_addr;
} host_rx_event_msg_t;

/* Message Structure for HOST_TX (Data/Config) event */
typedef struct
{
    UWORD8      host_type;
    msdu_desc_t fdesc;
} host_tx_event_msg_t;

/* Event queue element */
typedef struct
{
    UWORD8  *event_buf;  /* Base address of event buffer array               */
    UWORD8  *buf_status; /* Base address of buffer status array              */
    UWORD16 event_size;  /* Size of the event message structure              */
    UWORD16 max_events;  /* Number of events can be accomadated in the queue */
    UWORD16 event_cnt;   /* Current number of valid events                   */
    UWORD16 tail_index;  /* Last index inserted                              */
    UWORD16 head_index;  /* Next index to be read                            */
} event_q_struct_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern BOOL_T is_event_q_ready(UWORD8 qid);
extern void   post_event(UWORD8 *event, UWORD8 qid);
extern void   *event_mem_alloc(UWORD8 qid);
extern void   *_event_mem_alloc(UWORD8 qid); //the function don't lock the reset lock.
extern void   event_mem_free(void *event, UWORD8 qid);
extern void event_buf_detail_show(UWORD8 qid);	//chengwg debug use!
/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern event_q_struct_t g_event_q[MAX_NUM_EVENT_QUEUES];
extern UWORD8 g_buffer_host_rx_events[TOTAL_MEM_SIZE_HOST_RX_EVENTS];
extern UWORD8 g_buffer_wlan_rx_events[TOTAL_MEM_SIZE_WLAN_RX_EVENTS];
extern UWORD8 g_buffer_misc_events[TOTAL_MEM_SIZE_MISC_EVENTS];
extern UWORD8 g_buffer_host_dtx_events[TOTAL_MEM_SIZE_HOST_DTX_EVENTS];
extern UWORD8 *g_buffer_host_ctx_events_ptr[MAX_VAR_NUM_EVENT_QUEUES];
extern UWORD8 g_num_host_ctx_event_q;

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function is used to check the status of the buffer. */
INLINE UWORD8 check_buff_status(UWORD8 *buff, UWORD16 index)
{
    UWORD8 offset = 0;
    UWORD8 status = 0;

	//every two bit as a status of a event in buff --> chwg.
    /* --------------------------------------------------------------------  */
    /*                       Buffer Status Array Format                      */
    /* --------------------------------------------------------------------  */
    /* | BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0|BIT7|BIT6|BIT5|BIT4|BIT3|   */
    /* --------------------------------------------------------------------  */
    /* <------------ BYTE0 --------------------><------------ BYTE1 ------   */
    /*                                                                       */
    /* --------------------------------------------------------------------  */
    /* BIT0,BIT1 of BYTE0 indicates status of the first entry of buffer,     */
    /* BIT2,BIT3 of BYTE1 indicates status of the second entry of buffer etc */
    /* --------------------------------------------------------------------  */

    /* Derive the index of the status byte with the status array and also    */
    /* the bit offset within the status byte from the buffer index.          */
    offset = (index & 0x3) << 1;
    buff  += (index >> 2);

    /* Read out the status using the derived status byte and bit offset */
    status = (*buff >> offset) & 0x3;

    return status;
}

/* This function is used to update the status of the buffer. It is called    */
/* when event is added to the ring buffer or when event is removed           */
INLINE void update_buff_status(UWORD8 *buff, UWORD16 index, UWORD8 status)
{
    UWORD8 offset = 0;

    /* --------------------------------------------------------------------  */
    /*                       Buffer Status Array Format                      */
    /* --------------------------------------------------------------------  */
    /* | BIT7|BIT6|BIT5|BIT4|BIT3|BIT2|BIT1|BIT0|BIT7|BIT6|BIT5|BIT4|BIT3|   */
    /* --------------------------------------------------------------------  */
    /* <------------ BYTE0 --------------------><------------ BYTE1 ------   */
    /*                                                                       */
    /* --------------------------------------------------------------------  */
    /* BIT0,BIT1 of BYTE0 indicates status of the first entry of buffer,     */
    /* BIT2,BIT3 of BYTE1 indicates status of the second entry of buffer etc */
    /* --------------------------------------------------------------------  */

    /* Derive the index of the status byte with the status array and also    */
    /* the bit index within the status byte from the buffer index.           */
    offset = (index & 0x3) << 1;
    buff  += (index >> 2);

    /* Set the status using the derived status byte and bit offset */
    *buff = (status << offset) | (*buff & (~(0x3 << offset)));
}

/* This function initializes static event queue structures */
INLINE void init_static_event_manager_q(UWORD8 qid, UWORD16 evnt_sz,
                                        UWORD16 max_evnts, UWORD8 *evnt_buff,
                                        UWORD8 *buff_status)
{
    UWORD16 cnt               = 0;
    UWORD8  *walign_evnt_buff = 0;

    /* Word-align the event buffer pointer if not aligned already */
    walign_evnt_buff = (UWORD8 *)(WORD_ALIGN((UWORD32)evnt_buff));

    g_event_q[qid].event_size = evnt_sz;
    g_event_q[qid].max_events = max_evnts;
    g_event_q[qid].tail_index = max_evnts - 1;
    g_event_q[qid].head_index = 0;
    g_event_q[qid].event_buf  = walign_evnt_buff;
    g_event_q[qid].buf_status = buff_status;
    g_event_q[qid].event_cnt  = 0;

    /* initialize status of the buffer of all event queues as free */
    for(cnt = 0; cnt < ((max_evnts >> 2) + 1); cnt++)
    {
        /* EVENT_BUFFER_FREE */
        *(buff_status + cnt) = 0x00;
    }

    TROUT_DBG5("======%s info: qid=%d======\n", __func__, qid);
    TROUT_DBG5("g_event_q[%d].event_size: %d\n", qid, g_event_q[qid].event_size);
    TROUT_DBG5("g_event_q[%d].max_events: %d\n", qid, g_event_q[qid].max_events);
    TROUT_DBG5("g_event_q[%d].tail_index: %d\n", qid, g_event_q[qid].tail_index);
    TROUT_DBG5("g_event_q[%d].head_index: %d\n", qid, g_event_q[qid].head_index);
    TROUT_DBG5("g_event_q[%d].event_buf: 0x%p\n", qid, g_event_q[qid].event_buf);
    TROUT_DBG5("g_event_q[%d].buf_status: 0x%p\n", qid, g_event_q[qid].buf_status);
    TROUT_DBG5("g_event_q[%d].event_cnt: %d\n\n", qid, g_event_q[qid].event_cnt);
}

/* This function gets the queue element from the head of a given queue and   */
/* update the status of the buffer                                           */
INLINE UWORD8* event_q_remove_from_head(event_q_struct_t *event_q)
{
    UWORD8 status      = 0;
    UWORD8 *head_event = 0;
    int count = 0;
    int smart_type = MODE_START;
	//chenq add 2012-11-01
	//spin_lock(&g_event_spin_lock);
    if(event_q->event_cnt == 0)
    {
        /* Event Queue is empty */
        return NULL;
    }

    smart_type =  critical_section_smart_start(0,1);

    //while(1)
    while(count < event_q->max_events)	//modify by chengwg.
    {
        status = check_buff_status(event_q->buf_status, event_q->head_index);
        if(status == EVENT_BUFFER_VALID)
        {
            /* Read out the event at the head of the queue. */
            head_event = (event_q->event_buf) +
                            (event_q->head_index * event_q->event_size);

            /* Move to the next event in the queue */
            event_q->head_index = INCR_RING_BUFF_INDX(event_q->head_index,
                                                      event_q->max_events);
            break;
        }
#if 0
        else if(status == EVENT_BUFFER_FREE)
        {
            /* If status is EVENT_BUFFER_FREE then move to the next event.   */
            /* This is required to handle out-of-order removal of events.    */
            event_q->head_index = INCR_RING_BUFF_INDX(event_q->head_index,
                                                      event_q->max_events);
        }
        else
            break;
#else
		else	//modify by chengwg.
        {
            event_q->head_index = INCR_RING_BUFF_INDX(event_q->head_index,
                                                      event_q->max_events);
        }
#endif            
        count++;
    }

    critical_section_smart_end(smart_type);
	//chenq add 2012-11-01
	//spin_unlock(&g_event_spin_lock);

    return head_event;
}

/* This function is used to update the status of the buffer of elements of   */
/* specified type of event                                                   */
INLINE void event_q_remove_all_misc_events(event_q_struct_t *qh,
                                           UWORD16 event_type)
{
    UWORD8 *buff_status_addrs = 0;
    UWORD16 index             = 0;
    UWORD8 *evnt_buff         = 0;
    int smart_type = 0;
	//chenq add 2012-11-01
	//spin_lock(&g_event_spin_lock);

    if(qh->event_cnt == 0)
    {
        return;
    }

    buff_status_addrs = qh->buf_status;
    index             = qh->head_index;
    evnt_buff         = qh->event_buf + (index * qh->event_size);

    smart_type = critical_section_smart_start(0,1);

    while(1)
    {
        misc_event_msg_t *temp = (misc_event_msg_t *)evnt_buff;

        /* check all the events of specified type */
        if(temp->name == event_type)
        {
            if(check_buff_status(buff_status_addrs, index) == EVENT_BUFFER_VALID)
            {
                /* update the status of buffer as FREE */
                update_buff_status(buff_status_addrs, index, EVENT_BUFFER_FREE);
                qh->event_cnt--;
            }
        }

        if(index == qh->tail_index)
        {
            break;
        }

        /* Move to the next event in the queue  */
        index = INCR_RING_BUFF_INDX(index, qh->max_events);

        evnt_buff = qh->event_buf + (index * qh->event_size);
    }

    critical_section_smart_end(smart_type);
	//chenq add 2012-11-01
	//spin_unlock(&g_event_spin_lock);
}

/* This function removes all Host TX events (Data/Config) from the given     */
/* Host TX queue (Data/Config). It also frees the packet buffer of the frame */
/* associated with the event.                                                */
INLINE void event_q_remove_all_host_tx_events(UWORD8 hif_qid)
{
    UWORD8 *buff_status_addrs = 0;
    UWORD16 index             = 0;
    UWORD8 *evnt_buff         = 0;
    int smart_type = 0;

	//chenq mod 2012-11-01
	event_q_struct_t *qh = NULL;//&g_event_q[hif_qid];

	//chenq add 2012-11-01
	//spin_lock(&g_event_spin_lock);
	qh = &g_event_q[hif_qid];
    

    if(qh->event_cnt == 0)
    {
        return;
    }

    buff_status_addrs = qh->buf_status;
    index             = qh->head_index;
    evnt_buff         = qh->event_buf + (index * qh->event_size);

	//chenq mod critical_section_start => critical_section_start1 2012-11-22
    smart_type = critical_section_smart_start(1,0);
	//critical_section_start();

    while(1)
    {
        host_tx_event_msg_t *temp = (host_tx_event_msg_t *)evnt_buff;

        /* check all the events of specified type */
        if(check_buff_status(buff_status_addrs, index) == EVENT_BUFFER_VALID)
        {
            pkt_mem_free(temp->fdesc.buffer_addr);

            update_buff_status(buff_status_addrs, index, EVENT_BUFFER_FREE);
            qh->event_cnt--;
        }

        if(index == qh->tail_index)
        {
            break;
        }

        /* Move to the next event in the queue  */
        index = INCR_RING_BUFF_INDX(index, qh->max_events);

        evnt_buff = qh->event_buf + (index * qh->event_size);
    }

	//chenq mod critical_section_end => critical_section_end1 2012-11-22
    critical_section_smart_end(smart_type);
	//critical_section_end();

	//chenq add 2012-11-01
	//spin_unlock(&g_event_spin_lock);
}

/* This function adds a Host TX Config event queue with given event buffer   */
/* space. It updates the event queue identifier in the given QID variable.   */
INLINE void add_host_ctx_event_q(UWORD32 *qid, UWORD8 *event_buff)
{
    (*qid) = (HOST_CTX_EVENT_QID_INIT + g_num_host_ctx_event_q);
    g_buffer_host_ctx_events_ptr[g_num_host_ctx_event_q] = event_buff;
    g_num_host_ctx_event_q += 1;
}

/* This function sets the number of Host TX Config event queues (which in    */
/* turn implies the number of host interfaces defined)                       */
INLINE void set_num_host_ctx_event_q(UWORD8 num)
{
    g_num_host_ctx_event_q = num;
}

/* This function returns the total number of active event queues based on    */
/* the number of Host TX Config event queues defined                         */
INLINE UWORD8 get_num_event_q(void)
{
    return (MIN_NUM_EVENT_QUEUES + g_num_host_ctx_event_q);
}

INLINE UWORD16 get_total_num_pending_events(void)
{
    UWORD8 qid  = 0;
    UWORD8 numq = get_num_event_q();
    UWORD16 tot_events = 0;

    for(qid = 0; qid < numq; qid++)
    {
        tot_events += g_event_q[qid].event_cnt;
    }

    return tot_events;
}

INLINE void flush_pending_events(void)	//add by chengwg.
{
    UWORD8 qid  = 0;
    UWORD8 numq = get_num_event_q();

    for(qid = 0; qid < numq; qid++)
    {
        g_event_q[qid].event_cnt = 0;
    }
}

/* This function returns the total number of pending events for the given    */
/* event queue if the same is defined.                                       */
INLINE UWORD16 get_num_pending_events_in_q(UWORD8 qid)
{
    if(qid >= get_num_event_q())
        return 0;

    return (g_event_q[qid].event_cnt);
}

/* This function gets an event from the given event queue */
INLINE UWORD8* get_event(UWORD8 qid)
{
    return (UWORD8*)event_q_remove_from_head(&g_event_q[qid]);
}

/* This function initializes all the MAC controller event queues */
INLINE void init_event_q(UWORD8 qid)
{
    if(qid == HOST_RX_EVENT_QID)
    {
        /* Initialize host rx event queues */
       init_static_event_manager_q(HOST_RX_EVENT_QID, HOST_RX_EVENT_MSG_SIZE,
                           MAX_NUM_HOST_RX_EVENTS,
                           g_buffer_host_rx_events + HOST_RX_EVENT_BUF_SIZE,
                           g_buffer_host_rx_events);
    }
    else if(qid == WLAN_RX_EVENT_QID)
    {
        /* Initialize wlan rx event queues */
       init_static_event_manager_q(WLAN_RX_EVENT_QID, WLAN_RX_EVENT_MSG_SIZE,
                           MAX_NUM_WLAN_RX_EVENTS,
                           g_buffer_wlan_rx_events + WLAN_RX_EVENT_BUF_SIZE,
                           g_buffer_wlan_rx_events);
    }
    else if(qid == MISC_EVENT_QID)
    {
        /* Initialize misc event queues */
        init_static_event_manager_q(MISC_EVENT_QID, MISC_EVENT_MSG_SIZE,
                           MAX_NUM_MISC_EVENTS,
                           g_buffer_misc_events + MISC_EVENT_BUF_SIZE,
                           g_buffer_misc_events);
    }
    else if(qid == HOST_DTX_EVENT_QID)
    {
        /* Initialize host data tx event queues */
        init_static_event_manager_q(HOST_DTX_EVENT_QID, HOST_TX_EVENT_MSG_SIZE,
                         MAX_NUM_HOST_DTX_EVENTS,
                         g_buffer_host_dtx_events + HOST_DTX_EVENT_BUF_SIZE,
                         g_buffer_host_dtx_events);
    }
    else if(qid < get_num_event_q()) /* HOST_CTX_EVENT_QID[Host Type] */
    {
        UWORD8 idx = (qid - HOST_CTX_EVENT_QID_INIT);

        /* Initialize host config tx event queues */
        init_static_event_manager_q(qid, HOST_TX_EVENT_MSG_SIZE,
                     MAX_NUM_HOST_CTX_EVENTS,
                     g_buffer_host_ctx_events_ptr[idx] + HOST_CTX_EVENT_BUF_SIZE,
                     g_buffer_host_ctx_events_ptr[idx]);
    }
}

INLINE void reset_all_event_q(void)	//add by chengwg, 2013.07.05
{
	UWORD8 i;
			
	for(i = 0; i < get_num_event_q(); i++)
    {
        init_event_q(i);
    }
}
#endif /* EVENT_MANAGER_H */
