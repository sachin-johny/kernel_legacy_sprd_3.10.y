/*******************************************************************************
* Copyright (c) 2011-2012,Spreadtrum Corporation
* All rights reserved.
* Filename: trout_wifi_rx.h
* Abstract: 
* 
* Version: 1.0
* Revison Log: 
* 	2012/5/08, Chengwg: Create this file.
* CVS Log: 
* 	$Id$
*******************************************************************************/
#ifndef _TROUT_WIFI_RX_H
#define _TROUT_WIFI_RX_H

#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include "itypes.h"
#include "trout_share_mem.h"
#include "event_manager.h"

#define TROUT_WIFI

//#define BLOCK_WRITE_CAP		16

#define HOST_RX_Q_NUM		2 /* 2 */

#define RX_BUF_POINT_OFFSET			7
#define RX_NEXT_DSCR_POINT_OFFSET	8
#define RX_PREV_DSCR_POINT_OFFSET	9

#define RX_MEM_MGMT_FLAG_OFFSET		10
#define MEM_IN_USE_FLAG				0x64727073	/* "sprd" */


typedef struct
{
	UWORD8 	trout_rx_dscr[RX_DSCR_LEN];	/* rx dscr is (40+4)byte */
	UWORD8	trout_rx_buf[RX_PACKET_SIZE]; 
}trout_rx_mem_struct;

/*
typedef struct
{
	trout_rx_mem_struct  	trout_normal_pri_rx_q[NUM_RX_BUFFS];
    trout_rx_mem_struct  	trout_high_pri_rx_q[NUM_HIPR_RX_BUFFS];
}trout_rx_q_struct;
*/

typedef struct
{
	UWORD32 base_dscr;
	UWORD8  num_dscr;
	struct list_head list;
}rx_int_info;


typedef struct
{
	UWORD8  queue_pri;		/* rx queue priority */
	UWORD8  cell_num;		/* host rx queue node number */
	UWORD32 base_dscr;
	UWORD8  num_dscr;
	UWORD8  left_dscr_num;	/* remain need transact dscr node num */
	UWORD32 *queue_head;	/* point to host rxq head */
	UWORD32 *queue_tail;	/* point to host rxq tail */
	UWORD32 *trout_head;	/* point to trout rxq head */
	UWORD32 *trout_tail;	/* point to trout rxq tail */
	UWORD32 *cur_ptr;		/* point to current host rx queue should search dscr */
	UWORD32 *trout_cur_ptr;	/* point to trout current need rcv pkt dscr */
	//UWORD16 trout_rxq_count;	/* total of current rxq dscr number */
	struct work_struct work;
	trout_rx_mem_struct *trout_q_tail;	/* point to real time trout tail */
	//spinlock_t spin_lock;
	struct mutex rx_mutex_lock;	//add by chengwg!
	struct list_head rx_list;
}__attribute__ ((packed))rx_queue_struct;


typedef struct
{
	rx_queue_struct rx_q_info[NUM_RX_Q];	//normal priority & high priority.
	struct workqueue_struct *rx_work_q;
}rx_queue_handle;

typedef struct
{
	UWORD32 *sub_head;
	UWORD32 *next_dscr;
	UWORD8   sub_num;
}trout_rxq_info;

#define MAX_HOST_RX_BUF_SIZE	(MAX_NUM_RX_BUFFS * sizeof(trout_rx_mem_struct) * HOST_RX_Q_NUM)


extern UWORD16 g_normal_rxq_num;
extern rx_queue_handle *g_rx_handle;

INLINE int get_num_of_normal_rx_pkt(void)
{
	return g_normal_rxq_num;
}

INLINE void set_num_of_normal_rx_pkt(UWORD16 num)
{
	g_normal_rxq_num = num;
}

int init_mac_rx_queue(rx_queue_handle *rx_q_handle);
int init_trout_high_rx_queue(UWORD32 *h_rx_mem_s);
int init_trout_normal_rx_queue(UWORD32 *h_rx_mem_s);
void rx_complete_isr(UWORD8 q_num);
UWORD32 * free_host_rx_dscr_list(UWORD32 *base_dscr, UWORD8 num_dscr);
void dma_rx_complete_isr(UWORD8 q_num);
void process_wlan_rx(mac_struct_t *mac, UWORD8 *msg);
void host_rx_queue_free(void);
void flush_all_rxq(void);
void create_trout_default_share_mem_cfg(void);

INLINE void set_host_rxds_buffer_ptr(UWORD32 *dsc, UWORD32 *vir)
{
	UWORD32 *p = dsc + 7;
	
	*p = (UWORD32)vir;
}


INLINE void set_host_rxds_next_dscr(UWORD32 *dsc, UWORD32 *vir)
{
	UWORD32 *p = dsc + 8;
	
	*p = (UWORD32)vir;
}

INLINE void set_host_rxds_prev_dscr(UWORD32 *dsc, UWORD32 *vir)
{
	UWORD32 *p = dsc + 9;
	
	*p = (UWORD32)vir;
}


INLINE UWORD32 *get_host_rxds_buffer_ptr(UWORD32 *dsc)
{
	UWORD32 *p = dsc + 7;
	/* when CPU do write and read, needn't voliate by zhao */
	return (UWORD32 *)(*p);
}


INLINE UWORD32 *get_host_rxds_next_dscr(UWORD32 *dsc)
{
	UWORD32 *p = dsc + 8;
	/* when CPU do write and read, needn't voliate by zhao */
	return (UWORD32 *)(*p);
}

INLINE UWORD32 *get_host_rxds_prev_dscr(UWORD32 *dsc)
{
	UWORD32 *p = dsc + 9;
	/* when CPU do write and read, needn't voliate by zhao */
	return (UWORD32 *)(*p);
}

INLINE void clean_rx_dscr_pkt_status(UWORD32 *rx_dscr)
{
	set_dscr(RX_DSCR_FIELD_PREV_DSCR_OFFSET, FIELD_WIDTH_4, (rx_dscr + WORD_OFFSET_0), 0); 
}

INLINE void set_trout_next_dscr(UWORD32 *trout_dscr, UWORD32 value)
{
	UWORD32 h_addr[3];

	h_addr[2] = value;
	host_write_trout_ram((void *)&trout_dscr[RX_NEXT_DSCR_POINT_OFFSET], 
										(void *)&h_addr[2], sizeof(UWORD32));
}

INLINE UWORD32 get_trout_next_dscr(UWORD32 *trout_dscr)
{
	UWORD32 value = 0;
	
	host_read_trout_ram(&value, &trout_dscr[RX_NEXT_DSCR_POINT_OFFSET], sizeof(UWORD32));
	return value;
}

INLINE void set_trout_prev_dscr(UWORD32 *trout_dscr, UWORD32 value)
{
	UWORD32 h_addr[3];

	h_addr[2] = value;
	host_write_trout_ram((void *)&trout_dscr[RX_PREV_DSCR_POINT_OFFSET], 
										(void *)&h_addr[2], sizeof(UWORD32));
}

INLINE UWORD32 get_trout_prev_dscr(UWORD32 *trout_dscr)
{
	UWORD32 value = 0;
	
	host_read_trout_ram(&value, &trout_dscr[RX_PREV_DSCR_POINT_OFFSET], sizeof(UWORD32));
	return value;
}

INLINE void rx_mem_init_user(trout_rx_mem_struct *pkt_dscr, UWORD32 val)
{
	UWORD32 *rx_dscr = (UWORD32 *)(pkt_dscr->trout_rx_dscr);
	
	rx_dscr[RX_MEM_MGMT_FLAG_OFFSET] = val;
	TROUT_RX_DBG5("%s: INIT user dscr:%p num:%d\n", __FUNCTION__, 
		rx_dscr, rx_dscr[RX_MEM_MGMT_FLAG_OFFSET]);
}

INLINE void rx_mem_add_user(trout_rx_mem_struct *pkt_dscr)	//add by chengwg, debug.
{
	UWORD32 *rx_dscr = (UWORD32 *)(pkt_dscr->trout_rx_dscr);

	rx_dscr[RX_MEM_MGMT_FLAG_OFFSET] += 1;
	TROUT_RX_DBG5("%s: ADD user dscr:%p num:%d\n", __FUNCTION__, 
		rx_dscr, rx_dscr[RX_MEM_MGMT_FLAG_OFFSET]);
}

INLINE void rx_mem_dec_user(trout_rx_mem_struct *pkt_dscr)	//add by chengwg, debug.
{
	UWORD32 *rx_dscr = (UWORD32 *)(pkt_dscr->trout_rx_dscr);

	rx_dscr[RX_MEM_MGMT_FLAG_OFFSET] -= 1;
	TROUT_RX_DBG5("%s: DEC user dscr:%p num:%d\n", __FUNCTION__, 
		rx_dscr, rx_dscr[RX_MEM_MGMT_FLAG_OFFSET]);
}

INLINE void rx_mem_free(trout_rx_mem_struct *pkt_dscr)		//add by chengwg, debug.
{	
	UWORD32 *rx_dscr = (UWORD32 *)(pkt_dscr->trout_rx_dscr);
	
	if(0 == rx_dscr[RX_MEM_MGMT_FLAG_OFFSET])
	{
		TROUT_RX_DBG5("%s: CLEAN user dscr:%p\n", __FUNCTION__, 	rx_dscr);
		clean_rx_dscr_pkt_status(rx_dscr);		
	}
}

INLINE void rx_dscr_list_add_user(UWORD32 *rx_dscr, UWORD8 num_dscr)	//add by chengwg, debug.
{
	trout_rx_mem_struct *pkt_dscr = (trout_rx_mem_struct *)rx_dscr;
	UWORD32 *dscr = NULL;
	
    	while(num_dscr--)
    	{
        	if(pkt_dscr == NULL)
            		return;

		rx_mem_add_user(pkt_dscr);
		dscr = (UWORD32 *)pkt_dscr->trout_rx_dscr;
        	pkt_dscr = (trout_rx_mem_struct *)get_rx_dscr_next_dscr(dscr);
    	}
}

#ifdef TROUT_TRACE_DBG
void rx_share_ram_show(void);
void host_rx_buff_show(void);
#endif	/* TROUT_TRACE_DBG */

#ifdef TROUT_B2B_TEST_MODE
int trout_b2b_rx_low(mac_struct_t *mac, UWORD8 *msg);
#endif

#endif  /* _TROUT_WIFI_RX_H */

