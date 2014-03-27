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
/*  File Name         : qmu_if.h                                             */
/*                                                                           */
/*  Description       : This file contains all the definitions for queue     */
/*                      manager interface functions.                         */
/*                                                                           */
/*  List of Functions : init_mac_qmu                                         */
/*                      is_txq_full                                          */
/*                      get_txq_head_pointer                                 */
/*                      flush_all_qs                                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef QMU_IF_H
#define QMU_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "qmu.h"
#include "prot_if.h"
#include "qif.h"
#include "trout_share_mem.h"
#include "trout_wifi_rx.h"

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8 g_is_suspend_all_data_qs;// 20120709 caisf add, merged ittiam mac v1.2 code
extern BOOL_T g_rxq_replenish_reqd[NUM_RX_Q];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
#ifdef TROUT_TRACE_DBG
extern void tx_shareram_slot_stat(void);
#endif
extern UWORD8 qmu_add_rx_buffer(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num);
extern void   qmu_update_rx_q(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num,
                         UWORD32 *rx_dscr, UWORD8 rx_count);
extern void   qmu_sync_rx_packet(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num,
                                UWORD32 *rx_dscr);
extern UWORD8 qmu_add_tx_packet(qmu_tx_handle_t *tx_handle, UWORD8 q_num,
                         UWORD8 *tx_dscr);
extern UWORD8 qmu_add_tx_packet_list(qmu_tx_handle_t *tx_handle, UWORD8 q_num,
                              UWORD8 **dscr_head, UWORD8 **dscr_tail,
                              UWORD8 *num_dscr);
extern void   qmu_sync_tx_packet(qmu_tx_handle_t* tx_handle, UWORD8 q_num,
                                 UWORD8 *dscr);
extern UWORD8 qmu_del_tx_packet(qmu_tx_handle_t* tx_handle, UWORD8 q_num);
extern UWORD32 *qmu_find_pending_dscr(qmu_tx_handle_t* tx_handle, UWORD8 q_num);

extern void replenish_rx_queue(qmu_handle_t *q_handle, UWORD8 q_num);
extern void qmu_update_txq_chan_bw(qmu_tx_handle_t* tx_handle, UWORD8 q_num,
                            UWORD8 ch_mask, UWORD8 ch_band);

extern void trout_load_qmu(void);
extern void tx_shareram_manage_init(void);
extern void tx_shareram_slot_free(UWORD32 q_num);
extern int tx_shareram_busy(int q_num);
extern int tx_shareram_slot_valid(void);
extern int tx_shareram_wq_mount(void);
extern int tx_shareram_slot_busy(UWORD8 slot);
extern void tx_shareram_slot_packet_dec(UWORD8 slot);
extern void tx_pkt_process_new(UWORD8 slot, int call_flag);


/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/
INLINE int tx_shareram_slot_nr(void)
{
    return TX_SHARERAM_SLOTS;
}

/* Queue Manager Initialization function */
INLINE int init_mac_qmu(void)
{
	/* Initialize the RX Q-manager */
	if(init_mac_rx_queue(g_rx_handle) < 0)
	{
		TROUT_DBG1("%s: init rx queue fail!\n", __func__);
		return -1;
	}

	/* Initialize the TX Q-manager */
	qmu_init(&g_q_handle, get_no_tx_queues());
#ifdef TX_PKT_USE_DMA
	if(tx_dma_buf_init() < 0){
		/* don't forget to free the RX buffers  by zhao */
		host_rx_queue_free();
		return -1;
	}
#endif	/* TX_PKT_USE_DMA */
	create_trout_default_share_mem_cfg();
	return 0;
}

INLINE int reinit_mac_rxq(void)
{	
    set_machw_rx_buff_addr(0, NORMAL_PRI_RXQ);
    set_machw_rx_buff_addr(0, HIGH_PRI_RXQ);
    
    if(init_mac_rx_queue(g_rx_handle) < 0)
    {
		TROUT_DBG1("%s: init rx queue fail!\n", __func__);
		return -1;
	}
	return 0;
}

INLINE int reinit_tx_buf(qmu_handle_t *q_handle)
{
	qmu_tx_handle_t *tx_handle = &q_handle->tx_handle;
	UWORD32 trout_addr;
	
	trout_addr = g_normal_rxq_num * sizeof(trout_rx_mem_struct) + NORMAL_RX_MEM_BEGIN;
	if(trout_addr >= SHARE_MEM_END)
	{
		TROUT_DBG1("trout rx mem is so large, thus tx has no mem!\n");
		return -1;
	}
	
	tx_handle->tx_mem_start = trout_addr;
	tx_handle->tx_mem_size = (UWORD32)(SHARE_MEM_END - trout_addr);
	TROUT_DBG4("reinit tx: tx_start=0x%x, tx_size=%u\n", 
						tx_handle->tx_mem_start, tx_handle->tx_mem_size);	
	return 0;
}

#ifdef TROUT_TRACE_DBG

INLINE void print_qif_table_info(void)
{
	int i=0;

	printk("all txq info:\n");
	for(i=0; i<NUM_TX_QUEUE; i++)
	{
		printk("txq%d pointer value: 0x%x\n", i, host_read_trout_reg((UWORD32)(g_qif_table[i].addr)));
	}
}

INLINE void txq_handle_detail_show(void)
{
	qmu_tx_handle_t *tx_handle = &g_q_handle.tx_handle;
	//UWORD32 *tmp_dscr, *next_dscr;
	int i;

	TROUT_TX_DBG4("tx queue handle detail show:\n");
	TROUT_TX_DBG4("num_tx_q=%d, tx_curr_qnum=%d, tx_list_count=%d\n", g_q_handle.num_tx_q,
							tx_handle->tx_curr_qnum, tx_handle->tx_list_count);
	for(i=0; i<get_no_tx_queues(); i++)
	{
		if(tx_handle->tx_header[i].element_cnt == 0)
			continue;
		TROUT_TX_DBG5("tx queue pri(%d) handle info:\n", i);
		TROUT_TX_DBG5("element_cnt: %13d\n", tx_handle->tx_header[i].element_cnt);
		TROUT_TX_DBG5("element_head: %12p\n", tx_handle->tx_header[i].element_head);
		TROUT_TX_DBG5("element_tail: %12p\n", tx_handle->tx_header[i].element_tail);
		TROUT_TX_DBG5("element_to_load: %9p\n", tx_handle->tx_header[i].element_to_load);
		TROUT_TX_DBG5("q_status: %16d\n", tx_handle->tx_header[i].q_status);
	}

	TROUT_TX_DBG4("cur_slot=%u\n\n", tx_handle->cur_slot);
	tx_shareram_slot_stat();

	print_qif_table_info();
}
//		TROUT_TX_DBG4("handle_pkt:      %d\n", tx_handle->tx_header[i].handle_pkt);
#endif /* TROUT_TRACE_DBG */

INLINE BOOL_T is_this_txq_full_test(UWORD8 q_num)
{
    if(total_pending_pkt_cnt() >= MAX_PENDING_PKTS - 2)
    {
        if(g_q_handle.tx_handle.tx_header[q_num].element_cnt >
            get_max_txq_size(q_num))
        {
            return BTRUE;
        }
    }
    return BFALSE;
}

INLINE BOOL_T is_all_txq_full(void)
{
	if(total_pending_pkt_cnt() >= MAX_PENDING_PKTS - 2)
		return BTRUE;
	return BFALSE;
}
INLINE BOOL_T is_this_txq_full(UWORD8 q_num)
{
    /* Each Q shall be limited only if total number of pkts */
    /* pending in all Tx Qs & Rx Q exceed certain limit     */
    /* This is to make sure that throughput doesnt come down*/
    /* if only a single Q is running                        */
    //return BFALSE;

    if(total_pending_pkt_cnt() >= MAX_PENDING_PKTS)
    {
        if(g_q_handle.tx_handle.tx_header[q_num].element_cnt >
            get_max_txq_size(q_num))
        {
#ifdef DEBUG_MODE
            g_mac_stats.qfull[q_num]++;
#endif /* DEBUG_MODE */

            return BTRUE;
        }
    }
    return BFALSE;
}
/* This function checks whether the transmit queue is full or not. */
INLINE BOOL_T is_txq_full(UWORD8 q_num,UWORD8 sta_index)
{
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        if(is_qos_required(sta_index))
        {
            return is_this_txq_full(q_num);

        }
        return BFALSE;
    }
    else
    {
        if(get_wmm_enabled() == BTRUE)
        {
            return is_this_txq_full(q_num);
        }
        return BFALSE;
    }
}

/* This function returns the head pointer for a given transmit queue */
INLINE UWORD8 *get_txq_head_pointer(UWORD8 q_num)
{
    return (UWORD8 *)(g_q_handle.tx_handle.tx_header[q_num].element_head);
}

INLINE void flush_all_qs(void)
{
    UWORD16 temp = 0;

    /* Flush all transmit queues and free memory associated with transmit    */
    /* queues and cache.                                                     */
    for(temp = 0; temp < g_q_handle.num_tx_q; temp++)
    {
        qmu_flush_tx_queue(&g_q_handle, temp);
    }
}

/* This function destroys the existing Q-Manager */
INLINE void destroy_mac_qmu(qmu_handle_t *q_handle)
{
    qmu_close(q_handle);
}

/* This function replenishes the queues which were marked for updation */
INLINE void handle_rxq_replenishment(void)
{
    UWORD8 i = 0;

    for(i = 0; i < NUM_RX_Q; i++)
    {
        if(g_rxq_replenish_reqd[i] == BTRUE)
            replenish_rx_queue(&g_q_handle, i);
    }
}

/* This function suspends transmission of frames from the chosen queue. */
INLINE void suspend_tx_q(UWORD8 q_num, BOOL_T is_machw_already_suspended)
{
    UWORD8 *q_stat = &(g_q_handle.tx_handle.tx_header[q_num].q_status);

    if(Q_VALID == (*q_stat))
    {
        /* Suspend MAC HW before making Q headers to NULL */
        if(is_machw_already_suspended == BFALSE)
            set_machw_tx_suspend();

        set_machw_q_ptr(q_num, NULL);

        /* Resume MAC HW after the task */
        if(is_machw_already_suspended == BFALSE)
            set_machw_tx_resume();

        *q_stat = Q_SUSPENDED;
    }
}

/* This function resumes transmission of frames from the chosen queue. */
INLINE void resume_tx_q(UWORD8 q_num)
{
    UWORD8 *q_stat = &(g_q_handle.tx_handle.tx_header[q_num].q_status);

    if(Q_SUSPENDED == (*q_stat))
    {
        UWORD32 *dscr = qmu_find_pending_dscr(&g_q_handle.tx_handle, q_num);

        if(dscr != NULL)
        {
            set_machw_q_ptr(q_num, (UWORD32 *)dscr);
        }

        *q_stat = Q_VALID;
    }
}

/* This function suspends transmission of frames from all data queues. */
INLINE void suspend_all_data_qs(void)
{
    UWORD8 q_num = 0;

    /* Suspend MAC HW before suspending the individual queues */
    set_machw_tx_suspend();

    for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
    {
        if(q_num != HIGH_PRI_Q)
        {
            g_is_suspend_all_data_qs = 1;// 20120709 caisf add, merged ittiam mac v1.2 code
            suspend_tx_q(q_num, BTRUE);
        }
    }

    /* Resume MAC HW after the task */
    set_machw_tx_resume();
}

/* This function resumes transmission of frames from all data queues. */
INLINE void resume_all_data_qs(void)
{
    UWORD8 q_num = 0;

	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
	if(1 == g_is_suspend_all_data_qs)
	{
	    for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
	    {
	        if(q_num != HIGH_PRI_Q)
	        {
	            resume_tx_q(q_num);
	        }
		}

		g_is_suspend_all_data_qs = 0;
    }
}

//add by chengwg.
INLINE void disable_all_txq(void)
{
	UWORD8 q_num = 0;

	for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
		g_q_handle.tx_handle.tx_header[q_num].q_status = Q_SUSPENDED;
}


INLINE void enable_all_txq(void)
{
	UWORD8 q_num = 0;

	for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
		g_q_handle.tx_handle.tx_header[q_num].q_status = Q_VALID;
}

//add by zhao
INLINE void force_suspend_softtxq_above(UWORD32 q_given)
{
	UWORD8 q_num = 0;

	if(q_given > get_no_tx_queues()){
		printk("BUG: %s q_num out of range\n", __func__);
		return;
	}
	printk("TQNR=%d\n", q_given);
	for(q_num = 0; q_num < q_given; q_num++){
		g_q_handle.tx_handle.tx_header[q_num].q_status = Q_SUSPENDED;
	}
	for(q_num = q_given; q_num < get_no_tx_queues(); q_num++){
		g_q_handle.tx_handle.tx_header[q_num].q_status = Q_VALID;
	}
}

INLINE void force_resume_soft_txq_above(UWORD32 q_given)
{
	UWORD8 q_num = 0;

	if(q_given > get_no_tx_queues()){
		printk("BUG: %s q_num out of range\n", __func__);
		return;
	}
	for(q_num = 0; q_num < q_given; q_num++)
		g_q_handle.tx_handle.tx_header[q_num].q_status = Q_VALID;
}

/************  chenq add for scan state 2012-10-20 ************/
/* This function suspends transmission of frames from all data queues. */
INLINE void suspend_all_data_qs_not_suspend_hw(void)
{
    UWORD8 q_num = 0;
	UWORD8 *q_stat = NULL;

	TROUT_DBG6("chenq debug suspend_all_data_qs_not_suspend_hw  1\n");

    for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
    {
        if(q_num != HIGH_PRI_Q)
        {
            g_is_suspend_all_data_qs = 1;// 20120709 caisf add, merged ittiam mac v1.2 code
            q_stat = &(g_q_handle.tx_handle.tx_header[q_num].q_status);
            if(Q_VALID == (*q_stat))
			{
				*q_stat = Q_SUSPENDED;
            }	
        }
    }

}

/* This function resumes transmission of frames from all data queues. */
INLINE void resume_all_data_qs_not_resume_hw(void)
{
    UWORD8 q_num = 0;
	UWORD8 *q_stat = NULL;

	TROUT_DBG6("chenq debug resume_all_data_qs_not_resume_hw 1\n");

	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
	if(1 == g_is_suspend_all_data_qs)
	{
	    for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
	    {
	        if(q_num != HIGH_PRI_Q)
	        {
	            q_stat = &(g_q_handle.tx_handle.tx_header[q_num].q_status);
				TROUT_DBG6("chenq debug resume_all_data_qs_not_resume_hw 2\n");
				if(Q_SUSPENDED == (*q_stat))
				{
					TROUT_DBG6("chenq debug resume_all_data_qs_not_resume_hw 3\n");
					*q_stat = Q_VALID;
            	}
	        }
		}

		g_is_suspend_all_data_qs = 0;
    }
}

INLINE void suspend_all_data_qs_only_suspend_hw(void)
{
    UWORD8 q_num = 0;

    /* Suspend MAC HW before suspending the individual queues */
    set_machw_tx_suspend();

    for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
    {
        if(q_num != HIGH_PRI_Q)
        {
            g_is_suspend_all_data_qs = 1;// 20120709 caisf add, merged ittiam mac v1.2 code

			set_machw_q_ptr(q_num, NULL);
        }
    }

    /* Resume MAC HW after the task */
    set_machw_tx_resume();
}

INLINE void resume_all_data_qs_only_resume_hw(void)
{
    UWORD8 q_num = 0;

	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
	if(1 == g_is_suspend_all_data_qs)
	{
	    for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
	    {
	        if(q_num != HIGH_PRI_Q)
			{
		        UWORD32 *dscr = qmu_find_pending_dscr(&g_q_handle.tx_handle, q_num);

		        if(dscr != NULL)
		        {
		            set_machw_q_ptr(q_num, (UWORD32 *)dscr);
		        }
			}
		}

		g_is_suspend_all_data_qs = 0;
    }
}

/**************************************************************/


/* This function updates the Channel Bandwidth field of all the TX-Dscrs */
/* in all TX-Queues.                                                     */
INLINE void qmu_update_all_txq_chan_bw(UWORD8 ch_mask, UWORD8 ch_band)
{
    UWORD8 q_num = 0;
    BOOL_T tx_status = is_machw_tx_suspended();

    if(tx_status == BFALSE)
        set_machw_tx_suspend();

    for(q_num = 0; q_num < get_no_tx_queues(); q_num++)
    {
        qmu_update_txq_chan_bw(&g_q_handle.tx_handle, q_num, ch_mask, ch_band);
    }

    if(tx_status == BFALSE)
        set_machw_tx_resume();
}

INLINE void active_netif_queue(void)	//add by chengwg.
{
	if((!is_all_txq_full()) && netif_queue_stopped(g_mac_dev))
        netif_wake_queue(g_mac_dev);
}

#endif /* QMU_IF_H */
