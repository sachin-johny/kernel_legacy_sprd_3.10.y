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
/*  File Name         : pm_ap.c                                              */
/*                                                                           */
/*  Description       : This file contains all the power management related  */
/*                      functions, that are specific to AP mode of           */
/*                      operation.                                           */
/*                                                                           */
/*  List of Functions : is_ps_buff_pkt_ap                                    */
/*                      requeue_ps_packet                                    */
/*                      check_ps_state                                       */
/*                      tx_null_frame                                        */
/*                      handle_ps_poll                                       */
/*                      set_tim_bit                                          */
/*                      check_and_reset_tim_bit                              */
/*                      handle_ps_tx_comp_ap                                 */
/*                      handle_requeue_pending_packet                        */
/*                      check_pending_requeue_packet                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate_ap.h"
#include "mh.h"
#include "pm_ap.h"
#include "qmu_if.h"
#include "host_if.h"
#include "receive.h"
#include "core_mode_if.h"

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

static const UWORD8  g_bmap[8] = {1, 2, 4, 8, 16, 32, 64, 128}; /* Bit map */

extern UWORD32 g_num_ps_pkt;
extern UWORD16 g_num_mc_bc_qd_pkt;
extern asoc_entry_t *g_max_ps_ae;

/* export the functions by zhao 6-21 2013 */
extern struct mutex  vbp_mutex;
extern void get_vbp_mutex(unsigned long owner);
extern void put_vbp_mutex(void);

#ifdef ENABLE_PS_PKT_FLUSH
void check_ps_queue_flush(void);
PS_FLUSH_STATUS_T flush_one_ps_packet(asoc_entry_t *ae, list_buff_t *qh,
                                    BOOL_T ps_q_legacy, BOOL_T is_bcmc_pkt);
#endif /* ENABLE_PS_PKT_FLUSH */
/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_ps_buff_pkt_ap                                        */
/*                                                                           */
/*  Description   : This function checks if a packet requries buffering      */
/*                  based on the power save state of the destination.        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the association table                      */
/*                  2) Pointer to the destination address                    */
/*                  3) Pointer to the Q-number                               */
/*                  4) Pointer to the Q-head to which the packet must be q-d */
/*                                                                           */
/*  Globals       : g_mc_q                                                   */
/*                  g_num_mc_bc_pkt                                          */
/*                  g_num_sta_ps                                             */
/*                                                                           */
/*  Processing    : In case the packet is a broadcast/multicast packet, the  */
/*                  global g_num_sta_ps is checked to determine if any of    */
/*                  the associated stations are in power save mode.          */
/*                  In case of a unicast packet the association              */
/*                  entry is checked to determine the power save state of    */
/*                  the station. The queue limits are checked and if the     */
/*                  maximum is reached the queue header pointer is set to    */
/*                  zero so that the calling function can free the buffer.   */
/*                  Otherwise the queue details are set appropriately.       */
/*                                                                           */
/*  Outputs       : Q-Num to which the packet must be queued and the Queue   */
/*                  pointer to which the packet must be queued.              */
/*                                                                           */
/*  Returns       : BTRUE, if the packet should be buffered for power save   */
/*                  BFALSE, if the packet should not be buffered             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_ps_buff_pkt_ap(asoc_entry_t *ae, UWORD8 *da, void *dscr)
{
    BOOL_T  ps_del_en_ac    = BFALSE;
    BOOL_T  bc_mc_pkt       = is_group(da);
    UWORD8  priority        = 0;
    list_buff_t *qh         = NULL;

#ifdef ENABLE_PS_PKT_FLUSH
    check_ps_queue_flush();
#endif /* ENABLE_PS_PKT_FLUSH */

    /* check if pkt is BC/MC or Unicast */
    if(bc_mc_pkt == BTRUE)
    {
        if(g_num_sta_ps == 0)
        {
            /* All stations are in Active mode. No buffering required. */
            return BFALSE;
        }

        /* If the global BC/MC queue has not reached the maximum size */
        /* received BC/MC packet should be queued in it. Else packet  */
        /* should be dropped              */

        if(g_num_ps_pkt < PS_PKT_Q_MAX_LEN)
        {
            qh = &g_mc_q;
        }
    }
    else
    {
        if(ae == 0)
        {
            /* Exception. Should not occur. */
            return BFALSE;
        }

        if(ae->ps_state == ACTIVE_PS)
        {
            /* Station is in Active mode. No buffering required. */
            return BFALSE;
        }

        priority     = get_tx_dscr_priority((UWORD32 *)dscr);
        ps_del_en_ac = check_ac_is_del_en_prot(ae, priority);

        if(g_num_ps_pkt < PS_PKT_Q_MAX_LEN)
        {
            qh = (list_buff_t *)get_ps_q_ptr(ae, ps_del_en_ac);

        }
    }


    if( qh != NULL)
    {
        add_list_element(qh, dscr);
        update_ps_flags_ap(ae, bc_mc_pkt, 1, ps_del_en_ac);
    }
    else
    {
        free_tx_dscr((UWORD32 *)dscr);
    }

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_ps_flags_ap                                       */
/*                                                                           */
/*  Description   : This function updates the flags after a packet in enqued */
/*                  in the power save buffer                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the association table for the packet       */
/*                  2) Pointer to the destination address of the packet      */
/*                                                                           */
/*  Globals       : g_mc_q                                                   */
/*                  g_num_mc_bc_pkt                                          */
/*                  g_num_sta_ps                                             */
/*                                                                           */
/*  Processing    : This function updates the flags after a packet in enqued */
/*                  in the power save buffer                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_ps_flags_ap(asoc_entry_t *ae, BOOL_T bc_mc_pkt,
                        UWORD8 num_buff_added, BOOL_T ps_add_del_ac)
{
    if(num_buff_added == 0)
        return;

    /* the TIM bit should be protected! by zhao 6-21 2013 */
	get_vbp_mutex(__builtin_return_address(0));	
    if(bc_mc_pkt == BTRUE)
    {
        /* The global count for queued BC/MC packets is incremented here.    */
        /* This count is checked for resetting the AID0 bit in the TIM       */
        /* element once all buffered BC/MC packets have been transmitted. It */
        /* is decremented in Tx complete for BC/MC packets.                  */
        g_num_mc_bc_pkt += num_buff_added;
        g_num_ps_pkt++;

        /* Set the AID0 bit. This is reset BC/MC packet queue becomes empty. */
        /* It is updated in transmit complete processing.                    */
        set_dtim_bit(AID0_BIT);
        set_dtim_bit(DTIM_BIT);
		// 20120709 caisf add, merged ittiam mac v1.2 code
		if(g_update_active_bcn == 1){
			UWORD8 hw_bcn = (g_beacon_index + 1)%2;
			set_mc_bit_bcn(hw_bcn);
		}
    }
    else
    {
        if(BTRUE == update_ps_counts(ae, num_buff_added, ps_add_del_ac)){
         	put_vbp_mutex();
	   set_tim_bit(ae->asoc_id);
		get_vbp_mutex(__builtin_return_address(0));	
	}
    }
	put_vbp_mutex();	
}


#ifdef ENABLE_PS_PKT_FLUSH

/* This function checks if a packet from any PS queue is to be dropped to */
/* make way for new packets                                               */
void check_ps_queue_flush(void)
{
    /* If total number of packets in PS queues (BC/MC + All Legacy and    */
    /* Delivery enabled queues) is beyond a threshold, find the PS queue  */
    /* with highest number of packets.                                    */
    /* If BC/MC PS queue, do not drop the packet, but enqueue it in       */
    /* HW queue. If Legacy or Deliver enabled PS queue, drop the packet.  */
    if(g_num_ps_pkt >= PS_PKT_Q_MAX_LEN)
    {
        if(g_max_ps_ae != NULL)
        {
			if(g_max_ps_ae->num_ps_pkt_lgcy >= g_num_mc_bc_pkt)
			{
				// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
				/* Remove an older Unicast pkt from Q and delete it */
				flush_one_ps_packet(g_max_ps_ae, get_any_ps_q_ptr(g_max_ps_ae),
					BTRUE, BFALSE);
#else
                list_buff_t *qh = get_any_ps_q_ptr(g_max_ps_ae);
                BOOL_T lgcy_ps_q = (qh == &(g_max_ps_ae->ps_q_lgcy)) ?
                                   BTRUE : BFALSE;

				/* Remove an older Unicast pkt from Q and delete it */
                flush_one_ps_packet(g_max_ps_ae, qh, lgcy_ps_q, BFALSE);
#endif
			}
			else
			{
				/* Remove an older BC/MC pkt from Q and send it to air */
				requeue_ps_packet(NULL, &g_mc_q, BTRUE, BTRUE);
			}
		}
		else
		{
			/* Remove an older BCMC pkt from Q and send it to air */
			requeue_ps_packet(NULL, &g_mc_q, BTRUE, BTRUE);
		}
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : flush_one_ps_packet                                      */
/*                                                                           */
/*  Description   : This function deletes the packet after removing it from  */
/*                  the S/w queue where  it was buffered for power management*/
/*                                                                           */
/*  Inputs        : 1) Pointer to head of the queue                          */
/*                  2) Packet count associated with the queue                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The power save queue element buffer is freed after the   */
/*                  packet is removed from the head of the given ps queue    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : PKT_FLUSHED, if a packet was found in the queue          */
/*                  and flushed successfully                                 */
/*                  NO_PKT_IN_QUEUE, if no packet found in the given queue   */
/*                  PKT_NOT_FLUSHED, if not successfull for some reason      */
/*                  END_OF_QUEUE, if it is the last pkt in the queue         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
PS_FLUSH_STATUS_T flush_one_ps_packet(asoc_entry_t *ae, list_buff_t *qh,
                                    BOOL_T ps_q_legacy, BOOL_T is_bcmc_pkt)
{
    UWORD32 *tx_dscr = 0;

    /* Check of association entry in case of a unicast frame */
    if((is_bcmc_pkt == BFALSE) && (ae == NULL))
    {
        PRINTD("Error: PS Q AE null\n\r");
        return PKT_NOT_FLUSHED;
    }

    /* If h/w queue is not available pkt is not requeued */
    if(NULL == qh)
    {
        PRINTD("Warning: PS Q Qh NULL\n\r");
        return NO_PKT_IN_FLUSH_QUEUE;
    }

    /* If h/w queue is not available pkt is not requeued */
    if(qh->head == 0)
    {
        PRINTD("Warning: No packet in PS Q\n\r");
        return NO_PKT_IN_FLUSH_QUEUE;
    }

    /* Remove the power save queue element structure from the head of the    */
    /* given queue.                                                          */

    tx_dscr = remove_list_element_head(qh);

    if(NULL == tx_dscr)
    {
        PRINTD("Error: PS Q element null\n\r");
        return NO_PKT_IN_FLUSH_QUEUE;
	}


    /* Set the transmit rate to the required value. To support Multi-Rate  */
    /* the transmit rate is set to the minimum basic rate and the preamble */
    /* is set to Long.                                                     */
	if(is_bcmc_pkt == BTRUE)
    {
        /* Decrement Broadcast/Muticast Sw PS buffer count */
        g_num_mc_bc_pkt--;

    }
    else
    {
		// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
        /* Decrement Unicast Sw PS buffer count */
        ae->num_ps_pkt_lgcy--;
#else
        /* Decrement Unicast Sw PS buffer count */
        if(ps_q_legacy == BTRUE)
        	ae->num_ps_pkt_lgcy--;
#ifdef MAC_WMM
        else
            ae->num_ps_pkt_del_ac--;
#endif /* MAC_WMM */
#endif
    }
        g_num_ps_pkt--;
    free_tx_dscr(tx_dscr);

    /* If a packet was removed and requeued and it is the last packet then */
    /* return END_OF_QUEUE else return PKT_FLUSHED */
    if(peek_list(qh) == 0)
    {
        return END_OF_FLUSH_QUEUE;
    }
    else
    {
        return PKT_FLUSHED;
    }

}
#endif /* ENABLE_PS_PKT_FLUSH */
/*****************************************************************************/
/*                                                                           */
/*  Function Name : requeue_ps_packet                                        */
/*                                                                           */
/*  Description   : This function requeues a packet in the H/w queue for     */
/*                  transmission after removing it from the S/w queue where  */
/*                  it was buffered for power management purpose.            */
/*                                                                           */
/*  Inputs        : 1) Association entry for the station                     */
/*                  2) PS queue (legacy or del_en) pointer                   */
/*                  3) Flag indicating Legacy PS queue                       */
/*                  4) Flag indicating End of service period                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : First the H/w queue is checked to see if it is full.     */
/*                  If H/w queue is full then pkt is not requeued.           */
/*                  If H/w queue is available then pkt is requeued.          */
/*                  A packet is removed from the head of the given power     */
/*                  save queue. The transmit descriptor is extracted and     */
/*                  added to the H/w queue. The power save queue element     */
/*                  buffer is freed.                                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : PKT_REQUEUED, if a packet was found in the queue         */
/*                  and requeued                                             */
/*                  NO_PKT_IN_QUEUE, if no packet found in the given queue   */
/*                  PKT_NOT_REQUEUED, if h/w queue is full                   */
/*                  END_OF_QUEUE, if it is the last pkt in the queue         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

REQUEUE_STATUS_T requeue_ps_packet(asoc_entry_t *ae, list_buff_t *qh,
                                   BOOL_T ps_q_legacy, BOOL_T eosp)
{
    UWORD8   q_num        = 0;
    UWORD8   *tx_dscr     = 0;
    UWORD8   *mac_hdr     = 0;
    UWORD8   addr1[6]     = {0};
    UWORD8   tx_rate      = 0;
    UWORD8   pream        = 0;
    UWORD8   srv_cls      = 0;
    UWORD8   tid          = 0;
    UWORD32  phy_tx_mode  = 0;
    BOOL_T   is_qos       = BFALSE;
    UWORD32  retry_set[2] = {0};

	TROUT_FUNC_ENTER;
    /* Remove the power save queue element structure from the head of the    */
    /* given queue.                                                          */
    tx_dscr = remove_list_element_head(qh);

    /* If the queue has no packets return BFALSE to indicate that no packets */
    /* were available to be requeued.                                        */
    if(tx_dscr == 0)
    {
    	TROUT_FUNC_EXIT;
        return NO_PKT_IN_QUEUE;
	}

    /* Extract the pointer to the MAC header from the Tx-Descriptor */
    mac_hdr = (UWORD8 *)get_tx_dscr_buffer_addr((UWORD32 *)tx_dscr) +
                                    get_tx_dscr_mh_offset((UWORD32 *)tx_dscr);

    /* Extract the Q-number from the Tx-descriptor */
    q_num = get_tx_dscr_q_num((UWORD32 *)tx_dscr);

    /* Extract DA, TID and QoS from the MAC header of the MSDU */
    get_address1(mac_hdr, addr1);
    tid = get_priority_value(mac_hdr);
    is_qos = is_qos_bit_set(mac_hdr);

    if(is_group(addr1) == BTRUE)
    {
        tx_rate     = get_max_basic_rate();
        pream       = 1;
        update_retry_rate_set(0, tx_rate, 0, retry_set);
    }
    else
    {
	// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
        tx_rate     = get_tx_rate_to_sta(ae);
#else
	    if(get_type(mac_hdr) == MANAGEMENT || !ae)
		{
			tx_rate = get_max_basic_rate();
		}
		else
		{
        	tx_rate     = get_tx_rate_to_sta(ae);
		}
#endif
        pream       = get_preamble(tx_rate);
	    if(!ae){
		    /* what ever if we reach here, we must return, or will get kernel panic */
		    free_tx_dscr((UWORD32 *)tx_dscr);
		    return RE_Q_ERROR;
	    }
        update_retry_rate_set(1, tx_rate, ae, retry_set);
    }

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)ae);

    /* Update the transmit parameters */
    srv_cls = get_serv_class(ae, addr1, tid);
    set_tx_params(tx_dscr, tx_rate, pream, srv_cls, phy_tx_mode, retry_set);

    /* Set the HT power save parameters */
    set_ht_ps_params(tx_dscr, (void *)ae, tx_rate);

    /* Set the receiver address LUT index */
    /* This should be set to NULL during PS */
    set_ht_ra_lut_index(tx_dscr, NULL, tid, tx_rate);

    /* Peek and check if the queue is empty. If so, set the more data bit to */
    /* 0. Otherwise set it to 1.                                             */
    if(peek_list(qh) == NULL)
    {
        if(is_qos == BTRUE)
            set_qos_prot(mac_hdr);

        set_more_data(mac_hdr, 0);
    }
    else
    {
        set_more_data(mac_hdr, 1);
    }

#if 0
    /* Currently disabled */
    /* If current sp length = max sp length-1 eosp bit set and END_OF_QUEUE  */
    /* is returned                                                           */
    if(ps_q_legacy == BFALSE)
    {
        if(is_end_prot(ae) == BTRUE)
        {
            set_qos_prot(mac_hdr);
            end_of_q = BTRUE;
        }
    }
#endif

    /* If current sp length = max sp length-1 eosp bit set and END_OF_QUEUE  */
    /* is returned                                                           */
    if(BTRUE == eosp)
        set_qos_prot(mac_hdr);

    /* Update the TSF timestamp in the Tx-desc */
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    /* Update PS queue counters */
    if(ps_q_legacy == BTRUE)
    {
    if(is_group(addr1) == BTRUE)
        {
            g_num_mc_bc_pkt--;
        }
        else
        {
            ae->num_ps_pkt_lgcy--;
        }
	}
	// 20120709 caisf add the "ifdef", merged ittiam mac v1.2 code
#ifdef MAC_WMM
	else
	{
	    ae->num_ps_pkt_del_ac--;
	}
#endif /* MAC_WMM */
	g_num_ps_pkt--;

    /* Check if the packet needs to be buffered for BA. If Block ACK session */
    /* is active then donot queue in the HW queue.                           */
    /* When STA has come out of sleep then BA is !HALTED                     */

    if(BFALSE == is_serv_cls_buff_pkt((UWORD8 *)ae, q_num, tid, tx_dscr))
    {
        /* Queue the frame for transmission */
        if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
        {
            /* Exception. Do nothing. */
#ifdef DEBUG_MODE
            g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
            free_tx_dscr((UWORD32 *)tx_dscr);
			TROUT_FUNC_EXIT;
            return RE_Q_ERROR;
        }

		/* Update UC/BC/MC Hw queue packet count */
        if(is_group(addr1) == BTRUE)
        {
            g_num_mc_bc_qd_pkt++;
        }
        else
        {
            ae->num_qd_pkt++;
        }

    }

	TROUT_FUNC_EXIT;
    return PKT_REQUEUED;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_ps_state                                           */
/*                                                                           */
/*  Description   : This function checks the power save state of the station */
/*                  from all received packets.                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to association entry                          */
/*                  2) Power Save value                                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The power management bit in the received packet is       */
/*                  checked and the association entry of the corresponding   */
/*                  transmitting station is updated with the power save      */
/*                  state. Also if there is any change in power save state   */
/*                  the appropriate handling function is called.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void check_ps_state(asoc_entry_t* ae, STA_PS_STATE_T ps)
{
    /* 1 - POWER_SAVE, 0 - ACTIVE */
    if((ae == NULL ) || (ae->ps_state == ps))
    {
        /* There is no change in station power save mode. Do nothing. */
        return;
    }

    /* Station has changed power save state. Update the entry and handle the */
    /* change as required.                                                   */
    ae->ps_state = ps;

    if(ps == ACTIVE_PS)
    {
        handle_ps_sta_change_to_active(ae);
    }
    else
    {
        handle_ps_sta_change_to_powersave(ae);
    }

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_null_frame                                            */
/*                                                                           */
/*  Description   : This function prepares and sends a NULL frame to the     */
/*                  given station.                                           */
/*                                                                           */
/*  Inputs        : 1) Address of station to which NULL frame is directed    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The NULL frame is prepared and added to the H/w queue    */
/*                  with the required descriptor.                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void tx_null_frame(UWORD8 *sa, asoc_entry_t *ae, BOOL_T is_qos, UWORD8 priority,
                   UWORD8 more_data)
{
    UWORD8  q_num          = 0;
    UWORD8  tx_rate        = 0;
    UWORD8  pream          = 0;
    UWORD8  *tx_dscr       = 0;
    UWORD8  *msa           = 0;
    UWORD8  len            = 0;
    UWORD32 phy_tx_mode    = 0;
    UWORD32 retry_set[2]   = {0};

	TROUT_FUNC_ENTER;
    /* Allocate buffer for the NULL frame and set its contents */
    msa = (UWORD8*)mem_alloc(g_shared_pkt_mem_handle, MANAGEMENT_FRAME_LEN);
    if(msa == NULL)
    {
        TROUT_DBG2("No Mem for NULL frame\n\r");
        TROUT_FUNC_EXIT;
        return;
    }

    if(is_qos == BTRUE)
    {
        len = set_frame_ctrl_qos_null_ap(msa, priority, 0);
        q_num = get_txq_num(priority);
    }
    else
    {
        set_frame_control(msa, (UWORD16)NULL_FRAME);
        len = MAC_HDR_LEN + FCS_LEN;
        /* NULL frames will be put in Normal priority queue */
        q_num = NORMAL_PRI_Q;
    }

    /* Set the from ds bit */
    set_from_ds(msa, 1);

    /* Set the address fields */
    set_address1(msa, sa);
    set_address2(msa, mget_StationID());
    set_address3(msa, mget_StationID());

    /* Get the transmit rate for the associated station based on the         */
    /* auto-rate, multi-rate or user-rate settings. The preamble must be     */
    /* set accordingly.                                                      */
    tx_rate     = get_tx_rate_to_sta(ae);
    pream       = get_preamble(tx_rate);

    /* Update the retry set information for this frame */
    update_retry_rate_set(1, tx_rate, ae, retry_set);

    /* Get the PHY transmit mode based on the transmit rate and preamble */
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)ae);

    /* Create the transmit descriptor and set the contents */
	// 20120709 caisf add, merged ittiam mac v1.2 code
    //tx_dscr = create_default_tx_dscr(0, 0, 0);
    tx_dscr = create_default_tx_dscr(is_qos, priority, 0);
    if(tx_dscr == NULL)
    {
		PRINTD2("No Mem for NULL Tx DSCR\n");
        pkt_mem_free(msa);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Set various transmit descriptor parameters */
    set_tx_params(tx_dscr, tx_rate, pream, NORMAL_ACK, phy_tx_mode, retry_set);
    set_tx_buffer_details(tx_dscr, msa, 0, len-FCS_LEN, 0);
    set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);
    /* This is already done in create_default_tx_dscr */
    /* set_tx_security(tx_dscr, NO_ENCRYP, 0, se->sta_index); */
    set_ht_ps_params(tx_dscr, (void *)ae, tx_rate);
    set_ht_ra_lut_index(tx_dscr, NULL, 0, tx_rate);
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);


    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
#ifdef DEBUG_MODE
        g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */

        /* Exception. Free the transmit descriptor and packet buffers if it  */
        /* cannot be added to the H/w queue.                                 */
        free_tx_dscr((UWORD32 *)tx_dscr);
    }
    else if(ae != NULL)
    {
        ae->num_qd_pkt++;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_ps_poll                                           */
/*                                                                           */
/*  Description   : This function handles a received PS-Poll frame in the    */
/*                  access point mode.                                       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the PS-Poll frame                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if a previous PS-Poll frame is yet  */
/*                  to be processed (checks a flag in the association entry) */
/*                  If not, the first frame from the power save queue is     */
/*                  requeued. If requeuing is not successful, a null frame   */
/*                  is transmitted to the station.                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T, BTRUE, If it was a PS Poll frame                 */
/*                          BFALSE, otherwise                                */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T handle_ps_poll(UWORD8 *msg)
{
    wlan_rx_t    *wlan_rx  = (wlan_rx_t *)msg;
    asoc_entry_t *ae       = (asoc_entry_t*)(wlan_rx->sa_entry);

	TROUT_FUNC_ENTER;
    /* Check if the frame is a PS Poll frame and return BFALSE if not */
    if(wlan_rx->sub_type != PS_POLL)
    {
		TROUT_FUNC_EXIT;
        return BFALSE;
	}

    /* PS-Poll frames are processed only if the station is associated to AP. */
    if((ae == 0) || (ae->state != ASOC))
    {
        /* Send a De-authentication Frame to the station as PS-Poll is a     */
        /* Class 3 frame.                                                    */
        send_deauth_frame(wlan_rx->sa, (UWORD16)CLASS3_ERR);

        /* No further processing is required */
        TROUT_FUNC_EXIT;
        return BTRUE;
    }

    if(ae->ps_poll_rsp_qed == BTRUE)
    {
        /* If a frame has been queued in response to a previous PS Poll      */
        /* received, further PS Poll frames will not be honoured till the    */
        /* response frame exchange is complete (indicated by resetting this  */
        /* flag in the Tx Complete for this frame).                          */
		TROUT_FUNC_EXIT;
        return BTRUE;
    }

    /* Requeue one buffered packet from the station's non-delivery enabled Q */
    /* to the MAC H/w queue                                                  */
    /* If the non-delivery enabled Q is empty queue a NULL frame             */
    /* ASSUMPTION: legacy queue should be empty if all queues are delivery   */
    /* enabled                                                               */
    if(peek_list(&(ae->ps_q_lgcy)) == NULL)
        tx_null_frame(wlan_rx->sa, ae, BFALSE, 0, 0);
    else
        requeue_ps_packet(ae, &(ae->ps_q_lgcy), BTRUE, BFALSE) ;

    ae->ps_poll_rsp_qed = BTRUE;

	TROUT_FUNC_EXIT;
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_tim_bit                                              */
/*                                                                           */
/*  Description   : This function sets the TIM bit corresponding to the      */
/*                  association ID in virtual bit map.                       */
/*                                                                           */
/*  Inputs        : 1) Association ID                                        */
/*                                                                           */
/*  Globals       : g_vbmap                                                  */
/*                                                                           */
/*  Processing    : This function is called whenever a new packet is added   */
/*                  to the power save queue for a sleeping station. Given    */
/*                  the association ID, set the bit corresponding to that.   */
/*                  If this bit is not already included in the offset and    */
/*                  length fields of virtual bit map adjust offset and       */
/*                  length fields.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_tim_bit(UWORD16 asoc_id)
{
    UWORD8 byte_offset = 0;
    UWORD8 bit_offset  = 0;
    UWORD8 pvb_offset  = 0;
    UWORD8 length      = 0;

    /* Traffic Indication Virtual Bit Map within the AP, generates the TIM   */
    /* such that if a station has buffered packets, then the corresponding   */
    /* bit (which can be found from the association ID) is set. The byte     */
    /* offset is obtained by dividing the association ID by '8' and the bit  */
    /* offset is the remainder of the association ID when divided by '8'.    */
    byte_offset = (asoc_id & (~0xC000)) >> 3;
    bit_offset  = (asoc_id & (~0xC000)) & 0x07;

    /* protect the virtual bit map in beacon frame by zhao 6-21 2013 */
	get_vbp_mutex(__builtin_return_address(0));
    /* Calculate the current byte offset in vbmap */
    pvb_offset  = (g_vbmap[BMAP_CTRL_OFFSET] & 0xFE) >> 1;
    length      = g_vbmap[LENGTH_OFFSET];

    /* Compare the existing offset and the offset for the new STA. Create    */
    /* new length and offset and add that to vbmap.                          */
    if(byte_offset < pvb_offset)
    {
        pvb_offset = byte_offset;
        length     = pvb_offset - byte_offset;
    }
    else if(byte_offset > pvb_offset + length - MIN_TIM_LEN)
    {
        length += byte_offset - pvb_offset;
    }

    /* Set the TIM bit and length */
    g_vbmap[TIM_OFFSET + byte_offset] |= g_bmap[bit_offset];
    g_vbmap[LENGTH_OFFSET] = length;
	// 20120709 caisf add, merged ittiam mac v1.2 code
    if(g_update_active_bcn == 1)
    {
		UWORD8 hw_bcn = (g_beacon_index + 1)%2;

		set_tim_bit_bcn(hw_bcn, asoc_id);
	}
	put_vbp_mutex();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_and_reset_tim_bit                                  */
/*                                                                           */
/*  Description   : This function sets the TIM bit corresponding to the      */
/*                  association ID in virtual bit map.                       */
/*                                                                           */
/*  Inputs        : 1) Association ID                                        */
/*                                                                           */
/*  Globals       : g_vbmap                                                  */
/*                                                                           */
/*  Processing    : This function is called whenever a packet is removed     */
/*                  from the power save queue for a sleeping station. Given  */
/*                  the association ID, set the bit corresponding to that.   */
/*                  Recalculate length and offset fields and update the      */
/*                  the virtual bit map.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void check_and_reset_tim_bit(UWORD16 asoc_id)
{
    UWORD8  byte_offset    = 0;
    UWORD8  bit_offset     = 0;
    UWORD8  pvb_offset     = 0;
    UWORD8  length         = 0;
    UWORD8  new_pvb_offset = 0;
    WORD16  i              = 0;
    UWORD8  new_length     = MIN_TIM_LEN;

    /* Traffic Indication Virtual Bit Map within the AP, generates the TIM   */
    /* such that if a station has buffered packets, then the corresponding   */
    /* bit (which can be found from the association ID) is set. The byte     */
    /* offset is obtained by dividing the association ID by '8' and the bit  */
    /* offset is the remainder of the association ID when divided by '8'.    */
    byte_offset = (asoc_id & (~0xC000)) >> 3;
    bit_offset  = (asoc_id & (~0xC000)) & 0x07;

    /* protect the virtual bit map in beacon frame by zhao 6-21 2013 */
	get_vbp_mutex(__builtin_return_address(0));	
    /* Calculate the current byte offset in vbmap */
    pvb_offset  = (g_vbmap[BMAP_CTRL_OFFSET] & 0xFE) >> 1;
    length      = g_vbmap[LENGTH_OFFSET];

    /* Reset the TIM bit */
    g_vbmap[TIM_OFFSET + byte_offset] &= ~g_bmap[bit_offset];

    /* Calculate new offset using the following algorithm:                   */
    /* The new offset will be equal to or greater than the current offset as */
    /* the TIM bit is reset. Also the new TIM length cannot be more the      */
    /* existing length. So the algorithm is starting from old offset to      */
    /* old offset + TIM length, find out the first occurance of non-zero byte*/
    /* in the TIM element array. This will be the new offset. If no non-zero */
    /* bytes are found, new offset is zero (as initialized).                 */
    for(i = pvb_offset; i < pvb_offset + length - MIN_TIM_LEN; i++)
    {
        if(g_vbmap[TIM_OFFSET + i])
        {
            new_pvb_offset = i;
            break;
        }
    }

    /* Calculate new length using the following algorithm                    */
    /* The new TIM length will be less than or equal to old length as TIM bit*/
    /* is reset. Having new offset calculated, the new length can be         */
    /* calculated as the last occurance of non-zero byte from new offset to  */
    /* new offset+old length in the TIM element array. (In other words the   */
    /* first occurance of non-zero byte in the reverse direction)            */
    for(i = pvb_offset + length - MIN_TIM_LEN; i >= 0; i--)
    {
        if(g_vbmap[TIM_OFFSET + i])
        {
            new_length += (i - new_pvb_offset);
            break;
        }
    }

    /* Assign the new offset and length to the vbmap */
    g_vbmap[BMAP_CTRL_OFFSET] |= (new_pvb_offset & 0xEF) << 1;
    g_vbmap[LENGTH_OFFSET]     = new_length;
	// 20120709 caisf add, merged ittiam mac v1.2 code
    if(g_update_active_bcn == 1){
		UWORD8 hw_bcn = (g_beacon_index + 1)%2;
		reset_tim_bit_bcn(hw_bcn, asoc_id);
	}

	put_vbp_mutex();	
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_ps_tx_comp_ap                                     */
/*                                                                           */
/*  Description   : This function handles transmit complete interrupt for    */
/*                  power management purpose.                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit descriptor                    */
/*                                                                           */
/*  Globals       : g_num_mc_bc_pkt                                          */
/*                                                                           */
/*  Processing    : This function checks if the frame transmitted was a      */
/*                  broadcast/multicast frame. If so it reduces the global   */
/*                  bc/mc packet count in case it is non-zero (indicating    */
/*                  that the packets were queued). If it is unicast the      */
/*                  association entry is searched and corresponding number   */
/*                  of packets are decremented if non-zero (indicating they  */
/*                  were queued).                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
void handle_ps_tx_comp_ap(UWORD8 *tx_dscr, asoc_entry_t *ae)
{
    UWORD8 *msa  = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)tx_dscr);
#else
void handle_ps_tx_comp_ap(UWORD8 *tx_dscr, asoc_entry_t *ae, UWORD8 *msa)
{
#endif
    UWORD8 *da   = NULL;

    /* Get the Destination address */
    da = get_DA_ptr(msa);

    /* protect the virtual bit map in beacon frame by zhao 6-21 2013 */
	get_vbp_mutex(__builtin_return_address(0));
    if(is_group(da) ==  BTRUE){
        if(g_num_mc_bc_qd_pkt > 0){
            g_num_mc_bc_qd_pkt--;
            /* If no packets are queued in Sw PSQ or Hw for the */
            /* station, reset DTIM bit                          */
            if((g_num_mc_bc_pkt + g_num_mc_bc_qd_pkt) == 0)
            {
                reset_dtim_bit(AID0_BIT);
                reset_dtim_bit(DTIM_BIT);
			// 20120709 caisf add, merged ittiam mac v1.2 code
			if(g_update_active_bcn == 1){
				UWORD8 hw_bcn = (g_beacon_index + 1)%2;
				reset_mc_bit_bcn(hw_bcn);
			}
            }
        }
    }
    else
    {
        if(ae == 0)
        {
		put_vbp_mutex();
            /* Exception. Should not happen */
            return;
        }

        /* Reset the age counter as this STA is active */
        ae->aging_cnt = 0;

        /* Get the type of frame transmitted */
        if(get_type(msa) == DATA)
        {
            if(BTRUE == update_ps_counts_txcomp(ae, msa))
            {
		put_vbp_mutex();
                check_and_reset_tim_bit(ae->asoc_id);
		get_vbp_mutex(__builtin_return_address(0));
            }
        }

#ifdef PS_DSCR_JIT
        /* Check if there are packets pending that are to be requeued */
        /* This path is not valid currently. May be valid if descriptors are */
        /* not buffered and are created just in time before transmission.    */
        if(peek_list(g_ps_pending_q) != NULL)
        {
            check_pending_requeue_packet();
        }
#endif /* PS_DSCR_JIT */
    }
	put_vbp_mutex();
}

#ifdef PS_DSCR_JIT
/*****************************************************************************/
/* These functions are not used currently. May be used if descriptors are not*/
/* buffered and are created just in time before transmission                 */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_requeue_pending_packet                            */
/*                                                                           */
/*  Description   : This function handles the packet to be requeued when hw  */
/*                  queue is not available                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to association entry                          */
/*                  2) Pointer to head of the queue                          */
/*                  3) Legacy queue or not                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : A packet is removed from the head of the given power     */
/*                  save queue. The transmit descriptor is extracted and     */
/*                  added to the H/w queue. The power save queue element     */
/*                  buffer is freed.                                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_requeue_pending_packet(asoc_entry_t *ae, list_buff_t *qh,
                                   BOOL_T ps_q_legacy)
{
    pm_buff_element_t *qe;

    /* Since H/w queue was not available pending queue element structure is  */
    /* allocated, initialized and inserted into the required queue.          */
    qe = (pm_buff_element_t *)mem_alloc(g_local_mem_handle,
        sizeof(pm_buff_element_t));

    if(qe == NULL)
    {
        /* Do nothing */
        return ;
    }

    /* Update the queue element */
    qe->ae          = ae;
    qe->q_legacy    = ps_q_legacy ;

    /* Insert Q at tail */
    add_list_element(qh, (void*)qe);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_pending_requeue_packet                             */
/*                                                                           */
/*  Description   : This function handles the packet in the pending queue    */
/*                  to be requeued                                           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : if an element is present in the pending queue then the   */
/*                  queue element is removed and that particular STA s       */
/*                  processed for requeuing the packets in its power save Q  */
/*                  as long as H/w Q is available                            */
/*                  if H/w Q not available then the sta asoc entry is again  */
/*                  stored in the pending queue and returns                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
void check_pending_requeue_packet()
{
    pm_buff_element_t *pm_buff = 0;
    REQUEUE_STATUS_T  status   = RE_Q_ERROR;
    asoc_entry_t      *ae      = 0;

    /* Remove queue element from the pending Q */
    pm_buff = remove_list_element_head(&g_ps_pending_q);

    /* Get the association entry */
    ae      = pm_buff->ae;

    /* If the STA is in active mode then requeue the pending packets else
    drop the entry */
    if(ae->ps_state == ACTIVE_PS)
    {
        /*If there is no element in queue then return */
        if(pm_buff == 0)
        {
            /* exception */
            return ;
        }
        else
        {
            status = check_pending_packet_prot(ae);
            if(status == PKT_NOT_REQUEUED)
            {
                return;
            }
            else
            {
                while((status = requeue_ps_packet(ae, &ae->ps_q_lgcy, BTRUE))
                    == PKT_REQUEUED);

                if(status == PKT_NOT_REQUEUED)
                {
                    handle_requeue_pending_packet(ae, &g_ps_pending_q, BTRUE);
                }
            }
        }
    }
}
#else
void check_pending_requeue_packet()
{
    pm_buff_element_t *pm_buff = 0;
    REQUEUE_STATUS_T  status   = RE_Q_ERROR;
    asoc_entry_t      *ae      = 0;

    /* Remove queue element from the pending Q */
    pm_buff = remove_list_element_head(&g_ps_pending_q);

    /*If there is no element in queue then return */
    if(pm_buff == 0)
    {
        /* exception */
        return ;
    }

    /* Get the association entry */
    ae      = pm_buff->ae;

    /* Free the buffer having list pointer */
    mem_free(g_local_mem_handle, pm_buff);
    pm_buff = NULL;

    /* NULL check for Association entry */
    if(ae == NULL)
    	return;

    /* If the STA is in active mode then requeue the pending packets else
    drop the entry */
    if(ae->ps_state == ACTIVE_PS)
    {
            status = check_pending_packet_prot(ae);
            if(status == PKT_NOT_REQUEUED)
            {
                return;
            }
            else
            {
                while((status = requeue_ps_packet(ae, &ae->ps_q_lgcy, BTRUE))
                    == PKT_REQUEUED);

                if(status == PKT_NOT_REQUEUED)
                {
                    handle_requeue_pending_packet(ae, &g_ps_pending_q, BTRUE);
            	}
        	}
    }
}
#endif

#endif /* PS_DSCR_JIT */

#endif /* BSS_ACCESS_POINT_MODE */
