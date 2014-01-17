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
/*  File Name         : qmu_rx.c                                             */
/*                                                                           */
/*  Description       : This file contains all the Queue manager RX related  */
/*                      functions needed by the MAC S/W                      */
/*                                                                           */
/*  List of Functions : qmu_add_rx_buffer                                    */
/*                      qmu_update_rx_q                                      */
/*                      qmu_update_rx_q_defrag                               */
/*                      qmu_sync_rx_packet                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "imem_if.h"
#include "receive.h"
#include "qmu_if.h"

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

#ifndef TROUT_WIFI
BOOL_T g_rxq_replenish_reqd[NUM_RX_Q] = {BFALSE, BFALSE};

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void qmu_update_rx_q_defrag(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num,
                                   UWORD32 *rx_dscr, UWORD8 rx_count);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_add_rx_buffer                                        */
/*                                                                           */
/*  Description   : This function add a new empty recieve descriptor and     */
/*                  buffer to the end of the required receive queue.         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the receive queue handle                   */
/*                  2) Receive queue number                                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function adds new buffers to the required queue. It */
/*                  creates a new RX buffer and RX descriptor and updates    */
/*                  the required fields in it. The MAC H/w RX-Q pointer is   */
/*                  updated if it is NULL (RX-Q Empty Condition).            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : QMU_OK on success. Failure code otherwise.               */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 qmu_add_rx_buffer(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num)
{
    UWORD32    *rx_dscr     = NULL;
    UWORD8     *rx_buff     = NULL;
    UWORD32    *temp        = NULL;
    q_struct_t *q_rx_struct = &(q_rx_handle->rx_header[q_num]);
#ifndef MAC_HW_UNIT_TEST_MODE
    MEM_PRIORITY_LEVEL_T   plevel = MEM_PRI_HPRX;
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* Allocate memory for the receive Descriptor */
    rx_dscr = (UWORD32 *)mem_alloc(g_shared_dscr_mem_handle, RX_DSCR_LEN);

    /* Check whether allocation succeeded. Return error code otherwise */
    if(rx_dscr == NULL)
        return QMU_SHARED_MALLOC_FAILED;

#ifndef MAC_HW_UNIT_TEST_MODE
    /* Allocate memory for the receive buffer */
    plevel  = (q_num == HIGH_PRI_RXQ) ? MEM_PRI_HPRX : MEM_PRI_RX;
    rx_buff = (UWORD8 *)pkt_mem_alloc(plevel);
#else /* MAC_HW_UNIT_TEST_MODE */
    rx_buff = mem_alloc(g_shared_pkt_mem_handle, RX_BUFFER_SIZE);
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* Check whether allocation succeeded. */
    if(rx_buff == NULL)
    {
        /* Free the previously allocated receive descriptor and return error */
        /* code if allocation failed.                                        */
        mem_free(g_shared_dscr_mem_handle, rx_dscr);
        return QMU_SHARED_MALLOC_FAILED;
    }

    /* Reset the receive descriptor */
    reset_rx_dscr(rx_dscr);

    /* Update the buffer pointer in the receive descriptor */
    set_rx_dscr_buffer_ptr(rx_dscr, (UWORD32)rx_buff);

    /* The new receive descriptor is added at the end of the receive queue */
    set_rx_dscr_next_dscr(rx_dscr, 0);

    /* Start of Critical section */
    critical_section_start();

    /* Read the last element pointer and update the link in the receive      */
    /* descriptor with this                                                  */
    temp = q_rx_struct->element_tail;
    set_rx_dscr_prev_dscr(rx_dscr, (UWORD32)temp);

    if(temp != NULL)
    {
        /* If the cached last element pointer is not NULL, then update the */
        /* next descriptor pointer of it with the new descriptor address.  */
        set_rx_dscr_next_dscr(temp, (UWORD32)rx_dscr);
    }
    else
    {
        /* The last element pointer being NULL indicates that the new  */
        /* descriptor is being added to the head of the receive queue. */
        q_rx_struct->element_head = rx_dscr;
    }

    /* Update the last element of the RX Q cache */
    q_rx_struct->element_tail = rx_dscr;

    /* Increment queue packet counters */
    q_rx_struct->element_cnt++;
    q_rx_handle->rx_list_count++;

    /* If receive queue seen by the MAC H/w is empty, then update the MAC H/w*/
    /* Rx-Q register with the new receive descriptor address.                */
    /* The additional check ensures that the new receive descriptor has not  */
    /* already been used-up by the time we reach this point.                 */
    if((BTRUE == is_machw_rx_buff_null(q_num)) &&
       (RX_NEW == get_rx_dscr_pkt_status(rx_dscr)))
    {
        set_machw_rx_buff_addr((UWORD32)rx_dscr, q_num);
    }

    /* End of critical section */
    critical_section_end();

#ifdef DEBUG_MODE
    g_mac_stats.qrxa[q_num]++;
#endif /* DEBUG_MODE */

    return QMU_OK;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_update_rx_q                                          */
/*                                                                           */
/*  Description   : This function updates the receive queue to reflect       */
/*                  removal of the required number of receive descriptors    */
/*                  from it.                                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the receive queue handle                   */
/*                  2) Receive queue number                                  */
/*                  3) Base receive descriptor for the receive frame         */
/*                  4) Receive count for the receive frame                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The required number of RX-Descriptors starting from the  */
/*                  identified descriptor are parsed and the links in RX-Q   */
/*                  are updated. This function does not free the receive     */
/*                  descriptor itself. Its the responsibility of the calling */
/*                  function to do this.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void qmu_update_rx_q(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num,
                     UWORD32 *rx_dscr, UWORD8 rx_count)
{
    UWORD8     update_cnt     = 0;
    UWORD32    *first_rx_dscr = NULL;
    UWORD32    *last_rx_dscr  = NULL;
    q_struct_t *q_rx_struct   = &(q_rx_handle->rx_header[q_num]);

    /* For a fragmented frame call a separate function to update the receive */
    /* queue.                                                                */
    if(is_rx_frame_frag(rx_dscr) == BTRUE)
    {
        qmu_update_rx_q_defrag(q_rx_handle, q_num, rx_dscr, rx_count);
        return;
    }

    /* Start of Critical section */
    critical_section_start();

    /* Get the previous and next link pointers from the receive descriptor */
    first_rx_dscr = (UWORD32 *)get_rx_dscr_prev_dscr(rx_dscr);
    last_rx_dscr  = (UWORD32 *)get_rx_dscr_next_dscr(rx_dscr);

    /* Traverse through list to reach the end of the desired number of frames */
    for(update_cnt = 1; update_cnt < rx_count; update_cnt++)
    {
        /* Break the traversal in case the Rx-Q end is reached unexpectedly. */
        /* This condition signals a possibly recoverable sync loss between   */
        /* H/w and S/w.                                                      */
        if(last_rx_dscr == NULL)
        {
#ifdef DEBUG_MODE
            g_mac_stats.qrx_unexp_end[q_num]++;
#endif /* DEBUG_MODE */
            break;
        }

        last_rx_dscr = (UWORD32 *)get_rx_dscr_next_dscr(last_rx_dscr);
    }

    /* Unlink the chain of the desired number of frames from the queue */
    /* The first frame of the list is unlinked here.                   */
    if(first_rx_dscr == NULL)
        q_rx_struct->element_head = last_rx_dscr;
    else
        set_rx_dscr_next_dscr(first_rx_dscr, (UWORD32)last_rx_dscr);

    /* The last frame of the list is unlinked here. */
    if(last_rx_dscr == NULL)
        q_rx_struct->element_tail = first_rx_dscr;
    else
        set_rx_dscr_prev_dscr(last_rx_dscr, (UWORD32)first_rx_dscr);

    /* Update the queue packet counters */
    q_rx_struct->element_cnt   -= update_cnt;
    q_rx_handle->rx_list_count -= update_cnt;

    /* End of critical section */
    critical_section_end();

#ifdef DEBUG_MODE
    g_mac_stats.qrxd[q_num] += update_cnt;
#endif /* DEBUG_MODE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_update_rx_q_defrag                                   */
/*                                                                           */
/*  Description   : This function updates the receive queue to reflect       */
/*                  removal of the required number of receive descriptors    */
/*                  from it for a defragmented frame.                        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the receive queue handle                   */
/*                  2) Receive queue number                                  */
/*                  3) Base receive descriptor for the receive frame         */
/*                  4) Receive count for the receive frame                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The required number of RX-Descriptors starting from the  */
/*                  identified descriptor are parsed and the links in RX-Q   */
/*                  are updated. For a defragmented frame the links to be    */
/*                  followed are different from a non-defragmented frame and */
/*                  these links may not be in order. As a result a separate  */
/*                  function is called to do this. Note that this function   */
/*                  may be modified slightly to support non-defragmented     */
/*                  frames but this may not be optimal and hence 2 separate  */
/*                  functions are maintained. This function does not free    */
/*                  the receive descriptor itself. It is the responsibility  */
/*                  of the calling function to do this.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void qmu_update_rx_q_defrag(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num,
                            UWORD32 *rx_dscr, UWORD8 rx_count)
{
    UWORD8     update_cnt        = 0;
    UWORD32    *prev_rx_dscr     = NULL;
    UWORD32    *next_defrag_dscr = NULL;
    UWORD32    *next_rx_dscr     = NULL;
    UWORD32    *temp_dscr        = NULL;
    UWORD32    *fixed_head       = NULL;
    UWORD32    *fixed_tail       = NULL;
    BOOL_T     rxq_empty         = BFALSE;
    q_struct_t *q_rx_struct      = &(q_rx_handle->rx_header[q_num]);

    /* Start of Critical section */
    critical_section_start();

    /* Process the number of descriptors forming the received frame */
    for(update_cnt = 0; update_cnt < rx_count; update_cnt++)
    {
        /* Break the traversal in case the Rx-Q end is reached unexpectedly. */
        /* This condition signals a possibly recoverable sync loss between   */
        /* H/w and S/w.                                                      */
        if(rx_dscr == NULL)
        {
#ifdef DEBUG_MODE
            g_mac_stats.qrx_unexp_end[q_num]++;
#endif /* DEBUG_MODE */

            break;
        }

        /* Get the previous, next and next defrag descriptor pointers from   */
        /* the receive descriptor                                            */
        prev_rx_dscr     = (UWORD32 *)get_rx_dscr_prev_dscr(rx_dscr);
        next_defrag_dscr = (UWORD32 *)get_rx_dscr_next_defrag_buffer(rx_dscr);
        next_rx_dscr     = (UWORD32 *)get_rx_dscr_next_dscr(rx_dscr);

        /* Unlink this receive descriptor by updating the next and previous  */
        /* descriptor links in the previous and next descriptors. Update the */
        /* fixed head/tail if the prev/next descriptor is NULL (indicating   */
        /* the descriptor unlinked was currently at the head/tail of queue)  */
        if(next_rx_dscr == NULL)
            fixed_tail = prev_rx_dscr;
        else
            set_rx_dscr_prev_dscr(next_rx_dscr, (UWORD32)prev_rx_dscr);

        if(prev_rx_dscr == NULL)
        {
            fixed_head = next_rx_dscr;

            if(fixed_head == NULL)
                rxq_empty = BTRUE;
        }
        else
            set_rx_dscr_next_dscr(prev_rx_dscr, (UWORD32)next_rx_dscr);

        /* If the next descriptor and next defragmentation buffer descriptor */
        /* are not the same update the next link with the next defrag buffer */
        /* descriptor                                                        */
        if(next_defrag_dscr != next_rx_dscr)
            set_rx_dscr_next_dscr(rx_dscr, (UWORD32)next_defrag_dscr);

        /* If the prev descriptor and saved descriptor that was processed    */
        /* most recently are not the same update the prev link with the      */
        /* saved descriptor that was processed most recently (0 initially)   */
        if(temp_dscr != prev_rx_dscr)
            set_rx_dscr_prev_dscr(rx_dscr, (UWORD32)temp_dscr);

        /* Save this descriptor for getting the previous link for the next   */
        /* descriptor processed                                              */
        temp_dscr = rx_dscr;

        /* Move to processing the next defragmentation Rx descriptor */
        rx_dscr = next_defrag_dscr;
    }

    /* If new head and tail were found (due to removal of descriptor at the  */
    /* head or tail of the queue, update the information in the q structure  */
    if((fixed_head != NULL) || (rxq_empty == BTRUE))
        q_rx_struct->element_head = fixed_head;

    if((fixed_tail != NULL) || (rxq_empty == BTRUE))
        q_rx_struct->element_tail = fixed_tail;

    /* Update the queue packet counters */
    q_rx_struct->element_cnt   -= update_cnt;
    q_rx_handle->rx_list_count -= update_cnt;

    /* End of critical section */
    critical_section_end();

#ifdef DEBUG_MODE
    g_mac_stats.qrxd[q_num] += update_cnt;
#endif /* DEBUG_MODE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_sync_rx_packet                                       */
/*                                                                           */
/*  Description   : This function synchronizes the receive queue with        */
/*                  respect to the passed receive descriptor. This performs  */
/*                  the necessary cleanup of orphaned buffers.               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the receive queue handle                   */
/*                  2) Receive queue number                                  */
/*                  3) Receive descriptor to synchronize with                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the receive queue and frees all the */
/*                  buffers related to descriptors with non-zero sequence    */
/*                  numbers that are less than the sync sequence number that */
/*                  is extracted from the given descriptor. All other        */
/*                  descriptors with sequence number zero or more than the   */
/*                  sync sequence number are skipped. The queue is checked   */
/*                  till a new Rx buffer is found or the queue becomes empty */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void qmu_sync_rx_packet(qmu_rx_handle_t *q_rx_handle, UWORD8 q_num,
                        UWORD32 *rx_dscr)
{
    q_struct_t *q_rx_struct = &(q_rx_handle->rx_header[q_num]);
    UWORD32    *curr_dscr   = q_rx_struct->element_head;
    UWORD8     sync_seq_num = 0;

    /* Extract the sequence number from the given Rx descriptor. The queue   */
    /* needs to be synchronized to this sequence number. All the descriptors */
    /* with non-zero sequence numbers less than this will be removed from    */
    /* the queue                                                             */
    sync_seq_num = get_rx_dscr_int_seq_num(rx_dscr);


    /* Traverse the entire receive queue starting from the head of the queue */
    while(curr_dscr != NULL)
    {
        UWORD8 seq_num = 0;
        UWORD8 status  = RX_NEW;

        /* Get the sequence number of this descriptor only if it is the first */
        /* buffer of an MSDU. The intermediate buffers of the MSDU have their */
        /* status fields set to RX_NEW.                                       */
        status = get_rx_dscr_pkt_status(curr_dscr);

        if( (status != RX_NEW) && (status != DEFRAG_IN_PROGRESS))
        seq_num = get_rx_dscr_int_seq_num(curr_dscr);

        /* Packets with non-zero sequence numbers less than the synchronize  */
        /* sequence number will be deleted                                   */
        if((seq_num != 0) && (dscr_seqno_lt(seq_num, sync_seq_num) == BTRUE))
        {
            UWORD8  num_buff   = 0;
            UWORD32 *temp_dscr = 0;

            /* Extract and save the previous descriptor address from current */
            /* descriptor that will be removed now. This saved link will be  */
            /* used to resume traversal of the linked list.                  */
            temp_dscr = (UWORD32 *)get_rx_dscr_prev_dscr(curr_dscr);

            /* Get the number of buffers associated with this descriptor */
            num_buff = get_rx_dscr_num_buffers(curr_dscr);

            /* Remove the descriptor along with the buffers from the queue */
            qmu_update_rx_q(q_rx_handle, q_num, curr_dscr, num_buff);

            /* Free the entire receive descriptor list for this frame */
            free_rx_dscr_list(curr_dscr, num_buff);

            /* Set the current descriptor to the saved previous link. Note   */
            /* that the links in this descriptor have been updated correctly */
            /* by the qmu_update_rx_q function                               */
            curr_dscr = (temp_dscr == NULL)? q_rx_struct->element_head :
                        (UWORD32 *)get_rx_dscr_next_dscr(temp_dscr);

#ifdef DEBUG_MODE
            g_mac_stats.qrx_nosync_del[q_num]++;
#endif /* DEBUG_MODE */
        }
        /* Sequence numbers equal to 0 or more than the sync sequence number */
        /* are skipped. Do nothing.                                          */
        else
        {
#ifdef DEBUG_MODE
            g_mac_stats.qrx_nosync_nodel[q_num]++;
#endif /* DEBUG_MODE */
        /* Move to next descriptor link present in the current descriptor */
        curr_dscr = (UWORD32 *)get_rx_dscr_next_dscr(curr_dscr);
    }
}
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : replenish_rx_queue                                       */
/*                                                                           */
/*  Description   : This function adds buffers to the Receive queue to       */
/*                  maintain the size to the desired level.                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the receive queue handle                   */
/*                  2) Receive queue number                                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the current size of receive queue   */
/*                  and adds buffers to increase it to the desired level. In */
/*                  case it is not able to do this, it marks the queue for   */
/*                  later updation.                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void replenish_rx_queue(qmu_handle_t *q_handle, UWORD8 q_num)
{
    UWORD16 curr_num_bufs = q_handle->rx_handle.rx_header[q_num].element_cnt;
    UWORD16 trgt_num_bufs = 0;

	TROUT_FUNC_ENTER;
    /* The target number of buffers might be different for each queue */
    if(q_num == NORMAL_PRI_RXQ)
    {
        if(get_num_used_rx_buffers() < RX_PACKET_BUFFER_THRESHOLD)
        {
            UWORD16 trgt_buff_thresh = 0;

            trgt_buff_thresh = (RX_PACKET_BUFFER_THRESHOLD -
                                    get_num_used_rx_buffers()) + curr_num_bufs;

            if(trgt_buff_thresh < NUM_RX_BUFFS)
            {
                trgt_num_bufs = trgt_buff_thresh;
                g_rxq_replenish_reqd[NORMAL_PRI_RXQ] = BTRUE;
#ifdef DEBUG_MODE
                g_mac_stats.num_wlan_fc++;

                if(trgt_buff_thresh == 0)
                    g_mac_stats.num_wlan_fc_nobuff++;
#endif /* DEBUG_MODE */
            }
            else
            {
                trgt_num_bufs = NUM_RX_BUFFS;
                g_rxq_replenish_reqd[NORMAL_PRI_RXQ] = BFALSE;
            }
        }
        else
        {
			g_rxq_replenish_reqd[NORMAL_PRI_RXQ] = BTRUE;
            return;
    	}
    }
    else
    {
        trgt_num_bufs = NUM_HIPR_RX_BUFFS;
        g_rxq_replenish_reqd[HIGH_PRI_RXQ] = BFALSE;
    }

    while(curr_num_bufs < trgt_num_bufs)
    {
        if(qmu_add_rx_buffer(&(q_handle->rx_handle), q_num) != QMU_OK)
        {
            /* Free Buffers currently not available */
            g_rxq_replenish_reqd[q_num] = BTRUE;
            break;
        }

        curr_num_bufs = q_handle->rx_handle.rx_header[q_num].element_cnt;
    }

#ifndef OLD_MACHW_ERROR_INT
    /* RxQ-Empty Error Interrupts are unmasked only if the MAC H/w RxQ is not */
    /* empty.                                                                 */
    if(BFALSE == is_machw_rx_buff_null(q_num))
    {
        if(q_num == NORMAL_PRI_RXQ)
        {
            unmask_machw_error_int_code(UNEXPECTED_RX_Q_EMPTY);
        }
        else
        {
            unmask_machw_error_int_code(UNEXPECTED_HIRX_Q_EMPTY);
        }
    }
#endif /* OLD_MACHW_ERROR_INT */
	TROUT_FUNC_EXIT;
}

#endif
