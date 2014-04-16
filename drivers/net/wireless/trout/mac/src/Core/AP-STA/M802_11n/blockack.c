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
/*  File Name         : blockack.c                                           */
/*                                                                           */
/*  Description       : This file contains the Block ACK related functions   */
/*                      for 802.11e mode                                     */
/*                                                                           */
/*  List of Functions : reset_tx_ba_handle                                   */
/*                      delba_initiator_process                              */
/*                      reset_rx_ba_handle                                   */
/*                      delba_recp_process                                   */
/*                      send_addba_rsp                                       */
/*                      update_addba_list                                    */
/*                      send_delba                                           */
/*                      delete_rx_ba_node                                    */
/*                      buffer_ba_rx_frame                                   */
/*                      process_ba_rx_buff_frame                             */
/*                      addba_timeout_fn                                     */
/*                      is_ba_buff_tx_pkt                                    */
/*                      ba_timeout_fn                                        */
/*                      ba_pend_tx_timeout_fn                                */
/*                      retx_ba_frame                                        */
/*                      dequeue_tx_ba_frame                                  */
/*                      send_ba_req                                          */
/*                      send_frames_with_gap                                 */
/*                      send_frames_in_order                                 */
/*                      move_ba_window_ahead                                 */
/*                      handle_data_tx_failure                               */
/*                      process_ba_frame                                     */
/*                      handle_ba_tx_comp                                    */
/*                      ps_buff_ba_frames                                    */
/*                      get_bar_dscr                                         */
/*                      bar_post_tx_proc                                     */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"
#include "frame_11e.h"
#include "management_11e.h"
#include "management_11n.h"
#include "mh.h"
#include "receive.h"
#include "prot_if.h"
#include "qmu_if.h"
#include "host_if.h"
#include "mac_init.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/* Table to maintain QOS info of destination stations, in order of STA index */
UWORD8 g_ba_lut_index_table[BA_LUT_IDX_BMAP_LEN];
UWORD8 g_num_ba_tx_sessions = 0;
UWORD8 g_num_ba_rx_sessions = 0;


struct mutex ba_rx_mutex;	//add by chengwg 2013-06-21.
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void process_ba_rx_buff_frame(ba_rx_struct_t *ba, bfd_rx_pkt_t *pkt_ptr,
                              UWORD8 tid);
static BOOL_T retx_ba_frame(void *entry, UWORD8 *tx_dscr, UWORD8 tid);
static void handle_data_tx_failure(ba_tx_struct_t **ba, UWORD32 *tx_dscr,
                                   UWORD16 seq_num);
static void process_ba_frame(void *pv_ba, UWORD8 tid, UWORD16 last_seq_num,
                             UWORD16 start_seq_num, UWORD8 *bitmap);
static BOOL_T buffer_ba_rx_frame(ba_rx_struct_t **ba, UWORD8 *rx_base_dscr,
                                 UWORD8 num_dscr, UWORD8 *msa, UWORD16 rx_len,
                                 UWORD16 seq_num, CIPHER_T ct);
static UWORD32 *get_bar_dscr(ba_tx_struct_t *ba, UWORD8 tid, UWORD8 q_num);
static void bar_post_tx_proc(ba_tx_struct_t *ba);

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reset_tx_ba_handle                                    */
/*                                                                           */
/*  Description      : This functions resets the Transmit BA Handle          */
/*                                                                           */
/*  Inputs           : 1) The Block Ack Handle                               */
/*                     2) The TID Value                                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions resets the BA Handle                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void reset_tx_ba_handle(ba_tx_struct_t **ba, UWORD8 tid)
{
	TROUT_FUNC_ENTER;
	
    if((*ba) == NULL)
    {
        /* Exception */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Mark the Block-Ack session as invalid */
    (*ba)->is_ba = BFALSE;

    /* Delete all the Session Timers */
    stop_ba_timer(&(*ba)->ba_alarm);
    del_ba_timer(&(*ba)->ba_alarm);
    stop_ba_timer(&(*ba)->ba_pend_alarm);
    del_ba_timer(&(*ba)->ba_pend_alarm);

    /* Flush out the Retry-Queue. All frames in this Q are deleted. */
    flush_retry_q(*ba);

    /* Flush out the Pending-Queue. All frames in this Q are requeued. */
    dequeue_tx_ba_frame((*ba)->ba_data.entry, *ba, 0xFF, tid);

    /* This resets any other HT sessions associated with this Tx-BA Session */
    reset_tx_ba_state_prot((*ba)->ba_data.entry, tid);

    g_num_ba_tx_sessions--;
    decr_num_tx_ba_sessions((*ba)->ba_data.entry);

    mem_free(g_local_mem_handle, *ba);
    *ba = NULL;

    g_tx_ba_setup = BFALSE;

    /* ITM-DEBUG */
    TROUT_DBG4("Block-Ack Session Ended\n\r");
    TROUT_DBG4("TID = %d \n\r",tid);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : delba_initiator_process                               */
/*                                                                           */
/*  Description      : This functions processes the DELBA Message on the     */
/*                     initiator side.                                       */
/*                                                                           */
/*  Inputs           : 1) The Block Ack Handle                               */
/*                     2) Pointer to the received message                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes the DELBA Message            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void delba_initiator_process(ba_tx_struct_t **ba, UWORD8 *data)
{
    UWORD8 tid = (data[3] & 0xF0) >> 4;

	TROUT_FUNC_ENTER;
	
    if(*ba == NULL)
    {
        /* Exception */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Reset the BA handle for this TID */
    reset_tx_ba_handle(ba, tid);
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : reset_rx_ba_handle                                    */
/*                                                                           */
/*  Description      : This functions resets the Receive BA handle           */
/*                                                                           */
/*  Inputs           : 1) The Block Ack Handle                               */
/*                     2) The TID Value                                      */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions resets the BA Handle                   */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void reset_rx_ba_handle(ba_rx_struct_t **ba, UWORD8 tid)
{
	TROUT_FUNC_ENTER;
	
    if((*ba) == NULL)
    {
        /* Exception */
        TROUT_FUNC_EXIT;
        return;
    }

    // modify by Ke.Li at 2013-05-28 for CR 170666
    //stop_ba_timer(&(*ba)->ba_alarm);
    del_ba_timer(&(*ba)->ba_alarm);	//modify by chengwg 2013-06-21.

    /* Flush out the BA-Rx Queue */
    flush_reorder_q(*ba);

    if((*ba)->ba_policy == 1)
    {
        if((*ba)->sta_index != 0)
        {
            remove_machw_ba_lut_entry((*ba)->lut_index);
            del_ba_lut_index((*ba)->lut_index);
        }
    }

    g_num_ba_rx_sessions--;
    decr_num_rx_ba_sessions((*ba)->ba_data.entry);

    BUG_ON((*ba) == NULL);
    mem_free(g_local_mem_handle, *ba);
    *ba = NULL;

	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : delba_recp_process                                    */
/*                                                                           */
/*  Description      : This functions processes the DELBA Message on the     */
/*                     receipient side.                                      */
/*                                                                           */
/*  Inputs           : 1) The Block Ack Handle                               */
/*                     2) Pointer to the received message                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes the DELBA Message            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void delba_recp_process(ba_rx_struct_t **ba, UWORD8 *data)
{
    UWORD8 tid = (data[3] & 0xF0) >> 4;

	TROUT_FUNC_ENTER;
	
    if(*ba == NULL)
    {
        /* Exception */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Reset the BA handle for this TID */
    mutex_lock(&ba_rx_mutex);	//this mutex lock is essential, by chengwg 2013-06-21.!
    reset_rx_ba_handle(ba, tid);
    mutex_unlock(&ba_rx_mutex);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_addba_rsp                                        */
/*                                                                           */
/*  Description      : This functions processes the ADDBA Resquest           */
/*                                                                           */
/*  Inputs           : 1) The Block Ack Handle                               */
/*                     2) Pointer to the received ADDBA Response             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes the ADDBA Response           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void send_addba_rsp(ba_rx_struct_t **ba, UWORD8 *data, UWORD8 *da)
{
    UWORD8  tid              = (data[3] & 0x3C) >> 2;
    UWORD8  status           = SUCCESSFUL_STATUSCODE;
    void    *ht_entry        = NULL;
    UWORD16 frame_len        = 0;
    UWORD8  *addba_rsp_frame = 0;

    /* Exception. ADDBA-Requests are handled only when the status is BA_INIT. */
    if(((*ba) == NULL) || ((*ba)->conn_status != BA_INIT))
    {
        return;
    }

    /* Extract parameters from the ADDBA-Request Frame */
    /******************************************************************/
    /*       ADDBA Request Frame - Frame Body                         */
    /* ---------------------------------------------------------------*/
    /* | Category | Action | Dialog | Parameters | Timeout | SSN     |*/
    /* ---------------------------------------------------------------*/
    /* | 1        | 1      | 1      | 2          | 2       | 2       |*/
    /* ---------------------------------------------------------------*/
    /*                                                                */
    /* Parameters Field (2 bytes):                                    */
    /* ------------------------------------------------------         */
    /* | amsdu-sp | BA-Policy |    TID    |   Buffer Size   |         */
    /* ------------------------------------------------------         */
    /* |   B0     |    B1     | B2     B5 | B6          B15 |         */
    /* ------------------------------------------------------         */
    /******************************************************************/

    (*ba)->timeout      = data[5] | (data[6] << 8);
    (*ba)->ba_policy    = (data[3] & 0x02) >> 1;
    (*ba)->win_start    = (data[7] >> 4) | (data[8] << 4);
    (*ba)->conn_status  = BA_INPROGRESS;
    (*ba)->dialog_token = data[2];
    (*ba)->buff_size  = (data[3] & 0xC0) >> 6;
    (*ba)->buff_size |= (data[4] << 2);

    /* Perform a sanity check on the buffer size requested by the initiator */
    if(((*ba)->buff_size == 0) || ((*ba)->buff_size > MAX_BA_RX_BUFFERS))
    {
        (*ba)->buff_size = MAX_BA_RX_BUFFERS;
    }
    (*ba)->win_end  = SEQNO_ADD((*ba)->win_start, ((*ba)->buff_size - 1));

    (*ba)->win_tail = SEQNO_SUB((*ba)->win_start, 1);

    /* Update RX_BA structure with HT specific parameters */
    ht_entry = get_ht_handle_entry((*ba)->ba_data.entry);
    extract_addba_req_ht_params(ht_entry, *ba, data);

    if((*ba)->ba_policy == 1)
    {
        if(mget_ImmediateBlockAckOptionImplemented() == TV_FALSE)
        {
            status = INVALID_REQ_PARAMS;
        }
        else
        {
            /* If Block ACK is of type Immediate Compressed, the Block   */
            /* ACK LUT in MAC H/w needs to be updated.                   */
            if((*ba)->back_var == COMPRESSED_BACK)
            {
                /* Assign a new Block ACK LUT index */
                (*ba)->lut_index = get_ba_lut_index();

                /* If the LUT index is valid,  update the MAC H/w BA LUT */
                if((*ba)->lut_index != INVALID_BA_LUT_INDEX)
                {
                    /* Note that currently all Block ACK sessions are    */
                    /* set to full state operation (last argument passed */
                    /* as 0). This may be modified later.                */
#if 0
                    add_machw_ba_lut_entry((*ba)->lut_index,
                                           (*ba)->dst_addr,
                                           tid, (*ba)->win_start,
                                           (*ba)->buff_size, 0);
#else /* 0 */
                    add_machw_ba_lut_entry((*ba)->lut_index,
                                           (*ba)->dst_addr,
                                           tid, (*ba)->win_start,
                                           MAX_BA_WINSIZE, 0);
#endif /* 0 */
                }
                else
                {
                    /* If BA LUT is full, no further Block ACK sessions  */
                    /* can be set up. The request is therefor declined.  */
                    status = REQ_DECLINED;
                }
            }
            else
            {
            /* Uncompressed Block ACK is not supported */
                status = REQ_DECLINED;
            }
        }
    }
    else if((*ba)->ba_policy == 0)
    {
        /* Delayed Block-Ack is not supported */
        status = INVALID_REQ_PARAMS;
    }

    /* Allocate memory to the frame and send it */
    addba_rsp_frame = mem_alloc(g_shared_pkt_mem_handle, ADDBA_RSP_LEN);

    if(addba_rsp_frame == NULL)
    {
    	mutex_lock(&ba_rx_mutex); //add by chengwg 2013-06-21.
        reset_rx_ba_handle(ba, tid);
        mutex_unlock(&ba_rx_mutex);
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif        
        return;
    }

    /* If success, then add this struct to the queue */
    frame_len = prepare_addba_rsp(addba_rsp_frame, (*ba), tid, status);

    if(status != SUCCESSFUL_STATUSCODE)
    {
        (*ba)->conn_status  = BA_FAILED;
        mutex_lock(&ba_rx_mutex);	//add by chengwg 2013-06-21.
        reset_rx_ba_handle(ba, tid);
        mutex_unlock(&ba_rx_mutex);
    }
    else
    {
        (*ba)->is_ba = BTRUE;

        /* ITM-DEBUG */
        PRINTD("Block-Ack RX Session Successfully Setup\n");
        PRINTD("TID = %d BufSize = %d\n", tid, (*ba)->buff_size);
    }

    tx_mgmt_frame(addba_rsp_frame, frame_len, HIGH_PRI_Q, 0);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : update_addba_list                                     */
/*                                                                           */
/*  Description      : This functions processes the ADDBA Response           */
/*                                                                           */
/*  Inputs           : 1) The Block Ack Handle                               */
/*                     2) Pointer to the received ADDBA Response             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes the ADDBA Response           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void update_addba_list(ba_tx_struct_t **ba, UWORD8 *data)
{
    UWORD8 tid       = (data[5] & 0x3C) >> 2;
    void   *ht_entry = NULL;
    UWORD8 snd_delba = BFALSE;

    /* Exception */
    if((*ba) == NULL)
    {
        return;
    }

    /* Proceed only if status is successful, else fall back to normal ACK */
    if(data[3] == SUCCESSFUL_STATUSCODE)
    {
        /* Only if a request was sent, process the response */
        if(((*ba)->dialog_token == data[2]) && ((*ba)->is_ba == BTRUE))
        {
            /* Update the necessary parameters */
            if((*ba)->conn_status == BA_INIT)
            {
                UWORD16 buff_size = ((data[5] & 0xC0) >> 6) | (data[6] << 2);

                /* If parameters mismatch, delete the BA Session */
                if((*ba)->ba_policy != ((data[5] & 0x02) >> 1))
                    snd_delba = BTRUE;
                else
                {
                    (*ba)->timeout = data[7] | (data[8] << 8);

                    /* Update the Buffer-Size with the value suggested by */
                    /* the recipient if required.                         */
                    if(((*ba)->buff_size == 0) ||
                       ((*ba)->buff_size > buff_size))
                    {
                        (*ba)->buff_size = buff_size;
                    }

                    /* No point in pending for more than Buff-Size */
                    if((*ba)->max_pend >  (*ba)->buff_size)
                        (*ba)->max_pend  = (*ba)->buff_size;

#ifndef DISABLE_BABAR_SYNC
                    (*ba)->max_pend  = (*ba)->buff_size/2;
                    (*ba)->bar_int   = MIN((*ba)->buff_size/2, MAX_BAR_INTERVAL);
#else /* DISABLE_BABAR_SYNC */
                     (*ba)->bar_int = (*ba)->max_pend = (*ba)->buff_size;
#endif /* DISABLE_BABAR_SYNC */

                    (*ba)->pend_limit  = (*ba)->buff_size;
                    (*ba)->win_end     = SEQNO_ADD((*ba)->win_start,
                                                   ((*ba)->buff_size - 1));

                    (*ba)->conn_status = BA_INPROGRESS;
                    /* Extract non-11e specific parameters from the frame */
                    ht_entry = get_ht_handle_entry((*ba)->ba_data.entry);
                    extract_addba_resp_ht_params(ht_entry, *ba, data);

                    /* Prepare the Skeleton BAR frame with given information */
                    prepare_blockack_req((*ba)->bar_frame, (*ba), tid);

                    /* ITM-DEBUG */
                    TROUT_DBG4("Block-Ack TX Session Successfully Setup\n");
                    TROUT_DBG4("TID=%d, BufSize=%d, BAR int=%d, MAX pend=%d,conn_status=%d\n",
                        (*ba)->ba_data.tid, (*ba)->buff_size, (*ba)->bar_int,
                        (*ba)->max_pend, (*ba)->conn_status);
                }
            }
        }
        else
            snd_delba = BTRUE;
    }
    else
    {
        /* Only if the tokens match, process the packet */
        /* If the request was sent, process the packet  */
        if(((*ba)->dialog_token == data[2]) && ((*ba)->is_ba == BTRUE))
        {
            /* Status was not successful, so delete the bitmap if exists */
            reset_tx_ba_handle(ba, tid);
        }
        else
            snd_delba = BTRUE;
    }

    if(snd_delba == BTRUE)
    {
        send_delba((*ba)->dst_addr, tid, INITIATOR, UNSPEC_QOS_REASON);
        reset_tx_ba_handle(ba, tid);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_delba                                               */
/*                                                                           */
/*  Description   : DEL BA frames are sent using this function.              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC header                             */
/*                  2) TID of the packet                                     */
/*                  3) ACK Policy                                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : DEL BA frames are sent using this function.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_delba(UWORD8 *addr, UWORD8 tid, UWORD8 initiator, UWORD8 reason)
{
    UWORD16 delba_len  = 0;
    UWORD8  *delba_req = 0;

	TROUT_FUNC_ENTER;
#ifdef DEBUG_MODE
    g_mac_stats.basenddelba++;
#endif /* DEBUG_MODE */

    delba_req = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle, DELBA_LEN);

    if(delba_req == NULL)
    {
		TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif		
        return;
    }

    /* Prepare the DELBA frame */
    delba_len = prepare_delba_req(delba_req, addr, tid, initiator, reason);

    /* Send this frame to the required destination */
    tx_mgmt_frame(delba_req, delba_len, HIGH_PRI_Q, 0);

#ifdef DEBUG_MODE
    if(mget_ImmediateBlockAckOptionImplemented() == TV_TRUE)
    {
        TROUT_DBG3("DELBA Frame Sent:Initiator = %d TID = %d Reason=%d\n\r",
                                                initiator, tid, reason);
		TROUT_DBG3("%s: addr=%02x:%02x:%02x:%02x:%02x:%02x\n", 
							__FUNCTION__, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    }
#endif /* DEBUG_MODE */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : buffer_ba_rx_frame                                    */
/*                                                                           */
/*  Description      : This functions adds the rx-frame to the BA Buffer     */
/*                                                                           */
/*  Inputs           : 1) The Block Ack Handle                               */
/*                     2) Pointer to the list of Rx-Descriptors              */
/*                     3) Number of Rx-Descriptors                           */
/*                     4) The length of  received frame                      */
/*                     5) Sequence no. of received frame                     */
/*                     6) Cipher Type of the received frame                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions adds the rx-frame to the BA Buffer     */
/*                     sorted according to the sequence number               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : True on success; False otherwise                      */
/*                                                                           */
/*  Issues           : Fragment Number is not considered while buffering.    */
/*                                                                           */
/*****************************************************************************/

BOOL_T buffer_ba_rx_frame(ba_rx_struct_t **ba, UWORD8 *rx_base_dscr,
                          UWORD8 num_dscr, UWORD8 *msa, UWORD16 rx_len,
                          UWORD16 seq_num, CIPHER_T ct)
{
    bfd_rx_pkt_t *pkt_ptr  = NULL;

    /* Get the pointer to the buffered packet */
    pkt_ptr = buffer_frame_in_reorder_q(*ba, seq_num);

    /* Update the buffered receive packet details */
    pkt_ptr->rx_len    = rx_len;
    pkt_ptr->seq_num   = seq_num;
    pkt_ptr->ct        = ct;
    pkt_ptr->msa       = msa;
    pkt_ptr->dscr      = (UWORD32 *)rx_base_dscr;
    pkt_ptr->num_dscr  = num_dscr;

    (*ba)->buff_cnt++;
    //printk("%s: base=0x%p, num=%d, buf_cnt=%d\n", __func__, rx_base_dscr, num_dscr, (*ba)->buff_cnt);

#ifdef DEBUG_MODE
    g_mac_stats.bapendingrx++;
#endif /* DEBUG_MODE */

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : process_ba_rx_buff_frame                              */
/*                                                                           */
/*  Description      : This functions processes the buffered frames          */
/*                                                                           */
/*  Inputs           : 1) The buffer packet pointer                          */
/*                     2) Priority of the received frame                     */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes the buffered frames          */
/*                     This function is evoked after the BAR or BAR timeout  */
/*                     takes places                                          */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
void process_ba_rx_buff_frame(ba_rx_struct_t *ba, bfd_rx_pkt_t *pkt_ptr,
                              UWORD8 tid)
{
    wlan_rx_t       wlan_rx   = {0};

	TROUT_FUNC_ENTER;
#ifdef DEBUG_MODE
    g_mac_stats.bapendingrxhosttxd++;
#endif /* DEBUG_MODE */

    wlan_rx.msa           = pkt_ptr->msa;
    wlan_rx.ct            = pkt_ptr->ct;
    wlan_rx.rx_len        = pkt_ptr->rx_len;
    wlan_rx.priority_val  = tid;
    wlan_rx.sa_entry      = ba->ba_data.entry;
    wlan_rx.service_class = BLOCK_ACK;
    wlan_rx.base_dscr     = pkt_ptr->dscr;
    wlan_rx.num_dscr      = pkt_ptr->num_dscr;
    wlan_rx.sub_type      = get_sub_type(wlan_rx.msa);
    wlan_rx.frm_ds        = get_from_ds(wlan_rx.msa);
    wlan_rx.to_ds         = get_to_ds(wlan_rx.msa);

    /* Extract Addresses from the MAC Header */
    get_address1(wlan_rx.msa, wlan_rx.addr1);
    get_address2(wlan_rx.msa, wlan_rx.addr2);
    get_address3(wlan_rx.msa, wlan_rx.addr3);

    /* Update the SA, DA & BSSID pointers to corresponding addr1, addr2 or   */
    /* addr3 fields of wlan_rx structure.                                    */
    set_SA_DA_BSSID_ptr(&wlan_rx);

	if(wlan_rx.sa == NULL || wlan_rx.da == NULL)	//add by chengwg.
	{
//		printk("base: 0x%p, num: %d\n", pkt_ptr->dscr, pkt_ptr->num_dscr);
		TROUT_RX_DBG4("%s: frm_ds=%d, to_ds=%d\n", __func__, wlan_rx.frm_ds, wlan_rx.to_ds);
//		BUG();	//dbug!

		//xuanyang, 2013-10-31, free dscr of non-assoc entry
        	free_host_rx_dscr_list(pkt_ptr->dscr, pkt_ptr->num_dscr);
		return;
	}

    wlan_rx.is_grp_addr = is_group(wlan_rx.addr1);

    /* Adjust the headers */
    wlan_rx.hdr_len = modify_frame_length((CIPHER_T)(wlan_rx.ct), wlan_rx.msa,
                                   &(wlan_rx.rx_len), &(wlan_rx.data_len));

    /* Perfrom security checks if applicable before passing the frames to */
    /* the for further processing.                                        */
    if(check_sec_rx_mpdu(wlan_rx.sa_entry, (CIPHER_T)wlan_rx.ct, wlan_rx.msa,
                         ba->rx_pn_val) == BTRUE)
    {
        /* Process received unicast frame further after updating the relevant */
        /* MIB statistics.                                                    */
        update_rx_mib_prot(wlan_rx.msa, wlan_rx.data_len);
        wlan_rx_data(&g_mac, (UWORD8 *)&wlan_rx);
    }
    else
    {
#ifdef DEBUG_MODE
        PRINTD2("HwEr: BaRxSecChkFail\n");
        g_mac_stats.basechkfail++;
#endif /* DEBUG_MODE */
    }

    /* Free receive descriptors (and buffers) for this receive frame.    */
    /* Important: Note that users need to be added to the receive buffer */
    /* whenever it is sent for further processing (sent to Host, or WLAN */
    /* or buffered) and freed explicitly after such processing is done.  */
    //free_rx_dscr_list(pkt_ptr->dscr, pkt_ptr->num_dscr);
    free_host_rx_dscr_list(pkt_ptr->dscr, pkt_ptr->num_dscr);	//modify by chengwg.
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : is_ba_buff_tx_pkt                                     */
/*                                                                           */
/*  Description      : This functions checks if the packet needs to be       */
/*                     buffered for Block ACK purpose.                       */
/*                                                                           */
/*  Inputs           : 1) Block ACK handle                                   */
/*                     2) Pointer to the WLAN Tx request                     */
/*                     3) Pointer to the queue pointer                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions checks the Block ACK status and        */
/*                     decides whether or not to buffer the incoming packet  */
/*                     in a pending list. If buffering is required the queue */
/*                     header is updated with the pending list queue head.   */
/*                     The function also updates the required Block ACK      */
/*                     counts and sends a Block ACK request if required.     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T, BTRUE if buffering is required                */
/*                             BFALSE, if no buffering needs to be done      */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_ba_buff_tx_pkt(ba_struct_t *ba_hdl, UWORD8 q_num, UWORD8 tid,
                         UWORD8 *tx_dscr)
{
    ba_tx_struct_t **ba = NULL;

    /* This is an exception, do nothing */
    if((tid >= 16) || (ba_hdl->ba_tx_ptr[tid] == NULL))
    {
		TX_BA_DBG("tx ba session not setup!\n");
        return BFALSE;
    }

    ba = &(ba_hdl->ba_tx_ptr[tid]);

    /* Do no buffering if the Block ACK status is false, halted or failed */
    if(((*ba)->is_ba == BFALSE) || ((*ba)->conn_status == BA_FAILED))
    {
		TX_BA_DBG("tx ba status not correct!\n");
        return BFALSE;
    }

    /* If BA session is HALTED since the STA is in sleep then just update the*/
    /* last sequence num and return                                          */
    if((*ba)->conn_status == BA_HALTED)
    {
		// 20120709 caisf add, merged ittiam mac v1.2 code
        /* Ra-Lut index in Tx descriptor is set to zero in order to prevent  */
        /* MPDU aggregation                                                  */
        set_tx_dscr_ra_lut_index((UWORD32 *)tx_dscr, 0);

        (*ba)->last_seq_num = SEQNO_ADD((*ba)->last_seq_num, 1);
		TX_BA_DBG("tx ba is halted!\n");
        return BFALSE;
    }

    /* If Block ACK status is suspended the incoming frames need to be       */
    /* buffered in the pend list. There is a maximum limit on the pending    */
    /* list size since the buffers for the pend list are in shared memory    */
    /* and the shared memory is limited.                                     */
    /* This buffering of Tx-Frames is also done for frames which arrive when */
    /* the Block-Ack session is being setup.                                 */
    if(((*ba)->conn_status == BA_SUSPENDED) || ((*ba)->conn_status == BA_INIT))
    {
#if 0  /* Original-Code with Queue Limiting */
        if(((*ba)->num_pend + (*ba)->num_pipe) <
                                        ((*ba)->pend_limit + (*ba)->buff_size))
        {
            buffer_frame_in_pending_q(*ba, tx_dscr);
        }
        else
        {
            /* The Pending-Q is full. Discard the incoming frame. */
            free_tx_dscr((UWORD32 *)tx_dscr);
#ifdef DEBUG_MODE
            g_mac_stats.bapenddrop++;
#endif /* DEBUG_MODE */
        }
#else /* Modified-Code with Queue Limiting Disabled */
        buffer_frame_in_pending_q(*ba, tx_dscr);
#endif /* 0 */
        return BTRUE;
    }

    /* The Block ACK status must be 'BA_INPROGRESS'. In this state the       */
    /* transmit count is checked to make the buffering decision.             */

    /* If the transmit count has exceeded the buffer size. Also in such a    */
    /* case a Block ACK request frame is sent. There is a maximum limit on   */
    /* the pending list size since the buffers for the pending list are in   */
    /* shared memory and the shared memory is limited.                       */

    /* The Block-Ack status must be BA_INPROGRESS. In this state the MSDUs   */
    /* are buffered in order to group them together and improve Block-ACK    */
    /* efficiency.                                                           */
    buffer_frame_in_pending_q(*ba, tx_dscr);

    /* Check whether the buffer limit has been reached */
	TX_BA_DBG("%s: num_pend=%d, max_pend=%d, os_bar_cnt=%d\n", 
				__func__, (*ba)->num_pend, (*ba)->max_pend, (*ba)->os_bar_cnt);
    if((((*ba)->num_pend >= (*ba)->max_pend) &&
         (get_curr_dequeue_limit(ba) >= get_pend_dequeue_thresh(ba))) ||
       ((*ba)->os_bar_cnt == 0) || ((*ba)->buff_timeout == 0))
    {
        void *entry = (*ba)->ba_data.entry;

        /* Transmit allowed number of frames and follow it by a BAR */
        if((*ba)->pend_alarm_on == 1)
        {
            stop_ba_timer(&((*ba)->ba_pend_alarm));
            (*ba)->pend_alarm_on = 0;
        }
        transmit_frames_ba(entry, ba, q_num, tid);
    }
    else if((*ba)->pend_alarm_on == 0)
    {
        (*ba)->pend_alarm_on = 1;
        start_ba_pend_timer(&((*ba)->ba_pend_alarm), (*ba)->buff_timeout,
                            (UWORD32)(&((*ba)->ba_data)));
#ifdef DEBUG_MODE
        g_mac_stats.baemptyQ++;
#endif /* DEBUG_MODE */
    }
    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : addba_timeout_fn                                      */
/*                                                                           */
/*  Description      : This functions processes ADDBA Timeouts               */
/*                                                                           */
/*  Inputs           : 1) The BA Alarm Data pointer                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes ADDBA Timeouts               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void addba_timeout_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void addba_timeout_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
#ifdef OS_LINUX_CSL_TYPE
	ba_alarm_data_t *data = (ba_alarm_data_t *)(container_of(work, trout_timer_struct, work)->data);
#endif /* OS_LINUX_CSL_TYPE */
   if( reset_mac_trylock() == 0 ) return;
   ALARM_WORK_ENTRY(work);
    ba_tx_struct_t **ba = (ba_tx_struct_t **)(((ba_alarm_data_t *)data)->ba);
    UWORD8          tid = ((ba_alarm_data_t *)data)->tid;
    UWORD8       *entry = ((ba_alarm_data_t *)data)->entry;
    UWORD8    direction = ((ba_alarm_data_t *)data)->direction;

    /* Flush the Pending Queue */
    dequeue_tx_ba_frame(entry, *ba, 0xFF, tid);

    send_delba((*ba)->dst_addr, tid, direction, QSTA_TIMEOUT);
    reset_tx_ba_handle(ba, tid);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE    //add by Hugh

void addba_timeout_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : ba_timeout_fn                                         */
/*                                                                           */
/*  Description      : This functions processes BA Timeouts                  */
/*                                                                           */
/*  Inputs           : 1) The BA Alarm Data pointer                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes BA Timeouts                  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void ba_timeout_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void ba_timeout_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
   if( reset_mac_trylock() == 0 ) return ;
   ALARM_WORK_ENTRY(work);
#ifdef OS_LINUX_CSL_TYPE
	ba_alarm_data_t *data = (ba_alarm_data_t *)(container_of(work, trout_timer_struct, work)->data);
#endif /* OS_LINUX_CSL_TYPE */
    ba_rx_struct_t **ba1 = (ba_rx_struct_t **)(((ba_alarm_data_t *)data)->ba);
    ba_tx_struct_t **ba2 = (ba_tx_struct_t **)(((ba_alarm_data_t *)data)->ba);

    UWORD8           tid = ((ba_alarm_data_t *)data)->tid;
    UWORD8     direction = ((ba_alarm_data_t *)data)->direction;
    BOOL_T send_ba_frame = BTRUE;
	
    if(direction == RECIPIENT)
    {
        /* If the timeout interval has been negotiated to be zero, then then BA stall */
        /* recovery is performed. Otherwise, block-ack session is directly torndown   */
		mutex_lock(&ba_rx_mutex);	//add by chengwg 2013-06-21.
		if((*ba1) != NULL)
		{
	        if((*ba1)->timeout == 0)
	        {
	            misc_event_msg_t *misc = 0;

	            /* Create a MISCELLANEOUS event with the pointer to the BA-RX ctxt and   */
	            /* post it to the event queue.                                           */
	            misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
	            if(misc != NULL)
	            {
	                misc->data = (UWORD8 *)data;
	                misc->name = MISC_BA_RX_STALL;

	                post_event((UWORD8*)misc, MISC_EVENT_QID);

	                /* Could not recover from BA Hang. Hence teardown the BA session */
	                send_ba_frame = BFALSE;
	            }
	        }

	        if((send_ba_frame == BTRUE))
	        {
	        	printk("%s: rx ba timeout!\n", __func__);
	            send_delba((*ba1)->dst_addr, tid, direction, QSTA_TIMEOUT);
	            reset_rx_ba_handle(ba1, tid);
	        }
        }
        mutex_unlock(&ba_rx_mutex);
    }
    else
    {
        /* If the timeout interval has been negotiated to be zero, then then BA stall */
        /* recovery is performed. Otherwise, block-ack session is directly torndown   */
        if((*ba2) && (*ba2)->timeout == 0)	//modify by chengwg 2013-06-21.
        {
            misc_event_msg_t *misc       = 0;

            /* Create a MISCELLANEOUS event with the pointer to the BA-TX ctxt and   */
            /* post it to the event queue.                                           */
            misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
            if(misc != NULL)
            {

                misc->data = (UWORD8 *)data;
                misc->name = MISC_BA_TX_STALL;

                post_event((UWORD8*)misc, MISC_EVENT_QID);
            }
        }

        if(send_ba_frame == BTRUE)
        {
        	printk("%s: tx ba timeout!\n", __func__);
        	
            send_delba((*ba2)->dst_addr, tid, direction, QSTA_TIMEOUT);
            reset_tx_ba_handle(ba2, tid);
        }
    }
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE
void ba_timeout_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_tx_ba_stall                                    */
/*                                                                           */
/*  Description      : This function recovers from a TX-Block-Ack Stall      */
/*                     which can occasionally occur when posting of BA or    */
/*                     BAR events fail.                                      */
/*                                                                           */
/*  Inputs           : 1) The BA Alarm Data pointer                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function moves the BA-TX window ahead.           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void handle_tx_ba_stall(ba_alarm_data_t *data)
{
    ba_tx_struct_t **ba = (ba_tx_struct_t **)data->ba;
    UWORD8         q_num = 0;

	TROUT_FUNC_ENTER;
	
    if(((*ba) == NULL) || ((*ba)->is_ba == BFALSE))
    {
		TROUT_FUNC_EXIT;
        return;
	}
#ifdef DEBUG_MODE
    g_mac_stats.batxwinstallto++;
#endif /* DEBUG_MODE */

    q_num = get_txq_num(data->tid);

    /* A BA-TX Stall can happen due to two reasons:                        */
    /* 1. MAC H/w is not able to transmit at all due to Busy Channel       */
    /* 2. The TXCI raised for the BAR was lost possibly due TXCI-Interrupt */
    /*    FIFO overrun.                                                    */
    /* The two scenarios are detected by checking the MAC H/w Queue header.*/
    /* If non-Zero, the Scenario-1 is interpreted and we continue waiting. */
    /* Else, Scenario-2 has occured and we move the window immediately.    */
    if(is_machw_q_null(q_num) == BFALSE)
    {

        restart_ba_timer(&((*ba)->ba_alarm), BA_LOSS_RECOVERY_TIMEOUT,
                         (UWORD32)(&(*ba)->ba_data));

#ifdef DEBUG_MODE
    g_mac_stats.batxwinstalltoscn1++;
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }
    else
    {
#ifdef DEBUG_MODE
        g_mac_stats.batxwinstalltoscn2++;
#endif /* DEBUG_MODE */
    }

    move_ba_window_ahead(*ba, (*ba)->last_seq_num);

    /* Enqueue more frames (if available) from the pending queue */
    /* to the transmit queue under the following conditions.     */
    /* i)  The number of pending frames is greater than the      */
    /*     preset limit.                                         */
    /* ii) The Pending alarm is not running.                     */
    if(((*ba)->num_pend > 0) &&
      (((*ba)->num_pend >= (*ba)->max_pend) ||
      ((*ba)->os_bar_cnt   == 0)            ||
      ((*ba)->pend_alarm_on == 0)))
      {
        if((*ba)->pend_alarm_on == 1)
        {
            stop_ba_timer(&((*ba)->ba_pend_alarm));
            (*ba)->pend_alarm_on = 0;
        }
          transmit_frames_ba(data->entry, ba, q_num, data->tid);
      }
      
      TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_rx_ba_stall                                    */
/*                                                                           */
/*  Description      : This function recovers from possible stalling of the  */
/*                     RX-BA Reordering queue which can happen due to some   */
/*                     misbehaviour by the Txr.                              */
/*                                                                           */
/*  Inputs           : 1) The BA Alarm Data pointer                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function flushes the reorder queue and passes    */
/*                     all the locked frames up to the host.                 */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void handle_rx_ba_stall(ba_alarm_data_t *data)
{
    ba_rx_struct_t **ba = (ba_rx_struct_t **)data->ba;

	TROUT_FUNC_ENTER;
	
    /* Check whether the Block-Ack session is still valid */
    if(((*ba) == NULL) || ((*ba)->is_ba == BFALSE))
    {
		TROUT_FUNC_EXIT;
        return;
	}
    if((*ba)->buff_cnt > 0)
    {
        UWORD16 temp_seqno = SEQNO_ADD((*ba)->win_tail, 1);
        send_frames_with_gap((*ba), data->tid, temp_seqno);
        (*ba)->win_start = temp_seqno;
        (*ba)->win_end   = SEQNO_ADD((*ba)->win_start, ((*ba)->buff_size - 1));
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : ba_pend_tx_timeout_fn                                 */
/*                                                                           */
/*  Description      : This functions processes Timeouts of BA-Pending       */
/*                     Timer.                                                */
/*                                                                           */
/*  Inputs           : 1) The BA Alarm Data pointer                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function queues the allowed number of frames     */
/*                     from the Pending-queue for transmission.              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void ba_pend_tx_timeout_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void ba_pend_tx_timeout_fn(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    misc_event_msg_t *misc       = 0;

    /* Create a MISCELLANEOUS event with the pointer to the AMSDU ctxt and   */
    /* post it to the event queue.                                           */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(misc == NULL)
    {
        /* Exception */
        return;
    }

#ifdef DEBUG_MODE
    g_mac_stats.num_buffto++;
#endif /* DEBUG_MODE */

    misc->data = (UWORD8 *)data;
    misc->name = MISC_TX_PENDING_FRAMES;

    /* Post the event */
    post_event((UWORD8*)misc, MISC_EVENT_QID);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : retx_ba_frame                                         */
/*                                                                           */
/*  Description      : This functions retransmits a buffered BA frame        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the entry structure                     */
/*                     2) Pointer to the buffered frame                      */
/*                     3) TID of the frame                                   */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions allocates shared memory for the frame  */
/*                     and transmit descriptor. It copies the pending frame  */
/*                     contents (descriptor and packet) from local memory to */
/*                     the shared memory. The new buffer in shared memory is */
/*                     then added to the required queue. In case resources   */
/*                     are not available or queue addition failed the        */
/*                     function indicates the same to the caller.            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T, BTRUE if the packet was added successfully    */
/*                             for retrannsmission, BFALSE, otherwise.       */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T retx_ba_frame(void *entry, UWORD8 *tx_dscr, UWORD8 tid)
{
    UWORD8  q_num        = 0;
    UWORD8  tx_rate      = 0;
    UWORD8  pream        = 0;
    UWORD8  srv_cls      = 0;
    UWORD32 phy_tx_mode  = 0;
    UWORD32 retry_set[2] = {0};

    /* Extract the Q-number from the Tx-descriptor */
    q_num = get_tx_dscr_q_num((UWORD32 *)tx_dscr);

    /* Get the transmit parameters for the destination */
    srv_cls = COMP_BLOCK_ACK;
#ifdef AUTORATE_FEATURE
    tx_rate = get_ht_tx_rate_to_sta(entry);
#else  /* AUTORATE_FEATURE */
    tx_rate = get_tx_rate_to_sta(entry);
#endif /* AUTORATE_FEATURE */
    pream   = get_preamble(tx_rate);
    update_retry_rate_set(1, tx_rate, entry, retry_set);
    phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)entry);

    /* Update the transmit parameters */
    set_tx_params(tx_dscr, tx_rate, pream, srv_cls, phy_tx_mode, retry_set);

    /* Set the HT power save parameters */
    set_ht_ps_params(tx_dscr, (void *)entry, tx_rate);

    /* Set the receiver address LUT index */
    set_ht_ra_lut_index(tx_dscr, get_ht_handle_entry(entry), tid, tx_rate);

    /* Update the packet status in the Tx-Descriptor */
    update_tx_dscr_pkt_status((UWORD32 *)tx_dscr, PENDING);

    /* The Fragmentation Status is set to indicate that it is a Block-Ack */
    /* retry frame.                                                       */
    set_tx_dscr_frag_status_summary((UWORD32 *)tx_dscr, 0xFFFFFFFE);

#ifdef DEBUG_MODE
    g_mac_stats.badatretx++;
#endif /* DEBUG_MODE */

    /* Queue the frame for transmission */
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        /* Exception. Do nothing. */
#ifdef DEBUG_MODE
        g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
        free_tx_dscr((UWORD32 *)tx_dscr);
    }

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.bamsduretq2mhwtxq++;
#endif /* MEASURE_PROCESSING_DELAY */

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : dequeue_tx_ba_frame                                   */
/*                                                                           */
/*  Description      : This functions dequeues the given number of frames    */
/*                     from the pending list.                                */
/*                                                                           */
/*  Inputs           : 1) Pointer to source entry                            */
/*                     2) Pointer to the Block ACK Tx structure              */
/*                     3) Number of packets to dequeue                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function  dequeues the given number of packets   */
/*                     from the pending list, prepares a descriptor and      */
/*                     adds to the queue for transmission.                   */
/*                     Note that an input of max UWORD8 value (0xFF) will    */
/*                     dequeue all the packets in the list.                  */
/*                                                                           */
/*  Outputs          : None                                                  */
/*  Returns          : None                                                  */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD8 dequeue_tx_ba_frame(void *entry, ba_tx_struct_t *ba, UWORD8 num_dq,
                           UWORD8 tid)
{
    UWORD8   num_dqed       = 0;
    UWORD8   q_num          = 0;
    UWORD8   *tx_dscr       = 0;
    UWORD8   tx_rate        = 0;
    UWORD8   pream          = 0;
    UWORD8   srv_cls        = 0;
    UWORD32  phy_tx_mode    = 0;
    UWORD8   *first_tx_dscr = NULL;
    UWORD8   *last_tx_dscr  = NULL;
    UWORD32  retry_set[2]   = {0};
    BOOL_T   bar_tx_fail    = BFALSE;

	TROUT_FUNC_ENTER;
    /* No packets need to be dequeued, return */
    if(num_dq == 0)
    {
		TROUT_FUNC_EXIT;
        return 0;
	}
    /* Traverse the pending list till it is empty or the required number of  */
    /* packets have been dequeued.                                           */
    while(1)
    {
        /* If the Block ACK session is active, check the number of frames    */
        /* that have been dequeued. If it has exceeded the given number of   */
        /* frames to dequeue, no further processing is done.                 */
        if(BTRUE == ba->is_ba)
        {
            if(num_dqed >= num_dq)
                break;
        }

        tx_dscr = remove_frame_from_pending_q(ba);
        if(tx_dscr == NULL)
            break;

        num_dqed++;

        /* Get TX parameters updated only for the first descriptor being */
        /* enqueued. Same will be used for the others that follow.       */
        if(num_dqed == 1)
        {
            q_num   = get_tx_dscr_q_num((UWORD32 *)tx_dscr);
            srv_cls = (ba->is_ba == BFALSE)? g_ack_policy : COMP_BLOCK_ACK;
#ifdef AUTORATE_FEATURE
            tx_rate = get_ht_tx_rate_to_sta(entry);
#else  /* AUTORATE_FEATURE */
            tx_rate = get_tx_rate_to_sta(entry);
#endif /* AUTORATE_FEATURE */
            pream   = get_preamble(tx_rate);
            update_retry_rate_set(1, tx_rate, entry, retry_set);
            phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)entry);
        }

        /* Update the transmit parameters in the descriptor */
        set_tx_params(tx_dscr, tx_rate, pream, srv_cls, phy_tx_mode, retry_set);
        set_ht_ps_params(tx_dscr, (void *)entry, tx_rate);
        set_ht_ra_lut_index(tx_dscr, get_ht_handle_entry(entry), tid, tx_rate);

#ifdef DEBUG_MODE
        g_mac_stats.bapendingtxwlantxd++;
#endif /* DEBUG_MODE */

        if(first_tx_dscr == NULL)
        {
            first_tx_dscr = tx_dscr;
            last_tx_dscr  = first_tx_dscr;
        }
        else
        {
            set_tx_dscr_next_addr((UWORD32 *)last_tx_dscr, (UWORD32)tx_dscr);
            last_tx_dscr = tx_dscr;
        }
    }

    if(num_dqed == 0)
    {
		TROUT_FUNC_EXIT;
        return 0;
	}
    /* Update the Window Parameters and then link a BAR frame to the blcok of */
    /* data frames. This improves the probability of AMPDU being transmitted  */
    /* with Ack-Policy set to Normal Ack.                                     */
    if(ba->is_ba == BTRUE)
    {
        ba->last_seq_num       = SEQNO_ADD(ba->last_seq_num, num_dqed);
        ba->num_pipe          += num_dqed;
        ba->cnt_from_last_bar += num_dqed;
        TX_BA_DBG("update: last_seq_num=%d, num_pipe=%d, cnt_from_last_bar=%d, num_dqed=%d\n", 
        					ba->last_seq_num, ba->num_pipe, ba->cnt_from_last_bar, num_dqed);

        tx_dscr = (UWORD8 *)get_bar_dscr(ba, tid, q_num);
        if(tx_dscr == NULL)
        {
            bar_tx_fail = BTRUE;
        }
        else
        {
            set_tx_dscr_next_addr((UWORD32 *)last_tx_dscr, (UWORD32)tx_dscr);
            last_tx_dscr = tx_dscr;
            num_dqed++;
        }
    }

    num_dq = num_dqed;

    /* Queue the frames for transmission */
    if(qmu_add_tx_packet_list(&g_q_handle.tx_handle, q_num, &first_tx_dscr,
                              &last_tx_dscr, &num_dq) != QMU_OK)
    {
        UWORD8* nxt_tx_dscr = NULL;
        /* Exception. Free the Tx-Dscrs which were not transmitted */
        tx_dscr = first_tx_dscr;

        /* num_dq now contains the number of descriptors which could not be */
        /* enqueued to the Tx-Q. Compute the number of frames successfully  */
        /* enqueued from this.                                              */
        num_dqed -= num_dq;

        while(num_dq > 0)
        {
#ifdef DEBUG_MODE
            g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
            nxt_tx_dscr = (UWORD8 *)get_tx_dscr_next_addr((UWORD32 *)tx_dscr);
            free_tx_dscr((UWORD32 *)tx_dscr);
            tx_dscr = nxt_tx_dscr;

            /* Readjust the window parameters. If BAR is linked to the Block */
            /* of frames, then it is always the last frame. The window       */
            /* parameters should not be updated based on this frame.         */
            if((bar_tx_fail == BTRUE) || (num_dq > 1))
            {
                ba->last_seq_num       = SEQNO_SUB(ba->last_seq_num, 1);
                ba->num_pipe          -= 1;
                ba->cnt_from_last_bar -= 1;
            }

            num_dq--;
        }

        bar_tx_fail = BTRUE;
    }

    if(ba->is_ba == BTRUE)
    {
        if(bar_tx_fail == BFALSE)
        {
            bar_post_tx_proc(ba);
            num_dqed--;
        }
        else
        {
            move_ba_window_ahead(ba, ba->last_seq_num);

            /* The failed BAR frame will not be retransmitted */
            ba->cnt_from_last_bar = 0;
        }
    }
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.bamsdupndq2mhwtxq += num_dqed;
#endif /* MEASURE_PROCESSING_DELAY */

	TROUT_FUNC_EXIT;
	
    return num_dqed;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_ba_req                                           */
/*                                                                           */
/*  Description      : This functions sends a Block ACK request frame.       */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                     2) TID value                                          */
/*                     3) Queue number                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions prepares abd sends a Block ACK request */
/*                     frame. It sets the status to BA_SUSPENDED and the     */
/*                     process_ba flag to BTRUE. It also updates the data    */
/*                     for the Block ACK timer and restarts the same.        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T, BTRUE if the Block ACK Request was sent       */
/*                             successfully, BFALSE, otherwise.              */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T send_ba_req(ba_tx_struct_t **ba, UWORD8 tid, UWORD8 q_num)
{
    UWORD8 *tx_dscr = 0;

    tx_dscr = (UWORD8 *)get_bar_dscr(*ba, tid, q_num);

    if(tx_dscr == NULL)
        return BFALSE;

    /* Transmit the BAR frame from the given queue */
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, tx_dscr) != QMU_OK)
    {
        /* Exception. Do nothing. */
        free_tx_dscr((UWORD32 *)tx_dscr);
        return BFALSE;
    }

    bar_post_tx_proc(*ba);

    return BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : get_bar_dscr                                          */
/*                                                                           */
/*  Description      : This function creates a TX-Dscr for a BlockAck Request*/
/*                     frame.                                                */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                     2) TID value                                          */
/*                     3) Queue number                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function creates a TX-Dscr for the BAR frame.    */
/*                     It also updates the TX-Dscr with the current TX       */
/*                     parameters and copies the BAR frame to it.            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : TX-Dscr pointer for the BAR.                          */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD32 *get_bar_dscr(ba_tx_struct_t *ba, UWORD8 tid, UWORD8 q_num)
{
    UWORD32 *tx_dscr = 0;

    /* Update the SSN in the skeleton BAR frame */
    update_blockack_req_ssn(ba);
    //printk("%s: win_start=%d\n", __func__, ba->win_start);

    /* Prepare the control frame descriptor for the BAR */
    tx_dscr = (UWORD32 *)prepare_control_frame_dscr(NULL, 0, ba->ba_data.entry,
                                         q_num, BTRUE, tid);
    if(tx_dscr == 0)
        return NULL;

    update_bar_tx_dscr_fields(ba, tx_dscr);

    return tx_dscr;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : bar_post_tx_proc                                      */
/*                                                                           */
/*  Description      : This function performs the necessary context structure*/
/*                     updates after a BAR Frame has been sucessfully        */
/*                     transmitted.                                          */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                     2) TID value                                          */
/*                     3) Queue number                                       */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This function starts the BA timer and also updates the*/
/*                     relevant context structure parameters after a BAR     */
/*                     frame has been successfully transmitted.              */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BOOL_T, BTRUE if the Block ACK Request was sent       */
/*                             successfully, BFALSE, otherwise.              */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void bar_post_tx_proc(ba_tx_struct_t *ba)
{
    UWORD16 timeout = ba->timeout;

    /* Set the data required by the Block ACK timer amd restart the Block    */
    /* ACK timer with the same.                                              */
    if(timeout == 0)
        timeout = BA_LOSS_RECOVERY_TIMEOUT;

    restart_ba_timer(&(ba->ba_alarm), timeout, (UWORD32)(&(ba->ba_data)));

    /* Increment the outstanding BAR count each time a BAR is queued and */
    /* the timer is restarted.                                           */
    ba->os_bar_cnt++;

    /* Reset the packet count from last BAR */
    ba->cnt_from_last_bar = 0;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_frames_with_gap                                  */
/*                                                                           */
/*  Description      : This functions sends all frames from the start of the */
/*                     BA-Rx window to the specified sequence number.        */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                     2) TID value                                          */
/*                     3) Sequence number                                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : All frames which lie between the start of the BA-Rx   */
/*                     window and the specified sequence number are passed   */
/*                     to the higher layer. If any frame is found missing,   */
/*                     then it is skipped and the next sequence number is    */
/*                     processed. Hence gaps can exist in the sequence       */
/*                     number of MSDUs passed.                               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : Number of MSDUs passed to the higher layer.           */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 send_frames_with_gap(ba_rx_struct_t *ba, UWORD8 tid, UWORD16 last_seqno)
{
    UWORD16      seq_num   = ba->win_start;
    bfd_rx_pkt_t *pkt_ptr  = NULL;
    UWORD8       num_frms  = 0;

	TROUT_FUNC_ENTER;
    while(seq_num != last_seqno)
    {
        if((pkt_ptr = remove_frame_from_reorder_q(ba, seq_num)) != NULL)
        {
            process_ba_rx_buff_frame(ba, pkt_ptr, tid);
            num_frms++;
            ba->buff_cnt--;
        }
        else
        {
#ifdef DEBUG_MODE
            g_mac_stats.barxmiss++;
#endif /* DEBUG_MODE */
        }

        seq_num = SEQNO_ADD(seq_num, 1);
    }

	TROUT_FUNC_EXIT;
    return num_frms;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : send_frames_in_order                                  */
/*                                                                           */
/*  Description      : This functions sends MSDUs in order to the higher     */
/*                     layer.                                                */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                     2) TID value                                          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : All MSDUs with sequence number starting from the      */
/*                     start of the BA-Rx window are processed in order and  */
/*                     are sent to the higher layer. Processing is stopped   */
/*                     when the first missing MSDU is encountered.           */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : Seqeunce number of the next expected MSDU.            */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

UWORD16 send_frames_in_order(ba_rx_struct_t *ba, UWORD8 tid)
{
    UWORD16      seq_num   = ba->win_start;
    bfd_rx_pkt_t *pkt_ptr  = NULL;

	TROUT_FUNC_ENTER;
	
    while(1)
    {
        if((pkt_ptr = remove_frame_from_reorder_q(ba, seq_num)) != NULL)
        {
            process_ba_rx_buff_frame(ba, pkt_ptr, tid);
            ba->buff_cnt--;
        }
        else
            break;

        seq_num = SEQNO_ADD(seq_num, 1);
    }

	TROUT_FUNC_EXIT;
    return seq_num;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : move_ba_window_ahead                                  */
/*                                                                           */
/*  Description      : This function moves the BA-Tx window ahead by WinSize */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The Retry-Q is flushed and the BA-Tx Window           */
/*                     parameters are updated to move the window by WinSize. */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void move_ba_window_ahead(ba_tx_struct_t *ba, UWORD16 lsn)
{
    UWORD16 seq_num = 0;

	TROUT_FUNC_ENTER;
    /* Flush out the Retry-Queue. All frames in this Q are deleted. */
    cleanup_retry_q(ba, lsn);

    /* Move the Tx-BA Window */
    ba->win_start = SEQNO_ADD(lsn, 1);

    seq_num = SEQNO_ADD(ba->win_start, (ba->buff_size - 1));

    /* Update the BA Connection Status only when the BA-TX Window moves */
    if(seq_num != ba->win_end)
    {
         ba->win_end     = seq_num;
         ba->conn_status = BA_INPROGRESS;
         
         //print_symbol("%s -> ", (unsigned long)__builtin_return_address(0));
         TX_BA_DBG("%s: set ba conn-status to 'BA_INPROGRESS'!\n", __func__);
    }
    TX_BA_DBG("%s: lsn=%d, win_start=%d, win_end=%d\n", __func__, lsn, ba->win_start, ba->win_end);

    stop_ba_timer(&(ba->ba_alarm));

    /* Start Pending Alarm if not currently running. This will */
    /* flush out the pending-Q.                                */
    if((ba->num_pend > 0) && (ba->pend_alarm_on == 0))
    {
        start_ba_pend_timer(&(ba->ba_pend_alarm), ba->buff_timeout,
                        (UWORD32)(&(ba->ba_data)));
        ba->pend_alarm_on = 1;
    }
#ifdef DEBUG_MODE
    g_mac_stats.bawinmove++;
#endif /* DEBUG_MODE */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_bar_tx_failure                                 */
/*                                                                           */
/*  Description      : This function handles updation of Tx-BA context on    */
/*                     BAR Tx-Failure.                                       */
/*                                                                           */
/*  Inputs           : 1) Pointer to destination station entry               */
/*                     2) Pointer to the Block ACK Tx structure              */
/*                     3) Queue Number of the failed BAR                     */
/*                     4) TID of the failed out BAR                          */
/*                     5) Last Sequence Number for the failed BAR            */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The Tx-Window is moved ahead and further packets are  */
/*                     transmitted if available.                             */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void handle_bar_tx_failure(void *entry, ba_tx_struct_t **ba, UWORD8 q_num,
                           UWORD8 tid, UWORD16 lsn)
{
    UWORD16 bar_lsn = MIN(lsn, (*ba)->last_seq_num);

#ifdef DEBUG_MODE
    g_mac_stats.babarfail++;
#endif /* DEBUG_MODE */

    /* Move the BA Window only when the last OS BAR Fails */
    if((*ba)->os_bar_cnt == 0)
        move_ba_window_ahead(*ba, bar_lsn);

    /* Enqueue more frames (if available) from the pending queue */
    /* to the transmit queue under the following conditions.     */
    /* i)  The number of pending frames is greater than the      */
    /*     preset limit.                                         */
    /* ii) The Pending alarm is not running.                     */
    if(((*ba)->num_pend > 0) &&
        (((*ba)->num_pend >= (*ba)->max_pend) ||
        ((*ba)->os_bar_cnt   == 0)            ||
        ((*ba)->pend_alarm_on == 0)))
    {
        if((*ba)->pend_alarm_on == 1)
        {
            stop_ba_timer(&((*ba)->ba_pend_alarm));
            (*ba)->pend_alarm_on = 0;
        }

        transmit_frames_ba(entry, ba, q_num, tid);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_data_tx_failure                                */
/*                                                                           */
/*  Description      : This function handles updation of Tx-BA context on    */
/*                     Data frame Tx-Failure.                                */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                     2) Tx-Descriptor of the failed data frame             */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The Tx-BA window parameters are updated depending     */
/*                     upon whether the failure occured before the first     */
/*                     transmission attempt or after.                        */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void handle_data_tx_failure(ba_tx_struct_t **ba, UWORD32 *tx_dscr,
                            UWORD16 seq_num)
{
    UWORD32 frag_status = get_tx_dscr_frag_status_summary(tx_dscr);

    if(get_tx_dscr_status(tx_dscr) == TX_TIMEOUT)
    {
        if((frag_status & 0x3) == PENDING_MPDU)
        {
            (*ba)->last_seq_num = SEQNO_SUB((*ba)->last_seq_num, 1);
        }
        else
        {
            move_ba_window_ahead(*ba, seq_num);
        }

#ifdef DEBUG_MODE
    g_mac_stats.badatfail++;
#endif /* DEBUG_MODE */
    }
    else if((get_tx_dscr_short_retry_count(tx_dscr) >=
                                                     mget_ShortRetryLimit()) ||
       (get_tx_dscr_long_retry_count(tx_dscr) >= mget_LongRetryLimit()))
    {
        /* The retry limit has been reached and hence the packet is  */
        /* discarded. When Block-Ack is used, only RTS frames can    */
        /* be retried. If the First Fragment status is PENDING, then */
        /* this is a fresh MPDU without an assigned sequence number. */
        /* Hence update the Window parameters for this frame.        */
        if((frag_status & 0x3) == PENDING_MPDU)
        {
            (*ba)->last_seq_num = SEQNO_SUB((*ba)->last_seq_num, 1);
        }
        else
        {
            move_ba_window_ahead(*ba, seq_num);
        }

#ifdef DEBUG_MODE
    g_mac_stats.badatfail++;
#endif /* DEBUG_MODE */
    }
    else
    {
        /* Do Nothing */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : process_ba_frame                                      */
/*                                                                           */
/*  Description      : This functions processes the received 11e/11n Block   */
/*                     ACK frame                                             */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Block ACK Tx structure              */
/*                     2) TID                                                */
/*                     3) Last sequence number at the time of BAR queuing    */
/*                     4) Start Sequence Number field of the received BA     */
/*                     5) Bitmap field of the received BA                    */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : This functions processes the received Block ACK frame */
/*                     information. It dequeues frames from the buffer list  */
/*                     and frees the frames that have been acknowledged. If  */
/*                     the frame is not acknowledged it is added to the H/w  */
/*                     shared memory queue. On transmit complete this frame  */
/*                     will again be added to the buffer list.               */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void process_ba_frame(void *pv_ba, UWORD8 tid, UWORD16 last_seq_num,
                      UWORD16 start_seq_num, UWORD8 *bitmap)
{
    UWORD8       i             = 0;
    UWORD16      first_retx    = 0;
    UWORD16      seq_num       = 0;
    BOOL_T       re_tx_req  = BFALSE;
    UWORD16      num_miss      = 0;
    ba_tx_struct_t **ba        = (ba_tx_struct_t **) pv_ba;
    UWORD8       *tx_dscr    = NULL;

	TX_BA_DBG("%s: tid=%d, last_seq_num=%d, start_seq_num=%d, win_start=%d, win_end=%d\n", 
				__func__, tid, last_seq_num, start_seq_num, (*ba)->win_start, (*ba)->win_end);
	
    /* Check whether this is a useful Block-Ack */
    if((BTRUE == seqno_leq(start_seq_num, last_seq_num)) &&
       (BTRUE == seqno_geq(start_seq_num, SEQNO_SUB((*ba)->win_start, 63))))
    {	
        if(BTRUE == seqno_gt(start_seq_num, (*ba)->win_start))
        {
            /* If the SSN in the Block-Ack has moved beyond the WinStart     */
            /* then all the unacked MSDUs between SSN & Winstart are deleted */
            /* from the Retry Queue.                                         */
            seq_num = (*ba)->win_start;
            while(seq_num != start_seq_num)
            {
                tx_dscr = remove_frame_from_retry_q(*ba, seq_num);
                if(tx_dscr != NULL)
                {
                    /* Delete the buffered packet from the buffer list */
                    free_tx_dscr((UWORD32 *)tx_dscr);
                    (*ba)->num_pipe--;
                }

                seq_num = SEQNO_ADD(seq_num, 1);
            }
            /* Update WinStart */
            (*ba)->win_start = start_seq_num;
        }
        else
        {
            i = SEQNO_SUB((*ba)->win_start, start_seq_num);
        }
    }
    else
        return;

    /* Process the MSDUs from the starting sequence number to the last */
    /* transmitted sequence number in the Block ACK bitmap.            */
    seq_num = (*ba)->win_start;

    while((seqno_leq(seq_num, last_seq_num) == BTRUE))
    {
        /* Check the buffer list for the packet with this sequence number */
        tx_dscr = remove_frame_from_retry_q(*ba, seq_num);
        //printk("remove_frame_from_retry_q: seq_num=%d\n", seq_num);

       /* HACK-ALERT: This condition can occur when the Data frame has */
       /* been retransmitted in response to a BA and the subsequent BA */
       /* NACKs the same frame.                                        */
       /* Some change is required to handle this properly.             */
       if((tx_dscr == NULL) && (BTRUE == is_ba_missing_msdu(bitmap, i)))
       {
#ifdef DEBUG_MODE
            g_mac_stats.babufmiss++;
#endif /* DEBUG_MODE */
            if(re_tx_req == BFALSE)
            {
                first_retx = seq_num;
                re_tx_req  = BTRUE;
            }
       }

        /* In case this packet has already been successfully transmitted     */
        /* earlier it is possible that no packet with matching sequence      */
        /* number is found in the buffer list. Ignore this and continue  */
        /* with the following sequence numbers.                          */
        if(tx_dscr != NULL)
        {
             /* Check whether the MSDU is reported missing */
            if(BTRUE == is_ba_missing_msdu(bitmap, i))
            {
                /* If this is the first retry requirement detected set the  */
                /* flag and also save the sequence number of the first MSDU */
                /* to retry.                                                */
                if(re_tx_req == BFALSE)
                {
                    first_retx = seq_num;
                    re_tx_req  = BTRUE;
                }

                /* Call a function to retry the given packet pointer. In case */
                /* this function is not able to successfully queue the frame  */
                /* for retransmission, send a DELBA and terminate the Block   */
                /* ACK session.                                               */
                if(retx_ba_frame((*ba)->ba_data.entry, tx_dscr, tid) == BFALSE)
                {
                    send_delba((*ba)->dst_addr, tid, INITIATOR,
                               UNSPEC_QOS_FAIL);
                    reset_tx_ba_handle(ba, tid);
                    return;
                }

                num_miss++;

                /* Increment the count from last BAR */
                (*ba)->cnt_from_last_bar++;

                /* Update the autorate TX failure statistics for this station */
                ar_tx_failure((*ba)->ba_data.entry);
            }
            else
            {
                (*ba)->num_pipe--;
                /* Autorate counters updated */
                ar_tx_success((*ba)->ba_data.entry, 0);

            	/* Delete the buffered packet from the buffer list */
                free_tx_dscr((UWORD32 *)tx_dscr);
            }
        }
        seq_num = SEQNO_ADD(seq_num, 1);
        i++;
    }

    /* Update the Block-Ack Transmit Window */
    if(re_tx_req == BTRUE)
    {
        if((((*ba)->win_start == first_retx) &&
            ((*ba)->win_end == (*ba)->last_seq_num)) ||
            ((*ba)->num_pend == 0)                   ||
            ((*ba)->cnt_from_last_bar >= (*ba)->bar_int))
        {
            UWORD8 q_num = get_txq_num(tid);

            /* HACK-ALERT: This is a hack to recover from a stalled window.  */
            /* This condition occurs when the Data Frames being retransitted */
            /* no longer exist in the Retry or MAC H/w Q since they have     */
            /* already Timed-Out.                                            */
            /* Some Design change is required to handle this condition       */
            /* properly.                                                     */
            if(num_miss == 0)
               (*ba)->batxwinstall++;
            else
                (*ba)->batxwinstall = 0;

            if((*ba)->batxwinstall > BATX_WIN_STALL_THRESHOLD)
            {

#ifdef DEBUG_MODE
    g_mac_stats.batxwinstallco++;
#endif /* DEBUG_MODE */

                move_ba_window_ahead(*ba, (*ba)->last_seq_num);
                (*ba)->batxwinstall = 0;

                return;
            }
            else
            {
                /* Window is full and has not been moved. Hence transmit a BAR */
                if((num_miss > 0) && (BFALSE == send_ba_req(ba, tid, q_num)))
                {
                    move_ba_window_ahead(*ba, (*ba)->last_seq_num);

                    /* The failed BAR frame will not be retransmitted */
                    (*ba)->cnt_from_last_bar = 0;
#ifdef DEBUG_MODE
                    g_mac_stats.babartxqfail++;
#endif /* DEBUG_MODE */
                    return;
                }
            }
        }
        else
            (*ba)->batxwinstall = 0;


        /* Move WinStart to the first packet to be retried */
        (*ba)->win_start = first_retx;
    }
    else
    {
        /* If no retries are required set the sequence number to the next    */
        /* sequence number after the last sequence number Acked.             */
        (*ba)->win_start = seq_num;
        (*ba)->batxwinstall = 0;
    }

    seq_num = SEQNO_ADD((*ba)->win_start, ((*ba)->buff_size - 1));

    /* Update the BA Connection Status only when the BA-TX Window moves */
    if(seq_num != (*ba)->win_end)
    {
        (*ba)->win_end     = seq_num;
        (*ba)->conn_status = BA_INPROGRESS;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : handle_ba_tx_comp                                     */
/*                                                                           */
/*  Description      : This function handles any processing to be done after */
/*                     Tx-Complete Interrupt for frames related to an on     */
/*                     going block-ack session.                              */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Tx Descriptor of the frame          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : The TxC processing following frames are handled here. */
/*                     1) ADDBA Request frame                                */
/*                     2) Block-Ack request frame                            */
/*                     3) QoS Data frames under blcok-ack session            */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/
// 20120709 caisf mod, merged ittiam mac v1.2 code
//void handle_ba_tx_comp(UWORD8 *dscr, void *entry)
void handle_ba_tx_comp(UWORD8 *dscr, void *entry, UWORD8* msa)
{
    UWORD8 tid       = 0;
    UWORD16 seq_num  = 0;
    //UWORD8 *msa      = 0;
    ba_struct_t   *ba_ctxt = NULL;
    TYPESUBTYPE_T frm_st   = DATA;

	TROUT_FUNC_ENTER;
	
    ba_ctxt = get_ba_handle_entry(entry);

    /* Further processing is required only when a Block-Ack session has been */
    /* initiated to the AP/STA.                                              */
    if((ba_ctxt == NULL) || (ba_ctxt->num_ba_tx == 0))
    {
		TROUT_FUNC_EXIT;
        return;
    }
	// 20120709 caisf masked, merged ittiam mac v1.2 code
    //msa = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);
    frm_st = get_sub_type(msa);

    /* Check if the packet is ADDBA and if so update the Block ACK handle    */
    /* with the starting sequence number. This is appended by hardware to    */
    /* the ADDBA frame at the time of transmission                           */
    /* This function also extracts other paranmeters that will be used later.*/
    if((frm_st == ACTION) && (is_addba_frame(msa, &tid, &seq_num) == BTRUE))
    {
        update_ba_start_seq_num(ba_ctxt, tid, seq_num);
    }
    else if(frm_st == BLOCKACK_REQ)	//BAR
    {
        UWORD8         tid      = 0;
        ba_tx_struct_t **ba     = NULL;

        /* Get the TID and the corresponding BA transmit handle */
        tid = (msa[17] & 0xF0) >> 4;
        ba  = &(ba_ctxt->ba_tx_ptr[tid]);

        /* If no BA transmit handle is found, return */
        if((*ba) == NULL)
        {
#ifdef DEBUG_MODE
            g_mac_stats.babarcvdigned++;
#endif /* DEBUG_MODE */
			TROUT_FUNC_EXIT;
            return;
        }

#ifdef DEBUG_MODE
        g_mac_stats.babartxd++;
#endif /* DEBUG_MODE */

        if((*ba)->is_ba == BTRUE && ((*ba)->conn_status != BA_HALTED))
        {
            UWORD8  q_num        = 0;
            UWORD16 ba_ssn       = 0;
            UWORD16 ba_ctrl      = 0;
            UWORD16 ba_sw_barlsn = 0;
            UWORD32 ba_hw_flags  = 0;
            UWORD32 ba_bmap_w0   = 0;
            UWORD32 ba_bmap_w1   = 0;
            UWORD8  bitmap[8]    = {0};

            /* Get the transmit queue number for this TID */
            q_num = get_txq_num(tid);

            /* Extract the BA H/w flags from the BAR Tx Dscr */
            ba_hw_flags = get_bar_tx_dscr_ba_hw_flags((UWORD32 *)dscr);

            /* Extract the Last Sequence Number at time of BAR queuing */
            ba_sw_barlsn = get_bar_tx_dscr_ba_sw_barlsn((UWORD32 *)dscr);

            /* Extract the Block ACK BAR Control field */
            ba_ctrl = get_bar_tx_dscr_ba_ctrl((UWORD32 *)dscr);

            /* Decrement the outstanding BAR count */
            if((*ba)->os_bar_cnt != 0)
                (*ba)->os_bar_cnt--;

            /* Stop the BA timer if the outstanding BAR count becomes zero */
            if((*ba)->os_bar_cnt == 0)
                stop_ba_timer(&((*ba)->ba_alarm));

            /* Check whether the BA is currently useful */
            if(BTRUE == seqno_lt(ba_sw_barlsn, (*ba)->win_start))
            {
				TROUT_FUNC_EXIT;
                return;
			}
				
            /* If the BAR times out or no corresponding BA is received, the  */
            /* BA Tx window is moved forward beyond the block of frames for  */
            /* which the BAR frame was generated.                            */
            if((is_tx_success((UWORD32 *)dscr)  != BTRUE) ||
               (IS_BA_INFO_PRESENT(ba_hw_flags) != BTRUE) ||
               (is_valid_ba(ba_ctrl, tid)       == BFALSE))
            {
                handle_bar_tx_failure(entry, ba, q_num, tid, ba_sw_barlsn);
                TROUT_FUNC_EXIT;
                return;
            }

            /* A BA has been received in response to this BAR. The required  */
            /* information is updated in the BAR Tx Descriptor. The same is  */
            /* extracted here and the BA is then processed.                  */

            /* Block ACK Start Sequence Number field */
            ba_ssn = get_bar_tx_dscr_ba_ssn((UWORD32 *)dscr);
			
            /* Block ACK bitmap */
            ba_bmap_w1 = get_bar_tx_dscr_ba_bmap_w0((UWORD32 *)dscr);
            ba_bmap_w0 = get_bar_tx_dscr_ba_bmap_w1((UWORD32 *)dscr);
            bitmap[0]  = (ba_bmap_w0 & 0x000000FF);
            bitmap[1]  = ((ba_bmap_w0 & 0x0000FF00)) >> 8;
            bitmap[2]  = ((ba_bmap_w0 & 0x00FF0000)) >> 16;
            bitmap[3]  = ((ba_bmap_w0 & 0xFF000000)) >> 24;
            bitmap[4]  = (ba_bmap_w1 & 0x000000FF);
            bitmap[5]  = ((ba_bmap_w1 & 0x0000FF00)) >> 8;
            bitmap[6]  = ((ba_bmap_w1 & 0x00FF0000)) >> 16;
            bitmap[7]  = ((ba_bmap_w1 & 0xFF000000)) >> 24;

            /* Process the BA frame */
            process_ba_frame(ba, tid, ba_sw_barlsn, ba_ssn, bitmap);

            /* Transmit further frames from the pending queue if possible */
            if((*ba) != NULL)
            {
                /* Check whether the buffer limit has been reached */
                if((((*ba)->num_pend >= (*ba)->max_pend) &&
                     (get_curr_dequeue_limit(ba) >= get_pend_dequeue_thresh(ba))) ||
                   ((*ba)->os_bar_cnt   == 0)                                     ||
                   ((*ba)->buff_timeout == 0))
                {
                    void *entry = (*ba)->ba_data.entry;

                    /* Transmit allowed number of frames and follow it by a BAR */
                    if((*ba)->pend_alarm_on == 1)
                    {
                        stop_ba_timer(&((*ba)->ba_pend_alarm));
                        (*ba)->pend_alarm_on = 0;
                    }

                    transmit_frames_ba(entry, ba, q_num, tid);
                }
                else if((*ba)->pend_alarm_on == 0)
                {
                    (*ba)->pend_alarm_on = 1;
                    start_ba_pend_timer(&((*ba)->ba_pend_alarm), (*ba)->buff_timeout,
                                        (UWORD32)(&((*ba)->ba_data)));
                }
            }
        }
    }
    /* BA is implemented only for the frames of type DATA */
    else if(frm_st == QOS_DATA)
    {
        UWORD8  tid   = get_tid_value(msa);
        ba_tx_struct_t **ba = NULL;

        ba = &(ba_ctxt->ba_tx_ptr[tid]);

        seq_num = get_sequence_number(msa);
        //printk("%s: Qos data send complete, seq_num=%d!\n", __func__, seq_num);

        /* If BA is enabled for the current TID, update the Buffer list for */
        /* Block ack handle                                                 */
        if(((*ba) != NULL) && ((*ba)->is_ba == BTRUE))
        {
            if(is_tx_success((UWORD32 *)dscr) != BTRUE)
            {
                /* TBD: Update Debug Counters */
                handle_data_tx_failure(ba, (UWORD32 *)dscr, seq_num);
                (*ba)->num_pipe--;
				TROUT_FUNC_EXIT;
                return;
            }

            /* IF STATE is not halted */
            if(((*ba)->conn_status == BA_INPROGRESS) ||
               ((*ba)->conn_status == BA_SUSPENDED))
            {
                seq_num = get_sequence_number(msa);
                if(is_tx_data_useful(*ba, seq_num) == BTRUE)
                {
                	//printk("Qos data: seq_num=%d!\n", seq_num);
                    /* Buffer the frame in the Retry-Q */
                    buffer_frame_in_retry_q(*ba, seq_num, dscr);

                    /* Deletion of the buffered Tx-Descriptor is prevented by*/
                    /* adding an extra user to the Tx-descritor and all the  */
                    /* associated buffers.                                   */
                    add_user_tx_dscr((UWORD32 *)dscr);

#ifdef DEBUG_MODE
                    g_mac_stats.banumbuff++;
#endif /* DEBUG_MODE */
                }
                else
                {
                    (*ba)->num_pipe--;
                }
            }
            else
            {
                /* Do Nothing */
            }
        }
    }

	TROUT_FUNC_EXIT;
    return;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : ps_buff_ba_frames                                     */
/*                                                                           */
/*  Description      : This function handles the event of the block-ack      */
/*                     recipient having entered power-save mode.             */
/*                     NOTE: THIS FUNCTION SHOULD NOT BE CALLED IN STA MODE  */
/*                                                                           */
/*  Inputs           : 1) Pointer to the STA's association entry             */
/*                     2) Pointer to the Block ACK Tx structure              */
/*                     3) TID of the stream                                  */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : When the Block-Ack recipient enters power save mode,  */
/*                     all the frames in the retry queue and the pending     */
/*                     queue are moved to the PS queue. Also, the Block-Ack  */
/*                     session is HALTED.                                    */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : None                                                  */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

void ps_buff_ba_frames(void *entry, ba_tx_struct_t **ba, UWORD8 tid)
{

    list_buff_t *qh             = NULL;
    BOOL_T ps_del_en_ac         = BFALSE;
    UWORD16 num_el_moved        = 0;


    /* Suspend the block-ack session until the STA wakes up */
    stop_ba_timer(&((*ba)->ba_alarm));
    stop_ba_timer(&((*ba)->ba_pend_alarm));
    (*ba)->conn_status = BA_HALTED;

    /* Delete all existing frames in the Retry queue */
    flush_retry_q(*ba);

    /* Check if TID is delivery enabled AC and merge the BA pending queue    */
    /* to the appropriate PS Queue                                           */
    /* NO LIMIT CHECKS ARE DONE to avoid any packet drop at this point       */

    qh = get_ps_q_ptr_mode(entry, tid, &ps_del_en_ac);
    num_el_moved = (*ba)->pend_list.count;
    merge_list(qh, &((*ba)->pend_list));

    update_ps_flags(entry, num_el_moved, ps_del_en_ac);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : filter_wlan_ba_serv_cls                               */
/*                                                                           */
/*  Description      : This function checks whether the received frame       */
/*                     should be filtered based on the Ack-Policy.           */
/*                                                                           */
/*  Inputs           : 1) Pointer to the Base Rx-Descriptor                  */
/*                     2) Number of Rx-Descriptors in the current MPDU       */
/*                     3) Pointer to the WLAN RX information structure       */
/*                     4) Pointer to Block-Ack RX Context Structure          */
/*                                                                           */
/*  Globals          : None                                                  */
/*                                                                           */
/*  Processing       : If the rxd frame is under a block-ack session, and is */
/*                     out-of-order, then it is buffered. Otherwise, it is   */
/*                     allowed to be passed to the host.                     */
/*                                                                           */
/*  Outputs          : None                                                  */
/*                                                                           */
/*  Returns          : BTRUE - If frame should be filtered.                  */
/*                     BFALSE - If the frame can be passed to the host.      */
/*                                                                           */
/*  Issues           : None                                                  */
/*                                                                           */
/*****************************************************************************/

BOOL_T filter_wlan_ba_serv_cls(wlan_rx_t      *wlan_rx,
                               ba_rx_struct_t **ba)
{
    UWORD8   *msa    = wlan_rx->msa;
    UWORD16  rx_len  = wlan_rx->rx_len;
    CIPHER_T ct      = (CIPHER_T)wlan_rx->ct;
    UWORD8   pri_val = wlan_rx->priority_val;
    UWORD8   *entry  = (UWORD8 *)wlan_rx->sa_entry;
    UWORD8   sc      = wlan_rx->service_class;
    BOOL_T  filter_frame = BFALSE;
    UWORD8  *rx_dscr = (UWORD8 *)wlan_rx->base_dscr;
    UWORD8  num_dscr = wlan_rx->num_dscr;

    if(mget_HighThroughputOptionImplemented() == TV_TRUE)
    {
        /* Check if block ACK is enabled for current TID */
        if(((*ba) != NULL) && ((*ba)->is_ba == BTRUE))
        {
            /* Filtering is applicable only for QoS-Data frames */
            if(wlan_rx->sub_type != QOS_DATA)
                return BFALSE;

            if(wlan_rx->is_grp_addr == BFALSE)
            {
                UWORD16 seq_num = get_sequence_number(msa);

                if(BTRUE == seqno_geq(seq_num, (*ba)->win_start))
                {
                    /* The BA-Timer is restarted each time a frame under the */
                    /* BA-session is successfully received.                  */
                    if((*ba)->timeout > 0)
                    {
                        restart_ba_timer(&((*ba)->ba_alarm), (*ba)->timeout,
                                         (UWORD32)(&(*ba)->ba_data));
                    }

                    if(seqno_geq(seq_num, (*ba)->win_tail) == BTRUE)
                        (*ba)->win_tail = seq_num;

                    /* If the sequence number of the frame is same as Win-Start, */
                    /* and the reorder queue is empty, then the new packet need  */
                    /* not be buffered in the reordering queue.                  */
                    /* For this type of frames just update the window parameters */
                    /* and pass it up to the host                                */
                    if((seq_num == (*ba)->win_start) && ((*ba)->buff_cnt == 0))
                    {
                        (*ba)->win_start = SEQNO_ADD((*ba)->win_start, 1);
                        (*ba)->win_end   = SEQNO_ADD((*ba)->win_end, 1);
#ifdef DEBUG_MODE
                        g_mac_stats.banobuffrx++;
#endif /* DEBUG_MODE */
                        if(check_sec_rx_mpdu(entry, ct, msa, (*ba)->rx_pn_val) == BFALSE)
                        {
                            filter_frame = BTRUE;
                            printk("%s: check sec rx mpdu is fail!\n", __func__);
#ifdef DEBUG_MODE
                            g_mac_stats.basechkfail++;
#endif /* DEBUG_MODE */
                        }
                    }
                    else
                    {
#if 0
                        /* Buffer the new MSDU */
                        if(buffer_ba_rx_frame(ba, rx_dscr, num_dscr, msa, rx_len,
                                              seq_num, ct) == BTRUE)
                        {
                            /* Extra user is registered for the buffered frame */
                            add_user_rx_dscr_list((UWORD32 *)rx_dscr, num_dscr, 1);
                        }
#else
			   //rx_frame_show(wlan_rx);	//dbug!
			  buffer_ba_rx_frame(ba, rx_dscr, num_dscr, msa, rx_len, seq_num, ct);
			  rx_dscr_list_add_user((UWORD32 *)rx_dscr, num_dscr);	//chwg add, debug.
#endif
                        /* Pass the reordered MSDUs up the stack */
                        reorder_ba_rx_buffer_data((*ba), pri_val, seq_num);

                        if((*ba)->timeout == 0)
                        {
                            if((*ba)->buff_cnt > 0)
                            {
                                restart_ba_timer(&((*ba)->ba_alarm), BA_LOSS_RECOVERY_TIMEOUT,
                                                 (UWORD32)(&(*ba)->ba_data));
                             }
                             else
                             {
                                stop_ba_timer(&((*ba)->ba_alarm));
                             }
                        }

                        filter_frame = BTRUE;
                    }
                }
                else
                {
                    /* Discard the frame */
                    filter_frame = BTRUE;
#ifdef DEBUG_MODE
                    g_mac_stats.barxdatoutwin++;
#endif /* DEBUG_MODE */
                }
            }
            /* Check for Sync loss and flush the reorder queue when one is detected */
            if(((*ba)->win_tail == SEQNO_SUB((*ba)->win_start, 1)) &&
                ((*ba)->buff_cnt > 0))
            {
                flush_reorder_q(*ba);
            }
        }
        else if(sc == BLOCK_ACK)
        {
            filter_frame = BTRUE;
        }
    }

    return filter_frame;
}
#endif /* MAC_802_11N */

