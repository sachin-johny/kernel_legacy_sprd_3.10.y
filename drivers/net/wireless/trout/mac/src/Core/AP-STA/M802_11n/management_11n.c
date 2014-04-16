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
/*  File Name         : management_11n.c                                     */
/*                                                                           */
/*  Description       : This file contains all the functions related to      */
/*                      management in 802.11n mode.                          */
/*                                                                           */
/*  List of Functions : misc_11n_event                                       */
/*                      set_11n_p_action_req                                 */
/*                      update_tx_mib_11n                                    */
/*                      update_rx_mib_11n                                    */
/*                      handle_mlme_amsdu_start                              */
/*                      handle_mlme_amsdu_end                                */
/*                      handle_mlme_ampdu_start                              */
/*                      handle_mlme_ampdu_end                                */
/*                      handle_mlme_delba_req                                */
/*                      handle_mlme_addba_req                                */
/*                      handle_wlan_addba_req                                */
/*                      handle_wlan_addba_rsp                                */
/*                      handle_wlan_delba                                    */
/*                      handle_action_req_query_tid                          */
/*                      handle_action_req_query_all                          */
/*                      process_q_all_act_req_entry                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "amsdu_aggr.h"
#include "maccontroller.h"
#include "management_11n.h"
#include "frame_11n.h"
#include "core_mode_if.h"
#include "blockack.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8 g_rifs_prot_enabled = 0;
UWORD8 g_ampdu_lut_index_table[AMPDU_LUT_IDX_BMAP_LEN];

#ifdef NO_ACTION_RESET
save_action_req_t g_saved_action_req[MAX_NUM_SAVED_ACTION_REQ];
#endif /* NO_ACTION_RESET */

BOOL_T g_tx_ba_setup = BFALSE;	//add by chengwg!

UWORD8 g_act_req_q_type  = 0;
UWORD8 g_act_req_q_tid   = 0;
UWORD8 g_act_req_q_ra[6] = {0};

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_11n                                           */
/*                                                                           */
/*  Description   : This function performs all 11n related initializations.  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All 11n tables, structures and variables are initialized */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_11n(void)
{
    initialize_ba();
    init_lut(g_ampdu_lut_index_table, AMPDU_LUT_IDX_BMAP_LEN);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : misc_11n_event                                           */
/*                                                                           */
/*  Description   : This function handles miscellaneous events for 11n.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the mac handle                             */
/*                  2) Type of event                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The miscellaneous 11n events are processed as required   */
/*                  based on the type.                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void misc_11n_event(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

	TROUT_FUNC_ENTER;
	
    switch(misc_event_msg->name)
    {
    case MISC_AMSDU_TX:
    {
        amsdu_tx((amsdu_ctxt_t *)(misc_event_msg->data));
    }
    break;

    case MISC_TX_PENDING_FRAMES:
    {
        ba_alarm_data_t  *data = (ba_alarm_data_t *)misc_event_msg->data;
        UWORD8           q_num = 0;
        ba_tx_struct_t   **ba  = (ba_tx_struct_t **)data->ba;

        q_num = get_txq_num(data->tid);

        (*ba)->pend_alarm_on = 0;

		TROUT_DBG6("%s: transmit frame from pending queue with BA!\n", __func__);
        /* Transmit frames from the pending queue */
        transmit_frames_ba(data->entry, ba, q_num, data->tid);
    }
    break;

    case MISC_BA_TX_STALL:
    {
        handle_tx_ba_stall((ba_alarm_data_t *)(misc_event_msg->data));
    }
    break;

    case MISC_BA_RX_STALL:
    {
        handle_rx_ba_stall((ba_alarm_data_t *)(misc_event_msg->data));
    }
    break;

    default:
    {
        /* Do nothing */
    }
    break;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_11n_p_action_req                                     */
/*                                                                           */
/*  Description   : This function performs the requested action.             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming private action request frame  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses the incoming private action request */
/*                  and configures the required features.                    */
/*                  Note that the private action requests are messages that  */
/*                  have proprietary formats and are used for enabling/      */
/*                  disabling various 11n features. These messages are not   */
/*                  defined in the standard. Each message description can be */
/*                  found in the respective functions processing the same.   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_11n_p_action_req(UWORD8 *msg)
{
    UWORD8 cat  = 0;
    UWORD8 type = 0;
    UWORD8 *req = 0;

    /* Format of the private action request message                          */
    /* --------------------------------------------------------------------- */
    /* | Message Length | Private Action Message                           | */
    /* |                |- - - - - - - - - - - - - - - - - - - - - - - - - | */
    /* |                | Action Category | Action Type | Frame Body       | */
    /* --------------------------------------------------------------------- */
    /* | 1 (L bytes)    |         1       |     1       | (L - 3) bytes    | */
    /* --------------------------------------------------------------------- */

    cat  = msg[1];
    type = msg[2];
    req  = &msg[3];

    if(cat == N_P_ACTION_REQ)
    {
        switch(type)
        {
            case A_MSDU_START:
            {
                handle_mlme_amsdu_start(req);
            }
            break;
            case A_MSDU_END:
            {
                handle_mlme_amsdu_end(req);
            }
            break;
            case A_MPDU_START:
            {
                handle_mlme_ampdu_start(req);
            }
            break;
            case A_MPDU_END:
            {
                handle_mlme_ampdu_end(req);
            }
            break;
            case A_QUERY_TID:
            {
                /* Format of Query-TID Request  */
                /* +------------------------    */
                /* | Rxr MAC Address | TID |    */
                /* +------------------------    */
                /* |    6            | 1   |    */
                /* +------------------------    */
                mac_addr_cpy(g_act_req_q_ra, req);
                g_act_req_q_tid = req[6];
                g_act_req_q_type = A_QUERY_TID;
            }
            break;
            case A_QUERY_ALL:
            {
                g_act_req_q_type = A_QUERY_ALL;
            }
            break;
            default:
            {
                /* Unknown private action request type. Do nothing. */
            }
            break;
        }
    }

#ifdef NO_ACTION_RESET
    if(type == A_ALL_CLEAR)
    {
        clear_all_action_req();
    }
    else
    {
        save_action_req(req, type, 0);
    }
#endif /* NO_ACTION_RESET */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_tx_mib_11n                                        */
/*                                                                           */
/*  Description   : Update the 11n MIB parameters related to transmission.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmitted frame                      */
/*                  2) Retry count                                           */
/*                  3) Number of bytes transmitted                           */
/*                  4) Transmission status                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All the transmit related 11n MIB counters are updated.   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_tx_mib_11n(UWORD8 *msa, UWORD8 retry_count, UWORD16 num_bytes,
                       BOOL_T is_success)
{
    /* Check whether the transmitted frame was an A-MSDU and update the      */
    /* relevant MIB parameters if it is an A-MSDU frame.                     */
    if(0 == is_amsdu_bit_set(msa))
        return;

    if(retry_count != 0)
    {
        mset_RetryAMSDUCount(mget_RetryAMSDUCount() + 1);

        if(retry_count > 1)
            mset_MultipleRetryAMSDUCount(mget_MultipleRetryAMSDUCount() + 1);
    }

    mset_AMSDUAckFailureCount(mget_AMSDUAckFailureCount() + retry_count);

    if(is_success == BTRUE)
    {
        mset_TransmittedAMSDUCount(mget_TransmittedAMSDUCount() + 1);
        mincr_TransmittedOctetsInAMSDU(num_bytes);
    }
    else
    {
        mset_FailedAMSDUCount(mget_FailedAMSDUCount() + 1);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_rx_mib_11n                                        */
/*                                                                           */
/*  Description   : Update the 11n MIB parameters related to reception.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the received frame                         */
/*                  2) Number of bytes received                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All the receive related 11n MIB counters are updated.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_rx_mib_11n(UWORD8 *msa, UWORD16 num_bytes)
{
    /* Check whether the received frame iss an A-MSDU and update the         */
    /* relevant MIB parameters if it is an A-MSDU frame.                     */
    if(1 == is_amsdu_bit_set(msa))
    {
        mset_ReceivedAMSDUCount(mget_ReceivedAMSDUCount() + 1);
        mincr_ReceivedOctesInAMSDUCount(num_bytes);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_amsdu_start                                  */
/*                                                                           */
/*  Description   : This function handles the A-MSDU Start request.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming request message               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses the request message and updates the */
/*                  AMSDU control parameters for the specified RA/TID pair   */
/*                  with the specified values.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_amsdu_start(UWORD8 *req)
{
    UWORD8         tid              = 0;
    UWORD8         ra[6]            = {0};
    void           *entry           = NULL;
    ht_struct_t    *ht_hdl          = NULL;
    ht_tx_struct_t *ht_tx_hdl       = NULL;

	TROUT_FUNC_ENTER;
    /* Format of A-MSDU Start Request                                       */
    /* +-----------------------------------------------------------------+  */
    /* | Rxr MAC Address | TID | Max Num MSDU | Max AMSDU Size | Timeout |  */
    /* +-----------------------------------------------------------------+  */
    /* |    6            | 1   | 1            | 2              |     2   |  */
    /* +-----------------------------------------------------------------+  */

    mac_addr_cpy(ra, req);

    update_action_req_ra(ra);

    entry = find_entry(ra);

    if(NULL == entry)
    {
    	TROUT_FUNC_EXIT;
        return;
    }

#ifdef NO_ACTION_RESET
    if(mac_addr_cmp(ra, mget_StationID()) == BTRUE)
    {
        /* Do not modify the AMSDU-Parameters if the connection already exists */
		TROUT_FUNC_EXIT;
        return;
    }
#endif /* NO_ACTION_RESET */

    /* Get the HT Control handle from the receiver entry */
    ht_hdl = (ht_struct_t *)get_ht_handle_entry(entry);

    /* AMSDU setup only if the receiver is HT capable and QoS is enabled */
    if((0 == ht_hdl->ht_capable) ||
       (is_dst_wmm_capable(get_sta_index(entry)) == BFALSE))
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Extract the TID from the message */
    tid = req[6];

    /* Create Tx HT Cntl entry for the specified TID if not present already */
    if(NULL == ht_hdl->ht_tx_ptr[tid])
    {
        /* Initialize the handle to the buffer within the structure */
        ht_hdl->ht_tx_ptr[tid] = &(ht_hdl->ht_tx_buff[tid]);
        mem_set(ht_hdl->ht_tx_ptr[tid], 0, sizeof(ht_tx_struct_t));
    }

    ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

    /* Update the AMSDU parameters with the specified details */
    ht_tx_hdl->amsdu_maxnum   = req[7];
    ht_tx_hdl->amsdu_maxsize  = req[9];
    ht_tx_hdl->amsdu_maxsize  = (ht_tx_hdl->amsdu_maxsize << 8) | req[8];

    /* Max-size of the AMSDU is upper bounded by the Rxr capability */
    if(ht_tx_hdl->amsdu_maxsize > ht_hdl->sta_amsdu_maxsize)
        ht_tx_hdl->amsdu_maxsize = ht_hdl->sta_amsdu_maxsize;
    else if(ht_tx_hdl->amsdu_maxsize < MIN_AMSDU_LEN)
        ht_tx_hdl->amsdu_maxsize = MIN_AMSDU_LEN;

    /* In case A-MPDU has been set up for this RA, TID (indicated by a value */
    /* of 'ampdu_maxnum' not equal to 0) ensure that the maximum A-MSDU size */
    /* is limited by the maximum A-MSDU size allowed with MPDU aggregation.  */
    if(ht_tx_hdl->ampdu_maxnum != 0)
    {
        if(ht_tx_hdl->amsdu_maxsize > MAX_AMSDU_SIZE_WITH_AMPDU)
            ht_tx_hdl->amsdu_maxsize = MAX_AMSDU_SIZE_WITH_AMPDU;
    }

    /* The amsdu_maxsize parameter contains the max size of the payload */
    /* within an A-MSDU.                                                */
    if(ht_hdl->htc_support == 1)
    {
        /* Account for HT-MAC Header, FCS & Security headers */
        ht_tx_hdl->amsdu_maxsize -= (30 + 4 + 16);
    }
    else
    {
        /* Account for QoS-MAC Header, FCS & Security headers */
        ht_tx_hdl->amsdu_maxsize -= (26 + 4 + 16);
    }

    ht_tx_hdl->amsdu_to_intvl = req[11];
    ht_tx_hdl->amsdu_to_intvl = (ht_tx_hdl->amsdu_to_intvl << 8) | req[10];

    TROUT_DBG4("A-MSDU Session Setup:");
    TROUT_DBG4("RA=%02x:%02x:%02x:%02x:%02x:%02x TID=%d MaxSize=%d MaxNum=%d ToItvl=%d, entry=0x%p\n",
            ra[0],ra[1],ra[2],ra[3],ra[4],ra[5], tid, ht_tx_hdl->amsdu_maxsize,
            ht_tx_hdl->amsdu_maxnum,ht_tx_hdl->amsdu_to_intvl, entry);

	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_amsdu_end                                    */
/*                                                                           */
/*  Description   : This function handles the A-MSDU End request.            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming request message               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The message is parsed and the AMSDU parameters           */
/*                  corresponding to the specified RA/TID pair are reset.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_amsdu_end(UWORD8 *req)
{
    UWORD8         tid         = 0;
    UWORD8         ra[6]       = {0};
    void           *entry           = NULL;

    /* Format of A-MSDU End Request                                          */
    /* --------------------------------------------------                    */
    /* | Receiver MAC Address           | TID           |                    */
    /* --------------------------------------------------                    */
    /* | 6                              | 1             |                    */
    /* --------------------------------------------------                    */

    mac_addr_cpy(ra, req);

    update_action_req_ra(ra);

    entry = find_entry(ra);

    if(NULL == entry)
        return;

    /* Extract the TID from the message */
    tid = req[6];

    reset_tx_amsdu_session(entry, tid);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_tx_amsdu_session                                   */
/*                                                                           */
/*  Description   : This function resets the specified AMSDU Tx Session.     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the RA-STA entry.                          */
/*                  2) TID of the AMSDU session to be ended                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The AMSDU parameters corresponding to the specified      */
/*                  RA/TID pair are reset.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reset_tx_amsdu_session(void *entry, UWORD8 tid)
{
    ht_struct_t    *ht_hdl          = NULL;
    ht_tx_struct_t *ht_tx_hdl       = NULL;
    void           *amsdu_ctxt = NULL;

    /* Get the HT Control handle from the receiver entry */
    ht_hdl = (ht_struct_t *)get_ht_handle_entry(entry);

    if(ht_hdl == NULL || NULL == ht_hdl->ht_tx_ptr[tid])
        return;

    ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

    /* Update the AMSDU parameters with the specified details */
    ht_tx_hdl->amsdu_maxnum   = 0;
    ht_tx_hdl->amsdu_maxsize  = 0;
    ht_tx_hdl->amsdu_to_intvl = 0;

    /* Flush the frames currently being aggregated for the RA/TID */
    amsdu_ctxt = peek_amsdu_ctxt(entry, tid);
    //printk("%s: sub_amsdu_num = %d\n", __FUNCTION__, ((amsdu_ctxt_t *)amsdu_ctxt)->num_entries);
    if(NULL != amsdu_ctxt)
        amsdu_tx(amsdu_ctxt);

    /* Free the HT Tx Control entry if A-MPDU is also not set up (indicated  */
    /* by a value of 'amsdu_maxnum' equal to 0)                              */
    if(ht_tx_hdl->ampdu_maxnum == 0)
    {
        ht_hdl->ht_tx_ptr[tid] = NULL;
    }
    PRINTD("A-MSDU Session Ended:");
    PRINTD("TID=%d \n\r",tid);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_ampdu_start                                  */
/*                                                                           */
/*  Description   : This function handles the A-MPDU Start request.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming request message               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses the request message and updates the */
/*                  AMPDU control parameters for the specified RA/TID pair   */
/*                  with the specified values.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_ampdu_start(UWORD8 *req)
{
    UWORD8         tid              = 0;
    UWORD8         ra[6]            = {0};
    UWORD8         ackpol           = 0;
    void           *entry           = NULL;
    ht_struct_t    *ht_hdl          = NULL;
    ht_tx_struct_t *ht_tx_hdl       = NULL;

    /* Format of A-MPDU Start Request                      */
    /* +-------------------------------------------------------------+ */
    /* | Rxr MAC Address | TID | Max Num MSDU | Timeout | Ack-Policy | */
    /* +-------------------------------------------------------------+ */
    /* |    6            | 1   | 1            | 2       |    1       | */
    /* +-------------------------------------------------------------+ */
	TROUT_FUNC_ENTER;
	
    mac_addr_cpy(ra, req);

    update_action_req_ra(ra);

    entry = find_entry(ra);

    if(entry == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
	}
#ifdef NO_ACTION_RESET
    if(mac_addr_cmp(ra, mget_StationID()) == BTRUE)
    {
        /* Do not modify the AMSDU-Parameters if the connection already exists */
		TROUT_FUNC_EXIT;
        return;
    }
#endif /* NO_ACTION_RESET */

    /* Get the HT Control handle from the receiver entry */
    ht_hdl = (ht_struct_t *)get_ht_handle_entry(entry);

    /* A-MPDU set up can be done only if the receiver is HT capable */
    if(ht_hdl->ht_capable == 0)
    {
		TROUT_FUNC_EXIT;
        return;
	}
    /* Extract the TID from the message */
    tid = req[6];
    ackpol = req[10];

    /* If the chosen Ack-Policy is Normal-Ack, check if Block ACK is set  */
    /* up for this RA, TID combination. A-MPDU can not be set up unless   */
    /* there is a Block ACK session set up for this.                      */
    if((ackpol == NORMAL_ACK) &&
       (is_txba_session_active(entry, tid) == BFALSE))
    {
		TROUT_FUNC_EXIT;
        return;
    }

    ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

    /* Create a HT Tx control entry for the specified TID if not present */
    if(ht_tx_hdl == NULL)
    {
        /* Initialize the handle to the buffer within the structure */
        ht_hdl->ht_tx_ptr[tid] = &(ht_hdl->ht_tx_buff[tid]);
        ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

        mem_set(ht_tx_hdl, 0, sizeof(ht_tx_struct_t));
    }

#ifdef NO_ACTION_RESET
    if(ht_tx_hdl->ampdu_maxnum != 0)
    {
        /* Do not modify the AMPDU-Parameters if the connection already exists */
		TROUT_FUNC_EXIT;
        return;
    }
#endif /* NO_ACTION_RESET */

    if(ht_tx_hdl->ampdu_maxnum == 0)
    {
        /* Do not modify the AMPDU-Count if the connection already exists */
        incr_num_tx_ampdu_sessions(entry);
#ifdef AUTORATE_FEATURE
	    /* Update the transmit rate to the station to support the new AMPDU */
	    /* session.                                                         */
	    reinit_tx_rate_idx(entry);
#endif /* AUTORATE_FEATURE */
    }

    /* Check if previous AMPDU session was present. If so the same LUT index */
    /* can be reused. The entry in H/w AMPDU LUT will be overwritten. If no  */
    /* session was present, allocate a new AMPDU LUT index.                  */
    if(ht_tx_hdl->ampdu_maxnum == 0)
        ht_tx_hdl->ampdu_lut_idx = get_ampdu_lut_index();

    /* Proceed only if valid LUT index is found. Otherwise an A-MPDU session */
    /* session cannot be set up.                                             */
    if(ht_tx_hdl->ampdu_lut_idx == INVALID_AMPDU_LUT_INDEX)
    {
        /* Free HT Tx Control entry if A-MSDU is also not set up (indicated  */
        /* by a value of 'amsdu_maxnum' equal to 0)                          */
        if(0 == ht_tx_hdl->amsdu_maxnum)
        {
            ht_hdl->ht_tx_ptr[tid] = NULL;
        }

		TROUT_FUNC_EXIT;
        return;
    }

    /* Update the AMPDU parameters with the specified details */
    ht_tx_hdl->ampdu_maxnum = req[7];

    /* In case A-MSDU has been set up for this RA, TID (indicated by a value */
    /* of 'amsdu_maxnum' not equal to 0) ensure that the maximum A-MSDU size */
    /* is limited by the maximum A-MSDU size allowed with MPDU aggregation.  */
    if(ht_tx_hdl->amsdu_maxnum != 0)
    {
        if(ht_tx_hdl->amsdu_maxsize > MAX_AMSDU_SIZE_WITH_AMPDU)
        {
            ht_tx_hdl->amsdu_maxsize = (ht_hdl->htc_support == 1)?
                                            (MAX_AMSDU_SIZE_WITH_AMPDU - 50) :
                                            (MAX_AMSDU_SIZE_WITH_AMPDU - 46);
        }
    }

    ht_tx_hdl->ampdu_to_intvl = req[9];
    ht_tx_hdl->ampdu_to_intvl = (ht_tx_hdl->ampdu_to_intvl << 8) | req[8];
    ht_tx_hdl->ampdu_ack_pol  = ackpol;

    /* Add this A-MPDU entry to the H/w A-MPDU LUT */
    add_machw_ampdu_lut_entry(ht_tx_hdl->ampdu_lut_idx,
                              ht_tx_hdl->ampdu_maxnum,
                              ht_hdl->min_mpdu_start_spacing,
                              ht_hdl->max_rx_ampdu_factor);

    /* If the chosen Ack-Policy is Normal-Ack, then update the grouping     */
    /* buffer parameters of the corresponding block-ack session.            */
    if(ht_tx_hdl->ampdu_ack_pol == NORMAL_ACK)
    {
    	update_txba_session_params(entry, tid, ht_tx_hdl->ampdu_to_intvl,
                                   ht_tx_hdl->ampdu_maxnum);
    }
    TROUT_DBG4("A-MPDU Session Setup:");
    TROUT_DBG4("RA=%x:%x:%x:%x:%x:%x TID=%d MMSS=%d MRAF=%d MaxNum=%d ToItvl=%d AckPol=%d LutIdx=%d\n",
            ra[0],ra[1],ra[2],ra[3],ra[4],ra[5], tid,
            ht_hdl->min_mpdu_start_spacing, ht_hdl->max_rx_ampdu_factor,
            ht_tx_hdl->ampdu_maxnum,ht_tx_hdl->ampdu_to_intvl, ackpol,
            ht_tx_hdl->ampdu_lut_idx);

	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_ampdu_end                                    */
/*                                                                           */
/*  Description   : This function handles the A-MPDU End request.            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming request message               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The message is parsed and the AMPDU parameters           */
/*                  corresponding to the specified RA/TID pair are reset.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_ampdu_end(UWORD8 *req)
{
    UWORD8         tid         = 0;
    UWORD8         ra[6]       = {0};
    void           *entry      = NULL;

    /* Format of A-MPDU End Request                       */
    /* -------------------------------------------------- */
    /* | Receiver MAC Address           | TID           | */
    /* -------------------------------------------------- */
    /* | 6                              | 1             | */
    /* -------------------------------------------------- */

    mac_addr_cpy(ra, req);

    update_action_req_ra(ra);

    entry = find_entry(ra);

    if(NULL == entry)
        return;

    /* Extract the TID from the message */
    tid = req[6];

    reset_tx_ampdu_session(entry, tid);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_action_req_query_tid                              */
/*                                                                           */
/*  Description   : This function handles the Query of Action Request of type*/
/*                  Query-TID                                                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_cfg_val                                                */
/*                                                                           */
/*  Processing    : This function processes for AMPDU, AMSDU and Block ACK   */
/*                  Session Status for given RA-TID pair and updates the     */
/*                  config response buffer                                   */
/*                                                                           */
/*  Outputs       : Pointer to g_cfg_val                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8* handle_action_req_query_tid(void)
{
    UWORD8         tid              = g_act_req_q_tid;
    UWORD8         index            = 0;
    UWORD8         *ra              = g_act_req_q_ra;
    void           *entry           = NULL;
    ht_struct_t    *ht_hdl          = NULL;
    ht_tx_struct_t *ht_tx_hdl       = NULL;
    ba_struct_t    *ba_hdl          = NULL;

/* Format of QUERY-TID Response                                                           */
/* +--------+----------+------+------+----------+---------+---------+---------+---------+ */
/* |        |          |      |      |          |AMSDU-TX |AMPDU-TX |  BA-TX  |  BA-RX  | */
/* | Length | Category | Type |  RA  | TID_INFO | params  | params  | params  | params  | */
/* +--------+----------+------+------+----------+---------+---------+---------+---------+ */
/* |   1    |    1     |  1   |  6   |     1    |    5    |    4    |    9    |    4    | */
/* +--------+----------+------+------+----------+---------+---------+---------+---------+ */

    /* Reset the response buffer to Zero */
    mem_set(g_cfg_val, 0, (N_P_ACTION_QUERY_TID_LEN + 1));

    /* First byte is set to the length of the response */
    g_cfg_val[index++] = (N_P_ACTION_QUERY_TID_LEN+1);

    /* Update the Length, Category and Type Field */
    g_cfg_val[index++] = N_P_ACTION_QUERY_TID_LEN;
    g_cfg_val[index++] = N_P_ACTION_REQ;
    g_cfg_val[index++] = A_QUERY_TID;

    /* Convert AID to MAC-Address */
    update_action_req_ra(ra);

    entry = find_entry(ra);

    /* If entry not found then return */
    if(entry == NULL)
    {
        return g_cfg_val;
    }

    /* Update the Receiver Address field */
    mac_addr_cpy((g_cfg_val + index), ra);

    index += 6;

    /* Update the TID in TID_INFO field */
    g_cfg_val[index++] = (tid << 4);

    /* Get the HT Control handle from the receiver entry */
    ht_hdl = (ht_struct_t *)get_ht_handle_entry(entry);

    /* If receiver is not HT capable then return */
    if(ht_hdl->ht_capable == 0)
        return g_cfg_val;

    /*   TID_INFO field format    */
    /* +----------+-------------+ */
    /* | TID_INFO | Information | */
    /* +----------+-------------+ */
    /* | BIT[7:4] |     TID     | */
    /* +----------+-------------+ */
    /* |   BIT3   |   AMSDU-TX  | */
    /* +----------+-------------+ */
    /* |   BIT2   |   AMPDU-TX  | */
    /* +----------+-------------+ */
    /* |   BIT1   |     BA-TX   | */
    /* +----------+-------------+ */
    /* |   BIT0   |     BA-RX   | */
    /* +----------+-------------+ */

    ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

    if(ht_tx_hdl != NULL)
    {
        if(ht_tx_hdl->amsdu_maxnum != 0)
        {
            g_cfg_val[N_P_ACT_Q_TID_INFO_OFFSET] |=
            (UWORD8)N_P_ACTION_AMSDU_TX_SETUP;
        }

        if(ht_tx_hdl->ampdu_maxnum != 0)
        {
            g_cfg_val[N_P_ACT_Q_TID_INFO_OFFSET] |=
            (UWORD8)N_P_ACTION_AMPDU_TX_SETUP;
        }

        /* Update AMSDU-TX parameters */
        index += set_amsdu_session_params((g_cfg_val + index),
                                          ht_tx_hdl->amsdu_maxnum,
                                          ht_tx_hdl->amsdu_maxsize,
                                          ht_tx_hdl->amsdu_to_intvl);

        /* Update AMPDU-TX parameters */
        index += set_ampdu_session_params((g_cfg_val + index),
                                          ht_tx_hdl->ampdu_maxnum,
                                          ht_tx_hdl->ampdu_to_intvl,
                                          ht_tx_hdl->ampdu_ack_pol);

    }
    else
    {
        index += 9;
    }

    ba_hdl = (ba_struct_t *)get_ba_handle_entry(entry);

    if(ba_hdl != NULL)
    {
        ba_rx_struct_t **ba_rx_hdl = &(ba_hdl->ba_rx_ptr[tid]);
        ba_tx_struct_t **ba_tx_hdl = &(ba_hdl->ba_tx_ptr[tid]);

        if((*ba_tx_hdl != NULL) && ((*ba_tx_hdl)->is_ba == BTRUE))
        {
            g_cfg_val[N_P_ACT_Q_TID_INFO_OFFSET] |=
            (UWORD8)N_P_ACTION_BA_TX_SETUP;

            /* Update BA-TX parameters */
            index += set_ba_initiator_params((g_cfg_val + index),
                                             (*ba_tx_hdl)->ba_policy,
                                             (*ba_tx_hdl)->buff_size,
                                             (*ba_tx_hdl)->timeout,
                                             0,
                                             (*ba_tx_hdl)->max_pend,
                                             (*ba_tx_hdl)->buff_timeout);

        }
        else
        {
            index += 9;
        }

        if((*ba_rx_hdl != NULL) && ((*ba_rx_hdl)->is_ba == BTRUE))
        {

            g_cfg_val[N_P_ACT_Q_TID_INFO_OFFSET] |=
            (UWORD8)N_P_ACTION_BA_RX_SETUP;

            /* Update BA-RX parameters */
            index += set_ba_recepient_params((g_cfg_val + index),
                                             (*ba_rx_hdl)->ba_policy,
                                             (*ba_rx_hdl)->buff_size,
                                             (*ba_rx_hdl)->timeout);
        }
        else
        {
            index += 4;
        }
    }
    else
    {
        index += 13;
    }

    return g_cfg_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_q_all_act_req_entry                              */
/*                                                                           */
/*  Description   : This function processes the QUERY-ALL type action request*/
/*                  for a given entry                                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function finds if there is AMPDU-TX, AMSDU-TX, Block*/
/*                  ACK-TX and Block ACK-RX session setup for given entry and*/
/*                  updates corresponding flags in the query response buffer */
/*                                                                           */
/*  Outputs       : Index Offset to next AID field                           */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 process_q_all_act_req_entry(void *entry, UWORD8 *buffer)
{
    UWORD8         index      = 0;
    UWORD8         tid        = 0;
    UWORD8         asoc_id   = get_entry_sta_index(entry);
    ht_struct_t    *ht_hdl    = NULL;
    ht_tx_struct_t *ht_tx_hdl = NULL;
    ba_struct_t    *ba_hdl    = NULL;

    ht_hdl = (ht_struct_t *)get_ht_handle_entry(entry);

    /* If receiver is not HT capable then return */
    if(ht_hdl->ht_capable == 0)
        return index;


    /*   TID_INFO field format    */
    /* +----------+-------------+ */
    /* | TID_INFO | Information | */
    /* +----------+-------------+ */
    /* | BIT[7:4] |     TID     | */
    /* +----------+-------------+ */
    /* |   BIT3   |   AMSDU-TX  | */
    /* +----------+-------------+ */
    /* |   BIT2   |   AMPDU-TX  | */
    /* +----------+-------------+ */
    /* |   BIT1   |     BA-TX   | */
    /* +----------+-------------+ */
    /* |   BIT0   |     BA-RX   | */
    /* +----------+-------------+ */


    for(tid = 0; tid < 8; tid++)
    {
        BOOL_T field_updated = BFALSE;
        ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

        if(ht_tx_hdl != NULL)
        {
            if(ht_tx_hdl->amsdu_maxnum != 0)
            {
                buffer[index + 1] |= (UWORD8)N_P_ACTION_AMSDU_TX_SETUP;
                field_updated = BTRUE;
            }

            if(ht_tx_hdl->ampdu_maxnum != 0)
            {
                buffer[index + 1] |= (UWORD8)N_P_ACTION_AMPDU_TX_SETUP;
                field_updated = BTRUE;
            }

        }

        ba_hdl = (ba_struct_t *)get_ba_handle_entry(entry);

        if(ba_hdl != NULL)
        {
            ba_rx_struct_t **ba_rx_hdl = &(ba_hdl->ba_rx_ptr[tid]);
            ba_tx_struct_t **ba_tx_hdl = &(ba_hdl->ba_tx_ptr[tid]);

            if((*ba_tx_hdl != NULL) && ((*ba_tx_hdl)->is_ba == BTRUE))
            {
                buffer[index + 1] |= (UWORD8)N_P_ACTION_BA_TX_SETUP;
                field_updated = BTRUE;
            }

            if((*ba_rx_hdl != NULL) && ((*ba_rx_hdl)->is_ba == BTRUE))
            {
                buffer[index + 1] |= (UWORD8)N_P_ACTION_BA_RX_SETUP;
                field_updated = BTRUE;
            }
        }

        /* If Field Update is required then update AID & TID_INFO field */
        if(field_updated == BTRUE)
        {
            /* Update the AID field and TID in TID_INFO field */
            buffer[index] = asoc_id;
            buffer[index + 1] |= (tid << 4);
            index += 2;
        }
    }

    return index;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_action_req_query_all                              */
/*                                                                           */
/*  Description   : This function handles the Query of Action Request of type*/
/*                  Query-ALL                                                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_cfg_val                                                */
/*                                                                           */
/*  Processing    : This function processes for AMPDU, AMSDU and Block ACK   */
/*                  Session Status for each of the STA entry.                */
/*                                                                           */
/*  Outputs       : Pointer to g_cfg_val                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8* handle_action_req_query_all(void)
{
    UWORD32 i         = 0;
    UWORD32 index     = 0;
    void         *entry   = 0;
    table_elmnt_t *tbl_elm = 0;

    /* Set the entire response buffer to ZERO */
    mem_set(&g_cfg_val[0], 0, MAX_CFG_LEN);

    /* Format of QUERY-ALL Response                                   */
    /* +--------+----------+------+----------------+----------------+ */
    /* |        |          |      |    Session 1   |    Session 2   | */
    /* | Length | Category | Type +-----+----------+-----+----------+ */
    /* |        |          |      | AID | TID_INFO | AID | TID_INFO | */
    /* +--------+----------+------+-----+----------+-----+----------+ */
    /* |   1    |    1     |  1   |  1  |     1    |  1  |     1    | */
    /* +--------+----------+------+-----+----------+-----+----------+ */
    g_cfg_val[index++] = 0;
    g_cfg_val[index++] = 0;
    g_cfg_val[index++] = N_P_ACTION_REQ;
    g_cfg_val[index++] = A_QUERY_ALL;

    /* Traverse entire association table and process all non-zero entries */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];

        while(tbl_elm)
        {
            entry = tbl_elm->element;

            if(entry == 0)
                break;

            if(index >= MAX_CFG_LEN)
                break;

            /* Update TID+INFO field for this entry if relevant */
            index += process_q_all_act_req_entry(entry, (g_cfg_val + index));

            /* Next Entry */
            tbl_elm = tbl_elm->next_hash_elmnt;
        } /* end of while loop */
    } /* end of for loop */

    /* Set the Length Field */
    g_cfg_val[1] = (index - 1);

    /* First Byte in the response buffer indicates the length of the response*/
    g_cfg_val[0] = index;

    return g_cfg_val;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_tx_ampdu_session                                   */
/*                                                                           */
/*  Description   : This function resets the specified AMPDU Tx Session.     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the RA-STA entry.                          */
/*                  2) TID of the AMPDU session to be ended                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Tthe AMPDU parameters corresponding to the specified     */
/*                  RA/TID pair are reset.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reset_tx_ampdu_session(void *entry, UWORD8 tid)
{
    ht_struct_t    *ht_hdl     = NULL;
    ht_tx_struct_t *ht_tx_hdl  = NULL;

    /* Get the HT Control handle from the receiver entry */
    ht_hdl = (ht_struct_t *)get_ht_handle_entry(entry);

    if(ht_hdl == NULL || NULL == ht_hdl->ht_tx_ptr[tid])
        return;

    ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

    /* Update the AMSDU parameters with the specified details */
    ht_tx_hdl->ampdu_maxnum   = 0;
    ht_tx_hdl->ampdu_to_intvl = 0;

    /* Free the HT Tx Control entry if A-MSDU is also not set up (indicated  */
    /* by a value of 'amsdu_maxnum' equal to 0)                              */
    if(0 == ht_tx_hdl->amsdu_maxnum)
    {
        ht_hdl->ht_tx_ptr[tid] = NULL;
    }

    /* Remove this A-MPDU entry from the H/w A-MPDU LUT and delete the index */
    remove_machw_ampdu_lut_entry(ht_tx_hdl->ampdu_lut_idx);
    del_ampdu_lut_index(ht_tx_hdl->ampdu_lut_idx);
    decr_num_tx_ampdu_sessions(entry);

    PRINTD("A-MPDU Session Ended:");
    PRINTD("TID=%d LutIdx=%d\n\r",tid, ht_tx_hdl->ampdu_lut_idx);

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_addba_req                                    */
/*                                                                           */
/*  Description   : This function handles the incoming MLME ADDBA request.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming ADDBA request message         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Ths function prepares an ADDBA request frame with the    */
/*                  incoming parameters. This frame is transmitted. The      */
/*                  associated context structure is updated. A timeout timer */
/*                  is started.                                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_addba_req(UWORD8 *aba_req)
{
    UWORD8         tid           = 0;
    UWORD8         ba_policy_req = 0;
    UWORD16        addba_len     = 0;
    UWORD16        timeout       = 0;
    void           *entry        = NULL;
    UWORD8         *addba_req    = 0;
    ba_struct_t    *ba_ctxt      = NULL;
    ba_tx_struct_t **ba          = NULL;
    UWORD8         ra[6]         = {0};

	TROUT_FUNC_ENTER;
	
    /* If HT is OFF, then ignore the Block Ack Req */
    if(get_ht_enable() == 0)
    {
		TROUT_FUNC_EXIT;
    	return;
	}
    /* If Immediate Block ACK Option is not implemented return */
    if(mget_ImmediateBlockAckOptionImplemented() == TV_FALSE)
    {
		TROUT_FUNC_EXIT;
        return;
	}
    /* This request is processed further only if the limit is not exceeded */
    if(is_ba_limit_reached() == BTRUE)
    {
        TROUT_DBG4("Num BA Sessions limit reached\n");
        TROUT_FUNC_EXIT;
        return;
    }

    aba_req += 2;

    /*************************************************************************/
    /* Format of ADDBA Body                                                  */
    /* --------------------------------------------------------------------- */
    /* | Addr  | TID |   BA   | Buff |    BA   |  AddBA  | Group |  Group  | */
    /* |       |     | Policy | Size | Timeout | Timeout | Size  | Timeout | */
    /* --------------------------------------------------------------------- */
    /* |   6   |  1  |   1    |  2   |   2     |    2    |   1   |    2    | */
    /* --------------------------------------------------------------------- */
    /*************************************************************************/

    /* Check that the incoming Block ACK request policy is Immediate. If not */
    /* reject the same as no other policy is supported.                      */
    ba_policy_req = aba_req[7];
    if(ba_policy_req != 1)
    {
		TROUT_FUNC_EXIT;
        return;
	}
    tid = aba_req[6];

    mac_addr_cpy(ra, aba_req);

    update_action_req_ra(ra);

    entry = find_entry(ra);
    if(entry == NULL)
    {
        TROUT_DBG2("Error: STA Not Associated\n");
#ifdef DEBUG_MODE
        g_mac_stats.stanotassc++;
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }

    /* Block-Ack session is initiated only if the destination is HT-capable */
    /* and has WMM enabled.                                                 */
    /* The check for Immediate Block-Ack Capability has been removed since  */
    /* none of the Standard devices seem to set it.                         */
    if((is_ht_capable(entry) == BFALSE) ||
       (is_dst_wmm_capable(get_sta_index(entry)) == BFALSE))
    {
        TROUT_DBG2("Error: STA Not HT and WMM Capable\n");
        TROUT_FUNC_EXIT;
        return;
    }

    /* Get the Block-Ack Context structure */
    ba_ctxt = (ba_struct_t *)get_ba_handle_entry(entry);

    /* Block-Ack session is in progress for this DA-TID. */
    /* The new request is discarded here. The implication of this is that  */
    /* a BA parameter can be changed only by teardown and re-establishment */
    /* of the session                                                      */
    if(ba_ctxt == NULL || ba_ctxt->ba_tx_ptr[tid] != NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Create a buffer for transmitting the ADDBA-Request frame */
    addba_req = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle, ADDBA_REQ_LEN);
    if(addba_req == NULL)
    {
		TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE
		g_mac_stats.no_mem_count++;
#endif		
        return;
    }

    /* Create a new context structure for  */
    ba_ctxt->ba_tx_ptr[tid] = (ba_tx_struct_t *)mem_alloc(
        g_local_mem_handle, sizeof(ba_tx_struct_t));

    if(ba_ctxt->ba_tx_ptr[tid] == NULL)
    {
        pkt_mem_free(addba_req);
        TROUT_FUNC_EXIT;
        return;
    }

    ba = &(ba_ctxt->ba_tx_ptr[tid]);

    mem_set(*ba, 0, sizeof(ba_tx_struct_t));

    /* Extract session parameters from the user input */
    (*ba)->conn_status       = BA_INIT;
    (*ba)->sta_index         = get_sta_index(entry);
    (*ba)->is_ba             = BTRUE;
    (*ba)->dialog_token      = get_random_byte();
    (*ba)->timeout           = aba_req[10] | (aba_req[11] << 8);
    (*ba)->buff_size         = aba_req[8]  | (aba_req[9] << 8);
    (*ba)->ba_policy         = ba_policy_req;
    (*ba)->ba_data.ba        = (UWORD8 *)ba;
    (*ba)->ba_data.tid       = tid;
    (*ba)->ba_data.entry     = (UWORD8 *)entry;
    (*ba)->ba_data.direction = INITIATOR;
    (*ba)->max_pend          = aba_req[14];
    (*ba)->buff_timeout      = aba_req[15] | (aba_req[16] << 8);

    if((*ba)->buff_timeout == 0)
    {
        (*ba)->buff_timeout = DEF_BA_GRP_BUFF_TO;
        TROUT_DBG4("Grouping Buffer Timeout set to default 100ms\n");
    }

    /* Upper limit can imposed on the number of TX Buffers reserved for */
    /* the TX-Retry queue.                                              */
    if((*ba)->buff_size > MAX_BA_TX_BUFFERS)
        (*ba)->buff_size = MAX_BA_TX_BUFFERS;
    /* The maximum number of frames which can be buffered when the window */
    /* parameters permit a transmission is upper bounded by the Block-Ack */
    /* session buffer size.                                               */
    if((*ba)->max_pend > (*ba)->buff_size)
        (*ba)->max_pend  = (*ba)->buff_size;

    (*ba)->max_pend  = (*ba)->buff_size/2;
    (*ba)->bar_int   = MIN((*ba)->buff_size/2, MAX_BAR_INTERVAL);

    /* The maximum size of the pending queue is set to twice that of the  */
    /* retry queue.                                                       */
    (*ba)->pend_limit        = (*ba)->buff_size;
    memcpy(ba_ctxt->dst_addr, ra, 6);
    (*ba)->dst_addr = ba_ctxt->dst_addr;

    timeout = aba_req[12] | (aba_req[13] << 8);

    /* Initialize the Pending Queue. Done here since all frames arriving from */
    /* the host are buffered till the setup is complete.                      */
    /* A List Buffer is used as the Pending Queue. The TX Dscr is enqueued    */
    /* with the field containing the link to next descriptor used as Link Ptr */
    init_list_buffer(&(*ba)->pend_list, (TX_DSCR_NEXT_ADDR_WORD_OFFSET*4));

	g_tx_ba_setup = BTRUE;
	
    /* Create ADDBA-Request Action Frame */
    addba_len = prepare_addba_req(addba_req, (*ba), tid);

    /* Send this frame to the required destination */
    tx_mgmt_frame(addba_req, addba_len, get_txq_num(tid),
                  get_sta_index(entry));

    start_addba_timer(&((*ba)->ba_alarm), timeout, (UWORD32)&((*ba)->ba_data));
    g_num_ba_tx_sessions++;
    incr_num_tx_ba_sessions(entry);
#ifdef AUTORATE_FEATURE
    /* Update the transmit rate to the station to support the new Block-Ack */
    /* session.                                                             */
    reinit_tx_rate_idx(entry);
#endif /* AUTORATE_FEATURE */

    TROUT_DBG4("ADDBA-Request sent:\n");
    TROUT_DBG4("TID=%d, BufSize=%d, Timeout=%d, MaxPend=%d, Grouping_Buff_timeout=%d\n",
        (*ba)->ba_data.tid, (*ba)->buff_size, (*ba)->timeout,
        (*ba)->max_pend, (*ba)->buff_timeout);

    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_mlme_delba_req                                    */
/*                                                                           */
/*  Description   : This function handles the incoming MLME DELBA request.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming DELBA request message         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Ths function prepares an DELBA request frame with the    */
/*                  incoming parameters and transmits it. The associated     */
/*                  context structure is reset.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_mlme_delba_req(UWORD8 *dba_req)
{
    UWORD8      tid       = 0;
    UWORD8      direction = 0;
    void        *entry    = NULL;
    ba_struct_t *ba_ctxt  = NULL;
    UWORD8      ra[6]     = {0};

    dba_req += 2;

    /*********************************************************/
    /* Format of DELBA Body                                  */
    /* ----------------------------------------------------- */
    /* | Address   | TID    | Direction | Result Code      | */
    /* ----------------------------------------------------- */
    /* |   6       |     1  |     1     |     1            | */
    /* ----------------------------------------------------- */
    /*********************************************************/

    tid         = dba_req[6];
    direction   = dba_req[7];

    mac_addr_cpy(ra, dba_req);

    update_action_req_ra(ra);

    entry = find_entry(ra);

    if(entry == NULL)
    {
        return;
    }

    /* Get the Block-Ack Context structure */
    ba_ctxt = (ba_struct_t *)get_ba_handle_entry(entry);

    if(direction == INITIATOR)
    {
        ba_tx_struct_t **ba = &(ba_ctxt->ba_tx_ptr[tid]);

        if(((*ba) == NULL) || ((*ba)->is_ba  != BTRUE))
        {
            return;
        }

        /* Send the delba request to destination address */
        send_delba(ra, tid, direction, dba_req[8]);
        reset_tx_ba_handle(ba, tid);
    }
    else
    {
        ba_rx_struct_t **ba = &(ba_ctxt->ba_rx_ptr[tid]);

        if(((*ba) == NULL) || ((*ba)->is_ba  != BTRUE))
        {
            return;
        }

        /* Send the delba request to destination address */
        send_delba(ra, tid, direction, dba_req[8]);
        mutex_lock(&ba_rx_mutex);	//add by chengwg 2013-06-21.
        reset_rx_ba_handle(ba, tid);
        mutex_unlock(&ba_rx_mutex);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_wlan_addba_req                                    */
/*                                                                           */
/*  Description   : This function handles the ADDBA Request received from    */
/*                  the WLAN interface.                                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the address of ADDBA Initiator.            */
/*                  2) Pointer to the incoming ADDBA request message         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Ths function parses the incoming ADDBA-Request frame     */
/*                  from the initiator and sends a ADDBA-Response in return. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_wlan_addba_req(UWORD8 *sa, UWORD8 *data)
{
    UWORD8         tid      = 0;
    void           *entry   = NULL;
    ba_struct_t    *ba_ctxt = NULL;
    ba_rx_struct_t **ba     = NULL;

	TROUT_FUNC_ENTER;

    /* If HT is OFF, then ignore the Block Ack Req */
    if(get_ht_enable() == 0)
    {
		TROUT_FUNC_EXIT;
		TROUT_DBG4("ht capability info is disabled!\n");
    	return;
	}

    /* Get the station entry corresponding to the Initiator */
    entry  = find_entry(sa);
    if(entry == NULL)
    {
		TROUT_DBG4("entry: %02x:%02x:%02x:%02x:%02x:%02x not found!\n", 
									sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
		TROUT_FUNC_EXIT;
        return;
    }
    TROUT_DBG4("handle addba of entry: %02x:%02x:%02x:%02x:%02x:%02x\n", 
									sa[0], sa[1], sa[2], sa[3], sa[4], sa[5]);
	
    /* Get the Block-Ack Context structure */
    ba_ctxt = (ba_struct_t *)get_ba_handle_entry(entry);

    /******************************************************************/
    /*       ADDBA Request Frame - Frame Body                         */
    /* ---------------------------------------------------------------*/
    /* | Category | Action | Dialog | Parameters | Timeout | SSN     |*/
    /* ---------------------------------------------------------------*/
    /* | 1        | 1      | 1      | 2          | 2       | 2       |*/
    /* ---------------------------------------------------------------*/
    /*                                                                */
    /******************************************************************/

    tid = (data[3] & 0x3C) >> 2;

    memcpy(ba_ctxt->dst_addr, sa, 6);
    ba_ctxt->sta_index   = get_sta_index(entry);

	/* for some reason, sta disconnected and immediately reconnect to ap, but */
	/* ap still has the sta block-ack session info, so it ignore the sta's addba */
	/* request. because of this, before handle addba request, we should judge ba_*/
	/* _rx_ptr is empty or not, if not, free the ba_rx_ptr info first!--chengwg */
	if(ba_ctxt->ba_rx_ptr[tid] != NULL)
	{
		TROUT_DBG4("before handle addba req, free has handled ba info!\n");

		mutex_lock(&ba_rx_mutex);	//add by chengwg 2013-06-21.
		reset_rx_ba_handle(&(ba_ctxt->ba_rx_ptr[tid]), tid);
		mutex_unlock(&ba_rx_mutex);
	}

    if(ba_ctxt->ba_rx_ptr[tid] == NULL)
    {
        /* This request is processed further only if the limit is not exceeded */
        if((mget_ImmediateBlockAckOptionImplemented() == TV_TRUE) &&
           (is_ba_limit_reached() == BFALSE))
        {
            ba_ctxt->ba_rx_ptr[tid] = (ba_rx_struct_t *)mem_alloc(
                g_local_mem_handle, sizeof(ba_rx_struct_t));

            if(ba_ctxt->ba_rx_ptr[tid] == NULL)		//add by chengwg 2013-06-21.
            	printk("%s: alloc ba_rx_struct fail, no mem!\n", __func__);
        }
		TROUT_DBG6("handle addba: ba_rx_ptr[%d]=0x%p\n", tid, ba_ctxt->ba_rx_ptr[tid]);
        if(ba_ctxt->ba_rx_ptr[tid] == NULL)
        {
			//add by chengwg 2013-06-21.
			TROUT_DBG4("ba_limit_reached=%d, immedia_ba=%d\n", is_ba_limit_reached(),
							mget_ImmediateBlockAckOptionImplemented());
			TROUT_DBG4("sta of ba info not found, send delba...\n");
            send_delba(ba_ctxt->dst_addr, tid, RECIPIENT, UNSPEC_QOS_REASON);
            TROUT_FUNC_EXIT;
            return;
        }
    }
#if 0
    else
    {
        /* Any ADDBA Frame received when a Block-ACK Session */
        /* is in progress is discarded.                      */
        TROUT_FUNC_EXIT;
		TROUT_DBG4("handle addba: block-ack session is already in progress!\n");
        return;
    }
#endif	
    g_num_ba_rx_sessions++;
    incr_num_rx_ba_sessions(entry);
    ba = &(ba_ctxt->ba_rx_ptr[tid]);

    mem_set(*ba, 0, sizeof(ba_rx_struct_t));

    (*ba)->conn_status   = BA_INIT;
    (*ba)->sta_index     = ba_ctxt->sta_index;
    (*ba)->dst_addr      = ba_ctxt->dst_addr;
    (*ba)->ba_data.entry = entry;
    (*ba)->ba_data.tid   = tid;
    (*ba)->ba_data.ba    = (UWORD8 *)ba;
    (*ba)->ba_data.direction = RECIPIENT;
	
    send_addba_rsp(ba, data, sa);
    
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_wlan_addba_rsp                                    */
/*                                                                           */
/*  Description   : This function handles the ADDBA Response frame received  */
/*                  from the WLAN interface.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the address of ADDBA Recipient.            */
/*                  2) Pointer to the incoming ADDBA Response message        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Ths function parses the incoming ADDBA-Response frame    */
/*                  from the recipient and sends starts data transfer under  */
/*                  the Block-Ack session if the setup is successful.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_wlan_addba_rsp(UWORD8 *sa, UWORD8 *data)
{
    UWORD8 tid              = 0;
    void           *entry   = NULL;
    ba_struct_t    *ba_ctxt = NULL;
    ba_tx_struct_t **ba     = NULL;

	TROUT_FUNC_ENTER;
	
    /* Get the station entry corresponding to the Recipient */
    entry  = find_entry(sa);

    if(entry == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Get the Block-Ack Context structure */
    ba_ctxt = (ba_struct_t *)get_ba_handle_entry(entry);

    /******************************************************************/
    /*       ADDBA Response Frame - Frame Body                        */
    /* ---------------------------------------------------------------*/
    /* | Category | Action | Dialog | Status  | Parameters | Timeout |*/
    /* ---------------------------------------------------------------*/
    /* | 1        | 1      | 1      | 2       | 2          | 2       |*/
    /* ---------------------------------------------------------------*/
    /*                                                                */
    /******************************************************************/

    tid = (data[5] & 0x3C) >> 2;

    ba = &(ba_ctxt->ba_tx_ptr[tid]);

    if((*ba) == NULL)
    {
        /* Error case */
        send_delba(sa, tid, INITIATOR, UNSPEC_QOS_REASON);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Stop ADDBA Alarm */
    stop_ba_timer(&((*ba)->ba_alarm));
    del_ba_timer(&((*ba)->ba_alarm));

    update_addba_list(ba, data);

    if(((*ba) != NULL) && ((*ba)->conn_status == BA_INPROGRESS))
    {
        /* Enqueue the packets for transmission. Data transfer under */
        /* Block Ack session is started.                             */
        transmit_frames_ba(entry, ba, get_txq_num(tid), tid);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_wlan_delba                                        */
/*                                                                           */
/*  Description   : This function handles the DELBA frame received from the  */
/*                  WLAN interface.                                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the address of DELBA sender.               */
/*                  2) Pointer to the incoming DELBA message                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Ths function parses the incoming DELBA frame and clears  */
/*                  the corresponding Block-Ack session if it exists.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_wlan_delba(UWORD8 *sa, UWORD8 *data)
{
    UWORD8      tid       = 0;
    UWORD8      initiator = 0;
    void        *entry    = 0;
    ba_struct_t *ba_ctxt  = NULL;

	TROUT_FUNC_ENTER;
    /* Get the station entry corresponding to the Recipient */
    entry  = find_entry(sa);

    if(entry == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    /* Get the Block-Ack Context structure */
    ba_ctxt = (ba_struct_t *)get_ba_handle_entry(entry);

    /************************************************/
    /*       DELBA Response Frame - Frame Body      */
    /* -------------------------------------------- */
    /* | Category | Action | Parameters | Reason  | */
    /* -------------------------------------------- */
    /* | 1        | 1      | 2          | 2       | */
    /* -------------------------------------------- */
    /*                                              */
    /************************************************/

    tid       = (data[3] & 0xF0) >> 4;
    initiator = data[3] & 0x08;


    if(initiator == 0)
    {
        ba_tx_struct_t **ba = &(ba_ctxt->ba_tx_ptr[tid]);

        delba_initiator_process(ba, data);
    }
    else
    {
        ba_rx_struct_t **ba = &(ba_ctxt->ba_rx_ptr[tid]);

        delba_recp_process(ba, data);
    }
    TROUT_FUNC_EXIT;
}


#ifdef NO_ACTION_RESET

/*****************************************************************************/
/*                                                                           */
/*  Function Name : save_action_req                                          */
/*                                                                           */
/*  Description   : This function saves an incoming Action request.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming Action request                */
/*                  2) Action request type                                   */
/*                  3) Action category (for private action 0 is used)        */
/*                                                                           */
/*  Globals       : g_saved_action_req                                       */
/*                                                                           */
/*  Processing    : This function saves the incoming action request based on */
/*                  the type. Broadly 2 types of processing is done. Add/    */
/*                  Update for Action requests initiating a session and      */
/*                  Delete for Action requests ending a session.             */
/*                  1. Find an entry with matching DA, TID, Action Category  */
/*                     or a new one in case no matching entry is found       */
/*                  2. Processing 1: Add/Update:                             */
/*                     a. If no matching entry was found update the DA, TID, */
/*                        Action Category and length in the new entry        */
/*                     b. Copy incoming request to the new or existing entry */
/*                  3. Processing 2: Delete:                                 */
/*                     a. If no matching entry was found, do nothing         */
/*                     b. If matching entry was found clear the same         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void save_action_req(UWORD8 *req, UWORD8 type, UWORD8 action_category)
{
    UWORD8 idx   = 0;
    UWORD8 tid   = 0;
    UWORD8 cat   = INVALID_ACTION_CAT;
    UWORD8 len   = 0;
    UWORD8 da[6] = {0};

    /* Get the Action Category, TID, Destination address and Action length   */
    /* based on the type of Action request.                                  */
    switch(action_category)
    {
    case BA_CATEGORY:
    {
        switch(type)
        {
        case MLME_ADDBA_REQ_TYPE:
        case MLME_DELBA_REQ_TYPE:
        {
            cat = BA_ACTION_CAT;
            len = 19; /* ADDBA Request Length */
            tid = req[8];
            mac_addr_cpy(da, req + 2);
        }
        break;
        default:
        {
            /* Do nothing */
        }
        break;
        }
    }
    break;
    default: /* Private Action Category */
    {
        switch(type)
        {
        case A_MSDU_START:
        case A_MSDU_END:
        {
            cat = AMSDU_ACTION_CAT;
            len = 12; /* A-MSDU Start Request Length */
            tid = req[6];
            mac_addr_cpy(da, req);
        }
        break;
        case A_MPDU_START:
        case A_MPDU_END:
        {
            cat = AMPDU_ACTION_CAT;
            len = 11; /* A-MPDU Start Request Length */
            tid = req[6];
            mac_addr_cpy(da, req);
        }
        break;
        default:
        {
            /* Do nothing */
        }
        break;
        }
    }
    break;
    }

    if(mac_addr_cmp(da, mget_StationID()) == BTRUE)
    {
        return;
    }

    /* If the action request does not belong to any category that requires   */
    /* save/restore action, do nothing and return.                           */
    if(cat == INVALID_ACTION_CAT)
        return;

    /* Find the index to an entry in the global saved action request list    */
    /* with matching DA, TID and Action Category. In case no matching entry  */
    /* is found get a new entry.                                             */
    idx = get_saved_action_req_idx(da, tid, cat);

    /* If no valid index is found do nothing and return. This implies that   */
    /* the global action request list is full and no further actions can be  */
    /* saved.                                                                */
    if(idx >= MAX_NUM_SAVED_ACTION_REQ)
        return;

    /* Based on the action request type update the entry with the incoming   */
    /* request (for action requests that initiate a session) or clear the    */
    /* same (for action requests that end a session).                        */
    switch(action_category)
    {
    case BA_CATEGORY:
    {
        switch(type)
        {
        case MLME_ADDBA_REQ_TYPE:
        {
            update_action(idx, cat, len, tid, da, req);
        }
        break;
        /* Delete */
        case MLME_DELBA_REQ_TYPE:
        {
            clear_action(idx);
        }
        break;
        default:
        {
            /* Do nothing */
        }
        break;
        }
    }
    break;
    default: /* Private Action Category */
    {
        switch(type)
        {
        case A_MSDU_START:
        case A_MPDU_START:
        {
            update_action(idx, cat, len, tid, da, req);
        }
        break;
        /* Delete */
        case A_MSDU_END:
        case A_MPDU_END:
        {
            clear_action(idx);
        }
        break;
        default:
        {
            /* Do nothing */
        }
        break;
        }
    break;
    }
    }
}

#endif /* NO_ACTION_RESET */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_txop_frag_min_len_11n                                */
/*                                                                           */
/*  Description   : This function returns the Minimum Length of the frame    */
/*                  which can be subjected to TXOP fragmentation when 802    */
/*                  11n protocol is enabled.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry structure.               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : TXOP fragmentation is not allowed for frames transmitted */
/*                  under Block-Ack or AMPDU session. This function checks   */
/*                  whether there exists any active Block-Ack or AMPDU       */
/*                  sessions with the specified station. If yes, then the    */
/*                  Max frame length is returned. Otherwise, the Min-TXOP    */
/*                  Fragmentation Threshold (EDCA default) is returned.      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 get_txop_frag_min_len_11n(void *entry)
{
    UWORD16 retval = DEFAULT_MIN_TXOP_FRAG_LENGTH;

    if((get_num_tx_ba_sessions(entry) > 0) ||
       (get_num_tx_ampdu_sessions(entry) > 0))
    {
       retval = RX_BUFFER_SIZE;
    }

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : delete_all_ba_handles                                    */
/*                                                                           */
/*  Description   : This function deletes all Block-Ack handles.             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the station entry structure.               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function traverses through the station entry table  */
/*                  and deletes all block-ack handles associated with active */
/*                  station entries.                                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void delete_all_ba_handles(void)
{
    UWORD8        i        = 0;
    void          *te      = 0;
    table_elmnt_t *tbl_elm = 0;

	UWORD8 *pt;
    /* Traverse entire station/association table and process all non-zero    */
    /* entries                                                               */
    for(i = 0; i < MAX_HASH_VALUES; i++)
    {
        tbl_elm = g_sta_table[i];
        while(tbl_elm)
        {
            te = tbl_elm->element;
            if(te == 0)
                break;

			pt = tbl_elm->key;
			printk("<%d>: %02X:%02X:%02X:%02X:%02X:%02X\n", i, pt[0], pt[1], pt[2], pt[3], pt[4], pt[5]);
            reset_ba_handle(get_ba_handle_entry(te));

            tbl_elm = tbl_elm->next_hash_elmnt;
        }
    }
}
#endif /* MAC_802_11N */
