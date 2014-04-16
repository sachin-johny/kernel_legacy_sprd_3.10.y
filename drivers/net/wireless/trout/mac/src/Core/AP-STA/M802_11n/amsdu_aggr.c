/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : amsdu_aggr.c                                         */
/*                                                                           */
/*  Description       : This file contains all the functions related to MSDU */
/*                      aggregation.                                         */
/*                                                                           */
/*  List of Functions : amsdu_tx                                             */
/*                      get_amsdu_ctxt                                       */
/*                      amsdu_tx_timeout_handler                             */
/*                      amsdu_set_submsdu_info                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11N

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "transmit.h"
#include "maccontroller.h"
#include "amsdu_aggr.h"
#include "qmu_if.h"
#include "management_11n.h"
#include "core_mode_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* 说明:当前TX Block-Ack可以工作，但是开启TX_BLOCK_ACK功能后，发包时伴随有RTS*/
/* 导致吞吐上不去，因此，非特殊情况下，建议不开启TX_BLOCK_ACK功能。          */
/* 另外，TX-AMPDU功能还不完善，只是在验证硬件逻辑功能时，可以打开，平时不建议*/
/* 打开。                                                                    */
/*****************************************************************************/
//#define ENABLE_AMSDU
#undef ENABLE_AMPDU
#undef TX_BLOCK_ACK

amsdu_ctxt_t g_amsdu_ctxt[MAX_NUM_SIMUL_AMSDU];

void config_amsdu_func(UWORD8 *ra)
{
	UWORD8 *buf = NULL;
	UWORD8 amsdu_cfg[] = 
    {
		0x0F, 	/* total length(include length field) */
		0x07, 	/* HT-Category Action Frame */
		0x00, 	/* Action type: A-MSDU start */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00,	/* TID */
		0x10, 	/* Max Num-MSDUs */
		0xFF, 	/* Max-AMSDU len(LSB) */
		0x0E, 	/* Max-AMSDU len(MSB) */
		0x0A, 	/* Timeout in ms(LSB) */
		0x00,	/* Timeout in ms(MSB) */
    };

	TROUT_DBG4("Config A-MSDU function...\n");

	if(ra == NULL)
    {
		buf = mget_bssid();
		if(buf == NULL)
		{
			TROUT_DBG2("Config A-MSDU: get bssid failed!\n");
			TROUT_FUNC_EXIT;
			return;
		}
	}
	else
		buf = ra;

	memcpy((void *)(&amsdu_cfg[3]), buf, 6);
	set_action_request(amsdu_cfg);
}

/* This function config and start a block-ack tx session, after session setup */
/* tx block-ack is function is started. By the way, rx block-ack is setuped in*/
/* condition of rx received a action frame with ADDBA request                 */
void config_block_ack_func(UWORD8 *ra, BOOL_T B_ACK)
{
	UWORD8 *buf = NULL;
	UWORD8 b_ack_cfg[] = 
	{
		0x14,	/* total length(include length field) */
		0x03,	/* BA Category Action Frame */
		0x00,	/* Action type: ADDBA Request */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00,	/* TID */
		0x01,	/* BA Policy */
		0x20,	/* BA Window size(LSB) */
		0x00,	/* BA Window size(MSB) */
		0x00,	/* BA Timeout in ms(LSB) */
		0x00,	/* BA Timeout in ms(MSB) */
		0xF4,	/* ADDBA Timeout in ms(LSB) */
		0x01, 	/* ADDBA Timeout in ms(MSB) */
		0x10,	/* Grouping Buffer Max Frames */
		0x64,	/* Grouping Buffer Timeout in ms(LSB) */
		0x00,	/* Grouping Buffer Timeout in ms(MSB) */
	};

	TROUT_DBG4("Config Block-Ack function...\n");

	if(ra == NULL)
    {
		buf = mget_bssid();
		if(buf == NULL)
		{
			TROUT_DBG2("Config Block-Ack: get bssid failed!\n");
			return;
		}
	}
	else
		buf = ra;
		
	memcpy((void *)(&b_ack_cfg[3]), buf, 6);	

	if((B_ACK) && get_immediate_block_ack_enable() == 1)
		set_action_request(b_ack_cfg);
}


void config_ampdu_func(UWORD8 *ra)	//not complete!
{
	UWORD8 *buf = NULL;
	UWORD8 ampdu_cfg[] = 
    {
		0x0E, 	/* total length(include length field) */
		0x07, 	/* HT-Category Action Frame */
		0x02, 	/* Action type: A-MPDU start */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00, 	/* Receiver Address */
		0x00,	/* TID(0~7) */
		0x10, 	/* Max Num-MPDUs */
		0x64, 	/* AMPDU Timeout in ms(LSB) */
		0x00,	/* AMPDU Timeout in ms(MSB) */
		0x00,	/* ACK Policy */
    };

	TROUT_DBG4("Config A-MPDU function...\n");

	if(ra == NULL)
    {
		buf = mget_bssid();
		if(buf == NULL)
		{
			TROUT_DBG2("Config A-MPDU: get bssid failed!\n");
			return;
		}
	}
	else
		buf = ra;
	memcpy((void *)(&ampdu_cfg[3]), buf, 6);

	/* if B-ACK is disabled, use normally ack policy */
	if(get_immediate_block_ack_enable() == 0)
	{
		ampdu_cfg[0xD] = 0x01;
	}
	set_action_request(ampdu_cfg);
}


/* This function configure 802.11n new feature of A-MSDU, A-MPDU & Block-Ack  */
void config_802_11n_feature(UWORD8 *ra, BOOL_T B_ACK)
{
	/* before BA-Tx/Rx setup and A-MPDU setup with ack policy is normal-ack,  */
	/* immediate block ack enable variable must be set to 1 first             */
	if(B_ACK)
	{
		if(get_immediate_block_ack_enable() == 0)
			set_immediate_block_ack_enable(1);
	}
	else
	{
		if(get_immediate_block_ack_enable() == 1)
			set_immediate_block_ack_enable(0);
	}

#ifdef ENABLE_AMSDU
	config_amsdu_func(ra);
#endif

#ifdef TX_BLOCK_ACK
	config_block_ack_func(ra, B_ACK);
#endif

#ifdef ENABLE_AMPDU
	config_ampdu_func(ra);
#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : amsdu_tx                                                 */
/*                                                                           */
/*  Description   : This function handles adding of the AMSDU Tx descriptor  */
/*                  to the Tx queue.                                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the AMSDU context structure                */
/*                                                                           */
/*  Globals       : g_amsdu_ctxt                                             */
/*                                                                           */
/*  Processing    : The tx descriptor of the AMSDU is updated with the       */
/*                  length parameters and is queued for transmission         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T, BTRUE if the packet has been added to the H/w    */
/*                          queue for transmission. BFALSE otherwise.        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
BOOL_T amsdu_tx(amsdu_ctxt_t *amsdu_ctxt)
{
    BOOL_T  ret_val   = BFALSE;
    UWORD16 frame_len = 0;
    UWORD32 *tx_dscr  = (UWORD32 *)amsdu_ctxt->tx_dscr;

	TROUT_FUNC_ENTER;
    if(0 == amsdu_ctxt->in_use_flag)
    {
		TROUT_FUNC_EXIT;
        return ret_val;
	}
    /* Stop the AMSDU Tx Timeout Timer */
    stop_amsdu_timer(&(amsdu_ctxt->to_alarm));

    /* Adjust the AMSDU parameters before transmission */
    adjust_amsdu_before_txn(amsdu_ctxt);

    /* Set the AMSDU length in the tx descriptor */
    frame_len = get_tx_dscr_mh_len((UWORD32 *)tx_dscr) +
                                   amsdu_ctxt->amsdu_size + FCS_LEN;
    set_tx_dscr_frame_len((UWORD32 *)tx_dscr, frame_len);
    set_tx_dscr_frag_len((UWORD32 *)tx_dscr, frame_len, 0);

    /* Set the number of sub-MSDUs in the tx descriptor */
    set_tx_dscr_num_submsdu(tx_dscr, amsdu_ctxt->num_entries);

    /* Check whether the MSDU is to be buffered */
    if(buffer_tx_packet(amsdu_ctxt->entry, amsdu_ctxt->addr,
                        amsdu_ctxt->priority_val, amsdu_ctxt->q_num,
                        (UWORD8 *)tx_dscr) == BFALSE)
    {
        /* Enqueue the MSDU for transmission */
        TX_PATH_DBG("%s: add tx pkt\n", __func__);
        if(qmu_add_tx_packet(&g_q_handle.tx_handle, amsdu_ctxt->q_num,
                            (UWORD8 *)tx_dscr) != QMU_OK)
        {
            /* Exception. Do nothing. */
#ifdef DEBUG_MODE
            g_mac_stats.qaexc++;
#endif /* DEBUG_MODE */
			//caisf add 2013-02-15
            if(g_mac_net_stats)
                g_mac_net_stats->tx_dropped++;

            print_log_debug_level_1(
                "\n[DL1][INFO][Tx] {AMSDU Packet not added to queue}");
            free_tx_dscr(tx_dscr);
        }
        else
        {
            ret_val = BTRUE;
            print_log_debug_level_1(
                "\n[DL1][INFO][Tx] {AMSDU Packet added to queue}");
        }

    }

    /* Free the AMSDU context structure */
    free_amsdu_ctxt(amsdu_ctxt);

	TROUT_FUNC_EXIT;
    return ret_val;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_amsdu_ctxt                                           */
/*                                                                           */
/*  Description   : This function manages the AMSDU context structures.      */
/*                                                                           */
/*  Inputs        : 1) Receiver Address of the MSDU                          */
/*                  2) TID of the MSDU                                       */
/*                  3) Q-Number                                              */
/*                  4) RA-STA related entry                                  */
/*                  5) Current transmission rate                             */
/*                                                                           */
/*  Globals       : g_amsdu_ctxt                                             */
/*                                                                           */
/*  Processing    : The AMSDU context structure is returned if an entry      */
/*                  already exists for the RA,TID combination. Else, a new   */
/*                  structure is created if possible and initialized.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : Pointer to the AMSDU context structure.                  */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

amsdu_ctxt_t *get_amsdu_ctxt(UWORD8 *rx_addr, UWORD8 tid, UWORD8 q_num,
                             void *entry, UWORD16 msdu_len, UWORD8 num_frags,
                             UWORD8 curr_tx_rate)
{
    UWORD32        indx = 0;
    UWORD32        nxt_indx = MAX_NUM_SIMUL_AMSDU;
    ht_struct_t    *ht_hdl          = NULL;
    ht_tx_struct_t *ht_tx_hdl       = NULL;

	TROUT_FUNC_ENTER;
    /* AMSDU aggregation of Bcst/Mcst frames not supported */
    if((tid >= 16) || (NULL == entry))
    {
		TROUT_FUNC_EXIT;
        return NULL;
	}
    /* Check whether AMSDU is setup for the RA/TID pair */
    ht_hdl    = (ht_struct_t *)get_ht_handle_entry(entry);
    ht_tx_hdl = ht_hdl->ht_tx_ptr[tid];

    /* If the Receiver is not HT capable or if AMSDU session is not setup */
    /* then AMSDU aggregation is not possible                             */

	//printk("tid=%d, entry=0x%p, ht_capable=%d, ", tid, entry, ht_hdl->ht_capable);
	//if(ht_tx_hdl != NULL)
	//	printk("ht_tx_hdl=0x%p, amsdu_maxnum=%d, curr_tx_rate=0x%x\n", ht_tx_hdl, ht_tx_hdl->amsdu_maxnum, curr_tx_rate);
		
    if((0 == ht_hdl->ht_capable) || (NULL == ht_tx_hdl) ||
       (0 == ht_tx_hdl->amsdu_maxnum))
    {   
		TROUT_FUNC_EXIT;
        return NULL;
	}
	

    /* first to recycle invalid amsdu ctxt due to timeout XXX etc... by zhao */
    for(indx = 0; indx < MAX_NUM_SIMUL_AMSDU; indx++){
	if( g_amsdu_ctxt[indx].invalid ){
		if(g_amsdu_ctxt[indx].tx_dscr)
			free_tx_dscr(g_amsdu_ctxt[indx].tx_dscr);
		else
			printk("@@@:BUG! amsdu_ctxt invalid, but tx_dscr is NULL\n");
		free_amsdu_ctxt(&g_amsdu_ctxt[indx]);
	}
    }
    for(indx = 0; indx < MAX_NUM_SIMUL_AMSDU; indx++)
    {
        /* Check whether a AMSDU context structure exists for the specified */
        /* RA & TID combination                                             */
        if(1 == g_amsdu_ctxt[indx].in_use_flag)
        {
            if((mac_addr_cmp(g_amsdu_ctxt[indx].addr, rx_addr) == BTRUE) &&
               (g_amsdu_ctxt[indx].priority_val == tid))
            {
                /* Check whether the new MSDU can fit into the A-MSDU */
                if(BTRUE == check_amsdu_space_status(&g_amsdu_ctxt[indx],
                                                      msdu_len, num_frags))
                {
                    return (&g_amsdu_ctxt[indx]);
                }

                /* Transmit the existing A-MSDU if the new MSDU cannot be */
                /* in it.                                                 */
				TX_PATH_DBG("%s: amsdu tx\n", __func__);
                amsdu_tx(&g_amsdu_ctxt[indx]);

                nxt_indx = indx;
                break;
            }
        }else{
		nxt_indx = indx;
		/* if find avaliable ctxt, then exit! by zhao */
		break;
	}
    }
    /* Can't aggregate if the all aggregation contexts are in use or the */
    /* MSDU length exceed the set maximum A-MSDU payload length.         */
    if((MAX_NUM_SIMUL_AMSDU == nxt_indx) ||
       (msdu_len >= ht_tx_hdl->amsdu_maxsize))
    {
		TROUT_FUNC_EXIT;
        return NULL;
    }

    /* Before creating  new AMSDU context structure and starting aggregation */
    /* check if the current transmit rate is a HT rate. If not aggregation   */
    /* cannot be done.                                                       */
    if(IS_RATE_MCS(curr_tx_rate) == BFALSE)
    {
		TROUT_FUNC_EXIT;
        return NULL;
    }

    /* Initialize the AMSDU context structure */
    g_amsdu_ctxt[nxt_indx].in_use_flag   = 1;
    g_amsdu_ctxt[nxt_indx].num_msdu      = 0;
    g_amsdu_ctxt[nxt_indx].num_entries   = 0;
    /* The max AMSDU size is to be set on a per STA basis */
#ifndef ALLOW_AMSDU_FRAGMENTATION
    {
        UWORD16 max_unfrag_frame_len = 0;

        max_unfrag_frame_len = mget_FragmentationThreshold() -
                            (HT_MAC_HDR_LEN + FCS_LEN + MAX_SEC_HEADER_LENGTH);

        g_amsdu_ctxt[nxt_indx].amsdu_maxsize = MIN(ht_tx_hdl->amsdu_maxsize,
                                                   max_unfrag_frame_len);
    }
#else /* ALLOW_AMSDU_FRAGMENTATION */
    g_amsdu_ctxt[nxt_indx].amsdu_maxsize = ht_tx_hdl->amsdu_maxsize;
#endif /* ALLOW_AMSDU_FRAGMENTATION */
    g_amsdu_ctxt[nxt_indx].max_num       = ht_tx_hdl->amsdu_maxnum;
    g_amsdu_ctxt[nxt_indx].amsdu_size    = 0;
    g_amsdu_ctxt[nxt_indx].last_msdu_padlen = 0;
    g_amsdu_ctxt[nxt_indx].priority_val  = tid;
    g_amsdu_ctxt[nxt_indx].tx_dscr       = NULL;
    g_amsdu_ctxt[nxt_indx].q_num         = q_num;
    g_amsdu_ctxt[nxt_indx].entry         = entry;
    memcpy(g_amsdu_ctxt[nxt_indx].addr, rx_addr, 6);
    g_amsdu_ctxt[nxt_indx].invalid       = 0;

    start_amsdu_timer(&(g_amsdu_ctxt[nxt_indx].to_alarm),
                      ht_tx_hdl->amsdu_to_intvl,
                      (UWORD32)(&(g_amsdu_ctxt[nxt_indx])));

	TROUT_FUNC_EXIT;
    return (&g_amsdu_ctxt[nxt_indx]);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : amsdu_set_submsdu_info                                   */
/*                                                                           */
/*  Description   : This function sets the sub-MSDU information table in the */
/*                  Tx-descriptor.                                           */
/*                                                                           */
/*  Inputs        : 1) Tx Descriptor                                         */
/*                  2) List of payload buffer descriptors                    */
/*                  3) Number of payload buffers                             */
/*                  4) Size of the payload                                   */
/*                  5) Length of the MAC header                              */
/*                  6) MSDU Source Address                                   */
/*                  7) MSDU Destination Address                              */
/*                  8) AMSDU Context structure                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The sub-MSDU table within the tx-dscr is updated with    */
/*                  the specified parameters. The AMSDU context is also      */
/*                  updated if AMSDU aggregation is enabled.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None.                                                    */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void amsdu_set_submsdu_info(UWORD8 *tx_dscr, buffer_desc_t *buff_list,
                            UWORD16 num_buff, UWORD16 buff_len,
                            UWORD16 mac_hdr_len, UWORD8 *sa,
                            UWORD8 *da, void *amsdu_handle)
{
    UWORD8  *sub_msdu_hdr  = 0;
    UWORD8  *sub_msdu_tail = 0;
    UWORD16 last_sub_frag  = num_buff - 1;
    UWORD16 sub_msdu_len   = 0;
    UWORD8  pad_length     = 0;
    UWORD16 submsdu_idx    = 0;
    UWORD16 frame_len      = 0;
    UWORD16 indx           = 0;
    amsdu_ctxt_t *amsdu_ctxt = (amsdu_ctxt_t *)amsdu_handle;

	TROUT_FUNC_ENTER;
    if(NULL != amsdu_ctxt)
    {
        /* Compute the pointers to the sub-MSDU header & tail */
        sub_msdu_hdr  = buff_list[0].buff_hdl + buff_list[0].data_offset -
                        SUB_MSDU_HEADER_LENGTH;
        sub_msdu_tail = buff_list[last_sub_frag].buff_hdl +
                        buff_list[last_sub_frag].data_offset +
                        buff_list[last_sub_frag].data_length;
        sub_msdu_len  = buff_len + SUB_MSDU_HEADER_LENGTH;

        /* Set the sub-MSDU header */
        set_submsdu_header(sub_msdu_hdr, sa, da, buff_len);

        /* Set the sub-MSDU padding */
        pad_length = set_submsdu_padding(sub_msdu_tail, sub_msdu_len);

        /* Update the Fragment Buffer parameters */
        buff_list[0].data_offset -= SUB_MSDU_HEADER_LENGTH;
        buff_list[0].data_length += SUB_MSDU_HEADER_LENGTH;
        buff_list[last_sub_frag].data_length += pad_length;

        /* Get the index of the next sub-MSDU from the AMSDU handle */
        submsdu_idx = amsdu_ctxt->num_entries;

        /* Update the AMSDU context parameters */
        amsdu_ctxt->amsdu_size += sub_msdu_len + pad_length;
        amsdu_ctxt->last_msdu_padlen = pad_length;
        amsdu_ctxt->num_entries     += num_buff;
        amsdu_ctxt->num_msdu        += 1;
    }
    else
    {
        /* Set the length of the entire frame (including MAC header and FCS) */
        frame_len = buff_len + mac_hdr_len + FCS_LEN;

        /* Set the frame length in the Tx Descriptor */
        set_tx_dscr_frame_len((UWORD32 *)tx_dscr, frame_len);
        //printk("%s: num_buff = %d\n", __FUNCTION__, num_buff);
        set_tx_dscr_num_submsdu((UWORD32 *)tx_dscr, num_buff);
        set_tx_dscr_frag_len((UWORD32 *)tx_dscr, frame_len, 0);

        submsdu_idx = 0;

#ifndef DEBUG_KLUDGE
        {
            buffer_desc_t *buffer_desc = &buff_list[num_buff - 1];

            /* This Kludge fixes the TX-Hang issue in MAC H/w */
            buffer_desc->data_length += 4;
        }
#endif /* DEBUG_KLUDGE */

    }

    /* Set the data buffer related information in the Tx-descriptor */
    for(indx = 0; indx < num_buff; indx++)
    {
        set_tx_dscr_submsdu_buff_info((UWORD32 *)tx_dscr,
                                      (UWORD32)(&(buff_list[indx])),
                                      submsdu_idx);
        submsdu_idx++;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : amsdu_tx_timeout_handler                                 */
/*                                                                           */
/*  Description   : This function handles the AMSDU transmission timeout     */
/*                  interrupt.                                               */
/*                                                                           */
/*  Inputs        : 1) Context structure of the timed-out AMSDU.             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : A miscellaneous event is posted corresponding to the     */
/*                  timeout event.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void amsdu_tx_timeout_handler(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void amsdu_tx_timeout_handler(ADDRWORD_T data)
#endif /* OS_LINUX_CSL_TYPE */
{
    amsdu_ctxt_t     *amsdu_ctxt = (amsdu_ctxt_t *)data;
    misc_event_msg_t *misc       = 0;

    /* Exception */
    if(0 == amsdu_ctxt->in_use_flag)
        return;

    /* Create a MISCELLANEOUS event with the pointer to the AMSDU ctxt and   */
    /* post it to the event queue.                                           */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
    if(misc == NULL)
    {
	/* if misc mem alloc fialed , we shouldn't free amdsu->tx_dscr directly, 
	 * we mark this amsdu as invalid, later, someone get amsdu_ctxt, we will 
	 * free it by zhao 
	 */
	amsdu_ctxt->invalid = 1;
	return;
    }
    else
    {
        misc->name = MISC_AMSDU_TX;
    }

    misc->data = (UWORD8 *)amsdu_ctxt;
    //misc->name = MISC_AMSDU_TX;
    // End modify by Ke.Li at 2013-07-17 for fixing bug amsdu null pointer

    post_event((UWORD8*)misc, MISC_EVENT_QID);
}

#endif /* MAC_802_11N */
