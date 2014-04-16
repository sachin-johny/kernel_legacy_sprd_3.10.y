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
/*  File Name         : wlan_rx_data_ap.c                                    */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the AP MAC FSM on receiving      */
/*                      WLAN_RX (MSDU) events.                               */
/*                                                                           */
/*  List of Functions : ap_enabled_rx_data                                   */
/*                      msdu_indicate_ap                                     */
/*                      wlan_to_wlan                                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "autorate_ap.h"
#include "host_if.h"
#include "prot_if.h"
#include "receive.h"
#include "qmu_if.h"
#include "core_mode_if.h"
#include "ap_prot_if.h"
#include "iconfig.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void msdu_indicate_ap(msdu_indicate_t *msdu);
static void wlan_to_wlan(msdu_indicate_t *msdu, CIPHER_T ct, UWORD8 key_type,
                         UWORD8 key_index, TYPESUBTYPE_T frm_type,
                         asoc_entry_t *ae, BOOL_T use_same_buffer);

extern UWORD16 g_num_mc_bc_qd_pkt;
/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_rx_data                                       */
/*                                                                           */
/*  Description   : This function handles the incoming DATA frame to send to */
/*                  the host.                                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to WLAN receive structure                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the MSDU Indicate function with the  */
/*                  required structure as input.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_enabled_rx_data(mac_struct_t *mac, UWORD8 *msg)
{
    UWORD8             priority_rx   = 0;
    STA_PS_STATE_T     ps            = ACTIVE_PS;
    UWORD8             data_trailer  = 0;
    wlan_rx_t          *wlan_rx      = (wlan_rx_t *)msg;
    UWORD8             *msa          = wlan_rx->msa;
    asoc_entry_t       *da_ae        = 0;
    asoc_entry_t       *sa_ae        = 0;
    msdu_desc_t        *frame_desc   = 0;
    CIPHER_T           ct            = (CIPHER_T)wlan_rx->ct;
    MSDU_PROC_STATUS_T status        = PROC_ERROR;
    TYPESUBTYPE_T      frm_type      = DATA;
    msdu_indicate_t    msdu          = {{0},};
    msdu_proc_state_t  msdu_state    = {0};

    /* Get association entry for the source address */
    sa_ae = (asoc_entry_t *)wlan_rx->sa_entry;
	
    /* Power management checks */
    ps       = (STA_PS_STATE_T)get_pwr_mgt(msa);
    frm_type = (TYPESUBTYPE_T)wlan_rx->sub_type;

    /* Check for WMM-PS trigger frame and process accordingly */
    pwr_mgt_handle_prot(sa_ae, ps, priority_rx, msa);

    /* Check the Power Save Bit in the receive frame */
    check_ps_state(sa_ae, ps);

    /* Check if the received function is the Null function Packet */
    if((frm_type == NULL_FRAME) ||
       (is_sub_type_null_prot(frm_type) == BTRUE))
    {
		// 20120709 caisf add, merged ittiam mac v1.2 code
		check_ps_state(sa_ae, ps);
#ifdef DEBUG_MODE
        g_mac_stats.pewrxnf++;
#endif /* DEBUG_MODE */
        return;
    }

    /* Get the priority of the incoming frame */
    priority_rx   = wlan_rx->priority_val;
    msdu.priority = priority_rx;

    /* Create the MSDU descriptors for the received frame */
    do
    {
        /* Create the MSDU decsriptor */
        status = update_msdu_info(wlan_rx, &msdu, &msdu_state);

        if(PROC_ERROR == status)
            break;

        /* Get the frame descriptor pointer */
        frame_desc = &(msdu.frame_desc);

        /* Get association entry based on the source address to determine    */
        /* the next path of the packet.                                      */
        sa_ae  = (asoc_entry_t* )find_entry(msdu.sa);
        da_ae  = (asoc_entry_t* )find_entry(msdu.da);

        if((da_ae != 0) && (BTRUE == get_p2p_intra_bss_prot()))
        {
            UWORD8 key_type        = 0;
            BOOL_T wlan2wlan       = BFALSE;
            BOOL_T use_same_buffer = BTRUE;

            /* CipherType is reset to no encryption and is set as per the    */
            /* policy used for the out going sta                             */
            ct  = NO_ENCRYP;

            /* Set the Key type required for transmission */
            key_type = UCAST_KEY_TYPE;

            /* If the station is associated with the AP, the packet is put   */
            /* onto the wireless network.                                    */
            if(da_ae->state == ASOC)
            {
#ifndef TX_MACHDR_IN_DSCR_MEM
                /* If the received frame is not a QoS frame, create a    */
                /* copy of the frame in a separate buffer to fit the QoS */
                /* header.                                               */
                if(is_qos_bit_set(msa) == BFALSE)
                    use_same_buffer = BFALSE;

                /* The MAC Header offset field in TX-Dscr is 8 bits long. */
                /* Hence a frame copy is created if the MAC Header field  */
                /* exceeds 256 when it is created in place.               */
#if 0	//shield by chengwg.
                if(frame_desc->data_offset > 255)
                        use_same_buffer = BFALSE;
#else	//because rx frame is alloced by ourself, so we mustn't use the buffer in tx frame!!!
				use_same_buffer = BFALSE;	
#endif
#endif /* TX_MACHDR_IN_DSCR_MEM */

                /* Before forwarding the packet onto the WLAN interface      */
                /* security checks needs to performed on the states of the   */
                /* transmitting and receiving stations                       */
                wlan2wlan = check_sec_ucast_wlan_2_wlan_ap(da_ae, sa_ae, &ct,
                                                           &data_trailer);

                /* Update length of the frame to accommodate the security    */
                /* trailers if any.                                          */
                update_frame_length(frame_desc, data_trailer);
                if(wlan2wlan == BTRUE)
                {
                    wlan_to_wlan(&msdu, ct, key_type, da_ae->sta_index,
                                 frm_type, da_ae, use_same_buffer);
                                 
                }

#ifdef DEBUG_MODE
                g_mac_stats.pewrxu++;
#endif /* DEBUG_MODE */

            }

        }
        else
        {
            /* Broadcast/Multicast frames need to be forwarded on WLAN also */
            if((is_group(msdu.da) == BTRUE) && 
               (BTRUE == get_p2p_intra_bss_prot()))
            {
                CIPHER_T grp_ct    = NO_ENCRYP;

                /* Before forwarding the packet on WLAN and HOST interface   */
                /* security checks needs to performed on the states of the   */
                /* transmitting station                                      */
                if(ap_check_sec_tx_sta_state(sa_ae) != BTRUE)
                {
                    continue;
                }

#ifdef DEBUG_MODE
                g_mac_stats.pewrxb++;
#endif /* DEBUG_MODE */

                /* Before forwarding the packet on the WLAN interface        */
                /* security checks needs to performed on the states of the   */
                /* transmitting station                                      */
                data_trailer = check_sec_bcast_wlan_2_wlan_ap(&grp_ct);

                /* Update the length of the frame to accommodate the         */
                /* security trailers if any.                                 */
                update_frame_length(frame_desc, data_trailer);

                wlan_to_wlan(&msdu, grp_ct, BCAST_KEY_TYPE, 0,
                             frm_type, NULL, BFALSE);
            }
            else
            {
                /* Packet is a Unicast packet to the AP */
#ifdef DEBUG_MODE
                g_mac_stats.pewrxu++;
#endif /* DEBUG_MODE */
            }

            /* Check for SNAP header at beginning of the data and set the    */
            /* data pointer and length accordingly.                          */
            if(BTRUE == is_snap_header_present(frame_desc))
            {
                /* If received packet is a security handshake packet process */
                /* it in the security layer                                  */
                if(is_sec_handshake_pkt_ap(sa_ae, frame_desc->buffer_addr,
                                           frame_desc->data_offset,
                                           frame_desc->data_len,
                                           (CIPHER_T)wlan_rx->ct) == BTRUE)
                {
                    continue;
                }

                /* Adjust the frame to account for the SNAP header */
                adjust_for_snap_header(frame_desc);
            }
            else
            {
                /* Before forwarding the packet on WLAN and HOST interface   */
                /* security checks needs to performed on the states of the   */
                /* transmitting station                                      */
                if(ap_check_sec_tx_sta_state(sa_ae) != BTRUE)
                {
                    continue;
                }
            }

            /* Call MSDU Indicate API with the MSDU to be sent to the host */
            msdu_indicate_ap(&msdu);
        }

    } while(PROC_MORE_MSDU == status);

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : msdu_indicate_ap                                         */
/*                                                                           */
/*  Description   : This function sends the received MSDU to the host.       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MSDU Indicate message                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function creates and sends the frame to the host    */
/*                  based on the host type.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void msdu_indicate_ap(msdu_indicate_t *msdu)
{
    UWORD8      host_type   = get_host_data_if_type();
    msdu_desc_t *frame_desc = &(msdu->frame_desc);

    /* Add one user to the buffer being sent to the host. This ensures that  */
    /* the buffer will remain allocated even after the process rx frame      */
    /* function frees it.                                                    */
    //mem_add_users(g_shared_pkt_mem_handle, frame_desc->buffer_addr, 1);

    /* Prepare required frame for the current configured data host interface */
    prepare_host_data_frame(msdu, host_type);

    /* Send the data frame to the required host */
    send_data_frame_to_host_if(frame_desc, host_type);

    print_log_debug_level_1("\n[DL1][INFO][Rx] {MSDU Indicate API}");

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wlan_to_wlan                                             */
/*                                                                           */
/*  Description   : This function prepares and sends packet from WLAN        */
/*                  network back to it after appropriate header changes.     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame received from the WLAN           */
/*                  2) Length of the frame                                   */
/*                  3) MAC header length                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function changes the header format of the packet    */
/*                  received from the WLAN to send to the station in the     */
/*                  same WLAN network. It changes the order of the addresses */
/*                  & sets ToDs bit to indicate that the frame is from the   */
/*                  AP.                                                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wlan_to_wlan(msdu_indicate_t *msdu, CIPHER_T ct, UWORD8 key_type,
                  UWORD8 key_index, TYPESUBTYPE_T frm_type,
                  asoc_entry_t *ae, BOOL_T use_same_buffer)
{
    UWORD8        q_num        = 0;
    UWORD8        tx_rate      = 0;
    UWORD8        pream        = 0;
    UWORD8        service_class= 0;
    UWORD8        *mac_hdr     = 0;
    UWORD8        *tx_dscr     = 0;
    UWORD16       mac_hdr_len  = 0;
    void          *amsdu_ctxt  = NULL;
    UWORD8        is_amsdu     = 0;
    BOOL_T        is_qos       = BFALSE;
    BOOL_T        is_htc       = BFALSE;
    msdu_desc_t   *buff_desc   = &(msdu->frame_desc);
    UWORD32       phy_tx_mode  = 0;
    UWORD32       sta_index    = INVALID_ID;
    buffer_desc_t buffer_desc  = {0};
    UWORD32       retry_set[2] = {0};

#ifdef DEBUG_MODE
        g_mac_stats.w2w++;
#endif /* DEBUG_MODE */

    /* Find the queue to which this packet needs to be added */
    q_num = get_w2w_txq_num_ap(msdu->priority);

    if(ae != NULL)
        sta_index = ae->sta_index;

    /* If the queue for which this packet belongs is full, then return */
    if(is_txq_full(q_num, sta_index) == BTRUE)
    {
        return;
    }

    /* Prepare the buffer descriptor for the payload based on the flag to    */
    /* use the same buffer. If the frame is to be sent on both WLAN as well  */
    /* as the host there may be cases in which the payload portion needs to  */
    /* be modified to be sent to the host. This causes the packet being sent */
    /* on WLAN to be altered if the same buffer is used. As a result the     */
    /* same buffer cannot be used for transmitting on WLAN in this case.     */
    /* However if the packet is to be sent only on WLAN this problem does    */
    /* not occur and hence the same buffer can be used.                      */
    if(use_same_buffer == BTRUE)
    {
        /* Add one user to the incoming buffer. This ensures that the buffer */
        /* will remain allocated even after the process rx frame function    */
        /* frees it.                                                         */
        mem_add_users(g_shared_pkt_mem_handle, buff_desc->buffer_addr, 1);

        /* Prepare a buffer descriptor with required information */
        buffer_desc.buff_hdl    = buff_desc->buffer_addr;
        buffer_desc.data_offset = buff_desc->data_offset;
        buffer_desc.data_length = buff_desc->data_len;
    }
    else
    {
        UWORD8 *out_data = 0;

        /* Allocate a new buffer for the payload outgoing on WLAN */
        out_data = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);
        if(out_data == NULL)
            return;

        /* Prepare a buffer descriptor with required information */
        buffer_desc.buff_hdl    = out_data;
        buffer_desc.data_offset = MAX_MAC_HDR_LEN;
        buffer_desc.data_length = buff_desc->data_len;

	if(buffer_desc.data_offset + buff_desc->data_len > SHARED_PKT_MEM_BUFFER_SIZE){
		printk("@@@:BUG! bad memcpy offset:%d, len:%d\n", buffer_desc.data_offset,
			buff_desc->data_len);
	}
        /* Copy the payload from the incoming packet to the outgoing buffer */
        memcpy(buffer_desc.buff_hdl + buffer_desc.data_offset,
               buff_desc->buffer_addr + buff_desc->data_offset,
               buff_desc->data_len);
    }

    /* Set the transmit rate to the required value. To support Multi-Rate    */
    /* the transmit rate is set to the maximum basic rate and the preamble   */
    /* is set to Long.                                                       */

    if(is_group(msdu->da) == BTRUE)
    {
        /* For multicast frames send frames at maximum rate */
        tx_rate = get_max_basic_rate();
        pream   = 1;
        service_class = BCAST_NO_ACK;

        /* Update the retry set information for this frame */
        update_retry_rate_set(0, tx_rate, 0, retry_set);
    }
    else
    {
        /* Get the transmit rate for the associated station based on the     */
        /* auto-rate, multi-rate or user-rate settings. The preamble must be */
        /* set accordingly.                                                  */
	    if(!ae)
		    printk("@@@:BUG ae==NULL to get rate\n");
        tx_rate = get_tx_rate_to_sta(ae);
        pream   = get_preamble(tx_rate);
        service_class = g_ack_policy;

        /* Update the retry set information for this frame */
        update_retry_rate_set(1, tx_rate, ae, retry_set);
    }

    /* AMSDU on WLAN-WLAN path is disabled since no space currently exists */
    /* in the RX-Buffer to add the sub-MSDU header                         */
#ifdef TX_MACHDR_IN_DSCR_MEM
    /* Check whether AMSDU aggregation is possible */
    amsdu_ctxt = get_amsdu_handle(msdu->da, msdu->priority, q_num, ae,
                                  buff_desc->data_len, 1, tx_rate);
#endif /* TX_MACHDR_IN_DSCR_MEM */

    /* Check whether AMSDU option is enabled */
    is_amsdu = (amsdu_ctxt != NULL);

    /* Check whether AMSDU aggregation is not supported or if this is the */
    /* first sub-MSDU of an AMSDU.                                        */
    if((0 == is_amsdu) || (NULL == amsdu_get_tx_dscr(amsdu_ctxt)))
    {
        /* Check whether QoS option is implemented */
        is_qos = is_qos_required(sta_index);

        /* Check whether the receiver is HT capable */
        is_htc = is_htc_capable(ae);

        /* Create the transmit descriptor and set default contents */
        tx_dscr = create_default_tx_dscr(is_qos, msdu->priority, is_amsdu);
        if(tx_dscr == NULL)
        {
            if(is_amsdu == 1)
            {
                /* If allocation of the AMSDU decriptor has failed, then */
                /* attempt transmission of the MSDU as a normal-MSDU.    */
                free_amsdu_handle(amsdu_ctxt);
                is_amsdu   = 0;
                amsdu_ctxt = NULL;
                tx_dscr = create_default_tx_dscr(is_qos, msdu->priority,
                                                 is_amsdu);
            }

            if(tx_dscr == NULL)
            {
				//caisf add 2013-02-15
                if(g_mac_net_stats)
                    g_mac_net_stats->tx_dropped++;
					
                pkt_mem_free(buffer_desc.buff_hdl);
                return;
            }
        }

        /* Update the AMSDU handle with the tx descriptor */
        if(1 == is_amsdu)
            amsdu_set_tx_dscr(amsdu_ctxt, tx_dscr);

        /* Create the MAC header of the out-going frame */
#ifdef TX_MACHDR_IN_DSCR_MEM
        mac_hdr = mem_alloc(g_shared_dscr_mem_handle, MAX_MAC_HDR_LEN);
        if(NULL == mac_hdr)
        {
            free_tx_dscr((UWORD32 *)tx_dscr);
			
			//caisf add 2013-02-15
            if(g_mac_net_stats)
                g_mac_net_stats->tx_dropped++;
				
            pkt_mem_free(buffer_desc.buff_hdl);
            free_amsdu_handle(amsdu_ctxt);
            return;
        }
#else  /* TX_MACHDR_IN_DSCR_MEM */
        mac_hdr = buffer_desc.buff_hdl + buffer_desc.data_offset - MAX_MAC_HDR_LEN;
#endif /* TX_MACHDR_IN_DSCR_MEM */

        /* Reset the frame control field. Set type/subtype extracted from the    */
        /* incoming frame and set 'from_ds' bit in the outgoing frame.           */
        /* Also set the Security type for the outgoing frame based on cipher type*/
        mac_hdr_len = set_mac_hdr_prot(mac_hdr, msdu->priority, service_class,
                                       is_qos, is_htc, is_amsdu);

        set_from_ds(mac_hdr, 1);
        if(ct != NO_ENCRYP)
            set_wep(mac_hdr, 1);

        /* Set Destination Address (DA), BSSID and Source Address (SA) in the   */
        /* outgoing frame.                                                      */
        set_address1(mac_hdr, msdu->da);
        set_address2(mac_hdr, mget_bssid());
        set_address3(mac_hdr, msdu->sa);

        /* Set the mac header related parameters in the Tx descriptor */
        set_tx_dscr_buffer_addr((UWORD32 *)tx_dscr, (UWORD32)mac_hdr);
        set_tx_dscr_mh_len((UWORD32 *)tx_dscr, mac_hdr_len);
        set_tx_dscr_mh_offset((UWORD32 *)tx_dscr, 0);

        /* Get the PHY transmit mode based on the transmit rate and preamble */
        phy_tx_mode = get_dscr_phy_tx_mode(tx_rate, pream, (void *)ae);

        /* Set the Tx-descriptor parameters */
        set_tx_params(tx_dscr, tx_rate, pream, service_class, phy_tx_mode,
                      retry_set);
        set_tx_dscr_q_num((UWORD32 *)tx_dscr, q_num);

        /* Set Security Parameters for the frame */
        set_tx_security(tx_dscr, ct, key_type, key_index);

        /* Set the HT power save parameters */
        set_ht_ps_params(tx_dscr, (void *)ae, tx_rate);

        /* Set the receiver address LUT index */
        set_ht_ra_lut_index(tx_dscr, get_ht_handle_entry((void *)ae),
                            msdu->priority, tx_rate);

    }
    else
        tx_dscr = amsdu_get_tx_dscr(amsdu_ctxt);


    /* Set the sub-MSDU info table in the Tx-Descriptor */
    set_submsdu_info(tx_dscr, &buffer_desc, 1, buffer_desc.data_length,
                     mac_hdr_len, msdu->sa, msdu->da, amsdu_ctxt);

    /* Update the TSF timestamp */
    update_tx_dscr_tsf_ts((UWORD32 *)tx_dscr);

    /* Transmit the MSDU */
    if(BTRUE == is_tx_ready(amsdu_ctxt))
    {
		// 20120709 caisf mod, merged ittiam mac v1.2 code
        if(BTRUE == tx_data_packet((UWORD8 *)ae, msdu->da, msdu->priority, q_num,
                                   tx_dscr, amsdu_ctxt))
        {
		    /* Increment count indicating packet added to Hw queue */
		    /* Check if packet Bcast/Mcast or Unicast */
		    if(is_group(msdu->da) == BTRUE)
		    {
		        g_num_mc_bc_qd_pkt++;
		    }
		    else
		    {
		        ae->num_qd_pkt++;
            }
        }
    }
}

#endif /* BSS_ACCESS_POINT_MODE */

