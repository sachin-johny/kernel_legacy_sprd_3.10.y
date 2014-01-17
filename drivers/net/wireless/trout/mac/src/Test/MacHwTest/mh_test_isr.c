/*****************************************************************************/
/*                                                                           */
/*              Ittiam 802.11 MAC HARDWARE UNIT TEST CODE                    */
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
/*  File Name         : mh_test_isr.c                                        */
/*                                                                           */
/*  Description       : This file contains the interrupt service routines    */
/*                      (ISRs). All ISRs will acknowledge the interrupt.     */
/*                                                                           */
/*  List of Functions : mac_isr                                              */
/*                      tx_complete_isr                                      */
/*                      rx_complete_isr                                      */
/*                      tbtt_isr                                             */
/*                      deauth_isr                                           */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mh_test.h"
#include "receive.h"
#include "mh_test_utils.h"
#include "host_if.h"
#include "iconfig.h"

#ifdef MAC_802_11I
#include "tkip.h"
#endif /* MAC_802_11I */

#include "spi_interface.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void tx_complete_isr(void);
static void rx_complete_isr(UWORD8 q_num);
static void tbtt_isr(void);
static void error_isr(void);
static void deauth_isr(void);
static void radar_detect_isr(void);

/*****************************************************************************/
/* Global Varible Declarations                                               */
/*****************************************************************************/

BOOL_T  g_rxq_cleanup_reqd[NUM_RX_Q] = {BFALSE, BFALSE};
UWORD8 g_last_rx_int_seqno = 0;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_isr                                                  */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the MAC H/w    */
/*                  interrupt.                                               */
/*                                                                           */
/*  Inputs        : 1) Interrupt vector                                      */
/*                  2) Data (unused)                                         */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the MAC H/w interrupt status and    */
/*                  mask registers to determine which interrupt has occurred */
/*                  and calls the required interrupt service routine. It     */
/*                  acknowledges the interrupt.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 mac_isr(UWORD32 vector, UWORD32 data)
{
    led_display(0x0F);

    if(get_machw_hprx_comp_int() == BTRUE)
    {
        rx_complete_isr(HIGH_PRI_RXQ);
        reset_machw_hprx_comp_int();
        g_test_stats.hprxci++;
    }
    else if(get_machw_rx_comp_int() == BTRUE)
    {
        rx_complete_isr(NORMAL_PRI_RXQ);
        reset_machw_rx_comp_int();
        g_test_stats.rxci++;
    }
    else if(get_machw_tx_comp_int() == BTRUE)
    {
        tx_complete_isr();
        reset_machw_tx_comp_int();
        g_test_stats.txci++;
    }
    else if(get_machw_tbtt_int() == BTRUE)
    {
        tbtt_isr();
        reset_machw_tbtt_int();
        g_test_stats.tbtti++;
    }
    else if(get_machw_error_int() == BTRUE)
    {
        error_isr();
        reset_machw_error_int();
        g_test_stats.erri++;
    }
    else if(get_machw_deauth_int() == BTRUE)
    {
        deauth_isr();
        reset_machw_deauth_int();
        g_test_stats.deauthi++;
    }
    else if(get_machw_txsus_int() == BTRUE)
    {
        reset_machw_txsus_int();
        g_test_stats.txsusi++;
    }
    else if(get_machw_radar_det_int() == BTRUE)
    {
        radar_detect_isr();
        reset_machw_radar_det_int();
        g_test_stats.radardeti++;
    }
    else if(get_machw_cap_end_int() == BTRUE)
    {
        reset_machw_cap_end_int();
        g_test_stats.capei++;
    }
    else
    {
        g_test_stats.uki++;
    }

    acknowledge_interrupt(vector);

    /* Kill time after interrupt is Acked */
    add_delay(10);

#ifndef PHY_TEST_CODE
    led_display(0xF0);
#endif /* PHY_TEST_CODE */

    return OS_ISR_HANDLED;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_complete_isr                                          */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the transmit   */
/*                  complete interrupt. This interrupt is raised when        */
/*                  1) a successful transmission occurs, 2) the retry limit  */
/*                  is reached or 3) packet lifetime expires.                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_stats                                             */
/*                  g_q_handle                                               */
/*                                                                           */
/*  Processing    : This function processes the Tx complete interrupt as     */
/*                  required in the unit test mode.                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void tx_complete_isr(void)
{
    UWORD8  q_num        = 0;
    UWORD16 i            = 0;
    UWORD16 num_dat_pkts = 0;
    UWORD8  *buff_hdl    = 0;
    UWORD8  *msa         = 0;
    UWORD16 num_tx_dscr  = get_machw_num_tx_frames();
    UWORD32 tx_dscr      = get_machw_tx_frame_pointer();

    /* Process the Tx complete interrupt for special test modes */
    process_txc_burst_tx_mode();

    /* Get the queue number from the descriptor and update test statistics */
    q_num = get_tx_dscr_q_num((UWORD32*)tx_dscr);
    g_test_stats.txd.txc[q_num]++;

    /* The given first transmit frame pointer should be at the head of the   */
    /* transmit queue. If it is not, re-synchronize by deleting all the      */
    /* packets till the given frame.                                         */
    qmu_sync_tx_packet(&g_q_handle.tx_handle, q_num, (UWORD8 *)tx_dscr);

    /* Read out the TSSI value from the first descriptor */
    update_tssi_stats((UWORD32 *)tx_dscr);

    /* Update the MIB statistics */
    update_tx_mib((UWORD8 *)tx_dscr, num_tx_dscr, NULL);

    /* Process and remove the given number of frames in order from the head  */
    /* of the transmit queue.                                                */
    for(i = 0; i < num_tx_dscr; i++)
    {
        UWORD8 dr = 0;

        /* Get the next Tx-dscr */
        tx_dscr = (UWORD32)get_txq_head_pointer(q_num);

        /* Get the transmit data rate and update the Tx data rate statistics */
        dr = get_tx_dscr_data_rate_0((UWORD32*)tx_dscr);
        update_mac_tx_stats(dr);

        msa = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)tx_dscr);

        if(get_type(msa) == DATA_BASICTYPE)
            num_dat_pkts++;

        /* Free the transmit buffer in frame-injector mode only. In other    */
        /* modes the same transmit buffer is reused.                         */
        if(g_test_config.oper_mode == 1)
        {
            buff_hdl = (UWORD8 *)get_tx_dscr_buffer_addr((UWORD32 *)tx_dscr);
            mem_free(g_shared_pkt_mem_handle, buff_hdl);
        }

        /* Delete the packet from the transmit queue */
        qmu_del_tx_packet(&g_q_handle.tx_handle, q_num);
    }

    g_test_stats.txd.data += num_dat_pkts;
    g_test_stats.txd.oth  += (num_tx_dscr - num_dat_pkts);

    /* Check if configured number of packets have been transmitted. If not   */
    /* add 1 more packet here.                                               */
    if(is_tx_test_complete() == BFALSE)
    {
        UWORD32  num_pkts_q  = 0;
        q_info_t *temp_qinfo = NULL;

        /* The number of frames to queue depends upon whether the test is */
        /* time or number bound.                                          */
        if(g_test_config.tx.test_time == 0)
        {
            num_pkts_q = g_num_frames_txqd - get_hut_num_tx_pkts();
        if(num_pkts_q > num_dat_pkts)
            num_pkts_q = num_dat_pkts;
        }
        else
            num_pkts_q = num_dat_pkts;

        for(i = 0; i < num_pkts_q; i++)
        {
            alloc_set_dscr();
            temp_qinfo = (q_info_t *)get_txc_qinfo_struct(q_num);
            queue_packet(temp_qinfo);
            g_num_frames_txqd++;
            check_and_send_ctrl_frames(temp_qinfo);
        }
    }
    else
    {
        /* The Tx Test has completed. Note down the end time and compute the */
        /* total time taken for transmission.                                */
        if(g_total_tx_time == 0)
        {
            UWORD32 tsf_hi = 0;
            UWORD32 tsf_lo = 0;

            get_machw_tsf_timer(&tsf_hi, &tsf_lo);

            g_total_tx_time = tsf_lo - g_tx_start_time_lo;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : rx_complete_isr                                          */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the receive    */
/*                  complete interrupt. This interrupt is raised when the    */
/*                  reception of a frame is complete.                        */
/*                                                                           */
/*  Inputs        : 1) Receive queue number                                  */
/*                                                                           */
/*  Globals       : g_test_stats                                             */
/*                  g_q_handle                                               */
/*                                                                           */
/*  Processing    : This function processes the Rx complete interrupt as     */
/*                  required in the unit test mode.                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void rx_complete_isr(UWORD8 q_num)
{
    CIPHER_T ct         = NO_ENCRYP;
    UWORD8  *msa        = 0;
    UWORD8  sa[6]       = {0};
    UWORD8  da[6]       = {0};
    UWORD8  status      = 0;
    UWORD8  datarate    = 0;
    BOOL_T  is_amsdu    = BFALSE;
    UWORD8  msdu_idx    = 0;
    UWORD8  dscr_idx    = 0;
    UWORD8  buf_idx     = 0;
    UWORD8  num_bufs    = 0;
    UWORD8  num_msdu    = 0;
    BOOL_T  is_defrag   = BFALSE;
    UWORD16 buf_ofst    = 0;
    UWORD16 pkt_len     = 0;
    UWORD16 frame_type  = 0;
    UWORD16 num_rx_dscr = 0;
    UWORD32 prm         = 0;
    UWORD32 *rx_dscr    = NULL;
    UWORD32 *first_dscr = NULL;
    WORD32  frm_rssi    = 0;
#ifdef ITTIAM_PHY
    UWORD32 settled_VGA      = 0;
    UWORD32 settled_LNA      = 0;
    UWORD32 settled_tmp      = 0;
#endif /* ITTIAM_PHY */

    /* Read the interrupt parameters corresponding to the queue for which */
    /* the interrupt is raised                                            */
    num_rx_dscr = get_machw_num_rx_frames(q_num);
    first_dscr  = (UWORD32 *)get_machw_rx_frame_pointer(q_num);

    /* Update the PHY data rate statistics */
    update_phy_dr_stats();

    /* Update RSSI information */
    frm_rssi = get_rx_dscr_rssi_db(first_dscr);
    if(frm_rssi != INVALID_RSSI_DB)
    {
        frm_rssi = frm_rssi << 24;
        frm_rssi = frm_rssi >> 24;

        g_test_stats.rxd.rssi_accum += frm_rssi;
        g_test_stats.rxd.rssi_pkt_count++;
    }

#ifdef ITTIAM_PHY
    read_dot11_phy_reg(rAGCSETTLEDVGAGAIN, &settled_VGA);
    read_dot11_phy_reg(rAGCSETTLEDLNAGAIN, &settled_LNA);

    g_test_stats.rxd.avg_lna += settled_LNA;
    g_test_stats.rxd.avg_vga += settled_VGA;
#endif /* ITTIAM_PHY */

#ifdef ITTIAM_PHY_DEBUG_RSSI
    if(g_test_stats.rxd.rssi_pkt_count >= 1024)
    {
        PRINTK("RSSI for %d pkts = %d\n", g_test_stats.rxd.rssi_pkt_count,
               (g_test_stats.rxd.rssi_accum >> 10));
        g_test_stats.rxd.rssi_accum = 0;
        g_test_stats.rxd.rssi_pkt_count = 0;
    }
#endif /* ITTIAM_PHY_DEBUG_RSSI */

#ifdef ITTIAM_PHY_PD_TEST
    /* Suspend MAC HW before disabling it */
    set_machw_tx_suspend();

    /* Disable MACHW & PHY before unlocking PHY SPI */
    disable_machw_phy_and_pa();

    /* Shut down PHY */
    write_dot11_phy_reg(rPHYPOWDNCNTRL, PHYPOWDNCNTRL & ~(BIT0 | BIT1));

    PRINTK("Putting PHY in shutdown mode\n");
#endif /* ITTIAM_PHY_PD_TEST */

    /* Perform Rx-Queue clean-up if required */
    if(g_rxq_cleanup_reqd[q_num] == BTRUE)
    {
        qmu_sync_rx_packet(&g_q_handle.rx_handle, q_num, first_dscr);
        g_rxq_cleanup_reqd[q_num] = BFALSE;
    }

    /* Update the receive queue with the read out frames */
    qmu_update_rx_q(&g_q_handle.rx_handle, q_num, first_dscr, num_rx_dscr);

    /* Compute the average signal quality */
    compute_sig_qual_avg(first_dscr);

    /* Update RSSI */
    update_rssi(first_dscr);

    /* Outer loop: Runs on all MSDUs within an A-MPDU */
    /* Step-1: Identify the start and end of the MSDU */
    while(dscr_idx < num_rx_dscr)
    {
        /* Extract various parameters from the descriptor */
        msa        = (UWORD8*)get_rx_dscr_buffer_ptr(first_dscr);
        pkt_len    = get_rx_dscr_frame_len(first_dscr);
        status     = get_rx_dscr_pkt_status(first_dscr);
        datarate   = get_rx_dscr_data_rate(first_dscr);
        prm        = get_rx_dscr_prm(first_dscr);
        is_defrag  = is_rx_frame_frag(first_dscr);
        is_amsdu   = is_rx_frame_amsdu(first_dscr);
        num_bufs   = get_rx_dscr_num_buffers(first_dscr);

        /* This should be the first buffer of an MSDU. Its a sync loss */
        /* condition otherwise.                                        */
        if((is_amsdu == BTRUE) && (is_rx_frame_start_buf(first_dscr) != BTRUE))
        {
            /* Free rest of the frames in the list on sync loss */
            free_rx_dscr_list(first_dscr, num_rx_dscr - dscr_idx);
            g_test_stats.rxd.not_first_dscr++;

            break;
        }

        if(get_wep(msa))
            ct = (CIPHER_T)get_rx_dscr_cipher_type(first_dscr);
        else
            ct = NO_ENCRYP;

        /* Extract various parameters from the MAC header */
        frame_type = get_sub_type(msa);

        get_address1(msa, da);
        get_address2(msa, sa);

        /* Perform sanity check on the Rx-Interrupt sequence number */
        int_seqno_dup_chk(first_dscr);

        /* Update the various MAC statistics */
        update_mac_rx_stats(msa, status, ct, datarate, frame_type, is_amsdu,
                            is_defrag);

        /* Update the frame statistics */
        update_frm_rx_stats(msa, ct, datarate, status, pkt_len, prm, is_amsdu);

       /* Update AMPDU related statistics */
        if(BTRUE == is_ampdu(first_dscr))
        {
            g_test_stats.rxd.mpdus_in_ampdu++;
            g_test_stats.rxd.bytes_in_ampdu += pkt_len;

            if(dscr_idx == 0)
                g_test_stats.rxd.ampdu++;
        }

        /* Perform duplicate check in S/w */
        soft_dup_check(msa, sa);

        /* TBD: This function will work only for non-AMSDU frames. */
        /* Need to check whether this is also required for A-MSDU. */
        if(is_amsdu == BFALSE)
            soft_crc_check(msa, pkt_len);

        rx_dscr  = first_dscr;

        /* Perfrom Security and Integrity checks only on Data frames which */
        /* are transmitted from the intended source. The TA of the frames  */
        /* should match the address programmed in g_test_config.tx.da.     */
        if(BTRUE == filter_test_frame(msa))
        {
            UWORD8 frame_check_fail = 0;

        /* Perform TKIP integrity chack on the receive frames.           */
        perform_security_check(msa, pkt_len, status, ct);


        buf_ofst = get_mac_hdr_len(msa) + get_sec_header_len(ct);
#ifdef MAC_WMM
        buf_ofst += 2;
#endif /* MAC_WMM */

        /* Perform Integrity check on each of the sub-MSDUs */
        for(buf_idx = 0; buf_idx < num_bufs; buf_idx++)
        {
                /* Exception case: Rx Dscr becomes NULL before number of buffers */
                /* (num_bufs) as given by MAC H/w are processed.                 */
                if(buf_idx > 0)
                {
                    if(rx_dscr == NULL)
                    {
                        g_test_stats.rxd.unexp_rx_bufend++;
                        break;
                    }

                    msa = (UWORD8 *)get_rx_dscr_buffer_ptr(rx_dscr);
                    buf_ofst = 0;
                }

                num_msdu = (is_amsdu == BTRUE)? get_rx_dscr_num_msdus(rx_dscr):1;
            for(msdu_idx = 0; msdu_idx < num_msdu; msdu_idx++)
            {
                if(is_amsdu == BTRUE)
                {
                    /* For Sub-MSDUs account for the Filler Bytes inserted by*/
                    /* H/w to fit the host header                            */
                    buf_ofst += HOST_HEADER_GAP;
                    pkt_len   = get_submsdu_len(msa + buf_ofst);
                    buf_ofst += SUB_MSDU_HEADER_LENGTH;

                    g_test_stats.rxd.bytes_in_amsdu += pkt_len;
                    g_test_stats.rxd.smsdus_in_amsdu++;
                }
                else
                {
                    /* Calculate MSDU-Body length for normal MSDUs */
                    pkt_len -= (buf_ofst + FCS_LEN + get_sec_trailer_len(ct));
#ifdef MAC_WMM
                    /* In case of 11e, buf_ofst is 2 bytes more than actual */
                    /* due to the alignment filler bytes.                   */
                    pkt_len += 2;
#endif /* MAC_WMM */
                }

                /* Exception case: Buffer offset with Packet length exceeds  */
                /* Rx buffer size. This can happen if there is some error in */
                /* this processing or if number of MSDUs indicated by MAC    */
                /* H/w (num_msdu) is incorrect.                              */
                if(buf_ofst + pkt_len > RX_BUFFER_SIZE)
                {
                    g_test_stats.rxd.rx_bufsz_exceeded++;
                    break;
                }

                    if(BFALSE == check_frame_integrity(msa + buf_ofst, pkt_len))
                    {
                        frame_check_fail = 1;
                    }

                if(is_amsdu == BTRUE)
                {
                    /* Update the offsets for the padding bytes after each sub-MSDU */
                    buf_ofst += pkt_len;
                    buf_ofst += get_submsdu_pad_len(pkt_len + SUB_MSDU_HEADER_LENGTH);
                }
            }

            rx_dscr = (UWORD32 *)get_rx_dscr_next_dscr(rx_dscr);
            }

            if(frame_check_fail == 1)
            {
                if(is_amsdu == BTRUE)
                    g_test_stats.rxd.amsdu_soft_intchkf++;
                else
                    g_test_stats.rxd.soft_intchkf++;
            }
        }
        else
        {
            /* Move to the last descriptor of this MPDU */
            for(buf_idx = 0; buf_idx < num_bufs; buf_idx++)
                rx_dscr = (UWORD32 *)get_rx_dscr_next_dscr(rx_dscr);
        }

        /* Free the receive buffers */
        free_rx_dscr_list(first_dscr, num_bufs);
        dscr_idx += num_bufs;
        first_dscr = rx_dscr;
    }

    /* Replenish the receive queue with required number of buffers */
    replenish_test_rx_queue(q_num);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tbtt_isr                                                 */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the TBTT       */
/*                  interrupt. This interrupt is raised at every TBTT.       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_stats                                             */
/*                                                                           */
/*  Processing    : This function processes the TBTT interrupt as required   */
/*                  in the unit test mode.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void tbtt_isr(void)
{
    /* Swap the Beacons if beacon swap test is enabled */
    if(g_test_config.tsf.bcn_swap_test == 1)
    {
        UWORD32 temp = 0;

        temp = get_machw_beacon_pointer();
        set_machw_beacon_pointer(g_test_config.tsf.next_bcn_addr);
        set_machw_beacon_tx_params(g_test_config.tsf.bcn_len,
                                   get_phy_rate(g_test_config.tsf.bcn_rate));
        set_machw_beacon_ptm(get_reg_phy_tx_mode(g_test_config.tsf.bcn_rate, 1));

        g_test_config.tsf.next_bcn_addr = temp;
    }

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : deauth_isr                                               */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the Deauth     */
/*                  interrupt.                                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function gets the Deauth reason code and saves it.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void deauth_isr(void)
{
    UWORD8 i = g_test_stats.deauthi;

    /* Only the last 10 deauthentication reason codes can be saved */
    i = (i % 10);

    /* Reac MAC H/w register to get the deauthentication reason code */
    g_test_stats.exp.deauth_rc[i] = get_machw_deauth_reason_code();

    PRINTD("Deauth_isr:RC=%x\n\r",g_test_stats.exp.deauth_rc[i]);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : error_isr                                                */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the ERROR      */
/*                  interrupt. This interrupt is raised at every ERROR.      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function creates a MISC_ERROR event with no message.*/
/*                  This event is posted to the event queue.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void error_isr(void)
{
    UWORD32 err_stat = 0;
    UWORD8  error_code   = 0;

    /* Schedule Clean-up of the receive queues on receiving an Error Interrupt */
    g_rxq_cleanup_reqd[NORMAL_PRI_RXQ] = BTRUE;
    g_rxq_cleanup_reqd[HIGH_PRI_RXQ]   = BTRUE;

#ifdef OLD_MACHW_ERROR_INT
    error_code   = get_machw_error_code();
    err_stat     = (1 << error_code);
#else /* OLD_MACHW_ERROR_INT */
    //chenq mod
    //err_stat = convert_to_le(rMAC_ERROR_STAT);
    err_stat = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_ERROR_STAT));
#endif /* OLD_MACHW_ERROR_INT */

    if(err_stat & (1 << UNEXPECTED_RX_Q_EMPTY))
    {
        replenish_test_rx_queue(NORMAL_PRI_RXQ);
        g_test_stats.exp.erxqemp++;
        error_code   = UNEXPECTED_RX_Q_EMPTY;
    }
    else if(err_stat & (1 << UNEXPECTED_HIRX_Q_EMPTY))
    {
        replenish_test_rx_queue(HIGH_PRI_RXQ);
        g_test_stats.exp.ehprxqemp++;
        error_code   = UNEXPECTED_HIRX_Q_EMPTY;
    }
    else if(err_stat & (1 << TX_INTR_FIFO_OVERRUN))
    {
        g_test_stats.exp.etxfifo++;
        error_code   = TX_INTR_FIFO_OVERRUN;
    }
    else if(err_stat & (1 << RX_INTR_FIFO_OVERRUN))
    {
        g_test_stats.exp.erxfifo++;

        error_code   = RX_INTR_FIFO_OVERRUN;
    }
    else if(err_stat & (1 << HIRX_INTR_FIFO_OVERRUN))
    {
        g_test_stats.exp.ehprxfifo++;

        error_code   = HIRX_INTR_FIFO_OVERRUN;
    }
    else if(err_stat & (1 << RX_PATH_WDT_TO_FCS_FAILED))
        {
        g_test_stats.exp.ewdtofcsfailed++;
        error_code   = RX_PATH_WDT_TO_FCS_FAILED;
            }
    else if(err_stat & (1 << RX_PATH_WDT_TO_FCS_PASSED))
            {
        g_test_stats.exp.ewdtofcspassed++;
        error_code   = RX_PATH_WDT_TO_FCS_PASSED;
    }
    else if(err_stat & (1 << UNEXPECTED_MAC_TX_HANG))
    {
        g_test_stats.exp.etxsus1machang++;
        error_code   = UNEXPECTED_MAC_TX_HANG;
    }
    else if(err_stat & (1 << UNEXPECTED_PHY_TX_HANG))
    {
        g_test_stats.exp.etxsus1phyhang++;
        error_code   = UNEXPECTED_PHY_TX_HANG;
    }
    else if(err_stat & (1 << UNEXPECTED_TX_STATUS3))
    {
        g_test_stats.exp.etxsus3++;
        error_code   = UNEXPECTED_TX_STATUS3;
    }
    else if(err_stat & (1 << UNEXPECTED_TX_Q_EMPTY))
    {
        g_test_stats.exp.etxqempt++;
        error_code   = UNEXPECTED_TX_Q_EMPTY;
    }
    else if(err_stat & (1 << BUS_ERROR))
    {
        g_test_stats.exp.ebus++;
        error_code   = BUS_ERROR;
    }
    else if(err_stat & (1 << UNEXPECTED_MSDU_ADDR))
    {
        g_test_stats.exp.emsaddr++;
        error_code   = UNEXPECTED_MSDU_ADDR;
    }
    else if(err_stat & (1 << BUS_WRAP_SIG_ERROR))
    {
        g_test_stats.exp.ebwrsig++;
        error_code   = BUS_WRAP_SIG_ERROR;
    }
    else if(err_stat & (1 << DMA_NO_ERROR))
    {
        g_test_stats.exp.edmanoerr++;
        error_code   = DMA_NO_ERROR;
    }
    else if(err_stat & (1 << TX_CE_NOT_READY))
    {
        g_test_stats.exp.etxcenr++;
        error_code   = TX_CE_NOT_READY;
    }
    else if(err_stat & (1 << RX_CE_NOT_READY))
    {
        g_test_stats.exp.erxcenr++;
        error_code   = RX_CE_NOT_READY;
    }
    else if(err_stat & (1 << SEQNUM_GEN_ADDR_FAIL))
    {
        g_test_stats.exp.esgaf++;
        error_code   = SEQNUM_GEN_ADDR_FAIL;
    }
    else if(err_stat & (1 << UNEXPECTED_NON_PENDING_MSDU))
    {
        UWORD8  i     = 0;
        UWORD16 num_q = get_no_tx_queues();

        for(i = 0; i < num_q; i++)
            {
            /* Reset the transmit queue cache headers for all the queues and */
            /* the last element pointers for transmit queue cache for all    */
            /* queues.                                                       */
            set_machw_q_ptr((UWORD8)i, NULL);
        }

        error_code   = UNEXPECTED_NON_PENDING_MSDU;
        g_test_stats.exp.enpmsdu++;
    }
    else
    {
#ifndef OLD_MACHW_ERROR_INT
         /* Reset UnAcked Error Interrupt */
        //chenq mod
        //rMAC_ERROR_STAT = rMAC_ERROR_STAT;
        host_write_trout_reg(
            host_read_trout_reg( (UWORD32)rMAC_ERROR_STAT ),
            (UWORD32)rMAC_ERROR_STAT );
#endif /* OLD_MACHW_ERROR_INT */
        g_test_stats.exp.eother++;
    }

#ifndef OLD_MACHW_ERROR_INT
    /* Acknowledge the Error Interrupt */
    reset_machw_error_int_code(error_code);
#endif /* OLD_MACHW_ERROR_INT */

    PRINTD("error_isr: error_code = %d\n\r",error_code);
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : radar_detect_isr                                         */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the radar      */
/*                  detect interrupt.                                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       :                                                          */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void radar_detect_isr(void)
{
    disable_machw_phy_and_pa();

    /* Clear the radar status bit */
    clear_phy_radar_status_bit();

    enable_machw_phy_and_pa();
}

#endif /* MAC_HW_UNIT_TEST_MODE */

