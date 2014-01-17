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
/*  File Name         : wlan_misc_ap.c                                       */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the AP MAC FSM on receiving      */
/*                      MISC events.                                         */
/*                                                                           */
/*  List of Functions : ap_wait_start_misc                                   */
/*                      ap_enabled_misc                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "ap_prot_if.h"
#include "autorate.h"
#include "controller_mode_if.h"
#include "mac_init.h"
#include "pm_ap.h"
#include "qmu_if.h"
#include "metrics.h"
#include "iconfig.h"

/*****************************************************************************/
/* Static Function Declaration                                               */
/*****************************************************************************/

void process_tx_comp_ap(UWORD8 q_num, UWORD8 num_dscr);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_wait_start_misc                                       */
/*                                                                           */
/*  Description   : This function handles the hardware input as appropriate  */
/*                  in the WAIT_START state.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle hardware input */
/*                  and state change may happen based on the function output.*/
/*                                                                           */
/*  Outputs       : MAC state of the input MAC library structure may change  */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_wait_start_misc(mac_struct_t *mac, UWORD8 *msg)
{
    start_rsp_t      start_rsp       = {0};
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
    case MISC_TBTT:
    {
        /* Reception of TBTT indicates that Start was successful. Cancel the */
        /* Start timeout timer and prepare a start response message with     */
        /* SUCCESS status and send it to the host.                           */
        cancel_mgmt_timeout_timer();

        start_rsp.result_code = SUCCESS_MLMESTATUS;

        /* The MAC state is changed to Enabled */
        set_mac_state(ENABLED);

        /* Convey the current MAC status to Host */
		//chenq mask 0727
        //send_mac_status(MAC_CONNECTED);

        /* For 11g mode if auto protection mode is enabled the MAC Hardware  */
        /* Beacon Filter is disabled so that overlapping OLBC are detected.  */
        if((get_phy_type() != PHY_B_DSSS) && (get_phy_type() != PHY_A_OFDM))
        {
            if(is_autoprot_enabled() == BTRUE)
                disable_machw_beacon_filter();
        }

        /* Reset the BSS Type to Infrastructure for Access Point */
        mset_DesiredBSSType(INFRASTRUCTURE);

        /* Create the alarm for checking the inactive STA */
        g_aging_timer = create_alarm(ap_aging_timeout_fn, 0, NULL);

        /* Start the checking timer again */
        start_alarm(g_aging_timer, 1000 * AGING_TIMER_PERIOD_IN_SEC);

        /* Start the autorate alarm */
        start_ar_timer();

        /* Send the response to host now.*/
        send_mlme_rsp_to_host(mac, MLME_START_RSP, (UWORD8 *)(&start_rsp));

        /* Start the CPU utiltization test */
        start_cpu_util_test();

        /* ITM_DEBUG */
        PRINTD("Status: Start Successful. Entering Enable State. \n");
    }
    break;

    case MISC_TIMEOUT:
    {
        /* Timeout indicates that Start was not successful. Prepare a start  */
        /* response message with TIMEOUT status and send it to the host.     */
        start_rsp.result_code = TIMEOUT;

        /* The MAC state is changed to Disabled state. Further MLME requests */
        /* are processed in this state.                                      */
        set_mac_state(DISABLED);

        /* Send the response to host now. */
        send_mlme_rsp_to_host(mac, MLME_START_RSP, (UWORD8 *)(&start_rsp));

        /* ITM_DEBUG */
        PRINTD("Status: Start Timed-out. Restarting. \n");
    }
    break;

    case MISC_ERROR:
    {
        raise_system_error(SYSTEM_ERROR);
    }
    break;

    case MISC_CH_AVAIL_CHECK_TIMEOUT:
    {
        handle_chnl_av_chk_timeout_event();
    }
    break;

    default:
    {
        /* Process the event based on protocol defined */
        wait_start_misc_event_prot_ap(mac, msg);
    }
    break;

    } /* end of switch(msg_name) */
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : ap_enabled_misc                                          */
/*                                                                           */
/*  Description   : This function handles hardware input in ENABLED state.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle hardware input */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void ap_enabled_misc(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;
     UWORD32 *trout_beacon_buf = 0, beacon_len = 0;

    switch(misc_event_msg->name)
    {
    case MISC_TX_COMP:
    {
        UWORD8 num_dscr = misc_event_msg->info;
        UWORD8 *base_dscr = misc_event_msg->data;
        UWORD8 *dscr      = 0;
        UWORD8 q_num      = get_tx_dscr_q_num((UWORD32*)base_dscr);
        BOOL_T sync_found = BFALSE;


        /* The given first transmit frame pointer should be at the head of   */
        /* the transmit queue. If it is not, re-synchronize by first         */
        /* processing all the packets till the given frame.                  */
        while(sync_found == BFALSE)
        {
            dscr = get_txq_head_pointer(q_num);

            if(dscr == NULL)
                return;

            if(base_dscr == dscr)
            {
                process_tx_comp_ap(q_num, num_dscr);
                sync_found = BTRUE;
            }
            else
            {
                process_tx_comp_ap(q_num, 1);
            }
        }
    }
    break;

    case MISC_TBTT:
    {
#if 0
        UWORD8  dtim_count  = 0;
        UWORD8  old_bcn_idx = 0;
        UWORD16 i           = 0;
		// 20120709 caisf add, merged ittiam mac v1.2 code
        process_tbtt_ap_prot();

        /* Get the index which is currently being used by H/w */
        old_bcn_idx = (g_beacon_index + 1) % 2;

        /* Read the current DTIM count from H/w */
        dtim_count = get_machw_dtim_count();

        /* If the beacon pointer has been updated to a new value, the free   */
        /* beacon buffer index is updated to the other buffer.               */
        if(dtim_count == 0)
        {
            dtim_count = mget_DTIMPeriod() - 1;

            /* The beacon transmitted at this TBTT is the DTIM. Requeue all  */
            /* MC/BC packets to the high priority queue.                     */
            /* Check if the beacon that is being transmitted has the MC bit  */
            /* set                                                           */
            if(BTRUE == get_mc_bit_bcn(old_bcn_idx))
                while(requeue_ps_packet(NULL, &g_mc_q, BTRUE, BFALSE) == PKT_REQUEUED);
        }
        else
        {
            /* Do nothing */
            dtim_count--;
        }

        g_vbmap[DTIM_CNT_OFFSET] = dtim_count;

        for(i = 0; i < g_vbmap[LENGTH_OFFSET] + 2; i++)
        {
            g_beacon_frame[g_beacon_index][g_tim_element_index + i + SPI_SDIO_WRITE_RAM_CMD_WIDTH] =
                                                                    g_vbmap[i];
        }
        g_beacon_len = g_tim_element_index + g_tim_element_trailer_len +
                                          g_vbmap[LENGTH_OFFSET] + 2 + FCS_LEN;

        /* The status of BC/MC packets queued for PS should be updated only  */
        /* in DTIM beacon dtim_count==0. For the rest of the becons reset    */
        /* the BC/MC bit in TIM                                              */
        if(dtim_count != 0)
            reset_mc_bit_bcn(g_beacon_index);
		
		//add by chengwg.
		beacon_len = g_beacon_len;
		if(g_beacon_len % 4 != 0)
			beacon_len += 4 - (g_beacon_len % 4);
		if(beacon_len > BEACON_MEM_SIZE)
    	{
			printk("%s: beacon mem is too small(%d : %d)!\n", __func__, beacon_len, BEACON_MEM_SIZE);
			return;
    	}	
		trout_beacon_buf = (UWORD32 *)BEACON_MEM_BEGIN;
		host_write_trout_ram((void *)trout_beacon_buf, g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH, beacon_len);
		
        /* Swap the beacon buffer used by MAC H/w */
        set_machw_beacon_pointer((UWORD32)trout_beacon_buf);
        
        set_machw_beacon_tx_params(g_beacon_len, get_phy_rate(get_beacon_tx_rate()));
        set_machw_beacon_ptm(get_reg_phy_tx_mode(get_beacon_tx_rate(), 1));

		// 20120709 caisf add, merged ittiam mac v1.2 code
		critical_section_start();
		if(dtim_count == 0)
			g_update_active_bcn = 1;
		critical_section_end();

        /* Copy the contents from the current beacon to the old beacon */
        for(i = 0; i < g_beacon_len; i++)
            g_beacon_frame[old_bcn_idx][i+SPI_SDIO_WRITE_RAM_CMD_WIDTH] = 
            		g_beacon_frame[g_beacon_index][i+SPI_SDIO_WRITE_RAM_CMD_WIDTH];

        /* S/w will now make updates if required in the old beacon */
        g_beacon_index = old_bcn_idx;

        /* Channel management related TBTT tasks */
        handle_tbtt_chan_mgmt_ap(dtim_count);
        handle_tbtt_prot_ap();
#endif
    }
    break;

    case MISC_ERROR:
    {
        raise_system_error(SYSTEM_ERROR);
    }
    break;

    case MISC_CH_AVAIL_CHECK_TIMEOUT:
    {
        handle_chnl_av_chk_timeout_event();
    }
    break;


    case MISC_RUN_AGING_FN:
    {
            ap_aging_fn();
    }
    break;

    case MISC_TIMEOUT:
    {
        handle_ap_enabled_misc_to_prot(mac);
    }
    break;

    default:
    {
        ap_enabled_misc_prot(mac, msg);
    }
    break;

    } /* end of switch(msg_name) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_tx_comp_ap                                       */
/*                                                                           */
/*  Description   : This function processes frames from the TX-Q after TX    */
/*                  Complete Interrupt is raised.                            */
/*                                                                           */
/*  Inputs        : 1) Current MAC State                                     */
/*                  2) Q-Number of the Queue to be processed                 */
/*                  3) Number of Descriptors to be processed                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : TX Complete interrupt is processed by reading out the    */
/*                  descriptors from the TX-Q one at a time.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_tx_comp_ap(UWORD8 q_num, UWORD8 num_dscr)
{
    UWORD8 i      = 0;
    UWORD8 *dscr  = 0;
    UWORD8 ra[6]  = {0};
    UWORD8 *msa   = NULL;
    void   *entry = NULL;

    /* Process and remove the given number of frames in order from the   */
    /* head of the transmit queue.                                       */
    for(i = 0; i < num_dscr; i++)
    {
        /* Get the transmit queue head pointer */
        dscr = get_txq_head_pointer(q_num);

        /* If the queue has become empty, this is an exception. Return */
        if(dscr == NULL)
            return;

		// 20120709 caisf add, merged ittiam mac v1.2 code
       /* Extract MSA */
        msa = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);

        if(i == 0)
        {
#ifdef ENABLE_MACHW_KLUDGE
            /* Workaround for MAC H/w issue which can cause the num_dscr     */
            /* count to go wrong when TX of a Normal-MPDU frame following an */
            /* AMPDU fails.                                                  */
            if((num_dscr > 1) && (is_tx_success((UWORD32 *)dscr) == BFALSE))
            {
                if((get_tx_dscr_short_retry_count((UWORD32 *)dscr) >= mget_ShortRetryLimit())
                   && (get_tx_dscr_cts_failure((UWORD32 *)dscr) >= mget_ShortRetryLimit()))
                {
                    num_dscr = 1;
                }
            }
#endif /* ENABLE_MACHW_KLUDGE */

            /* The Station entry is extracted for the transmitted frame. */
            /* Since all MPDUs of an AMPDU have the same RA, this can be */
            /* done for only once.                                       */
            //msa = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);// 20120709 caisf masked, merged ittiam mac v1.2 code
            get_address1(msa, ra);
            entry = find_entry(ra);

            /* Update the MIB counters for this MPDU */
            update_tx_mib(dscr, num_dscr, entry);

            /* Update required MIB for the aggregation of frames transmitted */
            update_aggr_tx_mib(num_dscr);
        }

        print_log_debug_level_1("\n[DL1][INFO][Tx] {Transmit complete event}");

        if(get_mac_state() == ENABLED)
        {
			// 20120709 caisf masked, merged ittiam mac v1.2 code
			#if 0
            /* Update protocol table at tx complete */
            handle_prot_tx_comp_ap(dscr, entry);

            /* Power Management */
            handle_ps_tx_comp_ap(dscr, entry);

            /* Management related tasks on Tx-CI */
            update_asoc_entry_txci(dscr, entry);
			#else
            /* Update protocol table at tx complete */
            handle_prot_tx_comp_ap(dscr, entry, msa);

            /* Power Management */
            handle_ps_tx_comp_ap(dscr, entry, msa);

            /* Management related tasks on Tx-CI */
            update_asoc_entry_txci(dscr, entry, msa);
			#endif
        }

        /* Call the latency test function */
        latency_test_function(dscr);
#ifdef MEASURE_PROCESSING_DELAY
        g_delay_stats.numsubmsduontxcmp += get_tx_dscr_num_submsdu((UWORD32*)dscr);

        g_delay_stats.numtxdscrontxcmp++;
#endif /* MEASURE_PROCESSING_DELAY */
        /* Delete the packet from the head of the transmit queue */
        if(qmu_del_tx_packet(&g_q_handle.tx_handle, q_num) != QMU_OK)
        {
            print_log_debug_level_1("\n[DL1][EXC][Tx] {Packet not deleted from queue}");
        }
        else
        {
            print_log_debug_level_1("\n[DL1][INFO][Tx] {Packet deleted from queue}");
        }
    }
}

#endif /* BSS_ACCESS_POINT_MODE */
