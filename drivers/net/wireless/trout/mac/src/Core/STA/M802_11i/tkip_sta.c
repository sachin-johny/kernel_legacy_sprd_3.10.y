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
/*  File Name         : tkip_sta.c                                           */
/*                                                                           */
/*  Description       : This file contains the MAC S/W TKIP related          */
/*                      functions                                            */
/*                                                                           */
/*  List of Functions : tkip_cntr_msr_supp                                   */
/*                      send_mic_error_report                                */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "tkip_sta.h"
#include "maccontroller.h"
#include "iconfig.h"
#include "mac_init.h"
#include "controller_mode_if.h"
#include "rsna_supp_km.h"
#include "qmu_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name :  tkip_cntr_msr_supp                                      */
/*                                                                           */
/*  Description   :  This function implements the TKIP counter measures      */
/*                                                                           */
/*  Inputs        :  None                                                    */
/*                                                                           */
/*  Globals       :  None                                                    */
/*                                                                           */
/*  Processing    :  The Authenticator MAC deauthenticates all the RSNAs     */
/*                   and suspends its activity for a period of sixty seconds */
/*                   after which the settings are reloaded and the MAC is    */
/*                   reinitialized                                           */
/*                                                                           */
/*  Outputs       :  None                                                    */
/*                                                                           */
/*  Returns       :  None                                                    */
/*                                                                           */
/*  Issues        :  None                                                    */
/*                                                                           */
/*****************************************************************************/

void tkip_cntr_msr_supp(mac_struct_t *mac)
{
    UWORD8 i                     = 0;
    rsna_supp_persta_t *rsna_ptr = NULL;

    /* Counter measure requires the MAC to De-auth with all the RSNAs and */
    /* stay in cntr meaure state for a period of 60 seconds               */
    /*                                                                    */
    /* *. Suspend MAC TX                                                  */
    /* *. Flush MISC event queues with event type TX Complete             */
    /* *. Flush all the queues                                            */
    /* *. Resume MAC TX                                                   */
    /* *. Send deauth to all the STAs - with reason MIC Failure           */
    /* *. start a 60 second timer                                         */
    /* *. Set the TKIP Cntr Measure Flag                                  */

    /* Suspend the MAC HW For transmission                                   */
    /* It is assumed that after the suspension, MAC H/W Will not give any TX */
    /* complete INT                                                          */
    set_machw_tx_suspend();

    /* Flush all the QMU - queues */
    flush_all_qs();

    /* Flush all MISC event queue with event type - Tx-complete */
    event_q_remove_all_misc_events(&g_event_q[MISC_EVENT_QID], MISC_TX_COMP);

    /* Resume MAC HW Tx */
    set_machw_tx_resume();

    /* Load the RSNA Ptr to create the De-Auth message                    */
    for(i = 0 ; i < NUM_SUPP_STA; i++)
    {
        rsna_ptr = g_rsna_supp.rsna_ptr[i];

        /* If RSNA Entry is valid */
        if(rsna_ptr != NULL)
        {
            /* Deauthenticate for the BSS STA */
            if(mget_DesiredBSSType() == INFRASTRUCTURE)
            {
                UWORD8      bssid[6]      = {0};
                
                TROUT_DBG4("P2P: deauth. MIC_FAIL");
                /* Set the Deauthication reason to Unspecified reason */
                rsna_ptr->deauth_reason = MIC_FAIL;

                /* Send the Deauthentication frame */
                send_deauth_frame(rsna_ptr->addr, rsna_ptr->deauth_reason);

                /* Remove the entry for the sta                             */
                /* This does the RSNA Cleanup along with that of Asoc table */
                delete_entry(rsna_ptr->addr);

                /* Reset BSS ID of Station */
                mset_bssid(bssid);

                /* Change state of the MAC structure to DISABLED */
                set_mac_state(DISABLED);

                /* Convey the current MAC status to Host */
				//chenq mask 2012-10-18
                //send_mac_status(MAC_DISCONNECTED);

                /* Stop the Autorate timer as it is going to be started again */
                /* when STA associates with AP after the cntr msr period      */
                stop_ar_timer();

                /* Stop the activity (power save) timer */
                stop_activity_timer();

                break;
            }
            else
            {
                /* IntegrateTBD: For IBSS Mode */
            }
        }
    }

    /* Wait on the transmission of the management frames */
    {
        UWORD8 q_num = HIGH_PRI_Q;
        while(is_machw_q_null(q_num) == BFALSE)
        {
            add_delay(0xFFF);
        }
    }

    stop_sec_km();

    enable_security();

    /* Set the Cntr Measure Flag */
    g_rsna_supp.global.cntr_msr_in_progress = BTRUE;

    if(BFALSE == start_supp_cntr_msr_60s_timer())
    {
        /* If timer could not be started for 60 secs */
        /* Reset the status immediately */
        stop_supp_cntr_msr_timer();
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name :  send_mic_error_report                                   */
/*                                                                           */
/*  Description   :  This function sends the Michael MIC Error Report Frame  */
/*                                                                           */
/*  Inputs        :  1) pointer to the RSNA Handle                           */
/*                                                                           */
/*  Globals       :  None                                                    */
/*                                                                           */
/*  Processing    :  This function sends the Michael MIC Error Report Frame  */
/*                                                                           */
/*  Outputs       :  None                                                    */
/*                                                                           */
/*  Returns       :  None                                                    */
/*                                                                           */
/*  Issues        :  None                                                    */
/*                                                                           */
/*****************************************************************************/

void send_mic_error_report(rsna_supp_persta_t *rsna_ptr, BOOL_T is_bcast_error)
{
    UWORD8 *buffer  = 0;

    /* Fetch memory for the EAPOL Packet to be sent to the Remote Sta */
    buffer = (UWORD8*)pkt_mem_alloc(MEM_PRI_TX);

    /* If general buffers are not available then take from management buffers */
    if(buffer == NULL)
    {
        buffer = mem_alloc(g_shared_pkt_mem_handle, EAPOL_PKT_MAX_SIZE);
    }

    if(buffer != NULL)
    {
        UWORD16 key_info      = 0;
        UWORD8  *eapol_buffer = 0;
        UWORD16 eapol_length  = 0;
        UWORD8  *replay_cnt   = 0;

        /* Send EAPOL(0, 0, 1, 0, P, 0, ANonce, 0, 0, 0) */

        /* reset the buffer */
        mem_set(buffer, 0, EAPOL_PKT_MAX_SIZE);

        eapol_buffer = buffer + MAX_MAC_HDR_LEN + EAPOL_1X_HDR_LEN;

        if(is_bcast_error == BFALSE)
        {
        /* Prepare the Key Information field with the required parameters.*/
        key_info  = prepare_key_info(rsna_ptr->key_version, 1, 1, 0, 0, P,
                                     0, 0, rsna_ptr->mode_802_11i);
        }
        else
        {
            /* Prepare the Key Information field with the required parameters.*/
            key_info  = prepare_key_info(rsna_ptr->key_version, 1, 1, 0, 0, G,
                                         0, 0, rsna_ptr->mode_802_11i);
        }

        key_info |= 0x0F00;

        /* Get the replay count */
        incr_rply_cnt(rsna_ptr->ReplayCount);

        replay_cnt = rsna_ptr->ReplayCount;

        /* Prepare the EAPOL Key frame */
        eapol_length = prepare_eapol_key_frame(key_info, 0, NULL,
                                               0, 0, 0, 0, replay_cnt,
                                               eapol_buffer,
                                               rsna_ptr->mode_802_11i);

        prepare_1x_hdr(buffer + MAX_MAC_HDR_LEN, EAPOL_KEY, eapol_length,
                       rsna_ptr->mode_802_11i);

        eapol_length += EAPOL_1X_HDR_LEN;

#ifdef UTILS_11I
        /* Calculate MIC over the body of the EAPOL-Key frame with Key MIC  */
        /* field zeroed. The MIC is computed using the KCK derived from PTK.*/
        add_eapol_mic(eapol_buffer, KEY_DATA_OFFSET, rsna_ptr->PTK,
                rsna_ptr->key_version);
#endif /* UTILS_11I */

        /* Schedule the EAPOL frame for transmission */
        send_eapol(rsna_ptr->addr, buffer, eapol_length, BTRUE);

        if(is_bcast_error == BFALSE)
        {
			PRINTD2("P-MIC Error Report Sent\n");
		}
		else
		{
			PRINTD2("G-MIC Error Report Sent\n");
		}
    }
    else
    {
		PRINTD2("No Mem to send MIC Error Report\n");
    }
}

#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
