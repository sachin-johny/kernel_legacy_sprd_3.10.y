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
/*  File Name         : tkip_auth.c                                          */
/*                                                                           */
/*  Description       : This file contains the MAC S/W TKIP related          */
/*                      functions                                            */
/*                                                                           */
/*  List of Functions : tkip_cntr_msr_auth                                   */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "tkip_auth.h"
#include "maccontroller.h"
#include "iconfig.h"
#include "mac_init.h"
#include "controller_mode_if.h"
#include "rsna_auth_km.h"
#include "qmu_if.h"

#ifdef AUTH_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name :  tkip_cntr_msr_auth                                      */
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

void tkip_cntr_msr_auth(mac_struct_t *mac)
{
    UWORD8 i                     = 0;
    rsna_auth_persta_t *rsna_ptr = NULL;

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

    /* Poll the PA status for Suspension */
    while(get_machw_spnd_tx_stat() == BFALSE);

    /* Flush all the QMU - queues */
    flush_all_qs();

    /* Flush all MISC event queue with event type - Tx-complete */
    event_q_remove_all_misc_events(&g_event_q[MISC_EVENT_QID], MISC_TX_COMP);

    /* Resume MAC HW Tx */
    set_machw_tx_resume();

    /* Load the RSNA Ptr to create the De-Auth message                    */
    for(i = 0 ; i < NUM_SUPP_STA; i++)
    {
        rsna_ptr = g_rsna_auth.rsna_ptr[i];

        /* If RSNA Entry is valid */
        if(rsna_ptr != NULL)
        {
            TROUT_DBG4("P2P: deauth. MIC_FAIL");
            /* Set the Deauthication reason to MIC FAIL */
            rsna_ptr->deauth_reason = MIC_FAIL;

            /* Send the Deauthentication frame */
            if(mget_DesiredBSSType() != INDEPENDENT)
            {
                asoc_entry_t *ae = (asoc_entry_t*)find_entry(rsna_ptr->addr);

                if(ae != NULL)
                {
                    /* Send the sta leaving information to host */
                    send_join_leave_info_to_host(ae->asoc_id, rsna_ptr->addr,
                                                 BFALSE);
                }

                send_deauth_frame(rsna_ptr->addr, rsna_ptr->deauth_reason);
            }

            /* Remove the entry for the sta                             */
            /* This does the RSNA Cleanup along with that of Asoc table */
            delete_entry(rsna_ptr->addr);
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
    g_rsna_auth.global.cntr_msr_in_progress = BTRUE;

    start_auth_cntr_msr_60s_timer();
}

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
