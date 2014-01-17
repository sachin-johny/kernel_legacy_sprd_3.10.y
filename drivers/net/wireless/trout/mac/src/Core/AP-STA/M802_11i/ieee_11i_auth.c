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
/*  File Name         : ieee_11i_auth.c                                      */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the Auth MAC FSM on receiving    */
/*                      802.11i MISC events.                                 */
/*                                                                           */
/*  List of Functions : misc_11i_event_auth                                  */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I
/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "controller_mode_if.h"
#include "mac_init.h"
#include "maccontroller.h"
#include "management.h"
#include "transmit.h"
#include "ieee_11i_auth.h"
#include "qmu_if.h"
#include "tkip_auth.h"
#include "iconfig.h"

#ifdef AUTH_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name : misc_11i_event_auth                                      */
/*                                                                           */
/*  Description   : This function handles 802.11i events in ENABLED state.   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Pointer to the incoming message                       */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC core function is called to handle 802.11i events */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void misc_11i_event_auth(mac_struct_t *mac, UWORD8 *msg)
{
    misc_event_msg_t *misc_event_msg = (misc_event_msg_t *)msg;

    switch(misc_event_msg->name)
    {
        case MISC_AUTH_SND_DEAUTH:
        {
            /* Load the RSNA Ptr to create the Send Auth message */
            rsna_auth_persta_t *rsna_ptr = (rsna_auth_persta_t *)
                                           misc_event_msg->data;

            /* If RSNA Entry is valid */
            if(rsna_ptr != NULL)
            {
                /* Deauth the RSNA Associated STA */
#ifdef BSS_ACCESS_POINT_MODE
                if(mget_DesiredBSSType() != INDEPENDENT)
                {
                    asoc_entry_t *ae = (asoc_entry_t*)find_entry(rsna_ptr->addr);

                    if(ae != NULL)
                    {
                        /* Send the sta leaving information to host */
                        send_join_leave_info_to_host(ae->asoc_id,
                                                     rsna_ptr->addr, BFALSE);
                    }

                    send_deauth_frame(rsna_ptr->addr, rsna_ptr->deauth_reason);
                }
#endif /* BSS_ACCESS_POINT_MODE */

                /* Remove the entry for the sta                              */
                /* This does the RSNA Cleanup along with that of Asoc table  */
                delete_entry(rsna_ptr->addr);
            }
            else
            {
                /* Should not come here */
            }
        }
        break;

        /* This Event is started to handle the initiate the REKEYing */
        case MISC_STRT_REKEY:
        {
            UWORD8 i = 0;
            for(i = 0 ; i < NUM_SUPP_STA; i++)
            {
                if(g_rsna_auth.rsna_ptr[i] != NULL)
                {
                    auth_rsn_fsm_run(g_rsna_auth.rsna_ptr[i]);
                }
            }
        }
        break;

        /* This Event is started to handle the TKIP Countermeasures */
        case MISC_AUTH_TKIP_CNT_MSR:
        {
            tkip_cntr_msr_auth(mac);
        }
        break;

        case MISC_RSNA_TIMEOUT:
        {
            rsna_auth_persta_t *rsna_ptr = (rsna_auth_persta_t*)misc_event_msg->data;
        
            if(rsna_ptr != NULL)
            {
                printk("@@@invoke rsna timeout fn.@@@\n");
                handle_rsna_timeout(rsna_ptr);
            }   
            else
            {     
                printk("@@@@have lost the association!@@@\n");
            }     
        }
        break;
        default:
        {
        }
        break;
    }
}
#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
