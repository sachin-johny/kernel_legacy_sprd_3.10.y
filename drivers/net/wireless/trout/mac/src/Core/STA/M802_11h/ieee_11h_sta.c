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
/*  File Name         : ieee_11h_sta.c                                       */
/*                                                                           */
/*  Description       : This file contains all the functions related to the  */
/*                      functions called by the STA MAC FSM on receiving     */
/*                      802.11ih MISC events.                                */
/*                                                                           */
/*  List of Functions : handle_tbtt_dfs_sta                                  */
/*                      handle_tx_complete_dfs_sta                           */
/*                      update_11i_fail_stats_sta                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "dfs_sta.h"
#include "tpc_sta.h"
#include "controller_mode_if.h"
#include "mac_init.h"
#include "index_util.h"
#include "receive.h"
#include "metrics.h"
#include "iconfig.h"
#include "cglobals_sta.h"
#include "mib1.h"
#include "frame.h"
#include "mh.h"
#include "prot_if.h"
#include "itypes.h"
#include "frame.h"
#include "transmit.h"
#include "qmu_if.h"
#include "rf_if.h"
#include "imem_if.h"
#include "core_mode_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : handle_tx_complete_dfs_sta                               */
/*                                                                           */
/*  Description   : This function is called when ever Tx complete            */
/*                  interrupt is occured                                     */
/*                                                                           */
/*  Inputs        : 1) pointer to discriptor                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : If the transmitted packet is a autonomous report         */
/*                  frame then suspend Tx,if status is Tx time out then      */
/*                  resend it.                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void handle_tx_complete_dfs_sta(UWORD32 *dscr)
{
    /* Get the buffer address of the transmitted frame                   */
    UWORD8  *msa     = (UWORD8*)get_tx_dscr_mac_hdr_addr((UWORD32 *)dscr);

	TROUT_FUNC_ENTER;
    if(get_sub_type(msa) == ACTION)
    {
        if(get_action_field(msa) == MEASUREMENT_REPORT_TYPE)
        {
            if(get_dialog_token_field(msa) == AUTONOMOUS)
            {
                if(get_meas_token_field(msa) == AUTONOMOUS)
                {
                    if(get_tx_dscr_status((UWORD32 *)dscr) != TX_TIMEOUT)
                    {
                        /* If the transmitted packet is a autonomous report */
                        /* frame then suspend Tx                            */
                        suspend_tx_dueto_radar();
                        g_sending_aut_report_in_progress = BFALSE;
                    }

                    else
                    {
                        /* do nothing */
                    }

                }
            }
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : misc_ieee_11h_radar_detected                             */
/*                                                                           */
/*  Description   : This function is called when ever Radar detected         */
/*                  interrupt is occured.                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_autonomous_basic_report                                */
/*                  g_sending_aut_report_in_progress                         */
/*                                                                           */
/*  Processing    : If radar detection interrupt is occured, STA will send   */
/*                  autonomous report to AP if sending autonomous report     */
/*                  sending is enabled by AP.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void misc_ieee_11h_radar_detected(void)
{

    /* Set the global with radar detected information */
    g_radar_detected = BTRUE;

    if(g_autonomous_basic_report == BTRUE)
    {
        if(BTRUE == can_dfs_sta_transmit_now())
        {
            /* if this condition is false,means STA has already */
            /* sent the autonomous report */
            if(g_sending_aut_report_in_progress == BFALSE)
            {
                g_sending_aut_report_in_progress = BTRUE;
                send_autonomous_report();
            }
        }

    }
}

#endif /* MAC_802_11H */

#endif /* IBSS_BSS_STATION_MODE */
