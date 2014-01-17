/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2009                               */
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
/*  File Name         : sta_uapsd.c                                          */
/*                                                                           */
/*  Description       : This file contains all the functions related to STA  */
/*                      WMM PS (UAPSD) feature.                              */
/*                                                                           */
/*  List of Functions : uapsd_handle_wlan_rx_sta                             */
/*                      uapsd_trigger_sp_sta                                 */
/*                      uapsd_handle_tx_complete_sta                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "sta_uapsd.h"
#include "pm_sta.h"
#include "core_mode_if.h"
#include "frame_11e.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

BOOL_T              g_qap_uapsd_capable = BFALSE;
STA_UAPSD_SP_STAT_T g_uapsd_sp_status   = SP_NOT_IN_PROGRESS;
sta_uapsd_config_t  g_sta_uapsd_config  = {0};
static UWORD8       g_eosp_timeout_cnt  = 0;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : uapsd_handle_wlan_rx_sta                                 */
/*                                                                           */
/*  Description   : This function handles the reception of a packet on WLAN  */
/*                  in STA mode for UAPSD                                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame                                  */
/*                  2) Priority of the frame                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function processes received frames for delivery     */
/*                  enabled queues. The EOSP and More Data bits of the       */
/*                  received frame are checked. If the EOSP bit is not set   */
/*                  the UAPSD service period continues and nothing is done.  */
/*                  If this bit is set and More Data bit is 0 the STA can    */
/*                  stop the service period and go to doze. If More Data bit */
/*                  is set indicating the AP has more frames buffered the    */
/*                  STA can continue the service period by sending a NULL    */
/*                  frame (which will act as a trigger frame) from any queue */
/*                  that is trigger enabled.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void uapsd_handle_wlan_rx_sta(UWORD8 *msa, UWORD8 priority)
{
    /* Received frames are processed only for delivery enabled ACs */
    if(is_delivery_enabled(priority) == BTRUE)
    {
		// 20120709 caisf add, merged ittiam mac v1.2 code
        /* Reception of packets within a SP indicates there is no hang       */
        /* Hence reset the eosp timeout counter which is used to track any   */
        /* such hang scenearios                                              */
        if(is_uapsd_sp_not_in_progress() == BFALSE)
        {
            g_eosp_timeout_cnt = 0;
        }

        /* If EOSP is set indicating the UAPSD service period is ended check */
        /* what the STA should do next.                                      */
        if(get_eosp(msa) == 1)
        {
            /* Stop the UAPSD service period */
            stop_uapsd_sp();
			// 20120709 caisf masked, merged ittiam mac v1.2 code
   			//g_eosp_timeout_cnt = 0;

            if(get_more_data(msa) == 1)
            {
                /* If More Data bit is set it indicates that the AP has more */
                /* packets buffered for STA. In this case trigger a service  */
                /* period.                                                   */
                uapsd_trigger_sp_sta();
            }
            else
            {
                /* If More Data bit is not set go back to doze */
                sta_doze();
            }
        }
    }
    else if(is_trigger_enabled(priority) == BTRUE)
    {
        /* Exception case. Should not occur. If it is an only trigger        */
        /* AC there should be no downlink traffic.                           */
#ifdef DEBUG_MODE
        g_mac_stats.wmmpssta_tacdl++;
#endif /* DEBUG_MODE */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : uapsd_trigger_sp_sta                                     */
/*                                                                           */
/*  Description   : This function triggers a UAPSD service period in STA     */
/*                  mode.                                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function triggers a service period in order to      */
/*                  retrieve the frames buffered by the AP for delivery      */
/*                  enabled ACs. If the STA is already waiting for a service */
/*                  period to start or if a service period has already been  */
/*                  started then there is no need to trigger again. Hence do */
/*                  nothing. This is possible if by the time the trigger     */
/*                  function is called some other process has triggered the  */
/*                  service period. This function finds the highest priority */
/*                  AC available that is trigger enabled. On finding such an */
/*                  AC it sends a QOS NULL trigger frame from this AC and    */
/*                  starts the UAPSD service period. Note that the frame     */
/*                  transmission status needs to be checked before actually  */
/*                  starting the service period. This is done during         */
/*                  transmit complete processing. If no trigger enabled AC   */
/*                  is found the STA cannot send a trigger frame. In such a  */
/*                  case it does not start the service period and goes back  */
/*                  to DOZE.                                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : It is currently assumed that even if there are more      */
/*                  frames to receive and none of the queues are trigger     */
/*                  enabled there is no way to retrieve these frames.        */
/*                                                                           */
/*****************************************************************************/

void uapsd_trigger_sp_sta(void)
{
    UWORD8 pri = INVALID_PRI;

#ifdef DEBUG_MODE
    g_mac_stats.wmmpssta_trigsp++;
#endif /* DEBUG_MODE */

    /* The service period is triggered only if it is not in progress. If in  */
    /* progress or waiting to start there is no need to trigger. Do nothing. */
    if(is_uapsd_sp_not_in_progress() == BFALSE)
    {
		g_eosp_timeout_cnt++;

		if(g_eosp_timeout_cnt <= EOSP_TIMEOUT_CNT)
		{
#ifdef DEBUG_MODE
    		g_mac_stats.wmmpssta_trspnr++;
#endif /* DEBUG_MODE */
			return;
		}
		else
		{
			PRINTD2("ESOP TIMEOUT\n");
		}
    }

    pri = get_highest_trigger_enabled_priority();

    /* If no trigger enabled AC is found the STA is unable to start a UAPSD  */
    /* service period and can switch back to DOZE mode.                      */
    if(pri == INVALID_PRI)
    {
        sta_doze();
    }
    /* If a trigger enabled AC is found the STA sends a QoS NULL trigger     */
    /* frame from this AC and starts the UAPSD service period.               */
    else
    {
        wait_uapsd_sp_start();
        send_null_frame_to_AP(STA_DOZE, BTRUE, pri);
    	g_eosp_timeout_cnt = 0;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : uapsd_handle_tx_complete_sta                             */
/*                                                                           */
/*  Description   : This function handles the transmit complete event in STA */
/*                  mode for UAPSD                                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the frame transmitted                      */
/*                  2) Transmit success flag (derived from the descriptor)   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks if the frame transmitted was a      */
/*                  trigger frame. If not no processing is required. If it   */
/*                  is a trigger frame and the STA is waiting for a service  */
/*                  period to be started the status is checked. If status    */
/*                  is successful the service period is started. If timeout  */
/*                  the STA tries to trigger a service period again. Note    */
/*                  that it must stop the service period first (i.e. reset   */
/*                  the wait status) since the trigger can be done only if   */
/*                  service period is not in progress. Also the trigger      */
/*                  function must be explicitly called for this (instead     */
/*                  of simply retrying a QoS NULL frame from the AC to which */
/*                  the timed out frame belongs) since the WMM-PS            */
/*                  configuration may have been modified meanwhile.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void uapsd_handle_tx_complete_sta(UWORD8 *msa, UWORD8 tx_success)
{
    if((is_uapsd_sp_wait_in_progress() == BTRUE) &&
       (is_trigger_frame(msa) == BTRUE))
    {
        if(tx_success == 1)
        {
            start_uapsd_sp();
        }
        else
        {
            stop_uapsd_sp();
            uapsd_trigger_sp_sta();
        }
    }
    else
    {
        /* Do nothing */
    }
}

#endif /* MAC_WMM */
#endif /* IBSS_BSS_STATION_MODE */
