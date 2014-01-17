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
/*  File Name         : radius_timers.c                                      */
/*                                                                           */
/*  Description       : This file contains the definitions related to the    */
/*                      various timers used by the radius state meachines.   */
/*                                                                           */
/*  List of Functions : start_radius_timer                                   */
/*                      stop_radius_timer                                    */
/*                      radius_alarm_fn                                      */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11I
#ifdef MAC_802_1X

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "radius_client.h"
#include "radius_timers.h"
#include "cglobals.h"
#include "mac_init.h"


/*****************************************************************************/
/*                                                                           */
/*  Function Name    : start_radius_timer                                    */
/*                                                                           */
/*  Description      : This function creates and starts a aWhile timer       */
/*                     with period equal to the serverTimeout period.        */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void start_radius_timer(radius_node_t *rad_node, UWORD32 period)
{
    if(rad_node != NULL)
    {
        rad_node->serv_timer = create_alarm(radius_alarm_fn,
                                            (UWORD32)rad_node, radius_alarm_work);

        /* aWhile is set to serverTimeout prior to calling this function */
        start_alarm(rad_node->serv_timer, period);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : stop_aWhile_timer                                     */
/*                                                                           */
/*  Description      : This function stops and deletes a aWhile timer        */
/*                     alarm resources.                                      */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*  Processing       : NONE                                                  */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

void stop_radius_timer(radius_node_t* rad_node)
{
    if((rad_node != NULL) && (rad_node->serv_timer != NULL))
    {
        stop_alarm(rad_node->serv_timer);
        delete_alarm(&(rad_node->serv_timer));
        rad_node->serv_timer = NULL;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name    : radius_alarm_fn                                       */
/*                                                                           */
/*  Description      : This function is the call back function for the       */
/*                     radius timer.                                         */
/*                                                                           */
/*  Inputs           : 1) 802.1x Handle                                      */
/*                                                                           */
/*  Globals          : NONE                                                  */
/*                                                                           */
/*  Processing       : This function is the call back function for the       */
/*                     radius timer.                                         */
/*                                                                           */
/*  Outputs          : NONE                                                  */
/*  Returns          : NONE                                                  */
/*  Issues           : NONE                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef ENABLE_AP_1X_LINUX_PROCESSING
void radius_alarm_fn(ALARM_HANDLE_T* alarm, ADDRWORD_T data)
#else /* ENABLE_AP_1X_LINUX_PROCESSING */
void radius_alarm_work(struct work_struct *work)
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
{
#ifdef ENABLE_AP_1X_LINUX_PROCESSING
	radius_node_t* rad_node = (radius_node_t *)(container_of(work, trout_timer_struct, work)->data);
#else
    radius_node_t* rad_node = (radius_node_t*)data;
#endif /* OS_LINUX_CSL_TYPE */

    if(rad_node == NULL)
    {
        return;
    }
    if( reset_mac_trylock() == 0 ) 
    {
	return;
    }
    ALARM_WORK_ENTRY(work);

    /* Indicate timer completion by resetting the aWhile timer variable     */
    stop_radius_timer(rad_node);

    /* Delete the pending node corresponding to this radius handle */
    get_pending_node(rad_node->req_id);

    if(rad_node->auth_1x != NULL)
    {
        /* Retry sending the message */
        if(send_msg_2_server(rad_node->auth_1x) == BFALSE)
        {
            rad_node->tx_count = 0;
            if(rad_node->rad_msg != NULL)
            {
                pkt_mem_free(rad_node->rad_msg);
                rad_node->rad_msg = NULL;
            }
            rad_node->auth_1x->global.aWhileTimeout = BTRUE;
            auth_onex_fsm_run(rad_node->auth_1x);
        }
    }
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

//add by Hugh
#ifdef ENABLE_AP_1X_LINUX_PROCESSING

void radius_alarm_fn(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif


#endif /* MAC_802_1X */
#endif /* MAC_802_11I */
#endif /* BSS_ACCESS_POINT_MODE */
