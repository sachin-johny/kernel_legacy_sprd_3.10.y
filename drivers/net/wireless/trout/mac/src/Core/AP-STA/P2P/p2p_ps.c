/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : p2p_ps.c                                             */
/*                                                                           */
/*  Description       : This file conatins the power save management         */
/*                      functions for P2P protocol                           */
/*                                                                           */
/*  List of Functions : post_absent_period_event                             */
/*                      p2p_absent_period_isr                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_P2P

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "mib_p2p.h"
#include "p2p_ps.h"

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

#ifdef MAC_P2P_HW_SUPP
static void post_absent_period_event(EVENT_TYPESUBTYPE_P2P_T type);
#endif /* MAC_P2P_HW_SUPP */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : post_absent_period_event                                 */
/*                                                                           */
/*  Description   : This function posts the appropriate Absent period event. */
/*                                                                           */
/*  Inputs        : 1) Miscellaneous event sub-type                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : A MISC event of the given type is allocated and posted   */
/*                  in the MISC_EVENT queue.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void post_absent_period_event(EVENT_TYPESUBTYPE_P2P_T type)
{
    misc_event_msg_t *misc = 0;

    /* Allocate an event buffer */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    if(misc == NULL)
    {
        return;
    }

    misc->name = type;

    /* Post the event */
    post_event((UWORD8 *)misc, MISC_EVENT_QID);
}

#ifdef MAC_P2P_HW_SUPP

/*****************************************************************************/
/*                                                                           */
/*  Function Name : p2p_absent_period_isr                                    */
/*                                                                           */
/*  Description   : This is the P2P Absent Period ISR.                       */
/*                                                                           */
/*  Inputs        : 1) Miscellaneous event sub-type                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function posts the appropriate MISC event.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void p2p_absent_period_isr(EVENT_TYPESUBTYPE_P2P_T type)
{
    post_absent_period_event(type);
}

#endif /* MAC_P2P_HW_SUPP */

#endif /* MAC_P2P */
