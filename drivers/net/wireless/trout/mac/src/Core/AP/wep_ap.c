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
/*  File Name         : wep_ap.c                                             */
/*                                                                           */
/*  Description       : This file contains all the WEP management related    */
/*                      functions for the Access point mode                  */
/*                                                                           */
/*  List of Functions : msg_3_timeout                                        */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "management.h"
#include "management_ap.h"
#include "wep_ap.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : msg_3_timeout                                            */
/*                                                                           */
/*  Description   : This is the Shared key message-3time out call back       */
/*                  function                                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Alarm Handle                           */
/*                  2) The alarm callback function data                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function removes the association table entry        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifndef OS_LINUX_CSL_TYPE
void msg_3_timeout(HANDLE_T* alarm, ADDRWORD_T data)
#else /* OS_LINUX_CSL_TYPE */
void msg_3_timeout_work(struct work_struct *work)
#endif /* OS_LINUX_CSL_TYPE */
{
#ifdef OS_LINUX_CSL_TYPE
	UWORD8 *data = (UWORD8 *)(container_of(work, trout_timer_struct, work)->data);
#endif
	if( reset_mac_trylock() == 0 ){
		return;
	}
	ALARM_WORK_ENTRY(work);
    /* Send the De-authentication Frame to the station */
    send_deauth_frame((UWORD8 *)data, (UWORD16)AUTH_NOT_VALID);

    /* If the message # 3 is timed out, remove the association table entry */
    delete_entry((UWORD8 *)data);
    ALARM_WORK_EXIT(work);
    reset_mac_unlock();
}

#ifdef OS_LINUX_CSL_TYPE    //add by Hugh

void msg_3_timeout(ADDRWORD_T data)
{
    alarm_fn_work_sched(data);
}

#endif

#endif /* BSS_ACCESS_POINT_MODE */
