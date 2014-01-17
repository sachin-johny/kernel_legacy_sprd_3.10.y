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
/*  File Name         : sme_ap.h                                             */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to SME functions specific to AP mode.        */
/*                                                                           */
/*  List of Functions : initiate_start                                       */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE

#ifndef SME_AP_H
#define SME_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "fsm_ap.h"
#include "maccontroller.h"
#include "management.h"

#ifdef MAC_WMM
#include "management_11e.h"
#include "frame_11e.h"
#endif /* MAC_WMM */

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define START_TIMEOUT   300

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void send_mlme_rsp_to_host_ap(mac_struct_t *mac, UWORD8 type,
                                      UWORD8 *message);
extern void prepare_mlme_start_req(start_req_t *start_req);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function prepares the start request message and calls the MAC API to */
/* initiate the start                                                        */
INLINE void initiate_start(mac_struct_t *mac)
{
    start_req_t start_req = {{0,},};

    prepare_mlme_start_req(&start_req);

    mlme_start_req(mac, (UWORD8*)&start_req);
}

#endif /* SME_AP_H */

#endif /* BSS_ACCESS_POINT_MODE */
