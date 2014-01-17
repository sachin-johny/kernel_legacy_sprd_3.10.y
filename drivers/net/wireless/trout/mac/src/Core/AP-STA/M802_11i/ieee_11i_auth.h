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
/*  File Name         : ieee_11i_auth.h                                      */
/*                                                                           */
/*  Description       : This file contains definitions and inline utility    */
/*                      functions required by the MAC protocol interface for */
/*                      AP mode for 802.11i                                  */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I

#ifndef IEEE_11I_AUTH_H
#define IEEE_11I_AUTH_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "eapol_key_auth.h"
#include "index_util.h"
#include "mib_11i.h"
#include "management_11i.h"
#include "rkmal_auth.h"
#include "rsna_auth_km.h"
#include "tkip_auth.h"
#include "utils.h"

#ifdef AUTH_11I
/*****************************************************************************/
/* Extern Functions                                                          */
/*****************************************************************************/

extern void misc_11i_event_auth(mac_struct_t *mac, UWORD8 *msg);

#endif /* IEEE_11I_AUTH_H */

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
