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
/*  File Name         : tkip_ap.h                                            */
/*                                                                           */
/*  Description       : This file contains the MAC S/W TKIP related          */
/*                      functions declarations                               */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef MAC_802_11I
#ifndef TKIP_AUTH_H
#define TKIP_AUTH_H


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "maccontroller.h"
#include "tkip.h"

#ifdef AUTH_11I
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void tkip_cntr_msr_auth(mac_struct_t *mac);

#endif /* TKIP_AUTH_H */

#endif /* AUTH_11I */
#endif /* MAC_802_11I */
#endif /* MAC_HW_UNIT_TEST_MODE */
