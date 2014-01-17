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
/*  File Name         : management_11h.h                                     */
/*                                                                           */
/*  Description       : This file contains the MAC management related        */
/*                      definitions for 802.11h mode                         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_802_11H

#ifndef MANAGEMENT_11H_H
#define MANAGEMENT_11H_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define SENSITIVITY (WORD8) -70 /* Sensitivity in dBm */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* 11h events */
typedef enum {MISC_RADAR_DETECTED           = 0x60,
              MISC_DFS_EXPIRY_TIMER_TRIGGER = 0x62
} EVENT_TYPESUBTYPE_11H_T;

#endif /* MANAGEMENT_11H_H */

#endif /* MAC_802_11H */
