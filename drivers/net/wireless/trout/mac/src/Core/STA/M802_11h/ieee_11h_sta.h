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
/*  File Name         : ieee_11h_sta.h                                       */
/*                                                                           */
/*  Description       : This file contains all the functions declarations    */
/*                      that are used in ieee_11h_sta.c                      */
/*                      Station mode of operation.                           */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11H

#ifndef IEEE_11H_STA_H
#define IEEE_11H_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void handle_tx_complete_dfs_sta(UWORD32 *dscr);
extern void misc_ieee_11h_radar_detected(void);
extern void handle_tbtt_dfs_sta(void);

#endif /* IEEE_11H_STA_H */

#endif /* MAC_802_11H */

#endif /* IBSS_BSS_STATION_MODE */
