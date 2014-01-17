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
/*  File Name         : tkip_sta.h                                           */
/*                                                                           */
/*  Description       : This file contains the MAC S/W TKIP related          */
/*                      functions declarations                               */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE

#ifdef MAC_802_11I

#ifndef TKIP_STA_H
#define TKIP_STA_H


/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "maccontroller.h"
#include "rsna_supp_km.h"
#include "tkip.h"

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void tkip_cntr_msr_supp(mac_struct_t *mac);
extern void send_mic_error_report(rsna_supp_persta_t *,BOOL_T);

#endif /* TKIP_STA_H */
#endif /* MAC_802_11I */
#endif /* IBSS_BSS_STATION_MODE */
