/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2007                               */
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
/*  File Name         : p2p_wlan_rx_mgmt_ap.h                                */
/*                                                                           */
/*  Description       : This file contains the extern declaration of the     */
/*                      functions requried for P2P WLAN RX Management        */
/*                                                                           */
/*  List of Functions : ap_enabled_rx_p2p_action                             */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_P2P

#ifndef P2P_WLAN_RX_MGMT_AP_H
#define P2P_WLAN_RX_MGMT_AP_H

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void ap_enabled_rx_p2p_action(UWORD8 *msg, mac_struct_t *mac);

#endif /* P2P_WLAN_RX_MGMT_AP_H */
#endif /* MAC_P2P */
#endif /* BSS_ACCESS_POINT_MODE */

