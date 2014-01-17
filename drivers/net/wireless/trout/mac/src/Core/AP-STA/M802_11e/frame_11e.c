/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2008                               */
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
/*  File Name         : frame_11e.c                                          */
/*                                                                           */
/*  Description       : This file contains the  functions for setting        */
/*                      various fields of the MAC frames for 802.11e.        */
/*                                                                           */
/*  List of Functions : set_mac_hdr_11e                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_WMM

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "core_mode_if.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_mac_hdr_11e                                          */
/*                                                                           */
/*  Description   : This function sets the 11e MAC header.                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC header                             */
/*                  2) TID                                                   */
/*                  3) ACK Policy                                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the MAC header for QoS frames.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, QoS MAC header length                            */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 set_mac_hdr_11e(UWORD8 *mac_hdr, UWORD8 tid, UWORD8 ack_policy)
{
    set_frame_control(mac_hdr, QOS_DATA);
    set_qos_control(mac_hdr, tid, ack_policy);

    return QOS_MAC_HDR_LEN;
}

#endif /* MAC_WMM */
