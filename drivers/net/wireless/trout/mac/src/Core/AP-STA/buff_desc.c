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
/*  File Name         : buff_desc.c                                          */
/*                                                                           */
/*  Description       : This file contains functions for creating buffer     */
/*                      descriptors for normal MSDUs.                        */
/*                                                                           */
/*  List of Functions : create_msdu_desc                                     */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "buff_desc.h"
#include "receive.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_msdu_desc                                         */
/*                                                                           */
/*  Description   : This function updates the frame descriptor in the given  */
/*                  MSDU indicate structure.                                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WLAN frame Rx structure                */
/*                  2) Pointer to the MSDU Indicate strcuture                */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The frame descriptor in the MSDU indicate structure is   */
/*                  updated with the required data buffer details. The host  */
/*                  header details are not updated here as it depends on the */
/*                  specific host interface in use.                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : MSDU_PROC_STATUS_T, Status of MSDU processing.           */
/*                  PROC_ERROR if there is a processing error                */
/*                  PROC_LAST_MSDU if MSDU processing is successfully done   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

MSDU_PROC_STATUS_T create_msdu_desc(void *wlan_rx_struct, void *msdu_struct)
{
    wlan_rx_t       *wlan_rx    = (wlan_rx_t *)wlan_rx_struct;
    msdu_indicate_t *msdu       = (msdu_indicate_t *)msdu_struct;
    msdu_desc_t     *frame_desc = 0;

    frame_desc = &(msdu->frame_desc);

    /* Set the information related to the MSDU payload buffers. */
    frame_desc->buffer_addr = wlan_rx->msa;
    frame_desc->data_len    = wlan_rx->data_len;
    frame_desc->data_offset = wlan_rx->hdr_len;

    /* Host header depends upon the specific host interface and is hence not */
    /* updated here.                                                         */
    frame_desc->host_hdr_len    = 0;
    frame_desc->host_hdr_offset = 0;

    /* Update address information for the MSDU */
    mac_addr_cpy(msdu->sa, wlan_rx->sa);
    mac_addr_cpy(msdu->da, wlan_rx->da);

    return PROC_LAST_MSDU;
}

