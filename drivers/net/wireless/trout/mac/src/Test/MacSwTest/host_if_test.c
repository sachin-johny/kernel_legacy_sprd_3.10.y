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
/*  File Name         : host_if_test.c                                       */
/*                                                                           */
/*  Description       : This file contains the functions required for        */
/*                      testing of the host interface.                       */
/*                                                                           */
/*  List of Functions : process_host_rx_loopback                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef HOST_LOOP_BACK_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#include "maccontroller.h"
#include "imem_if.h"
#include "host_if.h"
#include "buff_desc.h"
#include "host_if_test.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_host_rx_loopback                                 */
/*                                                                           */
/*  Description   : This function prepares frame descriptor and Queues the   */
/*                  packet in HostTX queue.                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the HOST RX EVENT                          */
/*                                                                           */
/*  Globals       :  None                                                    */
/*                                                                           */
/*  Processing    : Frame Descriptor is formatted according to the host type */
/*                  PacketType field in Host header is updated and the packet*/
/*                  is queued in HostTx Queue. This way every data packet    */
/*                  that comes from the host is looped back.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void process_host_rx_loopback(UWORD8* event)
{
    UWORD8      host_type    = ((host_rx_event_msg_t *)event)->host_if_type;
    UWORD8*     buffer_addr  = ((host_rx_event_msg_t *)event)->buffer_addr;
    UWORD16     rx_pkt_len   = ((host_rx_event_msg_t *)event)->rx_pkt_len;
    msdu_desc_t frame_desc   = {0};
    UWORD8      host_hdr_len = get_host_hdr_len(host_type);
    UWORD8      host_hdr_offset = get_host_header_offset(host_type);

	TROUT_FUNC_ENTER;
    frame_desc.buffer_addr     = buffer_addr;
    frame_desc.data_len        = rx_pkt_len - host_hdr_len;
    frame_desc.host_hdr_len    = host_hdr_len;
    frame_desc.data_offset     = host_hdr_offset + host_hdr_len;
    frame_desc.host_hdr_offset = host_hdr_offset;

#ifdef  ETHERNET_HOST
    /* Change PacketType to DATAIN */
    swap_addresses_for_loopback(&frame_desc);
#endif /* ETHERNET_HOST */

    send_data_frame_to_host_if(&frame_desc, host_type);
    TROUT_FUNC_EXIT;
}

#endif /* HOST_LOOP_BACK_MODE */
