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
/*  File Name         : event_parser.c                                       */
/*                                                                           */
/*  Description       : This file contains the event parser functions for    */
/*                      STA and AP mode.                                     */
/*                                                                           */
/*  List of Functions : process_host_rx_mlme                                 */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "iconfig.h"
#include "host_if.h"
#include "maccontroller.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_host_rx_mlme                                     */
/*                                                                           */
/*  Description   : This function parses the HOST_RX_MLME event, prepares a  */
/*                  host request and calls the MAC API.                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to the host buffer                            */
/*                                                                           */
/*  Globals       : g_config_eth_addr                                        */
/*                  g_src_ip_addr                                            */
/*                  g_config_ip_addr                                         */
/*                                                                           */
/*  Processing    : This function parses the configuration packet received   */
/*                  from the host and performs the necessary MIB set/get. In */
/*                  case a configuration parameter requiring reset is        */
/*                  changed, reset_mac is called and thereafter a scan       */
/*                  request is prepared and an appropriate API called to     */
/*                  start scanning. Note that the configuration message      */
/*                  buffer is freed here and need not be freed elsewhere.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_host_rx_mlme(mac_struct_t *mac, UWORD8* event)
{
    UWORD8 buffer_ofst  = 0;
    UWORD8 host_if_type = 0;
    UWORD8 *buffer_addr = 0;
    UWORD8 *host_req    = 0;

    print_log_debug_level_1("\n[DL1][INFO][Tx] {Host Rx Config Event}");
    TROUT_FUNC_ENTER;
    
    /* Extract the host request buffer. The length is not required. */
    buffer_addr = ((host_rx_event_msg_t *)event)->buffer_addr;
    buffer_ofst = ((host_rx_event_msg_t *)event)->pkt_ofst;
    host_if_type = (((host_rx_event_msg_t *)event)->host_if_type);

#ifdef ETHERNET_HOST
    if(host_if_type == ETHERNET_HOST_TYPE)
    {
        /* Save the host IP address and the host ethernet address in global      */
        /* variables for the first time. These are used to prepare the           */
        /* configuration response packets.                                       */
        {
            UWORD8 *eth_hdr     = 0;

            /* Initialize the ethernet header pointer */
            eth_hdr  = buffer_addr + buffer_ofst + ETH_ETHERNET_HDR_OFFSET;

            mac_addr_cpy(g_config_eth_addr, eth_hdr + 6);
            ip_addr_cpy(g_config_ip_addr, eth_hdr + 26);
        }
    }
#endif /* ETHERNET_HOST */

    /* Initialize the host request pointer to the UDP data as per the host   */
    /* protocol in use.                                                      */
    host_req = buffer_addr + buffer_ofst + get_config_pkt_hdr_offset(host_if_type);

    /* Parse the configuration message and take appropriate action */
    parse_config_message(mac, host_req, buffer_addr,host_if_type);

    TROUT_FUNC_EXIT;
}
