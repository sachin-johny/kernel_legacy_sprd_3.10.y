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
/*  File Name         : host_if_test.h                                       */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to host interface testing.                   */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef HOST_LOOP_BACK_MODE
#ifndef HOST_IF_TEST_H
#define HOST_IF_TEST_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "host_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define IP_ADDRESS_LEN            4
#define IP_SRC_ADDR_OFFSET        26

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void process_host_rx_loopback(UWORD8* event);

/*****************************************************************************/
/* Inline Function Definitions                                               */
/*****************************************************************************/

/* This function returns the host header offset from buffer base depending */
/* on the host type.                                                       */
INLINE UWORD8 get_host_header_offset(UWORD8 host_type)
{
    UWORD8 hdr_offset = 0;
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        hdr_offset = ETH_ETHERNET_HDR_OFFSET;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    if(hdr_offset == 0)
        g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */
    return hdr_offset;
}

/* This function swaps the source and destination addresses of the Ethernet */
/* and IP headers.                                                          */
INLINE void swap_addresses_for_loopback(msdu_desc_t *frame_desc)
{
    UWORD8* eth_pkt = frame_desc->buffer_addr + frame_desc->host_hdr_offset;
    UWORD8  dst_eth_addr[6] = {0};
    UWORD8  src_ip_addr[4]  = {0};

    /* SWAP Ethernet addresses */
    memcpy(dst_eth_addr, eth_pkt, 6);
    memcpy(eth_pkt, eth_pkt + 6, 6);
    memcpy(eth_pkt + 6, dst_eth_addr, 6);

    /* SWAP the IP addresses   */
    memcpy(src_ip_addr, eth_pkt + IP_SRC_ADDR_OFFSET, IP_ADDRESS_LEN);
    memcpy(eth_pkt + IP_SRC_ADDR_OFFSET,
           eth_pkt + IP_SRC_ADDR_OFFSET + IP_ADDRESS_LEN,
           IP_ADDRESS_LEN);
    memcpy(eth_pkt + IP_SRC_ADDR_OFFSET + IP_ADDRESS_LEN,
           src_ip_addr, IP_ADDRESS_LEN);

}

#endif  /* HOST_IF_TEST_H */
#endif /* HOST_LOOP_BACK_MODE */
