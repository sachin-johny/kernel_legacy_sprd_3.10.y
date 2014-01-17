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
/*  File Name         : host_if.h                                            */
/*                                                                           */
/*  Description       : This file contains definitions and inline functions  */
/*                      required to interface with the host driver in use.   */
/*                                                                           */
/*  List of Functions : receive_from_host                                    */
/*                      is_snap_header_present                               */
/*                      adjust_for_snap_header                               */
/*                      is_host_ready                                        */
/*                      is_host_busy                                         */
/*                      get_src_eth_addr                                     */
/*                      get_config_pkt_hdr_len                               */
/*                      get_config_pkt_hdr_offset                            */
/*                      get_hif_cnfg_pkt_tx_q_cnt                            */
/*                      get_cnfg_pkt_q_count                                 */
/*                      get_eth_hdr_offset                                   */
/*                      get_host_hdr_len                                     */
/*                      set_host_hdr                                         */
/*                      send_frame_to_host                                   */
/*                      prepare_host_data_frame                              */
/*                      is_eth_config_pkt                                    */
/*                      prepare_config_pkt_eth_host                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef HOST_IF_H
#define HOST_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "frame.h"
#include "maccontroller.h"
#include "buff_desc.h"
#include "cglobals.h"
#include "receive.h"
#ifdef MEASURE_PROCESSING_DELAY
#include "metrics.h"
#endif /* MEASURE_PROCESSING_DELAY */

#ifdef IBSS_BSS_STATION_MODE
#include "cglobals_sta.h"
#endif /* IBSS_BSS_STATION_MODE */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Host types */
typedef enum {
#ifdef ETHERNET_HOST
              ETHERNET_HOST_TYPE = 0x00,
#endif /* ETHERNET_HOST */




              INVALID_HOST_TYPE = 0x05
} HOST_TYPE_T;

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_BUFFER_LEN           1596
#define HOST_BUSY_TIME_OUT_COUNT 200

/*****************************************************************************/
/* Generic host header/packet format related constants                       */
/*****************************************************************************/

/* 802.11 frame format                                                       */
/* +-------------------+----------------------------------------------+      */
/* | MAC Header        | Data = SNAP (IP/ARP type) + Ethernet Payload |      */
/* +-------------------+----------------------------------------------+      */
/* | 24/26/30/32 bytes | x (+ 8) bytes                                |      */
/* +-------------------+----------------------------------------------+      */
/*                                                                           */
/* 802.3 frame format                                                        */
/* +----------------+------------------+                                     */
/* | Ethenet Header | Ethernet Payload |                                     */
/* +----------------+------------------+                                     */
/* | 14 bytes       | x bytes          |                                     */
/* +----------------+------------------+                                     */
/*                                                                           */
/* IP/UDP headers/offsets                                                    */
/* +-------------------------------------------------------------------+     */
/* | Eth DST ADDR | Eth SRC ADDR | TYPE |  | IP protocol | IP SRC ADDR |     */
/* +-------------------------------------------------------------------+     */
/* |  0              6              12    14  23            26         |     */
/* +-------------------------------------------------------------------+     */
/* | IP DST ADDR | UDP SRC Port | UDP DST Port | UDP PktLen | Checksum |     */
/* +-------------------------------------------------------------------+     */
/* |  30            34             36             38           40      |     */
/* +-------------------------------------------------------------------+     */

#define ETH_PKT_TYPE_OFFSET       12
#define WAPI_TYPE                 0x88B4
#define IPV6_TYPE                 0x86DD
#define IP_TYPE                   0x0800
#define ARP_TYPE                  0x0806
#define ONE_X_TYPE                0x888E
#define VLAN_TYPE                 0x8100
#define LLTD_TYPE                 0x88D9
#define UDP_TYPE                  0x11
#define TCP_TYPE                  0x06
#define ETHERNET_HDR_LEN          14
#define IP_HDR_OFFSET             ETHERNET_HDR_LEN
#define IP_HDR_LEN                20
#define IP_PROT_OFFSET            23
#define UDP_HDR_OFFSET            (IP_HDR_LEN + IP_HDR_OFFSET)
#define UDP_HDR_LEN               8
#define UDP_DATA_OFFSET           (UDP_HDR_OFFSET + UDP_HDR_LEN)
#define UDP_SRC_PORT_OFFSET       UDP_HDR_OFFSET
#define UDP_DST_PORT_OFFSET       (UDP_HDR_OFFSET + 2)
#define VLAN_HDR_LEN              18
#define TOS_FIELD_OFFSET          15
#define VLAN_TID_FIELD_OFFSET     14
#define MAC_UDP_DATA_LEN          1472
#define MAX_UDP_IP_PKT_LEN        (MAC_UDP_DATA_LEN + UDP_DATA_OFFSET)

#ifdef HOST_LOOP_BACK_MODE
#define MAX_HOST_IF_Q_CNT         50
#else  /* HOST_LOOP_BACK_MODE */
#define MAX_HOST_IF_Q_CNT         10
#endif /* HOST_LOOP_BACK_MODE */

/*****************************************************************************/
/* Ethernet host related constants                                           */
/*****************************************************************************/

#ifdef ETHERNET_HOST

/* ------------------------------------------------------------------------- */
/* Ethernet Host header                                                      */
/* ------------------------------------------------------------------------- */
/* Ethernet destination addr   | Ethernet source addr   | Ethernet type    | */
/* ------------------------------------------------------------------------- */
/* Byte 0 - 5                  | Byte 6 - 11            | Byte 12 - 14     | */
/* ------------------------------------------------------------------------- */

/* Worst Case Buffer Pad (Ethernet Header Offset in the buffers)             */
/* = (MAX_MAC_HEADER_LEN + SNAP_HEADER_LEN) - // 26 + 8                      */
/*   ETHERNET_HDR_LEN) +  // 14                                              */
/*   WORD_ALIGNMENT_PAD + // 0                                               */
/*   SUB_MSDU_HEADER_LENGTH // 14                                            */
/* = 34                                                                      */
/*                                                                           */
/* Buffer format for reading data from Ethernet Host                         */
/* +--------------------------------------------------------------+          */
/* | Buffer pad      | Ethernet Header     | Ethernet Payload     |          */
/* +--------------------------------------------------------------+          */
/* | 34 bytes        | 14 bytes            | x bytes              |          */
/* +--------------------------------------------------------------+          */

#define WORD_ALIGNMENT_PAD        0

#define ETH_ETHERNET_HDR_OFFSET   (MAX_MAC_HDR_LEN + SUB_MSDU_HEADER_LENGTH + \
                                   SNAP_HDR_LEN - ETHERNET_HDR_LEN + WORD_ALIGNMENT_PAD)

#define ETH_CONFIG_PKT_HDR_LEN    UDP_DATA_OFFSET
#define ETH_CONFIG_PKT_HDR_OFFSET (ETH_ETHERNET_HDR_OFFSET + \
                                   ETH_CONFIG_PKT_HDR_LEN)

/* Ethernet Configuration message frame format (UDP) related */
#define HOST_UDP_PORT             50000
#define WLAN_UDP_PORT             50000

/* The gap inserted before each sub MSDU frame body to allow insertion of    */
/* the host header.                                                          */
/* Gap = MAX(ETHERNET_HDR_LEN - SUB_MSDU_HEADER_LENGTH,                      */
/*           MAX_MAC_HDR_LEN - SUB_MSDU_HEADER_LENGTH + FCS_LEN)             */
#define ETHERNET_HOST_HEADER_GAP  16

#else /* ETHERNET_HOST */

/* Required constants set to 0 if Ethernet host is not defined */
#define ETH_CONFIG_PKT_HDR_LEN    0
#define ETHERNET_HOST_HEADER_GAP  0

#endif /* ETHERNET_HOST */

/*****************************************************************************/
/* SDIO host related constants                                               */
/*****************************************************************************/


/* Required constants set to 0 if SDIO host is not defined */
#define SDIO_CONFIG_PKT_HDR_LEN    0
#define SDIO_HOST_HEADER_GAP       0


/*****************************************************************************/
/* UART host related constants                                               */
/*****************************************************************************/


/* Required constants set to 0 if UART host is not defined */
#define UART_CONFIG_PKT_HDR_LEN    0
#define UART_HOST_HEADER_GAP       0


/*****************************************************************************/
/* USB host related constants                                               */
/*****************************************************************************/


/* Required constants set to 0 if USB host is not defined */
#define USB_CONFIG_PKT_HDR_LEN    0
#define USB_HOST_HEADER_GAP       0


/*****************************************************************************/
/* Generic constants derived from multiple host constants                    */
/*****************************************************************************/

#define MAX_CONFIG_PKT_HDR_LEN MAX(ETH_CONFIG_PKT_HDR_LEN, \
                                   MAX(UART_CONFIG_PKT_HDR_LEN, \
                                       MAX(SDIO_CONFIG_PKT_HDR_LEN, \
                                           USB_CONFIG_PKT_HDR_LEN )))

#define HOST_HEADER_GAP        MAX(ETHERNET_HOST_HEADER_GAP, \
                                   MAX(UART_HOST_HEADER_GAP, \
                                       MAX(SDIO_HOST_HEADER_GAP,\
                                           USB_HOST_HEADER_GAP )))

#define MAX_NUM_HOST_TYPES     5
/*****************************************************************************/
/* Extern variable declarations                                              */
/*****************************************************************************/

extern UWORD8   g_host_data_if_type;
extern UWORD8   g_host_config_if_type;
extern UWORD8   g_src_ip_addr[4];
extern UWORD8   g_src_netmask_addr[6];
extern UWORD8   g_snap_header[SNAP_HDR_ID_LEN];

#ifdef ETHERNET_HOST
extern UWORD8   g_config_eth_addr[6];
extern UWORD8   g_config_ip_addr[4];
extern UWORD8   g_src_eth_addr[6];
extern UWORD32  g_eth_host_ctx_event_qid;
extern UWORD8   g_eth_buffer_host_ctx_events[TOTAL_MEM_SIZE_HOST_CTX_EVENTS];
#endif /* ETHERNET_HOST */




/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    init_host_interface(void);
extern void    reset_host_interface(void);
extern void    soft_reset_host_if(void);
extern void    send_data_frame_to_host_if(msdu_desc_t *fdesc, UWORD8 hst_type);
extern void    send_config_frame_to_host_if(msdu_desc_t *host_frame_desc,
                                            UWORD8 host_type);
extern void    send_pending_frames_to_host(void);
extern BOOL_T  is_config_pkt(UWORD8* buffer, UWORD8 host_if_type);
extern UWORD16 prepare_config_pkt_hdr(UWORD8 *data, UWORD16 len, UWORD8 host_type);
extern UWORD16 prepare_log_pkt(UWORD8 *data, UWORD8 *in_data, UWORD16 len,
                               UWORD8 host_type);
extern void    host_frame_completion_fn(void* data, UWORD32 status,
                                        UWORD8 host_type);
extern BOOL_T  adjust_snap_header(msdu_desc_t *frame_desc);
extern void    wait_for_host_idle(void);
extern void    remove_all_in_hifq(void);
extern void    select_default_hif(void);
extern void    send_enabled_msg_to_host(void);


extern UWORD16 checksum(UWORD8 *buffer, UWORD32 size);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function initializes all Host TX Config event queue information */
INLINE void init_host_ctx_event_q_info(void)
{
    /* Initialize number of Host TX Config event queues to 0 */
    set_num_host_ctx_event_q(0);

    /* Add 1 Host TX Config event queue per host interface defined */
#ifdef ETHERNET_HOST
    add_host_ctx_event_q(&g_eth_host_ctx_event_qid,
                         g_eth_buffer_host_ctx_events);
#endif /* ETHERNET_HOST */



}

/* This function gets the Host TX Config event queue ID for given host type */
INLINE UWORD8 get_host_ctx_event_qid(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return g_eth_host_ctx_event_qid;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return 0;
}

/* This function gets the Host TX Config event queue ID for given host type */
INLINE UWORD8 get_host_type(UWORD8 host_ctx_event_qid)
{
#ifdef ETHERNET_HOST
    if(host_ctx_event_qid == g_eth_host_ctx_event_qid)
        return ETHERNET_HOST_TYPE;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return INVALID_HOST_TYPE;
}

/* This function posts a HOST_RX event on completing reception from the host */
/* The event is processed by the MAC controller and further action taken.    */
INLINE int receive_from_host(UWORD8* rx_buffer, UWORD16 len, UWORD8 offset,
                              UWORD8 host_type)
{
    host_rx_event_msg_t *host_rx = 0;

    /* Free received buffer and event buffer if length exceeds maximum limit */
    if(len > MAX_BUFFER_LEN)
    {
#ifdef DEBUG_MODE
        TROUT_TX_DBG2("SwEr:HostRxLenExc %d > %d\n",len,MAX_BUFFER_LEN);
        g_mac_stats.ehrxexc1++;
#endif /* DEBUG_MODE */
        pkt_mem_free(rx_buffer);
        return -2;
    }

    /* Allocate a buffer for the Host Rx event */
    host_rx = (host_rx_event_msg_t*)event_mem_alloc(HOST_RX_EVENT_QID);
    /* Free received buffer if allocation fails */
    if(host_rx == NULL)
    {
    	TROUT_TX_DBG2("%s: alloc event qid=%d failed!\n", __func__, HOST_RX_EVENT_QID);
    	event_buf_detail_show(HOST_RX_EVENT_QID);
    	mac_event_schedule();	//add by chengwg for notify event process!
        pkt_mem_free(rx_buffer);
        return -1;
    }

    host_rx->buffer_addr  = rx_buffer;
    host_rx->rx_pkt_len   = len;
    host_rx->pkt_ofst     = offset;
    host_rx->host_if_type = host_type;

    /* Post the event */
    post_event((UWORD8*)host_rx, HOST_RX_EVENT_QID);
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.hostrxevent++;
#endif /* MEASURE_PROCESSING_DELAY */

    return 0;
}

/* This function checks whether SNAP header is present in the frame */
INLINE BOOL_T is_snap_header_present(msdu_desc_t *frame_desc)
{
    UWORD8 *data = NULL;

    if(frame_desc->data_len < SNAP_HDR_LEN)
        return BFALSE;

    data = frame_desc->buffer_addr + frame_desc->data_offset;

    if(memcmp(data, g_snap_header, SNAP_HDR_ID_LEN) != 0)
            return BFALSE;

    return BTRUE;
}

/* This function adjusts the frame descriptor data length and offset to      */
/* account for presence of SNAP header                                       */
INLINE void adjust_for_snap_header(msdu_desc_t *frame_desc)
{
    frame_desc->data_len    -= SNAP_HDR_LEN;
    frame_desc->data_offset += SNAP_HDR_LEN;
}

/* This function is used to check if given host type is ready for operation */
INLINE BOOL_T is_host_ready(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return BTRUE;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return BFALSE;
}

/* This function is used to check if given host type is busy */
INLINE BOOL_T is_host_busy(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return BFALSE;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return BTRUE;
}

/* This function is used to get the source ethernet address */
INLINE UWORD8 *get_src_eth_addr(void)
{
#ifdef ETHERNET_HOST
    return g_src_eth_addr;
#endif /* ETHERNET_HOST */




    return 0;
}

/* This function is used to get the configuration packet header length based */
/* on the host type given by the user.                                       */
INLINE UWORD8 get_config_pkt_hdr_len(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return ETH_CONFIG_PKT_HDR_LEN;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return 0;
}

/* This function is used to get the configuration packet header offset based */
/* on the host type given by the user.                                       */
INLINE UWORD8 get_config_pkt_hdr_offset(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return ETH_CONFIG_PKT_HDR_OFFSET;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return 0;
}

/* This function is used to get the total number of pending configuration    */
/* packets based on the host types defined                                   */
INLINE UWORD32 get_hif_cnfg_pkt_tx_q_cnt(void)
{
    UWORD32 temp = 0;

#ifdef ETHERNET_HOST
    temp += get_num_pending_events_in_q(g_eth_host_ctx_event_qid);
#endif /* ETHERNET_HOST */




    return temp;
}

INLINE UWORD32 get_hif_data_pkt_tx_q_cnt(void)
{
    return get_num_pending_events_in_q(HOST_DTX_EVENT_QID);
}

/* This function is used to get the ethernet header offset based on the host */
/* type given by the user.                                                   */
INLINE UWORD8 get_eth_hdr_offset(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return ETH_ETHERNET_HDR_OFFSET;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return 0;
}

/* This function is used to get the host header length based on the host     */
/* type given by the user.                                                   */
INLINE UWORD16 get_host_hdr_len(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return ETHERNET_HDR_LEN;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return 0;
}

/* This function is used to get only the host header length based on the     */
/* host type given by the user.                                              */
INLINE UWORD16 get_host_msg_hdr_len(UWORD8 host_type)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        return 0;
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

    return 0;
}

/* This function is used to set the particular host msg header based on the  */
/* host type given by the user.                                              */
INLINE void set_host_msg_hdr(UWORD8 host_type, UWORD8 *host_hdr,
                         UWORD16 host_data_len)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
    {
        return;
    }
#endif /* ETHERNET_HOST */




}

/* This function is used to set the particular host header based on the host */
/* type given by the user.                                                   */
INLINE void set_host_eth_addr(UWORD8 host_type, UWORD8 *host_hdr,
                        msdu_indicate_t *msdu)
{
#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
    {
        /* Set the ethernet header source and destination address fields.    */
        /* Note that the Ethernet type has already been set.                 */
#ifdef IBSS_BSS_STATION_MODE
        mac_addr_cpy(host_hdr, g_ethernet_addr);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
        mac_addr_cpy(host_hdr, msdu->da);
#endif /* BSS_ACCESS_POINT_MODE */

        mac_addr_cpy(host_hdr + 6, msdu->sa);
        return;
    }
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */
}

/* This function is used to set the particular host header and Ethernet      */
/* address based on the host type given by the user.                         */
INLINE void set_host_hdr(UWORD8 host_type, UWORD8 *host_hdr,
                         UWORD16 host_data_len, msdu_indicate_t *msdu)
{
    set_host_msg_hdr(host_type, host_hdr, host_data_len);
    set_host_eth_addr(host_type, host_hdr, msdu);
}

/* This function is used to send frame to host */
INLINE void send_frame_to_host(UWORD8 host_type, msdu_desc_t *frame_desc)
{
    /* Lock the buffer to be sent to host */
//    lock_the_buffer(frame_desc->buffer_addr);		//shield by chengwg.

#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
    {
        /* Send the pending frame to the host */
        send_packet_to_stack(frame_desc);

		//chenq add for wapi 2012-09-29
		#ifdef MAC_WAPI_SUPP
		if(frame_desc->wapi_mode == 1)
		{
			pkt_mem_free(frame_desc->buffer_addr);
			host_frame_completion_fn(NULL, 0, ETHERNET_HOST_TYPE);
		}
		else
		{
			host_frame_completion_fn(frame_desc->buffer_addr, 0, ETHERNET_HOST_TYPE);
		}
		#else
        host_frame_completion_fn(frame_desc->buffer_addr, 0, ETHERNET_HOST_TYPE);
		#endif
        return;
    }
#endif /* ETHERNET_HOST */

#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */
}

/* This functuion is used to prepare frame format for host interface */
INLINE void prepare_host_data_frame(msdu_indicate_t *msdu,UWORD8  host_type)
{
    msdu_desc_t *frame_desc   = &(msdu->frame_desc);
    UWORD16 host_data_len     = 0;
    UWORD8 *host_hdr          = 0;
    UWORD16 host_hdr_len      = get_host_hdr_len(host_type);

    /* The host header fits into the space available before the payload in   */
    /* the same buffer. The exact host header length and offset is base on   */
    /* the actual host interface in use.                                     */

    /* --------------------------------------------------------------------- */
    /* Buffer format (frame descriptor structure has this information)       */
    /* --------------------------------------------------------------------- */
    /* | <-- Host header offset --> | <-- Host header --> |                | */
    /* ---------------------------------------------------| Data           | */
    /* | <--------------   Data offset   -------------->  |                | */
    /* --------------------------------------------------------------------- */

    frame_desc->host_hdr_len    = host_hdr_len;
    frame_desc->host_hdr_offset = frame_desc->data_offset - host_hdr_len;


    host_hdr      = frame_desc->buffer_addr + frame_desc->host_hdr_offset;

    host_data_len = frame_desc->data_len + ETHERNET_HDR_LEN;

    /* Set the host message length */
    set_host_hdr(host_type, host_hdr,host_data_len,msdu);
}

#ifdef ETHERNET_HOST

/* This function checks if this is a configuration packet for ethernet host */
INLINE BOOL_T is_eth_config_pkt(UWORD8* buffer)
{
    UWORD8  ip_protocol  = 0;
    UWORD16 eth_type     = 0;
    UWORD16 udp_src_port = 0;
    UWORD16 udp_dst_port = 0;
    UWORD8  *eth_hdr     = 0;

    /* Initialize the pointer to the start of the ethernet header in the     */
    /* buffer received from ethernet.                                        */
    eth_hdr = buffer + ETH_ETHERNET_HDR_OFFSET;

    /* Extract the type of the ethernet packet. Extract the ethernet type    */
    /* and the IP protocol. If the packet is a UDP packet, check if the      */
    /* source and destination ports match the values as used by the          */
    /* configuration packets.                                                */
    eth_type    = ((eth_hdr[ETH_PKT_TYPE_OFFSET] << 8) |
                    eth_hdr[ETH_PKT_TYPE_OFFSET + 1]);

    ip_protocol = eth_hdr[IP_PROT_OFFSET];

    udp_src_port = ((eth_hdr[UDP_SRC_PORT_OFFSET] << 8) |
                     eth_hdr[UDP_SRC_PORT_OFFSET + 1]);

    udp_dst_port = ((eth_hdr[UDP_DST_PORT_OFFSET] << 8) |
                     eth_hdr[UDP_DST_PORT_OFFSET + 1]);

    if((eth_type == IP_TYPE) && (ip_protocol == UDP_TYPE))
    {
        if((udp_src_port == HOST_UDP_PORT) && (udp_dst_port == WLAN_UDP_PORT))
        {
            //chenq mod
            //return BTRUE;
            return BFALSE;
        }
    }

    return BFALSE;
}

/* This function prepares the configuration packet for ethernet host */
INLINE UWORD16 prepare_config_pkt_eth_host_hdr(UWORD8 *data, UWORD16 len)
{
    UWORD8  index         = 0;
    UWORD16 src_port      = WLAN_UDP_PORT;
    UWORD16 dest_port     = HOST_UDP_PORT;
    UWORD16 csum          = 0;
    UWORD16 udp_len       = 0;
    UWORD16 ip_len        = 0;
    UWORD16 ret_len       = 0;
    UWORD16 frag_offset   = 0;
    UWORD8  *dest_hw_addr = g_config_eth_addr;
    UWORD8  *src_eth_addr = g_src_eth_addr;
    UWORD8  *src_ip_addr  = g_src_ip_addr;
    UWORD8  *dest_ip_addr = g_config_ip_addr;
    BOOL_T  more_frag     = BFALSE;

    /* Prepare the Ethernet Header - 14 bytes (format is shown below)        */
    /*                                                                       */
    /* +---------------------+-------------------+-------------+             */
    /* | Destination address | Source address    | Packet Type |             */
    /* +---------------------+-------------------+-------------+             */
    /* |  0                  | 6                 |12           |             */
    /* +---------------------+-------------------+-------------+             */

    /* Set the Ethernet destination address and Source address */
    mac_addr_cpy(data, dest_hw_addr);
    mac_addr_cpy(data + 6, src_eth_addr);

    /* Set the type to IP */
    data[12] = (IP_TYPE & 0xFF00) >> 8;
    data[13] = (IP_TYPE & 0xFF);

    /* Prepare the IP Header - 20 bytes (format is shown below)              */
    /*                                                                       */
    /* --------------------------------------------------------------------- */
    /* | Version | IHL | TOS | Total length | Identification | Flags         */
    /* --------------------------------------------------------------------- */
    /*  0               1      2              4                6             */
    /* --------------------------------------------------------------------- */
    /* | Fragment offset | TTL | Protocol   | Header checksum |              */
    /* --------------------------------------------------------------------- */
    /*  7                  8     9           10                              */
    /* --------------------------------------------------------------------- */
    /* | Source IP address            | Destination IP address             | */
    /* --------------------------------------------------------------------- */
    /*  12                              16                                   */
    /* --------------------------------------------------------------------- */

    index = IP_HDR_OFFSET;

    if(frag_offset == 0)
    {
        ip_len = len + UDP_HDR_LEN + IP_HDR_LEN;
    }
    else
    {
        ip_len = len + IP_HDR_LEN;
    }

    data[index]     = 0x45;
    data[index + 1] = 0x00;
    data[index + 2] = (ip_len & 0xFF00) >> 8;
    data[index + 3] = (ip_len & 0xFF);
    data[index + 4] = 0x00;
    data[index + 5] = 0x00;

    /* Flags & Fragment Offset */
    if(more_frag == BTRUE)
    {
        data[index + 6] = 0x20;
    }
    else
    {
        data[index + 6] = 0x00;
    }

    data[index + 6] |= (frag_offset >> 11) & 0xE0;
    data[index + 7]  = (frag_offset >> 3) & 0xFF;

    /* TTL */
    data[index + 8] = 0x80;

    /* Set the IP protocol to UDP */
    data[index + 9] = UDP_TYPE;

    /* Set the checksum to zero */
    data[index + 10] = 0;
    data[index + 11] = 0;

    index += 12;

    /* Set the IP source address to some value with different subnet */
    ip_addr_cpy(data + index, src_ip_addr);
    index += 4;

    /* Set the IP destination address */
    ip_addr_cpy(data + index, dest_ip_addr);
    index += 4;

    /* Compute the checksum and set the field */
    csum = checksum(&data[IP_HDR_OFFSET], IP_HDR_LEN);
    data[IP_HDR_OFFSET + 11] = (csum & 0xFF00) >> 8;
    data[IP_HDR_OFFSET + 10] = (csum & 0xFF);

    if((frag_offset == 0) && (more_frag == BFALSE))
    {
        /* Prepare the UDP Header - 8 bytes (format is shown below) */
        /*                                                          */
        /* +--------------+--------------+--------+----------+      */
        /* | UDP SRC Port | UDP DST Port | Length | Checksum |      */
        /* +--------------+--------------+--------+----------+      */
        /* |  0           | 2            |4       | 6        |      */
        /* +--------------+--------------+--------+----------+      */

        index = UDP_HDR_OFFSET;

        /* Set the UDP source and destination ports */
        data[index]     = (src_port & 0xFF00) >> 8;
        data[index + 1] = (src_port & 0xFF);
        data[index + 2] = (dest_port & 0xFF00) >> 8;
        data[index + 3] = (dest_port & 0xFF);
        index += 4;

        /* Set the length to the UDP data + UDP header */
        udp_len         = len + UDP_HDR_LEN;
        data[index]     = (udp_len & 0xFF00) >> 8;
        data[index + 1] = (udp_len & 0xFF);
        index += 2;

        /* Set the checksum to zero - currently not set as it is not mandatory */
        data[index] = 0;
        data[index + 1] = 0;

        ret_len = (len + ETH_CONFIG_PKT_HDR_LEN);
    }
    else
    {
        ret_len = (len + ETH_CONFIG_PKT_HDR_LEN - UDP_HDR_OFFSET);
    }

    return ret_len;
}

#endif /* ETHERNET_HOST */

#ifdef MAC_802_1X
#ifndef ENABLE_AP_1X_LINUX_PROCESSING
/* This function prepares the configuration packet for 1x packets */
INLINE UWORD16 prepare_1x_config_pkt(UWORD8 *data, UWORD8 temp, UWORD16 len,
                        UWORD8 *g_server_eth_addr,
                        UWORD8 *g_auth_eth_addr,
                        UWORD8 *g_local_ip_addr,
                        UWORD8 *g_server_ip_addr,
                        UWORD16 g_local_port,
                        UWORD16 g_server_port,
                        BOOL_T more_frag, UWORD16 frag_offset)
{
    UWORD8  index         = 0;
    UWORD16 src_port      = g_local_port;
    UWORD16 dest_port     = g_server_port;
    UWORD16 csum          = 0;
    UWORD16 udp_len       = 0;
    UWORD16 ip_len        = 0;
    UWORD16 ret_len       = 0;
    UWORD8  *src_eth_addr = g_auth_eth_addr;
    UWORD8  *dest_hw_addr = g_server_eth_addr;
    UWORD8  *src_ip_addr  = g_src_ip_addr;
    UWORD8  *dest_ip_addr = g_server_ip_addr;

    /* Prepare the Ethernet Header - 14 bytes (format is shown below)        */
    /*                                                                       */
    /* +---------------------+-------------------+-------------+             */
    /* | Destination address | Source address    | Packet Type |             */
    /* +---------------------+-------------------+-------------+             */
    /* |  0                  | 6                 |12           |             */
    /* +---------------------+-------------------+-------------+             */

    /* Set the Ethernet destination address and Source address */
    mac_addr_cpy(data, dest_hw_addr);
    mac_addr_cpy(data + 6, src_eth_addr);

    /* Set the type to IP */
    data[12] = (IP_TYPE & 0xFF00) >> 8;
    data[13] = (IP_TYPE & 0xFF);

    /* Prepare the IP Header - 20 bytes (format is shown below)              */
    /*                                                                       */
    /* --------------------------------------------------------------------- */
    /* | Version | IHL | TOS | Total length | Identification | Flags         */
    /* --------------------------------------------------------------------- */
    /*  0               1      2              4                6             */
    /* --------------------------------------------------------------------- */
    /* | Fragment offset | TTL | Protocol   | Header checksum |              */
    /* --------------------------------------------------------------------- */
    /*  7                  8     9           10                              */
    /* --------------------------------------------------------------------- */
    /* | Source IP address            | Destination IP address             | */
    /* --------------------------------------------------------------------- */
    /*  12                              16                                   */
    /* --------------------------------------------------------------------- */

    index = IP_HDR_OFFSET;

    if(frag_offset == 0)
    //if(more_frag == BFALSE)
    {
        ip_len = len + UDP_HDR_LEN + IP_HDR_LEN;
    }
    else
    {
        ip_len = len + IP_HDR_LEN;
    }

    data[index]     = 0x45;
    data[index + 1] = 0x00;
    data[index + 2] = (ip_len & 0xFF00) >> 8;
    data[index + 3] = (ip_len & 0xFF);
    data[index + 4] = 0x00;
    data[index + 5] = 0x00;

    /* Flags & Fragment Offset */
    if(more_frag == BTRUE)
    {
        data[index + 6] = 0x20;
    }
    else
    {
        data[index + 6] = 0x00;
    }

    data[index + 7]  = frag_offset;

    /* TTL */
    data[index + 8] = 0x80;

    /* Set the IP protocol to UDP */
    data[index + 9] = UDP_TYPE;

    /* Set the checksum to zero */
    data[index + 10] = 0;
    data[index + 11] = 0;

    index += 12;

    /* Set the IP source address to some value with different subnet */
    ip_addr_cpy(data + index, src_ip_addr);
    index += 4;

    /* Set the IP destination address */
    ip_addr_cpy(data + index, dest_ip_addr);
    index += 4;

    /* Compute the checksum and set the field */
    csum = checksum(&data[IP_HDR_OFFSET], IP_HDR_LEN);
    data[IP_HDR_OFFSET + 11] = (csum & 0xFF00) >> 8;
    data[IP_HDR_OFFSET + 10] = (csum & 0xFF);

    if((frag_offset == 0) && (more_frag == BFALSE))
    //if((frag_offset == 0))
    {
        /* Prepare the UDP Header - 8 bytes (format is shown below) */
        /*                                                          */
        /* +--------------+--------------+--------+----------+      */
        /* | UDP SRC Port | UDP DST Port | Length | Checksum |      */
        /* +--------------+--------------+--------+----------+      */
        /* |  0           | 2            |4       | 6        |      */
        /* +--------------+--------------+--------+----------+      */

        index = UDP_HDR_OFFSET;

        /* Set the UDP source and destination ports */
        data[index]     = (src_port & 0xFF00) >> 8;
        data[index + 1] = (src_port & 0xFF);
        data[index + 2] = (dest_port & 0xFF00) >> 8;
        data[index + 3] = (dest_port & 0xFF);
        index += 4;

        /* Set the length to the UDP data + UDP header */
        udp_len         = len + UDP_HDR_LEN;
        data[index]     = (udp_len & 0xFF00) >> 8;
        data[index + 1] = (udp_len & 0xFF);
        index += 2;

        /* Set the checksum to zero - currently not set as it is not mandatory */
        data[index] = 0;
        data[index + 1] = 0;

        ret_len = (len + ETH_CONFIG_PKT_HDR_LEN);
    }
    else
    {
        //ret_len = (len + ETH_CONFIG_PKT_HDR_LEN - UDP_HDR_LEN);
        ret_len = (len + ETH_CONFIG_PKT_HDR_LEN - UDP_HDR_OFFSET);
    }

    return ret_len;
}
#endif /* ENABLE_AP_1X_LINUX_PROCESSING */
#endif /* MAC_802_1X */

/* This function checks whether any of the many host interfaces supported */
/* is currently busy.                                                     */
INLINE BOOL_T is_any_host_busy(void)
{
    UWORD8 i = 0;

    /* Parse through all the defined host interfaces by checking all active  */
    /* Host TX Config event queues. 1 event queue is present for each host   */
    /* interface defined                                                     */
    for(i = HOST_CTX_EVENT_QID_INIT; i < get_num_event_q(); i++)
    {
        UWORD8 host_type = get_host_type(i);

        if(INVALID_HOST_TYPE == host_type)
            continue;

        if(is_host_busy(host_type) == BTRUE)
        {
            return BTRUE;
        }
    }

    return BFALSE;
}

/* This function checks whether the host interface is valid as a Data Host */
/* Interface.                                                              */
INLINE BOOL_T is_valid_data_hif(UWORD8 host_type)
{
    BOOL_T retval = BFALSE;

#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        retval = BTRUE;
#endif /* ETHERNET_HOST */




    return retval;
}

/* This function checks whether the host interface is valid as a Config Host */
/* Interface.                                                                */
INLINE BOOL_T is_valid_cfg_hif(UWORD8 host_type)
{
    BOOL_T retval = BFALSE;

#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
        retval = BTRUE;
#endif /* ETHERNET_HOST */




    return retval;
}

/* This function handles all host interface specific flow control tasks */
INLINE void handle_hif_flow_control(void)
{
#ifdef ETHERNET_HOST
    /* No Flow control currently defined for this host interface */
#endif /* ETHERNET_HOST */
}

INLINE void tx_completion_service(void)
{
}

INLINE void disable_hif_interrupts(void)
{

}

INLINE void enable_hif_interrupts(void)
{


}

#endif /* HOST_IF_H */
