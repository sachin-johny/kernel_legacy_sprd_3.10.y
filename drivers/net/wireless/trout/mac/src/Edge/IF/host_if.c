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
/*  File Name         : host_if.c                                            */
/*                                                                           */
/*  Description       : This file contains functions required to interface   */
/*                      with the host driver in use.                         */
/*                                                                           */
/*  List of Functions : init_host_interface                                  */
/*                      reset_host_interface                                 */
/*                      soft_reset_host_if                                   */
/*                      send_data_frame_to_host_if                           */
/*                      send_config_frame_to_host_if                         */
/*                      send_pending_frames_to_host                          */
/*                      is_config_pkt                                        */
/*                      prepare_config_pkt                                   */
/*                      host_frame_completion_fn                             */
/*                      checksum                                             */
/*                      wait_for_host_idle                                   */
/*                      remove_all_in_hifq                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "controller_mode_if.h"
#include "host_if.h"
#include "mh.h"
#include "iconfig.h"
#include "trout_wifi_rx.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8   g_num_host_types               = 0;
UWORD8   g_host_data_if_type            = 0;
UWORD8   g_host_config_if_type          = 0;
UWORD8   g_src_ip_addr[4]               = {192, 168, 20, 221};
UWORD8   g_src_netmask_addr[6]          = {0};
UWORD8   g_snap_header[SNAP_HDR_ID_LEN] = {0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00};

#ifdef ETHERNET_HOST
UWORD8   g_config_eth_addr[6]        = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
UWORD8   g_config_ip_addr[4]         = {0xFF, 0xFF, 0xFF, 0xFF};
UWORD8   g_src_eth_addr[6]           = {0x00, 0x50, 0xC2, 0x5E, 0x10, 0x82};
UWORD32  g_eth_host_ctx_event_qid    = 0;
UWORD8   g_eth_buffer_host_ctx_events[TOTAL_MEM_SIZE_HOST_CTX_EVENTS] = {0};
#endif /* ETHERNET_HOST */




/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_host_interface                                      */
/*                                                                           */
/*  Description   : This function initializes the host interfaces defined.   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_transmit_to_host_q                                     */
/*                  g_sdio_host_busy                                         */
/*                  g_uart_host_busy                                         */
/*                                                                           */
/*  Processing    : The global transmit to host data queue is initialized.   */
/*                  For the various host types defined, the host interface   */
/*                  and related globals are initialized. The default host    */
/*                  type is set to a particular host type if it is defined.  */
/*                  In case of multiple hosts the host type will be set to   */
/*                  the last host type that is initialized. The number of    */
/*                  host types global is incremented with initialization of  */
/*                  each different host type.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_host_interface(void)
{
	TROUT_FUNC_ENTER;
#ifdef ETHERNET_HOST
    /* Increment the number of host types global */
    g_num_host_types++;
#endif /* ETHERNET_HOST */


    /* Select the default host interface */
    select_default_hif();
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : select_default_hif                                       */
/*                                                                           */
/*  Description   : This function selects the deafult host interface for     */
/*                  data and configuration paths.                            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Following order of priority is used while choosing the   */
/*                  default host interfaces:                                 */
/*                  1. Ethernet                                              */
/*                  2. USB                                                   */
/*                  3. SDIO                                                  */
/*                  4. UART                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void select_default_hif(void)
{
#ifndef GENERIC_HOST
    UWORD8 hif_selected = 0;
#endif /* GENERIC_HOST */

#ifdef ETHERNET_HOST
    if(hif_selected == 0)
    {
        /* Set default host interface type to ETHERNET host */
        set_host_data_if_type(ETHERNET_HOST_TYPE);
        set_host_config_if_type(ETHERNET_HOST_TYPE);

        hif_selected = 1;
    }
#endif /* ETHERNET_HOST */



}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_host_interface                                     */
/*                                                                           */
/*  Description   : This function resets the host interfaces in use.         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the required host reset functions    */
/*                  for all the host interfaces defined.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reset_host_interface(void)
{
#ifdef ETHERNET_HOST
    /* Do nothing */
#endif /* ETHERNET_HOST */



}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : soft_reset_host_if                                       */
/*                                                                           */
/*  Description   : This function resets the software part of the host       */
/*                  interfaces in use.                                       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the required host reset functions    */
/*                  for all the host interfaces defined.                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void soft_reset_host_if(void)
{
#ifdef ETHERNET_HOST
    /* Do nothing */
#endif /* ETHERNET_HOST */



}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_data_frame_to_host_if                               */
/*                                                                           */
/*  Description   : This function sends a data frame to the host interface.  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MSDU descriptor structure              */
/*                  2) Data host interface type                              */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function queues the MSDU descriptor structure in    */
/*                  global transmit to host data queue. If the insertion     */
/*                  fails the frame descriptor is freed. On successful       */
/*                  insertion the global host interface queue count is       */
/*                  incremented. A function is then called to send pending   */
/*                  data frames to the host interface.                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_data_frame_to_host_if(msdu_desc_t *fdesc, UWORD8 host_type)
{
    host_tx_event_msg_t *host_tx =
            (host_tx_event_msg_t *)event_mem_alloc(HOST_DTX_EVENT_QID);
	TROUT_FUNC_ENTER;
	
    if(host_tx == NULL)
    {
    		printk("%s: event alloc fail!\n", __func__);
#ifdef TROUT_WIFI
		//chenq add for wapi 2012-09-29
#ifdef MAC_WAPI_SUPP
		if(fdesc->wapi_mode == 1)
		{
			pkt_mem_free(fdesc->buffer_addr);	
		}
#endif	/* MAC_WAPI_SUPP */
#else
        pkt_mem_free(fdesc->buffer_addr);
#endif	/* TROUT_WIFI */
		TROUT_FUNC_EXIT;
        return;
    }

    /* Update the Host TX event with  frame details */
    host_tx->host_type = host_type;
    memcpy((UWORD8 *)(&(host_tx->fdesc)), (UWORD8 *)fdesc, sizeof(msdu_desc_t));

    //post_event((UWORD8 *)host_tx, HOST_DTX_EVENT_QID);

    print_log_debug_level_1("\n[DL1][INFO][Rx] {Frame queued for Tx to Host}");

#ifdef DEBUG_MODE
    g_mac_stats.hifq++;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.nummsduhiftxq++;
#endif /* MEASURE_PROCESSING_DELAY */

    /* Schedule the processing of the Host TX Data event queue */
    //process_host_event_q(HOST_DTX_EVENT_QID);
    process_host_tx_event((UWORD32)host_tx);	//modify by chengwg.
    event_mem_free((void *)host_tx, HOST_DTX_EVENT_QID);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_config_frame_to_host_if                             */
/*                                                                           */
/*  Description   : This function sends a configuration frame to the host    */
/*                  interface specified by the user.                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MSDU descriptor structure              */
/*                  2) Host type                                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function queues the MSDU descriptor in the global   */
/*                  transmit to host queue for the particular host type. If  */
/*                  the insertion fails the frame descriptor is freed. On    */
/*                  successful insertion the global host interface queue     */
/*                  count for the host type is incremented. A function is    */
/*                  then called to send pending configuration frames to the  */
/*                  host interface.                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_config_frame_to_host_if(msdu_desc_t *frame_desc, UWORD8 host_type)
{
    host_tx_event_msg_t *host_tx = NULL;
    UWORD8 host_ctx_event_qid = 0;

    host_ctx_event_qid = get_host_ctx_event_qid(host_type);

    host_tx = (host_tx_event_msg_t *)event_mem_alloc(host_ctx_event_qid);

    if(host_tx == NULL)
    {
        pkt_mem_free(frame_desc->buffer_addr);
        return;
    }

    /* Update the Host TX event with  frame details */
    host_tx->host_type = host_type;
    memcpy((UWORD8 *)(&(host_tx->fdesc)), (UWORD8 *)frame_desc, sizeof(msdu_desc_t));

    post_event((UWORD8 *)host_tx, host_ctx_event_qid);

    print_log_debug_level_1("\n[DL1][INFO][Rx] {Frame queued for Tx to Host}");

#ifdef DEBUG_MODE
    g_mac_stats.hifq++;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.nummsduhiftxq++;
#endif /* MEASURE_PROCESSING_DELAY */

    /* Schedule the processing of this Host TX Config event queue */
	//chenq mask 2012-10-30
	//process_host_event_q(host_ctx_event_qid);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_pending_frames_to_host                              */
/*                                                                           */
/*  Description   : This function sends all pending frames to the host       */
/*                  interfaces.                                              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls functions to send pending data       */
/*                  frames to the current configured data host interface and */
/*                  configuration packets to all the host interfaces active. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
extern void mac_event_schedule(void);

void send_pending_frames_to_host(void)
{
	//chenq mod 2012-10-30
	#if 0
    UWORD8 i = 0;

    /* Process all Host TX event queues (Host TX Data Event queue and all    */
    /* the Host TX Config event queues, 1 per Host Type defined)             */
    for(i = HOST_DTX_EVENT_QID; i < get_num_event_q(); i++)
    {
        process_host_event_q(i);
    }
	#else
	mac_event_schedule();
	#endif
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : is_config_pkt                                            */
/*                                                                           */
/*  Description   : This function determines if the incoming packet is a     */
/*                  configuration packet for the given host interface type.  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the incoming host frame buffer             */
/*                  2) Host type                                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Different checks are done on the host header based on    */
/*                  the host type to determine if the packet is a            */
/*                  configuration message.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BTRUE, if the packet is a configuration message          */
/*                  BFALSE, otherwise                                        */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

BOOL_T is_config_pkt(UWORD8* buffer,UWORD8 host_if_type)
{
#ifdef ETHERNET_HOST
    if(host_if_type == ETHERNET_HOST_TYPE)
    {
        return is_eth_config_pkt(buffer);
    }
#endif /* ETHERNET_HOST */




#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */
    return BFALSE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_config_pkt_hdr                                   */
/*                                                                           */
/*  Description   : This function prepares a configuration packet as per the */
/*                  host type specified by the user.                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the configuration message buffer           */
/*                  2) Length of the host response packet                    */
/*                  3) Host type                                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Configuration packets are prepared based on the host     */
/*                  type and the total configuration packet length is        */
/*                  returned.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD16, length of the configuration message             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_config_pkt_hdr(UWORD8 *data, UWORD16 len, UWORD8 host_type)
{
    UWORD16 ret_len = 0;

#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
    {
        ret_len = prepare_config_pkt_eth_host_hdr(data, len);
    }
#endif /* ETHERNET_HOST */




    if(ret_len == 0)
    {
        /* Return length zero indicates host type is invalid. Do nothing */
#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */
    }

    return ret_len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_log_pkt                                          */
/*                                                                           */
/*  Description   : This function prepares a log packet                      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the log message buffer                     */
/*                  2) Pointer to the log packet                             */
/*                  3) Length of the log packet                              */
/*                  4) Host type                                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Log packets are prepared and packet length is returned.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD16, length of the configuration message             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 prepare_log_pkt(UWORD8 *data, UWORD8 *in_data, UWORD16 len,
                        UWORD8 host_type)
{
    UWORD16 ret_len = 0;

#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
    {
        ret_len = prepare_config_pkt_eth_host_hdr(data, len);
    }
#endif /* ETHERNET_HOST */




    if(ret_len == 0)
    {
        /* Return length zero indicates host type is invalid. Do nothing */
#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */
    }
    else
    {
        /* Copy the UDP data and free the incoming data buffer */
        if(in_data != NULL)
        {
            memcpy(data + get_config_pkt_hdr_len(host_type), in_data, len);
            pkt_mem_free(in_data);
        }
    }

    return ret_len;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : host_frame_completion_fn                                 */
/*                                                                           */
/*  Description   : This function is called on completing transmission to    */
/*                  host.                                                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer                                 */
/*                  2) Status                                                */
/*                  3) Host type                                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The buffer allocated to the frame header and the frame   */
/*                  fragments are freed. Host busy flags are reset for host  */
/*                  interfaces as required.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void host_frame_completion_fn(void* data, UWORD32 status,UWORD8 host_type)
{
    UWORD8 *buffer_addr = (UWORD8 *)data;

	TROUT_FUNC_ENTER;
	
//    unlock_the_buffer(buffer_addr);	//shield by chengwg.
#ifdef TROUT_WIFI	//modified by chengwg.
#else
	pkt_mem_free(buffer_addr);
#endif

    tx_completion_service();

    print_log_debug_level_1("\n[DL1][INFO][Rx] {Frame Tx to host complete}");
#ifdef DEBUG_MODE
    g_mac_stats.lsphc++;
#endif /* DEBUG_MODE */

#ifdef ETHERNET_HOST
    if(host_type == ETHERNET_HOST_TYPE)
    {
        /* Do nothing */
        TROUT_FUNC_EXIT;
        return;
    }
#endif /* ETHERNET_HOST */

#ifdef DEBUG_MODE
    g_mac_stats.hifinvldtype++;
#endif /* DEBUG_MODE */

	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : remove_all_in_hifq                                       */
/*                                                                           */
/*  Description   : This deletes all frames currently buffered in the        */
/*                  different host interface queues.                         */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All valid host interface queues are parsed and all       */
/*                  existing buffers in them are deleted.                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void remove_all_in_hifq(void)
{
    UWORD8 i = 0;
    
	TROUT_FUNC_ENTER;
    /* For all the Host TX event queues (Host TX Data Event queue and all    */
    /* the Host TX Config event queues, 1 per Host Type defined), remove all */
    /* the events and free the buffers.                                      */
    for(i = HOST_DTX_EVENT_QID; i < get_num_event_q(); i++)
    {
        event_q_remove_all_host_tx_events(i);
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : wait_for_host_idle                                       */
/*                                                                           */
/*  Description   : This function waits till all the Host-Interfaces are not */
/*                  busy.                                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All valid host interface queues are checked and the      */
/*                  function blocks until the presently on-going             */
/*                  transmission is complete.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void wait_for_host_idle(void)
{
    UWORD32 wait_cnt = 0;
    UWORD8  i        = 0;

	TROUT_FUNC_ENTER;
    /* Parse through all the defined host interfaces by checking all active  */
    /* Host TX Config event queues. 1 event queue is present for each host   */
    /* interface defined                                                     */
    for(i = HOST_CTX_EVENT_QID_INIT; i < get_num_event_q(); i++)
    {
        UWORD8 host_type = get_host_type(i);

        if(INVALID_HOST_TYPE == host_type)
            continue;

        wait_cnt = 0;

        /* Wait for the current trasmission for this host to be complete */
        while((is_host_busy(host_type) == BTRUE) &&
              (wait_cnt < HOST_BUSY_TIME_OUT_COUNT))
        {
            add_delay(0xFFF);
            wait_cnt++;
        }
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : checksum                                                 */
/*                                                                           */
/*  Description   : This function computes the 16-bit CRC.                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the buffer                                 */
/*                  2) Length of the buffer                                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function computes 16-bit CRC for the given packet.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : 16-bit CRC checksum                                      */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 checksum(UWORD8 *buffer, UWORD32 size)
{
    UWORD32 cksum = 0;

    while (size > 1)
    {
        cksum += *buffer + (*(buffer+1)  << 8);
        buffer += 2 ;
        size -= 2;
    }

    if (size)
    {
        cksum += *buffer;
    }

    cksum = (cksum >> 16) + (cksum & 0xFFFF);
    cksum += (cksum >> 16);

    return (UWORD16)(~cksum);
}

void send_enabled_msg_to_host(void)
{
    /*  Notice device ready message to host using I-frame  */

#ifdef ETHERNET_HOST
    /* Do nothing */
#endif /* ETHERNET_HOST */


}

