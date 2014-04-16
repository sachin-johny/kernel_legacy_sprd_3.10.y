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
/*  File Name         : qmu.c                                                */
/*                                                                           */
/*  Description       : This file contains all the queue manager related     */
/*                      functions.                                           */
/*                                                                           */
/*  List of Functions : qmu_init                                             */
/*                      qmu_close                                            */
/*                      qmu_flush_tx_queue                                   */
/*                      qmu_flush_rx_queue                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "imem_if.h"
#include "qmu_if.h"
#include "prot_if.h"
#include "receive.h"
#include "trout_wifi_rx.h"


/*****************************************************************************/
/* Global Variable Declarations                                              */
/*****************************************************************************/
// 20120709 caisf add, merged ittiam mac v1.2 code
UWORD8 g_is_suspend_all_data_qs = 0;

#ifdef TX_PKT_USE_DMA
tx_dma_handle_t g_tx_dma_handle;	//add by chengwg.
#endif
/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/
extern void flush_tx_dscr_head(void);		//add by chengwg.

static q_struct_t g_tx_header[NUM_TX_QUEUE];

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_init                                                 */
/*                                                                           */
/*  Description   : This function performs all the queue manager related     */
/*                  initialization operations                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the queue handle                           */
/*                  2) Number of transmit queues                             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Pointers to the queue header and queue structures are    */
/*                  created. Memory is allocated for the local cache and the */
/*                  packet descriptors                                       */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : QMU_OK on success                                        */
/*                  Failure code on failure                                  */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 qmu_init(qmu_handle_t *q_handle, UWORD16 num_tx_q)
{
    UWORD16         i          = 0;
    qmu_tx_handle_t *tx_handle = 0;
    UWORD32 trout_addr = 0;
    //UWORD32 temp32[3];
	TROUT_FUNC_ENTER;

    /* Initialize the number of transmit queues */
    q_handle->num_tx_q = num_tx_q;

    /* Initialize the transmit queue handle */
    tx_handle = &(q_handle->tx_handle);

    /* Initialize the total number of Tx buffers to zero */
    tx_handle->tx_list_count = 0;
	
	trout_addr = g_normal_rxq_num * sizeof(trout_rx_mem_struct) + NORMAL_RX_MEM_BEGIN;
	if(trout_addr >= SHARE_MEM_END)
	{
		TROUT_DBG1("trout rx mem is so large, thus tx has no mem!\n");
		return -1;
	}
	
	tx_handle->tx_mem_start = trout_addr;
	tx_handle->tx_mem_size = 
		(UWORD32)COEX_SELF_CTS_NULL_DATA_BEGIN - (UWORD32)COEX_SLOT_INFO_SIZE - trout_addr ;
	
	if(tx_handle->tx_mem_size < MIN_TX_MEM_SIZE)
		TROUT_DBG2("Warning: trout tx mem size is too small(%u)!\n", tx_handle->tx_mem_size);
	tx_handle->tx_mem_end = tx_handle->tx_mem_start + tx_handle->tx_mem_size;

	TROUT_DBG4("txq init: tx_mem_start=0x%x, tx_mem_end=0x%x, size=%u\n", 
				tx_handle->tx_mem_start, tx_handle->tx_mem_end, tx_handle->tx_mem_size);
	
    /* Initialize the Transmit queue handle header list to the global list   */
    /* of transmit queue headers                                             */
    tx_handle->tx_header = g_tx_header;
    
    mutex_init(&tx_handle->txq_lock); //add by Hugh
    mutex_init(&tx_handle->hwtx_mutex); //add by zhao
	tx_handle->cur_slot = 0;			//add by zhao
    /* Reset the MAC H/w Q header registers for all the queues */
    
    mutex_lock(&tx_handle->txq_lock);
    for(i = 0; i < num_tx_q; i++)
    {
        tx_handle->tx_header[i].q_status     = Q_VALID;
        tx_handle->tx_header[i].element_cnt  = 0;
        tx_handle->tx_header[i].element_head = NULL;
        tx_handle->tx_header[i].element_tail = NULL;
        tx_handle->tx_header[i].element_to_load = NULL; //Hugh add
//        tx_handle->tx_header[i].trout_head = 0; //Hugh add
//        tx_handle->tx_header[i].handle_pkt = 0;	//chengwg add.
        set_machw_q_ptr((UWORD8)i, NULL);
    }

	/*hugh: init first tx dscr's status, make the status is invalid */
    //temp32[2] = 0;
    //host_write_trout_ram((UWORD32 *)TX_MEM_BEGIN, &temp32[2], 4);
    //flush_tx_dscr_head();

    mutex_unlock(&tx_handle->txq_lock);
    
	TROUT_FUNC_EXIT;
    return QMU_OK;
}


#ifdef TX_PKT_USE_DMA
int tx_dma_buf_init(void)
{
	UWORD32 tx_mem_size = g_q_handle.tx_handle.tx_mem_size;
	
	if(g_tx_dma_handle.tx_dma_buf != NULL)
		return 0;
	if(tx_mem_size < MIN_TX_MEM_SIZE)
	{
		TROUT_TX_DBG1("Warning: mac tx mem is too small(%u)!\n", tx_mem_size);
	}

	//g_tx_dma_handle.tx_dma_buf = (UWORD8 *)kmalloc(TX_MEM_SIZE + SPI_SDIO_WRITE_RAM_CMD_WIDTH + 4, GFP_KERNEL);
	g_tx_dma_handle.tx_dma_buf = (UWORD8 *)kmalloc(tx_mem_size + SPI_SDIO_WRITE_RAM_CMD_WIDTH + 4, GFP_KERNEL);
	if(g_tx_dma_handle.tx_dma_buf == NULL)
	{
		TROUT_TX_DBG1("alloc tx dma buffer failed!\n");
		return (-ENOMEM);
	}
	
	mutex_init(&g_tx_dma_handle.tx_dma_lock);
	
	return 0;
}

void tx_dma_buf_free(void)
{
	TROUT_TX_DBG3("free tx dma buffer!\n");
	if(g_tx_dma_handle.tx_dma_buf != NULL)
	{
		kfree(g_tx_dma_handle.tx_dma_buf);
		g_tx_dma_handle.tx_dma_buf = NULL;
	}
}
#endif	/* TX_PKT_USE_DMA */
/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_close                                                */
/*                                                                           */
/*  Description   : This function frees all the memory allocated by the      */
/*                  queue manager                                            */
/*                                                                           */
/*  Inputs        : 1) Pointer to the queue handle                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All the queues are flushed and the memory allocated to   */
/*                  the packet descriptors and local cache is freed.         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : QMU_OK on success                                        */
/*                  Failure code on failure                                  */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifdef TROUT_WIFI
//add by Hugh
void qmu_close(qmu_handle_t *q_handle)
{
    UWORD8 temp = 0;

	TROUT_FUNC_ENTER;
#ifdef BURST_TX_MODE
    /* When Burst_tx mode is enabled, qmu_closing is not required */
    if(g_burst_tx_mode_enabled_earlier == BTRUE)
    {
        set_machw_q_ptr(0, NULL);
        set_machw_q_ptr(1, NULL);
        set_machw_q_ptr(2, NULL);
        TROUT_FUNC_EXIT;
        return;
    }
#endif /* BURST_TX_MODE */

    /* Flush all transmit queues */
    for(temp = 0; temp < NUM_TX_QUEUE; temp++)
    {
        /* Write null to the MAC H/w Q header register */
        set_machw_q_ptr(temp, NULL);
    }

    /* Flush all receive queues */
    for(temp = 0; temp < NUM_RX_Q; temp++)
    {
        /* Reset the H/w buffer pointers */
        set_machw_rx_buff_addr(0, temp);
    }
    TROUT_FUNC_EXIT;
}
#else
void qmu_close(qmu_handle_t *q_handle)
{
    UWORD8 temp = 0;

#ifdef BURST_TX_MODE
    /* When Burst_tx mode is enabled, qmu_closing is not required */
    if(g_burst_tx_mode_enabled_earlier == BTRUE)
    {
        set_machw_q_ptr(0, NULL);
        set_machw_q_ptr(1, NULL);
        set_machw_q_ptr(2, NULL);
        return;
    }
#endif /* BURST_TX_MODE */

    /* Flush all transmit queues */
    for(temp = 0; temp < q_handle->num_tx_q; temp++)
    {
        qmu_flush_tx_queue(q_handle, temp);
    }

    /* Flush all receive queues */
    for(temp = 0; temp < NUM_RX_Q; temp++)
    {
        qmu_flush_rx_queue(q_handle, temp);
    }
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_flush_tx_queue                                       */
/*                                                                           */
/*  Description   : This function flushes a specified queue                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the queue handle                           */
/*                  2) Queue number to be flushed                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Updates the header for the new queue. Then the queue is  */
/*                  scanned for all the packets and all the packets are      */
/*                  deleted. The local cache is also updated                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Returns       : QMU_OK on success else a failure code on failure         */
/*                                                                           */
/*****************************************************************************/

UWORD8 qmu_flush_tx_queue(qmu_handle_t *q_handle, UWORD8 q_num)
{
    UWORD32    *element_ptr = 0;
    UWORD32    *ptr_next    = 0;
    q_struct_t *tx_struct   = 0;

    /* If given queue number exceeds the number of active transmit queues,   */
    /* do nothing and return appropriate status                              */
    if(q_num >= get_no_tx_queues())
    {
        return QMU_Q_INACTIVE;
    }

    tx_struct = &(q_handle->tx_handle.tx_header[q_num]);

    mutex_lock(&q_handle->tx_handle.txq_lock);  //Hugh
    /* If element count is zero, Q is already empty */
    if(tx_struct->element_cnt == 0)
    {
        /* If the Q is empty return success status */
        mutex_unlock(&q_handle->tx_handle.txq_lock);  //Hugh
        return QMU_OK;
    }

    /* Reset the element count for the current Q */
    tx_struct->element_cnt = 0;

    /* Write null to the MAC H/w Q header register */
    set_machw_q_ptr(q_num, NULL);

    /* Empty the queue */
    element_ptr = tx_struct->element_head;

    while(element_ptr != NULL)
    {
        ptr_next = (UWORD32 *)get_tx_dscr_next_addr(element_ptr);
        free_tx_dscr(element_ptr);
        element_ptr = ptr_next;
    }

    /* Reset the pointers to first and last element of the Q */
    tx_struct->element_tail = NULL;
    tx_struct->element_head = NULL;
    tx_struct->element_to_load = NULL;
//    tx_struct->trout_head = 0; //Hugh add

    mutex_unlock(&q_handle->tx_handle.txq_lock);  //Hugh

    return QMU_OK;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_flush_rx_queue                                       */
/*                                                                           */
/*  Description   : This function flushes a specified queue                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the queue handle                           */
/*                  2) Queue number to be flushed                            */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Updates the header for the new queue. Then the queue is  */
/*                  scanned for all the packets and all the packets are      */
/*                  deleted. The local cache is also updated                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Returns       : QMU_OK on success else a failure code on failure         */
/*                                                                           */
/*****************************************************************************/

UWORD8 qmu_flush_rx_queue(qmu_handle_t *q_handle, UWORD8 q_num)
{
    UWORD32    *element_ptr = 0;
    UWORD32    *ptr_next    = 0;
    q_struct_t *rx_struct   = &(q_handle->rx_handle.rx_header[q_num]);

    /* If element count is zero, Q is already empty */
    if(rx_struct->element_cnt == 0)
    {
        /* If the Q is empty return success status */
        return QMU_OK;
    }

    /* Reset the element count for the current Q */
    rx_struct->element_cnt = 0;

    /* Empty the queue */
    element_ptr = rx_struct->element_head;

    while(element_ptr != NULL)
    {
        ptr_next = (UWORD32 *)get_rx_dscr_next_dscr(element_ptr);
        free_rx_dscr(element_ptr);
        element_ptr = ptr_next;
    }

    /* Reset the H/w buffer pointers */
    set_machw_rx_buff_addr(0, q_num);

    /* Reset the pointers to first and last element of the Q */
    rx_struct->element_tail = NULL;
    rx_struct->element_head = NULL;
    rx_struct->element_to_load = NULL;
//    rx_struct->trout_head = 0; //Hugh add

    return QMU_OK;
}
