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
/*  File Name         : qmu.h                                                */
/*                                                                           */
/*  Description       : This file contains all the definitions for the       */
/*                      queue manager unit.                                  */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef QMU_H
#define QMU_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#ifdef WAKE_LOW_POWER_POLICY
#define LOW_POWER_TX_THRESHOLD			1
#define LOW_POWER_TOTAL_PKT_THRESHOLD	4
#endif

#define RX_NEXT_ELEMENT_OFFSET          1
#define TX_NEXT_ELEMENT_OFFSET          2
/* The total number of receive queues */
#define NUM_RX_Q                        2

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

typedef enum {QMU_OK                    = 0,
              QMU_LOCAL_MALLOC_FAILED   = 1,
              QMU_SHARED_MALLOC_FAILED  = 2,
              QMU_PACKET_NOT_FOUND      = 3,
              QMU_Q_INACTIVE            = 4
} QMU_RET_CODE_T;


typedef enum {Q_INVALID     = 0x0,
              Q_VALID       = 0x1,
              Q_SUSPENDED   = 0x2,
              Q_STATUS_MAX  = 0x3
} Q_STATUS_T;
/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Transmit queue header structure */
typedef struct
{
    UWORD32 *element_head; /* Pointer to the first element in queue */
    UWORD32 *element_tail; /* Pointer to the last element in queue  */
    UWORD32 *element_to_load; /*Hugh:  Pointer to the element  ready to load to trout */
    UWORD16 element_cnt;   /* Number of elements in the queue       */
//    UWORD32 trout_head;    /* Hugh: Pointer to the head element in trout */
    UWORD8  q_status;      /* Total number of frames received       */
//    UWORD8  handle_pkt;	   /* the pkt has cpy to trout and not transacted(chwg) */
} q_struct_t;

/* Transmit queue handle structure */
typedef struct
{
    q_struct_t *tx_header;    /* List of Tx Q structures        */
    UWORD16    tx_curr_qnum;  /* Number of buffers in all Tx Qs */
    UWORD16    tx_list_count; /* Number of buffers in all Tx Qs */
    struct mutex txq_lock;      //add by hugh
	UWORD32 tx_mem_start;	//add by chengwg for record trout tx share memory begin info.
	UWORD32 tx_mem_size;	//add by chengwg.
	UWORD32 tx_mem_end;		//add by chengwg.
	UWORD32	cur_slot;	//the current slot which is transmiting
	struct mutex  hwtx_mutex;	//hardware tx need a mutex
} qmu_tx_handle_t;

/* Receive queue handle structure */
typedef struct
{
    q_struct_t rx_header[NUM_RX_Q]; /* List of Rx Q structures        */
    UWORD16    rx_list_count;       /* Number of buffers in all Rx Qs */
} qmu_rx_handle_t;

/* Queue manager handle structure */
typedef struct
{
    UWORD8          num_tx_q;  /* Number of active transmit queues */
    qmu_tx_handle_t tx_handle; /* Handle to the transmit queues    */
    qmu_rx_handle_t rx_handle; /* Handle to the receive queue      */
} qmu_handle_t;

typedef struct
{
	UWORD8 *tx_dma_buf;			/* point to tx dma buffer */
	struct mutex tx_dma_lock;
}tx_dma_handle_t;
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
#ifdef TX_PKT_USE_DMA
extern tx_dma_handle_t g_tx_dma_handle;
extern int tx_dma_buf_init(void);
extern void tx_dma_buf_free(void);
#endif	/* TX_PKT_USE_DMA */

extern UWORD8 qmu_init(qmu_handle_t* q_handle, UWORD16 q_num);
extern UWORD8 qmu_flush_tx_queue(qmu_handle_t* q_handle, UWORD8 q_num);
extern UWORD8 qmu_flush_rx_queue(qmu_handle_t *q_handle, UWORD8 q_num);
extern void   qmu_close(qmu_handle_t* q_handle);

#endif /* QMU_H */
