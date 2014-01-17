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
/*  File Name         : shared_mem_config.h                                  */
/*                                                                           */
/*  Description       : This file contains all the configuration parameters  */
/*                      for the shared memory.                               */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef SHARED_MEM_CONFIG_H
#define SHARED_MEM_CONFIG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "csl_if.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/* Shared Descriptor Memory Pool                                             */
/*                                                                           */
/* This includes all transmit, receive, schedule descriptor buffers that are */
/* shared between H/w and S/w and the access time is critical for H/w.       */
/*                                                                           */
/*****************************************************************************/

/* Descriptor Base Address Definition */
#ifdef GENERIC_PLATFORM
#define SHARED_DSCR_MEM_POOL_BASE 0x00000000
#endif /* GENERIC_PLATFORM */

#ifdef MWLAN
/* For MWLAN platform the base for this pool is defined as DPSRAM0 and the   */
/* size assigned is 12 KB. Note that the exact size is different and is      */
/* calculated based on the number of chunks and chunk sizes.                 */
#ifndef __KERNEL__	//add by chengwg.
#define SHARED_DSCR_MEM_POOL_BASE DPSRAM0_ADDR
#else
#define SHARED_DSCR_MEM_POOL_BASE	g_shared_mem_base.shared_dscr_mem_pool_base
#endif
#define SHARED_DSCR_MEM_POOL_SIZE (32 * 1024)
#endif /* MWLAN */

//caisf add 
#define TROUT_MEM_CHUNK_RESERV_SIZE      (8)
#define TROUT_MEM_CHUNK_DESCR_SIZE_1     (44+TROUT_MEM_CHUNK_RESERV_SIZE)
#define TROUT_MEM_CHUNK_DESCR_SIZE_2     (160+TROUT_MEM_CHUNK_RESERV_SIZE)
#define TROUT_MEM_CHUNK_DESCR_SIZE_3     (288+TROUT_MEM_CHUNK_RESERV_SIZE)


/* This pool is divided into 3 sub-pools with following details.             */
/*                                                                           */
/* ------------------------------------------------------------------------- */
/* | Subpool chunk size | Number of chunks | Buffer allocation             | */
/* ------------------------------------------------------------------------- */
/* | 44                 | 85               | Receive descriptor            | */
/* | 160                | 64               | Min Transmit descriptor       | */
/* |                    |                  | Actual: 152b, Alignment = 32b | */
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
/* | 288                | 64               | Max Transmit descriptor       | */
/* |                    |                  | Actual: 272b, Alignment = 32b | */
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
/* ------------------------------------------------------------------------- */

#define SHARED_DSCR_MEM_NUM_SUB_POOLS 3
#define SHARED_DSCR_MEM_ALIGNMENT     32

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef BSS_ACCESS_POINT_MODE
#define SHARED_DSCR_MEM_NUM_CHUNK_44  104
#ifndef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
#define SHARED_DSCR_MEM_NUM_CHUNK_160 139
#define SHARED_DSCR_MEM_NUM_CHUNK_288 0
#else /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#define SHARED_DSCR_MEM_NUM_CHUNK_160 112
#define SHARED_DSCR_MEM_NUM_CHUNK_288 34
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#else  /* BSS_ACCESS_POINT_MODE */
#define SHARED_DSCR_MEM_NUM_CHUNK_44  104
#ifndef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
#define SHARED_DSCR_MEM_NUM_CHUNK_160 129
#define SHARED_DSCR_MEM_NUM_CHUNK_288 0
#else /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#define SHARED_DSCR_MEM_NUM_CHUNK_160 112
#define SHARED_DSCR_MEM_NUM_CHUNK_288 34
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#endif  /* BSS_ACCESS_POINT_MODE */
#else /* MAC_HW_UNIT_TEST_MODE */
#define SHARED_DSCR_MEM_NUM_CHUNK_44  64
#define SHARED_DSCR_MEM_NUM_CHUNK_160 64
#define SHARED_DSCR_MEM_NUM_CHUNK_288 64
#endif /* MAC_HW_UNIT_TEST_MODE */

/* Compute the maximum number of chunks (used for defining the stack size in */
/* memory manager algorithm) and the exact size (used for exception checks)  */
/* The maximum length also includes the overhead for enforcing buffer        */
/* alignment.                                                                */

#define SHARED_DSCR_MEM_MAX_CHUNK_NUM (SHARED_DSCR_MEM_NUM_CHUNK_44  + \
                                       SHARED_DSCR_MEM_NUM_CHUNK_288 + \
                                       SHARED_DSCR_MEM_NUM_CHUNK_160)

#define SHARED_DSCR_MEM_SIZE          (SHARED_DSCR_MEM_NUM_CHUNK_44  * TROUT_MEM_CHUNK_DESCR_SIZE_1  + \
                                       SHARED_DSCR_MEM_NUM_CHUNK_160 * TROUT_MEM_CHUNK_DESCR_SIZE_2 + \
                                       SHARED_DSCR_MEM_NUM_CHUNK_288 * TROUT_MEM_CHUNK_DESCR_SIZE_3 + \
                                       SHARED_DSCR_MEM_ALIGNMENT * SHARED_DSCR_MEM_NUM_SUB_POOLS)

/*****************************************************************************/
/*                                                                           */
/* Shared Packet Memory Pool                                                 */
/*                                                                           */
/* This pool includes all the transmit, receive, beacon and management frame */
/* buffers that are shared between H/w and S/w but the access time is not    */
/* critical for H/w.                                                         */
/*                                                                           */
/*****************************************************************************/

/* Index to the sub-pool containing the TX/RX packet buffers in the Shared */
/* Packet Memory.                                                          */
#ifdef MAC_HW_UNIT_TEST_MODE
    #define SHARED_PKT_MEM_TXRX_BUFFER_IDX  2
#else  /* MAC_HW_UNIT_TEST_MODE */
    #define SHARED_PKT_MEM_TXRX_BUFFER_IDX  1
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* Packet Memory Base address definition (Platform specific) */
#ifdef GENERIC_PLATFORM
#define SHARED_PKT_MEM_POOL_BASE 0x00000000
#endif /* GENERIC_PLATFORM */

#ifdef MWLAN
/* For MWLAN platform the base for this pool is defined as DPSRAM0 offset by */
/* SHARED_DSCR_MEM_POOL_SIZE and the size assigned is 128 KB. Note that the  */
/* exact size is different and is calculated based on the number of chunks   */
/* and chunk sizes.                                                          */
#define SHARED_PKT_MEM_POOL_SIZE (256 * 1024)

#ifndef __KERNEL__	//add by chengwg.
#define SHARED_PKT_MEM_POOL_BASE (DPSRAM0_ADDR + SHARED_DSCR_MEM_POOL_SIZE)
#else
#define SHARED_PKT_MEM_POOL_BASE	g_shared_mem_base.shared_pkt_mem_pool_base
#endif
#endif /* MWLAN */

//caisf add 
#define TROUT_MEM_CHUNK_PKT_SIZE_1     (448+TROUT_MEM_CHUNK_RESERV_SIZE)
#define TROUT_MEM_CHUNK_PKT_SIZE_2     (1600+TROUT_MEM_CHUNK_RESERV_SIZE)
#define TROUT_MEM_CHUNK_PKT_SIZE_3     (1792+TROUT_MEM_CHUNK_RESERV_SIZE)

#ifndef MAC_HW_UNIT_TEST_MODE
/* This pool is divided into 4 sub-pools with following details.             */
/*                                                                           */
/* ------------------------------------------------------------------------- */
/* | Subpool chunk size | Number of chunks | Buffer allocation             | */
/* ------------------------------------------------------------------------- */
/* | 448                | 12               | Management Tx & Rx buffers    | */
/* | 1664               | 121              | Tx, Rx & Beacon buffers       | */
/* ------------------------------------------------------------------------- */

#define SHARED_PKT_MEM_NUM_SUB_POOLS  3
//modified by caisf 
//#define SHARED_PKT_MEM_BUFFER_SIZE    (1600 - MEM_CHUNK_INFO_SIZE)
#define SHARED_PKT_MEM_BUFFER_SIZE    (TROUT_MEM_CHUNK_PKT_SIZE_2 - MEM_CHUNK_INFO_SIZE)
#define SHARED_PKT_MEM_ALIGNMENT      32

#ifdef BSS_ACCESS_POINT_MODE
#define SHARED_PKT_MEM_NUM_CHUNK_448  5
#define SHARED_PKT_MEM_NUM_CHUNK_1600 159
#define SHARED_PKT_MEM_NUM_CHUNK_1792 2
#else /* BSS_ACCESS_POINT_MODE */
#define SHARED_PKT_MEM_NUM_CHUNK_448  5
#define SHARED_PKT_MEM_NUM_CHUNK_1600 140
#define SHARED_PKT_MEM_NUM_CHUNK_1792 0
#endif /* BSS_ACCESS_POINT_MODE */

/* Compute the maximum number of chunks (used for defining the stack size in */
/* memory manager algorithm) and the exact size (used for exception checks)  */

#define SHARED_PKT_MEM_MAX_CHUNK_NUM  (SHARED_PKT_MEM_NUM_CHUNK_448  + \
                                       SHARED_PKT_MEM_NUM_CHUNK_1600 + \
                                       SHARED_PKT_MEM_NUM_CHUNK_1792)
//modified by caisf 
#define SHARED_PKT_MEM_SIZE           (SHARED_PKT_MEM_NUM_CHUNK_448  * TROUT_MEM_CHUNK_PKT_SIZE_1  + \
                                       SHARED_PKT_MEM_NUM_CHUNK_1600 * TROUT_MEM_CHUNK_PKT_SIZE_2 + \
                                       SHARED_PKT_MEM_NUM_CHUNK_1792 * TROUT_MEM_CHUNK_PKT_SIZE_3 + \
                                       SHARED_PKT_MEM_ALIGNMENT * SHARED_PKT_MEM_NUM_SUB_POOLS)

#else /* MAC_HW_UNIT_TEST_MODE */
/* This pool is divided into 4 sub-pools with following details.             */
/*                                                                           */
/* ------------------------------------------------------------------------- */
/* | Subpool chunk size | Number of chunks | Buffer allocation             | */
/* ------------------------------------------------------------------------- */
/* | 36                 | 150              | Data MSDU WLAN header         | */
/* | 320                | 20               | Management Tx & Rx buffers    | */
/* | 1664               | 100              | Tx(Nml), Rx & Beacon buffers  | */
/* | 4128               | 32               | Tx(AMSDU), Rx 64K PHY tests   | */
/* ------------------------------------------------------------------------- */

#define SHARED_PKT_MEM_NUM_SUB_POOLS  4
#define SHARED_PKT_MEM_ALIGNMENT      32
#define SHARED_PKT_MEM_BUFFER_SIZE    (1600 - MEM_CHUNK_INFO_SIZE)

#define SHARED_PKT_MEM_NUM_CHUNK_36   42
#define SHARED_PKT_MEM_NUM_CHUNK_320  42
#define SHARED_PKT_MEM_NUM_CHUNK_1600 64
#define SHARED_PKT_MEM_NUM_CHUNK_4128 32

/* Compute the maximum number of chunks (used for defining the stack size in */
/* memory manager algorithm) and the exact size (used for exception checks)  */

#define SHARED_PKT_MEM_MAX_CHUNK_NUM  (SHARED_PKT_MEM_NUM_CHUNK_36   + \
                                       SHARED_PKT_MEM_NUM_CHUNK_320  + \
                                       SHARED_PKT_MEM_NUM_CHUNK_1600 + \
                                       SHARED_PKT_MEM_NUM_CHUNK_4128)

#define SHARED_PKT_MEM_SIZE           (SHARED_PKT_MEM_NUM_CHUNK_36   * 36   + \
                                       SHARED_PKT_MEM_NUM_CHUNK_320  * 320  + \
                                       SHARED_PKT_MEM_NUM_CHUNK_1600 * 1600 + \
                                       SHARED_PKT_MEM_NUM_CHUNK_4128 * 4128 + \
                                       SHARED_PKT_MEM_ALIGNMENT * SHARED_PKT_MEM_NUM_SUB_POOLS)

#endif /* MAC_HW_UNIT_TEST_MODE */

/*****************************************************************************/
/* Flow Control Parameters which can be tweaked based on the number of       */
/* buffers available on the target platform and the host configuration.      */
/*****************************************************************************/
/* Parameters related to reservation of buffers for various users */
#define NUM_PKTS_RESVD_CONFIG           2
#define NUM_PKTS_RESVD_HPTX             2
#define NUM_PKTS_RESVD_HPRX             3
#define NUM_PKTS_RESVD_RX               0
#define NUM_PKTS_RESVD_TX               0

/* Parameters related to flow control on the TX/RX paths */
#define TX_PACKET_BUFFER_THRESHOLD      80
#define RX_PACKET_BUFFER_THRESHOLD      80

/* Limit the Max Tx Pending Pkts to 0.75 * SHARED_MEM_NUM_CHUNK_1600 */
/* This is to make sure the proity differentiation happens fine among*/
/* different 11e Tx & Rx Queues                                      */
#ifndef MWLAN /* Disable Queue-Limiting in MAC S/w */
    #define MAX_PENDING_PKTS            (SHARED_PKT_MEM_NUM_CHUNK_1600)
#else /* MWLAN */
    /* Queue Limiting is still required on MWLAN since flow control is not */
    /* currently possible.                                                 */
    #define MAX_PENDING_PKTS           ((3 * SHARED_PKT_MEM_NUM_CHUNK_1600) >> 2)
#endif /* MWLAN */

#ifdef STATIC_ARRAY_SUB_POOL
#define MAX_SMEM_STACK_SZ (SHARED_DSCR_MEM_MAX_CHUNK_NUM + \
                           SHARED_PKT_MEM_MAX_CHUNK_NUM)
#endif /* STATIC_ARRAY_SUB_POOL */

#endif /* SHARED_MEM_CONFIG_H */
