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
/*  File Name         : local_mem_config.h                                   */
/*                                                                           */
/*  Description       : This file contains all the configuration parameters  */
/*                      for the local memory pool.                           */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef LOCAL_MEM_CONFIG_H
#define LOCAL_MEM_CONFIG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "csl_if.h"
#include "index_util.h"
#include "core_mode_if.h"

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11H
#include "dfs_sta.h"
#endif /* MAC_802_11H */
#endif /* IBSS_BSS_STATION_MODE */

#ifdef OS_LINUX_CSL_TYPE
#include <linux/timer.h>
#endif /* OS_LINUX_CSL_TYPE */
/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/* Local Memory Pool                                                         */
/*                                                                           */
/* This pool includes all S/w data structures. This pool is divided into sub */
/* pools based on the configuration.                                         */
/*                                                                           */
/*****************************************************************************/

/* The base for this pool is defined as a global variable */
#define LOCAL_MEM_POOL_BASE ((UWORD32)g_local_mem)

/* Alignment of local memory pool buffers, modify by caisf */
#define LOCAL_MEM_ALIGNMENT 16 // 4

/* Maximum number of local memory sub pools supported (used for defining the */
/* sub_pool_info array). Exact number of sub pools is dynamically defined    */
/* based on the actual compile-time configuration (chunk sizes and number of */
/* chunks).                                                                  */
#define MAX_LOCAL_MEM_NUM_SUB_POOLS 6

/* Chunk sizes and number of chunks for the maximum number of local memory   */
/* sub pools, specified based on the modes/protocols defined.                */
/*                                                                           */
/* ------------------------------------------------------------------------- */
/* | Structure      | Number of instances      | Configuration dependency  | */
/* ------------------------------------------------------------------------- */
/* | timer_list     | NUM_SW_TIMER_SUPPORTED   | Valid for Linux OS CSL    | */
/* | table_elmnt_t  | NUM_STA_SUPPORTED        | Valid for AP/STA Mode     | */
/* | asoc_entry_t   | NUM_STA_SUPPORTED        | Valid for AP Mode         | */
/* | sta_entry_t    | NUM_STA_SUPPORTED        | Valid for STA Mode        | */
/* | ba_tx_struct_t | NUM_BA_SESSION_SUPPORTED | Valid for 11N Protocol    | */
/* | ba_rx_struct_t | NUM_BA_SESSION_SUPPORTED | Valid for 11N Protocol    | */
/* | req_meas_elem  | DOT11H_REQ_CHUNKS        | Valid for 11H Protocol    | */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* Note - Chunk sizes should correspond to the size of structures that are   */
/* to be allocated from the local memory pool. The number of chunks should   */
/* correspond to the appropriate parameter that is decided at compile-time   */

#ifdef OS_LINUX_CSL_TYPE
// added by caisf 
#define LOCAL_MEM_CHUNK_SIZE_RESERVE  (8)

#define LOCAL_MEM_CHUNK_SIZE_1  (sizeof(struct timer_list) + MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE) /*(sizeof(timer_list))*/
#define LOCAL_MEM_NUM_CHUNK_1   (NUM_SW_TIMER_SUPPORTED)

#else /* OS_LINUX_CSL_TYPE */

#define LOCAL_MEM_CHUNK_SIZE_1  (MEM_CHUNK_INFO_SIZE)
#define LOCAL_MEM_NUM_CHUNK_1   (0)

#endif /* OS_LINUX_CSL_TYPE */

#ifdef MAC_HW_UNIT_TEST_MODE

#define LOCAL_MEM_CHUNK_SIZE_2  (MEM_CHUNK_INFO_SIZE)
#define LOCAL_MEM_NUM_CHUNK_2   (0)

#define LOCAL_MEM_CHUNK_SIZE_3  (MEM_CHUNK_INFO_SIZE)
#define LOCAL_MEM_NUM_CHUNK_3   (0)

#else /* MAC_HW_UNIT_TEST_MODE */

#define LOCAL_MEM_CHUNK_SIZE_2  (sizeof(table_elmnt_t) + MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_2   (NUM_STA_SUPPORTED)

#ifdef BSS_ACCESS_POINT_MODE

#define LOCAL_MEM_CHUNK_SIZE_3  (sizeof(asoc_entry_t) + MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_3   (NUM_STA_SUPPORTED)

#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE

#define LOCAL_MEM_CHUNK_SIZE_3  (sizeof(sta_entry_t) + MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_3   (NUM_STA_SUPPORTED)

#endif /* IBSS_BSS_STATION_MODE */

#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef MAC_802_11N

#define LOCAL_MEM_CHUNK_SIZE_4  (sizeof(ba_tx_struct_t) + MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_4   (NUM_BA_SESSION_SUPPORTED)

#define LOCAL_MEM_CHUNK_SIZE_5  (sizeof(ba_rx_struct_t) + MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_5   (NUM_BA_SESSION_SUPPORTED)

#else /* MAC_802_11N */

#define LOCAL_MEM_CHUNK_SIZE_4  (MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_4   (0)

#define LOCAL_MEM_CHUNK_SIZE_5  (MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_5   (0)

#endif /* MAC_802_11N */

#ifdef MAC_802_11H

#ifdef IBSS_BSS_STATION_MODE
#define LOCAL_MEM_CHUNK_SIZE_6  (sizeof(req_meas_elem) + MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_6   (DOT11H_REQ_CHUNKS)
#else /* IBSS_BSS_STATION_MODE */
#define LOCAL_MEM_CHUNK_SIZE_6  (MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_6   (0)
#endif /* IBSS_BSS_STATION_MODE */

#else /* MAC_802_11H */
#define LOCAL_MEM_CHUNK_SIZE_6  (MEM_CHUNK_INFO_SIZE + LOCAL_MEM_CHUNK_SIZE_RESERVE)
#define LOCAL_MEM_NUM_CHUNK_6   (0)

#endif /* MAC_802_11H */


/* Compute the maximum number of chunks (used for defining the stack size in */
/* memory manager algorithm) and the exact size (used for exception checks)  */

#define LOCAL_MEM_MAX_CHUNK_NUM (LOCAL_MEM_NUM_CHUNK_1  + \
                                 LOCAL_MEM_NUM_CHUNK_2  + \
                                 LOCAL_MEM_NUM_CHUNK_3  + \
                                 LOCAL_MEM_NUM_CHUNK_4  + \
                                 LOCAL_MEM_NUM_CHUNK_5  + \
                                 LOCAL_MEM_NUM_CHUNK_6)

#define LOCAL_MEM_SIZE (LOCAL_MEM_NUM_CHUNK_1 * LOCAL_MEM_CHUNK_SIZE_1 + \
                        LOCAL_MEM_NUM_CHUNK_2 * LOCAL_MEM_CHUNK_SIZE_2 + \
                        LOCAL_MEM_NUM_CHUNK_3 * LOCAL_MEM_CHUNK_SIZE_3 + \
                        LOCAL_MEM_NUM_CHUNK_4 * LOCAL_MEM_CHUNK_SIZE_4 + \
                        LOCAL_MEM_NUM_CHUNK_5 * LOCAL_MEM_CHUNK_SIZE_5 + \
                        LOCAL_MEM_NUM_CHUNK_6 * LOCAL_MEM_CHUNK_SIZE_6 + \
                        LOCAL_MEM_ALIGNMENT * MAX_LOCAL_MEM_NUM_SUB_POOLS)


#ifdef LOCALMEM_TX_DSCR
/*****************************************************************************/
/*                                                                           */
/* Local Descriptor Memory Pool                                              */
/*                                                                           */
/* This pool includes all Tx-Descriptors which are created in the DMA        */
/* capable region of SDRAM.                                                  */
/*                                                                           */
/*****************************************************************************/

/* The base for this pool is defined as a global variable and should be      */
/* assigned to region of SDRAM which can be DMAed to.                        */

#define LOCAL_DSCR_MEM_POOL_BASE        (UWORD32)g_local_mem_dma

/* This pool is divided into 1 sub-pool with following details.              */
/*                                                                           */
/* ------------------------------------------------------------------------- */
/* | Subpool chunk size | Number of chunks | Buffer allocation             | */
/* ------------------------------------------------------------------------- */
/* | 160                | 112              | Min Transmit descriptor       | */
/* |                    |                  | Actual: 152b, Alignment = 32b | */
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
/* | 288                | 11               | Max Transmit descriptor       | */
/* |                    |                  | Actual: 272b, Alignment = 32b | */
/* ------------------------------------------------------------------------- */
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
#define LOCAL_DSCR_MEM_NUM_SUB_POOLS    2
#else /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#define SHARED_DSCR_MEM_NUM_SUB_POOLS 1
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#define LOCAL_DSCR_MEM_ALIGNMENT        32

#ifndef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
#define LOCAL_DSCR_MEM_NUM_CHUNK_1      112
#else /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#define LOCAL_DSCR_MEM_NUM_CHUNK_1      112
#define LOCAL_DSCR_MEM_NUM_CHUNK_2      11
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

#define LOCAL_DSCR_MEM_CHUNK_SIZE_1     160
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
#define LOCAL_DSCR_MEM_CHUNK_SIZE_2     288
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */


// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
#define LOCAL_DSCR_MEM_MAX_CHUNK_NUM (LOCAL_DSCR_MEM_NUM_CHUNK_1\
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
                                      + LOCAL_DSCR_MEM_NUM_CHUNK_2\
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
                                      )

#define LOCAL_DSCR_MEM_SIZE          (LOCAL_DSCR_MEM_NUM_CHUNK_1 * LOCAL_DSCR_MEM_CHUNK_SIZE_1 + \
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
                                      LOCAL_DSCR_MEM_NUM_CHUNK_2 * LOCAL_DSCR_MEM_CHUNK_SIZE_2 +\
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
                                      LOCAL_DSCR_MEM_ALIGNMENT * LOCAL_DSCR_MEM_NUM_SUB_POOLS)
#else
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
#define LOCAL_DSCR_MEM_MAX_CHUNK_NUM (LOCAL_DSCR_MEM_NUM_CHUNK_1 +\
                                      LOCAL_DSCR_MEM_NUM_CHUNK_2)
#else  /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#define LOCAL_DSCR_MEM_MAX_CHUNK_NUM (LOCAL_DSCR_MEM_NUM_CHUNK_1)
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
	#define LOCAL_DSCR_MEM_SIZE  (LOCAL_DSCR_MEM_NUM_CHUNK_1 * LOCAL_DSCR_MEM_CHUNK_SIZE_1 + \
                                  LOCAL_DSCR_MEM_NUM_CHUNK_2 * LOCAL_DSCR_MEM_CHUNK_SIZE_2 +\
                                  LOCAL_DSCR_MEM_ALIGNMENT * LOCAL_DSCR_MEM_NUM_SUB_POOLS)
#else  /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
    #define LOCAL_DSCR_MEM_SIZE  (LOCAL_DSCR_MEM_NUM_CHUNK_1 * LOCAL_DSCR_MEM_CHUNK_SIZE_1 + \
                                  LOCAL_DSCR_MEM_ALIGNMENT * LOCAL_DSCR_MEM_NUM_SUB_POOLS)
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
#endif

#endif /* LOCALMEM_TX_DSCR */


/*****************************************************************************/
/*                                                                           */
/* Compute the maximum memory pool stack size for the memory manager based   */
/* on the total number of chunks for all the memory pools.                   */
/*                                                                           */
/*****************************************************************************/

#ifndef LOCALMEM_TX_DSCR
#define LOCAL_DSCR_MEM_MAX_CHUNK_NUM 0
#endif /* LOCALMEM_TX_DSCR */

#ifdef STATIC_ARRAY_SUB_POOL
#define MAX_LMEM_STACK_SZ (LOCAL_MEM_MAX_CHUNK_NUM       + \
                           LOCAL_DSCR_MEM_MAX_CHUNK_NUM)
#endif /* STATIC_ARRAY_SUB_POOL */

#endif /* LOCAL_MEM_CONFIG_H */
