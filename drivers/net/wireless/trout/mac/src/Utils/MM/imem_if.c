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
/*  File Name         : imem_if.c                                            */
/*                                                                           */
/*  Description       : This file contains the global definitions and        */
/*                      functions required for the memory manager interface. */
/*                                                                           */
/*  List of Functions : init_mem_regions                                     */
/*                      init_shared_dscr_mem                                 */
/*                      init_shared_pkt_mem                                  */
/*                      init_local_mem_subpool_info                          */
/*                      init_local_mem                                       */
/*                      init_mem_handle                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "imem_if.h"
#include "shared_mem_config.h"
#include "local_mem_config.h"
#include "proc_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/* Memory handles for all the memory pools */
mem_handle_t   g_shared_dscr_mem_pool_handle = {0};
mem_handle_t   g_shared_pkt_mem_pool_handle  = {0};
mem_handle_t   g_local_mem_pool_handle       = {0};
#ifdef LOCALMEM_TX_DSCR
mem_handle_t   g_local_dscr_mem_pool_handle  = {0};
#endif /* LOCALMEM_TX_DSCR */

/* Sub pool headers for all sub pools in each memory pool */
mem_sub_pool_t g_shared_dscr_mem_sub_pool_arr[SHARED_DSCR_MEM_NUM_SUB_POOLS]  = {{0,},};
mem_sub_pool_t g_shared_pkt_mem_sub_pool_arr[SHARED_PKT_MEM_NUM_SUB_POOLS] = {{0,},};
mem_sub_pool_t g_local_mem_sub_pool_arr[MAX_LOCAL_MEM_NUM_SUB_POOLS] = {{0,},};

#ifdef LOCALMEM_TX_DSCR
mem_sub_pool_t g_local_dscr_mem_sub_pool_arr[LOCAL_DSCR_MEM_NUM_SUB_POOLS] = {{0,},};
#endif /* LOCALMEM_TX_DSCR */

/* Global memory space used for local and event memory pool base */
UWORD8         g_local_mem[LOCAL_MEM_SIZE];

#ifdef LOCALMEM_TX_DSCR
void           *g_local_mem_dma = NULL;
#endif /* LOCALMEM_TX_DSCR */

UWORD8 g_mem_regions_initialized = 0;

pktmem_priority_t g_pktmem_pri[NUM_MEM_PRI_LEVELS] = {{0,},};

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static UWORD8 init_local_mem_subpool_info(UWORD16 *sub_pool_info,
                                          UWORD8 num_sub_pool);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_mem_regions                                         */
/*                                                                           */
/*  Description   : This function initializes all memory regions to zero.    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mem_regions_initialized                                */
/*                                                                           */
/*  Processing    : This function checks if the memory regions have already  */
/*                  been initialized and if not sets all memory regions to   */
/*                  zeroes.                                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

share_mem_s g_shared_mem_base;

int init_mem_regions(void)
{
	TROUT_FUNC_ENTER;
    if(g_mem_regions_initialized == 1)
		return 0;

	if(gsb->tx_start != NULL){
		g_shared_mem_base.shared_pkt_mem_pool_base = gsb->tx_start;
	}else{
	#ifdef SYS_RAM_256M
		g_shared_mem_base.shared_pkt_mem_pool_base = vmalloc(SHARED_PKT_MEM_SIZE);
	#else	
		g_shared_mem_base.shared_pkt_mem_pool_base = kmalloc(SHARED_PKT_MEM_SIZE, GFP_KERNEL);
	#endif
		if(g_shared_mem_base.shared_pkt_mem_pool_base == NULL){
			TROUT_DBG2("alloc shared pkt mem pool failed!\n");
			TROUT_FUNC_EXIT;
			return -1;
		}
	}

    mem_set((UWORD8 *)SHARED_PKT_MEM_POOL_BASE, 0, SHARED_PKT_MEM_SIZE);
    mem_set((UWORD8 *)LOCAL_MEM_POOL_BASE, 0, LOCAL_MEM_SIZE);

	if(gsb->td_start != NULL){
		g_shared_mem_base.shared_dscr_mem_pool_base = gsb->td_start;
	}else{
	#ifdef SYS_RAM_256M
		g_shared_mem_base.shared_dscr_mem_pool_base = vmalloc(SHARED_DSCR_MEM_SIZE);
	#else	
		g_shared_mem_base.shared_dscr_mem_pool_base = kmalloc(SHARED_DSCR_MEM_SIZE, GFP_KERNEL);
	#endif
		if(g_shared_mem_base.shared_dscr_mem_pool_base == NULL){	
			TROUT_DBG2("alloc shared dscr mem pool failed!\n");
			goto out;
		}
	}
	
    mem_set((UWORD8 *)SHARED_DSCR_MEM_POOL_BASE, 0, SHARED_DSCR_MEM_SIZE);
    mem_set((UWORD8 *)SCRATCH_MEM_POOL_BASE, 0,
            SCRATCH_MEM_MAX_SIZE + MAX_WORD_ALIGNMENT_BUFFER);

#ifdef LOCALMEM_TX_DSCR
    mem_set((UWORD8 *)LOCAL_DSCR_MEM_POOL_BASE, 0, LOCAL_DSCR_MEM_SIZE);
#endif /* LOCALMEM_TX_DSCR */

	g_mem_regions_initialized = 1;

	TROUT_FUNC_EXIT;
	return 0;
	
out:
	if(g_shared_mem_base.shared_pkt_mem_pool_base != NULL){
		if(NULL == gsb->tx_start){
#ifdef SYS_RAM_256M
			vfree(g_shared_mem_base.shared_pkt_mem_pool_base);
#else
			kfree(g_shared_mem_base.shared_pkt_mem_pool_base);
#endif
		}
		g_shared_mem_base.shared_pkt_mem_pool_base = NULL;
	}
	g_mem_regions_initialized = 0;
	
	TROUT_FUNC_EXIT;
	return -1;
}


void free_mem_regions(void) //add by Hugh
{
    if(g_mem_regions_initialized == 0)
        return;

    if(g_shared_mem_base.shared_pkt_mem_pool_base != NULL){
	if(NULL == gsb->tx_start){
#ifdef SYS_RAM_256M
        	vfree(g_shared_mem_base.shared_pkt_mem_pool_base);
#else        
		kfree(g_shared_mem_base.shared_pkt_mem_pool_base);
#endif
	}
	g_shared_mem_base.shared_pkt_mem_pool_base = NULL;
    }

    if(g_shared_mem_base.shared_dscr_mem_pool_base != NULL){
	if(NULL == gsb->td_start){
#ifdef SYS_RAM_256M
        vfree(g_shared_mem_base.shared_dscr_mem_pool_base);
#else        
	kfree(g_shared_mem_base.shared_dscr_mem_pool_base);
#endif
	}
        g_shared_mem_base.shared_dscr_mem_pool_base = NULL;
	}
    mem_set((UWORD8 *)LOCAL_MEM_POOL_BASE, 0, LOCAL_MEM_SIZE);

    mem_set((UWORD8 *)SCRATCH_MEM_POOL_BASE, 0,
            SCRATCH_MEM_MAX_SIZE + MAX_WORD_ALIGNMENT_BUFFER);

#ifdef LOCALMEM_TX_DSCR
    mem_set((UWORD8 *)LOCAL_DSCR_MEM_POOL_BASE, 0, LOCAL_DSCR_MEM_SIZE);
#endif /* LOCALMEM_TX_DSCR */

    mem_set(&g_shared_mem_base, 0, sizeof(share_mem_s));

    g_mem_regions_initialized = 0;
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_shared_dscr_mem                                     */
/*                                                                           */
/*  Description   : This function initializes the shared descriptor memory.  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_shared_dscr_mem_pool_handle                            */
/*                  g_shared_dscr_mem_sub_pool_arr                           */
/*                                                                           */
/*  Processing    : This function initializes the sub pool information in    */
/*                  the required format (Chunk Size x, Number of chunks of   */
/*                  size x). It calls the memory manager function to         */
/*                  initialize the shared descriptor memory handle and pool. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_shared_dscr_mem(void)
{
    UWORD16 shared_dscr_mem_sub_pool_info[SHARED_DSCR_MEM_NUM_SUB_POOLS][2] =
    {
        {TROUT_MEM_CHUNK_DESCR_SIZE_1  - MEM_CHUNK_INFO_SIZE, SHARED_DSCR_MEM_NUM_CHUNK_44},
        {TROUT_MEM_CHUNK_DESCR_SIZE_2  - MEM_CHUNK_INFO_SIZE, SHARED_DSCR_MEM_NUM_CHUNK_160},
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
        {TROUT_MEM_CHUNK_DESCR_SIZE_3  - MEM_CHUNK_INFO_SIZE, SHARED_DSCR_MEM_NUM_CHUNK_288}
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
    };
    
	TROUT_FUNC_ENTER;

    /* Call a function to initialize the memory pool handle */
    init_mem_pool_handle(&g_shared_dscr_mem_pool_handle,
                         (UWORD32)SHARED_DSCR_MEM_POOL_BASE,
                         (UWORD32)SHARED_DSCR_MEM_SIZE,
                         g_shared_dscr_mem_sub_pool_arr,
                         (UWORD8)SHARED_DSCR_MEM_NUM_SUB_POOLS);

    /* Call a function to initialize the memory pool */
    init_mem_pool(&g_shared_dscr_mem_pool_handle,
                  (UWORD16 *)shared_dscr_mem_sub_pool_info,
                  SHARED_DSCR_MEM_ALIGNMENT);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_shared_pkt_mem                                      */
/*                                                                           */
/*  Description   : This function initializes the shared packet memory.      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_shared_pkt_mem_pool_handle                             */
/*                  g_shared_pkt_mem_sub_pool_arr                            */
/*                                                                           */
/*  Processing    : This function initializes the sub pool information in    */
/*                  the required format (Chunk Size x, Number of chunks of   */
/*                  size x). It calls the memory manager function to         */
/*                  initialize the shared packet memory handle and pool.     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_shared_pkt_mem(void)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    UWORD16 shared_pkt_mem_sub_pool_info[SHARED_PKT_MEM_NUM_SUB_POOLS][2] =
    {
        //modified by caisf 
        {TROUT_MEM_CHUNK_PKT_SIZE_1 - MEM_CHUNK_INFO_SIZE, SHARED_PKT_MEM_NUM_CHUNK_448 },
        {TROUT_MEM_CHUNK_PKT_SIZE_2 - MEM_CHUNK_INFO_SIZE, SHARED_PKT_MEM_NUM_CHUNK_1600},
        {TROUT_MEM_CHUNK_PKT_SIZE_3 - MEM_CHUNK_INFO_SIZE, SHARED_PKT_MEM_NUM_CHUNK_1792},
    };
#else /* MAC_HW_UNIT_TEST_MODE */
    UWORD16 shared_pkt_mem_sub_pool_info[SHARED_PKT_MEM_NUM_SUB_POOLS][2] =
    {
        {36   - MEM_CHUNK_INFO_SIZE, SHARED_PKT_MEM_NUM_CHUNK_36  },
        {320  - MEM_CHUNK_INFO_SIZE, SHARED_PKT_MEM_NUM_CHUNK_320 },
        {1600 - MEM_CHUNK_INFO_SIZE, SHARED_PKT_MEM_NUM_CHUNK_1600},
        {4128 - MEM_CHUNK_INFO_SIZE, SHARED_PKT_MEM_NUM_CHUNK_4128}
    };
#endif /* MAC_HW_UNIT_TEST_MODE */

	TROUT_FUNC_ENTER;

    /* Call a function to initialize the memory pool handle */
    init_mem_pool_handle(&g_shared_pkt_mem_pool_handle,
                         (UWORD32)SHARED_PKT_MEM_POOL_BASE,
                         SHARED_PKT_MEM_SIZE,
                         g_shared_pkt_mem_sub_pool_arr,
                         SHARED_PKT_MEM_NUM_SUB_POOLS);

    /* Call a function to initialize the memory pool */
    init_mem_pool(&g_shared_pkt_mem_pool_handle,
                  (UWORD16 *)shared_pkt_mem_sub_pool_info,
                  SHARED_PKT_MEM_ALIGNMENT);

    /* Reserve Buffers for individual modules */
    g_pktmem_pri[MEM_PRI_CONFIG].num_resvd = NUM_PKTS_RESVD_CONFIG;
    g_pktmem_pri[MEM_PRI_HPTX].num_resvd   = NUM_PKTS_RESVD_HPTX;
    g_pktmem_pri[MEM_PRI_HPRX].num_resvd   = NUM_PKTS_RESVD_HPRX;
    g_pktmem_pri[MEM_PRI_RX].num_resvd     = NUM_PKTS_RESVD_RX;
    g_pktmem_pri[MEM_PRI_TX].num_resvd     = NUM_PKTS_RESVD_TX;

    init_pktmem_pri_table(g_pktmem_pri, NUM_MEM_PRI_LEVELS);
    TROUT_FUNC_EXIT;
}

/* This function provides a wrapper around the mem_alloc function for providing */
/* prioritized access to packet memory. The counter associated with the         */
/* particular priority level are also updated.                                  */
void *pkt_mem_alloc(MEM_PRIORITY_LEVEL_T pri_level)
{
    void *pkt_buff = NULL;
    int smart_type = MODE_START;

#ifdef DEBUG_MODE
    {
        UWORD16 num_free_buff = get_num_free_packet_buffers();

        if(num_free_buff == 0)
            g_mac_stats.no_free_pbuff++;
        else if((pri_level == 0) && (num_free_buff < g_pktmem_pri[1].num_thresh))
            g_mac_stats.rsvd_buff_alloc[0]++;
        else if((pri_level == 1) && (num_free_buff < g_pktmem_pri[2].num_thresh))
            g_mac_stats.rsvd_buff_alloc[1]++;
        else if((pri_level == 2) && (num_free_buff < g_pktmem_pri[3].num_thresh))
            g_mac_stats.rsvd_buff_alloc[2]++;
        else if((pri_level == 3) && (num_free_buff < g_pktmem_pri[4].num_thresh))
            g_mac_stats.rsvd_buff_alloc[3]++;
    }
#endif /* DEBUG_MODE */
    /* Packet buffer is allocated only if the module has not exceeded its */
    /* assigned quota.                                                    */
    if(get_num_free_packet_buffers() > g_pktmem_pri[pri_level].num_thresh)
    {
        pkt_buff = mem_alloc(g_shared_pkt_mem_handle, SHARED_PKT_MEM_BUFFER_SIZE);
        if(pkt_buff != NULL)
        {
            /* Embed the priority level in the buffer */
            set_user_id(pkt_buff, pri_level);
            smart_type = critical_section_smart_start(0,1);
            g_pktmem_pri[pri_level].num_allocd++;
            critical_section_smart_end(smart_type);
        }
    }

    return pkt_buff;
}

/* This function provides a wrapper around the mem_free function for providing  */
/* prioritized access to packet memory. The counter associated with the         */
/* particular priority level are also updated.                                  */
void pkt_mem_free(void *pkt_buff)
{
    MEM_PRIORITY_LEVEL_T  pri_level = NUM_MEM_PRI_LEVELS;
    UWORD8                 native_id = 0;
    signed char            status    = -1;
    int smart_type = MODE_START;

    pri_level = (MEM_PRIORITY_LEVEL_T)get_user_id(pkt_buff);
    native_id = get_native_pool_id(pkt_buff);

    status = mem_free(g_shared_pkt_mem_handle, pkt_buff);

    if((native_id == SHARED_PKT_MEM_TXRX_BUFFER_IDX) &&
       (pri_level < NUM_MEM_PRI_LEVELS) &&
       (status == 0))
    {
        smart_type = critical_section_smart_start(0,1);
        if(g_pktmem_pri[pri_level].num_allocd > 0)
            g_pktmem_pri[pri_level].num_allocd--;
        critical_section_smart_end(smart_type);
    }    
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_local_mem_subpool_info                              */
/*                                                                           */
/*  Description   : This function initializes the local memory sub pool      */
/*                  information as required for the memory manager           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the sub pool information 2-d array         */
/*                  2) Number of sub pools                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sorts the sub pool information given in    */
/*                  the required format (Chunk Size x, Number of chunks of   */
/*                  size x) in ascending order of chunk sizes. Before sort   */
/*                  is done, sub pools with matching chunk sizes are first   */
/*                  detected and accumulated into a single sub pool. Sub     */
/*                  pools with chunk size 0 indicates invalid/unused sub     */
/*                  pools. In the sort procedure these sub pools are moved   */
/*                  to the end of the array such that they are ignored for   */
/*                  any further processing.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8; Number of active/valid sub pools                 */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 init_local_mem_subpool_info(UWORD16 *sub_pool_info, UWORD8 num_sub_pool)
{
    UWORD8 i         = 0;
    UWORD8 size0_cnt = 0;
    UWORD8 swap_idx  = 0;
    UWORD8 init_idx  = 0;
    UWORD8 curr_idx  = 0;

	TROUT_FUNC_ENTER;
    /* Find all the sub pools with matching chunk sizes. Accumulate these    */
    /* into a single sub pool. Reset the remaining sub pools to 0 chunk size */
    /* and 0 number of chunks. These shall be moved to the end of the sub    */
    /* pool information array during the sort procedure and ignored for any  */
    /* further processing.                                                   */
    for(i = 0; i < num_sub_pool; i++)
    {
        for(curr_idx = (i + 1); curr_idx < num_sub_pool; curr_idx++)
        {
            SWORD8 cmp_result = 0;

            /* Compare the current sub pool information with the initial one */
            cmp_result = sub_pool_cmp(sub_pool_info, i, curr_idx);

            /* If the sub pools are equal, merge the current sub pool to the */
            /* initial sub pool                                              */
            if(cmp_result == 0)
            {
                merge_sub_pool(sub_pool_info, i, curr_idx);
            }
        }
    }

    /* Sort the sub pool information array in ascending order of chunk sizes */

    /* Traverse the sub pool information array starting from last element */
    for(i = 1; i <= num_sub_pool; i++)
    {
        /* Initialize  the index to the last element for this iteration */
        init_idx = (num_sub_pool - i);

        /* If the sub pool chunk size is 0 increment the zero size count and */
        /* and continue traversal (since this is the maximum and requires no */
        /* further processing.                                               */
        if(get_sub_pool_size(sub_pool_info, init_idx) == 0)
        {
            size0_cnt++;
            continue;
        }

        /* If the sub pool chunk size is non-zero, save the initial index */
        swap_idx = init_idx;

        /* Traverse sub pool information array from start till this index */
        for(curr_idx = 0; curr_idx < init_idx; curr_idx++)
        {
            SWORD8 cmp_result = 0;

            /* Compare current sub pool information with last identified one */
            cmp_result = sub_pool_cmp(sub_pool_info, curr_idx, swap_idx);

            /* If current sub pool is greater than or equal to the initial   */
            /* sub pool, save the current index as the one with which swap   */
            /* is required to be done                                        */
            if(cmp_result >= 0)
            {
                swap_idx = curr_idx;
            }
        }

        /* Swap the initial sub pool information with the sub pool with      */
        /* which swap is required to be done (as found in the above loop)    */
        if(swap_idx != init_idx)
        {
            swap_sub_pool(sub_pool_info, init_idx, swap_idx);

            /* If swapped sub pool chunk size is 0 increment zero size count */
            if(get_sub_pool_size(sub_pool_info, init_idx) == 0)
                size0_cnt++;
        }
    }

    /* Return the number of active/valid sub pools by decrementing the given */
    /* number of sub pools by the number of sub pools found with zero chunk  */
    /* size (indiciating invalid/absent sub pools)                           */
	TROUT_FUNC_EXIT;
    return (num_sub_pool - size0_cnt);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_local_mem                                           */
/*                                                                           */
/*  Description   : This function initializes the local memory.              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_local_mem_pool_handle                                  */
/*                  g_local_mem_sub_pool_arr                                 */
/*                                                                           */
/*  Processing    : This function initializes the sub pool information in    */
/*                  the required format (Chunk Size x, Number of chunks of   */
/*                  size x). It calls the memory manager function to         */
/*                  initialize the local memory handle and pool.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_local_mem(void)
{
    UWORD8 local_mem_num_sub_pools = 0;

    UWORD16 sub_pool_info[MAX_LOCAL_MEM_NUM_SUB_POOLS][2]  =
    {
        {LOCAL_MEM_CHUNK_SIZE_1 - MEM_CHUNK_INFO_SIZE, LOCAL_MEM_NUM_CHUNK_1},
        {LOCAL_MEM_CHUNK_SIZE_2 - MEM_CHUNK_INFO_SIZE, LOCAL_MEM_NUM_CHUNK_2},
        {LOCAL_MEM_CHUNK_SIZE_3 - MEM_CHUNK_INFO_SIZE, LOCAL_MEM_NUM_CHUNK_3},
        {LOCAL_MEM_CHUNK_SIZE_4 - MEM_CHUNK_INFO_SIZE, LOCAL_MEM_NUM_CHUNK_4},
        {LOCAL_MEM_CHUNK_SIZE_5 - MEM_CHUNK_INFO_SIZE, LOCAL_MEM_NUM_CHUNK_5},
        {LOCAL_MEM_CHUNK_SIZE_6 - MEM_CHUNK_INFO_SIZE, LOCAL_MEM_NUM_CHUNK_6}
    };
    
	TROUT_FUNC_ENTER;

#ifdef LOCALMEM_TX_DSCR
    UWORD16 local_dscr_mem_sub_pool_info[LOCAL_DSCR_MEM_NUM_SUB_POOLS][2] =
    {
        {LOCAL_DSCR_MEM_CHUNK_SIZE_1 - MEM_CHUNK_INFO_SIZE,
         LOCAL_DSCR_MEM_NUM_CHUNK_1},
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
        {LOCAL_DSCR_MEM_CHUNK_SIZE_2 - MEM_CHUNK_INFO_SIZE,
         LOCAL_DSCR_MEM_NUM_CHUNK_2}
#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
    };
#endif /* LOCALMEM_TX_DSCR */

    /* Call a function to initialize the local memory sub pools based on the */
    /* compile-time configuration of modes/protocols and chunk sizes. Pools  */
    /* with equal sized chunks are collapsed to a single pool while pools    */
    /* with no chunks are removed.                                           */
    local_mem_num_sub_pools =
                      init_local_mem_subpool_info((UWORD16 *)sub_pool_info,
                                                  MAX_LOCAL_MEM_NUM_SUB_POOLS);

    /* Call a function to initialize the memory pool handle */
    init_mem_pool_handle(&g_local_mem_pool_handle,
                         LOCAL_MEM_POOL_BASE,
                         LOCAL_MEM_SIZE,
                         g_local_mem_sub_pool_arr,
                         local_mem_num_sub_pools);

    /* Call a function to initialize the memory pool */
    init_mem_pool(&g_local_mem_pool_handle,
                  (UWORD16 *)sub_pool_info,
                  LOCAL_MEM_ALIGNMENT);

#ifdef LOCALMEM_TX_DSCR
    /* Call a function to initialize the memory pool handle */
    init_mem_pool_handle(&g_local_dscr_mem_pool_handle,
                         LOCAL_DSCR_MEM_POOL_BASE,
                         LOCAL_DSCR_MEM_SIZE,
                         g_local_dscr_mem_sub_pool_arr,
                         LOCAL_DSCR_MEM_NUM_SUB_POOLS);

    /* Call a function to initialize the memory pool */
    init_mem_pool(&g_local_dscr_mem_pool_handle,
                  (UWORD16 *)local_dscr_mem_sub_pool_info,
                  LOCAL_DSCR_MEM_ALIGNMENT);
#endif /* LOCALMEM_TX_DSCR */
	TROUT_FUNC_EXIT;
}
