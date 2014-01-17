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
/*  File Name         : imem_if.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions and declarations  */
/*                      for the memory manager interface.                    */
/*                                                                           */
/*  List of Functions : init_mac_mem_pools                                   */
/*                      get_mem_pool_hdl                                     */
/*                      lock_the_buffer                                      */
/*                      unlock_the_buffer                                    */
/*                      get_mem_pool_hdl_type                                */
/*                      get_num_free_packet_buffers                          */
/*                      get_num_used_tx_buffers                              */
/*                      get_num_used_rx_buffers                              */
/*                      init_pktmem_pri_table                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef IMEM_IF_H
#define IMEM_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "imem.h"
#include "csl_if.h"
#include "shared_mem_config.h"
#include <linux/vmalloc.h>
#include <asm-generic/memory_model.h>


#define SYS_RAM_256M

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Structure used to maintain packet reservation related information for */
/* different priority levels.                                            */
typedef struct
{
    UWORD16 num_resvd;  /* Number of buffers to be Reserved */
    UWORD16 num_thresh; /* Threshold for making buffer allocation decisions */
    UWORD16 num_allocd; /* Number of buffers currently alloced */
} pktmem_priority_t;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* This Enum contains the various priority levels supported by the Memory    */
/* Manager for allocating Packet Memory. Lower index in the priority table   */
/* indicates higher priority.                                                */

typedef enum {MEM_PRI_CONFIG = 0,
              MEM_PRI_HPTX   = 1,
              MEM_PRI_HPRX   = 2,
              MEM_PRI_RX     = 3,
              MEM_PRI_TX     = 4,
              NUM_MEM_PRI_LEVELS = 5
} MEM_PRIORITY_LEVEL_T;

//add by chengwg.
typedef struct
{
	UWORD8 *shared_pkt_mem_pool_base;
	//UWORD8 *local_mem_pool_base;
	UWORD8 *shared_dscr_mem_pool_base;
	//UWORD8 *scratch_mem_pool_base;
#ifdef LOCALMEM_TX_DSCR
	UWORD8 *local_dscr_mem_pool_base;
#endif	
}share_mem_s;


/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
//add by chengwg.
extern share_mem_s g_shared_mem_base;

extern mem_handle_t g_shared_dscr_mem_pool_handle;
extern mem_handle_t g_shared_pkt_mem_pool_handle;
extern mem_handle_t g_local_mem_pool_handle;
#ifdef LOCALMEM_TX_DSCR
extern mem_handle_t g_local_dscr_mem_pool_handle;
extern void *g_local_mem_dma;
#endif /* LOCALMEM_TX_DSCR */
extern pktmem_priority_t g_pktmem_pri[NUM_MEM_PRI_LEVELS];

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#define g_shared_dscr_mem_handle (&g_shared_dscr_mem_pool_handle)
#define g_shared_pkt_mem_handle  (&g_shared_pkt_mem_pool_handle)
#define g_local_mem_handle       (&g_local_mem_pool_handle)

#ifdef LOCALMEM_TX_DSCR
#define g_local_dscr_mem_handle  (&g_local_dscr_mem_pool_handle)
#endif /* LOCALMEM_TX_DSCR */

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   init_mem_manager(void);
extern void   init_shared_dscr_mem(void);
extern void   init_shared_pkt_mem(void);
extern void   init_local_mem(void);
extern void   *mem_alloc(mem_handle_t *mh, UWORD16 size);
extern SWORD8 mem_free(mem_handle_t *mh, void *buffer_addr);
extern UWORD8 mem_add_users(mem_handle_t *mh, void *buffer_addr, UWORD8 cnt);
extern int init_mem_regions(void);
extern void free_mem_regions(void); //add by Hugh
extern void   *pkt_mem_alloc(MEM_PRIORITY_LEVEL_T pri_level);
extern void   pkt_mem_free(void *pkt_buff);
extern void get_wifi_buf(unsigned long *start, unsigned long *size);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/
#ifdef TROUT_TRACE_DBG
INLINE void pkt_mem_pri_table_show(void)
{
	int i;
	
	for(i = 0; i < NUM_MEM_PRI_LEVELS; i++)
    {
		TROUT_DBG6("packet memory priority(%d) Table:\n", i);
		TROUT_DBG6("num_resvd:  %d\n", g_pktmem_pri[i].num_resvd);
		TROUT_DBG6("num_thresh: %d\n", g_pktmem_pri[i].num_thresh);
		TROUT_DBG6("num_allocd: %d\n", g_pktmem_pri[i].num_allocd);
    }
}

INLINE void print_mem_info(UWORD8 *name, mem_handle_t *mem_handle)
{
	int i;
	
	TROUT_DBG6("%s(0x%p) memory pool detail:\n", name, mem_handle);
	TROUT_DBG6("mem_pool_base: 0x%p\n", mem_handle->mem_pool_base);
	TROUT_DBG6("mem_pool_size: 0x%08x\n", mem_handle->mem_pool_size);
	TROUT_DBG6("num_sub_pools: %d\n", mem_handle->num_sub_pools);
	TROUT_DBG6("sub_pool:      0x%p\n", mem_handle->sub_pool);

	for(i=0; i<mem_handle->num_sub_pools; i++)
	{
		if(mem_handle->sub_pool[i].total_chunk_cnt == 0)
			continue;
		TROUT_DBG6("mem_handle 0x%p sub pool %d info:\n", mem_handle, i);
		TROUT_DBG6("free_chunk_cnt:   %d\n", mem_handle->sub_pool[i].free_chunk_cnt);
		TROUT_DBG6("total_chunk_cnt:  %d\n", mem_handle->sub_pool[i].total_chunk_cnt);
		TROUT_DBG6("chunk_size:       %d\n", mem_handle->sub_pool[i].chunk_size);
		TROUT_DBG6("buffer_list_head: 0x%p\n", mem_handle->sub_pool[i].buffer_list_head);
	}
}

INLINE void mac_mem_detail_show(void)
{
	print_mem_info("g_shared_dscr_mem_pool_handle", &g_shared_dscr_mem_pool_handle);
	print_mem_info("g_shared_pkt_mem_pool_handle", &g_shared_pkt_mem_pool_handle);
	print_mem_info("g_local_mem_pool_handle", &g_local_mem_pool_handle);
	pkt_mem_pri_table_show();
}
#endif	/* TROUT_TRACE_DBG */

/* This function initializes the memory manager and all the MAC memory pools */
INLINE int init_mac_mem_pools(void)
{
    init_mem_manager();
    
    if(init_mem_regions() < 0)
    	return -1;
    
    init_shared_dscr_mem();
    init_shared_pkt_mem();
    init_local_mem();
    
#ifdef TROUT_TRACE_DBG
	mac_mem_detail_show();
#endif	/* TROUT_TRACE_DBG */

	return 0;
}


/* This function returns the memory pool to which the buffer belongs based   */
/* on the buffer address range.                                              */
INLINE mem_handle_t *get_mem_pool_hdl(void *buffer)
{
    if(is_buf_in_pool(g_local_mem_handle, buffer) == BTRUE)
        return g_local_mem_handle;

#ifdef LOCALMEM_TX_DSCR
    if(is_buf_in_pool(g_local_dscr_mem_handle, buffer) == BTRUE)
        return g_local_dscr_mem_handle;
#endif /* LOCALMEM_TX_DSCR */

    if(is_buf_in_pool(g_shared_dscr_mem_handle, buffer) == BTRUE)
        return g_shared_dscr_mem_handle;

    if(is_buf_in_pool(g_shared_pkt_mem_handle, buffer) == BTRUE)
        return g_shared_pkt_mem_handle;

    return 0;
}

INLINE void lock_the_buffer(void *buffer)
{
    /* Lock the buffer if it is an allocated buffer */
    if(get_pattern(buffer) == PATTERN_ALLOCATED_BUFFER)
    {
        set_pattern(buffer,PATTERN_LOCKED_BUFFER);
    }
}

INLINE void unlock_the_buffer(void *buffer)
{
    /* Unlock the buffer only if it is a locked one */
    if(get_pattern(buffer) == PATTERN_LOCKED_BUFFER)
    {
        set_pattern(buffer,PATTERN_ALLOCATED_BUFFER);
    }
}
#ifdef DEBUG_MODE
/* This function returns the memory pool to which the buffer belongs based   */
/* on the buffer address range.                                              */
INLINE UWORD8 *get_mem_pool_hdl_type(mem_handle_t *mem_pool)
{
    if(g_local_mem_handle == mem_pool)
        return "LM";

#ifdef LOCALMEM_TX_DSCR
    if(g_local_dscr_mem_handle == mem_pool)
        return "LDM";
#endif /* LOCALMEM_TX_DSCR */

    if(g_shared_dscr_mem_handle == mem_pool)
        return "SDM";

    if(g_shared_pkt_mem_handle == mem_pool)
        return "SPM";

    return 0;
}
#endif /* DEBUG_MODE */

/* This function returns the number of free packet buffers */
INLINE UWORD16 get_num_free_packet_buffers(void)
{
    return g_shared_pkt_mem_pool_handle.sub_pool[SHARED_PKT_MEM_TXRX_BUFFER_IDX].free_chunk_cnt;
}

/* This function returns the number of packet buffers which are currently */
/* used in the TX path.                                                   */
INLINE UWORD16 get_num_used_tx_buffers(void)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    return (g_pktmem_pri[MEM_PRI_HPTX].num_allocd + g_pktmem_pri[MEM_PRI_TX].num_allocd);
#else /* MAC_HW_UNIT_TEST_MODE */
    /* This number is not valid in HUT Mode */
    return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function returns the number of packet buffers which are currently */
/* used in the RX path.                                                   */
INLINE UWORD16 get_num_used_rx_buffers(void)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    return (g_pktmem_pri[MEM_PRI_HPRX].num_allocd + g_pktmem_pri[MEM_PRI_RX].num_allocd);
#else /* MAC_HW_UNIT_TEST_MODE */
    /* This number is not valid in HUT Mode */
    return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

INLINE UWORD16 total_pending_pkt_cnt(void)
{
    return (get_num_used_tx_buffers() + get_num_used_rx_buffers());
}

/* This function initializes the Packet Memory Priority Table */
INLINE void init_pktmem_pri_table(pktmem_priority_t *pri_table,
                                  UWORD32 num_pri_levels)
{
    UWORD32 i = 0;
    UWORD32 thresh = 0;

    /* There is no threshold applicable for the highest priority module */
    pri_table[0].num_thresh = 0;
    pri_table[0].num_allocd = 0;
    thresh = pri_table[0].num_resvd;

    for(i = 1; i < num_pri_levels; i++)
    {
        pri_table[i].num_thresh = thresh;
        pri_table[i].num_allocd = 0;

        thresh += pri_table[i].num_resvd;
    }  
}


#endif /* IMEM_IF_H */
