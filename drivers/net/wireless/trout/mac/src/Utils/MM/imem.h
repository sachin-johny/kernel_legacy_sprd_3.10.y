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
/*  File Name         : imem.h                                               */
/*                                                                           */
/*  Description       : This file contains the definitions and declarations  */
/*                      required by the memory manager.                      */
/*                                                                           */
/*  List of Functions : init_mem_pool_handle                                 */
/*                      set_native_pool_id                                   */
/*                      get_native_pool_id                                   */
/*                      set_pattern                                          */
/*                      get_pattern                                          */
/*                      set_num_users                                        */
/*                      get_num_users                                        */
/*                      decr_num_users                                       */
/*                      is_buf_in_pool                                       */
/*                      get_sub_pool_size                                    */
/*                      get_sub_pool_cnt                                     */
/*                      set_sub_pool_size                                    */
/*                      set_sub_pool_cnt                                     */
/*                      sub_pool_cmp                                         */
/*                      swap_sub_pool                                        */
/*                      merge_sub_pool                                       */
/*                      init_scratch_mem                                     */
/*                      reinit_scratch_mem                                   */
/*                      get_scratch_mem_idx                                  */
/*                      restore_scratch_mem_idx                              */
/*                      set_next_buffer_addr                                 */
/*                      get_next_buffer_addr                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef IMEM_H
#define IMEM_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */

/* The memory pool is divided into sub pools of fixed chunk sizes. The sub   */
/* pool information is maintained in an array in the following format. The   */
/* user needs to initialize this.                                            */
/*                                                                           */
/* Sub pool information array elements,                                      */
/* (Chunk Size x, Number of chunks of size x)                                */

#define CHUNK_SIZE_IDX 0
#define CHUNK_CNT_IDX  1

/* The memory sub pool is divided into chunks. Each chunk has a header that  */
/* contains information regarding the chunk. The structure of this header is */
/* as shown below.                                                           */
/*                                                                           */
/* +----------------+----------------+------------+-----------------------+  */
/* | Native pool ID |  Fixed Pattern |  Num Users | User-ID               |  */
/* +----------------+----------------+------------+-----------------------+  */
/* | 1 byte         |  1 byte        |   1 byte   |  1 byte               |  */
/* +----------------+----------------+------------+-----------------------+  */

#if 1
#define MEM_CHUNK_TROUT_WRITE_SIZE       8 //caisf: reserved 8 bytes for SPI transfer.

#define MEM_CHUNK_INFO_SIZE              (4+MEM_CHUNK_TROUT_WRITE_SIZE)
#define MEM_CHUNK_NATIVE_POOL_ID_OFFSET  (4+MEM_CHUNK_TROUT_WRITE_SIZE)
#define MEM_CHUNK_FIXED_PATTERN_OFFSET   (3+MEM_CHUNK_TROUT_WRITE_SIZE)
#define MEM_CHUNK_NUM_USERS_OFFSET       (2+MEM_CHUNK_TROUT_WRITE_SIZE)
#define MEM_CHUNK_USER_ID_OFFSET         (1+MEM_CHUNK_TROUT_WRITE_SIZE)
#else
#define MEM_CHUNK_INFO_SIZE              4
#define MEM_CHUNK_NATIVE_POOL_ID_OFFSET  4
#define MEM_CHUNK_FIXED_PATTERN_OFFSET   3
#define MEM_CHUNK_NUM_USERS_OFFSET       2
#define MEM_CHUNK_USER_ID_OFFSET         1
#endif

/* Patterns to check if the buffer is an allocated buffer or a free buffer   */

#define PATTERN_ALLOCATED_BUFFER 0xAA
#define PATTERN_FREE_BUFFER      0x55
#define PATTERN_LOCKED_BUFFER    0xFF

/* Maximum number of users for a buffer */

#define MAX_NUM_USERS 255

/* The scratch memory is maintained as an array of bytes (g_scratch_mem) and */
/* an index (g_scratch_mem_idx) to the first free byte in the same.          */

/* The maximum size of the scratch memory is the maximum array size          */

#define SCRATCH_MEM_MAX_SIZE 5440

/* Define the type of data structure for the memory manager sub pool */
#define DYNAMIC_LINKLIST_SUB_POOL
//#define STATIC_ARRAY_SUB_POOL

/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/

#define ACTUAL_CHUNK_SIZE(chunk_size) (chunk_size + MEM_CHUNK_INFO_SIZE)

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Memory sub pool header */
typedef struct
{
    UWORD16 free_chunk_cnt;            /* Number of free chunks present      */
    UWORD16 total_chunk_cnt;           /* Total chunks in the sub pool       */
    UWORD16 chunk_size;                /* Size of the chunks in sub pool     */

#ifdef STATIC_ARRAY_SUB_POOL
    void    **free_chunk_stack;        /* Stack to maintain the free chunks  */
#endif /* STATIC_ARRAY_SUB_POOL */

#ifdef DYNAMIC_LINKLIST_SUB_POOL
    UWORD8  *buffer_list_head;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

} mem_sub_pool_t;

/* Memory pool handle */
typedef struct
{
    UWORD8         *mem_pool_base;     /* Memory pool base address           */
    UWORD32        mem_pool_size;      /* Memory pool size                   */
    UWORD8         num_sub_pools;      /* Number of sub pools                */
    mem_sub_pool_t *sub_pool;          /* List of sub pools                  */
} mem_handle_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8  g_scratch_mem[SCRATCH_MEM_MAX_SIZE + MAX_WORD_ALIGNMENT_BUFFER];
extern UWORD16 g_scratch_mem_idx;
extern UWORD8  *g_scratch_mem_ptr;

extern mem_handle_t g_shared_dscr_mem_pool_handle;
extern mem_handle_t g_shared_pkt_mem_pool_handle;
extern mem_handle_t g_local_mem_pool_handle;

#ifdef MEM_DEBUG_MODE
extern UWORD16 g_min_scratch_mem_index;
#endif /* MEM_DEBUG_MODE */
/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/

#define SCRATCH_MEM_POOL_BASE ((UWORD32)g_scratch_mem)

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void init_mem_pool(mem_handle_t *mem_handle, UWORD16 *sub_pool_info,
                          UWORD8 pool_align);
extern BOOL_T validate_buffer(void *buffer);

extern void *scratch_mem_alloc_fn(UWORD16 size);

#ifdef ENABLE_SCRATCH_MEM_ESTIMATION
    #define scratch_mem_alloc(n) 0xF0000000;\
    {\
        volatile UWORD8 buffer[n] = {0};\
        buffer[n-1] = 0xFF;\
    }
#else /* ENABLE_SCRATCH_MEM_ESTIMATION */
    #define scratch_mem_alloc(n) scratch_mem_alloc_fn(n)
#endif /* ENABLE_SCRATCH_MEM_ESTIMATION */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function checks if a buffer address is in the range of a specified   */
/* memory pool.                                                              */
INLINE BOOL_T is_buf_in_pool(mem_handle_t *mem_handle, void *buffer)
{
    if(((UWORD32)buffer < (UWORD32)(mem_handle->mem_pool_base)) ||
       ((UWORD32)buffer > ((UWORD32)(mem_handle->mem_pool_base) +
                                     mem_handle->mem_pool_size)))
    {
        return BFALSE;
    }

    return BTRUE;
}

/* This function initializes the memory pool handle with given details */
INLINE void init_mem_pool_handle(mem_handle_t *mem_handle, UWORD32 base,
                                 UWORD32 size, mem_sub_pool_t *sub_pool,
                                 UWORD8 num)
{
    mem_handle->mem_pool_base = (UWORD8 *)base;
    mem_handle->mem_pool_size = size;
    mem_handle->num_sub_pools = num;
    mem_handle->sub_pool      = sub_pool;
}

/* This function sets the native pool ID in the given buffer */
INLINE void set_native_pool_id(void *buffer, UWORD8 id)
{
    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("set_native_pool_id: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return;
    }

    *((UWORD8 *)buffer - MEM_CHUNK_NATIVE_POOL_ID_OFFSET) = id;
}

/* This function gets the native pool ID from the given buffer */
INLINE UWORD8 get_native_pool_id(void *buffer)
{
    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("get_native_pool_id: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return 0;
    }

    return *((UWORD8 *)buffer - MEM_CHUNK_NATIVE_POOL_ID_OFFSET);
}

/* This function sets the pattern in the given buffer */
INLINE void set_pattern(void *buffer, UWORD8 pattern)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("set_pattern: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return;
    }

    *((UWORD8 *)buffer - MEM_CHUNK_FIXED_PATTERN_OFFSET) = pattern;
}

/* This function gets the pattern from the given buffer */
INLINE UWORD8 get_pattern(void *buffer)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("get_pattern: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return 0;
    }

    return *((UWORD8 *)buffer - MEM_CHUNK_FIXED_PATTERN_OFFSET);
}

/* This function sets the number of users in the given buffer */
INLINE void set_num_users(void *buffer, UWORD8 num_users)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("set_num_users: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return;
    }

    *((UWORD8 *)buffer - MEM_CHUNK_NUM_USERS_OFFSET) = num_users;
}

/* This function gets the number of users from the given buffer */
INLINE UWORD8 get_num_users(void *buffer)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("get_num_users: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return 0;
    }

    return *((UWORD8 *)buffer - MEM_CHUNK_NUM_USERS_OFFSET);
}

/* This function decrements the number of users by the specified value and   */
/* returns the updated number of users from the given buffer.                */
INLINE UWORD8 decr_num_users(void *buffer, UWORD8 decr_count)
{
    UWORD8 *ptr_num_users = ((UWORD8 *)buffer - MEM_CHUNK_NUM_USERS_OFFSET);

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("decr_num_users: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return 0;
    }

    *ptr_num_users = *ptr_num_users - decr_count;

    return (*ptr_num_users);
}

/* This function sets the number of users in the given buffer */
INLINE void set_user_id(void *buffer, UWORD8 user_id)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("set_user_id: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return ;
    }

    *((UWORD8 *)buffer - MEM_CHUNK_USER_ID_OFFSET) = user_id;
}

/* This function gets the number of users from the given buffer */
INLINE UWORD8 get_user_id(void *buffer)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("get_user_id: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return 0;
    }

    return *((UWORD8 *)buffer - MEM_CHUNK_USER_ID_OFFSET);
}

/* This function gets the sub pool chunk size */
INLINE UWORD16 get_sub_pool_size(UWORD16 *sub_pool_info, UWORD8 idx)
{
    return sub_pool_info[2 * idx + CHUNK_SIZE_IDX];
}

/* This function gets the sub pool chunk count */
INLINE UWORD16 get_sub_pool_cnt(UWORD16 *sub_pool_info, UWORD8 idx)
{
    return sub_pool_info[2 * idx + CHUNK_CNT_IDX];
}

/* This function sets the sub pool chunk size */
INLINE void set_sub_pool_size(UWORD16 *sub_pool_info, UWORD8 idx, UWORD16 val)
{
    sub_pool_info[2 * idx + CHUNK_SIZE_IDX] = val;
}

/* This function sets the sub pool chunk count */
INLINE void set_sub_pool_cnt(UWORD16 *sub_pool_info, UWORD8 idx, UWORD16 val)
{
    sub_pool_info[2 * idx + CHUNK_CNT_IDX] = val;
}

/* This function compares 2 memory sub pools based on the chunk size. Chunk  */
/* size of 0 is considered to be greater than non-zero chunk size.           */
/* The function returns 0 if the sizes are equal, +1 if the first sub pool   */
/* is greater and -1 if the first sub pool is lesser.                        */
INLINE SWORD8 sub_pool_cmp(UWORD16 *sub_pool_info, UWORD8 idx1, UWORD8 idx2)
{
    UWORD16 size1 = get_sub_pool_size(sub_pool_info, idx1);
    UWORD16 size2 = get_sub_pool_size(sub_pool_info, idx2);

    if(size1 == size2)
        return 0;

    if(size1 == 0)
        return 1;

    if(size2 == 0) /* (size1 != 0) && */
        return -1;

    if(size1 > size2) /* (size1 > 0) && (size2 > 0) */
        return 1;

    return -1;
}


/* This function swaps 2 memory sub pools - chunk size and number of chunks */
INLINE void swap_sub_pool(UWORD16 *sub_pool_info, UWORD8 idx1, UWORD8 idx2)
{
    UWORD16 size1 = 0;
    UWORD16 size2 = 0;
    UWORD16 cnt1  = 0;
    UWORD16 cnt2  = 0;

    size1 = get_sub_pool_size(sub_pool_info, idx1);
    cnt1  = get_sub_pool_cnt(sub_pool_info, idx1);

    size2 = get_sub_pool_size(sub_pool_info, idx2);
    cnt2  = get_sub_pool_cnt(sub_pool_info, idx2);

    set_sub_pool_size(sub_pool_info, idx1, size2);
    set_sub_pool_cnt(sub_pool_info, idx1, cnt2);

    set_sub_pool_size(sub_pool_info, idx2, size1);
    set_sub_pool_cnt(sub_pool_info, idx2, cnt1);
}

/* This function merges 2 memory sub pools of equal chunk size. The number   */
/* of chunks of the second sub pool is added to the first. The second sub    */
/* pool is then reset to 0 - chunk size and number of chunks.                */
INLINE void merge_sub_pool(UWORD16 *sub_pool_info, UWORD8 idx1, UWORD8 idx2)
{
    UWORD16 cnt1 = 0;
    UWORD16 cnt2 = 0;

    cnt1 = get_sub_pool_cnt(sub_pool_info, idx1);
    cnt2 = get_sub_pool_cnt(sub_pool_info, idx2);

    set_sub_pool_cnt(sub_pool_info, idx1, cnt1 + cnt2);

    set_sub_pool_size(sub_pool_info, idx2, 0);
    set_sub_pool_cnt(sub_pool_info, idx2, 0);
}

/* This function initializes the scratch memory (pointer and index) */
INLINE void init_scratch_mem(void)
{
    g_scratch_mem_ptr = (UWORD8 *)(WORD_ALIGN((UWORD32)g_scratch_mem));
    g_scratch_mem_idx = SCRATCH_MEM_MAX_SIZE;
}

/* This function re-initializes the scratch memory */
INLINE void reinit_scratch_mem(void)
{
    g_scratch_mem_idx = SCRATCH_MEM_MAX_SIZE;
}

/* This function gets the scratch memory index */
INLINE UWORD16 get_scratch_mem_idx(void)
{
#ifdef MEM_DEBUG_MODE
    g_mem_stats.scrsave++;
#endif /* MEM_DEBUG_MODE */

    return g_scratch_mem_idx;
}

/* This function restores the scratch memory index */
INLINE void restore_scratch_mem_idx(UWORD16 mem_idx)
{
    if(mem_idx > SCRATCH_MEM_MAX_SIZE)
    {
#ifdef MEM_DEBUG_MODE
        g_mem_stats.scrrestoreerr++;
#endif /* MEM_DEBUG_MODE */

        mem_idx = SCRATCH_MEM_MAX_SIZE;
    }

    g_scratch_mem_idx = mem_idx;

#ifdef MEM_DEBUG_MODE
    g_mem_stats.scrrestore++;
#endif /* MEM_DEBUG_MODE */
}

#ifdef MEM_DEBUG_MODE
/* This function returns the maximum size of Scratch Memory used from the */
/* last reset. If 0xFFFFFFFF is returned then it indicates indicates      */
/* insufficient Scratch Memory.                                           */
INLINE UWORD16 get_max_scratch_mem_usage(void)
{
    if(g_min_scratch_mem_index == 0xFFFF)
        return 0xFFFF;
    else
        return (SCRATCH_MEM_MAX_SIZE - g_min_scratch_mem_index);
}

/* Updates the counter for tracking the maximum scratch memory usage in */
/* the system. The input overflow should be set to indicate failure to  */
/* allocate scratch memory.                                             */
INLINE void update_scratch_mem_usage(BOOL_T overflow)
{
    if(overflow == BTRUE)
    {
        g_min_scratch_mem_index = 0xFFFF;
    }

    if(g_min_scratch_mem_index != 0xFFFF)
    {
        if(g_scratch_mem_idx < g_min_scratch_mem_index)
            g_min_scratch_mem_index = g_scratch_mem_idx;
    }
}

/* Reset the Max Scratch Memory usage tracker. Can be called on every */
/* Memory Manager initialization.                                     */
INLINE void reset_scratch_mem_usage(void)
{
    g_min_scratch_mem_index = SCRATCH_MEM_MAX_SIZE;
}
#endif /* MEM_DEBUG_MODE */

#ifdef DYNAMIC_LINKLIST_SUB_POOL
/* This function sets the native pool ID in the given buffer */
INLINE void set_next_buffer_addr(UWORD8 *buffer, UWORD8 *next_buffer_addr)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("set_next_buffer_addr: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return;
    }

    *((UWORD32 *)buffer) = (UWORD32)next_buffer_addr;
}

/* This function gets the native pool ID from the given buffer */
INLINE UWORD8 *get_next_buffer_addr(UWORD8 *buffer)
{

    if(BFALSE == validate_buffer(buffer))
    {
        TROUT_DBG2("get_next_buffer_addr: Invalid buffer passed: %x\n",(UWORD32)buffer);
        return 0;
    }

    return (UWORD8 *)(*((UWORD32 *)buffer));
}
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

#endif /* IMEM_H */
