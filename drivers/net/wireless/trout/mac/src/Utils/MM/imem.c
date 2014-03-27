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
/*  File Name         : imem.c                                               */
/*                                                                           */
/*  Description       : This file contains the functions related to the      */
/*                      memory manager.                                      */
/*                                                                           */
/*  List of Functions : init_mem_manager                                     */
/*                      init_mem_pool                                        */
/*                      init_sub_pool                                        */
/*                      mem_alloc                                            */
/*                      mem_free                                             */
/*                      mem_add_users                                        */
/*                      scratch_mem_alloc_fn                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "host_if.h"
#include "imem_if.h"
#include "imem.h"
#include "local_mem_config.h"

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/

#ifdef STATIC_ARRAY_SUB_POOL
static UWORD8 *g_mem_pool_stack[MAX_SMEM_STACK_SZ + MAX_LMEM_STACK_SZ] = {0};
static UWORD8 **g_mem_stack_ptr                          = 0;
#endif /* STATIC_ARRAY_SUB_POOL */

UWORD8 g_scratch_mem[SCRATCH_MEM_MAX_SIZE + MAX_WORD_ALIGNMENT_BUFFER] = {0};

UWORD8  *g_scratch_mem_ptr = 0;
UWORD16 g_scratch_mem_idx  = 0;
BOOL_T  g_scratch_mem_init = BFALSE;

#ifdef MEM_DEBUG_MODE
UWORD16 g_min_scratch_mem_index = SCRATCH_MEM_MAX_SIZE;
#endif /* MEM_DEBUG_MODE */

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static UWORD8 *init_sub_pool(mem_sub_pool_t *sub_pool, UWORD8 *base,
                            UWORD8 mem_id, UWORD16 chunk_size,
                            UWORD16 total_chunk_cnt);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_mem_manager                                         */
/*                                                                           */
/*  Description   : This function initializes the memory manager.            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mem_stack_ptr                                          */
/*                  g_mem_pool_stack                                         */
/*                                                                           */
/*  Processing    : This function initializes the global stack pointer to    */
/*                  the base of the memory pool stack maintained as a        */
/*                  separate array. This is used by the memory manager to    */
/*                  manage the various pools.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_mem_manager(void)
{
	TROUT_FUNC_ENTER;
#ifdef STATIC_ARRAY_SUB_POOL
    g_mem_stack_ptr = g_mem_pool_stack;
#endif /* STATIC_ARRAY_SUB_POOL */

#ifdef MEM_DEBUG_MODE
    g_mem_stats.sdtotalfree = 0;
    g_mem_stats.sptotalfree = 0;
    g_mem_stats.ltotalfree  = 0;
    g_mem_stats.etotalfree  = 0;
#endif /* MEM_DEBUG_MODE */

    if(g_scratch_mem_init == BFALSE)
    {
        init_scratch_mem();
        g_scratch_mem_init = BTRUE;
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_mem_pool                                            */
/*                                                                           */
/*  Description   : This function initializes a memory pool.                 */
/*                                                                           */
/*  Inputs        : 1) Pointer to the memory handle structure                */
/*                  2) Pointer to the memory sub pool information list       */
/*                  3) Alignment Requirements of the Memory Pool             */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The memory sub pool information list contains the        */
/*                  details of all the sub pools in this memory pool (this   */
/*                  includes chunk size, number of chunks). This information */
/*                  is extracted for each sub pool from the given list and   */
/*                  the corresponding sub pool is initialized.               */
/*                  The function also tries to enforce the required alignment*/
/*                  on the sub-pools created by it. The individual sub-pools */
/*                  can all have their own buffer alignment requirements.    */
/*                  However, the length of the sub-pool(including            */
/*                  MEM_CHUNK_INFO_SIZE) should be a multiple of the         */
/*                  alignment length. The alignment requirement for the      */
/*                  pool (indicated by pool_align argument) is then the LCM  */
/*                  of the alignment requirements of the sub-pools included  */
/*                  in it.                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_mem_pool(mem_handle_t *mem_handle, UWORD16 *sub_pool_info,
                   UWORD8 pool_align)
{
    UWORD8  i                   = 0;
    UWORD16 chunk_size          = 0;
    UWORD16 total_chunk_cnt     = 0;
    UWORD8  *mem_pool_base_addr = 0;
    UWORD32 algn_offset         = 0;

	TROUT_FUNC_ENTER;
    /* Initialize the sub pools with the actual memory pool addresses. Also  */
    /* initialize the stack pointer values. The global g_mem_stack_ptr is    */
    /* used to track the stack pointer memory space.                         */
    mem_pool_base_addr = mem_handle->mem_pool_base;

    /* The Memory Pool Base Address is modified such that the buffer address */
    /* has the desired alignment after the overhead of the buffer headers are*/
    /* accounted for.                                                        */
    if(pool_align != 0)
    {
        algn_offset = get_align_offset((UWORD32)mem_pool_base_addr, pool_align);
        if(algn_offset < MEM_CHUNK_INFO_SIZE)
            algn_offset += pool_align;
        algn_offset -= MEM_CHUNK_INFO_SIZE;
        mem_pool_base_addr += algn_offset;
    }

    for(i = 0; i < mem_handle->num_sub_pools; i++)
    {
        /* The sub pool information field is received in the following       */
        /* format (Chunk Size x, Number of chunks of size x). The elements   */
        /* are accessed as a single dimension array.                         */
        chunk_size      = sub_pool_info[2 * i + CHUNK_SIZE_IDX];
        total_chunk_cnt = sub_pool_info[2 * i + CHUNK_CNT_IDX];
		
		mem_handle->sub_pool[i].chunk_size      = 0;
		mem_handle->sub_pool[i].total_chunk_cnt = 0;
		mem_handle->sub_pool[i].free_chunk_cnt  = 0;

        if(total_chunk_cnt > 0)
        {
			/* Initialize the sub pool with the given number of chunks and the   */
			/* given chunk size.                                                 */
			mem_pool_base_addr = init_sub_pool(
						  &(mem_handle->sub_pool[i]), /* Memory sub pool handle  */
						  mem_pool_base_addr,         /* Sub pool base address   */
						  i,                          /* Sub pool memory ID      */
						  chunk_size,                 /* Sub pool chunk size     */
						  total_chunk_cnt);           /* Total chunk count       */
			
			/* The Memory Pool Base Address is realigned for each sub-pool.      */
			/* This allows each sub-pool to have its own alignment while wasting */
			/*  minimum memory.                                                  */
			if(pool_align != 0)
			{
				algn_offset = get_align_offset((UWORD32)mem_pool_base_addr, pool_align);
				if(algn_offset < MEM_CHUNK_INFO_SIZE)
					algn_offset += pool_align;
				algn_offset -= MEM_CHUNK_INFO_SIZE;
				mem_pool_base_addr += algn_offset;
			}

#ifdef MEM_DEBUG_MODE
			if(mem_handle == g_shared_dscr_mem_handle)
				g_mem_stats.sdtotalfree += total_chunk_cnt;
			else if(mem_handle == g_shared_pkt_mem_handle)
				g_mem_stats.sptotalfree += total_chunk_cnt;
			else if(mem_handle == g_local_mem_handle)
				g_mem_stats.ltotalfree += total_chunk_cnt;
#endif /* MEM_DEBUG_MODE */
		}
    }
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name :  init_sub_pool                                           */
/*                                                                           */
/*  Description   :  This function initializes a memory pool.                */
/*                                                                           */
/*  Inputs        :  1) Pointer to the memory sub pool header                */
/*                   2) Memory sub pool base address                         */
/*                   3) Memory sub pool identifier                           */
/*                   4) Memory sub pool chunk size                           */
/*                   5) Total chunk count for the sub pool                   */
/*                                                                           */
/*  Globals       :  g_mem_stack_ptr                                         */
/*                                                                           */
/*  Processing    :  This function initializes the free chunk stack for the  */
/*                   memory sub pool with the actual chunk buffer addresses. */
/*                   It also updates the other information in the chunk that */
/*                   include the native pool identifier.                     */
/*                                                                           */
/*  Outputs       :  The memory sub pool free chunk stack is initialized     */
/*                                                                           */
/*  Returns       :  WORD8 *, Next available base address                    */
/*                                                                           */
/*  Issues        :  None                                                    */
/*                                                                           */
/*****************************************************************************/
#if 0
UWORD8 *init_sub_pool(mem_sub_pool_t *sub_pool, UWORD8 *base, UWORD8 mem_id,
                   UWORD16 chunk_size, UWORD16 total_chunk_cnt)
{
    UWORD16 i = 0;
#ifdef DYNAMIC_LINKLIST_SUB_POOL
    UWORD8 *last_buffer_addr = 0;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

	//UWORD8 *p = NULL;

	TROUT_FUNC_ENTER;
    /* Initialize the chunk size and the total chunk count for the memory    */
    /* sub pool.                                                             */
    sub_pool->chunk_size      = chunk_size;
    sub_pool->total_chunk_cnt = total_chunk_cnt;
    sub_pool->free_chunk_cnt  = 0;

#ifdef STATIC_ARRAY_SUB_POOL
    /* Initialize the free chunk stack pointer for the sub pool to the       */
    /* global stack pointer.                                                 */
    sub_pool->free_chunk_stack = (void **)g_mem_stack_ptr;

    /* Update the global stack pointer to point to the base address of the   */
    /* free chunk stack of the next memory pool/sub pool.                    */
    g_mem_stack_ptr += total_chunk_cnt;
#endif /* STATIC_ARRAY_SUB_POOL */

    /* Initialize the elements for the sub pool free chunk stack with the    */
    /* chunk buffer memory addresses and native pool identifier.             */
    for(i = 0; i < total_chunk_cnt; i++)
    {
        UWORD8 *buffer_addr = 0;

        buffer_addr = (base + MEM_CHUNK_INFO_SIZE);

        if(get_pattern(buffer_addr) != PATTERN_LOCKED_BUFFER)
        {
            set_native_pool_id(buffer_addr, mem_id);
            set_pattern(buffer_addr,PATTERN_FREE_BUFFER);
            set_num_users(buffer_addr, 0);

#ifdef STATIC_ARRAY_SUB_POOL
            /* Add the starting address of the buffer in the free chunk stack */
            sub_pool->free_chunk_stack[sub_pool->free_chunk_cnt] = buffer_addr;
#endif /* STATIC_ARRAY_SUB_POOL */

#ifdef DYNAMIC_LINKLIST_SUB_POOL
            /* Set the next buffer address in this buffer. This link is used */
            /* by the memory manager only.                                   */
            set_next_buffer_addr(buffer_addr, last_buffer_addr);
            last_buffer_addr = buffer_addr;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

            /* Increment the count of number of free buffers */
            sub_pool->free_chunk_cnt += 1;
        }
        else
        {
            /* There can be only 1 user of a locked buffer. The events */
            /* corresponding to the other users will be deleted during */
            /* the S/w Reset.                                          */
            set_num_users(buffer_addr, 1);
        }

		//p = buffer_addr - MEM_CHUNK_NATIVE_POOL_ID_OFFSET;
		//TROUT_DBG5("buffer_addr(0x%p): id=%d, pattern=%d, user_num=%d, user_id=%d, next_buff(0x%08x)\n", 
		//				buffer_addr, p[0], p[1], p[2], p[3], *((UWORD32 *)buffer_addr));
		
        /* Update the sub pool base address to point to the base address of  */
        /* the next chunk.                                                   */
        base += ACTUAL_CHUNK_SIZE(chunk_size);
    }

#ifdef DYNAMIC_LINKLIST_SUB_POOL
    /* Set the sub pool buffer list head to last buffer added to the list */
    sub_pool->buffer_list_head = last_buffer_addr;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

	TROUT_FUNC_EXIT;
    return base;
}
#else	//modify by chengwg.
UWORD8 *init_sub_pool(mem_sub_pool_t *sub_pool, UWORD8 *base, UWORD8 mem_id,
                   UWORD16 chunk_size, UWORD16 total_chunk_cnt)
{
    UWORD16 i = 0;
#ifdef DYNAMIC_LINKLIST_SUB_POOL
    UWORD8 *last_buffer_addr = 0;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

	TROUT_FUNC_ENTER;
    /* Initialize the chunk size and the total chunk count for the memory    */
    /* sub pool.                                                             */
    sub_pool->chunk_size      = chunk_size;
    sub_pool->total_chunk_cnt = total_chunk_cnt;
    sub_pool->free_chunk_cnt  = 0;

#ifdef STATIC_ARRAY_SUB_POOL
    /* Initialize the free chunk stack pointer for the sub pool to the       */
    /* global stack pointer.                                                 */
    sub_pool->free_chunk_stack = (void **)g_mem_stack_ptr;

    /* Update the global stack pointer to point to the base address of the   */
    /* free chunk stack of the next memory pool/sub pool.                    */
    g_mem_stack_ptr += total_chunk_cnt;
#endif /* STATIC_ARRAY_SUB_POOL */

    /* Initialize the elements for the sub pool free chunk stack with the    */
    /* chunk buffer memory addresses and native pool identifier.             */
    for(i = 0; i < total_chunk_cnt; i++)
    {
        UWORD8 *buffer_addr = 0;

        buffer_addr = (base + MEM_CHUNK_INFO_SIZE);

    	set_pattern(buffer_addr, PATTERN_FREE_BUFFER);
        set_native_pool_id(buffer_addr, mem_id);
        set_pattern(buffer_addr,PATTERN_FREE_BUFFER);
        set_num_users(buffer_addr, 0);

#ifdef STATIC_ARRAY_SUB_POOL
        /* Add the starting address of the buffer in the free chunk stack */
        sub_pool->free_chunk_stack[sub_pool->free_chunk_cnt] = buffer_addr;
#endif /* STATIC_ARRAY_SUB_POOL */

#ifdef DYNAMIC_LINKLIST_SUB_POOL
        /* Set the next buffer address in this buffer. This link is used */
        /* by the memory manager only.                                   */
        set_next_buffer_addr(buffer_addr, last_buffer_addr);
        last_buffer_addr = buffer_addr;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

        /* Increment the count of number of free buffers */
        sub_pool->free_chunk_cnt += 1;
		
        /* Update the sub pool base address to point to the base address of  */
        /* the next chunk.                                                   */
        base += ACTUAL_CHUNK_SIZE(chunk_size);
    }

#ifdef DYNAMIC_LINKLIST_SUB_POOL
    /* Set the sub pool buffer list head to last buffer added to the list */
    sub_pool->buffer_list_head = last_buffer_addr;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

	TROUT_FUNC_EXIT;
    return base;
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name :  mem_alloc                                               */
/*                                                                           */
/*  Description   :  This function allocates a buffer of requested size.     */
/*                                                                           */
/*  Inputs        :  1) Pointer to the memory pool handle                    */
/*                   2) Buffer size                                          */
/*                                                                           */
/*  Globals       :  None                                                    */
/*                                                                           */
/*  Processing    :  The memory pool is searched for the sub pool having a   */
/*                   chunk size greater than or equal to the one requested.  */
/*                   The buffer if found is returned to the user.            */
/*                                                                           */
/*  Outputs       :  None                                                    */
/*                                                                           */
/*  Returns       :  Pointer to the buffer                                   */
/*                   NULL, in case no buffer could be allocated              */
/*                                                                           */
/*  Issues        :  None                                                    */
/*                                                                           */
/*****************************************************************************/

void *mem_alloc(mem_handle_t *mem_handle, UWORD16 size)
{
    UWORD8         i         = 0;
    UWORD8         *temp     = 0;
    UWORD8         pattern   = 0;
    mem_sub_pool_t *sub_pool = 0;
    int smart_type = 0;
	
    /* Start of critical section. Disable all interrupts. Hugh: enforce SMP protect */
    smart_type = critical_section_smart_start(0,1);

    /* Search for a sub pool with chunk size satisfying the request */
    for(i = 0; i < mem_handle->num_sub_pools; i++)
    {
        sub_pool = &(mem_handle->sub_pool[i]);

        if(sub_pool == NULL)
        {
			PRINTD("MemMgrErr: Subpool NULL\n");
			critical_section_smart_end(smart_type);
			return NULL;
		}

        if(sub_pool->chunk_size >= size)
            break;
    }

    /* Exception Case 1: No sub pools are found with chunk size greater than */
    /* the requested size                                                    */
    if(i == mem_handle->num_sub_pools)
    {
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s NoSizeAlloc : %d\n\r", pt, size);
        }
#endif /* DEBUG_MODE */
#ifdef MEM_DEBUG_MODE
        g_mem_stats.nosizeallocexc++;
#endif /* MEM_DEBUG_MODE */
	  critical_section_smart_end(smart_type);
        return NULL;
    }

    if(smart_type == MODE_START){
    	/* Start of critical section. Disable all interrupts */
    	smart_type = critical_section_smart_start(1,0);
    }

    /* Exception Case 2: No free chunks are available in the sub pool */
    if(sub_pool->free_chunk_cnt == 0)
    {
        /* End of critical section. Enable all interrupts */
        critical_section_smart_end(smart_type);

#ifdef MEM_DEBUG_MODE
        g_mem_stats.nofreeallocexc++;

        if(mem_handle == g_shared_dscr_mem_handle)
        {
            g_mem_stats.nofreeDscrallocexc[i]++;
        }
        else if(mem_handle == g_shared_pkt_mem_handle)
        {
            g_mem_stats.nofreePktallocexc[i]++;
        }
        else if(mem_handle == g_local_mem_handle)
        {
            g_mem_stats.nofreeLocalallocexc[i]++;
        }
        else
        {
            g_mem_stats.nofreeEventallocexc++;
        }
#endif /* MEM_DEBUG_MODE */

        return NULL;
    }

#ifdef STATIC_ARRAY_SUB_POOL
    /* Decrement the free_chunk_cnt associated with the memory sub pool and  */
    /* get the corresponding free chunk from the stack.                      */
    temp      = sub_pool->free_chunk_stack[--(sub_pool->free_chunk_cnt)];
#endif /* STATIC_ARRAY_SUB_POOL */

#ifdef DYNAMIC_LINKLIST_SUB_POOL
    /* Get the buffer at the sub pool buffer list head. Update the sub pool  */
    /* buffer list head to the address saved in the next buffer address in   */
    /* the buffer. Decrement the free chunk count for this sub-pool.         */
    temp                       = sub_pool->buffer_list_head;
    sub_pool->buffer_list_head = get_next_buffer_addr(temp);
    (sub_pool->free_chunk_cnt)--;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

    pattern   = get_pattern(temp);
    if(pattern == PATTERN_FREE_BUFFER)
    {
        set_pattern(temp, PATTERN_ALLOCATED_BUFFER);

        /* More buffer users can be added later by using mem_add_user api */
        set_num_users(temp, 1);
    }
    else
    {
        critical_section_smart_end(smart_type);

#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s AllcErrPtrn : %d\n\r",pt,pattern);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        if(pattern == PATTERN_ALLOCATED_BUFFER)
             g_mem_stats.reallocexc++;
        else
            g_mem_stats.corruptallocexc++;
#endif /* MEM_DEBUG_MODE */

    return NULL;
    }

#ifdef MEM_DEBUG_MODE
    if(mem_handle == g_local_mem_handle)
    {
        g_mem_stats.ltotalfree--;
        g_mem_stats.lalloc++;
    }
    else if(mem_handle == g_shared_dscr_mem_handle)
    {
        g_mem_stats.sdtotalfree--;
        g_mem_stats.sdalloc++;
    }
    else if(mem_handle == g_shared_pkt_mem_handle)
    {
        g_mem_stats.sptotalfree--;
        g_mem_stats.spalloc++;
    }
#endif /* MEM_DEBUG_MODE */

   /* End of critical section. Enable all interrupts */
    critical_section_smart_end(smart_type);

    /* Return the address of the buffer offset by chunk information size     */
    /* from the actual chunk address to the user.                            */

    return ((void *)(UWORD8 *)temp);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mem_free                                                 */
/*                                                                           */
/*  Description   : This function deallocates the given buffer.              */
/*                                                                           */
/*  Inputs        : 1) Pointer to the memory pool handle                     */
/*                  2) Pointer to the buffer                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The sub pool identifier is extracted from the buffer and */
/*                  the appropriate sub pool free chunk stack is updated     */
/*                  with the freed buffer.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : -1 on Error                                              */
/*                  Number of Pending Buffer Users (0 on Successful free)    */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

SWORD8 mem_free(mem_handle_t *mem_handle, void *buffer_addr)
{
    UWORD8         native_id  = 0;
    UWORD8         pattern    = 0;
    mem_sub_pool_t *sub_pool  = 0;
    UWORD8         num_users  = 0;
    int smart_type = 0;


    /* Exception Case 1: Buffer address to be freed is NULL */
    if (buffer_addr == NULL)
    {
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s NullFreeExc\n\r",pt);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        g_mem_stats.nullfreeexc++;
#endif /* MEM_DEBUG_MODE */

        return -1;
    }

    /* Exception Case 2: Buffer address is out of the pool boundary */
    if(is_buf_in_pool(mem_handle, buffer_addr) == BFALSE)
    {
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s OOBFreeExc\n",pt);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        g_mem_stats.oobfreeexc++;
#endif /* MEM_DEBUG_MODE */
		printk("trout:err: buf addr(0x%p) is out of pool!\n", buffer_addr);
        return -1;
    }

    /* Extract the native pool identifier and find the sub pool to which     */
    /* this buffer belongs.                                                  */
    native_id = get_native_pool_id(buffer_addr);

    /* Exception Case 3: Sub pool ID extracted is invalid */
    if(native_id >= mem_handle->num_sub_pools)
    {
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s InvldFreeExc: Addr:%x -1:%x 0:%x\n", pt,
                (UWORD32)buffer_addr, *(((UWORD32*)buffer_addr) - 1),
                *((UWORD32*)buffer_addr));
        }
#endif /* DEBUG_MODE */
#ifdef MEM_DEBUG_MODE
        g_mem_stats.invalidfreeexc++;
#endif /* MEM_DEBUG_MODE */
        printk("trout:err: native_id(%d) >= num_sub_pools(%d)\n", native_id, mem_handle->num_sub_pools);
        return -1;
    }

    sub_pool = &(mem_handle->sub_pool[native_id]);

    /* Start of critical section. Disable all interrupts */
	smart_type = critical_section_smart_start(1,1);
    //critical_section_start1();

    pattern = get_pattern(buffer_addr);

    /* Exception Case 4: Buffer is already freed or header is corrupt */
    if((pattern != PATTERN_ALLOCATED_BUFFER) && (pattern != PATTERN_LOCKED_BUFFER))
    {
        /* End of critical section. Enable all interrupts */
	  critical_section_smart_end(smart_type);
        //critical_section_end1();

#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s FreeErrPtrn : %d\n",pt,pattern);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
       if(pattern == PATTERN_FREE_BUFFER)
           g_mem_stats.refreeexc++;
       else
           g_mem_stats.corruptfreeexc++;
#endif /* MEM_DEBUG_MODE */

        printk("trout:pattern(0x%x) err, addr: 0x%08x!\n", pattern, (UWORD32)buffer_addr);
       return -1;
    }

    /* Decrement the number of buffer users by 1. Buffer is freed only when */
    /* the number of users is 0.                                            */
    num_users = decr_num_users(buffer_addr, 1);

    /* Check whether any more users of the buffer still exist */
    if(num_users != 0)
    {
        /* End of critical section. Enable all interrupts */
	  critical_section_smart_end(smart_type);
        //critical_section_end1();

        return num_users;
    }

    /* Exception Case 5: Locked buffer freed without unlocking. */
    if((num_users == 0) && (pattern == PATTERN_LOCKED_BUFFER))
    {
        /* A Locked buffer will definitely have one user */
        set_num_users(buffer_addr, 1);

        /* End of critical section. Enable all interrupts */
		critical_section_smart_end(smart_type);
       // critical_section_end1();

#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s LockedFreeExc\n\r",pt);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        g_mem_stats.lockedfreeexc++;
#endif /* MEM_DEBUG_MODE */

        return 1;
    }

    /* Exception Case 6: Free chunks exceed total chunks */
    if(sub_pool->free_chunk_cnt >= sub_pool->total_chunk_cnt)
    {

        /* End of critical section. Enable all interrupts */
		critical_section_smart_end(smart_type);
       // critical_section_end1();

#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s ExcessFreeExc\n\r",pt);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        g_mem_stats.excessfreeexc++;
#endif /* MEM_DEBUG_MODE */

        printk("trout:pattern(%u) >= total_chunk_cnt(%u).\n", sub_pool->free_chunk_cnt, sub_pool->total_chunk_cnt);
        return -1;
    }

    /* Mark as free buffer*/
    set_pattern(buffer_addr, PATTERN_FREE_BUFFER);
// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MEM_DEBUG_MODE_1
    memset(buffer_addr, 0xAA, sub_pool->chunk_size);
#endif /* MEM_DEBUG_MODE_1 */


#ifdef STATIC_ARRAY_SUB_POOL
    /* Update the free chunk stack associated with the sub pool with the     */
    /* freed chunk.                                                          */
    sub_pool->free_chunk_stack[(sub_pool->free_chunk_cnt)++] = buffer_addr;
#endif /* STATIC_ARRAY_SUB_POOL */

#ifdef DYNAMIC_LINKLIST_SUB_POOL
    /* Link this buffer to the sub pool buffer list head by updating the     */
    /* next buffer address in this buffer. Update the sub pool buffer list   */
    /* head to this buffer. Increment the free chunk count for this sub-pool */
    set_next_buffer_addr(buffer_addr, sub_pool->buffer_list_head);
    sub_pool->buffer_list_head = buffer_addr;
    (sub_pool->free_chunk_cnt)++;
#endif /* DYNAMIC_LINKLIST_SUB_POOL */

#ifdef MEM_DEBUG_MODE
    if(mem_handle == g_local_mem_handle)
    {
        g_mem_stats.ltotalfree++;
        g_mem_stats.lfree++;
    }
    else if(mem_handle == g_shared_dscr_mem_handle)
    {
        g_mem_stats.sdtotalfree++;
        g_mem_stats.sdfree++;
    }
    else if(mem_handle == g_shared_pkt_mem_handle)
    {
        g_mem_stats.sptotalfree++;
        g_mem_stats.spfree++;
    }
#endif /* MEM_DEBUG_MODE */

    /* End of critical section. Enable all interrupts */
	critical_section_smart_end(smart_type);
    //critical_section_end1();

    return 0;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mem_add_users                                            */
/*                                                                           */
/*  Description   : This function adds the required number of users for the  */
/*                  specified buffer.                                        */
/*                                                                           */
/*  Inputs        : 1) Pointer to the memory pool handle                     */
/*                  2) Pointer to the buffer                                 */
/*                  3) Number of users to be added                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The user count is incremented after the given buffer is  */
/*                  validated. Note that each user needs to separately free  */
/*                  the buffer. Only after all the users have freed the      */
/*                  buffer it will be returned to the free pool.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Status of operation. 0 - Failure, 1 - Success    */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 mem_add_users(mem_handle_t *mem_handle, void *buffer_addr,
                  UWORD8 incr_count)
{
    UWORD8 native_id = 0;
    UWORD8 pattern   = 0;
    UWORD8 num_users = 0;
    int smart_type = 0;

    /* Exception Case 1: Buffer address is NULL */
    if (buffer_addr == NULL)
    {
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s NullAddExc\n\r",pt);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        g_mem_stats.nulladdexc++;
#endif /* MEM_DEBUG_MODE */

        return 0;
    }

    /* Exception Case 2: Buffer address is out of the pool boundary */
    if(is_buf_in_pool(mem_handle, buffer_addr) == BFALSE)
    {
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s OOBAddExc\n\r",pt);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        g_mem_stats.oobaddexc++;
#endif /* MEM_DEBUG_MODE */
        return 0;
    }

    /* Exception Case 4: Sub pool ID is invalid */
    native_id = get_native_pool_id(buffer_addr);

    if(native_id >= mem_handle->num_sub_pools)
    {
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s InvalidAddExc\n\r",pt);
        }
#endif /* DEBUG_MODE */
#ifdef MEM_DEBUG_MODE
        g_mem_stats.invalidaddexc++;
#endif /* MEM_DEBUG_MODE */

        return 0;
    }
    /* Start of critical section. Disable all interrupts */
	smart_type = critical_section_smart_start(1,1);
    //critical_section_start1();

/* Exception Case 3: Buffer is not yet allocated */
    pattern = get_pattern(buffer_addr);

    if((pattern != PATTERN_ALLOCATED_BUFFER) && (pattern != PATTERN_LOCKED_BUFFER))
    {
        /* End of critical section. Enable all interrupts */
	 critical_section_smart_end(smart_type);
        //critical_section_end1();

#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s FreeAddExc\n\r",pt);
        }
#endif /* DEBUG_MODE */

#ifdef MEM_DEBUG_MODE
        g_mem_stats.freeaddexc++;
#endif /* MEM_DEBUG_MODE */

        return 0;
    }

    /* Extract the present number of users */
    num_users = get_num_users(buffer_addr);

    /* Exception Case 5: If maximum number of allowed users is exceeded by   */
    /* adding the new users.                                                 */
    if((UWORD16)(num_users + incr_count) > MAX_NUM_USERS)
    {
        /* End of critical section. Enable all interrupts */
	 critical_section_smart_end(smart_type);
        //critical_section_end1();
#ifdef DEBUG_MODE
        {
            UWORD8 *pt = get_mem_pool_hdl_type(mem_handle);
            PRINTD2("SwEr:%s ExcessAddExc\n\r",pt);
        }
#endif /* DEBUG_MODE */
#ifdef MEM_DEBUG_MODE
        g_mem_stats.excessaddexc++;
#endif /* MEM_DEBUG_MODE */

        return 0;
    }

    /* Update the number of users */
    set_num_users(buffer_addr, num_users + incr_count);

    /* End of critical section. Enable all interrupts */
	critical_section_smart_end(smart_type);
    //critical_section_end1();

    return 1;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : scratch_mem_alloc_fn                                     */
/*                                                                           */
/*  Description   : This function allocates a buffer of required size from   */
/*                  the scratch memory.                                      */
/*                                                                           */
/*  Inputs        : 1) Size of the buffer required                           */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function allocates a buffer of required size from   */
/*                  the scratch memory. When a buffer is allocated, the      */
/*                  global index is checked to see if enough space is        */
/*                  available. If yes, the global index is decremented by    */
/*                  requested size and the location at that index in the     */
/*                  global scratch memory array is returned as the buffer.   */
/*                  The size is modified to be a multiple of 4 (if not) to   */
/*                  ensure word-aligned buffers.                             */
/*                                                                           */
/*                  NOTE: This function cannot be called from the interrupt  */
/*                  context. It can be called only for buffers that are used */
/*                  within a single event processing context.                */
/*                  Also, reuse of scratch memory within a function is not   */
/*                  allowed.                                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : void *, Pointer to the buffer of requested size          */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void *scratch_mem_alloc_fn(UWORD16 size)
{
    void *buffer = NULL;

#ifdef MEM_DEBUG_MODE
    g_mem_stats.scralloc++;
#endif /* MEM_DEBUG_MODE */

    size = WORD_ALIGN(size);

    /* Check if sufficient memory is available. Decrement the global index   */
    /* and return the appropriate location in the scratch memory array.      */
    if(g_scratch_mem_idx >= size)
    {
        g_scratch_mem_idx -= size;
        buffer = &g_scratch_mem_ptr[g_scratch_mem_idx];
    }

    if(NULL == buffer)
    {
#ifdef DEBUG_MODE
        PRINTD2("SwEr:scratch_mem_alloc fail\n");
#endif /* DEBUG_MODE */
#ifdef MEM_DEBUG_MODE
        g_mem_stats.scrallocfail++;
        update_scratch_mem_usage(BTRUE);
#endif /* MEM_DEBUG_MODE */
    }
#ifdef MEM_DEBUG_MODE
    else
        update_scratch_mem_usage(BFALSE);
#endif /* MEM_DEBUG_MODE */

    return buffer;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : validate_buffer                                          */
/*                                                                           */
/*  Description   : This function validates if a buff addr is in right range */
/*                                                                           */
/*  Inputs        : Pointer to the buffer                                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Checks for address randge in each pool to see if it falls*/
/*                  and return FALSE if doesnt fall in any address range     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : BOOL_T, Address validated Info, 0 - Failure, 1 - Success */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*  Revision History:                                                        */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes (Describe the changes made)  */
/*         31 08 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

BOOL_T validate_buffer(void *buffer)
{
    if(BTRUE == is_buf_in_pool(&g_shared_dscr_mem_pool_handle,buffer))
    return BTRUE;

    if(BTRUE == is_buf_in_pool(&g_shared_pkt_mem_pool_handle,buffer))
    return BTRUE;

    if(BTRUE == is_buf_in_pool(&g_local_mem_pool_handle,buffer))
    return BTRUE;

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef LOCALMEM_TX_DSCR
    if(BTRUE == is_buf_in_pool(&g_local_dscr_mem_pool_handle,buffer))
    return BTRUE;
#endif /* LOCALMEM_TX_DSCR */

    raise_system_error(MEM_CORRUPTED);

//#ifdef OS_LINUX_CSL_TYPE
//    dump_stack();
//#endif /* OS_LINUX_CSL_TYPE */

    return BFALSE;
}
