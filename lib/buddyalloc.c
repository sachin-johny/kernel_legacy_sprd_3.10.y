/*
 * Buddy allocator for managing special purpose memory
 * not managed by the regular kmalloc/kfree interface.
 * Uses for this includes on-device special memory, uncached memory
 * etc.
 *
 * Copyright 2005 (C) Jes Sorensen <jes@trained-monkey.org>
 * Copyright 2014 (C) Horace Hsieh <horace.xie@spreadtrum.com>
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitmap.h>
#include <linux/buddyalloc.h>
#ifdef CONFIG_ION_BUDDY_CHECKPAGE
#include <asm/mach/map.h>
#endif

#define BUDDY_IS_FREE(chunk, idx)	!(chunk->bits[idx].allocated)
#define BUDDY_ORDER(chunk, idx)		chunk->bits[idx].order
#define BUDDY_INDEX(chunk, idx)		(idx ^ (1<<BUDDY_ORDER(chunk, idx)))
#define BUDDY_NEXT_INDEX(chunk, idx)	(idx + (1<<BUDDY_ORDER(chunk, idx)))

/**
 * buddy_pool_create - create a new special memory pool
 * @min_alloc_order: log base 2 of number of bytes each bitmap bit represents
 * @nid: node id of the node the pool structure should be allocated on, or -1
 *
 * Create a new special memory pool that can be used to manage special purpose
 * memory not managed by the regular kmalloc/kfree interface.
 */
struct buddy_pool *buddy_pool_create(int min_alloc_order, int nid)
{
	struct buddy_pool *pool;

	pool = kmalloc_node(sizeof(struct buddy_pool), GFP_KERNEL, nid);
	if (pool != NULL) {
		rwlock_init(&pool->lock);
		INIT_LIST_HEAD(&pool->chunks);
		pool->min_alloc_order = min_alloc_order;
	}
	return pool;
}
EXPORT_SYMBOL(buddy_pool_create);

/**
 * buddy_pool_add_virt - add a new chunk of special memory to the pool
 * @pool: pool to add new memory chunk to
 * @virt: virtual starting address of memory chunk to add to pool
 * @phys: physical starting address of memory chunk to add to pool
 * @size: size in bytes of the memory chunk to add to pool
 * @nid: node id of the node the chunk structure and bitmap should be
 *       allocated on, or -1
 *
 * Add a new chunk of special memory to the specified pool.
 *
 * Returns 0 on success or a -ve errno on failure.
 */
int buddy_pool_add_virt(struct buddy_pool *pool, unsigned long virt, phys_addr_t phys,
		 size_t size, int nid)
{
	struct buddy_pool_chunk *chunk;
	int nbits = size >> pool->min_alloc_order;
	int nbytes = sizeof(struct buddy_pool_chunk) +
				(nbits * sizeof(struct buddy_bit));
	int i, idx = 0;

	chunk = kzalloc_node(nbytes, GFP_KERNEL | __GFP_ZERO, nid);
	if (unlikely(chunk == NULL))
		return -ENOMEM;

	for (i = sizeof(nbits) * 8 - 1; i > 0; i--)
		if (nbits & (1 << i)) {
			BUDDY_ORDER(chunk, idx) = i;
			idx = BUDDY_NEXT_INDEX(chunk, idx);
		}

	spin_lock_init(&chunk->lock);
	chunk->phys_addr = phys;
	chunk->start_addr = virt;
	chunk->end_addr = virt + size;
	chunk->nbits = nbits;

	write_lock(&pool->lock);
	list_add(&chunk->next_chunk, &pool->chunks);
	write_unlock(&pool->lock);

	return 0;
}
EXPORT_SYMBOL(buddy_pool_add_virt);

/**
 * buddy_pool_destroy - destroy a special memory pool
 * @pool: pool to destroy
 *
 * Destroy the specified special memory pool. Verifies that there are no
 * outstanding allocations.
 */
void buddy_pool_destroy(struct buddy_pool *pool)
{
	struct list_head *_chunk, *_next_chunk;
	struct buddy_pool_chunk *chunk;
	int start_bit;

	list_for_each_safe(_chunk, _next_chunk, &pool->chunks) {
		chunk = list_entry(_chunk, struct buddy_pool_chunk, next_chunk);
		list_del(&chunk->next_chunk);

		for (start_bit = 0; start_bit < chunk->nbits; start_bit++)
			BUG_ON(chunk->bits[start_bit].allocated);

		kfree(chunk);
	}
	kfree(pool);
	return;
}
EXPORT_SYMBOL(buddy_pool_destroy);

#define BUDDY_POOL_MIN_ALLOC PAGE_SIZE
static unsigned long buddy_pool_order(unsigned long len)
{
        int i;

        len = (len + BUDDY_POOL_MIN_ALLOC - 1)/BUDDY_POOL_MIN_ALLOC;
        len--;
        for (i = 0; i < sizeof(len)*8; i++)
                if (len >> i == 0)
                        break;
        return i;
}

/*
 * Allocate the requested number of bytes from the specified chunk.
 * Uses the simple buddy algorithm
 */
void show_carveout_buddy_info(int rst);
static int buddy_chunk_alloc(struct buddy_pool_chunk *chunk, size_t nbits)
{
	int best_fit = -1;
	int start_bit = 0;
	int end_bit = chunk->nbits;
	int size = nbits;
	unsigned long flags;
	
	nbits = buddy_pool_order(nbits);

	if (nbits > BUDDY_MAX_ORDER)
                 return 0;

	spin_lock_irqsave(&chunk->lock, flags);
	while (start_bit < end_bit) {
		if (BUDDY_IS_FREE(chunk, start_bit)) {
			if (BUDDY_ORDER(chunk, start_bit) ==
					(unsigned char)nbits) {
				/* set the not free bit and clear others */
				best_fit = start_bit;
				break;
			}
			if (BUDDY_ORDER(chunk, start_bit) >
					(unsigned char)nbits &&
					(best_fit < 0 ||
					 BUDDY_ORDER(chunk, start_bit) <
					 BUDDY_ORDER(chunk, best_fit)))
				best_fit = start_bit;
		}
		start_bit = BUDDY_NEXT_INDEX(chunk, start_bit);
	}
	/* if best_fit < 0, there are no suitable slots,
	 * return an error
	 */
	if (best_fit < 0) {
		if (nbits > 0) {
			printk("%s ---- cant't get buddy memory: %d  rawsize: %d\n", __func__, 1UL << nbits, size);
			show_carveout_buddy_info(0);
		}
		spin_unlock_irqrestore(&chunk->lock, flags);
		return -1;
	}

	/* now partition the best fit:
	 *   split the slot into 2 buddies of order - 1
	 *   repeat until the slot is of the correct order
	 */
	while (BUDDY_ORDER(chunk, best_fit) > (unsigned char)nbits) {
		int buddy;
		BUDDY_ORDER(chunk, best_fit) -= 1;
		buddy = BUDDY_INDEX(chunk, best_fit);
		BUDDY_ORDER(chunk, buddy) = BUDDY_ORDER(chunk, best_fit);
	}
	chunk->bits[best_fit].allocated = 1;
	chunk->bits[best_fit].size = size;
	spin_unlock_irqrestore(&chunk->lock, flags);
	return best_fit;
}

/**
 * buddy_pool_alloc - allocate special memory from the pool
 * @pool: pool to allocate from
 * @size: number of bytes to allocate from the pool
 *
 */
unsigned long buddy_pool_alloc(struct buddy_pool *pool, size_t size)
{
	struct list_head *_chunk;
	struct buddy_pool_chunk *chunk;
	unsigned long addr;
	int order = pool->min_alloc_order, nbits, best_fit;

	if (size == 0)
		return 0;

	read_lock(&pool->lock);
	list_for_each(_chunk, &pool->chunks) {
		chunk = list_entry(_chunk, struct buddy_pool_chunk, next_chunk);
		best_fit = buddy_chunk_alloc(chunk, size); 
		if (best_fit < 0)
			continue;
		addr = chunk->start_addr + ((unsigned long)best_fit << order);
		read_unlock(&pool->lock);
		return addr;
	}
	read_unlock(&pool->lock);
	return 0;
}
EXPORT_SYMBOL(buddy_pool_alloc);

/*
 * Free previously allocated special memory back to the specified chunk.
 */
static void buddy_chunk_free(struct buddy_pool_chunk *chunk, unsigned long idx)
{
	unsigned long flags;
	unsigned long buddy, curr = idx;

	spin_lock_irqsave(&chunk->lock, flags);
	/* clean up the bitmap, merging any buddies */
	chunk->bits[curr].allocated = 0;
	/* find a slots buddy Buddy# = Slot# ^ (1 << order)
	 * if the buddy is also free merge them
	 * repeat until the buddy is not free or end of the bitmap is reached
	 */
	do {
		buddy = BUDDY_INDEX(chunk, curr);
		if (BUDDY_IS_FREE(chunk, buddy) &&
			BUDDY_ORDER(chunk, buddy) == BUDDY_ORDER(chunk, curr)) {
			BUDDY_ORDER(chunk, buddy)++;
			BUDDY_ORDER(chunk, curr)++;
			curr = min(buddy, curr);
		} else {
			break;
		}
	} while (curr < chunk->nbits);
	spin_unlock_irqrestore(&chunk->lock, flags);
}

/**
 * buddy_pool_free - free allocated special memory back to the pool
 * @pool: pool to free to
 * @addr: starting address of memory to free back to pool
 * @size: size in bytes of memory to free
 *
 * Free previously allocated special memory back to the specified pool.
 */
void buddy_pool_free(struct buddy_pool *pool, unsigned long addr)
{
	struct list_head *_chunk;
	struct buddy_pool_chunk *chunk;
	int order = pool->min_alloc_order;

	read_lock(&pool->lock);
	list_for_each(_chunk, &pool->chunks) {
		chunk = list_entry(_chunk, struct buddy_pool_chunk, next_chunk);

		if (addr >= chunk->start_addr && addr < chunk->end_addr) {
			int bit = (addr - chunk->start_addr) >> order;
			buddy_chunk_free(chunk, bit);
			break;
		}
	}
	read_unlock(&pool->lock);
}
EXPORT_SYMBOL(buddy_pool_free);

void show_buddy_info(int rst, struct buddy_pool_chunk *chunk)
{
	int i;
	int start_bit = 0;
	int end_bit = chunk->nbits;
	unsigned long flags;
	void *kvaddr;
	unsigned long phyaddr;

	printk("chunk->nbits  %d\n", end_bit);

	spin_lock_irqsave(&chunk->lock, flags);
	while (start_bit < end_bit) {
		if (BUDDY_ORDER(chunk, start_bit)) {
			if (BUDDY_IS_FREE(chunk, start_bit))
				printk("free  start_bit  %d,  size: %d\n", start_bit, 1UL << BUDDY_ORDER(chunk, start_bit));
			else {
				printk("alloc start_bit  %d,  size: %d,  raw_size: %d\n", start_bit, 1UL << BUDDY_ORDER(chunk, start_bit),
											 chunk->bits[start_bit].size);
#ifdef CONFIG_ION_BUDDY_CHECKPAGE
				phyaddr = chunk->start_addr + ((unsigned long )start_bit << 12) + chunk->bits[start_bit].size;
				kvaddr = NULL;
				kvaddr = __arch_ioremap(phyaddr - 4096, 4096, MT_MEMORY_NONCACHED);
				if (kvaddr) {
					for (i = 0; i < 1024; i++) {
						if (0xffffffff != *((unsigned long *)kvaddr + i)) {
							printk("  ---- error phyaddr: 0x%p   value:0x%p\n",
								  phyaddr - 4096 + i, *((unsigned long *)kvaddr + i));
							if (rst)
								panic("buddy protect page is modified\n");
							break;
						}
					}
					__arch_iounmap(kvaddr);
				}
#endif
			}
		}
		start_bit = BUDDY_NEXT_INDEX(chunk, start_bit);
	}
	spin_unlock_irqrestore(&chunk->lock, flags);
}


void show_pool_info(int rst, struct buddy_pool *pool)
{
        struct list_head *_chunk, *_next_chunk;
        struct buddy_pool_chunk *chunk;
        list_for_each_safe(_chunk, _next_chunk, &pool->chunks) {
                chunk = list_entry(_chunk, struct buddy_pool_chunk, next_chunk);
                show_buddy_info(rst, chunk);
        }
}
