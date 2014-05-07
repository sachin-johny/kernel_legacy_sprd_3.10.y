/*
 * Buddy allocator for managing special purpose memory
 * not managed by the regular kmalloc/kfree interface.
 * Uses for this includes on-device special memory, uncached memory
 * etc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 */


#ifndef __BUDDYALLOC_H__
#define __BUDDYALLOC_H__
/*
 *  Buddy memory pool descriptor.
 */
struct buddy_pool {
	rwlock_t lock;
	struct list_head chunks;	/* list of chunks in this pool */
	int min_alloc_order;		/* minimum allocation order */
};

/*
 *  Buddy bitmap.
 */
struct buddy_bit {
	unsigned allocated:1;
	unsigned order:7;
	size_t   size;
};

//#define BUDDY_MAX_ORDER		(1<<128)	
#define BUDDY_MAX_ORDER		128	

/*
 *  Buddy memory pool chunk descriptor.
 */
struct buddy_pool_chunk {
	spinlock_t lock;
	struct list_head next_chunk;	/* next chunk in pool */
	phys_addr_t phys_addr;		/* physical starting address of memory chunk */
	unsigned long start_addr;	/* starting address of memory chunk */
	unsigned long end_addr;		/* ending address of memory chunk */
	unsigned long nbits;		/* number of bits in bitmap */
	struct buddy_bit bits[0];	/* bitmap for allocating memory chunk */
};

extern struct buddy_pool *buddy_pool_create(int, int);
extern int buddy_pool_add_virt(struct buddy_pool *, unsigned long, phys_addr_t,
			     size_t, int);
/**
 * buddy_pool_add - add a new chunk of special memory to the pool
 * @pool: pool to add new memory chunk to
 * @addr: starting address of memory chunk to add to pool
 * @size: size in bytes of the memory chunk to add to pool
 * @nid: node id of the node the chunk structure and bitmap should be
 *       allocated on, or -1
 *
 * Add a new chunk of special memory to the specified pool.
 *
 * Returns 0 on success or a -ve errno on failure.
 */
static inline int buddy_pool_add(struct buddy_pool *pool, unsigned long addr,
			       size_t size, int nid)
{
	return buddy_pool_add_virt(pool, addr, -1, size, nid);
}
extern void buddy_pool_destroy(struct buddy_pool *);
extern unsigned long buddy_pool_alloc(struct buddy_pool *, size_t);
extern void buddy_pool_free(struct buddy_pool *, unsigned long);
#endif /* __BUDDYALLOC_H__ */
