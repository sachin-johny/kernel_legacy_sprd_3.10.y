/*
 * drivers/gpu/ion/ion_carveout_buddy_heap.c
 *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/spinlock.h>

#include <linux/err.h>
#include <linux/buddyalloc.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "ion_priv.h"

#include <asm/mach/map.h>

struct ion_carveout_buddy_heap {
	struct ion_heap heap;
	struct buddy_pool *pool;
	ion_phys_addr_t base;
};

struct buddy_pool *g_carveout_pool = NULL;

void proc_show_pool_info(struct seq_file *s, struct buddy_pool *pool);
void show_pool_info(int rst, struct buddy_pool *pool);

void show_carveout_buddy_info(int rst)
{
	show_pool_info(rst, g_carveout_pool);
}

ion_phys_addr_t ion_carveout_buddy_allocate(struct ion_heap *heap,
					    unsigned long size,
					    unsigned long align,
					    unsigned long flags)
{
	struct ion_carveout_buddy_heap *carveout_buddy_heap =
		container_of(heap, struct ion_carveout_buddy_heap, heap);
	unsigned long offset = buddy_pool_alloc(carveout_buddy_heap->pool, size);

	pr_debug("ion: malloc: size=0x%lx, pool=0x%08x, offset=0x%lx \n",
		 size, (unsigned int)carveout_buddy_heap->pool, offset);

	if (!offset)
		return ION_CARVEOUT_ALLOCATE_FAIL;

#ifdef CONFIG_ION_PAGECACHE
	size >>= PAGE_SHIFT;
	heap->allocated += size;
	if (flags & ION_ALLOC_PAGECACHE_MASK)
		heap->cachedpages += size;
#endif
	return offset;
}

void ion_carveout_buddy_free(struct ion_heap *heap, ion_phys_addr_t addr,
			     unsigned long size, unsigned long flags)
{
	struct ion_carveout_buddy_heap *carveout_buddy_heap =
		container_of(heap, struct ion_carveout_buddy_heap, heap);

	if (addr == ION_CARVEOUT_ALLOCATE_FAIL)
		return;

	pr_debug("ion: free: size=0x%lx, pool=0x%08x, offset=0x%lx \n",
		 size, (unsigned int)carveout_buddy_heap->pool, addr);
	
	buddy_pool_free(carveout_buddy_heap->pool, addr);
#ifdef CONFIG_ION_PAGECACHE
	size >>= PAGE_SHIFT;
	heap->allocated -= size;
	if (flags & ION_ALLOC_PAGECACHE_MASK)
		heap->cachedpages -= size;
#ifdef CONFIG_ION_BUDDY_CHECKPAGE
	else
		heap->allocated -= 1;
#endif
#endif
}

static int ion_carveout_buddy_heap_allocate(struct ion_heap *heap,
				      struct ion_buffer *buffer,
				      unsigned long size, unsigned long align,
				      unsigned long flags)
{
#ifdef CONFIG_ION_BUDDY_CHECKPAGE
	void *kvaddr = NULL;
	if (flags & ION_ALLOC_PAGECACHE_MASK)
		buffer->priv_phys = ion_carveout_buddy_allocate(heap, size, align, flags);
	else {
		buffer->priv_phys = ion_carveout_buddy_allocate(heap, size + 4096, align, flags);
		if (buffer->priv_phys != ION_CARVEOUT_ALLOCATE_FAIL) {
			kvaddr = __arch_ioremap(buffer->priv_phys, size + 4096, MT_MEMORY_NONCACHED);
			if (!kvaddr)
				printk("ion_ioremap error\n");
			else {
				memset((char *)kvaddr + size, 0xff, 4096);
				__arch_iounmap(kvaddr);
			}
		}
	}
#else
	buffer->priv_phys = ion_carveout_buddy_allocate(heap, size, align, flags);
#endif
	pr_debug("ion buffer flags 0x%lx\n", flags);
	buffer->flags = flags;
	return buffer->priv_phys == ION_CARVEOUT_ALLOCATE_FAIL ? -ENOMEM : 0;
}

static void ion_carveout_buddy_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;

	ion_carveout_buddy_free(heap, buffer->priv_phys, buffer->size,
				buffer->flags);
	buffer->priv_phys = ION_CARVEOUT_ALLOCATE_FAIL;
}

int ion_carveout_heap_phys(struct ion_heap *heap, struct ion_buffer *buffer,
			   ion_phys_addr_t *addr, size_t *len);
void *ion_carveout_heap_map_kernel(struct ion_heap *heap,
				   struct ion_buffer *buffer);
void ion_carveout_heap_unmap_kernel(struct ion_heap *heap,
				    struct ion_buffer *buffer);
int ion_carveout_heap_map_user(struct ion_heap *heap, struct ion_buffer *buffer,
			       struct vm_area_struct *vma);

static struct ion_heap_ops carveout_buddy_heap_ops = {
	.allocate = ion_carveout_buddy_heap_allocate,
	.free = ion_carveout_buddy_heap_free,
	.phys = ion_carveout_heap_phys,
	.map_user = ion_carveout_heap_map_user,
	.map_kernel = ion_carveout_heap_map_kernel,
	.unmap_kernel = ion_carveout_heap_unmap_kernel,
};

int g_buddy_pool_size = 0;
struct ion_heap *ion_carveout_buddy_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_carveout_buddy_heap *carveout_buddy_heap;

	carveout_buddy_heap = kzalloc(sizeof(struct ion_carveout_buddy_heap),
				      GFP_KERNEL);
	if (!carveout_buddy_heap)
		return ERR_PTR(-ENOMEM);

	carveout_buddy_heap->pool = buddy_pool_create(12, -1);
	g_carveout_pool = carveout_buddy_heap->pool;
	if (!carveout_buddy_heap->pool) {
		kfree(carveout_buddy_heap);
		return ERR_PTR(-ENOMEM);
	}
	carveout_buddy_heap->base = heap_data->base;
	buddy_pool_add(carveout_buddy_heap->pool, carveout_buddy_heap->base,
			heap_data->size, -1);

	g_buddy_pool_size = heap_data->size;

	carveout_buddy_heap->heap.ops = &carveout_buddy_heap_ops;
	carveout_buddy_heap->heap.type = ION_HEAP_TYPE_CARVEOUT;
#ifdef CONFIG_ION_PAGECACHE
	carveout_buddy_heap->heap.size = heap_data->size >> PAGE_SHIFT;
#endif

	return &carveout_buddy_heap->heap;
}

void ion_carveout_buddy_heap_destroy(struct ion_heap *heap)
{
	struct ion_carveout_buddy_heap *carveout_buddy_heap =
	     container_of(heap, struct  ion_carveout_buddy_heap, heap);

	buddy_pool_destroy(carveout_buddy_heap->pool);
	kfree(carveout_buddy_heap);
	carveout_buddy_heap = NULL;
}
