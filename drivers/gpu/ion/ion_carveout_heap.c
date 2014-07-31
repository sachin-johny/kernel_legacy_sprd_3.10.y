/*
 * drivers/gpu/ion/ion_carveout_heap.c
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
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "ion_priv.h"

#include <asm/mach/map.h>

struct ion_carveout_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	ion_phys_addr_t base;
};

ion_phys_addr_t ion_carveout_allocate(struct ion_heap *heap,
				      unsigned long size,
				      unsigned long align, unsigned long flags)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);
	unsigned long offset = gen_pool_alloc(carveout_heap->pool, size);

	//printk("ion: malloc: size=%08x, pool=%08x, offset=%08x \n", size, carveout_heap->pool, offset);

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

void ion_carveout_free(struct ion_heap *heap, ion_phys_addr_t addr,
		       unsigned long size, unsigned long flags)
{
	struct ion_carveout_heap *carveout_heap =
		container_of(heap, struct ion_carveout_heap, heap);

	if (addr == ION_CARVEOUT_ALLOCATE_FAIL)
		return;

	//printk("ion: free  : size=%08x, pool=%08x, offset=%08x \n", size, carveout_heap->pool, addr);
	
	gen_pool_free(carveout_heap->pool, addr, size);
#ifdef CONFIG_ION_PAGECACHE
	size >>= PAGE_SHIFT;
	heap->allocated -= size;
	if (flags & ION_ALLOC_PAGECACHE_MASK)
		heap->cachedpages -= size;
#endif
}

int ion_carveout_heap_phys(struct ion_heap *heap,
				  struct ion_buffer *buffer,
				  ion_phys_addr_t *addr, size_t *len)
{
	*addr = buffer->priv_phys;
	*len = buffer->size;
	return 0;
}

static int ion_carveout_heap_allocate(struct ion_heap *heap,
				      struct ion_buffer *buffer,
				      unsigned long size, unsigned long align,
				      unsigned long flags)
{
	buffer->priv_phys = ion_carveout_allocate(heap, size, align, flags);
	pr_debug("ion buffer flags 0x%lx\n", flags);
	buffer->flags = flags;
	return buffer->priv_phys == ION_CARVEOUT_ALLOCATE_FAIL ? -ENOMEM : 0;
}

static void ion_carveout_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;

	ion_carveout_free(heap, buffer->priv_phys, buffer->size, buffer->flags);
	buffer->priv_phys = ION_CARVEOUT_ALLOCATE_FAIL;
}

struct scatterlist *ion_carveout_heap_map_dma(struct ion_heap *heap,
					      struct ion_buffer *buffer)
{
	return ERR_PTR(-EINVAL);
}

void ion_carveout_heap_unmap_dma(struct ion_heap *heap,
				 struct ion_buffer *buffer)
{
	return;
}

void *ion_carveout_heap_map_kernel(struct ion_heap *heap,
				   struct ion_buffer *buffer)
{
	return __arch_ioremap(buffer->priv_phys, buffer->size,
			      MT_MEMORY_NONCACHED);
}

void ion_carveout_heap_unmap_kernel(struct ion_heap *heap,
				    struct ion_buffer *buffer)
{
	__arch_iounmap(buffer->vaddr);
	buffer->vaddr = NULL;
	return;
}

int ion_carveout_heap_map_user(struct ion_heap *heap, struct ion_buffer *buffer,
			       struct vm_area_struct *vma)
{
	if((buffer->flags & (1<<31)) )
	{	
		//printk(KERN_INFO "pgprot_cached buffer->flags 0x%x\n",buffer->flags);
		return remap_pfn_range(vma, vma->vm_start,
			       __phys_to_pfn(buffer->priv_phys) + vma->vm_pgoff,
			       buffer->size,
			       (vma->vm_page_prot));

	}
	else
	{
		//printk(KERN_INFO "pgprot_noncached buffer->flags 0x%x\n",buffer->flags);
		return remap_pfn_range(vma, vma->vm_start,
			       __phys_to_pfn(buffer->priv_phys) + vma->vm_pgoff,
			       buffer->size,
			       pgprot_noncached(vma->vm_page_prot));

	}
}

static struct ion_heap_ops carveout_heap_ops = {
	.allocate = ion_carveout_heap_allocate,
	.free = ion_carveout_heap_free,
	.phys = ion_carveout_heap_phys,
	.map_user = ion_carveout_heap_map_user,
	.map_kernel = ion_carveout_heap_map_kernel,
	.unmap_kernel = ion_carveout_heap_unmap_kernel,
};

struct ion_heap *ion_carveout_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_carveout_heap *carveout_heap;
#ifdef CONFIG_ION_PAGECACHE
	struct page *page;
#endif
	carveout_heap = kzalloc(sizeof(struct ion_carveout_heap), GFP_KERNEL);
	if (!carveout_heap)
		return ERR_PTR(-ENOMEM);

	carveout_heap->pool = gen_pool_create(12, -1);
	if (!carveout_heap->pool) {
		kfree(carveout_heap);
		return ERR_PTR(-ENOMEM);
	}
	carveout_heap->base = heap_data->base;
	gen_pool_add(carveout_heap->pool, carveout_heap->base, heap_data->size,
		     -1);
	carveout_heap->heap.ops = &carveout_heap_ops;
	carveout_heap->heap.type = ION_HEAP_TYPE_CARVEOUT;
#ifdef CONFIG_ION_PAGECACHE
	carveout_heap->heap.size = heap_data->size >> PAGE_SHIFT;
	page = phys_to_page(carveout_heap->base);
	carveout_heap->heap.rangeshrunk = page_to_pfn(page);
#endif

	return &carveout_heap->heap;
}

void ion_carveout_heap_destroy(struct ion_heap *heap)
{
	struct ion_carveout_heap *carveout_heap =
	     container_of(heap, struct  ion_carveout_heap, heap);

	gen_pool_destroy(carveout_heap->pool);
	kfree(carveout_heap);
	carveout_heap = NULL;
}

unsigned long ion_carveout_heap_start_pfn(struct ion_heap *heap)
{
	struct page *page;
	struct ion_carveout_heap *carveout_heap =
	     container_of(heap, struct  ion_carveout_heap, heap);
	page = phys_to_page(carveout_heap->base);
	return page_to_pfn(page);
}
