/*
 * drivers/gpu/ion/ion.c
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

#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/ion.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/rbtree.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/memcontrol.h>
#include <linux/pagemap.h>
#include <linux/rmap.h>

#include "ion_priv.h"
#define DEBUG

/**
 * struct ion_device - the metadata of the ion device node
 * @dev:		the actual misc device
 * @buffers:	an rb tree of all the existing buffers
 * @lock:		lock protecting the buffers & heaps trees
 * @heaps:		list of all the heaps in the system
 * @user_clients:	list of all the clients created from userspace
 */
struct ion_device {
	struct miscdevice dev;
	struct rb_root buffers;
	struct mutex lock;
	struct rb_root heaps;
	long (*custom_ioctl) (struct ion_client *client, unsigned int cmd,
			      unsigned long arg);
	struct rb_root user_clients;
	struct rb_root kernel_clients;
	struct dentry *debug_root;
};

/**
 * struct ion_client - a process/hw block local address space
 * @ref:		for reference counting the client
 * @node:		node in the tree of all clients
 * @dev:		backpointer to ion device
 * @handles:		an rb tree of all the handles in this client
 * @lock:		lock protecting the tree of handles
 * @heap_mask:		mask of all supported heaps
 * @name:		used for debugging
 * @task:		used for debugging
 *
 * A client represents a list of buffers this client may access.
 * The mutex stored here is used to protect both handles tree
 * as well as the handles themselves, and should be held while modifying either.
 */
struct ion_client {
	struct kref ref;
	struct rb_node node;
	struct ion_device *dev;
	struct rb_root handles;
	struct mutex lock;
	unsigned int heap_mask;
	const char *name;
	struct task_struct *task;
	pid_t pid;
	struct dentry *debug_root;
};

/**
 * ion_handle - a client local reference to a buffer
 * @ref:		reference count
 * @client:		back pointer to the client the buffer resides in
 * @buffer:		pointer to the buffer
 * @node:		node in the client's handle rbtree
 * @kmap_cnt:		count of times this client has mapped to kernel
 * @dmap_cnt:		count of times this client has mapped for dma
 * @usermap_cnt:	count of times this client has mapped for userspace
 *
 * Modifications to node, map_cnt or mapping should be protected by the
 * lock in the client.  Other fields are never changed after initialization.
 */
struct ion_handle {
	struct kref ref;
	int magicnum;
	struct ion_client *client;
	struct ion_buffer *buffer;
	struct rb_node node;
	unsigned int kmap_cnt;
	unsigned int dmap_cnt;
	unsigned int usermap_cnt;
#ifdef CONFIG_ION_PAGECACHE
	struct list_head list;
	struct rb_node pagecache;
	struct task_struct *task;
#endif
};

#ifdef CONFIG_ION_PAGECACHE
extern void __clear_page_mlock(struct page *page);
extern int prep_new_page(struct page *page, int order, gfp_t gfp_flags);

struct ion_client *ion_client_pagecache;

int ion_pagecache_rb_insert(struct rb_root *root, struct ion_handle *handle)
{
	struct rb_node **new = &(root->rb_node), *parent = NULL;
	struct ion_buffer *chunk = handle->buffer;

	/* Figure out where to put new node */
	while (*new) {
		struct ion_handle *this = container_of(*new, struct ion_handle, pagecache);
		struct ion_buffer *buffer = this->buffer;

		parent = *new;
		if (chunk->priv_phys < buffer->priv_phys)
			new = &((*new)->rb_left);
		else if (chunk->priv_phys > buffer->priv_phys)
			new = &((*new)->rb_right);
		else
			return -EEXIST;
	}

	/* Add new node and rebalance tree. */
	rb_link_node(&handle->pagecache, parent, new);
	rb_insert_color(&handle->pagecache, root);
	return 0;
}

struct ion_handle *ion_pagecache_rb_search(struct rb_root *root,
					  ion_phys_addr_t addr)
{
	struct rb_node *node = root->rb_node;

	while (node) {
		struct ion_handle *handle = container_of(node, struct ion_handle, pagecache);
		struct ion_buffer *chunk = handle->buffer;

		if (addr < chunk->priv_phys)
			node = node->rb_left;
		else if (addr > chunk->priv_phys)
			node = node->rb_right;
		else
			return handle;
	}
	return NULL;
}

#ifdef CONFIG_ION_PAGECACHE
static atomic_t ion_pagecache_flag = ATOMIC_INIT(1);
static atomic_t ion_open_count = ATOMIC_INIT(0);
unsigned long long total_allocated_ion_cache = 0;
int ion_normal_allocated_page_count(void);
static int ion_normal_allocing = 0;
#endif
struct page *ion_pagecache_alloc(gfp_t gfp)
{
	struct ion_handle *handle;
	ion_phys_addr_t addr;
	size_t size;
	struct ion_heap *heap;
	struct ion_buffer *buffer;
	struct page *page;

#ifdef CONFIG_ION_PAGECACHE
	if (ion_normal_allocated_page_count() > 0)
		return NULL;

	if (ion_normal_allocing)
		return NULL;

	if (!atomic_read(&ion_pagecache_flag))
		return NULL;
#endif

	if (!ion_client_pagecache)
		return NULL;

	handle = ion_alloc(ion_client_pagecache,
			   PAGE_SIZE, PAGE_SIZE, (unsigned int)-1);
	if (IS_ERR_OR_NULL(handle))
		return NULL;
	buffer = handle->buffer;
	heap = buffer->heap;
	if (!heap->ops->phys ||
	    heap->ops->phys(heap, buffer, &addr, &size)) {
		ion_free(ion_client_pagecache, handle);
		return NULL;
	}

	page = phys_to_page(addr);
	ClearPageReserved(page);
	ClearPageIONBacked(page);
	/* Clean the pagecount set in stage of bootmem */
	atomic_set(&page->_count, 0);
	if (prep_new_page(page, 0, gfp)) {
		dump_page(page);
		BUG();
	}
	SetPageIONBacked(page);

	spin_lock(&heap->pagecache_lock);
	ion_pagecache_rb_insert(&heap->pagecaches, handle);
	spin_unlock(&heap->pagecache_lock);
#ifdef CONFIG_ION_PAGECACHE
	total_allocated_ion_cache++;
#endif
	return page;
}

int ion_pagecache_release(struct page *page)
{
	unsigned long addr = page_to_phys(page);
	struct ion_handle *handle;
	struct rb_node *n;
	struct ion_device *ion_dev;

	if (!ion_client_pagecache)
		return -1;
	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps); n != NULL; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);

		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;
		if (heap->cachedpages <= 0)
			continue;

		spin_lock(&heap->pagecache_lock);
		handle = ion_pagecache_rb_search(&heap->pagecaches, addr);
		if (!handle) {
			spin_unlock(&heap->pagecache_lock);
			continue;
		}
		list_del_init(&handle->list);
		rb_erase(&handle->pagecache, &heap->pagecaches);
		spin_unlock(&heap->pagecache_lock);

		atomic_inc(&page->_count);
		SetPageIONBacked(page);
		SetPageReserved(page);
		ion_free(ion_client_pagecache, handle);
		return 0;
	}
	return -1;
}

int ion_pagecache_complete(struct page *page)
{
	unsigned long addr = page_to_phys(page);
	struct ion_handle *handle;
	struct rb_node *n;
	struct ion_device *ion_dev;

	/* The page has been released. */
	if (PageReserved(page) || !page_count(page))
		return -1;
	if (!ion_client_pagecache)
		return -1;
	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps), handle = NULL;
	     n != NULL && handle == NULL;
	     n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);

		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;
		if (heap->cachedpages <= 0)
			continue;

		spin_lock(&heap->pagecache_lock);
		handle = ion_pagecache_rb_search(&heap->pagecaches, addr);
		if (handle)
			list_add_tail(&handle->list, &heap->pagecache_lru);
		spin_unlock(&heap->pagecache_lock);
	}
	return handle ? 0 : -1;
}

static int __ion_pagecache_range_shrink(struct ion_heap *heap, unsigned long start_pfn,
				  unsigned long end_pfn, int tryhard, gfp_t gfp_mask)
{
	int shrunk = 0;
	struct page *page;
	struct ion_handle *handle, *next;
	int loop_count = 0;
	int repeat_cnt = 0;
	int fail_cnt = 0;
	LIST_HEAD(failed_pages);

	atomic_inc(&heap->shrinking);
	printk("==================  shrink start\n");
	mem_cgroup_uncharge_start();

	page = NULL;
	spin_lock(&heap->pagecache_lock);

	list_for_each_entry_safe(handle, next, &heap->pagecache_lru, list) {
		struct address_space *mapping;
		int locked;
		ion_phys_addr_t addr;
		size_t size;
		unsigned long pfn;

		loop_count++;
		spin_unlock(&heap->pagecache_lock);

		if (heap->ops->phys(heap, handle->buffer, &addr, &size)) {
			page = NULL;
			spin_lock(&heap->pagecache_lock);
			continue;
		}

		page = phys_to_page(addr);
		pfn = page_to_pfn(page);

		if ((pfn < start_pfn) || (pfn > end_pfn)) {
			page = NULL;
			spin_lock(&heap->pagecache_lock);
			continue;
		}

		locked = trylock_page(page);

		if (!(tryhard || locked)) {
			page = NULL;
			spin_lock(&heap->pagecache_lock);
			continue;
		}

		spin_lock(&heap->pagecache_lock);
		list_del_init(&handle->list);
		spin_unlock(&heap->pagecache_lock);

		if (!locked) {
			might_sleep();
			__lock_page(page);
		}

		if (unlikely(TestClearPageMlocked(page)))
			__clear_page_mlock(page);
		mapping = page_mapping(page);
		if (page_mapped(page) && mapping) {
			if (try_to_unmap(page, TTU_UNMAP|TTU_IGNORE_ACCESS)
					!= SWAP_SUCCESS)
				goto failed;
		}
		if (page_has_private(page) &&
				!try_to_release_page(page, gfp_mask)) {
			goto failed;
		} else {
			if (mapping) {
				spin_lock_irq(&mapping->tree_lock);
				__delete_from_page_cache(page);
				spin_unlock_irq(&mapping->tree_lock);
				mem_cgroup_uncharge_cache_page(page);
			}
		}
		unlock_page(page);

		if (likely(page_count(page) > 0)) {
			page_cache_release(page);	/* pagecache ref */
			shrunk++;
		}

		if (list_empty(&heap->pagecache_lru)) {
			spin_lock(&heap->pagecache_lock);
			break;
		}
repeat:
		page = NULL;
		spin_lock(&heap->pagecache_lock);
	}

	list_splice_tail(&failed_pages, &heap->pagecache_lru);
	spin_unlock(&heap->pagecache_lock);

	mem_cgroup_uncharge_end();
	printk("==================  shrink end loop_count: %d, fail_cnt: %d, shrunk: %d\n", loop_count, fail_cnt, shrunk);
	atomic_dec(&heap->shrinking);
	return shrunk;

failed:
	unlock_page(page);
	list_add(&handle->list, &failed_pages);
	fail_cnt++;
	goto repeat;
}

static int __ion_pagecache_shrink(struct ion_heap *heap, unsigned long max_scan,
				  int tryhard, gfp_t gfp_mask)
{
	int shrunk = 0;
	struct page *page;
	struct ion_handle *handle;
	LIST_HEAD(failed_pages);

	atomic_inc(&heap->shrinking);
	mem_cgroup_uncharge_start();
repeat:
	page = NULL;
	spin_lock(&heap->pagecache_lock);
	list_for_each_entry(handle, &heap->pagecache_lru, list) {
		struct address_space *mapping;
		int locked;
		ion_phys_addr_t addr;
		size_t size;

		if (heap->ops->phys(heap, handle->buffer, &addr, &size)) {
			page = NULL;
			continue;
		}
		page = phys_to_page(addr);

		locked = trylock_page(page);
		if (!(tryhard || locked)) {
			page = NULL;
			continue;
		}
		list_del_init(&handle->list);
		spin_unlock(&heap->pagecache_lock);
		if (!locked) {
			might_sleep();
			__lock_page(page);
		}

		if (unlikely(TestClearPageMlocked(page)))
			__clear_page_mlock(page);
		mapping = page_mapping(page);
		if (page_mapped(page) && mapping) {
			if (try_to_unmap(page, TTU_UNMAP|TTU_IGNORE_ACCESS)
					!= SWAP_SUCCESS)
				goto failed;
		}
		if (page_has_private(page) &&
				!try_to_release_page(page, gfp_mask)) {
			//printk(KERN_ERR "ion: pagecache_shrink: "
			//		"failed to release page %p\n", page);
			goto failed;
		} else {
			if (mapping) {
				spin_lock_irq(&mapping->tree_lock);
				__delete_from_page_cache(page);
				spin_unlock_irq(&mapping->tree_lock);
				mem_cgroup_uncharge_cache_page(page);
			}
		}
		unlock_page(page);
		break;
	}

	if (page) {
		if (likely(page_count(page) > 0)) {
			page_cache_release(page);	/* pagecache ref */
			shrunk++;
		}
		if (shrunk < max_scan && !list_empty(&heap->pagecache_lru))
			goto repeat;
		spin_lock(&heap->pagecache_lock);
	}
	list_splice_tail(&failed_pages, &heap->pagecache_lru);
	spin_unlock(&heap->pagecache_lock);

	mem_cgroup_uncharge_end();
	atomic_dec(&heap->shrinking);
//printk("tryhard: %d max_scan:%d gfp:%x shrunk: %d\n",
//	tryhard, max_scan, gfp_mask, shrunk);
//printk("heap size %d allocated:%d cachedpages: %d\n",
//	heap->size, heap->allocated, heap->cachedpages);
	return shrunk;
failed:
	unlock_page(page);
	list_add(&handle->list, &failed_pages);
	goto repeat;
}


#ifdef CONFIG_ION_PAGECACHE
int ion_normal_allocated_page_count(void)
{
	int count = 0;
	struct ion_device *ion_dev;
	struct rb_node *n;

	if (!ion_client_pagecache)
		return -1;

	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps); n != NULL; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);

		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;
		count += heap->allocated -heap->cachedpages;
	}
	return count;
}
#endif

int ion_pagecache_shrink(unsigned long max_scan, gfp_t gfp_mask)
{
	int shrunk;
	struct ion_device *ion_dev;
	struct rb_node *n;

	if (!ion_client_pagecache)
		return -1;
	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps), shrunk = 0;
	     n != NULL && shrunk < max_scan; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);

		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;
		shrunk += heap->size - heap->allocated;
	}
/*
	for (n = rb_first(&ion_dev->heaps); n != NULL && shrunk < max_scan;
			n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);

		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;
		if (heap->cachedpages > 0)
			shrunk += __ion_pagecache_shrink(heap,
					max_scan - shrunk, 0, gfp_mask);
	}
*/
	return shrunk > max_scan ? max_scan : shrunk;
}

#ifdef CONFIG_ION_PAGECACHE
void ion_pagecache_shrink_all(void)
{
	int shrunk;
	struct ion_device *ion_dev;
	struct rb_node *n;

	if (!ion_client_pagecache)
		return -1;
	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps); n != NULL && n != NULL; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);

		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;

		if (heap->cachedpages > 0)
			__ion_pagecache_shrink(heap,
					heap->cachedpages, 0, 0);
	}
}

void ion_pagecache_range_shrink_reset(void)
{
	struct ion_device *ion_dev;
	struct rb_node *n;

	if (!ion_client_pagecache)
		return -1;
	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps); n != NULL && n != NULL; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);
		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;
		heap->rangeshrunk = ion_carveout_heap_start_pfn(heap);
	}
	return;
}

void ion_pagecache_range_shrink(int size)
{
	int shrunk;
	struct ion_device *ion_dev;
	struct rb_node *n;

	if (!ion_client_pagecache)
		return -1;
	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps); n != NULL && n != NULL; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);
		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;

		if (heap->cachedpages > 0) {
			unsigned long heap_end_pfn;
			heap_end_pfn = ion_carveout_heap_start_pfn(heap) + heap->size - 1;
			if (heap->rangeshrunk <= heap_end_pfn) {
				unsigned long start_pfn;
				unsigned long end_pfn;

				start_pfn = heap->rangeshrunk;
				end_pfn = start_pfn + size - 1;
				if (end_pfn > heap_end_pfn)
					end_pfn = heap_end_pfn;

				__ion_pagecache_range_shrink(heap,
					start_pfn, end_pfn, 0, 0);
				heap->rangeshrunk += size;
			}
		}
	}
}
#endif

void ion_activate_page(struct page *page)
{
	unsigned long addr = page_to_phys(page);
	struct ion_device *ion_dev;
	struct rb_node *n;
	struct ion_handle *handle;

	if (!ion_client_pagecache)
		return;
	ion_dev = ion_client_pagecache->dev;

	for (n = rb_first(&ion_dev->heaps), handle = NULL;
	     n != NULL && handle == NULL; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);

		if (!((1 << heap->type) & ion_client_pagecache->heap_mask))
			continue;
		if (!heap->cachedpages)
			continue;
		spin_lock(&heap->pagecache_lock);
		handle = ion_pagecache_rb_search(&heap->pagecaches, addr);
		if (handle) {
			list_del(&handle->list);
			list_add_tail(&handle->list, &heap->pagecache_lru);
		}
		spin_unlock(&heap->pagecache_lock);
	}
}
#endif

/* this function should only be called while dev->lock is held */
static void ion_buffer_add(struct ion_device *dev,
			   struct ion_buffer *buffer)
{
	struct rb_node **p = &dev->buffers.rb_node;
	struct rb_node *parent = NULL;
	struct ion_buffer *entry;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_buffer, node);

		if (buffer < entry) {
			p = &(*p)->rb_left;
		} else if (buffer > entry) {
			p = &(*p)->rb_right;
		} else {
			pr_err("%s: buffer already found.", __func__);
			BUG();
		}
	}

	rb_link_node(&buffer->node, parent, p);
	rb_insert_color(&buffer->node, &dev->buffers);
}

/* this function should only be called while dev->lock is held */
static struct ion_buffer *ion_buffer_create(struct ion_heap *heap,
				     struct ion_device *dev,
				     unsigned long len,
				     unsigned long align,
				     unsigned long flags)
{
	struct ion_buffer *buffer;
	int ret;
#ifdef CONFIG_ION_PAGECACHE
	if ((flags & ION_ALLOC_PAGECACHE_MASK) && atomic_read(&heap->shrinking))
		return ERR_PTR(-EAGAIN);
#endif

	buffer = kzalloc(sizeof(struct ion_buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	buffer->heap = heap;
	kref_init(&buffer->ref);

	ret = heap->ops->allocate(heap, buffer, len, align, flags);
	if (ret) {
		kfree(buffer);
		return ERR_PTR(ret);
	}
	buffer->dev = dev;
	buffer->size = len;
	mutex_init(&buffer->lock);
	ion_buffer_add(dev, buffer);
	return buffer;
}

static void ion_buffer_destroy(struct kref *kref)
{
	struct ion_buffer *buffer = container_of(kref, struct ion_buffer, ref);
	struct ion_device *dev = buffer->dev;

	buffer->heap->ops->free(buffer);
	mutex_lock(&dev->lock);
	rb_erase(&buffer->node, &dev->buffers);
	mutex_unlock(&dev->lock);
	kfree(buffer);
}

static void ion_buffer_get(struct ion_buffer *buffer)
{
	kref_get(&buffer->ref);
}

static int ion_buffer_put(struct ion_buffer *buffer)
{
	return kref_put(&buffer->ref, ion_buffer_destroy);
}

static struct ion_handle *ion_handle_create(struct ion_client *client,
				     struct ion_buffer *buffer)
{
	struct ion_handle *handle;

	handle = kzalloc(sizeof(struct ion_handle), GFP_KERNEL);
	if (!handle)
		return ERR_PTR(-ENOMEM);
	kref_init(&handle->ref);
	rb_init_node(&handle->node);
	handle->client = client;
	ion_buffer_get(buffer);
	handle->buffer = buffer;
#ifdef CONFIG_ION_PAGECACHE
	INIT_LIST_HEAD(&handle->list);
	rb_init_node(&handle->pagecache);
#endif
	handle->magicnum = 0x12121212;
	return handle;
}

static void ion_handle_destroy(struct kref *kref)
{
	struct ion_handle *handle = container_of(kref, struct ion_handle, ref);
	/* XXX Can a handle be destroyed while it's map count is non-zero?:
	   if (handle->map_cnt) unmap
	 */
	ion_buffer_put(handle->buffer);
	if (!RB_EMPTY_NODE(&handle->node))
		rb_erase(&handle->node, &handle->client->handles);
	handle->magicnum = 0x34343434;
	kfree(handle);
}

struct ion_buffer *ion_handle_buffer(struct ion_handle *handle)
{
	return handle->buffer;
}

static void ion_handle_get(struct ion_handle *handle)
{
	kref_get(&handle->ref);
}

static int ion_handle_put(struct ion_handle *handle)
{
	return kref_put(&handle->ref, ion_handle_destroy);
}

static struct ion_handle *ion_handle_lookup(struct ion_client *client,
					    struct ion_buffer *buffer)
{
	struct rb_node *n;

	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);
		if (handle->buffer == buffer)
			return handle;
	}
	return NULL;
}

static bool ion_handle_validate(struct ion_client *client, struct ion_handle *handle)
{
	struct rb_node *n = client->handles.rb_node;

	while (n) {
		struct ion_handle *handle_node = rb_entry(n, struct ion_handle,
							  node);
		if (handle < handle_node)
			n = n->rb_left;
		else if (handle > handle_node)
			n = n->rb_right;
		else
			return true;
	}
	return false;
}

static void ion_handle_add(struct ion_client *client, struct ion_handle *handle)
{
	struct rb_node **p = &client->handles.rb_node;
	struct rb_node *parent = NULL;
	struct ion_handle *entry;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_handle, node);

		if (handle < entry)
			p = &(*p)->rb_left;
		else if (handle > entry)
			p = &(*p)->rb_right;
		else
			WARN(1, "%s: buffer already found.", __func__);
	}

	rb_link_node(&handle->node, parent, p);
	rb_insert_color(&handle->node, &client->handles);
}

struct ion_handle *ion_alloc(struct ion_client *client, size_t len,
			     size_t align, unsigned int flags)
{
	struct rb_node *n;
	struct ion_handle *handle;
	struct ion_device *dev = client->dev;
	struct ion_buffer *buffer = NULL;
#ifdef CONFIG_ION_PAGECACHE
	int pagecache = flags & ION_ALLOC_PAGECACHE_MASK;
	int tryhard = 0, num = len >> PAGE_SHIFT;

	if (!pagecache)
		ion_pagecache_range_shrink(num + 1);
repeat:
#endif

	/*
	 * traverse the list of heaps available in this system in priority
	 * order.  If the heap type is supported by the client, and matches the
	 * request of the caller allocate from it.  Repeat until allocate has
	 * succeeded or all heaps have been tried
	 */
	mutex_lock(&dev->lock);
	for (n = rb_first(&dev->heaps); n != NULL; n = rb_next(n)) {
		struct ion_heap *heap = rb_entry(n, struct ion_heap, node);
		/* if the client doesn't support this heap type */
		if (!((1 << heap->type) & client->heap_mask))
			continue;
		/* if the caller didn't specify this heap type */
		if (!((1 << heap->id) & flags))
			continue;
		buffer = ion_buffer_create(heap, dev, len, align, flags);
		if (!IS_ERR_OR_NULL(buffer))
			break;
	}
	mutex_unlock(&dev->lock);

	if (IS_ERR_OR_NULL(buffer)) {
#ifdef CONFIG_ION_PAGECACHE
		if (pagecache)
			return ERR_PTR(PTR_ERR(buffer));
		mutex_lock(&dev->lock);
		for (n = rb_first(&dev->heaps); n != NULL; n = rb_next(n)) {
			struct ion_heap *heap = rb_entry(n, struct ion_heap, node);
			/* if the client doesn't support this heap type */
			if (!((1 << heap->type) & client->heap_mask))
				continue;
			/* if the caller didn't specify this heap type */
			if (!((1 << heap->id) & flags))
				continue;
			if ((heap->size - heap->allocated + heap->cachedpages)
					< num)
				continue;
			mutex_unlock(&dev->lock);

			__ion_pagecache_shrink(heap, (tryhard + 1) * num, tryhard, GFP_USER);

			if (tryhard++ < 5) {
				if ((tryhard == 5) || ((tryhard + 1) * num) > heap->size)
					ion_pagecache_shrink_all();
				cond_resched();
				goto repeat;
			}

			mutex_lock(&dev->lock);
		}
		mutex_unlock(&dev->lock);
#endif
		return ERR_PTR(PTR_ERR(buffer));
	}

	handle = ion_handle_create(client, buffer);

	/*
	 * ion_buffer_create will create a buffer with a ref_cnt of 1,
	 * and ion_handle_create will take a second reference, drop one here
	 */
	ion_buffer_put(buffer);

	if (IS_ERR_OR_NULL(handle))
		return handle;

	mutex_lock(&client->lock);
	ion_handle_add(client, handle);
	mutex_unlock(&client->lock);

#ifdef CONFIG_ION_PAGECACHE
	buffer->pid = current->pid;
#endif
	return handle;
}

void ion_free(struct ion_client *client, struct ion_handle *handle)
{
	bool valid_handle;

	BUG_ON(client != handle->client);

	mutex_lock(&client->lock);
	valid_handle = ion_handle_validate(client, handle);
	if (!valid_handle) {
		mutex_unlock(&client->lock);
		WARN("%s: invalid handle passed to free.\n", __func__);
		return;
	}
	ion_handle_put(handle);
	mutex_unlock(&client->lock);
}

static void ion_client_get(struct ion_client *client);
static int ion_client_put(struct ion_client *client);

static bool _ion_map(int *buffer_cnt, int *handle_cnt)
{
	bool map;

	BUG_ON(*handle_cnt != 0 && *buffer_cnt == 0);

	if (*buffer_cnt)
		map = false;
	else
		map = true;
	if (*handle_cnt == 0)
		(*buffer_cnt)++;
	(*handle_cnt)++;
	return map;
}

static bool _ion_unmap(int *buffer_cnt, int *handle_cnt)
{
	BUG_ON(*handle_cnt == 0);
	(*handle_cnt)--;
	if (*handle_cnt != 0)
		return false;
	BUG_ON(*buffer_cnt == 0);
	(*buffer_cnt)--;
	if (*buffer_cnt == 0)
		return true;
	return false;
}

int ion_phys(struct ion_client *client, struct ion_handle *handle,
	     ion_phys_addr_t *addr, size_t *len)
{
	struct ion_buffer *buffer;
	int ret;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		mutex_unlock(&client->lock);
		return -EINVAL;
	}

	buffer = handle->buffer;

	if (!buffer->heap->ops->phys) {
		pr_err("%s: ion_phys is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return -ENODEV;
	}
	mutex_unlock(&client->lock);
	ret = buffer->heap->ops->phys(buffer->heap, buffer, addr, len);
	return ret;
}

void *ion_map_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;
	void *vaddr;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to map_kernel.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}

	buffer = handle->buffer;
	mutex_lock(&buffer->lock);

	if (!handle->buffer->heap->ops->map_kernel) {
		pr_err("%s: map_kernel is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&buffer->lock);
		mutex_unlock(&client->lock);
		return ERR_PTR(-ENODEV);
	}

	if (_ion_map(&buffer->kmap_cnt, &handle->kmap_cnt)) {
		vaddr = buffer->heap->ops->map_kernel(buffer->heap, buffer);
		if (IS_ERR_OR_NULL(vaddr))
			_ion_unmap(&buffer->kmap_cnt, &handle->kmap_cnt);
		buffer->vaddr = vaddr;
	} else {
		vaddr = buffer->vaddr;
	}
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
	return vaddr;
}

struct scatterlist *ion_map_dma(struct ion_client *client,
				struct ion_handle *handle)
{
	struct ion_buffer *buffer;
	struct scatterlist *sglist;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to map_dma.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);

	if (!handle->buffer->heap->ops->map_dma) {
		pr_err("%s: map_kernel is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&buffer->lock);
		mutex_unlock(&client->lock);
		return ERR_PTR(-ENODEV);
	}
	if (_ion_map(&buffer->dmap_cnt, &handle->dmap_cnt)) {
		sglist = buffer->heap->ops->map_dma(buffer->heap, buffer);
		if (IS_ERR_OR_NULL(sglist))
			_ion_unmap(&buffer->dmap_cnt, &handle->dmap_cnt);
		buffer->sglist = sglist;
	} else {
		sglist = buffer->sglist;
	}
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
	return sglist;
}

void ion_unmap_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	if (_ion_unmap(&buffer->kmap_cnt, &handle->kmap_cnt)) {
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
}

void ion_unmap_dma(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	if (_ion_unmap(&buffer->dmap_cnt, &handle->dmap_cnt)) {
		buffer->heap->ops->unmap_dma(buffer->heap, buffer);
		buffer->sglist = NULL;
	}
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
}


struct ion_buffer *ion_share(struct ion_client *client,
				 struct ion_handle *handle)
{
	bool valid_handle;

	mutex_lock(&client->lock);
	valid_handle = ion_handle_validate(client, handle);
	mutex_unlock(&client->lock);
	if (!valid_handle) {
		WARN("%s: invalid handle passed to share.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	/* do not take an extra reference here, the burden is on the caller
	 * to make sure the buffer doesn't go away while it's passing it
	 * to another client -- ion_free should not be called on this handle
	 * until the buffer has been imported into the other client
	 */
	return handle->buffer;
}

struct ion_handle *ion_import(struct ion_client *client,
			      struct ion_buffer *buffer)
{
	struct ion_handle *handle = NULL;

	mutex_lock(&client->lock);
	/* if a handle exists for this buffer just take a reference to it */
	handle = ion_handle_lookup(client, buffer);
	if (!IS_ERR_OR_NULL(handle)) {
		ion_handle_get(handle);
		goto end;
	}
	handle = ion_handle_create(client, buffer);
	if (IS_ERR_OR_NULL(handle))
		goto end;
	ion_handle_add(client, handle);
end:
	mutex_unlock(&client->lock);
	return handle;
}

static const struct file_operations ion_share_fops;

struct ion_handle *ion_import_fd(struct ion_client *client, int fd)
{
	struct file *file = fget(fd);
	struct ion_handle *handle;

	if (!file) {
		pr_err("%s: imported fd not found in file table.\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	if (file->f_op != &ion_share_fops) {
		pr_err("%s: imported file is not a shared ion file.\n",
		       __func__);
		handle = ERR_PTR(-EINVAL);
		goto end;
	}
	handle = ion_import(client, file->private_data);
end:
	fput(file);
	return handle;
}

void show_carveout_buddy_info(int rst);
static int ion_debug_client_show(struct seq_file *s, void *unused)
{
	struct ion_client *client = s->private;
	struct rb_node *n;
	size_t sizes[ION_NUM_HEAPS] = {0};
	const char *names[ION_NUM_HEAPS] = {0};
#ifdef CONFIG_ION_PAGECACHE
	int allocated[ION_NUM_HEAPS] = {0};
	int cachedpages[ION_NUM_HEAPS] = {0};
#endif

	int i;

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);
		enum ion_heap_type type = handle->buffer->heap->type;
#ifdef CONFIG_ION_PAGECACHE
		if (!names[type]) {
			names[type] = handle->buffer->heap->name;
			allocated[type] = handle->buffer->heap->allocated;
			cachedpages[type] = handle->buffer->heap->cachedpages; 
		}
#else
		if (!names[type])
			names[type] = handle->buffer->heap->name;
#endif
		sizes[type] += handle->buffer->size;
	}
	mutex_unlock(&client->lock);

#ifdef CONFIG_ION_PAGECACHE
	seq_printf(s, "ion_pagecache_flag: %d  ion_open_count: %d\n",
			atomic_read(&ion_pagecache_flag), atomic_read(&ion_open_count));
	seq_printf(s, "%16.16s: %16.16s, %16s, %16s\n", "heap_name", "size_in_bytes", "allocated", "cachedpages");
	seq_printf(s, "%llu %d\n", total_allocated_ion_cache, ion_normal_allocated_page_count());
#else
	seq_printf(s, "%16.16s: %16.16s\n", "heap_name", "size_in_bytes");
#endif
	for (i = 0; i < ION_NUM_HEAPS; i++) {
		if (!names[i])
			continue;
#ifdef CONFIG_ION_PAGECACHE
		seq_printf(s, "%16.16s: %16u  %16d  %16d %d\n", names[i], sizes[i], allocated[i], cachedpages[i],
				atomic_read(&client->ref.refcount));
#else
		seq_printf(s, "%16.16s: %16u %d\n", names[i], sizes[i],
			   atomic_read(&client->ref.refcount));
#endif
	}

#ifdef CONFIG_ION_BUDDY_CARVEOUT
	show_carveout_buddy_info(0);
#endif
	return 0;
}

static int ion_debug_client_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_client_show, inode->i_private);
}

static const struct file_operations debug_client_fops = {
	.open = ion_debug_client_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct ion_client *ion_client_lookup(struct ion_device *dev,
					    struct task_struct *task)
{
	struct rb_node *n = dev->user_clients.rb_node;
	struct ion_client *client;

	mutex_lock(&dev->lock);
	while (n) {
		client = rb_entry(n, struct ion_client, node);
		if (task == client->task) {
			ion_client_get(client);
			mutex_unlock(&dev->lock);
			return client;
		} else if (task < client->task) {
			n = n->rb_left;
		} else if (task > client->task) {
			n = n->rb_right;
		}
	}
	mutex_unlock(&dev->lock);
	return NULL;
}

struct ion_client *ion_client_create(struct ion_device *dev,
				     unsigned int heap_mask,
				     const char *name)
{
	struct ion_client *client;
	struct task_struct *task;
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct ion_client *entry;
	char debug_name[64];
	pid_t pid;

	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	pid = task_pid_nr(current->group_leader);
	/* don't bother to store task struct for kernel threads,
	   they can't be killed anyway */
	if (current->group_leader->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);

	/* if this isn't a kernel thread, see if a client already
	   exists */
	if (task) {
		client = ion_client_lookup(dev, task);
		if (!IS_ERR_OR_NULL(client)) {
			put_task_struct(current->group_leader);
			return client;
		}
	}

	client = kzalloc(sizeof(struct ion_client), GFP_KERNEL);
	if (!client) {
		put_task_struct(current->group_leader);
		return ERR_PTR(-ENOMEM);
	}

	client->dev = dev;
	client->handles = RB_ROOT;
	mutex_init(&client->lock);
	client->name = name;
	client->heap_mask = heap_mask;
	client->task = task;
	client->pid = pid;
	kref_init(&client->ref);

	mutex_lock(&dev->lock);
	if (task) {
		p = &dev->user_clients.rb_node;
		while (*p) {
			parent = *p;
			entry = rb_entry(parent, struct ion_client, node);

			if (task < entry->task)
				p = &(*p)->rb_left;
			else if (task > entry->task)
				p = &(*p)->rb_right;
		}
		rb_link_node(&client->node, parent, p);
		rb_insert_color(&client->node, &dev->user_clients);
	} else {
		p = &dev->kernel_clients.rb_node;
		while (*p) {
			parent = *p;
			entry = rb_entry(parent, struct ion_client, node);

			if (client < entry)
				p = &(*p)->rb_left;
			else if (client > entry)
				p = &(*p)->rb_right;
		}
		rb_link_node(&client->node, parent, p);
		rb_insert_color(&client->node, &dev->kernel_clients);
	}

	snprintf(debug_name, 64, "%u", client->pid);
	client->debug_root = debugfs_create_file(debug_name, 0664,
						 dev->debug_root, client,
						 &debug_client_fops);
	mutex_unlock(&dev->lock);

	return client;
}

static void _ion_client_destroy(struct kref *kref)
{
	struct ion_client *client = container_of(kref, struct ion_client, ref);
	struct ion_device *dev = client->dev;
	struct rb_node *n;

	pr_debug("%s: %d\n", __func__, __LINE__);
	while ((n = rb_first(&client->handles))) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);
		ion_handle_destroy(&handle->ref);
	}
	mutex_lock(&dev->lock);
	if (client->task) {
		rb_erase(&client->node, &dev->user_clients);
		put_task_struct(client->task);
	} else {
		rb_erase(&client->node, &dev->kernel_clients);
	}
	debugfs_remove_recursive(client->debug_root);
	mutex_unlock(&dev->lock);

	kfree(client);
}

static void ion_client_get(struct ion_client *client)
{
	kref_get(&client->ref);
}

static int ion_client_put(struct ion_client *client)
{
	return kref_put(&client->ref, _ion_client_destroy);
}

void ion_client_destroy(struct ion_client *client)
{
	ion_client_put(client);
}

static int ion_share_release(struct inode *inode, struct file* file)
{
	struct ion_buffer *buffer = file->private_data;

	pr_debug("%s: %d\n", __func__, __LINE__);
	/* drop the reference to the buffer -- this prevents the
	   buffer from going away because the client holding it exited
	   while it was being passed */
	ion_buffer_put(buffer);
	return 0;
}

static void ion_vma_open(struct vm_area_struct *vma)
{

	struct ion_buffer *buffer = vma->vm_file->private_data;
	struct ion_handle *handle = vma->vm_private_data;
	struct ion_client *client;

	pr_debug("%s: %d\n", __func__, __LINE__);
	/* check that the client still exists and take a reference so
	   it can't go away until this vma is closed */
	client = ion_client_lookup(buffer->dev, current->group_leader);
	if (IS_ERR_OR_NULL(client)) {
		vma->vm_private_data = NULL;
		return;
	}
	ion_handle_get(handle);
	pr_debug("%s: %d client_cnt %d handle_cnt %d alloc_cnt %d\n",
		 __func__, __LINE__,
		 atomic_read(&client->ref.refcount),
		 atomic_read(&handle->ref.refcount),
		 atomic_read(&buffer->ref.refcount));
}

static void ion_vma_close(struct vm_area_struct *vma)
{
	struct ion_handle *handle = vma->vm_private_data;
	struct ion_buffer *buffer = vma->vm_file->private_data;
	struct ion_client *client;

	pr_debug("%s: %d\n", __func__, __LINE__);
	/* this indicates the client is gone, nothing to do here */
	if (!handle)
		return;

	client = handle->client;
	pr_debug("%s: %d client_cnt %d handle_cnt %d alloc_cnt %d\n",
		 __func__, __LINE__,
		 atomic_read(&client->ref.refcount),
		 atomic_read(&handle->ref.refcount),
		 atomic_read(&buffer->ref.refcount));
	printk("%s ---- magicnum: %p, client:%p, buffer:%p\n", __func__, handle->magicnum, client, handle->buffer);
	mutex_lock(&client->lock);
	ion_handle_put(handle);
	mutex_unlock(&client->lock);
	ion_client_put(client);
	pr_debug("%s: %d client_cnt %d handle_cnt %d alloc_cnt %d\n",
		 __func__, __LINE__,
		 atomic_read(&client->ref.refcount),
		 atomic_read(&handle->ref.refcount),
		 atomic_read(&buffer->ref.refcount));
}

static struct vm_operations_struct ion_vm_ops = {
	.open = ion_vma_open,
	.close = ion_vma_close,
};

static int ion_share_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ion_buffer *buffer = file->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	struct ion_client *client;
	struct ion_handle *handle;
	int ret;

	pr_debug("%s: %d\n", __func__, __LINE__);
	/* make sure the client still exists, it's possible for the client to
	   have gone away but the map/share fd still to be around, take
	   a reference to it so it can't go away while this mapping exists */
	client = ion_client_lookup(buffer->dev, current->group_leader);
	if (IS_ERR_OR_NULL(client)) {
		pr_err("%s: trying to mmap an ion handle in a process with no "
		       "ion client\n", __func__);
		return -EINVAL;
	}

	if ((size > buffer->size) || (size + (vma->vm_pgoff << PAGE_SHIFT) >
				     buffer->size)) {
		pr_err("%s: trying to map larger area than handle has available"
		       "\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	/* find the handle and take a reference to it */
	handle = ion_import(client, buffer);
	if (IS_ERR_OR_NULL(handle)) {
		ret = -EINVAL;
		goto err;
	}

	if (!handle->buffer->heap->ops->map_user) {
		pr_err("%s: this heap does not define a method for mapping "
		       "to userspace\n", __func__);
		ret = -EINVAL;
		goto err1;
	}

	mutex_lock(&buffer->lock);
	/* now map it to userspace */
	ret = buffer->heap->ops->map_user(buffer->heap, buffer, vma);
	mutex_unlock(&buffer->lock);
	if (ret) {
		pr_err("%s: failure mapping buffer to userspace\n",
		       __func__);
		goto err1;
	}

	vma->vm_ops = &ion_vm_ops;
	/* move the handle into the vm_private_data so we can access it from
	   vma_open/close */
	vma->vm_private_data = handle;
	pr_debug("%s: %d client_cnt %d handle_cnt %d alloc_cnt %d\n",
		 __func__, __LINE__,
		 atomic_read(&client->ref.refcount),
		 atomic_read(&handle->ref.refcount),
		 atomic_read(&buffer->ref.refcount));
	return 0;

err1:
	/* drop the reference to the handle */
	mutex_lock(&client->lock);
	ion_handle_put(handle);
	mutex_unlock(&client->lock);
err:
	/* drop the reference to the client */
	ion_client_put(client);
	return ret;
}

static const struct file_operations ion_share_fops = {
	.owner		= THIS_MODULE,
	.release	= ion_share_release,
	.mmap		= ion_share_mmap,
};

static int ion_ioctl_share(struct file *parent, struct ion_client *client,
			   struct ion_handle *handle)
{
	int fd = get_unused_fd();
	struct file *file;

	if (fd < 0)
		return -ENFILE;

	file = anon_inode_getfile("ion_share_fd", &ion_share_fops,
				  handle->buffer, O_RDWR);
	if (IS_ERR_OR_NULL(file))
		goto err;
	ion_buffer_get(handle->buffer);
	fd_install(fd, file);

	return fd;

err:
	put_unused_fd(fd);
	return -ENFILE;
}

static int ion_check_all_handle_magicnum(void);
static long ion_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ion_client *client = filp->private_data;

/*
ION_IOC_ALLOC          0xc0104900
ION_IOC_FREE           0xc0044901
ION_IOC_MAP            0xc0084902
ION_IOC_SHARE          0xc0084904
ION_IOC_IMPORT         0xc0044905
ION_IOC_CUSTOM         0xc0084906
ION_IOC_ENABLE_CACHE   0xc0044907
ION_IOC_DISABLE_CACHE  0xc0044908
*/
	printk("\nion_ioctl start  cmd 0x%p\n", cmd);
	ion_check_all_handle_magicnum();
#ifdef CONFIG_ION_BUDDY_CHECKPAGE
	show_carveout_buddy_info(1);
#endif

	switch (cmd) {
	case ION_IOC_ALLOC:
	{
		struct ion_allocation_data data;

		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;

#ifdef CONFIG_ION_PAGECACHE
		ion_normal_allocing = 1;
#endif
		data.handle = ion_alloc(client, data.len, data.align,
					     data.flags);
#ifdef CONFIG_ION_PAGECACHE
		ion_normal_allocing = 0;
#endif
		if (IS_ERR_OR_NULL(data.handle))
			return -EINVAL;
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		break;
	}
	case ION_IOC_FREE:
	{
		struct ion_handle_data data;
		bool valid;

		if (copy_from_user(&data, (void __user *)arg,
				   sizeof(struct ion_handle_data)))
			return -EFAULT;
		mutex_lock(&client->lock);
		valid = ion_handle_validate(client, data.handle);
		mutex_unlock(&client->lock);
		if (!valid)
			return -EINVAL;
		ion_free(client, data.handle);
		break;
	}
	case ION_IOC_MAP:
	case ION_IOC_SHARE:
	{
		struct ion_fd_data data;

		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;
		mutex_lock(&client->lock);
		if (!ion_handle_validate(client, data.handle)) {
			pr_err("%s: invalid handle passed to share ioctl.\n",
			       __func__);
			mutex_unlock(&client->lock);
			return -EINVAL;
		}
		data.fd = ion_ioctl_share(filp, client, data.handle);
		mutex_unlock(&client->lock);
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		break;
	}
	case ION_IOC_IMPORT:
	{
		struct ion_fd_data data;
		if (copy_from_user(&data, (void __user *)arg,
				   sizeof(struct ion_fd_data)))
			return -EFAULT;

		data.handle = ion_import_fd(client, data.fd);
		if (IS_ERR(data.handle))
			data.handle = NULL;
		if (copy_to_user((void __user *)arg, &data,
				 sizeof(struct ion_fd_data)))
			return -EFAULT;
		break;
	}
	case ION_IOC_CUSTOM:
	{
		struct ion_device *dev = client->dev;
		struct ion_custom_data data;

		if (!dev->custom_ioctl)
			return -ENOTTY;
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(struct ion_custom_data)))
			return -EFAULT;
		return dev->custom_ioctl(client, data.cmd, data.arg);
	}

#ifdef CONFIG_ION_PAGECACHE
	case ION_IOC_ENABLE_CACHE:
	{
		atomic_set(&ion_pagecache_flag, 1);
		ion_pagecache_range_shrink_reset();
		break;
	}

	case ION_IOC_DISABLE_CACHE:
	{
		atomic_set(&ion_pagecache_flag, 0);
		break;
	}
#endif
	default:
		return -ENOTTY;
	}

	printk("ion_ioctl end cmd 0x%p\n", cmd);
	ion_check_all_handle_magicnum();
#ifdef CONFIG_ION_BUDDY_CHECKPAGE
	show_carveout_buddy_info(1);
#endif
	return 0;
}

static int ion_release(struct inode *inode, struct file *file)
{
	struct ion_client *client = file->private_data;

	pr_debug("%s: %d\n", __func__, __LINE__);
	ion_client_put(client);

#ifdef CONFIG_ION_PAGECACHE
	atomic_dec(&ion_open_count);
	if (!atomic_read(&ion_open_count)) {
		ion_pagecache_range_shrink_reset();
		atomic_set(&ion_pagecache_flag, 1);
	}
#endif
	return 0;
}

static int ion_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ion_device *dev = container_of(miscdev, struct ion_device, dev);
	struct ion_client *client;

	pr_debug("%s: %d\n", __func__, __LINE__);
	client = ion_client_create(dev, -1, "user");

	if (IS_ERR_OR_NULL(client))
		return PTR_ERR(client);
	file->private_data = client;
#ifdef CONFIG_ION_PAGECACHE
	atomic_inc(&ion_open_count);
#endif
	return 0;
}

static const struct file_operations ion_fops = {
	.owner          = THIS_MODULE,
	.open           = ion_open,
	.release        = ion_release,
	.unlocked_ioctl = ion_ioctl,
};

static size_t ion_debug_heap_total(struct ion_client *client,
				   enum ion_heap_type type, int id)
{
	size_t size = 0;
	struct rb_node *n;

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n,
						     struct ion_handle,
						     node);
		if (handle->buffer->heap->id != id)
			continue;
		if (handle->buffer->heap->type == type)
			size += handle->buffer->size;
	}
	mutex_unlock(&client->lock);
	return size;
}
static int ion_debug_heap_show(struct seq_file *s, void *unused)
{
	struct ion_heap *heap = s->private;
	struct ion_device *dev = heap->dev;
	struct rb_node *n;

	seq_printf(s, "%16.s %16.s %16.s\n", "client", "pid", "size");
	for (n = rb_first(&dev->user_clients); n; n = rb_next(n)) {
		struct ion_client *client = rb_entry(n, struct ion_client,
						     node);
		char task_comm[TASK_COMM_LEN];
		size_t size = ion_debug_heap_total(client, heap->type,
						   heap->id);
		if (!size)
			continue;

		get_task_comm(task_comm, client->task);
		seq_printf(s, "%16.s %16u %16u\n", task_comm, client->pid,
			   size);
	}

	for (n = rb_first(&dev->kernel_clients); n; n = rb_next(n)) {
		struct ion_client *client = rb_entry(n, struct ion_client,
						     node);
		size_t size = ion_debug_heap_total(client, heap->type,
						   heap->id);
		if (!size)
			continue;
		seq_printf(s, "%16.s %16u %16u\n", client->name, client->pid,
			   size);
	}


	seq_printf(s, "-----------------buffer list------------------------\n");
	mutex_lock(&dev->lock);
	for (n = rb_first(&dev->buffers); n; n = rb_next(n)) {
		struct ion_buffer *buffer = rb_entry(n, struct ion_buffer,
						     node);
		struct page *page;
		if (buffer->heap->id != heap->id)
			continue;
		if (buffer->heap->type != heap->type)
			continue;
		page = phys_to_page((unsigned int)buffer->priv_virt);
		/*
		seq_printf(s, "--- size= %16u kmap_cnt= %2d ref= %2d \
flag= 0x%8x phy= 0x%8x vaddr= 0x%8x\n",
			buffer->size,
			buffer->kmap_cnt,
			atomic_read(&buffer->ref.refcount),
			(unsigned int)buffer->flags,
			(unsigned int)buffer->priv_virt,
			(unsigned int)buffer->vaddr);
		*/
		seq_printf(s, "--- size= %16u kmap_cnt= %2d ref= %2d \
flag= 0x%8x pfn = 0x%8d vaddr= 0x%8x\n",
			buffer->size >> PAGE_SHIFT,
			buffer->kmap_cnt,
			atomic_read(&buffer->ref.refcount),
			(unsigned int)buffer->flags,
			page_to_pfn(page),
			(unsigned int)buffer->vaddr);
	}
	mutex_unlock(&dev->lock);
	seq_printf(s, "----------------------------------------------------\n");

	return 0;
}


static void ion_check_handle_magicnum(struct ion_client *client,
				   enum ion_heap_type type, int id)
{
	struct rb_node *n;

	//printk("%-16s %-16s %16u\n", client->name, client->task->comm, client->pid);
	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n,
						     struct ion_handle,
						     node);
		if (handle->buffer->heap->id != id)
			continue;

		if (handle->magicnum != 0x12121212)
			panic("magicnumerror  ---task(%d)%s, client:%s, ion_handle:0x%p, magicnum:0x%p, client:0x%p, buffer:0x%p\n",
			       client->pid, client->task->comm, client->name, handle, handle->magicnum, handle->client, handle->buffer);
	}
	mutex_unlock(&client->lock);

}

extern int num_heaps;
extern struct ion_heap **heaps;
static int ion_check_all_handle_magicnum(void)
{
	struct ion_heap *heap;
	struct ion_device *dev;
	int i;
	struct rb_node *n;

	for (i = 0; i < num_heaps; i++) {
		//printk("======  *heaps = 0x%p\n", heaps[0]);
		heap = heaps[i];
		dev = heap->dev;
		//printk("====== user handle's magicnum ====\n");
		for (n = rb_first(&dev->user_clients); n; n = rb_next(n)) {
			struct ion_client *client = rb_entry(n, struct ion_client, node);
			ion_check_handle_magicnum(client, heap->type, heap->id);
		}

		//printk("====== kernel handle's magicnum====\n");
		for (n = rb_first(&dev->kernel_clients); n; n = rb_next(n)) {
			struct ion_client *client = rb_entry(n, struct ion_client, node);
			ion_check_handle_magicnum(client, heap->type, heap->id);
		}
	}

}

static int ion_debug_heap_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_heap_show, inode->i_private);
}

static const struct file_operations debug_heap_fops = {
	.open = ion_debug_heap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void ion_device_add_heap(struct ion_device *dev, struct ion_heap *heap)
{
	struct rb_node **p = &dev->heaps.rb_node;
	struct rb_node *parent = NULL;
	struct ion_heap *entry;

	heap->dev = dev;
	mutex_lock(&dev->lock);
	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_heap, node);

		if (heap->id < entry->id) {
			p = &(*p)->rb_left;
		} else if (heap->id > entry->id ) {
			p = &(*p)->rb_right;
		} else {
			pr_err("%s: can not insert multiple heaps with "
				"id %d\n", __func__, heap->id);
			goto end;
		}
	}

	rb_link_node(&heap->node, parent, p);
	rb_insert_color(&heap->node, &dev->heaps);
	debugfs_create_file(heap->name, 0664, dev->debug_root, heap,
			    &debug_heap_fops);
end:
	mutex_unlock(&dev->lock);
}

struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg))
{
	struct ion_device *idev;
	int ret;

	idev = kzalloc(sizeof(struct ion_device), GFP_KERNEL);
	if (!idev)
		return ERR_PTR(-ENOMEM);

	idev->dev.minor = MISC_DYNAMIC_MINOR;
	idev->dev.name = "ion";
	idev->dev.fops = &ion_fops;
	idev->dev.parent = NULL;
	ret = misc_register(&idev->dev);
	if (ret) {
		pr_err("ion: failed to register misc device.\n");
		return ERR_PTR(ret);
	}

	idev->debug_root = debugfs_create_dir("ion", NULL);
	if (IS_ERR_OR_NULL(idev->debug_root))
		pr_err("ion: failed to create debug files.\n");

	idev->custom_ioctl = custom_ioctl;
	idev->buffers = RB_ROOT;
	mutex_init(&idev->lock);
	idev->heaps = RB_ROOT;
	idev->user_clients = RB_ROOT;
	idev->kernel_clients = RB_ROOT;
#ifdef CONFIG_ION_PAGECACHE
	if (!ion_client_pagecache)
		ion_client_pagecache = ion_client_create(idev, ION_HEAP_CARVEOUT_MASK, "pagecache");
#endif
	return idev;
}

void ion_device_destroy(struct ion_device *dev)
{
#ifdef CONFIG_ION_PAGECACHE
	if (ion_client_pagecache) {
		ion_client_destroy(ion_client_pagecache);
		ion_client_pagecache = NULL;
	}
#endif
	misc_deregister(&dev->dev);
	/* XXX need to free the heaps and clients ? */
	kfree(dev);
}
