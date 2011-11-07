/*
 ****************************************************************
 *
 * Component = VLX Virtual Block Device v.2 back-end driver
 *
 * Copyright (C) 2005-2011, VirtualLogix.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * #ident  "@(#)vlx-vbd-be.c 1.74     10/06/15 VirtualLogix"
 *
 * Contributor(s):
 *   Vladimir Grouzdev (vladimir.grouzdev@VirtualLogix.COM) VirtualLogix
 *   Eric Lescouet (eric.lescouet@VirtualLogix.COM) VirtualLogix
 *   Adam Mirowski (adam.mirowski@VirtualLogix.COM) VirtualLogix
 *
 ****************************************************************
 *
 * This driver is based on the original work done in Xen by
 * Keir Fraser & Steve Hand, for the virtual block driver back-end:
 */

/******************************************************************************
 * arch/xen/drivers/blkif/backend/main.c
 *
 * Back-end of the driver for virtual block devices. This portion of the
 * driver exports a 'unified' block-device interface that can be accessed
 * by any operating system that implements a compatible front end. A
 * reference front-end implementation can be found in:
 *  arch/xen/drivers/blkif/frontend
 *
 * Copyright (c) 2003-2004, Keir Fraser & Steve Hand
 *
 * This file may be distributed separately from the Linux kernel, or
 * incorporated into other software packages, subject to the following license:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/*----- System header files -----*/

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif
#include <linux/module.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/blkdev.h>
#include <linux/proc_fs.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    /* This include exists in 2.6.6 but functions are not yet exported */
#include <linux/kthread.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#include <linux/preempt.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
#include <linux/freezer.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
#include  <linux/sched.h>
#endif
#endif
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/pgalloc.h>
#include <linux/init.h>		/* module_init() in 2.6.0 and before */
#include <linux/cdrom.h>
#include <linux/loop.h>
#include <linux/smp_lock.h>	/* lock_kernel(), unlock_kernel() */
#include <asm/uaccess.h>
#include <nk/nkern.h>

/*----- Local configuration -----*/

#if 0
#define VBD_DEBUG
#endif

#ifdef CONFIG_X86
#define VBD2_ATAPI
#endif

#ifndef CONFIG_NKERNEL_VBD_NR
#define	CONFIG_NKERNEL_VBD_NR	8
#endif

#define	VBD_LINK_MAX_SEGS_PER_REQ	128
#define	VBD_LINK_DEFAULT_MSG_COUNT	64

    /*
     * Maximum number of requests pulled from a given link
     * before scheduling the next one. As there is only one
     * kernel thread currently for all the link interfaces,
     * it gives a chance to all interfaces to be scheduled
     * in a more fair way.
     */
#define VBD_LINK_MAX_REQ	16

/*----- Local header files -----*/

#define VLX_SERVICES_THREADS
#ifdef CONFIG_ARM
#define VLX_SERVICES_PROC_NK
#endif
#include "vlx-services.c"

#include <vlx/vbd2_common.h>
#include "vlx-vmq.h"

/*----- Tracing -----*/

#define TRACE(_f, _a...)	printk (KERN_INFO "VBD2-BE: " _f , ## _a)
#define ETRACE(_f, _a...)	printk (KERN_ERR  "VBD2-BE: " _f , ## _a)
#define EFTRACE(_f, _a...)	ETRACE ("%s: " _f, __func__, ## _a)
#define XTRACE(_f, _a...)

#ifdef VBD_DEBUG
#define DTRACE(_f, _a...)	printk (KERN_ALERT "%s: " _f, __func__, ## _a)
#define VBD_ASSERT(c)		BUG_ON (!(c))
#else
#define DTRACE(_f, _a...)
#define VBD_ASSERT(c)
#endif

/*----- Version compatibility functions -----*/

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,7)
    static void*
kzalloc (size_t size, unsigned flags)
{
    void* ptr = kmalloc (size, flags);

    if (ptr) {
	memset (ptr, 0, size);
    }
    return ptr;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define PDE(i)	((struct proc_dir_entry *)((i)->u.generic_ip))

#define mutex_lock(x)	down(x)
#define mutex_unlock(x)	up(x)
#define mutex_init(x)	init_MUTEX(x)
#define preempt_disable()
#define preempt_enable()
#ifndef BIO_MAX_PAGES
#define BIO_MAX_PAGES	128
#endif
#endif

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

/*----- Definitions -----*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
typedef sector_t	vbd_sector_t;
#else
typedef u64		vbd_sector_t;
#endif

#define VBD_PAGE_64_MASK	~((NkPhAddr)0xfff)

    /*
     *  The backend can avoid informing the frontend about specific
     *  vdisks until they are actually available, or better, till
     *  their sizes are also non-zero. This allows e.g. to hold the
     *  boot of a frontend guest using a CD for root until losetup(1)
     *  is performed in the backend guest.
     */
typedef enum {
    VBD_VDISK_WAIT_NO,		/* Do not wait for this vdisk */
    VBD_VDISK_WAIT_YES,		/* Wait for this vdisk */
    VBD_VDISK_WAIT_NON_ZERO	/* Wait for vdisk to be non-zero sized */
} vbd_vdisk_wait_t;

static const char vbd_vdisk_wait_modes [3] [3] = {"nw", "wa", "nz"};

    /* The "vdisk" property identifies a virtual disk */
typedef struct {
    nku32_f		tag;
    NkOsId		owner;	/* frontend */
    vbd2_devid_t	devid;
    vbd_vdisk_wait_t	wait;
} vbd_prop_vdisk_t;

    static inline void
vbd_prop_vdisk_init (vbd_prop_vdisk_t* pvd, const nku32_f tag,
		     const NkOsId owner, const vbd2_devid_t devid,
		     const vbd_vdisk_wait_t wait)
{
    pvd->tag   = tag;
    pvd->owner = owner;
    pvd->devid = devid;
    pvd->wait  = wait;
}

    /* The "extent" property identifies a virtual disk extent */
typedef struct {
    vbd_sector_t start;		/* Start sector */
    vbd_sector_t sectors;	/* Size in sectors (0 - up to the end) */
    nku32_f access;		/* Access rights */
    nku32_f minor;		/* Disk minor */
    nku32_f tag;		/* Vdisk tag */
    int     major;		/* Disk major (-1 if unused) */
    _Bool   bound;		/* Extent is bound to the real device */
} vbd_prop_extent_t;

    static inline void
vbd_prop_extent_init (vbd_prop_extent_t* pde, const vbd_sector_t start,
		      const vbd_sector_t sectors, const nku32_f access,
		      const nku32_f minor, const nku32_f tag, const int major)
{
    pde->start   = start;
    pde->sectors = sectors;
    pde->access  = access;
    pde->minor   = minor;
    pde->tag     = tag;
    pde->major   = major;
	/* bound = false; */
}

#define	VBD_DISK_ACC_R	0x1
#define	VBD_DISK_ACC_W	0x2
#define	VBD_DISK_ACC_RW	(VBD_DISK_ACC_R | VBD_DISK_ACC_W)

typedef struct vbd_be_t			vbd_be_t;
typedef struct vbd_link_t		vbd_link_t;
typedef struct vbd_fast_map_t		vbd_fast_map_t;
typedef struct vbd_pending_req_t	vbd_pending_req_t;
typedef struct vbd_vdisk_t		vbd_vdisk_t;

    /* Link descriptor */
struct vbd_link_t {
    vmq_link_t*		link;
    _Bool		connected;
    vmq_xx_config_t	xx_config;	/* Receive side */
    unsigned		segs_per_req_max;
    vbd_vdisk_t*	vdisks;
    vbd_be_t*		be;
    struct list_head	blkio_list;	/* (if set) on be.blkio_list */
    atomic_t		refcount;
    vbd_fast_map_t*	fast_map;
    spinlock_t		fast_map_lock;
    struct list_head	done_list;
    spinlock_t		done_list_lock;
    _Bool		vdisks_ok;
    vbd2_req_header_t*	pending_msg;	/* If resource error */
    vbd_pending_req_t*	pending_xreq;	/* If resource error */
    unsigned		changes;
    _Bool		changes_signaled;

	/* Performance counters */
    unsigned long long	bytes_read;
    unsigned long long	bytes_written;

    unsigned		fast_map_probes;
    unsigned		fast_map_reads;
    unsigned		fast_map_writes;
    unsigned long long	fast_map_read_bytes;
    unsigned long long	fast_map_written_bytes;

    unsigned		nk_mem_map_probes;
    unsigned		nk_mem_map_reads;
    unsigned		nk_mem_map_writes;
    unsigned long long	nk_mem_map_read_bytes;
    unsigned long long	nk_mem_map_written_bytes;

    unsigned		dma_reads;
    unsigned		dma_writes;
    unsigned long long	dma_read_bytes;
    unsigned long long	dma_written_bytes;

    unsigned		msg_replies;
    unsigned		requests;
    unsigned		segments;
    unsigned		no_struct_page;

    unsigned		page_allocs;
    unsigned		page_freeings;
    unsigned		alloced_pages;
    unsigned		max_alloced_pages;

    unsigned		first_meg_copy;
    unsigned		resource_errors;
};

#define VBD_LINK_FOR_ALL_VDISKS(_vd,_bl) \
    for ((_vd) = (_bl)->vdisks; (_vd); (_vd) = (_vd)->next)

#ifdef VBD2_ATAPI
    /* Virtual disk ATAPI descriptor */
typedef struct {
    vbd2_req_header_t*	req;
    vbd2_devid_t	devid;
    NkPhAddr		paddr;
    uint8_t		cdb [VBD2_ATAPI_PKT_SZ];
    uint32_t		buflen;
} vbd_atapi_req_t;

typedef struct {
    vbd_atapi_req_t	req;	/* Pending ATAPI request from frontend */
    uint32_t		count;	/* Request count (0-1) */
    wait_queue_head_t	wait;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    struct mutex	lock;
#else
    struct semaphore	lock;
#endif
    int			ctrl_open;
    int			data_open;
    char		ctrl_name [20];
    char		data_name [20];
} vbd_atapi_t;
#endif /* VBD2_ATAPI */

typedef struct vbd_extent_t vbd_extent_t;

    /* Virtual disk descriptor */
struct vbd_vdisk_t {
    nku32_f		tag;		/* Virtual disk tag */
    vbd2_devid_t	devid;		/* Virtual disk ID */
    vbd2_genid_t	genid;		/* Generation of devid */
    vbd_extent_t*	extents;	/* List of extents */
    vbd_vdisk_t*	next;		/* Next virtual disk */
    vbd_link_t*		bl;		/* Link to frontend */
    vbd_vdisk_wait_t	wait;		/* Vdisk wait mode (none, existence,
					   non-zero size) */
    _Bool		extents_ok;	/* Tmp used during extent scanning */
    char		name [24];	/* (guest,major,minor) string */
#ifdef VBD2_ATAPI
    vbd_atapi_t*	atapi;		/* ATAPI device support */
#endif
};

    static inline void
vbd_vdisk_init (vbd_vdisk_t* vd, nku32_f tag, vbd2_devid_t devid,
		vbd_link_t* bl, vbd_vdisk_wait_t wait)
{
    vd->tag        = tag;
    vd->devid      = devid;
	/* genid = 0; */
	/* extents = NULL; */
	/* next = NULL; */
    vd->bl   = bl;
    vd->wait = wait;
	/* extents_ok = false; */
    snprintf (vd->name, sizeof vd->name, "(%d,%d,%d:%d)",
	      vmq_peer_osid (bl->link), VBD2_DEVID_MAJOR (vd->devid),
	      VBD2_DEVID_MINOR (vd->devid), vd->genid);
	/* atapi = NULL; */
}

#define VBD_VDISK_FOR_ALL_EXTENTS(_ex,_vd) \
    for ((_ex) = (_vd)->extents; (_ex); (_ex) = (_ex)->next)

    /* Virtual disk extent descriptor */
struct vbd_extent_t {
    vbd_extent_t*		next;	/* Next extent */
    vbd_sector_t		start;	/* Start sector */
    vbd_sector_t		sectors;/* Size in sectors */
    dev_t			dev;	/* Real device id */
    struct block_device*	bdev;	/* Real block device */
    nku32_f			access;	/* Access rights */
    const vbd_prop_extent_t*	prop;	/* Property */
    vbd_vdisk_t*		vdisk;	/* Virtual disk */
};

    static inline void
vbd_extent_init (vbd_extent_t* ex, vbd_sector_t start,
		 vbd_sector_t sectors, dev_t dev, nku32_f access,
		 const vbd_prop_extent_t* prop, vbd_vdisk_t* vdisk)
{
    ex->start   = start;
    ex->sectors = sectors;
    ex->dev     = dev;
	/* bdev = NULL;	set at connect time */
    ex->access  = access;
    ex->prop    = prop;
    ex->vdisk   = vdisk;
}

    /* Driver descriptor */
struct vbd_be_t {
    vmq_links_t*	links;
    atomic_t		refcount;		/* Reference count */
    _Bool		resource_error;
    vbd_prop_extent_t	prop_extents [CONFIG_NKERNEL_VBD_NR];
    vbd_prop_vdisk_t	prop_vdisks [CONFIG_NKERNEL_VBD_NR];
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    kmem_cache_t*	pending_req_cachep;
#else
    struct kmem_cache*	pending_req_cachep;
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    kmem_cache_t*	buffer_head_cachep;
#endif
    wait_queue_head_t	blkio_wait;
    vlx_thread_t	blkio_thread_desc;
    struct list_head	blkio_list;
    spinlock_t		blkio_list_lock;
    struct task_struct*	blkio_task_struct;
    vlx_thread_t	extent_thread_desc;
    _Bool		extent_abort;
    _Bool		sysconf;
    unsigned		changes;
	/* Performance counters */
    unsigned long long	sleeps_no_timeout;
    unsigned		sleeps_with_timeout;
    unsigned		timeouts;
};

#define	VBD_VDISK(x)	((vbd_vdisk_t*)(x))
#define	VBD_EXTENT(x)	((vbd_extent_t*)(x))
#define	VBD_BE(x)	((vbd_be_t*)(x))

static int vbd2_dma = 1;	/* DMA (default) or COPY mode */

    /*
     * Each outstanding request that we've passed to the lower device
     * layers has a 'pending_req' allocated with it. Each buffer_head
     * that completes decrements the pendcount towards zero. When it
     * hits zero, a response is queued for with the saved 'cookie'
     * passed back.
     */
typedef struct vbd_pending_seg_t {
    NkPhAddr      gaddr;	/* Only used when reading in copy mode */
    unsigned long vaddr;
    unsigned int  size;		/* Only used when reading in copy mode */
} vbd_pending_seg_t;

struct vbd_pending_req_t {
    vbd_link_t*		bl;
    vbd2_req_header_t*	req;
    vbd2_op_t		op;	/* Trusted backup of original request */
    int			error;
    nku32_f		pendcount;
    struct list_head	list;
    nku32_f		nsegs;
    vbd_pending_seg_t	segs [VBD_LINK_MAX_SEGS_PER_REQ];
    vbd2_sector_t	vsector;
};

    static inline void
vbd_pending_req_init (vbd_pending_req_t* xreq, vbd_link_t* bl,
		      vbd2_req_header_t* req)
{
    xreq->bl        = bl;
    xreq->req       = req;
    xreq->op        = req->op;	/* In case frontend maliciously changes it */
    xreq->error     = FALSE;
    xreq->pendcount = 1;
	/* xreq->list does not require init */
    xreq->nsegs     = 0;
	/* xreq->segs[] does not require init */
    xreq->vsector   = req->sector;
}

    /* Only from vbd_link_init() <- vbd_init() */
#define vbd_be_get(_be)	\
    do { \
	atomic_inc (&(_be)->refcount); \
	DTRACE ("be.refcount now %d\n", (_be)->refcount.counter); \
    } while (0)

    /* Only used by vbd_link_release() <- vbd_link_put() <- many */
#define vbd_be_put(_be) \
    do { \
	if (atomic_dec_and_test (&(_be)->refcount)) \
	    vbd_be_release (_be); \
    } while (0)

#define vbd_link_get(_link)	atomic_inc (&(_link)->refcount)
#define vbd_link_put(_link) \
    do { \
	if (atomic_dec_and_test (&(_link)->refcount)) \
	    vbd_link_release (_link); \
    } while (0)

    /*
     * When drivers are built-in, Linux errors if __exit is
     * defined on functions which are called by normal code.
     */
#ifdef MODULE
#define VBD_EXIT __exit
#else
#define VBD_EXIT
#endif

#define VBD_ARRAY_ELEMS(a)	(sizeof (a) / sizeof (a) [0])

/*----- Fast map -----*/

#ifdef CONFIG_X86
#define	VBD_BIO_MAX_VECS	(BIO_MAX_PAGES/2)

struct vbd_fast_map_t {
    void*		addr;
    vbd_fast_map_t*	next;
#if defined CONFIG_X86_PAE || defined CONFIG_X86_64
    u64*		pte;
#ifdef VBD_DEBUG
    u64			inactive_pte;
    u64			active_pte;
#endif
#else
    u32*		pte;
#ifdef VBD_DEBUG
    u32			inactive_pte;
    u32			active_pte;
#endif
#endif
};

#ifdef VBD_DEBUG
    static void
vbd_fast_map_print (const char* func, const unsigned line,
		    const vbd_fast_map_t* map)
{
    TRACE ("%s:%d addr %p next %p ptep %p *ptep %llx inactive %llx "
	   "active %llx\n", func, line, map->addr, map->next, map->pte,
	   (u64) *map->pte, (u64) map->inactive_pte, (u64) map->active_pte);
}

#define VBD_FAST_MAP_CHECK(map,field) \
    if ((*(map)->pte & ~0x70) != ((map)->field & ~0x70)) { \
	vbd_fast_map_print (__func__, __LINE__, (map)); \
    }
#define VBD_FAST_MAP_SAVE(map,field,val)	(map)->field = (val)
#else
#define VBD_FAST_MAP_CHECK(map,field)		do {} while (0)
#define VBD_FAST_MAP_SAVE(map,field,val)	do {} while (0)
#endif

#if defined CONFIG_X86_64
    static u64*
vbd_x86_get_ptep (void* vaddr)
{
    u64 pte;

    __asm__ __volatile__("movq  %%cr3, %0" : "=r"(pte));
    pte = ((u64*)__va (pte & PAGE_MASK))[(((u64) vaddr) >> 39) & 0x1ff];
    pte = ((u64*)__va (pte & PAGE_MASK))[(((u64) vaddr) >> 30) & 0x1ff];
    pte = ((u64*)__va (pte & PAGE_MASK))[(((u64) vaddr) >> 21) & 0x1ff];
    return ((u64*)__va (pte & PAGE_MASK)) + ((((u64) vaddr) >> 12) & 0x1ff);
}
#elif defined CONFIG_X86_PAE
    static u64*
vbd_x86_get_ptep (void* vaddr)
{
    u32  cr3;
    u64  pte;

    __asm__ __volatile__("movl  %%cr3, %0" : "=r"(cr3));
    pte = ((u64*)__va (cr3))[(((u32) vaddr) >> 30) & 0x3];
    pte = ((u64*)__va (pte & VBD_PAGE_64_MASK))[(((u32) vaddr) >> 21) & 0x1ff];
    return ((u64*)__va (pte & VBD_PAGE_64_MASK)) + (((u32) vaddr >> 12) & 0x1ff);
}
#else
    static u32*
vbd_x86_get_ptep (void* vaddr)
{
    u32 pte;

    __asm__ __volatile__("movl  %%cr3, %0" : "=r"(pte));
    pte = ((u32*)__va (pte & PAGE_MASK))[(((u32) vaddr) >> 22) & 0x3ff];
    return ((u32*)__va (pte & PAGE_MASK)) + ((((u32) vaddr) >> 12) & 0x3ff);
}
#endif

    /*
     * Only called from vbd_link_init() <- vbd_init().
     */

    static void __init
vbd_link_fast_map_create (vbd_link_t* bl)
{
	/* 64 * 128 * 256/2 */
    int pages = (bl->xx_config.msg_count * bl->segs_per_req_max) /
	VBD_BIO_MAX_VECS;

    spin_lock_init (&bl->fast_map_lock);
    bl->fast_map = NULL;
    while (pages--) {
	void*		addr;
	vbd_fast_map_t*	map;

	addr = nkops.nk_mem_map ((unsigned int) PAGE_MASK, 1 << PAGE_SHIFT);
	if (unlikely (!addr)) {
	    ETRACE ("Fast mapping allocation failure\n");
	    return;
	}
	map = kmalloc (sizeof *map, GFP_KERNEL);
	if (unlikely (!map)) {
	    nkops.nk_mem_unmap (addr, (unsigned int) PAGE_MASK,
				1 << PAGE_SHIFT);
	    ETRACE ("Fast mapping allocation failure\n");
	    return;
	}
	map->addr = addr;
	map->pte  = vbd_x86_get_ptep (addr);
	VBD_FAST_MAP_SAVE (map, inactive_pte, *map->pte);
	map->next = bl->fast_map;
	bl->fast_map = map;
	DTRACE ("vaddr 0x%p ptep 0x%p (pte 0x%llx)\n",
		map->addr, map->pte, (u64) *map->pte);
    }
}

    static void
vbd_link_fast_map_delete (vbd_link_t* bl)
{
    while (bl->fast_map) {
	vbd_fast_map_t* map = bl->fast_map;

	bl->fast_map = map->next;
	nkops.nk_mem_unmap (map->addr, (unsigned int) PAGE_MASK,
			    1 << PAGE_SHIFT);
	kfree (map);
    }
}

    static vbd_fast_map_t*
vbd_link_fast_map_alloc (vbd_link_t* bl)
{
    unsigned long	flags;
    vbd_fast_map_t*	map;

    spin_lock_irqsave (&bl->fast_map_lock, flags);
    map = bl->fast_map;
    if (map) {
	bl->fast_map = map->next;
    }
    spin_unlock_irqrestore (&bl->fast_map_lock, flags);
    return map;
}

    static void
vbd_link_fast_map_free (vbd_link_t* bl, vbd_fast_map_t* map)
{
    unsigned long flags;

    spin_lock_irqsave (&bl->fast_map_lock, flags);
    map->next = bl->fast_map;
    bl->fast_map = map;
    spin_unlock_irqrestore (&bl->fast_map_lock, flags);
}

    static inline void
vbd_x86_flush (char* addr)
{
#if 0
    nku32_f cr3;
#endif

    __asm__ __volatile__("invlpg %0"::"m" (*addr));

#if 0
    __asm__ __volatile__(
	"movl %%cr3, %0;\n\t"
	"movl %0, %%cr3\n\t;"
    : "=r" (cr3) :: "memory");
#endif
}

#if defined CONFIG_X86_64 || defined CONFIG_X86_PAE
    static inline void
vbd_fast_map_map (vbd_fast_map_t* map, u64 paddr)
{
    preempt_disable();
    VBD_FAST_MAP_CHECK (map, inactive_pte);
    *map->pte = paddr | (1 << 8) | (1 << 1) | (1 << 0);
    VBD_FAST_MAP_SAVE (map, active_pte, paddr | (1 << 8) | (1 << 1) | (1 << 0));
    vbd_x86_flush ((char*) map->addr);
}
#else
    static inline void
vbd_fast_map_map (vbd_fast_map_t* map, u32 paddr)
{
    preempt_disable();
    VBD_FAST_MAP_CHECK (map, inactive_pte);
    *map->pte = paddr | (1 << 8) | (1 << 1) | (1 << 0);
    VBD_FAST_MAP_SAVE (map, active_pte, paddr | (1 << 8) | (1 << 1) | (1 << 0));
    vbd_x86_flush ((char*) map->addr);
}
#endif

    static inline void
vbd_fast_map_unmap (vbd_fast_map_t* map)
{
    VBD_FAST_MAP_CHECK (map, active_pte);
    *map->pte = (unsigned int) PAGE_MASK | (1 << 8) | (1 << 1) | (1 << 0);
    vbd_x86_flush ((char*) map->addr);
    VBD_FAST_MAP_CHECK (map, inactive_pte);
    preempt_enable();
}
#else	/* not CONFIG_X86 */

struct vbd_fast_map_t {
    void* addr;
};

    static inline void
vbd_link_fast_map_create (vbd_link_t* bl)
{
}

    static inline void
vbd_link_fast_map_delete (vbd_link_t* bl)
{
}

    static inline vbd_fast_map_t*
vbd_link_fast_map_alloc (vbd_link_t* bl)
{
    return NULL;
}

    static inline void
vbd_link_fast_map_free (vbd_link_t* bl, vbd_fast_map_t* map)
{
}

    static inline void
vbd_fast_map_map (vbd_fast_map_t* map, unsigned long paddr)
{
}

    static inline void
vbd_fast_map_unmap (vbd_fast_map_t* map)
{
}
#endif  /* not CONFIG_X86 */

/*----- VBD link -----*/

#define VBD_LINK(link) \
    ((vbd_link_t*) ((vmq_link_public*) (link))->priv)
#undef VBD_LINK
#define VBD_LINK(link) (*(vbd_link_t**) (link))

typedef struct {
    NkOsId	osid;
    vmq_link_t*	link;
} vbd_links_find_osid_t;

    /* Only called by vbd_links_find_osid() <- vbd_be_vdisk_create() */

    static _Bool __init
vbd_link_match_osid (vmq_link_t* link, void* cookie)
{
    vbd_links_find_osid_t* ctx = cookie;

    if (vmq_peer_osid (link) != ctx->osid) return false;
    ctx->link = link;
    return true;
}

    /* Only called by vbd_be_vdisk_create() <- vbd_init() */

    static vbd_link_t* __init
vbd_links_find_osid (vmq_links_t* links, NkOsId osid)
{
    vbd_links_find_osid_t	ctx;

    ctx.osid = osid;
    if (!vmq_links_iterate (links, vbd_link_match_osid, &ctx)) return NULL;
    return VBD_LINK (ctx.link);
}

static void vbd_vdisk_extents_disconnect (vbd_vdisk_t* vd);

    /* Called by vbd_link_destroy() and vbd_be_vdisk_create() */

    static inline void
vbd_vdisk_free (vbd_vdisk_t* vd)
{
    kfree (vd);
}

#ifdef VBD2_ATAPI
static void VBD_EXIT vbd_atapi_vdisk_exit (vbd_vdisk_t* vd);
#endif

    /* Only called from vbd_be_destroy() <- vbd_exit() */

    static _Bool VBD_EXIT
vbd_link_destroy (vmq_link_t* link, void* cookie)
{
    vbd_link_t* bl = VBD_LINK (link);
    vbd_vdisk_t* vd;

    while ((vd = bl->vdisks) != NULL) {
	bl->vdisks = vd->next;
	vbd_vdisk_extents_disconnect (vd);
#ifdef VBD2_ATAPI
	vbd_atapi_vdisk_exit (vd);
#endif
	vbd_vdisk_free (vd);
    }
    vbd_link_fast_map_delete (bl);
    kfree (bl);
    return false;
}

    /* Called by vbd_be_put() and vbd_link_release() */

    static void
vbd_be_release (vbd_be_t* be)
{
    DTRACE ("entered\n");
	/* Awake the thread */
    wake_up (&be->blkio_wait);
}

    /* Only used by vbd_link_put() macro */

    static void
vbd_link_release (vbd_link_t* bl)
{
    vbd_be_put (bl->be);
}

    /*
     *  This routine is called both from the cross-interrupt through
     *  which requests are received from the frontend, in case of
     *  synchronous requests which can be answered immediately,
     *  like probes, and from hardware disk interrupt, in case of
     *  asynchronous requests which really went to disk. These two
     *  interrupts can nest.
     */

    static void
vbd_link_resp (vbd_link_t* bl, vbd2_req_header_t* req, vbd2_status_t status)
{
    vbd2_resp_t* resp = (vbd2_resp_t*) req;

    resp->count = status;
    vmq_msg_return (bl->link, resp);
    ++bl->msg_replies;
}

static void vbd_vdisk_probe (const vbd_vdisk_t* vd, vbd2_probe_t* probe);

    static vbd_vdisk_t*
vbd_link_vdisk_lookup (vbd_link_t* bl, const vbd2_req_header_t* req)
{
    vbd_vdisk_t* vd;

    VBD_LINK_FOR_ALL_VDISKS (vd, bl) {
	if (vd->devid == req->devid && vd->genid == req->genid) {
	    return vd;
	}
    }
    ETRACE ("(%d,%d,%d:%d) virtual disk not found\n",
	    vmq_peer_osid (bl->link), VBD2_DEVID_MAJOR (req->devid),
	    VBD2_DEVID_MINOR (req->devid), req->genid);
    return NULL;
}

    static vbd_prop_extent_t*
vbd_be_prop_extent_lookup (vbd_be_t* be, nku32_f tag)
{
    int i;

    for (i = 0; i < VBD_ARRAY_ELEMS (be->prop_extents); ++i) {
	if (be->prop_extents [i].tag == tag) {
	    return be->prop_extents + i;
	}
    }
    return NULL;
}

static _Bool vbd_vdisk_extent_create (vbd_vdisk_t*, vbd_prop_extent_t*);

    /* Operations called by vbd_link_probe() for each requests */
typedef vbd2_status_t (*vbd_probe_op_t)(vbd_link_t* bl,
					const vbd2_req_header_t* req,
					vbd2_probe_t* probe, NkPhSize psize);

    /* Initial probing of vdisks */

    static vbd2_status_t
vbd_link_op_probe (vbd_link_t* bl, const vbd2_req_header_t* req,
		   vbd2_probe_t* probe, NkPhSize psize)
{
    vbd_vdisk_t*	vd;
    vbd2_status_t	status = 0;
    unsigned		skip = req->sector;

    if (!req->sector) {		/* Beginning of probing */
	bl->changes = bl->be->changes;
	bl->changes_signaled = false;
    }
    VBD_LINK_FOR_ALL_VDISKS (vd, bl) {
	if (psize < sizeof (vbd2_probe_t)) break;
	    /*
	     * If creating extents has failed at module insertion,
	     * then try again at probe time.
	     */
	if (!vd->extents) {
	    vbd_vdisk_extent_create (vd, vbd_be_prop_extent_lookup (bl->be,
				     vd->tag));
	}
	switch (vd->wait) {
	case VBD_VDISK_WAIT_NO:
	    break;

	case VBD_VDISK_WAIT_YES:
	    if (!vd->extents) continue;
	    break;

	case VBD_VDISK_WAIT_NON_ZERO:
	    if (!vd->extents) continue;
	    if (!vd->extents->sectors) continue;
	    break;
	}
	if (skip > 0) {
	    --skip;
	    continue;
	}
	vbd_vdisk_probe (vd, probe);
	psize -= sizeof (vbd2_probe_t);
	probe++;
	status++;
    }
    return status;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#define VBD_GENDISK(bdev)		(bdev)->bd_disk
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
#define VBD_BDEV_CLOSE(bdev,mode)	blkdev_put (bdev, mode)
#else
#define VBD_BDEV_CLOSE(bdev,mode)	blkdev_put (bdev)
#endif
#else
#define VBD_GENDISK(bdev)		get_gendisk ((bdev)->bd_inode->i_rdev)
#define VBD_BDEV_CLOSE(bdev,mode)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
typedef int (*vbd_fops_ioctl_t) (struct block_device*, fmode_t, unsigned,
				 unsigned long);
#define VBD_GET_IOCTL(fops)						\
  ((fops)->locked_ioctl ? (fops)->locked_ioctl :			\
   ((fops)->ioctl ? (fops)->ioctl : (fops)->compat_ioctl))
#define VBD_DO_IOCTL(ioctl,bdev,mode,cmd,arg) ioctl (bdev, mode, cmd, arg)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17)
    /*
     *  This code is suitable for versions from 2.6.27 down to 2.6.17
     *  at least, but might be required for older releases as well.
     */
typedef int (*vbd_fops_ioctl_t) (struct inode*, struct file*, unsigned,
				 unsigned long);
#define VBD_GET_IOCTL(fops)	fops->ioctl
#define VBD_DO_IOCTL(ioctl,bdev,mode,cmd,arg) ioctl (bdev->bd_inode, \
					      (struct file*) NULL, cmd, arg)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
typedef int (*vbd_fops_ioctl_t) (struct inode*, fmode_t, unsigned,
				 unsigned long);
#define VBD_GET_IOCTL(fops)	fops->ioctl
#define VBD_DO_IOCTL(ioctl,bdev,mode,cmd,arg) \
		ioctl (bdev->bd_inode, mode, cmd, arg)
#else
typedef int (*vbd_fops_ioctl_t) (struct inode*, struct file*, unsigned,
				 unsigned long);
#define VBD_GET_IOCTL(fops)	fops->ioctl
#define VBD_DO_IOCTL(ioctl,bdev,mode,cmd,arg) \
		ioctl (bdev->bd_inode, mode, cmd, arg)
#endif

    /*
     * Removable media probing (called periodically by the front-end)
     * Use for virtual ATAPI - LOOP device only
     */

    static vbd2_status_t
vbd_link_op_media_probe (vbd_link_t* bl, const vbd2_req_header_t* req,
			 vbd2_probe_t* probe, NkPhSize psize)
{
    vbd_vdisk_t*				vd;
    vbd_extent_t*				ex;
    struct gendisk*				gdisk;
    const struct block_device_operations*	fops;

    probe->devid = req->devid;
    probe->genid = req->genid;
    vd = vbd_link_vdisk_lookup (bl, req);
    if (!vd) return VBD2_STATUS_ERROR;	/* Message already issued */
    if (psize < sizeof (vbd2_probe_t)) {
	EFTRACE ("invalid request\n");
	return VBD2_STATUS_ERROR;
    }
    if (!vd->extents) {
	if (!vbd_vdisk_extent_create (vd,
		vbd_be_prop_extent_lookup (bl->be, vd->tag))) {
	    DTRACE ("no extent\n");
	    return VBD2_STATUS_ERROR;
	}
    }
    ex = vd->extents;
    if (!ex || ex->next) {
	EFTRACE ("exactly 1 extent expected for removable media\n");
	return VBD2_STATUS_ERROR;
    }
    if (!(gdisk = VBD_GENDISK (ex->bdev))) {
	EFTRACE ("no gendisk\n");
	return VBD2_STATUS_ERROR;
    }
    if (!(fops = gdisk->fops)) {
	EFTRACE ("no fops\n");
	return VBD2_STATUS_ERROR;
    }
    vbd_vdisk_probe (vd, probe);
    if (!probe->sectors) {
	DTRACE ("zero size - disconnecting\n");
	vbd_vdisk_extents_disconnect (vd);
	return VBD2_STATUS_ERROR;
    }
    return VBD2_STATUS_OK;
}

    /*
     * Removable media Load/Eject.
     * Use for virtual ATAPI - LOOP device only.
     */

    static vbd2_status_t
vbd_link_op_media_control (vbd_link_t* bl, const vbd2_req_header_t* req,
			   vbd2_probe_t* probe, NkPhSize psize)
{
    vbd_vdisk_t*				vd;
    vbd_extent_t*				ex;
    struct gendisk*				gdisk;
    const struct block_device_operations*	fops;
    vbd2_sector_t				flags;
    vbd_fops_ioctl_t				ioctl;
    int						res;

    probe->devid = req->devid;
    probe->genid = req->genid;
    vd = vbd_link_vdisk_lookup (bl, req);
    if (!vd) return VBD2_STATUS_ERROR;	/* Message already issued */
    if (psize < sizeof (vbd2_probe_t)) {
	EFTRACE ("invalid request\n");
	return VBD2_STATUS_ERROR;
    }
    ex = vd->extents;
    if (!ex || ex->next) {
	ETRACE ("exactly 1 extent expected for removable media\n");
	return VBD2_STATUS_ERROR;
    }
    if (!(gdisk = VBD_GENDISK (ex->bdev))) {
	EFTRACE ("no gendisk\n");
	return VBD2_STATUS_ERROR;
    }
    if (!(fops = gdisk->fops)) {
	EFTRACE ("no fops\n");
	return VBD2_STATUS_ERROR;
    }
    if (!(ioctl = VBD_GET_IOCTL (fops))) {
	EFTRACE ("no ioctl fops\n");
	return VBD2_STATUS_ERROR;
    }
    flags = req->sector;
    DTRACE ("flags=0x%llx\n", flags);

	/* Start always succeeds with LOOP device */
    if (!(flags & VBD2_FLAG_START)) {
	    /* Stop always succeeds with LOOP device */
	if (flags & VBD2_FLAG_LOEJ) {
		/* Eject */
	    DTRACE ("eject - disconnecting\n");
	    lock_kernel();
		/*
		 * Detach the loop device.
		 * -> will require the user to re-attach (new media)
		 */
	    res = VBD_DO_IOCTL (ioctl, ex->bdev, 0, LOOP_CLR_FD, 0);
	    vbd_vdisk_extents_disconnect (vd);
	    unlock_kernel();
	    if (res) {
		EFTRACE ("eject failed (%d)\n", res);
		probe->info |= VBD2_FLAG_LOCKED;
		return VBD2_STATUS_ERROR;
	    }
	}
    }
    return VBD2_STATUS_OK;
}

    /*
     * Removable media lock/unlock.
     * Use for virtual ATAPI - LOOP device only.
     */

    static vbd2_status_t
vbd_link_op_media_lock (vbd_link_t* bl, const vbd2_req_header_t* req,
			vbd2_probe_t* probe, NkPhSize psize)
{
    return VBD2_STATUS_OK;	/* Always succeed with LOOP device */
}

#ifdef VBD2_ATAPI
    /* Removable media - generic PACKET commmand (ATAPI) */

    static _Bool
vbd_vdisk_is_atapi (vbd_vdisk_t* vd, int* major, int* minor)
{
    vbd_prop_extent_t* prop = vbd_be_prop_extent_lookup (vd->bl->be, vd->tag);

    if (prop && (prop->major == SCSI_CDROM_MAJOR)) {
	*major = prop->major;
	*minor = prop->minor;
	return TRUE;
    }
    return FALSE;
}

    static int
vbd_atapi_ctrl_proc_open (struct inode* inode, struct file* file)
{
    vbd_vdisk_t* vd = PDE (inode)->data;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    MOD_INC_USE_COUNT;
#endif
    vd->atapi->ctrl_open++;
    return 0;
}

    static int
vbd_atapi_ctrl_proc_release (struct inode* inode, struct file* file)
{
    vbd_vdisk_t* vd = PDE (inode)->data;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    MOD_DEC_USE_COUNT;
#endif
    vd->atapi->ctrl_open--;
    return 0;
}

    static loff_t
vbd_atapi_ctrl_proc_lseek (struct file* file, loff_t off, int whence)
{
    loff_t    new;

    switch (whence) {
    case 0:  new = off; break;
    case 1:  new = file->f_pos + off; break;
    case 2:
    default: return -EINVAL;
    }
    if (new > sizeof (vbd2_atapi_req_t)) {
	return -EINVAL;
    }
    return file->f_pos = new;
}

    static ssize_t
vbd_atapi_ctrl_proc_read (struct file* file, char* buf, size_t count,
			  loff_t* ppos)
{
    vbd_vdisk_t* vd    = PDE (file->f_dentry->d_inode)->data;
    vbd_atapi_t* atapi = vd->atapi;

    if (*ppos || count != (VBD2_ATAPI_PKT_SZ + sizeof (uint32_t))) {
	return -EINVAL;
    }
    if (wait_event_interruptible (atapi->wait, atapi->count)) {
	return -EINTR;
    }
    if (copy_to_user (buf, atapi->req.cdb, count)) {
	return -EFAULT;
    }
    *ppos += count;
    return count;
}

    /* Check if extents need connect/disconnect on */
#define TEST_UNIT_READY		0x00

    static void
vbd_atapi_vdisk_extent_check_tur (vbd_vdisk_t* vd, vbd2_atapi_req_t* areq)
{
    if (areq->status) {	/* Unit is not ready */
	if (vd->extents) {
	    DTRACE ("disconnecting - media removed "
		    "status=%d key(%x/%x/%x)\n",
		    areq->status, areq->sense.sense_key,
		    areq->sense.asc, areq->sense.ascq);
	    vbd_vdisk_extents_disconnect (vd);
	}
    } else {		/* Unit is ready */
	if (!vd->extents) {
	    DTRACE ("connecting - media present\n");
	    vbd_vdisk_extent_create (vd,
		vbd_be_prop_extent_lookup (vd->bl->be, vd->tag));
	}
    }
}

    /* Check if extents need connect/disconnect on */

#define GET_EVENT_STATUS_NOTIFICATION	0x4a
#define		GESN_OPCHANGE		1	/* Operational Change Class */
#define		GESN_MEDIA		4	/* Media Class */
#define		GESN_MASK(e)		(1 << (e))
#define		GESN_MEDIA_NEW		0x02    /* Event */
#define		GESN_MEDIA_REMOVAL	0x03    /* Event */
#define		GESN_MEDIA_PRESENT	0x02    /* Status */
#define		GESN_TRAY_OPEN		0x01    /* Status */

    static _Bool
vbd_atapi_vdisk_extent_check_gesn (vbd_vdisk_t* vd, uint8_t* reply)
{
    if ((reply [2] == GESN_MEDIA) && (reply [1] > 4)) { /* MEDIA event */
	if (reply [5] == GESN_MEDIA_PRESENT) {	/* Status */
	    if (!vd->extents) {
		    /*
		     * HACK for Windows:
		     * The backend driver may have "consumed" the NewMedia
		     * event issued immediately after inserting the media.
		     * We just inject one to be sure the frontend will have
		     * it, as some guests rely on the event and not on the
		     * status.
		     */
		reply [4] = GESN_MEDIA_NEW; /* NewMedia event */
		DTRACE ("connecting - media present\n");
		vbd_vdisk_extent_create (vd,
		    vbd_be_prop_extent_lookup (vd->bl->be, vd->tag));
	    }
	} else {
	    if (vd->extents) {
		    /*
		     * HACK for Windows:
		     * The backend driver may have "consumed" the MediaRemoval
		     * event issued immediately after ejecting the media.
		     * We just inject one to be sure the frontend will have it,
		     * as some guests rely on the event and not on the status.
		     */
		reply [4] = GESN_MEDIA_REMOVAL; /* MediaRemoval event */
		DTRACE ("disconnecting - media removed\n");
		vbd_vdisk_extents_disconnect (vd);
	    }
	}
	return FALSE;
    }
    if ((reply [2] == GESN_OPCHANGE) && (reply [1] >= 6) && /* OPC event */
	(reply [4] == 0x02) && (reply [7] == 0x01)) {	  /* Feature change */
	if (!vd->extents) {
	    DTRACE ("(lost?) MEDIA event needed\n");
	    return TRUE; /* Notify user process that media event is needed */
	}
    }
    return FALSE;
}

    static ssize_t
vbd_atapi_ctrl_proc_write (struct file* file, const char* buf, size_t size,
			   loff_t* ppos)
{
    vbd_vdisk_t*	vd = PDE (file->f_dentry->d_inode)->data;
    vbd_link_t*		bl = vd->bl;
    vbd_atapi_t*	atapi = vd->atapi;
    vbd2_status_t	status;
    ssize_t		res;
    NkPhAddr		paddr;
    vbd2_atapi_req_t*	areq;
    vbd_fast_map_t*	map;

    if (!atapi->count ||
	(*ppos != (uint32_t)&(((vbd2_atapi_req_t*)0)->status)) ||
	(size  != (sizeof (uint32_t) + sizeof (vbd2_atapi_sense_t)))) {
	return -EINVAL;
    }
    status = VBD2_STATUS_OK;
    res    = size;
    paddr  = atapi->req.paddr; /* ATAPI request is page aligned */
    *ppos  = 0;

    if ((map = vbd_link_fast_map_alloc (bl)) != NULL) {
	vbd_fast_map_map (map, paddr);	/* Preemption is disabled */
	++bl->fast_map_probes;
	areq = (vbd2_atapi_req_t*) map->addr;
	if (copy_from_user (&areq->status, buf, size)) {
	    res    = -EFAULT;
	    status = VBD2_STATUS_ERROR;
	} else if (atapi->req.cdb [0] == TEST_UNIT_READY) {
		/* Check if extent needs to be connected/disconnected */
	    vbd_atapi_vdisk_extent_check_tur (vd, areq);
	}
	vbd_fast_map_unmap (map);		/* Preemption is enabled */
	vbd_link_fast_map_free (bl, map);
    } else {
	areq = (vbd2_atapi_req_t*) nkops.nk_mem_map (paddr, size);
	++bl->nk_mem_map_probes;
	if (!areq) {
	    ETRACE ("nk_mem_map(0x%llx, 0x%llx) failure\n", (long long) paddr,
		    (long long) size);
	    atapi->count = 0; /* Release process request */
	    vbd_link_resp (bl, atapi->req.req, VBD2_STATUS_ERROR);
	    return -EFAULT;
	}
	if (copy_from_user (&areq->status, buf, size)) {
	    res    = -EFAULT;
	    status = VBD2_STATUS_ERROR;
	} else if (atapi->req.cdb [0] == TEST_UNIT_READY) {
		/* Check if extent needs to be connected/disconnected */
	    vbd_atapi_vdisk_extent_check_tur (vd, areq);
	}
	nkops.nk_mem_unmap (areq, paddr, size);
    }
    atapi->count = 0;  /* Release process request */
	/* Reply to front-end */
    vbd_link_resp (bl, atapi->req.req, status);
    return res;
}

    static int
vbd_atapi_data_proc_open (struct inode* inode, struct file* file)
{
    vbd_vdisk_t*	vd = PDE (inode)->data;
    vbd_atapi_t*	atapi = vd->atapi;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    MOD_INC_USE_COUNT;
#endif
    atapi->data_open++;
    return 0;
}

    static int
vbd_atapi_data_proc_release (struct inode* inode, struct file* file)
{
    vbd_vdisk_t*	vd = PDE (inode)->data;
    vbd_atapi_t*	atapi = vd->atapi;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    MOD_DEC_USE_COUNT;
#endif
    atapi->data_open--;
    return 0;
}

    static loff_t
vbd_atapi_data_proc_lseek (struct file* file, loff_t off, int whence)
{
    loff_t new;

    switch (whence) {
    case 0:  new = off; break;
    case 1:  new = file->f_pos + off; break;
    case 2:
    default: return -EINVAL;
    }
    if (new > VBD2_ATAPI_REP_SZ) {
	return -EINVAL;
    }
    return (file->f_pos = new);
}

    static ssize_t
vbd_atapi_data_proc_write (struct file* file, const char* buf, size_t size,
			   loff_t* ppos)
{
    vbd_vdisk_t*	vd = PDE (file->f_dentry->d_inode)->data;
    vbd_link_t*		bl = vd->bl;
    vbd_atapi_t*	atapi = vd->atapi;
    ssize_t		res;
    NkPhAddr		paddr;
    void*		vaddr;
    vbd_fast_map_t*	map;

    if (!atapi->count || (*ppos + size > VBD2_ATAPI_REP_SZ)) {
	return -EINVAL;
    }
    if (!size) {
	return 0;
    }
    res    = size;
    paddr  = atapi->req.paddr + 0x1000; /* ATAPI reply is on next page */

    if ((map = vbd_link_fast_map_alloc (bl)) != NULL) {
	char* src = (char*) buf;

	while (size) {
	    size_t csize = (~VBD_PAGE_64_MASK) + 1; /* Page size */
	    _Bool  first = TRUE;

	    vbd_fast_map_map (map, paddr);	/* Preemption is disabled */
	    ++bl->fast_map_probes;
	    vaddr = map->addr;
	    if (size < csize) {
		csize = size;
	    }
	    if (copy_from_user (vaddr, src, csize)) {
		res = -EFAULT;
		vbd_fast_map_unmap (map);	/* Preemption is enabled */
		break;
	    }
	    if (first &&
		(atapi->req.cdb [0] == GET_EVENT_STATUS_NOTIFICATION)) {
		    /* Check if extent needs to be connected/disconnected */
		if (vbd_atapi_vdisk_extent_check_gesn (vd, vaddr)) {
		    res = -EAGAIN; /* Media event needed */
		}
	    }
	    first  = FALSE;
	    vbd_fast_map_unmap (map);	/* Preemption is enabled */
	    size  -= csize;
	    src   += csize;
	    paddr += csize;
	}
	vbd_link_fast_map_free (bl, map);
    } else {
	vaddr = nkops.nk_mem_map (paddr, size);
	++bl->nk_mem_map_probes;
	if (!vaddr) {
	    ETRACE ("nk_mem_map(0x%llx, 0x%llx) failure\n",
		    (long long) paddr, (long long) size);
	    return -EFAULT;
	}
	if (copy_from_user (vaddr, buf, size)) {
	    res = -EFAULT;
	} else if (atapi->req.cdb [0] == GET_EVENT_STATUS_NOTIFICATION) {
		/* Check if extent needs to be connected/disconnected */
	    if (vbd_atapi_vdisk_extent_check_gesn (vd, vaddr)) {
		res = -EAGAIN; /* Media event needed */
	    }
	}
	nkops.nk_mem_unmap (vaddr, paddr, size);
    }
    return res;
}

    static ssize_t
vbd_atapi_data_proc_read (struct file* file, char* buf, size_t count,
			  loff_t* ppos)
{
    return -EPERM;
}

    static _Bool
vbd_link_atapi (vbd_link_t* bl, vbd2_req_header_t* req)
{
    vbd_vdisk_t*	vd;
    vbd_atapi_t*	atapi;
    vbd2_atapi_req_t*	areq;
    NkPhAddr		paddr;
    vbd_fast_map_t*	map;

    if (req->count != 1) {
	vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	EFTRACE ("Invalid request count\n");
	return TRUE;
    }
    vd = vbd_link_vdisk_lookup (bl, req);
    if (!vd) {
	    /* Message already issued */
	vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	return TRUE;
    }
    if (!(atapi = vd->atapi)) {
	EFTRACE ("%s not an ATAPI device\n", vd->name);
	vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	return TRUE;
    }
	/* Check if user process is present */
    if (!atapi->ctrl_open || !atapi->data_open) {
	EFTRACE ("User process not present\n");
	vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	return TRUE;
    }
	/* Only one request at a time */
    mutex_lock (&atapi->lock);
    if (atapi->count) {
	EFTRACE ("too many requests\n");
	vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	mutex_unlock (&atapi->lock);
	return TRUE;
    }
    atapi->count = 1;	/* Get the process request */
    mutex_unlock (&atapi->lock);

	/* Map the front-end's ATAPI request */
    paddr = *VBD2_FIRST_BUF (req);
    paddr = VBD2_BUF_BASE (paddr);
    if ((map = vbd_link_fast_map_alloc (bl)) != NULL) {
	unsigned int off = (paddr & ~VBD_PAGE_64_MASK);
	vbd_fast_map_map (map, paddr);
	    /* Now preemption is disabled */
	areq = (vbd2_atapi_req_t*) ((unsigned long) map->addr + off);
	++bl->fast_map_probes;
    } else {
	areq = (vbd2_atapi_req_t*) nkops.nk_mem_map (paddr,
						     sizeof (vbd2_atapi_req_t));
	++bl->nk_mem_map_probes;
    }
    if (!areq) {
	ETRACE ("nk_mem_map(0x%llx, 0x%llx) failure\n", (long long) paddr,
		(long long) sizeof (vbd2_atapi_req_t));
	atapi->count = 0;  /* Release the process request */
	vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	return TRUE;
    }
	/* Prepare the user process' request */
    atapi->req.req    = req;
    atapi->req.devid  = req->devid;
    atapi->req.paddr  = paddr;
    memcpy (atapi->req.cdb, areq->cdb, VBD2_ATAPI_PKT_SZ);
    atapi->req.buflen = areq->buflen;

	/* Unmap the front-end's ATAPI request */
    if (map) {
	vbd_fast_map_unmap (map);
	vbd_link_fast_map_free (bl, map);
    } else {
	nkops.nk_mem_unmap (areq, paddr, sizeof (vbd2_atapi_req_t));
    }
	/* Wake up the user process */
    wake_up_interruptible (&atapi->wait);
    return FALSE; /* No reply yet */
}
#endif	/* VBD2_ATAPI */

    static _Bool
vbd_link_probe (vbd_link_t* bl, vbd_probe_op_t probe_op, vbd2_req_header_t* req)
{
    vbd2_probe_link_t*	probe = (vbd2_probe_link_t*) req;
    vbd2_status_t	status;

	/* Call request-specific operation to fill in the vbd2_probe_t data */
    status = probe_op (bl, req, probe->probe,
    		       req->count * sizeof (vbd2_probe_t));
    vbd_link_resp (bl, req, status);
    return TRUE;
}

    /* Can set be->resource_error */
static _Bool vbd_vdisk_rw (vbd_vdisk_t* vd, vbd2_req_header_t* req);

    /*
     * Called only by vbd_link_do_blkio_op().
     * Return value of TRUE indicates that "req" has been used
     * and does not need to be re-submitted.
     * Can set be->resource_error.
     */

    static _Bool
vbd_link_rw (vbd_link_t* bl, vbd2_req_header_t* req)
{
    vbd_vdisk_t* vd = vbd_link_vdisk_lookup (bl, req);

    if (!vd) {
	    /* Message already issued */
	vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	return TRUE;
    }
    return vbd_vdisk_rw (vd, req);
}

/*----- Virtual disk -----*/

    /* Only called by vbd_be_vdisk_create() <- vbd_init() */

    static vbd_vdisk_t* __init
vbd_vdisk_alloc (void)
{
    void* vd;

    if (unlikely ((vd = kzalloc (sizeof (vbd_vdisk_t), GFP_KERNEL)) == NULL)) {
	ETRACE ("out of memory for vdisk\n");
	return NULL;
    }
    return VBD_VDISK (vd);
}

#ifdef VBD2_ATAPI
static _Bool __init vbd_atapi_vdisk_init (vbd_vdisk_t* vd);
#endif

static void vbd_be_signal_changes (vbd_be_t* be);

    /* Called from vbd_init() only */

    static int __init
vbd_be_vdisk_create (vbd_be_t* be, const vbd_prop_vdisk_t* prop)
{
    vbd_link_t*		bl;
    vbd_vdisk_t*	vd;
    vbd_vdisk_t**	link;

    bl = vbd_links_find_osid (be->links, prop->owner);
    if (!bl) {
	ETRACE ("no link to %d found\n", prop->owner);
	return -ESRCH;
    }
    VBD_LINK_FOR_ALL_VDISKS (vd, bl) {
	if (vd->devid == prop->devid) {
		/* Take the strongest wait mode among vbd2= parameters */
	    if (prop->wait > vd->wait) {
		vd->wait = prop->wait;
	    }
	    return 0;		/* Vdisk already exists */
	}
    }
    vd = vbd_vdisk_alloc();
    if (!vd) return -ENOMEM;	/* Message already issued */
    vbd_vdisk_init (vd, prop->tag, prop->devid, bl, prop->wait);

#ifdef VBD2_ATAPI
    if (!vbd_atapi_vdisk_init (vd)) {
	vbd_vdisk_free (vd);
	ETRACE ("%s - ATAPI initialization failure\n", vd->name);
	return -EINVAL;
    }
#endif
    link = &bl->vdisks;
    while (*link) link = &(*link)->next;
    *link = vd;

    TRACE ("%s created", vd->name);
#ifdef VBD2_ATAPI
    if (vd->atapi) {
	printk (" - ATAPI ctrl: %s data: %s",
		vd->atapi->ctrl_name, vd->atapi->data_name);
    }
#endif
    printk ("\n");
	/* Do not signal the backend as the vdisk has no extents yet */
    return 0;
}

    /*
     *  On 2.6.x we can have zero-sized peripherals,
     *  typically initially empty loopback devices. In
     *  this case, we try to acquire the size at every
     *  vbd_vdisk_probe() and vbd_vdisk_translate(),
     *  so that the guest is informed about the most
     *  current value and in order not to refuse an
     *  I/O request which is actually valid now.
     */

    static inline void
vbd_extent_maybe_acquire_size (vbd_extent_t* ex)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    if (!ex->sectors) {
	if (ex->bdev->bd_part) {
	    ex->sectors = ex->bdev->bd_part->nr_sects;
	} else {
	    ex->sectors = get_capacity (ex->bdev->bd_disk);
	}
    }
#else
    (void) ex;
#endif
}

    static vbd_sector_t
vbd_vdisk_sectors (const vbd_vdisk_t* vd)
{
    vbd_sector_t	sectors = 0;
    vbd_extent_t*	ex;

    VBD_VDISK_FOR_ALL_EXTENTS (ex, vd) {
	vbd_extent_maybe_acquire_size (ex);
	sectors += ex->sectors;
    }
    return sectors;
}

    static _Bool
vbd_vdisk_readonly (const vbd_vdisk_t* vd)
{
    const vbd_extent_t* ex;

    VBD_VDISK_FOR_ALL_EXTENTS (ex, vd) {
	if (ex->access & VBD_DISK_ACC_W) {
	    return FALSE;
	}
    }
    return TRUE;
}

    static void
vbd_vdisk_probe (const vbd_vdisk_t* vd, vbd2_probe_t* probe)
{
    vbd_prop_extent_t* prop = vbd_be_prop_extent_lookup (vd->bl->be, vd->tag);

    probe->sectors = vbd_vdisk_sectors (vd);
    probe->devid   = vd->devid;
    probe->genid   = vd->genid;
    if (prop && (prop->major == FLOPPY_MAJOR)) {
	    /* Vdisk is a floppy */
	probe->info = VBD2_TYPE_FLOPPY;
    } else {
	    /* Vdisk is a hard disk */
	probe->info = VBD2_TYPE_DISK;
    }
    if (prop && (prop->major == LOOP_MAJOR)) {
	    /* Vdisk is connected to a loop device */
	probe->info |= VBD2_FLAG_VIRT;
    }
    if (vbd_vdisk_readonly (vd)) {
	probe->info |= VBD2_FLAG_RO;
    }
    XTRACE ("%s, %lld sectors, flags 0x%x\n", vd->name,
	    (long long) probe->sectors, probe->info);
}

    static vbd_extent_t*
vbd_vdisk_translate (vbd_vdisk_t* vd, nku32_f acc, vbd2_sector_t vsec,
		     vbd_sector_t* psec)
{
    vbd_extent_t* ex;

    VBD_VDISK_FOR_ALL_EXTENTS (ex, vd) {
	vbd_extent_maybe_acquire_size (ex);
	if (vsec < ex->sectors) {
	    break;
	}
	vsec -= ex->sectors;
    }
    if (!ex) {
	DTRACE ("%s extent not found (%s extents)\n", vd->name,
		vd->extents ? "but do have" : "no");
	return NULL;
    }
    if (!(ex->access & acc)) {
	return NULL;
    }
    *psec = (ex->start + vsec);
    return ex;
}

/*----- Extent management -----*/

    /* Only called from vbd_be_extent_create() <- vbd_extent_thread() */

    static _Bool
vbd_extent_connect (vbd_extent_t* ex)
{
    vbd_sector_t sectors;

	/* Force the extent to start at the beginning of the block device */
    ex->start = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    ex->bdev = open_by_devnum (ex->dev,
			       ex->access == VBD_DISK_ACC_R ? FMODE_READ
							    : FMODE_WRITE);
    if (IS_ERR (ex->bdev)) {
	DTRACE ("device (%d,%d) does not exist.\n",
		ex->prop->major, ex->prop->minor);
	return FALSE;
    }
    if (!ex->bdev->bd_disk) {
	DTRACE ("device (%d,%d) does not exist 2.\n",
		ex->prop->major, ex->prop->minor);
	VBD_BDEV_CLOSE (ex->bdev,
			ex->access == VBD_DISK_ACC_R ? FMODE_READ
						     : FMODE_WRITE);
	return FALSE;
    }
	/* Get block device size in sectors (512 bytes) */
    if (ex->bdev->bd_part) {
	sectors = ex->bdev->bd_part->nr_sects;
    } else {
	sectors = get_capacity (ex->bdev->bd_disk);
    }
    if (!sectors && ex->vdisk->wait == VBD_VDISK_WAIT_NON_ZERO) {
	DTRACE ("device (%d,%d) size is 0.\n",
		ex->prop->major, ex->prop->minor);
	VBD_BDEV_CLOSE (ex->bdev, ex->access ==  VBD_DISK_ACC_R ? FMODE_READ
								: FMODE_WRITE);
	return FALSE;
    }
	/*
	 * Work-around strange kernel behavior where the
	 * block device bd_inode->i_size is not updated when inserting
	 * a media (maybe due to using: open_by_devnum()?).
	 * This leads to common block device code to fail in
	 * checking submitted bio(s) accessing sectors beyond 0x1fffff,
	 * on /dev/sr0 (sr initializes bd_inode->i_size with that value).
	 */
    if (sectors != (i_size_read (ex->bdev->bd_inode) >> 9)) {
	    /* Update bdev inode size to make blk-core.c happy */
	DTRACE ("device (%d,%d): hacking bdev inode size (%llu -> %llu).\n",
		ex->prop->major, ex->prop->minor,
		i_size_read (ex->bdev->bd_inode), (loff_t) sectors << 9);
	i_size_write (ex->bdev->bd_inode, (loff_t) sectors << 9);
    }
#else
    if (!blk_size [MAJOR (ex->dev)]) {
	DTRACE ("device (%d,%d) does not exist.\n",
		ex->prop->major, ex->prop->minor);
	return FALSE;
    }
	/* Convert blocks (1KB) to sectors */
    sectors = blk_size [MAJOR (ex->dev)] [MINOR (ex->dev)] * 2;
    if (!sectors) {
	DTRACE ("device (%d,%d) does not exist.\n",
		ex->prop->major, ex->prop->minor);
	return FALSE;
    }
#endif
    DTRACE ("extent size requested %llu actual %llu sectors, access %i\n",
	    (unsigned long long) ex->sectors, (long long) sectors, ex->access);
	/* NB: this test assumes ex->start == 0 */
    if (!ex->sectors || (ex->sectors > sectors)) {
	ex->sectors = sectors;
    }
    return TRUE;
}

typedef struct {
    nku32_f		tag;	/* Vdisk tag */
    vbd_vdisk_t*	vd;
} vbd_link_find_tag_t;

    /* Called by vbd_be_lookup_vd_by_prop() only */

    static _Bool
vbd_link_match_tag (vmq_link_t* link, void* cookie)
{
    vbd_link_find_tag_t*	ctx = cookie;
    vbd_vdisk_t*		vd;

    VBD_LINK_FOR_ALL_VDISKS (vd, VBD_LINK (link)) {
	if (vd->tag == ctx->tag) {
	    ctx->vd = vd;
	    return true;
	}
    }
    return false;
}

    /*
     * Only called from vbd_be_extent_create() <- vbd_extent_thread().
     */

    static vbd_vdisk_t*
vbd_be_lookup_vd_by_prop (vbd_be_t* be, const vbd_prop_extent_t* prop)
{
    vbd_link_find_tag_t	ctx;

    ctx.tag = prop->tag;
    if (!vmq_links_iterate (be->links, vbd_link_match_tag, &ctx)) return NULL;
    return ctx.vd;
}

    /*
     * We consider that a vdisk is composed of only one extent.
     * Return TRUE if ok, FALSE on error.
     * Called from vbd_be_extent_create() and from vbd_link_op_media_probe().
     */

    static _Bool
vbd_vdisk_extent_create (vbd_vdisk_t* vd, vbd_prop_extent_t* prop)
{
    vbd_extent_t* ex;

    if (!prop || !vd) {
	return FALSE;
    }
    ex = VBD_EXTENT (kzalloc (sizeof (vbd_extent_t), GFP_KERNEL));
    if (!ex) {
	ETRACE ("extent allocation failure\n");
	vd->extents_ok = FALSE;
	return FALSE;
    }
    vbd_extent_init (ex, 0, prop->sectors, MKDEV (prop->major, prop->minor),
		     prop->access, prop, vd);

    if (vbd_extent_connect (ex)) {
	vbd_extent_t** last = &vd->extents;

	while (*last) last = &(*last)->next;
	*last = ex;
	prop->bound = TRUE;	/* extent now bound to real device */
	DTRACE ("Found %s, %lld sectors.\n", vd->name, (long long) ex->sectors);
    } else {
	kfree (ex);
	DTRACE ("connect failed for guest %d\n", vmq_peer_osid (vd->bl->link));
	vd->extents_ok = FALSE;
	return FALSE;
    }
    return TRUE;
}

    /* Called from vbd_extent_thread() only */

    static _Bool
vbd_be_extent_create (vbd_be_t* be, vbd_prop_extent_t* prop)
{
    vbd_vdisk_t* vd;

    vd = vbd_be_lookup_vd_by_prop (be, prop);
    if (!vd) return FALSE;
    return vbd_vdisk_extent_create (vd, prop);
}

    /* Called from vbd_be_maybe_vdisks_ok() only */

    static _Bool
vbd_link_maybe_vdisks_ok (vmq_link_t* link, void* cookie)
{
    _Bool			vdisks_ok = TRUE;
    vbd_link_t*			bl = VBD_LINK (link);
    vbd_vdisk_t*		vd;

    if (bl->vdisks_ok) return false;
    VBD_LINK_FOR_ALL_VDISKS (vd, bl) {
	if (!vd->extents_ok && vd->wait != VBD_VDISK_WAIT_NO) {
	    vdisks_ok = FALSE;
	    break;
	}
    }
    if (vdisks_ok) {
	DTRACE ("vdisks now OK for guest %d\n", vmq_peer_osid (bl->link));
	bl->vdisks_ok = TRUE;
	    /* Notify the frontend */
	vbd_be_signal_changes (bl->be);
    } else {
	*(int*) cookie = FALSE;
    }
    return false;
}

    /*
     * Called from vbd_extent_thread() only.
     * Return TRUE if we have all extents/vdisks now.
     */

    static _Bool
vbd_be_maybe_vdisks_ok (vbd_be_t* be)
{
    int blkifs_ok = TRUE;

    vmq_links_iterate (be->links, vbd_link_maybe_vdisks_ok, &blkifs_ok);
    return blkifs_ok;
}

    /*
     * Called from vbd_link_destroy() <- from vbd_be_destroy() <- vbd_exit().
     * Also called from vbd_link_op_media_probe().
     */

    static void
vbd_vdisk_extents_disconnect (vbd_vdisk_t* vd)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    vbd_extent_t* ex = vd->extents;

    while (ex) {
	vbd_extent_t* prev;

	VBD_BDEV_CLOSE (ex->bdev, FMODE_READ | FMODE_WRITE);
	prev = ex;
	ex   = ex->next;
	kfree (prev);
    }
    vd->extents = NULL;
#endif
}

    static _Bool
vbd_link_mark_vdisks (vmq_link_t* link, void* cookie)
{
    vbd_link_t*		bl = VBD_LINK (link);
    vbd_vdisk_t*	vd;

    if (bl->vdisks_ok) return false;
    VBD_LINK_FOR_ALL_VDISKS (vd, bl) {
	vd->extents_ok = TRUE;
    }
    return false;
}

    /*
     *  The extents list for a vbd_vdisk_t only contains already bound
     *  extents, so scanning it does not allow to determine if
     *  a vdisk is now complete. So we have "missing extents" notion
     *  for vbd_vdisk_ts which is updated as the vbd_be.extents[] is scanned.
     *  Before scanning, all vdisks are temporarily set to "extents_ok"
     *  state, which is removed if some extent is missing.
     */
    static inline void
vbd_be_mark_vdisks (vbd_be_t* be)
{
    vmq_links_iterate (be->links, vbd_link_mark_vdisks, NULL);
}

    static int
vbd_extent_thread (void* arg)
{
    vbd_be_t* be = arg;

	/* Analyze virtual disk extents */
    while (!be->extent_abort) {
	const vbd_prop_extent_t* pe_max =
	    be->prop_extents + VBD_ARRAY_ELEMS (be->prop_extents);
	vbd_prop_extent_t* pe;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
	try_to_freeze();
#endif

	vbd_be_mark_vdisks (be);
	for (pe = be->prop_extents; pe < pe_max && pe->major; ++pe) {
	    if (pe->bound) continue;
	    vbd_be_extent_create (be, pe);
	}
	if (vbd_be_maybe_vdisks_ok (be)) {
	    DTRACE ("all extents bound, suiciding.\n");
	    break;
	}
	    /*
	     *  When waiting for an extent to appear, we could
	     *  go to sleep, waiting for the vlx_vbd_disk_created()
	     *  callback which is performed from add_disk(). But
	     *  we also sometimes wait until a disk size is non-zero,
	     *  which requires periodic polling, so we would need
	     *  to setup a sleep with timeout, and this requires
	     *  more work than it is worth. So we just poll every
	     *  half second until we have all the devices in OK state.
	     */
	set_current_state (TASK_INTERRUPTIBLE);
	schedule_timeout (HZ/2);
    }
    return 0;
}

/******************************************************************
 * BLOCK-DEVICE SCHEDULER LIST MAINTENANCE
 */

    static inline _Bool
vbd_link_on_blkio_list (vbd_link_t* bl)
{
    return bl->blkio_list.next != NULL;
}

    /* Called from vbd_blkio_thread() only */

    static void
vbd_link_remove_from_blkio_list (vbd_link_t* bl)
{
    unsigned long flags;

    if (!vbd_link_on_blkio_list (bl)) {
	return;
    }
    spin_lock_irqsave (&bl->be->blkio_list_lock, flags);
    if (vbd_link_on_blkio_list (bl)) {
	list_del (&bl->blkio_list);
	bl->blkio_list.next = NULL;
	vbd_link_put (bl);
    }
    spin_unlock_irqrestore (&bl->be->blkio_list_lock, flags);
}

    /* Called when a link requires attention */

    static void
vbd_link_add_to_blkio_list_tail (vbd_link_t* bl)
{
    unsigned long flags;

    if (vbd_link_on_blkio_list (bl)) {
	    /* Already enlisted on vbd_be.blkio_list */
	return;
    }
    spin_lock_irqsave (&bl->be->blkio_list_lock, flags);
    if (!vbd_link_on_blkio_list (bl))  {
	list_add_tail (&bl->blkio_list, &bl->be->blkio_list);
	    /* Increment refcount */
	vbd_link_get (bl);
    }
    spin_unlock_irqrestore (&bl->be->blkio_list_lock, flags);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    static vbd_pending_req_t*
vbd_link_get_from_done_list (vbd_link_t* bl)
{
    unsigned long	flags;
    vbd_pending_req_t*	req;

    spin_lock_irqsave (&bl->done_list_lock, flags);
    if (list_empty (&bl->done_list)) {
	req = 0;
    } else {
	struct list_head* head = bl->done_list.next;

	    /* Convert pointer to field into pointer to struct */
	req = list_entry (head, vbd_pending_req_t, list);
	list_del (head);
    }
    spin_unlock_irqrestore (&bl->done_list_lock, flags);
    return req;
}
#endif

    static void
vbd_be_maybe_wake_up_blkio (vbd_be_t* be)
{
    XTRACE ("entered\n");
	/*
	 * Needed so that two processes, who together make the
	 * following predicate true, don't both read stale values
	 * and evaluate the predicate incorrectly.
	 */
    smp_mb();
    if (!list_empty (&be->blkio_list)) {
	DTRACE ("wake_up\n");
	wake_up (&be->blkio_wait);
    }
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    static void
vbd_link_put_to_done_list (vbd_link_t* bl, vbd_pending_req_t* req)
{
    unsigned long	flags;
    _Bool		empty;

    spin_lock_irqsave (&bl->done_list_lock, flags);
    empty = list_empty (&bl->done_list);
    list_add_tail (&req->list, &bl->done_list);
    spin_unlock_irqrestore (&bl->done_list_lock, flags);
    if (empty) {
	vbd_link_add_to_blkio_list_tail (bl);
	vbd_be_maybe_wake_up_blkio (bl->be);
    }
}
#endif

/******************************************************************
 * SCHEDULER FUNCTIONS
 */
static int  vbd_link_do_blkio_op (vbd_link_t* bl, int max_to_do);

    static void
vbd_be_timeout (unsigned long data)
{
    vbd_be_t* be = (vbd_be_t*) data;

    ++be->timeouts;
    wake_up_process (be->blkio_task_struct);
}

    void
vbd_be_schedule_with_timeout (vbd_be_t* be)
{
    struct timer_list timer;

    init_timer (&timer);
    timer.expires  = jiffies + HZ / 10;
    timer.data     = (unsigned long) be;
    timer.function = vbd_be_timeout;

    add_timer (&timer);
    schedule();
    del_timer_sync (&timer);
}

    static int
vbd_blkio_thread (void* arg)
{
    vbd_be_t*		be = arg;
    DECLARE_WAITQUEUE	(wq, current);

    DTRACE ("starting\n");
    be->blkio_task_struct = current;
    while (atomic_read (&be->refcount)) {
	    /* Wait for work to do */
	add_wait_queue (&be->blkio_wait, &wq);
	set_current_state (TASK_INTERRUPTIBLE);

	if (be->resource_error) {
	    be->resource_error = FALSE;
	    ++be->sleeps_with_timeout;
	    vbd_be_schedule_with_timeout (be);
	} else if (atomic_read (&be->refcount) &&
		list_empty (&be->blkio_list) && !be->sysconf) {
	    ++be->sleeps_no_timeout;
	    schedule();
	}
	__set_current_state (TASK_RUNNING);
	remove_wait_queue (&be->blkio_wait, &wq);
	DTRACE ("wakeup\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
	    /* Check if freeze signal has been received */
	try_to_freeze();
#endif
	if (be->sysconf) {
	    be->sysconf = false;
	    vmq_links_sysconf (be->links);
	}
	    /* Queue up a batch of requests */
	while (!list_empty (&be->blkio_list) && !be->resource_error) {
	    struct list_head*	ent = be->blkio_list.next;
		/* Convert pointer to field into pointer to struct */
	    vbd_link_t*		bl = list_entry (ent, vbd_link_t, blkio_list);

	    vbd_link_get (bl);
	    vbd_link_remove_from_blkio_list (bl);
		/*
		 *  In case of be->resource_error, vbd_link_do_blkio_op()
		 *  returns 0 even though "bl" still requires
		 *  attention, so check for that specifically.
		 */
	    if (vbd_link_do_blkio_op (bl, VBD_LINK_MAX_REQ) ||
		be->resource_error) {
		    /*
		     *  We have processed all the VBD_LINK_MAX_REQ
		     *  quota of requests, so there can be more, so we
		     *  put back "bl" on the list, at the end/tail.
		     */
		vbd_link_add_to_blkio_list_tail (bl);
	    }
	    vbd_link_put (bl);
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	    /* Push the batch through to disk */
	run_task_queue (&tq_disk);
#endif
    }
    DTRACE ("exiting\n");
    return 0;
}

    static void
vbd_req_blkio_done (vbd_pending_req_t *req)
{
    vbd_link_t*	bl = req->bl;

    vbd_link_resp (bl, req->req,
		   req->error ? VBD2_STATUS_ERROR : VBD2_STATUS_OK);
    vbd_link_put (bl);
    kmem_cache_free (bl->be->pending_req_cachep, req);
    vbd_be_maybe_wake_up_blkio (bl->be);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)

    /* Called from vbd_bh_end_blkio_op() */

    static void
vbd_req_end_blkio_op (vbd_pending_req_t* xreq, _Bool uptodate)
{
    if (!uptodate) {
	xreq->error = TRUE;
    }
    if (!nkops.nk_sub_and_test (&xreq->pendcount, 1)) {
	vbd_req_blkio_done (xreq);
    }
}

    /*
     *  This is a callback set up in vbd_vdisk_rw(),
     *  through the "bh->b_end_io" field.
     *  (In 2.6, we have "bio->bi_end_io")
     */

    static void
vbd_bh_end_blkio_op (struct buffer_head* bh, int uptodate)
{
    vbd_pending_req_t* xreq = bh->b_private;

    vbd_req_end_blkio_op (xreq, uptodate);
    nkops.nk_mem_unmap (bh->b_data, __pa (bh->b_data), bh->b_size);
    kmem_cache_free (xreq->bl->be->buffer_head_cachep, bh);
}

#else	/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
    /*
     * Since 2.6.25 blk_get_queue and blk_put_queue are not exported
     * anymore. It seems the VBD code should be changed to get rid of
     * these functions.
     */

    static inline int
vbd_blk_get_queue (struct request_queue* q)
{
    if (likely (!test_bit (QUEUE_FLAG_DEAD, &q->queue_flags))) {
	kobject_get (&q->kobj);
	return 0;
    }
    return 1;
}

    static inline void
vbd_blk_put_queue (struct request_queue* q)
{
    kobject_put (&q->kobj);
}

#define blk_get_queue vbd_blk_get_queue
#define blk_put_queue vbd_blk_put_queue
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25) */

    static void
vbd_req_submit_bio (vbd_pending_req_t* req, struct bio* bio)
{
    struct request_queue* q;

	/*
	 *  If __get_free_page() fails in vbd2_dma=0 mode,
	 *  we can be called with bio->bi_size=0. We must
	 *  not call submit_bio() then, because the routine
	 *  will panic.
	 *  xreq->error should be TRUE already then.
	 */
    if (!bio->bi_size) {
	bio_put (bio);
	return;
    }
    q = bdev_get_queue (bio->bi_bdev);
    if (q) {
	blk_get_queue (q);
	nkops.nk_atomic_add (&req->pendcount, 1);
	submit_bio (bio->bi_rw, bio);
	if (q->unplug_fn) {
	    q->unplug_fn (q);
	}
	blk_put_queue (q);
    } else {
	req->error = TRUE;
    }
}

    /* Called only from vbd_req_end_blkio_op() */

    static void
vbd_req_end_blkio_op_read_fast (vbd_pending_req_t* req, vbd_fast_map_t* map)
{
    vbd_pending_seg_t* seg   = req->segs;
    vbd_pending_seg_t* limit = req->segs + req->nsegs;

    while (seg != limit) {
	if (seg->vaddr) {
	    const unsigned int  off = (seg->gaddr & ~VBD_PAGE_64_MASK);
	    const unsigned long dst = (unsigned long) map->addr + off;
	    const unsigned long src = seg->vaddr + off;

	    vbd_fast_map_map (map, seg->gaddr & VBD_PAGE_64_MASK);
	    memcpy ((void*) dst, (void*) src, seg->size);
	    vbd_fast_map_unmap (map);
	    free_page (seg->vaddr);
		/* Collect statistics */
	    ++req->bl->fast_map_reads;
	    req->bl->fast_map_read_bytes += seg->size;
	    ++req->bl->page_freeings;
	    --req->bl->alloced_pages;
	}
	seg++;
    }
    vbd_req_blkio_done (req);
}

    static void
vbd_req_end_blkio_op (vbd_pending_req_t* req)
{
    if (!nkops.nk_sub_and_test (&req->pendcount, 1)) {
	if (req->op == VBD2_OP_READ || req->op == VBD2_OP_READ_EXT) {
	    vbd_link_t*		bl  = req->bl;
	    vbd_fast_map_t*	map = vbd_link_fast_map_alloc (bl);

	    if (map) {
		vbd_req_end_blkio_op_read_fast (req, map);
		vbd_link_fast_map_free (bl, map);
	    } else {
		vbd_link_put_to_done_list (req->bl, req);
	    }
	} else {
	    vbd_pending_seg_t* seg   = req->segs;
	    const vbd_pending_seg_t* limit = req->segs + req->nsegs;

	    while (seg != limit) {
		if (seg->vaddr) {
		    free_page (seg->vaddr);
		    ++req->bl->page_freeings;
		    --req->bl->alloced_pages;
		}
		seg++;
	    }
	    vbd_req_blkio_done (req);
	}
    }
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
    static int
vbd_bio_end_blkio_op (struct bio* bio, unsigned int done, int error)
{
    vbd_pending_req_t* req = bio->bi_private;

    if (!done || error) {
	req->error = TRUE;
    }
    vbd_req_end_blkio_op (req);
    bio_put (bio);
    return error;
}
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */
    static void
vbd_bio_end_blkio_op (struct bio* bio, int error)
{
    vbd_pending_req_t* req = bio->bi_private;

    if (error) {
	req->error = TRUE;
    }
    vbd_req_end_blkio_op (req);
    bio_put (bio);
}
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24) */
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0) */

    /*
     * Called only by vbd_link_rw() <- vbd_link_do_blkio_op() <-
     * vbd_blkio_thread().
     * Return value of TRUE indicates that the "req" has been
     * consumed and does not need to be resubmitted.
     */

    static _Bool
vbd_vdisk_rw (vbd_vdisk_t* vd, vbd2_req_header_t* req)
{
    const _Bool		is_read = req->op == VBD2_OP_READ ||
				  req->op == VBD2_OP_READ_EXT;
    const _Bool		is_ext  = req->op == VBD2_OP_WRITE_EXT ||
				  req->op == VBD2_OP_READ_EXT;
    vbd_link_t*		bl = vd->bl;
    const nku32_f	acc = is_read ? VBD_DISK_ACC_R : VBD_DISK_ACC_W;
    vbd_pending_req_t*	xreq;
    vbd_extent_t*	ex;
    unsigned int	seg;
    unsigned int	count;

    XTRACE ("%s req %d sector %lld count %d\n", vd->name, req->op,
	    req->sector, req->count);
    ++bl->requests;

    count = req->count;
    if (!count) {
	count = 256;
    }
    if (bl->pending_xreq) {
	xreq = bl->pending_xreq;
	bl->pending_xreq = NULL;
    } else {
	xreq = kmem_cache_alloc (bl->be->pending_req_cachep, GFP_ATOMIC);

	if (unlikely (!xreq)) {
	    ETRACE ("%s sector %lld, out of descriptors; will retry\n",
		    vd->name, req->sector);
	    bl->be->resource_error = TRUE;
	    bl->pending_msg = req;
	    ++bl->resource_errors;
	    return FALSE;	/* "req" not consumed */
	}
	vbd_pending_req_init (xreq, bl, req);
    }
    vbd_link_get (bl);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    for (seg = 0; seg < count; seg++) {
	NkPhAddr	paddr;
	nku32_f		psize;
	vbd_sector_t	sectors;
	struct buffer_head* bh;
	void*		vaddr;
	vbd_sector_t	psector = 0;
	const int	operation = (acc == VBD_DISK_ACC_W ? WRITE : READ);

	if (is_ext) {
	    ETRACE ("%s op %d not supported\n", vd->name, xreq->op);
	    xreq->error = TRUE;
	    break;
	}
	paddr = *(VBD2_FIRST_BUF (req) + seg);

	XTRACE (" 0x%llx:%lld:%lld\n",
		VBD2_BUF_PAGE (paddr),
		VBD2_BUF_SSECT (paddr),
		VBD2_BUF_ESECT (paddr));

	sectors = VBD2_BUF_SECTS (paddr);
	psize   = VBD2_BUF_SIZE (paddr);
	paddr   = VBD2_BUF_BASE (paddr);

	++bl->segments;

	ex = vbd_vdisk_translate (vd, acc, xreq->vsector, &psector);
	if (unlikely (!ex)) {
	    ETRACE ("%s sector %lld translation failure\n", vd->name,
		    xreq->vsector);
	    xreq->error = TRUE;
	    break;
	}
	bh = kmem_cache_alloc (bl->be->buffer_head_cachep, GFP_ATOMIC);
	if (unlikely (!bh)) {
	    ETRACE ("%s buffer header allocation failure\n", vd->name);
	    xreq->error = TRUE;
	    break;
	}
	vaddr = nkops.nk_mem_map (paddr, psize);
	if (!vaddr) {
	    kmem_cache_free (bl->be->buffer_head_cachep, bh);
	    ETRACE ("%s nk_mem_map(0x%llx, 0x%x) failure\n", vd->name,
		    (long long) paddr, psize);
	    xreq->error = TRUE;
	    break;
	}
	memset (bh, 0, sizeof *bh);

	init_waitqueue_head (&bh->b_wait);
	bh->b_size    = psize;
	bh->b_dev     = ex->dev;
	bh->b_rdev    = ex->dev;
	bh->b_rsector = (unsigned long) psector;
	bh->b_data    = (char*) vaddr;
	bh->b_page    = virt_to_page (vaddr);
	bh->b_end_io  = vbd_bh_end_blkio_op;
	bh->b_private = xreq;
	bh->b_state   = ((1 << BH_Mapped) | (1 << BH_Lock) |
			 (1 << BH_Req)    | (1 << BH_Launder));
	if (operation == WRITE)
	    bh->b_state |= (1 << BH_JBD) | (1 << BH_Req) | (1 << BH_Uptodate);

	atomic_set (&bh->b_count, 1);

	nkops.nk_atomic_add (&xreq->pendcount, 1);
	    /*
	     * Dispatch a single request. We'll flush it to disc later.
	     * This is a linux kernel function.
	     */
	generic_make_request (operation, bh);

	xreq->vsector += sectors;
    }
    vbd_req_end_blkio_op (xreq, !xreq->error);
#else
    {
	_Bool		one_bio = 0;
	struct bio*	bio = 0;
	vbd_sector_t	nsector = 0;
	vbd_fast_map_t*	map = is_read ? 0 : vbd_link_fast_map_alloc (bl);
	unsigned int	bi_max_vecs = 0;
	_Bool		finish_later = FALSE;

	for (seg = xreq->nsegs; seg < count; seg++) {
	    NkPhAddr		paddr;
	    nku32_f		psize;
	    vbd_sector_t	sectors;
	    struct bio_vec*	bv;
	    vbd_sector_t	psector = 0;
	    vbd_pending_seg_t*	pseg;
	    _Bool		copy_mode;

	    paddr = *(VBD2_FIRST_BUF (req) + seg);

	    if (is_ext) {
		XTRACE (" 0x%llx:%lld\n",
			(long long) VBD2_BUF_BASE_EXT (paddr),
			(long long) VBD2_BUF_SIZE_EXT (paddr));
	    } else {
		XTRACE (" 0x%llx:%lld:%lld\n",
			(long long) VBD2_BUF_PAGE (paddr),
			(long long) VBD2_BUF_SSECT (paddr),
			(long long) VBD2_BUF_ESECT (paddr));
	    }
	    if (is_ext) {
		psize = VBD2_BUF_SIZE_EXT (paddr);
		paddr = VBD2_BUF_BASE_EXT (paddr);
		if (psize & (VBD2_SECT_SIZE - 1)) {
		    sectors = 0;
		    one_bio = 1;
		} else {
		    sectors = psize >> VBD2_SECT_SIZE_BITS;
		}
	    } else {
		sectors = VBD2_BUF_SECTS (paddr);
		psize   = VBD2_BUF_SIZE (paddr);
		paddr   = VBD2_BUF_BASE (paddr);
	    }
	    ++bl->segments;
	    if (!vbd2_dma) {
		copy_mode = 1;
	    } else {
		    /* Check if we are allowed to use pfn_to_page() below */
		if (!pfn_valid (paddr >> PAGE_SHIFT)) {
		    DTRACE ("%s op %d vsector %lld physical address "
			    "%llx translation failure.\n", vd->name,
			    req->op, xreq->vsector,
			    (unsigned long long) paddr);
		    ++bl->no_struct_page;
		    copy_mode = 1;
		} else {
#ifdef CONFIG_X86
			/*
			 *  This check is only necessary on x86/VT
			 *  and on ARM the function is not implemented.
			 */
		    NkMhAddr maddr = nkops.nk_machine_addr (paddr);
		    if (maddr < 0x100000 && maddr != paddr) {
			DTRACE ("COPY mode forced (mach=0x%llx phys=0x%llx)\n",
				maddr, paddr);
			++bl->first_meg_copy;
			copy_mode = 1;
		    } else
#endif
		    {
			copy_mode = 0;
		    }
		}
	    }
#if defined CONFIG_X86 && !defined CONFIG_X86_PAE && !defined CONFIG_X86_64
		/*
		 *  Fix for BugId 4676708 ("VBD-BE crashes if passed a
		 *  physical address above 4GB when not compiled for PAE
		 *  or 64 bits"). Catch it here, rather than waiting
		 *  till the vbd_fast_map_map() call.
		 */
	    if (paddr > (nku32_f) -1) {
		ETRACE ("%s op %d vsector %lld physical address "
			"%llx is not reachable.\n", vd->name,
			req->op, xreq->vsector, paddr);
		xreq->error = TRUE;
		break;
	    }
#endif
	    ex = vbd_vdisk_translate (vd, acc, xreq->vsector, &psector);
	    if (unlikely (!ex)) {
		ETRACE ("%s sector %lld translation failure on %s\n", vd->name,
			xreq->vsector, is_read ? "read" : "write");
		xreq->error = TRUE;
		break;
	    }
	    if (!bio || bio->bi_vcnt == bi_max_vecs || psector != nsector) {
		unsigned int		vsize;
		unsigned int		nr_vecs;
		struct request_queue*	q;

		if (bio) {
		    vbd_req_submit_bio (xreq, bio);
		}
		vsize = count - seg;
		    /*
		     * We cannot place a request longer than the bio
		     * hw sector size, so in case of large requests
		     * we create chunks of number of bio vectors accepted
		     * by the physical device if needed.
		     */
		q	= bdev_get_queue (ex->bdev);
		nr_vecs	= bio_get_nr_vecs (ex->bdev);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,7)
		{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30)
		    const unsigned max_hw_pg = queue_max_hw_sectors (q) >> 3;
#else
		    const unsigned max_hw_pg = q->max_hw_sectors >> 3;
#endif
		    if (nr_vecs > max_hw_pg) {
			nr_vecs = max_hw_pg;
		    }
		}
#endif
		if (vsize > nr_vecs) {
		    vsize = nr_vecs;
		}
		if (one_bio && vsize != count) {
		    ETRACE ("%s cannot alloc several bios\n", vd->name);
		    xreq->error = TRUE;
		    bio		= NULL;
		    break;
		}
		bio = bio_alloc (GFP_ATOMIC, vsize);
		if (unlikely (!bio)) {
		    ETRACE ("%s sector %lld, out of bios; will retry\n",
			    vd->name, xreq->vsector);
		    bl->be->resource_error = TRUE;
		    bl->pending_xreq = xreq;
		    ++bl->resource_errors;
		    finish_later = TRUE;
		    break;
		}
		    /*
		     * bio_alloc() fixes vsize at bio's allocation time
		     * according to the memory pool where the bio has
		     * been allocated - we have to backup the value of
		     * vsize to be able to send the bio request when the
		     * allocated space is full.
		     */
		bi_max_vecs	= vsize;
		bio->bi_bdev    = ex->bdev;
		bio->bi_private = xreq;
		bio->bi_end_io  = vbd_bio_end_blkio_op;
		bio->bi_sector  = psector;
		bio->bi_rw      = acc == VBD_DISK_ACC_W ? WRITE : READ;
		bio->bi_size    = 0;
		bio->bi_vcnt    = 0;
	    }
	    bv = bio_iovec_idx (bio, bio->bi_vcnt);
	    bv->bv_len    = psize;
	    bv->bv_offset = paddr & ~VBD_PAGE_64_MASK;

	    pseg = xreq->segs + xreq->nsegs;

	    if (copy_mode) {
		unsigned long vaddr = __get_free_page (0);

		if (unlikely (!vaddr)) {
		    ETRACE ("%s sector %lld, out of pages; will retry\n",
			    vd->name, xreq->vsector);
		    bl->be->resource_error = TRUE;
		    bl->pending_xreq = xreq;
		    ++bl->resource_errors;
		    finish_later = TRUE;
			/*
			 *  "bio" retains all the segments which have
			 *  been prepared and for which variables like
			 *  xreq->nsegs, xreq->vsector and stats have
			 *  been updated. It will be submitted before
			 *  returning from this function.
			 */
		    break;
		}
		++bl->page_allocs;
		++bl->alloced_pages;
		if (bl->alloced_pages > bl->max_alloced_pages) {
		    bl->max_alloced_pages = bl->alloced_pages;
		}
		bv->bv_page = virt_to_page (vaddr);
		pseg->vaddr = vaddr;
		if (is_read) {
		    pseg->gaddr = paddr;
		    pseg->size  = psize;
			/*
			 *  We do not know yet if this is going to
			 *  be a fast_map_read or an nk_mem_map_read.
			 */
		} else {
			/* Copy guest page */
		    if (map) {
			unsigned long src = (unsigned long) map->addr +
			    bv->bv_offset;
			unsigned long dst = vaddr + bv->bv_offset;
			vbd_fast_map_map (map, paddr & VBD_PAGE_64_MASK);
			memcpy ((void*) dst, (void*) src, psize);
			vbd_fast_map_unmap (map);
			++bl->fast_map_writes;
			bl->fast_map_written_bytes += psize;
		    } else {
			void*		src = nkops.nk_mem_map (paddr, psize);
			unsigned long	dst = vaddr + bv->bv_offset;

			if (!src) {
			    ETRACE ("%s sector %lld, cannot mem map; fatal\n",
				    vd->name, xreq->vsector);
			    free_page (vaddr);
			    ++bl->page_freeings;
			    --bl->alloced_pages;
			    xreq->error = TRUE;
			    break;
			}
			memcpy ((void*) dst, src, psize);
			nkops.nk_mem_unmap (src, paddr, psize);
			++bl->nk_mem_map_writes;
			bl->nk_mem_map_written_bytes += psize;
		    }
		}
	    } else {
		pseg->vaddr = 0; /* No page */
		bv->bv_page = pfn_to_page (paddr >> PAGE_SHIFT);
		if (is_read) {
		    bl->dma_read_bytes += psize;
		    ++bl->dma_reads;
		} else {
		    bl->dma_written_bytes += psize;
		    ++bl->dma_writes;
		}
	    }
	    if (is_read) {
		bl->bytes_read += psize;
	    } else {
		bl->bytes_written += psize;
	    }
	    bio->bi_vcnt++;
	    bio->bi_size += psize;
		/* Increment nsegs only now, to validate "pseg" */
	    ++xreq->nsegs;

	    xreq->vsector += sectors;
	    nsector  = psector + sectors;
	}
	if (map) {
	    vbd_link_fast_map_free (bl, map);
	}
	if (bio) {
	    vbd_req_submit_bio (xreq, bio);
	}
	    /*
	     *  The xreq->pendcount reference count, which has been set
	     *  to 1 initially, protects xreq against being considered
	     *  as complete when all "bios" submitted up to now complete.
	     */
	if (!finish_later) {
	    vbd_req_end_blkio_op (xreq);
	}
    }
#endif
    return TRUE;	/* "req" consumed */
}

    /* Interrupt-level callback */

    static void
vbd_cb_receive_notify (vmq_link_t* link)
{
    vbd_link_t* bl = VBD_LINK (link);

    DTRACE ("guest %d\n", vmq_peer_osid (link));
    vbd_link_add_to_blkio_list_tail (bl);
    vbd_be_maybe_wake_up_blkio (bl->be);
}

/******************************************************************
 * DOWNWARD CALLS -- These interface with the block-device layer proper.
 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)

    /* Called by vbd_link_do_blkio_op() only */

    static void
vbd_link_blkio_done (vbd_link_t* bl)
{
    vbd_pending_req_t* req;

    while ((req = vbd_link_get_from_done_list (bl)) != NULL) {
	vbd_pending_seg_t* seg   = req->segs;
	vbd_pending_seg_t* limit = req->segs + req->nsegs;
	while (seg != limit) {
	    if (seg->vaddr) {
		void*		dst = nkops.nk_mem_map (seg->gaddr, seg->size);
		unsigned long	src =
			    seg->vaddr + (seg->gaddr & ~VBD_PAGE_64_MASK);

		if (dst) {
		    memcpy (dst, (void*) src, seg->size);
		    nkops.nk_mem_unmap (dst, seg->gaddr, seg->size);
		    ++bl->nk_mem_map_reads;
		    bl->nk_mem_map_read_bytes += seg->size;
		} else {
		    req->error   = TRUE;
		}
		free_page (seg->vaddr);
		++bl->page_freeings;
		--bl->alloced_pages;
	    }
	    seg++;
	}
	vbd_req_blkio_done (req);
    }
}
#endif	/* 2.6 */

static const char vbd_op_names[VBD2_OP_MAX][13] = {VBD2_OP_NAMES};

    /*
     * Called only by vbd_blkio_thread().
     * Can set be->resource_error.
     */

    static int
vbd_link_do_blkio_op (vbd_link_t* bl, int max_to_do)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    vbd_link_blkio_done (bl);
#endif
    while (max_to_do > 0) {
	vbd2_req_header_t* req;

	if (bl->pending_msg) {
	    req = bl->pending_msg;
	    bl->pending_msg = NULL;
	} else {
	    if (vmq_msg_receive (bl->link, (void**) &req)) break;
	}
	DTRACE ("guest %d: %s\n", vmq_peer_osid (bl->link),
		vbd_op_names [req->op % VBD2_OP_MAX]);
	switch (req->op) {
	case VBD2_OP_PROBE:
	    vbd_link_probe (bl, vbd_link_op_probe, req);
	    break;

	case VBD2_OP_MEDIA_PROBE:
	    vbd_link_probe (bl, vbd_link_op_media_probe, req);
	    break;

	case VBD2_OP_MEDIA_CONTROL:
	    vbd_link_probe (bl, vbd_link_op_media_control, req);
	    break;

	case VBD2_OP_MEDIA_LOCK:
	    vbd_link_probe (bl, vbd_link_op_media_lock, req);
	    break;

#ifdef VBD2_ATAPI
	case VBD2_OP_ATAPI:
	    vbd_link_atapi (bl, req);
	    break;
#endif
	case VBD2_OP_READ_EXT:
	case VBD2_OP_WRITE_EXT:
	case VBD2_OP_READ:
	case VBD2_OP_WRITE: {
	    _Bool msg_used = vbd_link_rw (bl, req);

	    VBD_ASSERT (msg_used ||
			(bl->be->resource_error && bl->pending_msg));
	    (void) msg_used;
	    break;
	}
	case VBD2_OP_OPEN:
	case VBD2_OP_CLOSE: {
	    vbd_vdisk_t* vd = vbd_link_vdisk_lookup (bl, req);

		/* Just check the devid and genid */
	    if (!vd) {
		    /* Message already issued */
		vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
		break;
	    }
	    vbd_link_resp (bl, req, VBD2_STATUS_OK);
	    break;
	}
	case VBD2_OP_GETGEO:
	default:
	    vbd_link_resp (bl, req, VBD2_STATUS_ERROR);
	    break;
	}
	    /*
	     *  In case of be->resource_error, current request
	     *  is not finished yet.
	     */
	if (bl->be->resource_error) break;
	max_to_do--;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	vbd_link_blkio_done (bl);
#endif
    }
    return !max_to_do;
}

/*----- Notification of VBD device list changes -----*/

    /* Can sleep for place in FIFO */

    static vbd2_msg_t*
vbd_link_alloc_async (vmq_link_t* link, vbd2_op_t op)
{
    vbd2_msg_t*	msg;

    if (unlikely (vmq_msg_allocate (link, 0, (void**) &msg, NULL)))
	return NULL;
    memset (msg, 0, sizeof *msg);
    msg->req.op = op;
    return msg;
}

    static _Bool
vbd_link_signal_change (vmq_link_t* link, void* cookie)
{
    vbd_link_t* bl =  VBD_LINK (link);

    DTRACE ("\n");
    (void) cookie;
    if (bl->changes < bl->be->changes && !bl->changes_signaled) {
	vbd2_msg_t* async;

	async = vbd_link_alloc_async (link, VBD2_OP_CHANGES);
	if (async) {
	    vmq_msg_send (link, async);
	    bl->changes_signaled = true;
	} else {
	    DTRACE ("failed to alloc async msg\n");
	}
    }
    return false;
}

    static void
vbd_be_signal_changes (vbd_be_t* be)
{
    DTRACE ("\n");
    ++be->changes;
    if (!be->links) return;
    vmq_links_iterate (be->links, vbd_link_signal_change, NULL);
}

/*----- /proc/nk/vbd2-be management -----*/

#include <linux/seq_file.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) && \
    LINUX_VERSION_CODE != KERNEL_VERSION(2,4,21) && \
    LINUX_VERSION_CODE != KERNEL_VERSION(2,4,25)

    static void*
single_start (struct seq_file *p, loff_t *pos)
{
    return NULL + (*pos == 0);
}

    static void*
single_next (struct seq_file *p, void *v, loff_t *pos)
{
    ++*pos;
    return NULL;
}

    static void
single_stop (struct seq_file *p, void *v)
{
}

    static int
single_open (struct file *file, int (*show)(struct seq_file *, void *),
	     void *data)
{
    struct seq_operations *op = kmalloc (sizeof *op, GFP_KERNEL);
    int res = -ENOMEM;

    if (op) {
	op->start = single_start;
	op->next = single_next;
	op->stop = single_stop;
	op->show = show;
	res = seq_open (file, op);
	if (!res) {
	    ((struct seq_file*) file->private_data)->private = data;
	} else {
	    kfree (op);
	}
    }
    return res;
}

    static int
single_release (struct inode *inode, struct file *file)
{
    struct seq_operations *op = ((struct seq_file*) file->private_data)->op;
    int res = seq_release (inode, file);

    kfree (op);
    return res;
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

    static _Bool
vbd_link_proc_show (vmq_link_t* link, void* cookie)
{
    vbd_link_t*		bl = VBD_LINK (link);
    struct seq_file*	seq = cookie;
    vbd_vdisk_t*	vd;

    seq_printf (seq, "I/O with guest %d (%s connected, vdisks %s ready):\n",
	vmq_peer_osid (bl->link), bl->connected ? "is" : "not",
	bl->vdisks_ok ? "all" : "not");
    seq_printf (seq, " General: requests %u segments %u read %llu"
	" written %llu\n",
	bl->requests,
	bl->segments,
	bl->bytes_read,
	bl->bytes_written);
    seq_printf (seq, " DMA: reads %u writes %u read %llu written %llu\n",
	bl->dma_reads,
	bl->dma_writes,
	bl->dma_read_bytes,
	bl->dma_written_bytes);
    seq_printf (seq,
	" fast-map: probes %u reads %u writes %u read %llu written %llu\n",
	bl->fast_map_probes,
	bl->fast_map_reads,
	bl->fast_map_writes,
	bl->fast_map_read_bytes,
	bl->fast_map_written_bytes);
    seq_printf (seq,
	" nk-mem-map: probes %u reads %u writes %u read %llu "
	"written %llu\n",
	bl->nk_mem_map_probes,
	bl->nk_mem_map_reads,
	bl->nk_mem_map_writes,
	bl->nk_mem_map_read_bytes,
	bl->nk_mem_map_written_bytes);
    seq_printf (seq, " no_struct_page %u page_allocs %u freeings %u"
	" max %u replies %u\n", bl->no_struct_page, bl->page_allocs,
	bl->page_freeings, bl->max_alloced_pages, bl->msg_replies);
    seq_printf (seq, " first_meg_copy %u resource_errors %u\n",
	bl->first_meg_copy, bl->resource_errors);

    VBD_LINK_FOR_ALL_VDISKS (vd, bl) {
	const vbd_extent_t* ex;
	unsigned count = 0;

	VBD_VDISK_FOR_ALL_EXTENTS (ex, vd) ++count;
	seq_printf (seq, " vdisk %d: %s %u extent(s), %llu "
		    "sectors, %s mode",
		    vd->tag, vd->name, count,
		    (unsigned long long) vbd_vdisk_sectors (vd),
		    vbd_vdisk_wait_modes [vd->wait % 3]);
#ifdef VBD2_ATAPI
	if (vd->atapi) {
	    seq_printf (seq, ", ATAPI(%s,%s)",
			vd->atapi->ctrl_name, vd->atapi->data_name);
	}
#endif
	seq_printf (seq, "\n");
    }
    return false;
}

    static int
vbd_seq_proc_show (struct seq_file* seq, void* v)
{
    vbd_be_t* be = seq->private;

    (void) v;
    if (be->links) {
	vmq_links_iterate (be->links, vbd_link_proc_show, seq);
    }
    seq_printf (seq, "Sleeps %llu withTimeout %u timeouts %u\n",
		be->sleeps_no_timeout, be->sleeps_with_timeout, be->timeouts);
    return 0;
}

    static int
vbd_proc_open (struct inode* inode, struct file* file)
{
    return single_open (file, vbd_seq_proc_show, PDE (inode)->data);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
static struct file_operations vbd_proc_fops =
#else
static const struct file_operations vbd_proc_fops =
#endif
{
    .owner	= THIS_MODULE,
    .open	= vbd_proc_open,
    .read	= seq_read,
    .llseek	= seq_lseek,
    .release	= single_release,
};

    /* On ARM, /proc/nk is created, but no proc_root_nk symbol is offered */

#ifndef CONFIG_ARM
extern struct proc_dir_entry* proc_root_nk;
#define vlx_proc_nk_lookup() proc_root_nk
#endif

#ifdef VBD2_ATAPI
static struct file_operations vbd_atapi_ctrl_proc_fops = {
    open:    vbd_atapi_ctrl_proc_open,
    release: vbd_atapi_ctrl_proc_release,
    llseek:  vbd_atapi_ctrl_proc_lseek,
    read:    vbd_atapi_ctrl_proc_read,
    write:   vbd_atapi_ctrl_proc_write,
};

static struct file_operations vbd_atapi_data_proc_fops = {
    open:    vbd_atapi_data_proc_open,
    release: vbd_atapi_data_proc_release,
    llseek:  vbd_atapi_data_proc_lseek,
    read:    vbd_atapi_data_proc_read,
    write:   vbd_atapi_data_proc_write,
};
#endif /* VBD2_ATAPI */

    static int __init
vbd_be_proc_init (vbd_be_t* be)
{
    struct proc_dir_entry* nk_ent = vlx_proc_nk_lookup();
    struct proc_dir_entry* ent;

    if (!nk_ent) {
	ETRACE ("Did not find /proc/nk\n");
	return -EAGAIN;
    }
    ent = create_proc_entry ("vbd2-be", 0, nk_ent);
    if (!ent) {
	ETRACE ("Could not create /proc/nk/vbd2-be\n");
	return -ENOMEM;
    }
    ent->proc_fops = &vbd_proc_fops;
    ent->data      = be;
    return 0;
}

    static void VBD_EXIT
vbd_proc_exit (void)
{
    struct proc_dir_entry* nk = vlx_proc_nk_lookup();

    if (nk) {
	remove_proc_entry ("vbd2-be", nk);
    }
}

#ifdef VBD2_ATAPI
    static _Bool __init
vbd_atapi_vdisk_init (vbd_vdisk_t* vd)
{
    struct proc_dir_entry*	nk_ent = vlx_proc_nk_lookup();
    struct proc_dir_entry*	ent;
    int				major, minor;
    vbd_atapi_t*		atapi;

    if (!vbd_vdisk_is_atapi (vd, &major, &minor) || !nk_ent) {
	return TRUE;
    }
    if (unlikely (!(atapi = kzalloc (sizeof *atapi, GFP_KERNEL)))) {
	EFTRACE ("out of memory\n");
	return FALSE;
    }
    mutex_init (&atapi->lock);
    init_waitqueue_head (&atapi->wait);

    snprintf (atapi->ctrl_name, sizeof atapi->ctrl_name,
	      "vbd-atapi-c-%u-%u", major, minor);
    ent = create_proc_entry (atapi->ctrl_name, 0, nk_ent);
    if (!ent) {
	EFTRACE ("could not create /proc/nk/%s\n", atapi->ctrl_name);
	kfree (atapi);
	return FALSE;
    }
    ent->data = vd;
    ent->proc_fops = &vbd_atapi_ctrl_proc_fops;

    snprintf (atapi->data_name, sizeof atapi->data_name,
	      "vbd-atapi-d-%u-%u", major, minor);
    ent = create_proc_entry (atapi->data_name, 0, nk_ent);
    if (!ent) {
	EFTRACE ("could not create /proc/nk/%s\n", atapi->data_name);
	remove_proc_entry (atapi->ctrl_name, nk_ent);
	kfree (atapi);
	return FALSE;
    }
    ent->data = vd;
    ent->proc_fops = &vbd_atapi_data_proc_fops;

    vd->atapi = atapi;
    return TRUE;
}

    static void VBD_EXIT
vbd_atapi_vdisk_exit (vbd_vdisk_t* vd)
{
    struct proc_dir_entry* nk_ent = vlx_proc_nk_lookup();

    if (vd->atapi && nk_ent) {
	remove_proc_entry (vd->atapi->ctrl_name, nk_ent);
	remove_proc_entry (vd->atapi->data_name, nk_ent);
	kfree (vd->atapi);
	vd->atapi = 0;
    }
}
#endif /* VBD2_ATAPI */

/*----- Command line / module options management -----*/

static vbd_be_t vbd_be;		/* Driver global data */

    static _Bool __init
vbd_vbd2_syntax (const char* opt)
{
    ETRACE ("Syntax error near '%s'\n", opt);
    return FALSE;
}

    static _Bool inline __init
vbd_vbd2_end (const char ch)
{
    return ch == ')' || ch == ';';
}

    static _Bool __init
vbd_vbd2_one (char* start, char** endp)
{
#ifdef MODULE
    vbd_vdisk_wait_t	wait = VBD_VDISK_WAIT_NO;
#else
    vbd_vdisk_wait_t	wait = VBD_VDISK_WAIT_YES;
#endif
    NkOsId		owner;
    long		vmajor;
    long		vminor;
    long		major;
    long		minor;
    long		acc;
    int			idx;
    char*		end;

    owner = simple_strtoul (start, &end, 0);
    if (end == start || *end != ',') {
	return vbd_vbd2_syntax (end);
    }
    start = end+1;

    vmajor = simple_strtoul (start, &end, 0);
    if (end == start || *end != ',') {
	return vbd_vbd2_syntax (end);
    }
    start = end+1;

    vminor = simple_strtoul (start, &end, 0);
    if (end == start || (*end != ':' && *end != '|' && *end != '/')) {
	return vbd_vbd2_syntax (end);
    }
    start = end+1;

    major = simple_strtoul (start, &end, 0);
    if (end == start || *end != ',') {
	return vbd_vbd2_syntax (end);
    }
    start = end+1;

    minor = simple_strtoul (start, &end, 0);
    if (end == start || *end != ',') {
	return vbd_vbd2_syntax (end);
    }
    start = end+1;

    if (!strncmp (start, "ro", 2)) {
	acc = VBD_DISK_ACC_R;
    } else if (!strncmp (start, "rw", 2)) {
	acc = VBD_DISK_ACC_RW;
    } else {
	return vbd_vbd2_syntax (start);
    }
    start += 2;

	/*
	 * vbd2=(... [,[nw|wa|nz]])
	 */
    if (*start == ',') {
	++start;
	if (*start != ',' && !vbd_vbd2_end (*start)) {
	    if (!strncmp (start, vbd_vdisk_wait_modes
			  [VBD_VDISK_WAIT_NO], 2)) {
		wait = VBD_VDISK_WAIT_NO;
	    } else if (!strncmp (start, vbd_vdisk_wait_modes
				 [VBD_VDISK_WAIT_YES], 2)) {
		wait = VBD_VDISK_WAIT_YES;
	    } else if (!strncmp (start, vbd_vdisk_wait_modes
				 [VBD_VDISK_WAIT_NON_ZERO], 2)) {
		wait = VBD_VDISK_WAIT_NON_ZERO;
	    } else {
		return vbd_vbd2_syntax (start);
	    }
	    start += 2;
	}
    }
    if (!vbd_vbd2_end (*start)) {
	return vbd_vbd2_syntax (start);
    }
    for (idx = 0; idx < VBD_ARRAY_ELEMS (vbd_be.prop_vdisks); ++idx) {
	if (!vbd_be.prop_vdisks [idx].devid) {
	    break;
	}
	if ((vbd_be.prop_vdisks [idx].owner == owner) &&
	    (vbd_be.prop_vdisks [idx].devid == VBD2_DEVID (vmajor, vminor))) {
	    ETRACE ("overwriting vdisk(%ld,%ld) config.\n", vmajor, vminor);
	    break;
	}
    }
    if (idx < VBD_ARRAY_ELEMS (vbd_be.prop_vdisks) &&
        idx < VBD_ARRAY_ELEMS (vbd_be.prop_extents)) {
	vbd_prop_vdisk_init (&vbd_be.prop_vdisks [idx], idx, owner,
			     VBD2_DEVID (vmajor, vminor), wait);
	vbd_prop_extent_init (&vbd_be.prop_extents [idx], 0, 0, acc, minor,
			      idx, major);
    } else {
	ETRACE ("too many disks: vdisk(%ld,%ld) ignored\n", vmajor, vminor);
    }
    *endp = start;
    return TRUE;
}

    /*
     * This function needs to be "int" for the __setup() macro
     * It returns 1 on success and 0 on failure.
     */

    static int __init
vbd_vbd2_setup (char* start)
{
    do {
	if (*start != '(') {
	    return vbd_vbd2_syntax (start);
	}
	++start;
	do {
	    if (!vbd_vbd2_one (start, &start)) {
		return FALSE;
	    }
	} while (*start++ == ';');
    } while (*start++ == ',');
    return TRUE;
}

    static int __init
vbd_vbd2_dma_setup (char* start)
{
    char* end;

    vbd2_dma = simple_strtoul (start, &end, 0);
    return 1;
}

#ifndef MODULE
__setup ("vbd2=",     vbd_vbd2_setup);
__setup ("vbd2_dma=", vbd_vbd2_dma_setup);
#else

    /* Loading parameters */
static char* vbd2 [CONFIG_NKERNEL_VBD_NR];
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
MODULE_PARM (vbd2, "1-" __MODULE_STRING (CONFIG_NKERNEL_VBD_NR) "s");
MODULE_PARM (vbd2_dma, "i");
#else
module_param_array (vbd2, charp, NULL, 0);
module_param       (vbd2_dma, int, 0);
#endif
MODULE_PARM_DESC (vbd2_dma, " Use DMA to front-end buffers (dflt: 1).");
MODULE_PARM_DESC (vbd2,
		 "  virtual disks configuration:\n\t\t"
		 "   \"vbd2=(<owner>,<vmaj>,<vmin>/<maj>,<min>,"
			"<access>)[,...]\"\n\t\t"
		 "  where:\n\t\t"
		 "   <owner>       is the owner OS ID in [0..31]\n\t\t"
		 "   <vmaj>,<vmin> is the virtual disk ID (major,minor)\n\t\t"
		 "   <maj>,<min>   is the associated real device "
			"major,minor\n\t\t"
		 "   <access>      defines access rights: \"ro\" or"
		 " \"rw\"\n");
#ifdef CONFIG_ARM
#define vlx_command_line	saved_command_line
#else
extern char* vlx_command_line;
#endif
#endif

    static void* __init
vbd_vlink_syntax (const char* opt)
{
    ETRACE ("Syntax error near '%s'\n", opt);
    return NULL;
}

/*----- Initialization and exit entry points -----*/

#include "vlx-vbd2-common.c"

    /* Only called from vbd_be_destroy() <- vbd_exit() */

    static _Bool VBD_EXIT
vbd_link_stop (vmq_link_t* link, void* cookie)
{
    vbd_link_t* bl = VBD_LINK (link);

    vbd_link_put (bl);
    DTRACE ("be.refcount %d link.refcount %d\n",
	    bl->be->refcount.counter, bl->refcount.counter);
    return false;
}

    /* Called from vbd_exit() only */

    static void VBD_EXIT
vbd_be_destroy (vbd_be_t* be)
{
    if (be->links) {
	vmq_links_abort (be->links);
	vmq_links_iterate (be->links, vbd_link_stop, NULL);
    }
    be->extent_abort = 1;	/* Wake up the extent thread */
    vlx_thread_join (&be->blkio_thread_desc);
    vlx_thread_join (&be->extent_thread_desc);

    if (be->links) {
	vmq_links_iterate (be->links, vbd_link_destroy, NULL);
	vmq_links_finish (be->links);
	be->links = NULL;
    }
    if (be->pending_req_cachep) {
	kmem_cache_destroy (be->pending_req_cachep);
	be->pending_req_cachep = NULL;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    if (be->buffer_head_cachep) {
	kmem_cache_destroy (be->buffer_head_cachep);
	be->buffer_head_cachep = NULL;
    }
#endif
}

    /* Only called from vbd_init() */

    static _Bool __init
vbd_link_init (vmq_link_t* link, void* cookie)
{
    vbd_link_t*	bl = VBD_LINK (link);
    vbd_be_t*	be = cookie;

    bl->link = link;
	/* Not yet connected, wait for "link_on" */
	/* No vdisks yet */
    bl->be = be;
	/*
	 * bl->blkio_list should not be initialized
	 * with INIT_LIST_HEAD() because this would
	 * fail the logic in vbd_link_on_blkio_list().
	 */

	/* This map can also be used in "DMA" mode if necessary */
    vbd_link_fast_map_create (bl);

#ifdef CONFIG_X86
    if (!bl->fast_map) {
	TRACE ("no fast map, performance will be reduced\n");
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    if (!pfn_valid ((800*1024*1024) >> PAGE_SHIFT)) {
	TRACE ("no high page, performance will be reduced\n");
    }
#endif
#endif
    INIT_LIST_HEAD (&bl->done_list);
    spin_lock_init (&bl->done_list_lock);
    vbd_be_get (be);	/* Bump be->refcount for every link */
    vbd_link_get (bl);	/* Bump bl->refcount for link */
    return false;
}

    static void VBD_EXIT
vbd_exit (void)
{
    DTRACE ("entered\n");
    vbd_proc_exit();
    vbd_be_destroy (&vbd_be);
    TRACE ("finished\n");
}

#define VBD_FIELD(name,value)	value

    static void
vbd_cb_link_on (vmq_link_t* link)
{
    DTRACE ("guest %d\n", vmq_peer_osid (link));
    VBD_LINK (link)->connected = 1;
}

    static void
vbd_cb_link_off (vmq_link_t* link)
{
    DTRACE ("guest %d\n", vmq_peer_osid (link));
    VBD_LINK (link)->connected = 0;
}

    static void
vbd_cb_link_off_completed (vmq_link_t* link)
{
    DTRACE ("guest %d\n", vmq_peer_osid (link));
}

#define VBD_LINKS(links) \
    ((vbd_be_t*) ((vmq_links_public*) (links))->priv)
#undef VBD_LINKS
#define VBD_LINKS(links) (*(vbd_be_t**) (links))

    static void
vbd_cb_sysconf_notify (vmq_links_t* links)
{
    vbd_be_t* be = VBD_LINKS (links);

    DTRACE ("entered\n");
    be->sysconf = true;
    wake_up (&be->blkio_wait);
}

    static const vmq_callbacks_t
vbd_callbacks = {
    VBD_FIELD (link_on,			vbd_cb_link_on),
    VBD_FIELD (link_off,		vbd_cb_link_off),
    VBD_FIELD (link_off_completed,	vbd_cb_link_off_completed),
    VBD_FIELD (sysconf_notify,		vbd_cb_sysconf_notify),
    VBD_FIELD (receive_notify,		vbd_cb_receive_notify),
    VBD_FIELD (return_notify,		NULL),
    VBD_FIELD (get_tx_config,		NULL),
    VBD_FIELD (get_rx_config,		vbd_cb_get_xx_config)
};

    static const vmq_xx_config_t
vbd_tx_config = {
    VBD_FIELD (msg_count,	4),
    VBD_FIELD (msg_max,		sizeof (vbd2_msg_t)),
    VBD_FIELD (data_count,	0),
    VBD_FIELD (data_max,	0)
};

#undef VBD_FIELD

    static int __init
vbd_be_init (vbd_be_t* be)
{
    be->pending_req_cachep = kmem_cache_create ("vbd2_pending_req_cache",
	sizeof (vbd_pending_req_t), 0, SLAB_HWCACHE_ALIGN,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
	NULL,
#endif
	NULL);
    if (!be->pending_req_cachep) {
	ETRACE ("could not create kmem_cache\n");
	return -ENOMEM;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    be->buffer_head_cachep = kmem_cache_create ("vbd2_buffer_head_cache",
	sizeof (struct buffer_head), 0, SLAB_HWCACHE_ALIGN, NULL, NULL);
    if (!be->buffer_head_cachep) {
	ETRACE ("could not create kmem_cache\n");
	return -ENOMEM;
    }
#endif
    init_waitqueue_head (&be->blkio_wait);
    INIT_LIST_HEAD (&be->blkio_list);
    spin_lock_init (&be->blkio_list_lock);
    return 0;
}

    static int __init
vbd_init (void)
{
    vbd_be_t*	be = &vbd_be;
    signed	diag;

    diag = vbd_be_init (be);
    if (diag) goto error;
#ifdef MODULE
    {
	char**	opt;
	char*	cmdline;

	    /* Parse kernel command line options first */
	cmdline = vlx_command_line;
	while ((cmdline = strstr (cmdline, "vbd2="))) {
	    cmdline += 5;
	    if (!vbd_vbd2_setup (cmdline)) {
		diag = -EINVAL;
		goto error;
	    }
	}
	cmdline = vlx_command_line;
	if ((cmdline = strstr (cmdline, "vbd2_dma="))) {
	    vbd_vbd2_dma_setup (cmdline + 9);	/* Never fails */
	}
	    /* Then arguments given to insmod */
	for (opt = vbd2; *opt; ++opt) {
	    if (!vbd_vbd2_setup (*opt)) {
		diag = -EINVAL;
		goto error;
	    }
	}
    }
#endif	/* MODULE */
    if (!be->prop_vdisks [0].devid) {
	ETRACE ("No virtual disk configured\n");
	diag = -EINVAL;
	goto error;
    }
    diag = vmq_links_init_ex (&be->links, "vbd2", &vbd_callbacks,
			      &vbd_tx_config, NULL /*rx_config*/, be, false);
    if (diag) goto error;
    vmq_links_iterate (be->links, vbd_link_init, be); /* Cannot fail */
    {
	const vbd_prop_vdisk_t*	pvd_max =
	    be->prop_vdisks + VBD_ARRAY_ELEMS (be->prop_vdisks);
	const vbd_prop_vdisk_t*	pvd;

	for (pvd = be->prop_vdisks; pvd < pvd_max && pvd->devid; ++pvd) {
	    diag = vbd_be_vdisk_create (be, pvd);
	    if (diag) goto error;
	}
    }
    vbd_be_proc_init (be);
    diag = vmq_links_start (be->links);
    if (diag) goto error;
    diag = vlx_thread_start (&be->blkio_thread_desc,
			     vbd_blkio_thread, be, "vbd2-be-blkio");
    if (diag) goto error;
    diag = vlx_thread_start_ex (&be->extent_thread_desc, vbd_extent_thread,
				be, "vbd2-be-extent", 1 /*will_suicide*/);
    if (diag) goto error;
    TRACE ("initialized\n");
    return 0;

error:
    ETRACE ("init failed (%d)\n", diag);
    vbd_exit();
    return diag;
}

module_init (vbd_init);
module_exit (vbd_exit);

/*----- Module description -----*/

MODULE_DESCRIPTION ("VLX Virtual Block Device v.2 backend driver");
MODULE_AUTHOR ("Adam Mirowski <adam.mirowski@virtuallogix.com>");
MODULE_LICENSE ("GPL");

/*----- End of file -----*/
