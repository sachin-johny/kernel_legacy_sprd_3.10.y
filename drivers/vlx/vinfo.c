/*
 ****************************************************************
 *
 *  Component: VLX Information Driver
 *
 *  Copyright (C) 2010 VirtualLogix.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Contributor(s):
 *    Adam Mirowski (adam.mirowski@VirtualLogix.COM) VirtualLogix
 *
 ****************************************************************
 */

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif
#include <linux/module.h>
#include <asm/system.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/init.h>		/* module_init() in 2.6.0 and before */
#include <linux/mman.h>

#include <nk/nkern.h>
#ifdef CONFIG_XEN
#include <asm/setup.h>
#include <asm/xen/hypervisor.h>
#else
#include <asm/nk/nktags.h>
#include <asm/nk/f_nk.h>
#include <vlx/vlcd_common.h>
#endif

/*----- Local configuration -----*/

#if 0
#define VINFO_MEMINFO
#endif

/*----- Debugging macros -----*/

#ifdef  DEBUG
#define TRACE(_f, _a...)  printk (KERN_ALERT "VINFO: " _f , ## _a)
#define DTRACE(_f, _a...) \
	do {printk (KERN_ALERT "%s: " _f, __func__, ## _a);} while (0)
#define XTRACE(_f, _a...)
#define ASSERT(c)      do {if (!(c)) BUG();} while (0)
#else
#define TRACE(_f, _a...)  printk (KERN_INFO  "VINFO: " _f , ## _a)
#define DTRACE(_f, _a...)
#define XTRACE(_f, _a...)
#define ASSERT(c)
#endif
#define ETRACE(_f, _a...) printk (KERN_ALERT "VINFO: " _f , ## _a)

/*----- Compatibility code -----*/

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
    struct seq_operations *op = kmalloc(sizeof(*op), GFP_KERNEL);
    int res = -ENOMEM;

    if (op) {
	op->start = single_start;
	op->next = single_next;
	op->stop = single_stop;
	op->show = show;
	res = seq_open(file, op);
	if (!res)
	    ((struct seq_file *)file->private_data)->private = data;
	else
	    kfree(op);
    }
    return res;
}

    static int
single_release (struct inode *inode, struct file *file)
{
    struct seq_operations *op = ((struct seq_file *)file->private_data)->op;
    int res = seq_release(inode, file);

    kfree(op);
    return res;
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define PDE(i)	((struct proc_dir_entry *)((i)->u.generic_ip))
#endif

/*----- /proc/nk/tags ------*/

#ifdef CONFIG_XEN
typedef struct tag_header NkTagHeader;
#endif

    static inline NkTagHeader*
vinfo_next_tag (NkTagHeader* t)
{
    return (NkTagHeader*)((nku32_f*) t + t->size);
}

#ifdef ATAG_MAP_DESC
    /* NK_MD_... constants */
static const char* vinfo_md_name[] = {"RAM", "FRAM", "ROM", "?", "IOME"};
#endif

    static void
vinfo_tags (struct seq_file* seq)
{
#ifdef CONFIG_XEN
    NkTagHeader *t = (NkTagHeader*) xen_start_info->arch.tags;
#else
    NkTagHeader* t = (NkTagHeader*) os_ctx->taglist;
#endif

    if (t->tag != ATAG_CORE) {
	seq_printf(seq, "No tag list found.\n");
	return;
    }
    for (; t->size; t = vinfo_next_tag(t)) {
	nku32_f* tag = (nku32_f*)(t + 1);

	seq_printf(seq, "%2x ", t->size);
	switch (t->tag) {
	case ATAG_CORE:
	    seq_printf(seq, "     core: flags 0x%08x pagesize 0x%08x "
		       "rootdev 0x%08x\n", tag[0], tag[1], tag[2]);
	    break;

	case ATAG_CMDLINE:
	    seq_printf(seq, "  cmdline: <%s>\n", (char*) tag);
	    break;

	case ATAG_MEM:
	    seq_printf(seq, "      mem: start 0x%08x size 0x%08x\n",
		       tag[1], tag[0]);
	    break;

	case ATAG_RAMDISK:
	    seq_printf(seq, "  ramdisk: start 0x%08x size 0x%08x"
		       " flags %d\n", tag[2], tag[1], tag[0]);
	    break;

	case ATAG_INITRD:
	    seq_printf(seq, "   initrd: start 0x%08x size 0x%08x\n",
		       tag[0], tag[1]);
	    break;

	case ATAG_INITRD2:
	    seq_printf(seq, "  initrd2: start 0x%08x size 0x%08x\n",
		       tag[0], tag[1]);
	    break;

	case ATAG_SERIAL:
	    seq_printf(seq, "   serial: 0x%08x%08x\n", tag[1], tag[0]);
	    break;

	case ATAG_REVISION:
	    seq_printf(seq, " revision: 0x%08x\n", tag[0]);
	    break;

#ifdef ATAG_ARCH_ID
	case ATAG_ARCH_ID:
	    seq_printf(seq, "  arch-id: 0x%x\n", tag[0]);
	    break;
#endif

#ifdef ATAG_MAP_DESC
	case ATAG_MAP_DESC: {
	    unsigned md = tag[4];
	    char buf [16];
	    const char* owner = buf;

	    if (md > NK_MD_IO_MEM) md = 3;
	    switch ((int) tag[5]) {
	    case NK_OS_NKERN:
		owner = "NK";
		break;
	    case NK_OS_ANON:
		owner = "ANON";
		break;
	    case 31:
		owner = "SHARED";
		break;
	    default:
		snprintf (buf, sizeof buf, "OS %2d", tag[5]);
		break;
	    }
	    seq_printf(seq, " map-desc: 0x%08x-0x%08x => 0x%08x pte 0x%04x "
		       "%-4s %s\n", tag[0], tag[1], tag[2], tag[3],
		       vinfo_md_name[md], owner);
	    break;
	}
#endif

#ifdef ATAG_TTB_FLAGS
	case ATAG_TTB_FLAGS:
	    seq_printf(seq, "ttb-flags: 0x%x\n", tag[0]);
	    break;
#endif

#ifdef ATAG_TAG_REF
	case ATAG_TAG_REF:
	    seq_printf(seq, "  tag-ref: 0x%08x\n", tag[0]);
	    break;
#endif

#ifdef ATAG_HYPIO
	case ATAG_HYPIO:
	    seq_printf(seq, "    hypio: type %d start %x size %x\n",
		       tag[0], tag[1], tag[2]);
	    break;
#endif

	default:
	    seq_printf(seq, " %8x: 0x%x\n", t->tag, tag[0]);
	    break;
	}
    }
}

/*----- /proc/nk/vdevs -----*/

    static const char*
vinfo_link_state (int state)
{
    switch (state) {
    case NK_DEV_VLINK_OFF:   return "OFF";
    case NK_DEV_VLINK_RESET: return "RST";
    case NK_DEV_VLINK_ON:    return "ON";
    default: break;
    }
    return "?";
}

    static const char*
vinfo_link_info (NkPhAddr info)
{
    if (info) {
	return (const char*) nkops.nk_ptov (info);
    }
    return "";
}

#ifndef CONFIG_XEN
    static const char*
vinfo_resource_type (NkResourceType type)
{
    switch (type) {
    case NK_RESOURCE_PDEV: return "PDEV";
    case NK_RESOURCE_PMEM: return "PMEM";
    case NK_RESOURCE_PXIRQ: return "PXIRQ";
    default: break;
    }
    return "?";
}

    static void
vinfo_vlcd_pdev (struct seq_file* seq, const NkDevVlcd* d)
{
    unsigned i;

    seq_printf (seq, "  event (%x) %s%s%s%s flags (%x) %s cxirq %d\n",
	d->modified,
	d->modified & VLCD_EVT_INIT           ? "I" : "",
	d->modified & VLCD_EVT_SIZE_MODIFIED  ? "S" : "",
	d->modified & VLCD_EVT_COLOR_MODIFIED ? "C" : "",
	d->modified & VLCD_EVT_FLAGS_MODIFIED ? "F" : "",
	d->flags,
	d->flags    & VLCD_FLAGS_SWITCH_OFF   ? "O" : "",
	d->cxirq);

    for (i = 0; i < VLCD_MAX_CONF_NUMBER; ++i) {
	const vlcd_pconf_t*      pconf = d->pconf + i;
	const vlcd_color_conf_t* cconf = &pconf->color_conf;

	if (!cconf->bpp) continue;
	seq_printf (seq, "  %d: %dx%dx%d RGB %d:%d %d:%d %d:%d align %d\n",
	    i, pconf->xres, pconf->yres, cconf->bpp,
	    cconf->  red_offset, cconf->  red_length,
	    cconf->green_offset, cconf->green_length,
	    cconf-> blue_offset, cconf-> blue_length, cconf->align);
    }
    {
	const vlcd_conf_t*        conf = &d->current_conf;
	const vlcd_color_conf_t* cconf = &conf->color_conf;

	seq_printf (seq, "  %dx%dx%d (%dx%d) offs %x/%x dma %x/%x\n",
	    conf->xres, conf->yres, cconf->bpp,
	    conf->xres_virtual, conf->yres_virtual,
	    conf->xoffset, conf->yoffset,
	    conf->dma_zone_paddr, conf->dma_zone_size);
	seq_printf (seq, "  RGB %d:%d %d:%d %d:%d align %d\n",
	    cconf->  red_offset, cconf->  red_length,
	    cconf->green_offset, cconf->green_length,
	    cconf-> blue_offset, cconf-> blue_length, cconf->align);
    }
}
#endif

    static void
vinfo_vdevs (struct seq_file* seq)
{
    int cls;
    static const char* classNames[] = {"Gen", "PCI", "P64", "ISA",
	    "VME", "QUI"};
	/*
	 *  It is easy to lookup all devices, but requires writing
	 *  different code for VT and non-VT environments. For now,
	 *  we have this generic code, which benefits from the fact
	 *  that classes are numbered linearly and that there are
	 *  very few of them, which is not the case for types.
	 */
/*
CL PDEV     VDEV     CLASS   DEV_ID   NAME CLASSHDR DEV_HDR  LCK OWN
0  1050000c 1050000c 0 (Gen) 53595342 BSYS 00000000 10500028 0   0
*/
    seq_printf (seq, "C PDEV     VDEV     CLASS   DEV_ID   NAME "
	"CLASSHDR DEV_HDR  LCK OWN\n");
    for (cls = NK_DEV_CLASS_GEN; cls <= NK_DEV_CLASS_QUICC; ++cls) {
	NkPhAddr pdev = 0;

	while ((pdev = nkops.nk_dev_lookup_by_class (cls, pdev)) != 0) {
	    NkDevDesc* vdev = (NkDevDesc*) nkops.nk_ptov (pdev);

	    seq_printf (seq, "%d %8llx %8x %1d (%3s) %8x %c%c%c%c "
		"%8llx %8llx %3d %3d\n",
		cls, (long long) pdev, (int) vdev, vdev->class_id,
		classNames [vdev->class_id], vdev->dev_id,
#if 1
		((char*) &vdev->dev_id) [3],
		((char*) &vdev->dev_id) [2],
		((char*) &vdev->dev_id) [1],
		((char*) &vdev->dev_id) [0],
#else
		((char*) &vdev->dev_id) [0],
		((char*) &vdev->dev_id) [1],
		((char*) &vdev->dev_id) [2],
		((char*) &vdev->dev_id) [3],
#endif
		(long long) vdev->class_header,
		(long long) vdev->dev_header,
		vdev->dev_lock, vdev->dev_owner);

#define NK_DEV_ID_SHM2	0x53484d32	/* "SHM2" */

	    if (vdev->dev_id == NK_DEV_ID_NKIO) {
		NkDevNkIO* i = (NkDevNkIO*) nkops.nk_ptov (vdev->dev_header);

		seq_printf (seq, " base %x size %x\n", i->base, i->size);
	    } else if (vdev->dev_id == NK_DEV_ID_SYSBUS ||
		       vdev->dev_id == NK_DEV_ID_SHM2) {
		NkDevSysBus* sb1 = (NkDevSysBus*)
		    nkops.nk_ptov (vdev->dev_header);
		int os;

		for (os = NK_OS_NKERN; os < NK_OS_LIMIT; ++os) {
		    NkDevSysBridge* sb2 = sb1->bridge + os;

		    if (!(sb1->connected & (1 << os))) continue;
		    seq_printf (seq, " %2d: pmem_start %8llx pmem_size "
			"%6llx xirq_base %d xirq_limit %d stamp %d\n",
			os, (long long) sb2->pmem_start,
			(long long) sb2->pmem_size,
			sb2->xirq_base, sb2->xirq_limit,
			sb2->stamp);
		}
	    } else if (vdev->dev_id == NK_DEV_ID_RING) {
		NkDevRing* r = (NkDevRing*)
		    nkops.nk_ptov (vdev->dev_header);

		seq_printf (seq, " pid %d type %c%c%c%c cx/pxirq %d/%d "
		    "dsize %x imask %x iresp/q %d/%d base %llx\n",
		    r->pid,		/* Producer OS ID */
		    ((char*) &r->type) [3],	/* Ring type */
		    ((char*) &r->type) [2],
		    ((char*) &r->type) [1],
		    ((char*) &r->type) [0],
		    r->cxirq,	/* consumer XIRQ */
		    r->pxirq,	/* producer XIRQ */
		    r->dsize,	/* ring descriptor size */
		    r->imask,	/* ring index mask */
		    r->iresp,	/* consumer response ring index */
		    r->ireq,	/* producer request ring index */
		    (long long) r->base); /* ring physical base address */
	    } else if (vdev->dev_id == NK_DEV_ID_VLINK) {
		NkDevVlink* v = (NkDevVlink*) nkops.nk_ptov (vdev->dev_header);
#ifndef CONFIG_XEN
		NkResource* r;
#endif

		seq_printf (seq, " %5s,%d serv %d/%s/'%s' cli %d/%s/'%s'\n",
		    v->name, v->link,
		    v->s_id, vinfo_link_state (v->s_state),
		    vinfo_link_info (v->s_info),
		    v->c_id, vinfo_link_state (v->c_state),
		    vinfo_link_info (v->c_info));

#ifndef CONFIG_XEN
		    /*
		     *  With the Isolator, it is not possible to list the
		     *  resources, because the list is outside the address
		     *  space of the guest. NkDevVlink::resrc is either an
		     *  unusable machine address or NULL. So it makes no
		     *  sense to compile this code in.
		     */
		r = (NkResource*) &v->resrc;
		while (r->next) {
		    r = (NkResource*) nkops.nk_ptov (r->next);
		    seq_printf (seq, "  %-5s %d ",
			vinfo_resource_type (r->type), r->id);
		    switch (r->type) {
		    case NK_RESOURCE_PDEV:
		    case NK_RESOURCE_PMEM:
			seq_printf (seq, "addr %x size %x\n",
			    r->r.pdev.addr, r->r.pdev.size);
			if (r->type == NK_RESOURCE_PDEV &&
				!strcmp (v->name, "vlcd")) {
			    vinfo_vlcd_pdev (seq,
				nkops.nk_ptov (r->r.pdev.addr));
			}
			break;

		    case NK_RESOURCE_PXIRQ:
			seq_printf (seq, "osid %d base %d numb %d\n",
			    r->r.pxirq.osid,
			    r->r.pxirq.base,
			    r->r.pxirq.numb);
			break;

		    default:
			seq_printf (seq, "\n");
			break;
		    }
		}
#endif
	    }
	}
    }
}

/*----- /proc/nk/checkpages -----*/

    static bool inline
vinfo_page_valid (struct page* p)
{
    return p >= mem_map && p - mem_map < max_mapnr &&
	((unsigned long) p - (unsigned long) mem_map) % sizeof (*p) == 0;
}

    static bool inline
vinfo_vaddr_owned (void* addr)
{
    return pfn_valid(__phys_to_pfn(__pa(addr)));
}

    bool
vinfo_ptr_is_ok (void* ptr, const char* name, const bool quiet)
{
    if ((unsigned long) ptr & 0x3) {
	if (!quiet) TRACE ("'%s' has bad alignement: %p\n", name, ptr);
	return 0;
    }
    if (!vinfo_vaddr_owned (ptr)) {
	if (!quiet) TRACE ("'%s' is an invalid kernel address: %p\n", name,
			   ptr);
	return 0;
    }
    return 1;
}

    /* page->private is small values or vaddrs */
    /* This function can be called from kernel, so no static */

    bool
vinfo_page_private_is_ok (const unsigned long private, const bool quiet)
{
    if (private < 16) {
	return 1;
    }
    return vinfo_ptr_is_ok ((void*) private, "private", quiet);
}

    static bool
vinfo_page_list_ptr_is_ok (void* ptr, struct page* me, const bool quiet)
{
    struct page* container = container_of(ptr, struct page, lru.next);

    if (ptr == LIST_POISON1) return 1;
    if (ptr == LIST_POISON2) return 1;
    if (container == me) return 1;
    if (vinfo_page_valid (container)) return 1;
    return vinfo_ptr_is_ok (ptr, "list ptr", quiet);
}

    static void
vinfo_format_page_list_ptr (void* ptr, struct page* me, char* buf,
			    size_t buflen)
{
    struct page* container = container_of(ptr, struct page, lru.next);

    if (ptr == LIST_POISON1) {
	snprintf (buf, buflen, "POISON_1");
    } else if (ptr == LIST_POISON2) {
	snprintf (buf, buflen, "POISON_2");
    } else if (container == me) {
	snprintf (buf, buflen, "Self");
    } else if (vinfo_page_valid (container)) {
	snprintf (buf, buflen, "Pag%5x", container - mem_map);
    } else {
	snprintf (buf, buflen, "%p", ptr);
    }
}

    static bool
vinfo_list_is_ok (const struct list_head* const start, const bool quiet)
{
    struct list_head* curr;
    unsigned elems;
    bool print_elems = 0;

    if (start->next == LIST_POISON1) {
	return 1;
    }
rescan:
    if (print_elems) {
	TRACE ("List: %p", start);
    }
    for (curr = start->next,  elems = 0;
	 curr != start     && elems < max_mapnr;
	 curr = curr->next, ++elems) {

	if (print_elems) {
	    printk (" -> %p", curr);
	}
	if (!vinfo_ptr_is_ok (curr, "list.next", quiet || print_elems)) {
	    if (quiet) return 0;
	    if (print_elems) {
		printk ("\n");
		return 0;
	    }
	    TRACE ("List at %p has invalid curr %p (elems %d)\n", start, curr,
		   elems);
	    print_elems = 1;
	    goto rescan;
	}
    }
    if (elems >= max_mapnr) {
	if (!quiet) {
	    TRACE ("List at %p too long (> %d elems), next %p prev %p\n",
		   start, elems, start->next, start->prev);
	}
	return 0;
    }
    return 1;
}

    /* Can be called from kernel */

    bool
vinfo_page_is_ok (struct page* page, const bool quiet)
{
    bool is_ok = 1;

	/* No need to check "page" itself */
    if (!quiet) {
	char next [32];
	char prev [32];

	vinfo_format_page_list_ptr (page->lru.next, page, next, sizeof next);
	vinfo_format_page_list_ptr (page->lru.prev, page, prev, sizeof prev);

	TRACE ("%4x %p: %08lx %2d %8x %8lx %8lx %5lx %8s/%8s",
		page - mem_map, page, page->flags, page->_count.counter,
		page->_mapcount.counter, page_private (page), (unsigned
		long) page->mapping, page->index, next, prev);

	if (PageSlab (page)) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	    struct kmem_cache* km = (struct kmem_cache*) page->lru.next;
		/*
		 * page->flags contain 0x80
		 * According to mm/slab.c:
		 * lru.next = struct kmem_cache*
		 * lru.prev = struct slab*
		 */
	    printk (" '%s'", km->name);
#endif
	}
	printk ("\n");
    }
    if (!vinfo_page_private_is_ok (page_private (page), quiet)) {
	is_ok = 0;
    }
#ifdef VLX_STRUCT_PAGE_HAS_PRIVATE2
    if (page_private (page) != page->private2) {
	if (!quiet) TRACE ("private2 is different: %lx\n", page->private2);
	is_ok = 0;
    }
#endif
    if (!vinfo_page_list_ptr_is_ok (page->lru.next, page, quiet)) {
	is_ok = 0;
    }
    if (!vinfo_page_list_ptr_is_ok (page->lru.prev, page, quiet)) {
	is_ok = 0;
    }
    if (PageSlab (page)) {
	/* In version 2.6.29, kmem_cache is opaque for slab */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	struct kmem_cache* km = (struct kmem_cache*) page->lru.next;

	(void) km;
#ifdef CONFIG_SLAB
	if (!vinfo_list_is_ok (&km->next, quiet)) {
	    is_ok = 0;
	}
#endif
#ifdef CONFIG_SLUB
	if (!vinfo_list_is_ok (&km->list, quiet)) {
	    is_ok = 0;
	}
#endif
#endif
	    /* Could check list in struct slab, but this struct is opaque */
    } else {
	if (!vinfo_list_is_ok (&page->lru, quiet)) {
	    is_ok = 0;
	}
    }
    return is_ok;
}

    static void
vinfo_checkpages (struct seq_file* seq)
{
    unsigned total = 0, bad = 0;
    unsigned long pfn;

    for (pfn = 0; pfn <= 0xfffff; ++pfn) {
	if (!pfn_valid (pfn)) continue;
	++total;
	printk ("%lx\r", pfn);
	if (!vinfo_page_is_ok (pfn_to_page (pfn), 1)) {
	     vinfo_page_is_ok (pfn_to_page (pfn), 0);
	     ++bad;
	}
    }
    if (bad) {
	seq_printf (seq, "%d of %d pages did not check ok. "
		    "Look at kernel messages for details.\n",
		    bad, total);
    } else {
	seq_printf (seq, "All %d pages checked ok.\n", total);
    }
}

/*----- /proc/nk/meminfo -----*/

#ifdef VINFO_MEMINFO
    static void
vinfo_print_pfn_ranges (struct seq_file* seq)
{
    bool valid = 0;
    unsigned long pfn;

    seq_printf (seq, "PFN ranges\n");
    for (pfn = 0; pfn <= 0xfffff; ++pfn) {
	if (pfn_valid (pfn)) {
	    if (!valid) {
		seq_printf (seq, "%lx - ", pfn);
		valid = 1;
	    }
	} else if (valid) {
	    seq_printf (seq, "%lx\n", pfn-1);
	    valid = 0;
	}
    }
    if (valid) {
	seq_printf (seq, "%lx\n", pfn-1);
    }
}

    static void
vinfo_meminfo (struct seq_file* seq)
{
    seq_printf (seq, "ZONES_PGSHIFT %x\n", ZONES_PGSHIFT);
    seq_printf (seq, "ZONES_MASK %lx\n", ZONES_MASK);
    seq_printf (seq, "MAX_NR_ZONES %x\n", MAX_NR_ZONES);
#ifdef MAX_ZONELISTS
    seq_printf (seq, "MAX_ZONELISTS %x\n", MAX_ZONELISTS);
#endif
    seq_printf (seq, "nr_zones %d\n", contig_page_data.nr_zones);
#ifdef NR_LRU_LISTS
    seq_printf (seq, "NR_LRU_LISTS %x\n", NR_LRU_LISTS);
#endif
    seq_printf (seq, "MAX_ORDER %d\n", MAX_ORDER);
    {
	unsigned i;

	for (i = 0; i < contig_page_data.nr_zones; ++i) {
	    struct zone* z = &contig_page_data.node_zones [i];
	    unsigned j;

	    seq_printf (seq, "Zone: '%s'\n", z->name);
	    (void) j;
#ifdef NR_LRU_LISTS
	    for (j = 0; j < NR_LRU_LISTS; ++j) {
		seq_printf (seq, "LRU: %d\n", j);
		vinfo_list_is_ok (&z->lru [j].list, 0);
	    }
#endif
#ifdef MIGRATE_TYPES
	    for (j = 0; j < MAX_ORDER; ++j) {
		struct free_area* fa = z->free_area + j;
		unsigned k;

		seq_printf (seq, "Order: %d\n", j);
		    /* fa->nr_free is not size of used area in free_list */
		for (k = 0; k < MIGRATE_TYPES; ++k) {
		    seq_printf (seq, "FreeArea: %d\n", k);
		    vinfo_list_is_ok (fa->free_list + k, 0);
		}
	    }
#endif
	}
    }
    seq_printf (seq, "mem_map %p  max_mapnr 0x%lx\n", mem_map, max_mapnr);
    seq_printf (seq, "high_memory %p num_physpages 0x%lx\n",
		high_memory, num_physpages);
    seq_printf (seq, "sizeof(struct page) 0x%x\n", sizeof (struct page));
    seq_printf (seq, "MAX_ORDER_NR_PAGES %d\n", MAX_ORDER_NR_PAGES);
    {
	pg_data_t *n = NODE_DATA (0 /*node*/);
	struct page* p = n->node_mem_map;

	seq_printf (seq, "node_mem_map   %p\n", n->node_mem_map);
	seq_printf (seq, "node_start_pfn %lx\n", n->node_start_pfn);
	seq_printf (seq, "node_present_pages %ld\n", n->node_present_pages);
	seq_printf (seq, "node_spanned_pages %ld\n", n->node_spanned_pages);
	seq_printf (seq, "ARCH_PFN_OFFSET %lx\n", (long) ARCH_PFN_OFFSET);
	seq_printf (seq, "map[0]: flags %lx _count %d _mapcount %d "
		    "private %lx index %lx\n",
		    p->flags, p->_count.counter, p->_mapcount.counter,
		    page_private (p), p->index);
    }
    vinfo_print_pfn_ranges (seq);
}
#endif	/* VINFO_MEMINFO */

/*----- /proc/nk/id -----*/

    static void
vinfo_id (struct seq_file* seq)
{
    seq_printf (seq, "%d", nkops.nk_id_get());
}

/*----- /proc/nk/last -----*/

    static void
vinfo_last (struct seq_file* seq)
{
    seq_printf (seq, "%d", nkops.nk_last_id_get());
}

/*----- /proc/nk/state -----*/

    static void
vinfo_state (struct seq_file* seq)
{
    seq_printf (seq, "%d", nkops.nk_running_ids_get());
}

/*----- Common /proc/nk code -----*/

    static int
vinfo_proc_show (struct seq_file* seq, void* v)
{
    ((void (*)(struct seq_file*)) seq->private) (seq);
    return  0;
}

    static int
vinfo_proc_open (struct inode *inode, struct file *file)
{
    return single_open(file, vinfo_proc_show, PDE(inode)->data);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
static struct file_operations vinfo_proc_fops =
#else
static const struct file_operations vinfo_proc_fops =
#endif
{
    .owner	= THIS_MODULE,
    .open	= vinfo_proc_open,
    .read	= seq_read,
    .llseek	= seq_lseek,
    .release	= single_release,
};

    /* On ARM, /proc/nk is created, but no proc_root_nk symbol is offered */

#ifdef CONFIG_ARM
#define VLX_SERVICES_PROC_NK
#include "vlx-services.c"
#else
extern struct proc_dir_entry* proc_root_nk;
#define vlx_proc_nk_lookup() proc_root_nk
#endif

static struct {
    const char* name;
    void (*func) (struct seq_file* seq);
} vinfo_files[] = {
#ifdef VINFO_MEMINFO
    {"meminfo",    vinfo_meminfo},
#endif
    {"checkpages", vinfo_checkpages},
    {"tags",       vinfo_tags},
    {"vdevs",      vinfo_vdevs},
    {"id",         vinfo_id},
    {"last",       vinfo_last},
    {"state",      vinfo_state}
};

#define VINFO_MAX	(sizeof vinfo_files / sizeof vinfo_files [0])

    static void
vinfo_cleanup (unsigned until)
{
    struct proc_dir_entry* nk = vlx_proc_nk_lookup();
    unsigned i;

    if (!nk) return;
    for (i = 0; i < until; ++i) {
	remove_proc_entry (vinfo_files [i].name, nk);
    }
}

    static int __init
vinfo_proc_init (void)
{
    struct proc_dir_entry* nk_ent = vlx_proc_nk_lookup();
    int i;

    if (!nk_ent) {
	return -EAGAIN;
    }
    for (i = 0; i < (int) VINFO_MAX; ++i) {
	struct proc_dir_entry* ent;

	ent = create_proc_entry (vinfo_files[i].name, 0 /*mode_t*/, nk_ent);
	if (!ent) {
	    vinfo_cleanup (i - 1);
	    return -ENOMEM;
	}
	ent->proc_fops = &vinfo_proc_fops;
	ent->data = vinfo_files [i].func;
    }
    return 0;
}

    static void __exit
vinfo_proc_exit (void)
{
    vinfo_cleanup (VINFO_MAX);
}

MODULE_DESCRIPTION("VirtualLogix VLX information driver");
MODULE_AUTHOR("Adam Mirowski <adam.mirowski@virtuallogix.com>");
MODULE_LICENSE("GPL");

#ifdef CONFIG_ARM
#define vlx_command_line	saved_command_line
#else
extern char* vlx_command_line;
#endif

    static int __init
vinfo_module_init (void)
{
    vinfo_proc_init();

    TRACE("module loaded\n");
    return 0;
}

    static void __exit
vinfo_module_exit (void)
{
    vinfo_proc_exit();

    TRACE("module unloaded\n");
}

module_init(vinfo_module_init);
module_exit(vinfo_module_exit);

