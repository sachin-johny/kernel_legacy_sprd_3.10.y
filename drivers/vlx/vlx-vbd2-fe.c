/*
 ****************************************************************
 *
 * Component = VLX Virtual Block Device v.2 frontend driver
 *
 * Copyright (C) 2005-2010 VirtualLogix.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * #ident  "@(#)vlx-vbd-fe.c 1.76     10/01/09 VirtualLogix"
 *
 * Contributor(s):
 *   Vladimir Grouzdev (vladimir.grouzdev@VirtualLogix.COM) VirtualLogix
 *   Eric Lescouet (eric.lescouet@VirtualLogix.COM) VirtualLogix
 *   Adam Mirowski (adam.mirowski@VirtualLogix.COM) VirtualLogix
 *
 ****************************************************************
 */

/******************************************************************************
 * arch/xen/drivers/blkif/frontend/common.h
 *
 * Shared definitions between all levels of XenoLinux Virtual block devices.
 */

/******************************************************************************
 * blkfront.c
 *
 * XenLinux virtual block-device driver.
 *
 * Copyright (c) 2003-2004, Keir Fraser & Steve Hand
 * Modifications by Mark A. Williamson are (c) Intel Research Cambridge
 * Copyright (c) 2004, Christian Limpach
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

/******************************************************************************
 * arch/xen/drivers/blkif/frontend/vbd.c
 *
 * Xenolinux virtual block-device driver.
 *
 * Copyright (c) 2003-2004, Keir Fraser & Steve Hand
 * Modifications by Mark A. Williamson are (c) Intel Research Cambridge
 */

/*----- Detection of MontaVista CGE 3.1 -----*/

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#include <linux/termios.h>		/* Only to detect Montavista */
#include <linux/tty_driver.h>		/* Only to detect Montavista */
#ifdef _KOBJECT_H_
#define VBD_MV_CGE_31		/* Specific support for MontaVista CGE 3.1 */
#endif
#endif

/*----- System header files -----*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
#include <linux/freezer.h>
#endif
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/hdreg.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>
#include <linux/major.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
#include <linux/devfs_fs_kernel.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,7)
    /* This include exists in 2.6.6 but functions are not yet exported */
#include <linux/kthread.h>
#endif
#ifdef VBD_MV_CGE_31
#include <linux/kobject.h>
#endif
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#include <linux/blk.h>
#include <linux/tqueue.h>
#include <linux/signal.h>
#endif
#include <linux/cdrom.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <scsi/scsi.h>
#include <linux/blkpg.h>	/* blk_ioctl() */
#include <linux/proc_fs.h>
#include <asm/io.h>		/* ioremap */
#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,6,27)
#include <linux/semaphore.h>
#endif
#include <linux/init.h>		/* module_init() in 2.6.0 and before */

#include <nk/nkern.h>
#include <nk/rdisk.h>
#include "vlx-vmq.h"

/*----- Local configuration -----*/

#if 1
#define VBD_DEBUG
#endif

#if 1
#define VBD_DEBUG_NOISY
#endif

#if 1
#define VBD_DEBUG_STATS
#endif

#if 1
#define VBD_DEBUG_REQUESTS
#endif

#if 1
#define VBD_DEBUG_OBSOLETE
#endif

    /* Select at most one of the histogram-enabling #defines below */
#if 0
#define VBD_DEBUG_HISTO_BHS
#elif 0
#define VBD_DEBUG_HISTO_SEGMENTS
#elif 1
#define VBD_DEBUG_HISTO_SECTORS
#endif

    /*
     * For convenience we distinguish between ide, scsi and 'other' (i.e.
     * potentially combinations of the two) in the naming scheme and in a few
     * other places (like default readahead, etc).
     */
#define VBD_NUM_IDE_MAJORS	10
#define VBD_NUM_SCSI_MAJORS	9
#define VBD_NUM_VBD_MAJORS	1
#define VBD_NUM_FD_MAJORS	1
#ifdef MMC_BLOCK_MAJOR
#define VBD_NUM_MMC_MAJORS	1
#else
#define VBD_NUM_MMC_MAJORS	0
#endif

#define VBD_LINK_MAX_DISKS		64
#define	VBD_LINK_MAX_SEGS_PER_REQ	128
#define VBD_LINK_DEFAULT_MSG_COUNT	64

/*----- Tracing -----*/

#define TRACE(_f, _a...)  printk (KERN_INFO    "VBD2-FE: " _f, ## _a)
#define WTRACE(_f, _a...) printk (KERN_WARNING "VBD2-FE: " _f, ## _a)
#define ETRACE(_f, _a...) printk (KERN_ERR     "VBD2-FE: " _f, ## _a)

#ifdef	VBD_DEBUG
#define DTRACE(_f, _a...) \
	do {printk (KERN_ALERT "%s: " _f, __func__, ## _a);} while (0)
#define XTRACE(_f, _a...) DTRACE (_f,  ## _a)
#define VBD_CATCHIF(cond,action)	if (cond) action;
#define VBD_ASSERT(c)	do {if (!(c)) BUG();} while (0)
#else
#define DTRACE(_f, _a...) ((void)0)
#define XTRACE(_f, _a...) ((void)0)
#define VBD_CATCHIF(cond,action)
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

/*----- Data types -----*/

typedef struct {
    int		partn_shift;
    int		devs_per_major;
    const char*	name;
} vbd_type_info_t;

   /*
    * We have one of these per vbd, whether ide, scsi or 'other'.
    * They hang in private_data off the gendisk structure.
    */
typedef struct {
    int				major;
    int				usage;
    const vbd_type_info_t*	type;
    int				index;
} vbd_major_info_t;

typedef struct vbd_disk_info {
    int				xd_device;
    vbd_major_info_t*		mi;
    struct gendisk*		gd;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    struct request_queue*	gd_queue;
    _Bool			req_is_pending;
    _Bool			is_part;
    devfs_handle_t		devfs_handle;
#endif
    int				device;
    struct vbd_disk_info*	next;
} vbd_disk_info_t;

/*----- Const data -----*/

    static const vbd_type_info_t
vbd_ide_type = {
    .partn_shift = 6,
	 /* XXXcl todo blksize_size [major]  = 1024; */
    .devs_per_major = 2,
	/* XXXcl todo read_ahead [major] = 8; from drivers/ide/ide-probe.c */
    .name = "hd"
};

    static const vbd_type_info_t
vbd_scsi_type = {
    .partn_shift = 4,
	/* XXXcl todo blksize_size [major]  = 1024; XXX 512; */
    .devs_per_major = 16,
	/* XXXcl todo read_ahead [major]    = 0; XXX 8; -- guessing */
    .name = "sd"
};

    static const vbd_type_info_t
vbd_vbd_type = {
    .partn_shift = 4,
	/* XXXcl todo blksize_size [major]  = 512; */
	/* XXXcl todo read_ahead [major]    = 8; */
    .name = "xvd"
};

    static const vbd_type_info_t
vbd_fd_type = {
    .partn_shift = 0,
    .devs_per_major = 1,
    .name = "fd"
};

#ifdef MMC_BLOCK_MAJOR
    static vbd_type_info_t
vbd_mmc_type = {
    .partn_shift = 3,
    .devs_per_major = 256 >> 3,
    .name = "mmc"
};
#endif

/*----- Data -----*/

    static vbd_major_info_t*
vbd_major_info [VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS +
		VBD_NUM_VBD_MAJORS + VBD_NUM_FD_MAJORS + VBD_NUM_MMC_MAJORS];

#define VBD_MAJOR_XEN(dev)	((dev) >> 8)
#define VBD_MINOR_XEN(dev)	((dev) & 0xff)

typedef enum {
    VBD_LINK_STATE_CLOSED,
    VBD_LINK_STATE_DISCONNECTED,
    VBD_LINK_STATE_CONNECTED
} vbd_link_state_t;

static const char vbd_link_state [3] [4] = {"Clo", "Dis", "Con"};

typedef struct vbd_link_t {
    vmq_link_t*		link;
    u32			msg_max;
    int			ndisks;		/* number of disks */
    vbd_disk_info_t*	disks;
    RDiskProbe*		probe_info;	/* disk info */
    vbd_link_state_t	state;
    spinlock_t		io_lock;
    struct semaphore	probe_sem;
    RDiskResp		probe_resp;
} vbd_link_t;

#define VBD_LINK_FOR_ALL_DISKS(_di,_vbd) \
    for ((_di) = (_vbd)->disks; (_di); (_di) = (_di)->next)

static volatile _Bool	vbd_wait_flag;
static RDiskDevId	vbd_wait_id;

#define	VBD_LINK_MAX_SEGMENTS_PER_REQUEST(vbd) \
    (((vbd)->msg_max - sizeof (RDiskReqHeader)) / sizeof (RDiskBuffer))

    /* We plug the I/O ring if the driver is suspended or out of msgs */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
static int  vbd_blkif_open    (struct block_device* bdev, fmode_t fmode);
static int  vbd_blkif_release (struct gendisk* gd, fmode_t fmode);
static int  vbd_blkif_ioctl   (struct block_device* bdev, fmode_t fmode,
			       unsigned command, unsigned long argument);
#else
static int  vbd_blkif_open    (struct inode* inode, struct file* filep);
static int  vbd_blkif_release (struct inode* inode, struct file* filep);
static int  vbd_blkif_ioctl   (struct inode* inode, struct file* filep,
			       unsigned command, unsigned long argument);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
static int  vbd_blkif_getgeo  (struct block_device*, struct hd_geometry*);
#endif

static void vbd_do_blkif_request (struct request_queue* rq);

    static struct block_device_operations
vbd_block_fops = {
    .owner   = THIS_MODULE,
    .open    = vbd_blkif_open,
    .release = vbd_blkif_release,
    .ioctl   = vbd_blkif_ioctl,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
    .getgeo  = vbd_blkif_getgeo
#endif
};

/*----- Functions -----*/

    /* Called from vbd_link_get_disk_info() only */

    static int
vbd_link_probe_send (vbd_link_t* const vbd, const NkPhAddr paddr)
{
    RDiskReqHeader* rreq;
    int diag;

    diag = vmq_msg_allocate (vbd->link, 0 /*data_len*/, (void**) &rreq,
			     NULL /*data_offset*/);
    if (diag) return diag;
    rreq->op         = RDISK_OP_PROBE;
    rreq->sector [0] = 0;
    rreq->sector [1] = 0;
    rreq->devid      = 0;
    rreq->cookie     = 0;
    rreq->count      = 1;
    RDISK_FIRST_BUF (rreq) [0] = RDISK_BUFFER (paddr, 0, 7);
    vmq_msg_send (vbd->link, rreq);
    return 0;
}

    /* Called from vbd_link_acquire_disks() only */

    static int
vbd_link_get_disk_info (vbd_link_t* const vbd, RDiskProbe* const disk_info)
{
	//XXX: We should use pmem, either a call descriptor or a data buffer
    RDiskProbe* buf = (RDiskProbe*) __get_free_page (GFP_KERNEL);
    RDiskStatus status;
    int         count;

    count = vbd_link_probe_send (vbd, virt_to_phys (buf));
    if (count) return count;
    down (&vbd->probe_sem);
    status = vbd->probe_resp.status;
    if (status == RDISK_STATUS_ERROR) {
	ETRACE ("Could not probe disks (%d)\n", status);
	return -1;
    }
    if ((count = status) > VBD_LINK_MAX_DISKS) {
	count = VBD_LINK_MAX_DISKS;
    }
    TRACE ("%d virtual disk(s) detected\n", count);
    memcpy (disk_info, buf, count * sizeof (RDiskProbe));
    free_page ((unsigned long) buf);
    return count;
}

    /* Called from vbd_init_device() only */

    static vbd_major_info_t*
vbd_get_major_info (const int xd_device, int* const minor)
{
    const int xd_major = VBD_MAJOR_XEN (xd_device);
    const int xd_minor = VBD_MINOR_XEN (xd_device);
    int mi_idx, new_major;
    vbd_major_info_t* mi;

    *minor = xd_minor;

    switch (xd_major) {
    case IDE0_MAJOR: mi_idx = 0; new_major = IDE0_MAJOR; break;
    case IDE1_MAJOR: mi_idx = 1; new_major = IDE1_MAJOR; break;
    case IDE2_MAJOR: mi_idx = 2; new_major = IDE2_MAJOR; break;
    case IDE3_MAJOR: mi_idx = 3; new_major = IDE3_MAJOR; break;
    case IDE4_MAJOR: mi_idx = 4; new_major = IDE4_MAJOR; break;
    case IDE5_MAJOR: mi_idx = 5; new_major = IDE5_MAJOR; break;
    case IDE6_MAJOR: mi_idx = 6; new_major = IDE6_MAJOR; break;
    case IDE7_MAJOR: mi_idx = 7; new_major = IDE7_MAJOR; break;
    case IDE8_MAJOR: mi_idx = 8; new_major = IDE8_MAJOR; break;
    case IDE9_MAJOR: mi_idx = 9; new_major = IDE9_MAJOR; break;
    case SCSI_DISK0_MAJOR: mi_idx = 10; new_major = SCSI_DISK0_MAJOR;break;
    case SCSI_DISK1_MAJOR ... SCSI_DISK7_MAJOR:
	mi_idx = 11 + xd_major - SCSI_DISK1_MAJOR;
	new_major = SCSI_DISK1_MAJOR + xd_major - SCSI_DISK1_MAJOR;
	break;
    case SCSI_CDROM_MAJOR: mi_idx = 18; new_major = SCSI_CDROM_MAJOR;
	break;
    case FLOPPY_MAJOR: mi_idx = 19; new_major = FLOPPY_MAJOR; break;
#ifdef MMC_BLOCK_MAJOR
    case MMC_BLOCK_MAJOR: mi_idx = 20; new_major = MMC_BLOCK_MAJOR; break;
    default: mi_idx = 21; new_major = 0;/* XXXcl notyet */ break;
#else
    default: mi_idx = 20; new_major = 0;/* XXXcl notyet */ break;
#endif
    }
    if (vbd_major_info [mi_idx]) {
	return vbd_major_info [mi_idx];
    }
    mi = vbd_major_info [mi_idx] = kzalloc (sizeof *mi, GFP_KERNEL);
    if (!mi) {
	return NULL;
    }
    switch (mi_idx) {
    case 0 ... VBD_NUM_IDE_MAJORS - 1:
	mi->type = &vbd_ide_type;
	mi->index = mi_idx;
	break;

    case VBD_NUM_IDE_MAJORS ...  VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS - 1:
	mi->type = &vbd_scsi_type;
	mi->index = mi_idx - VBD_NUM_IDE_MAJORS;
	break;

    case VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS ...
	    VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS + VBD_NUM_FD_MAJORS - 1:
	mi->type = &vbd_fd_type;
	mi->index = mi_idx - (VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS);
	break;

#ifdef MMC_BLOCK_MAJOR
    case VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS + VBD_NUM_FD_MAJORS ...
	    VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS + VBD_NUM_FD_MAJORS +
	    VBD_NUM_MMC_MAJORS - 1:
	mi->type = &vbd_mmc_type;
	mi->index = mi_idx -
		(VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS + VBD_NUM_FD_MAJORS);
	break;
#endif
    case VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS + VBD_NUM_FD_MAJORS +
	    VBD_NUM_MMC_MAJORS ...
	    VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS + VBD_NUM_FD_MAJORS +
	    VBD_NUM_MMC_MAJORS + VBD_NUM_VBD_MAJORS - 1:
	mi->type = &vbd_vbd_type;
	mi->index = mi_idx -
		(VBD_NUM_IDE_MAJORS + VBD_NUM_SCSI_MAJORS +
		 VBD_NUM_FD_MAJORS + VBD_NUM_MMC_MAJORS);
	break;
    }
    mi->major = new_major;
    if (register_blkdev (mi->major, mi->type->name
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
			 , &vbd_block_fops
#endif
			 )) {
	ETRACE ("Cannot get major %d with name %s\n",
		mi->major, mi->type->name);
	goto out;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    devfs_mk_dir (mi->type->name);
#endif
#endif
    return mi;

out:
    kfree (vbd_major_info [mi_idx]);
    vbd_major_info [mi_idx] = NULL;
    return NULL;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    static struct gendisk*
vbd_get_gendisk (vbd_link_t* const vbd, vbd_major_info_t* const mi,
		 const int xd_minor, RDiskProbe* const xd)
{
    struct gendisk*        gd;
    vbd_disk_info_t*       di;
    int                    part;

    di = kzalloc (sizeof (vbd_disk_info_t), GFP_KERNEL);
    if (!di) {
	return NULL;
    }
	/* Construct an appropriate gendisk structure */
    part = xd_minor & ((1 << mi->type->partn_shift) - 1);
    if (part) {
	    /* VBD partitions are not expected to contain sub-partitions */
	DTRACE ("xd_minor=%d -> alloc_disk(%d)\n", xd_minor, 1);
	gd = alloc_disk (1);
    } else {
	    /* VBD disks can contain sub-partitions */
	DTRACE ("xd_minor=%d -> alloc_disk(%d)\n",
		xd_minor, 1 << mi->type->partn_shift);
	gd = alloc_disk (1 << mi->type->partn_shift);
    }
    if (!gd) goto out;

    gd->major        = mi->major;
    gd->first_minor  = xd_minor;
    gd->fops         = &vbd_block_fops;
    gd->private_data = di;
    if (part) {
#ifdef MMC_BLOCK_MAJOR
	if (mi->major == MMC_BLOCK_MAJOR) {
	    sprintf (gd->disk_name, "mmcblk%dp%d",
		     xd_minor >> mi->type->partn_shift, part);
	} else
#endif
	{
	    sprintf (gd->disk_name, "%s%c%d", mi->type->name,
		     'a' + (mi->index << 1) +
		     (xd_minor >> mi->type->partn_shift), part);
	}
    } else {
	    /* Floppy disk special naming rules */
	if (!strcmp (mi->type->name, "fd")) {
	    sprintf (gd->disk_name, "%s%c", mi->type->name, '0');
#ifdef MMC_BLOCK_MAJOR
	} else if (mi->major == MMC_BLOCK_MAJOR) {
	    sprintf (gd->disk_name, "mmcblk%d",
		     xd_minor >> mi->type->partn_shift);
#endif
	} else {
	    sprintf (gd->disk_name, "%s%c", mi->type->name,
		     'a' + (mi->index << 1) +
		     (xd_minor >> mi->type->partn_shift));
	}
    }
    if (xd->size [0]) {
	set_capacity (gd, xd->size [0]);	/* VG_FIXME: size [1] ??? */
    } else {
	set_capacity (gd, 0xffffffffL);
    }
    DTRACE ("%s: gd=0x%p, di=0x%p, major=%d first minor=%d\n",
	    gd->disk_name, gd, di, gd->major, gd->first_minor);
    gd->queue = blk_init_queue (vbd_do_blkif_request, &vbd->io_lock);
    if (!gd->queue) goto out;

    gd->queue->queuedata = vbd;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9)
    elevator_init (gd->queue, "noop");
#else
    elevator_init (gd->queue, &elevator_noop);
#endif
	/*
	 * Turn off barking 'headactive' mode. We dequeue buffer
	 * heads as soon as we pass them to the back-end driver.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
    blk_queue_headactive (gd->queue, 0);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
    blk_queue_logical_block_size (gd->queue, 512);
#else
    blk_queue_hardsect_size (gd->queue, 512);
#endif

#if defined CONFIG_ARM && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	/* Limit max hw read size to 128 (255 loopback limitation) */
    blk_queue_max_sectors (gd->queue, 128);
#else
    blk_queue_max_sectors (gd->queue,
	  VBD_LINK_MAX_SEGMENTS_PER_REQUEST (vbd) * (PAGE_SIZE/512));
#endif

    blk_queue_segment_boundary (gd->queue, PAGE_SIZE - 1);
    blk_queue_max_segment_size (gd->queue, PAGE_SIZE);

    blk_queue_max_phys_segments (gd->queue,
		VBD_LINK_MAX_SEGMENTS_PER_REQUEST (vbd));
    blk_queue_max_hw_segments (gd->queue,
		VBD_LINK_MAX_SEGMENTS_PER_REQUEST (vbd));

    blk_queue_dma_alignment (gd->queue, 511);

    di->mi        = mi;
    di->xd_device = xd->devid;
    di->gd        = gd;
    di->device    = MKDEV (mi->major, xd_minor);
    di->next      = vbd->disks;
    vbd->disks    = di;
    add_disk (gd);
    return gd;

out:
    if (gd) del_gendisk (gd);
    kfree (di);
    return NULL;
}

#else	/* 2.4.x */

    /*
     *  Private gendisk->flags[] values.
     *  Linux 2.4 only defines GENHD_FL_REMOVABLE (1).
     *  More recent Linuxes define other flags too, which
     *  could clash with our private flags, except we do
     *  not use private flags on 2.6: we just use the
     *  official GENHD_FL_CD (8) in 2.6 specific code.
     */
#define GENHD_FL_VIRT_PARTNS	4	/* Are unit partitions virtual? */

    /* The below are for the generic drivers/block/ll_rw_block.c code */
static int vbd_ide_blksize_size [256];
static int vbd_ide_hardsect_size [256];
static int vbd_ide_max_sectors [256];
static int vbd_scsi_blksize_size [256];
static int vbd_scsi_hardsect_size [256];
static int vbd_scsi_max_sectors [256];
static int vbd_vbd_blksize_size [256];
static int vbd_vbd_hardsect_size [256];
static int vbd_vbd_max_sectors [256];
static int vbd_fd_blksize_size [256];
static int vbd_fd_hardsect_size [256];
static int vbd_fd_max_sectors [256];
#ifdef MMC_BLOCK_MAJOR
static int vbd_mmc_blksize_size [256];
static int vbd_mmc_hardsect_size [256];
static int vbd_mmc_max_sectors [256];
#endif

#define VBD_FD_DISK_MAJOR(M) ((M) == FLOPPY_MAJOR)
#ifdef MMC_BLOCK_MAJOR
#define VBD_MMC_DISK_MAJOR(M) ((M) == MMC_BLOCK_MAJOR)
#endif

    static struct gendisk*
vbd_get_gendisk (vbd_link_t* const vbd, vbd_major_info_t* const mi,
		 const int xd_minor, RDiskProbe* const xd)
{
    const int device = xd->devid;
    const int major  = MAJOR (device);
    const int minor  = MINOR (device);
    const int is_ide = IDE_DISK_MAJOR (major);
    const int is_scsi= SCSI_BLK_MAJOR (major);
    const int is_fd  = VBD_FD_DISK_MAJOR (major);
#ifdef MMC_BLOCK_MAJOR
    const int is_mmc  = VBD_MMC_DISK_MAJOR (major);
#endif
    struct gendisk* gd;
    vbd_disk_info_t* di;
    unsigned long capacity;
    int partno;
    int max_part;
    unsigned char buf [64];

    XTRACE ("dev=0x%x\n", device);
    max_part = 1 << mi->type->partn_shift;

	/* Construct an appropriate gendisk structure */
    if ((gd = get_gendisk (device)) == NULL) {
	gd = kzalloc (sizeof *gd, GFP_KERNEL);
	XTRACE ("gd=0x%p\n", gd);
	if (!gd) goto out;
	gd->major       = mi->major;
	    /*
	     * major_name is used by devfs as a directory to automatically
	     * create partition devices (and discs/disc? symlink).
	     */
	gd->major_name  = mi->type->name;
	gd->max_p       = max_part;
	gd->minor_shift = mi->type->partn_shift;
	gd->nr_real     = mi->type->devs_per_major;
	gd->next        = NULL;
	gd->fops        = &vbd_block_fops;
	    /*
	     * The sizes[] and part[] arrays hold the sizes and other
	     * information about every partition with this 'major' (i.e.
	     * every disk sharing the 8 bit prefix * max partns per disk).
	     */
#ifdef VBD_MV_CGE_31
	gd->sizes = kzalloc (256 * sizeof (int), GFP_KERNEL);
#else
	gd->sizes = kzalloc (max_part * gd->nr_real * sizeof (int), GFP_KERNEL);
#endif
	if (!gd->sizes) goto out;

#ifdef VBD_MV_CGE_31
	gd->part  = kzalloc (256 * sizeof (struct hd_struct), GFP_KERNEL);
#else
	gd->part  = kzalloc (max_part * gd->nr_real * sizeof (struct hd_struct),
			     GFP_KERNEL);
#endif
	if (!gd->part) goto out;
	    /* VBD private data (disk info) */
	gd->real_devices = kzalloc (max_part * gd->nr_real *
				    sizeof (vbd_disk_info_t), GFP_KERNEL);
	if (!gd->real_devices) goto out;

	gd->flags = kzalloc (gd->nr_real * sizeof *gd->flags, GFP_KERNEL);
	if (!gd->flags) goto out;

#ifdef VBD_MV_CGE_31
	{
	    int unit;

	    gd->kobj = kzalloc (256 * sizeof (struct kobject), GFP_ATOMIC);
	    if (!gd->kobj) goto out;

	    for (unit = 0; unit < gd->nr_real; ++unit) {
		sprintf (gd->kobj [unit].name, "%s%c", mi->type->name,
			 'a' + (mi->index * gd->nr_real) +
			 (xd_minor >> mi->type->partn_shift));
	    }
	}
#endif
	if (is_ide) {
	    blksize_size [major]  = vbd_ide_blksize_size;
	    hardsect_size [major] = vbd_ide_hardsect_size;
	    max_sectors [major]   = vbd_ide_max_sectors;
	    read_ahead [major]    = 8; /* From drivers/ide/ide-probe.c */
	} else if (is_scsi) {
	    blksize_size [major]  = vbd_scsi_blksize_size;
	    hardsect_size [major] = vbd_scsi_hardsect_size;
	    max_sectors [major]   = vbd_scsi_max_sectors;
	    read_ahead [major]    = 0; /* XXX 8; -- guessing */
	} else if (is_fd) {
	    blksize_size [major]  = vbd_fd_blksize_size;
	    hardsect_size [major] = vbd_fd_hardsect_size;
	    max_sectors [major]   = vbd_fd_max_sectors;
	    read_ahead [major]    = 0;
#ifdef MMC_BLOCK_MAJOR
	} else if (is_mmc) {
	    blksize_size [major]  = vbd_mmc_blksize_size;
	    hardsect_size [major] = vbd_mmc_hardsect_size;
	    max_sectors [major]   = vbd_mmc_max_sectors;
	    read_ahead [major]    = 0;
#endif
	} else {
	    blksize_size [major]  = vbd_vbd_blksize_size;
	    hardsect_size [major] = vbd_vbd_hardsect_size;
	    max_sectors [major]   = vbd_vbd_max_sectors;
	    read_ahead [major]    = 8;
	}
	{
		/* Make sure Linux does not make single-segment requests */
	    int minor2;

	    for (minor2 = 0; minor2 < 256; ++minor2) {
		    /* Maximum number of sectors per request */
		max_sectors [major] [minor2] =
		    VBD_LINK_MAX_SEGMENTS_PER_REQUEST (vbd) * (PAGE_SIZE/512);

#ifndef VBD_MV_CGE_31
		    /*
		     * On RHEL3, we can get a "too many segments in req (128)"
		     * panic on filesystems with 1KB blocks (BugId 4676706).
		     * 128 is VBD_LINK_MAX_SEGMENTS_PER_REQUEST.
		     * Observation shows
		     * that Linux uses a different memory page and a different
		     * buffer head for every block in the request, that there
		     * can be more buffer heads than 128, and that after
		     * coalescing same-page buffer heads here in the driver,
		     * we always get 129 pages, which is still greater than
		     * the backend's limit of 128, hence panic. We try to
		     * avoid this situation by limiting the transfer size to
		     * 128KB, which with 1KB blocks can result in maximum
		     * 128 chunks. This is not ideal, since the problem can
		     * reappear if the filesystem uses 512-byte blocks, or if
		     * for some reason (I don't know if this is possible) an
		     * I/O request is performed as a long list of sector-sized
		     * buffer heads.
		     */
		max_sectors [major] [minor2] /= (PAGE_SIZE/1024);
#endif
	    }
	    TRACE ("max_sectors for major %d is %d\n", major,
		   max_sectors [major][0]);
	}
	add_gendisk (gd);

	blk_size [major] = gd->sizes;
	((struct request_queue*) BLK_DEFAULT_QUEUE (major))->queuedata = vbd;
	blk_init_queue (BLK_DEFAULT_QUEUE (major), vbd_do_blkif_request);

#ifdef VBD_DEBUG_OBSOLETE
#ifndef VBD_MV_CGE_31
    {
	int new_max_segments = VBD_LINK_MAX_SEGMENTS_PER_REQUEST (vbd) - 10;

	TRACE ("Changing max_segments for major %d from %d to %d\n", major,
	       (BLK_DEFAULT_QUEUE (major))->max_segments, new_max_segments);
	(BLK_DEFAULT_QUEUE (major))->max_segments = new_max_segments;
    }
#endif
#endif
	    /*
	     * Turn off barking 'headactive' mode. We dequeue buffer
	     * heads as soon as we pass them to the back-end driver.
	     */
	blk_queue_headactive (BLK_DEFAULT_QUEUE (major), 0);
    }
    partno = minor & (max_part-1);
	/* Private data */
    di = (vbd_disk_info_t*) gd->real_devices + minor;
    XTRACE ("di=0x%p mi=0x%p\n", di, mi);
    di->mi        = mi;
    di->xd_device = xd->devid;
    di->gd_queue  = BLK_DEFAULT_QUEUE (major);
    di->gd        = gd;
    di->device    = MKDEV (mi->major, minor);
    di->next      = vbd->disks;
    vbd->disks    = di;

    if (xd->info & RDISK_FLAG_RO) {
	set_device_ro (device, 1);
    }
    if (xd->size [0]) {
	capacity = xd->size [0];	/* VG_FIXME: size [1] ??? */
    } else {
	capacity = 0xffffffff;
    }
    if (partno) {
	    /*
	     * If this was previously set up as a real disc we will have set
	     * up partition-table information. Virtual partitions override
	     * 'real' partitions, and the two cannot coexist on a device.
	     */
	if (!(gd->flags [minor >> gd->minor_shift] & GENHD_FL_VIRT_PARTNS) &&
	     (gd->sizes [minor & ~(max_part-1)] != 0)) {
		/*
		 * Any non-zero sub-partition entries must be cleaned out
		 * before installing 'virtual' partition entries. The two
		 * types cannot coexist, and virtual partitions are favored.
		 */
	    int i;
	    kdev_t dev = device & ~(max_part-1);

	    for (i = max_part - 1; i > 0; i--) {
		invalidate_device (dev + i, 1);
		gd->part [MINOR (dev + i)].start_sect = 0;
		gd->part [MINOR (dev + i)].nr_sects   = 0;
		gd->sizes [MINOR (dev + i)]           = 0;
	    }
	    WTRACE ("Virtual partitions found for /dev/%s - ignoring any "
		    "real partition information we may have found.\n",
		    disk_name (gd, MINOR (device), buf));
	}
	    /* Need to skankily setup 'partition' information */
	gd->part [minor].start_sect = 0;
	gd->part [minor].nr_sects   = capacity;
	gd->sizes [minor]           = capacity >> (BLOCK_SIZE_BITS-9);
	gd->flags [minor >> gd->minor_shift] |= GENHD_FL_VIRT_PARTNS;
	DTRACE ("devfs_register(%s)\n", disk_name (gd, minor, buf));
	di->devfs_handle = devfs_register (NULL, disk_name (gd, minor, buf),
					   0, major, minor,
					   S_IFBLK|S_IRUSR|S_IWUSR|S_IRGRP,
					   &vbd_block_fops, di);
    } else {
	char short_name [8];
	    /* Get disk short name. Must be done before register_disk() */
	if (is_fd) {
	    sprintf (short_name, "%s%c", gd->major_name, '0' + minor);
	    xd->info = (xd->info & ~RDISK_TYPE_MASK) | RDISK_TYPE_FLOPPY;
	} else {
	    (void) disk_name (gd, minor, short_name);
	}
	DTRACE ("short_name %s\n", short_name);

	gd->part [minor].nr_sects = capacity;
	gd->sizes [minor] = capacity >> (BLOCK_SIZE_BITS-9);
	    /* Some final fix-ups depending on the device type */
	switch (RDISK_TYPE (xd->info)) {
	case RDISK_TYPE_CDROM:
	case RDISK_TYPE_FLOPPY:
	case RDISK_TYPE_TAPE:
	    gd->flags [minor >> gd->minor_shift] |= GENHD_FL_REMOVABLE;
	    WTRACE ("Skipping partition check on %s /dev/%s\n",
		    RDISK_TYPE (xd->info)==RDISK_TYPE_CDROM ? "cdrom" :
		   (RDISK_TYPE (xd->info)==RDISK_TYPE_TAPE ? "tape" :
		    "floppy"), short_name);
	    break;

	case RDISK_TYPE_DISK:
		/* Only check partitions on real discs (not virtual) */
	    if (gd->flags [minor >> gd->minor_shift] & GENHD_FL_VIRT_PARTNS) {
		WTRACE ("Skipping partition check on virtual /dev/%s\n",
			disk_name (gd, MINOR (device), buf));
		break;
	    }
	    XTRACE ("register_disk: 0x%x %d %ld %d(%d %d %d %d)\n",
		    device,gd->max_p,capacity,
		    (mi->index * gd->nr_real) + (minor >> gd->minor_shift),
		    mi->index, gd->nr_real, minor, gd->minor_shift);

	    register_disk (gd, device, gd->max_p, &vbd_block_fops, capacity
#ifdef VBD_MV_CGE_31
			  ,(mi->index * gd->nr_real) + (minor >>gd->minor_shift)
#endif
			  );
		/*
		 * register_disk() has checked for internal disk partitions
		 * and initialized gd->part[] accordingly.
		 * For each valid partition:
		 * - we duplicate the disk_info (di) structure created for the
		 *   disk (minor 0)
		 * - we update the copy to indicate it is a "partition"
		 *   in particular the copy (p_di) is not linked into the
		 *   disk list
		 *   Therefore, the next pointer is used to point to the
		 *   di of the
		 *   full disk. This pointer will be used to set the
		 *   'req_is_pending' field in the parent disk di when a
		 *   ring full condition is encountered (and not in the copy).
		 */
	    {
		int i;
		kdev_t dev = device & ~(max_part-1);
		char symlink [8];

		for (i = max_part - 1; i > 0; i--) {
		    vbd_disk_info_t*  p_di;
		    int               p_min = MINOR (dev + i);

		    if (gd->part [p_min].nr_sects) {
			DTRACE ("part %s: di %d -> %d\n",
				disk_name (gd, p_min, buf), minor, p_min);
			p_di  = (vbd_disk_info_t*)(gd->real_devices) + p_min;
			*p_di = *di;
			p_di->device  = MKDEV (mi->major, p_min);
			p_di->is_part = 1;  /* Set "partition" flag */
			p_di->next    = di; /* Point to parent disk di */
			    /*
			     * Add short name devices as symlink to
			     * the partition in devfs.
			     */
			sprintf (symlink, "%s%d", short_name, p_min);
			DTRACE ("devfs_mk_symlink(%s -> %s)\n",
				symlink, disk_name (gd, p_min, buf));
			devfs_mk_symlink (NULL, symlink, DEVFS_FL_DEFAULT,
					  disk_name (gd, p_min, buf),
					  &p_di->devfs_handle, NULL);
		    }
		}
		DTRACE ("devfs_mk_symlink(%s -> %s)\n",
			short_name, disk_name (gd, minor, buf));
		devfs_mk_symlink (NULL, short_name, DEVFS_FL_DEFAULT,
				  disk_name (gd, minor, buf),
				  &di->devfs_handle, NULL);
	    }
	    break;

	default:
	    WTRACE ("unknown device type %d\n", RDISK_TYPE (xd->info));
	    break;
	}
    }
    return gd;

out:
    if (gd) {
	del_gendisk (gd);
	kfree (gd->sizes);
	kfree (gd->part);
	kfree (gd->real_devices);
	kfree (gd->flags);
#ifdef VBD_MV_CGE_31
	kfree (gd->kobj);
#endif
    }
    return NULL;
}

#endif	/* 2.4.x */

    /*
     * vbd_init_device - initialize a VBD device
     * @disk:              a RDiskProbe describing the VBD
     *
     * Takes a RDiskProbe* that describes a VBD the guest has access to.
     * Performs appropriate initialization and registration of the device.
     *
     * Care needs to be taken when making re-entrant calls to ensure that
     * corruption does not occur.  Also, devices that are in use should not have
     * their details updated.  This is the caller's responsibility.
     */
    static int
vbd_init_device (vbd_link_t* const vbd, RDiskProbe* const xd)
{
    int err = -ENOMEM;
    struct block_device* bd;
    struct gendisk* gd;
    vbd_major_info_t* mi;
    int device;
    int minor;

    XTRACE ("\n");
    mi = vbd_get_major_info (xd->devid, &minor);
    if (mi == NULL) {
	return -EPERM;
    }
    device = MKDEV (mi->major, minor);
    if ((bd = bdget (device)) == NULL) {
	return -EPERM;
    }
	/*
	 * Update of partition info, and check of usage count,
	 * is protected by the per-block-device semaphore.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    down (&bd->bd_sem);
#endif
    gd = vbd_get_gendisk (vbd, mi, minor, xd);
    if (!gd || !mi) {
	err = -EPERM;
	goto out;
    }
    mi->usage++;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    if (xd->info & RDISK_FLAG_RO) {
	set_disk_ro (gd, 1);
    }
	/* Some final fix-ups depending on the device type */
    switch (RDISK_TYPE (xd->info)) {
    case RDISK_TYPE_CDROM:
	gd->flags |= GENHD_FL_REMOVABLE | GENHD_FL_CD;
	/* FALLTHROUGH */
    case RDISK_TYPE_FLOPPY:
    case RDISK_TYPE_TAPE:
	gd->flags |= GENHD_FL_REMOVABLE;
	break;

    case RDISK_TYPE_DISK:
	break;

    default:
	ETRACE ("unknown device type %d\n", RDISK_TYPE (xd->info));
	break;
    }
#endif
    if (vbd_wait_flag && vbd_wait_id == xd->devid) {
	vbd_wait_flag = 0;
    }
    err = 0;
out:
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    up (&bd->bd_sem);
#endif
    bdput (bd);
    return err;
}

    static void
vbd_destroy_device (vbd_disk_info_t* const di)
{
    vbd_major_info_t* mi = di->mi;
    struct block_device* bd;

    XTRACE ("\n");
    bd = bdget (di->device);
    if (bd) {
	if (di->gd) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	    if (di->gd->queue) {
		blk_cleanup_queue (di->gd->queue);
	    }
#else
	    if (di->gd_queue) {
		blk_cleanup_queue (di->gd_queue);
	    }
#endif
	    del_gendisk (di->gd);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	    put_disk (di->gd);
#endif
	}
	bdput (bd);
    }
    if (!(--(mi->usage))) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
	devfs_remove (mi->type->name);
#endif
#else
	devfs_unregister (di->devfs_handle);
#endif
	unregister_blkdev (mi->major, mi->type->name);
	kfree (mi);
    }
    kfree (di);
}

    /* Called from vbd_link_init() only */

    static int
vbd_link_acquire_disks (vbd_link_t* vbd)
{
    XTRACE ("\n");
    vbd->probe_info = kmalloc (VBD_LINK_MAX_DISKS * sizeof (RDiskProbe),
			       GFP_KERNEL);
    if (!vbd->probe_info) {
	ETRACE ("kmalloc(%lld) failure\n",
	        (u64) (VBD_LINK_MAX_DISKS * sizeof (RDiskProbe)));
	return -EAGAIN;
    }
    vbd->ndisks = vbd_link_get_disk_info (vbd, vbd->probe_info);
    if (vbd->ndisks < 0) {
	DTRACE ("vbd_link_get_disk_info failed\n");
	kfree (vbd->probe_info);
	vbd->probe_info = NULL;
	vbd->ndisks = 0;
    } else {
	int i;

	for (i = 0; i < vbd->ndisks; i++) {
	    int err = vbd_init_device (vbd, vbd->probe_info + i);
	    if (err) {
		ETRACE ("device (%d,%d) creation failure %d\n",
			VBD_MAJOR_XEN (vbd->probe_info [i].devid),
			VBD_MINOR_XEN (vbd->probe_info [i].devid), err);
	    } else {
		DTRACE ("device (%d,%d) created\n",
			VBD_MAJOR_XEN (vbd->probe_info [i].devid),
			VBD_MINOR_XEN (vbd->probe_info [i].devid));
	    }
	}
    }
    return 0;
}

    static void
vbd_link_down (vbd_link_t* vbd)
{
    DTRACE ("\n");
    while (vbd->disks) {
	vbd_disk_info_t* di = vbd->disks;
	vbd->disks = di->next;
	vbd_destroy_device (di);
    }
    kfree (vbd->probe_info);
    vbd->probe_info = NULL;
}

    static void
vbd_update (void)
{
    XTRACE ("\n");
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)

    /*
     * "kick" means "resume" here.
     * Only called from the function below.
     */

    static inline void
vbd_kick_pending_request_queue (struct request_queue* rq)
{
    if (rq && test_bit (QUEUE_FLAG_STOPPED, &rq->queue_flags)) {
	blk_start_queue (rq);
	rq->request_fn (rq);
    }
}

    /* Called from interrupt handler only, under vbd->io_lock */

    static void
vbd_kick_pending_request_queues (vbd_link_t* vbd)
{
    vbd_disk_info_t* di;

    VBD_LINK_FOR_ALL_DISKS (di, vbd) {
	if (di->gd) {
	    vbd_kick_pending_request_queue (di->gd->queue);
	}
    }
}

    static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
vbd_blkif_open (struct block_device* bdev, fmode_t fmode)
#else
vbd_blkif_open (struct inode* inode, struct file* filep)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
    struct gendisk* gd = bdev->bd_disk;
#else
    struct gendisk* gd = inode->i_bdev->bd_disk;
#endif
    vbd_disk_info_t* di = (vbd_disk_info_t*) gd->private_data;

    DTRACE ("gd=0x%p, di=0x%p", gd, di);
	/* Update of usage count is protected by per-device semaphore */
    di->mi->usage++;
    return 0;
}

    static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
vbd_blkif_release (struct gendisk* gd, fmode_t fmode)
#else
vbd_blkif_release (struct inode* inode, struct file* filep)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    struct gendisk* gd = inode->i_bdev->bd_disk;
#endif
    vbd_disk_info_t* di = (vbd_disk_info_t*) gd->private_data;
	/*
	 * When usage drops to zero it may allow more VBD updates to occur.
	 * Update of usage count is protected by a per-device semaphore.
	 */
    if (--di->mi->usage == 0) {
	vbd_update();
    }
    return 0;
}

#else	/* 2.4.x kernel versions */

#ifdef VBD_DEBUG_STATS
struct {
    int sectors_per_buffer_head [8];
    int new_bh_same_page [2];
    int len_histo [1024];  /* Histogram of buffer_heads, segments or sectors */
    int was_already_pending;
    int was_not_not_yet_pending;
    int was_pending_at_irq;
    int not_pending_at_irq;
    int ring_not_full;
} vbd_stats;
#endif

#define VBD_ARRAY_ELEMS(a)	(sizeof (a) / sizeof (a) [0])

    /*
     * "kick" means "resume" here.
     * Called from interrupt handler only, under vbd->io_lock.
     */

    static void
vbd_kick_pending_request_queues (vbd_link_t* vbd)
{
#ifdef VBD_DEBUG_NOISY
    static long next_jiffies;
#endif
    vbd_disk_info_t* di;

    VBD_LINK_FOR_ALL_DISKS (di, vbd) {
	    /*
	     *  req_is_pending was set in previous vbd_do_blkif_request()
	     *  and means that FIFO was full.
	     *  "di" can only be a real disk here, or maybe a standalone
	     *  partition. Indeed, di's for partitions are not put on the
	     *  vbd->disks list if they have been found by Linux.
	     */
	VBD_CATCHIF (!di->gd,       WTRACE ("<!di->gd>"));
	VBD_CATCHIF (!di->gd_queue, WTRACE ("<!di->gd_queue>"));

#ifdef VBD_DEBUG_STATS
	if (MAJOR (di->device) == 8) {
	    if (di->req_is_pending) {
		++vbd_stats.was_pending_at_irq;
	    } else {
		++vbd_stats.not_pending_at_irq;
	    }
	}
#endif

#ifdef VBD_DEBUG_NOISY
	if (MAJOR (di->device) == 8 && jiffies > next_jiffies) {
	    const int major = MAJOR (di->device);
	    const struct request_queue* q = BLK_DEFAULT_QUEUE (major);

	    WTRACE ("%s: di=0x%p dev %x pend %d\n", __FUNCTION__, di,
		    di->xd_device, di->req_is_pending);
	    VBD_ASSERT (q->queuedata == vbd);

#if LINUX_VERSION_CODE == KERNEL_VERSION(2,4,18)
	    printk ("queue %d: plugged %d head_active %d\n", major,
		    q->plugged, q->head_active);
#else
	    printk ("queue %d: nr_requests %d batch_requests %d plugged %d "
		    "head_active %d\n", major, q->nr_requests,
		    q->batch_requests, q->plugged, q->head_active);
#endif

#ifdef VBD_MV_CGE_31
	    printk ("queue %d: nr_sectors %d batch_sectors %d "
		    "max_queue_sectors %d full %d can_throttle %d "
		    "low_latency %d\n", major, atomic_read (&q->nr_sectors),
		    q->batch_sectors, q->max_queue_sectors, q->full,
		    q->can_throttle, q->low_latency);
#endif
	    next_jiffies = jiffies + 10 * HZ;
#ifdef VBD_DEBUG_STATS
	    printk ("AlrPend %d notyet %d pend@irq %d not %d !full %d\n",
		    vbd_stats.was_already_pending,
		    vbd_stats.was_not_not_yet_pending,
		    vbd_stats.was_pending_at_irq,
		    vbd_stats.not_pending_at_irq,
		    vbd_stats.ring_not_full);
	    printk ("Sectors per buf/head: %d %d %d %d  %d %d %d %d ",
		    vbd_stats.sectors_per_buffer_head [0],
		    vbd_stats.sectors_per_buffer_head [1],
		    vbd_stats.sectors_per_buffer_head [2],
		    vbd_stats.sectors_per_buffer_head [3],
		    vbd_stats.sectors_per_buffer_head [4],
		    vbd_stats.sectors_per_buffer_head [5],
		    vbd_stats.sectors_per_buffer_head [6],
		    vbd_stats.sectors_per_buffer_head [7]);
	    printk (" SamePage %d %d\n",
		    vbd_stats.new_bh_same_page [0],
		    vbd_stats.new_bh_same_page [1]);
	    {
		int i;

		for (i = 0; i < VBD_ARRAY_ELEMS (vbd_stats.len_histo);
		     i += 16) {
		    printk ("len_histo(%3d): %d %d %d %d  %d %d %d %d  "
			    "%d %d %d %d  %d %d %d %d\n", i,
			vbd_stats.len_histo [i+ 0], vbd_stats.len_histo [i+ 1],
			vbd_stats.len_histo [i+ 2], vbd_stats.len_histo [i+ 3],
			vbd_stats.len_histo [i+ 4], vbd_stats.len_histo [i+ 5],
			vbd_stats.len_histo [i+ 6], vbd_stats.len_histo [i+ 7],
			vbd_stats.len_histo [i+ 8], vbd_stats.len_histo [i+ 9],
			vbd_stats.len_histo [i+10], vbd_stats.len_histo [i+11],
			vbd_stats.len_histo [i+12], vbd_stats.len_histo [i+13],
			vbd_stats.len_histo [i+14], vbd_stats.len_histo [i+15]);
		}
	    }
#endif /* VBD_DEBUG_STATS */
	    {
		static _Bool already_done;
		int minor;

		if (!already_done) {
		    already_done = 1;

		    printk ("blk_size:");
		    for (minor = 0; minor < 256 && blk_size [8]; ++minor) {
			printk (" %d", blk_size [8] [minor]);
		    }
		    printk ("\n");

		    printk ("blksize_size:");
		    for (minor = 0; minor < 256 && blksize_size [8]; ++minor) {
			printk (" %d", blksize_size [8] [minor]);
		    }
		    printk ("\n");

		    printk ("hardsect_size:");
		    for (minor = 0; minor < 256 && hardsect_size [8]; ++minor) {
			printk (" %d", hardsect_size [8] [minor]);
		    }
		    printk ("\n");

		    printk ("max_readahead:");
		    for (minor = 0; minor < 256 && max_readahead [8]; ++minor) {
			printk (" %d", max_readahead [8] [minor]);
		    }
		    printk ("\n");

		    printk ("max_sectors:");
		    for (minor = 0; minor < 256 && max_sectors [8]; ++minor) {
			printk (" %d", max_sectors [8] [minor]);
		    }
		    printk ("\n");
		}
	    }
	}
#endif /* VBD_DEBUG_NOISY */
	if (di->gd && di->req_is_pending && di->gd_queue) {
	    XTRACE ("di=0x%p\n", di);
	    di->req_is_pending = 0;
	    vbd_do_blkif_request (di->gd_queue);
	}
    }
}

    /* Linux 2.4 only */

    static int
vbd_blkif_open (struct inode* inode, struct file* filep)
{
    struct gendisk*	gd = get_gendisk (inode->i_rdev);
    vbd_disk_info_t*	di;

    (void) filep;
    XTRACE ("i_rdev %x gd %p\n", inode->i_rdev, gd);
    VBD_ASSERT (gd);
    di = (vbd_disk_info_t*) gd->real_devices + MINOR (inode->i_rdev);
    VBD_ASSERT (di);
	/* We get a NULL here on RHEL3 when non-existing minor is opened */
    if (!di->mi) {
	WTRACE ("Open of non-existing device %x\n", inode->i_rdev);
	return -ENODEV;
    }
	/* Update of usage count is protected by per-device semaphore */
    di->mi->usage++;
    return 0;
}

    /* Linux 2.4 only */

    static int
vbd_blkif_release (struct inode* inode, struct file* filep)
{
    struct gendisk*	gd = get_gendisk (inode->i_rdev);
    vbd_disk_info_t*	di;

    (void) filep;
    XTRACE ("gd=0x%p\n", gd);
    VBD_ASSERT (gd);
    di = (vbd_disk_info_t*) gd->real_devices + MINOR (inode->i_rdev);
    XTRACE ("di=0x%p\n", di);
	/*
	 * When usage drops to zero it may allow more VBD updates to occur.
	 * Update of usage count is protected by a per-device semaphore.
	 */
    if (--di->mi->usage == 0) {
	vbd_update();
    }
    return 0;
}
#endif	/* 2.4.x */

    static int
vbd_blkif_ioctl (
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
		 struct block_device* bdev, fmode_t fmode,
#else
		 struct inode* inode, struct file* filep,
#endif
		 unsigned command, unsigned long argument)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
    XTRACE ("command: 0x%x, argument: 0x%lx, dev: 0x%04x\n",
	    command, (long) argument, bdev->bd_inode->i_rdev);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    XTRACE ("command: 0x%x, argument: 0x%lx, dev: 0x%04x\n",
	    command, (long) argument, inode->i_rdev);
#else
    const kdev_t dev = inode->i_rdev;
    const struct gendisk* gd = get_gendisk (dev);
    const int minor = MINOR (dev);
    const struct hd_struct* part = &gd->part [minor];

    XTRACE ("command: 0x%x, argument: 0x%lx, dev: 0x%04x\n",
	    command, (long) argument, inode->i_rdev);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
    (void) fmode;
#else
    (void) filep;
#endif

    switch (command) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	/* This is used by "fdisk -l" */
    case BLKGETSIZE:
	return put_user (part->nr_sects, (unsigned long*) argument);

	/* This is not used by "fdisk -l" */
    case BLKGETSIZE64:
	return put_user ((u64) part->nr_sects * 512, (u64*) argument);

#if 0
	/*
	 * This ioctl is not used by fdisk -l. Before activating this
	 * code, we need to bring blkif_revalidate() from an older
	 * version of this driver.
	 */
    case BLKRRPART: /* Re-read partition table */
	return blkif_revalidate (dev);
#endif

    case BLKROGET:	/* Used by "hdparm /dev/sda" */
    case BLKFLSBUF:	/* Used by "hdparm -f/-t /dev/sda" */
	    /* We could use this for most of ioctls here */
	return blk_ioctl (dev, command, argument);

	/* This is used by "fdisk -l" */
    case BLKSSZGET:
	    /*
	     * This value is 0 for now, because of still missing
	     * initialization code. This is fixed by routine.
	     */
	return blk_ioctl (dev, command, argument);

	/* These 4 ioctls are not used by "fdisk -l" */
    case BLKBSZGET: /* Get block size */
	return blk_ioctl (dev, command, argument);

    case BLKBSZSET: /* Set block size */
	break;

    case BLKRASET: /* Set read-ahead */
	break;

    case BLKRAGET: /* Get read-ahead */
	    /* Used by "hdparm -a /dev/sda" */
	return blk_ioctl (dev, command, argument);

    case HDIO_GETGEO:
    case HDIO_GETGEO_BIG: {
	struct hd_geometry *geo = (struct hd_geometry*) argument;
	const u8 heads = 0xff;
	const u8 sectors = 0x3f;
	const unsigned long start_sect = gd->part [minor].start_sect;
	unsigned cylinders;

	    /* This code is never reached on 2.6.15 */
	    /* This is used by "fdisk -l" and by hdparm */
	if (!argument) return -EINVAL;
	    /*
	     * We don't have real geometry info, but let's at least return
	     * values consistent with the size of the device
	     * We take the start_sect from the partition, but the size from
	     * the whole disk, except if it is zero, so this is a virtual
	     * partition, in which case we take its size.
	     */
	part = &gd->part [minor & ~((1 << gd->minor_shift)-1)];
	if (!part->nr_sects) {
	    part = &gd->part [minor];
	}
	cylinders = part->nr_sects / (heads * sectors);

	if (put_user (heads, &geo->heads)) return -EFAULT;
	if (put_user (sectors, &geo->sectors)) return -EFAULT;
	if (command == HDIO_GETGEO) {
	    if (put_user (cylinders, &geo->cylinders)) return -EFAULT;
	    if (put_user (start_sect, &geo->start)) return -EFAULT;
	} else {
	    struct hd_big_geometry* geobig = (struct hd_big_geometry*) argument;
	    if (put_user (cylinders, &geobig->cylinders)) return -EFAULT;
	    if (put_user (start_sect, &geobig->start)) return -EFAULT;
	}
	break;
    }
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0) */

    default:
	DTRACE ("ioctl %08x not supported\n", command);
	return -ENOSYS;
    }
    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
    /*
     *  Implement the HDIO_GETGEO ioctl: the HDIO_GETGEO
     *  value is not passed to the common vbd_blkif_ioctl() above
     *  on 2.6.15 and maybe other versions.
     *  We always return the full disk geometry, even if a
     *  partition has been opened. Only "start" changes.
     *  This is how the ioctl behaves on both 2.4 and 2.6
     *  Linuxes. This makes me wonder if the implementation
     *  on 2.4 in vbd_blkif_ioctl() is correct.
     */

    static int
vbd_blkif_getgeo (struct block_device* bdev, struct hd_geometry* geo)
{
    struct gendisk* gd = bdev->bd_disk;
    sector_t sectors = get_capacity (gd);

	/* geo->start was set by caller blkdev_ioctl() in block/ioctl.c */
#ifdef MMC_BLOCK_MAJOR
    if (gd->major == MMC_BLOCK_MAJOR) {
	geo->heads     = 4;
	geo->sectors   = 16;	/* Per track */
    } else
#endif
    {
	geo->heads     = 0xff;
	geo->sectors   = 0x3f;	/* Per track */
    }
	/*
	 *  get_capacity() returns an unsigned 64-bit sector_t.
	 *  Performing a division by heads*sectors generates an
	 *  unresolved call to __udivdi3(). Given that capacity
	 *  was earlier set as a 32 bit integer, we could safely
	 *  typecast it to 32 bits here. But we use sector_div()
	 *  for universality. It returns the remainder and modifies
	 *  its first parameter to be the result.
	 */
    sector_div (sectors, geo->heads * geo->sectors);
    geo->cylinders = sectors;
    return 0;
}
#endif

    /*
     * Request block io.
     * Called from vbd_do_blkif_request() only.
     *
     * id: for guest use only.
     * operation: RDISK_OP_{READ,WRITE,PROBE}
     * buffer: buffer to read/write into. This should be a
     * virtual address in the guest os.
     */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    static int
vbd_link_queue_request (vbd_link_t* vbd, struct request* req,
			RDiskReqHeader* rreq)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	/* blk_rq_pos() is an inline function returning req->__sector */
    const sector_t	req_sector = blk_rq_pos (req);
#else
    const sector_t	req_sector = req->sector;
#endif
    struct gendisk*	gd = req->rq_disk;
    vbd_disk_info_t*	di = (vbd_disk_info_t*) gd->private_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
    struct req_iterator iter;
#else
    struct bio*         bio;
    int                 idx;
#endif
    struct bio_vec*     bvec;
    struct page*        page;
    unsigned long       paddr;
    unsigned int        fsect, lsect, pfsect, plsect;
    u32		        count;

    if (unlikely (vbd->state != VBD_LINK_STATE_CONNECTED)) {
	return 1;
    }
    rreq->cookie    = (unsigned long) req;
    rreq->op        = rq_data_dir (req) ? RDISK_OP_WRITE : RDISK_OP_READ;
    XTRACE ("%c %0llx\n", rq_data_dir (req) ? 'w' : 'r',
	    (unsigned long long) req_sector);

    rreq->sector [0] = (RDiskSector) req_sector;
    rreq->sector [1] = (RDiskSector)(((u64) req_sector) >> 32);
    rreq->devid      = di->xd_device;

    page   = 0;
    count  = 0;
    paddr  = 0;
    plsect = 0;
    pfsect = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
    rq_for_each_segment (bvec, req, iter)
#else
    rq_for_each_bio (bio, req) {
	bio_for_each_segment (bvec, bio, idx)
#endif
	{
	    fsect = bvec->bv_offset >> 9;
	    lsect = fsect + (bvec->bv_len >> 9) - 1;
	    if (unlikely (lsect > 7)) {
		BUG();
	    }
	    if ((page == bvec->bv_page) && (fsect == (plsect+1))) {
		plsect = lsect;
		RDISK_FIRST_BUF (rreq) [count-1] =
		    RDISK_BUFFER (paddr, pfsect, plsect);
	    } else {
		if (unlikely (count==VBD_LINK_MAX_SEGMENTS_PER_REQUEST (vbd))) {
		    BUG();
	        }
		page   = bvec->bv_page;
		paddr  = page_to_phys (page);
		pfsect = fsect;
		plsect = lsect;
		RDISK_FIRST_BUF (rreq) [count++] =
		    RDISK_BUFFER (paddr, pfsect, plsect);
	    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	}
#endif
    }
    rreq->count = (RDiskCount) count;
    vmq_msg_send (vbd->link, rreq);
    return 0;
}

    /*
     * Linux 2.6 code.
     * Read a block. Request is in a request queue.
     * Called from the kernel only.
     */
    static void
vbd_do_blkif_request (struct request_queue* rq)
{
    struct request* req;
    vbd_link_t*     vbd = rq->queuedata;

    DTRACE ("entered, vbd=0x%p\n", vbd);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
    while ((req = blk_peek_request (rq)) != NULL)
#else
    while ((req = elv_next_request (rq)) != NULL)
#endif
    {
	RDiskReqHeader* rreq;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
	if (!blk_fs_request (req)) {
	    end_request (req, 0);
	    continue;
	}
#endif
	if (vmq_msg_allocate_ex (vbd->link, 0 /*data_len*/, (void**) &rreq,
				 NULL /*data_offset*/, 1 /*nonblocking*/)) {
	    blk_stop_queue (rq);
	    break;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	blk_start_request (req);

	if (!blk_fs_request (req)) {
	    __blk_end_request_all (req, -EIO);
	    vmq_return_msg_free (vbd->link, rreq);
	    continue;
	}
	DTRACE ("%p: cmd %p, sec %llx, (%u/%u) buffer:%p [%s]\n",
		req, req->cmd, (u64) blk_rq_pos (req),
		blk_rq_cur_sectors (req), blk_rq_sectors (req),
		req->buffer, rq_data_dir (req) ? "write" : "read");
#else
	DTRACE ("%p: cmd %p, sec %llx, (%u/%li) buffer:%p [%s]\n",
		req, req->cmd, (u64) req->sector, req->current_nr_sectors,
		req->nr_sectors, req->buffer,
		rq_data_dir (req) ? "write" : "read");

	blkdev_dequeue_request (req);
#endif
	if (vbd_link_queue_request (vbd, req, rreq)) {
	    ETRACE ("Badness in %s at %s:%d\n",
		    __FUNCTION__, __FILE__, __LINE__);
	    blk_stop_queue (rq);
	    vmq_return_msg_free (vbd->link, rreq);
	    break;
	}
    }
}

#else /* 2.4.x */

#ifdef VBD_DEBUG_REQUESTS
    static void
vbd_print_struct_request (const struct request* req)
{
    int			count = 0;
    unsigned long	paddr = 0;
    unsigned long	page = 0;
    unsigned int        plsect = 0;	/* Previous last sector */
    int			continued = 0;
    struct buffer_head*	bh;

    printk ("request %p: sector %lu cmd %d\n", req, req->sector, req->cmd);
    if (!req->bh) {
	printk ("req->bh NULL\n");
	return;
    }
    bh = req->bh;
    while (bh) {
	unsigned long b_paddr = virt_to_phys (bh->b_data);
	const unsigned fsect = (b_paddr & ~PAGE_MASK) >> 9;
	const unsigned lsect = fsect + (bh->b_size >> 9) - 1;

	printk ("%d: bh %p b_rsector %lu b_paddr %lx ", count++, bh,
		bh->b_rsector, b_paddr);
	printk ("b_size %d fsect %d lsect %d", bh->b_size, fsect, lsect);

	b_paddr &= PAGE_MASK;
	if ((page == b_paddr) && (fsect == (plsect+1))) {
		/* New buffer header is in same page, continuing old one */
	    plsect = lsect;
	    printk (" <CONT>");
	    ++continued;
	} else {
	    page   = b_paddr;
	    paddr  = page;
	    plsect = lsect;
	}
	printk ("\n");
	bh = bh->b_reqnext;
    }
    printk ("Continued: %d in %d\n", continued, count);
}
#endif

    /*
     * Linux 2.4 only.
     * Called from vbd_do_blkif_request() only.
     */

    static int
vbd_link_queue_request (vbd_link_t* vbd, const struct request* req,
			RDiskReqHeader*	rreq)
{
    const struct gendisk*	gd;
    const vbd_disk_info_t*	di;
    const struct buffer_head*	bh;
    unsigned long		page;
    unsigned long		paddr;
    unsigned int		fsect, lsect, pfsect, plsect;
    u32				count;
#ifdef VBD_DEBUG_HISTO_BHS
    int				histo_bhs = 0;
#endif
#ifdef VBD_DEBUG_HISTO_SECTORS
    int				histo_sectors = 0;
#endif
#ifdef VBD_DEBUG
    unsigned long		bh_sector;
#endif

    XTRACE ("vbd=0x%p\n", vbd);
    if (unlikely (vbd->state != VBD_LINK_STATE_CONNECTED)) {
	return 1;
    }
    if (unlikely (!req->bh)) {
	return 1;
    }
    gd = get_gendisk (req->rq_dev);
    di = (vbd_disk_info_t*) gd->real_devices + MINOR (req->rq_dev);
    VBD_ASSERT (di->gd == gd);

    rreq->cookie     = (unsigned long) req;
    rreq->op         = req->cmd == WRITE ? RDISK_OP_WRITE : RDISK_OP_READ;
    VBD_ASSERT (req->sector == req->bh->b_rsector);
    rreq->sector [0] = (RDiskSector)(req->bh->b_rsector +
				     gd->part [MINOR (req->rq_dev)].start_sect);
    rreq->sector [1] = 0;
    rreq->devid      = di->xd_device;

    page   = 0;
    count  = 0;
    paddr  = 0;
    plsect = 0;		/* Previous  last sector */
    pfsect = 0;		/* Previous first sector */

    bh = req->bh;
#ifdef VBD_DEBUG
    bh_sector = bh->b_rsector;
#endif
    while (bh) {
	unsigned long b_paddr = virt_to_phys (bh->b_data);

#ifdef VBD_DEBUG_HISTO_BHS
	++histo_bhs;
#endif
	VBD_ASSERT (bh->b_rsector == bh_sector);
	if (unlikely ((b_paddr & ((1<<9)-1)) != 0)) {
		/* Not sector aligned */
	    BUG();
	}
	fsect = (b_paddr & ~PAGE_MASK) >> 9;
	lsect = fsect + (bh->b_size >> 9) - 1;
	if (unlikely (lsect > 7)) {
	    BUG();
	}
#ifdef VBD_DEBUG_HISTO_SECTORS
	histo_sectors += bh->b_size >> 9;
#endif
#ifdef VBD_DEBUG_STATS
	VBD_ASSERT (lsect-fsect <= 7 && lsect-fsect >= 0);
	++vbd_stats.sectors_per_buffer_head [lsect-fsect];
#endif
	b_paddr &= PAGE_MASK;
	if ((page == b_paddr) && (fsect == (plsect+1))) {
		/* New buffer header is in same page, continuing old one */
	    plsect = lsect;
	    RDISK_FIRST_BUF (rreq) [count-1] =
		RDISK_BUFFER (paddr, pfsect, plsect);

#ifdef VBD_DEBUG_STATS
	    ++vbd_stats.new_bh_same_page [0];
#endif
	} else {
	    if (unlikely (count == VBD_LINK_MAX_SEGMENTS_PER_REQUEST (vbd))) {
		ETRACE ("too many segments in req (%d)\n", count);
#ifdef VBD_DEBUG_REQUESTS
		vbd_print_struct_request (req);
#endif
		BUG();
	    }
	    page   = b_paddr;
	    paddr  = page;
	    pfsect = fsect;
	    plsect = lsect;
	    RDISK_FIRST_BUF (rreq) [count++] =
		RDISK_BUFFER (paddr, pfsect, plsect);
#ifdef VBD_DEBUG_STATS
	    ++vbd_stats.new_bh_same_page [1];
#endif
	}
#ifdef VBD_DEBUG
	bh_sector += bh->b_size >> 9;
#endif
	bh = bh->b_reqnext;
    }
    rreq->count = (RDiskCount) count;

#ifdef VBD_DEBUG_STATS
#ifdef VBD_DEBUG_HISTO_BHS
    if (histo_bhs >= VBD_ARRAY_ELEMS (vbd_stats.len_histo)) {
	histo_bhs  = VBD_ARRAY_ELEMS (vbd_stats.len_histo) - 1;
    }
    ++vbd_stats.len_histo [histo_bhs];
#elif defined VBD_DEBUG_HISTO_SECTORS
    if (histo_sectors >= VBD_ARRAY_ELEMS (vbd_stats.len_histo)) {
	histo_sectors  = VBD_ARRAY_ELEMS (vbd_stats.len_histo) - 1;
    }
    ++vbd_stats.len_histo [histo_sectors];
#elif defined VBD_DEBUG_HISTO_SEGMENTS
	/* Only modify "count" after it has been used, above */
    if (count >= VBD_ARRAY_ELEMS (vbd_stats.len_histo)) {
	count  = VBD_ARRAY_ELEMS (vbd_stats.len_histo) - 1;
    }
    ++vbd_stats.len_histo [count];
#endif
#endif
    vmq_msg_send (vbd->link, rreq);
    return 0;
}

    /*
     *  Linux 2.4 only.
     *  Called by the kernel, and also specifically on 2.4,
     *  from vbd_kick_pending_request_queues(), after some place
     *  has just been made in a full FIFO.
     */

    static void
vbd_do_blkif_request (struct request_queue *rq)
{
    vbd_link_t*     vbd = rq->queuedata;
    struct request* req;

    DTRACE ("entered, vbd=0x%p\n", vbd);
    while (!rq->plugged && !list_empty (&rq->queue_head) &&
	   (req = blkdev_entry_next_request (&rq->queue_head)) != NULL) {
	RDiskReqHeader*	rreq;

	VBD_CATCHIF (rq != BLK_DEFAULT_QUEUE (MAJOR (req->rq_dev)),
	    WTRACE("<Non-def-queue:%d:%d>", MAJOR (req->rq_dev),
		  MINOR (req->rq_dev)));
	VBD_CATCHIF (((vbd_disk_info_t*) get_gendisk(req->rq_dev)->real_devices
	    + MINOR (req->rq_dev))->gd_queue !=
	    BLK_DEFAULT_QUEUE (MAJOR (req->rq_dev)),
	    WTRACE ("<Bad-gd_queue:%d:%d>", MAJOR (req->rq_dev),
		    MINOR (req->rq_dev)));

	if (vmq_msg_allocate_ex (vbd->link, 0 /*data_len*/, (void**) &rreq,
			         NULL /*data_offset*/, 1 /*nonblocking*/)) {
	    struct gendisk*	gd = get_gendisk (req->rq_dev);
	    vbd_disk_info_t*	di;

	    VBD_ASSERT (gd);
	    di = (vbd_disk_info_t*) gd->real_devices + MINOR (req->rq_dev);

	    if (di->is_part) {
		    /* We come here when accessing /dev/sdaX */
#ifdef VBD_DEBUG_STATS
		if (di->next->req_is_pending) {
		    ++vbd_stats.was_already_pending;
		} else {
		    ++vbd_stats.was_not_not_yet_pending;
		}
#endif
		di->next->req_is_pending = 1;
	    } else {
		    /* We come here when accessing /dev/sda */
		di->req_is_pending = 1;
	    }
	    XTRACE ("ring full! (di=0x%p)\n", di);
	    break;
	}
#ifdef VBD_DEBUG_STATS
	++vbd_stats.ring_not_full;
#endif
	DTRACE ("%p: cmd %x, sec %llx, (%lu/%li) buffer:%p [%s]\n",
		req, req->cmd, (u64) req->sector, req->current_nr_sectors,
		req->nr_sectors, req->buffer,
		req->cmd == WRITE ? "write" : "read");
	VBD_ASSERT ((req->cmd == READA) || (req->cmd == READ) ||
		    (req->cmd == WRITE));

	blkdev_dequeue_request (req);
	if (vbd_link_queue_request (vbd, req, rreq)) {
	    ETRACE ("Badness in %s at %s:%d\n",
		    __FUNCTION__, __FILE__, __LINE__);
	    vmq_return_msg_free (vbd->link, rreq);
	    break;
	}
    }
}
#endif	/* 2.4.x */

    static void
vbd_link_return_msg (vbd_link_t* vbd, RDiskResp* resp)
{
    struct request*	req  = (struct request*)(unsigned long) resp->cookie;
    unsigned long	flags;

    XTRACE ("\n");
    spin_lock_irqsave (&vbd->io_lock, flags);
    switch (resp->op) {
    case RDISK_OP_READ:
    case RDISK_OP_WRITE: {
	const int error = resp ->status != RDISK_STATUS_OK;

	VBD_CATCHIF (error,
		 ETRACE ("Bad return from blkdev data request: %x\n",
			 resp->status));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
	    /* __blk_end_request() is no more GPL */
	__blk_end_request (req, error ? -EIO : 0, blk_rq_bytes (req));
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28)
	    /*
	     * Should call:
	     * __blk_end_request(req, error ? -EIO : 0,
	     *                   req->hard_nr_sectors << 9);
	     * but it is EXPORT_SYMBOL_GPL
	     */
	req->hard_cur_sectors = req->hard_nr_sectors;
	end_request (req, !error);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
	end_dequeued_request (req, !error);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	if (unlikely (end_that_request_first (req, !error,
		      req->hard_nr_sectors))) {
	    BUG();
	}
#else
	    /*
	     *  This loop is necessary on 2.4 if we have
	     *  several buffer_head structures linked from
	     *  req->bh.
	     */
	while (end_that_request_first (req, !error, "VBD2"));
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
	    /* Nothing to do */
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
	end_that_request_last (req, !error);
#elif LINUX_VERSION_CODE == KERNEL_VERSION(2,6,15)
	    /*
	     * This is a work around in order to distinguish between a
	     * vanilla Linux kernel 2.6.15 and a patched FC5 kernel.
	     */
#if defined MODULES_ARE_ELF32 || defined MODULES_ARE_ELF64
	end_that_request_last (req, !error);
#else
	end_that_request_last (req);
#endif
#else
	end_that_request_last (req);
#endif
	break;
    }
    case RDISK_OP_PROBE:
	DTRACE ("response to probe received\n");
	memcpy (&vbd->probe_resp, resp, sizeof *resp);
	up (&vbd->probe_sem);
	break;

    default:
	BUG();
    }
    vbd_kick_pending_request_queues (vbd);
    spin_unlock_irqrestore (&vbd->io_lock, flags);
    vmq_return_msg_free (vbd->link, resp);
}

/*----- Module thread -----*/

#define VLX_SERVICES_THREADS
#include "vlx-services.c"

static struct semaphore	vbd_sem;		/* Thread semaphore */
static _Bool		vbd_thread_aborted;
static _Bool		vbd_sysconf;
static vlx_thread_t	vbd_thread_desc;

    static void
vbd_thread_aborted_notify (void)
{
    vbd_thread_aborted = true;
    up (&vbd_sem);
}

    static int
vbd_thread (void* arg)
{
    DTRACE ("started\n");
    while (!vbd_thread_aborted) {
	int diag = down_interruptible (&vbd_sem);
	(void) diag;
	DTRACE ("%s %s\n",
		vbd_thread_aborted  ? "thread-aborted"  : "",
		vbd_sysconf         ? "sysconf"         : "");
	if (vbd_thread_aborted) break;
	if (vbd_sysconf) {
	    vbd_sysconf = false;
	    vmq_links_sysconf (arg);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
	    /* Check if freeze signal has been received */
	try_to_freeze();
#endif
    }
    DTRACE ("exiting\n");
    return 0;
}

/*----- VMQ event handlers (callbacks) -----*/

#define VBD_LINK(link) \
    (*(vbd_link_t**) &((vmq_link_public_t*) (link))->priv)

    static void
vbd_cb_link_on (vmq_link_t* link)
{
    vbd_link_t* vbd = VBD_LINK (link);

    DTRACE ("link on (local <-> OS %d).\n", vmq_peer_osid (link));
    if (vbd->state != VBD_LINK_STATE_CONNECTED) {
	if (!vbd_link_acquire_disks (vbd)) {
	    vbd->state = VBD_LINK_STATE_CONNECTED;
	}
    }
}

    /*
     *  Callback from vlx-vmq.c in the context of vbd_thread().
     */

    static void
vbd_cb_link_off (vmq_link_t* link)
{
    vbd_link_t* vbd = VBD_LINK (link);

    DTRACE ("link %d\n", vmq_peer_osid (link));
    vbd->state = VBD_LINK_STATE_DISCONNECTED;
	/*
	 *  Mark deleted to make all future invocations
	 *  fail, until the client decides to close device
	 *  finally.
	 */
#if 0
    vipc_ctx_abort_calls (&VBD_LINK (link)->vipc_ctx);
#endif
}

    static void
vbd_cb_link_off_completed (vmq_link_t* link)
{
}

    static void
vbd_cb_sysconf_notify (vmq_links_t* links)
{
    (void) links;
    vbd_sysconf = true;
    up (&vbd_sem);
}

    /* No vbd_cb_receive_notify() yet */

    static void
vbd_cb_return_notify (vmq_link_t* link)
{
    void*  msg;

    while (!vmq_return_msg_receive (link, &msg)) {
	vbd_link_return_msg (VBD_LINK (link), (RDiskResp*) msg);
    }
}

/*----- Support for /proc/vbd2-fe -----*/

static struct proc_dir_entry*	vbd_proc;
static vmq_links_t*		vbd_links;

typedef struct {
    char* page;
    int   len;
} vbd_proc_t;

    static _Bool
vbd_link_proc (vmq_link_t* link, void* cookie)
{
    vbd_link_t* vbd = VBD_LINK (link);
    vbd_proc_t* ctx = cookie;

    ctx->len += sprintf (ctx->page + ctx->len,
			 "Peer %d MsgMax %d Disks %d State %s\n",
			 vmq_peer_osid (link), vbd->msg_max, vbd->ndisks,
			 vbd_link_state [vbd->state]);
    return false;
}

    static int
vbd_read_proc (char* page, char** start, off_t off, int count, int* eof,
	       void* data)
{
    vbd_proc_t	ctx;
    off_t	begin = 0;

    (void) data;
    ctx.len = sprintf (page, "Legend\n");

    ctx.page = page;
    vmq_links_iterate (vbd_links, vbd_link_proc, &ctx);
    page = ctx.page;

    if (ctx.len + begin > off + count)
	goto done;
    if (ctx.len + begin < off) {
	begin += ctx.len;
	ctx.len = 0;
    }
    *eof = 1;
done:
    if (off >= ctx.len + begin) return 0;
    *start = page + off - begin;
    return (count < begin + ctx.len - off ? count : begin + ctx.len - off);
}

/*----- Initialization and exit -----*/

    static int __init
vbd_wait (char* start)
{
    long  vmajor;
    long  vminor;
    char* end;

    if (*start++ != '(') {
	return 0;
    }
    vmajor = simple_strtoul (start, &end, 0);
    if (end == start || *end != ',') {
	return 0;
    }
    start = end + 1;
    vminor = simple_strtoul (start, &end, 0);
    if (end == start || *end != ')') {
	return 0;
    }
    vbd_wait_flag = 1;
    vbd_wait_id   = (vmajor << 8) | vminor;
    return 1;
}

#ifndef MODULE
__setup ("vbd2-wait=", vbd_wait);
#else
#ifdef CONFIG_ARM
#define vlx_command_line	saved_command_line
#else
extern char* vlx_command_line;
#endif
#endif

    static int __init
vbd_vlink_syntax (const char* opt)
{
    ETRACE ("Syntax error near '%s'\n", opt);
    return -EINVAL;
}

#define	VBD_DESC_SIZE(segs_per_req) \
    (sizeof (RDiskReqHeader) + sizeof (RDiskBuffer) * (segs_per_req))

typedef union {
    RDiskReqHeader req;
    RDiskResp      resp;
    RDiskProbe	   probe;
} NkDevVbdMsg;

    static int
vbd_cb_get_tx_config (vmq_links_t* links, const char* start,
		      vmq_xx_config_t* tx_config)
{
    long msg_count = VBD_LINK_DEFAULT_MSG_COUNT;
    long segs_per_req = VBD_LINK_MAX_SEGS_PER_REQ;
	/*
	 * vdev=(vbd,<linkid>|[<elem>][,[<segs_per_req>]])
	 */
    if (start) {
	if (*start && *start != ',') {
	    char* end;

	    msg_count = simple_strtoul (start, &end, 0);
	    if (end == start) return vbd_vlink_syntax (end);
	    start = end;
	    if (msg_count > 0) {
		    /* Round down to highest power of 2 */
		msg_count = nkops.nk_bit2mask (nkops.nk_mask2bit (msg_count));
	    } else {
		msg_count = VBD_LINK_DEFAULT_MSG_COUNT;
	    }
	}
	if (*start == ',') {
	    ++start;
	    if (*start) {
		char* end;

		segs_per_req = simple_strtoul (start, &end, 0);
		if (end == start) return vbd_vlink_syntax (end);
		start = end;
		if (segs_per_req <= 0 ||
		    segs_per_req > VBD_LINK_MAX_SEGS_PER_REQ) {
		    segs_per_req = VBD_LINK_MAX_SEGS_PER_REQ;
		}
	    }
	}
	if (*start) {
	    return vbd_vlink_syntax (start);
	}
    }
    memset (tx_config, 0, sizeof *tx_config);
    tx_config->msg_count = msg_count;
    tx_config->msg_max   = VBD_DESC_SIZE (segs_per_req);
    return 0;
}

    /* Called from vbd_init() only */

    static _Bool
vbd_link_init (vmq_link_t* link, void* cookie)
{
    vbd_link_t* vbd = kzalloc (sizeof *vbd, GFP_KERNEL);

    if (!vbd) {
	*(int*) cookie = -ENOMEM;
	return true;
    }
    VBD_LINK (link) = vbd;
    vbd->link    = link;
    vbd->msg_max = vmq_msg_max (link);
    vbd->io_lock = SPIN_LOCK_UNLOCKED;
    vbd->state   = VBD_LINK_STATE_CLOSED;
    sema_init (&vbd->probe_sem, 0);
    return false;
}

    static _Bool
vbd_link_free (vmq_link_t* link, void* cookie)
{
    (void) cookie;
    vbd_link_down (VBD_LINK (link));
    kfree (VBD_LINK (link));
    return false;
}

#define VBD_FIELD(name,value)	value

static const vmq_callbacks_t vbd_callbacks = {
    VBD_FIELD (link_on,			vbd_cb_link_on),
    VBD_FIELD (link_off,		vbd_cb_link_off),
    VBD_FIELD (link_off_completed,	vbd_cb_link_off_completed),
    VBD_FIELD (sysconf_notify,		vbd_cb_sysconf_notify),
    VBD_FIELD (receive_notify,		NULL),
    VBD_FIELD (return_notify,		vbd_cb_return_notify),
    VBD_FIELD (get_tx_config,		vbd_cb_get_tx_config)
};

    static const vmq_xx_config_t
vbd_rx_config = {
    VBD_FIELD (msg_count,	4),
    VBD_FIELD (msg_max,		sizeof (NkDevVbdMsg)),
    VBD_FIELD (data_count,	0),
    VBD_FIELD (data_max,	0)
};

#undef VBD_FIELD

    static void
vbd_exit (void)
{
    DTRACE ("\n");
    if (vbd_links) {
	vmq_links_abort (vbd_links);
    }
    vbd_thread_aborted_notify();
    vlx_thread_join (&vbd_thread_desc);
    if (vbd_links) {
	vmq_links_iterate (vbd_links, vbd_link_free, NULL);
	vmq_links_finish (vbd_links);
	vbd_links = NULL;
    }
    if (vbd_proc) {
	remove_proc_entry ("vbd2-fe", NULL);
    }
}

    static int __init
vbd_init (void)
{
    int diag;

    DTRACE ("\n");
#ifdef MODULE
    {
	char* cmdline;
	cmdline = vlx_command_line;
	while ((cmdline = strstr (cmdline, "vbd2-wait="))) {
	    cmdline += 11;
	    vbd_wait (cmdline);
	}
    }
#endif
    vbd_proc = create_proc_read_entry ("vbd2-fe", 0, NULL, vbd_read_proc, NULL);
    sema_init (&vbd_sem, 0);	/* Before it is signaled */
    diag = vmq_links_init (&vbd_links, "vbd2", &vbd_callbacks,
			   NULL /*tx_config*/, &vbd_rx_config);
    if (diag) goto error;
    if (vmq_links_iterate (vbd_links, vbd_link_init, &diag)) goto error;
    diag = vlx_thread_start (&vbd_thread_desc, vbd_thread, vbd_links,
			     "vbd2-fe");
    if (diag) {
	ETRACE ("thread start failure\n");
	goto error;
    }
    while (vbd_wait_flag) {	/* Waiting for the virtual disk */
	set_current_state (TASK_INTERRUPTIBLE);
	schedule_timeout (1);
    }
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

MODULE_DESCRIPTION ("VLX Virtual Block Device v.2 frontend driver");
MODULE_AUTHOR ("Adam Mirowski <adam.mirowski@virtuallogix.com>");
MODULE_LICENSE ("GPL");

/*----- End of file -----*/
