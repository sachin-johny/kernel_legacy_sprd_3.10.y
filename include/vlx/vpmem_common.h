/*
 ****************************************************************
 *
 *  Component:	VirtualLogix virtual Android Physical Memory
 *
 *  Copyright (C) 2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *   Christophe Lizzi (Christophe.Lizzi@virtuallogix.com)
 *
 ****************************************************************
 */

#ifndef _VPMEM_COMMON_H_
#define _VPMEM_COMMON_H_

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>	/* struct inode for android_pmem.h */
#include <linux/android_pmem.h>

#include <nk/nkern.h>
#include <nk/nkdev.h>
#include <nk/nk.h>

typedef struct vpmem_dev {
    unsigned int                      id;
    NkPhAddr                          plink;
    NkDevVlink*                       vlink;
    char*                             info;
    char                              name[25];
    unsigned int                      pmem_size;
    NkPhAddr                          pmem_phys;
    unsigned char*                    pmem_base;

#ifdef CONFIG_VPMEM_FRONTEND
    struct platform_device            plat_dev;
    struct android_pmem_platform_data plat_data;
#endif

} vpmem_dev_t;

#define VPMEM_DEV_MAX        16

#define VPMEM_DEFAULT_SIZE   (1024*1024)

#define VPMEM_VLINK_NAME     "vpmem"

#ifndef VPMEM_DRV_NAME
#define VPMEM_DRV_NAME       "vpmem"
#endif

#ifdef VPMEM_DEBUG
#define DTRACE(fmt, args...)    printk(VPMEM_DRV_NAME ": %s: " fmt, __func__, ## args)
#else
#define DTRACE(fmt, args...)    do {} while (0)
#endif

#define DTRACE1(fmt, args...)   printk(VPMEM_DRV_NAME ": %s: " fmt, __func__, ## args) // always enabled
#define DTRACE0(fmt, args...)   do {} while (0)                                        // always disabled

#define ETRACE(fmt, args...)    printk(VPMEM_DRV_NAME ": ERROR: %s: " fmt, __func__, ## args)

int          __init vpmem_module_init (int is_client);
int          __init vpmem_dev_init    (vpmem_dev_t* vpmem);
int          __init vpmem_info_name   (char* info, char* name, int maxlen);
unsigned int __init vpmem_info_size   (char* info);

void         __exit vpmem_module_exit (void);
int          __exit vpmem_dev_exit    (vpmem_dev_t* vpmem);

#endif // _VPMEM_COMMON_H_
