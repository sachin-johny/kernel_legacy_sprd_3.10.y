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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/version.h>

#define VPMEM_DEBUG

#define VPMEM_DRV_NAME "vpmem-be"

#include "vlx/vpmem_common.h"


    int __init
vpmem_dev_init (vpmem_dev_t* vpmem)
{
    vpmem->info = nkops.nk_ptov(vpmem->vlink->c_info);
    if (!vpmem->info || !*vpmem->info) {
	vpmem->info = "vpmem";
    }

    vpmem_info_name(vpmem->info, vpmem->name, sizeof(vpmem->name));

    vpmem->pmem_size = vpmem_info_size(vpmem->info);
    vpmem->pmem_phys = nkops.nk_pmem_alloc(vpmem->plink, 0, vpmem->pmem_size);
    if (!vpmem->pmem_phys) {

	ETRACE("vpmem id %u, name %s, nk_pmem_alloc(%d bytes) failed\n", vpmem->id, vpmem->name, vpmem->pmem_size);
	return -ENOMEM;
    }

    //vpmem->pmem_base = (char*)nkops.nk_mem_map(vpmem->pmem_phys, vpmem->pmem_size);

    DTRACE1("vpmem id %u, name %s, pmem size %u, phys addr [0x%lx -> 0x%lx] initialized\n",
      vpmem->id, vpmem->name, vpmem->pmem_size,
      (unsigned long)vpmem->pmem_phys, (unsigned long)vpmem->pmem_phys + vpmem->pmem_size);

    return 0;
}


    int __exit
vpmem_dev_exit (vpmem_dev_t* vpmem)
{
    DTRACE("removing vpmem id %u, name %s\n", vpmem->id, vpmem->name);

    if (vpmem->pmem_base && vpmem->pmem_phys) {
	nkops.nk_mem_unmap(vpmem->pmem_base, vpmem->pmem_phys, vpmem->pmem_size);
    }

    DTRACE1("vpmem id %u, name %s, pmem size %u, phys addr [0x%lx -> 0x%lx] removed\n",
      vpmem->id, vpmem->name, vpmem->pmem_size,
      (unsigned long)vpmem->pmem_phys, (unsigned long)vpmem->pmem_phys + vpmem->pmem_size);

    return 0;
}


    static int __init
vpmem_init (void)
{
    return vpmem_module_init(0);
}


    static void __exit
vpmem_exit (void)
{
    vpmem_module_exit();
}


module_init(vpmem_init);
module_exit(vpmem_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("VLX virtual Android Physical Memory back-end");
