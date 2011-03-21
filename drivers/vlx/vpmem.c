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

#define VPMEM_DEBUG

#include "vlx/vpmem_common.h"
#include "vpmem.h"


static vpmem_dev_t* vpmem_dev_table[ VPMEM_DEV_MAX ] = { NULL };
static unsigned int vpmem_dev_count = 0;


    static vpmem_dev_t*
vpmem_dev_alloc (void)
{
    vpmem_dev_t* vpmem;
    unsigned int i;

    if (vpmem_dev_count == VPMEM_DEV_MAX) {
	return NULL;
    }

    vpmem = kzalloc(sizeof(vpmem_dev_t), GFP_KERNEL);
    if (vpmem == NULL) {
	return NULL;
    }

    for (i = 0; i < VPMEM_DEV_MAX; i++) {
	if (vpmem_dev_table[ i ] == NULL) {
	    break;
	}
    }
    BUG_ON(i == VPMEM_DEV_MAX);

    vpmem_dev_table[ i ] = vpmem;
    vpmem_dev_count++;

    vpmem->id = i;

    DTRACE1("allocated new vpmem %p, id %u\n", vpmem, vpmem->id);

    return vpmem;
}


    static void
vpmem_dev_free (vpmem_dev_t* vpmem)
{
    unsigned int i;

    if (vpmem == NULL) {
	return;
    }

    for (i = 0; i < VPMEM_DEV_MAX; i++) {
	if (vpmem_dev_table[ i ] == vpmem) {
	    break;
	}
    }
    BUG_ON(i == VPMEM_DEV_MAX);

    DTRACE("freeing vpmem %p, id %u\n", vpmem, vpmem->id);

    vpmem_dev_table[ i ] = NULL;
    vpmem_dev_count--;

    kfree(vpmem);
}


    static char* __init
_a2ui (char* s, unsigned int* i)
{
    unsigned int xi = 0;
    char         c  = *s;

    while (('0' <= c) && (c <= '9')) {
	xi = xi * 10 + (c - '0');
	c = *(++s);
    }

    if        ((*s == 'K') || (*s == 'k')) {
	xi *= 1024;
	s  += 1;
    } else if ((*s == 'M') || (*s == 'm')) {
	xi *= (1024*1024);
	s  += 1;
    }

    *i = xi;

    return s;
}


    int __init
vpmem_info_name (char* info, char* name, int maxlen)
{
    int len = 0;

    if (info) {
	while (info[len] && (info[len] != ',')) {
	    name[len] = info[len];
	    if (++len == (maxlen - 1)) {
		break;
	    }
	}
    }
    name[len] = '\0';

    DTRACE("info %s -> name %s (len %d)\n", info, name, len);

    return len;
}


    unsigned int __init
vpmem_info_size (char* info)
{
    unsigned int size = VPMEM_DEFAULT_SIZE;

    if (info) {
	while (*info && (*info != ',')) info++;
	if (*info) {
	    info = _a2ui(info+1, &size);
	    if (*info) {
		size = VPMEM_DEFAULT_SIZE;
	    }
	}
    }

    if (size < PAGE_SIZE) {
	size = PAGE_SIZE;
    }

    DTRACE("info %s -> size %u\n", info, size);

    return size;
}


    int __init
vpmem_module_init (int is_client)
{
    NkPhAddr       plink;
    NkDevVlink*    vlink;
    int 	   err = 0;
    NkOsId         my_id = nkops.nk_id_get();
    NkOsId         vlink_id;
    vpmem_dev_t*   vpmem;

    DTRACE("initializing module, my_id %ld\n", (unsigned long)my_id);

    plink = 0;
    while ((plink = nkops.nk_vlink_lookup(VPMEM_VLINK_NAME, plink)) != 0) {

	vlink = nkops.nk_ptov(plink);

	vlink_id = is_client ? vlink->c_id : vlink->s_id;

	DTRACE("comparing my_id %d to vlink_id %d (c_id %d, s_id %d)\n", my_id, vlink_id, vlink->c_id, vlink->s_id);

	if (vlink_id == my_id) {

	    vpmem = vpmem_dev_alloc();
	    if (vpmem == NULL) {
		err = -ENOMEM;
		break;
	    }

	    vpmem->plink = plink;
	    vpmem->vlink = vlink;

	    err = vpmem_dev_init(vpmem);
	    if (err != 0) {
		vpmem_dev_free(vpmem);
		break;
	    }

	    printk(KERN_INFO "device %s (vpmem id %d) is created"
	                     " for OS#%d<-OS#%d link=%d\n",
	      vpmem->name, vpmem->id, vlink->s_id, vlink->c_id, vlink->link);
	}
    }

    DTRACE1("module initialized, %u vpmem devices created, err %d\n", vpmem_dev_count, err);

    return err;
}


    void __exit
vpmem_module_exit (void)
{
    unsigned int i;
    unsigned int err = 0;

    DTRACE("removing module, %u vpmem devices to remove\n", vpmem_dev_count);

    for (i = 0; i < VPMEM_DEV_MAX; i++) {

	if (vpmem_dev_count == 0) {
	    break;
	}

	if (vpmem_dev_table[ i ] != NULL) {
	    err = vpmem_dev_exit(vpmem_dev_table[ i ]);
	    if (err != 0) {
		break;
	    }
	    vpmem_dev_free(vpmem_dev_table[ i ]);
	}
    }

    DTRACE1("module removed, still %u vpmem devices, err %d\n", vpmem_dev_count, err);
}


    static vpmem_dev_t*
vpmem_dev_lookup (char* name)
{
    vpmem_dev_t* vpmem = NULL;
    unsigned int i;

    for (i = 0; i < VPMEM_DEV_MAX; i++) {
	if (vpmem_dev_table[ i ] == NULL) {
	    continue;

	}
	if (strcmp(vpmem_dev_table[ i ]->name, name) == 0) {
	    vpmem = vpmem_dev_table[ i ];
	    break;
	}
    }

    if (vpmem) {
	DTRACE("name %s -> vpmem %p (id %u, pmem size %u, phys addr [0x%lx -> 0x%lx]\n",
	  name, vpmem, vpmem->id, vpmem->pmem_size,
	  (unsigned long)vpmem->pmem_phys, (unsigned long)vpmem->pmem_phys + vpmem->pmem_size);
    } else {
	DTRACE("vpmem device name %s not found\n", name);
    }

    return vpmem;
}


// vpmem API for use by the other virtual drivers

    vpmem_handle_t
vpmem_lookup (char* name)
{
    return (vpmem_handle_t) vpmem_dev_lookup(name);
}


    unsigned char*
vpmem_map (vpmem_handle_t handle)
{
    vpmem_dev_t* vpmem = (vpmem_dev_t*) handle;

    if ((vpmem->pmem_base == NULL) && (vpmem->pmem_phys != 0)) {
	vpmem->pmem_base = (char*)nkops.nk_mem_map(vpmem->pmem_phys, vpmem->pmem_size);
    }

    return vpmem->pmem_base;
}


    void
vpmem_unmap (vpmem_handle_t handle)
{
    vpmem_dev_t* vpmem = (vpmem_dev_t*) handle;

    if (vpmem->pmem_base && vpmem->pmem_phys) {
	nkops.nk_mem_unmap(vpmem->pmem_base, vpmem->pmem_phys, vpmem->pmem_size);
	vpmem->pmem_base = NULL;
    }
}


    unsigned long
vpmem_phys (vpmem_handle_t handle)
{
    vpmem_dev_t* vpmem = (vpmem_dev_t*) handle;

    return (unsigned long) vpmem->pmem_phys;
}


    unsigned int
vpmem_size (vpmem_handle_t handle)
{
    vpmem_dev_t* vpmem = (vpmem_dev_t*) handle;

    return vpmem->pmem_size;
}


    unsigned int
vpmem_id (vpmem_handle_t handle)
{
    vpmem_dev_t* vpmem = (vpmem_dev_t*) handle;

    return vpmem->id;
}

