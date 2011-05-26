/*
 ****************************************************************
 *
 * Component = Nano-Kernel control file /proc/nk
 *
 * Copyright (C) 2002-2007 VirtualLogix SA.
 *
 * This program is free software;  you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * #ident  "@(#)proc.c 1.4     08/03/04 VirtualLogix"
 *
 * Contributor(s):
 *   Vladimir Grouzdev (vladimir.grouzdev@virtuallogix.com) VirtualLogix SA
 *
 ****************************************************************
 */

/*
 * This module provides the /proc/nk/restart, /proc/nk/stop and
 * /proc/nk/resume files allowing to stop/resume and restart a
 * secondary kernel.
 *
 * This module also provides /proc/nk/guest#X directories which
 * the following entries:
 * -guest_bank#1 [guest_bank#X] (guests banks)
 * -epoint  (entry point for guest)
 * -cmdline (cmdline for guest)
 * -status  (current status of guest)
 * This enables the "dynamic guest" start: Guests banks
 * are not included in the hole system image and can be
 * changed dynamicaly
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <nk/nkern.h>
#include <asm/uaccess.h>
#include <asm/nkern.h>
#include <asm/nk/nktags.h>
#include <nk/bconf.h>

#define SMALL_BUFF      10
#define BIG_BUFF        (SMALL_BUFF + NK_COMMAND_LINE_SIZE)
#define BANK_BUFF_SIZE  43
#ifdef DEBUG
#define TRACE(...) printk(__VA_ARGS__)
#else
#define TRACE(...)
#endif

typedef struct GuestBank {
    struct DynGuest*       parent;
    char*                  name;
    unsigned int           size;
    unsigned int           cfgSize;
    NkPhAddr               pstart;
    NkVmAddr               vstart;
    BankType               type;
    unsigned int           inuse;
    struct proc_dir_entry* file;
    struct GuestBank*      next;
} GuestBank;

typedef struct DynGuest {
    NkOsId                 id;
    char*                  name;
    int                    type;
    GuestBank*             banks;
    nku32_f*               epoint;
    NkTagHeader*           tags;
    char*                  cmd;
    spinlock_t             tlock;
    unsigned int           inuse;
    unsigned int           started;
    unsigned int           suspended;
    struct proc_dir_entry* dir;
    struct proc_dir_entry* cfile;
    struct proc_dir_entry* efile;
    struct proc_dir_entry* sfile;
    struct DynGuest*       next;
} DynGuest;

typedef enum IndType {
    ITYPE_CMD,
    ITYPE_EPT,
    ITYPE_STS
} IndType;

typedef enum AtagClass {
    ATAG_CLASS_CMD,
    ATAG_CLASS_RMD
} AtagClass;

static DynGuest*              guests;
static NkXIrqId               scf_xid;
static spinlock_t             glock;
static struct proc_dir_entry* nk;

    static int
_fixup_guest_tags (DynGuest* dg, AtagClass ac)
{
    NkTagHeader* tmp_tag;

    spin_lock(&(dg->tlock));

    tmp_tag = dg->tags;

    if (ac == ATAG_CLASS_CMD) {
        while ((tmp_tag = find_tag(tmp_tag, ATAG_CMDLINE)) != 0) {
            remove_tag(tmp_tag);
        }
        tmp_tag = add_cmd_line_tag(dg->tags, dg->cmd, NK_TAG_LIST_SIZE);
    } else if (ac == ATAG_CLASS_RMD) {
        GuestBank* gb = dg->banks;

        while ((tmp_tag = find_tag(tmp_tag, ATAG_INITRD)) != 0) {
            remove_tag(tmp_tag);
        }

        tmp_tag = dg->tags;
        while ((tmp_tag = find_tag(tmp_tag, ATAG_INITRD2)) != 0) {
            remove_tag(tmp_tag);
        }

        while (gb) {
            if (BANK_TYPE(gb->type) == BANK_TYPE_RAMDISK) {
                tmp_tag = add_ram_disk_tag(dg->tags,
                                           nkops.nk_vtop((void*)gb->vstart),
                                           gb->size, NK_TAG_LIST_SIZE);
            }
            gb = gb->next;
        }
    } else {
        tmp_tag = 0;
    }

    spin_unlock(&(dg->tlock));

    return tmp_tag ? 1:0;
}

    static DynGuest*
_id2dg (NkOsId id)
{
    DynGuest*     dg = guests;
    while (dg) {
        if (dg->id == id) {
            return dg;
        }
        dg = dg->next;
    }
    return NULL;
}

    static DynGuest*
_inum2dg (unsigned int inum, IndType type)
{
    DynGuest*     dg = guests;
    unsigned  int tmp;
    while (dg) {
        switch (type) {
            case ITYPE_CMD:
                tmp = dg->cfile->low_ino;
                break;
            case ITYPE_EPT:
                tmp = dg->efile->low_ino;
                break;
            case ITYPE_STS:
                tmp = dg->sfile->low_ino;
                break;
            default:
                return NULL;
        }
        if (tmp == inum) {
                return dg;
        }
        dg = dg->next;
    }
    return NULL;
}

   static GuestBank*
_inum2gbank (unsigned int inum)
{
    DynGuest* dg = guests;
    while (dg) {
        GuestBank* gb = dg->banks;
        while (gb) {
            if (gb->file->low_ino == inum) {
                return gb;
            }
            gb = gb->next;
        }
        dg = dg->next;
    }
    return NULL;
}

   static int
_nk_proc_cmd_open (struct inode* inode,
               struct file*  file)
{
    DynGuest* dg = _inum2dg((unsigned int)inode->i_ino, ITYPE_CMD);
    int       err;

    if (!dg){
        printk("NK -- error: no such guest\n");
        return -ENOENT;
    }

    spin_lock(&glock);
    if (dg->started && (!dg->suspended)) {
        err = -EAGAIN;
    } else {
        dg->inuse++;
        file->private_data = (void*)dg;
        err = 0;
    }
    spin_unlock(&glock);

    return err;
}

    static int
_nk_proc_cmd_release (struct inode* inode,
                  struct file*  file)
{
    DynGuest* dg = file->private_data;
    int       err;

    if(!_fixup_guest_tags(dg, ATAG_CLASS_CMD)) {
        printk("NK -- error: can't add guest command line tag\n");
        err = -EINVAL;
    } else {
        err = 0;
    }

    spin_lock(&glock);
    dg->inuse--;
    spin_unlock(&glock);

    return err;
}

    static ssize_t
_nk_proc_cmd_read (struct file* file,
               char*        buf,
               size_t       count,
               loff_t*      ppos)
{
    DynGuest* dg = (DynGuest*)file->private_data;
    size_t    csize = strlen(dg->cmd);
    char      pbuf[NK_COMMAND_LINE_SIZE + 1];

    BUG_ON(csize > NK_COMMAND_LINE_SIZE);

    if (*ppos || !count) {
        return 0;
    }

    if (count > (csize + 1)) {
        count = csize + 1;
    }

    sprintf(pbuf, "%s\n", dg->cmd);

    if (copy_to_user(buf, pbuf, count)) {
        printk("NK -- error: can't copy to user provided buffer\n");
        return -EFAULT;
    }

    *ppos += count;
    return count;
}

    static ssize_t
_nk_proc_cmd_write (struct file* file,
                       const char*  ubuf,
                       size_t       size,
                       loff_t*      ppos)
{
    DynGuest* dg = (DynGuest*)file->private_data;

    if (*ppos || !size) {
        return 0;
    }
    if (size > (NK_COMMAND_LINE_SIZE - 1)) {
        size = NK_COMMAND_LINE_SIZE - 1;
    }

    if (copy_from_user(dg->cmd, ubuf, size)) {
        printk("NK -- error: can't copy command line\n");
        return -EINVAL;
    }
    dg->cmd[size] = 0;

    *ppos += size;
    return size;
}


   static int
_nk_proc_bank_open (struct inode* inode,
               struct file*  file)
{
    GuestBank*  gb  = _inum2gbank((unsigned int)inode->i_ino);
    int         err;
    if (!gb){
        printk("NK -- error: no such bank\n");
        return -ENOENT;
    }

    spin_lock(&glock);
    if (gb->parent->started && (!gb->parent->suspended)) {
        err = -EAGAIN;
    } else {
        gb->parent->inuse++;
        file->private_data = (void*)gb;
        err = 0;
    }
    spin_unlock(&glock);

    return err;
}

    static loff_t
_nk_proc_bank_lseek (struct file* file,
		     loff_t       off,
		     int          whence)
{
    loff_t new;
    GuestBank* gb  = file->private_data;
    int        err = 0;

    if (!gb) {
	return -EINVAL;
    }

    switch (whence) {
	case 0:	 new = off; break;
	case 1:	 new = file->f_pos + off; break;
	case 2:	 new = gb->cfgSize + off; break;
	default: return -EINVAL;
    }

    if (new > gb->cfgSize) {
	return -EINVAL;
    }

    return (file->f_pos = new);
}


    static int
_nk_proc_bank_release (struct inode* inode,
                  struct file*  file)
{
    GuestBank* gb  = file->private_data;
    int        err = 0;

    if (BANK_TYPE(gb->type) == BANK_TYPE_RAMDISK) {
        if(!_fixup_guest_tags(gb->parent, ATAG_CLASS_RMD)) {
            printk("NK -- error: can't add guest ram disk tag\n");
            err = -EINVAL;
        }
    }

    gb->size = 0;

    spin_lock(&glock);
    gb->parent->inuse--;
    spin_unlock(&glock);

    return err;
}

    static ssize_t
_nk_proc_bank_read (struct file* file,
               char*        buf,
               size_t       count,
               loff_t*      ppos)
{
    GuestBank* gb = (GuestBank*)file->private_data;
    char       pbuf[BANK_BUFF_SIZE];

    if (*ppos || !count) {
        return 0;
    }

    snprintf(pbuf, BANK_BUFF_SIZE,"virtual start: 0x%08x size:0x%08x\n", gb->vstart, gb->cfgSize);
    if (count > (BANK_BUFF_SIZE -1)) {
        count = BANK_BUFF_SIZE -1;
    }

    if (copy_to_user(buf, pbuf, count)) {
        printk("NK -- error: can't copy to user provided buffer\n");
        return -EFAULT;
    }
    *ppos += count;

    return count;
}

    static ssize_t
_nk_proc_bank_write (struct file* file,
                       const char*  ubuf,
                       size_t       size,
                       loff_t*      ppos)
{
    GuestBank* gb = (GuestBank*)file->private_data;
    void*      dst;
    printk("Writing nk_proc_bank_write\n");

    TRACE("Writing nk_proc_bank_write pos:%llx buf:%x size:%x\n", 
           *ppos,ubuf,size);

    if ((*ppos + size) > gb->cfgSize) {
        return -EFBIG;
    }

    dst = nkops.nk_mem_map(gb->pstart + *ppos, size);
    if (!dst) {
        printk("NNK -- error: can't map guest memory bank\n");
        return -EINVAL;
    }
    if (copy_from_user(dst, ubuf, size)) {
        printk("NK -- error can't copy command line\n");
        return -EINVAL;
    }

    if ((*ppos + size) > gb->size) {
        gb->size = *ppos + size;
    }

    *ppos += size;
    return size;
}


   static int
_nk_proc_ept_open (struct inode* inode,
               struct file*  file)
{
    DynGuest* dg = _inum2dg((unsigned int)inode->i_ino, ITYPE_EPT);
    int       err;

    if (!dg){
        printk("NK -- error: no such guest\n");
        return -ENOENT;
    }

    spin_lock(&glock);
    if (dg->started && (!dg->suspended)) {
        err = -EAGAIN;
    } else {
        dg->inuse++;
        file->private_data = (void*)dg;
        err = 0;
    }
    spin_unlock(&glock);

    return err;
}

    static int
_nk_proc_ept_release (struct inode* inode,
                  struct file*  file)
{
    DynGuest* dg = file->private_data;

    spin_lock(&glock);
    dg->inuse--;
    spin_unlock(&glock);

    return 0;
}

    static ssize_t
_nk_proc_ept_read (struct file* file,
               char*        buf,
               size_t       count,
               loff_t*      ppos)
{
    DynGuest* dg = (DynGuest*)file->private_data;
    char      pbuf[12];

    if (*ppos || !count) {
        return 0;
    }

    if (count > 12) {
        count = 12;
    }

    sprintf(pbuf, "0x%08x\n", *dg->epoint);

    if (copy_to_user(buf, pbuf, count)) {
        printk("NK -- error: can't copy to user provided buffer\n");
        return -EFAULT;
    }

    *ppos += count;

    return count;
}

    static ssize_t
_nk_proc_ept_write (struct file* file,
                       const char*  ubuf,
                       size_t       size,
                       loff_t*      ppos)
{
    DynGuest*     dg = (DynGuest*)file->private_data;
    char          pbuf[11];
    unsigned long tmp;

    if (*ppos || !size) {
        return 0;
    }

    if (size > 11) {
        size = 11;
    }

    if (copy_from_user(pbuf, ubuf, size)) {
        printk("NK -- error: can't copy command line\n");
        return -EINVAL;
    }

    tmp = simple_strtoul(pbuf, NULL, 0);

    *dg->epoint = tmp;

    *ppos += size;

    return size;
}


   static int
_nk_proc_sts_open (struct inode* inode,
               struct file*  file)
{
    DynGuest* dg = _inum2dg((unsigned int)inode->i_ino, ITYPE_STS);

    if (!dg){
        printk("NK -- error: no such guest\n");
        return -ENOENT;
    }
    file->private_data = (void*)dg;
    return 0;
}


    static loff_t
_nk_proc_sts_lseek (struct file* file,
	        loff_t       off,
	        int          whence)
{
    loff_t new;
    DynGuest*    dg = (DynGuest*)file->private_data;

    switch (whence) {
	case 0:	 new = off; break;
	case 1:	 new = file->f_pos + off; break;
	case 2:	 new = 28 + off; break;
	default: return -EINVAL;
    }

    return (file->f_pos = new);
}


    static int
_nk_proc_sts_release (struct inode* inode,
                  struct file*  file)
{
    return 0;
}

    static ssize_t
_nk_proc_sts_read (struct file* file,
               char*        buf,
               size_t       count,
               loff_t*      ppos)
{
    DynGuest*    dg = (DynGuest*)file->private_data;
    char         pbuf[28];
    unsigned int len;

    if (!count) {
        return 0;
    }

    sprintf(pbuf, "%s started, %s suspended\n", dg->started? "  ":"not", dg->suspended? "   ":"not");

    len   = strlen(pbuf);
    if (*ppos > len) {
	return 0;
    }
    len = len - *ppos;
    count = (len > count) ? count : len;

     if (copy_to_user(buf + *ppos, pbuf, count)) {
        printk("NK -- error: can't copy to user provided buffer\n");
        return -EFAULT;
    }

    *ppos += count;

    return count;
}

    static ssize_t
_nk_proc_sts_write (struct file* file,
                       const char*  ubuf,
                       size_t       size,
                       loff_t*      ppos)
{
    return -EFAULT;
}

    static int
_nk_proc_open (struct inode* inode,
               struct file*  file)
{
    return 0;
}

    static int
_nk_proc_release (struct inode* inode,
                  struct file*  file)
{
    return 0;
}

    static loff_t
_nk_proc_lseek (struct file* file,
                loff_t       off,
                int          whence)
{
    loff_t new;

    switch (whence) {
        case 0:  new = off; break;
        case 1:  new = file->f_pos + off; break;
        case 2:  new = 1 + off; break;
        default: return -EINVAL;
    }

    if (new) {
        return -EINVAL;
    }

    return (file->f_pos = new);
}

    static ssize_t
_nk_proc_read (struct file* file,
               char*        buf,
               size_t       count,
               loff_t*      ppos)
{
    return 0;
}

    static int
_nk_proc_copyin(char* sbuf, const char* ubuf, size_t size, size_t limit)
{
    if (size > (limit - 1)) size = limit - 1;

    if (copy_from_user(sbuf, ubuf, size)) {
            return -EFAULT;
    }

    sbuf[size] = 0;
    return 0;
}

    static const char*
_nk_proc_getid (const char* buf, NkOsId* os_id)
{
    NkOsId id = 0;
    for (;;) {
        char digit;

        digit = *buf;

        if ((digit < '0') || ('9' < digit)) {
            break;
        }

        buf++;
        id = (id * 10) + (digit - '0');
    }

    *os_id = id;
    return buf;
}

    static ssize_t
_nkrestart_proc_write (struct file* file,
                       const char*  ubuf,
                       size_t       size,
                       loff_t*      ppos)
{
    NkOsId    id;
    int       res;
    DynGuest* dg;
    char      sbuf[SMALL_BUFF];

    if (*ppos || !size) {
        return 0;
    }

    res = _nk_proc_copyin(sbuf, ubuf, size, SMALL_BUFF);
    if (res < 0) {
        return res;
    }

    _nk_proc_getid(sbuf, &id);

    if (!(dg = _id2dg(id))) {
        os_ctx->restart(os_ctx, id);
        return size;
    }

    spin_lock(&glock);
    if ((dg->started && (!dg->suspended)) || dg->inuse) {
        spin_unlock(&glock);
        return -EAGAIN;
    }

    dg->suspended = 0;
    dg->started   = 1;
    spin_unlock(&glock);
    os_ctx->restart(os_ctx, id);

    return size;
}


    static ssize_t
_nkstop_proc_write (struct file* file,
                    const char*  ubuf,
                    size_t       size,
                    loff_t*      ppos)
{
    NkOsId    id;
    int       res;
    DynGuest* dg;
    char      sbuf[SMALL_BUFF];

    if (*ppos || !size) {
        return 0;
    }

    res = _nk_proc_copyin(sbuf, ubuf, size, SMALL_BUFF);
    if (res < 0) {
        return res;
    }

    _nk_proc_getid(sbuf, &id);

    if (!(dg = _id2dg(id))) {
        os_ctx->stop(os_ctx, id);
        return size;
    }

    spin_lock(&glock);
    if (!dg->started) {
        spin_unlock(&glock);
        return -EAGAIN;
    }

    dg->suspended = 1;

    spin_unlock(&glock);

    os_ctx->stop(os_ctx, id);

    return size;
}

    static ssize_t
_nkresume_proc_write (struct file* file,
                      const char*  ubuf,
                      size_t       size,
                      loff_t*      ppos)
{
    NkOsId    id;
    int       res;
    DynGuest* dg;
    char      sbuf[SMALL_BUFF];

    if (*ppos || !size) {
        return 0;
    }

    res = _nk_proc_copyin(sbuf, ubuf, size, SMALL_BUFF);
    if (res < 0) {
        return res;
    }

    _nk_proc_getid(sbuf, &id);

    if (!(dg = _id2dg(id))) {
           os_ctx->resume(os_ctx, id);
           return size;
    }

    spin_lock(&glock);
    if (dg->inuse) {
        spin_unlock(&glock);
        return -EAGAIN;
    }
    dg->suspended = 0;
    spin_unlock(&glock);

    os_ctx->resume(os_ctx, id);

    return size;
}

static BLOCKING_NOTIFIER_HEAD(focus_notifier_list);
int focus_register_client(struct notifier_block *nb)
{
        return blocking_notifier_chain_register(&focus_notifier_list, nb);
}
EXPORT_SYMBOL(focus_register_client);
int focus_unregister_client(struct notifier_block *nb)
{
        return blocking_notifier_chain_unregister(&focus_notifier_list, nb);
}
EXPORT_SYMBOL(focus_unregister_client);

    static ssize_t
_nkevent_proc_write (struct file* file,
                      const char*  ubuf,
                      size_t       size,
                      loff_t*      ppos)
{
    NkOsId id;
    int    res;
    char   sbuf[SMALL_BUFF];

    if (*ppos || !size) {
        return 0;
    }

    res = _nk_proc_copyin(sbuf, ubuf, size, SMALL_BUFF);
    if (res < 0) {
        return res;
    }

    _nk_proc_getid(sbuf, &id);

    blocking_notifier_call_chain(&focus_notifier_list, id, 0);

    return size;
}

    void
nk_change_focus(NkOsId id)
{
    blocking_notifier_call_chain(&focus_notifier_list, id, 0);
}
EXPORT_SYMBOL(nk_change_focus);

static struct file_operations _nkrestart_proc_fops = {
    open:    _nk_proc_open,
    release: _nk_proc_release,
    llseek:  _nk_proc_lseek,
    read:    _nk_proc_read,
    write:   _nkrestart_proc_write,
};

static struct file_operations _nkstop_proc_fops = {
    open:    _nk_proc_open,
    release: _nk_proc_release,
    llseek:  _nk_proc_lseek,
    read:    _nk_proc_read,
    write:   _nkstop_proc_write,
};

static struct file_operations _nkresume_proc_fops = {
    open:    _nk_proc_open,
    release: _nk_proc_release,
    llseek:  _nk_proc_lseek,
    read:    _nk_proc_read,
    write:   _nkresume_proc_write,
};

static struct file_operations _nkevent_proc_fops = {
    open:    _nk_proc_open,
    release: _nk_proc_release,
    llseek:  _nk_proc_lseek,
    read:    _nk_proc_read,
    write:   _nkevent_proc_write,
};

static struct file_operations _nkbank_proc_fops = {
    open:    _nk_proc_bank_open,
    release: _nk_proc_bank_release,
    llseek:  _nk_proc_bank_lseek,
    read:    _nk_proc_bank_read,
    write:   _nk_proc_bank_write,
};

static struct file_operations _nkcmd_proc_fops = {
    open:    _nk_proc_cmd_open,
    release: _nk_proc_cmd_release,
    llseek:  _nk_proc_lseek,
    read:    _nk_proc_cmd_read,
    write:   _nk_proc_cmd_write,
};

static struct file_operations _nkept_proc_fops = {
    open:    _nk_proc_ept_open,
    release: _nk_proc_ept_release,
    llseek:  _nk_proc_lseek,
    read:    _nk_proc_ept_read,
    write:   _nk_proc_ept_write,
};

static struct file_operations _nksts_proc_fops = {
    open:    _nk_proc_sts_open,
    release: _nk_proc_sts_release,
    llseek:  _nk_proc_sts_lseek,
    read:    _nk_proc_sts_read,
    write:   _nk_proc_sts_write,
};

    static  struct proc_dir_entry*
_nk_proc_create (struct proc_dir_entry*  parent,
                 const char*             name,
                 struct file_operations* fops)
{
    struct proc_dir_entry* file;
    file = create_proc_entry(name, (S_IFREG|S_IRUGO|S_IWUSR), parent);
    if (!file) {
        printk("NK --error: create_proc_entry(%s) failed\n", name);
        return NULL;
    }

    file->proc_fops = fops;

    return file;
}

    static DynGuest*
_guest_lookup (NkOsId id)
{
    DynGuest* dg = guests;

    while (dg) {
        if (dg->id == id) {
            return dg;
        }
        dg = dg->next;
    }
    return 0;
}

    static DynGuest*
_guest_create (NkOsId id)
{
    NkBootInfo vinfo;
    NkPhAddr   paddr;
    DynGuest*  dg = kzalloc(sizeof(DynGuest), GFP_KERNEL);
    if (!dg) {
        printk("NK -- error:cant create dynamic guest: out of memory\n");
        return 0;
    }

    spin_lock_init((&dg->tlock));


    dg->next = guests;
    guests   = dg;

    dg->id = id;

    paddr      = nkops.nk_vtop(&vinfo);
    vinfo.osid = id;

    vinfo.ctrl = INFO_CMDLINE;
    os_ctx->binfo(os_ctx, paddr);
    dg->cmd    = (char*)(nkops.nk_ptov(vinfo.data));

    vinfo.ctrl = INFO_OSTAGS;
    os_ctx->binfo(os_ctx, paddr);
    dg->tags    = (NkTagHeader*)(nkops.nk_ptov(vinfo.data));

    vinfo.ctrl = INFO_EPOINT;
    os_ctx->binfo(os_ctx, paddr);
    dg->epoint = (nku32_f*)(nkops.nk_ptov(vinfo.data));

    return dg;
}

    static int
_bank_create (DynGuest* dg, BankDesc* bd)
{
    GuestBank* gb = kzalloc(sizeof(GuestBank), GFP_KERNEL);
    if (!gb) {
        printk("NK -- error: cant create dynamic guest: out of memory\n");
        return 0;
    }

    gb->parent = dg;
    gb->name   = kzalloc(strlen(bd->id), GFP_KERNEL);
    if (!(gb->name)) {
        printk("NK --error: cant create dynamic guest: out of memory\n");
        kfree(gb);
        return 0;
    }

    strcpy(gb->name, bd->id);

    gb->cfgSize = bd->cfgSize;
    gb->pstart  = nkops.nk_vtop((void*)bd->vaddr);
    gb->vstart  = (NkVmAddr)bd->vaddr;
    gb->type    = bd->type;

    gb->next  = dg->banks;
    dg->banks = gb;

    return 1;
}

static nku32_f   all_banks_empty = -1; /* Maximum 32 OS's */
    static int
_guests_init (int count, BankDesc* bd)
{
    spin_lock_init((&glock));

    int       count_tmp = count;
    BankDesc* bd_tmp    = bd;

#ifdef L
    while (count_tmp--) {
        NkOsId id = BANK_OS_ID(bd_tmp->type);
        if (/*bd_tmp->size ||*/ (id = BANK_OS_ID(BANK_OS_SHARED))) {
            all_banks_empty &= ~(1 << id);
        }
        bd_tmp++;
    }
#else
    all_banks_empty = 1 << 2;
#endif

    while (count--) {
        NkOsId  id = BANK_OS_ID(bd->type);
        DynGuest*    dg;

        if (!(all_banks_empty & (1 << id))) {
            bd++;
            continue;
        }

        if (!(dg = _guest_lookup(id))) {
            if (!(dg = _guest_create(id))) {
                return 0;
            }
        }

        if (!_bank_create(dg, bd)) {
            return 0;
        }
        bd++;
    }
    return 1;
}


static int __init bank_setup(char* str)
{
    char*		res;
    unsigned long	cnt;

    cnt = simple_strtoul(str, &res, 0);
    if (str != res) {
	all_banks_empty = cnt;
    }
    return 1;
}

__setup("show-guest-banks=", bank_setup);

    static int
_guests_proc_init (void)
{
    DynGuest* dg = guests;

    while (dg) {
        GuestBank* gb = dg->banks;
        char       buf[10];

        sprintf(buf, "guest-%02d", dg->id);
        dg->dir = proc_mkdir(buf, nk);

        if (!(dg->dir)) {
            printk("NK -- error: proc_mkdir(/proc/nk/%s) failed\n", buf);
            return 0;
        }

        dg->cfile  = _nk_proc_create(dg->dir, "cmdline", &_nkcmd_proc_fops);
        dg->efile  = _nk_proc_create(dg->dir, "epoint",  &_nkept_proc_fops);
        dg->sfile  = _nk_proc_create(dg->dir, "status",  &_nksts_proc_fops);

        while (gb) {
            gb->file = _nk_proc_create(dg->dir, gb->name, &_nkbank_proc_fops);
            gb = gb->next;
        }

        dg = dg->next;
    }
    return 1;
}

    static void
_guests_release (void)
{
    DynGuest* dg = guests;
    while (dg) {
        DynGuest*  tdg;
        GuestBank* gb = dg->banks;

        while (gb) {
            GuestBank* tgb;
            remove_proc_entry(gb->name, dg->dir);
            tgb = gb;
            gb  = gb->next;
            kfree(tgb);
        }

        remove_proc_entry("cmdline", dg->dir);
        remove_proc_entry("epoint", dg->dir);
        remove_proc_entry("status", dg->dir);

        tdg = dg;
        dg  = dg->next;
        remove_proc_entry(tdg->name, nk);
        kfree(tdg);
    }

    guests = NULL;
}

    static void
nkproc_sysconf_hdl (void* cookie, NkXIrq xirq)
{
    NkOsMask  running = nkops.nk_running_ids_get();
    DynGuest* dg = guests;

    while (dg) {
        dg->started = ((running >> dg->id) & 1);
        if (!dg->started) {
            dg->suspended = 0;
        }
        dg = dg->next;
    }
}

static void _nkcontrol_module_exit (void);

    static int
_nkcontrol_module_init (void)
{
    NkPhAddr   paddr;
    BankDesc*  banks;
    NkBootInfo vinfo;
    NkBankInfo bankInfo;
    int        err;

    nk = proc_mkdir("nk", NULL);

    if (!nk) {
        printk("NK: error -- proc_mkdir(/proc/nk) failed\n");
        return 0;
    }
    _nk_proc_create(nk, "restart", &_nkrestart_proc_fops);
    _nk_proc_create(nk, "stop",    &_nkstop_proc_fops);
    _nk_proc_create(nk, "resume",  &_nkresume_proc_fops);
    _nk_proc_create(nk, "focus",   &_nkevent_proc_fops);


    vinfo.data = nkops.nk_vtop(&bankInfo);
    vinfo.ctrl = INFO_BOOTCONF;
    paddr      = nkops.nk_vtop(&vinfo);
    err        = os_ctx->binfo(os_ctx, paddr);

    if (err) {
         return 0;
    }

    if (!(banks = (BankDesc*)(nkops.nk_ptov(bankInfo.banks)))) {
        printk("NK -- error: cannot map bankDesc\n");
        return 0;
    }

    if (!_guests_init(bankInfo.numBanks, banks)) {
        printk("NK -- error: cannot init guests descriptors\n");
        return 0;
    }

    if (!_guests_proc_init()) {
        printk("NK -- error: cannot init procfs entries\n");
        return 0;
    }

    scf_xid  = nkops.nk_xirq_attach(NK_XIRQ_SYSCONF, nkproc_sysconf_hdl, NULL);
    if (!scf_xid) {
        printk("NK -- error: can't attach sysconf handler\n");
        _nkcontrol_module_exit();
    }

    return 0;
}

    static void
_nkcontrol_module_exit (void)
{
    if (scf_xid) {
        nkops.nk_xirq_detach(scf_xid);
    }

    if (nk) {
        remove_proc_entry("restart", nk);
        remove_proc_entry("stop",    nk);
        remove_proc_entry("resume",  nk);
        remove_proc_entry("focus",   nk);
        _guests_release();
        remove_proc_entry("nk", NULL);
    }
}

module_init(_nkcontrol_module_init);
module_exit(_nkcontrol_module_exit);
