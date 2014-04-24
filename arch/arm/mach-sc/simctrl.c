/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/pinmap.h>
//#include "internal.h"

#define	STRING_LEN	4

struct simdesc {
	/* may be "0", "1", ... */
	char		name[STRING_LEN];
	/* may be "t", "w" */
	char		type[STRING_LEN];

	uint32_t	addr;
	uint32_t	bitm;

	struct proc_dir_entry	*node;
};

#ifdef CONFIG_ARCH_SCX35

static struct simdesc sim_cards[] = {
	{ "0", "t", 0x08, 0x00100000 },
	{ "1", "w", 0x08, 0x00400000 },
};
static int sim_count = ARRAY_SIZE(sim_cards);
#else
static struct simdesc sim_cards[] = {
};
static int sim_count = ARRAY_SIZE(sim_cards);
#endif

static struct proc_dir_entry *simdir;

static int simctrl_write_proc(struct file *file, const char __user *buffer,
			   unsigned long count, void *data);
static int simctrl_open_proc(struct inode *inode, struct file *filp);
static ssize_t simctrl_read_proc(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos);

static const struct file_operations simctrl_ops = {
	.owner		= THIS_MODULE,
    .open		= simctrl_open_proc,
	.read		= simctrl_read_proc,
	.write		= simctrl_write_proc,
};


static int simctrl_open_proc(struct inode *inode, struct file *filp)
{
	struct simdesc *entry = (struct simdesc *)PDE_DATA(inode);
	filp->private_data = entry;

	return 0;
}

static ssize_t simctrl_read_proc(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct simdesc *entry = (struct simdesc *)filp->private_data;

	if (*ppos >= strlen(entry->type)) {
		return 0;
	}
	
	if (copy_to_user(buf, entry->type, strlen(entry->type))) {
		printk("simctrl read proc: copy error \n");
		return -EFAULT;
	}

	count = strlen(entry->type);
	*ppos += count;

    return count;
}

static int simctrl_write_proc(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	char sim_type[STRING_LEN];
	struct simdesc *sim = (struct simdesc *)data;
	uint32_t value;

	count = (count < STRING_LEN) ? count : STRING_LEN -1;
	if (copy_from_user(sim_type, buffer, count))
		return -EFAULT;

	if (sim_type[count - 1] == '\n') {
		sim_type[count - 1] = '\0';
		
	} else {
		sim_type[count] = '\0';
	}

	if (strcmp(sim_type, "t") == 0) {

		value = pinmap_get(sim->addr) | sim->bitm;
		pinmap_set(sim->addr, value);
	} else if (strcmp(sim_type, "w") == 0) {
		value = pinmap_get(sim->addr) & ~sim->bitm;
		pinmap_set(sim->addr, value);
	} else {
		pr_info("Unknow sim type %s\n", sim_type);
		return count;
	}

	strcpy(sim->type, sim_type);
	return count;
}

static int simctrl_register(struct simdesc *sim)
{
	struct proc_dir_entry *node;

	node = proc_create_data(sim->name, S_IWUSR | S_IRUSR, simdir, &simctrl_ops, sim);
	//create_proc_entry(sim->name, S_IWUSR | S_IRUSR, simdir);
	if (!node) {
		return -EFAULT;
	}
	
	sim->node = node;

	return 0;
}

static int __init simctrl_init(void)
{
	int i;

	simdir = proc_mkdir("sim", NULL);

	for (i = 0; i < sim_count; i++)
		simctrl_register(&sim_cards[i]);

	return 0;
}

module_init(simctrl_init);
