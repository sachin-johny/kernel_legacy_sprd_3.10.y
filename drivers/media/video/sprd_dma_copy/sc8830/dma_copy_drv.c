/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/file.h>
#include <mach/dma.h>
#include <linux/sched.h>
#include <video/sprd_dma_copy_k.h>
#include <linux/slab.h>

#if 0
#define DMA_COPY_PRINT printk
#else
#define DMA_COPY_PRINT pr_debug
#endif
#define DMA_COPY_TIMEOUT 5000/*ms*/
#define DMA_COPY_MINOR MISC_DYNAMIC_MINOR
#define DMA_COPY_USER_MAX 4
#define INVALID_USER_ID PID_MAX_DEFAULT

struct dm_copy_user {
	pid_t pid;
};

struct dm_copy_ctx {
	struct semaphore sem_dev_open;
	struct semaphore sem_copy;
};

static struct dm_copy_user *g_dma_copy_user = NULL;
static struct dm_copy_ctx g_dma_copy_ctx; 
static struct dm_copy_ctx *g_dma_copy_ctx_ptr = &g_dma_copy_ctx;

static struct dm_copy_user *dma_copy_get_user(pid_t user_pid)
{
	struct dm_copy_user *ret_user = NULL;
	int i;

	for (i = 0; i < DMA_COPY_USER_MAX; i ++) {
		if ((g_dma_copy_user + i)->pid == user_pid) {
			ret_user = g_dma_copy_user + i;
			break;
		}
	}

	if (ret_user == NULL) {
		for (i = 0; i < DMA_COPY_USER_MAX; i ++) {
			if ((g_dma_copy_user + i)->pid == INVALID_USER_ID) {
				ret_user = g_dma_copy_user + i;
				ret_user->pid = user_pid;
				break;
			}
		}
	}

	return ret_user;
}

int dma_copy_k_open(struct inode *node, struct file *file)
{
	struct dm_copy_user *p_user = NULL;
	int ret = 0;

	DMA_COPY_PRINT("dma_copy_k_open start");
	down(&(g_dma_copy_ctx_ptr->sem_dev_open));
	p_user = dma_copy_get_user(current->pid);
	if (NULL == p_user) {
		printk("dma_copy_k_open user cnt full  pid:%d. \n",current->pid);
		up(&(g_dma_copy_ctx_ptr->sem_dev_open));
		return -1;
	}
	file->private_data = p_user;
	up(&(g_dma_copy_ctx_ptr->sem_dev_open));
	DMA_COPY_PRINT("dma_copy_k_open end");

	return ret;
}

int dma_copy_k_release(struct inode *node, struct file *file)
{
	DMA_COPY_PRINT("dma_copy_k_release");

	((struct dm_copy_user *)(file->private_data))->pid = INVALID_USER_ID;

	return 0;
}

static int dma_copy_k_start(DMA_COPY_CFG_T * param_ptr)
{
	int32_t ret = 0;

	if (0 == param_ptr->src_addr || 0 == param_ptr->dst_addr ||
		0 == param_ptr->len)
		return -1;

	DMA_COPY_PRINT("dma_copy_k_start  src=%x dst=%x len=%d \n",
		param_ptr->src_addr,param_ptr->dst_addr,param_ptr->len);

	ret = sci_dma_memcpy(param_ptr->dst_addr, param_ptr->src_addr, param_ptr->len);

	return ret;
}

static long dma_copy_k_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	down(&(g_dma_copy_ctx_ptr->sem_copy));
	{
		int ret = 0;
		DMA_COPY_CFG_T params;

		ret = copy_from_user(&params, (DMA_COPY_CFG_T *) arg, sizeof(DMA_COPY_CFG_T));
		if (0 == ret){
			if (dma_copy_k_start(&params)) {
				printk("dma_copy_k_ioctl fail. \n");
				ret = -EFAULT;
			}
		}
		up(&(g_dma_copy_ctx_ptr->sem_copy));
		return ret;
	}
}

static struct file_operations dma_copy_fops = {
	.owner = THIS_MODULE,
	.open = dma_copy_k_open,
	.unlocked_ioctl = dma_copy_k_ioctl,
	.release = dma_copy_k_release,
};

static struct miscdevice dma_copy_dev = {
	.minor = DMA_COPY_MINOR,
	.name = "sprd_dma_copy",
	.fops = &dma_copy_fops,
};

int dma_copy_k_probe(struct platform_device *pdev)
{
	int ret, i;
	struct dm_copy_user *p_user;
	printk(KERN_ALERT "dma_copy_k_probe called\n");

	ret = misc_register(&dma_copy_dev);
	if (ret) {
		printk(KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
			DMA_COPY_MINOR, ret);
		return ret;
	}

	g_dma_copy_user = kzalloc(DMA_COPY_USER_MAX * sizeof(struct dm_copy_user), GFP_KERNEL);
	if (NULL == g_dma_copy_user) {
		printk("dm_copy_user, no mem");
		return -1;
	}

	p_user = g_dma_copy_user;
	for (i = 0; i < DMA_COPY_USER_MAX; i++) {
		p_user->pid = INVALID_USER_ID;
		p_user ++;
	}

	printk(KERN_ALERT " dma_copy_k_probe Success\n");
	return 0;
}

static int dma_copy_k_remove(struct platform_device *dev)
{
	printk(KERN_INFO "dma_copy_k_remove called !\n");
	if (g_dma_copy_user) {
		kfree(g_dma_copy_user);
	}
	misc_deregister(&dma_copy_dev);
	printk(KERN_INFO "dma_copy_k_remove Success !\n");
	return 0;
}

static struct platform_driver dma_copy_driver = {
	.probe = dma_copy_k_probe,
	.remove = dma_copy_k_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_dma_copy",
		},
};

int __init dma_copy_k_init(void)
{
	printk(KERN_INFO "dma_copy_k_init called !\n");
	if (platform_driver_register(&dma_copy_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}
	sema_init(&(g_dma_copy_ctx_ptr->sem_dev_open), 1);
	sema_init(&(g_dma_copy_ctx_ptr->sem_copy), 1);
	return 0;
}

void dma_copy_k_exit(void)
{
	printk(KERN_INFO "dma_copy_k_exit called !\n");
	platform_driver_unregister(&dma_copy_driver);
}

module_init(dma_copy_k_init);
module_exit(dma_copy_k_exit);

MODULE_DESCRIPTION("DMA Copy Driver");
MODULE_LICENSE("GPL");
