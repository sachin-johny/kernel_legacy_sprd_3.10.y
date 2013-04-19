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
#include <linux/sched.h>
#include <video/sprd_rot_k.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include "img_rot.h"
#include <linux/delay.h>

#define RTT_PRINT pr_debug
//#define RTT_PRINT printk
//#define ROTATION_DEBUG 0

#define ROT_TIMEOUT 100/*ms*/
#define ROTATION_MINOR MISC_DYNAMIC_MINOR
#define ROT_USER_MAX 4
#define INVALID_USER_ID PID_MAX_DEFAULT

struct rot_context {
	atomic_t start_flag;
	struct timer_list rot_timer;
};

struct rot_user {
	pid_t pid;
	uint32_t is_exit_force;
	uint32_t is_rot_enable;
	struct semaphore sem_open;
	struct semaphore sem_done;
};

static wait_queue_head_t wait_queue;
static struct semaphore g_sem_dev_open;
static struct semaphore g_sem_rot;
static struct rot_context rot_conext;
static struct rot_context *rot_cnt = &rot_conext;
static struct rot_user *g_rot_user = NULL;
static atomic_t rot_users = ATOMIC_INIT(0);
static int rot_condition;
static pid_t cur_task_pid;

static struct rot_user *rot_get_user(pid_t user_pid)
{
	struct rot_user *ret_user = NULL;
	int i;

	for (i = 0; i < ROT_USER_MAX; i ++) {
		if ((g_rot_user + i)->pid == user_pid) {
			ret_user = g_rot_user + i;
			break;
		}
	}

	if (ret_user == NULL) {
		for (i = 0; i < ROT_USER_MAX; i ++) {
			if ((g_rot_user + i)->pid == INVALID_USER_ID) {
				ret_user = g_rot_user + i;
				ret_user->pid = user_pid;
				break;
			}
		}
	}

	return ret_user;
}

static int rot_start_timer(struct timer_list *rot_timer, uint32_t time_val)
{
	int ret;
	ret = mod_timer(rot_timer, jiffies + msecs_to_jiffies(time_val));
	if (ret)
		printk("rot:Error in mod_timer\n");
	return 0;
}

static void rot_stop_timer(struct timer_list *rot_timer)
{
	del_timer_sync(rot_timer);
}

static void rot_timer_callback(unsigned long data)
{
	struct rot_user *p_user = NULL;

	if (1 == atomic_read(&rot_cnt->start_flag)) {
		printk("rot timeout.\n");
		atomic_set(&rot_cnt->start_flag, 0);
		p_user = rot_get_user(cur_task_pid);
		up(&p_user->sem_done);

	}
}

static int rot_init_timer(struct timer_list *rot_timer)
{
	RTT_PRINT("Timer module installing\n");
	setup_timer(rot_timer, rot_timer_callback, 0);
	RTT_PRINT("Timer module installing e\n");
	return 0;
}

static void rot_k_irq(void)
{
	RTT_PRINT("%s, come\n", __func__ );
	rot_condition = 1;
	wake_up(&wait_queue);
	RTT_PRINT("rotation_dma_irq X .\n");
}

static int rot_k_condition_init(void)
{
	rot_condition = 0;
	return 0;
}

static void rot_k_wait_stop(void)
{
	RTT_PRINT("rotation_dma_wait_stop E .\n");
	wait_event(wait_queue, rot_condition);
	RTT_PRINT("ok to rotation_dma_wait_stop.\n");
}

static int rot_k_start(void)
{
	int ret = 0;
	struct rot_user *p_user = NULL;

	atomic_set(&rot_cnt->start_flag, 1);
	rot_start_timer(&rot_cnt->rot_timer,ROT_TIMEOUT);

	rot_k_condition_init();
	rot_k_done();
	rot_k_wait_stop();

	if (0 == rot_k_is_end()) {
		RTT_PRINT("rot_k_start y done, uv start. \n");
		rot_k_condition_init();
		rot_k_set_UV_param();
		rot_k_done();
		rot_k_wait_stop();
	}

	rot_k_disable();
	atomic_set(&rot_cnt->start_flag, 0);
	rot_stop_timer(&rot_cnt->rot_timer);
	p_user = rot_get_user(cur_task_pid);
	up(&p_user->sem_done);

	RTT_PRINT("rot_k_thread  done \n");

	return ret;
}

static int rot_k_open(struct inode *node, struct file *file)
{
	struct rot_user *p_user = NULL;
	int ret = 0;

	down(&g_sem_dev_open);

	p_user = rot_get_user(current->pid);
	if (NULL == p_user) {
		printk("rot_k_open user cnt full  pid:%d. \n",current->pid);
		return -1;
	}
	file->private_data = p_user;
	if (1 == atomic_inc_return(&rot_users)) {
		ret = rot_k_module_en();
		if (unlikely(ret)) {
			printk("Failed to enable rot module \n");
			ret = -EIO;
			goto faile;
		}

		ret = rot_k_isr_reg(rot_k_irq);
		if (unlikely(ret)) {
			printk("Failed to register rot ISR \n");
			ret = -EACCES;
			goto reg_faile;
		} else {
			goto exit;
		}
	} else {
		goto exit;
	}
reg_faile:
	rot_k_module_dis();
faile:
	atomic_dec(&rot_users);
	p_user->pid = INVALID_USER_ID;
	file->private_data = NULL;
exit:
	up(&g_sem_dev_open);

	return ret;

}

static ssize_t rot_k_write(struct file *file, const char __user * u_data, size_t cnt, loff_t *cnt_ret)
{
	(void)file; (void)u_data; (void)cnt_ret;
	((struct rot_user *)(file->private_data))->is_exit_force = 1;
	up(&(((struct rot_user *)(file->private_data))->sem_done));

	return 1;
}

static int rot_k_release(struct inode *node, struct file *file)
{
	((struct rot_user *)(file->private_data))->pid = INVALID_USER_ID;

	if (0 == atomic_dec_return(&rot_users)) {
		rot_k_isr_unreg();
		rot_k_module_dis();
	}

	return 0;
}

static long rot_k_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	RTT_PRINT("rot_k_ioctl, 0x%x \n", cmd);

	switch (cmd) {
	case ROT_IO_CFG:
		down(&g_sem_rot);
		{
			int ret = 0;
			ROT_CFG_T params;

			cur_task_pid = ((struct rot_user *)(file->private_data))->pid;
			((struct rot_user *)(file->private_data))->is_rot_enable = 1;

			ret = copy_from_user(&params, (ROT_CFG_T *) arg, sizeof(ROT_CFG_T));
			if (0 == ret){
				ret = rot_k_io_cfg(&params);
			}

			if(ret) {
				printk("rot_k_ioctl   fail.\n");
				up(&g_sem_rot);
			}

			RTT_PRINT("rot_k_ioctl, ROT_IO_CFG, %d \n", ret);
			return ret;
		}

	case ROT_IO_START:
		{
			int ret = 0;

			if (rot_k_start()) {
				ret = -EFAULT;
				up(&g_sem_rot);
			}

			RTT_PRINT("rot_k_ioctl, ROT_IO_START, %d \n", ret);
			return ret;
		}

	case ROT_IO_IS_DONE:
		{
			down(&(((struct rot_user *)(file->private_data))->sem_done));
			{
				int ret = 0;
				if (((struct rot_user *)(file->private_data))->is_exit_force) {
					((struct rot_user *)(file->private_data))->is_exit_force = 0;
					ret = -1;
				}

				if(((struct rot_user *)(file->private_data))->is_rot_enable) {
					((struct rot_user *)(file->private_data))->is_rot_enable = 0;
					up(&g_sem_rot);
				}

				return ret;
			}
		}

	default:
		return 0;
	}

}

static struct file_operations rotation_fops = {
	.owner = THIS_MODULE,
	.open = rot_k_open,
	.write = rot_k_write,
	.unlocked_ioctl = rot_k_ioctl,
	.release = rot_k_release,
};

static struct miscdevice rotation_dev = {
	.minor = ROTATION_MINOR,
	.name = "sprd_rotation",
	.fops = &rotation_fops,
};

#if 0
struct task_struct *g_rot_task_test;
ROT_ANGLE_E rot_angle_test = ROT_90;
ROT_DATA_FORMAT_E data_format_test = ROT_YUV422;
void rot_k_irq_test(void)
{
	RTT_PRINT("%s, come\n", __func__ );
	rot_condition = 1;
	wake_up(&wait_queue);
	RTT_PRINT("rotation_dma_irq X .\n");
}

int rot_k_condition_init_test(void)
{
	rot_condition = 0;
	return 0;
}

void rot_k_wait_stop_test(void)
{
	RTT_PRINT("rotation_dma_wait_stop E .\n");
	wait_event(wait_queue, rot_condition);
	RTT_PRINT("ok to rotation_dma_wait_stop.\n");
}

int rot_k_open_test(void)
{
	struct rot_user *p_user = NULL;
	int ret = 0;

	down(&g_sem_dev_open);

	p_user = rot_get_user(current->pid);
	if (NULL == p_user) {
		printk("rot_k_open user cnt full  pid:%d. \n",current->pid);
		return -1;
	}

	if (1 == atomic_inc_return(&rot_users)) {
		ret = rot_k_module_en();
		if (unlikely(ret)) {
			printk("Failed to enable rot module \n");
			ret = -EIO;
			goto faile;
		}

		ret = rot_k_isr_reg(rot_k_irq_test);
		if (unlikely(ret)) {
			printk("Failed to register rot ISR \n");
			ret = -EACCES;
			goto reg_faile;
		} else {
			goto exit;
		}
	} else {
		goto exit;
	}
reg_faile:
	rot_k_module_dis();
faile:
	atomic_dec(&rot_users);
	p_user->pid = INVALID_USER_ID;
exit:
	up(&g_sem_dev_open);

	return ret;

}

int rot_k_release_test(void)
{
	if (0 == atomic_dec_return(&rot_users)) {
		rot_k_isr_unreg();
		rot_k_module_dis();
	}

	return 0;
}

long rot_k_ioctl_test(void)
{
	int ret = 0;
	ROT_CFG_T params = {0};

	params.angle = rot_angle_test;
	params.format = data_format_test;
	params.img_size.w = 240;
	params.img_size.h = 320;
	params.src_addr.y_addr = 0x8C800000;
	params.src_addr.u_addr = 0x8C900000;
	params.src_addr.v_addr = 0x00;
	params.dst_addr.y_addr = 0x8CA00000;
	params.dst_addr.u_addr = 0x8CB00000;
	params.dst_addr.v_addr = 0x00;
	params.src_endian = 1;
	params.dst_endian = 1;

	rot_k_io_cfg(&params);

	return ret;
}

static int rot_k_thread_test(void *data_ptr)
{
while(1) {
	rot_k_open_test();

	rot_k_ioctl_test();

	RTT_PRINT("rot_k_thread y start \n");
	rot_k_condition_init_test();
	rot_k_done();
	rot_k_wait_stop_test();

	if (0 == rot_k_is_end()) {
		RTT_PRINT("rot_k_thread y done, uv start \n");
		rot_k_condition_init_test();
		rot_k_set_UV_param();
		rot_k_done();
		rot_k_wait_stop_test();
	}

	rot_k_disable();

	RTT_PRINT("rot_k_thread  done \n");

	rot_k_release_test();
	}
return 0;
}
#endif

int rot_k_probe(struct platform_device *pdev)
{
	int ret, i;
	struct rot_user *p_user;
	printk(KERN_ALERT "rot_k_probe called\n");

	ret = misc_register(&rotation_dev);
	if (ret) {
		printk(KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
			ROTATION_MINOR, ret);
		return ret;
	}

	g_rot_user = kzalloc(ROT_USER_MAX * sizeof(struct rot_user), GFP_KERNEL);
	if (NULL == g_rot_user) {
		printk("rot_user, no mem");
		return -1;
	}

	sema_init(&g_sem_rot, 1);
	sema_init(&g_sem_dev_open, 1);
	init_waitqueue_head(&wait_queue);
	rot_init_timer(&rot_cnt->rot_timer);
	p_user = g_rot_user;
	for (i = 0; i < ROT_USER_MAX; i++) {
		p_user->pid = INVALID_USER_ID;
		p_user->is_exit_force = 0;
		p_user->is_rot_enable = 0;
		sema_init(&p_user->sem_open, 1);
		sema_init(&p_user->sem_done, 0);
		p_user ++;
	}
#if 0
	g_rot_task_test = kthread_create(rot_k_thread_test, NULL, "rot thread test");
	if (g_rot_task_test == 0) {
		printk("rot: create thread error \n");
		return -1;
	}else{
		wake_up_process(g_rot_task_test);
	}
#endif
	printk(KERN_ALERT " rot_k_probe Success\n");
	return 0;
}

static int rot_k_remove(struct platform_device *dev)
{
	printk(KERN_INFO "rot_k_remove called !\n");
	misc_deregister(&rotation_dev);
	printk(KERN_INFO "rot_k_remove Success !\n");
	return 0;
}

static struct platform_driver rotation_driver = {
	.probe = rot_k_probe,
	.remove = rot_k_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "sprd_rotation",
		},
};

int __init rot_k_init(void)
{
	printk(KERN_INFO "rot_k_init called !\n");
	if (platform_driver_register(&rotation_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}
	return 0;
}

void rot_k_exit(void)
{
	printk(KERN_INFO "rot_k_exit called !\n");
	platform_driver_unregister(&rotation_driver);
}

module_init(rot_k_init);
module_exit(rot_k_exit);

MODULE_DESCRIPTION("rotation Driver");
MODULE_LICENSE("GPL");

