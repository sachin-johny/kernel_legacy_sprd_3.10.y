/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: hw_dv.c
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/
#include <linux/types.h>
#include <linux/kallsyms.h>
#include "trout_tx.h"
#include "trout_rx.h"
#include "virt_irq.h"
#include "dv_ioctl.h"
#include "normal_reg.h"
#include "sim.h"
#include "trout_reg_def.h"
#include "trout.h"

int hwDvRegReadAll(char *pBuf, int bufLen, int kernMode)
{
	return 0;
}

int hwDvRegWriteAll(char *pBuf, int bufLen, int kernMode)
{
	return 0;
}

static inline wait_queue_head_t *get_tx_wait_queue_head(void)
{
	static DECLARE_WAIT_QUEUE_HEAD(wait_head);

	return &wait_head;
}

void wake_up_trout_send(void)
{
	wake_up_interruptible(get_tx_wait_queue_head());
}

int hwDvDataRxInWrite(DV_IOCTL_T *ioctl, char *pBuf)
{
	return trout_recv_from(ioctl, pBuf);
}

int hwDvDataTxoutRead(struct file *filp, DV_IOCTL_T *ioctl, char *pBuf)
{
	int ret;
	DEFINE_WAIT(wait);
	wait_queue_head_t *wq = get_tx_wait_queue_head();

	while (1) {
		prepare_to_wait(wq, &wait, TASK_INTERRUPTIBLE);
		ret = trout_send_to(ioctl, pBuf);

		if (ret != -EAGAIN || filp->f_flags & O_NONBLOCK) {
			break;
		}

		schedule();

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
	};

	finish_wait(wq, &wait);

	return ret;
}

int hwDvErrCodeGet(void)
{
	return 0;
}

int hwDvSimIsDone(int timeout)
{
	return 0;
}

int hwDvRegReset(void)
{
	reg_space_reset();
	return 0;
}

static int ioctl_intr_opt(DV_IOCTL_T *ioctl)
{
	int enable = ioctl->data;
	unsigned int irqno = ioctl->dataLen;

	return (enable) ? enable_virt_irq(irqno) : disable_virt_irq(irqno);
}

static int set_intr_status(DV_IOCTL_T *ioctl)
{
	u32 flags = ioctl->data;

	dv_set_reg_bit(flags, rCOMM_INT_STAT);
	return 0;
}

int hwDvUsrDefineOp(DV_IOCTL_T *ioctl)
{
	switch (ioctl->addr) {
	case USER_INT_OPT:
		return ioctl_intr_opt(ioctl);
	case USER_INT_STATUS:
		return set_intr_status(ioctl);
	default:
		DV_DBG("invalid user defined opt code: %d\n", ioctl->addr);
		return -EINVAL;
	}
}

int get_kern_addr(DV_IOCTL_T *ioctl)
{
	char var_name[KSYM_NAME_LEN + 1];
	typedef unsigned long (*lookup_t)(const char *);
	unsigned long lookup_fun_addr;

	if (strncpy_from_user(var_name, (const char __user *)ioctl->addr,
			sizeof(var_name)) < 0) {
		return -EFAULT;
	}

	lookup_fun_addr = kallsyms_lookup_name("module_kallsyms_lookup_name");
	if (!lookup_fun_addr) {
		return -ENOENT;
	}

	ioctl->data = ((lookup_t)lookup_fun_addr)(var_name);
	ioctl->dataLen = sizeof(ioctl->data);

	if (!ioctl->data) {
		return -ENXIO;
	}

	return 0;
}

int get_kern_data(DV_IOCTL_T *ioctl)
{
	if (copy_to_user((void __user *)ioctl->data, (const void *)ioctl->addr,
			ioctl->dataLen)) {
		return -EFAULT;
	} else {
		return 0;
	}
}

int set_kern_data(DV_IOCTL_T * ioctl)
{
	if (copy_from_user((void *)ioctl->addr, (const void __user *)ioctl->data, 
			ioctl->dataLen)) {
		return -EFAULT;
	} else {
		return 0;
	}
}

int hwDvSimInit(void)
{
	return hwSimInit();
}

void hwDvSimExit(void)
{
	hwSimExit();
}


/*
 *$Log$
 */


