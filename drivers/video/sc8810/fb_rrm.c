/* drivers/video/sc8800g/sc8800g_rrm.c
 *
 * SC8800g LCDC Refresh Request Manager.
 *
 * Copyright (C) 2010 Spreadtrum 
 * 
 * Author: Geng Ren <geng.ren@spreadtrum.com>
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <linux/wait.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/mm.h>

#include "fb_rrm.h"
#include "lcdc_manager.h"

/* #define  RRM_DEBUG  */
#ifdef RRM_DEBUG
#define RRM_PRINT printk
#else
#define RRM_PRINT(...)
#endif

/* frame_state */
#define FS_IDLE 0
#define FS_BUSY 1

#define DEFAULT_BUF_NUM 2

static struct rrmanager rrm;
static DECLARE_WAIT_QUEUE_HEAD(idle_wq);


/* assumed to be called with irq off, so no race condition to be worry about.
 * return available num */ 
static int rrq_enqueue(struct rrqueue *self, struct rr *r)
{
	self->q_ptr[self->index_in++] = *r;
	RRM_PRINT("RRM[%s] c %x, d %x, in %d, avl %d\n", __FUNCTION__,
			self->q_ptr[(self->index_in -1)].callback,
			self->q_ptr[(self->index_in -1)].data,
			self->index_in, (self->available - 1));
	if (self->index_in == self->depth)
		self->index_in = 0;

	self->available--;

	return self->available;
}

/* assumed to be called in irq context (irq disabled), therefore no need to 
 * worry about race condition too.
 * return -1 when queue is empty, return available num otherwise */
static int rrq_dequeue(struct rrqueue *self, struct rr *r)
{
	RRM_PRINT("RRM[%s] availabe:%d\n", __FUNCTION__, self->available);
	if (self->available == self->depth){
		r->callback = NULL;
		r->data = NULL;
		return -1; /* queue empty */
	}

	*r = self->q_ptr[self->index_out++];
	if (self->index_out == self->depth)
		self->index_out = 0;

	self->available++;

	return self->available;    
}

static int rrq_wait(struct rrqueue *self)
{
	if (down_timeout(&self->waitlock, msecs_to_jiffies(500)) != 0) {
             return -1;
	}

	return 0;
}

static int rrq_wakeup(struct rrqueue *self)
{
	up(&self->waitlock);
	return 0;
}

static struct rrqueue * rrq_init(int buf_num)
{
	struct rrqueue *rrq = 0;

	if (buf_num < 1) {
		printk(KERN_ERR "RRM[%d]: illegal buf_num\n", __LINE__);
		return NULL;
	}

	rrq = kzalloc(sizeof(struct rrqueue),GFP_NOWAIT);

	if(!rrq) {
		printk(KERN_ERR "RRM[%d]: no memory!\n", __LINE__);
		return NULL;
	}

	rrq->depth = buf_num;
	rrq->available = buf_num;

	rrq->q_ptr = kzalloc(sizeof(struct rr) * rrq->depth, GFP_NOWAIT);
	if(!rrq->q_ptr) {
		printk(KERN_ERR "RRM[%d]: no memory!\n", __LINE__);
		goto _rrq_init_err;
	}

	sema_init(&rrq->waitlock, 0);

	rrq->enqueue = rrq_enqueue;
	rrq->dequeue = rrq_dequeue;
	rrq->wait = rrq_wait;
	rrq->wakeup = rrq_wakeup;

	return rrq;

_rrq_init_err:
	kfree(rrq);
	return NULL;
}

static void rrq_release(struct rrqueue *rrq)
{
	kfree(rrq->q_ptr);
	kfree(rrq);
	return;
}

static void rre_refresh(struct rrexecuter *self, int id, struct rr *r)
{
	if (r == NULL) {
		int i;

		RRM_PRINT("RRM[%s] from irq\n", __FUNCTION__);
		for(i=0;i<rrm.layer_num;i++) {
			if (self->set_layer[i] && self->executing[i].data)
				self->set_layer[i](self->executing[i].data);
		}
	} else {
		RRM_PRINT("RRM[%s] direct execute\n", __FUNCTION__);
		if (self->set_layer[id])
			self->set_layer[id](r->data);
		self->executing[id] = *r;
	}

	if (self->hw_refresh)
		self->hw_refresh(self->para);
}

static struct rrexecuter * rre_init(int layer_num, 
			void (*hw_refresh)(void* p), void * para)
{
	struct rrexecuter *rre = 0;

	rre = kzalloc(sizeof(struct rrexecuter),GFP_KERNEL);
	if(!rre) {
		printk(KERN_ERR "RRM[%d]: no memory!\n", __LINE__);
		return NULL;
	}

	rre->executing = kzalloc(sizeof(struct rr) * layer_num, GFP_KERNEL);
	if(!rre->executing) {
		printk(KERN_ERR "RRM[%d]: no memory!\n", __LINE__);
		goto _rre_init_err_1;
	}

	rre->set_layer = kzalloc(sizeof(void*) * layer_num, GFP_KERNEL);
	if (!rre->set_layer) {
		printk(KERN_ERR "RRM[%d]: no memory!\n", __LINE__);
		goto _rre_init_err_2;
	}

	rre->refresh = rre_refresh;
	rre->hw_refresh = hw_refresh;
	rre->para = para;

	return rre;

_rre_init_err_2:
	kfree(rre->executing);
_rre_init_err_1:
	kfree(rre);
	return NULL;
}

int rrm_refresh(int id, void (*callback)(void* data), void *data)
{
	unsigned long flags;
	struct rr r = {callback, data};

	/* acquire lcdc first */
	if(lm_acquire(id)) {
		printk(KERN_ERR "lm_acquire failed!\n");
		return -1;
	}

	spin_lock_irqsave(&rrm.lock, flags);

	if (rrm.frame_state == FS_BUSY) {
		int available;
		available = rrm.que[id]->enqueue(rrm.que[id], &r);
		if(available == 0) {
			/* we have just used the last slot, we need to wait */
			/* until a release happened */ 
			RRM_PRINT("RRM[%s] layer[%d] is waiting\n",
					__FUNCTION__, id);
			spin_unlock_irqrestore(&rrm.lock, flags);
			if (rrm.que[id]->wait(rrm.que[id]) != 0) {
				rrm.frame_state = FS_IDLE;
				return -1;
			}
			spin_lock_irqsave(&rrm.lock, flags); 
			RRM_PRINT("RRM[%s] layer[%d] has waked up\n",
					__FUNCTION__, id);
		}
	} else {
		rrm.frame_state = FS_BUSY;
		rrm.exec->refresh(rrm.exec, id, &r);
	}

	spin_unlock_irqrestore(&rrm.lock, flags);
	return 0;
}
EXPORT_SYMBOL(rrm_refresh);

void rrm_interrupt(struct rrmanager *rrm)
{
	int i;
	int cnt = 0;

	/* invoke callbacks of the requests */
	for(i=0; i< rrm->layer_num; i++) {
		RRM_PRINT("RRM[%s] callback[%d]:%x\n", __FUNCTION__,
				i, rrm->exec->executing[i].callback);
		/* FIXME: assume all requests have data */
		if (rrm->exec->executing[i].data != NULL)
			lm_release(i);

		if (rrm->exec->executing[i].callback != NULL) {
			rrm->exec->executing[i].callback(
					rrm->exec->executing[i].data);
			rrm->exec->executing[i].callback = NULL;
		}
		rrm->exec->executing[i].data = NULL;
	}

	/* collect requests in queue */
	for(i=0; i< rrm->layer_num; i++) {
		int rt;
		rt=rrm->que[i]->dequeue(rrm->que[i], &rrm->exec->executing[i]);
		if(rt >= 0) /* not empty */
			cnt++;
		if(rt == 1) /* there's someone waiting */
			rrm->que[i]->wakeup(rrm->que[i]);
	}

	RRM_PRINT("RRM[%s] found %d layers request\n", __FUNCTION__, cnt);

	if (cnt == 0) {
		/* no request collected */
		rrm->frame_state = FS_IDLE;
		wake_up(&idle_wq);
	} else {
		rrm->exec->refresh(rrm->exec, 0, NULL);
	}
}

void rrm_wait_for_idle(void)
{
	wait_event_timeout(idle_wq, (rrm.frame_state == FS_IDLE), HZ);
}

/* TEMP */
extern int enable_layer(int id);
extern int disable_layer(int id);

int rrm_layer_init(int id, int buf_num, void (*set_layer)(void *data))
{
	unsigned long flags;

	if(id >= LID_OSD2) /* FIXME: hardcoded layer usage */
		return -1;

	if (rrm.que == NULL) {
		printk(KERN_ERR "RRM is not initialized!\n");
		return -1;
	}

	spin_lock_irqsave(&rrm.lock, flags);

	if (lm_register_layer(id, LMODE_DISPLAY, enable_layer, disable_layer)){
		printk(KERN_ERR "lm_regsiter_layer failed!\n");
		return -1;
	}
	lm_enable_layer(id);

	if (rrm.que[id] != NULL)
		rrq_release(rrm.que[id]);

	/* que depth should be 1 less than the buf num */
	rrm.que[id] = rrq_init(buf_num - 1);

	rrm.exec->set_layer[id] = set_layer;

	spin_unlock_irqrestore(&rrm.lock, flags);

	return 0;
}
EXPORT_SYMBOL(rrm_layer_init);

int rrm_layer_exit(int id)
{
	lm_disable_layer(id);
	if (lm_unregister_layer(id)){
		printk(KERN_ERR "lm_regsiter_layer failed!\n");
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(rrm_layer_exit);

struct rrmanager* rrm_init(void (*hw_refresh)(void *p), void *para)
{
        int i;

        rrm.frame_state = FS_IDLE;
        rrm.layer_num = LID_OSD2; /* FIXME: hardcoded layer usage */

        if (rrm.que == NULL)
                rrm.que = kzalloc(sizeof(void*) * rrm.layer_num, GFP_KERNEL);
        if (!rrm.que) {
                printk(KERN_ERR "RRM[%d]: no memory!\n", __LINE__);
                return NULL;
        }
        for(i=0;i<rrm.layer_num;i++) {
                if (rrm.que[i] == NULL)
                        rrm.que[i] = rrq_init(DEFAULT_BUF_NUM);
                if (!rrm.que[i])
                        goto _rrm_init_err;
        }
        rrm.exec = rre_init(rrm.layer_num, hw_refresh, para);
        spin_lock_init(&rrm.lock);

        return &rrm;

_rrm_init_err:
        for(i=0;i<rrm.layer_num;i++) {
                if (rrm.que[i] != NULL)
                        rrq_release(rrm.que[i]);
        }
        kfree(rrm.que);
        return NULL;
}

int rrm_exit(void)
{
	return 0;
}

