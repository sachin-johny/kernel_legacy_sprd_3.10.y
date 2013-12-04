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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/sipc.h>
#include "sblock.h"

static struct sblock_mgr *sblocks[SIPC_ID_NR][SMSG_CH_NR];

/* recover sblock ringbuf and txuinits list */
static void sblock_recover(void* data)
{
	struct sblock_mgr *sblock = data;
	volatile struct sblock_ring_header *ringhd;
	unsigned long pflags, qflags;
	int i, blknum;

	ringhd = sblock->ring->header;
	sblock->state = SBLOCK_STATE_IDLE;
	wake_up_interruptible_all(&(sblock->ring->getwait));
	wake_up_interruptible_all(&(sblock->ring->recvwait));

	spin_lock_irqsave(&(sblock->ring->rxlock), pflags);
	/* clean rxblk ring buffer */
	ringhd->rxblk_rdptr = ringhd->rxblk_wrptr;
	spin_lock_irqsave(&(sblock->ring->elock), qflags);
	for (i = 0; i < sblock->rxblknum; i++) {
		if (sblock->ring->rxunits[i].state == SBLOCK_RXUNIT_STATE_RECV ||
				sblock->ring->rxunits[i].state == SBLOCK_RXUNIT_STATE_CONFLICT) {
			sblock->ring->rxunits[i].state = SBLOCK_RXUNIT_STATE_PENDING;
		}
	}
	spin_unlock_irqrestore(&(sblock->ring->elock), qflags);
	spin_unlock_irqrestore(&(sblock->ring->rxlock), pflags);

	spin_lock_irqsave(&(sblock->ring->txlock), pflags);
	/* clean txblk ring buffer */
	ringhd->txblk_wrptr = ringhd->txblk_rdptr;
	/* recover the txunits list */
	spin_lock_irqsave(&(sblock->ring->plock), qflags);
	blknum = sblock->txblknum - sblock->ring->txblk_count;
	for (i = 0; blknum != 0 && i < sblock->txblknum; i++) {
		/* add the delected txuint to list */
		if (sblock->ring->txunits[i].state == SBLOCK_TXUNIT_STATE_SENT) {
			sblock->ring->txunits[i].state = SBLOCK_TXUNIT_STATE_READY;
			list_add_tail(&sblock->ring->txunits[i].list, &sblock->ring->txpool);
			sblock->ring->txblk_count++;
			blknum--;
		} else if (sblock->ring->txunits[i].state == SBLOCK_TXUNIT_STATE_PENDING) {
			blknum--;
		}
	}
	spin_unlock_irqrestore(&sblock->ring->plock, qflags);
	spin_unlock_irqrestore(&(sblock->ring->txlock), pflags);
}

void sblock_put(uint8_t dst, uint8_t channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	void* virt_addr;
	uint32_t index =0;
	unsigned long flags;

	if (!sblock) {
		return;
	}
	spin_lock_irqsave(&sblock->ring->plock, flags);
	virt_addr = (void*)(blk->addr);
	index = (virt_addr - sblock->smem_virt) / sblock->ring->header->txblk_size;
	if (index < sblock->txblknum &&
			sblock->ring->txunits[index].state != SBLOCK_TXUNIT_STATE_READY) {
		list_add_tail(&sblock->ring->txunits[index].list, &sblock->ring->txpool);
		sblock->ring->txblk_count++;
		sblock->ring->txunits[index].state = SBLOCK_TXUNIT_STATE_READY;
	}
	spin_unlock_irqrestore(&sblock->ring->plock, flags);
}

static int sblock_thread(void *data)
{
	struct sblock_mgr *sblock = data;
	struct smsg mcmd, mrecv;
	struct sblock blk;
	int rval;
	int recovered = 0;
	struct sched_param param = {.sched_priority = 90};

	/*set the thread as a real time thread, and its priority is 90*/
	sched_setscheduler(current, SCHED_RR, &param);

	/* since the channel open may hang, we call it in the sblock thread */
	rval = smsg_ch_open(sblock->dst, sblock->channel, -1);
	if (rval != 0) {
		printk(KERN_ERR "Failed to open channel %d\n", sblock->channel);
		/* assign NULL to thread poniter as failed to open channel */
		sblock->thread = NULL;
		return rval;
	}

	/* handle the sblock events */
	while (!kthread_should_stop()) {

		/* monitor sblock recv smsg */
		smsg_set(&mrecv, sblock->channel, 0, 0, 0);
		rval = smsg_recv(sblock->dst, &mrecv, -1);
		if (rval == -EIO) {
		/* channel state is FREE */
			msleep(5);
			continue;
		}

		pr_debug("sblock thread recv msg: dst=%d, channel=%d, "
				"type=%d, flag=0x%04x, value=0x%08x\n",
				sblock->dst, sblock->channel,
				mrecv.type, mrecv.flag, mrecv.value);

		switch (mrecv.type) {
		case SMSG_TYPE_OPEN:
			/* handle channel recovery */
			if (recovered) {
				if (sblock->handler) {
					sblock->handler(SBLOCK_NOTIFY_CLOSE, sblock->data);
				}
				sblock_recover(sblock);
			}
			smsg_open_ack(sblock->dst, sblock->channel);
			break;
		case SMSG_TYPE_CLOSE:
			/* handle channel recovery */
			smsg_close_ack(sblock->dst, sblock->channel);
			if (sblock->handler) {
				sblock->handler(SBLOCK_NOTIFY_CLOSE, sblock->data);
			}
			sblock->state = SBLOCK_STATE_IDLE;
			break;
		case SMSG_TYPE_CMD:
			/* respond cmd done for sblock init */
			WARN_ON(mrecv.flag != SMSG_CMD_SBLOCK_INIT);
			smsg_set(&mcmd, sblock->channel, SMSG_TYPE_DONE,
					SMSG_DONE_SBLOCK_INIT, sblock->smem_addr);
			smsg_send(sblock->dst, &mcmd, -1);
			if (sblock->handler) {
				sblock->handler(SBLOCK_NOTIFY_OPEN, sblock->data);
			}
			sblock->state = SBLOCK_STATE_READY;
			recovered = 1;
			break;
		case SMSG_TYPE_EVENT:
			/* handle sblock send/release events */
			switch (mrecv.flag) {
			case SMSG_EVENT_SBLOCK_SEND:
				wake_up_interruptible_all(&sblock->ring->recvwait);
				if (sblock->handler) {
					sblock->handler(SBLOCK_NOTIFY_RECV, sblock->data);
				}
				break;
			case SMSG_EVENT_SBLOCK_RELEASE:
				blk.addr = (void *)(mrecv.value - sblock->smem_addr + (uint32_t)sblock->smem_virt);
				blk.length = sblock->txblksz;
				sblock_put(sblock->dst, sblock->channel, &blk);
				wake_up_interruptible_all(&(sblock->ring->getwait));
				if (sblock->handler) {
					sblock->handler(SBLOCK_NOTIFY_GET, sblock->data);
				}
				break;
			default:
				rval = 1;
				break;
			}
			break;
		default:
			rval = 1;
			break;
		};
		if (rval) {
			printk(KERN_WARNING "non-handled sblock msg: %d-%d, %d, %d, %d\n",
					sblock->dst, sblock->channel,
					mrecv.type, mrecv.flag, mrecv.value);
			rval = 0;
		}
	}

	printk(KERN_WARNING "sblock %d-%d thread stop", sblock->dst, sblock->channel);
	return rval;
}

int sblock_create(uint8_t dst, uint8_t channel,
		uint32_t txblocknum, uint32_t txblocksize,
		uint32_t rxblocknum, uint32_t rxblocksize)
{
	struct sblock_mgr *sblock;
	volatile struct sblock_ring_header *ringhd;
	uint32_t hsize;
	int i, result;

	sblock = kzalloc(sizeof(struct sblock_mgr) , GFP_KERNEL);
	if (!sblock) {
		return -ENOMEM;
	}

	sblock->state = SBLOCK_STATE_IDLE;
	sblock->dst = dst;
	sblock->channel = channel;
	sblock->txblksz = txblocksize;
	sblock->rxblksz = rxblocksize;
	sblock->txblknum = txblocknum;
	sblock->rxblknum = rxblocknum;


	/* allocate smem */
	hsize = sizeof(struct sblock_ring_header);
	sblock->smem_size = hsize +
		txblocknum * txblocksize + rxblocknum * rxblocksize +
		(txblocknum + rxblocknum) * sizeof(struct sblock_blks);
	sblock->smem_addr = smem_alloc(sblock->smem_size);
	if (!sblock->smem_addr) {
		printk(KERN_ERR "Failed to allocate smem for sblock\n");
		kfree(sblock);
		return -ENOMEM;
	}
	sblock->smem_virt = ioremap(sblock->smem_addr, sblock->smem_size);
	if (!sblock->smem_virt) {
		printk(KERN_ERR "Failed to map smem for sblock\n");
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return -EFAULT;
	}

	/* initialize ring and header */
	sblock->ring = kzalloc(sizeof(struct sblock_ring), GFP_KERNEL);
	if (!sblock->ring) {
		printk(KERN_ERR "Failed to allocate ring for sblock\n");
		iounmap(sblock->smem_virt);
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return -ENOMEM;
	}
	ringhd = (volatile struct sblock_ring_header *)(sblock->smem_virt);
	ringhd->txblk_addr = sblock->smem_addr + hsize;
	ringhd->txblk_count = txblocknum;
	ringhd->txblk_size = txblocksize;
	ringhd->txblk_rdptr = 0;
	ringhd->txblk_wrptr = 0;
	ringhd->txblk_blks = sblock->smem_addr + hsize +
		txblocknum * txblocksize + rxblocknum * rxblocksize;
	ringhd->rxblk_addr = ringhd->txblk_addr + txblocknum * txblocksize;
	ringhd->rxblk_count = rxblocknum;
	ringhd->rxblk_size = rxblocksize;
	ringhd->rxblk_rdptr = 0;
	ringhd->rxblk_wrptr = 0;
	ringhd->rxblk_blks = ringhd->txblk_blks + txblocknum * sizeof(struct sblock_blks);

	sblock->ring->header = sblock->smem_virt;
	sblock->ring->txblk_virt = sblock->smem_virt +
		(ringhd->txblk_addr - sblock->smem_addr);
	sblock->ring->txblks = sblock->smem_virt +
		(ringhd->txblk_blks - sblock->smem_addr);
	sblock->ring->rxblk_virt = sblock->smem_virt +
		(ringhd->rxblk_addr - sblock->smem_addr);
	sblock->ring->rxblks = sblock->smem_virt +
		(ringhd->rxblk_blks - sblock->smem_addr);

	sblock->ring->txunits = kzalloc(sizeof(struct sblock_txunit) * txblocknum, GFP_KERNEL);
	if (!sblock->ring->txunits) {
		printk(KERN_ERR "Failed to allocate txunits for sblock\n");
		kfree(sblock->ring);
		iounmap(sblock->smem_virt);
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&sblock->ring->txpool);
	for (i = 0; i < txblocknum; i++) {
		sblock->ring->txunits[i].state = SBLOCK_TXUNIT_STATE_READY;
		sblock->ring->txunits[i].addr = sblock->ring->txblk_virt + i * txblocksize;
		list_add_tail(&sblock->ring->txunits[i].list, &sblock->ring->txpool);
		sblock->ring->txblk_count++;
	}

	sblock->ring->rxunits = kzalloc(sizeof(struct sblock_rxunit) * rxblocknum, GFP_KERNEL);
	if (!sblock->ring->rxunits) {
		printk(KERN_ERR "Failed to allocate rxunits for sblock\n");
		kfree(sblock->ring->txunits);
		kfree(sblock->ring);
		iounmap(sblock->smem_virt);
		smem_free(sblock->smem_addr, sblock->smem_size);
		kfree(sblock);
		return -ENOMEM;
	}
	for (i = 0; i < rxblocknum; i++) {
		sblock->ring->rxunits[i].state = SBLOCK_RXUNIT_STATE_FREE;
		sblock->ring->rxunits[i].capture = 0;
	}

	init_waitqueue_head(&sblock->ring->getwait);
	init_waitqueue_head(&sblock->ring->recvwait);
	spin_lock_init(&sblock->ring->txlock);
	spin_lock_init(&sblock->ring->rxlock);
	spin_lock_init(&sblock->ring->plock);
	spin_lock_init(&sblock->ring->elock);

	sblock->thread = kthread_create(sblock_thread, sblock,
			"sblock-%d-%d", dst, channel);
	if (IS_ERR(sblock->thread)) {
		printk(KERN_ERR "Failed to create kthread: sblock-%d-%d\n", dst, channel);
		kfree(sblock->ring->rxunits);
		kfree(sblock->ring->txunits);
		kfree(sblock->ring);
		iounmap(sblock->smem_virt);
		smem_free(sblock->smem_addr, sblock->smem_size);
		result = PTR_ERR(sblock->thread);
		kfree(sblock);
		return result;
	}

	sblocks[dst][channel]=sblock;
	wake_up_process(sblock->thread);

	return 0;
}

void sblock_destroy(uint8_t dst, uint8_t channel)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];

	if (sblock == NULL) {
		return;
	}

	sblock->state = SBLOCK_STATE_IDLE;
	smsg_ch_close(dst, channel, -1);

	/* stop sblock thread if it's created successfully and still alive */
	if (!IS_ERR_OR_NULL(sblock->thread)) {
		kthread_stop(sblock->thread);
	}

	if (sblock->ring) {
		wake_up_interruptible_all(&sblock->ring->recvwait);
		wake_up_interruptible_all(&sblock->ring->getwait);
		if (sblock->ring->txunits) {
			kfree(sblock->ring->txunits);
		}
		kfree(sblock->ring);
	}
	if (sblock->smem_virt) {
		iounmap(sblock->smem_virt);
	}
	smem_free(sblock->smem_addr, sblock->smem_size);
	kfree(sblock);

	sblocks[dst][channel]=NULL;
}

int sblock_register_notifier(uint8_t dst, uint8_t channel,
		void (*handler)(int event, void *data), void *data)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];

	if (!sblock) {
		printk(KERN_ERR "sblock-%d-%d not ready!\n", dst, channel);
		return -ENODEV;
	}
#ifndef CONFIG_SIPC_WCN
	if (sblock->handler) {
		printk(KERN_ERR "sblock handler already registered\n");
		return -EBUSY;
	}
#endif
	sblock->handler = handler;
	sblock->data = data;

	return 0;
}

int sblock_get(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct list_head *head;
	struct sblock_txunit *txunit;
	int rval = 0;
	unsigned long flags;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		printk(KERN_ERR "sblock-%d-%d not ready!\n", dst, channel);
		return sblock ? -EIO : -ENODEV;
	}

	ring = sblock->ring;
	ringhd = ring->header;
	head = &sblock->ring->txpool;

	if (list_empty(head)) {
		if (timeout == 0) {
			/* no wait */
			printk(KERN_WARNING "sblock_get %d-%d is empty!\n",
				dst, channel);
			rval = -ENODATA;
		} else if (timeout < 0) {
			/* wait forever */
			rval = wait_event_interruptible(ring->getwait, !list_empty(head) ||
					sblock->state == SBLOCK_STATE_IDLE);
			if (rval < 0) {
				printk(KERN_WARNING "sblock_get wait interrupted!\n");
			}

			if (sblock->state == SBLOCK_STATE_IDLE) {
				printk(KERN_ERR "sblock_get sblock state is idle!\n");
				rval = -EIO;
			}
		} else {
			/* wait timeout */
			rval = wait_event_interruptible_timeout(ring->getwait,
					!list_empty(head) || sblock == SBLOCK_STATE_IDLE,
					timeout);
			if (rval < 0) {
				printk(KERN_WARNING "sblock_get wait interrupted!\n");
			} else if (rval == 0) {
				printk(KERN_WARNING "sblock_get wait timeout!\n");
				rval = -ETIME;
			}

			if(sblock->state == SBLOCK_STATE_IDLE) {
				printk(KERN_ERR "sblock_get sblock state is idle!\n");
				rval = -EIO;
			}
		}
	}

	if (rval) {
		return rval;
	}

	/* multi-gotter may cause got failure */
	spin_lock_irqsave(&ring->plock, flags);
	if (!list_empty(head) || sblock->state == SBLOCK_STATE_READY) {
		txunit = list_entry(head->next, struct sblock_txunit, list);
		blk->addr = txunit->addr;
		blk->length = sblock->txblksz;
		list_del(head->next);
		ring->txblk_count--;
		txunit->state = SBLOCK_TXUNIT_STATE_PENDING;
	} else {
		rval = sblock->state != SBLOCK_STATE_READY ? -EIO : -EAGAIN;
	}
	spin_unlock_irqrestore(&ring->plock, flags);

	return rval;
}

int sblock_send(uint8_t dst, uint8_t channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct smsg mevt;
	int txpos;
	int rval = 0;
	uint32_t index;
	unsigned long pflags, qflags;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		printk(KERN_ERR "sblock-%d-%d not ready!\n", dst, channel);
		if (sblock) {
			index = (blk->addr - sblock->smem_virt) / sblock->ring->header->txblk_size;
			if (index < sblock->txblknum) {
				spin_lock_irqsave(&sblock->ring->plock, qflags);
				ring->txunits[index].state = SBLOCK_TXUNIT_STATE_SENT;
				spin_unlock_irqrestore(&sblock->ring->plock, qflags);
			}
		}
		return sblock ? -EIO : -ENODEV;
	}

	pr_debug("sblock_send: dst=%d, channel=%d, addr=%p, len=%d\n",
			dst, channel, blk->addr, blk->length);

	ring = sblock->ring;
	ringhd = ring->header;

	spin_lock_irqsave(&ring->txlock, pflags);
	if (sblock->state == SBLOCK_STATE_READY) {
		txpos = ringhd->txblk_wrptr % ringhd->txblk_count;
		ring->txblks[txpos].addr = blk->addr - sblock->smem_virt + sblock->smem_addr;
		ring->txblks[txpos].length = blk->length;
		pr_debug("sblock_send: channel=%d, wrptr=%d, txpos=%d, addr=%x\n",
				channel, ringhd->txblk_wrptr, txpos, ring->txblks[txpos].addr);
		ringhd->txblk_wrptr = ringhd->txblk_wrptr + 1;
		smsg_set(&mevt, channel, SMSG_TYPE_EVENT, SMSG_EVENT_SBLOCK_SEND, 0);
		rval = smsg_send(dst, &mevt, 0);
	} else {
		/* sblock state had been updated cause' sblock recover or destroy hanppened */
		rval = -EIO;
	}
	index = (blk->addr - sblock->smem_virt) / sblock->ring->header->txblk_size;
	if (index < sblock->txblknum) {
		spin_lock_irqsave(&sblock->ring->plock, qflags);
		ring->txunits[index].state = SBLOCK_TXUNIT_STATE_SENT;
		spin_unlock_irqrestore(&sblock->ring->plock, qflags);
	}
	spin_unlock_irqrestore(&ring->txlock, pflags);

	return rval ;
}

int sblock_receive(uint8_t dst, uint8_t channel, struct sblock *blk, int timeout)
{
	struct sblock_mgr *sblock = sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	int rxpos, rval = 0;
	uint32_t index;
	unsigned long pflags, qflags;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		printk(KERN_ERR "sblock-%d-%d not ready!\n", dst, channel);
		return sblock ? -EIO : -ENODEV;
	}

	ring = sblock->ring;
	ringhd = ring->header;

	pr_debug("sblock_receive: dst=%d, channel=%d, timeout=%d\n",
			dst, channel, timeout);
	pr_debug("sblock_receive: channel=%d, wrptr=%d, rdptr=%d",
			channel, ringhd->rxblk_wrptr, ringhd->rxblk_rdptr);

	if (ringhd->rxblk_wrptr == ringhd->rxblk_rdptr) {
		if (timeout == 0) {
			/* no wait */
			printk(KERN_WARNING "sblock_receive %d-%d is empty!\n",
				dst, channel);
			rval = -ENODATA;
		} else if (timeout < 0) {
			/* wait forever */
			rval = wait_event_interruptible(ring->recvwait,
				ringhd->rxblk_wrptr != ringhd->rxblk_rdptr ||
				sblock->state == SBLOCK_STATE_IDLE);
			if (rval < 0) {
				printk(KERN_WARNING "sblock_receive wait interrupted!\n");
			}

			if (sblock->state == SBLOCK_STATE_IDLE) {
				printk(KERN_ERR "sblock_receive sblock state is idle!\n");
				rval = -EIO;
			}

		} else {
			/* wait timeout */
			rval = wait_event_interruptible_timeout(ring->recvwait,
				ringhd->rxblk_wrptr != ringhd->rxblk_rdptr ||
				sblock->state == SBLOCK_STATE_IDLE, timeout);
			if (rval < 0) {
				printk(KERN_WARNING "sblock_receive wait interrupted!\n");
			} else if (rval == 0) {
				printk(KERN_WARNING "sblock_receive wait timeout!\n");
				rval = -ETIME;
			}

			if (sblock->state == SBLOCK_STATE_IDLE) {
				printk(KERN_ERR "sblock_receive sblock state is idle!\n");
				rval = -EIO;
			}
		}
	}

	if (rval < 0) {
		return rval;
	}

	/* multi-receiver may cause recv failure */
	spin_lock_irqsave(&ring->rxlock, pflags);

	if (ringhd->rxblk_wrptr != ringhd->rxblk_rdptr && sblock->state == SBLOCK_STATE_READY) {
		rxpos = ringhd->rxblk_rdptr % ringhd->rxblk_count;
		blk->addr = ring->rxblks[rxpos].addr - sblock->smem_addr + sblock->smem_virt;
		blk->length = ring->rxblks[rxpos].length;
		ringhd->rxblk_rdptr = ringhd->rxblk_rdptr + 1;
		pr_debug("sblock_receive: channel=%d, rxpos=%d, addr=%p, len=%d\n",
			channel, rxpos, blk->addr, blk->length);

		index = ((blk->addr - sblock->smem_virt) / ringhd->rxblk_size) - sblock->txblknum;
		if (index < sblock->rxblknum) {
			spin_lock_irqsave(&ring->elock, qflags);
			ring->rxunits[index].capture++;
			switch (ring->rxunits[index].state) {
				case SBLOCK_RXUNIT_STATE_FREE:
					ring->rxunits[index].state = SBLOCK_RXUNIT_STATE_RECV;
					break;
				case SBLOCK_RXUNIT_STATE_PENDING:
					ring->rxunits[index].state = SBLOCK_RXUNIT_STATE_CONFLICT;
					break;
				default:
					break;
			}
			spin_unlock_irqrestore(&ring->elock, qflags);
		}
	} else {
		rval = sblock->state != SBLOCK_STATE_READY ? -EIO : -EAGAIN;
	}
	spin_unlock_irqrestore(&ring->rxlock, pflags);

	return rval;
}

int sblock_get_free_count(uint8_t dst, uint8_t channel)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	int blk_count = 0;
	unsigned long flags;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		printk(KERN_ERR "sblock-%d-%d not ready!\n", dst, channel);
		return sblock ? -EIO : -ENODEV;
	}

	ring = sblock->ring;
	spin_lock_irqsave(&ring->plock, flags);
	blk_count= ring->txblk_count;
	spin_unlock_irqrestore(&ring->plock, flags);

	return blk_count;
}

int sblock_release(uint8_t dst, uint8_t channel, struct sblock *blk)
{
	struct sblock_mgr *sblock = (struct sblock_mgr *)sblocks[dst][channel];
	struct sblock_ring *ring;
	volatile struct sblock_ring_header *ringhd;
	struct smsg mevt;
	uint32_t addr;
	unsigned long flags;
	uint32_t index;
	int send = 0;

	if (!sblock || sblock->state != SBLOCK_STATE_READY) {
		printk(KERN_ERR "sblock-%d-%d not ready!\n", dst, channel);
		return sblock ? -EIO : -ENODEV;
	}

	pr_debug("sblock_release: dst=%d, channel=%d, addr=%p, len=%d\n",
			dst, channel, blk->addr, blk->length);

	ring = sblock->ring;
	ringhd = ring->header;

	addr = blk->addr - sblock->smem_virt + sblock->smem_addr;
	pr_debug("sblock_release: addr=%x\n", addr);

	index = ((blk->addr - sblock->smem_virt) / ringhd->rxblk_size) - sblock->txblknum;
	spin_lock_irqsave(&(sblock->ring->elock), flags);
	if (index < sblock->rxblknum) {
		switch (sblock->ring->rxunits[index].state) {
			case SBLOCK_RXUNIT_STATE_RECV:
				sblock->ring->rxunits[index].state = SBLOCK_RXUNIT_STATE_FREE;
				sblock->ring->rxunits[index].capture--;
				send = 1;
				break;
			case SBLOCK_RXUNIT_STATE_PENDING:
				/* update state, but not send smsg */
				sblock->ring->rxunits[index].capture--;
				if (sblock->ring->rxunits[index].capture == 0) {
					sblock->ring->rxunits[index].state = SBLOCK_RXUNIT_STATE_FREE;
				}
				break;
			case SBLOCK_RXUNIT_STATE_CONFLICT:
				/* update state, but not send smsg */
				sblock->ring->rxunits[index].capture--;
				if (sblock->ring->rxunits[index].capture == 1) {
					sblock->ring->rxunits[index].state = SBLOCK_RXUNIT_STATE_RECV;
				}
				break;
			default:
				break;
		}
	}

	if (send) {
		smsg_set(&mevt, channel, SMSG_TYPE_EVENT, SMSG_EVENT_SBLOCK_RELEASE, addr);
		smsg_send(dst, &mevt, -1);
	}
	spin_unlock_irqrestore(&(sblock->ring->elock), flags);

	return 0;
}

#if defined(CONFIG_DEBUG_FS)
static int sblock_debug_show(struct seq_file *m, void *private)
{
	struct sblock_mgr *sblock = NULL;
	struct sblock_ring  *ring = NULL;
	struct sblock_ring_header	 *header= NULL;
	int i, j;

	for (i = 0; i < SIPC_ID_NR; i++) {
		for (j=0;  j< SMSG_CH_NR; j++) {
			sblock = sblocks[i][j];
			if (!sblock) {
				continue;
			}
			seq_printf(m, "sblock dst 0x%0x, channel: 0x%0x, state: %d, smem_virt: 0x%0x, smem_addr: 0x%0x, smem_size: 0x%0x, txblksz: %d, rxblksz: %d \n",
				   sblock->dst, sblock->channel, sblock->state, (uint32_t)sblock->smem_virt, sblock->smem_addr, sblock->smem_size, sblock->txblksz, sblock->rxblksz );
			ring = sblock->ring;
			header = sblock->ring->header;
			seq_printf(m, "sblock ring: txblk_virt :0x%0x, rxblk_virt :0x%0x, txblk_count :%d \n",  (uint32_t)ring->txblk_virt, (uint32_t)ring->rxblk_virt, ring->txblk_count );
			seq_printf(m, "sblock header: rxblk_addr :0x%0x, rxblk_rdptr :0x%0x, rxblk_wrptr :0x%0x, rxblk_size :%d, rxblk_count :%d, rxblk_blks: 0x%0x \n", 
							header->rxblk_addr, header->rxblk_rdptr, header->rxblk_wrptr, header->rxblk_size, header->rxblk_count, header->rxblk_blks );
			seq_printf(m, "sblock header: txblk_addr :0x%0x, txblk_rdptr :0x%0x, txblk_wrptr :0x%0x, txblk_size :%d, txblk_count :%d, txblk_blks: 0x%0x \n", 
							header->txblk_addr, header->txblk_rdptr, header->txblk_wrptr, header->txblk_size, header->txblk_count, header->txblk_blks );
		}
	}
	return 0;

}

static int sblock_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, sblock_debug_show, inode->i_private);
}

static const struct file_operations sblock_debug_fops = {
	.open = sblock_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int  sblock_init_debugfs(void *root )
{
	if (!root)
		return -ENXIO;
	debugfs_create_file("sblock", S_IRUGO, (struct dentry *)root, NULL, &sblock_debug_fops);
	return 0;
}

#endif /* CONFIG_DEBUG_FS */

EXPORT_SYMBOL(sblock_put);
EXPORT_SYMBOL(sblock_create);
EXPORT_SYMBOL(sblock_destroy);
EXPORT_SYMBOL(sblock_register_notifier);
EXPORT_SYMBOL(sblock_get);
EXPORT_SYMBOL(sblock_send);
EXPORT_SYMBOL(sblock_receive);
EXPORT_SYMBOL(sblock_get_free_count);
EXPORT_SYMBOL(sblock_release);

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC/SBLOCK driver");
MODULE_LICENSE("GPL");
