/* drivers/video/sc8800g/sc8800g_rrm.h
 *
 * SC8800g LCDC Refresh Request Manager
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

#ifndef __SC8800G_RRM_H
#define __SC8800G_RRM_H

#include <linux/semaphore.h>
#include <linux/spinlock.h>

/* 
 * refresh request data structures 
 */

/* refresh request */
struct rr {
        void (*callback)(void* data);
        void *data;
};

/* refresh request queue */
struct rrqueue {
        int depth;
        int available;
        struct rr * q_ptr;
        int index_in;
        int index_out;
        struct semaphore waitlock;
        int (*enqueue)(struct rrqueue *self, struct rr *r);
        int (*dequeue)(struct rrqueue *self, struct rr *r);
        int (*wait)(struct rrqueue *self);
        int (*wakeup)(struct rrqueue *self);
};

/* refresh request executer */
struct rrexecuter {
        struct rr *executing;
        void (**set_layer)(void *data);
        void (*refresh)(struct rrexecuter *self, int id, struct rr *r);
        void (*hw_refresh)(void *para);
        void *para;
};

/* refresh request manager */
struct rrmanager {
        int frame_state;
        int layer_num;
        struct rrqueue **que;
        struct rrexecuter *exec;
        spinlock_t lock;
};

/* 
 * refresh request APIs 
 */

/**
 * rrm_refresh - send a refresh request
 * @ id:       id of the requester/layer
 * @ callback: callback func which will be called when refresh done
 * @ data:     pointer to the data which will be passed to layer specific
 *             set_layer() before the hw_refresh() is called, and will be 
 *             sent back as parameter of the callback()
 *
 * return 0 on success, non-0 on failure
 * 
 * NOTE: can only be used in thread context and irq enabled, and this func
 *       could be blocked when you request to refresh the last free buf
 */
int rrm_refresh(int id, void (*callback)(void* data), void *data);

/**
 * rrm_layer_init - init rrm layer according to settings
 * @ id:        id of the requester/layer
 * @ buf_num:   number of buffers
 * @ set_layer: the func to setup layer for each refresh request
 *
 * return 0 on success, non-0 on failure
 */
int rrm_layer_init(int id, int buf_num, void (*set_layer)(void *data));

/**
 * rrm_layer_exit - release resources allocated for the layer
 * @ id:        id of the requester/layer
 *
 * return 0 on success, non-0 on failure
 */
int rrm_layer_exit(int id);

/**
 * rrm_interrupt - handle refresh request when frame done irq is detected
 * @ rrm:       the pointer to the rrmanager initialized via rrm_init.
 * 
 * NOTE: should be called in HARD_IRQ context
 */
void rrm_interrupt(struct rrmanager *rrm);

/**
 * rrm_init - init refresh request manager
 * @ hw_refresh: func to do the real h/w refresh operation
 */
struct rrmanager* rrm_init(void (*hw_refresh)(void*para), void* para);

/**
 * rrm_exit - release resources allocated for the rrm
 */
int rrm_exit(void);

#endif
