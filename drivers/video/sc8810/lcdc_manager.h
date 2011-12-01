/* drivers/video/sc8800g/sc8800g_lcdc_manager.h
 *
 * SC8800g LCDC Manager
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

/*
 * LCDC manager is the module to manage:
 * 	- per-layer status: disabled/enabled
 * 	- per-layer mode:   display/capture
 */
#ifndef __SC8800G_LCDC_MANAGER_H
#define __SC8800G_LCDC_MANAGER_H

#include <linux/semaphore.h>
#include <linux/spinlock.h>

/* layer ID */
#define LID_Image 0
#define LID_OSD1  1
#define LID_OSD2  2
#define LID_OSD3  3
#define LID_OSD4  4
#define LID_OSD5  5
#define LID_MAX   6

/* working mode */
#define LMODE_DISPLAY 0
#define LMODE_CAPTURE 1
#define LMODE_INVALID 2


/* layer status */
#define L_REGISTERED    (1<<0)
#define L_ENABLED       (1<<1)
#define L_WAIT          (1<<2)

/* per layer infomation needed by lcdc manager */
struct layer_info {
	int status; /* a bitmap to hold status like whether the layer has  */
	            /* been registered, enabled or not, or if there's user */
	            /* waiting for mode switch */
	int mode;
        struct semaphore wait;
        int (*enable)(int id);
        int (*disable)(int id);
};

/* lcdc manager attributes */
struct lcdc_manager {
        int mode;
        int ref_cnt;
        int layer_num;
        spinlock_t lock;
	struct layer_info *linfo;
};


/* 
 * lcdc manager APIs 
 */

/**
 * lm_acquire - let the lcdc manager know a layer is acquiring the lcdc
 *
 * this will guarantee the lcdc is in the layer's mode, and all layers of the
 * same mode have been enabled if were originally enabled, all layers of
 * the other mode have been disabled.
 *
 * @ id:       id of the layer
 *
 * return 0 on success, non-0 on failure
 * 
 * NOTE: can only be used in thread context, and this func could be blocked
 *       when the lcdc is busy working in the other mode
 */
int lm_acquire(int id);

/**
 * lm_release - let the lcdc manager know the layer has finished using it.
 * @ id: id of the layer
 *
 * return 0 on success, non-0 on failure
 */
int lm_release(int id);

/**
 * lm_register_layer - register a layer to the lcdc manager
 * @ id: id of the layer
 *
 * return 0 on success, non-0 on failure
 */
int lm_register_layer(int id, int mode, 
		int (*enable)(int id), int (*disable)(int id));

/**
 * lm_unregister_layer - 
 * @ id: id of the layer
 */
int lm_unregister_layer(int id);

/**
 * lm_init - init the lcdc manager
 */
struct lcdc_manager * lm_init(int num_of_layers);

/**
 * lm_exit - release resources allocated for the lcdc manager
 */
int lm_exit(void);

/**
 * lm_enable_layer - enable a layer
 * @ id: id of the layer
 */
int lm_enable_layer(int id);

/**
 * lm_disable_layer - disable a layer
 * @ id: id of the layer
 */
int lm_disable_layer(int id);
#endif
