/* drivers/video/sc8800g/sc8800g_copybit_rotation.h
 *
 * copybit rotation driver based on sc8800g
 *
 * Copyright (C) 2010 Spreadtrum 
 * 
 * Author: Xiaozhe wang <xiaozhe.wang@spreadtrum.com>
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

#ifndef __SC8800G_COPYBIT_ROTATION_H
#define __SC8800G_COPYBIT_ROTATION_H

#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <mach/sc8800g_2d.h>


/**
 * do_copyblit_rotation 
 *
 */
int do_copybit_rotation(struct s2d_blit_req * req);

#endif

