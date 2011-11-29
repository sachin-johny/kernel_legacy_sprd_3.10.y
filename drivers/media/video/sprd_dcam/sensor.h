/*
* drivers/media/video/sprd_dcam/dcam_common.h
 * Dcam driver based on sc8800g2
 *
 * Copyright (C) 2011 Spreadtrum 
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

#ifndef _SENSOR_H_
#define _SENSOR_H_

#ifdef CONFIG_ARCH_SC8810
#include "sc8810/sensor_cfg.h"
#include "sc8810/sensor_drv.h"
#include <linux/delay.h>
#else
#include "sensor_cfg.h"
#include "sensor_drv.h"
#include <linux/delay.h>
#endif

#endif  // _SENSOR_H_