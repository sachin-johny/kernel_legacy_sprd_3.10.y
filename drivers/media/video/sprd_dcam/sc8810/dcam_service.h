/*
* drivers/media/video/sprd_dcam/dcam_service.h
 * 
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
#ifndef _DCAM_SERVICE_H_
#define _DCAM_SERVICE_H_

//for 8810 chip platform
#include "dcam_service_sc8810.h"

/**---------------------------------------------------------------------------*
**                                                               Debug Micro Define                                                                         **
**----------------------------------------------------------------------------*/
//#define DCAM_V4L2_DEBUG 1

#ifdef DCAM_V4L2_DEBUG
#define DCAM_V4L2_PRINT printk 
#else
#define DCAM_V4L2_PRINT(...)
#endif
#define DCAM_V4L2_ERR printk 

#endif //_DCAM_SERVICE_H_
