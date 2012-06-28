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
#ifndef _SPRD_FB_H_
#define _SPRD_FB_H_

enum {
	SPRD_DATA_FORMAT_YUV422 = 0,
	SPRD_DATA_FORMAT_YUV420,
	SPRD_DATA_FORMAT_YUV400,
	SPRD_DATA_FORMAT_RGB888,
	SPRD_DATA_FORMAT_RGB666,
	SPRD_DATA_FORMAT_RGB565,
	SPRD_DATA_FORMAT_RGB555,
	SPRD_DATA_FORMAT_LIMIT
};

typedef struct overlay_setting_rect {
	uint16_t x; //start point - x
	uint16_t y; //start point - y
	uint16_t w; //width
	uint16_t h; //height
}overlay_setting_rect;

typedef struct overlay_setting{
	int data_type;
	overlay_setting_rect rect;
	unsigned char *buffer;
}overlay_setting;


/*
int sprdfb_IOinit(void);
int sprdfb_IOdeinit(void);
*/


#define SPRD_FB_IOCTL_MAGIC 'm'
#define SPRD_FB_SET_OVERLAY _IOW(SPRD_FB_IOCTL_MAGIC, 1, unsigned int)
#endif
