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

#include <linux/wait.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/mm.h>

#include <mach/rotation_sc8800g2.h>
#include <mach/hardware.h>
#include "sc8800g_copybit_rotation.h"

//#define COPYBIT_ROTATION_DEBUG
#ifdef COPYBIT_ROTATION_DEBUG
#define ROT_PRINT printk
#else
#define ROT_PRINT(...)
#endif

#define ROT_OUTPUT_BUF 0x0FE00000

static ROTATION_DATA_FORMAT_E get_data_format(uint32_t format)
{
	ROTATION_DATA_FORMAT_E out_fmt = ROTATION_MAX;

	switch(format)
	{
		case S2D_YUV_422:
			out_fmt = ROTATION_YUV422;
			break;
		case S2D_YUV_420:
			out_fmt = ROTATION_YUV420;
			break;
		case S2D_YUV_400:
			out_fmt = ROTATION_YUV400;
			break;		
		case S2D_RGB_565:
			out_fmt = ROTATION_RGB565;
			break;	
		case S2D_BGRA_8888:
			out_fmt = ROTATION_RGB888;
			break;
		case S2D_ARGB_8888:		
		case S2D_ARGB_666:
		case S2D_ARGB_1555:
		case S2D_GREY:
		default:
			out_fmt = ROTATION_MAX;
			ROT_PRINT("NOT support the color format.\n");
			break;
	}

	return out_fmt;
}

static ROTATION_DIR_E get_rotation_dir(uint32_t flags)
{
	ROTATION_DIR_E rot_dir = ROTATION_DIR_MAX;
	uint32_t rot_tmp = flags & 0x7;

	if(S2D_ROT_90 == rot_tmp)
		rot_dir = ROTATION_90;
	else if(S2D_ROT_180 == rot_tmp)
		rot_dir = ROTATION_180;
	else if(S2D_ROT_270 == rot_tmp)
		rot_dir = ROTATION_270;
	else
		rot_dir = ROTATION_DIR_MAX;

	return rot_dir;
}
static inline int check_param(struct s2d_blit_req * req)
{
	//only need to check for the rgb565
	if(S2D_RGB_565 == req->src.format)
	{
		if(req->src.width & 0x1) 
		{
			ROT_PRINT("the source width is not aligned by 2. width: %d.\n", req->src.width);
			return -1;
		}
	}
	return 0;
}
static int get_param(ROTATION_PARAM_T *rot_param, struct s2d_blit_req * req)
{
	rot_param->data_format = get_data_format(req->src.format);
	rot_param->img_size.w = req->src.width;
	rot_param->img_size.h = req->src.height;
	rot_param->rotation_dir = get_rotation_dir(req->flags);
	rot_param->src_addr.y_addr = req->src.base;
	rot_param->dst_addr.y_addr =  ROT_OUTPUT_BUF;
	if(rot_param->data_format < ROTATION_YUV400) //for YUV422 and YUV420
	{
		rot_param->src_addr.uv_addr = rot_param->src_addr.y_addr + rot_param->img_size.w * rot_param->img_size.h;
		rot_param->src_addr.v_addr = rot_param->src_addr.uv_addr;
		rot_param->dst_addr.uv_addr = rot_param->dst_addr.y_addr + rot_param->img_size.w * rot_param->img_size.h;
		rot_param->dst_addr.v_addr = rot_param->dst_addr.uv_addr;	
	}
	else
	{
		rot_param->src_addr.uv_addr = 0;
		rot_param->src_addr.v_addr = 0;
		rot_param->dst_addr.uv_addr = 0;
		rot_param->dst_addr.v_addr = 0;	
	}

	//wxz20110113: handle the height which is not aligned by 2. 
	if(req->src.height & 0x1)
	{
		rot_param->img_size.h += 1;
		ROT_PRINT("Modify the source height. old h: %d, new h: %d.\n", req->src.height, rot_param->img_size.h);
	}
	
	
	return 0;
}

static int update_param(ROTATION_PARAM_T *rot_param, struct s2d_blit_req * req)
{
	uint32_t a, b;

	switch(rot_param->rotation_dir)
	{
		case ROTATION_90:
			req->src.width = rot_param->img_size.h;
			req->src.height = rot_param->img_size.w;
			a = rot_param->img_size.h - req->src_rect.y - req->src_rect.h;
			b = req->src_rect.x;
			req->src_rect.x = a;
			req->src_rect.y = b;
			//wxz20110113: handle for height.
			if(req->dst.width & 0x1)
			{
				req->src_rect.x += 1;
			}
			break;
		case ROTATION_180:
			//wxz20110113: handle for height.
			if(req->dst.height & 0x1)
			{
				req->src.height = rot_param->img_size.h;
				req->src_rect.y += 1;
			}
			break;
		case ROTATION_270:
			req->src.width = rot_param->img_size.h;
			req->src.height = rot_param->img_size.w;			
			a = req->src_rect.y;
			b = rot_param->img_size.w - req->src_rect.x - req->src_rect.w;
			req->src_rect.x = a;
			req->src_rect.y = b;
			//wxz20110113: handle for height.
			if(req->dst.width & 0x1)
			{
				req->src_rect.x -= 1;
			}			
			break;	
		case ROTATION_MIRROR:
			break;	
		default:
			return -1;
	}	
	req->src.base = rot_param->dst_addr.y_addr;
	req->src_rect.w = req->dst_rect.w;
	req->src_rect.h = req->dst_rect.h;	

	return 0;
}

int do_copybit_rotation(struct s2d_blit_req * req)
{
	ROTATION_PARAM_T rot_params;
	
	if((req->flags & S2D_ROT_90)  || (req->flags & S2D_ROT_180) || (req->flags & S2D_ROT_270))
	{
		ROT_PRINT("###need to do rotation###\n");		
	}
	else
	{		
		return 0;
	}
	
	ROT_PRINT("before rotation:src.format 0x%x\n"
	       "src.width  %d\n"
	       "src.height %d\n"
	       "src.base   0x%x\n"
	       "src_rect   x %d, y %d, w %d, h %d\n"
	       "dst.format %d\n"
	       "dst.width  %d\n"
	       "dst.height %d\n"
	       "dst.base   0x%x\n"
	       "dst_rect   x %d, y %d, w %d, h %d\n"
	       "flag: %d\n",
	       req->src.format, req->src.width, req->src.height, req->src.base,
	       req->src_rect.x, req->src_rect.y, 
	       req->src_rect.w, req->src_rect.h, 
	       req->dst.format, req->dst.width, req->dst.height, req->dst.base,
	       req->dst_rect.x, req->dst_rect.y, 
	       req->dst_rect.w, req->dst_rect.h,
	       req->flags
	       );
	
	if(0 != check_param(req)) /* to fulfill the alignment restriction */
	{
		return -1;
	}

	if(0 != get_param(&rot_params, req))
	{
		return -1;
	}
	
	if(0 != rotation_start(&rot_params))
	{
		ROT_PRINT("Fail to do_rotation.\n");
		return -1;
	}

	if(0 != update_param(&rot_params, req)) 
	{
		return -1;
	}

	ROT_PRINT("after rotation: src.format 0x%x\n"
	       "src.width  %d\n"
	       "src.height %d\n"
	       "src.base   0x%x\n"
	       "src_rect   x %d, y %d, w %d, h %d\n"
	       "dst.format %d\n"
	       "dst.width  %d\n"
	       "dst.height %d\n"
	       "dst.base   0x%x\n"
	       "dst_rect   x %d, y %d, w %d, h %d\n"
	       "flag: %d\n",
	       req->src.format, req->src.width, req->src.height, req->src.base,
	       req->src_rect.x, req->src_rect.y, 
	       req->src_rect.w, req->src_rect.h, 
	       req->dst.format, req->dst.width, req->dst.height, req->dst.base,
	       req->dst_rect.x, req->dst_rect.y, 
	       req->dst_rect.w, req->dst_rect.h,
	       req->flags
	       );
	
	return 0;	
}



