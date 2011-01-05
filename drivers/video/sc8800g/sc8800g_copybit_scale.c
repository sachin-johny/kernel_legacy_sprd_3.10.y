/* drivers/video/sc8800g/sc8800g_copybit_scale.c
 *
 * copybit alpha blending/blit driver based on sc8800g scale
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
 
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/mm.h>

#include <mach/scale_sc8800g2.h>
#include <mach/hardware.h>
#include "sc8800g_copybit_scale.h"


#define COPYBIT_SCALE_DEBUG
#ifdef COPYBIT_SCALE_DEBUG
#define SCALE_PRINT printk
#else
#define SCALE_PRINT(...)
#endif

#define SCALE_OUTPUT_BUF 0x0FD00000

typedef struct scale_param
{
	SCALE_DATA_FORMAT_E in_fmt;
	SCALE_SIZE_T in_size;
	SCALE_RECT_T in_rect;
	SCALE_ADDRESS_T in_addr;
	SCALE_DATA_FORMAT_E out_fmt;
	SCALE_SIZE_T out_size;	
	SCALE_ADDRESS_T out_addr;	
}SCALE_PARAM_T;

static inline int check_param(struct s2d_blit_req * req)
{
	
	return 0;
}

static SCALE_DATA_FORMAT_E get_input_data_format(uint32_t format)
{
	SCALE_DATA_FORMAT_E in_fmt = SCALE_DATA_MAX;

	switch(format)
	{
		case S2D_YUV_422:
			in_fmt = SCALE_DATA_YUV422;
			break;
		case S2D_YUV_420:
			in_fmt = SCALE_DATA_YUV420;
			break;
		case S2D_YUV_400:
			in_fmt = SCALE_DATA_YUV400;
			break;		
		case S2D_RGB_565:
			in_fmt = SCALE_DATA_RGB565;
			break;	
		case S2D_ARGB_8888:
		case S2D_BGRA_8888:
		case S2D_ARGB_666:
		case S2D_ARGB_1555:
		case S2D_GREY:
		default:
			in_fmt = SCALE_DATA_MAX;
			SCALE_PRINT("input NOT support the color format.\n");
			break;
	}

	return in_fmt;
}
static SCALE_DATA_FORMAT_E get_output_data_format(uint32_t format)
{
	SCALE_DATA_FORMAT_E out_fmt = SCALE_DATA_MAX;

	switch(format)
	{		
		case S2D_RGB_565:
			out_fmt = SCALE_DATA_RGB565;
			break;	
		case S2D_YUV_422:
		case S2D_YUV_420:
		case S2D_YUV_400:
		case S2D_ARGB_8888:
		case S2D_BGRA_8888:
		case S2D_ARGB_666:
		case S2D_ARGB_1555:
		case S2D_GREY:
		default:
			out_fmt = SCALE_DATA_MAX;
			SCALE_PRINT("output NOT support the color format.\n");
			break;
	}

	return out_fmt;
}

static int get_param(SCALE_PARAM_T *scale_param, struct s2d_blit_req * req)
{
	uint32_t scale_size = 1;
	 struct s2d_rect rect;
	
	if((req->flags & S2D_ROT_90) || (req->flags & S2D_ROT_270))
	{
		memcpy(&rect, &req->dst_rect, sizeof(struct s2d_rect));
		req->dst_rect.w = rect.h;
		req->dst_rect.h = rect.w;
	}
	
	scale_size = (req->dst_rect.w  + req->src_rect.w - 1) / req->src_rect.w;

	scale_param->in_size.w = req->src.width;
	scale_param->in_size.h = req->src.height;
	
	scale_param->out_size.w = req->dst_rect.w;
	scale_param->in_rect.w = req->src_rect.w;
	scale_param->in_rect.x = req->src_rect.x;
	if((req->src_rect.x + req->src_rect.w) & 0x3)
	{
		scale_param->in_rect.w = scale_param->in_rect.w + (3 - ((req->src_rect.x + req->src_rect.w) & (0x3)));	
		if((scale_param->in_rect.x + scale_param->in_rect.w) > req->src.width)
		{
			SCALE_PRINT("the in_rect.w is over.\n");
			return -1;
		}		
		scale_param->out_size.w = scale_param->out_size.w +  (3 -( (req->src_rect.x + req->src_rect.w) & 0x3)) * scale_size;
	}
	if(req->src_rect.x & 0x3)
	{			
		scale_param->in_rect.x = req->src_rect.x & (~0x3);		
		scale_param->in_rect.w += req->src_rect.x & (0x3);		
		scale_param->out_size.w += (req->src_rect.x & (0x3)) * scale_size;
	}
	if(scale_param->out_size.w & (0x3))
	{		
		scale_param->out_size.w = (scale_param->out_size.w + 3) / 4 * 4;
	}
	if(req->dst_rect.h & (0x1))
	{		
		scale_param->out_size.h = req->dst_rect.h + 1;
	}
	else
	{
		scale_param->out_size.h = req->dst_rect.h;
	}	
	
	scale_param->in_fmt = get_input_data_format(req->src.format);
	if(SCALE_DATA_MAX == scale_param->in_fmt)
		return -1;

	scale_param->in_rect.y = req->src_rect.y;
	scale_param->in_rect.h = req->src_rect.h;	
	scale_param->in_addr.yaddr = req->src.base;
	scale_param->in_addr.uaddr = scale_param->in_addr.yaddr + scale_param->in_size.w * scale_param->in_size.h;
	scale_param->in_addr.vaddr = scale_param->in_addr.uaddr;	

	scale_param->out_fmt = get_output_data_format(req->dst.format);	
	if(SCALE_DATA_MAX == scale_param->out_fmt)
		return -1;

	scale_param->out_addr.yaddr = SCALE_OUTPUT_BUF;	
	scale_param->out_addr.uaddr = scale_param->out_addr.yaddr + scale_param->out_size.w * scale_param->out_size.h;
	scale_param->out_addr.vaddr = scale_param->out_addr.uaddr;	

	if((req->flags & S2D_ROT_90) || (req->flags & S2D_ROT_270))
	{
		memcpy(&req->dst_rect, &rect, sizeof(struct s2d_rect));
	}
	
	return 0;
}

static int update_param(SCALE_PARAM_T *scale_param, struct s2d_blit_req * req)
{
	uint32_t scale_size;
	 struct s2d_rect rect;

	if((req->flags & S2D_ROT_90) || (req->flags & S2D_ROT_270))
	{
		memcpy(&rect, &req->dst_rect, sizeof(struct s2d_rect));
		req->dst_rect.w = rect.h;
		req->dst_rect.h = rect.w;
	}
	
	scale_size =  (req->dst_rect.w  + req->src_rect.w - 1) / req->src_rect.w;

	req->src.base = scale_param->out_addr.yaddr; 
	req->src.format = req->dst.format;
	req->src.width = scale_param->out_size.w;
	req->src.height = scale_param->out_size.h;		
	req->src_rect.x =  (req->src_rect.x & (0x3)) * scale_size;
	req->src_rect.y = 0;
	req->src_rect.w = req->dst_rect.w;
	req->src_rect.h = req->dst_rect.h;
	
	if((req->flags & S2D_ROT_90) || (req->flags & S2D_ROT_270))
	{
		memcpy(&req->dst_rect, &rect, sizeof(struct s2d_rect));
	}
	
	return 0;
}

static int do_scale(SCALE_PARAM_T *scale_param)
{

	SCALE_CONFIG_T scale_config;	
	SCALE_MODE_E scale_mode;
	
	if(0 != _SCALE_DriverIOInit())
	{
		SCALE_PRINT("Fail to _SCALE_DriverInit.\n");
		return -1;
	}
			
	//set mode
	scale_config.id = SCALE_PATH_MODE;	
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;	 
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	scale_config.param = &(scale_param->in_fmt);
	SCALE_PRINT("foramt:in: %d, out: %d.\n", scale_param->in_fmt, scale_param->out_fmt);
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	scale_config.param = &scale_param->out_fmt;
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE; 
	scale_config.param = &scale_param->in_size;
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_config.param = &scale_param->out_size;
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}	
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_config.param = &scale_param->in_rect;
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	scale_config.param =& scale_param->in_addr;
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_config.param = &scale_param->out_addr;
	if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))	
	{
		SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}	
	//done	 
	if(0 != _SCALE_DriverIODone())	
	{
		SCALE_PRINT("Fail to SCALE_IOC_DONE");
		return -1;
	}
	else
		SCALE_PRINT("OK to SCALE_IOC_DONE.");

	if(0 != _SCALE_DriverIODeinit())
	{
		SCALE_PRINT("Fail to _SCALE_DriverDeinit.\n");
		return -1;
	}

	return 0;
}


int do_copybit_scale(struct s2d_blit_req * req)
{
	SCALE_PARAM_T scale_params;		

	if((req->src.format == req->dst.format) && (req->src_rect.w == req->dst_rect.w) &&(req->src_rect.h == req->dst_rect.h))
	{		
		return 0;
	}
	else
	{
		if((S2D_YUV_422 != req->src.format) && (S2D_YUV_420 != req->src.format) && (S2D_RGB_565 != req->src.format))
		{
			return 0;
		}
		else
			SCALE_PRINT("###########need to scale##########\n");
	}

	SCALE_PRINT("src.format 0x%x\n"
	       "src.width  %d\n"
	       "src.height %d\n"
	       "src.base   0x%x\n"
	       "src_rect   x %d, y %d, w %d, h %d\n"
	       "dst.format %d\n"
	       "dst.width  %d\n"
	       "dst.height %d\n"
	       "dst.base   0x%x\n"
	       "dst_rect   x %d, y %d, w %d, h %d\n"
	       "flags: %d\n",
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

	if(0 != get_param(&scale_params, req))
	{
		return -1;
	}
	
	SCALE_PRINT("scale_params: src.format %d\n"
	       "src.width  %d\n"
	       "src.height %d\n"
	       "src.base   0x%x\n"
	       "src_rect   x %d, y %d, w %d, h %d\n"
	       "dst.format %d\n"
	       "dst.width  %d\n"
	       "dst.height %d\n"
	       "dst.base   0x%x\n" ,
	       scale_params.in_fmt, scale_params.in_size.w, scale_params.in_size.h, scale_params.in_addr.yaddr,
	       scale_params.in_rect.x, scale_params.in_rect.y, 
	       scale_params.in_rect.w, scale_params.in_rect.h, 
	       scale_params.out_fmt, scale_params.out_size.w, scale_params.out_size.h, scale_params.out_addr.yaddr
	       );	
	
	if(0 != do_scale(&scale_params))
	{
		SCALE_PRINT("Fail to do_scale.\n");
		return -1;
	}

	if(0 != update_param(&scale_params, req)) //for debug
	{
		return -1;
	}

	SCALE_PRINT("src.format 0x%x\n"
	       "src.width  %d\n"
	       "src.height %d\n"
	       "src.base   0x%x\n"
	       "src_rect   x %d, y %d, w %d, h %d\n"
	       "dst.format %d\n"
	       "dst.width  %d\n"
	       "dst.height %d\n"
	       "dst.base   0x%x\n"
	       "dst_rect   x %d, y %d, w %d, h %d\n"
	       "flags: %d\n",
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
