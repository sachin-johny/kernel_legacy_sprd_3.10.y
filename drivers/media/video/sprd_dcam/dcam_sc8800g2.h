/*
* drivers/media/video/sprd_dcam/dcam_sc8800g2.h
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
#ifndef _DCAM_SC8800G2_H_
#define _DCAM_SC8800G2_H_

#include "dcam_drv_sc8800g2.h"

#define ISP_QXGA_H                      1536
#define ISP_UXGA_H                      1200
#define ISP_SXGA_H                      1024
#define ISP_SXGAM_H                     960
#define ISP_XGA_H                       768
#define ISP_SVGA_H                      600
#define ISP_QSVGA_H                     512
#define ISP_VGA_H                       480
#define ISP_CIF_H                       288
#define ISP_QCIF_H                      240
#define ISP_QQVGA_H                     120


#define ISP_DISPLAY_NONE                0xFF

typedef int (*get_data)( ISP_ADDRESS_T addr, uint32_t width, uint32_t height);

typedef enum
{
    ISP_ROTATION_0 = 0,
    ISP_ROTATION_90,
    ISP_ROTATION_180,
    ISP_ROTATION_270,
    ISP_ROTATION_MIRROR,
    ISP_ROTATION_MAX
}ISP_ROTATION_E;

typedef enum
{
  DCAM_DATA_YUV422 = 0,
  DCAM_DATA_RGB,
  DCAM_DATA_MAX
}DCAM_DATA_FORMAT_E;

typedef enum
{
  DCAM_MODE_TYPE_IDLE = 0,
  DCAM_MODE_TYPE_PREVIEW,
  DCAM_MODE_TYPE_CAPTURE = 3,
  DCAM_MODE_TYPE_REVIEW,
  DCAM_MODE_TYPE_MAX
}DCAM_MODE_TYPE_E;
typedef enum
{
  YUV_YUYV = 0,
  YUV_YVYU,
  YUV_UYVY,
  YUV_VYUY,
  YUV_MAX
}DCAM_YUV_PATTERN_E;
typedef enum
{
  RGB_565 = 0, 
  RGB_RESERVED,
  RGB_666, 
  RGB_888,
  RGB_MAX
}RGB_TYPE_E;
typedef enum
{
  DCAM_ROTATION_0 = 0,
  DCAM_ROTATION_90,
  DCAM_ROTATION_270,
  DCAM_ROTATION_MAX
}DCAM_ROTATION_E;

typedef struct dcam_size
{
  uint32_t w;
  uint32_t h;
}DCAM_SIZE_T0;

typedef struct dcam_polarity
{
  uint32_t hsync; //0: low, 1: high
  uint32_t vsync; //0: low, 1: high
  uint32_t pclk; //0: low, 1: high
}DCAM_POLARITY_T;

typedef struct dcam_rect
{
  uint32_t x;
  uint32_t y;
  uint32_t w;
  uint32_t h;
}DCAM_RECT_T0;

typedef struct dcam_init_param
{
  DCAM_MODE_TYPE_E mode;
  DCAM_DATA_FORMAT_E format;
  DCAM_YUV_PATTERN_E yuv_pattern;
  RGB_TYPE_E display_rgb_type;
  DCAM_SIZE_T0 input_size;
  DCAM_POLARITY_T polarity;
  DCAM_RECT_T0 input_rect;
  DCAM_RECT_T0 display_rect;
  DCAM_RECT_T0 encoder_rect;
  DCAM_ROTATION_E rotation;
  int skip_frame;
  uint32_t first_buf_addr;
}DCAM_INIT_PARAM_T;

typedef enum
{
    DCAM_CB_SENSOR_SOF = 0,
    DCAM_CB_SENSOR_EOF,
    DCAM_CB_CAP_SOF,
    DCAM_CB_CAP_EOF,
    DCAM_CB_PATH1_DONE,
    DCAM_CB_CAP_FIFO_OF,
    DCAM_CB_SENSOR_LINE_ERR,
    DCAM_CB_SENSOR_FRAME_ERR,
    DCAM_CB_JPEG_BUF_OF,
    DCAM_CB_PATH2_DONE,
    DCAM_CB_NUMBER,
} DCAM_CB_ID_E;
typedef void (*CALLBACK_FUNC_PTR) (void);

int dcam_open(void);
int dcam_close(void);
int dcam_parameter_init(DCAM_INIT_PARAM_T *init_param);
int dcam_start(void);
int dcam_stop(void);
PUBLIC uint32_t dcam_callback_fun_register(DCAM_CB_ID_E cb_id,
                                      CALLBACK_FUNC_PTR user_func);
PUBLIC uint32_t dcam_set_buffer_address(uint32_t address);
void tmp_autocopy(void);
#endif //_DCAM_SC8800G2_H_