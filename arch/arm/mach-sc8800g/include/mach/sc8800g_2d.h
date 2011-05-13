/*
 * arch/arm/mach-sc8800g/include/mach/sc8800g_2d.h
 */
#ifndef _SC8800G_2D_H_
#define _SC8800G_2D_H_

#include <linux/types.h>

#define SC8800G_2D_IOCTL_MAGIC 'm'
#define SC8800G_2D_BLIT _IOW(SC8800G_2D_IOCTL_MAGIC, 1, unsigned int)

enum {
	S2D_YUV_422,
	S2D_YUV_420,
	S2D_YUV_420_3P,
	S2D_YUV_400,
	S2D_ARGB_8888,
	S2D_BGRA_8888,
	S2D_ARGB_666,
	S2D_RGB_565,
	S2D_ARGB_1555,
	S2D_GREY,
	S2D_DFMT_LIMIT
};

enum {
 PMEM_IMG,
 FB_IMG,
};

#define S2D_ROT_NOP 0
#define S2D_FLIP_LR 0x1
#define S2D_FLIP_UD 0x2
#define S2D_ROT_90 0x4
#define S2D_ROT_180 (S2D_FLIP_UD|S2D_FLIP_LR)
#define S2D_ROT_270 (S2D_ROT_90|S2D_FLIP_UD|S2D_FLIP_LR)
#define S2D_DITHER 0x8
#define S2D_BLUR 0x10
#define S2D_BLEND_FG_PREMULT 0x20000

#define S2D_TRANSP_NOP 0xffffffff
#define S2D_ALPHA_NOP 0xff

struct s2d_rect {
 uint32_t x;
 uint32_t y;
 uint32_t w;
 uint32_t h;
};

struct s2d_img {
 uint32_t width;
 uint32_t height;
 uint32_t format;
 uint32_t base;
};

struct s2d_blit_req {
 struct s2d_img src;
 struct s2d_img dst;
 struct s2d_rect src_rect;
 struct s2d_rect dst_rect;
 uint32_t alpha;
 uint32_t flags; 
 uint32_t do_flags; //flag for scale, rotation and blending to do. 
};

struct s2d_blit_req_list {
 uint32_t count;
 struct s2d_blit_req req[10];
};

#endif

