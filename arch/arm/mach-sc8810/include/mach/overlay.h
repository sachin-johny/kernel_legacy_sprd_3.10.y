
#ifndef _SPRD_OVERLAY_H_
#define _SPRD_OVERLAY_H_

#include <linux/types.h>


enum {
	SPRD_DATA_TYPE_YUV422 = 0,
	SPRD_DATA_TYPE_YUV420,
	SPRD_DATA_TYPE_YUV400,
	SPRD_DATA_TYPE_RGB888,
	SPRD_DATA_TYPE_RGB666, 
	SPRD_DATA_TYPE_RGB565,
	SPRD_DATA_TYPE_RGB555, 
	SPRD_DATA_TYPE_LIMIT
};

typedef struct overlay_rect {
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
}overlay_rect;

int overlay_open(void);
int overlay_update(int type, overlay_rect *rect, unsigned char *buffer);
int overlay_close(void);

#endif

