/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VLCD Interface
 *
 *  Copyright (C) 2008-2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *    Adam Mirowski (adam.mirowski@vlx.com)
 *
 ****************************************************************
 */

#ifndef VLCD_COMMON_H
#define VLCD_COMMON_H

#include <nk/nkern.h>

    /* Trace macros are still used by VLX code in native drivers */
#define VLCD_TRACE 0
#ifdef VLCD_TRACE
#if (VLCD_TRACE >= 0)
#define VLCD_TRACE_0(x...) printk(x)
#else
#define VLCD_TRACE_0(x...)
#endif
#if (VLCD_TRACE >= 1)
#define VLCD_TRACE_1(x...) printk(x)
#else
#define VLCD_TRACE_1(x...)
#endif
#if (VLCD_TRACE >= 2)
#define VLCD_TRACE_2(x...) printk(x)
#else
#define VLCD_TRACE_2(x...)
#endif
#else /* ! VLCD_TRACE */
#define VLCD_TRACE_0(x...)
#define VLCD_TRACE_1(x...)
#define VLCD_TRACE_2(x...)
#endif /* VLCD_TRACE */

#define VLCD_MAX_CONF_NUMBER 10

    /*
     * Commands sent from frontend to backend.
     */

#define VLCD_EVT_INIT            0x00000001	/* Frontend is connected */
#define VLCD_EVT_SIZE_MODIFIED   0x00000002	/* Screen size or position */
#define VLCD_EVT_COLOR_MODIFIED  0x00000004
#define VLCD_EVT_FLAGS_MODIFIED  0x00000008
#define VLCD_EVT_RECT_MODIFIED   0x00000010
    /*
     * vlcd_color_conf_t allows to describe all True Color configurations.
     * Configurations based on a palette cannot be described using this
     * structure. Transparency is not supported.
     */
typedef struct {
    nku8_f  bpp;          /* Bits per pixels (0 means undefined) */
    nku8_f  red_offset;   /* Offset of red color if bpp > 8 */
    nku8_f  red_length;   /* Length in bits */
    nku8_f  green_offset; /* Offset of green color if bpp > 8 */
    nku8_f  green_length; /* Length in bits */
    nku8_f  blue_offset;  /* Offset of blue color if bpp > 8 */
    nku8_f  blue_length;  /* Length in bits */
    nku8_f  align;        /* Alignment of colors (0=>MSB is left bit) */
} vlcd_color_conf_t;

    /*
     * Describes a screen configuration supported by the native driver.
     */
typedef struct {
    nku16_f xres;     /* width of the screen (in pixel) */
    nku16_f yres;     /* height of the screen (in pixel) */
    vlcd_color_conf_t color_conf;
} vlcd_pconf_t;

    /*
     * Describes the current configuration of the screen.
     */
typedef struct {
    nku16_f xres;                     /* Width of the displayed part */
    nku16_f yres;                     /* Height of the displayed part */
    nku16_f xres_virtual;             /* Width of the virtual screen */
    nku16_f yres_virtual;             /* Height of the virtual screen */
    nku16_f xoffset;                  /* Offset from xres to xres_virtual */
    nku16_f yoffset;                  /* Offset from yres to yres_virtual */
    nku32_f dma_zone_size;            /* Size of dma zone in bytes */
    nku32_f dma_zone_paddr;           /* Physical address of dma_zone */
    vlcd_color_conf_t color_conf;     /* Color configuration */
} vlcd_conf_t;

typedef struct {
    nku16_f left;
    nku16_f top;
    nku16_f right;
    nku16_f bottom;
} vlcd_rect_t;

    /* Bits inside the "flags" field of NkDevVlcd */
#define VLCD_FLAGS_SWITCH_OFF		(1<<0)

    /* Bits inside the "caps" field of NkDevVlcd */
#define VLCD_CAPS_PARTIAL_UPDATE	(1<<0)

typedef struct {
	/*
	 * Filled during backend init. Possible configurations.
	 * First one is default configuration.
	 */
    vlcd_pconf_t pconf[VLCD_MAX_CONF_NUMBER];
	/* Filled during frontend init (using pconf) */
    vlcd_conf_t current_conf;
	/* Bitfield set by frontend to tell backend what has been modified */
    nku32_f modified;
	/* Cross IT from frontend to backend */
    NkXIrq  cxirq;
	/* Bitfield set by frontend to inform the backend about a new state */
    nku32_f flags;
       /* Bitfield to tell the frontend about the backend's capabilities */
    nku32_f caps;
        /* Display region to update */
    vlcd_rect_t rect;
} NkDevVlcd;

#endif /* VLCD_COMMON_H */
