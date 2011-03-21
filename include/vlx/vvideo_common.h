/*
 ****************************************************************
 *
 *  Component:	VirtualLogix virtual video
 *
 *  Copyright (C) 2010, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *   Christophe Lizzi (Christophe.Lizzi@virtuallogix.com)
 *
 ****************************************************************
 */

#ifndef _VVIDEO_COMMON_H_
#define _VVIDEO_COMMON_H_

#define VVIDEO_REQ_IOCTL_ARG_SIZE 128
#define VVIDEO_REQ_IOCTL_EXT_SIZE 2048

#define VVIDEO_REQ_NONE    0
#define VVIDEO_REQ_OPEN    1
#define VVIDEO_REQ_RELEASE 2
#define VVIDEO_REQ_IOCTL   3
#define VVIDEO_REQ_MMAP    4

#define VVIDEO_REQ_PENDING  0x12345678

typedef struct VVideoIoctl {
    unsigned int  cmd;
    unsigned int  arg_size;
    unsigned int  ext_size;
    unsigned long arg_value; // arg passed by value (used when arg_size == 0)
    // arg[] and ext[] below must be contiguous
    unsigned char arg[VVIDEO_REQ_IOCTL_ARG_SIZE];
    unsigned char ext[VVIDEO_REQ_IOCTL_EXT_SIZE];
} VVideoIoctl;


typedef struct VVideoMMap {
    unsigned long pgoff;
    unsigned long size;
    unsigned long paddr;
    unsigned int  cacheable;
} VVideoMMap;


typedef struct VVideoOpen {
    int minor;
} VVideoOpen;


typedef struct VVideoRelease {
    int minor;
} VVideoRelease;


typedef struct VVideoRequest {
    int req;
    int result;
    union {
	VVideoOpen    open;
	VVideoRelease release;
	VVideoIoctl   ioctl;
	VVideoMMap    mmap;
    } u;
} VVideoRequest;


#define VVIDEO_TYPE_NONE    0
#define VVIDEO_TYPE_OVERLAY 1
#define VVIDEO_TYPE_CAMERA  2

typedef struct VVideoDesc {
    int           type;
    unsigned long pmem_size;
} VVideoDesc;


typedef struct VVideoShared {
    VVideoDesc    desc;
    VVideoRequest req;
} VVideoShared;


#define VVIDEO_MAJOR		241
#define VVIDEO_MINOR_BASE       1

#endif /*  _VVIDEO_COMMON_H_ */
