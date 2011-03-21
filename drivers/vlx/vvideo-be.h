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

#ifndef _VVIDEO_BE_H_
#define _VVIDEO_BE_H_

typedef struct vvideo_hw_ops_t {
    unsigned int  version;
    int (*open)   (int minor, NkPhAddr plink, VVideoDesc* desc, void** private_data);
    int (*release)(void* private_data);
    int (*ioctl)  (void* private_data, unsigned int cmd, void* arg);
    int (*mmap)   (void* private_data, unsigned long pgoff, unsigned long* bus_addr);
} vvideo_hw_ops_t;

#define VVIDEO_HW_OPS_VERSION 2

#endif /* _VVIDEO_BE_H_ */
