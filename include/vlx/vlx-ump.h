/*
 ****************************************************************
 *
 *  Component:	VLX User Mode virtual driver Proxy driver
 *
 *  Copyright (C) 2011, Red Bend Software. All Rights Reserved.
 *
 *  Contributor(s):
 *    Adam Mirowski <adam.mirowski@redbend.com>
 *
 ****************************************************************
 */

#ifndef VLX_UMP_H
#define VLX_UMP_H

#include <asm/ioctl.h>

#define UMPIOC_GET_VERSION	_IOR('y', 1, int)
#define UMPIOC_GET_NKOSCTX	_IOR('y', 2, int)
#define UMPIOC_READ_PHYS	_IOR('y', 3, int)
#define UMPIOC_READ_VIRT	_IOR('y', 4, int)

typedef struct {
    unsigned long	addr;
    unsigned int	size;
    void*		buf;
} ump_ioctl_t;

#endif
