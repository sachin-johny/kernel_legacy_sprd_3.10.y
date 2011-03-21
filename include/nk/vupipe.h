/*
 ****************************************************************
 *
 *  Component:	VirtualLogix VUPIPE
 *
 *  Copyright (C) 2008-2009, VirtualLogix. All Rights Reserved.
 *
 *  Contributor(s):
 *
 ****************************************************************
 */
#ifndef _VUPIPE_H
#define _VUPIPE_H

#include <asm/io.h>

#define PIOC_READ_NOTIFY     	_IOW('y', 1, int)
#define PIOC_WRITE_NOTIFY  	_IOW('y', 2, int)
#define PIOC_AVAILABLE_DATA 	_IOR('y', 3, int)
#define PIOC_AVAILABLE_ROOM 	_IOR('y', 4, int)
#define PIOC_GET_BUFFER_SIZE	_IOR('y', 5, int)
#define PIOC_GET_OFFSET		_IOR('y', 6, int)

#endif
