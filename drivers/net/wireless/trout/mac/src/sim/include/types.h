/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: types.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _TYPES_H
#define _TYPES_H

#ifndef __KERNEL__ 
typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long s64;
typedef unsigned long u64;
#endif

#ifndef DV_TRUE
#define DV_TRUE 1
#endif

#ifndef DV_FALSE
#define DV_FALSE 0
#endif



#endif /* #ifndef _TYPES_H */

/*
 *$Log$
 */

