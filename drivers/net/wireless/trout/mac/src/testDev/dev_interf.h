/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: dev_interf.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/08/27, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _DEV_INTERF_H
#define _DEV_INTERF_H

#include <linux/ioctl.h> 

/* Use 'H' as magic number */
#define TROUT_IOC_MAGIC 'H'

/*
 * S means "Set" through a ptr,
 * G means "Get": reply by setting through a pointer
 */
#define TROUT_IOC_SRXTIMEOUT _IOW(TROUT_IOC_MAGIC, 1, int)
#define TROUT_IOC_GTXRXCOUNT _IOR(TROUT_IOC_MAGIC, 2, int[4])


#endif /* #ifndef _DEV_INTERF_H */

/*
 *$Log$
 */

