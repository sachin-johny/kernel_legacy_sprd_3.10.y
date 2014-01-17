/******************************************************************************
 * Copyright (c) 2012-2013, Trcaicio Co., Ltd.
 * All rights reserved.
 * 
 * Filename: debug.h
 * Abstract: 
 *     
 * Author:   Hugh
 * Version:  1.00
 * Revison Log:
 *     2012/03/09, Hugh: Create this file.
 * CVS Log:
 *     $Id$
 ******************************************************************************/

#ifndef _DEBUG_H
#define _DEBUG_H

#define DEBUG

#ifdef DEBUG
extern unsigned int dbgGate;
#ifndef __KERNEL__ 
#define DV_DBG(arg, fmt...) printf(arg, ##fmt)
#define PRINT(arg,fmt...) printf(arg,##fmt)
#define DBG_INFO(level,arg,fmt...) do { \
        if (level & dbgGate)        \
        {                    \
            printf(arg, ##fmt);  \
        }                     \
    } while (0)     
#else
#define DV_DBG(arg, fmt...) printk(arg, ##fmt)
#define DBG_INFO(level,arg,fmt...) do { \
        if (level & dbgGate)        \
        {                    \
            printk(arg, ##fmt);  \
        }                     \
    } while (0)   

#endif
#else
#define DV_DBG(arg, fmt...)
#define DBG_INFO(level,arg,fmt...)
#define PRINT(arg,fmt...)

#endif

#endif /* #ifndef _DEBUG_H */

/*
 *$Log$
 */

