/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2005                               */
/*                                                                           */
/*  This program  is  proprietary to  Ittiam  Systems  Private  Limited  and */
/*  is protected under Indian  Copyright Law as an unpublished work. Its use */
/*  and  disclosure  is  limited by  the terms  and  conditions of a license */
/*  agreement. It may not be copied or otherwise  reproduced or disclosed to */
/*  persons outside the licensee's organization except in accordance with the*/
/*  terms  and  conditions   of  such  an  agreement.  All  copies  and      */
/*  reproductions shall be the property of Ittiam Systems Private Limited and*/
/*  must bear this notice in its entirety.                                   */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  File Name         : wpse_common.h                                        */
/*                                                                           */
/*  Description       : This file contains all the common definitions for    */
/*                      MAC implementation.                                  */
/*                                                                           */
/*  List of Functions : mac_addr_cmp                                         */
/*                      mac_addr_cpy                                         */
/*                      ip_addr_cpy                                          */
/*                      memcpy32                                             */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 05 2005   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifndef COMMON_H
#define COMMON_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#ifdef OS_LINUX_CSL_TYPE
#include <linux/string.h>
#include <linux/kernel.h>
#else /* OS_LINUX_CSL_TYPE */
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#endif /* OS_LINUX_CSL_TYPE */

#include "wpse_itypes.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
#define MAC_ADDRESS_LEN         6

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/
#define MAX(a, b)                   ((a > b)?a:b)
#define MIN(a, b)                   ((a < b)?a:b)

// 20120830 caisf mod, merged ittiam mac v1.3 code
#ifdef DEBUG_MODE
#ifdef OS_LINUX_CSL_TYPE
    #define DBG_PRINT(args...) printk(args)
#endif /* OS_LINUX_CSL_TYPE */
#else  /* DEBUG_MODE */
    #define DBG_PRINT(args...)
#endif /* DEBUG_MODE */

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/
/* Status */
typedef enum {FAILURE = 0,
SUCCESS = 1
} STATUS_T;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

#endif /* COMMON_H */
