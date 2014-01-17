/*****************************************************************************/
/*                                                                           */
/*                           Ittiam WPS SOFTWARE                             */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2011                               */
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
/*  File Name         : common.h                                             */
/*                                                                           */
/*  Description       : This file contains all the common definitions for    */
/*                      WPS Library.                                         */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_REG_SUPP

#ifndef COMMON_H
#define COMMON_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_itypes.h"
#ifdef OS_LINUX_CSL_TYPE
#include <linux/string.h>
#include <linux/kernel.h>
#else /* OS_LINUX_CSL_TYPE */
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#endif /* OS_LINUX_CSL_TYPE */

/*****************************************************************************/
/* Constants Macros                                                          */
/*****************************************************************************/
#define FW_VERSION              "v2.0.D"

#define NSID_LEN                16
#define MAC_ADDRESS_LEN          6

#define UNSPEC_REASON           1
#define AUTH_1X_FAIL            23

/* Bit Values */
#define BIT31                   (1 << 31)
#define BIT30                   (1 << 30)
#define BIT29                   (1 << 29)
#define BIT28                   (1 << 28)
#define BIT27                   (1 << 27)
#define BIT26                   (1 << 26)
#define BIT25                   (1 << 25)
#define BIT24                   (1 << 24)
#define BIT23                   (1 << 23)
#define BIT22                   (1 << 22)
#define BIT21                   (1 << 21)
#define BIT20                   (1 << 20)
#define BIT19                   (1 << 19)
#define BIT18                   (1 << 18)
#define BIT17                   (1 << 17)
#define BIT16                   (1 << 16)
#define BIT15                   (1 << 15)
#define BIT14                   (1 << 14)
#define BIT13                   (1 << 13)
#define BIT12                   (1 << 12)
#define BIT11                   (1 << 11)
#define BIT10                   (1 << 10)
#define BIT9                    (1 << 9)
#define BIT8                    (1 << 8)
#define BIT7                    (1 << 7)
#define BIT6                    (1 << 6)
#define BIT5                    (1 << 5)
#define BIT4                    (1 << 4)
#define BIT3                    (1 << 3)
#define BIT2                    (1 << 2)
#define BIT1                    (1 << 1)
#define BIT0                    (1 << 0)
#define ALL                     0xFFFF

/*****************************************************************************/
/* Function Macros                                                           */
/*****************************************************************************/
#define MAX(a, b)                   ((a > b)?a:b)
#define MIN(a, b)                   ((a < b)?a:b)

#define GET_U16_BE(buff)    (UWORD16)(((UWORD16)(buff)[0]<<8)+(UWORD16)buff[1])

#define GET_U32_BE(buff)    (UWORD32)(((UWORD32)(buff)[0] << 24) | \
                                      ((UWORD32)(buff)[1] << 16) | \
                                      ((UWORD32)(buff)[2] <<  8) | \
                                      ((UWORD32)(buff)[0]))

#define PUT_U16_BE(buff, val)   (buff)[0] = (UWORD8)(((UWORD16)val>>8)&0xFF);\
                                (buff)[1] = (UWORD8)((UWORD16)val&0xFF);

#define PUT_U32_BE(buff, val)   (buff)[0] = (UWORD8)(((UWORD32)val>>24)&0xFF);\
                                (buff)[1] = (UWORD8)(((UWORD32)val>>16)&0xFF);\
                                (buff)[2] = (UWORD8)(((UWORD32)val>>8)&0xFF); \
                                (buff)[3] = (UWORD8)((UWORD32)val&0xFF);

// 20120830 caisf mod, merged ittiam mac v1.3 code
#ifdef DEBUG_MODE
#ifdef OS_LINUX_CSL_TYPE
    #define DBG_PRINT(args...) printk(args)
#endif /* OS_LINUX_CSL_TYPE */
#else  /* DEBUG_MODE */
    #define DBG_PRINT(args...)
#endif /* DEBUG_MODE */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
/* Status */
typedef enum {FAILURE = 0,
              SUCCESS = 1
} STATUS_T;

/* System error codes */
typedef enum {NO_ERROR       = 0x00,
              NO_LOCAL_MEM   = 0xE0,
              NO_SHRED_MEM   = 0xE1,
              SYSTEM_ERROR   = 0xBB
} ERROR_CODE_T;


/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/


/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void wps_gen_uuid(UWORD8 *mac_addr, UWORD8 *uuid, UWORD8 *temp);
extern void wps_get_rand_byte_array(UWORD8* inp_ptr, UWORD16 arr_len);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* This function compares two given MAC addresses (m1 and m2).               */
/* Returns BOOL_T, BTRUE if the two addresses are same and BFALSE otherwise. */
/*****************************************************************************/
INLINE BOOL_T mac_addr_cmp(UWORD8 *m1, UWORD8 *m2)
{
    if(memcmp(m1, m2, 6) == 0)
        return BTRUE;

    return BFALSE;
}

/* This function copies the MAC address from a source to destination */
INLINE void mac_addr_cpy(UWORD8 *dst, UWORD8 *src)
{
    UWORD8 i = 0;

    for(i = 0; i < 6; i++)
        dst[i] = src[i];
}

/* This function copies the IP address from a source to destination */
INLINE void ip_addr_cpy(UWORD8 *dst, UWORD8 *src)
{
    UWORD8 i = 0;

    for(i = 0; i < 4; i++)
        dst[i] = src[i];
}

/* This function converts BCD to binary */
INLINE UWORD8 bcd_2_bin(UWORD8 val)
{
    return (((val >> 4) & 0x0F) * 10) + (val & 0x0F);
}
#endif /* COMMON_H */
#endif /* INT_WPS_REG_SUPP */
