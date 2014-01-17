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
/*  File Name         : csl_types.h                                          */
/*                                                                           */
/*  Description       : This file contains all all the data type definitions */
/*                      for the chip support library.                        */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CSL_TYPES_H
#define CSL_TYPES_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "common.h"
#ifdef OS_LINUX_CSL_TYPE
struct work_struct;
#endif
/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/
typedef void    HANDLE_T;
typedef UWORD32 ADDRWORD_T;
typedef UWORD32 VECTOR_T;
typedef void    ALARM_HANDLE_T;
#ifndef OS_LINUX_CSL_TYPE
typedef void    ALARM_FUNC_T(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
typedef void    ALARM_FUNC_T(ADDRWORD_T data);
typedef void    ALARM_FUNC_WORK_T(struct work_struct *work);        //add by Hugh

#endif /* OS_LINUX_CSL_TYPE */
typedef void    INTERRUPT_HANDLE_T;
typedef UWORD32 INTERRUPT_VECTOR_T;
typedef UWORD32 INTERRUPT_PRIORITY_T;
typedef UWORD32 ISR_FN_T(VECTOR_T vector, ADDRWORD_T data);
typedef void    DSR_FN_T(VECTOR_T  vector, UWORD32 count, ADDRWORD_T data);
typedef void    RTC_HANDLE_T;
typedef void    TASK_FUNC_T(ADDRWORD_T data);
typedef void    TASK_HANDLE_T;
typedef WORD8   TASK_STACK_BASE_T;
typedef WORD8   MEM_POOL_BASE_T;

#ifdef GENERIC_CSL
typedef struct
{
    ISR_FN_T *isr;
    DSR_FN_T *dsr;
} isr_t;

typedef isr_t ISR_T;
#endif /* GENERIC_CSL */

#endif /* CSL_TYPES_H */
