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
/*  File Name         : csl_if.h                                             */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to the chip support library.                 */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CSL_IF_H
#define CSL_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/


#ifdef OS_LINUX_CSL_TYPE
#include "csl_linux.h"
#endif /* OS_LINUX_CSL_TYPE */

#include "csl_types.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#ifdef GENERIC_PLATFORM
#define PA_BASE             (0x00000000)
#define CE_BASE             (0x00000000)
#define PLD0_ADDR           (0x00000000)
#endif /* GENERIC_PLATFORM */

/* Interrupt priorites and data */
#define MAC_INTERRUPT_PRI 100
#define MAC_INTERRUPT_DAT 0

#ifdef GENERIC_CSL
#define OS_ISR_HANDLED      1
#define OS_ISR_CALL_DSR     2
#endif /* GENERIC_CSL */

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

/*****************************************************************************/
/* Initialization and Reset related functions                                */
/*****************************************************************************/
#ifndef GENERIC_CSL
extern void csl_init(void);
extern void csl_mem_init(void);
extern void csl_reset(void);

extern void csl_led_init(void);
extern void led_set_pat(UWORD32 pat);
extern void led_clr_pat(UWORD32 pat);
extern void led_toggle_pat(UWORD32 pat);
#endif /* GENERIC_CSL */

/*****************************************************************************/
/* Interrupt related functions                                               */
/*****************************************************************************/
#ifndef GENERIC_CSL
extern INTERRUPT_HANDLE_T* create_interrupt(INTERRUPT_VECTOR_T vec,
                                            INTERRUPT_PRIORITY_T pri,
                                            ADDRWORD_T data, ISR_T *isr);
extern void                delete_interrupt(INTERRUPT_HANDLE_T *handle);
extern void                disable_interrupt(VECTOR_T int_vec);
extern void                enable_interrupt(VECTOR_T int_vec);
extern void                acknowledge_interrupt(VECTOR_T int_vec);
#endif /* GENERIC_CSL */

/*****************************************************************************/
/* Alarm related functions                                                   */
/*****************************************************************************/
#ifndef GENERIC_CSL
#define ALARM_WORK_ENTRY(work_struct)   do{((trout_timer_struct *)container_of(work_struct, trout_timer_struct, work))->wid = (unsigned int)current;}while(0);
#define ALARM_WORK_EXIT(work_struct)   do{((trout_timer_struct *)container_of(work_struct, trout_timer_struct, work))->wid = 0xffffffff;}while(0);
#define ALARM_STATE_IN(alarm, states)  do{((trout_timer_struct*)alarm)->state |= (0x1<<states);}while(0);
extern atomic_t g_mac_reset_done;

extern ALARM_HANDLE_T* create_alarm(ALARM_FUNC_T* func, ADDRWORD_T data, ALARM_FUNC_WORK_T *work_func);
extern void            delete_alarm(ALARM_HANDLE_T** handle);
extern void            start_alarm(ALARM_HANDLE_T* handle, UWORD32 timeout_ms);
extern void            stop_alarm(ALARM_HANDLE_T* handle);
INLINE void alarm_fn_work_sched(ADDRWORD_T data)
{
    ALARM_STATE_IN(data, ALARM_RUN_START);
#ifdef ALARM_WQ
    struct trout_private *tp = netdev_priv(&g_mac_dev);
    if((data != NULL) && ((BOOL_T)atomic_read(&g_mac_reset_done) == BTRUE))
    {
       ALARM_STATE_IN(data, ALARM_RUN_MID);
       queue_work_on(0, tp->alarm_wq, &((trout_timer_struct*)data)->work);
    }
#else
   if((data != 0) && ((BOOL_T)atomic_read(&g_mac_reset_done) == BTRUE))
   {
      ALARM_STATE_IN(data, ALARM_RUN_MID);
      schedule_work_on(0, &((trout_timer_struct *)data)->work);
    }
#endif
    ALARM_STATE_IN(data, ALARM_RUN_END);
}
#endif /* GENERIC_CSL */

/*****************************************************************************/
/* Task related functions                                                    */
/*****************************************************************************/

extern void start_mac_controller(void);

/*****************************************************************************/
/* Mode switching function                                                   */
/*****************************************************************************/
extern void switch_operating_mode(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

INLINE void send_packet_to_stack(void *msdu_desc)
{
#ifdef OS_LINUX_CSL_TYPE
    send_packet_to_stack_lnx(msdu_desc);
#endif /* OS_LINUX_CSL_TYPE */
}

/* Function to clear Interrupts */
#ifndef GENERIC_CSL
INLINE void clear_interrupt(VECTOR_T int_vec)
{
    acknowledge_interrupt(int_vec);
}
#endif /* GENERIC_CSL */

INLINE void print_banner(void)
{
#ifdef OS_LINUX_CSL_TYPE
    print_banner_lnx();
#endif /* OS_LINUX_CSL_TYPE */
}

#ifdef GENERIC_CSL
/* This function converts the virtual address to physical address */
INLINE UWORD32 virt_to_phy_addr(UWORD32 addr)
{
    return addr;
}

/* This function converts the physical address to virtual address */
INLINE UWORD32 phy_to_virt_addr(UWORD32 addr)
{
    return addr;
}

/* This function creates an interrupt with the specified parameters */
/* The interrupt handle is returned on Success and NULL on failure  */
INLINE INTERRUPT_HANDLE_T* create_interrupt(INTERRUPT_VECTOR_T vec,
                                            INTERRUPT_PRIORITY_T pri,
                                            ADDRWORD_T data, ISR_T *isr)
{
    return NULL;
}

/* This function deletes the specified interrupt */
INLINE void delete_interrupt(INTERRUPT_HANDLE_T *handle)
{
    return;
}

/* This function disables the specified interrupt */
INLINE void disable_interrupt(VECTOR_T int_vec)
{
    return;
}

/* This function enables the specified interrupt */
INLINE void enable_interrupt(VECTOR_T int_vec)
{
    return;
}

/* This function acknowledges the specified interrupt */
INLINE void acknowledge_interrupt(VECTOR_T int_vec)
{
    return;
}

/* This function creates an alarm with the specified parameters */
/* The alarm handle is returned on Success and NULL on failure  */
INLINE ALARM_HANDLE_T* create_alarm(ALARM_FUNC_T* func, ADDRWORD_T data, ALARM_FUNC_WORK_T *work_func)
{
    return (INTERRUPT_HANDLE_T*)NULL;
}

/* This function deletes the specified alarm */
INLINE void delete_alarm(ALARM_HANDLE_T** handle)
{
    return;
}

/* This function starts the specified alarm for the required duration */
INLINE BOOL_T start_alarm(ALARM_HANDLE_T* handle, UWORD32 timeout_ms)
{
    return BFALSE;
}

/* This function stops the specified alarm */
INLINE void stop_alarm(ALARM_HANDLE_T* handle)
{
    return;
}

/* This function performs the required CSL Initialization */
INLINE void csl_init(void)
{
    return;
}

/* This function performs the necessary initialization of Memory Used for CSL */
INLINE void csl_mem_init(void)
{
    return;
}

/* This function resets the CSL */
INLINE void csl_reset(void)
{
    return;
}

#endif /* GENERIC_CSL */

#endif /* CSL_IF_H */
