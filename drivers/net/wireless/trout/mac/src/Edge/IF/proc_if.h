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
/*  File Name         : proc_if.h                                            */
/*                                                                           */
/*  Description       : This file contains the definitions and declarations  */
/*                      for the processor interface.                         */
/*                                                                           */
/*  List of Functions : init_int_vec_globals                                 */
/*                      init_proc                                            */
/*                      critical_section_start                               */
/*                      critical_section_end                                 */
/*                      led_on                                               */
/*                      led_off                                              */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef PROC_IF_H
#define PROC_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "csl_if.h"
#include "runmode.h"

#ifdef MWLAN
#include "mpc8349.h"
#endif /* MWLAN */


/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAC_ADDRESS_LOCATION ((UWORD8 *)0x00000000) /* TBD */


/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD32 g_mac_int_vector;
extern UWORD32 g_int_mask_val;

#ifdef MWLAN
#ifdef OS_LINUX_CSL_TYPE
extern unsigned long g_saved_context;
extern UWORD32 g_disable_interrupt_counter;
#endif /* OS_LINUX_CSL_TYPE */
#endif /* MWLAN */

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void led_display(UWORD8 nibble);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

INLINE void init_int_vec_globals(void)
{
#ifdef MWLAN
    g_mac_int_vector = MPC83xx_IRQ_EXT1;
#endif /* MWLAN */


}

INLINE void init_proc(void)
{
#ifdef MWLAN
    init_led_control();

    init_mwlan_wdt();
#endif /* MWLAN */
}

INLINE void service_wdt(void)
{
#ifdef MWLAN
    service_mwlan_wdt();
#endif /* MWLAN */

}

INLINE void critical_section_start(void)
{
    return; // Hugh
#ifdef MWLAN
#ifdef OS_LINUX_CSL_TYPE
    if(0 == g_disable_interrupt_counter)
    {
        local_irq_save (g_saved_context);
    }
    g_disable_interrupt_counter++;

#endif /* OS_LINUX_CSL_TYPE */
#endif /* MWLAN */
}

INLINE void critical_section_end(void)
{
    return; // Hugh
#ifdef MWLAN
#ifdef OS_LINUX_CSL_TYPE
    g_disable_interrupt_counter--;

    if(g_disable_interrupt_counter == 0)
    {
        local_irq_restore (g_saved_context);
    }

#endif /* OS_LINUX_CSL_TYPE */
#endif /* MWLAN */
}



INLINE void critical_section_start1(void)
{
#ifdef MWLAN
#ifdef OS_LINUX_CSL_TYPE
    if(0 == g_disable_interrupt_counter)
    {
        local_irq_save (g_saved_context);
    }
    g_disable_interrupt_counter++;

#endif /* OS_LINUX_CSL_TYPE */
#endif /* MWLAN */
}

INLINE void critical_section_end1(void)
{
#ifdef MWLAN
#ifdef OS_LINUX_CSL_TYPE
    g_disable_interrupt_counter--;

    if(g_disable_interrupt_counter == 0)
    {
        local_irq_restore (g_saved_context);
    }

#endif /* OS_LINUX_CSL_TYPE */
#endif /* MWLAN */
}

extern spinlock_t cr_sec_lock;
extern unsigned long cr_irq_flags;

INLINE void critical_section_start11(void)
{
    //print_symbol("crit_start: caller: %s\n", (unsigned long)__builtin_return_address(0));
    spin_lock_irqsave(&cr_sec_lock, cr_irq_flags);
}

INLINE void critical_section_end11(void)
{
    //print_symbol("crit_end: caller: %s\n", (unsigned long)__builtin_return_address(0));
    spin_unlock_irqrestore(&cr_sec_lock, cr_irq_flags);
}

INLINE int critical_section_smart_start(char s,char m)
{
	int mode = MODE_START;
	//spin_lock(&runmode_lock);
	//trout_runmode_lock();
    	mode = trout_get_runmode();
	//BUG_ON(mode >MODE_START && mode <MODE_END);
	if(mode == SINGLE_CPU_MODE && s){
		critical_section_start1();
	}else if(mode == SMP_MODE && m){
		critical_section_start11();
	}else{
		mode = MODE_START;
	}
	return mode;
}

INLINE void critical_section_smart_end(int mode)
{
	//BUG_ON(mode >MODE_START && mode <MODE_END);
	if(mode == SINGLE_CPU_MODE){
		critical_section_end1();
	}else if(mode == SMP_MODE){
			critical_section_end11();
	}
	//trout_runmode_unlock();
	//spin_unlock(&runmode_lock);
}


#ifdef USE_PROCESSOR_DMA
/* This function does a DMA transfer of len bytes from the source (src) to  */
/* destination (dst). Both src and dst arguments should be physical         */
/* addresses.                                                               */
/* This function returns 0 on Success and 1 on Failure.                     */
INLINE UWORD8 do_proc_dma(void *dst, void *src, UWORD32 len)
{
#ifdef MWLAN
    return do_chan0_dma(dst, src, len);
#endif /* MWLAN */

    return 1;
}
#endif /* USE_PROCESSOR_DMA */

/* This function switches off the LED */
INLINE void led_on(UWORD8 val)
{
#ifdef MWLAN
    if(val == 1)
        TURN_ON_LED(1);
    else if(val == 2)
        TURN_ON_LED(2);
    else if(val == 3)
        TURN_ON_LED(3);
    else if(val == 4)
        TURN_ON_LED(4);
    else if(val == 5)
        TURN_ON_LED(5);
#endif /* MWLAN */


}

/* This function switches on the LED */
INLINE void led_off(UWORD8 val)
{
#ifdef MWLAN
    if(val == 1)
        TURN_OFF_LED(1);
    else if(val == 2)
        TURN_OFF_LED(2);
    else if(val == 3)
        TURN_OFF_LED(3);
    else if(val == 4)
        TURN_OFF_LED(4);
    else if(val == 5)
        TURN_OFF_LED(5);
#endif /* MWLAN */


}

#endif /* PROC_IF_H */
