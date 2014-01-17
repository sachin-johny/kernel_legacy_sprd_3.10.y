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
/*  File Name         : proc_if.c                                            */
/*                                                                           */
/*  Description       : This file contains the functions for the processor   */
/*                      interface.                                           */
/*                                                                           */
/*  List of Functions : led_display                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "itypes.h"
#include "proc_if.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD32 g_mac_int_vector = 0;
UWORD32 g_int_mask_val   = 0;

#ifdef MWLAN
#ifdef OS_LINUX_CSL_TYPE
unsigned long g_saved_context;
UWORD32 g_disable_interrupt_counter = 0;

#endif /* OS_LINUX_CSL_TYPE */
#endif /* MWLAN */


/*****************************************************************************/
/*                                                                           */
/*  Function Name : led_display                                              */
/*                                                                           */
/*  Description   : This function is to switch on various combinations of    */
/*                  LEDs on the Ittiam validation platform.                  */
/*                                                                           */
/*  Inputs        : 1) Desired LED pattern                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the register to display the LEDs in   */
/*                  the desired pattern.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void led_display(UWORD8 a)
{
    /* Currently no function present. Depends on platform support. */
}

DEFINE_SPINLOCK(cr_sec_lock);
unsigned long cr_irq_flags = 0;