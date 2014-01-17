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
/*  File Name         : csl_if.c                                             */
/*                                                                           */
/*  Description       : This file contains the initialization related CSL    */
/*                      functions with no operating system in use.           */
/*                                                                           */
/*  List of Functions : user_main                                            */
/*                      start_mac_controller                                 */
/*                      switch_operating_mode                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "mac_init.h"
#include "maccontroller.h"

/*****************************************************************************/
/*                                                                           */
/*  Function Name : user_main                                                */
/*                                                                           */
/*  Description   : This is the entry point function for CSL with no OS.     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the main function of MAC software.   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void user_main(void)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    test_main_function();
#else /* MAC_HW_UNIT_TEST_MODE */
    main_function();
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_mac_controller                                     */
/*                                                                           */
/*  Description   : This function invokes the MAC Controller.                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The exact action performed by this function depends upon */
/*                  how MAC is ported on the system. In a No-OS type of      */
/*                  system, this function can call mac_controller_task(). In */
/*                  a device driver type of porting, this function does      */
/*                  nothing.                                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_mac_controller(void)
{
#ifdef GENERIC_CSL
    mac_controller_task(0);
#endif /* GENERIC_CSL */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : switch_operating_mode                                    */
/*                                                                           */
/*  Description   : This function switches the operating mode (STA/AP).      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls the appropriate CSL interface        */
/*                  function to switch the operating mode between STA and AP.*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void switch_operating_mode(void)
{
#ifdef OS_LINUX_CSL_TYPE
#ifdef MAC_P2P
    schedule_mod_switch();
#endif /* MAC_P2P */
#endif /* OS_LINUX_CSL_TYPE */
}
