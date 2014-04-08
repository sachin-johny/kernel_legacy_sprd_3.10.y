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
/*  File Name         : mac_init.h                                           */
/*                                                                           */
/*  Description       : This file contains extern declarations for all the   */
/*                      functions required for initialization.               */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_INIT_H
#define MAC_INIT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "maccontroller.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define DEAUTH_SEND_TIME_OUT_COUNT 20

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE
extern void test_main_function(void);
extern void configure_mac(void);
extern void test_reinit_sw(mac_struct_t *mac);
#else /* MAC_HW_UNIT_TEST_MODE */
extern int main_function(void);
#endif /* MAC_HW_UNIT_TEST_MODE */

extern void pre_reset_tasks_host(mac_struct_t *mac);
extern void pre_reset_tasks_wlan(mac_struct_t *mac);
extern int initialize_macsw(mac_struct_t *mac);
extern void reset_mac(mac_struct_t *mac, BOOL_T init_mac_sw);
extern void restart_mac(mac_struct_t *mac, UWORD32 delay);
extern void start_mac_and_phy(mac_struct_t *mac);
extern void stop_mac_and_phy(void);
extern int reset_mac_trylock(void);
extern void reset_mac_unlock(void);
extern void reset_mac__lock(void);
extern int wsem_is_locked(void);

//chenq add 2012-10-29
extern void initialize_plus(void);

//chenq add 2012-12-26
extern void restart_mac_plus(mac_struct_t *mac, BOOL_T init_mac_sw);
/* export to other one by zhao */
void wait_for_tx_finsh(void);

#ifdef BSS_ACCESS_POINT_MODE
void raise_beacon_th(void);
int create_beacon_thread(void);
void delete_beacon_thread(void);
#endif	/* BSS_ACCESS_POINT_MODE */

#endif /* MAC_INIT_H */
