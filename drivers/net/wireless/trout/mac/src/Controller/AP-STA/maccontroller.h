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
/*  File Name         : maccontroller.h                                      */
/*                                                                           */
/*  Description       : This file contains all the declarations related to   */
/*                      the MAC controller (event manager).                  */
/*                                                                           */
/*  List of Functions : raise_system_error                                   */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MAC_CONTROLLER_H
#define MAC_CONTROLLER_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "csl_if.h"
#include "common.h"
#include "itypes.h"
#include "qmu.h"
#include "queue.h"
#include "event_manager.h"

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Event types */
typedef enum {HOST_RX = 0x00,
              WLAN_RX = 0x20,
              MISC    = 0x40
} EVENT_BASICTYPE_T;

/* Event sub types */
typedef enum {HOST_RX_MSDU = 0x00,
              HOST_RX_MLME = 0x01,
              WLAN_RX_DATA = 0x20,
              WLAN_RX_MGMT = 0x21,
              MISC_TBTT    = 0x40,
              MISC_TIMEOUT = 0x41,
              MISC_ATIM    = 0x42,
              MISC_TX_COMP = 0x43,
              MISC_ERROR   = 0x44,
/* 0x45 to 0x49 are 11i events and have been defined in 802.11i files */
/* 0x50 to 0x5F are 11n events and has been defined in 802.11n files  */
/* 0x60 is a 11h event and has been defined in 802.11h files          */
/* 0x70 to 0x7F are WPS events and has been defined in WPS files      */
/* 0x80 to 0x8F are P2P events and has been defined in P2P files      */
              MISC_RUN_AGING_FN = 0x4A,
              MISC_OBSS_TO      = 0x4B,
} EVENT_TYPESUBTYPE_T;

/* System error codes */

typedef enum {NO_ERROR        = 0x00,
              NO_LOCAL_MEM    = 0xE0,
              NO_SHRED_MEM    = 0xE1,
              NO_EVENT_MEM    = 0xE2,
              MEM_CORRUPTED  = 0xE3,
              RSNA_HS_FAIL    = 0xD0,
              WRONG_GTK_ID    = 0xD1,
              CUSTOM_CNTRMSR  = 0xD2,
              CORRUPTED_PN    = 0xD3,
              CE_UPDATE_FAIL  = 0xD4,
              PA_CORRUPT      = 0xC0,
              LINK_LOSS       = 0xC1,
              TX_Q_ERROR      = 0xC2,
              RX_Q_ERROR      = 0xC3,
              DEAUTH_RCD      = 0xC4,
              CORRUPTED_ADDR  = 0xC5,
              LINK_LOSS_PA    = 0xC6,
              LINK_LOSS_BUFF  = 0xC7,
              LINK_LOSS_CCA   = 0xC8,
              LINK_LOSS_RX    = 0xC9,
              RESCAN_NEEDED   = 0xCA,
              SWITCH_NETWK    = 0xCB,
              NO_NETWORKS     = 0xCC,
              AUTO_JOIN_REQ   = 0xCD,
              DFS_ERROR       = 0xB0,
              PHY_HW_ERROR    = 0xB1,
              MDOM_ERROR      = 0xB2, // 20120830 caisf add, merged ittiam mac v1.3 code
              SYSTEM_ERROR    = 0xBB
} ERROR_CODE_T;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern ERROR_CODE_T g_system_error;
extern mac_struct_t g_mac;
extern void         (*g_process_event[MAX_NUM_EVENT_QUEUES])(UWORD32);
extern qmu_handle_t g_q_handle;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void   init_event_manager(void);
extern void   process_all_events(void);
extern void   mac_controller_task(ADDRWORD_T data);
extern void   process_host_rx_event(UWORD32 event);
extern void   process_wlan_rx_event(UWORD32 event);
extern void   process_misc_event(UWORD32 event);
extern void   process_host_tx_event(UWORD32 event);
extern void   handle_system_error(void);
extern BOOL_T is_serious_error(ERROR_CODE_T error_code);
extern void   process_wlan_event_q(UWORD8 qid);
extern void   process_host_event_q(UWORD8 qid);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

//chenq add 2013-06-09
extern atomic_t g_mac_reset_done;

INLINE void raise_system_error(ERROR_CODE_T error_code)
{
    //chenq add 2013-06-09
    if((BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE)
    {
    	TROUT_DBG4("report system error, err_code=0x%x,but mac is resetting return\n", error_code);
		return;
	}

    /* If an error has already been logged, then it is overwritten only if   */
    /* the new error is a serious error.                                     */
    if((g_system_error == NO_ERROR) || (is_serious_error(error_code) == BTRUE))
    {
        g_system_error = error_code;
		TROUT_DBG4("report system error, err_code=0x%x\n", error_code);
#ifdef DEBUG_MODE
		print_symbol("caller: %s\n", (unsigned long)__builtin_return_address(0));
#endif
	}
#ifdef OS_LINUX_CSL_TYPE
    dump_stack();
#endif /* OS_LINUX_CSL_TYPE */

}


INLINE void raise_system_warning(ERROR_CODE_T error_code)
{
    /* If an error has already been logged, then it is overwritten only if   */
    /* the new error is a serious error.                                     */
    if(is_serious_error(error_code) == BTRUE)
    {
		TROUT_DBG4("report system warning, err_code=0x%x\n", error_code);
#ifdef DEBUG_MODE
		print_symbol("caller: %s\n", (unsigned long)__builtin_return_address(0));
#endif
	}
#ifdef OS_LINUX_CSL_TYPE
    dump_stack();
#endif /* OS_LINUX_CSL_TYPE */

}

#endif /* MAC_CONTROLLER_H */
