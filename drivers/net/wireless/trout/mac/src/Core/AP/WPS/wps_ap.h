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
/*  File Name         : wps_ap.h                                             */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      WPS AP                                               */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/
#ifdef BSS_ACCESS_POINT_MODE
#ifdef INT_WPS_SUPP

#ifndef WPS_AP_H
#define WPS_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_reg_if.h"
#include "common.h"
#include "management_ap.h"
#include "index_util.h"
#include "management.h"
#include "maccontroller.h"
#include "ap_management_wps.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/
#define WPS_REG_PROT_TIMEOUT    1
#define WPS_EAP_MSG_TIMEOUT     2
#define MAX_NESTED_SCRATCH_MEM_ALLOC   2

#define WPS_CONFIG_METH_USBA            0x0001
#define WPS_CONFIG_METH_ETHERNET        0x0002
#define WPS_CONFIG_METH_LABEL           0x0004
#define WPS_CONFIG_METH_DISPLAY         0x0008
#define WPS_CONFIG_METH_EXT_NFC_TOKEN   0x0010
#define WPS_CONFIG_METH_INT_NFC_TOKEN   0x0020
#define WPS_CONFIG_METH_NFC_INTERFACE   0x0040
#define WPS_CONFIG_METH_PUSHBUTTON      0x0080
#define WPS_CONFIG_METH_KEYPAD          0x0100
#define WPS_CONFIG_METH_PB_ON_WEB_PAGE  0x4000
#define WPS_CONFIG_METH_PIN_ON_WEB_PAGE 0x8000
#define WPS_CONFIG_METHOD_VIRTUAL_PBC   0x0280
#define WPS_CONFIG_METHOD_PHYSICAL_PBC  0x0480
#define WPS_CONFIG_METHOD_VIRTUAL_PIN   0x2008
#define WPS_CONFIG_METHOD_PHYSICAL_PIN  0x4008

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
/* WPS events */
typedef enum
{
    WPS_TIMEOUT_EVENT     = 0x70
} EVENT_TYPESUBTYPE_WPS_T;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T g_wps_reg_enabled;
extern BOOL_T g_wps_allow_config;
extern BOOL_T g_wps_reg_init_done;
extern wps_config_t *g_wps_config_ptr;
extern wps_t *g_wps_ctxt_ptr;
extern wps_eap_tx_t *g_wps_eap_tx_ptr;
extern wps_eap_rx_t *g_wps_eap_rx_ptr;
extern wps_priv_t *g_wps_priv_ptr;
extern wps_attr_t *g_wps_attr_ptr;
extern wps_reg_t *g_wps_reg_ptr;
extern BOOL_T g_wps_implemented_reg;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void sys_init_wps_reg(void);
extern void sys_start_wps_reg(WPS_PROT_TYPE_T prot_type);
extern void sys_stop_wps_reg(UWORD16 reason_code, UWORD8 *mac_addr,
                             BOOL_T start_req);
extern BOOL_T handle_wps_event_ap(UWORD32 event);
extern BOOL_T wps_allow_sys_restart_ap(UWORD16 sys_error);
#ifndef OS_LINUX_CSL_TYPE
extern void wps_timeout_fn_ap(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void wps_timeout_fn_ap(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */
extern void* wps_local_mem_alloc(UWORD16 size);
extern void wps_local_mem_free(void *buffer_addr);
extern void* wps_pkt_mem_alloc(UWORD16 size);
extern void wps_pkt_mem_free(void *buffer_addr);
extern UWORD8 wps_get_num_users(void *buffer);
extern void wps_add_mem_users(void *buffer, UWORD8 num_user);
extern void* wps_create_alarm(WPS_ALARM_FUNC_T* func, UWORD32 data, ALARM_FUNC_WORK_T *work_func);
extern void wps_delete_alarm(void** handle);
extern BOOL_T wps_start_alarm(void* handle, UWORD32 timeout_ms);
extern void wps_stop_alarm(void* handle);
extern BOOL_T wps_handle_event_ap(UWORD32 event);
extern void wps_get_tsf_timestamp(UWORD32 *tsf_timer_hi,
                                  UWORD32 *tsf_timer_lo);
extern void wps_handle_sys_err_reg(UWORD16 sys_error);
extern void wps_handle_disconnect_req_ap(asoc_entry_t *ae, UWORD8 *sta_addr);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This fucntion return BTRUE if the WPS Registrar functionality is          */
/* implemented else returns BFALSE                                           */
INLINE BOOL_T get_wps_implemented_reg(void)
{
    return g_wps_implemented_reg;
}

/* This function sets g_wps_implemented_reg variable with input value        */
INLINE void set_wps_implemented_reg(BOOL_T val)
{
    g_wps_implemented_reg = val;
}

/* This fucntion return BTRUE if the Registrar is enabled else returns BFALSE*/
INLINE BOOL_T get_wps_reg_enabled(void)
{
    return g_wps_reg_enabled;
}

/* This function sets g_wps_reg_enabled variable with specified input value  */
INLINE void set_wps_reg_enabled(BOOL_T val)
{
    BOOL_T update_beacon = BFALSE;

    if(val != g_wps_reg_enabled)
        update_beacon = BTRUE;

    g_wps_reg_enabled = val;

    if(BTRUE == update_beacon)
    {
        /* If the registrar is going to be disabled then Stop any WPS process*/
        /* in progress.                                                      */
        if(BFALSE == val)
        {
            sys_stop_wps_reg(UNSPEC_REASON, g_wps_config_ptr->mac_addr,
                             BFALSE);
        }
        /* If the registrar is getting enabled then WSC-IE needs to be       */
        /* included in the beacon frames                                     */
        else
        {
            wps_update_beacon_wsc_ie_ap();
        }
    }
}

/* This function sets g_wps_allow_config variable with specified input value */
INLINE void set_wps_allow_config(BOOL_T val)
{
    g_wps_allow_config = val;
}

/* This function returns the value of g_wps_allow_config variable            */
INLINE BOOL_T get_wps_allow_config(void)
{
    return g_wps_allow_config;
}

/* This function gets the WPS Protocol Mode the AP is currently configured   */
INLINE UWORD8 get_wps_prot_reg(void)
{
    return (UWORD8)g_wps_config_ptr->prot_type;
}

/* This function sets ths WPS Protocol Mode of the AP                        */
INLINE void set_wps_prot_reg(UWORD8 val)
{
// 20120709 caisf masked, merged ittiam mac v1.2 code
/*
    if(BFALSE == get_wps_reg_enabled())
        return;
*/
    if((BTRUE == is_wps_in_prog_reg(g_wps_ctxt_ptr)) ||
       (WPS_PROT_NONE == val))
       sys_stop_wps_reg(UNSPEC_REASON, g_wps_config_ptr->mac_addr, BFALSE);

    if(WPS_PROT_NONE != val)
        sys_start_wps_reg((WPS_PROT_TYPE_T)val);
}

/* This function returns the programmed Password ID of the AP/Enrollee       */
INLINE UWORD16 get_wps_pass_id_reg(void)
{
    return g_wps_config_ptr->dev_pass_id;
}

/* This function programs the Device Password ID of the AP or Enrollee       */
INLINE void set_wps_pass_id_reg(UWORD16 val)
{
    g_wps_config_ptr->dev_pass_id = val;
}

/* This function returns the Configuration Methods Programmed in the AP      */
INLINE UWORD16 get_wps_config_method_reg(void)
{
    return g_wps_config_ptr->config_methods;
}

/* This function programs the valid WPS  Configuration Methods in the AP     */
INLINE void set_wps_config_method_reg(UWORD16 val)
{
    g_wps_config_ptr->config_methods = val;
}

/* This function returns the PIN programmed in the AP                        */
INLINE UWORD8 *get_wps_pin_reg(void)
{
    g_cfg_val[0] = g_wps_config_ptr->dev_pin_len;

    memcpy((g_cfg_val + 1), g_wps_config_ptr->dev_pin, g_cfg_val[0]);
    return g_cfg_val;
}

/* This function programs the WPS PIN of the AP or the Enrollee              */
INLINE void set_wps_pin_reg(UWORD8 *val)
{
    UWORD8 len = strlen((const char *)val);

    len = (len > MAX_WPS_PIN_LEN) ? MAX_WPS_PIN_LEN : len;
    memcpy(&g_wps_config_ptr->dev_pin, val, len);
    g_wps_config_ptr->dev_pin_len = len;
}

/* This function updates all the memory handles in WPS Context strcuture     */
INLINE void init_wps_ctxt_hdl(void)
{
    g_wps_ctxt_ptr->wps_eap_rx_hdl = g_wps_eap_rx_ptr;
    g_wps_ctxt_ptr->wps_eap_tx_hdl = g_wps_eap_tx_ptr;
    g_wps_ctxt_ptr->wps_config_hdl = g_wps_config_ptr;
    g_wps_ctxt_ptr->wps_priv_hdl   = g_wps_priv_ptr;
    g_wps_ctxt_ptr->wps_attr_hdl   = g_wps_attr_ptr;
    g_wps_ctxt_ptr->wps_reg_hdl    = g_wps_reg_ptr;

    g_wps_ctxt_ptr->wps_timer_cb_fn_ptr = wps_timeout_fn_ap;
}

/* This function initializes the WPS AP Configuration structure */
INLINE void wps_init_config_ap(void)
{
    UWORD8 j = 0;

    g_wps_config_ptr->prot_type = WPS_PROT_NONE;

    /* Indicate in all Messages as a WPS AP */
    g_wps_config_ptr->rsp_type  = AP;
    g_wps_config_ptr->dev_pass_id = PASS_ID_DEFAULT;
    g_wps_config_ptr->config_methods = (CONFIG_METHOD_VIRTUAL_PBC |
                                        CONFIG_METHOD_VIRTUAL_PIN);
    g_wps_config_ptr->wps_state = WPS_CONFIGURED;
    g_wps_config_ptr->conn_type_flags = 1; /* ESS */

    /* Lock the AP setup to indicate that no external registrar can configure*/
    /* the AP                                                                */
    g_wps_config_ptr->ap_setup_locked = BTRUE;

#ifndef WPS_1_0_SEC_SUPP
    g_wps_config_ptr->encr_type_flags |= (WPS_NONE | WPS_AES);

    g_wps_config_ptr->auth_type_flags |= (WPS_OPEN | WPS_WPA2_PSK);
#ifdef MAC_802_1X
    g_wps_config_ptr->auth_type_flags |= WPS_WPA2;
#endif /* MAC_802_1X */
#else  /* WPS_1_0_SEC_SUPP */
    g_wps_config_ptr->encr_type_flags |= (WPS_WEP | WPS_NONE);
    g_wps_config_ptr->encr_type_flags |= (WPS_TKIP | WPS_AES | WPS_AES_TKIP);

    g_wps_config_ptr->auth_type_flags |= (WPS_OPEN | WPS_WPA_PSK | WPS_SHARED |
                                          WPS_WPA2_PSK);
#ifdef MAC_802_1X
    g_wps_config_ptr->auth_type_flags |= (WPS_WPA | WPS_WPA2 | WPS_WPA_WPA2_MIXED);
#endif /* MAC_802_1X */
#endif /* WPS_1_0_SEC_SUPP */

    g_wps_config_ptr->rf_bands = get_rf_band();

    /* Set the Version2 subelement to indicate WSC 2.0 capable */
    g_wps_config_ptr->version2.id = WPS_SUB_ELEM_VER2;
    g_wps_config_ptr->version2.len = WPS_VER2_LEN;
    g_wps_config_ptr->version2.val[0] = WPS_VER2_VAL;

    /* Initialize Authorized MACs list */
    g_wps_config_ptr->auth_macs.id = WPS_SUB_ELEM_AUTH_MACS;
    g_wps_config_ptr->auth_macs.len = WPS_AUTH_MACS_LEN;

    for(j = 0; j < MAC_ADDRESS_LEN; j++)
        g_wps_config_ptr->auth_macs.val[j] = WPS_AUTH_MACS_VAL;
}
#endif /* WPS_AP_H */
#endif /* INT_WPS_SUPP */
#endif /* BSS_ACCESS_POINT_MODE */
