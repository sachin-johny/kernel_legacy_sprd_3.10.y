/*****************************************************************************/
/*                                                                           */
/*                     Ittiam 802.11 MAC SOFTWARE                            */
/*                                                                           */
/*                  ITTIAM SYSTEMS PVT LTD, BANGALORE                        */
/*                           COPYRIGHT(C) 2010                               */
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
/*  File Name         : wps_sta.h                                            */
/*                                                                           */
/*  Description       : This file contains all definitions and functions     */
/*                      related to WPS feature in BSS STA mode.              */
/*                                                                           */
/*  List of Functions : get_wps_mode                                         */
/*                      set_wps_mode                                         */
/*                      disable_wps_mode                                     */
/*                      get_wps_prot_enr                                     */
/*                      set_wps_prot_enr                                     */
/*                      get_wps_dev_mode_enr                                 */
/*                      set_wps_dev_mode_enr                                 */
/*                      get_wps_pass_id_sta                                  */
/*                      set_wps_pass_id_sta                                  */
/*                      get_wps_config_method_sta                            */
/*                      set_wps_config_method_sta                            */
/*                      get_wps_pin_sta                                      */
/*                      set_wps_pin_sta                                      */
/*                      get_mac_cred_wps_status_sta                          */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef INT_WPS_SUPP
#ifdef IBSS_BSS_STATION_MODE

#ifndef WPS_STA_H
#define WPS_STA_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "wps_enr_if.h"
#include "mib.h"
#include "mib_11i.h"
#include "fsm.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

//#define WPS_TEST_BYPASS_EEPROM_CRED_RDWR
#define WPS_USE_SYS_CAP_FOR_SEC_AUTH
#define WPS_DATA_MEM_GLOBAL

#define MAX_CRED_SIZE          132  /* Size of Credential saved as WID list */
#define MAX_WPS_CRED_LIST_SIZE 1402
#define WPS_CRED_LIST_LEN_OFF         0
#define WPS_CRED_LIST_CNT_OFF         (WPS_CRED_LIST_LEN_OFF+2)
#define WPS_CRED_LIST_CRED_START_OFF  (WPS_CRED_LIST_CNT_OFF+1)

#define MAX_WPS_NW_JOIN_TIME   5000 /* 5 seconds */

/* Security mode related */
#define SEC_MODE_CT_OFFSET   5
#define SEC_MODE_CT_MASK     0x03
#define SEC_MODE_AUTH_OFFSET 3
#define SEC_MODE_AUTH_MASK   0x03
#define SEC_MODE_1X_AUTH_BIT BIT2
#define WPS_MODE_MASK        0x7
#define SEC_MODE_MASK        0x7F

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

#define DEFAULT_WPS_CONFIG_METH         (WPS_CONFIG_METHOD_VIRTUAL_PBC \
                                        | WPS_CONFIG_METHOD_VIRTUAL_PIN)

#define WSC_IE_IN_PROB_REQ              BIT0
#define WSC_IE_IN_ASSOC_REQ             BIT1

/*****************************************************************************/
/* mac_cred_wps_status bit format                                            */
/*****************************************************************************/
/* ------------------------------------------------------------------------- */
/* |Bit field|   Items             |           Parameters                  | */
/* ------------------------------------------------------------------------- */
/* |  Bit0   |  WPS process mode   | 0 = Normal mode(legacy mode)          | */
/* |         |                     | 1 = WPS process mode                  | */
/* ------------------------------------------------------------------------- */
/* |  Bit1   |  WPS authentication | 0 = Not running                       | */
/* |         |  mode               | 1 = Running                           | */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* |  Bit2-3 |  Credential status  | 00 = No Credential                    | */
/* |         |                     | 01 = Valid Credential                 | */
/* |         |                     | 02 = Invalid Credetial                | */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* |  Bit4   |  Credential activity| 0 = Inactive Credential               | */
/* |         |                     | 1 = Active Credential                 | */
/* ------------------------------------------------------------------------- */
#define MAC_CRED_WPS_ST_PROC_MD_FD_OFF 0
#define MAC_CRED_WPS_ST_PROC_MD_FD_LEN 1

#define MAC_CRED_WPS_ST_AUTH_MD_FD_OFF (MAC_CRED_WPS_ST_PROC_MD_FD_OFF + \
                                        MAC_CRED_WPS_ST_PROC_MD_FD_LEN)
#define MAC_CRED_WPS_ST_AUTH_MD_FD_LEN 1

#define MAC_CRED_WPS_ST_CRED_ST_FD_OFF (MAC_CRED_WPS_ST_AUTH_MD_FD_OFF + \
                                        MAC_CRED_WPS_ST_AUTH_MD_FD_LEN)
#define MAC_CRED_WPS_ST_CRED_ST_FD_LEN 2

#define MAC_CRED_WPS_ST_CRED_AT_FD_OFF (MAC_CRED_WPS_ST_CRED_ST_FD_OFF + \
                                        MAC_CRED_WPS_ST_CRED_ST_FD_LEN)
#define MAC_CRED_WPS_ST_CRED_AT_FD_LEN 1


#define MAC_CRED_WPS_ST_PROC_MD_NRM    0
#define MAC_CRED_WPS_ST_PROC_MD_WPS    1

#define MAC_CRED_WPS_ST_AUTH_MD_NO     0
#define MAC_CRED_WPS_ST_AUTH_MD_YES    (1 << MAC_CRED_WPS_ST_AUTH_MD_FD_OFF)

#define MAC_CRED_WPS_ST_NO_CRED        0
#define MAC_CRED_WPS_ST_VALID_CRED     (1 << MAC_CRED_WPS_ST_CRED_ST_FD_OFF)
#define MAC_CRED_WPS_ST_INVALID_CRED   (2 << MAC_CRED_WPS_ST_CRED_ST_FD_OFF)

#define MAC_CRED_WPS_ST_INACT_CRED     0
#define MAC_CRED_WPS_ST_ACT_CRED       (1 << MAC_CRED_WPS_ST_CRED_AT_FD_OFF)

#define MAX_NESTED_SCRATCH_MEM_ALLOC   4

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* WPS modes */
typedef enum {WPS_DISABLED = 0,
              WPS_ENABLED  = 1
} WPS_ENR_MODE_T;

/* WPS device modes */
typedef enum {WPS_STANDALONE_DEVICE   = 0,
              WPS_HOST_MANAGED_DEVICE = 1
} WPS_DEVICE_MODE_T;

/* WPS events */
typedef enum {WPS_TIMEOUT_EVENT     = 0x70,
              WPS_CRED_JOIN_TIMEOUT = 0x71
} EVENT_TYPESUBTYPE_WPS_T;

/* WPS Timeout types */
typedef enum {JOIN_WPS_CRED_LIST = 0x00,
              REJOIN_WPS_CRED    = 0x01
} WPS_TIMEOUT_TYPE_T;

/* WPS State Value */
typedef enum {WPS_NOT_CONFIGURED = 0x01,
              WPS_CONFIGURED     = 0x02
} WPS_STATE_VAL_T;

/* Security mode Cipher Types */
typedef enum {SEC_MODE_CT_AES      = 0x01,
              SEC_MODE_CT_TKIP     = 0x02,
              SEC_MODE_CT_AES_TKIP = 0x03
} SEC_MODE_CT_VAL_T;

/* Security mode Auth Types */
typedef enum {SEC_MODE_AUTH_WPA   = 0x01,
              SEC_MODE_AUTH_WPA2  = 0x02,
              SEC_MODE_AUTH_MIXED = 0x03
} SEC_MODE_AUTH_VAL_T;

/* Security mode Types */
typedef enum {SEC_MODE_NONE   = 0x00,
              SEC_MODE_WEP40  = 0x03,
              SEC_MODE_WEP104 = 0x07
} SEC_MODE_VAL_T;

/* Security mode Types */
typedef enum {CONFIG_OPEN_AUTH   = 0x01,
              CONFIG_SHARED_AUTH = 0x02,
              CONFIG_OTHER_AUTH  = 0x03
} CONFIG_AUTH_TYPE_T;

/* Usage of WPS Credentials */
typedef enum {WPS_CRED_NOT_USED         = 0x00,
              WPS_CRED_BEING_PROGRAMMED = 0x01,
              WPS_CRED_PROGRAMMED       = 0x02
} WPS_CRED_USAGE_STATUS_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* Internal parameters structure */
typedef struct
{
    UWORD8  wps_mode;
    UWORD8  wps_prot;
    UWORD16 wps_pass_id;
    UWORD16 wps_config_method;
    UWORD8  wps_pin[MAX_WPS_PIN_LEN+1];
} wps_internal_params_t;

#ifdef WPS_1_0_SEC_SUPP
/* Structure to save WPS credential keys - WEP or PSK */
typedef struct
{
    UWORD8 num_key;
    UWORD8 key_id[NUM_DOT11WEPDEFAULTKEYVALUE];
    UWORD8 key[NUM_DOT11WEPDEFAULTKEYVALUE][13];
} wps_wep_key_t;
#endif /* WPS_1_0_SEC_SUPP */

typedef union
{
#ifdef WPS_1_0_SEC_SUPP
    wps_wep_key_t wep_key;
#endif /* WPS_1_0_SEC_SUPP */
    UWORD8        psk[MAX_PSK_PASS_PHRASE_LEN];
} key_val_t;

/* Structure to save WPS credentials in EEPROM */
typedef struct
{
    UWORD8    auth_type;
    UWORD8    sec_mode;
    UWORD8    key_len;
    key_val_t key;
    UWORD8    ssid[MAX_SSID_LEN];
} wps_store_cred_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern wps_internal_params_t g_wps_params;
extern wps_enrollee_t *g_wps_enrollee_ptr;
extern wps_enr_config_struct_t *g_wps_config_ptr;
extern UWORD8  g_wps_dev_mode;
extern WPS_CRED_USAGE_STATUS_T  g_wps_use_creds;
extern UWORD8 g_wps_cred_list[MAX_WPS_CRED_LIST_SIZE];
extern ALARM_HANDLE_T *g_wps_cred_join_timer;
extern BOOL_T  g_config_write_from_host;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

/* WPS memory management functions */
extern STATUS_T allocate_wps_handle_mem(void);
extern void free_wps_handle_mem(void);

/* WPS API Functions (used by WPS Enrollee */
extern STATUS_T sys_start_wps(WPS_PROT_TYPE_T prot_type);
extern void end_wps_enrollee(WPS_STATUS_T status_code);
extern BOOL_T rec_wps_cred(wps_cred_t *cred, BOOL_T ver2_cap_ap);
#ifndef OS_LINUX_CSL_TYPE
extern void wps_timeout_fn(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void wps_timeout_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */
extern void start_wps_scan(BOOL_T deauth_1x_req);

/* WPS MAC SW Functions (Prot-IF) */
extern UWORD16 set_wsc_info_element(UWORD8 *data, UWORD16 index,
                                   UWORD8 frm_type);
extern void handle_mlme_rsp_wps_sta(UWORD8 rsp_type, UWORD8 *rsp_msg);
extern BOOL_T handle_wps_event_sta(UWORD32 event);
extern void process_wps_pkt_sta(UWORD8 *rx_data, UWORD16 rx_len);
extern void handle_wps_scan_rsp_sta(UWORD8 *mac, UWORD8 *msa,
                                    UWORD16 rx_len, WORD8 rssi);
extern void handle_wps_scan_complete_sta(UWORD8 *mac);
extern void start_wps_cred_rejoin_timer(void);

/* WPS Configuration Functions */
extern void send_wps_status(UWORD8 status_code, UWORD8* msg_ptr,
                            UWORD16 msg_len);
extern void set_wps_cred_list_enr(UWORD8 *cred_list);
extern UWORD8* get_wps_cred_list_enr(void);
extern void join_wps_cred_from_eeprom(void);
extern void save_cred_to_pers_mem(wps_store_cred_t* store_cred);
extern void init_wps_sta_globals(void);
extern void set_wps_dev_mode_enr(UWORD8 val);
extern BOOL_T wps_allow_sys_restart_sta(ERROR_CODE_T sys_error);
extern void handle_host_non_wps_cred_req(void);
#ifdef MAC_P2P
extern void prepare_wps_config(wps_enr_config_struct_t *config_ptr,
                               WPS_PROT_TYPE_T prot_type);
#endif /* MAC_P2P */

/*****************************************************************************/
/* Static Inline Functions                                                   */
/*****************************************************************************/

/* WPS mode related functions */

INLINE UWORD8 get_wps_mode(void)
{
    return g_wps_params.wps_mode;
}

INLINE void set_wps_mode(UWORD8 val)
{
    g_wps_params.wps_mode = val;
}

INLINE void disable_wps_mode(void)
{
    if(WPS_ENABLED == get_wps_mode())
    {
        wps_stop_enrollee(g_wps_enrollee_ptr);
        end_wps_enrollee(WLAN_DIS_WPS_PROT);
    }
}

/* WPS configuration functions */

INLINE UWORD8 get_wps_prot_enr(void)
{
    return g_wps_params.wps_prot;
}

INLINE BOOL_T set_wps_prot_enr(UWORD8 val)
{
    BOOL_T ret_val = BFALSE;
    g_config_write_from_host = BFALSE;
    disable_wps_mode();
    if(NULL != g_wps_cred_join_timer)
    {
        /* Delete the WPS credential join timer                              */
        delete_alarm(&g_wps_cred_join_timer);
        g_wps_cred_join_timer = NULL;
    }
    g_wps_use_creds = WPS_CRED_NOT_USED;

    g_wps_params.wps_prot = val;
    switch (val)
    {
    case WPS_PROT_NONE:
        break;
    default:
        /* Return BTRUE to indicate that WPS can be started if possible */
        ret_val = BTRUE;
        break;
    }

    return ret_val;
}

INLINE UWORD8 get_wps_dev_mode_enr(void)
{
    return g_wps_dev_mode;
}


#if 0 /* Not used */
/* This function retuns the current status of WPS protocol and               */
/* WPS Credntials Management.                                                */
INLINE UWORD8 get_mac_cred_wps_status_enr(void)
{
    UWORD8 mac_cred_wps_status = 0;
   /* If WPS Protocol is running then it means credentials are not present   */
   /* as they will be received at the end of WPS protocol also WPS credntial */
   /* management scheme is active.                                           */
    if(WPS_PROT_NONE != get_wps_prot_enr())
    {
        mac_cred_wps_status = (MAC_CRED_WPS_ST_PROC_MD_WPS |
            MAC_CRED_WPS_ST_AUTH_MD_YES | MAC_CRED_WPS_ST_NO_CRED |
            MAC_CRED_WPS_ST_INACT_CRED);
    }
    else
    {
        mac_cred_wps_status = MAC_CRED_WPS_ST_AUTH_MD_NO;
        if(((0 == g_wps_cred_list[WPS_CRED_LIST_LEN_OFF]) &&
            (0 == g_wps_cred_list[WPS_CRED_LIST_LEN_OFF+1])) ||
            (0 == g_wps_cred_list[WPS_CRED_LIST_CNT_OFF]))
        {
            mac_cred_wps_status |= MAC_CRED_WPS_ST_NO_CRED;
        }
        else
        {
            mac_cred_wps_status |= MAC_CRED_WPS_ST_VALID_CRED;
        }

        if(DISABLED == get_mac_state())
        {
            mac_cred_wps_status |= MAC_CRED_WPS_ST_PROC_MD_WPS |
                MAC_CRED_WPS_ST_INACT_CRED;
        }
        else
        {
            if(WPS_CRED_NOT_USED == g_wps_use_creds)
            {
                mac_cred_wps_status |= MAC_CRED_WPS_ST_PROC_MD_NRM |
                    MAC_CRED_WPS_ST_INACT_CRED;
            }
            else
            {
                mac_cred_wps_status |= MAC_CRED_WPS_ST_PROC_MD_WPS |
                    MAC_CRED_WPS_ST_ACT_CRED;
            }
        }
    }
    return(mac_cred_wps_status);
}
#endif /* 0 */

INLINE UWORD16 get_wps_pass_id_enr(void)
{
    return g_wps_params.wps_pass_id;
}

INLINE void set_wps_pass_id_enr(UWORD16 val)
{
    g_wps_params.wps_pass_id = val;
}

INLINE UWORD16 get_wps_config_method_enr(void)
{
    return g_wps_params.wps_config_method;
}

INLINE void set_wps_config_method_enr(UWORD16 val)
{
    g_wps_params.wps_config_method = val;
}

INLINE UWORD8 *get_wps_pin_enr(void)
{
    return g_wps_params.wps_pin;
}

INLINE void set_wps_pin_enr(UWORD8 *val)
{
    UWORD8 len = strlen((const char *)val);
    len = (len > MAX_WPS_PIN_LEN) ? MAX_WPS_PIN_LEN : len;
    memcpy(&g_wps_params.wps_pin[1], val, len);
    g_wps_params.wps_pin[0] = len;
}

INLINE BOOL_T is_dev_mode_host_managed_enr(void)
{
    if(get_wps_dev_mode_enr() == WPS_HOST_MANAGED_DEVICE)
    {
        return BTRUE;
    }

    return BFALSE;
}

/* Set the config write flag */
INLINE void wps_set_config_write_sta(BOOL_T val)
{
    g_config_write_from_host = val;
}

#endif /* WPS_STA_H */
#endif /* IBSS_BSS_STATION_MODE */
#endif /* INT_WPS_SUPP */

