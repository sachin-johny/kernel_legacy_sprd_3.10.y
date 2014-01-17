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

#ifndef WPS_CRED_MGMT_AP_H
#define WPS_CRED_MGMT_AP_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_reg_if.h"
#include "common.h"
#include "management_ap.h"
#include "index_util.h"
#include "management.h"
#include "maccontroller.h"
#include "csl_if.h"
#include "wep.h"
#include "ap_management_wps.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/
#define MAX_AP_NW_CRED_SUPP     1
#define WPS_VALID_CRED          0xAA
#define WPS_CRED_VALID_FLAG_OFFSET 0
#define WPS_CRED_OFFSET         (WPS_CRED_VALID_FLAG_OFFSET + 1)

#define AUTH_1X                 1
#define AUTH_PSK                0
#define SEC_MODE_MASK           0x7F
#define SEC_MODE_NONE           0x00
#define SEC_MODE_WEP40          0x03
#define SEC_MODE_WEP104         0x07
// 20120709 caisf mod, merged ittiam mac v1.2 code
#define OOB_RESET_SEC_MODE      0x31// 0x39  /* WPA/WPA2-AES */
#define OOB_RESET_AUTH_TYPE     0

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
typedef enum
{
	AUTH_OPEN   = 0x1,
	AUTH_SHARED = 0x2,
	AUTH_ANY    = 0x3
} WPS_AUTH_T;

typedef enum
{
	ENCR_AES      = 0x1,
	ENCR_TKIP     = 0x2,
	ENCR_AES_TKIP = 0x3
} WPS_ENCR_T;

typedef enum
{
	TYPE_11I_WPA   = 0x1,
	TYPE_11I_WPA2  = 0x2,
	TYPE_11I_MIXED = 0x3
} WPS_11I_TYPE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/
/* Structure to save WPS credential keys - WEP or PSK */
typedef struct
{
    UWORD8 key_id;
    UWORD8 key[WEP104_KEY_SIZE];
} wps_wep_key_t;

typedef union
{
    wps_wep_key_t wep_key;
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
    UWORD8    bssid[6];
} wps_store_cred_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern BOOL_T g_cred_updated_ap;
extern BOOL_T g_wps_wid_restore_in_prog;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern BOOL_T wps_get_cur_cred_ap(wps_cred_t *wps_cred, UWORD8 cred_index);
extern UWORD8 wps_get_num_cred_ap(void);
extern void wps_handle_cred_update_ap(void);
extern BOOL_T wps_process_wid_write_ap(UWORD8 *req, UWORD16 rx_len);
extern void wps_handle_oob_reset_req_ap(void);
extern void wps_update_device_info_ap(void);
extern void wps_apply_cred_ap(wps_store_cred_t *cred);
extern STATUS_T wps_get_cred_eeprom_ap(wps_store_cred_t *store_cred);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function sets g_cred_updated_ap variable with specified input value  */
INLINE void set_cred_updated_ap(BOOL_T val)
{
	if(BFALSE == g_wps_wid_restore_in_prog)
		g_cred_updated_ap = val;
}

/* This function returns the value of g_cred_updated_ap variable             */
INLINE BOOL_T get_cred_updated_ap(void)
{
	return g_cred_updated_ap;
}

/* Update the flag that indicates the status of WID restoration */
INLINE void indicate_wid_restore_wps_ap(BOOL_T val)
{
	if(1 < (UWORD8)val)
		return;

	g_wps_wid_restore_in_prog = val;
}
#endif /* WPS_CRED_MGMT_AP_H */
#endif /* INT_WPS_SUPP */
#endif /* BSS_ACCESS_POINT_MODE */
