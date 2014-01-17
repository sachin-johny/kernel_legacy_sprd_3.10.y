/*****************************************************************************/
/*                                                                           */
/*                     Ittiam WPS Supplicant SOFTWARE                        */
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
/*  File Name         : wpse_prot.h                                          */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      implementing the WPS registration protocol           */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         01 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_ENR_SUPP

#ifndef WPS_PROT_H
#define WPS_PROT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_enr_if.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/
#define WPS_MSG_RETR_TIME               10000
#define MAX_EAP_MSG_RETRANS_LIMIT       3
#define WPS_REG_EAP_MSG_TIME            15000
#define WPS_REG_EAP_FAIL_MSG_TIME       1000
#define WPS_EAP_REG_TIME                120000

#define WPS_WALK_TIME                   120000
#define WPS_IE_DATA_OFFSET              6
#define PBC_SCN_CNT_SESSION_OVERLAP     2
#define WPS_MAX_CRED_CNT                8

//#define CHECK_DEV_PASS_ID_M2

/*****************************************************************************/
/* IGNORE_SEL_REG_FIELD_THESH_CNT indicates that after how many scans, WPS   */
/* should start ignoring the Selected Registrar field and should try to do   */
/* WPS protocol with any WPS capable AP/Registrar. This should be grater than*/
/* PBC_SCN_CNT_SESSION_OVERLAP                                               */
/*****************************************************************************/
#define IGNORE_SEL_REG_FIELD_THESH_CNT  4

#define WPS_RES_TYPE_ENR_INFO           0x00
#define WPS_RES_TYPE_ENR_1X             0x01
#define WPS_RES_TYPE_REG                0x02
#define WPS_RES_TYPE_AP                 0x03

#define WPS_REQ_TYPE_ENR_INFO           0x00
#define WPS_REQ_TYPE_ENR_1X             0x01
#define WPS_REQ_TYPE_REG                0x02
#define WPS_REQ_TYPE_WLAN_MAN_REG       0x03

#define MAC_HDR_LEN                     24
#define TIME_STAMP_LEN                  8
#define BEACON_INTERVAL_LEN             2
#define CAP_INFO_LEN                    2
#define FCS_LEN                         4
#define DISABLED                        0
#define ENABLED                         1


#define WPS_AES_BYTE_BLOCK_SIZE         16

#define EAPOL_VERSION_01                0x01
#define EAPOL_VERSION_02                0x02
#define ONE_X_PCK_TYPE_EAP              0x00
#define ONE_X_PCK_TYPE_EAPOL_START      0x01
#define ONE_X_PCK_TYPE_EAPOL_LOGOFF     0x02
#define ONE_X_PCK_TYPE_EAPOL_KEY        0x03
#define ONE_X_PCK_TYPE_EAPOL_EASF_AL    0x04

#define EAP_CODE_REQUEST                0x01
#define EAP_CODE_RESPONSE               0x02
#define EAP_CODE_SUCCESS                0x03
#define EAP_CODE_FAILURE                0x04
#define EAP_TYPE_IDENTITY               0x01
#define EAP_IDENTITY_LEN                29

#define WPS_MAX_MAC_HDR_LEN             26
#define WPS_MAC_HDR_LEN                 24
#define ONE_X_SNAP_ETH_TYPE_OFFSET      0
#define ONE_X_SNAP_ETH_TYPE_LEN         8
#define ONE_X_PROT_VER_OFFSET           (ONE_X_SNAP_ETH_TYPE_OFFSET+ \
                                                ONE_X_SNAP_ETH_TYPE_LEN)
#define ONE_X_PCK_TYPE_OFFSET           (ONE_X_PROT_VER_OFFSET+1)
#define ONE_X_PCK_LEN_OFFSET            (ONE_X_PCK_TYPE_OFFSET+1)
#define ONE_X_PCK_BODY_OFFSET           (ONE_X_PCK_LEN_OFFSET+2)
#define ONE_X_HEADER_LEN                ONE_X_PCK_BODY_OFFSET

#define WPS_EAPOL_1X_HDR_LEN            ONE_X_PCK_BODY_OFFSET

#define EAP_HEADER_OFFSET               (ONE_X_PCK_BODY_OFFSET)
#define EAP_HEADER_LEN                  4
#define EAP_CODE_OFFSET                 EAP_HEADER_OFFSET
#define EAP_IDENTIFIER_OFFSET           (EAP_CODE_OFFSET+1)
#define EAP_LENGTH_OFFSET               (EAP_IDENTIFIER_OFFSET+1)
#define EAP_TYPE_OFFSET                 (EAP_LENGTH_OFFSET+2)
#define EAP_WPS_VENDOR_ID_OFFSET        (EAP_TYPE_OFFSET+1)
#define EAP_WPS_VENDOR_TYPE_OFFSET      (EAP_WPS_VENDOR_ID_OFFSET+3)
#define EAP_WPS_OP_CODE_OFFSET          (EAP_WPS_VENDOR_TYPE_OFFSET+4)
#define EAP_WPS_FLAGS_OFFSET            (EAP_WPS_OP_CODE_OFFSET+1)
#define EAP_WPS_MSG_LEN_OFFSET          (EAP_WPS_FLAGS_OFFSET+1)
#define EAP_WPS_DATA_WI_LEN_OFFSET      (EAP_WPS_MSG_LEN_OFFSET+2)
#define EAP_WPS_DATA_WO_LEN_OFFSET      EAP_WPS_MSG_LEN_OFFSET

#define EAP_WPS_TYPE_VID_VT_LEN         8
#define WPS_CONN_TYPE_FLAGS_MASK        0x03

#define WPS_TLV_ATTR_OFF_SET            0
#define WPS_TLV_LEN_OFF_SET             WPS_TLV_ATTR_OFF_SET+2
#define WPS_TLV_DATA_OFF_SET            WPS_TLV_LEN_OFF_SET+2
#define WPS_TLV_MIN_LEN                 WPS_TLV_DATA_OFF_SET

#define WPS_PRIM_DEV_TYPE_LEN           8

#ifdef MAC_P2P
#define WPS_REQ_DEV_TYPE_LEN            8
#endif /* MAC_P2P */

#define WPS_OS_VERSION_LEN              4
#define WPS_AUTHENTICATOR_LEN           8
#define WPS_KEY_WRAP_AUTH_LEN           8


#define WPS_EAP_RED_ID_LEN              29


#define WPS_IBSS_MODE                   0
#define MAX_WPS_WLAN_FRAME_LEN          1500
#define MAX_WPS_WSC_EAP_MSG_LEN         (MAX_WPS_WLAN_FRAME_LEN - \
                                        WPS_MAX_MAC_HDR_LEN - 4)
#define WPS_EAPOL_START_PCK_LEN         (WPS_MAX_MAC_HDR_LEN+ \
                                        WPS_EAPOL_1X_HDR_LEN+4)
#define WPS_EAP_RES_ID_MSG_LEN          (WPS_MAX_MAC_HDR_LEN+ EAP_TYPE_OFFSET\
                                        + 1 + WPS_EAP_RED_ID_LEN + 4)

#define WPS_EAP_WSC_ACK_MSG_LEN         (WPS_MAX_MAC_HDR_LEN + \
                                        EAP_WPS_DATA_WI_LEN_OFFSET + \
                                        + 100)
#define WPS_EAP_WSC_FRAG_ACK_MSG_LEN    (WPS_MAX_MAC_HDR_LEN + \
                                        EAP_WPS_DATA_WI_LEN_OFFSET + \
                                        + 4)
#define WPS_EAP_WSC_NACK_MSG_LEN        (WPS_MAX_MAC_HDR_LEN + \
                                        EAP_WPS_DATA_WI_LEN_OFFSET + \
                                        + 100)

#define WFA_VEN_EXTN_ID_BYTE0           0x00
#define WFA_VEN_EXTN_ID_BYTE1           0x37
#define WFA_VEN_EXTN_ID_BYTE2           0x2A
#define WPS_MIN_VEN_EXTN_LEN            3  /* 3 bytes Vendor ID */

#define TIMESTAMP_LEN                   8
#define BCN_INT_LEN                     2
#define CAP_INFO_LEN                    2
#define STATUS_CODE_LEN                 2
#define ASSOC_ID_LEN                    2

#ifdef MAC_P2P
#define P2P_PUB_ACT_TAG_PARAM_OFF       8
#define P2P_GEN_ACT_TAG_PARAM_OFF       7
#endif /* MAC_P2P */

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/
#define GET_U16_BE(buff) (UWORD16)(((UWORD16)(buff)[0]<<8)+(UWORD16)(buff)[1])

#define PUT_U16_BE(buff, val) (buff)[0] = (UWORD8)(((UWORD16)(val) >> 8)&0xFF);\
                              (buff)[1] = (UWORD8)((UWORD16)(val)&0xFF);

#define PUT_U32_BE(buff, val) (buff)[0] = (UWORD8)(((UWORD32)(val) >> 24)&0xFF);\
                              (buff)[1] = (UWORD8)(((UWORD32)(val) >> 16)&0xFF);\
                              (buff)[2] = (UWORD8)(((UWORD32)(val) >> 8)&0xFF);\
                              (buff)[3] = (UWORD8)((UWORD32)(val)&0xFF);

/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/
/*****************************************************************************/
/* Frame Type and Subtype Codes (6-bit)                                      */
/*****************************************************************************/
typedef enum {ASSOC_REQ             = 0x00,
              ASSOC_RSP             = 0x10,
              REASSOC_REQ           = 0x20,
              REASSOC_RSP           = 0x30,
              PROBE_REQ             = 0x40,
              PROBE_RSP             = 0x50,
              BEACON                = 0x80,
              ATIM                  = 0x90,
              DISASOC               = 0xA0,
              AUTH                  = 0xB0,
              DEAUTH                = 0xC0,
              ACTION                = 0xD0,
              PS_POLL               = 0xA4,
              RTS                   = 0xB4,
              CTS                   = 0xC4,
              ACK                   = 0xD4,
              CFEND                 = 0xE4,
              CFEND_ACK             = 0xF4,
              DATA                  = 0x08,
              DATA_ACK              = 0x18,
              DATA_POLL             = 0x28,
              DATA_POLL_ACK         = 0x38,
              NULL_FRAME            = 0x48,
              CFACK                 = 0x58,
              CFPOLL                = 0x68,
              CFPOLL_ACK            = 0x78
} TYPESUBTYPE_T;

#ifdef MAC_P2P
/* P2P Public Action Frames */
typedef enum {
    P2P_GO_NEG_REQ    = 0,
    P2P_GO_NEG_RSP    = 1,
    P2P_GO_NEG_CNF    = 2,
    P2P_INV_REQ       = 3,
    P2P_INV_RSP       = 4,
    P2P_DEV_DISC_REQ  = 5,
    P2P_DEV_DISC_RSP  = 6,
    P2P_PROV_DISC_REQ = 7,
    P2P_PROV_DISC_RSP = 8
} P2P_PUB_ACTION_FRM_TYPE;
#endif /* MAC_P2P */


/*****************************************************************************/
/* Different types of WPS timeouts                                           */
/*****************************************************************************/
typedef enum
{

    WPS_WALK_TIMEOUT = 0,
    WPS_REG_TIMEOUT = 1,
    WPS_REG_MSG_TIMEOUT = 2
} WPS_TIMEOUT_TYPE_T;

/*****************************************************************************/
/* Different BSS types                                                       */
/*****************************************************************************/
typedef enum
{
    INFRASTRUCTURE  = 1,
    INDEPENDENT     = 2,
    ANY_BSS         = 3
} BSSTYPE_T;

/*****************************************************************************/
/* Capability Information field bit assignments                              */
/*****************************************************************************/
typedef enum {ESS           = 0x01,   /* ESS capability               */
              IBSS          = 0x02,   /* IBSS mode                    */
              POLLABLE      = 0x04,   /* CF Pollable                  */
              POLLREQ       = 0x08,   /* Request to be polled         */
              PRIVACY       = 0x10,   /* WEP encryption supported     */
              SHORTPREAMBLE  = 0x20,   /* Short Preamble is supported  */
              SHORTSLOT      = 0x400,  /* Short Slot is supported      */
              PBCC          = 0x40,   /* PBCC                         */
              CHANNELAGILITY = 0x80,   /* Channel Agility              */
              SPECTRUM_MGMT = 0x100,  /* Spectrum Management          */
              DSSS_OFDM      = 0x2000  /* DSSS-OFDM                    */
} CAPABILITY_T;

/*****************************************************************************/
/* Different Op-Codes of WPS-EAP message                                     */
/*****************************************************************************/
typedef enum
{
    WPS_OP_CODE_WSC_START       = 0x01,
    WPS_OP_CODE_WSC_ACK         = 0x02,
    WPS_OP_CODE_WSC_NACK        = 0x03,
    WPS_OP_CODE_WSC_MSG         = 0x04,
    WPS_OP_CODE_WSC_DONE        = 0x05,
    WPS_OP_CODE_WSC_FRAG_ACK    = 0x06
} WPS_EAP_OPCODE_T;
/*****************************************************************************/
/* Different fields in WPS-EAP message flags field                           */
/*****************************************************************************/
typedef enum
{

    WPS_EAP_MSG_FLAG_MF = 1,
    WPS_EAP_MSG_FLAG_LF = 2
} WPS_EAP_FLAG_T;

/*****************************************************************************/
/* WPS Registration Protocol Message types                                   */
/*****************************************************************************/
typedef enum
{
    WPS_MSG_M1       = 0x04,
    WPS_MSG_M2       = 0x05,
    WPS_MSG_M2D      = 0x06,
    WPS_MSG_M3       = 0x07,
    WPS_MSG_M4       = 0x08,
    WPS_MSG_M5       = 0x09,
    WPS_MSG_M6       = 0x0A,
    WPS_MSG_M7       = 0x0B,
    WPS_MSG_M8       = 0x0C,
    WPS_MSG_WSC_ACK  = 0x0D,
    WPS_MSG_WSC_NACK = 0x0E,
    WPS_MSG_WSC_DONE = 0x0F
} WPS_MSG_TYPE_T;

/*****************************************************************************/
/* WPS Registration Protocol Message types                                   */
/*****************************************************************************/
typedef enum
{
    ERR_INVALID_ENCR_DATA_LEN   = -1,
    ERR_INVALID_PAD             = -2
} WPS_DECRY_FAIL_ERR_CODE_T;

/*****************************************************************************/
/* WPS Attribute IDs                                                         */
/*****************************************************************************/
typedef enum
{
    WPS_ATTR_AP_CHANNEL =           0x1001,
    WPS_ATTR_ASSOC_STATE =          0x1002,
    WPS_ATTR_AUTH_TYPE =            0x1003,
    WPS_ATTR_AUTH_TYPE_FLAGS =      0x1004,
    WPS_ATTR_AUTHENTICATOR =        0x1005,
    WPS_ATTR_CONFIG_METHODS =       0x1008,
    WPS_ATTR_CONFIG_ERROR =         0x1009,
    WPS_ATTR_CONFIRM_URL4 =         0x100A,
    WPS_ATTR_CONFIRM_URL6 =         0x100B,
    WPS_ATTR_CONN_TYPE =            0x100C,
    WPS_ATTR_CONN_TYPE_FLAGS =      0x100D,
    WPS_ATTR_CREDENTIAL =           0x100E,
    WPS_ATTR_DEV_NAME =             0x1011,
    WPS_ATTR_DEV_PASS_ID =          0x1012,
    WPS_ATTR_E_HASH1 =              0x1014,
    WPS_ATTR_E_HASH2 =              0x1015,
    WPS_ATTR_E_SNONCE1 =            0x1016,
    WPS_ATTR_E_SNONCE2 =            0x1017,
    WPS_ATTR_ENCRY_SETTINGS =       0x1018,
    WPS_ATTR_ENCRY_TYPE =           0x100F,
    WPS_ATTR_ENCRY_TYPE_FLAGS =     0x1010,
    WPS_ATTR_ENR_NONCE =            0x101A,
    WPS_ATTR_FEATURE_ID =           0x101B,
    WPS_ATTR_IDENTITY =             0x101C,
    WPS_ATTR_IDENTITY_PROOF =       0x101D,
    WPS_ATTR_KEY_WRAP_AUTH =        0x101E,
    WPS_ATTR_KEY_ID =               0x101F,
    WPS_ATTR_MAC_ADDR =             0x1020,
    WPS_ATTR_MANUFACTURER =         0x1021,
    WPS_ATTR_MEG_TYPE =             0x1022,
    WPS_ATTR_MODEL_NAME =           0x1023,
    WPS_ATTR_MODEL_NUM =            0x1024,
    WPS_ATTR_NET_INDEX =            0x1026,
    WPS_ATTR_NET_KEY =              0x1027,
    WPS_ATTR_NET_KEY_INDEX =        0x1028,
    WPS_ATTR_NEW_DEVICE_NAME =      0x1029,
    WPS_ATTR_NEW_PASSWORD =         0x102A,
    WPS_ATTR_OOB_DEV_PASS =         0x102C,
    WPS_ATTR_OS_VERSION =           0x102D,
    WPS_ATTR_POWER_LEVEL =          0x102F,
    WPS_ATTR_PSK_CURRENT =          0x1030,
    WPS_ATTR_PSK_MAX =              0x1031,
    WPS_ATTR_PUBLIC_KEY =           0x1032,
    WPS_ATTR_RADIO_ENABLED =        0x1033,
    WPS_ATTR_REBOOT =               0x1034,
    WPS_ATTR_REG_CURRENT =          0x1035,
    WPS_ATTR_REG_ESTABLISHED =      0x1036,
    WPS_ATTR_REG_LIST =             0x1037,
    WPS_ATTR_REG_MAX =              0x1038,
    WPS_ATTR_REG_NONCE =            0x1039,
    WPS_ATTR_REQUEST_TYPE =         0x103A,
    WPS_ATTR_RESPONSE_TYPE =        0x103B,
    WPS_ATTR_RF_BANDS =             0x103C,
    WPS_ATTR_R_HASH1 =              0x103D,
    WPS_ATTR_R_HASH2 =              0x103E,
    WPS_ATTR_R_SNONCE1 =            0x103F,
    WPS_ATTR_R_SNONCE2 =            0x1040,
    WPS_ATTR_SEL_REG =              0x1041,
    WPS_ATTR_SERIAL_NUM =           0x1042,
    WPS_ATTR_WPS_STATE =            0x1044,
    WPS_ATTR_SSID =                 0x1045,
    WPS_ATTR_TOTAL_NET =            0x1046,
    WPS_ATTR_UUID_E =               0x1047,
    WPS_ATTR_UUID_R =               0x1048,
    WPS_ATTR_VENDOR_EXTENSION =     0x1049,
    WPS_ATTR_VERSION =              0x104A,
    WPS_ATTR_X_509_CERT_REQ =       0x104B,
    WPS_ATTR_X_509_CERT =           0x104C,
    WPS_ATTR_EAP_IDENTITY =         0x104D,
    WPS_ATTR_MSG_COUNTER =          0x104E,
    WPS_ATTR_PUBLIC_KEY_HASH =      0x104F,
    WPS_ATTR_REKEY_KEY =            0x1050,
    WPS_ATTR_KEY_LIFETIME =         0x1051,
    WPS_ATTR_PERM_CONFIG_METH =     0x1052,
    WPS_ATTR_SEL_REG_CONFIG_METH =  0x1053,
    WPS_ATTR_PRIM_DEV_TYPE =        0x1054,
    WPS_ATTR_SEC_DEV_TYPE_LIST =    0x1055,
    WPS_ATTR_PORT_DEV =             0x1056,
    WPS_ATTR_AP_SETUP_LOCKED =      0x1057,
    WPS_ATTR_APPL_EXTN =            0x1058,
    WPS_ATTR_EAP_TYPE =             0x1059,
    WPS_ATTR_INIT_VECTOR =          0x1060,
    WPS_ATTR_KEY_PROV_AUTO =        0x1061,
    WPS_ATTR_802_1X_ENABLED =       0x1062,
    WPS_ATTR_APPSESSIONKEY =        0x1063,
    WPS_ATTR_WEPTRANSMITKEY =       0x1064,
    WPS_ATTR_SETT_DEL_TIME =        0x1065,
    WPS_ATTR_VENDOR_URL =           0x1066,
#ifdef MAC_P2P
    WPS_ATTR_REQ_DEV_TYPE =         0x106A
#endif /* MAC_P2P */
} WPS_TLV_ATTR_ID_T;

/*****************************************************************************/
/* Attributes parsed by WPS enrollee                                         */
/*****************************************************************************/
typedef struct
{
    UWORD8* version;
    UWORD8* msg_type;
    UWORD8* enr_nonce;
    UWORD8* reg_nonce;
    UWORD8* uuid_r;
    UWORD8* pkr;
    UWORD8* auth_type_flags;
    UWORD8* encr_type_flags;
    UWORD8* conn_type_flags;
    UWORD8* config_meth;
    UWORD8* prim_dev_type;
    UWORD8* rf_bands;
    UWORD8* assoc_state;
    UWORD8* config_error;
    UWORD8* dev_pass_id;
    UWORD8* os_version;
    UWORD8* authenticator;
    UWORD8* r_hash1;
    UWORD8* r_hash2;
    UWORD8* wps_state;
    UWORD8* sel_reg;
    UWORD8* sel_reg_config_meth;
    UWORD8* res_type;
    UWORD8* r_snonce1;
    UWORD8* r_snonce2;
    UWORD8* key_wrap_authen;
    UWORD8* encrypted_settings;
    UWORD8* manufacturer;
    UWORD8* model_name;
    UWORD8* model_number;
    UWORD8* serial_number;
    UWORD8* device_name;
    UWORD8* credential[WPS_MAX_CRED_CNT];
    UWORD16 credential_len[WPS_MAX_CRED_CNT];
    UWORD8  cred_cnt;
    UWORD16 encrypted_settings_len;
    UWORD8  manufacturer_len;
    UWORD8  model_name_len;
    UWORD8  model_number_len;
    UWORD8  serial_number_len;
    UWORD8  device_name_len;
    UWORD8 *wfa_vendor;
    UWORD8 *version2;
    UWORD8 auth_macs_cnt;
    UWORD8 *auth_macs[MAX_AUTH_MACS];
#ifdef MAC_P2P
    UWORD8 *req_dev_type;
#endif /* MAC_P2P */
} wps_rx_attri_t;


/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void wps_handle_sys_err(wps_enrollee_t* wps_enrollee);
extern void wps_loc_start_scan(wps_enrollee_t* wps_enrollee);
extern void wps_walk_timeout(wps_enrollee_t* wps_enrollee);
extern void wps_delete_timers(wps_enrollee_t* wps_enrollee);
extern void wps_free_all_memory(wps_enrollee_t* wps_enrollee);
extern void wps_add_discarded_reg_ap(wps_enrollee_t* wps_enrollee,
    UWORD8* bssid);
extern void wps_eap_reg_start(wps_enrollee_t* wps_enrollee);
extern UWORD8 wps_parse_attri(UWORD8 *msg_ptr, wps_rx_attri_t** parse_attr,
                              UWORD16 msg_len, UWORD8 max_element_cnt,
                              BOOL_T is_wsc_ie);
extern STATUS_T wps_parse_creden(wps_enrollee_t* wps_enrollee, UWORD8** cred,
                          UWORD16* cred_len, UWORD8 cred_cnt);
extern void wps_get_rand_byte_array(UWORD8* inp_ptr, UWORD16 arr_len);
/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/
/*****************************************************************************/
/* This function extracts the SSID from a beacon/probe response frame        */
/*****************************************************************************/
INLINE UWORD8 get_ssid(UWORD8* data, UWORD8* ssid)
{
    UWORD8 ssid_len;
    UWORD8 ssid_idx;
    UWORD8 msg_idx;

    ssid_len = data[MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN +
        CAP_INFO_LEN + 1];
    msg_idx   = MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN +
        CAP_INFO_LEN + 2;

    for(ssid_idx = 0; ssid_idx < ssid_len; ssid_idx++, msg_idx++)
        ssid[ssid_idx] = data[msg_idx];
    return(ssid_len);
}

/*****************************************************************************/
/* This function extracts the 'from ds' bit from the MAC header of the input */
/* frame.                                                                    */
/* Returns the value in the LSB of the returned value.                       */
/*****************************************************************************/
INLINE UWORD8 get_from_ds(UWORD8* header)
{
    return ((header[1] & 0x02) >> 1);
}

/*****************************************************************************/
/* This function extracts the MAC Address in 'address2' field of the MAC     */
/* header and updates the MAC Address in the allocated 'addr' variable.      */
/*****************************************************************************/
INLINE void get_address2(UWORD8* msa, UWORD8* addr)
{
    memcpy(addr, msa + 10, 6);
}


/*****************************************************************************/
/* This function extracts the MAC Address in 'address3' field of the MAC     */
/* header and updates the MAC Address in the allocated 'addr' variable.      */
/*****************************************************************************/
INLINE void get_address3(UWORD8* msa, UWORD8* addr)
{
    memcpy(addr, msa + 16, 6);
}

/*****************************************************************************/
/* This function extracts the BSSID from the incoming WLAN packet based on   */
/* the 'from ds' bit, and updates the MAC Address in the allocated 'addr'    */
/* variable.                                                                 */
/*****************************************************************************/
INLINE void get_BSSID(UWORD8* data, UWORD8* bssid)
{
    if(get_from_ds(data) == 1)
        get_address2(data, bssid);
    else
        get_address3(data, bssid);
}


/*****************************************************************************/
/* This function determines the BSS type based on the capability information */
/* value                                                                     */
/*****************************************************************************/
INLINE BSSTYPE_T get_bss_type(UWORD16 cap_info)
{
    if((cap_info & ESS) != 0)
        return INFRASTRUCTURE;

    if((cap_info & IBSS) != 0)
        return INDEPENDENT;

    return ANY_BSS;
}

/*****************************************************************************/
/* This function extracts the 'frame type and sub type' bits from the MAC    */
/* header of the input frame.                                                */
/* Returns the value in the LSB of the returned value.                       */
/*****************************************************************************/
INLINE TYPESUBTYPE_T get_sub_type(UWORD8* header)
{
    return ((TYPESUBTYPE_T)(header[0] & 0xFC));
}

/*****************************************************************************/
/* This function extracts the capability info field from the beacon or probe */
/* response frame.                                                           */
/*****************************************************************************/
INLINE UWORD16 get_cap_info(UWORD8* data)
{
    UWORD16 cap_info = 0;

    cap_info  = data[MAC_HDR_LEN + TIME_STAMP_LEN + BEACON_INTERVAL_LEN];
    cap_info |= (UWORD16)((UWORD16)data[MAC_HDR_LEN + TIME_STAMP_LEN +
        BEACON_INTERVAL_LEN + 1] << 8);
    return cap_info;
}

/*****************************************************************************/
/* This always returns 1 as part of WPS Protocol Extensiblity, not version   */
/* checking should be done on received frame                                 */
/*****************************************************************************/
INLINE UWORD8 wps_check_ver2_supp(UWORD8* version2)
{
    if(version2 == NULL)
        return 0;

    return 1;
}

/*****************************************************************************/
/* This function checks the Selected Registrar field of the received message */
/*****************************************************************************/
INLINE UWORD8 wps_check_sel_reg(UWORD8* sel_reg)
{
    return (sel_reg && (0 != *sel_reg));
}

/*****************************************************************************/
/* This function checks the Response Type field of the received message      */
/*****************************************************************************/
INLINE UWORD8 wps_check_res_type(UWORD8* res_type)
{
    return (res_type && ((WPS_RES_TYPE_REG == *res_type) ||
        (WPS_RES_TYPE_AP == *res_type)));
}

/*****************************************************************************/
/* This function checks if the Enrollee is allowed to by parsing the         */
/* AuthorizedMACs list                                                       */
/*****************************************************************************/
INLINE BOOL_T wps_check_auth_macs_list(UWORD8 **auth_macs,
                                       UWORD8 auth_macs_cnt,
                                       UWORD8 *enr_mac_addr)
{
    UWORD8 cnt = 0;
    UWORD8 wild_card_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    /* This is a workaround for Ralink WPS2.0 Test bed AP. The AP does not   */
    /* includes AuthorizedMACs subelement in beacons/probe-response frames   */
    /* in PBC mode of WPS                                                    */
    if(0 == auth_macs_cnt)
        return BTRUE;

    for(cnt = 0; cnt < auth_macs_cnt; cnt++)
    {
        /* If Wild Card MAC address then Enrollee can run WPS with the AP */
        if((0 == memcmp(auth_macs[cnt], wild_card_addr, WPS_MAC_ADDR_LEN)) ||
           (0 == memcmp(auth_macs[cnt], enr_mac_addr, WPS_MAC_ADDR_LEN)))
            return BTRUE;
    }

    return BFALSE;
}

/*****************************************************************************/
/* This functions parses the received WFA Vendor Extension Attribute for Sub */
/* Elements                                                                  */
/*****************************************************************************/
INLINE void wps_parse_wsc_subelem_enr(wps_rx_attri_t *rx_attr,
                                      UWORD16 attr_len)
{
    UWORD16 index = 0;
    UWORD16 auth_macs_indx = 0;
    UWORD8 *wfa_vendor_attr = rx_attr->wfa_vendor;

    if(attr_len == 0)
        return;

    while(attr_len > index)
    {
        UWORD8 subelem_id = wfa_vendor_attr[index++];
        UWORD8 subelem_len = wfa_vendor_attr[index++];
        UWORD8 *subelem_val = (wfa_vendor_attr + index);

        switch(subelem_id)
        {
            case WPS_SUB_ELEM_VER2:
            {
                if(1 != subelem_len)
                    break;

                rx_attr->version2 = subelem_val;
            }
            break;
            case WPS_SUB_ELEM_AUTH_MACS:
            {
                if(WPS_MAC_ADDR_LEN > subelem_len)
                    break;

                while((auth_macs_indx < subelem_len) &&
                      (MAX_AUTH_MACS > rx_attr->auth_macs_cnt))
                {
                    rx_attr->auth_macs[rx_attr->auth_macs_cnt] = (subelem_val +
                                                                  auth_macs_indx);
                    rx_attr->auth_macs_cnt++;
                    auth_macs_indx += WPS_MAC_ADDR_LEN;
                }
            }
            break;
            default:
                /* Do nothing and ignore the subelement */
            break;
        }

        index += subelem_len;
    }
}

#endif /* WPS_PROT_H */
#endif /* INT_WPS_ENR_SUPP */
