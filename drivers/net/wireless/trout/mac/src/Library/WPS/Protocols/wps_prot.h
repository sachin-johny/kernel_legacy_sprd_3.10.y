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
/*  File Name         : wps_prot.h                                           */
/*                                                                           */
/*  Description       : This file contains all the common WPS and related    */
/*                      protocol definitions for WPS library                 */
/*                                                                           */
/*  List of Functions : None                                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/

#ifndef WPS_PROT_H
#define WPS_PROT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpsr_reg_if.h"

/*****************************************************************************/
/* Constants Macros                                                          */
/*****************************************************************************/
#define FCS_LEN                 4
#define WPS_IE_ID               221
#define WPS_IE_OUI_BYTE0        0x00
#define WPS_IE_OUI_BYTE1        0x50
#define WPS_IE_OUI_BYTE2        0xF2
#define WPS_IE_OUI_BYTE3        0x04
#define WPS_MAX_MANUFACT_LEN    64
#define WPS_MAX_MODEL_NAME_LEN  32
#define WPS_MAX_MODEL_NUM_LEN   32
#define WPS_MAX_SERIAL_NUM_LEN  32
#define WPS_MAX_DEVICE_NAME_LEN 32

#define WPS_MAX_RX_EAP_MSG_LEN  1400
#define WPS_MAX_TX_EAP_MSG_LEN  1000
#define WPS_MAC_ADDR_LEN        6

#define WPS_AUTH_TYPE_FLAGS_MASK    0x7F
#define WPS_ENCR_TYPE_FLAGS_MASK    0x1F

/* Constants Related to TLV Attribute Parsing */
#define WPS_IE_DATA_OFFSET       6
#define WPS_TLV_ATTR_OFF_SET     0
#define WPS_TLV_LEN_OFF_SET      WPS_TLV_ATTR_OFF_SET+2
#define WPS_TLV_DATA_OFF_SET     WPS_TLV_LEN_OFF_SET+2
#define WPS_TLV_MIN_LEN          WPS_TLV_DATA_OFF_SET

/* Constants related to 1X and EAP packet Types */

#define WPS_EAP_VERSION_1               0x01
#define WPS_EAP_VERSION_2               0x02
#define ONE_X_PKT_TYPE_EAP              0x00
#define ONE_X_PKT_TYPE_EAPOL_START      0x01
#define ONE_X_PKT_TYPE_EAPOL_LOGOFF     0x02
#define ONE_X_PKT_TYPE_EAPOL_KEY        0x03
#define ONE_X_PKT_TYPE_EAPOL_EASF_AL    0x04

#define EAP_CODE_REQUEST                0x01
#define EAP_CODE_RESPONSE               0x02
#define EAP_CODE_SUCCESS                0x03
#define EAP_CODE_FAILURE                0x04
#define EAP_TYPE_IDENTITY               0x01
#define EAP_TYPE_WPS_WSC                0xFE /* 254 */
#define EAP_WPS_IDENTITY_PKT_LEN        29

#define ONE_X_ETH_TYPE_EAPOL            0x888E

#define WPS_MAX_MAC_HDR_LEN             26
#define WPS_MAC_HDR_LEN                 24

/* 802.1x header format                                                      */
/* ---------------------------------------------------------------------     */
/* | SNAP    | Version | Type    | Body Length | Body                  |     */
/* ---------------------------------------------------------------------     */
/* | 8 octet | 1 octet | 1 octet | 2 octet     | (Length - 4) octets   |     */
/* ---------------------------------------------------------------------     */
#define SNAP_HEADER_OFFSET              0
#define SNAP_HDR_ETH_TYPE_OFFSET        6
#define SNAP_HEADER_LEN                 (SNAP_HDR_ETH_TYPE_OFFSET + 2)
#define ONE_X_PROT_VER_OFFSET           SNAP_HEADER_LEN
#define ONE_X_PKT_TYPE_OFFSET           (ONE_X_PROT_VER_OFFSET + 1)
#define ONE_X_PKT_LEN_OFFSET            (ONE_X_PKT_TYPE_OFFSET + 1)
#define ONE_X_PKT_BODY_OFFSET           (ONE_X_PKT_LEN_OFFSET  + 2)
#define ONE_X_HEADER_LEN                ONE_X_PKT_BODY_OFFSET

#define WPS_EAPOL_1X_HDR_LEN            ONE_X_PKT_BODY_OFFSET

#define EAP_HEADER_OFFSET               (ONE_X_PKT_BODY_OFFSET)
#define EAP_HEADER_LEN                  4
#define EAP_CODE_OFFSET                 EAP_HEADER_OFFSET
#define EAP_CODE_FIELD_LEN              1
#define EAP_IDENTIFIER_OFFSET           (EAP_CODE_OFFSET + \
                                         EAP_CODE_FIELD_LEN)
#define EAP_ID_FIELD_LEN                1
#define EAP_LEN_OFFSET                  (EAP_IDENTIFIER_OFFSET + \
                                         EAP_ID_FIELD_LEN)
#define EAP_TYPE_OFFSET                 (EAP_LEN_OFFSET + 2)
#define EAP_TYPE_FIELD_LEN              1
#define EAP_WPS_VENDOR_ID_OFFSET        (EAP_TYPE_OFFSET + \
                                         EAP_TYPE_FIELD_LEN)
#define EAP_WPS_VENDOR_TYPE_OFFSET      (EAP_WPS_VENDOR_ID_OFFSET+3)
#define EAP_WPS_OP_CODE_OFFSET          (EAP_WPS_VENDOR_TYPE_OFFSET+4)
#define EAP_WPS_FLAGS_OFFSET            (EAP_WPS_OP_CODE_OFFSET+1)
#define EAP_WPS_MSG_LEN_OFFSET          (EAP_WPS_FLAGS_OFFSET+1)
#define EAP_WPS_DATA_WI_LEN_OFFSET      (EAP_WPS_MSG_LEN_OFFSET+2)
#define EAP_WPS_DATA_WO_LEN_OFFSET      EAP_WPS_MSG_LEN_OFFSET
#define EAP_WPS_VID_VT_LEN              7

/* WPS Protocol related constants */
#define EAP_WSC_MF_BIT                  BIT0
#define EAP_WSC_LF_BIT                  BIT1


#ifdef MAC_P2P
#define WPS_REQ_DEV_TYPE_LEN            8
#endif /* MAC_P2P */
#define WPS_PRIM_DEV_TYPE_LEN           8
#define WPS_OS_VERSION_LEN              4
#define WPS_AUTHENTICATOR_LEN           8
#define WPS_KEY_WRAP_AUTH_LEN           8
#define WPS_AES_BYTE_BLOCK_SIZE         16
#define ERR_INVALID_ENCR_DATA_LEN       -1
#define ERR_INVALID_PAD                 -2

#define WPS_IBSS_MODE                   0
#define MAX_WPS_WLAN_FRAME_LEN          1500
#define MAX_WPS_WSC_EAP_MSG_LEN         (MAX_WPS_WLAN_FRAME_LEN - \
                                        WPS_MAX_MAC_HDR_LEN - 4)
#define WPS_EAPOL_START_PKT_LEN         (WPS_MAX_MAC_HDR_LEN+ \
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
#define WPS_ENCR_SET_DATA_OFFSET        (WPS_AES_BYTE_BLOCK_SIZE + 4)

#define WFA_VEN_EXTN_ID_BYTE0           0x00
#define WFA_VEN_EXTN_ID_BYTE1           0x37
#define WFA_VEN_EXTN_ID_BYTE2           0x2A
#define WPS_MIN_VEN_EXTN_LEN            3  /* 3 bytes Vendor ID */

#define LISTEN_INT_LEN                  2
#define CAP_INFO_LEN                    2

#ifdef MAC_P2P
#define P2P_PUB_ACT_TAG_PARAM_OFF       8
#define P2P_GEN_ACT_TAG_PARAM_OFF       7
#endif /* MAC_P2P */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
/* WLAN Frame Type and Sub Type codes */
typedef enum
{
    ASSOC_REQ             = 0x00,
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

/* WPS Attribute IDs */
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
    WPS_ATTR_MSG_TYPE =             0x1022,
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
// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
	WPS_ATTR_REQ_DEV_TYPE =         0x106A
#endif /* MAC_P2P */

} WPS_TLV_ATTR_ID_T;

/* Different Op-Codes of WPS-EAP message */
typedef enum
{
    WPS_OP_CODE_WSC_START       = 0x01,
    WPS_OP_CODE_WSC_ACK         = 0x02,
    WPS_OP_CODE_WSC_NACK        = 0x03,
    WPS_OP_CODE_WSC_MSG         = 0x04,
    WPS_OP_CODE_WSC_DONE        = 0x05,
    WPS_OP_CODE_WSC_FRAG_ACK    = 0x06
} WPS_EAP_OPCODE_T;

/* WPS Registration Protocol Message types */
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
/* Structures                                                                */
/*****************************************************************************/

/* Structure for parsing received WSC TLV attributes */
typedef struct
{
    UWORD8 manufacturer_len;
    UWORD8 model_name_len;
    UWORD8 model_number_len;
    UWORD8 serial_number_len;
    UWORD8 device_name_len;
    UWORD8 encrypted_settings_len;
    UWORD8 *version;
    UWORD8 *msg_type;
    UWORD8 *req_type;
    UWORD8 *rsp_type;
    UWORD8 *conn_type_flags;
    UWORD8 *rf_bands;
    UWORD8 *wps_state;
    UWORD8 *sel_reg;
    UWORD8 *auth_type_flags;
    UWORD8 *encr_type_flags;
    UWORD8 *config_meth;
    UWORD8 *assoc_state;
    UWORD8 *config_error;
    UWORD8 *dev_pass_id;
    UWORD8 *sel_reg_config_meth;
    UWORD8 *os_version;
    UWORD8 *mac_addr;
    UWORD8 *enr_nonce;
    UWORD8 *reg_nonce;
    UWORD8 *uuid;
    UWORD8 *pub_key;
    UWORD8 *authenticator;
#ifdef INT_WPS_REG_SUPP
    UWORD8 *e_hash1;
    UWORD8 *e_hash2;
    UWORD8 *e_snonce1;
    UWORD8 *e_snonce2;
#else  /* INT_WPS_REG_SUPP */
    UWORD8 *r_hash1;
    UWORD8 *r_hash2;
    UWORD8 *r_snonce1;
    UWORD8 *r_snonce2;
#endif /* INT_WPS_REG_SUPP */
    UWORD8 *key_wrap_authen;
    UWORD8 *encrypted_settings;
    UWORD8 *manufacturer;
    UWORD8 *model_name;
    UWORD8 *model_number;
    UWORD8 *serial_number;
    UWORD8 *prim_dev_type;
    UWORD8 *device_name;
    UWORD8 *wfa_vendor;
    UWORD8 *version2;
#if 0 /* TBD for ENROLLEE Support */
    UWORD8 auth_macs_cnt;
    UWORD8 *auth_macs[MAX_AUTH_MACS];
    UWORD8* credential[WPS_MAX_CRED_CNT];
    UWORD16 credential_len[WPS_MAX_CRED_CNT];
    UWORD8  cred_cnt;
#endif /* 0 */
#ifdef MAC_P2P
    UWORD8 *req_dev_type;
#endif /* MAC_P2P */
} wps_rx_attri_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern const UWORD8 g_eapol_snap_hdr[SNAP_HEADER_LEN];
extern const UWORD8 g_eap_wps_vid_vt[EAP_WPS_VID_VT_LEN];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD8 wps_parse_attri(UWORD8 *msg_ptr, wps_rx_attri_t** parse_attr,
                              UWORD16 msg_len, UWORD8 max_element_cnt,
                              BOOL_T is_wsc_ie);
extern STATUS_T wps_verify_auth_attr(UWORD8 *rx_msg_ptr, UWORD16 rx_msg_len,
                                     UWORD8 *tx_msg_ptr, UWORD16 tx_msg_len,
                                     UWORD8 *auth_key);
extern WORD16 wps_decrypt_encrypted_setting(UWORD8  *key_wrap_key,
                                            UWORD8  *encry_data,
                                            UWORD16 encry_data_len,
                                            UWORD8  *decry_data);
extern STATUS_T wps_verify_key_wrap_auth(UWORD8 *auth_key, UWORD8 *data,
                                         UWORD16 data_len);
extern STATUS_T wps_verify_pin_snonce(UWORD8 *enr_pub_key, UWORD8 *reg_pub_key,
                                      UWORD8 *s_nonce_ptr, UWORD8 *hash_ptr,
                                      UWORD8 *psk_ptr, UWORD8 *auth_key);
extern STATUS_T wps_gen_secr_keys(UWORD8  *rx_pub_key_ptr, UWORD32 *pub_key_buf,
                                  UWORD8  *enr_nonce, UWORD8  *reg_nonce,
                                  UWORD8  *enr_mac_addr, UWORD8  *auth_key,
                                  UWORD8  *key_wrap_key,
                                  long_num_struct_t *priv_key_ptr);
extern void wps_compute_auth_attr(UWORD8 *rx_msg_ptr, UWORD16 rx_msg_len,
                                  UWORD8 *tx_msg_ptr, UWORD16 tx_msg_len,
                                  UWORD8 *auth_key, UWORD8 *auth_attr);
extern UWORD16 wps_gen_pin_hash(WPS_PROT_TYPE_T prot_type, UWORD8 *dev_pin,
                                UWORD8 dev_pin_len, UWORD8 *buf_ptr,
                                WPS_TLV_ATTR_ID_T hash1,
                                WPS_TLV_ATTR_ID_T hash2, UWORD8 *auth_key,
                                UWORD8 *psk1, UWORD8 *psk2, UWORD8 *snonce1,
                                UWORD8 *snonce2, UWORD8 *reg_pub_key,
                                UWORD8 *enr_pub_key);
extern UWORD16 wps_gen_encr_settings(UWORD8 *buf_ptr, UWORD8 *attr_val_ptr,
                                     WPS_TLV_ATTR_ID_T attr_type,
                                     UWORD16 attr_len, BOOL_T  put_attr,
                                     UWORD8  *auth_key, UWORD8 *key_wrap_key);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* All WPS Protocol related inline functions                                 */
/*****************************************************************************/

/*****************************************************************************/
/* This function puts the WSC version number attribute at the location       */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_version(UWORD8 *buf_ptr)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_VERSION);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = WPS_VERSION;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC message type(value passed as input) attribute  */
/* at the location pointed by input pointer                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_msg_type(UWORD8 *buf_ptr, UWORD8 msg_type)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_MSG_TYPE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = msg_type;
    return(index);
}

/*****************************************************************************/
/* This function puts the WSC WPS State type attribute at the location       */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_wps_state(UWORD8 *buf_ptr, UWORD8 wps_state)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_WPS_STATE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = wps_state;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC AP Setup Locked type attribute at the location */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_ap_setup_locked(UWORD8 *buf_ptr, UWORD8 ap_setup_locked)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_AP_SETUP_LOCKED);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = ap_setup_locked;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC Selected registrar type attribute at the       */
/* location  pointed by input pointer                                        */
/*****************************************************************************/
INLINE UWORD16 put_wsc_sel_reg(UWORD8 *buf_ptr, BOOL_T sel_reg)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_SEL_REG);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = sel_reg;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC request type attribute at the location         */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_req_type(UWORD8 *buf_ptr, WPS_REQUEST_TYPE_T req_type)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_REQUEST_TYPE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = req_type;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC Response type attribute at the location        */
/* pointed by input pointer                                                  */
/*****************************************************************************/
INLINE UWORD16 put_wsc_rsp_type(UWORD8 *buf_ptr, UWORD8 rsp_type)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_RESPONSE_TYPE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = rsp_type;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC Enrollee nonce(value passed as input) attribute*/
/* at the location pointed by input pointer                                  */
/*****************************************************************************/
INLINE  UWORD16 put_wsc_enr_nonce(UWORD8 *buf_ptr, UWORD8* nonce_ptr)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_ENR_NONCE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_NONCE_LEN);
    index += 2;
    memcpy((buf_ptr + index), nonce_ptr, WPS_NONCE_LEN);
    index += WPS_NONCE_LEN;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC Registrar nonce(value passed as input)         */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD16 put_wsc_reg_nonce(UWORD8 *buf_ptr, UWORD8* nonce_ptr)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_REG_NONCE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_NONCE_LEN);
    index += 2;
    memcpy((buf_ptr + index), nonce_ptr, WPS_NONCE_LEN);
    index += WPS_NONCE_LEN;
    return index;
}

/*****************************************************************************/
/* This function puts the WSC Configuration error(value passed as input)     */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD16 put_wsc_config_error(UWORD8 *buf_ptr, UWORD16 config_error)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_CONFIG_ERROR);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), config_error);
    index += 2;
    return(index);
}

/*****************************************************************************/
/* This function puts the UUID-E(value passed as input)  attribute at the    */
/* location pointed by input pointer                                         */
/*****************************************************************************/
INLINE  UWORD16 put_wsc_uuid_e(UWORD8 *buf_ptr, UWORD8 *uuid_e)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_UUID_E);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_UUID_LEN);
    index += 2;
    memcpy((buf_ptr + index), uuid_e, WPS_UUID_LEN);
    index += WPS_UUID_LEN;
    return(index);
}

/*****************************************************************************/
/* This function puts the UUID-R(value passed as input)  attribute at the    */
/* location pointed by input pointer                                         */
/*****************************************************************************/
INLINE  UWORD16 put_wsc_uuid_r(UWORD8 *buf_ptr, UWORD8* uuid_r)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_UUID_R);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_UUID_LEN);
    index += 2;
    memcpy((buf_ptr + index), uuid_r, WPS_UUID_LEN);
    index += WPS_UUID_LEN;
    return(index);
}

/*****************************************************************************/
/* This function puts the MAC-Address(value passed as input) attribute at the*/
/* location pointed by input pointer                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_mac_addr(UWORD8 *buf_ptr, UWORD8* mac_addr)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_MAC_ADDR);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_MAC_ADDR_LEN);
    index += 2;
    memcpy((buf_ptr + index), mac_addr, WPS_MAC_ADDR_LEN);
    index += WPS_MAC_ADDR_LEN;
    return(index);
}

/*****************************************************************************/
/* This function puts the Public Key(value passed as input) attribute at the */
/* location pointed by input pointer                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_public_key(UWORD8 *buf_ptr, UWORD8* pub_key)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_PUBLIC_KEY);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_PUBLIC_KEY_LEN);
    index += 2;
    memcpy((buf_ptr + index), pub_key, WPS_PUBLIC_KEY_LEN);
    index += WPS_PUBLIC_KEY_LEN;
    return(index);
}

/*****************************************************************************/
/* This function puts the Authentication Type flags(value passed as input)   */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD16 put_wsc_auth_type_flags(UWORD8 *buf_ptr, UWORD16 auth_type_flag)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_AUTH_TYPE_FLAGS);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), auth_type_flag);
    index += 2;
    return(index);
}

/*****************************************************************************/
/* This function puts the Encryption Type flags(value passed as input)       */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE UWORD16 put_wsc_encr_type_flags(UWORD8 *buf_ptr, UWORD16 enc_type_flag)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_ENCRY_TYPE_FLAGS);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), enc_type_flag);
    index += 2;
    return(index);
}

/*****************************************************************************/
/* This function puts the Connection Type flags(value passed as input)       */
/* attribute at the location pointed by input pointer                        */
/*****************************************************************************/
INLINE  UWORD16 put_wsc_conn_type_flags(UWORD8 *buf_ptr, UWORD8 conn_type_flag)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_CONN_TYPE_FLAGS);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = conn_type_flag;
    return(index);
}

/*****************************************************************************/
/* This function puts the Config Methods(value passed as input) attribute at */
/* the location pointed by input pointer                                     */
/*****************************************************************************/
INLINE UWORD16 put_wsc_config_meth(UWORD8 *buf_ptr, UWORD16 config_meth)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_CONFIG_METHODS);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), config_meth);
    index += 2;
    return(index);
}

/*****************************************************************************/
/* This function puts the Selected Registrar Config Methods (value passed as */
/* input) attribute at the location pointed by input pointer                 */
/*****************************************************************************/
INLINE UWORD16 put_wsc_sel_reg_config_meth(UWORD8 *buf_ptr,
                                           UWORD16 config_meth)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_SEL_REG_CONFIG_METH);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), config_meth);
    index += 2;
    return(index);
}

/*****************************************************************************/
/* This function puts the Manufacturer attribute at the location pointed by  */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_manufacturer(UWORD8 *buf_ptr, UWORD8 *manufacturer,
                                    UWORD16 manufacturer_len)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_MANUFACTURER);
    index += 2;
    PUT_U16_BE((buf_ptr + index), manufacturer_len);
    index += 2;
    memcpy((buf_ptr + index), manufacturer, manufacturer_len);
    index += manufacturer_len;
    return(index);
}

/*****************************************************************************/
/* This function puts the Model name attribute at the location pointed by    */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_model_name(UWORD8 *buf_ptr, UWORD8 *model_name,
                                  UWORD16 len)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_MODEL_NAME);
    index += 2;
    PUT_U16_BE((buf_ptr + index), len);
    index += 2;
    memcpy((buf_ptr + index), model_name, len);
    index += len;
    return(index);
}

/*****************************************************************************/
/* This function puts the Model number attribute at the location pointed by  */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_model_num(UWORD8 *buf_ptr, UWORD8 *model_num,
                                 UWORD16 len)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_MODEL_NUM);
    index += 2;
    PUT_U16_BE((buf_ptr + index), len);
    index += 2;
    memcpy((buf_ptr + index), model_num, len);
    index += len;
    return(index);
}

/*****************************************************************************/
/* This function puts the Serial number attribute at the location pointed by */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_serial_num(UWORD8 *buf_ptr, UWORD8 *serial_num,
                                  UWORD16 len)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_SERIAL_NUM);
    index += 2;
    PUT_U16_BE((buf_ptr + index), len);
    index += 2;
    memcpy((buf_ptr + index), serial_num, len);
    index += len;
    return(index);
}

/*****************************************************************************/
/* This function puts the Primary Device type attribute at the location      */
/* pointed by the input pointer                                              */
/*****************************************************************************/
INLINE UWORD16 put_wsc_prim_dev_type(UWORD8 *buf_ptr, UWORD8 *prim_dev_cat_id)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_PRIM_DEV_TYPE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), WPS_PRIM_DEV_TYPE_LEN);
    index += 2;
    memcpy((buf_ptr + index), prim_dev_cat_id, WPS_PRIM_DEV_TYPE_LEN);
    index += WPS_PRIM_DEV_TYPE_LEN;
    return(index);
}

/*****************************************************************************/
/* This function puts the Device name attribute at the location pointed by   */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_device_name(UWORD8 *buf_ptr, UWORD8 *dev_name,
                                   UWORD16 len)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_DEV_NAME);
    index += 2;
    PUT_U16_BE((buf_ptr + index), len);
    index += 2;
    memcpy((buf_ptr + index), dev_name, len);
    index += len;
    return(index);
}

/*****************************************************************************/
/* This function puts the RF-Bands attribute at the location pointed by the  */
/* input pointer                                                             */
/*****************************************************************************/
INLINE UWORD16 put_wsc_rf_bands(UWORD8 *buf_ptr, UWORD8 rf_bands)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_RF_BANDS);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 1);
    index += 2;
    buf_ptr[index++] = rf_bands;
    return(index);
}

/*****************************************************************************/
/* This function puts the Association State attribute at the location pointed*/
/* by the input pointer                                                      */
/*****************************************************************************/
INLINE UWORD16 put_wsc_assoc_state(UWORD8 *buf_ptr, UWORD16 assoc_state)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_ASSOC_STATE);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), assoc_state);
    index += 2;
    return(index);
}

/*****************************************************************************/
/* This function puts the Device Password ID attribute at the location       */
/* pointed by the input pointer                                              */
/*****************************************************************************/
INLINE UWORD16 put_wsc_dev_pass_id(UWORD8* buf_ptr, UWORD16 dev_pass_id)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_DEV_PASS_ID);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 2);
    index += 2;
    PUT_U16_BE((buf_ptr + index), dev_pass_id);
    index += 2;
    return(index);
}

/*****************************************************************************/
/* This function puts the OS Version attribute at the location pointed by the*/
/* input pointer                                                             */
/*****************************************************************************/
INLINE UWORD16 put_wsc_os_ver(UWORD8 *buf_ptr, UWORD32 os_version)
{
    UWORD16 index = 0;

    PUT_U16_BE((buf_ptr + index), WPS_ATTR_OS_VERSION);
    index += 2;
    PUT_U16_BE((buf_ptr + index), 4);
    index += 2;
    PUT_U32_BE((buf_ptr + index), os_version);
    index += 4;
    return(index);
}

/*****************************************************************************/
/* This function puts the Authenticator attribute at the location pointed by */
/* the input pointer                                                         */
/*****************************************************************************/
INLINE UWORD16 put_wsc_auth_attr(UWORD8 *buffer, UWORD8 *auth_attr)
{
    UWORD16 index = 0;

    PUT_U16_BE((buffer + index), WPS_ATTR_AUTHENTICATOR);
    index += 2;
    PUT_U16_BE((buffer + index), WPS_AUTHENTICATOR_LEN);
    index += 2;
    memcpy((buffer + index), auth_attr, WPS_AUTHENTICATOR_LEN);

    return(WPS_AUTHENTICATOR_LEN + index);
}

/*****************************************************************************/
/* This function puts WFA Vendor Extension attribute with specified          */
/* subelements at specified location in the buffer                           */
/*****************************************************************************/
INLINE UWORD16 put_wsc_wfa_vendor(UWORD8* buffer, wps_sub_elem_t **elem,
                                  UWORD8 elem_cnt)
{
    UWORD16 index = 0;
    UWORD8  i = 0;

    /*************************************************************************/
    /* WFA Vendor Extension Attribute with Sub-Elements Format               */
    /*************************************************************************/
    /* +-----------+---------+----------+-----------+-----------+-----------+*/
    /* |  2-Bytes  | 2-Bytes |  3-Bytes |   1-Byte  |  1-Byte   |0-255 Bytes|*/
    /* +-----------+---------+----------+-----------+-----------+-----------+*/
    /* | Attribute |Attribute| Vendor-ID|Sub-Element|Sub-Element|Sub-Element|*/
    /* |ID (0x1049)|  Length |(0x00372A)|    ID-1   |   Length  |   value   |*/
    /* +-----------+---------+----------+-----------+-----------+-----------+*/
    /*************************************************************************/

    PUT_U16_BE((buffer + index), WPS_ATTR_VENDOR_EXTENSION);
    index += 4;

    *(buffer + index) = WFA_VEN_EXTN_ID_BYTE0;
    index++;
    *(buffer + index) = WFA_VEN_EXTN_ID_BYTE1;
    index++;
    *(buffer + index) = WFA_VEN_EXTN_ID_BYTE2;
    index++;

    for(i = 0; i < elem_cnt; i++)
    {
        *(buffer + index) = elem[i]->id;
        index++;
        *(buffer + index) = elem[i]->len;
        index++;

        memcpy((buffer + index), elem[i]->val, elem[i]->len);
        index += elem[i]->len;
    }

    PUT_U16_BE((buffer + 2), (index - 4));

    return index;
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
/* This function returns the WPS Message Attribute value located at given    */
/* memory location                                                           */
/*****************************************************************************/
INLINE UWORD8 get_wsc_msg_type(UWORD8 *buffer)
{
    UWORD8 retval = 0;

    if(NULL != buffer)
        retval = buffer[0];

    return retval;
}

/*****************************************************************************/
/* This function returns the WPS Connection Type Flags Attribute value       */
/* located at given memory location                                          */
/*****************************************************************************/
INLINE UWORD8 get_wsc_conn_type_flags(UWORD8 *buffer)
{
    UWORD8 retval = 0;

    if(NULL != buffer)
        retval = buffer[0];

    return retval;
}

/*****************************************************************************/
/* This function returns the WPS Config Methods  Attribute value located at  */
/* given memory location                                                     */
/*****************************************************************************/
INLINE UWORD16 get_wsc_config_meth(UWORD8 *buffer)
{
    UWORD16 retval = 0;

    if(NULL != buffer)
        retval = GET_U16_BE(buffer);

    return retval;
}

/*****************************************************************************/
/* This function returns the WPS Authentication Type Flags Attribute value   */
/* located at given memory location                                          */
/*****************************************************************************/
INLINE UWORD16 get_wsc_auth_type_flags(UWORD8 *buffer)
{
    UWORD16 retval = 0;

    if(NULL != buffer)
        retval = GET_U16_BE(buffer);

    return retval;
}

/*****************************************************************************/
/* This function returns the WPS Encryption Type Flags Attribute value       */
/* located at given memory location                                          */
/*****************************************************************************/
INLINE UWORD16 get_wsc_encr_type_flags(UWORD8 *buffer)
{
    UWORD16 retval = 0;

    if(NULL != buffer)
        retval = GET_U16_BE(buffer);

    return retval;
}

/*****************************************************************************/
/* This function returns the WPS Device Pass ID Attribute value located at   */
/* given memory location                                                     */
/*****************************************************************************/
INLINE UWORD16 get_wsc_dev_pass_id(UWORD8 *buffer)
{
    UWORD16 retval = 0;

    if(NULL != buffer)
        retval = GET_U16_BE(buffer);

    return retval;
}

/*****************************************************************************/
/* Function related to 802.1x and EAP                                        */
/*****************************************************************************/

/*****************************************************************************/
/* This function checks if there is EAPOL Snap header present. If not then   */
/* return BFALSE else BTRUE                                                  */
/*****************************************************************************/
INLINE BOOL_T is_eapol_snap_header(UWORD8 *buffer)
{
    if(0 != memcmp(buffer, g_eapol_snap_hdr, SNAP_HEADER_LEN))
        return BFALSE;

    return BTRUE;
}

/* This function copies a EAPOL type SNAP header at given buffer location    */
INLINE void set_eapol_snap_header(UWORD8 *buffer)
{
    memcpy(buffer, g_eapol_snap_hdr, SNAP_HEADER_LEN);
    return;
}

/*****************************************************************************/
/* This function return the 802.1x Protocol Version from 1x header Protocol  */
/* Version Field                                                             */
/*****************************************************************************/
INLINE UWORD8 get_1x_version(UWORD8 *buffer)
{
    UWORD8 onex_version = buffer[ONE_X_PROT_VER_OFFSET];

    return onex_version;
}

/* This function sets specified 802.1X Version in 1X header */
INLINE void set_1x_version(UWORD8 *buffer, UWORD8 version)
{
    buffer[ONE_X_PROT_VER_OFFSET] = version;
    return;
}


/*****************************************************************************/
/* This function return the 802.1x Packet type from 1x header Protocol       */
/* Packet Type                                                               */
/*****************************************************************************/
INLINE UWORD8 get_1x_pkt_type(UWORD8 *buffer)
{
    UWORD8 onex_pkt_type = buffer[ONE_X_PKT_TYPE_OFFSET];

    return onex_pkt_type;
}

/* This function sets specified value at 802.1X Packet Type field */
INLINE void set_1x_pkt_type(UWORD8 *buffer, UWORD8 onex_pkt_type)
{
    buffer[ONE_X_PKT_TYPE_OFFSET] = onex_pkt_type;
    return;
}

/*****************************************************************************/
/* This function return the 802.1x Packet Length from 1x header`s Packet     */
/* Length Field                                                              */
/*****************************************************************************/
INLINE UWORD16 get_1x_pkt_len(UWORD8 *buffer)
{
    UWORD16 onex_pkt_len = GET_U16_BE((buffer + ONE_X_PKT_LEN_OFFSET));

    return onex_pkt_len;
}

/* This function sets specified value at 802.1X Packet Length field */
INLINE void set_1x_pkt_len(UWORD8 *buffer, UWORD16 onex_pkt_len)
{
    PUT_U16_BE((buffer + ONE_X_PKT_LEN_OFFSET), onex_pkt_len);
    return;
}

/* This function returns the EAP Code from Passed 1x EAPOL-Packet */
INLINE UWORD8 get_eap_code(UWORD8 *buffer)
{
    UWORD8 eap_code = buffer[EAP_CODE_OFFSET];

    return eap_code;
}

/* This function sets specified value at EAP Code field in a EAP Header */
INLINE void set_eap_code(UWORD8 *buffer, UWORD8 eap_code)
{
    buffer[EAP_CODE_OFFSET] = eap_code;
    return;
}

/* This function returns the EAP Identifier from Passed 1x EAPOL-Packet */
INLINE UWORD8 get_eap_id(UWORD8 *buffer)
{
    UWORD8 eap_id = buffer[EAP_IDENTIFIER_OFFSET];

    return eap_id;
}

/* This function sets specified value at EAP Identifier field in EAP Header  */
INLINE void set_eap_id(UWORD8 *buffer, UWORD8 eap_id)
{
    buffer[EAP_IDENTIFIER_OFFSET] = eap_id;
    return;
}

/* This function returns the EAP Length from Passed 1x EAPOL-Packet */
INLINE UWORD16 get_eap_len(UWORD8 *buffer)
{
    UWORD16 eap_len = GET_U16_BE((buffer + EAP_LEN_OFFSET));

    return eap_len;
}

/* This function sets specified value at EAP Length field in EAP Header */
INLINE void set_eap_len(UWORD8 *buffer, UWORD16 eap_len)
{
    PUT_U16_BE((buffer + EAP_LEN_OFFSET), eap_len);
    return;
}

/* This function returns the EAP Type from Passed 1x EAPOL-Packet */
INLINE UWORD8 get_eap_type(UWORD8 *buffer)
{
    UWORD8 eap_type = buffer[EAP_TYPE_OFFSET];

    return eap_type;
}

/* This function sets specified value at EAP Type field in EAP Header  */
INLINE void set_eap_type(UWORD8 *buffer, UWORD8 eap_type)
{
    buffer[EAP_TYPE_OFFSET] = eap_type;
    return;
}

/* This functions returns the value of Op-Code field from EAP WSC header */
INLINE UWORD8 get_wsc_opcode(UWORD8 *buffer)
{
    UWORD8 opcode = buffer[EAP_WPS_OP_CODE_OFFSET];

    return opcode;
}

/* This function sets specified value at WSC Op-Code field in EAP Header */
INLINE void set_wsc_opcode(UWORD8 *buffer, UWORD8 opcode)
{
    buffer[EAP_WPS_OP_CODE_OFFSET] = opcode;

    return;
}

/* This functions returns the value of Flags field from EAP WSC header */
INLINE UWORD8 get_wsc_flags(UWORD8 *buffer)
{
    UWORD8 flags = buffer[EAP_WPS_FLAGS_OFFSET];

    return flags;
}

/* This function sets specified value at WSC Op-Code field in EAP Header */
INLINE void set_wsc_flags(UWORD8 *buffer, UWORD8 flags)
{
    buffer[EAP_WPS_FLAGS_OFFSET] = flags;

    return;
}

/*****************************************************************************/
/* This function returns the value in EAP-WSC Message Length field from the  */
/* passed EAP WSC Header                                                     */
/*****************************************************************************/
INLINE UWORD16 get_wsc_msg_len(UWORD8 *buffer)
{
    UWORD16 msg_len = GET_U16_BE((buffer + EAP_WPS_MSG_LEN_OFFSET));

    return msg_len;
}

/* This function sets specified value at WSC Message Length field */
INLINE void set_wsc_msg_len(UWORD8 *buffer, UWORD16 msg_len)
{
    PUT_U16_BE((buffer + EAP_WPS_MSG_LEN_OFFSET), msg_len);
    return;
}

/*****************************************************************************/
/* This function returns BTRUE if More Fragment bit in the Flags field is set*/
/* else returns a BFALSE                                                     */
/*****************************************************************************/
INLINE BOOL_T is_wsc_mf_set(UWORD8 flags)
{
    BOOL_T retval = BFALSE;

    if(EAP_WSC_MF_BIT & flags)
        retval = BTRUE;

    return retval;
}

/*****************************************************************************/
/* This function returns BTRUE if Length bit in the Flags field is set else  */
/* returns a BFALSE                                                          */
/*****************************************************************************/
INLINE BOOL_T is_wsc_lf_set(UWORD8 flags)
{
    BOOL_T retval = BFALSE;

    if(EAP_WSC_LF_BIT & flags)
        retval = BTRUE;

    return retval;
}


/* This function inserts SNAP and 802.1X header at given mempry location     */
INLINE void wps_prep_1x_header(UWORD8 *buffer, UWORD8 one_x_type,
                               UWORD16 tx_len)
{
    /*************************************************************************/
    /* 802.1x header format                                                  */
    /* +---------+---------+---------+-------------+-----------------------+ */
    /* | SNAP    | Version | Type    | Body Length | Body                  | */
    /* +---------+---------+---------+-------------+-----------------------+ */
    /* | 8 octet | 1 octet | 1 octet | 2 octet     | (Length - 4) octets   | */
    /* +---------+---------+---------+-------------+-----------------------+ */
    /*************************************************************************/

    set_eapol_snap_header(buffer);

    set_1x_version(buffer, WPS_EAP_VERSION_1);

    set_1x_pkt_type(buffer, one_x_type);

    set_1x_pkt_len(buffer, tx_len);

    return;
}

/* This function prepares the EAP Header */
INLINE void wps_prep_eap_header(UWORD8 *buffer, UWORD8 eap_code, UWORD8 eap_id,
                                UWORD16 eap_len)
{
    /* EAP header format                                                  */
    /* +----------+-----------+--------+ */
    /* | EAP-Code | Idenifier | Length | */
    /* +----------+-----------+--------+ */
    /* | 1 octet  | 1 octet   | 2 octet| */
    /* +----------+-----------+--------+ */

    /* Set the EAP Code in EAP Header */
    set_eap_code(buffer, eap_code);

    /* Set the EAP Identifier in EAP Header */
    set_eap_id(buffer, eap_id);

    /* Set the EAP Length in EAP Header */
    set_eap_len(buffer, eap_len);
}

/* This function set the Vendor ID and Vendor Type Fields in EAP-WSC header  */
INLINE void set_wsc_vid_vt(UWORD8 *buffer)
{
    memcpy((buffer + EAP_WPS_VENDOR_ID_OFFSET), g_eap_wps_vid_vt,
            EAP_WPS_VID_VT_LEN);
    return;
}

/* This function sets the 802.1X, EAP and WSC headers for a EAP WSC packet   */
INLINE void wps_prep_1x_eap_wsc_header(UWORD8 *buffer, UWORD8 eap_code,
                                       UWORD8 eap_id, UWORD16 eap_len,
                                       UWORD8 opcode, UWORD8 flags,
                                       UWORD16 msg_len)
{
    /*************************************************************************/
    /* The EAP Packet format for WPS                                         */
    /*************************************************************************/
    /*         0       |      1        |       2       |       3       |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |     Code      |  Identifier   |            Length             |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |     Type      |               Vendor-Id                       |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |                          Vendor-Type                          |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |  Op-Code      |    Flags      | Message Length                |     */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+     */
    /* |  Message data                                                       */
    /* +-+-+-+-+-+-+-+-+-+-+-+-+-+-                                          */
    /*************************************************************************/

    /* Set 1X and snap header */
    wps_prep_1x_header(buffer, ONE_X_PKT_TYPE_EAP, eap_len);

    /* Set EAP header */
    wps_prep_eap_header(buffer, eap_code, eap_id, eap_len);

    /* Set the EAP Type in EAP Header */
    set_eap_type(buffer, EAP_TYPE_WPS_WSC);

    /* Set Vendor ID and Vendor Type  */
    set_wsc_vid_vt(buffer);

    /* Set WSC Opcode field */
    set_wsc_opcode(buffer, opcode);

    /* Set Flags field */
    set_wsc_flags(buffer, flags);

    /* Set Message Length only if LF bit is set in Flags field */
    if(is_wsc_lf_set(flags))
        set_wsc_msg_len(buffer, msg_len);
}

/*****************************************************************************/
/* This functions parses the received WFA Vendor Extension Attribute for Sub */
/* Elements                                                                  */
/*****************************************************************************/
INLINE void wps_parse_wsc_subelem_reg(wps_rx_attri_t *rx_attr,
                                      UWORD16 attr_len)
{
    UWORD16 index = 0;
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
            default:
                /* Do nothing and ignore the subelement */
            break;
        }

        index += subelem_len;
    }
}

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
INLINE UWORD16 put_wsc_req_dev_type(UWORD8* buf_ptr, UWORD8* req_dev_cat_id)
{
    UWORD16 index = 0;

	if(NULL != req_dev_cat_id)
    {
	    /* Put the requested device ID attribute only if present */
        PUT_U16_BE((buf_ptr + index), WPS_ATTR_REQ_DEV_TYPE);
        index += 2;
        PUT_U16_BE((buf_ptr + index), WPS_REQ_DEV_TYPE_LEN);
        index += 2;
        memcpy((buf_ptr + index), req_dev_cat_id, WPS_REQ_DEV_TYPE_LEN);
        index += WPS_REQ_DEV_TYPE_LEN;
    }

    return  index;
}
#endif /* MAC_P2P */

#endif /* WPS_PROT_H */
