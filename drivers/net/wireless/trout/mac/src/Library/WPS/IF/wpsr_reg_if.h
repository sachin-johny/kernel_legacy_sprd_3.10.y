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
/*  File Name         : wpsr_reg_if.h                                        */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      WPS library interface. It has function definitions   */
/*                      of APIs provided by WPS Library and APIs required    */
/*                      by WPS Library.                                      */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         17 02 2011   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_REG_SUPP

#ifndef WPS_REG_IF_H
#define WPS_REG_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wps_common.h"
#include "csl_types.h"

/*****************************************************************************/
/* Constant Macros                                                           */
/*****************************************************************************/

#define MAX_NET_KEY_LEN         64
#define MAX_NW_KEY_PER_CRED     1
#define MAX_ESSID_LEN           32

#define MAX_WPS_PIN_LEN         16
#define WPS_PBC_PASS_LEN        8
#define MAX_WPS_PASS_LEN        (MAX(MAX_WPS_PIN_LEN, WPS_PBC_PASS_LEN))

#define WPS_UUID_LEN            16
#define WPS_NONCE_LEN           16
#define SHA1_DIGEST_LEN         20
#define WPS_VERSION             0x10
#define WPS_PSK_LEN             16
#define WPS_AUTH_KEY_LEN        32
#define WPS_KEY_WRAP_KEY_LEN    16
#define WPS_KWK_KEY_LEN         WPS_KEY_WRAP_KEY_LEN
#define WPS_EMSK_LEN            32
#define SHA_256_HASH_LEN        32
#define WPS_HASH_LEN            SHA_256_HASH_LEN
#define WPS_DH_PRIME_MSB_POS    1536
#define WPS_DH_PRIME_LEN_WORD32 (WPS_DH_PRIME_MSB_POS >> 5)
#define WPS_DH_PRIME_LEN_BYTE   (WPS_DH_PRIME_MSB_POS >> 3)
#define WPS_PUBLIC_KEY_LEN      WPS_DH_PRIME_LEN_BYTE
#define WPS_PRIVATE_KEY_LEN     64
#define WPS_PRIVATE_KEY_LEN_WORD32 (WPS_PRIVATE_KEY_LEN >> 5)
#define MAX_AUTH_MACS           5
#define MAX_SUB_ELEM_LEN        6 /* Ideally MAC_ADDRESS_LEN * MAX_AUTH_MACS*/
#define MAX_SUB_ELEM_SUPP       2 /* Version2 and Authorized-MACs */
#define WPS_VER2_LEN            1
#define WPS_VER2_VAL            0x20
#define WPS_AUTH_MACS_LEN       MAC_ADDRESS_LEN
#define WPS_AUTH_MACS_VAL       0xFF

/*****************************************************************************/
/* Different types of Device Password IDs                                    */
/*****************************************************************************/
#define DEV_PASS_ID_PUSHBUTT    0x0004

/*****************************************************************************/
/* Typedefs                                                                  */
/*****************************************************************************/
#ifndef OS_LINUX_CSL_TYPE
typedef void    WPS_ALARM_FUNC_T(void* alarm, UWORD32 data);
#else /* OS_LINUX_CSL_TYPE */
typedef void    WPS_ALARM_FUNC_T(UWORD32 data);
#endif /* OS_LINUX_CSL_TYPE */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/
/* Type of WPS protocol. currently only two PIN and PBC are supported */
typedef enum
{
    WPS_PROT_NONE = 0,       /* No Protocol Selected                         */
    PIN           = 1,       /* PIN mode of WPS Registration selected        */
    PBC           = 2        /* PBC mode of WPS Registration selected        */
} WPS_PROT_TYPE_T;

/* Different types of Device Password IDs as defined in the standard */
typedef enum
{
    PASS_ID_DEFAULT        = 0x0000,
    PASS_ID_USER_SPECIFIED = 0x0001,
    PASS_ID_MACH_SPECIFIED = 0x0002,
    PASS_ID_REKEY          = 0x0003,
    PASS_ID_PUSHBUTTON     = 0x0004,
    PASS_ID_REG_SPECIFIED  = 0x0005
} WPS_DEV_PASS_ID_T;

/* Different WPS Configuration Methods as defined in the standard */
typedef enum
{
    CONFIG_METHOD_USBA_FLASH    = 0x0001,
    CONFIG_METHOD_ETHERNET      = 0x0002,
    CONFIG_METHOD_LABEL         = 0x0004,
    CONFIG_METHOD_DISPLAY       = 0x0008,
    CONFIG_METHOD_EXT_NFC_TOKEN = 0x0010,
    CONFIG_METHOD_INT_NFC_TOKEN = 0x0020,
    CONFIG_METHOD_NFC_IF        = 0x0040,
    CONFIG_METHOD_PUSHBUTTON    = 0x0080,
    CONFIG_METHOD_KEYPAD        = 0x0100,
    CONFIG_METHOD_PBC_WEB       = 0x4000,
    CONFIG_METHOD_PIN_WEB       = 0x8000,
    CONFIG_METHOD_VIRTUAL_PBC   = 0x0280,
    CONFIG_METHOD_PHYSICAL_PBC  = 0x0480,
    CONFIG_METHOD_VIRTUAL_PIN   = 0x2008,
    CONFIG_METHOD_PHYSICAL_PIN  = 0x4008,
} WPS_CONFIG_METHOD_T;

/* WPS Request Type */
typedef enum
{
    REQ_ENRO_INFO_ONLY          = 0x00,
    REQ_ENR_OPEN_802_1X         = 0x01,
    REQ_REGISTRAR               = 0x02,
    WLAN_MANAGER_REGISTRAR      = 0x03
} WPS_REQUEST_TYPE_T;

/* WPS Response Type */
typedef enum
{
    RSP_ENRO_INFO_ONLY          = 0x00,
    RSP_ENR_OPEN_802_1X         = 0x01,
    RSP_REGISTRAR               = 0x02,
    AP                          = 0x03
} WPS_RESPONSE_TYPE_T;

/* WPS Association Status values as defined in WPS standard */
typedef enum
{
    WPS_NOT_ASSOCIATED          = 0,
    WPS_CONNECTION_SUCCESS      = 1,
    WPS_CONFIGURATION_FAILURE   = 2,
    WPS_ASSOCIATION_FAILURE     = 3,
    WPS_IP_FAILURE              = 4
} WPS_ASSOC_STATE_T;

/* WPS Encryption Type as defined in WPS standard  */
typedef enum
{
    WPS_NONE        = 0x0001,
    WPS_WEP         = 0x0002,
    WPS_TKIP        = 0x0004,
    WPS_AES         = 0x0008,
    WPS_AES_TKIP    = 0x0010
} WPS_CRED_ENC_TYPE_T;

/* WPS Authentication Type as defined in WPS standard  */
typedef enum
{
    WPS_OPEN            = 0x0001,
    WPS_WPA_PSK         = 0x0002,
    WPS_SHARED          = 0x0004,
    WPS_WPA             = 0x0008,
    WPS_WPA2            = 0x0010,
    WPS_WPA2_PSK        = 0x0020,
    WPS_WPA_WPA2_MIXED  = 0x0040
} WPS_CRED_AUTH_TYPE_T;

/* WPS Configuration Error values as defined in WPS standard */
typedef enum
{
    WPS_NO_ERROR                    = 0,
    OOB_INTERFACE_READ_ERROR        = 1,
    DECRYPTION_CRC_FAILURE          = 2,
    CHANNEL_2_4_NOT_SUPPORTED       = 3,
    CHANNEL_5_0_NOT_SUPPORTED       = 4,
    SIGNAL_TOO_WEAK                 = 5,
    NETWORK_AUTH_FAILURE            = 6,
    NETWORK_ASSOCIATION_FAILURE     = 7,
    NO_DHCP_RESPONSE                = 8,
    FAILED_DHCP_CONFIG              = 9,
    IP_ADDRESS_CONFLICT             = 10,
    COULDNT_CONNECT_TO_REGISTRAR    = 11,
    MULTIPLE_PBC_SESSIONS_DETECTED  = 12,
    ROGUE_ACTIVITY_SUSPECTED        = 13,
    DEVICE_BUSY                     = 14,
    SETUP_LOCKED                    = 15,
    MESSAGE_TIMEOUT                 = 16,
    REGISTRATION_SESSION_TIMEOUT    = 17,
    DEVICE_PASSWORD_AUTH_FAILURE    = 18
} WPS_CONFIG_ERROR_T;

/* Wi-Fi Protected Setup State Value */
typedef enum
{
    WPS_NOT_CONFIGURED = 0x01,
    WPS_CONFIGURED     = 0x02
} WPS_STATE_VAL_T;

/* Overall WPS States of the Registrar */
typedef enum
{
    WPS_IDLE_ST     = 0,  /* When WPS Protocol is not in progress            */
    WPS_REG_PROT_ST = 1   /* When in Registration Protocol is in progress    */
} WPSR_STATE_T;

/* WPS Registration Protocol States of Registrar */
typedef enum
{
    REG_IDLE_ST   = 0,  /* When Registrar is Idle                            */
    REG_EAP_ID_ST = 1,  /* EAP-Identity State. Sent REQ/ID wait for RSP/ID   */
    REG_INIT_ST   = 2,  /* Initiation State. Send WSC_START wait for M1      */
    REG_MSG_ST    = 3   /* WPS Registration Message handshake state          */
} WPSR_REG_PROT_T;

/* WPS Registration Protocol Message States of Registrar */
typedef enum
{
    IDLE_ST = 0, /* In IDLE state                                            */
    M2_ST   = 1, /* On transmission of M2  Message, waiting for M3/ACK/NACK  */
    M2D_ST  = 2, /* On transmission of M2D Message, waiting for ACK/NACK     */
    M4_ST   = 3, /* On transmission of M4  Message, waiting for M5/ACK/NACK  */
    M6_ST   = 4, /* On transmission of M6  Message, waiting for M7/ACK/NACK  */
    M8_ST   = 5, /* On transmission of M8  Message, waiting for ACK/NACK/DONE*/
    NACK_ST = 6, /* On transmission of NACK, waiting for NACK                */

}WPSR_MSG_STATE_T;

/* WPS Status */
typedef enum
{
    WPS_REG_START_REQ           = 0x00,
    WPS_REG_PBC_SESSION_OVERLAP = 0x01,
    WPS_REG_REGPROT_START       = 0x02,
    WPS_REG_M2D_TX              = 0x03,
    WPS_REG_NACK_RX_FAIL        = 0x04,
    WPS_REG_PROT_COMP_SUCCESS   = 0x05,
    WPS_REG_RXED_WRONG_M3       = 0x07,
    WPS_REG_RXED_WRONG_M5       = 0x08,
    WPS_REG_RXED_WRONG_M7       = 0x09,
    WPS_REG_EAP_MSG_TIMEOUT     = 0x0A,
    WPS_REG_CONF_METH_MISMATCH  = 0x0B,
    WPS_REG_CONN_TYPE_MISMATCH  = 0x0C,
    WPS_REG_AUTH_TYPE_MISMATCH  = 0x0D,
    WPS_REG_ENCR_TYPE_MISMATCH  = 0x0E,
    WPS_REG_UNEXP_MSG_M3        = 0x0F,
    WPS_REG_UNEXP_MSG_M5        = 0x10,
    WPS_REG_UNEXP_MSG_M7        = 0x11,
    WPS_REG_PASS_ID_MISMATCH    = 0x12,
    WPS_REG_EAP_MSG_RETRY_LIMIT = 0x13,
    WPS_REG_EAP_BUFFER_NULL     = 0x14,
    WPS_REG_M2D_SUCCESS         = 0x15,
    WPS_REG_MULTIPLE_REG_ATTEMPT= 0x16,
    WPS_REG_PIN_FAILURE_M5      = 0x17,
    WPS_REG_PIN_FAILURE_M7      = 0x18,
    WPS_REG_REG_PROT_WALK_TIMEOUT= 0x19,
    WPS_REG_ENCR_EXC_WEP        = 0x20, /* WPS is run in WEP mode security   */
    WPS_REG_ENCR_EXC_TKIP       = 0x21, /* WPS is run in TKIP-only mode      */
    WPS_REG_AUTH_EXC_SHARED     = 0x22, /* WPS in Shared authentication mode */
    WPS_REG_AUTH_EXC_WPA        = 0x23, /* WPS is run in WPA-Only mode       */
    WPS_REG_MONITOR_EXC1        = 0x24, /* Enr info not found on RP comp     */
    WPS_REG_MONITOR_EXC2        = 0x25, /* UUID mismatch on M1 RX            */
    WPS_REG_RXED_WRONG_KWA_M5   = 0x26, /* KWA check failure in M5 message   */
    WPS_REG_RXED_WRONG_KWA_M7   = 0x27, /* KWA check failure in M7 message   */
    WPS_REG_WSC_IE_REASSEM_EXC  = 0x28, /* Reassembly buffer size exception  */
} WPSR_STATUS_T;

/* WPS Subelement IDs */
typedef enum
{
    WPS_SUB_ELEM_VER2            = 0x00,
    WPS_SUB_ELEM_AUTH_MACS       = 0x01,
    WPS_SUB_ELEM_NW_KEY_SHARABLE = 0x02,
    WPS_SUB_ELEM_REQ_2_ENROLL    = 0x03,
    WPS_SUB_ELEM_DELAY_TIME      = 0x04,
} WPSR_SUB_ELEM_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

/* This Structure is used to store big numbers */
typedef struct
{
    UWORD32 *val_ptr; /* Pointer ot the array of big number                  */
    UWORD16 length;   /* Length in WORD(4 bytes) of the big number           */
    UWORD16 msb_pos;  /* MSB position in of the big number                   */
} long_num_struct_t;

/* Network Key information structure */
typedef struct
{
    UWORD8  net_key_index;            /* Key index                           */
    UWORD8  net_key[MAX_NET_KEY_LEN]; /* Key Value                           */
    UWORD8  net_key_len;              /* Key Length                          */
    UWORD8  mac_addr[MAC_ADDRESS_LEN];/* Member Device MAC address           */

} wps_net_key_t;

/* This structure is used by WPS registrar to get Credential from SS */
typedef struct
{
    UWORD8  ssid_len;            /* SSID Length                              */
    UWORD8  ssid[MAX_ESSID_LEN]; /* SSID of the Network                      */
    UWORD8  num_key;             /* Number of Network Key Information        */
    UWORD16 auth_type;           /* Authentciation Type as defined in std    */
    UWORD16 encr_type;           /* Encryption Type as defined in std        */
    wps_net_key_t net_key_info[MAX_NW_KEY_PER_CRED]; /* N/w Key Information  */
} wps_cred_t;

/* This Structure is used to hold the WFA Vendor Extenstion subelement       */
typedef struct
{
    UWORD8 id;
    UWORD8 len;
    UWORD8 val[MAX_SUB_ELEM_LEN];
} wps_sub_elem_t;

/* Structure used in processing of received EAP messages */
typedef struct
{
    BOOL_T  defrag_in_prog; /* Flag to indicate EAP Defragmentation          */
    UWORD16 rx_eap_msg_len; /* Length of the received EAP message            */
    UWORD16 rx_m_msg_len;   /* Length of the received WSC_MSG frame          */
    UWORD8  *rx_eap_msg;    /* Pointer to received EAP frame                 */
    UWORD8  *rx_m_msg_ptr;  /* Pointer to start of WSC_MSG of EAP frame      */
} wps_eap_rx_t;

/* Structure used in transmission of EAP messages */
typedef struct
{
    UWORD8  tx_eap_id;         /* EAP ID of transmitted EAP frame            */
    UWORD8  eap_msg_retry_cnt; /* Re-transmission Counter for EAP frames     */
    UWORD16 tx_m_msg_len;      /* Length of WSC_MSG part in TXed EAP         */
    UWORD8  *tx_m_msg_ptr;     /* Pointer to Start of WSC-MSG of TXed EAP frm*/
    UWORD8  *tx_wlan_frm_buff; /* Buffer pointer to transmitted EAP frame    */
} wps_eap_tx_t;

/* WPS Configuration Structure. This has all the WSC attributes configured by*/
/* the user/AP                                                               */
typedef struct
{
    WPS_PROT_TYPE_T prot_type;        /* WPS Protocol Type 1=>PBC and 2=>PIN */
    UWORD8  dev_pin_len;              /* Device PIN length                   */
    UWORD8  conn_type_flags;          /* BSS/IBSS Connection Type supported  */
    UWORD8  wps_state;                /* Wi-Fi Protected Setup State         */
    UWORD8  ap_setup_locked;          /* AP Setup Locked State               */
    UWORD8  rf_bands;                 /* RF Bands available on the Device    */
    UWORD8  rsp_type;                 /* Response Type included in responses */
    UWORD8  manufacturer_len;         /* Length of Manufacturer              */
    UWORD8  model_name_len;           /* Length of Model Name                */
    UWORD8  model_num_len;            /* Length of Model Number              */
    UWORD8  device_name_len;          /* Length of Device Name               */
    UWORD8  serial_num_len;           /* Length of Device serial number      */
    UWORD8  auth_macs_cnt;            /* No of AuthorizedMACs Subelement     */
    UWORD8  mac_addr[MAC_ADDRESS_LEN];/* Enrollee`s MAC address              */
    UWORD8  dev_pin[MAX_WPS_PIN_LEN]; /* Device PIN Programmed               */
    UWORD16 dev_pass_id;              /* WPS Device Password-ID              */
    UWORD16 config_methods;           /* WPS config methods supported        */
    UWORD16 auth_type_flags;          /* Authentication Type Supported       */
    UWORD16 encr_type_flags;          /* Encryption Type Supported           */
    UWORD32 os_version;               /* OS Version running on the device    */
    UWORD8  *prim_dev_cat_id;         /* Primary Device Category ID          */
#ifdef MAC_P2P
    UWORD8* req_dev_cat_id;           /* Requested Device Category ID        */
#endif /* MAC_P2P */
    UWORD8  *manufacturer;            /* Device manufacturer in ASCII        */
    UWORD8  *model_name;              /* Device Model Name in ASCII string   */
    UWORD8  *model_num;               /* Device Model Number                 */
    UWORD8  *device_name;             /* Device Name                         */
    UWORD8  *serial_num;              /* Serial number of the Registrar      */
    wps_sub_elem_t version2;          /* Version 2 Subelement                */
    wps_sub_elem_t auth_macs;         /* AuthorizedMACs Subelement           */
} wps_config_t;

typedef union
{
    UWORD32 pk_u32[(WPS_PUBLIC_KEY_LEN+3)/4];
    UWORD8  pk[WPS_PUBLIC_KEY_LEN];
} pub_key_t;

/* This structure holds all the WSC attributes that are not configured by the*/
/* USER                                                                      */
typedef struct
{
    pub_key_t reg_pub_key;                  /* Registrar`s Public Key, PKr   */
    pub_key_t enr_pub_key;                  /* Enrollee`s  Public Key, PKe   */
    BOOL_T sel_reg;                         /* Selected Registrar            */
    UWORD8 assoc_state;                     /* Association Status            */
    UWORD8 config_error;                    /* Configuration Error           */
    UWORD8 reg_uuid[WPS_UUID_LEN];          /* Registrar`s UUID, UUID-R      */
    UWORD8 enr_uuid[WPS_UUID_LEN];          /* Enrollee`s UUID, UUID-E       */
    UWORD8 enr_nonce[WPS_NONCE_LEN];        /* WPS Enrollee`s Nonce, N1      */
    UWORD8 reg_nonce[WPS_NONCE_LEN];        /* WPS Registrar`s Nonce, N2     */
    UWORD8 snonce1[WPS_NONCE_LEN];          /* PIN Nonce-1 ,R-S1/E-S1        */
    UWORD8 snonce2[WPS_NONCE_LEN];          /* PIN Nonce-2,R-S2/E-S2         */
} wps_attr_t;

/* Structure to hold all the secret keys generated                           */
typedef struct
{
    UWORD8 auth_key[WPS_AUTH_KEY_LEN];    /* Authentication Key              */
    UWORD8 key_wrap_key[WPS_KWK_KEY_LEN]; /* KeyWrapKey for AES encrp-decrp  */
    UWORD8 psk1[WPS_PSK_LEN];             /* Hash of 1st Half of PIN         */
    UWORD8 psk2[WPS_PSK_LEN];             /* Hash of 2nd Half of PIN         */
} wps_sec_key_t;

/* Private Key Overlapping Structure */
typedef union
{
    UWORD32 priv_key[WPS_PRIVATE_KEY_LEN_WORD32];/* Diffie-Hellman Private(A/B)*/
    wps_sec_key_t secret_keys;            /* Secret Keys Structure           */
} olap_priv_key_t;

/* This is a WPS Private Structure that holds all the DH/PIN/Privater Keys   */
typedef struct
{
    olap_priv_key_t   olap_priv_key;      /* Overlaping Private Key Structure*/
    long_num_struct_t priv_key_ptr;       /* Handle to private key buffer    */
} wps_priv_t;

/* This structure has information regarding the PBC enrollees. This is used  */
/* for monitor window of the registrar to detect session overlap             */
typedef struct
{
    UWORD8  uuid_e[WPS_UUID_LEN]; /* Enrollee UUID-E in Probe Request        */
    UWORD8  mac_addr[6];          /* MAC address of enrollee in PBC mode     */
    UWORD32 tsf_timer_hi;         /* TSF time stamp on detecting a PBC enr   */
    UWORD32 tsf_timer_lo;         /* TSF time stamp on detecting a PBC enr   */
} wps_mon_pbc_t;

/* Structure to hold WPS Registrar Specific information */
typedef struct
{
    BOOL_T start_req;              /* Start Request from user                */
    BOOL_T ver2_cap_sta;           /* Flag to indicate Version2.0 capable STA*/
    UWORD8 pbc_enr_cnt;            /* Counter for no of enrollees in PBC mode*/
    UWORD8 enr_hash1[WPS_HASH_LEN];/* Enrollee`s 1st half of PIN Hash,E-Hash1*/
    UWORD8 enr_hash2[WPS_HASH_LEN];/* Enrollee`s 2nd half of PIN Hash,E-Hash2*/
    wps_mon_pbc_t pbc_enr_info[2]; /* PBC Enrollee information               */
    WPSR_STATE_T wpsr_state;       /* State of the Registrar                 */
    WPSR_REG_PROT_T reg_prot_state;/* State of the Registration Protocol     */
    WPSR_MSG_STATE_T msg_state;    /* Message state of Registration Protocol */
} wps_reg_t;

/* This is the main WPS Context Structure that holds all the WPS information */
typedef struct
{
    wps_eap_rx_t *wps_eap_rx_hdl;    /* Handle for EAP-RX structure          */
    wps_eap_tx_t *wps_eap_tx_hdl;    /* Handle for EAP-TX Structure          */
    wps_config_t *wps_config_hdl;    /* Handle to WPS Configuration Structure*/
    wps_priv_t   *wps_priv_hdl;      /* Handle to WPS Private Structure      */
    wps_attr_t   *wps_attr_hdl;      /* Handle to WPS Attribute Structure    */
    void         *reg_prot_timer;    /* Timer for Registration Protocol TO   */
    void         *eap_msg_timer;     /* Timer for EAP-TX Timeout             */
	// 20120709 caisf add, merged ittiam mac v1.2 code
    void         *deauth_timer;      /* Timer for Deauth Timeout             */
    WPS_ALARM_FUNC_T *wps_timer_cb_fn_ptr; /* Timer callback function pointer*/
    wps_reg_t    *wps_reg_hdl;       /* Handle to Registrar structure        */
} wps_t;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern void wps_init_registrar(wps_t *wps_ctxt_hdl);
extern STATUS_T wps_start_registrar(wps_t *wps_ctxt_hdl);
extern void wps_stop_registrar(wps_t *wps_ctxt_hdl, BOOL_T start_req);
extern UWORD16 wps_insert_wsc_ie(wps_t *wps_ctxt_hdl, UWORD8 *frame_ptr,
                                 UWORD8 frm_type);
extern void wps_process_probe_req(wps_t *wps_ctxt_hdl, UWORD8 *msa, UWORD8 *sa,
                                  UWORD16 rx_len, UWORD8 frm_type);
extern STATUS_T wps_handle_eap_rx(wps_t *wps_ctxt_hdl, UWORD8* msa,
                                  UWORD16 rx_len);
extern void wps_handle_event(wps_t *wps_ctxt_hdl, UWORD32 inp_data);
extern BOOL_T is_wps_in_prog_reg(wps_t *wps_ctxt_hdl);
extern void wps_set_ver2_cap_sta(wps_t *wps_ctxt_hdl, BOOL_T val);
extern BOOL_T check_wps_cap_assoc_req(wps_t *wps_ctxt_hdl, UWORD8 *msa,
                                      UWORD16 rx_len, UWORD8 frm_type,
                                      BOOL_T *wps_cap_sta,
                                      BOOL_T *wps_ver2_cap_sta);

/* Functions from MAC SW */
extern BOOL_T send_eapol(UWORD8 *addr, UWORD8 *buffer, UWORD16 len,
                         BOOL_T secured);
extern UWORD8 get_random_byte(void);
extern void sha1(UWORD8 *message, UWORD32 message_length, UWORD8 *digest);
extern void wps_get_tsf_timestamp(UWORD32 *tsf_timer_hi,
                                  UWORD32 *tsf_timer_lo);
extern void wps_update_beacon_wsc_ie_ap(void);
extern void wps_update_failed_enr_list(UWORD8* mac_addr);

/* Memory related functions */
extern void* wps_pkt_mem_alloc(UWORD16 buf_size);
extern void* wps_local_mem_alloc(UWORD16 buf_size);
extern void wps_pkt_mem_free(void *buffer);
extern void wps_local_mem_free(void *buffer);
extern UWORD8 wps_get_num_users(void *buffer);
extern void wps_add_mem_users(void *buffer, UWORD8 num_user);

/* Alarm related functions */
extern void* wps_create_alarm(WPS_ALARM_FUNC_T *func, UWORD32 data, ALARM_FUNC_WORK_T *work_func);
extern void wps_delete_alarm(void** handle);
extern BOOL_T wps_start_alarm(void* handle, UWORD32 timeout_ms);
extern void wps_stop_alarm(void* handle);

extern void sys_stop_wps_reg(UWORD16 reason_code, UWORD8 *mac_addr,
                             BOOL_T start_req);
extern void wps_handle_sys_err_reg(UWORD16 sys_error);

extern void send_wps_status(UWORD8 status_code, UWORD8* msg_ptr,
                            UWORD16 msg_len);
extern BOOL_T wps_get_cur_cred_ap(wps_cred_t *wps_cred, UWORD8 cred_index);
extern UWORD8 wps_get_num_cred_ap(void);


#ifdef MAC_P2P
extern UWORD16 wps_get_config_method(UWORD8 *msa, UWORD16 rx_len);
extern UWORD8 wps_add_config_method_ie(UWORD8 *frame_ptr, UWORD8 index,
                                            UWORD16 config_meth);
extern UWORD8 wps_add_go_neg_frm_wsc_ie(UWORD8 *frame_ptr, UWORD16 index,
                                        WPS_PROT_TYPE_T prot_type,
                                        UWORD16 dev_pas_id);
extern BOOL_T wps_process_p2p_prob_req_reg(wps_config_t *config_ptr,
                                           UWORD8 *msa, UWORD16 rx_len,
                                           UWORD8 *req_dev_type);
#endif /* MAC_P2P */

#endif /* WPS_REG_IF_H */
#endif /* INT_WPS_REG_SUPP */
