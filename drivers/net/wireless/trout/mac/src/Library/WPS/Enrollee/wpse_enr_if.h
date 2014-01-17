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
/*  File Name         : wpse_enr_if.h                                        */
/*                                                                           */
/*  Description       : This file contains all the data type definitions for */
/*                      WPS Enrollee Protocol. It has function definitions of*/
/*                      APIs provided by WPS enrollee and APIs required by   */
/*                      WPS Enrollee.                                        */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*  Revision History  :                                                      */
/*                                                                           */
/*         DD MM YYYY   Author(s)       Changes                              */
/*         18 03 2010   Ittiam          Draft                                */
/*                                                                           */
/*****************************************************************************/
#ifdef INT_WPS_ENR_SUPP

#ifndef WPS_ENR_IF_H
#define WPS_ENR_IF_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/
#include "wpse_common.h"
#include "csl_types.h"

/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/
/*****************************************************************************/
/* If defined, WPS_EXT_REG_SEL_REG_BUG_FIX enables Enrollee to do WPS        */
/* protocol with an AP even if Selected Registrar field is not set. Even     */
/* though first iteration will fail, external registrar will discover the    */
/* Enrollee and will be ready to register the Enrollee after some time       */
/*****************************************************************************/
#define WPS_EXT_REG_SEL_REG_BUG_FIX

/*****************************************************************************/
/* This enables the work around for bug in Wi-Fi Test bed Ralink AP when     */
/* using Windows Connect Now(WCN) Wired External Registrar. The bug is that  */
/* during Registration Protocol Enrollee Nonce included by WCN in M2 message */
/* is from the previous registartion protocol sessions. Multiple M2D         */
/* registration protocols are run when WPS_EXT_REG_SEL_REG_BUG_FIX flag is   */
/* enabled. So as part of this workaround the Enrollee nonce is kept same    */
/* for 2 minute WPS protocol window.                                         */
/*****************************************************************************/
//#define RALINK_AP_WCN_EXT_REG_BUG_FIX

/*****************************************************************************/
/* This flag enables generation of PKE as part of WPS start request from the */
/* user. When this flag is enabled Enrollee`s Public Key, PKe, is kept the   */
/* for entire durartion of 2 minute WPS protocol window. With this change the*/
/* time taken to respond with an M1 for an EAP-WSC-Start is substantialy     */
/* reduced,                                                                  */
/*****************************************************************************/
//#define OPTIMIZED_PKE_GENERATION

#define SHA1_DIGEST_LEN         20
#define WPS_VERSION             0x10
#define WPS_NONCE_LEN           16
#define WPS_UUID_LEN            16
#define MAX_REG_PER_AP          2
#define MAX_WSC_IE_CNT          3
#define MAX_DIS_REG_CNT         10
#define WPS_PSK_LEN             16
#define WPS_AUTH_KEY_LEN        32
#define WPS_KEY_WRAP_KEY_LEN    16
#define WPS_EMSK_LEN            32
#define SHA_256_HASH_LEN        32
#define WPS_DH_PRIME_MSB_POS    1536
#define WPS_DH_PRIME_LEN_WORD32 (WPS_DH_PRIME_MSB_POS/32)
#define WPS_DH_PRIME_LEN_BYTE   (WPS_DH_PRIME_LEN_WORD32*4)
#define WPS_PUBLIC_KEY_LEN      WPS_DH_PRIME_LEN_BYTE
#define WPS_PRIVATE_KEY_LEN_WORD32     (64/32)
#define WPS_PRIVATE_KEY_LEN     (WPS_PRIVATE_KEY_LEN_WORD32*4)
#define MAX_NET_KEY_LEN         64
#define MAX_NW_KEY_PER_CRED     1
#define WPS_IE_ID               221
#define WPS_IE_OUI_BYTE0        0x00
#define WPS_IE_OUI_BYTE1        0x50
#define WPS_IE_OUI_BYTE2        0xF2
#define WPS_IE_OUI_BYTE3        0x04
#define WPS_UUID_LEN            16
#define WPS_MAX_MANUFACT_LEN    64
#define WPS_MAX_MODEL_NAME_LEN  32
#define WPS_MAX_MODEL_NUM_LEN   32
#define WPS_MAX_SERIAL_NUM_LEN  32
#define WPS_MAX_DEVICE_NAME_LEN 32

#define WPS_MAX_RX_EAP_MSG_LEN  1400
#define WPS_MAX_TX_EAP_MSG_LEN  1000
#define MAX_ESSID_LEN 32
#define WPS_MAC_ADDR_LEN        6
#define PKR_LEN                 (MAX(WPS_DH_PRIME_LEN_BYTE, \
                                ((2*WPS_HASH_LEN + WPS_AUTH_KEY_LEN + \
                                WPS_KEY_WRAP_KEY_LEN + \
                                WPS_EMSK_LEN -1)/WPS_HASH_LEN)))
#define WPS_HASH_LEN            SHA_256_HASH_LEN
#define NSID_LEN                16
/*****************************************************************************/
/* Different types of Device Password IDs                                    */
/*****************************************************************************/
#define DEV_PASS_ID_DEFAULT     0x0000
#define DEV_PASS_ID_USER_SPE    0x0001
#define DEV_PASS_ID_MACH_SPE    0x0002
#define DEV_PASS_ID_REKEY       0x0003
#define DEV_PASS_ID_PUSHBUTT    0x0004
#define DEV_PASS_ID_REG_SPE     0x0005

#define MAX_WPS_PIN_LEN         16
#define WPS_PBC_PASS_LEN        8
#define MAX_WPS_PASS_LEN        (MAX(MAX_WPS_PIN_LEN, WPS_PBC_PASS_LEN))

#define WPS_AUTH_TYPE_FLAGS_MASK    0x7F
#define WPS_ENCR_TYPE_FLAGS_MASK    0x1F
#define MAX_AUTH_MACS           5
#define MAX_SUB_ELEM_LEN        (MAC_ADDRESS_LEN * MAX_AUTH_MACS)
#define MAX_SUB_ELEM_SUPP       2 /* Version2 and Authorized-MACs */
#define WPS_VER2_LEN            1
#define WPS_VER2_VAL            0x20
#define WPS_AUTH_MACS_LEN       MAC_ADDRESS_LEN
#define WPS_AUTH_MACS_VAL       0xFF

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/
/*****************************************************************************/
/* Data Types                                                                */
/*****************************************************************************/
#ifndef OS_LINUX_CSL_TYPE
typedef void    WPS_ALARM_FUNC_T(void* alarm, UWORD32 data);
#else /* OS_LINUX_CSL_TYPE */
typedef void    WPS_ALARM_FUNC_T(UWORD32 data);
#endif /* OS_LINUX_CSL_TYPE */
/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/
/*****************************************************************************/
/* WPS status for display                                                    */
/*****************************************************************************/
typedef enum
{
    IN_PROGRESS                 = (0x00),
    ASSOC_PASS                  = (IN_PROGRESS+1),
    REG_PROT_SUCC_COMP          = (ASSOC_PASS+1),
    /*************************************************************************/
    /* Error status                                                          */
    /*************************************************************************/
    ERR_ASSOC_FAIL              = (0x80),
    ERR_SYSTEM                  = (ERR_ASSOC_FAIL+1),   // 0x81
    ERR_WALK_TIMEOUT            = (ERR_SYSTEM+1),       // 0x82
    SESSION_OVERLAP_DETECTED    = (ERR_WALK_TIMEOUT+1), // 0x83
    ERR_PBC_REC_FAIL            = (SESSION_OVERLAP_DETECTED+1), // 0x84
    ERR_REC_FAIL                = (ERR_PBC_REC_FAIL+1),         // 0x85
    ERR_REC_NACK                = (ERR_REC_FAIL+1),             // 0x86
    ERR_DIFF_PASS_ID_M2         = (ERR_REC_NACK+1),             // 0x87
    ERR_REC_WRONG_M2            = (ERR_DIFF_PASS_ID_M2+1),      // 0x88
    REC_M2D                     = (ERR_REC_WRONG_M2+1),         // 0x89
    ERR_REC_WRONG_M4            = (REC_M2D+1),
    ERR_REC_WRONG_M6            = (ERR_REC_WRONG_M4+1),
    ERR_REC_WRONG_M8            = (ERR_REC_WRONG_M6+1),
    ERR_REG_MSG_TIMEOUT         = (ERR_REC_WRONG_M8+1),
    ERR_PBC_REG_MSG_TIMEOUT     = (ERR_REG_MSG_TIMEOUT+1),
    ERR_REG_PROT_TIMEOUT        = (ERR_PBC_REG_MSG_TIMEOUT+1),
    ERR_STA_DISCONNECT          = (ERR_REG_PROT_TIMEOUT+1),
    ERR_WPS_INIT_FAIL           = (ERR_STA_DISCONNECT + 1),
    ERR_WSC_IE_REASSEM_EXC      = (ERR_WPS_INIT_FAIL + 1),

    /*************************************************************************/
    /* Configuration message status                                          */
    /*************************************************************************/
    RCV_CRED_VALUE              = (0x40),
    CRED_JOIN_FAILURE           = (RCV_CRED_VALUE+1),
    CRED_JOIN_SUCCESS           = (CRED_JOIN_FAILURE+1),
    CRED_JOIN_LIST_NULL         = (CRED_JOIN_SUCCESS+1),
    REC_INVALID_CRED_JOIN_LIST  = (CRED_JOIN_LIST_NULL+1),
    WLAN_DIS_WPS_PROT           = (0xC0)
} WPS_STATUS_T;
/*****************************************************************************/
/* Overall WPS protocol state                                                */
/*****************************************************************************/
typedef enum
{
    IN_SCAN         = 0,
    IN_JOIN         = (IN_SCAN+1),
    IN_REG_PROT     = (IN_JOIN + 1)
} WPS_STATE_T;

/*****************************************************************************/
/* State of WPS protocol after connection with AP/Registrar is established   */
/*****************************************************************************/
typedef enum
{
    EAPOL_START_ST  = 0,
    RES_ID_ST       = 1,
    MSG_ST          = 2,
    WAIT_FACK_ST    = 3
} WPS_REG_STATE_T;

/*****************************************************************************/
/* State of WPS Protocol during Mx Message exchange                          */
/*****************************************************************************/
typedef enum
{
    NACK_ST = 0,
    M1_ST   = 1,
    M3_ST   = 2,
    M5_ST   = 3,
    M7_ST   = 4,
    DONE_ST = 5
} WPS_MSG_STATE_T;

/*****************************************************************************/
/* Type of WPS protocol. currently only two PIN and PBC are supported        */
/*****************************************************************************/
typedef enum
{
    WPS_PROT_NONE = 0,
    PIN = (WPS_PROT_NONE+1),
    PBC = (PIN+1)
} WPS_PROT_TYPE_T;

/*****************************************************************************/
/* WPS action that MAC needs to take                                         */
/*****************************************************************************/
typedef enum
{
    WPS_INVALID,
    SKIP_FRAME,
    KEEP_FRAME,
    CONT_SCAN,
    JOIN_NW
} WPS_ACTION_T;

/*****************************************************************************/
/* WPS Association request status                                            */
/*****************************************************************************/
typedef enum
{
    WPS_ASSOC_FAIL,
    WPS_ASSOC_TIMEOUT,
    WPS_ASSOC_SUCCESS,
    WPS_ASSOC_FAILURE
} WPS_ASSOC_STATUS_T;


/*****************************************************************************/
/* WPS Association Status values as defined in WPS standard                  */
/*****************************************************************************/
typedef enum
{
    WPS_NOT_ASSOCIATED          = 0,
    WPS_CONNECTION_SUCCESS      = 1,
    WPS_CONFIGURATION_FAILURE   = 2,
    WPS_ASSOCIATION_FAILURE     = 3,
    WPS_IP_FAILURE              = 4
} WPS_ASSOC_STATE_T;
/*****************************************************************************/
/* WPS Credentials Encryption Type                                           */
/*****************************************************************************/
typedef enum
{
    WPS_NONE        = 0x0001,
    WPS_WEP         = 0x0002,
    WPS_TKIP        = 0x0004,
    WPS_AES         = 0x0008,
    WPS_AES_TKIP    = 0x000C,
    WPS_AES_TKIP_MIXED = 0x0040  /* Not valid for WSC 2.0 */
} WPS_CRED_ENC_TYPE_T;

/*****************************************************************************/
/* WPS Credentials Authentication Type                                       */
/*****************************************************************************/
typedef enum
{
    WPS_OPEN            = 0x0001,
    WPS_WPA_PSK         = 0x0002,
    WPS_SHARED          = 0x0004,
    WPS_WPA             = 0x0008,
    WPS_WPA2            = 0x0010,
    WPS_WPA2_PSK        = 0x0020,
    WPS_WPA_WPA2_MIXED  = 0x0040, /* Not valid for WSC 2.0 */
    WPS_WPA_WPA2_PSK    = 0x0022
} WPS_CRED_AUTH_TYPE_T;
/*****************************************************************************/
/* WPS Configuration Error values as defined in WPS standard                 */
/*****************************************************************************/
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

/*****************************************************************************/
/* WPS Request Type                                                          */
/*****************************************************************************/
typedef enum
{
    ENROLLEE_INFO_ONLY          = 0x00,
    ENROLLEE_OPEN_802_1X        = 0x01,
    REGISTRAR                   = 0x02,
    WLAN_MANAGER_REGISTRAR      = 0x03

} WPS_REQUEST_TYPE_T;

/* WPS Subelement IDs */
typedef enum
{
    WPS_SUB_ELEM_VER2            = 0x00,
    WPS_SUB_ELEM_AUTH_MACS       = 0x01,
    WPS_SUB_ELEM_NW_KEY_SHARABLE = 0x02,
    WPS_SUB_ELEM_REQ_2_ENROLL    = 0x03,
    WPS_SUB_ELEM_DELAY_TIME      = 0x04,
} WPSE_SUB_ELEM_T;

/*****************************************************************************/
/* Structure used to send M2d information to Host/user                       */
/*****************************************************************************/
typedef struct
{
    UWORD8  uuid_r[WPS_UUID_LEN];
    UWORD8  manufacturer[WPS_MAX_MANUFACT_LEN];
    UWORD8  manufacturer_len;
    UWORD8  model_name[WPS_MAX_MODEL_NAME_LEN];
    UWORD8  model_name_len;
    UWORD8  model_number[WPS_MAX_MODEL_NUM_LEN];
    UWORD8  model_number_len;
    UWORD8  serial_number[WPS_MAX_SERIAL_NUM_LEN];
    UWORD8  serial_number_len;
    UWORD8  device_name[WPS_MAX_DEVICE_NAME_LEN];
    UWORD8  device_name_len;
} wps_m2d_dis_info_t;

/*****************************************************************************/
/* Structure used to send NACK information to Host/user                      */
/*****************************************************************************/
typedef struct
{
    UWORD8  uuid_r[WPS_UUID_LEN];
    UWORD16 config_error;
} wps_nack_dis_info_t;

/*****************************************************************************/
/* This Structure has information of a AP/registrar selected for WPS Protocol*/
/*****************************************************************************/
typedef struct
{

    UWORD8  bssid[WPS_MAC_ADDR_LEN];
    /*************************************************************************/
    /* These are disabled as changed design does not store these fields      */
    /*************************************************************************/
#if 0
    UWORD8  ssid[MAX_ESSID_LEN];
    UWORD16 dev_pass_id[MAX_REG_PER_AP];
    UWORD16 sel_reg_config_meth[MAX_REG_PER_AP];
    UWORD8  ssid_len;
    UWORD8  bss_type;
    UWORD8  res_type;
#endif
    UWORD8  reg_num;
    BOOL_T  sel_reg;
} wps_ap_info_struct_t;

/*****************************************************************************/
/* This Structure has information of a AP/registrar discarded for WPS Protocol*/
/*****************************************************************************/
typedef struct
{

    UWORD8 bssid[WPS_MAC_ADDR_LEN];
} wps_dis_ap_reg_info_struct_t;

/*****************************************************************************/
/* This Structure is used to store big numbers                               */
/*****************************************************************************/
typedef struct
{
    /*************************************************************************/
    /* This variable will point to the array storing the big number          */
    /*************************************************************************/
    UWORD32 *val_ptr;
    /*************************************************************************/
    /* This variable will store the number of UWORD32 words allocated        */
    /*************************************************************************/
    UWORD16 length;
    /*************************************************************************/
    /* This variable will store the bit position of most significant bit(0:) */
    /*************************************************************************/
    UWORD16 msb_pos;
} long_num_struct_t;

/*****************************************************************************/
/* This structure has buffers that can be overlapped with A nonce buffer     */
/*****************************************************************************/
typedef struct
{
    UWORD8 e_snonce1[WPS_NONCE_LEN];
    UWORD8 e_snonce2[WPS_NONCE_LEN];
    UWORD8 psk1[WPS_PSK_LEN];
    UWORD8 psk2[WPS_PSK_LEN];
    UWORD8 r_hash1[WPS_HASH_LEN];
    UWORD8 r_hash2[WPS_HASH_LEN];
} wps_nonce_hash_struct_t;

/*****************************************************************************/
/* PKR buffer is also used as temporary buffer during key generation.        */
/* define maximum size that is required.                                     */
/*****************************************************************************/
typedef union
{
    UWORD32 pkr_u32[(PKR_LEN+3)/4];
    UWORD8  pkr[PKR_LEN];
} pub_key_t;

/*****************************************************************************/
/* Overlap A Nonce buffer and Hash and Nonce used after A nonce is not       */
/* required                                                                  */
/*****************************************************************************/
typedef union
{
    UWORD32 a_nonce_buff[WPS_PRIVATE_KEY_LEN_WORD32];
    wps_nonce_hash_struct_t nonce_hash;
}non_olap_t;


/* This Structure is used to hold the WFA Vendor Extenstion subelement       */
typedef struct
{
    UWORD8 id;
    UWORD8 len;
    UWORD8 val[MAX_SUB_ELEM_LEN];
} wps_sub_elem_t;

/*****************************************************************************/
/* This structure has all the configuration data required by WPS Enrollee    */
/*****************************************************************************/
typedef struct
{
    WPS_PROT_TYPE_T prot_type;
    UWORD16 dev_pass_id;
    UWORD8* dev_pass;
    UWORD8  dev_pass_len;
    UWORD8* mac_address;
    UWORD16 config_meth;
    /*************************************************************************/
    /* 0x1- ESS, 0x2 - IBSS                                                  */
    /*************************************************************************/
    UWORD8  con_type_flag;
    /*************************************************************************/
    /* Category ID, OUI and sub category ID in sequence                      */
    /*************************************************************************/
    UWORD8* prim_dev_cat_id;
    UWORD16 auth_type_flags;
    UWORD16 enc_type_flags;
    UWORD8* manufacturer;
    /*************************************************************************/
    /* Length should be <= 64 bytes, no check will be done                   */
    /*************************************************************************/
    UWORD8  manufacturer_len;
    UWORD8* model_name;
    /*************************************************************************/
    /* Length should be <= 32 bytes, no check will be done                   */
    /*************************************************************************/
    UWORD8  model_name_len;
    UWORD8* model_num;
    /*************************************************************************/
    /* Length should be <= 32 bytes, no check will be done                   */
    /*************************************************************************/
    UWORD8  model_num_len;
    UWORD8* dev_name;
    /*************************************************************************/
    /* Length should be <= 32 bytes, no check will be done                   */
    /*************************************************************************/
    UWORD8  dev_name_len;
    UWORD32 os_version;
    UWORD8* serial_num;
    /*************************************************************************/
    /* Length should be <= 32 bytes, no check will be done                   */
    /*************************************************************************/
    UWORD8  serial_num_len;
    UWORD8  wps_state;
    UWORD8  rf_bands;
    /*************************************************************************/
    /* Pointer to function that shall be passed as callback function for     */
    /* timers to call.                                                       */
    /*************************************************************************/
    WPS_ALARM_FUNC_T* timer_cb_fn_ptr;
    wps_sub_elem_t version2;          /* Version 2 Subelement                */
#ifdef MAC_P2P
    UWORD8* req_dev_cat_id;
    UWORD16 pref_config_method;
#endif /* MAC_P2P */
} wps_enr_config_struct_t;

/*****************************************************************************/
/* This structure is WPS Enrollee persistent data structure.                 */
/*****************************************************************************/
typedef struct
{
    wps_enr_config_struct_t* config_ptr;
    BOOL_T  process_walk_to;
    WPS_STATE_T state;
    WPS_REG_STATE_T reg_state;
    WPS_MSG_STATE_T msg_st;
    wps_ap_info_struct_t sel_ap_info;
    wps_dis_ap_reg_info_struct_t dis_ap_reg_list[MAX_DIS_REG_CNT];
#ifdef WPS_EXT_REG_SEL_REG_BUG_FIX
    BOOL_T ignore_sel_reg_field;
#endif/* WPS_EXT_REG_SEL_REG_BUG_FIX */
    UWORD8 discarded_reg_cnt;
    UWORD8 scan_cnt;
    UWORD16 assoc_state;
    UWORD16 config_error;
    UWORD8 eapol_start_cnt;
    UWORD8 op_code;
    UWORD8 eap_received_id;
    UWORD8 enr_nonce[WPS_NONCE_LEN];
    UWORD8 reg_nonce[WPS_NONCE_LEN];
    UWORD8 reg_uuid[WPS_UUID_LEN];
    UWORD8 enr_uuid[WPS_UUID_LEN];
    WPS_REQUEST_TYPE_T req_type;
    long_num_struct_t a_nonce;
    long_num_struct_t pke_big;
    UWORD8 pke[WPS_DH_PRIME_LEN_BYTE];
    pub_key_t pub_key;
    non_olap_t non_olap;

    UWORD8 auth_key[WPS_AUTH_KEY_LEN];
    UWORD8 key_wrap_key[WPS_KEY_WRAP_KEY_LEN];

    void* walk_timer;
    void* reg_msg_timer;
    void* reg_prot_timer;
    UWORD8* rx_eap_msg;
    /*************************************************************************/
    /* If processing of fragmented received EAP packets supported. This can  */
    /* be dynamically allocated from packet memory as well.                  */
    /*************************************************************************/
    UWORD16 rx_eap_msg_buff_len;
    UWORD16 rx_eap_msg_used_len;
    UWORD16 rx_m_msg_len;
    UWORD8* rx_m_msg_ptr;
    /*************************************************************************/
    /* This can be a packet buffer if the same can be locked.                */
    /*************************************************************************/
    UWORD8  *tx_eap_msg;
    UWORD16 tx_eap_txed_msg_len;
    UWORD8* tx_m_msg_ptr;
    UWORD16 tx_m_msg_len;
    UWORD8  *tx_wlan_frm_buff;
    UWORD16 eap_msg_frag_size;
    BOOL_T  ver2_cap_ap;
} wps_enrollee_t;

/*****************************************************************************/
/* This structure is used by WPS Enrollee to pass received credentials to    */
/* MAC                                                                       */
/*****************************************************************************/
typedef struct
{
    UWORD8  ssid[MAX_ESSID_LEN];
    UWORD8  ssid_len;
    UWORD16 auth_type;
    UWORD16 enc_type;
    UWORD8  net_key_index[MAX_NW_KEY_PER_CRED];
    UWORD8  net_key[MAX_NW_KEY_PER_CRED][MAX_NET_KEY_LEN];
    UWORD8  net_key_len[MAX_NW_KEY_PER_CRED];
} wps_cred_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/
/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/
extern STATUS_T wps_start_enrollee(wps_enrollee_t* wps_enrollee,
                            wps_enr_config_struct_t* config_ptr);
extern void wps_stop_enrollee(wps_enrollee_t* wps_enrollee);
extern UWORD16 wps_gen_wsc_ie(wps_enrollee_t* wps_enrollee,
                              UWORD8* buff_ptr, UWORD8 frm_type);
extern WPS_ACTION_T wps_process_scan_rsp(wps_enrollee_t* wps_enrollee,
                                   UWORD8* msg_ptr, UWORD16 msg_len);
extern void wps_handle_event(wps_enrollee_t* wps_enrollee_t, UWORD16 inp_data);
extern void wps_assoc_comp(wps_enrollee_t* wps_enrollee,
                    WPS_ASSOC_STATUS_T assoc_status);
extern WPS_ACTION_T wps_scan_complete(wps_enrollee_t* wps_enrollee);
extern void wps_process_eap_msg(wps_enrollee_t* wps_enrollee, UWORD8 *wps_msg,
                                UWORD16 wps_len);
extern STATUS_T wps_process_disconnect(wps_enrollee_t* wps_enrollee);

#ifdef MAC_P2P
extern BOOL_T wps_process_p2p_prob_req_enr(wps_enr_config_struct_t *config_ptr,
                                           wps_enrollee_t* wps_enrollee,
                                           UWORD8 *msa, UWORD16 rx_len);
extern void wps_init_enrollee(wps_enrollee_t* wps_enrollee,
                       wps_enr_config_struct_t* config_ptr);
extern UWORD16 p2p_add_wsc_ie(wps_enrollee_t *wps_enrollee_ptr,
                              UWORD8* frm_ptr, UWORD8 frm_type);
extern void wps_process_p2p_prob_rsp_enr(UWORD8 *msa, UWORD16 rx_len,
                                     UWORD16 *config);
extern UWORD16 wps_get_config_method(UWORD8 *msa, UWORD16 rx_len);
extern UWORD8 wps_add_config_method_ie(UWORD8 *frame_ptr, UWORD8 index,
                                            UWORD16 config_meth);
extern UWORD8 wps_add_go_neg_frm_wsc_ie(UWORD8 *frame_ptr, UWORD16 index,
                                        WPS_PROT_TYPE_T prot_type,
                                        UWORD16 dev_pas_id);
#endif /* MAC_P2P */

/*****************************************************************************/
/* Functions from MAC & System                                               */
/*****************************************************************************/
extern void start_wps_scan(BOOL_T deauth_1x_req);
extern void send_wps_status(UWORD8 status_code, UWORD8* msg_ptr,
                            UWORD16 msg_len);
extern void end_wps_enrollee(WPS_STATUS_T status_code);
extern BOOL_T rec_wps_cred(wps_cred_t* wps_cred, BOOL_T ver2_cap_ap);
extern BOOL_T send_eapol(UWORD8 *addr, UWORD8 *buffer, UWORD16 len,
                         BOOL_T secured);
extern UWORD8 get_random_byte(void);
extern void sha1(UWORD8 *message, UWORD32 message_length, UWORD8 *digest);
/*****************************************************************************/
/* Memory related functions                                                  */
/*****************************************************************************/
extern void* wps_mem_alloc(UWORD16 size);
extern void wps_mem_free(void *buffer_addr);
extern void* wps_frame_mem_alloc(UWORD16 size);
extern void wps_frame_mem_free(void *buffer_addr);
/*****************************************************************************/
/* Alarm related functions                                                   */
/*****************************************************************************/
extern void    *wps_create_alarm(WPS_ALARM_FUNC_T* func, UWORD32 data, ALARM_FUNC_WORK_T *work_func);
extern void     wps_delete_alarm(void** handle);
extern BOOL_T   wps_start_alarm(void* handle, UWORD32 timeout_ms);
extern void     wps_stop_alarm(void* handle);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

#endif /* WPS_ENR_IF_H */
#endif /* INT_WPS_ENR_SUPP */
