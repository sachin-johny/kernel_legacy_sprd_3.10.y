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
/*  File Name         : iconfig.h                                            */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to configuration message parsing.            */
/*                                                                           */
/*  List of Functions : Functions to get/set various WIDs                    */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "core_mode_if.h"
#include "itypes.h"
#include "maccontroller.h"
#include "mib.h"
#include "mh.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "host_if.h"
#include "host_if.h"
#include "cglobals.h"
#include "controller_mode_if.h"
#include "metrics.h"
#include "ch_info.h"
#ifdef AUTORATE_FEATURE
#include "autorate.h"
#endif /* AUTORATE_FEATURE */

#ifdef MAC_HW_UNIT_TEST_MODE
#include "mh_test.h"
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef TX_ABORT_FEATURE
#ifdef TEST_TX_ABORT_FEATURE
#include "mh_test_txabort.h"
#endif /* TEST_TX_ABORT_FEATURE */
#endif /* TX_ABORT_FEATURE */

#include "spi_interface.h"

/*****************************************************************************/
/* Macros                                                                    */
/*****************************************************************************/

#define GET_HEX_CHAR(A) (((A) > 9)? ((A-10) + 'A') : ((A) + '0'))

#ifdef 	IBSS_BSS_STATION_MODE
//chenq add do ap list merge logic 2013-08-28
#define MAX_SCAN_DEL_TIME   120
#define MAX_SCAN_ACT_CNT    2
#endif

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/
/* Number of rates supported (maximum) */
#define NUM_RATES           12

/* Miscellaneous */
#define MSG_DATA_OFFSET     4
#define WID_VALUE_OFFSET    3
#define WID_LENGTH_OFFSET   2
#define WRITE_RSP_LEN       4
// 20120709 caisf mod, merged ittiam mac v1.2 code
#define MAX_QRSP_LEN        1596 // 1000
#define MAX_WRSP_LEN        20
#define MSG_HDR_LEN         4
#define MAX_ADDRESS_LEN     6
#define WRSP_SUCCESS        1
#define MAX_SUPRATE_LEN     34
#define MAX_PROD_VER_LEN    16
#define MAX_GRPADDR_LEN     38
#define SITE_SURVEY_ELM_LEN (MAX_SSID_LEN + 1 + 1 + 1)
#define WID_BIN_DATA_LEN_MASK  0x3FFF

#define  LENGTH_OF_SERIAL_NUMBER          16
#define MAX_SERIAL_NUM_LEN  (LENGTH_OF_SERIAL_NUMBER * 2)

#define MAX_MANUFACTURER_ID_LEN 64
#define MAX_MODEL_NAME_LEN      32
#define MAX_MODEL_NUM_LEN       32

#define PRIM_DEV_TYPE_LEN       8
#define REQ_DEV_TYPE_LEN        8
#define DEFAULT_DEV_OS_VERSION  0x0001

/* MAC Status related */
#define MAC_CONNECTED    1
#define MAC_DISCONNECTED 0
#define STATUS_MSG_LEN   8

//chenq add
#define MAC_SCAN_CMP 2
#define MAC_AP_UP 3
#define MAC_AP_DOWN 4

#define MAC_CTL_OK 5

// add by Ke.Li at 2013-04-04 for fix scan ap bug
#define MAC_CONNECT_FAILED 6
// end add by Ke.Li at 2013-04-04 for fix scan ap bug

//chenq add for post close event to os 2013-05-13
#define MAC_CLOSE 7
//chenq add end

#define PS_ACTIVE                         1
#define PS_SLEEP                          0
#define WAKE_STATUS_MSG_LEN               8

#define DEVICE_READY_MSG_LEN              8

#ifdef MAC_ADDRESS_FROM_FLASH
/* Offsets of Flash Contents */
#define FLASH_MAC_ADDRESS_OFFSET          0

/* Magic numbers used for validating Flash contents */
#define FLASH_MAC_ADDR_ID1                77  /* ASCII-M */
#define FLASH_MAC_ADDR_ID2                65  /* ASCII-A */
#endif /* MAC_ADDRESS_FROM_FLASH */

/* Constants defined for WID_FIRMWARE_INFO */
#define FIRMWARE_INFO_LENGTH     7
#define HOST_FLAGS_OFFSET        0
#define PHY_RF_FLAGS_OFFSET      1
#define MAC_FLAGS_OFFSET         2
#define PLAFORM_FLAGS_OFFSET     4
#define EXTRA_OPT_FLAGS_OFFSET   5


#ifdef INT_WPS_SUPP
#define MAX_WPS_STATUS_MSG_LEN   240  /* WID_STR length - Config/WID headers  */
#endif /* INT_WPS_SUPP */

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* WID Data Types */
typedef enum {WID_CHAR     = 0,
              WID_SHORT    = 1,
              WID_INT      = 2,
              WID_STR      = 3,
              WID_BIN_DATA = 4
} WID_TYPE_T;

/* WLAN Identifiers */
typedef enum {WID_NIL                            = -1,

              /* EMAC Character WID list */
              WID_BSS_TYPE                       = 0x0000,
              WID_CURRENT_TX_RATE                = 0x0001,
              WID_PRIMARY_CHANNEL                = 0x0002,
              WID_PREAMBLE                       = 0x0003,
              WID_11G_OPERATING_MODE             = 0x0004,
              WID_STATUS                         = 0x0005,
              WID_SCAN_TYPE                      = 0x0007,
              WID_PRIVACY_INVOKED                = 0x0008,
              WID_KEY_ID                         = 0x0009,
              WID_QOS_ENABLE                     = 0x000A,
              WID_POWER_MANAGEMENT               = 0x000B,
              WID_11I_MODE                       = 0x000C,
              WID_AUTH_TYPE                      = 0x000D,
              WID_SITE_SURVEY                    = 0x000E,
              WID_LISTEN_INTERVAL                = 0x000F,
              WID_DTIM_PERIOD                    = 0x0010,
              WID_ACK_POLICY                     = 0x0011,
              WID_RESET                          = 0x0012,
              WID_BCAST_SSID                     = 0x0015,
              WID_DISCONNECT                     = 0x0016,
              WID_READ_ADDR_SDRAM                = 0x0017,
              WID_TX_POWER_LEVEL_11A             = 0x0018,
              WID_REKEY_POLICY                   = 0x0019,
              WID_SHORT_SLOT_ALLOWED             = 0x001A,
              WID_PHY_ACTIVE_REG                 = 0x001B,
              WID_TX_POWER_LEVEL_11B             = 0x001D,
              WID_START_SCAN_REQ                 = 0x001E,
              WID_RSSI                           = 0x001F,
              WID_JOIN_REQ                       = 0x0020,
              WID_USER_CONTROL_ON_TX_POWER       = 0x0027,
              WID_MEMORY_ACCESS_8BIT             = 0x0029,
              WID_UAPSD_SUPPORT_AP               = 0x002A,
              WID_CURRENT_MAC_STATUS             = 0x0031,
              WID_AUTO_RX_SENSITIVITY            = 0x0032,
              WID_DATAFLOW_CONTROL               = 0x0033,
              WID_SCAN_FILTER                    = 0x0036,
              WID_LINK_LOSS_THRESHOLD            = 0x0037,
              WID_AUTORATE_TYPE                  = 0x0038,
              WID_802_11H_DFS_MODE               = 0x003B,
              WID_802_11H_TPC_MODE               = 0x003C,
              WID_WPS_ENABLE                     = 0x0041,
              WID_WPS_START                      = 0x0043,
              WID_WPS_DEV_MODE                   = 0x0044,
              WID_OOB_RESET_REQ                  = 0x0046,
              WID_ENABLE_INT_SUPP                = 0x0047,
              WID_DEVICE_MODE                    = 0x0048,
			  
			  // 20120830 caisf add, merged ittiam mac v1.3 code
              WID_ENABLE_MULTI_DOMAIN            = 0x0049,
              WID_CURRENT_REG_DOMAIN             = 0x0050,
              WID_CURRENT_REG_CLASS              = 0x0051,

			  //chenq add snr
			  WID_SNR                            = 0x0052,

			  //chenq add for wapi 2012-09-19
              #ifdef MAC_WAPI_SUPP
			  WID_WAPI_MODE                      = 0x0053,
			  #endif
              
			  // 20121224 caisf add for multi-domain support, get/set scan channel number
			  WID_SCAN_REQ_CHANNEL_NO            = 0x0054,

              /* NMAC Character WID list */
              WID_11N_PROT_MECH                  = 0x0080,
              WID_11N_ERP_PROT_TYPE              = 0x0081,
              WID_11N_ENABLE                     = 0x0082,
              WID_11N_OPERATING_TYPE             = 0x0083,
              WID_11N_OBSS_NONHT_DETECTION       = 0x0084,
              WID_11N_HT_PROT_TYPE               = 0x0085,
              WID_11N_RIFS_PROT_ENABLE           = 0x0086,
              WID_11N_SMPS_MODE                  = 0x0087,
              WID_11N_CURRENT_TX_MCS             = 0x0088,
              WID_11N_PRINT_STATS                = 0x0089,
              WID_HUT_FCS_CORRUPT_MODE           = 0x008A,
              WID_HUT_RESTART                    = 0x008B,
              WID_HUT_TX_FORMAT                  = 0x008C,
              WID_11N_SHORT_GI_ENABLE            = 0x008D,
              WID_HUT_BANDWIDTH                  = 0x008E,
              WID_HUT_OP_BAND                    = 0x008F,
              WID_HUT_STBC                       = 0x0090,
              WID_HUT_ESS                        = 0x0091,
			  // 20120830 caisf mod, merged ittiam mac v1.3 code
			  #if 0
              WID_HUT_ANTSET                     = 0x0092,
			  #else
			  WID_CUR_TX_ANTSET                  = 0x0092,
			  #endif
              WID_HUT_HT_OP_MODE                 = 0x0093,
              WID_RIFS_MODE                      = 0x0094,
              WID_HUT_SMOOTHING_REC              = 0x0095,
              WID_HUT_SOUNDING_PKT               = 0x0096,
              WID_HUT_HT_CODING                  = 0x0097,
              WID_HUT_TEST_DIR                   = 0x0098,
              WID_HUT_PHY_TEST_MODE              = 0x009A,
              WID_HUT_PHY_TEST_RATE_HI           = 0x009B,
              WID_HUT_PHY_TEST_RATE_LO           = 0x009C,
              WID_HUT_DISABLE_RXQ_REPLENISH      = 0x009D,
              WID_HUT_KEY_ORIGIN                 = 0x009E,
              WID_HUT_BCST_PERCENT               = 0x009F,
              WID_HUT_GROUP_CIPHER_TYPE          = 0x00A0,
              WID_TX_ABORT_CONFIG                = 0x00A1,
              WID_HOST_DATA_IF_TYPE              = 0x00A2,
              WID_HOST_CONFIG_IF_TYPE            = 0x00A3,
              WID_HUT_TSF_TEST_MODE              = 0x00A4,
              WID_HUT_PKT_TSSI_VALUE             = 0x00A5,
              WID_REG_TSSI_11B_VALUE             = 0x00A6,
              WID_REG_TSSI_11G_VALUE             = 0x00A7,
              WID_REG_TSSI_11N_VALUE             = 0x00A8,
              WID_TX_CALIBRATION                 = 0x00A9,
              WID_DSCR_TSSI_11B_VALUE            = 0x00AA,
              WID_DSCR_TSSI_11G_VALUE            = 0x00AB,
              WID_DSCR_TSSI_11N_VALUE            = 0x00AC,
              WID_HUT_RSSI_EX                    = 0x00AD,
              WID_HUT_ADJ_RSSI_EX                = 0x00AE,
              WID_11N_IMMEDIATE_BA_ENABLED       = 0x00AF,
              WID_11N_TXOP_PROT_DISABLE          = 0x00B0,
              WID_TX_POWER_LEVEL_11N             = 0x00B1,
              WID_2040_COEXISTENCE               = 0x00C1,
              WID_HUT_FC_TXOP_MOD                = 0x00C2,
              WID_HUT_FC_PROT_TYPE               = 0x00C3,
              WID_HUT_SEC_CCA_ASSERT             = 0x00C4,
              WID_2040_ENABLE                    = 0x00C5,
              WID_2040_40MHZ_INTOLERANT          = 0x00C7,
              WID_11N_CURRENT_TX_BW              = 0x00C8,
              WID_TX_POWER_LEVEL_11N_40MHZ       = 0x00C9,

              /* Custom Character WID list */
              WID_P2P_ENABLE                     = 0x0201,
              WID_P2P_DISCOVERABLE               = 0x0202,
              WID_P2P_LISTEN_CHAN                = 0x0203,
              WID_P2P_FIND_TO                    = 0x0204,
              WID_P2P_GO_INT_VAL                 = 0x0205,
              WID_P2P_PERSIST_GRP                = 0x0206,
              WID_P2P_AUTO_GO                    = 0x0207,
              WID_P2P_INTRA_BSS                  = 0x0208,
              WID_P2P_CT_WINDOW                  = 0x0209,
			  // 20120709 caisf add, merged ittiam mac v1.2 code
              WID_P2P_LISTEN_MODE                = 0x020A,

              /* EMAC Short WID list */
              WID_RTS_THRESHOLD                  = 0x1000,
              WID_FRAG_THRESHOLD                 = 0x1001,
              WID_SHORT_RETRY_LIMIT              = 0x1002,
              WID_LONG_RETRY_LIMIT               = 0x1003,
              WID_BEACON_INTERVAL                = 0x1006,
              WID_MEMORY_ACCESS_16BIT            = 0x1008,
              WID_RX_SENSE                       = 0x100B,
              WID_ACTIVE_SCAN_TIME               = 0x100C,
              WID_PASSIVE_SCAN_TIME              = 0x100D,
              WID_SITE_SURVEY_SCAN_TIME          = 0x100E,
              WID_JOIN_START_TIMEOUT             = 0x100F,
              WID_AUTH_TIMEOUT                   = 0x1010,
              WID_ASOC_TIMEOUT                   = 0x1011,
              WID_11I_PROTOCOL_TIMEOUT           = 0x1012,
              WID_EAPOL_RESPONSE_TIMEOUT         = 0x1013,
              WID_WPS_PASS_ID                    = 0x1017,
              WID_WPS_CONFIG_METHOD              = 0x1018,

              WID_USER_PREF_CHANNEL              = 0x1020,
              WID_CURR_OPER_CHANNEL              = 0x1021,

              /* NMAC Short WID list */
              WID_HUT_FRAME_LEN                  = 0x1081,
              WID_HUT_TXOP_LIMIT                 = 0x1082,
              WID_HUT_SIG_QUAL_AVG               = 0x1083,
              WID_HUT_SIG_QUAL_AVG_CNT           = 0x1084,
              WID_11N_SIG_QUAL_VAL               = 0x1085,
              WID_HUT_RSSI_EX_COUNT              = 0x1086,
              WID_CCA_THRESHOLD                  = 0x1087,

              /* Custom Short WID list */

              /* EMAC Integer WID list */
              WID_FAILED_COUNT                   = 0x2000,
              WID_RETRY_COUNT                    = 0x2001,
              WID_MULTIPLE_RETRY_COUNT           = 0x2002,
              WID_FRAME_DUPLICATE_COUNT          = 0x2003,
              WID_ACK_FAILURE_COUNT              = 0x2004,
              WID_RECEIVED_FRAGMENT_COUNT        = 0x2005,
              WID_MCAST_RECEIVED_FRAME_COUNT     = 0x2006,
              WID_FCS_ERROR_COUNT                = 0x2007,
              WID_SUCCESS_FRAME_COUNT            = 0x2008,
              WID_HUT_TX_COUNT                   = 0x200A,
              WID_TX_FRAGMENT_COUNT              = 0x200B,
              WID_TX_MULTICAST_FRAME_COUNT       = 0x200C,
              WID_RTS_SUCCESS_COUNT              = 0x200D,
              WID_RTS_FAILURE_COUNT              = 0x200E,
              WID_WEP_UNDECRYPTABLE_COUNT        = 0x200F,
              WID_REKEY_PERIOD                   = 0x2010,
              WID_REKEY_PACKET_COUNT             = 0x2011,
              WID_1X_SERV_ADDR                   = 0x2012,
              WID_STACK_IP_ADDR                  = 0x2013,
              WID_STACK_NETMASK_ADDR             = 0x2014,
              WID_HW_RX_COUNT                    = 0x2015,
              WID_MEMORY_ADDRESS                 = 0x201E,
              WID_MEMORY_ACCESS_32BIT            = 0x201F,
              WID_RF_REG_VAL                     = 0x2021,
              WID_DEV_OS_VERSION                 = 0x2025,

              /* NMAC Integer WID list */
              WID_11N_PHY_ACTIVE_REG_VAL         = 0x2080,
              WID_HUT_NUM_TX_PKTS                = 0x2081,
              WID_HUT_TX_TIME_TAKEN              = 0x2082,
              WID_HUT_TX_TEST_TIME               = 0x2083,
			  // 20120830 caisf add, merged ittiam mac v1.3 code
              WID_TX_POWER_LEVELS                = 0x2084,

              /* Custom Integer WID list */

              /* EMAC String WID list */
              WID_SSID                           = 0x3000,
              WID_FIRMWARE_VERSION               = 0x3001,
              WID_OPERATIONAL_RATE_SET           = 0x3002,
              WID_BSSID                          = 0x3003,
              WID_WEP_KEY_VALUE                  = 0x3004,
              WID_11I_PSK                        = 0x3008,
              WID_11E_P_ACTION_REQ               = 0x3009,
              WID_1X_KEY                         = 0x300A,
              WID_HARDWARE_VERSION               = 0x300B,
              WID_MAC_ADDR                       = 0x300C,
              WID_HUT_DEST_ADDR                  = 0x300D,
              WID_MISC_TEST_MODES                = 0x300E,
              WID_PHY_VERSION                    = 0x300F,
              WID_SUPP_USERNAME                  = 0x3010,
              WID_SUPP_PASSWORD                  = 0x3011,
              WID_SITE_SURVEY_RESULTS            = 0x3012,
              WID_RX_POWER_LEVEL                 = 0x3013,
              WID_ADD_WEP_KEY                    = 0x3019,
              WID_REMOVE_WEP_KEY                 = 0x301A,
              WID_ADD_PTK                        = 0x301B,
              WID_ADD_RX_GTK                     = 0x301C,
              WID_ADD_TX_GTK                     = 0x301D,
              WID_REMOVE_KEY                     = 0x301E,
              WID_ASSOC_REQ_INFO                 = 0x301F,
              WID_ASSOC_RES_INFO                 = 0x3020,
              WID_WPS_STATUS                     = 0x3024,
              WID_WPS_PIN                        = 0x3025,
              WID_MANUFACTURER                   = 0x3026,
              WID_MODEL_NAME                     = 0x3027,
              WID_MODEL_NUM                      = 0x3028,
              WID_DEVICE_NAME                    = 0x3029,
			  // 20120709 caisf add, merged ittiam mac v1.2 code
              WID_11I_PSK_VALUE                  = 0x302a,
			  // 20120830 caisf add, merged ittiam mac v1.3 code
			  WID_SUPP_REG_DOMAIN_INFO           = 0x3030,

			  //chenq add for wapi 2012-09-24
		      #ifdef MAC_WAPI_SUPP
			  WID_WAPI_KEY                       = 0x3033,
              #endif

              /* NMAC String WID list */
              WID_11N_P_ACTION_REQ               = 0x3080,
              WID_HUT_TEST_ID                    = 0x3081,
              WID_PMKID_INFO                     = 0x3082,
              WID_FIRMWARE_INFO                  = 0x3083,

              /* Custom String WID list */
              WID_SERIAL_NUMBER                  = 0x3102,

              WID_P2P_TARGET_DEV_ID              = 0x3201,
              WID_P2P_INVIT_DEV_ID               = 0x3202,
              WID_P2P_PERSIST_CRED               = 0x3203,
              WID_P2P_NOA_SCHEDULE               = 0x3204,

              /* EMAC Binary WID list */
              WID_UAPSD_CONFIG                   = 0x4001,
              WID_UAPSD_STATUS                   = 0x4002,
              WID_WMM_AP_AC_PARAMS               = 0x4003,
              WID_WMM_STA_AC_PARAMS              = 0x4004,
              WID_NETWORK_INFO                   = 0x4005,

              WID_WPS_CRED_LIST                  = 0x4006,
              WID_PRIM_DEV_TYPE                  = 0x4007,
              WID_STA_JOIN_INFO                  = 0x4008,
              WID_CONNECTED_STA_LIST             = 0x4009,


              /* NMAC Binary WID list */
              WID_11N_AUTORATE_TABLE             = 0x4080,
              WID_HUT_TX_PATTERN                 = 0x4081,
              WID_HUT_STATS                      = 0x4082,
              WID_HUT_LOG_STATS                  = 0x4083,

              /* Custom Binary WID list */


              WID_P2P_REQ_DEV_TYPE               = 0x4201,



              /* Miscellaneous WIDs */
              WID_ALL                            = 0x7FFE,
              WID_MAX                            = 0xFFFF
} WID_T;

/* Reset requests from user */
typedef enum{DONT_RESET = 0,
             DO_RESET   = 1,
             NO_REQUEST = 2,
             NUM_RESET_REQ
} RESET_REQ_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD16 id;    /* WID Identifier       */
    BOOL_T  rsp;   /* WID_ALL response     */
    BOOL_T  reset; /* Reset MAC required   */
    void    *get;  /* Get Function Pointer */
    void    *set;  /* Set Function Pointer */
} host_wid_struct_t;

/*****************************************************************************/
/* Extern Global Variables                                                   */
/*****************************************************************************/

/*****************************************************************************/
/* WID get/set tables                                                        */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
extern const host_wid_struct_t g_char_wid_struct[];
#else /* MAC_HW_UNIT_TEST_MODE */
extern host_wid_struct_t g_char_wid_struct[];
#endif /* MAC_HW_UNIT_TEST_MODE */

extern const host_wid_struct_t g_short_wid_struct[];
extern const host_wid_struct_t g_int_wid_struct[];
extern const host_wid_struct_t g_str_wid_struct[];
extern const host_wid_struct_t g_binary_data_wid_struct[];
extern void init_dev_spec_info(void);
#ifdef INT_WPS_SUPP
extern void send_wps_status(UWORD8 status_code, UWORD8* msg_ptr,
                            UWORD16 msg_len);
#endif /* INT_WPS_SUPP */

/*****************************************************************************/
/* Configuration parameters                                                  */
/*****************************************************************************/

extern UWORD16      g_current_len;
extern BOOL_T       g_reset_mac;
extern RESET_REQ_T  g_reset_req_from_user;
extern UWORD8       g_current_settings[MAX_QRSP_LEN];
extern UWORD8       g_phy_ver[MAX_PROD_VER_LEN + 1];
extern UWORD8       g_info_id;

#ifdef ENABLE_OVERWRITE_SUBTYPE
extern UWORD8 g_overwrite_frame_type;
#endif /* ENABLE_OVERWRITE_SUBTYPE */

/*****************************************************************************/
/* Device Specific Configuration parameters                                  */
/*****************************************************************************/

extern UWORD8 g_serial_num[MAX_SERIAL_NUM_LEN + 1];
extern UWORD32 g_dev_os_version;
extern UWORD8  g_manufacturer[MAX_MANUFACTURER_ID_LEN + 1];
extern UWORD8  g_model_name[MAX_MODEL_NAME_LEN + 1];
extern UWORD8  g_model_num[MAX_MODEL_NUM_LEN + 1];
extern UWORD8  g_device_name[MAX_DEVICE_NAME_LEN + 1];
extern UWORD8  g_prim_dev_type[PRIM_DEV_TYPE_LEN+2];

#ifdef MAC_P2P
extern UWORD8 g_req_dev_type[REQ_DEV_TYPE_LEN + 2];
#endif /* MAC_P2P */

extern const UWORD8 g_default_manufacturer[];
extern const UWORD8 g_default_model_name[];
extern const UWORD8 g_default_model_num[];
extern const UWORD8 g_default_device_name[];

/*****************************************************************************/
/* Custom configuration parameters                                           */
/*****************************************************************************/

extern UWORD16      g_aging_thresh_in_sec;

#ifdef 	IBSS_BSS_STATION_MODE
//chenq add do ap list merge logic 2013-08-28
extern UWORD8        g_link_list_bss_count;
#endif

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern UWORD16 process_query(UWORD8* wid_req, UWORD8* wid_rsp, UWORD16 len);
extern void    process_write(UWORD8* wid_req, UWORD16 req_len);
extern void    send_host_rsp(UWORD8 *host_rsp, UWORD16 host_rsp_len,
                                                           UWORD8 host_if_type);
extern void    set_wid(UWORD8 *wid, UWORD16 len, UWORD8 count, UWORD8 wid_type);
extern UWORD16 get_wid(UWORD8 *wid, UWORD8 count, UWORD8 wid_type);
extern void    parse_config_message(mac_struct_t *mac, UWORD8* host_req,
                                  UWORD8 *buffer_addr,UWORD8 host_if_type);
extern void    save_wids(void);
extern void    restore_wids(void);
extern void    resolve_wid_conflicts(mac_struct_t *mac);
extern void    config_op_params(mac_struct_t *mac);
extern void    send_mac_status(UWORD8 mac_status);
extern void    send_network_info_to_host(UWORD8 *msa, UWORD16 rx_len,
                                         signed char rssi);
extern void    send_join_leave_info_to_host(UWORD16 aid, UWORD8* sta_addr,
                                            BOOL_T joining);
#ifdef DEBUG_MODE	//add by chengwg for register debug.
extern void display_general_register(void);
extern void display_reception_register(void);
extern void display_EDCA_register(void);
extern void display_HCCA_STA_register(void);
extern void display_TSF_register(void);
extern void display_protection_SIFS_response_register(void);
extern void display_channel_access_mgmt_register(void);
extern void display_retry_register(void);
extern void display_sequence_number_register(void);
extern void display_PCF_register(void);
extern void display_power_mgmt_register(void);
extern void display_interrupt_register(void);
extern void display_phy_interface_register(void);
extern void display_block_ack_register(void);
#endif	/* DEBUG_MODE */

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function Enables\Disables Protection for Tx-OP */
INLINE void set_disable_txop_protection(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_disable_rtscts_norsp = val;

#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_config.tx.rtscts_txop = val;
#else  /* MAC_HW_UNIT_TEST_MODE */

    if(g_disable_rtscts_norsp == 1)
        disable_machw_rtscts_norsp();
    else
        enable_machw_rtscts_norsp();
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function gets the current setting for Tx-OP protection */
INLINE UWORD8 get_disable_txop_protection(void)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    return (g_test_config.tx.rtscts_txop);
#else  /* MAC_HW_UNIT_TEST_MODE */
    return g_disable_rtscts_norsp;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Get the Current MAC Status */
INLINE UWORD8 get_current_mac_status(void)
{
    return g_current_mac_status;
}

INLINE UWORD8  get_auto_rx_sensitivity(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return 0;
#else   /*  IBSS_BSS_STATION_MODE   */
    return 0;
#endif  /*  IBSS_BSS_STATION_MODE   */
}

INLINE void  set_auto_rx_sensitivity(UWORD8 input)
{
#ifdef IBSS_BSS_STATION_MODE
#endif  /*  IBSS_BSS_STATION_MODE   */
}

INLINE UWORD8  get_dataflow_control(void)
{
    return g_rx_buffer_based_ack;
}

INLINE void  set_dataflow_control(UWORD8 input)
{
    if(input)
    {
        if(g_rx_buffer_based_ack != BTRUE)
        {
            g_rx_buffer_based_ack = BTRUE;
            enable_rx_buff_based_ack();
        }
    }
    else
    {
        if(g_rx_buffer_based_ack != BFALSE)
        {
            g_rx_buffer_based_ack = BFALSE;
            disable_rx_buff_based_ack();
        }
    }
}

INLINE UWORD8 get_scan_filter(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return g_scan_filter;
#else  /*  IBSS_BSS_STATION_MODE   */
    return 0;
#endif  /*  IBSS_BSS_STATION_MODE   */
}

INLINE void set_scan_filter(UWORD8 input)
{
#ifdef IBSS_BSS_STATION_MODE
    g_scan_filter = input;
#endif  /*  IBSS_BSS_STATION_MODE   */
}

INLINE UWORD8 get_link_loss_threshold(void)
{
#ifdef IBSS_BSS_STATION_MODE
    // Modify by Yiming.Li at 2014-03-10 for fix bug281486: 
    if(g_wifi_bt_coex)
		g_link_loss_threshold = COEXIST_LINK_LOSS_THRESHOLD;    //Yiming.LI In coexist mode, wifi disconnect easily with BT is calling.
    else
		g_link_loss_threshold = DEFAULT_LINK_LOSS_THRESHOLD;    //Yiming.LI In wifi only mode, vitual AP is in results of scanning.

    return g_link_loss_threshold;
#endif  /*  IBSS_BSS_STATION_MODE   */

#ifdef BSS_ACCESS_POINT_MODE
    return g_aging_thresh_in_sec;
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

INLINE void set_link_loss_threshold(UWORD8 input)
{
#ifdef IBSS_BSS_STATION_MODE
    if(MIN_LINK_LOSS_THRESHOLD > input)
    {
        g_link_loss_threshold = MIN_LINK_LOSS_THRESHOLD;
    }
    else
    {
        g_link_loss_threshold = input;
    }
#endif  /*  IBSS_BSS_STATION_MODE   */

#ifdef BSS_ACCESS_POINT_MODE
    g_aging_thresh_in_sec = input;
#endif /* BSS_ACCESS_POINT_MODE */
}

INLINE UWORD8 get_autorate_type(void)
{
    return g_autorate_type;
}

INLINE void set_autorate_type(UWORD8 input)
{
    g_autorate_type = input;
}

INLINE UWORD16 get_cca_threshold(void)
{
    return get_phy_cca_threshold();
}

INLINE void set_cca_threshold(UWORD16 input)
{
    set_phy_cca_threshold(input);
}

/* Set the Short Slot option                                */
/* This function needs to be here because it is used in the */
/* subsequent functions                                     */
INLINE void set_short_slot_allowed(UWORD8 val)
{
    BOOL_T short_slot_enable = BFALSE;

    /* Input Sanity Check */
    if(val > 1)
        return;

    g_short_slot_allowed = (BOOL_T) val;

    /* Disable Short Slot */
    if(g_short_slot_allowed == BFALSE)
    {
       short_slot_enable = BFALSE;
    }
    /* Enable Short Slot if applicable */
    else
    {
        /* For IBSS mode, disable short slot */
        if(mget_DesiredBSSType() == INDEPENDENT)
        {
            short_slot_enable = BFALSE;
        }

    #ifdef BURST_TX_MODE
        /* Short slot must be enabled in Burst tx mode */
        else if (g_burst_tx_mode_enabled == BTRUE)
        {
            short_slot_enable = BTRUE;
        }
    #endif /* BURST_TX_MODE */
        else
        {
            short_slot_enable = BTRUE;
        }
    }


    if(get_current_start_freq() == RC_START_FREQ_5)
    {
        short_slot_enable = BTRUE;
    }

    /* Update PHY & MAC HW registers */
    if(short_slot_enable == BTRUE)
    {
        enable_short_slot();
        set_machw_short_slot_select();
    }
    else
    {
        disable_short_slot();
        set_machw_long_slot_select();
    }
}

/* This function returns the current slot option */
INLINE UWORD8 get_short_slot_allowed(void)
{
    return g_short_slot_allowed;
}

/* This function returns the SSID in the format required by the host. */
INLINE WORD8* get_DesiredSSID(void)
{
    WORD8 *ssid_temp = mget_DesiredSSID();

    g_cfg_val[0] = strlen(ssid_temp);
    strcpy((WORD8 *)( g_cfg_val + 1 ), ssid_temp);

    return ((WORD8 *)g_cfg_val);
}

/* This function returns the group address in the format required by host. */
INLINE UWORD8* get_GroupAddress(void)
{
    UWORD8 *grpa_temp = mget_GroupAddress();

    g_cfg_val[0] = grpa_temp[GROUP_ADDRESS_SIZE_OFFSET]*6;
    strcpy((WORD8 *)(g_cfg_val + 1), (WORD8 *)grpa_temp);

    return g_cfg_val;
}

/* This function returns the product version in the format required by host. */
INLINE UWORD8* get_manufacturerProductVersion(void)
{
    UWORD8 *prod_ver_temp = mget_manufacturerProductVersion();

    g_cfg_val[0] = strlen((WORD8 *)prod_ver_temp);
    strcpy((WORD8 *)(g_cfg_val + 1), (WORD8 *)prod_ver_temp);

    return g_cfg_val;
}

/* This function returns the product version in the format required by host. */
INLINE UWORD8* get_hardwareProductVersion(void)
{
    UWORD32 hard_ver_temp = get_machw_pa_ver();
    UWORD8  index         = 1;
    UWORD8  temp          = 0;

    /* MAC H/w version register format                  */
    /* ------------------------------------------------ */
    /* | 31 - 28 | 27 - 24 | 23 - 16 | 15 - 8 | 7 - 0 | */
    /* ------------------------------------------------ */
    /* | BN      | Y1      | Y2      | Y3     | Y4    | */
    /* ------------------------------------------------ */

    /* Format the version as BN.Y1.Y2.Y3.Y4 with all values in hex i.e. the  */
    /* version string would be X.X.XX.XX.XX.                                 */
    /* For e.g. 9A1201F1 saved in the version register would translate to    */
    /* the configuration interface version number 9.A.12.01.F1               */

    temp = (hard_ver_temp & 0xF0000000) >> 28; /* BN: 4-bit, 1 char */
    g_cfg_val[index++] = GET_HEX_CHAR(temp);
    g_cfg_val[index++] = '.';

    temp = (hard_ver_temp & 0x0F000000) >> 24; /* Y1: 4-bit, 1 char */
    g_cfg_val[index++] = GET_HEX_CHAR(temp);
    g_cfg_val[index++] = '.';

    temp = (hard_ver_temp & 0x00FF0000) >> 16; /* Y2: 8-bit, 2 char */
    g_cfg_val[index++] = GET_HEX_CHAR(temp/16);
    temp -= (((UWORD8)(temp/16))*16);
    g_cfg_val[index++] = GET_HEX_CHAR(temp);
    g_cfg_val[index++] = '.';

    temp = (hard_ver_temp & 0x0000FF00) >> 8; /* Y3: 8-bit, 2 char */
    g_cfg_val[index++] = GET_HEX_CHAR(temp/16);
    temp -= (((UWORD8)(temp/16))*16);
    g_cfg_val[index++] = GET_HEX_CHAR(temp);
    g_cfg_val[index++] = '.';

    temp = (hard_ver_temp & 0x000000FF); /* Y4: 8-bit, 2 char */
    g_cfg_val[index++] = GET_HEX_CHAR(temp/16);
    temp -= (((UWORD8)(temp/16))*16);
    g_cfg_val[index++] = GET_HEX_CHAR(temp);

    g_cfg_val[0] = index - 1;
    return g_cfg_val;
}


/* This function returns the product version in the format required by host. */
INLINE UWORD8* get_phyProductVersion(void)
{
    return g_phy_ver;
}

/* This function returns the BSSID in the format required by the host.*/
INLINE UWORD8* get_bssid(void)
{
    UWORD8 *bssid_temp = mget_bssid();

    g_cfg_val[0] = MAX_ADDRESS_LEN;
    memcpy(g_cfg_val + 1, bssid_temp, MAX_ADDRESS_LEN);

    return g_cfg_val;
}

/* This function sets the BSSID required for the MH/PHY Test.*/
INLINE void set_bssid(UWORD8 *val)
{

    handle_wps_cred_update();

#ifdef MAC_HW_UNIT_TEST_MODE
    memcpy(g_test_params.bssid, val, MAX_ADDRESS_LEN);
    mset_bssid(val);
#endif /* MAC_HW_UNIT_TEST_MODE */
#ifdef IBSS_BSS_STATION_MODE
    if(g_reset_mac_in_progress == BFALSE)
    {
        memcpy(g_prefered_bssid, val, 6);
    }
#endif /* IBSS_BSS_STATION_MODE */
}

/* This function returns the MAC Address in the format required by the host.*/
INLINE UWORD8* get_mac_addr(void)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    UWORD8 *mac_addr_temp = mget_StationID();

    g_cfg_val[0] = MAX_ADDRESS_LEN;
    memcpy(g_cfg_val + 1, mac_addr_temp, MAX_ADDRESS_LEN);

    return g_cfg_val;
#else /* MAC_HW_UNIT_TEST_MODE */
    g_cfg_val[0] = MAX_ADDRESS_LEN;
    memcpy(&g_cfg_val[1], g_test_params.mac_address, MAX_ADDRESS_LEN);
    return g_cfg_val;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function returns the Supported Rates in the format required by host. */
INLINE UWORD8* get_supp_rates(void)
{
#ifdef PHY_802_11n
    UWORD8 *str = (UWORD8 *)"1,2,5.5,11,6,9,12,18,24,36,48,54";
    g_cfg_val[0] = strlen((WORD8 *)str);
    memcpy(g_cfg_val + 1, str, g_cfg_val[0]);
#endif /* PHY_802_11n */

    return g_cfg_val;
}

/*  BSS Type formats for Host and MAC                                        */
/*  --------------------------------------------------------------------     */
/*                  Infrastructure    Independent    Access Point            */
/*  Host :                 0               1            2                    */
/*  MIB  :                 1               2            3                    */
/*  --------------------------------------------------------------------     */

/* This function returns the BSS Type in the format required by the host. */
INLINE UWORD8 get_DesiredBSSType(void)
{
    UWORD8 btype = get_operating_bss_type();

    return (btype - 1);
}

/* This function returns the BSS Type in the format required by the MAC.*/
INLINE void set_DesiredBSSType(UWORD8 bss_type)
{

    handle_wps_cred_update();

#ifdef MAC_HW_UNIT_TEST_MODE
    /* Reseting MAC is not required, when rate changes in Standard modes     */
    g_char_wid_struct[1].reset = BFALSE;
    /* Reseting MAC is not required, when Preamble changes in Standard modes */
    g_char_wid_struct[3].reset = BFALSE;
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef PHY_CONTINUOUS_TX_MODE
    if(bss_type == 3)
    {
        g_phy_continuous_tx_mode_enable = BTRUE;
        /* If Rate changes, Reset MAC is needed        */
        g_char_wid_struct[1].reset = BTRUE;
        /* If Preamble changes, Reset MAC is needed    */
        g_char_wid_struct[3].reset = BTRUE;
    }
    else
    {
        g_phy_continuous_tx_mode_enable = BFALSE;
    }
#endif /* PHY_CONTINUOUS_TX_MODE */

#ifdef BURST_TX_MODE
    if(bss_type == 4)
    {
        g_burst_tx_mode_enabled = BTRUE;
        /*If Rate changes, Reset MAC is needed        */
        g_char_wid_struct[1].reset = BTRUE;
        /* If Preamble changes, Reset MAC is needed    */
        g_char_wid_struct[3].reset = BTRUE;
    }
    else
    {
        g_burst_tx_mode_enabled = BFALSE;
    }
#endif /* BURST_TX_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_params.mac_mode = bss_type + 1;

    if(bss_type == 2) /* Access Point Mode */
        set_bssid(mget_StationID());
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef IBSS_BSS_STATION_MODE
    /* Reset BSSID if BSS type changes from IBSS STA to BSS STA so that no   */
    /* deauthentication is sent in reset_mac                                 */
    if(mget_DesiredBSSType() == INDEPENDENT)
    {
        UWORD8 temp_addr[6] = {0};
        mset_bssid(temp_addr);
    }
    /* Inform the AP if BSS type is changing from BSS STA to IBSS STA if it  */
    /* is already connected to AP                                            */
    else if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        if(get_mac_state() == ENABLED)
            send_deauth_frame(mget_bssid(), (UWORD16)UNSPEC_REASON);
    }
#endif /* IBSS_BSS_STATION_MODE  */

#ifdef BSS_ACCESS_POINT_MODE
    mset_DesiredBSSType(INFRASTRUCTURE);
#else /* BSS_ACCESS_POINT_MODE */
    /* Set the MIB parameters */
    if(bss_type >= 2)
        bss_type = 0;
    mset_DesiredBSSType((UWORD8)(bss_type + 1));
#endif /* BSS_ACCESS_POINT_MODE */

   /* Re-set slot type after changing BSS type */
    set_short_slot_allowed(g_short_slot_allowed);
}

/* Returns the current MAC channel number */
INLINE UWORD8 get_mac_chnl_num(void)
{
    UWORD8 ch_idx = mget_CurrentChannel();
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_num = get_ch_num_from_idx(freq, ch_idx);

    return ch_num;
}

/* This function returns the Channel in the format required by the MAC.*/
INLINE void set_mac_chnl_num(UWORD8 ch_num)
{
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_idx = get_ch_idx_from_num(freq, ch_num);

    /* Input Sanity Check */
    if(is_ch_valid(ch_idx) == BFALSE)
        return;

    mset_CurrentChannel(ch_idx);

    if(g_user_control_enabled == BFALSE)
    {
        set_default_tx_power_levels();
    }

#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_params.channel = ch_idx;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/*  Channel formats for Host                                                 */
/*  --------------------------------------------------------------------     */
/*  | BIT 15     |  BIT 14-10 |     BIT 9-8      |   BIT 7-0           |     */
/*  --------------------------------------------------------------------     */
/*  | Freq Band  |   Reserved |  Sec Ch Offset   |   Prim Ch Number    |     */
/*  --------------------------------------------------------------------     */
/*  | 0 - 2.4 GHz|            | 0/1/3 - SCN/A/B  |   1 to 14           |     */
/*  --------------------------------------------------------------------     */
/*  | 1 - 5 GHz  |            | 0/1/3 - SCN/A/B  |   36 to 161         |     */
/*  --------------------------------------------------------------------     */

/* This function returns the Channel in the format required by the host.*/
INLINE UWORD16 get_user_pref_channel(void)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    return g_user_pref_channel;
#else /* MAC_HW_UNIT_TEST_MODE */
    UWORD8  chan_num  = 0;
    UWORD16 chan_info = 0;

    chan_num  = get_ch_num_from_idx(g_test_params.start_freq,
                                    g_test_params.channel);

    chan_info = ((g_test_params.start_freq & 0x1) << 15)     |
                ((g_test_params.sec_chan_offset & 0x3) << 8) |
                (chan_num & 0xFF);

    return chan_info;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function sets the user preferred channel information */
INLINE void set_user_pref_channel(UWORD16 host_ch_info)
{
    UWORD8 freq   = 0;
    UWORD8 ch_num = 0;
    UWORD8 ch_idx = 0;

    /* Get the preferred frequency band */
    if(host_ch_info & BIT15)
    {
        freq = RC_START_FREQ_5;
    }
    else
    {
        freq = RC_START_FREQ_2;
    }

    /* Get the preferred primary channel number */
    ch_num = host_ch_info & 0xFF;

    /* If its not auto channel selection */
    if(ch_num)
    {
        /* Compute the channel index using user selected freq range */
        ch_idx = get_ch_idx_from_num(freq, ch_num);

        /* Check if the set channel-freq combo is fine */
        if(is_ch_valid(ch_idx) == BTRUE)
        {
            /* Set the user preferred freq band */
            g_user_pref_channel &= 0x7FFF;
            g_user_pref_channel |= (host_ch_info & BIT15);

            /* Set the preferred primary channel */
            g_user_pref_channel &= 0xFF00;
            g_user_pref_channel |= ch_num;

            /* Set the user preferred freq band to MAC*/
            set_current_start_freq(freq);

            /* Initialize MAC with user preferred primary channel */
            set_mac_chnl_num(ch_num);
#ifdef MAC_P2P
            mset_p2p_oper_chan(ch_idx);
#endif /* MAC_P2P */
        }
        else
        {
            /* Try the other frequency band */
            if(RC_START_FREQ_5 == freq)
            {
                freq = RC_START_FREQ_2;
            }
            else
            {
                freq = RC_START_FREQ_5;
            }

            /* Recompute the channel index using this freq band*/
            ch_idx = get_ch_idx_from_num(freq, ch_num);

            /* If the chan index is valid now, then set it */
            if(is_ch_valid(ch_idx) == BTRUE)
            {
                /* Set the preferred freq band */
                if(RC_START_FREQ_5 == freq)
                {
                    g_user_pref_channel |= BIT15;
                }
                else
                {
                    g_user_pref_channel &= ~BIT15;
                }

                /* Set the preferred primary channel */
                g_user_pref_channel &= 0xFF00;
                g_user_pref_channel |= ch_num;

                /* Set the user preferred freq band to MAC*/
                set_current_start_freq(freq);

                /* Initialize MAC with user preferred primary channel */
                set_mac_chnl_num(ch_num);
				// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MAC_P2P
                mset_p2p_oper_chan(ch_idx);
#endif /* MAC_P2P */
            }
        }

    }
    else
    {
        /* Set the user preferred freq band & channel */
        g_user_pref_channel &= 0x7F00;
        g_user_pref_channel |= (host_ch_info & BIT15);

        /* Set the user preferred freq band to MAC*/
        set_current_start_freq(freq);
    }


#ifdef MAC_HW_UNIT_TEST_MODE
    if(ch_num)
    {
        g_test_params.start_freq      = freq;
        g_test_params.channel         = ch_idx;
    }
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Funtion to set only primary channel */
INLINE void set_prim_chnl_num(UWORD8 prim_ch)
{
    /* Get the Sec ch info from database */
    UWORD8 sec_ch = ((g_user_pref_channel & 0x0300) >> 8);

    /* Prepare the 16 bit word */
    UWORD16 host_ch_info = MAKE_WORD16(prim_ch, sec_ch);

    /* Update with Freq band info */
    host_ch_info |= (g_user_pref_channel & BIT15);

    /* Set the user preferred channel info */
    set_user_pref_channel(host_ch_info);
}


/* Funtion to read current operation channel info */
INLINE UWORD16 get_curr_op_channel(void)
{
    UWORD8 ch_idx = mget_CurrentChannel();
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_num = get_ch_num_from_idx(freq, ch_idx);
    UWORD8 sec_ch = get_bss_sec_chan_offset();
    UWORD16 ch_info =  MAKE_WORD16(ch_num, sec_ch);

    /* Update the Frequency band bit */
    if(RC_START_FREQ_5 == freq)
    {
        ch_info |= BIT15;
    }
    else
    {
        ch_info &= ~BIT15;
    }

    return ch_info;
}

/* Funtion to read current operation primary channel  */
INLINE UWORD8 get_prim_chnl_num(void)
{
    return((0x00FF & get_curr_op_channel()));
}

/*  Rate formats for Host and MAC                                            */
/*  -----------------------------------------------------------------------  */
/*          1   2   5.5   11   6  9  12  18  24  36  48   54  Auto           */
/*  Host :  1   2     3    4   5  6   7   8   9  10  11   12  0              */
/*  MAC  :  1   2     5   11   6  9  12  18  24  36  48   54  Not supported  */
/*  -----------------------------------------------------------------------  */

/* This function returns the transmission rate to the user.*/
INLINE UWORD8 get_tx_rate(void)
{
    if(is_autorate_enabled() == BTRUE)
        return 0;

    return get_curr_tx_rate();
}

/* This function sets the transmission rate as requested by the user.*/
INLINE void set_tx_rate(UWORD8 rate)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    if(rate != 0)
        set_curr_tx_rate(rate);

    /* Update the Current rate only if the input is valid */
    if(get_tx_rate() == rate)
        g_test_params.tx_rate = rate;
#else /* MAC_HW_UNIT_TEST_MODE */
    if(rate == 0)
    {
        enable_autorate();
    }
    else
    {
        disable_autorate();
        set_curr_tx_rate(rate);
    }

    /* Update the retry rate set for all the entries */
        update_per_entry_retry_set_info();
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/*  Preamble formats for Host and MAC                                        */
/*  -----------------------------------------------------------------------  */
/*          Long          Short                                              */
/*  Host :  1              0                                                 */
/*  MAC  :  1              0                                                 */
/*  -----------------------------------------------------------------------  */

/* This function returns the preamble type as set by the user.*/
INLINE UWORD8 get_preamble_type(void)
{
    return g_preamble_type;
}

/* This function sets the preamble type as requested by the user.*/
INLINE void set_preamble_type(UWORD8 idx)
{
    /* Input Sanity Check */
    if(idx > 2)
        return;
	
    g_preamble_type = idx;
	/*junbin.wang mask for cr 229369.2013-11-13*/
    //set_preamble(idx);

#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_params.preamble = idx;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This functions sets the 11g operating mode. The operating mode effects the */
/* transmit rate being used. Hence a back-up of the transmit rate is taken    */
/* before updating the operating mode. The rate is restored after the         */
/* operating mode is changed.                                                 */
INLINE void set_11g_op_mode(UWORD8 val)
{
    UWORD8 tx_rate = 0;

    tx_rate = get_tx_rate();

    set_running_mode(val);

    set_tx_rate(tx_rate);
}

/* This function returns the current 11g operating mode */
INLINE UWORD8 get_11g_op_mode(void)
{
    return get_running_mode();
}

/* This function sets the transmission MCS as requested by the user */
INLINE void set_tx_mcs(UWORD8 mcs)
{
#ifdef PHY_802_11n
    /* In case of no auto rate update the current Tx MCS and the retry rate  */
    /* set for all entries in the station/association table for the current  */
    /* transmit MCS.                                                         */
    if(is_autorate_enabled() == BFALSE)
    {
        if((mcs <= MAX_MCS_SUPPORTED) ||
           (mcs == INVALID_MCS_VALUE))
        {
            set_curr_tx_mcs(mcs);
        }

        update_per_entry_retry_set_info();
    }
#endif /* PHY_802_11n */
}

/* This function sets the RTS threshold as requested by the user. The MAC    */
/* H/w register is also set accordingly.                                     */
INLINE void set_RTSThreshold(UWORD16 limit)
{
    /* Input Sanity Check */
    if(limit < 256)
        return;

    mset_RTSThreshold(limit);
    set_machw_rts_thresh(limit);

#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_params.rts_threshold = limit;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function sets the Fragmentation threshold as requested by the user.  */
/* The MAC H/w register is also set accordingly.                             */
INLINE void set_FragmentationThreshold(UWORD16 limit)
{
    /* Input Sanity Check */
    if((limit < 256) || (limit > 7936))
        return;

    /* Fragmentation threshold must be even as per standard */
    if((limit % 2) != 0)
        limit -= 1;

    mset_FragmentationThreshold(limit);
    set_machw_frag_thresh(limit);

#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_params.frag_threshold = limit;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function sets the Short Retry Limit as requested by the user. The    */
/* MAC H/w register is also set accordingly.                                 */
INLINE void set_ShortRetryLimit(UWORD16 limit)
{
    /* Input Sanity Check */
    if((limit == 0) || (limit > 255))
        return;

    mset_ShortRetryLimit(limit);
    set_machw_srl(mget_ShortRetryLimit());
}

/* This function sets the Long Retry Limit as requested by the user. The     */
/* MAC H/w register is also set accordingly.                                 */
INLINE void set_LongRetryLimit(UWORD16 limit)
{
    /* Input Sanity Check */
    if((limit == 0) || (limit > 255))
        return;

    mset_LongRetryLimit(limit);
    set_machw_lrl(mget_LongRetryLimit());
}

/* This function returns the current protection mode */
INLINE UWORD8 get_protection_mode(void)
{
    if(is_autoprot_enabled() == BTRUE)
        return 0;

    return get_protection();
}

/* This function sets the protection mode */
INLINE void set_protection_mode(UWORD8 prot_mode)
{
    /* Input Sanity Check */
    if(prot_mode < NUM_PROTECTION_MODE)
    {
        if(prot_mode == 0)
        {
            enable_autoprot();
        }
        else
        {
            disable_autoprot();
            set_protection(prot_mode);
        }

#ifdef MAC_HW_UNIT_TEST_MODE
        g_test_params.prot_type = prot_mode;
#endif /* MAC_HW_UNIT_TEST_MODE */
    }
}

/* Get the current value of power management mode */
INLINE UWORD8 get_PowerManagementMode(void)
{
#ifdef IBSS_BSS_STATION_MODE
    if(mget_PowerManagementMode() == MIB_POWERSAVE)
    {
        return get_user_ps_mode();
    }
    else
        return (NO_POWERSAVE);
#else /* IBSS_BSS_STATION_MODE */
    return(0);
#endif /* IBSS_BSS_STATION_MODE */
}


// 20130216 caisf add for power management mode policy to take effect
INLINE void apply_PowerManagementMode_policy(void)
{
#ifdef IBSS_BSS_STATION_MODE
    if(g_powermanagement_wait_apply_flag)
    {
        TROUT_DBG4("ps: dot11PowerManagementMode_wait_apply = %s\n",
            dot11PowerManagementMode_wait_apply==MIB_POWERSAVE ? "MIB_POWERSAVE":"MIB_ACTIVE");

        if(dot11PowerManagementMode_wait_apply == MIB_POWERSAVE)  // power save
        {
            set_user_ps_mode(g_user_ps_mode_wait_apply);
            g_receive_dtim = g_receive_dtim_wait_apply;
            mset_PowerManagementMode(dot11PowerManagementMode_wait_apply);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            g_wifi_suspend_status = wifi_suspend_early_suspend;
#endif
        }
        else // active
        {
            mset_PowerManagementMode(dot11PowerManagementMode_wait_apply);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            g_wifi_suspend_status = wifi_suspend_nosuspend;
#endif
        }
        
        g_powermanagement_wait_apply_flag = BFALSE;
    }
#endif	/* IBSS_BSS_STATION_MODE */
}

/* Set the power management mode according to value */
INLINE void set_PowerManagementMode(UWORD8 val)
{
	TROUT_FUNC_ENTER;
#ifdef IBSS_BSS_STATION_MODE
    TROUT_DBG4("PS: set power save mode to %d\n", val);
    TROUT_DBG4("PS: set power save mode to %s\n", USER_PS_MODE_2STR(val));

    if(g_powermanagement_wait_apply_flag)
    {
        TROUT_DBG4("PS: set power save mode to %d failed, device busy!\n", val);
        return;
    }
    
    if(val != NO_POWERSAVE)
    {
        // 20130216 caisf add for power management mode policy to take effect
        if((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE))
        {
            // debug info
            if(get_mac_state() == ENABLED)
                TROUT_DBG4("PS: get_mac_state() ENABLED\n");
            else if(g_keep_connection == BTRUE)
                TROUT_DBG4("PS: g_keep_connection BTRUE in obss scan\n");
            
            if((val == MAX_FAST_PS) || (val == MAX_PSPOLL_PS))
                g_receive_dtim_wait_apply = BFALSE;
            else
                g_receive_dtim_wait_apply = BTRUE;
				
            g_user_ps_mode_wait_apply = val;
            dot11PowerManagementMode_wait_apply = MIB_POWERSAVE;
            
            g_powermanagement_wait_apply_flag = BTRUE;
        }
        else
        {
            if((val == MAX_FAST_PS) || (val == MAX_PSPOLL_PS))
                g_receive_dtim = BFALSE;
            else
                g_receive_dtim = BTRUE;

            set_user_ps_mode(val);

            mset_PowerManagementMode(MIB_POWERSAVE);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            g_wifi_suspend_status = wifi_suspend_early_suspend;
#endif
        }
    }
    else
    {
        // 20130216 caisf add for power management mode policy to take effect
        if((get_mac_state() == ENABLED) || (g_keep_connection == BTRUE))
        {
            // debug info
            if(get_mac_state() == ENABLED)
                TROUT_DBG4("PS: get_mac_state() ENABLED\n");
            else if(g_keep_connection == BTRUE)
                TROUT_DBG4("PS: g_keep_connection BTRUE in obss scan\n");
            
            dot11PowerManagementMode_wait_apply = MIB_ACTIVE;

            g_powermanagement_wait_apply_flag = BTRUE;
        }
        else
        {
            mset_PowerManagementMode(MIB_ACTIVE);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
            g_wifi_suspend_status = wifi_suspend_nosuspend;
#endif
        }
    }
#endif /* IBSS_BSS_STATION_MODE */
	TROUT_FUNC_EXIT;
}

/* Get the current value of ack policy type */
INLINE UWORD8 get_ack_type(void)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    return g_ack_policy;
#else /* MAC_HW_UNIT_TEST_MODE */
    return g_test_config.tx.ac_vo.ack_policy;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Set the value of the Ack Policy to the value */
INLINE void set_ack_type(UWORD8 val)
{
    /* Input Sanity check */
    if(val < NUM_ACK_POLICY)
    {
#ifndef MAC_HW_UNIT_TEST_MODE
        g_ack_policy = val;
#else /* MAC_HW_UNIT_TEST_MODE */
        g_test_config.tx.ac_vo.ack_policy = val;
#endif /* MAC_HW_UNIT_TEST_MODE */
    }
}

/* Set the reset request from user */
INLINE void set_reset_req(UWORD8 val)
{
    /* Input Sanity Check */
    if(val < NUM_RESET_REQ)
        g_reset_req_from_user = (RESET_REQ_T)val;
}

/* Get the current value of PCF mode supported */
INLINE UWORD8 get_bcst_ssid(void)
{
    return g_bcst_ssid;
}

/* Set the value of the of PCF mode to the value */
INLINE void set_bcst_ssid(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_bcst_ssid = val;
}

/* This function sets the MAC Address required for the MH/PHY Test.*/
INLINE void set_mac_addr(UWORD8 *val)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    memcpy(g_test_params.mac_address, val, MAX_ADDRESS_LEN);
    mset_StationID(val);
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function sets the MAC Address required for the MH/PHY Test.*/
INLINE UWORD8* get_phy_test_dst_addr(void)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    g_cfg_val[0] = MAX_ADDRESS_LEN;
    memcpy(&g_cfg_val[1], g_test_config.tx.da, MAX_ADDRESS_LEN);
    return g_cfg_val;
#else /* MAC_HW_UNIT_TEST_MODE */
    return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

INLINE void set_phy_test_dst_addr(UWORD8* val)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    memcpy(g_test_config.tx.da, val, MAX_ADDRESS_LEN);
#endif /* MAC_HW_UNIT_TEST_MODE */
}


/* This function sets the MAC Address required for the MH/PHY Test.*/
INLINE UWORD32 get_phy_test_txd_pkt_cnt(void)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    return g_test_stats.txci;
#else /* MAC_HW_UNIT_TEST_MODE */
    return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

INLINE void set_phy_test_txd_pkt_cnt(UWORD32 val)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    g_test_stats.txci = 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function is used to disconnect the chosen STA */
INLINE void disconnect_station(UWORD8 assoc_id)
{
#ifdef IBSS_BSS_STATION_MODE
    handle_wps_cred_update();
    disconnect_sta(assoc_id);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    if(assoc_id == 0)
    {
       /* Inform all the associated stations and delete their entries */
       unjoin_all_stations(UNSPEC_REASON);
    }
    else
    {
       /* Inform  the required station and delete its entry */
       unjoin_station(UNSPEC_REASON, assoc_id);
    }
#endif /* BSS_ACCESS_POINT_MODE */
}


/* This function sets the MAC Address required for the MH/PHY Test.*/
INLINE UWORD8* get_misc_test_modes(void)
{
    return 0;
}

INLINE void set_misc_test_modes(UWORD8* val)
{
#ifdef DEBUG_MODE
    /* Dont do anything during a mac reset */
    if(g_reset_mac_in_progress == BTRUE)
    {
        return;
    }

    if(val == NULL)
    {
        return;
    }

    /* Print statistics if string entered is 'Px' */
    if((val[0] == 'P') || (val[0] == 'p'))
    {
        /* print the statistics */

    }
#ifdef IBSS_BSS_STATION_MODE
    else if((val[0] == 'S') || (val[0] == 's'))
    {
        if(val[1] == '0')
        {
            set_machw_listen_interval(mget_listen_interval());
            reset_machw_ps_rdtim_bit();
            set_machw_ps_doze_mode();
            set_machw_ps_pm_tx_bit();
            PRINTD("PS Doze Set-Wake On DTIM Reset\n\r");
        }
        else if(val[1] == '1')
        {
            set_machw_listen_interval(mget_listen_interval());
            set_machw_ps_rdtim_bit();
            set_machw_ps_doze_mode();
            set_machw_ps_pm_tx_bit();
            PRINTD("PS Doze Set-Wake On DTIM Set\n\r");
        }
        else if(val[1] == '2')
        {
            reset_hw_ps_mode();
            PRINTD("PS Awake Set");
        }
        else if(val[1] == '3')
        {
            /* Suspend MAC H/w transmission before disabling it */
            set_machw_tx_suspend();
            /* Disable MAC H/w and PHY */
            disable_machw_phy_and_pa();
            /* Power down the PHY */
            power_down_phy();
            PRINTD("MAC-HW suspended/disabled\n\r");
        }
        else if(val[1] == '4')
        {
            /* Power up the PHY */
            power_up_phy();
            /* Enable MAC H/w and PHY */
            enable_machw_phy_and_pa();
            /* Resume MAC H/w transmission after enabling it */
            set_machw_tx_resume();
            PRINTD("MAC-HW resumed/enabled\n\r");
        }

        //chenq mod
        //PRINTD("PM-Con-Register=0x%.8x\n\r", rMAC_PM_CON);
        PRINTD("PM-Con-Register=0x%.8x\n\r", host_read_trout_reg( (UWORD32)rMAC_PM_CON));
   }
#endif /* IBSS_BSS_STATION_MODE */
#ifdef MAC_802_11H
    else if((val[0] == 'H') || (val[0] == 'h'))
    {
        if(val[1] == '0')
        {
#ifdef BSS_ACCESS_POINT_MODE
            print_channel_table();
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
        //chenq mod
        //PRINTD("TSF H,L : %x,%x\n",rMAC_TSF_TIMER_HI,rMAC_TSF_TIMER_LO);
        PRINTD("TSF H,L : %x,%x\n",
                host_read_trout_reg( (UWORD32)rMAC_TSF_TIMER_HI ),
                host_read_trout_reg( (UWORD32)rMAC_TSF_TIMER_LO ));
#endif /* IBSS_BSS_STATION_MODE */

        }
        else if(val[1] == '1')
        {
            post_radar_detect_event();
        }
        return;
    }
#endif /* MAC_802_11H */
#endif /* DEBUG_MODE */
}

/* Set the read MAC Address from the SDRAM */
INLINE void set_read_addr_sdram(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_addr_sdram = val;
}

/* Get the read MAC Address from the SDRAM */
INLINE UWORD8 get_read_addr_sdram(void)
{
    return g_addr_sdram;
}

/* Set the new DTIM Period */
INLINE void set_dtim_period(UWORD8 val)
{
    mset_DTIMPeriod(val);
    init_dtim_period(val);

#ifdef MAC_HW_UNIT_TEST_MODE
        g_test_params.dtim_period = val;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* Set the local Stack IP Address */
INLINE void set_stack_ip_addr(UWORD32 val)
{
    g_src_ip_addr[0] = (val & 0xFF000000) >> 24;
    g_src_ip_addr[1] = (val & 0x00FF0000) >> 16;
    g_src_ip_addr[2] = (val & 0x0000FF00) >> 8;
    g_src_ip_addr[3] = val & 0x000000FF;
}

/* Get the local Stack IP Address */
INLINE UWORD32 get_stack_ip_addr(void)
{
    UWORD32 temp = 0;
    temp = g_src_ip_addr[3] |
           ((g_src_ip_addr[2] << 8) & 0x0000FF00) |
           ((g_src_ip_addr[1] << 16) & 0x00FF0000) |
           ((g_src_ip_addr[0] << 24) & 0xFF000000);
    return temp;
}


/* Set the local Stack netmask */
INLINE void set_stack_netmask_addr(UWORD32 val)
{
    g_src_netmask_addr[0] = val & 0x000000FF;
    g_src_netmask_addr[1] = (val & 0x0000FF00) >> 8;
    g_src_netmask_addr[2] = (val & 0x00FF0000) >> 16;
    g_src_netmask_addr[3] = (val & 0xFF000000) >> 24;
}

/* Get the local Stack netmask */
INLINE UWORD32 get_stack_netmask_addr(void)
{
    UWORD32 temp = 0;
    temp = g_src_netmask_addr[0] |
           ((g_src_netmask_addr[1] << 8) & 0x0000FF00) |
           ((g_src_netmask_addr[2] << 16) & 0x00FF0000) |
           ((g_src_netmask_addr[3] << 24) & 0xFF000000);
    return temp;
}

/* Set Phy active Register */
INLINE void set_phy_active_reg(UWORD8 val)
{
    g_phy_active_reg = val;
}

/* Get Phy active Register */
INLINE UWORD8 get_phy_active_reg(void)
{
    return g_phy_active_reg;
}

/* Get Phy active Register value */
INLINE UWORD32 get_phy_active_reg_val(void)
{
    UWORD32 temp = 0;
    if(is_valid_phy_reg(g_phy_active_reg) == BTRUE)
        read_dot11_phy_reg(g_phy_active_reg, &temp);
    return temp;
}

/* Set Phy active Register value */
INLINE void set_phy_active_reg_val(UWORD32 val)
{
	/* this make change the phy bank, so return by zhao */
	printk("@@@ WARNNING: this is called unexpected!val:%x\n",val);
	return;
	if(is_valid_phy_reg(g_phy_active_reg) == BTRUE){
		update_phy_init_table(g_phy_active_reg, val);
		write_dot11_phy_reg(g_phy_active_reg, val);
	}
}

/* Set RF Register value */
INLINE void set_rf_reg_val(UWORD32 val)
{
    UWORD32 addr  = val & 0xF;
    UWORD32 value = (val & 0x000FFFF0) >> 4;
    write_RF_reg(addr, value);
}

/* Get the RSSI */
INLINE WORD8 get_rssi(void)
{
	//printk("chenq_itm: get_rssi %d\n",g_rssi);
    return g_rssi;
}

//chenq add get the snr
INLINE WORD8 get_snr(void)
{
	//printk("chenq_itm: get_snr %d\n",g_snr);
    return g_snr;
}

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP

/* get the wapi mode */
INLINE WORD8 get_wapi_mode(void)
{
	
    if( mget_wapi_enable() == TV_TRUE )
    {
    	//printk("chenq_itm: get_wapi_mode 1\n");
		return 1;
	}
	else
	{
		//printk("chenq_itm: get_wapi_mode 0\n");
		return 0;
	}
}

/* set the wapi mode */
INLINE void set_wapi_mode(UWORD8 val)
{
	printk("chenq_itm: set_wapi_mode val = %d\n",val);
	if(val)
	{
		mset_wapi_enable(TV_TRUE);
		printk("chenq_itm: set_wapi_mode TV_TRUE\n");
	}
	else
	{
		mset_wapi_enable(TV_FALSE);
		printk("chenq_itm: set_wapi_mode TV_FALSE\n");
	}
}

INLINE void set_wapi_key(UWORD8 * val)
{
	int index  = 0;
	int offset = 0;
/*************************************************************************/
/*                        set_wapi_key wid Format                        */
/* --------------------------------------------------------------------  */
/* |PAIRWISE/group|key index|BSSID|RSC|key(rxtx + mic)|                  */
/* --------------------------------------------------------------------  */
/* | 1            |1        |6    |16 |32 |                              */
/* --------------------------------------------------------------------  */
/*                                                                       */
/*************************************************************************/

	printk("chenq set_wapi_key !\n");

	if(val[offset] == 0)//group
	{
		offset++;
		index = val[offset];
		mset_wapi_group_key_index(index);

		offset++;
		//chenq mask 2013-02-16
		//mset_wapi_address(&val[offset]);
		
		offset += 6;
		mset_wapi_group_key_rsc(&val[offset]);

		offset += 16;
		mset_wapi_group_pkt_key(index,&val[offset]);

		offset += 16;
		mset_wapi_group_mic_key(index,&val[offset]);

		mset_wapi_key_ok(TV_TRUE);
	}
	else if(val[offset] == 1)//pairwise
	{
		offset++;
		index = val[offset];
		mset_wapi_pairwise_key_index(index);

		offset++;
		mset_wapi_address(&val[offset]);
		
		offset += 6;
		mset_wapi_pairwise_key_txrsc(&val[offset]);

		/* add rx src*/
		val[offset + 15] += 1;
		mset_wapi_pairwise_key_rxrsc(&val[offset]);

		offset += 16;
		mset_wapi_pairwise_pkt_key(index,&val[offset]);

		offset += 16;
		mset_wapi_pairwise_mic_key(index,&val[offset]);
	}

}
#endif

INLINE void set_FailedCount_to_zero(UWORD32 val)
{
    mset_FailedCount(0);
}

INLINE void set_RetryCount_to_zero(UWORD32 val)
{
    mset_RetryCount(0);
}

INLINE void set_MultipleRetryCount_to_zero(UWORD32 val)
{
    mset_MultipleRetryCount(0);
}

INLINE void set_FrameDuplicateCount_to_zero(UWORD32 val)
{
    mset_FrameDuplicateCount(0);

    /* Reset the value of Duplicate count register */
    set_dup_count(0);
}

INLINE void set_ACKFailureCount_to_zero(UWORD32 val)
{
    mset_ACKFailureCount(0);
}

INLINE void set_ReceivedFragmentCount_to_zero(UWORD32 val)
{
    mset_ReceivedFragmentCount(0);
}

INLINE void set_MulticastReceivedFrameCount_to_zero(UWORD32 val)
{
    mset_MulticastReceivedFrameCount(0);
}

INLINE void set_FCSErrorCount_to_zero(UWORD32 val)
{
    mset_FCSErrorCount(0);

    /* Reset the value of FCS error count register */
    set_fcs_count(0);
}

INLINE void set_TransmittedFrameCount_to_zero(UWORD32 val)
{
    mset_TransmittedFrameCount(0);
}


INLINE UWORD8 get_user_control_enabled(void)
{
    return g_user_control_enabled;
}

INLINE void set_user_control_enabled(UWORD8 val)
{
    g_user_control_enabled = (BOOL_T) val;
    if(g_reset_mac_in_progress == BFALSE)
    {
        if(g_user_control_enabled == BFALSE)
        {
            set_default_tx_power_levels();
        }
    }
}

INLINE UWORD8 get_tx_cal(void)
{
    return g_wid_tx_cal;
}

INLINE void set_tx_cal(UWORD8 input)
{
    g_wid_tx_cal = input;
}

/* Get the Current TSSI 11b */
INLINE UWORD8 get_dscr_tssi_11b(void)
{
    return g_dscr_tssi_11b;
}

/* Get the Current TSSI 11g */
INLINE UWORD8 get_dscr_tssi_11g(void)
{
    return g_dscr_tssi_11gn;
}

/* Get the Current TSSI 11n */
INLINE UWORD8 get_dscr_tssi_11n(void)
{
    return g_dscr_tssi_11gn;
}

#ifdef MAC_HW_UNIT_TEST_MODE
/* Get the Extra RSSI */
INLINE WORD8 get_rssi_ex(void)
{
    return g_rssi_ex;
}

/* Get the Extra RSSI (adjusted) */
INLINE WORD8 get_adj_rssi_ex(void)
{
    return g_adj_rssi_ex;
}

/* Get the Extra RSSI count */
INLINE UWORD16 get_rssi_ex_count(void)
{
    return g_rssi_ex_count;
}

/* Set the Extra RSSI count */
INLINE void set_rssi_ex_count(UWORD16 count)
{
    g_rssi_ex_count = count;
}
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
/* Force reset MAC and restart the H/w Unit test */
INLINE void set_restart_hut_test(UWORD8 val)
{
    g_reset_mac = BTRUE;
}
#endif /* MAC_HW_UNIT_TEST_MODE */

/* This function sets the SMPS mode */
INLINE void set_smps_mode(UWORD8 val)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    set_11n_smps_mode(val);
#else  /* MAC_HW_UNIT_TEST_MODE */
    g_test_params.smps_mode = val;
    mset_MIMOPowerSave(g_test_params.smps_mode);
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function gets the SMPS Mode */
INLINE UWORD8 get_smps_mode(void)
{
#ifndef MAC_HW_UNIT_TEST_MODE
    return get_11n_smps_mode();
#else  /* MAC_HW_UNIT_TEST_MODE */
    return g_test_params.smps_mode;
#endif /* MAC_HW_UNIT_TEST_MODE */
}
/* ITM_DEBUG */
#ifndef MAC_HW_UNIT_TEST_MODE
/* This function prints different types of test statistics.                  */
INLINE UWORD8 print_tx_log(void)
{
    UWORD8 print_flag = 0;
    
    PRINTK("\n====  Transmit statistics log ====\n");
#ifdef DEBUG_MODE
    print_flag |= printe("TxC Interrupts", g_mac_stats.itxc);
    print_flag |= printe("TBTT Interrupts", g_mac_stats.itbtt);
    print_flag |= printe("tx is busy!count", g_mac_stats.litxe); //dumy add for tx busy count
#endif /* DEBUG_MODE */
    print_flag |= printe("Tx Fragments", mget_TransmittedFragmentCount());
    print_flag |= printe("Tx Frames", mget_TransmittedFrameCount());
    print_flag |= printe("ACK Failures", mget_ACKFailureCount());
    print_flag |= printe("Failed Frames", mget_FailedCount());
    print_flag |= printe("Retried Frames", mget_RetryCount());
    print_flag |= printe("Multiple Retried Frames",
                                       mget_MultipleRetryCount());
#ifdef PHY_802_11n
    print_flag |= printe("Tx AMSDU Frames", mget_TransmittedAMSDUCount());
    print_flag |= printe("Failed AMSDU Frames", mget_FailedAMSDUCount());
    print_flag |= printe("Retried AMSDU Frames", mget_RetryAMSDUCount());
    print_flag |= printe("Mult Ret AMSDU Frames",
                                       mget_MultipleRetryAMSDUCount());
    print_flag |= printe("AMSDU ACK Failures", mget_AMSDUAckFailureCount());
    print_flag |= printe("Tx AMPDU Frames", mget_TransmittedAMPDUCount());
#endif /* PHY_802_11n */
    PRINTK("==================================\n");
    return print_flag;
}

INLINE UWORD8 print_rx_log(void)
{
    UWORD8 print_flag = 0;
#ifdef DEBUG_MODE
    UWORD32 val = 0; //i = 0;
#endif

    PRINTK("\n====  Receive statistics log ===\n");
#ifdef DEBUG_MODE
    print_flag |= printe("RxC Interrupt", g_mac_stats.irxc);
    print_flag |= printe("HighPri RxC Interrupt", g_mac_stats.ihprxc);
    print_flag |= printe("invalid rx complete isr 000!count", g_mac_stats.lirrd); //dumy add for invalid rx complete isr 000 count
#endif /* DEBUG_MODE */
    //print_flag |= printe("Rx Frames", mget_ReceivedFragmentCount());
    print_flag |= printe("Rx Data Frames", mget_ReceivedFragmentCount());//dumy add for test 0726
    
#if 0
    set_rx_frame_filter_count(0);
    set_rx_mac_header_filter_count(0);
    set_rxq_full_filter_count(0);
#endif
    
#ifdef PHY_802_11n
    print_flag |= printe("Rx AMSDU Frames", mget_ReceivedAMSDUCount());
    print_flag |= printe("Rx AMPDU Frames", mget_AMPDUReceivedCount());
#endif /* PHY_802_11n */

#ifdef DEBUG_MODE
    if(g_mac_stats.rx_data_count != 0)
    {
        val = ((g_mac_stats.rx_data_dscr_total_num*10) / g_mac_stats.rx_data_count);
        print_flag |= printe("DMA Data num_dscr average val ", val); //dumy add for rx data 
    }
    if(g_mac_stats.rx_manage_count != 0)
    {
        val = ((g_mac_stats.rx_manage_dscr_total_num*10) / g_mac_stats.rx_manage_count);
        print_flag |= printe("WiFi Rx Data num_dscr average val", val); //dumy add for rx manage 
    }
    
    print_flag |= printe("WiFi Rx Data num_dscr = 1", g_mac_stats.num_1_count);    
    print_flag |= printe("WiFi Rx Data num_dscr = 2", g_mac_stats.num_2_count);        
    print_flag |= printe("WiFi Rx Data 2< num_dscr < 10", g_mac_stats.num_lis_10_count);   
    print_flag |= printe("WiFi Rx Data 10< num_dscr < 20", g_mac_stats.num_lis_20_count);       
    print_flag |= printe("WiFi Rx Data 20< num_dscr < 30", g_mac_stats.num_lis_30_count);       
    print_flag |= printe("WiFi Rx Data 30< num_dscr < 40", g_mac_stats.num_lis_40_count);       
  
	print_flag |= printe("scaned AP count", g_mac_stats.scan_ap_count);         
	print_flag |= printe("rcv prob rsp count", g_mac_stats.rcv_prob_rsp_count);
  	
    preempt_disable();
    g_mac_stats.rx_data_dscr_total_num = 0;
    g_mac_stats.rx_data_count = 0;
    g_mac_stats.rx_manage_dscr_total_num = 0;    
    g_mac_stats.rx_manage_count = 0;
    g_mac_stats.num_1_count = 0;
    g_mac_stats.num_2_count = 0;
    g_mac_stats.num_lis_10_count = 0;
    g_mac_stats.num_lis_20_count = 0;
    g_mac_stats.num_lis_30_count = 0;    
    g_mac_stats.num_lis_40_count = 0; 

    g_mac_stats.scan_ap_count = 0;
    g_mac_stats.rcv_prob_rsp_count = 0;
    preempt_enable();
#endif /* DEBUG_MODE */
    PRINTK("================================\n");
    return print_flag;
}


INLINE UWORD8 print_err_log(void)
{
    UWORD8 print_flag = 0;
#ifdef DEBUG_MODE
    PRINTK("\n====  Error statistics log ===\n");
    print_flag |= printe("NoOfMACHwErrorInt", g_mac_stats.ierr);
    print_flag |= printe("UnExpectedRxQEmpty", g_mac_stats.erxqemp);
    print_flag |= printe("UnExpectedHRxQEmpty", g_mac_stats.ehprxqemp);
    print_flag |= printe("UnExpectedNonPndMSDU", g_mac_stats.enpmsdu);
    print_flag |= printe("TxIntrFifoOverrun", g_mac_stats.etxfifo);
    print_flag |= printe("RxIntrFifoOverrun", g_mac_stats.erxfifo);
    print_flag |= printe("UnExpectedTxQueEmpty", g_mac_stats.etxqempt);
    print_flag |= printe("HPRxIntrFifoOverrun", g_mac_stats.ehprxfifo);
    print_flag |= printe("UnExpectedMacHang", g_mac_stats.etxsus1machang);
    print_flag |= printe("UnExpectedPhyHang", g_mac_stats.etxsus1phyhang);
    print_flag |= printe("RxPathWdtToFcsFailed", g_mac_stats.ewdtofcsfailed);
    print_flag |= printe("RxPathWdtToFcsPassed", g_mac_stats.ewdtofcspassed);
    print_flag |= printe("UnExpectedMsduAddr", g_mac_stats.emsaddr);
    print_flag |= printe("UnExpectedTxStatus3", g_mac_stats.etxsus3);
    print_flag |= printe("BusError", g_mac_stats.ebus);
    print_flag |= printe("BusWrapSigError", g_mac_stats.ebwrsig);
    print_flag |= printe("OtherErrors", g_mac_stats.eother);
    print_flag |= printe("RxFrmHdrAddr4Prsnt", g_mac_stats.rxfrmhdraddr4prsnt);
    print_flag |= printe("RxFrmHdrBssidChkFail", g_mac_stats.rxfrmhdrbssidchkfail);
    print_flag |= printe("RxFrmHdrProtVerFail", g_mac_stats.rxfrmhdrprotverfail);
    print_flag |= printe("RxFrmHdrAddrChkFail", g_mac_stats.rxfrmhdraddrchkfail);
    print_flag |= printe("RxUnexpFrm", g_mac_stats.rxunexpfrm);
    print_flag |= printe("HifBusy", g_mac_stats.hifbusy);
    print_flag |= printe("HifTxQueFull", g_mac_stats.hifnotready);
    print_flag |= printe("HifTxDataQueFail", g_mac_stats.hifqfail);
    print_flag |= printe("HifTxCnfgQueFail", g_mac_stats.hifcnfgqfail);
    print_flag |= printe("HostRxLenException", g_mac_stats.ehrxexc1);
    print_flag |= printe("InvalidCT", g_mac_stats.invalidct);
    print_flag |= printe("RxMaxLenExc", g_mac_stats.pwrx_maxlenexc);
    print_flag |= printe("RxMinLenExc", g_mac_stats.pwrx_minlenexc);
    print_flag |= printe("BroadcastRxFrameDropped", g_mac_stats.brx_frame_droped);
    print_flag |= printe("UnicastRxFrameDropped", g_mac_stats.urx_frame_droped);
    print_flag |= printe("Tx-Rate Reinit Failed", g_mac_stats.txrate_reinit_err);
    print_flag |= printe("NoMCSIndexExc", g_mac_stats.no_mcs_index);
    print_flag |= printe("NonMCSTxRateExc", g_mac_stats.non_mcs_txrate);
    print_flag |= printe("STA_Not_Associated", g_mac_stats.stanotassc);
    print_flag |= printe("NonHTWMM_STA", g_mac_stats.nonhtwmmsta);
    print_flag |= printe("No_STA_Found", g_mac_stats.nostafound);
    print_flag |= printe("Re-AllcErrPtrn", g_mem_stats.reallocexc);
    print_flag |= printe("Corrupt-AllcErrPtrn", g_mem_stats.corruptallocexc);
    print_flag |= printe("NoSizeAlloc", g_mem_stats.nosizeallocexc);
    print_flag |= printe("NullFreeExc", g_mem_stats.nullfreeexc);
    print_flag |= printe("OOBFreeExc", g_mem_stats.oobfreeexc);
    print_flag |= printe("InvldFreeExc", g_mem_stats.invalidfreeexc);
    print_flag |= printe("Re-FreeErrPtrn", g_mem_stats.refreeexc);
    print_flag |= printe("Corrupt-FreeErrPtrn", g_mem_stats.corruptfreeexc);
    PRINTK("================================\n");
#endif /* DEBUG_MODE */
    return print_flag;
}

INLINE UWORD8 print_ps_stats(void)
{
    UWORD8 print_flag = 0;
    PRINTK("\n====  Powersave statistics log ===\n");
#ifdef DEBUG_MODE
    print_flag |= printe("NoHostRxWakeUps", g_mac_stats.num_wakeup_on_hostrx);
    print_flag |= printe("NoTbttWakeUps", g_mac_stats.num_wakeup_on_tbtt);
    print_flag |= printe("NoDtimWakeUps", g_mac_stats.num_wakeup_on_dtim);
    print_flag |= printe("NoTimWakeUps", g_mac_stats.num_wakeup_on_tim);
    print_flag |= printe("NoAwake2Active", g_mac_stats.num_wake2active_trn);
    print_flag |= printe("NoActive2Awake", g_mac_stats.num_active2wake_trn);
    print_flag |= printe("NoStaSleeps", g_mac_stats.num_sta_doze);
#endif /* DEBUG_MODE */
    PRINTK("================================\n");
    return print_flag;
}

INLINE void print_mac_register(void)	//add by chengwg.
{
#ifdef DEBUG_MODE
	display_general_register();
	display_reception_register();
	display_EDCA_register();
	display_HCCA_STA_register();
	display_TSF_register();
	display_protection_SIFS_response_register();
	display_channel_access_mgmt_register();
	display_retry_register();
	display_sequence_number_register();
	display_PCF_register();
	display_power_mgmt_register();
	display_interrupt_register();
	display_phy_interface_register();
	display_block_ack_register();
#endif	/* DEBUG_MODE */	
}

INLINE void clear_all_statistics(void)
{
#ifdef DEBUG_MODE
    mem_set((UWORD8*)&g_mac_stats, 0, sizeof(g_mac_stats));
    mem_set((UWORD8*)&g_mem_stats, 0, sizeof(g_mem_stats));
#endif /* DEBUG_MODE */
}

INLINE void print_test_stats(UWORD8 val)
{
#ifdef DEBUG_MODE
    UWORD8 tx_log_bit  = 0;
    UWORD8 rx_log_bit  = 0;
    UWORD8 err_log_bit = 0;
    UWORD8 mem_log_bit = 0;
    UWORD8 ps_log_bit  = 0;

    if(g_reset_mac_in_progress == BTRUE)
        return;

    if((val == 0) || (val == 1))
        tx_log_bit  = print_tx_log();

    if((val == 0) || (val == 2))
        rx_log_bit  = print_rx_log();

    if((val == 0) || (val == 3))
        err_log_bit = print_err_log();

	if(val == 0)
		print_mac_register();	//add by chengwg.
	
    if(val == 4)
        clear_all_statistics();

#ifdef MEM_DEBUG_MODE
    if((val == 0) || (val == 7))
        mem_log_bit = print_mem_stats();
#endif /* MEM_DEBUG_MODE */

    if((val == 0) || (val == 8))
    {
        ps_log_bit = print_ps_stats();
    }

    g_11n_print_stats = (mem_log_bit << 6) + (ps_log_bit << 3) +
                        (err_log_bit << 2) + (rx_log_bit << 1) + tx_log_bit;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
    if((val == 0) || (val == 5))
        print_delay_statistics();

    if(val == 6)
        clear_delay_statistics();
#endif /* MEASURE_PROCESSING_DELAY */

}
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
/* This function gets the time taken for completing the Tx Test */
INLINE UWORD32 get_hut_tx_time(void)
{
    return g_total_tx_time;
}
#endif /* MAC_HW_UNIT_TEST_MODE */

/* This function configures the Tx Abort feature */
INLINE void set_tx_abort_config(UWORD8 val)
{
#ifdef TX_ABORT_FEATURE
    switch(val)
    {
    case 0: /* Disable Self CTS transmission on Tx Abort */
    {
        disable_machw_selfcts_on_txab();
#ifdef TEST_TX_ABORT_FEATURE
        g_machw_selfcts_on_txab = BFALSE;
#endif /* TEST_TX_ABORT_FEATURE */
    }
    break;
    case 1: /* Enable Self CTS transmission on Tx Abort */
    {
        enable_machw_selfcts_on_txab();
#ifdef TEST_TX_ABORT_FEATURE
        g_machw_selfcts_on_txab = BTRUE;
#endif /* TEST_TX_ABORT_FEATURE */
    }
    break;
    case 2: /* Disable Tx Abort and Resume Normal Tx */
    {
        disable_machw_tx_abort();
    }
    break;
    case 3: /* Enable Tx Abort */
    {
        enable_machw_tx_abort();
#ifdef ENABLE_MACHW_KLUDGE
        disable_machw_phy_and_pa();
        /* Wait for about 20us for MAC H/w to enter TX-Abort state */
        add_calib_delay(2);
        reset_pacore();
        reset_cecore();
        enable_machw_phy_and_pa();
#endif /* ENABLE_MACHW_KLUDGE */

    }
    break;
    case 4: /* Enable H/w triggered Tx Abort */
    {
        enable_machw_txabort_hw_trig();
    }
    break;
    case 5: /* Enable S/w triggered Tx Abort */
    {
        enable_machw_txabort_sw_trig();
    }
    break;
    case 6: /* Disable S/w triggered Tx Abort test */
    {
#ifdef TEST_TX_ABORT_FEATURE
        g_sw_trig_tx_abort_test = BFALSE;
#endif /* TEST_TX_ABORT_FEATURE */
    }
    break;
    case 7: /* Enable S/w triggered Tx Abort test */
    {
#ifdef TEST_TX_ABORT_FEATURE
        if(g_sw_trig_tx_abort_test == BFALSE)
        {
            g_sw_trig_tx_abort_test = BTRUE;
            start_tx_abort_timer(MIN_INTER_ABORT_DUR_IN_MS);
        }
#endif /* TEST_TX_ABORT_FEATURE */
    }
    break;
    case 8:
    {
#ifdef TEST_TX_ABORT_FEATURE
        PRINTK("Num Txabrt = %d\n",g_numtxabrt);
#endif /* TEST_TX_ABORT_FEATURE */
    }

    default:
    {
        /* Do nothing */
    }
    break;
    }
#endif /* TX_ABORT_FEATURE */
}

/* Function to add WEP key for STA */
INLINE void add_wep_key_bss_sta(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
    add_wep_key_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Function to remove WEP key */
INLINE void remove_wep_key(UWORD8 *index)
{
#ifdef IBSS_BSS_STATION_MODE
    remove_wep_key_sta(index);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Function to add PTK */
INLINE void add_ptk(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
    add_ptk_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Function to add rx GTK */
INLINE void add_rx_gtk(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
    add_rx_gtk_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Function to remove key */
INLINE void remove_key(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
    remove_key_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Function to update PMKID Information */
INLINE void set_pmkid_info(UWORD8 *val)
{
#ifdef IBSS_BSS_STATION_MODE
    set_pmkid_cache(val);
#endif /* IBSS_BSS_STATION_MODE */
}

/* Function to query PMKID Information */
INLINE UWORD8 *get_pmkid_info(void)
{
    UWORD8 *retval = NULL;

#ifdef IBSS_BSS_STATION_MODE
    retval = get_pmkid_cache();
#endif /* IBSS_BSS_STATION_MODE */

    return retval;
}

/* Function to get association request information */
/* this information is useful for external supplicant */
INLINE UWORD8* get_assoc_req_info(void)
{
    UWORD8 *assoc_req = NULL;

#ifdef IBSS_BSS_STATION_MODE
        assoc_req = (UWORD8*)g_assoc_req_info;
#endif /* IBSS_BSS_STATION_MODE */
    return assoc_req;
}


/* Get the current value of site servey */
INLINE UWORD8 get_site_survey_status(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return (UWORD8)g_site_survey_enabled;
#else  /* IBSS_BSS_STATION_MODE */
    return 0;
#endif /* IBSS_BSS_STATION_MODE */
}

/* Set the site survey option according to the value */
INLINE void set_site_survey(UWORD8 val)
{
#ifdef IBSS_BSS_STATION_MODE
    /* Handle WPS related credential update */
    handle_wps_cred_update();

    /* Input Sanity check */
    if(val > 2)
        return;

    g_site_survey_enabled = (SITE_SURVEY_T)val;

    if(g_site_survey_enabled == SITE_SURVEY_OFF)
    {
        g_reset_mac = BTRUE;
    }
#endif /* IBSS_BSS_STATION_MODE */
}


/* Function to get association response information */
/* this information is useful for external supplicant */
INLINE UWORD8* get_assoc_res_info(void)
{
    UWORD8 *assoc_res = NULL;
#ifdef IBSS_BSS_STATION_MODE
    assoc_res = (UWORD8*)g_assoc_res_info;
#endif /* IBSS_BSS_STATION_MODE */
    return assoc_res;
}

/* This function is used to configure Host interface for data packets */
INLINE void set_host_data_if_type(UWORD8 host_type)
{
    if(is_valid_data_hif(host_type) == BTRUE)
    {
        g_host_data_if_type = host_type;
    }
}

/* This function is used to get Host interface for to send data packets */
INLINE UWORD8 get_host_data_if_type(void)
{
    return  g_host_data_if_type;
}

/* This function is used to configure Host interface for config packets */
INLINE void set_host_config_if_type(UWORD8 host_type)
{
    if(is_valid_cfg_hif(host_type) == BTRUE)
    {
        g_host_config_if_type   = host_type;
    }

}

/* This function is used to get Host interface for to send config packets */
INLINE UWORD8 get_host_config_if_type(void)
{
    return  g_host_config_if_type;
}

/* This function gets the current test statistics. To interpret these stats at */
/* the host, structure test_stats_struct_t should be the same at the host and  */
/* device side.                                                                */
INLINE UWORD8* get_hut_stats(void)
{
#ifdef MAC_HW_UNIT_TEST_MODE
#ifdef ITTIAM_PHY

    UWORD32 trigger_stat_lsb = 0;
    UWORD32 trigger_stat_msb = 0;

    read_dot11_phy_reg(rAGCTRIGSTATLSB, &trigger_stat_lsb);
    read_dot11_phy_reg(rAGCTRIGSTATMSB, &trigger_stat_msb);
    g_test_stats.rxd.agc_trig_stats = (trigger_stat_msb << 8) + trigger_stat_lsb;

    /* Reset & start the statistics counter*/
    //write_dot11_phy_reg(rPHYRFCNTRL3, 0x03);
    //write_dot11_phy_reg(rPHYRFCNTRL3, 0x01);

    write_dot11_phy_reg(rPHYRFCNTRL3, (PHYRFCNTRL3 |0x03));
    write_dot11_phy_reg(rPHYRFCNTRL3, PHYRFCNTRL3);
#endif /* ITTIAM_PHY */

    g_stats_buf[0] = sizeof(test_stats_struct_t) & 0x00FF;
    g_stats_buf[1] = (sizeof(test_stats_struct_t) & 0xFF00) >> 8;

    memcpy(&g_stats_buf[2], &g_test_stats, sizeof(test_stats_struct_t));

    return g_stats_buf;
#else  /* MAC_HW_UNIT_TEST_MODE */
    return 0;
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function clears the current test statistics. */
INLINE void set_hut_stats(UWORD8* val)
{
#ifdef MAC_HW_UNIT_TEST_MODE
    mem_set(&g_test_stats, 0, sizeof(test_stats_struct_t));
#endif /* MAC_HW_UNIT_TEST_MODE */
}
/* 802.11h related configuration functions */

/* This function is used to set DFS feature */
INLINE void set_802_11H_DFS_mode(BOOL_T val)
{
#ifdef MAC_802_11H
    mset_enableDFS(val);
#endif /* MAC_802_11H */

    if(BFALSE == val)
    {
        disable_radar_detection();
        mask_machw_radar_det_int();
    }
}

/* This function is used to get DFS mode */
INLINE BOOL_T get_802_11H_DFS_mode(void)
{
#ifdef MAC_802_11H
    return mget_enableDFS();
#endif /* MAC_802_11H */

    return  BFALSE;
}

/* This functionis used to set TPC feature */
INLINE void set_802_11H_TPC_mode(BOOL_T val)
{
#ifdef MAC_802_11H
    mset_enableTPC(val);
#endif /* MAC_802_11H */
}

/* This function is used to get TPC mode */
INLINE BOOL_T get_802_11H_TPC_mode(void)
{
#ifdef MAC_802_11H
    return mget_enableTPC();
#endif /* MAC_802_11H */

    return BFALSE;
}

INLINE UWORD8* get_firmware_info(void)
{
    UWORD8* firmware_info = g_cfg_val + 1;

    g_cfg_val[0] = FIRMWARE_INFO_LENGTH;

    mem_set(firmware_info, 0, FIRMWARE_INFO_LENGTH);

    /* Check and set the HOST related flags */
#ifdef ETHERNET_HOST
    firmware_info[HOST_FLAGS_OFFSET] |= BIT0;
#endif /* ETHERNET_HOST */

    /* Check and set the PHY and RF related flags */

#ifdef ITTIAM_PHY
    firmware_info[PHY_RF_FLAGS_OFFSET] |= (0x2 << 1);
#endif /* ITTIAM_PHY */


#ifdef ITTIAM_PHY
#ifdef RF_MAXIM_ITTIAM
#ifdef MAX2830_32
    firmware_info[PHY_RF_FLAGS_OFFSET] |= (0x1 << 4);
#endif /* MAX2830_32 */
#endif /* RF_MAXIM_ITTIAM */

#ifdef RF_AIROHA_ITTIAM
    firmware_info[PHY_RF_FLAGS_OFFSET] |= BIT3;

#ifdef AL2236
    firmware_info[PHY_RF_FLAGS_OFFSET] |= (0x2 << 4);
#endif /* AL2236 */
#ifdef AL7230
    firmware_info[PHY_RF_FLAGS_OFFSET] |= (0x3 << 4);
#endif /* AL7230 */
#endif /* RF_AIROHA_ITTIAM */
#endif /* ITTIAM_PHY */


#ifdef BSS_ACCESS_POINT_MODE
    firmware_info[MAC_FLAGS_OFFSET] |= 0x1;
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
    firmware_info[MAC_FLAGS_OFFSET] |= 0x2;
#endif /* IBSS_BSS_STATION_MODE */

#ifdef MAC_HW_UNIT_TEST_MODE
    firmware_info[MAC_FLAGS_OFFSET] |= 0x3;
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef MAC_802_11I
    firmware_info[MAC_FLAGS_OFFSET] |= BIT2;
#endif /* MAC_802_11I */

// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
#ifdef INTERNAL_11I
    firmware_info[MAC_FLAGS_OFFSET] |= BIT3;
#endif /* INTERNAL_11I */
#else
#ifdef SUPP_11I
    firmware_info[MAC_FLAGS_OFFSET] |= BIT3;
#endif /* SUPP_11I */
#endif

#ifdef MAC_WMM
    firmware_info[MAC_FLAGS_OFFSET] |= BIT4;
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    firmware_info[MAC_FLAGS_OFFSET] |= BIT5;
#endif /* MAC_802_11N */

#ifdef MAC_802_1X
    firmware_info[MAC_FLAGS_OFFSET] |= BIT6;
#endif /* MAC_802_1X */

#ifdef MAC_802_11H
    firmware_info[MAC_FLAGS_OFFSET] |= BIT7;
#endif /* MAC_802_11H */

#ifdef EXCLUDE_11I_IBSS
    firmware_info[MAC_FLAGS_OFFSET + 1] |= BIT0;
#endif /* EXCLUDE_11I_IBSS */

#ifdef NMAC_1X1_MODE
    firmware_info[MAC_FLAGS_OFFSET + 1] |= BIT1;
#endif /* NMAC_1X1_MODE */

#ifdef OS_LINUX_CSL_TYPE
    firmware_info[PLAFORM_FLAGS_OFFSET] |= BIT4;
#endif /* OS_LINUX_CSL_TYPE */

#ifdef DEBUG_MODE
    firmware_info[EXTRA_OPT_FLAGS_OFFSET] |= BIT0;
#endif /* DEBUG_MODE */


#ifdef EDCA_DEMO_KLUDGE
    firmware_info[EXTRA_OPT_FLAGS_OFFSET] |= BIT2;
#endif /* EDCA_DEMO_KLUDGE */

#ifdef AUTORATE_FEATURE
    firmware_info[EXTRA_OPT_FLAGS_OFFSET] |= BIT3;
#endif /* AUTORATE_FEATURE */

#ifdef PHY_TEST_MAX_PKT_RX
    firmware_info[EXTRA_OPT_FLAGS_OFFSET] |= BIT4;
#endif /* PHY_TEST_MAX_PKT_RX */

#ifdef DEFAULT_SME
    firmware_info[EXTRA_OPT_FLAGS_OFFSET] |= BIT5;
#endif /* DEFAULT_SME */

#ifdef NO_ACTION_RESET
    firmware_info[EXTRA_OPT_FLAGS_OFFSET] |= BIT6;
#endif /* NO_ACTION_RESET */


#ifdef TX_ABORT_FEATURE
    firmware_info[EXTRA_OPT_FLAGS_OFFSET + 1] |= BIT0;
#endif /* TX_ABORT_FEATURE */

#ifdef ENABLE_MACHW_KLUDGE
    firmware_info[EXTRA_OPT_FLAGS_OFFSET + 1] |= BIT1;
#endif /* ENABLE_MACHW_KLUDGE */

#ifdef SEND_MAC_STATUS
    firmware_info[EXTRA_OPT_FLAGS_OFFSET + 1] |= BIT3;
#endif /* SEND_MAC_STATUS */

    return g_cfg_val;

}

/* Function to set memory address */
INLINE void set_memory_address(UWORD32 input)
{
    g_memory_address = input;
}

/* Function to get memory address */
INLINE UWORD32 get_memory_address(void)
{
    return g_memory_address;
}

/* Function to get memory access */
INLINE UWORD32 get_memory_access_32bit(void)
{
    if(g_reset_mac_in_progress == BFALSE)
    {
       return *((UWORD32 *)g_memory_address);
    }

    return 0;
}
INLINE UWORD8 get_memory_access_8bit(void)
{
    if(g_reset_mac_in_progress == BFALSE)
    {
        return *(( UWORD8 *)g_memory_address );
    }
    else
    {
        return 0;
    }
}

INLINE void set_memory_access_8bit(UWORD8 input)
{
    if(g_reset_mac_in_progress == BFALSE)
    {
        *(( UWORD8 *)g_memory_address) = input;
    }
}

INLINE UWORD16 get_memory_access_16bit(void)
{
    if(g_reset_mac_in_progress == BFALSE)
    {
        return *((UWORD16 *)g_memory_address);
    }
    else
    {
        return 0;
    }
}

INLINE void set_memory_access_16bit(UWORD16 input)
{
    if(g_reset_mac_in_progress == BFALSE)
    {
        *((UWORD16 *)g_memory_address) = input;
    }
}

/* Function to set memory access */
INLINE void set_memory_access_32bit(UWORD32 input)
{
    if(g_reset_mac_in_progress == BFALSE)
    {
        *((UWORD32 *)g_memory_address) = input;
    }
}

INLINE UWORD16 get_rx_sense(void)
{
    return get_phy_rx_sense();
}

INLINE void set_rx_sense(UWORD16 val)
{
    set_phy_rx_sense(val);
}

INLINE UWORD16 get_active_scan_time(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return g_active_scan_time;
#else /* IBSS_BSS_STATION_MODE */
    return 0;
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE void set_active_scan_time(UWORD16 val)
{
#ifdef IBSS_BSS_STATION_MODE
    g_active_scan_time = MAX(MIN(val,MAX_SCAN_TIME),MIN_SCAN_TIME);
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE UWORD16 get_passive_scan_time(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return g_passive_scan_time;
#else /* IBSS_BSS_STATION_MODE */
    return 0;
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE void set_passive_scan_time(UWORD16 val)
{
#ifdef IBSS_BSS_STATION_MODE
    g_passive_scan_time = MAX(MIN(val,MAX_SCAN_TIME),MIN_SCAN_TIME);
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE UWORD16 get_site_survey_scan_time(void)
{
#ifdef IBSS_BSS_STATION_MODE
    return g_site_survey_scan_time;
#else /* IBSS_BSS_STATION_MODE */
    return 0;
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE void set_site_survey_scan_time(UWORD16 val)
{
#ifdef IBSS_BSS_STATION_MODE
    g_site_survey_scan_time = MAX(MIN(val,MAX_SCAN_TIME),MIN_SCAN_TIME);
#endif /* IBSS_BSS_STATION_MODE */
}

INLINE UWORD16 get_join_start_timeout(void)
{
    return g_join_start_timeout;
}

INLINE void set_join_start_timeout(UWORD16 val)
{
    g_join_start_timeout = val;
}

INLINE UWORD16 get_auth_timeout(void)
{
    return (UWORD16)mget_AuthenticationResponseTimeOut();
}

INLINE void set_auth_timeout(UWORD16 val)
{
    mset_AuthenticationResponseTimeOut((UWORD32)val);
}

INLINE UWORD16 get_asoc_timeout(void)
{
    return (UWORD16)mget_AssociationResponseTimeOut();
}

INLINE void set_asoc_timeout(UWORD16 val)
{
    mset_AssociationResponseTimeOut((UWORD32)val);
}

INLINE UWORD16 get_11i_protocol_timeout(void)
{
#ifdef MAC_802_11I
    return g_11i_protocol_timeout;
#else /* MAC_802_11I */
    return (UWORD16)0;
#endif /* MAC_802_11I */
}

INLINE void set_11i_protocol_timeout(UWORD16 val)
{
#ifdef MAC_802_11I
    g_11i_protocol_timeout = val;
#endif /* MAC_802_11I */
}

INLINE UWORD16 get_eapol_response_timeout(void)
{
#ifdef MAC_802_11I
    return g_eapol_response_timeout;
#else /* MAC_802_11I */
    return (UWORD16)0;
#endif /* MAC_802_11I */
}

INLINE void set_eapol_response_timeout(UWORD16 val)
{
#ifdef MAC_802_11I
    g_eapol_response_timeout = val;
#endif /* MAC_802_11I */
}

/* This function sets the RIFS mode setting desired by the user. However RIFS */
/* gets used only if allowed by the network settings.                         */
INLINE void set_rifs_mode(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_user_allow_rifs_tx = val;

#ifndef MAC_HW_UNIT_TEST_MODE
    if(g_user_allow_rifs_tx == 1)
    {
        /* Check whether RIFS is allowed in the current network */
        if(mget_RIFSMode() == TV_TRUE)
            enable_machw_rifs();
    }
    else
        disable_machw_rifs();
#else  /* MAC_HW_UNIT_TEST_MODE */
    set_hut_rifs_mode(val);
#endif /* MAC_HW_UNIT_TEST_MODE */
}

/* This function returns the RIFS mode setting desired by the user. */
INLINE UWORD8 get_rifs_mode(void)
{
    return g_user_allow_rifs_tx;
}

#ifdef DEBUG_MODE
/* Function to set test case name */
INLINE void set_test_case_name(const char* input)
{
    if(g_reset_mac_in_progress == BFALSE)
        PRINTK("\n\r ---- %s ---- \n\r",input);
}
#endif /* DEBUG_MODE */

/* Set auto rate table */
INLINE void set_auto_rate_table(UWORD8 *val)
{
#ifdef AUTORATE_FEATURE
    set_ar_table(val);
#endif /* AUTORATE_FEATURE */
}

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
/*****************************************************************************/
/* MultiDomain support configuration functions                               */
/*****************************************************************************/
INLINE void set_multi_domain_supp(UWORD8 val)
{
#ifdef MAC_MULTIDOMAIN
	if(val == 1)
	{
		mset_MultiDomainCapabilityEnabled(TV_TRUE);
	}
	else
	{
		mset_MultiDomainCapabilityEnabled(TV_FALSE);
	}
#endif /* MAC_MULTIDOMAIN */
}

INLINE UWORD8 get_multi_domain_supp(void)
{
	UWORD8 retval = 0;

#ifdef MAC_MULTIDOMAIN
    if(mget_MultiDomainOperationImplemented() == TV_TRUE)
        retval = 1;
#endif /* MAC_MULTIDOMAIN */

	return retval;
}
#endif

/*****************************************************************************/
/* 20/40 configuration functions                                             */
/*****************************************************************************/

/* This function returns current configuration of 20/40 coexistence in MAC */
INLINE UWORD8 get_2040_coex_support(void)
{
    UWORD8 retval = 0;


    return retval;
}

/* This function sets support for 20/40 coexistence in MAC */
INLINE void set_2040_coex_support(UWORD8 val)
{
}

/* This function returns current configuration of 40MHz operation in MAC */
INLINE UWORD8 get_2040_enable(void)
{
    UWORD8 retval = 0;


    return retval;
}

/* This function sets support for 40MHz operation in MAC */
INLINE void set_2040_enable(UWORD8 val)
{
}


/* This function returns current configuration of 40MHz intolerance in MAC */
INLINE UWORD8 get_40mhz_intolerant(void)
{
    UWORD8 retval = 0;

#ifdef MAC_802_11N
#ifndef MAC_HW_UNIT_TEST_MODE
    if(mget_FortyMHzIntolerant() == TV_TRUE)
        retval = 1;
#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_802_11N */

    return retval;
}

/* This function sets support for 40MHz intolerance in MAC */
INLINE void set_40mhz_intolerant(UWORD8 val)
{
#ifdef MAC_802_11N
    if(val > 1)
        return;

#ifndef MAC_HW_UNIT_TEST_MODE
    if(val == 1)
        mset_FortyMHzIntolerant(TV_TRUE);
    else
        mset_FortyMHzIntolerant(TV_FALSE);
#endif /* MAC_HW_UNIT_TEST_MODE */
#endif /* MAC_802_11N */
}

/*****************************************************************************/
/* WMM configuration functions for STA mode                                  */
/*****************************************************************************/

/* Function to set the STA UAPSD configuration. Note that currently the      */
/* configuration allows only both delivery and trigger enabled to be set per */
/* AC. It cannot selectively set trigger or delivery enabled for an AC.      */
INLINE void set_uapsd_config(UWORD8 *val)
{
#ifdef MAC_WMM
#ifdef IBSS_BSS_STATION_MODE
    /* First 2 bytes are WID Data length */
    val = val + 2;

    /* --------------------------------------------------------------------- */
    /* Format of U-APSD config packet                 */
    /* --------------------------------------------------------------------- */
    /* | MAX SP   | Trigger/Delivery enabled status | */
    /* | Length   |  AC_BK | AC_BE | AC_VI | AC_VO  | */
    /* --------------------------------------------------------------------- */
    /* |   1      |  1     | 1     | 1     | 1      | */
    /* --------------------------------------------------------------------- */
    set_uapsd_config_max_sp_len(val[0]);
    set_uapsd_config_ac(AC_BK, val[1]);
    set_uapsd_config_ac(AC_BE, val[2]);
    set_uapsd_config_ac(AC_VI, val[3]);
    set_uapsd_config_ac(AC_VO, val[4]);
#endif /* IBSS_BSS_STATION_MODE */
#endif /* MAC_WMM */
}

/* Function to get the STA UAPSD configuration */
INLINE UWORD8* get_uapsd_config(void)
{
    UWORD8 *retval = NULL;
#ifdef MAC_WMM
#ifdef IBSS_BSS_STATION_MODE
    /* ---------------------------------------------- */
    /* Format of U-APSD config message                */
    /* ---------------------------------------------- */
    /* | MAX SP   | Trigger/Delivery enabled status | */
    /* | Length   |  AC_BK | AC_BE | AC_VI | AC_VO  | */
    /* ---------------------------------------------- */
    /* |   1      |  1     | 1     | 1     | 1      | */
    /* ---------------------------------------------- */
    UWORD16 len  = 5; /* Length of U-APSD message */

    /* First 2 Bytes are length field and rest of the bytes are actual data  */
    /* Length field encoding */
    /* +----------------------------------+ */
    /* | BIT15  | BIT14  | BIT13 - BIT0   | */
    /* +----------------------------------+ */
    /* | First  | Last   | Message Length | */
    /* +----------------------------------+ */
    /* This packet is the first as well as last packet of the U-APSD config  */
    /* message                                                               */
    len           = len | 0xC000;
    g_cfg_val[0]  = (UWORD8)(len & 0x00FF);
    g_cfg_val[1]  = (UWORD8)(len & 0xFF00);

    /* Setting U-APSD config message body */
    g_cfg_val[2] = get_uapsd_config_max_sp_len();
    g_cfg_val[3] = get_uapsd_config_ac(AC_BK);
    g_cfg_val[4] = get_uapsd_config_ac(AC_BE);
    g_cfg_val[5] = get_uapsd_config_ac(AC_VI);
    g_cfg_val[6] = get_uapsd_config_ac(AC_VO);

    retval = &g_cfg_val[0];
#endif /* IBSS_BSS_STATION_MODE */
#endif /* MAC_WMM */
    return retval;
}

/* Function to get the current UAPSD status */
INLINE UWORD8* get_uapsd_status(void)
{
    UWORD8 *retval = NULL;

#ifdef MAC_WMM
#ifdef IBSS_BSS_STATION_MODE
    /* --------------------------------------------------------------------- */
    /* Format of U-APSD Status message                                         */
    /* --------------------------------------------------------------------- */
    /* | AP | MaxSP| Delivery enabled status   | Trigger enabled status    | */
    /* | Cap| Len  | AC_BK| AC_BE| AC_VI| AC_VO| AC_BK| AC_BE| AC_VI| AC_VO| */
    /* --------------------------------------------------------------------- */
    /* | 1   | 1     | 1    | 1    | 1    | 1    |  1   | 1    | 1    | 1    | */
    /* --------------------------------------------------------------------- */
    UWORD16 len  = 10; /* Length of U-APSD Status message */

    /* First 2 Bytes are length field and rest of the bytes are actual data  */
    /* Length field encoding */
    /* +----------------------------------+ */
    /* | BIT15  | BIT14  | BIT13 - BIT0   | */
    /* +----------------------------------+ */
    /* | First  | Last   | Message Length | */
    /* +----------------------------------+ */
    /* This packet is the first as well as last packet of the U-APSD Status  */
    /* message                                                               */
    len           = len | 0xC000;
    g_cfg_val[0]  = (UWORD8)(len & 0x00FF);
    g_cfg_val[1]  = (UWORD8)(len & 0xFF00);

    /* The UAPSD status values are set based on if the AP is UAPSD capable */
    if(is_ap_uapsd_capable() == BFALSE)
    {
        /* QAP UAPSD capability and UAPSD status values are set to all 0 if  */
        /* AP is not UAPSD capable                                           */
        mem_set(&g_cfg_val[2], 0, len);
    }
    else
    {
        /* QAP UAPSD capability */
        g_cfg_val[2] = 1;

        /* The UAPSD status values is same as the configured values if the   */
        /* AP is UAPSD capable. Note that currently the trigger and delivery */
        /* enabled configurations are the same. Hence the same values are    */
        /* copied for both.                                                  */
        g_cfg_val[3]  = get_uapsd_config_max_sp_len();
        g_cfg_val[4]  = get_uapsd_config_ac(AC_BK);
        g_cfg_val[5]  = get_uapsd_config_ac(AC_BE);
        g_cfg_val[6]  = get_uapsd_config_ac(AC_VI);
        g_cfg_val[7]  = get_uapsd_config_ac(AC_VO);
        g_cfg_val[8]  = g_cfg_val[4];
        g_cfg_val[9]  = g_cfg_val[5];
        g_cfg_val[10] = g_cfg_val[6];
        g_cfg_val[11] = g_cfg_val[7];
    }

    retval = &g_cfg_val[0];
#endif /* IBSS_BSS_STATION_MODE */
#endif /* MAC_WMM */
    return retval;
}

/*****************************************************************************/
/* WMM configuration functions for AP mode                                   */
/*****************************************************************************/

/* Function to set AC parameter values to be used by the AP */
INLINE void set_wmm_ap_ac_params(UWORD8 *val)
{
#ifdef MAC_WMM
#ifdef BSS_ACCESS_POINT_MODE
    set_ap_ac_params_prot_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
#endif /* MAC_WMM */
}

/* Function to get AC parameter values in use by the AP */
INLINE UWORD8* get_wmm_ap_ac_params(void)
{
    UWORD8 *retval = NULL;

#ifdef MAC_WMM
#ifdef BSS_ACCESS_POINT_MODE
    retval = get_ap_ac_params_prot_ap();
#endif /* BSS_ACCESS_POINT_MODE */
#endif /* MAC_WMM */
    return retval;
}

/* Function to set AC parameter values for STA associated with the AP */
INLINE void set_wmm_sta_ac_params(UWORD8 *val)
{
#ifdef MAC_WMM
#ifdef BSS_ACCESS_POINT_MODE
    set_sta_ac_params_prot_ap(val);
#endif /* BSS_ACCESS_POINT_MODE*/
#endif /* MAC_WMM */
}

/* Function to get AC parameter values for STA associated with the AP */
INLINE UWORD8* get_wmm_sta_ac_params(void)
{
    UWORD8 *retval = NULL;

#ifdef MAC_WMM
   retval = get_sta_ac_params_prot();
#endif /* MAC_WMM */
    return retval;
}

/* Function to set UAPSD SUPPORT in AP */
INLINE void set_uapsd_support_ap(UWORD8 val)
{
#ifdef MAC_WMM
#ifdef BSS_ACCESS_POINT_MODE
    mset_UAPSD_ap(val);
#endif /* BSS_ACCESS_POINT_MODE */
#endif /* MAC_WMM */
}

/* Function to get the UAPSD support of AP */
INLINE UWORD8 get_uapsd_support_ap(void)
{
    UWORD8 retval = 0;

#ifdef MAC_WMM
#ifdef BSS_ACCESS_POINT_MODE
    retval = mget_UAPSD_ap();
#endif /* BSS_ACCESS_POINT_MODE */
#endif /* MAC_WMM */
    return retval;
}

/* Get the connected station list */
INLINE UWORD8* get_connected_sta_list(void)
{
    UWORD16 total_len;
    total_len     = get_all_sta_join_info(&g_cfg_val[2], MAX_CFG_LEN - 2);

    total_len     = total_len | 0xC000;
    g_cfg_val[0]  = (UWORD8)(total_len & 0x00FF);
    g_cfg_val[1]  = (UWORD8)(total_len & 0xFF00);

    return &g_cfg_val[0];
}

/* This funtion returns the value of the g_11n_print_stats which gives the   */
/* information wheather each of Tx, Rx and error logs printed or not.        */
INLINE UWORD8 get_11n_print_stats(void)
{
#ifdef DEBUG_MODE
    return g_11n_print_stats;
#else /* DEBUG_MODE */
    return 0;
#endif /* DEBUG_MODE */
}


/* Generic function to set any String Type WID */
INLINE void set_str_wid(UWORD8 *inp_val, UWORD8 *saved_val, UWORD16 max_len)
{
    UWORD8 len = strlen((const char *)inp_val);
    len = (len > max_len) ? max_len : len;
    memcpy(&saved_val[1], inp_val, len);
    saved_val[0] = len;

    update_device_specific_info();
}

/*****************************************************************************/
/* Device Specific Configuration Functions                                   */
/*****************************************************************************/

INLINE UWORD32 get_dev_os_version(void)
{
    UWORD32 retval = 0;

#ifdef DEVICE_SPECIFIC_CONFIG
    retval = g_dev_os_version;
#endif /* DEVICE_SPECIFIC_CONFIG */

    return retval;
}

INLINE void set_dev_os_version(UWORD32 val)
{
#ifdef DEVICE_SPECIFIC_CONFIG
    g_dev_os_version = val;
#endif /* DEVICE_SPECIFIC_CONFIG */
}

INLINE UWORD8 *get_manufacturer(void)
{
    UWORD8* retval = 0;

#ifdef DEVICE_SPECIFIC_CONFIG
    retval = g_manufacturer;
#endif /* DEVICE_SPECIFIC_CONFIG */

    return retval;
}

INLINE void set_manufacturer(UWORD8 *val)
{
#ifdef DEVICE_SPECIFIC_CONFIG
    set_str_wid(val, g_manufacturer, MAX_MANUFACTURER_ID_LEN);
#endif /* DEVICE_SPECIFIC_CONFIG */
}

INLINE UWORD8 *get_model_name(void)
{
    UWORD8* retval = 0;

#ifdef DEVICE_SPECIFIC_CONFIG
    retval = g_model_name;
#endif /* DEVICE_SPECIFIC_CONFIG */

    return retval;
}

INLINE void set_model_name(UWORD8 *val)
{
#ifdef DEVICE_SPECIFIC_CONFIG
    set_str_wid(val, g_model_name, MAX_MODEL_NAME_LEN);
#endif /* DEVICE_SPECIFIC_CONFIG */
}

INLINE UWORD8 *get_model_num(void)
{
    UWORD8* retval = 0;

#ifdef DEVICE_SPECIFIC_CONFIG
    retval = g_model_num;
#endif /* DEVICE_SPECIFIC_CONFIG */

    return retval;
}

INLINE void set_model_num(UWORD8 *val)
{
#ifdef DEVICE_SPECIFIC_CONFIG
    set_str_wid(val, g_model_num, MAX_MODEL_NUM_LEN);
#endif /* DEVICE_SPECIFIC_CONFIG */
}

INLINE UWORD8 *get_dev_name(void)
{
    UWORD8* retval = 0;

#ifdef DEVICE_SPECIFIC_CONFIG
    retval = g_device_name;
#endif /* DEVICE_SPECIFIC_CONFIG */

    return retval;
}

INLINE void set_dev_name(UWORD8 *val)
{
#ifdef DEVICE_SPECIFIC_CONFIG
    set_str_wid(val, g_device_name, MAX_DEVICE_NAME_LEN);
#endif /* DEVICE_SPECIFIC_CONFIG */
}

INLINE UWORD8 *get_prim_dev_type(void)
{
    UWORD8* retval = 0;

#ifdef DEVICE_SPECIFIC_CONFIG
    retval = g_prim_dev_type;
#endif /* DEVICE_SPECIFIC_CONFIG */

    return retval;
}

INLINE void set_prim_dev_type(UWORD8 *val)
{
#ifdef DEVICE_SPECIFIC_CONFIG
    UWORD16 wid_len  = 0;
    wid_len  = val[0];
    wid_len |= ((UWORD16)val[1] << 8) & 0xFF00;
    if(PRIM_DEV_TYPE_LEN != wid_len)
    {
        return;
    }
    memcpy(g_prim_dev_type, (val), PRIM_DEV_TYPE_LEN+2);

    update_device_specific_info();
#endif /* DEVICE_SPECIFIC_CONFIG */
}



#ifdef MAC_P2P

/* This function sets the P2P listen channel. This function is placed here   */
/* since the valid input values are not accessible from the P2P header files */
INLINE void set_p2p_listen_chan(UWORD16 ch_info)
{
    UWORD8 ch_num = 0;
    UWORD8 ch_idx = 0;
    /* Get the preferred frequency band */
    if(RC_START_FREQ_2 == (ch_info & BIT15))
    {
        ch_num = ch_info & 0xFF;

        /* Compute the channel index using user selected freq range */
        ch_idx = get_ch_idx_from_num(RC_START_FREQ_2, ch_num);
    	TROUT_DBG4("P2P: ch_idx %d\n",ch_idx);

        if((ch_idx == CHANNEL1) || (ch_idx == CHANNEL6) || (ch_idx == CHANNEL11))
        {
            mset_p2p_listen_chan(ch_idx);
        }

    }
}

/* This function returns the P2P listen channel */
INLINE UWORD8 get_p2p_listen_chan(void)
{

    UWORD8 listen_chan = get_ch_num_from_idx(get_current_start_freq(),
                                             mget_p2p_listen_chan());

    return listen_chan;
}

/* This function sets the P2P invitation device ID. This function is placed  */
/* here since the check for MAC state is not accessible from P2P header file */
INLINE void set_p2p_invit_dev_id(UWORD8 *addr)
{
    /* Non persistent Invitation procedure is valid only in ENABLED state */
    if(ENABLED == get_mac_state())
        mset_p2p_invit_dev_id(addr);
}

INLINE UWORD8 *get_req_dev_type(void)
{
    return g_req_dev_type;
}

INLINE void set_req_dev_type(UWORD8 *val)
{
    UWORD16 wid_len  = 0;
    wid_len  = val[0];
    wid_len |= ((UWORD16)val[1] << 8) & 0xFF00;
    if(REQ_DEV_TYPE_LEN != wid_len)
    {
        /* If an invalid dev type is sent then reset the field */
        mem_set(g_req_dev_type, 0, REQ_DEV_TYPE_LEN + 2);
        return;
    }
    memcpy(g_req_dev_type, (val), REQ_DEV_TYPE_LEN + 2);

    update_device_specific_info();
}
#endif /* MAC_P2P */

INLINE UWORD8* get_serial_number(void)
{
    UWORD8* retval = 0;

#ifdef DEVICE_SPECIFIC_CONFIG
    if(0 == g_serial_num[0])
    {
        g_serial_num[0] = 4;
        g_serial_num[1] = '5';
        g_serial_num[2] = '6';
        g_serial_num[3] = '7';
        g_serial_num[4] = '8';
    }

    retval = g_serial_num;
#endif /* DEVICE_SPECIFIC_CONFIG */

    return retval;
}

INLINE void set_serial_number(UWORD8* val)
{
#ifdef DEVICE_SPECIFIC_CONFIG
    set_str_wid(val, g_serial_num, MAX_SERIAL_NUM_LEN);
#endif /* DEVICE_SPECIFIC_CONFIG */
}


INLINE UWORD8 get_device_mode(void)
{
    return g_device_mode;
}

INLINE void set_device_mode(UWORD8 val)
{
    if(BFALSE == g_reset_mac_in_progress)
    {
#ifndef MAC_P2P
        if(val > IBSS_STA)
           return;
#endif /* MAC_P2P */
        TROUT_DBG4("WPS: set device mode from %d to %d\n",g_device_mode,val);
        g_device_mode = val;
    }
}

INLINE void set_DesiredSSID(const WORD8* inp)
{
    handle_wps_cred_update();
    mset_DesiredSSID(inp);
}

/* Function to set WEP default key ID */
INLINE void set_WEPDefaultKeyID(UWORD8 inp)
{
    handle_wps_cred_update();

    mset_WEPDefaultKeyID(inp);
}

/* Set the authentication type */
INLINE void set_auth_type(UWORD8 val)
{
    handle_wps_cred_update();
    set_auth_type_prot(val);
}

/* If WEP is enabled, get the authentication type, otherwise, return 0 */
INLINE UWORD8 get_auth_type(void)
{
    return get_auth_type_prot();
}

/* Set the WEP Key depending on the WEP configuration */
INLINE void set_WEP_key(UWORD8 *val)
{
    handle_wps_cred_update();

    /* Add the WEP key to the MIB and to the CE LUT */
    /* Theres no need to check if WEP is enabled    */
    if(g_wep_type == WEP40)
    {
        /* Convert the received string to hex numbers */
        str_2_hex(val, 5);
        mset_WEPKeyValue(mget_WEPDefaultKeyID(), val, 40);

    }
    else if(g_wep_type == WEP104)
    {
        /* Convert the received string to hex numbers */
        str_2_hex(val, 13);
        mset_WEPKeyValue(mget_WEPDefaultKeyID(), val, 104);
    }
}

/* Set the WEP Key depending on the WEP configuration */
INLINE UWORD8 *get_WEP_key(void)
{
    WORD16 i    = 0;
    UWORD8 *val = mget_WEPDefaultKeyValue();
    UWORD8 size = mget_WEPDefaultKeySize() >> 3;

    g_cfg_val[0] = 2 * size;

    for(i = size - 1; i >= 0; i--)
    {
        g_cfg_val[2 * i + 2] = hex_2_char(val[i] & 0x0F);
        g_cfg_val[2 * i + 1] = hex_2_char((val[i] & 0xF0) >> 4);
    }

    return g_cfg_val;
}

/* Set the RSNA PSK Pass Phrase */
INLINE void set_RSNAConfigPSKPassPhrase(UWORD8* val)
{
    handle_wps_cred_update();
    set_RSNAConfigPSKPassPhrase_prot(val);
}

INLINE UWORD8* get_RSNAConfigPSKPassPhrase(void)
{
    return get_RSNAConfigPSKPassPhrase_prot();
}

/* This function returns the Encyrption Configuration */
INLINE UWORD8 get_802_11I_mode(void)
{
    return get_802_11I_mode_prot();
}

/* This function sets the protection mode */
INLINE void set_802_11I_mode(UWORD8 val)
{
    handle_wps_cred_update();
    set_802_11I_mode_prot(val);
}

/* This function returns the internal supplicant info */
INLINE UWORD8 get_int_supp_mode(void)
{
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
    UWORD8 ret_val = 0;

#ifdef IBSS_BSS_STATION_MODE
    ret_val = get_int_supp_mode_sta();
#endif /* IBSS_BSS_STATION_MODE */

    return ret_val;
#else
    return get_int_supp_mode_prot();
#endif
}

INLINE void set_int_supp_mode(UWORD8 val)
{
// 20120709 caisf mod, merged ittiam mac v1.2 code
#if 0
#ifdef IBSS_BSS_STATION_MODE
    set_int_supp_mode_sta(val);
#endif /* IBSS_BSS_STATION_MODE */
#else
#ifdef IBSS_BSS_STATION_MODE
    set_int_supp_mode_prot(val);
#endif /* IBSS_BSS_STATION_MODE */

#ifdef BSS_ACCESS_POINT_MODE
    /* This can be set in AP mode only once when switching from/to STA    */
    /* mode. Otherwise, it is a read-only WID in AP mode.                 */
    if(BTRUE == is_switch_in_progress())
        set_int_supp_mode_prot(val);
#endif /* BSS_ACCESS_POINT_MODE */
#endif
}

/*****************************************************************************/
/* Configuration functions to get/set TX power level                         */
/*****************************************************************************/

INLINE void set_tx_power_level_11a(UWORD8 val)
{
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_idx = mget_CurrentChannel();
	
	// 20120830 caisf mod, merged ittiam mac v1.3 code
    //set_curr_tx_power_dbm_11a(freq, ch_idx, val, g_user_control_enabled);
	set_curr_tx_power_dbm_11a(freq, ch_idx, val);
}

INLINE UWORD8 get_tx_power_level_11a(void)
{
    return get_curr_tx_power_dbm_11a();
}

INLINE void set_tx_power_level_11b(UWORD8 val)
{
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_idx = mget_CurrentChannel();
	
	// 20120830 caisf mod, merged ittiam mac v1.3 code
    //set_curr_tx_power_dbm_11b(freq, ch_idx, val, g_user_control_enabled);
	set_curr_tx_power_dbm_11b(freq, ch_idx, val);
}

INLINE UWORD8 get_tx_power_level_11b(void)
{
    return get_curr_tx_power_dbm_11b();
}

INLINE void set_tx_power_level_11n(UWORD8 val)
{
    UWORD8 freq   = get_current_start_freq();
    UWORD8 ch_idx = mget_CurrentChannel();
	
	// 20120830 caisf mod, merged ittiam mac v1.3 code
    //set_curr_tx_power_dbm_11n(freq, ch_idx, val, g_user_control_enabled);
	set_curr_tx_power_dbm_11n(freq, ch_idx, val);
}

INLINE UWORD8 get_tx_power_level_11n(void)
{
    return get_curr_tx_power_dbm_11n();
}

INLINE void set_tx_power_level_11n40(UWORD8 val)
{
}

INLINE UWORD8 get_tx_power_level_11n40(void)
{
    UWORD8 retval = 0;


    return retval;
}

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
INLINE UWORD32 get_tx_power_levels_dbm(void)
{
    return get_curr_tx_power_levels_dbm(g_user_control_enabled);
}

/* This function returns the current tx antenna set for 1x1 rates  */
INLINE UWORD8 get_curr_tx_ant_set(void)
{
	UWORD32 ant_vector = get_curr_tx_ant_set_prot();

    return ((UWORD8) (ant_vector & 0xF));
}

/* This function sets the current tx antenna set for 1x1 rates */
INLINE void set_curr_tx_ant_set(UWORD8 ant_set)
{
	UWORD32 ant_vector = get_curr_tx_ant_set_prot();

	if(ant_set)
	{
		/* Clear the value corresponding to 1x1 rates */
		ant_vector &= ~(0xF);

		/* Set the 1x1 value */
		ant_vector |= ant_set;

		/* Set the antenna vector to PHY */
		set_curr_tx_ant_set_prot(ant_vector);

		/* Set the antenna vector to MAC HW */
		set_machw_ant_set(ant_vector);
	}
}
#endif

#ifdef ENABLE_OVERWRITE_SUBTYPE
INLINE UWORD8 get_overwrite_frame_type(void)
{
    return g_overwrite_frame_type;
}

INLINE void set_overwrite_frame_type(UWORD8 val)
{
    /* A value of 0 indicates that the Frame Type/Sub-type field in MAC      */
    /* header should not be overwritten. Non-zero value shall trigger the    */
    /* over-writing. Details of this setting are as follows:                 */
    /* Bit 0 - 1 : Set to 1: Indicates that it should enable overwriting the */
    /*             Frame Type/Sub-type field in MAC header for data frames   */
    /*             transmitted.                                              */
    /* Bit 7 - 2 : Frame Type/Sub-type value to be used for overwriting      */
    g_overwrite_frame_type = val;
}
#endif /* ENABLE_OVERWRITE_SUBTYPE */


//chenq add for iw config
UWORD8 * config_if_for_iw(mac_struct_t *mac,UWORD8 * host_req,
					                UWORD16 host_req_len, char msgtype,
					                UWORD16 * trout_rsp_len);

#ifdef IBSS_BSS_STATION_MODE
UWORD8 * get_scan_ap_list(UWORD16 * trout_rsp_len);
#endif

UWORD8 send_disconnect_flg(UWORD8 inp);
#endif /* CONFIG_H */
