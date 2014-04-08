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
/*  File Name         : management.h                                         */
/*                                                                           */
/*  Description       : This file contains the MAC management related        */
/*                      definitions.                                         */
/*                                                                           */
/*  List of Functions : start_mgmt_timeout_timer                             */
/*                      cancel_mgmt_timeout_timer                            */
/*                                                                           */
/*  Issues            : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifndef MANAGEMENT_H
#define MANAGEMENT_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "cglobals.h"
#include "common.h"
#include "csl_if.h"
#include "frame.h"
#include "itypes.h"
#include "maccontroller.h"

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MANAGEMENT_FRAME_LEN 292

/* Continents for scanning channels */
#define NORTH_AMERICA      0
#define EUROPE             1
#define ASIA               2

/* Maximum number of elements in the (extended) supported rates field */
#define MAX_SUPRATES       8
#define MAX_EXTSUPRATES    256
#define MAX_OPRATESET_LEN  10
#define EDCA_PARAMS_LEN    18
#define QBSS_LOAD_LEN      5

/* Maximum number of sites to be scanned */
//chenq add for 20 ap info 2013-02-05

#ifdef IBSS_BSS_STATION_MODE //chenq add a ifdef for sta mode 20 ap info 2013-02-05

#ifdef CONFIG_NUM_AP_SCAN
    #define MAX_SITES_FOR_SCAN   CONFIG_NUM_AP_SCAN
#else /* CONFIG_NUM_AP_SCAN */
    #define MAX_SITES_FOR_SCAN   8
#endif /* CONFIG_NUM_AP_SCAN */

#else/*BSS_ACCESS_POINT_MODE*/

#ifdef CONFIG_NUM_STA
    #define MAX_SITES_FOR_SCAN   CONFIG_NUM_STA
#else /* CONFIG_NUM_STA */
    #define MAX_SITES_FOR_SCAN   8
#endif /* CONFIG_NUM_STA */

#endif

#ifdef COMBO_SCAN
//chenq add for combo scan 2013-03-12
#define MAX_AP_COMBO_SCAN 9
#define MAX_AP_COMBO_SCAN_LIST 12
//Yiming.li, reduce scan times
#define SCAN_IN_ONE_CHANNEL_CNT 2
#define COEX_SCAN_IN_ONE_CHANNEL_CNT 3

#endif

/* Maximum number of stations supported in AP/IBSS Modes*/
#define MAX_STA_SUPPORTED NUM_STA_SUPPORTED

/*  Scan filter parameter mask bit   */
#define SCAN_FILTER_BIT 0x03
#define SCAN_PRI_BIT    0x0C
#define SCAN_CH_BIT     0x10

/* Join or Start Timeout */
#define JOIN_START_TIMEOUT 10000

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* Priority values for a/b/g transmissions */
typedef enum{NORMAL_PRIORITY  = 0,
             CF_PRIORITY      = 1,
             MAX_PRIORITY     = 2
} ABG_PRIORITY_T;

/* BSS type */
typedef enum{INFRASTRUCTURE  = 1,
             INDEPENDENT     = 2,
             ANY_BSS         = 3
} BSSTYPE_T;

/* Cipher Type */
typedef enum{WEP40      = 0,
             NO_ENCRYP  = 3,
             WEP104     = 4
} CIPHER_T;

/* Authentication type */
typedef enum{OPEN_SYSTEM     = 0,
             SHARED_KEY      = 1,
             ANY             = 2
} AUTHTYPE_T;

/* Status code for MLME operation confirm */
typedef enum{SUCCESS_MLMESTATUS  = 0,
             INVALID_MLMESTATUS  = 1,
             TIMEOUT             = 2,
             REFUSED             = 3,
             TOMANY_REQ          = 4,
             ALREADY_BSS         = 5
} MLMESTATUS_T;

/* Scan type parameter for scan request */
typedef enum{PASSIVE_SCAN = 0,
             ACTIVE_SCAN  = 1,
             NUM_SCANTYPE
} SCANTYPE_T;


typedef enum {FILTER_NO       = 0x00,
              FILTER_AP_ONLY  = 0x01,
              FILTER_STA_ONLY = 0x02
} SCAN_CLASS_FITLER_T;

typedef enum{PRI_HIGH_RSSI    = 0x00,
             PRI_LOW_RSSI     = 0x04,
             PRI_DETECT       = 0x08
} SCAN_PRI_T;

typedef enum{CH_FILTER_OFF    = 0x00,
             CH_FILTER_ON     = 0x10
} CH_FILTER_T;

/* Capability Information field bit assignments  */
typedef enum{ESS            = 0x01,   /* ESS capability               */
             IBSS           = 0x02,   /* IBSS mode                    */
             POLLABLE       = 0x04,   /* CF Pollable                  */
             POLLREQ        = 0x08,   /* Request to be polled         */
             PRIVACY        = 0x10,   /* WEP encryption supported     */
             SHORTPREAMBLE  = 0x20,   /* Short Preamble is supported  */
             SHORTSLOT      = 0x400,  /* Short Slot is supported      */
             PBCC           = 0x40,   /* PBCC                         */
             CHANNELAGILITY = 0x80,   /* Channel Agility              */
             SPECTRUM_MGMT  = 0x100,  /* Spectrum Management          */
             DSSS_OFDM      = 0x2000  /* DSSS-OFDM                    */
} CAPABILITY_T;

/* Reason Codes for Deauthentication and Disassociation Frames */
typedef enum {UNSPEC_REASON    = 1,
              AUTH_NOT_VALID   = 2,
              DEAUTH_LV_SS     = 3,
              INACTIVITY       = 4,
              AP_OVERLOAD      = 5,
              CLASS2_ERR       = 6,
              CLASS3_ERR       = 7,
              DISAS_LV_SS      = 8,
              ASOC_NOT_AUTH    = 9,
              IEEE_802_1X_AUTH_FAIL = 23,
} REASON_CODE_T;

/* Status Codes for Authentication and Association Frames */
typedef enum {SUCCESSFUL_STATUSCODE  = 0,
              UNSPEC_FAIL            = 1,
              UNSUP_CAP              = 10,
              REASOC_NO_ASOC         = 11,
              FAIL_OTHER             = 12,
              UNSUPT_ALG             = 13,
              AUTH_SEQ_FAIL          = 14,
              CHLNG_FAIL             = 15,
              AUTH_TIMEOUT           = 16,
              AP_FULL                = 17,
              UNSUP_RATE             = 18,
              SHORT_PREAMBLE_UNSUP   = 19,
              PBCC_UNSUP             = 20,
              CHANNEL_AGIL_UNSUP     = 21,
              MISMATCH_SPEC_MGMT     = 22,
              MISMATCH_POW_CAP       = 23,
              MISMATCH_SUPP_CHNL     = 24,
              SHORT_SLOT_UNSUP       = 25,
              OFDM_DSSS_UNSUP        = 26,
              LARGE_LISTEN_INT       = 51,

} STATUS_CODE_T;

/* Queue number values for a/b/g transmit queues */
typedef enum {HIGH_PRI_Q   = 0,  /* High Priority queue   */
              NORMAL_PRI_Q = 1,  /* Normal Priority queue */
              CF_PRI_Q     = 2   /* Contention Free Priority queue */
} ABG_Q_NUM_T;

typedef enum {NOT_CONFIGURED = 0,
              ACCESS_POINT   = 1,
              BSS_STA        = 2,
              IBSS_STA       = 3,
              P2P_GO         = 4,
              P2P_DEVICE     = 5,
              P2P_CLIENT     = 6,
} DEVICE_MODE_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8 type; /* MLME Response Type    */
    UWORD8 *msg; /* MLME Response Message */
} mlme_rsp_t;

/* MLME Start Request structure */
typedef struct
{
    WORD8     ssid[MAX_SSID_LEN];
    BSSTYPE_T bss_type;
    UWORD16   beacon_period;
    UWORD32   time_stamp_msb;
    UWORD32   time_stamp_lsb;
    UWORD32   local_time_msb;
    UWORD32   local_time_lsb;
    WORD8     phy_param[IDSPARMS_LEN+IE_HDR_LEN];
    WORD8     cf_param[ICFPARMS_LEN+IE_HDR_LEN];
    WORD8     ibss_param[IIBPARMS_LEN+IE_HDR_LEN];
    UWORD8    dtim_period;
    UWORD16   cap_info;
    UWORD16   probe_delay;
    UWORD8    op_rate[MAX_OPRATESET_LEN];
    UWORD16   start_timeout;
} start_req_t;

/* MLME Start response structure */
typedef struct
{
    UWORD8 result_code;
} start_rsp_t;

typedef struct
{
    UWORD8   q_num;
    UWORD8   *mac_hdr;
    UWORD8   *buffer_addr;
    UWORD16  frame_len;
    UWORD8   service_class;
    CIPHER_T ct;
    UWORD8   key_type;
    UWORD8   sta_index;
    UWORD8   tx_rate;
    UWORD8   priority;
} tx_buff_elem_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD16 g_join_start_timeout;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern BSSTYPE_T get_bss_type(UWORD16 cap_info);
extern void      send_probe_rsp(UWORD8* msa, BOOL_T is_p2p);
extern BOOL_T    probe_req_ssid_cmp(UWORD8* probe_req, WORD8* ssid);

#ifndef OS_LINUX_CSL_TYPE
extern void      mgmt_timeout_alarm_fn(HANDLE_T* alarm, ADDRWORD_T data);
#else /* OS_LINUX_CSL_TYPE */
extern void      mgmt_timeout_alarm_fn(ADDRWORD_T data);
#endif /* OS_LINUX_CSL_TYPE */

//chenq add for combo scan 2013-03-13
#ifdef IBSS_BSS_STATION_MODE 
#ifdef COMBO_SCAN
extern enum hrtimer_restart mgmt_timeout_alarm_fn_combo_scan(struct hrtimer *timer);
#endif
#endif

extern BOOL_T    check_bss_capability_info(UWORD16 cap_info);
extern BOOL_T    buffer_tx_packet(UWORD8 *entry, UWORD8 *da, UWORD8 priority,
                                  UWORD8 q_num, UWORD8 *tx_dscr);
extern BOOL_T    send_deauth_frame(UWORD8 *da, UWORD16 error_code);
extern void      update_beacon_erp_info(void);
extern UWORD8    get_serv_class(void *entry, UWORD8 *da, UWORD8 tid);

#ifdef AUTORATE_FEATURE
extern UWORD8 is_rate_supp(UWORD8 rate, void *entry);
extern UWORD8 is_rate_allowed(UWORD8 rate ,void *entry);
#endif /* AUTORATE_FEATURE */

extern UWORD8 *get_mac_addr_from_sta_id(UWORD8 sta_id);

/*****************************************************************************/
/* Inline functions                                                          */
/*****************************************************************************/

/* This function starts the management timeout timer with the given time. */
INLINE void start_mgmt_timeout_timer(UWORD32 time)
{
    /* Stop the timer if it is already running */
    stop_alarm(g_mgmt_alarm_handle);
    
    /* Restart the timer */
#ifdef DV_SIM
    start_alarm(g_mgmt_alarm_handle, time*20);
#else
    start_alarm(g_mgmt_alarm_handle, time);
#endif
}

/* This function cancels the management timeout timer. */
INLINE void cancel_mgmt_timeout_timer(void)
{
    stop_alarm(g_mgmt_alarm_handle);
}

/* This function checks whether Short Preamble is supported */
INLINE BOOL_T get_bss_short_preamble_info(UWORD16 cap_info)
{
    return ((cap_info & SHORTPREAMBLE)?(BTRUE):(BFALSE));
}

/* This function frees all the beacon buffers */
INLINE void free_beacon_buffers(void)
{
    if(g_beacon_frame[0] != NULL)
    {
        /* Free the beacon buffers for beacons */
        pkt_mem_free(g_beacon_frame[0]);
        g_beacon_frame[0] = NULL;
    }

    if(g_beacon_frame[1] != NULL)
    {
        /* Free the beacon buffers for beacons */
        pkt_mem_free(g_beacon_frame[1]);
        g_beacon_frame[1] = NULL;
    }

    g_beacon_index = 0;
}
#endif /* MANAGEMENT_H */
