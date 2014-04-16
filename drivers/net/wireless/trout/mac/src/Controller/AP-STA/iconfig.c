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
/*  File Name         : iconfig.c                                            */
/*                                                                           */
/*  Description       : This file contains the configuration message parsing */
/*                      functions based on the SME mode in use.              */
/*                                                                           */
/*  List of Functions : parse_config_message                                 */
/*                      process_query                                        */
/*                      process_write                                        */
/*                      set_wid                                              */
/*                      get_wid                                              */
/*                      send_host_rsp                                        */
/*                      save_wids                                            */
/*                      restore_wids                                         */
/*                      send_mac_status                                      */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "controller_mode_if.h"
#include "mac_init.h"
#include "fsm.h"
#include "rf_if.h"
#include "buff_desc.h"
#include "iconfig.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "test_config.h"
#ifdef MAC_HW_UNIT_TEST_MODE
#include "mh_test_config.h"
#include "mh_test.h"
#endif /* MAC_HW_UNIT_TEST_MODE */

//chenq add
#include <linux/wireless.h>
#include <net/iw_handler.h>

/*leon liu added for CFG80211*/
#ifdef CONFIG_CFG80211
#include "trout_cfg80211.h"
#endif
#include "itm_wifi_iw.h"

#include <linux/time.h>
#include "qmu_tx.h"

extern struct net_device *g_mac_dev;
int scan_cmp_flag = 0;

//chenq add a ui scan cmd flag 2012-10-15
int is_scanlist_report2ui = 0;

/*xuan yang, 2013.6.3 get and set resume assoc flg*/
extern UWORD8 get_resume_assoc_flg(void);
extern UWORD8 set_resume_assoc_flg(UWORD8);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
extern void sleep_disconnected(void);
#endif
//chenq add do ap list merge logic 2013-08-28
#ifdef IBSS_BSS_STATION_MODE
extern UWORD8 *get_scan_ap_info(bss_dscr_t * bss_dscr,UWORD8 * srcbuf);
#endif

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
//int itm_mac_close_flag = 1;

/*****************************************************************************/
/* WID get/set tables                                                        */
/*****************************************************************************/

/* Host WLAN Identifier Structure                                            */
/* ID, Type, Response, Reset, Get Function Pointer,  Set Function Pointer    */
#ifndef MAC_HW_UNIT_TEST_MODE
const host_wid_struct_t g_char_wid_struct[] =
#else /* MAC_HW_UNIT_TEST_MODE */
host_wid_struct_t g_char_wid_struct[] =
#endif /* MAC_HW_UNIT_TEST_MODE */
{
    {WID_BSS_TYPE,                            /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_DesiredBSSType,               (void *)set_DesiredBSSType},

    {WID_CURRENT_TX_RATE,                     /* WID_CHAR, */
    BTRUE,                                    BFALSE,
	//chenq mod for cur tx rate 2012-10-17
	//(void *)get_tx_rate,                      (void *)set_tx_rate},
    (void *)get_cur_tx_rate,                  (void *)set_tx_rate},

    {WID_PRIMARY_CHANNEL,                     /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_prim_chnl_num,                (void *)set_prim_chnl_num},

    {WID_PREAMBLE,                            /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_preamble_type,                (void *)set_preamble_type},

    {WID_11G_OPERATING_MODE,                  /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_11g_op_mode,                 (void *)set_11g_op_mode},

    {WID_STATUS,                              /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    0,                                        0},

    {WID_SCAN_TYPE,                           /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)mget_scan_type,                   (void *)mset_scan_type},

    {WID_PRIVACY_INVOKED,                     /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)mget_PrivacyInvoked,              (void *)0},

    {WID_KEY_ID,                              /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)mget_WEPDefaultKeyID,             (void *)mset_WEPDefaultKeyID},

    {WID_QOS_ENABLE,                          /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_qos_enable,                   (void *)set_qos_enable},

    {WID_POWER_MANAGEMENT,                    /* WID_CHAR, */
    BTRUE,                                    BFALSE, // 0213 caisf mod for not restart mac
    (void *)get_PowerManagementMode,          (void *)set_PowerManagementMode},

    {WID_11I_MODE,                            /* WID_CHAR, */
    BTRUE,                                    BTRUE,
#ifndef MAC_HW_UNIT_TEST_MODE
    (void *)get_802_11I_mode,                 (void *)set_802_11I_mode},
#else /* MAC_HW_UNIT_TEST_MODE */
    (void *)get_hut_cip_type,                 (void *)set_hut_cip_type},
#endif /* MAC_HW_UNIT_TEST_MODE */

    {WID_AUTH_TYPE,                           /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_auth_type,                    (void *)set_auth_type},

    {WID_SITE_SURVEY,                         /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_site_survey_status,           (void *)set_site_survey},

    {WID_LISTEN_INTERVAL,                     /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)mget_listen_interval,             (void *)mset_listen_interval},

    {WID_DTIM_PERIOD,                         /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)mget_DTIMPeriod,                  (void *)set_dtim_period},

    {WID_ACK_POLICY,                          /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_ack_type,                     (void *)set_ack_type},

    {WID_RESET,                               /* WID_CHAR,*/
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)set_reset_req},

    {WID_BCAST_SSID,                          /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)get_bcst_ssid,                    (void *)set_bcst_ssid},

    {WID_DISCONNECT,                          /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)disconnect_station},

    {WID_READ_ADDR_SDRAM ,                    /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)get_read_addr_sdram,              (void *)set_read_addr_sdram},

    {WID_TX_POWER_LEVEL_11A,                  /* WID_CHAR, */
    BTRUE,                                   BFALSE,
    (void *)get_tx_power_level_11a,           (void *)set_tx_power_level_11a},

    {WID_REKEY_POLICY,                        /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_RSNAConfigGroupRekeyMethod,   (void *)set_RSNAConfigGroupRekeyMethod},

    {WID_SHORT_SLOT_ALLOWED,                  /* WID_CHAR, */
    BTRUE,                                    BTRUE,
#ifdef MAC_HW_UNIT_TEST_MODE
    (void *)get_hut_short_slot,               (void *)set_hut_short_slot},
#else  /* MAC_HW_UNIT_TEST_MODE */
    (void *)get_short_slot_allowed,           (void *)set_short_slot_allowed},
#endif /* MAC_HW_UNIT_TEST_MODE */

    {WID_PHY_ACTIVE_REG,                      /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_phy_active_reg,               (void *)set_phy_active_reg},

    {WID_TX_POWER_LEVEL_11B,                  /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_tx_power_level_11b,           (void *)set_tx_power_level_11b},

    {WID_START_SCAN_REQ,                      /* WID_CHAR, */
    BTRUE,                                   BFALSE,
    (void *)get_start_scan_req,               (void *)set_start_scan_req},

    {WID_RSSI,                                /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_rssi,                         (void *)0},

	//chenq add snr
	{WID_SNR,                                /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_snr,                         (void *)0},

    {WID_JOIN_REQ,                            /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)0,                                (void *)set_join_req},


    {WID_USER_CONTROL_ON_TX_POWER,            /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_user_control_enabled,         (void *)set_user_control_enabled},

#if 0 //wxb modify
    {WID_MEMORY_ACCESS_8BIT,                  /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_memory_access_8bit,           (void *)set_memory_access_8bit},
#else
    {WID_MEMORY_ACCESS_8BIT,                  /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)0,           (void *)0},
#endif

    {WID_UAPSD_SUPPORT_AP,                    /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_uapsd_support_ap,             (void *)set_uapsd_support_ap},

    {WID_CURRENT_MAC_STATUS,                  /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_current_mac_status,           (void *)0},

    {WID_AUTO_RX_SENSITIVITY,                 /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_auto_rx_sensitivity,           (void *)set_auto_rx_sensitivity},

    {WID_DATAFLOW_CONTROL,                    /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_dataflow_control,             (void *)set_dataflow_control},

    {WID_SCAN_FILTER,                         /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_scan_filter,                  (void *)set_scan_filter},

    {WID_LINK_LOSS_THRESHOLD,                 /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_link_loss_threshold,          (void *)set_link_loss_threshold},

    {WID_AUTORATE_TYPE,                       /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)get_autorate_type,                (void *)set_autorate_type},

    {WID_802_11H_DFS_MODE,                    /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_802_11H_DFS_mode,             (void *)set_802_11H_DFS_mode},

    {WID_802_11H_TPC_MODE,                    /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_802_11H_TPC_mode,             (void *)set_802_11H_TPC_mode},

    {WID_11N_PROT_MECH,                       /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_protection_mode,              (void *)set_protection_mode},

    {WID_11N_ERP_PROT_TYPE,                   /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_erp_prot_type,                (void *)set_erp_prot_type},

    {WID_11N_ENABLE,                          /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_ht_mib_enable,                (void *)set_ht_mib_enable},

    {WID_11N_OPERATING_TYPE,                  /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_11n_op_type,                  (void *)set_11n_op_type},

    {WID_11N_OBSS_NONHT_DETECTION,            /* WID_CHAR, */
    BFALSE,                                    BFALSE,
    (void *)get_11n_obss_detection,           (void *)set_11n_obss_detection},

    {WID_11N_HT_PROT_TYPE,                    /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_11n_ht_prot_type,             (void *)set_11n_ht_prot_type},

    {WID_11N_RIFS_PROT_ENABLE,                /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_11n_rifs_prot_enable,         (void *)set_11n_rifs_prot_enable},

    {WID_11N_SMPS_MODE,                       /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_11n_smps_mode,                (void *)set_11n_smps_mode},

    {WID_11N_CURRENT_TX_MCS,                  /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_curr_tx_mcs,                  (void *)set_tx_mcs},

    {WID_11N_CURRENT_TX_BW,                  /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_curr_tx_bw,                  (void *)set_curr_tx_bw},

    {WID_11N_PRINT_STATS,                     /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_11n_print_stats,              (void *)print_test_stats},

    {WID_11N_SHORT_GI_ENABLE,                 /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_short_gi_enable,              (void *)set_short_gi_enable},

    {WID_TX_ABORT_CONFIG,                     /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)set_tx_abort_config},

    {WID_HOST_DATA_IF_TYPE,                   /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_host_data_if_type,            (void *)set_host_data_if_type},

    {WID_HOST_CONFIG_IF_TYPE,                 /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_host_config_if_type,          (void *)set_host_config_if_type},

    {WID_REG_TSSI_11B_VALUE,                  /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_phy_tssi_11b,                 (void *)0},

    {WID_REG_TSSI_11G_VALUE,                  /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_phy_tssi_11g,                 (void *)0},

    {WID_REG_TSSI_11N_VALUE,                  /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_phy_tssi_11n,                 (void *)0},

    {WID_TX_CALIBRATION,                      /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_tx_cal,                       (void *)set_tx_cal},

    {WID_DSCR_TSSI_11B_VALUE,                 /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_dscr_tssi_11b,                (void *)0},

    {WID_DSCR_TSSI_11G_VALUE,                 /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_dscr_tssi_11g,                (void *)0},

    {WID_DSCR_TSSI_11N_VALUE,                 /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_dscr_tssi_11n,                (void *)0},

    {WID_11N_IMMEDIATE_BA_ENABLED,            /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_immediate_block_ack_enable,   (void *)set_immediate_block_ack_enable},

    {WID_11N_TXOP_PROT_DISABLE,               /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_disable_txop_protection,      (void *)set_disable_txop_protection},

    {WID_TX_POWER_LEVEL_11N,                  /* WID_CHAR, */
    BTRUE,                                   BFALSE,
    (void *)get_tx_power_level_11n,           (void *)set_tx_power_level_11n},

    {WID_WPS_ENABLE,                           /*WID_CHAR,*/
    BFALSE,                                   BTRUE,
    (void *)get_wps_enable,                   (void *)set_wps_enable},

    {WID_WPS_START,                           /*WID_CHAR,*/
    BFALSE,                                   BFALSE,
    (void *)get_wps_prot,                     (void *)set_wps_prot},

    {WID_WPS_DEV_MODE,                        /*WID_CHAR,*/
    BFALSE,                                   BFALSE,
    (void *)get_wps_dev_mode,                 (void *)set_wps_dev_mode},

    {WID_DEVICE_MODE,                         /*WID_CHAR,*/
    BFALSE,                                   BFALSE,
    (void *)get_device_mode,                  (void *)set_device_mode},

    {WID_OOB_RESET_REQ,                       /*WID_CHAR,*/
    BFALSE,                                   BTRUE,
    (void *)0,                               (void *)set_oob_reset_req},

    {WID_ENABLE_INT_SUPP,                     /*WID_CHAR,*/
    BFALSE,                                   BTRUE,
    (void *)get_int_supp_mode,                (void *)set_int_supp_mode},

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
    {WID_ENABLE_MULTI_DOMAIN,                 /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)get_multi_domain_supp,           (void *)set_multi_domain_supp},

    {WID_CURRENT_REG_DOMAIN,                  /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)get_current_reg_domain,           (void *)set_current_reg_domain},

    {WID_CURRENT_REG_CLASS,                   /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_current_reg_class,           (void *)0},
#endif

     //caisf mod 20130503
    {WID_SCAN_REQ_CHANNEL_NO,                 /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)get_scan_req_channel_no,           (void *)set_scan_req_channel_no},

    {WID_TX_POWER_LEVEL_11N_40MHZ,            /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_tx_power_level_11n40,         (void *)set_tx_power_level_11n40},

#ifdef MAC_HW_UNIT_TEST_MODE
    {WID_HUT_FCS_CORRUPT_MODE,                /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)set_fcs_corrupt_mode},

    {WID_HUT_RESTART,                         /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)0,                                (void *)set_restart_hut_test},

    {WID_HUT_TX_FORMAT,                       /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_tx_format,                (void *)set_hut_tx_format},

    {WID_HUT_BANDWIDTH,                       /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_bandwidth,                (void *)set_hut_bandwidth},

    {WID_HUT_OP_BAND,                         /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_op_band,                  (void *)set_hut_op_band},

    {WID_HUT_STBC,                            /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_stbc,                     (void *)set_hut_stbc},

    {WID_HUT_ESS,                             /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_ess,                      (void *)set_hut_ess},

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
    {WID_HUT_ANTSET,                          /* WID_CHAR, */
#else
    {WID_CUR_TX_ANTSET,                      /* WID_CHAR, */
#endif
    BFALSE,                                   BFALSE,
    (void *)get_hut_antset,                   (void *)set_hut_antset},

    {WID_HUT_HT_OP_MODE,                      /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_op_mode,                  (void *)set_hut_op_mode},

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
#else /* MAC_HW_UNIT_TEST_MODE */

    {WID_CUR_TX_ANTSET,                       /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_curr_tx_ant_set,              (void *)set_curr_tx_ant_set},
#endif

#endif /* MAC_HW_UNIT_TEST_MODE */

    {WID_RIFS_MODE,                           /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_rifs_mode,                    (void *)set_rifs_mode},

    {WID_2040_ENABLE,                         /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_2040_enable,                  (void *)set_2040_enable},

    {WID_2040_COEXISTENCE,                    /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)get_2040_coex_support,            (void *)set_2040_coex_support},

    {WID_2040_40MHZ_INTOLERANT,               /* WID_CHAR, */
    BFALSE,                                   BTRUE,
    (void *)get_40mhz_intolerant,             (void *)set_40mhz_intolerant},

#ifdef MAC_P2P
    {WID_P2P_ENABLE,                          /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)mget_p2p_enable,                  (void *)set_p2p_enable_mode},

    {WID_P2P_DISCOVERABLE,                    /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)mget_p2p_discoverable,            (void *)set_p2p_discoverable_mode},

    {WID_P2P_LISTEN_CHAN,                     /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_p2p_listen_chan,             (void *)set_p2p_listen_chan},

    {WID_P2P_FIND_TO,                         /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)get_p2p_find_to,                  (void *)set_p2p_find_to},

    {WID_P2P_GO_INT_VAL,                      /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)mget_p2p_GO_intent_val,           (void *)mset_p2p_GO_intent_val},

    {WID_P2P_PERSIST_GRP,                     /* WID_CHAR, */
    BTRUE,                                    BFALSE,
    (void *)mget_p2p_persist_grp,             (void *)set_p2p_persist_grp_mode},

    {WID_P2P_AUTO_GO,                         /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)mget_p2p_auto_go,                 (void *)mset_p2p_auto_go},

    {WID_P2P_INTRA_BSS,                       /* WID_CHAR, */
    BTRUE,                                    BTRUE,
    (void *)mget_p2p_intra_bss,               (void *)set_p2p_intra_bss_mode},

    {WID_P2P_CT_WINDOW,                       /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)mget_p2p_CTW,                     (void *)set_p2p_ct_window},
	
	// 20120709 caisf add, merged ittiam mac v1.2 code
    {WID_P2P_LISTEN_MODE,                     /* WID_CHAR, */
	BFALSE,                                   BFALSE,
    (void *)mget_p2p_listen_mode,             (void *)mset_p2p_listen_mode},
#endif /* MAC_P2P */


#ifdef MAC_HW_UNIT_TEST_MODE
    {WID_HUT_SMOOTHING_REC,                   /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_smoothing_rec,            (void *)set_hut_smoothing_rec},

    {WID_HUT_SOUNDING_PKT,                    /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_sounding_pkt,             (void *)set_hut_sounding_pkt},

    {WID_HUT_HT_CODING,                       /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_ht_coding,                (void *)set_hut_ht_coding},

    {WID_HUT_TEST_DIR,                        /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_test_dir,                 (void *)set_hut_test_dir},

    {WID_HUT_PHY_TEST_MODE,                   /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_phy_test_mode,            (void *)set_hut_phy_test_mode},

    {WID_HUT_PHY_TEST_RATE_HI,                 /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_phy_test_rate_hi,         (void *)set_hut_phy_test_rate_hi},

    {WID_HUT_PHY_TEST_RATE_LO,                /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_phy_test_rate_lo,         (void *)set_hut_phy_test_rate_lo},

    {WID_HUT_DISABLE_RXQ_REPLENISH,           /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_disable_rxq_repl,         (void *)set_hut_disable_rxq_repl},

    {WID_HUT_KEY_ORIGIN,                      /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_key_origin,               (void *)set_hut_key_origin},

    {WID_HUT_BCST_PERCENT,                      /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_bcst_percent,             (void *)set_hut_bcst_percent},

    {WID_HUT_GROUP_CIPHER_TYPE,               /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_bcst_ct,                 (void *)set_hut_bcst_ct},

    {WID_HUT_RSSI_EX,                         /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_rssi_ex,                      (void *)0},

    {WID_HUT_ADJ_RSSI_EX,                     /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_adj_rssi_ex,                  (void *)0},

    {WID_HUT_TSF_TEST_MODE,                   /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_tsf_test_mode,            (void *)set_hut_tsf_test_mode},

    {WID_HUT_PKT_TSSI_VALUE,                   /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_last_pkt_tssi_value,          (void *)0},

    {WID_HUT_FC_TXOP_MOD,                     /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_fc_txop_mod,              (void *)set_hut_fc_txop_mod},

    {WID_HUT_FC_PROT_TYPE,                    /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_fc_prot_type,             (void *)set_hut_fc_prot_type},

    {WID_HUT_SEC_CCA_ASSERT,                   /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_sec_cca_assert,           (void *)set_hut_sec_cca_assert},
#endif /* MAC_HW_UNIT_TEST_MODE */

//chenq add for wapi 2012-09-19
#ifdef MAC_WAPI_SUPP
	{WID_WAPI_MODE,                            /* WID_CHAR, */
    BFALSE,                                   BFALSE,
    (void *)get_wapi_mode,                    (void *)set_wapi_mode},
#endif	

    {WID_MAX,                                 /*WID_CHAR,*/
	BFALSE,                                   BFALSE,
    (void *)0,                                (void *)0},
};

const host_wid_struct_t g_short_wid_struct[] =
{
    {WID_USER_PREF_CHANNEL,                  /* WID_SHORT, */
    BTRUE,                                    BTRUE,
    (void *)get_user_pref_channel,           (void *)set_user_pref_channel},

    {WID_CURR_OPER_CHANNEL,                   /* WID_SHORT, */
    BTRUE,                                    BTRUE,
    (void *)get_curr_op_channel,             (void *)0},

    {WID_RTS_THRESHOLD,                       /* WID_SHORT, */
    BTRUE,                                    BFALSE,
    (void *)mget_RTSThreshold,                (void *)set_RTSThreshold},

    {WID_FRAG_THRESHOLD,                      /* WID_SHORT, */
    BTRUE,                                    BFALSE,
    (void *)mget_FragmentationThreshold,      (void *)set_FragmentationThreshold},

    {WID_SHORT_RETRY_LIMIT,                   /* WID_SHORT, */
    BTRUE,                                    BFALSE,
    (void *)mget_ShortRetryLimit,             (void *)set_ShortRetryLimit},

    {WID_LONG_RETRY_LIMIT,                    /* WID_SHORT, */
    BTRUE,                                    BFALSE,
    (void *)mget_LongRetryLimit,              (void *)set_LongRetryLimit},

    {WID_BEACON_INTERVAL,                     /* WID_SHORT, */
    BTRUE,                                    BTRUE,
    (void *)mget_BeaconPeriod,                (void *)mset_BeaconPeriod},

#if 0 //wxb modify
    {WID_MEMORY_ACCESS_16BIT,                 /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_memory_access_16bit,          (void *)set_memory_access_16bit},
#else
    {WID_MEMORY_ACCESS_16BIT,                 /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)0,          (void *)0},
#endif

    {WID_RX_SENSE,                            /* WID_SHORT, */
    BFALSE,                                   BTRUE,
    (void *)get_rx_sense,                     (void *)set_rx_sense},

    {WID_CCA_THRESHOLD,                       /* WID_SHORT, */
    BFALSE,                                   BTRUE,
    (void *)get_cca_threshold,                (void *)set_cca_threshold},

    {WID_ACTIVE_SCAN_TIME,                    /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_active_scan_time,             (void *)set_active_scan_time},

    {WID_PASSIVE_SCAN_TIME,                   /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_passive_scan_time,            (void *)set_passive_scan_time},

    {WID_SITE_SURVEY_SCAN_TIME,               /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_site_survey_scan_time,        (void *)set_site_survey_scan_time},

    {WID_JOIN_START_TIMEOUT,                  /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_join_start_timeout,           (void *)set_join_start_timeout},

    {WID_AUTH_TIMEOUT,                        /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_auth_timeout,                 (void *)set_auth_timeout},

    {WID_ASOC_TIMEOUT,                        /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_asoc_timeout,                 (void *)set_asoc_timeout},

    {WID_11I_PROTOCOL_TIMEOUT,                /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_11i_protocol_timeout,         (void *)set_11i_protocol_timeout},

    {WID_EAPOL_RESPONSE_TIMEOUT,              /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_eapol_response_timeout,       (void *)set_eapol_response_timeout},

    {WID_11N_SIG_QUAL_VAL,                    /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_signal_quality,               (void *)0},

    {WID_WPS_PASS_ID,                         /*WID_SHORT,*/
    BFALSE,                                   BFALSE,
    (void *)get_wps_pass_id,                  (void *)set_wps_pass_id},

    {WID_WPS_CONFIG_METHOD,                   /*WID_SHORT,*/
    BFALSE,                                   BFALSE,
    (void *)get_wps_config_method,            (void *)set_wps_config_method},

#ifdef MAC_HW_UNIT_TEST_MODE
    {WID_HUT_FRAME_LEN,                       /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_frame_len,                (void *)set_hut_frame_len},

    {WID_HUT_TXOP_LIMIT,                      /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_txop_limit,               (void *)set_hut_txop_limit},

    {WID_HUT_SIG_QUAL_AVG,                    /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_sig_qual_avg,             (void *)0},

    {WID_HUT_SIG_QUAL_AVG_CNT,                /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_sig_qual_avg_cnt,         (void *)set_hut_sig_qual_avg_cnt},

    {WID_HUT_RSSI_EX_COUNT,                   /* WID_SHORT, */
    BFALSE,                                   BFALSE,
    (void *)get_rssi_ex_count,                (void *)set_rssi_ex_count},
#endif /* MAC_HW_UNIT_TEST_MODE */

    {WID_MAX,                                 /*WID_CHAR,*/
	BFALSE,                                   BFALSE,
    (void *)0,                                (void *)0},

};

const host_wid_struct_t g_int_wid_struct[] =
{
    {WID_FAILED_COUNT,                        /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_FailedCount,                 (void *)set_FailedCount_to_zero},

    {WID_RETRY_COUNT,                         /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_RetryCount,                  (void *)set_RetryCount_to_zero},

    {WID_MULTIPLE_RETRY_COUNT,                /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_MultipleRetryCount,          (void *)set_MultipleRetryCount_to_zero},

    {WID_FRAME_DUPLICATE_COUNT,               /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_FrameDuplicateCount,         (void *)set_FrameDuplicateCount_to_zero},

    {WID_ACK_FAILURE_COUNT,                   /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_ACKFailureCount,             (void *)set_ACKFailureCount_to_zero},

    {WID_RECEIVED_FRAGMENT_COUNT,             /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_ReceivedFragmentCount,       (void *)set_ReceivedFragmentCount_to_zero},

    {WID_MCAST_RECEIVED_FRAME_COUNT,          /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_MulticastReceivedFrameCount, (void *)set_MulticastReceivedFrameCount_to_zero},

    {WID_FCS_ERROR_COUNT,                     /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_FCSErrorCount,               (void *)set_FCSErrorCount_to_zero},

    {WID_SUCCESS_FRAME_COUNT,                 /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_TransmittedFrameCount,       (void *)set_TransmittedFrameCount_to_zero},

    {WID_HUT_TX_COUNT,                        /* WID_INT, */
    BFALSE,                                   BTRUE,
    (void *)get_phy_test_txd_pkt_cnt,         (void *)set_phy_test_txd_pkt_cnt},

    {WID_TX_FRAGMENT_COUNT,                   /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_TransmittedFragmentCount,    (void *)0},

    {WID_TX_MULTICAST_FRAME_COUNT,               /* WID_INT, */
    BFALSE,                                      BFALSE,
    (void *)mget_MulticastTransmittedFrameCount, (void *)0},

    {WID_RTS_SUCCESS_COUNT,                   /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_RTSSuccessCount,             (void *)0},

    {WID_RTS_FAILURE_COUNT,                   /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_RTSFailureCount,             (void *)0},

    {WID_WEP_UNDECRYPTABLE_COUNT,             /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)mget_WEPICVErrorCount,            (void *)0},

    {WID_REKEY_PERIOD,                        /* WID_INT, */
    BTRUE,                                    BTRUE,
    (void *)get_RSNAConfigGroupRekeyTime ,    (void *)set_RSNAConfigGroupRekeyTime},

    {WID_REKEY_PACKET_COUNT,                  /* WID_INT, */
    BTRUE,                                    BTRUE,
    (void *)get_RSNAConfigGroupRekeyPackets,  (void *)set_RSNAConfigGroupRekeyPackets},

    {WID_1X_SERV_ADDR,                        /* WID_INT, */
    BTRUE,                                    BTRUE,
    (void *)get_1x_serv_addr,                 (void *)set_1x_serv_addr},

    {WID_STACK_IP_ADDR,                       /* WID_INT, */
    BTRUE,                                    BFALSE,
    (void *)get_stack_ip_addr,                (void *)set_stack_ip_addr},

    {WID_STACK_NETMASK_ADDR,                  /* WID_INT, */
    BTRUE,                                    BFALSE,
    (void *)get_stack_netmask_addr,           (void *)set_stack_netmask_addr},

    {WID_HW_RX_COUNT,                         /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)get_machw_rx_end_count,           (void *)0},

    {WID_RF_REG_VAL,                          /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)set_rf_reg_val},

#if 0 //wxb modify
    {WID_MEMORY_ADDRESS,                      /* WID_INT, */
    BFALSE,                                   BFALSE,

    {WID_MEMORY_ACCESS_32BIT,                 /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)get_memory_access_32bit,          (void *)set_memory_access_32bit},
#else
    {WID_MEMORY_ADDRESS,                      /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)0,               (void *)0},

    {WID_MEMORY_ACCESS_32BIT,                 /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)0,          (void *)0},
#endif

    {WID_11N_PHY_ACTIVE_REG_VAL,              /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)get_phy_active_reg_val,           (void *)set_phy_active_reg_val},

    {WID_DEV_OS_VERSION,                      /*WID_INT,*/
    BFALSE,                                   BFALSE,
    (void *)get_dev_os_version,               (void*)set_dev_os_version},

#ifdef MAC_HW_UNIT_TEST_MODE
    {WID_HUT_NUM_TX_PKTS,                     /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_num_tx_pkts,              (void *)set_hut_num_tx_pkts},

    {WID_HUT_TX_TIME_TAKEN,                   /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_tx_time,                  (void *)0},

    {WID_HUT_TX_TEST_TIME,                    /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_tx_test_time,             (void *)set_hut_tx_test_time},
#endif /* MAC_HW_UNIT_TEST_MODE */

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
    {WID_TX_POWER_LEVELS,                     /* WID_INT, */
    BFALSE,                                   BFALSE,
    (void *)get_tx_power_levels_dbm,          (void *)0},
#endif

    {WID_MAX,                                 /*WID_CHAR,*/
	BFALSE,                                   BFALSE,
    (void *)0,                                (void *)0},

};

const host_wid_struct_t g_str_wid_struct[] =
{
    {WID_SSID,                                /* WID_STR, */
    BTRUE,                                    BTRUE,
    (void *)get_DesiredSSID,                  (void *)set_DesiredSSID},

    {WID_FIRMWARE_VERSION,                    /* WID_STR, */
    BTRUE,                                    BFALSE,
#ifdef DEBUG_MODE
    (void *)get_manufacturerProductVersion,   (void *)set_test_case_name},
#else /* DEBUG_MODE */
    (void *)get_manufacturerProductVersion,   0},
#endif /* DEBUG_MODE */

    {WID_OPERATIONAL_RATE_SET,                /* WID_STR, */
    BTRUE,                                    BFALSE,
    (void *)get_supp_rates,                   0},

    {WID_BSSID,                               /* WID_STR, */
    BTRUE,                                    BFALSE,
    (void *)get_bssid,                        (void *)set_bssid},

    {WID_WEP_KEY_VALUE,                       /* WID_STR, */
    BTRUE,                                    BTRUE,
    (void *)get_WEP_key,                      (void *)set_WEP_key},

    {WID_11I_PSK,                             /* WID_STR, */
    BTRUE,                                    BTRUE,
    (void *)get_RSNAConfigPSKPassPhrase,      (void *)set_RSNAConfigPSKPassPhrase},
	
	// 20120709 caisf add, merged ittiam mac v1.2 code
    {WID_11I_PSK_VALUE,                       /* WID_STR, */
	BTRUE,                                    BTRUE,
    (void *)get_RSNAConfigPSKValue,           (void *)set_RSNAConfigPSKValue},

    {WID_11E_P_ACTION_REQ,                    /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_action_request,               (void *)set_action_request},

    {WID_1X_KEY,                              /* WID_STR, */
    BTRUE,                                    BTRUE,
    (void *)get_1x_key,                       (void *)set_1x_key},

    {WID_HARDWARE_VERSION,                    /* WID_STR, */
    BTRUE,                                    BFALSE,
    (void *)get_hardwareProductVersion,       0},

    {WID_MAC_ADDR,                            /* WID_STR, */
    BTRUE,                                    BTRUE,
    (void *)get_mac_addr,                     (void *)set_mac_addr},

    {WID_HUT_DEST_ADDR,                       /* WID_STR, */
    BFALSE,                                   BTRUE,
    (void *)get_phy_test_dst_addr,            (void *)set_phy_test_dst_addr},

    {WID_MISC_TEST_MODES,                     /*WID_STR,*/
    BFALSE,                                    BFALSE,
    (void *)get_misc_test_modes,               (void *)set_misc_test_modes},

    {WID_PHY_VERSION,                         /* WID_STR, */
    BTRUE,                                    BFALSE,
    (void *)get_phyProductVersion,            0},

    {WID_SUPP_USERNAME,                       /* WID_STR, */
    BTRUE,                                    BTRUE,
    (void *)get_supp_username,                (void *)set_supp_username},

    {WID_SUPP_PASSWORD,                       /* WID_STR, */
    BTRUE,                                    BTRUE,
    (void *)get_supp_password,                (void *)set_supp_password},

    {WID_SITE_SURVEY_RESULTS,                 /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_site_survey_results,          (void *)0},

    {WID_RX_POWER_LEVEL,                      /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_rx_power_level,               (void *)0},

    {WID_ADD_WEP_KEY,                         /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)add_wep_key_bss_sta},

    {WID_REMOVE_WEP_KEY,                      /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)remove_wep_key},

    {WID_ADD_PTK,                             /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)add_ptk},

    {WID_ADD_RX_GTK,                          /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)add_rx_gtk},

    {WID_ADD_TX_GTK,                          /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)add_tx_gtk},

    {WID_REMOVE_KEY,                          /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)remove_key},

    {WID_ASSOC_REQ_INFO,                      /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_assoc_req_info,               (void *)0},

    {WID_ASSOC_RES_INFO,                      /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_assoc_res_info,               (void *)0},

    {WID_11N_P_ACTION_REQ,                    /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_action_request,               (void *)set_action_request},

    {WID_PMKID_INFO,                          /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_pmkid_info,                   (void *)set_pmkid_info},

    {WID_FIRMWARE_INFO,                      /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_firmware_info,               (void *)0},

    {WID_SERIAL_NUMBER,                       /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_serial_number,                0},

#ifdef MAC_P2P
    {WID_P2P_TARGET_DEV_ID,                   /* WID_STR */
    BFALSE,                                   BTRUE,
    (void *)get_p2p_trgt_dev_id,              (void *)mset_p2p_trgt_dev_id},

    {WID_P2P_INVIT_DEV_ID,                    /* WID_STR */
    BFALSE,                                   BFALSE,
    (void *)get_p2p_invit_dev_id,             (void *)set_p2p_invit_dev_id},

    {WID_P2P_PERSIST_CRED,                    /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_p2p_persist_cred,             (void *)set_p2p_persist_cred},

    {WID_P2P_NOA_SCHEDULE,                    /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_p2p_noa_sched,                (void *)set_p2p_noa_sched},
#endif /* MAC_P2P */

    {WID_WPS_STATUS,                          /*WID_STR,*/
    BFALSE,                                   BFALSE,
    0,                                        0},

    {WID_WPS_PIN,                             /*WID_STR,*/
    BFALSE,                                   BFALSE,
    (void *)get_wps_pin,                      (void*)set_wps_pin},

    {WID_MANUFACTURER,                        /*WID_STR,*/
    BFALSE,                                   BFALSE,
    (void *)get_manufacturer,                 (void*)set_manufacturer},

    {WID_MODEL_NAME,                          /*WID_STR,*/
    BFALSE,                                   BFALSE,
    (void *)get_model_name,                   (void*)set_model_name},

    {WID_MODEL_NUM,                           /*WID_STR,*/
    BFALSE,                                   BFALSE,
    (void *)get_model_num,                    (void*)set_model_num},

    {WID_DEVICE_NAME,                         /*WID_STR,*/
    BFALSE,                                   BFALSE,
    (void *)get_dev_name,                     (void*)set_dev_name},

#ifdef MAC_HW_UNIT_TEST_MODE
    {WID_HUT_TEST_ID,                        /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)set_hut_test_id},
#endif /* MAC_HW_UNIT_TEST_MODE */

// 20120830 caisf add, merged ittiam mac v1.3 code
#if 1
    {WID_SUPP_REG_DOMAIN_INFO,                /* WID_STR, */
    BFALSE,                                   BFALSE,
    (void *)get_sup_reg_dom_info,             (void *)0},
#endif

	//chenq add for wapi 2012-09-24
#ifdef MAC_WAPI_SUPP
	{WID_WAPI_KEY,                            /*WID_BIN_DATA,*/
    BFALSE,                                   BFALSE,
    (void *)NULL,                             (void *)set_wapi_key},
#endif

    {WID_MAX,                                 /*WID_CHAR,*/
	BFALSE,                                   BFALSE,
    (void *)0,                                (void *)0},

};

const host_wid_struct_t g_binary_data_wid_struct[] =
{
    {WID_UAPSD_CONFIG,                        /* WID_BIN_DATA, */
    BFALSE,                                   BTRUE,
    (void *)get_uapsd_config,                 (void *)set_uapsd_config},

    {WID_UAPSD_STATUS,                        /* WID_BIN_DATA, */
    BFALSE,                                   BFALSE,
    (void *)get_uapsd_status,                 (void *)0},

    {WID_WMM_AP_AC_PARAMS,                    /* WID_BIN_DATA, */
    BFALSE,                                   BFALSE,
    (void *)get_wmm_ap_ac_params,             (void *)set_wmm_ap_ac_params},

    {WID_WMM_STA_AC_PARAMS,                   /* WID_BIN_DATA, */
    BFALSE,                                   BFALSE,
    (void *)get_wmm_sta_ac_params,            (void *)set_wmm_sta_ac_params},

    {WID_CONNECTED_STA_LIST,                   /*WID_BIN_DATA,*/
    BFALSE,                                   BFALSE,
    (void *)get_connected_sta_list,           (void *)0},

    {WID_11N_AUTORATE_TABLE,                  /* WID_BIN_DATA, */
    BFALSE,                                   BFALSE,
#ifdef MAC_HW_UNIT_TEST_MODE
    (void *)0,                                (void *)set_hut_ar_table},
#else  /* MAC_HW_UNIT_TEST_MODE */
    (void *)0,                                (void *)set_auto_rate_table},
#endif /* MAC_HW_UNIT_TEST_MODE */

    {WID_WPS_CRED_LIST,                       /*WID_BIN_DATA,*/
    BFALSE,                                   BFALSE,
    (void *)get_wps_cred_list,                (void *)set_wps_cred_list},

    {WID_PRIM_DEV_TYPE,                       /*WID_BIN_DATA,*/
    BFALSE,                                   BFALSE,
    (void *)get_prim_dev_type,                (void *)set_prim_dev_type},

#ifdef MAC_P2P
    {WID_P2P_REQ_DEV_TYPE,                    /*WID_BIN_DATA,*/
    BFALSE,                                   BFALSE,
    (void *)get_req_dev_type,                 (void *)set_req_dev_type},
#endif /* MAC_P2P */

#ifdef MAC_HW_UNIT_TEST_MODE
    {WID_HUT_TX_PATTERN,                      /* WID_BIN_DATA, */
    BFALSE,                                   BFALSE,
    (void *)0,                                (void *)set_hut_tx_pattern},

    {WID_HUT_STATS,                           /* WID_BIN_DATA, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_stats,                   (void *)set_hut_stats},

    {WID_HUT_LOG_STATS,                       /* WID_BIN_DATA, */
    BFALSE,                                   BFALSE,
    (void *)get_hut_log_stats,                (void *)set_hut_log_stats},
#endif /* MAC_HW_UNIT_TEST_MODE */

    {WID_MAX,                                 /*WID_CHAR,*/
	BFALSE,                                   BFALSE,
    (void *)0,                                (void *)0},

};

/*****************************************************************************/
/* Configuration parameters                                                  */
/*****************************************************************************/

UWORD8      g_info_id                        = 0;
UWORD16     g_current_len                    = 0;
BOOL_T      g_reset_mac                      = BFALSE;
RESET_REQ_T g_reset_req_from_user            = NO_REQUEST;
UWORD8      g_current_settings[MAX_QRSP_LEN] = {0};
UWORD8      g_cfg_val[MAX_CFG_LEN]           = {0};
UWORD8      g_phy_ver[MAX_PROD_VER_LEN + 1]  = {0};
UWORD8      g_network_info_id                = 0;

//chenq add for gscan
UWORD8      g_cfg_val_for_gscan[512]         = {0};

//chenq add for wpa/rsn ie buf
UWORD8 g_wpa_rsn_ie_buf_2[MAX_SITES_FOR_SCAN][512] = {{0},};
extern UWORD8 g_wpa_rsn_ie_buf_3[MAX_SITES_FOR_SCAN][512];

//chenq add 2012-10-29
extern struct task_struct * itm_scan_task;

/*****************************************************************************/
/* Device specific configuration parameters                                  */
/*****************************************************************************/

UWORD8 g_serial_num[MAX_SERIAL_NUM_LEN + 1] = {0};

UWORD32 g_dev_os_version                            = 0;
UWORD8  g_manufacturer[MAX_MANUFACTURER_ID_LEN + 1] = {0};
UWORD8  g_model_name[MAX_MODEL_NAME_LEN + 1]        = {0};
UWORD8  g_model_num[MAX_MODEL_NUM_LEN + 1]          = {0};
UWORD8  g_device_name[MAX_DEVICE_NAME_LEN + 1]      = {0};
UWORD8  g_prim_dev_type[PRIM_DEV_TYPE_LEN+2]        =
    {
        (PRIM_DEV_TYPE_LEN & 0xFF), ((PRIM_DEV_TYPE_LEN >> 8) & 0xFF),
        /*0x00, 0x01, 0x00, 0x50, 0xF2, 0x04, 0x00, 0x01*/ //wxb modify
        0x00, 0x0A, 0x00, 0x50, 0xF2, 0x04, 0x00, 0x05
    };
UWORD8 g_req_dev_type[REQ_DEV_TYPE_LEN + 2] = {0};

const UWORD8 g_default_manufacturer[] = "Ittiam Systems";
const UWORD8 g_default_model_name[]   = "WPS Wireless Device";
const UWORD8 g_default_model_num[]    = "1234";
const UWORD8 g_default_device_name[]  = "Ittiam MWLAN";

#ifdef IBSS_BSS_STATION_MODE
//chenq add 2 buf for save mac scan ap list 0727
UWORD8   g_ap_list_copy1[512] = {0};
UWORD8   g_ap_list_copy2[512] = {0};
UWORD16  g_ap_list_copy_len1 = 0;
UWORD16  g_ap_list_copy_len2 = 0;

UWORD8   g_ap_list_copy3[512] = {0};
UWORD8   g_ap_list_copy4[512] = {0};
UWORD16  g_ap_list_copy_len3 = 0;
UWORD16  g_ap_list_copy_len4 = 0;

atomic_t ap_list_copy_buf_lock = ATOMIC_INIT(0);

//chenq add do ap list merge logic 2013-08-28
UWORD16       timeoutValue= MAX_SCAN_DEL_TIME;

#endif

/*****************************************************************************/
/* Custom configuration parameters                                           */
/*****************************************************************************/


/*****************************************************************************/
/*                                                                           */
/*  Function Name : parse_config_message                                     */
/*                                                                           */
/*  Description   : This function parses the host configuration messages.    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC structure                          */
/*                  2) Pointer to configuration message                      */
/*                  3) Pointer to Buffer address                             */
/*                                                                           */
/*  Globals       : g_reset_mac                                              */
/*                  g_current_settings                                       */
/*                  g_current_len                                            */
/*                                                                           */
/*  Processing    : This function processes all the configuration messages.  */
/*                  from the host based on the SME type. Currently only      */
/*                  SME types 'Test' and 'Auto' are supported. In case of a  */
/*                  Query request the query response is prepared using the   */
/*                  values of the queried parameters. No MAC state change    */
/*                  occurs in this case. In case of a Write request the      */
/*                  parameters are set and the Write response prepared. If   */
/*                  any parameter writing requires the MAC to be reset, the  */
/*                  appropriate function is called to reset MAC. The         */
/*                  paramters are saved in a global list. After MAC is reset */
/*                  these parameters are set once again. Thereafter, a Scan  */
/*                  Request is prepared and sent to the MAC. Response to the */
/*                  host is sent in the required format based on the SME     */
/*                  mode.                                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void parse_config_message(mac_struct_t *mac, UWORD8* host_req,
                          UWORD8 *buffer_addr, UWORD8 host_if_type)
{
    UWORD8  msg_type  = 0;
    UWORD8  msg_id    = 0;
    UWORD16 msg_len   = 0;
    BOOL_T  free_flag = BTRUE;
    UWORD8  offset    = get_config_pkt_hdr_len(host_if_type);

	TROUT_FUNC_ENTER;
    /* Extract the Type, Length and ID of the incoming host message. The     */
    /* format of the message is:                                             */
    /* +-------------------------------------------------------------------+ */
    /* | Message Type | Message ID |  Message Length |Message body         | */
    /* +-------------------------------------------------------------------+ */
    /* |     1 Byte   |   1 Byte   |     2 Bytes     | Message Length      | */
    /* +-------------------------------------------------------------------+ */

    msg_type = host_req[0];
    msg_id   = host_req[1];
    msg_len  = (host_req[3] << 8) | host_req[2];
    msg_len -= MSG_HDR_LEN;

    /* The  valid types of incoming messages are 'Q' (Query) and 'W' (Write) */
    switch(msg_type)
    {
    case 'Q':
    {
        UWORD16 rsp_len    = 0;
        UWORD8  *query_rsp = 0;
        UWORD8  *rsp_buff  = 0;

        /* To a Query message a Response message needs to be sent. This      */
        /* message has a type 'R' and should have an ID equal to the         */
        /* incoming Query message.                                           */

        rsp_buff = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);

        if(rsp_buff == NULL)
        {
            pkt_mem_free(buffer_addr);
            TROUT_FUNC_EXIT;
            return;
        }

        query_rsp = rsp_buff + offset;

        /* Set the Message Type and ID. The Message Length is set after the  */
        /* message contents are written. The length is known only after that.*/
        query_rsp[0] = 'R';
        query_rsp[1] = msg_id;

        /* Process the Query message and set the contents as required. */
        rsp_len = process_query(&host_req[MSG_DATA_OFFSET],
                                &query_rsp[MSG_DATA_OFFSET], msg_len);

        /* The length of the message returned is set in the Message length   */
        /* field.                                                            */
        query_rsp[2] = (rsp_len + MSG_HDR_LEN) & 0xFF;
        query_rsp[3] = ((rsp_len + MSG_HDR_LEN) & 0xFF00) >> 8;

        /* The response is sent back to the host. No MAC state changes occur */
        /* on reception of a Query message.                                  */
        send_host_rsp(rsp_buff, (UWORD16)(rsp_len + MSG_HDR_LEN), host_if_type);
    }
    break;

    case 'W':
    {
        UWORD8  *write_rsp = 0;
        UWORD8  *rsp_buff  = 0;
		struct trout_private *tp;

        /* Check if any protocol disallows processing of this write request   */
        if(BFALSE == process_wid_write_prot((host_req + MSG_DATA_OFFSET),
                                            msg_len))
        {
            /* If the buffer is not freed yet, free it */
            if(free_flag == BTRUE)
                pkt_mem_free(buffer_addr);
                
			TROUT_FUNC_EXIT;
            return;
        }

#ifdef BURST_TX_MODE
        if(get_DesiredBSSType() != 4)
        {
            g_burst_tx_mode_enabled_earlier = BFALSE;
        }
        else
        {
            g_burst_tx_mode_enabled_earlier = BTRUE;
        }
#endif /* BURST_TX_MODE */

        /* This is done since some WPS specific WIDs can do an internal MAC  */
        /* reset.                                                            */
        lock_the_buffer(buffer_addr);

        /* Indicate that WID Config Write is in progress */
        indicate_config_write_prot(BTRUE);

        /* Process the Write message and set the parameters with the given   */
        /* values.                                                           */
        process_write(host_req + MSG_DATA_OFFSET, msg_len);

        /* Indicate that WID Config Write is in completed */
        indicate_config_write_prot(BFALSE);

        /* Handle WPS related configuration update */
        wps_handle_config_update();

        unlock_the_buffer(buffer_addr);

        /* Change MAC states as required. If it is already in Enabled state  */
        /* reset MAC and start again. The previous configuration values are  */
        /* retained.                                                         */
        if(g_reset_mac == BTRUE)
#ifndef MAC_HW_UNIT_TEST_MODE
        if((get_mac_state() != DISABLED) && (allow_mac_reset() == BTRUE))
#endif /* MAC_HW_UNIT_TEST_MODE */
        {
#ifdef IBSS_BSS_STATION_MODE
#ifdef INT_WPS_SUPP
            /* If there is reset because of config packet received from host */
            /* then disable the WPS protocol. Please note WPS specific WIDs  */
            /* must not require a reset                                      */
            disable_wps_mode();
#endif /* INT_WPS_SUPP */
#endif /* IBSS_BSS_STATION_MODE */
			//xuan yang, 2013-8-23, add wid mutex
			//tp = netdev_priv(g_mac_dev);
			//mutex_lock(&tp->rst_wid_mutex);

            g_reset_mac_in_progress    = BTRUE;

            /* Free the buffer */
            pkt_mem_free(buffer_addr);

            free_flag = BFALSE;

            /* Save the current configuration before going for Reset         */
            save_wids();

            /* Reset MAC - Bring down PHY and MAC H/W, disable MAC           */
            /* interrupts and release all OS structures. Further, this       */
            /* function restarts the MAC again from start.                   */
            reset_mac(mac, BTRUE);

            /* Restore the saved configuration before resetting              */
            restore_wids();

            g_reset_mac_in_progress    = BFALSE;

			//mutex_unlock(&tp->rst_wid_mutex);

#ifdef BURST_TX_MODE
            /* Start scaning only when burst tx mode is disabled */
            if(g_burst_tx_mode_enabled == BTRUE)
            {
               init_mac_fsm_bt(mac);
            }
#endif /* BURST_TX_MODE */
        }

        /* This function resolves all configuration related conflicts that */
        /* might arise between various WIDs.                               */
        resolve_wid_conflicts(mac);

#ifndef MAC_HW_UNIT_TEST_MODE
        if((get_mac_state() == DISABLED) && (allow_mac_reset() == BTRUE))
#else /* MAC_HW_UNIT_TEST_MODE */
        if(g_test_start == BTRUE)
#endif /* MAC_HW_UNIT_TEST_MODE */
        {
            start_mac_and_phy(mac);
        }

        /* To a Write message a Response message needs to be sent. This      */
        /* message has a type 'R' and should have an ID equal to the         */
        /* incoming Write message. The Message contents contain 'Y' to       */
        /* indicate Success.                                                 */
        if(free_flag == BFALSE)
            rsp_buff = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
        else
            rsp_buff = buffer_addr;

        if(rsp_buff == NULL)
        {
        	TROUT_FUNC_EXIT;
            return;
        }

        free_flag = BFALSE;
        write_rsp = rsp_buff + offset;

        write_rsp[0] = 'R';
        write_rsp[1] = msg_id;
        write_rsp[2] = (WRITE_RSP_LEN + MSG_HDR_LEN) & 0xFF;
        write_rsp[3] = ((WRITE_RSP_LEN + MSG_HDR_LEN) & 0xFF00) >> 8;

        /* Set the WID_STATUS, Length to 1, Value to SUCCESS */
        write_rsp[4] = WID_STATUS & 0xFF;
        write_rsp[5] = (WID_STATUS & 0xFF00) >> 8;
        write_rsp[6] = 1;
        write_rsp[7] = WRSP_SUCCESS;

        /* The response is sent back to the host                            */
        send_host_rsp(rsp_buff, WRITE_RSP_LEN + MSG_HDR_LEN,host_if_type);
    }
    break;

    default:
    {
        /* Unknown message type. Do nothing. */
    }
    } /* end of switch(msg_type) */

    /* If the buffer is not freed yet, free it */
    if(free_flag == BTRUE)
        pkt_mem_free(buffer_addr);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_wid_index                                            */
/*                                                                           */
/*  Description   : This function gets the WID index from WID ID & Type      */
/*                                                                           */
/*  Inputs        : 1) WID Identifier                                        */
/*                  2) WID Type                                              */
/*                                                                           */
/*  Globals       : 1) g_char_wid_struct                                     */
/*                  2) g_short_wid_struct                                    */
/*                  2) g_int_wid_struct                                      */
/*                  2) g_str_wid_struct                                      */
/*                  2) g_binary_data_wid_struct                              */
/*                                                                           */
/*  Processing    : This function searches for the WID ID in the relevent    */
/*                  global structure to find the index of the perticular WID */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Length of the query response                             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_wid_index(UWORD16 wid_id,UWORD8 wid_type)
{
    UWORD8 count;

    switch(wid_type)
    {
        case WID_CHAR:
            for(count = 0;
                g_char_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_char_wid_struct[count].id == wid_id)
                    return count;
            }
            break;
        case WID_SHORT:
            for(count = 0;
                g_short_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_short_wid_struct[count].id == wid_id)
                    return count;
            }
            break;
        case WID_INT:
            for(count = 0;
                g_int_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_int_wid_struct[count].id == wid_id)
                    return count;
             }
            break;
        case WID_STR:
            for(count = 0;
                g_str_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_str_wid_struct[count].id == wid_id)
                    return count;
            }
            break;
        case WID_BIN_DATA:
            for(count = 0;
                g_binary_data_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_binary_data_wid_struct[count].id == wid_id)
                    return count;
            }
            break;
    }

    return 0xFF;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_query                                            */
/*                                                                           */
/*  Description   : This function processes the host query message.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the query request message                  */
/*                  2) Pointer to the query response message                 */
/*                  3) Length of the query request                           */
/*                                                                           */
/*  Globals       : g_wid_struct                                             */
/*                                                                           */
/*  Processing    : This function reads the WID value in the Query message   */
/*                  and gets the corresponding WID length and value. It sets */
/*                  the WID, WID Length and value in the response message.   */
/*                  The process is repeated for all the WID types requested  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Length of the query response                             */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 process_query(UWORD8* wid_req, UWORD8* wid_rsp, UWORD16 req_len)
{
    UWORD8  count   = 0;
    UWORD16 wid_id  = 0;
    UWORD16 req_idx = 0;
    UWORD16 rsp_idx = 0;

    /* The format of a message body of a message type 'Q' is:                */
    /* +-------------------------------------------------------------------+ */
    /* | WID0          | WID1         | WID2         | ................... | */
    /* +-------------------------------------------------------------------+ */
    /* |     2 Bytes   |    2 Bytes   |    2 Bytes   | ................... | */
    /* +-------------------------------------------------------------------+ */

    /* The format of a message body of a message type 'R' is:                */
    /* +-------------------------------------------------------------------+ */
    /* | WID0      | WID0 Length | WID0 Value  | ......................... | */
    /* +-------------------------------------------------------------------+ */
    /* | 2 Bytes   | 1 Byte      | WID Length  | ......................... | */
    /* +-------------------------------------------------------------------+ */

    /* The processing of a Query message consists of the following steps:    */
    /* 1) Read the WID value in the Query message                            */
    /* 2) Get the corresponding WID length and value                         */
    /* 3) Set the WID, WID Length and value in the response message          */
    /* 4) Repeat for all the WID types requested by the host                 */
    /* 5) Return the length of the response message                          */

    while(req_idx < req_len)
    {
        /* Read the WID type (2 Bytes) from the Query message and increment  */
        /* the Request Index by 2 to point to the next WID.                  */
        wid_id = wid_req[req_idx + 1];
        wid_id = (wid_id << 8) | wid_req[req_idx];
        req_idx += 2;

        if(wid_id == WID_ALL)
        {
            /* If the WID type is WID_ALL all WID values need to be returned */
            /* to the host. Thus all the array elements of the global WID    */
            /* array are accessed one by one.                                */
            for(count = 0;
                g_char_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_char_wid_struct[count].rsp == BTRUE)
                    rsp_idx += get_wid(&wid_rsp[rsp_idx], count, WID_CHAR);
            }

            for(count = 0;
                g_short_wid_struct[count].id != WID_MAX;
                count++)
            {

                if(g_short_wid_struct[count].rsp == BTRUE)
                    rsp_idx += get_wid(&wid_rsp[rsp_idx], count, WID_SHORT);
            }

            for(count = 0;
                g_int_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_int_wid_struct[count].rsp == BTRUE)
                    rsp_idx += get_wid(&wid_rsp[rsp_idx], count, WID_INT);
            }

            for(count = 0;
                g_str_wid_struct[count].id != WID_MAX;
                count++)
            {
                if(g_str_wid_struct[count].rsp == BTRUE)
                    rsp_idx += get_wid(&wid_rsp[rsp_idx], count, WID_STR);
            }

            for(count = 0;
               g_binary_data_wid_struct[count].id != WID_MAX;
               count++)
            {
                if(g_binary_data_wid_struct[count].rsp == BTRUE)
                    rsp_idx += get_wid(&wid_rsp[rsp_idx], count, WID_BIN_DATA);
            }

#ifdef DEBUG_MODE
            if(rsp_idx >= 1472)
            {
                PRINTD("SwEr : WID-ALL Response Frame Limit Exceeded [%d > 1472]\n\r",rsp_idx);
            }
#endif /* DEBUG_MODE */

        }
        else
        {
            /* In case of any other WID, the static global WID array is      */
            /* searched to find a matching WID type for its corresponding    */
            /* length and value.                                             */
            UWORD8 wid_type = (wid_id & 0xF000) >> 12;
            count           = get_wid_index(wid_id,wid_type);

            if(count == 0xFF)
            {
                /* If the Queried WID type is not found it is an exception.  */
                /* Skip this and continue.                                   */
                continue;
            }
            else
            {
                /* In case of a valid WID, fill up the WID type, WID length  */
                /* and WID values in the Response message.                   */
                rsp_idx += get_wid(&wid_rsp[rsp_idx], count, wid_type);
            }
        }
    } /* end of while(index < req_len) */

    /* Return Response length (given by the updated response index value)    */
    return rsp_idx;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : process_write                                            */
/*                                                                           */
/*  Description   : This function processes the host write message.          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the write request message                  */
/*                  2) Length of the write request                           */
/*                                                                           */
/*  Globals       : g_wid_struct                                             */
/*                  g_reset_mac                                              */
/*                                                                           */
/*  Processing    : This function reads the WID value in the Write message   */
/*                  and sets the corresponding WID length and value. The     */
/*                  process is repeated for all the WID types requested by   */
/*                  the host. In case the setting of any WID requires the    */
/*                  reset of MAC, the global g_reset_mac is updated.         */
/*                                                                           */
/*  Outputs       : g_reset_mac is updated to indicate if a reset is needed  */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void process_write(UWORD8* wid_req, UWORD16 req_len)
{
    UWORD8  count    = 0;
    UWORD16 req_idx  = 0;
    UWORD16 wid_len  = 0;
    UWORD16 wid_id   = 0;
    UWORD8  *wid_val = 0;
    UWORD8  wid_type = 0;

	TROUT_FUNC_ENTER;
    /* Set the global flag for reset MAC to BFALSE. This will be updated to  */
    /* BTRUE if any parameter setting calls for a reset.                     */
    g_reset_mac = BFALSE;

    /* The format of a message body of a message type 'W' is:                */
    /* +-------------------------------------------------------------------+ */
    /* | WID0      | WID0 Length | WID0 Value  | ......................... | */
    /* +-------------------------------------------------------------------+ */
    /* | 2 Bytes   | 1 Byte      | WID Length  | ......................... | */
    /* +-------------------------------------------------------------------+ */

    /* The processing of a Write message consists of the following steps:    */
    /* 1) Read the WID value in the Write message                            */
    /* 2) Set the corresponding WID length and value                         */
    /* 3) Repeat for all the WID types requested by the host                 */

    while(req_idx + 3 < req_len)
    {
        /* Read the WID ID (2 Bytes) and the WID length from the Write       */
        /* message.                                                          */
        wid_id  = wid_req[req_idx + 1];
        wid_id  = (wid_id << 8) | wid_req[req_idx];
        wid_type = (wid_id & 0xF000) >> 12;
        count    = get_wid_index(wid_id,wid_type);

        if(wid_type != WID_BIN_DATA)
        {
            wid_len = wid_req[req_idx + 2];

            /* Set the 'wid' pointer to point to the WID value (skipping */
            /* the ID and Length fields). This is used in the set_wid    */
            /* function. It is assumed that all 'set' functions use the  */
            /* value directly as input.                                  */
            wid_val = &wid_req[req_idx + WID_VALUE_OFFSET];

            /* The Request Index is incremented by (WID Length + 3) to   */
            /* point to the next WID.                                    */
            req_idx += (wid_len + WID_VALUE_OFFSET);
        }
        else
        {
            UWORD16 i        = 0;
            UWORD8  checksum = 0;

            /* WID_BIN_DATA format */
            /* +---------------------------------------------------+ */
            /* | WID    | WID Length | WID Value        | Checksum | */
            /* +---------------------------------------------------+ */
            /* | 2Bytes | 2Bytes     | WID Length Bytes | 1Byte    | */
            /* +---------------------------------------------------+ */

            wid_len  = wid_req[req_idx + 2];
            wid_len |= ((UWORD16)wid_req[req_idx + 3] << 8) & 0xFF00 ;
            wid_len &= WID_BIN_DATA_LEN_MASK;

            /* Set the 'wid' pointer to point to the WID length (skipping */
            /* the ID field). This is used in the set_wid function. It    */
            /* is assumed that 'set' function for Binary data uses the    */
            /* length field for getting the WID Value                     */
            wid_val  = &wid_req[req_idx + WID_LENGTH_OFFSET];

            /* The Request Index is incremented by (WID Length + 5) to   */
            /* point to the next WID.                                    */
            req_idx += (wid_len + WID_LENGTH_OFFSET + 2 + 1);

            /* Compute checksum on the Data field */
            for(i = 0;i < wid_len;i++)
            {
                checksum += wid_val[i + 2];
            }
            /* Drop the packet, if checksum failed */
            if(checksum != wid_val[wid_len + 2])
            {
                continue;
            }
        }

        if(count == 0xFF)
        {
            /* If the Queried WID type is not found it is an exception.  */
            /* Skip this and continue.                                   */
            continue;
        }
        else
        {
            /* In case of a valid WID, set the corresponding parameter with  */
            /* the incoming value.                                           */
            set_wid(wid_val, wid_len, count, wid_type);

            /* Check if writing this WID requires resetting of MAC. The      */
            /* global g_reset_mac is 'OR'ed with the reset value obtained    */
            /* for this WID.                                                 */
#ifdef MAC_HW_UNIT_TEST_MODE
            if((wid_type == WID_CHAR) && (g_char_wid_struct[count].id == WID_HUT_RESTART))
#endif /* MAC_HW_UNIT_TEST_MODE */
            switch(wid_type)
            {
                case WID_CHAR:
                    g_reset_mac |= g_char_wid_struct[count].reset;
                    break;
                case WID_SHORT:
                    g_reset_mac |= g_short_wid_struct[count].reset;
                    break;
                case WID_INT:
                    g_reset_mac |= g_int_wid_struct[count].reset;
                    break;
                case WID_STR:
                    g_reset_mac |= g_str_wid_struct[count].reset;
                    break;
                case WID_BIN_DATA:
                    g_reset_mac |= g_binary_data_wid_struct[count].reset;
                    break;
            }
        }
    } /* end of while(index < req_len) */

    /* Overwrite the Reset Flag with User request  */
    if(g_reset_req_from_user == DONT_RESET)
    {
        g_reset_mac = BFALSE;
    }
    else if(g_reset_req_from_user == DO_RESET)
    {
        g_reset_mac = BTRUE;
    }

    /* Re-initialize user request variable */
    g_reset_req_from_user = NO_REQUEST;
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_wid                                                  */
/*                                                                           */
/*  Description   : This function gets the WID value from the global WID     */
/*                  array and updates the response message contents.         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WID field                              */
/*                  2) Index of the global WID array                         */
/*                  3) The WID type                                          */
/*                                                                           */
/*  Globals       : g_wid_struct                                             */
/*                                                                           */
/*  Processing    : This function sets the WID ID, Length and Value in the   */
/*                  the incoming packet.                                     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Length of WID response                                   */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD16 get_wid(UWORD8 *wid, UWORD8 count, UWORD8 wid_type)
{
    UWORD8  wid_len  = 0;
    UWORD16 wid_id   = 0;
    UWORD8  *wid_val = 0;

	//chenq add for gscan
	UWORD16 wid_len_for_gscan = 0;

    /* The WID is set in the following format:                               */
    /* +-------------------------------------------------------------------+ */
    /* | WID Type  | WID Length | WID Value                                | */
    /* +-------------------------------------------------------------------+ */
    /* | 2 Bytes   | 1 Byte     | WID length                               | */
    /* +-------------------------------------------------------------------+ */

    /* Set the WID value with the value returned by the get function         */
    /* associated with the particular WID type.                              */
    wid_val = (wid + WID_VALUE_OFFSET);

    /* If the member function is not NULL, get the value, else discard read */
    switch(wid_type)
    {
        case WID_CHAR:
        {
            wid_id   = g_char_wid_struct[count].id;
            if(g_char_wid_struct[count].get)
            {
                UWORD8 val =
                            ((UWORD8 (*)(void))g_char_wid_struct[count].get)();

                wid_len = 1;

                wid_val[0] = val;
            }
            else
            {
                wid_len = 0;
            }
        }
        break;

        case WID_SHORT:
        {
            wid_id   = g_short_wid_struct[count].id;
            if(g_short_wid_struct[count].get)
            {
                UWORD16 val =
                          ((UWORD16 (*)(void))g_short_wid_struct[count].get)();

                wid_len = 2;

                wid_val[0] = (UWORD8)(val & 0x00FF);
                wid_val[1] = (UWORD8)((val >> 8) & 0x00FF);
            }
            else
            {
                wid_len = 0;
            }
        }
        break;

        case WID_INT:
        {
            wid_id   = g_int_wid_struct[count].id;
            if(g_int_wid_struct[count].get)
            {
                UWORD32 val =
                            ((UWORD32 (*)(void))g_int_wid_struct[count].get)();

                wid_len = 4;

                wid_val[0] = (UWORD8)(val & 0x000000FF);
                wid_val[1] = (UWORD8)((val >> 8) & 0x000000FF);
                wid_val[2] = (UWORD8)((val >> 16) & 0x000000FF);
                wid_val[3] = (UWORD8)((val >> 24) & 0x000000FF);
            }
            else
            {
                wid_len = 0;
            }
        }
        break;

        case WID_STR:
        {
            wid_id   = g_str_wid_struct[count].id;
            if(g_str_wid_struct[count].get)
            {
                UWORD8 *val =
                            ((UWORD8 *(*)(void))g_str_wid_struct[count].get)();

                if(val != NULL)
                {
                	//chenq add
                	if(wid_id == 0x3012)
                	{
                		wid_val = (wid + WID_VALUE_OFFSET + 1);
						wid_len_for_gscan = val[0] | (val[1] <<8);
                		memcpy( wid_val, (val + 2), wid_len_for_gscan );
                	}
			else
			{
	                    wid_len = val[0];
	                    memcpy(wid_val, (val + 1), wid_len);
			}
                }
                else
                {
                    wid_len = 0;
                }
          }
          else
          {
                wid_len = 0;
          }
      }
      break;

        case WID_BIN_DATA:
        {
            /* WID_BIN_DATA is Set in the following format */
            /* +---------------------------------------------------+ */
            /* | WID    | WID Length | WID Value        | Checksum | */
            /* +---------------------------------------------------+ */
            /* | 2Bytes | 2Bytes     | WID Length Bytes | 1Byte    | */
            /* +---------------------------------------------------+ */

            wid_id  = g_binary_data_wid_struct[count].id;
            if(g_binary_data_wid_struct[count].get)
            {
                UWORD16 length      = 0;
                UWORD8  checksum    = 0;
                UWORD16 i           = 0;
                UWORD8  *val        =
                    ((UWORD8 *(*)(void))g_binary_data_wid_struct[count].get)();

                if(val == NULL)
                {
                    return 0;
                }

                /* First 2 Bytes are length field and rest of the bytes are */
                /* actual data                                              */

                /* Length field encoding */
                /* +----------------------------------+ */
                /* | BIT15  | BIT14  | BIT13 - BIT0   | */
                /* +----------------------------------+ */
                /* | First  | Last   | Message Length | */
                /* +----------------------------------+ */
                length  = (val[0] & 0x00FF);
                length |= ((UWORD16) val[1] << 8) & 0xFF00;
                length &= WID_BIN_DATA_LEN_MASK;

                if(length != 0 )
                {
                    /* Set the WID type. */
                    wid[0] = wid_id & 0xFF;
                    wid[1] = (wid_id & 0xFF00) >> 8;

                    /* Copy the data including the length field */
                    memcpy((wid + WID_LENGTH_OFFSET), val, (length + 2));

                    /* Compute Checksum for the data field & append it at    */
                    /* the end of the packet                                 */
                    for(i = 0;i < length;i++)
                    {
                        checksum += val[2 + i];
                    }
                    *(wid + WID_LENGTH_OFFSET + 2 + length) = checksum;

                    /* Return the WID response length */
                            /*WID_TYPE        WID_Length  WID_Value  Checksum*/
                    return (WID_LENGTH_OFFSET + 2         + length   + 1);
                }
                else
                {
                    return 0;
                }
            }
        }
        break;
    } /* end of switch(wid_len) */

	//chenq add
	if(wid_id == 0x3012)
	{
		wid[0] = wid_id & 0xFF;
        wid[1] = (wid_id & 0xFF00) >> 8;

		wid[2] = wid_len_for_gscan & 0xFF;
		wid[3] = (wid_len_for_gscan & 0xFF00) >> 8;

		return (wid_len_for_gscan+WID_VALUE_OFFSET+1);
	}
	
    if(wid_len != 0)
    {
        /* Set the WID type. The WID length field is set at the end after    */
        /* getting the value of the WID. In case of variable length WIDs the */
        /* length s known only after the value is obtained.                  */
        wid[0] = wid_id & 0xFF;
        wid[1] = (wid_id & 0xFF00) >> 8;

        /* Set the length of the WID */
        wid[2] = wid_len;

        /* Return the WID response length */
        return (wid_len + WID_VALUE_OFFSET);
    }
    else
    {
        return (0);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_wid                                                  */
/*                                                                           */
/*  Description   : This function sets the WID value with the incoming WID   */
/*                  value.                                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the WID value                              */
/*                  2) Length of the WID value                               */
/*                  3) Index of the global WID array                         */
/*                  4) The WID type                                          */
/*                                                                           */
/*  Globals       : g_wid_struct                                             */
/*                                                                           */
/*  Processing    : This function sets the WID Value with the incoming value.*/
/*                                                                           */
/*  Outputs       : The corresponding MAC parameter is set                   */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_wid(UWORD8 *wid, UWORD16 len, UWORD8 count, UWORD8 wid_type)
{
    switch(wid_type)
    {
    case WID_CHAR:
    {
        if(g_char_wid_struct[count].set)
        {
            ((void (*)(UWORD8))g_char_wid_struct[count].set)(wid[0]);
        }
        else
        {
            /* WID cannot be set. Do nothing. */
        }
    }
    break;

    case WID_SHORT:
    {
        UWORD16 val = 0;

        val = wid[1];
        val = (val << 8) | wid[0];

        if(g_short_wid_struct[count].set)
        {
            ((void (*)(UWORD16))g_short_wid_struct[count].set)(val);
        }
        else
        {
            /* WID cannot be set. Do nothing. */
        }
    }
    break;

    case WID_INT:
    {
        UWORD32 val = 0;

        val  = wid[3];
        val  = (val << 8) | wid[2];
        val  = (val << 8) | wid[1];
        val  = (val << 8) | wid[0];

        if(g_int_wid_struct[count].set)
        {
            ((void (*)(UWORD32))g_int_wid_struct[count].set)(val);
        }
        else
        {
            /* WID cannot be set. Do nothing. */
        }
    }
    break;

    case WID_STR:
    {
        UWORD8 val[MAX_STRING_LEN];

        memcpy(val, wid, len);
        val[len] = '\0';

        if(g_str_wid_struct[count].set)
        {
            ((void (*)(UWORD8*))g_str_wid_struct[count].set)(val);
        }
        else
        {
            /* WID cannot be set. Do nothing. */
        }
    }
    break;

    case WID_BIN_DATA:
    {
        if(g_binary_data_wid_struct[count].set)
        {
            ((void (*)(UWORD8*))g_binary_data_wid_struct[count].set)(wid);
        }
        else
        {
            /* WID cannot be set. Do nothing. */
        }
    }
    break;

    } /* end of switch(wid_type) */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_host_rsp                                            */
/*                                                                           */
/*  Description   : This function sends the response to the host based on    */
/*                  the SME mode and host protocol.                          */
/*                                                                           */
/*  Inputs        : 1) Pointer to the response buffer                        */
/*                  2) Length of the response message                        */
/*                  3) Config Host I/f to use                                */
/*                                                                           */
/*  Globals       : g_config_eth_addr                                        */
/*                  g_src_ip_addr                                            */
/*                  g_config_ip_addr                                         */
/*                                                                           */
/* Processing    : This function forwards the incoming response to the       */
/*                 required host interface after adding the Hif specific     */
/*                 headers. The header is added in-place. Hence it is        */
/*                 assumed that the config data in the buffer begins exactly */
/*                 after "host header length" number of bytes.               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_host_rsp(UWORD8 *host_rsp, UWORD16 host_rsp_len, UWORD8 host_if_type)
{
    /* In the auto mode the response is packed in the appropriate host       */
    /* format and sent to the host. In this case the response buffer will be */
    /* freed in the prepare_config_pkt function.                             */
    UWORD16     config_pkt_len = 0;
    msdu_desc_t frame_desc     = {0};
    UWORD8 config_pkt_hdr_len  = get_config_pkt_hdr_len(host_if_type);

	TROUT_FUNC_ENTER;
    /* Prepare config packet header */
    config_pkt_len = prepare_config_pkt_hdr(host_rsp, host_rsp_len,
                                        host_if_type);

    /* Check for invlaid host interface type */
    if(0 == config_pkt_len)
    {
        pkt_mem_free(host_rsp);
        TROUT_FUNC_EXIT;
        return;
    }

    /* Update the configuration host frame descriptor */
    frame_desc.buffer_addr     = host_rsp;
    frame_desc.host_hdr_len    = config_pkt_hdr_len;
    frame_desc.data_len        = config_pkt_len - config_pkt_hdr_len;
    frame_desc.host_hdr_offset = 0;
    frame_desc.data_offset     = config_pkt_hdr_len;

    send_config_frame_to_host_if(&frame_desc, host_if_type);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_network_info_to_host                                */
/*                                                                           */
/*  Description   : This function sends the beacon/probe response from scan  */
/*                                                                           */
/*  Inputs        : BSSID, RSSI & InfoElements                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares an Information frame having the   */
/*                  BSSID. RSSI value and Information elements and sends the */
/*                  prepared frame to host                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_network_info_to_host(UWORD8 *msa, UWORD16 rx_len, signed char rssi)
{
	//chenq add return 2012-10-30
	return;

// 20120830 caisf mod, merged ittiam mac v1.3 code
//#ifndef ETHERNET_HOST
	TROUT_FUNC_ENTER;
#ifdef IBSS_BSS_STATION_MODE
    if(g_int_supp_enable == BFALSE)
    {
    UWORD8  *info_msg      = NULL;
    UWORD16 ninfo_data_len = 0;
    UWORD8  host_type      = get_host_config_if_type();
    UWORD16 offset         = get_config_pkt_hdr_len(host_type);

    info_msg = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
    if(info_msg == NULL)
    {
        return;
    }
    rx_len -= FCS_LEN;
    ninfo_data_len = (rx_len + 9);


    info_msg[offset + 0] = 'N';
    info_msg[offset + 1] = g_network_info_id;
    /* Increment the id sequence counter by 1*/
    g_network_info_id++;

    info_msg[offset + 2] = ninfo_data_len & 0xFF;
    info_msg[offset + 3] = (ninfo_data_len & 0xFF00) >> 8;

    info_msg[offset + 4] = WID_NETWORK_INFO & 0xFF;
    info_msg[offset + 5] = (WID_NETWORK_INFO & 0xFF00) >> 8;

    info_msg[offset + 6] = (rx_len + 1) & 0xFF;
    info_msg[offset + 7] = ((rx_len + 1) & 0xFF00) >> 8;

    info_msg[offset + 8] = rssi;

    memcpy32(&info_msg[offset + 9], msa, rx_len);

    send_host_rsp(info_msg, ninfo_data_len, host_type);
	
// 20120830 caisf mod, merged ittiam mac v1.3 code
//#endif /* ETHERNET_HOST */
    }
    TROUT_FUNC_EXIT;
#endif /* IBSS_BSS_STATION_MODE */
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_sta_join_info_to_host                               */
/*                                                                           */
/*  Description   : This function sends info of a joining/leaving sta to host*/
/*                                                                           */
/*  Inputs        : station address                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares an Information frame having the   */
/*                  information about a joining/leaving station.             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_join_leave_info_to_host(UWORD16 aid, UWORD8* sta_addr, BOOL_T joining)
{
	//chenq add return 2012-10-30
	return;

#ifndef ETHERNET_HOST
    UWORD8   *info_msg  = NULL;
    UWORD8   host_type  = get_host_config_if_type();
    UWORD16  offset     = get_config_pkt_hdr_len(host_type);
    UWORD16  info_len   = 16;

	TROUT_FUNC_ENTER;
	
    info_msg = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
    if(info_msg == NULL)
    {
    	TROUT_FUNC_EXIT;
        return;
    }

    /* Get the information of joining station */
    if(joining == BTRUE)
    {
        info_len = get_sta_join_info(sta_addr, &info_msg[offset + 8]);
    }
    /* prepare the information of leaving station */
    else
    {
        /* Reset all the fields */
        mem_set(&info_msg[offset + 8], 0, info_len);

        /* Copy the association id of the leaving station */
        info_msg[offset + 8] = aid & 0xFF;

        /* Copy the MAC address of the leaving station */
        memcpy(&info_msg[offset + 9], sta_addr, 6);
    }

    if(info_len)
    {
        info_msg[offset + 0] = 'N';
        info_msg[offset + 1] =  g_info_id;

        /* Increment the id sequence counter by 1*/
        g_info_id++;

        /* Prepare the frame */
        info_msg[offset + 2] =  (info_len + 8) & 0xFF;
        info_msg[offset + 3] = ((info_len + 8) & 0xFF00) >> 8;

        info_msg[offset + 4] = WID_STA_JOIN_INFO & 0xFF;
        info_msg[offset + 5] = (WID_STA_JOIN_INFO & 0xFF00) >> 8;

        info_msg[offset + 6] =  info_len & 0xFF;
        info_msg[offset + 7] = (info_len & 0xFF00) >> 8;

        send_host_rsp(info_msg, (info_len + 8), host_type);
    }
    else
    {
        pkt_mem_free(info_msg);
    }
    TROUT_FUNC_EXIT;
#endif /* ETHERNET_HOST */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : save_wids                                                */
/*                                                                           */
/*  Description   : This function saves all the wid values to a global arry  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_char_wid_struct                                        */
/*                  g_short_wid_struct                                       */
/*                  g_int_wid_struct                                         */
/*                  g_str_wid_struct                                         */
/*                  g_binary_data_wid_struct                                 */
/*                  g_current_settings                                       */
/*                  g_current_len                                            */
/*                                                                           */
/*  Processing    : Checks if the WID has a get and set function and stores  */
/*                  the values of all such WIDs in to global array for       */
/*                  restoring from it later                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void save_wids(void)
{
    UWORD16 count = 0;
    UWORD16 idx   = 0;

	TROUT_FUNC_ENTER;
    /* Save all the settings in the global array. This is saved as a */
    /* dummy Write message with all WID values. It is used to set    */
    /* the parameters once reset is done.                            */

    /* The global WID array is accessed and all the current WID  */
    /* values are saved in the format of WID ID, Length, Value.  */

	for(count = 0;
		g_char_wid_struct[count].id != WID_MAX;
		count++)
    {
        if((g_char_wid_struct[count].set) &&
           (g_char_wid_struct[count].get))
            idx += get_wid(&g_current_settings[idx], count,
                           WID_CHAR);
    }

	for(count = 0;
		g_short_wid_struct[count].id != WID_MAX;
		count++)
    {
        if((g_short_wid_struct[count].set)&&
           (g_short_wid_struct[count].get))
            idx += get_wid(&g_current_settings[idx], count,
                           WID_SHORT);
    }

	for(count = 0;
		g_int_wid_struct[count].id != WID_MAX;
		count++)
    {
        if((g_int_wid_struct[count].set)&&
           (g_int_wid_struct[count].get))
            idx += get_wid(&g_current_settings[idx], count,
                           WID_INT);
    }

	for(count = 0;
		g_str_wid_struct[count].id != WID_MAX;
		count++)
    {
        if((g_str_wid_struct[count].set)&&
           (g_str_wid_struct[count].get))
            idx += get_wid(&g_current_settings[idx], count,
                           WID_STR);

        /* Check that it does not cross g_current_settings array boundary */
        if(idx >= (MAX_QRSP_LEN - 255))
            break;
    }

    /* Set the length of global current configuration settings */
    g_current_len = idx;

#ifdef DEBUG_MODE
    /* Check whether query response length exceeded max length */
    if(g_current_len > MAX_QRSP_LEN)
    {
		TROUT_DBG3("Err: WID Save Response exceeded the max size.\n");
        g_mac_stats.max_query_rsp_len_exceeded = BTRUE;
    }
#endif /* DEBUG_MODE */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : restore_wids                                             */
/*                                                                           */
/*  Description   : This function restores all the wids from the global arry */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_current_settings                                       */
/*                  g_current_len                                            */
/*                                                                           */
/*  Processing    : Calls the process write function for restoring all the   */
/*                  WID values                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void restore_wids(void)
{
	TROUT_FUNC_ENTER;
    /* Indicate start of WID restoration to all the Protocols */
    indicate_wid_restore_prot(BTRUE);

    /* Set all the required parameters from the saved configuration  */
    /* settings.                                                     */
    process_write(g_current_settings, g_current_len);

    /* Indicate completion of WID restoration to all the Protocols */
    indicate_wid_restore_prot(BFALSE);
    TROUT_FUNC_EXIT;
}




/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_device_mode                                       */
/*                                                                           */
/*  Description   : This function updates the device mode of operation       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function updates the device mode of operation       */
/*                  depending upon if P2P code is included or not. If P2P    */
/*                  code is present then it checks if p2p functionality and  */
/*                  p2p auto GO are enabled. Depending upon the various      */
/*                  combination of these two switches the mode of operation  */
/*                  is changed                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void update_device_mode(void)
{
	TROUT_FUNC_ENTER;
#ifdef BSS_ACCESS_POINT_MODE
    if(BSS_STA == get_device_mode())
    {
        /* TBD : Switch non-P2P STA */
    }
#ifdef MAC_P2P

    /* |------------------------------------------------------------------|*/
    /* | Curr Device-Mode | P2P enabled | P2P autoGO | Next Device-Mode   |*/
    /* |                  |             |   enabled  |                    |*/
    /* |------------------------------------------------------------------|*/
    /* |                  |     0       |     0      |    ACCESS_POINT    |*/
    /* |                  |-----------------------------------------------|*/
    /* |                  |     0       |     1      |    ACCESS_POINT    |*/
    /* |                  |             |            | (disable P2PautoGO)|*/
    /* |  NOT_CONFIGURED  |-----------------------------------------------|*/
    /* |                  |     1       |     0      |       P2P_GO       |*/
    /* |                  |             |            | (enable P2PautoGO) |*/
    /* |                  |-----------------------------------------------|*/
    /* |                  |     1       |     1      |       P2P_GO       |*/
    /* |------------------|-----------------------------------------------|*/
    /* |                  |     0       |     0      | No Change in Mode  |*/
    /* |                  |-----------------------------------------------|*/
    /* |                  |     0       |     1      |  No Change in Mode |*/
    /* |  ACCESS_POINT    |             |            | (disable P2PautoGO)|*/
    /* |                  |-------------|------------|--------------------|*/
    /* |                  |     1       |     0      |    P2P_DEVICE      |*/
    /* |                  |-------------|------------|--------------------|*/
    /* |                  |     1       |     1      |    P2P_DEVICE      |*/
    /* |------------------|-----------------------------------------------|*/
    /* |                  |     0       |     0      |    non-P2P STA     |*/
    /* |                  |-------------|------------|--------------------|*/
    /* |                  |     0       |     1      |    non-P2P STA     |*/
    /* |    P2P_GO        |-------------|------------|--------------------|*/
    /* |                  |     1       |     0      |    P2P_DEVICE      |*/
    /* |                  |-------------|------------|--------------------|*/
    /* |                  |     1       |     1      |  No Change in Mode |*/
    /* |------------------|-----------------------------------------------|*/

    switch(get_device_mode())
    {
    case NOT_CONFIGURED:
    {
        /* If P2P is enabled then change the mode to P2P GO. Auto GO should */
        /* also be enabled. If P2P is not enabled then change the mode to   */
        /* non P2P Access Point. The Auto GO should be disabled             */
        if(BTRUE == mget_p2p_enable())
        {
            set_device_mode(P2P_GO);
            mset_p2p_auto_go(BTRUE);
        }
        else
        {
            set_device_mode(ACCESS_POINT);
            mset_p2p_auto_go(BFALSE);
        }
    }
    break;
    case ACCESS_POINT:
    {
        /* If the device was in non P2P AP mode and if P2P is enabled    */
        /* then switch to P2P device mode                                */
        if(BTRUE == mget_p2p_enable())
        {
            TROUT_DBG4("AP : Switching to Client Mode\n\r");
            /* Switch to P2P Device mode */
            initiate_mod_switch(&g_mac);
        }
        else if(BTRUE == mget_p2p_auto_go())
        {
            /* If the device is in ACCESS_POINT mode and p2p is not enabled */
            /* but auto GO is enabled then disable auto GO                  */
            mset_p2p_auto_go(BFALSE);
        }
    }
    break;
    case P2P_GO:
    {
        /* If the device is in P2P GO mode and P2P or P2PAUTOGO is       */
        /* disabled then Switch to STA binary                            */
        if((BFALSE == mget_p2p_enable()) ||
           (BFALSE == mget_p2p_auto_go()))
        {
            TROUT_DBG4("P2P GO: Switching to Client Mode\n\r");
            /* Switch to non-P2P STA mode */
            initiate_mod_switch(&g_mac);
        }
    }
    break;
    default:
    {
        /* Nothing to do */
    }
  }

#else  /* MAC_P2P */
    /* Set the device mode to ACCESS_POINT */
    if(NOT_CONFIGURED == get_device_mode())
        set_device_mode(ACCESS_POINT);
#endif /* MAC_P2P */
#endif /* BSS_ACCESS_POINT_MODE */


#ifdef IBSS_BSS_STATION_MODE
    if(ACCESS_POINT == get_device_mode())
    {
        /* TBD : Switch non-P2P ACCESS_POINT */
    }
#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {
        if((BTRUE == is_p2p_grp_form_in_prog()) &&
           (P2P_DEVICE != get_device_mode()))
        {
            /* If P2P is enabled and the device is in group formation stage */
            /* the set the device mode to P2P_DEVICE                        */
            set_device_mode(P2P_DEVICE);
        }
        TROUT_FUNC_EXIT;
        return;
    }
#endif /* MAC_P2P */

    /* If the device is a Non-P2P STA then set the device mode to IBSS_STA or*/
    /* BSS_STA depending upon the BSS type                                   */
    if((IBSS_STA != get_device_mode()) &&
        (INDEPENDENT == mget_DesiredBSSType()))
    {
        set_device_mode(IBSS_STA);
    }
    else if((BSS_STA != get_device_mode()) &&
            (INFRASTRUCTURE == mget_DesiredBSSType()))
    {
        set_device_mode(BSS_STA);
    }
#endif /* IBSS_BSS_STATION_MODE */
	TROUT_FUNC_EXIT;
}

/* This function resolves various configuration level conflicts that might */
/* arise between WIDs. This is a temporary solution. Further clean-up is   */
/* required to meet MAC Coding standards.                                  */
void resolve_wid_conflicts(mac_struct_t *mac)
{
	TROUT_FUNC_ENTER;
#ifdef BSS_ACCESS_POINT_MODE
#ifdef MAC_802_11N
    /* In AP mode, Enable HT capability only under No-Security or if Security*/
    /* is enabled with AES                                                   */
    if(get_ht_mib_enable() == 1)
    {
        if((mget_PrivacyInvoked() == TV_TRUE) && (is_ccmp_enabled() == BFALSE))
            set_ht_enable(0);
        else
            set_ht_enable(1);
    }
    else
    {
    	set_ht_enable(0);
    }

#ifndef WPS_1_0_SEC_SUPP
    /* WSC 2.0 is not supported for WEP and TKIP Encryption type or if SSID  */
    /* broadcast is disabled                                                 */
    if(get_wps_implemented() == BTRUE)
    {
        if((get_bcst_ssid() == 1) ||
           ((mget_PrivacyInvoked() == TV_TRUE) &&
            (is_ccmp_enabled() == BFALSE)))
        {
            enable_wps_mode(BFALSE);
        }
        else
        {
            enable_wps_mode(BTRUE);
        }
    }
    else
    {
        set_wps_enable(BFALSE);
    }
#else  /* WPS_1_0_SEC_SUPP */
    /* WPS is not supported SSID broadcast is disabled */
    if(get_wps_implemented() == BTRUE)
    {
        if(get_bcst_ssid() == 1)
            enable_wps_mode(BFALSE);
        else
            enable_wps_mode(BTRUE);
    }
    else
    {
        set_wps_enable(BFALSE);
    }
#endif /* WPS_1_0_SEC_SUPP */
#endif /* MAC_802_11N */
#endif /* BSS_ACCESS_POINT_MODE */

#ifdef IBSS_BSS_STATION_MODE
#ifdef MAC_802_11N
    /* Disable HT if WEP is enabled */
    if(get_ht_mib_enable() == 1)
    {
        if(is_wep_enabled() == BTRUE)
        {
            set_ht_enable(0);
        }
        else
        {
            set_ht_enable(1);
        }
    }
    else
    {
    	set_ht_enable(0);
    }
#endif /* MAC_802_11N */
#endif /* IBSS_BSS_STATION_MODE */

#ifdef MAC_P2P
    if(BTRUE == mget_p2p_enable())
    {   /* Setting required mandatorily for P2P */
        set_11g_op_mode(G_ONLY_MODE);
        set_802_11I_mode(0x31); /* WPA2-AES */

#ifdef IBSS_BSS_STATION_MODE
        set_wps_dev_mode(WPS_STANDALONE_DEVICE);

        if(BTRUE == is_p2p_grp_form_in_prog())
        {
			/* If Group formation is in progress, set the Current Channel to */
			/* the same value as Listen Channel.                             */
			mset_CurrentChannel(mget_p2p_listen_chan());

			/* Resolve conflicts for WPS settings and P2P group formation */
            handle_wps_p2p_grp_form_in_prog();
        }

        if(BTRUE == mget_p2p_auto_go())
        {
			/* If we need to become a GO then end group formation */
            g_p2p_GO_role = BTRUE;

			// 20120709 caisf add, merged ittiam mac v1.2 code
            /* Disable any WPS protocol that was started */
            set_wps_prot(WPS_PROT_NONE);

            p2p_end_grp_form(mac);
        }

#endif /* IBSS_BSS_STATION_MODE */
        if(BFALSE == is_mac_addr_null(mget_p2p_invit_dev_id()))
        {
            mset_p2p_trgt_dev_id(mget_p2p_invit_dev_id());
            g_p2p_invit = SEND_INVITE;
			// 20120709 caisf mod, merged ittiam mac v1.2 code
            //p2p_start_invit_scan_req();
            p2p_start_invit_scan_req_mode();
        }
    }
#endif /* MAC_P2P */

    /* Update the Secondary Channel Setting based upon the current settings  */
    /* of Primary Channel, Secondary Channel and applicable Regulatory Domain*/
    update_operating_channel();

    update_device_mode();
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : config_op_params                                         */
/*                                                                           */
/*  Description   : This function starts the test configuration.             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : g_current_len                                            */
/*                  g_current_settings                                       */
/*                                                                           */
/*  Processing    : This function configures the operataing parameters.      */
/*                  if no restore data is present it initializes the default */
/*                  configuration. It prepares a 'W' configuration           */
/*                  message, packs it in the required host format and        */
/*                  simulates receiving the packet from the host by calling  */
/*                  a host interface function to receive from host.          */
/*                                                                           */
/*  Outputs       : An event with a configuration message is added to the    */
/*                  HOST_RX queue                                            */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void config_op_params(mac_struct_t *mac)
{
#ifdef DEFAULT_SME
    UWORD8  i       = 0;
#endif /* DEFAULT_SME */
    UWORD16 msg_len = 0;
    UWORD8  *msg    = 0;

	TROUT_FUNC_ENTER;
#ifdef DEFAULT_SME
    if(0 == g_current_len)
    {
        /* Initialize the global test configuration structure */
        initialize_test_config(); //dumy mark for debug 0815
    }
#endif /* DEFAULT_SME */

    /* Allocate shared memory buffer for the configuration message */
    msg = pkt_mem_alloc(MEM_PRI_CONFIG);
    if(msg == NULL)
    {
        /* Exception */
        TROUT_FUNC_EXIT;
        return;
    }

    /* Set the Message Type and ID. The Message Length is set after the      */
    /* message contents are written. The length is known only after that.    */
    msg[0] = 'W';
    msg[1] = 0;

    msg_len = MSG_DATA_OFFSET;

    if(0 == g_current_len)
    {
#ifdef DEFAULT_SME
        /* Set the contents of the message as per the user settings     */
        /* (read from the global). Update the length with every setting */
        for(i = 0; i < NUM_TEST_WIDS; i++)
        {
            msg_len += set_test_wid(&msg[msg_len], i);
        }
#endif /* DEFAULT_SME */
    }
    else
    {
        TROUT_DBG4("Configuring device from saved settings\n\r");
        memcpy(&msg[msg_len], g_current_settings, g_current_len);
        msg_len += g_current_len;

		/* Switching from other mode has started - This flag is set to allow */
		/* the setting of certain WIDs that should not be modified by the    */
		/* user but need to be set from the saved settings during a switch.  */
		set_switch_in_progress(BTRUE);
    }

    /* The length of the message returned is set in the Message length       */
    /* field.                                                                */
    msg[2] = msg_len & 0xFF;
    msg[3] = (msg_len & 0xFF00) >> 8;

	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    if(MSG_DATA_OFFSET != msg_len)
	{
		/* The test configuration message is directly parsed for further     */
		/* action. Note that no event is posted for test configuration.      */
    	/* Also, since this is not a user generated configuration message, no    */
    	/* response is sent back to the host.                                    */
    	parse_config_message(mac, msg, msg, INVALID_HOST_TYPE);
    }

	/* Switching is complete - This flag is reset to disallow the setting of */
	/* certain WIDs that should not be modified by the user.                 */
	set_switch_in_progress(BFALSE);

    start_p2p_prot();
    TROUT_FUNC_EXIT;
}

#ifdef DEBUG_MODE

/*****************************************************************************/
/*                                                                           */
/*  Function Name : printh                                                   */
/*                                                                           */
/*  Description   : This function sends a message for printing at the host.  */
/*                                                                           */
/*  Inputs        : 1) Variable list of arguments (similar to printf)        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares a configuration packet with the   */
/*                  given message to be printed and sends it to the host.    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void printh(WORD8 *msg, ...)
{
    UWORD8      host_type          = get_host_config_if_type();
    UWORD8      config_pkt_hdr_len = get_config_pkt_hdr_len(host_type);
    UWORD16     buflen             = 0;
    UWORD16     config_pkt_len     = 0;
    UWORD8      *buffer            = 0;
    msdu_desc_t frame_desc         = {0};
#ifdef OS_LINUX_CSL_TYPE
    va_list     args;
#endif /* OS_LINUX_CSL_TYPE */

    buffer = pkt_mem_alloc(MEM_PRI_CONFIG);

    if(buffer == NULL)
    {
            return;
    }

#ifdef OS_LINUX_CSL_TYPE
    /* Parse the arguments passed and create the message to be transmitted */
    va_start(args, msg);
    buflen = vsprintf(buffer + config_pkt_hdr_len, msg, args);
    va_end(args);
#endif /* OS_LINUX_CSL_TYPE */


    config_pkt_len = prepare_config_pkt_hdr(buffer, buflen, host_type);

    /* Update the configuration host frame descriptor */
    frame_desc.buffer_addr     = buffer;
    frame_desc.host_hdr_len    = config_pkt_hdr_len;
    frame_desc.data_len        = config_pkt_len - config_pkt_hdr_len;
    frame_desc.host_hdr_offset = 0;
    frame_desc.data_offset     = config_pkt_hdr_len;

    /* Send the packet to the host interface. The buffer will be freed in */
    /* the host completion function once the packet has been transmitted. */
    send_config_frame_to_host_if(&frame_desc, host_type);
}

#endif /* DEBUG_MODE */

#ifdef IBSS_BSS_STATION_MODE
//chenq add do ap list merge logic 2013-08-28
WORD32 merge_ap_list (void)
{
	int ret = 0;
    int get_ap_cnt = 0;
    int i=0,j=0;
	UWORD8 * trout_rsp = NULL;
    UWORD8   scan_results[SITE_SURVEY_RESULTS_ELEMENT_LENGTH*5+3] = {0};
	UWORD16  trout_rsp_len = 0;
    bss_dscr_t *itm_iw_bss_dscr[MAX_SITES_FOR_SCAN]; //= {{INFRASTRUCTURE,0,},};
    int frag_num = 0;
    int scan_cnt = 0;
	int index = 0;
	UWORD8 * scan_results_cur;
	

	//do ap list merge
	//bss_dscr_t * pbss;

	bss_link_dscr_t * sbss;

	bss_link_dscr_t * sbss_pre;

	bss_link_dscr_t * sbss_min_rssi;

	UWORD32 *tmp_32 = NULL;
	
	UWORD16 * generic_ie_len = NULL;

	struct timespec time;
	//struct timeval  time;
	
	UWORD32 now= 0 ;

	TROUT_DBG4("merge_ap_list fuc (sta mode)\n");
	
	//before we allocate the buffers, we should clear the buffer  zhao
	for(i = 0; i < MAX_SITES_FOR_SCAN; i++) //MAX_SITES_FOR_SCAN 16
		itm_iw_bss_dscr[i] = NULL;

    for( i = 0 ; i < MAX_SITES_FOR_SCAN ; i++ )
    {
        itm_iw_bss_dscr[i] = (bss_dscr_t *)kmalloc(sizeof(bss_dscr_t),
                                                    GFP_KERNEL);
        if( itm_iw_bss_dscr[i] != NULL)
        {
            itm_iw_bss_dscr[i]->bss_type = INFRASTRUCTURE;
        }
        else
        {
		   	TROUT_DBG2("@@@ merge_ap_list no mem 0\n");
			ret = -ENOMEM;
		   	goto out1;
        }
    }

    do
	{
		TROUT_DBG4("merge_ap_list fuc while 1 !!!\n");
		//trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&trout_rsp_len);
		trout_rsp = get_scan_ap_list(&trout_rsp_len);

		if( trout_rsp == NULL )
		{
		      TROUT_DBG4("merge_ap_list fuc while 2 !!!\n");  
			//ret = -EINVAL;
			ret = -EAGAIN;
			goto out1;
		}
		 
		trout_rsp_len -= MSG_HDR_LEN;
		if(get_trour_rsp_data( scan_results,
		                       sizeof(scan_results),
		                       &trout_rsp[MSG_HDR_LEN],
						       trout_rsp_len,WID_STR)  ==  0 )
		{
		    TROUT_DBG4("merge_ap_list fuc while 3 !!!\n");  
			ret = -EINVAL;
			goto out2;
		}


		frag_num  = scan_results[2] & 0xFF;
		scan_cnt  = (scan_results[0] | (scan_results[1] << 8)) / SITE_SURVEY_RESULTS_ELEMENT_LENGTH;

		TROUT_DBG4("merge_ap_list fuc in while ... frag_num:%d,scan_cnt:%d !!!\n",
						frag_num,scan_cnt);

		if( (frag_num >= 1) && (scan_cnt != 0) )
		{
		    index = 5*frag_num;
		}
		else
		{
		    index = 0;
		}

		if(scan_cnt > 0)
		{
			scan_results_cur = scan_results+3;
			for(i=0;i<scan_cnt;i++)
			{
				scan_results_cur = get_scan_ap_info(itm_iw_bss_dscr[index+i],
													  scan_results_cur);
				get_ap_cnt++;
			}
		}

		pkt_mem_free(trout_rsp);
		trout_rsp = NULL;
		               
		TROUT_DBG4("merge_ap_list fuc in while ... %d !!!\n",scan_results[2]);
	}
    while( scan_results[2] < 3 );

	TROUT_DBG4("merge_ap_list fuc found %d ap from user scan\n",get_ap_cnt);
	//////////////////////////////////////////////////////////////////////////////////////

	i=0;
	//sprd_nstime_get(&time);
	getnstimeofday(&time);
	now = (UWORD32)time.tv_sec;
	
	//begin modified by junwei.jiang 20121231 
    sbss_pre = NULL;
	sbss=g_user_getscan_aplist;
	while( sbss != NULL )
	{	
		if(sbss_pre == NULL)
		{
			tmp_32 = (UWORD32 *)sbss->bss_next;

			kfree(sbss->bss_curr);
			kfree(sbss);

			sbss = (bss_link_dscr_t *)tmp_32;
			g_user_getscan_aplist = (bss_link_dscr_t *)tmp_32;
		}
		else
		{
			sbss_pre->bss_next = sbss->bss_next;

			kfree(sbss->bss_curr);
			kfree(sbss);

			sbss = sbss_pre->bss_next;
		}
			g_link_list_bss_count--;
			TROUT_DBG5("first delete complete,g_link_list_bss_count = %d\n",g_link_list_bss_count);
			continue;
	}
	//end modified by junwei.jiang 20121231 
	
	g_merge_aplist_flag = 1;
	//add or update ap info
	if( g_user_getscan_aplist == NULL )
	{
		g_user_getscan_aplist = (bss_link_dscr_t *)kmalloc(sizeof(bss_link_dscr_t),
                                                    GFP_KERNEL); 

		if(!g_user_getscan_aplist)
		{
			TROUT_DBG2("@@@ merge_ap_list no mem 1\n");
            ret = -ENOMEM;
		   	goto out1;
		}

		g_user_getscan_aplist->bss_curr = itm_iw_bss_dscr[0];
		g_user_getscan_aplist->bss_next = NULL;

		g_user_getscan_aplist->ap_active  = 1;
		g_user_getscan_aplist->bss_actcnt = MAX_SCAN_ACT_CNT;
		g_user_getscan_aplist->bss_tstamp = (UWORD32)time.tv_sec;		

		generic_ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_3[0]);
		memcpy(g_user_getscan_aplist->generic_ie,g_wpa_rsn_ie_buf_3[0],*generic_ie_len + 2);
		itm_iw_bss_dscr[0] = NULL;

		i++;
		g_link_list_bss_count++;
		TROUT_DBG5("add head g_link_list_bss_count = %d\n",g_link_list_bss_count);
	}

	sbss=g_user_getscan_aplist;
	sbss_min_rssi=g_user_getscan_aplist;

	while(i<get_ap_cnt)
	{

		TROUT_DBG5("scan ap index is  %d \n",i);
        //begin modified by junwei.jiang 20121231
        #if 0   
		if (0x00 == memcmp(itm_iw_bss_dscr[i]->bssid,sbss->bss_curr->bssid, 6 ))
		{
			/*TROUT_DBG4("find bss %s => %s      %02x:%02x:%02x:%02x:%02x:%02x => %02x:%02x:%02x:%02x:%02x:%02x\n",
				sbss->bss_curr->ssid,itm_iw_bss_dscr[i]->ssid,
				sbss->bss_curr->bssid[0],sbss->bss_curr->bssid[1],sbss->bss_curr->bssid[2],
				sbss->bss_curr->bssid[3],sbss->bss_curr->bssid[4],sbss->bss_curr->bssid[5],
				itm_iw_bss_dscr[i]->bssid[0],itm_iw_bss_dscr[i]->bssid[1],itm_iw_bss_dscr[i]->bssid[2],
				itm_iw_bss_dscr[i]->bssid[3],itm_iw_bss_dscr[i]->bssid[4],itm_iw_bss_dscr[i]->bssid[5]);
			TROUT_DBG4("find bss %s \n",sbss->bss_curr->ssid);
			*/
			
			sbss->ap_active  = 1;
			sbss->bss_actcnt = /*MAX_SCAN_ACT_CNT;*/((sbss->bss_actcnt + 1) >= MAX_SCAN_ACT_CNT) ?
				                MAX_SCAN_ACT_CNT : sbss->bss_actcnt + 1;
			sbss->bss_tstamp = (UWORD32)time.tv_sec;
			
			kfree(sbss->bss_curr);
			sbss->bss_curr = itm_iw_bss_dscr[i];
			itm_iw_bss_dscr[i] = NULL;

			generic_ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_3[i]);
			if(*generic_ie_len + 2 > 512) {
				printk("%s[%d] invailed longth %d\n", __FUNCTION__, __LINE__,
					*generic_ie_len + 2);
			}
			memcpy(sbss->generic_ie,g_wpa_rsn_ie_buf_3[i],*generic_ie_len + 2);

			i++;
			sbss = g_user_getscan_aplist;
			sbss_min_rssi = g_user_getscan_aplist;
		}	
		else if( sbss->bss_next  != NULL )
		#else
		//end modified by junwei.jiang 20121231
		if( sbss->bss_next  != NULL )
		{
			if(sbss_min_rssi->bss_curr->rssi > sbss->bss_curr->rssi)
			{
				sbss_min_rssi=sbss;
				TROUT_DBG5("curr sbss_min_rssi 2  is %d\n",i);			
			}
			
			TROUT_DBG5("connot find bss\n");
			sbss = sbss->bss_next;
		}
		else
		{
			TROUT_DBG5("add new bss %s \n",itm_iw_bss_dscr[i]->ssid);
		
			if( g_link_list_bss_count >= MAX_SITES_FOR_SCAN )
			{
				if(sbss_min_rssi->bss_curr->rssi < itm_iw_bss_dscr[i]->rssi)
				{
					/*TROUT_DBG4("curr rssi >= %d,will update to %d\n",sbss_min_rssi->bss_curr->rssi ,i);
					TROUT_DBG4("curr p_bss %s => %s      %02x:%02x:%02x:%02x:%02x:%02x => %02x:%02x:%02x:%02x:%02x:%02x\n",
					sbss_min_rssi->bss_curr->ssid,itm_iw_bss_dscr[i]->ssid,
					sbss_min_rssi->bss_curr->bssid[0],sbss_min_rssi->bss_curr->bssid[1],sbss_min_rssi->bss_curr->bssid[2],
					sbss_min_rssi->bss_curr->bssid[3],sbss_min_rssi->bss_curr->bssid[4],sbss_min_rssi->bss_curr->bssid[5],
					itm_iw_bss_dscr[i]->bssid[0],itm_iw_bss_dscr[i]->bssid[1],itm_iw_bss_dscr[i]->bssid[2],
					itm_iw_bss_dscr[i]->bssid[3],itm_iw_bss_dscr[i]->bssid[4],itm_iw_bss_dscr[i]->bssid[5]);
					
					TROUT_DBG4("find bss %s \n",sbss->bss_curr->ssid);
					*/
					sbss_min_rssi->ap_active  = 1;
					sbss_min_rssi->bss_actcnt =MAX_SCAN_ACT_CNT;
					sbss_min_rssi->bss_tstamp = (UWORD32)time.tv_sec;

					kfree(sbss_min_rssi->bss_curr);
					sbss_min_rssi->bss_curr= itm_iw_bss_dscr[i];
					itm_iw_bss_dscr[i] = NULL;

					generic_ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_3[i]);
					if(*generic_ie_len + 2 > 512) {
						printk("%s[%d] invailed longth %d\n", __FUNCTION__, __LINE__,
						*generic_ie_len + 2);
					}
					memcpy(sbss_min_rssi->generic_ie,g_wpa_rsn_ie_buf_3[i],*generic_ie_len + 2);
					TROUT_DBG5("g_link_list_bss_count >= %d,will add new a high rssi\n",MAX_SITES_FOR_SCAN);
				}
				
				TROUT_DBG5("g_link_list_bss_count >= %d,will not add new\n",MAX_SITES_FOR_SCAN);
				
				i++;
				sbss=g_user_getscan_aplist;
				sbss_min_rssi=g_user_getscan_aplist;
			}
			else
			{
				sbss->bss_next = (bss_link_dscr_t *)kmalloc(sizeof(bss_link_dscr_t),
                                                    GFP_KERNEL); 
				if(sbss->bss_next == NULL)
				{
					i++;
					sbss=g_user_getscan_aplist;
					TROUT_DBG2("@@@ merge_ap_list no mem 2\n");
					continue;
				}

				/*TROUT_DBG4("add bss %s  %02x:%02x:%02x:%02x:%02x:%02x\n",
					itm_iw_bss_dscr[i]->ssid,
					itm_iw_bss_dscr[i]->bssid[0],itm_iw_bss_dscr[i]->bssid[1],itm_iw_bss_dscr[i]->bssid[2],
					itm_iw_bss_dscr[i]->bssid[3],itm_iw_bss_dscr[i]->bssid[4],itm_iw_bss_dscr[i]->bssid[5]);
				*/
			
				sbss = sbss->bss_next;
				sbss->bss_next = NULL;
		
				sbss->ap_active    = 1;
				sbss->bss_actcnt   = MAX_SCAN_ACT_CNT;
				sbss->bss_tstamp   = (UWORD32)time.tv_sec;
				sbss->bss_curr     = itm_iw_bss_dscr[i];
			
				generic_ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_3[i]);
				if(*generic_ie_len + 2 > 512) {
					printk("%s[%d] invailed longth %d\n", __FUNCTION__, __LINE__,
					*generic_ie_len + 2);
				}
				memcpy(sbss->generic_ie,g_wpa_rsn_ie_buf_3[i],*generic_ie_len + 2);
				itm_iw_bss_dscr[i] = NULL;

				i++;
				sbss = g_user_getscan_aplist;
				sbss_min_rssi = g_user_getscan_aplist;
				g_link_list_bss_count++;
				TROUT_DBG5("add new g_link_list_bss_count = %d\n",g_link_list_bss_count);
			}
		}
	}
	#endif
	sbss=g_user_getscan_aplist;
	sbss_pre = NULL;
#ifdef COMBO_SCAN
	for(j=0;j<MAX_AP_COMBO_SCAN_LIST;j++)
	{
		if((0 == g_combo_aplist[j].cur_flag) && (g_combo_aplist[j].cnt !=0))
		{
			while( sbss != NULL )
			{
				if(strcmp(sbss->bss_curr->ssid,g_combo_aplist[j].ssid) == 0)
				{
					if(sbss_pre == NULL)
					{

						tmp_32 = (UWORD32 *)sbss->bss_next;
						kfree(sbss->bss_curr);
						kfree(sbss);
						sbss = (bss_link_dscr_t *)tmp_32;
						g_user_getscan_aplist = (bss_link_dscr_t *)tmp_32;
					}
					else
					{
						sbss_pre->bss_next = sbss->bss_next;
						kfree(sbss->bss_curr);
						kfree(sbss);
						sbss = sbss_pre->bss_next;
					}
					printk("g_combo_aplist detele with j %d ssid:%s\n",j,g_combo_aplist[j].ssid);
					g_link_list_bss_count--;
					continue;
				}
				sbss_pre = sbss;		
				sbss	 = sbss->bss_next;
			}
		}
		if(j == (MAX_AP_COMBO_SCAN_LIST-1))
		printk("g_combo_aplist no find in merge ap list\n");
	}
#endif
    //begin modified by junwei.jiang 20121231
    #if 0
	//delete ap info
	sbss=g_user_getscan_aplist;
	sbss_pre = NULL;

	while( sbss != NULL )
	{
		if( sbss->ap_active == 0 )
		{
			TROUT_DBG5("start delete bss %s,actcnt = %d,now = %d,tstamp = %d \n",
				sbss->bss_curr->ssid,sbss->bss_actcnt,now,sbss->bss_tstamp);
			if (--sbss->bss_actcnt == 0 ||( (now - sbss->bss_tstamp) > timeoutValue) )
			{	
				if(sbss_pre == NULL)
				{
					tmp_32 = (UWORD32 *)sbss->bss_next;

					kfree(sbss->bss_curr);
					kfree(sbss);

					sbss = (bss_link_dscr_t *)tmp_32;
					g_user_getscan_aplist = (bss_link_dscr_t *)tmp_32;
				}
				else
				{
					sbss_pre->bss_next = sbss->bss_next;

					kfree(sbss->bss_curr);
					kfree(sbss);

					sbss = sbss_pre->bss_next;
				}
				g_link_list_bss_count--;
				TROUT_DBG5("delete complete,g_link_list_bss_count = %d!!!\n",g_link_list_bss_count);	
				continue;
			}
		}
		
		sbss->ap_active = 0;
		sbss_pre		= sbss;
		sbss			= sbss->bss_next;

	}
	#endif
	//end modified by junwei.jiang 20121231
	
	g_merge_aplist_flag = 0;
	TROUT_DBG5("merge is complete\n");

////////////////////////////////////////////////////////////////////////////////////////////
	out2:
	if(trout_rsp != NULL)
	{
		pkt_mem_free(trout_rsp);
	}
	
	out1:
    for( i = 0 ; i < MAX_SITES_FOR_SCAN ; i++ )
    {
        if( itm_iw_bss_dscr[i] != NULL)
        {
            kfree(itm_iw_bss_dscr[i]);
        }
    }
	
	return ret;
}

//chenq add this fuc 0727
UWORD8 * get_scan_ap_list(UWORD16 * trout_rsp_len)
{
    static int flag = 0;
	UWORD8 * msg = NULL;
	int i=0;

	UWORD16 * ie_len = NULL;

	if( atomic_read(&ap_list_copy_buf_lock) != 0)
	{
		flag = 0;
		return NULL;
	}

	atomic_inc(&ap_list_copy_buf_lock);
	
	if(flag == 0)
	{
		if(g_ap_list_copy_len1 == 0)
		{
			goto out;
		}
		
		msg =  (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
		if(msg == NULL)
		{
			goto out;
		}
		*trout_rsp_len = g_ap_list_copy_len1;
		memcpy(msg,g_ap_list_copy1,g_ap_list_copy_len1);
		flag++;

		for(i = 0 ; i < MAX_SITES_FOR_SCAN ; i++)
		{
			ie_len = (UWORD16 *)(g_wpa_rsn_ie_buf_2[i]);
		
			//memcpy(g_wpa_rsn_ie_buf_3[i],g_wpa_rsn_ie_buf_2[i],g_wpa_rsn_ie_buf_2[i][0]+1);
			memcpy(g_wpa_rsn_ie_buf_3[i],g_wpa_rsn_ie_buf_2[i],*ie_len + 2);
			TROUT_DBG5("g_wpa_rsn_ie_buf_2[%d] len = %d\n",i,g_wpa_rsn_ie_buf_2[i][0]);
		}
	}
	else if(flag == 1)
	{
		//mutex_lock(&ap_list_copy_buf_lock);
		if(g_ap_list_copy_len2 == 0)
		{
			goto out;
		}
		
		msg =  (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
		if(msg == NULL)
		{
			goto out;
		}
		*trout_rsp_len = g_ap_list_copy_len2;
		memcpy(msg,g_ap_list_copy2,g_ap_list_copy_len2);
		flag++;
	}
	else if(flag == 2)
	{
		//mutex_lock(&ap_list_copy_buf_lock);
		if(g_ap_list_copy_len3 == 0)
		{
			goto out;
		}
		
		msg =  (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
		if(msg == NULL)
		{
			goto out;
		}
		*trout_rsp_len = g_ap_list_copy_len3;
		memcpy(msg,g_ap_list_copy3,g_ap_list_copy_len3);
		flag++;
	}
	else if(flag == 3)
	{
		//mutex_lock(&ap_list_copy_buf_lock);
		if(g_ap_list_copy_len4 == 0)
		{
			goto out;
		}
		
		msg =  (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
		if(msg == NULL)
		{
			goto out;
		}
		*trout_rsp_len = g_ap_list_copy_len4;
		memcpy(msg,g_ap_list_copy4,g_ap_list_copy_len4);
		flag = 0;;
	}
	

	out:
	atomic_dec(&ap_list_copy_buf_lock);
	return msg;
}


//chenq add this fuc 0727
void copy_scan_ap_list(void)
{
	UWORD8 * trout_rsp = NULL;
	UWORD8 * host_req  = NULL;
	TROUT_FUNC_ENTER;
	if( atomic_read(&ap_list_copy_buf_lock) != 0)
	{
		TROUT_FUNC_EXIT;
		return;
	}
	atomic_inc(&ap_list_copy_buf_lock);
	
	host_req = g_ap_list_copy1;
	host_req[0] = WID_SITE_SURVEY_RESULTS & 0xFF;
	host_req[1] = (WID_SITE_SURVEY_RESULTS & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&g_ap_list_copy_len1);

	if( trout_rsp != NULL )
	{
	    memcpy(g_ap_list_copy1,trout_rsp,g_ap_list_copy_len1);
		pkt_mem_free(trout_rsp);
	}
	else
	{
		g_ap_list_copy_len1 = 0;
	}
	

	host_req = g_ap_list_copy2;
	host_req[0] = WID_SITE_SURVEY_RESULTS & 0xFF;
	host_req[1] = (WID_SITE_SURVEY_RESULTS & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&g_ap_list_copy_len2);

	if( trout_rsp != NULL )
	{
	    memcpy(g_ap_list_copy2,trout_rsp,g_ap_list_copy_len2);
		pkt_mem_free(trout_rsp);
	}
	else
	{
		g_ap_list_copy_len2 = 0;
	}

	host_req = g_ap_list_copy3;
	host_req[0] = WID_SITE_SURVEY_RESULTS & 0xFF;
	host_req[1] = (WID_SITE_SURVEY_RESULTS & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&g_ap_list_copy_len3);

	if( trout_rsp != NULL )
	{
	    memcpy(g_ap_list_copy3,trout_rsp,g_ap_list_copy_len3);
		pkt_mem_free(trout_rsp);
	}
	else
	{
		g_ap_list_copy_len3 = 0;
	}

	host_req = g_ap_list_copy4;
	host_req[0] = WID_SITE_SURVEY_RESULTS & 0xFF;
	host_req[1] = (WID_SITE_SURVEY_RESULTS & 0xFF00) >> 8;

	trout_rsp = config_if_for_iw(&g_mac,host_req,2,'Q',&g_ap_list_copy_len4);

	if( trout_rsp != NULL )
	{
	    memcpy(g_ap_list_copy4,trout_rsp,g_ap_list_copy_len4);
		pkt_mem_free(trout_rsp);
	}
	else
	{
		g_ap_list_copy_len4 = 0;
	}

	atomic_dec(&ap_list_copy_buf_lock);

	//chenq add do ap list merge logic 2013-08-28
	merge_ap_list ();	
	
	TROUT_FUNC_EXIT;
	return;
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_mac_status                                          */
/*                                                                           */
/*  Description   : This function sends the status of MAC to host            */
/*                                                                           */
/*  Inputs        : MAC Status                                               */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares an Information frame having the   */
/*                  MAC Status and calls send_host_rsp function to send the  */
/*                  prepared frame to host                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_mac_status(UWORD8 mac_status)
{
	//chenq mod all in this fuc
	union iwreq_data wrqu;
	static int report_dis_flag = 0;

	/*leon liu added connection result arguments*/
#ifdef CONFIG_CFG80211
	UWORD8 *bssid = NULL;
	UWORD8 *req_ie = NULL;
	UWORD32 req_ie_len = 0;
	UWORD8 *resp_ie = NULL;
	UWORD32 resp_ie_len = 0;
#endif
	TROUT_FUNC_ENTER;
	switch ( mac_status )
    {
        case MAC_CONNECTED:
        {
			#ifdef IBSS_BSS_STATION_MODE
			UWORD8 * ie_info = NULL;
			TROUT_DBG5("<=chenq send_mac_status: Connected\n");
			report_dis_flag = 1;
			
			ie_info = get_assoc_req_info();			
			wrqu.data.length = ie_info[0];
			wrqu.data.flags = 0;
			if(ie_info[0] == 0){
                #ifdef CONFIG_TROUT_WEXT
				wireless_send_event(g_mac_dev, IWEVASSOCREQIE, &wrqu, NULL);
                #endif
            }
			else{
                #ifdef CONFIG_TROUT_WEXT
				wireless_send_event(g_mac_dev, IWEVASSOCREQIE, &wrqu, &ie_info[1]);
                #endif
				/*leon liu added for CFG80211 layer*/
				#ifdef CONFIG_CFG80211
				req_ie = &ie_info[1];
				req_ie_len = ie_info[0];
				#endif
			}
			#ifdef CONFIG_CFG80211
			bssid = get_bssid();
			bssid++;
			#endif
			ie_info = get_assoc_res_info();
		    wrqu.data.length = ie_info[0];
			wrqu.data.flags = 0;
			if(ie_info[0] == 0){
                #ifdef CONFIG_TROUT_WEXT
				wireless_send_event(g_mac_dev, IWEVASSOCRESPIE, &wrqu,NULL);
                #endif
            }
			else{
                #ifdef CONFIG_TROUT_WEXT
				wireless_send_event(g_mac_dev, IWEVASSOCRESPIE, &wrqu, &ie_info[1]);
                #endif
				/*leon liu added for CFG80211 layer*/
				#ifdef CONFIG_CFG80211
				resp_ie = &ie_info[1];
				resp_ie_len = ie_info[0];
				#endif
            }
			memcpy(&wrqu.ap_addr.sa_data, (get_bssid() + 1), ETH_ALEN);
            #ifdef CONFIG_TROUT_WEXT
			wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
            #endif
			netif_carrier_on(g_mac_dev);

                        #ifdef IBSS_BSS_STATION_MODE
			//add by chengwg for bt&wifi coex.
			TROUT_DBG5("======%s: notify arm7 connected=====\n", __func__);
			host_notify_arm7_connect_status(BTRUE);
			#endif /* IBSS_BSS_STATION_MODE */
			/*leon liu added CFG80211 reporting connected event and bssid,assoc request and response*/
			#ifdef CONFIG_CFG80211
			//cfg80211_connect_result() will handle NULL pointers
			trout_cfg80211_report_connect_result(g_mac_dev, bssid, req_ie, req_ie_len
							, resp_ie, resp_ie_len, WLAN_STATUS_SUCCESS);
			#endif
			
			TROUT_DBG5("Connected.\n\r");
			TROUT_DBG5("=>chenq send_mac_status: Connected\n");	
			#endif
		}
        break;

        case MAC_DISCONNECTED:
        {  
			/*xuan yang, 2013.5.30, don't send state to sup if resume assoc*/
			if(1 == get_resume_assoc_flg()) {
				set_resume_assoc_flg(0);
				report_dis_flag = 0;
				printk("don't send mac disconnected state to ui\n");
				return;
			}

			#ifdef IBSS_BSS_STATION_MODE
			if(report_dis_flag)
			{
				TROUT_DBG4("<=chenq send_mac_status: Disconnected\n");
				wrqu.ap_addr.sa_family = ARPHRD_ETHER;
				memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);
                #ifdef CONFIG_TROUT_WEXT
	    		wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
                #endif
				/*leon liu added CFG80211 reporting disconncet event*/
				#ifdef CONFIG_CFG80211
				printk("Reporting DISCONNECT event to CFG80211\n");
				cfg80211_disconnected(g_mac_dev, 0, NULL, 0, GFP_KERNEL);
				#endif

				TROUT_DBG5("Disconnected.\n\r");
				TROUT_DBG5("=>chenq send_mac_status: Disconnected\n");
				netif_carrier_off(g_mac_dev);
				report_dis_flag = 0;
				
				//xuan yang, 2013.7.05, clean the disconnect flag;
				send_disconnect_flg(0);
			}
			else if(1 == send_disconnect_flg(0))
			{
				//xuan yang, 2013.7.05, send disconnect state to supplicant ,when the current essid is not null. 
				TROUT_DBG4("<=chenq send_mac_status: connect failed\n");
				wrqu.ap_addr.sa_family = ARPHRD_ETHER;
				memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);
                #ifdef CONFIG_TROUT_WEXT
	    		wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
                #endif
				TROUT_DBG5("Connect failed.\n\r");
				TROUT_DBG5("=>chenq send_mac_status: connect failed\n");
				netif_carrier_off(g_mac_dev);
				report_dis_flag = 0;
			}
			else
			{
				TROUT_DBG4("<=chenq send_mac_status: Disconnected\n");
				TROUT_DBG4("will not report Disconnected.\n\r");
				TROUT_DBG4("=>chenq send_mac_status: Disconnected\n");
			}

#ifdef IBSS_BSS_STATION_MODE
			//add by chengwg for bt&wifi coex.
			TROUT_DBG5("========%s: notify arm7 discon=======\n", __func__);
			host_notify_arm7_connect_status(BFALSE);
#endif	/* IBSS_BSS_STATION_MODE */

			//#endif   chenq mod 
			#else
				wrqu.ap_addr.sa_family = ARPHRD_ETHER;
				memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);
                #ifdef CONFIG_TROUT_WEXT
    			wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
                #endif
				netif_carrier_off(g_mac_dev);
			#endif
		}

        break;
        // add by Ke.Li at 2013-04-04 for fix refresh UI ap list
        case MAC_CONNECT_FAILED:
        {   
			#ifdef IBSS_BSS_STATION_MODE
		    TROUT_DBG4("<=chenq send_mac_status: Connected failed\n");
			wrqu.ap_addr.sa_family = ARPHRD_ETHER;
			memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);
            #ifdef CONFIG_TROUT_WEXT
	    	wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
            #endif
			TROUT_DBG5("Connect failed.\n\r");
			TROUT_DBG5("=>chenq send_mac_status: Connected failed\n");
			netif_carrier_off(g_mac_dev);
			#else
			wrqu.ap_addr.sa_family = ARPHRD_ETHER;
			memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);
            #ifdef CONFIG_TROUT_WEXT
    		wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
            #endif
			netif_carrier_off(g_mac_dev);
			#endif
			/*leon liu added CFG80211 reporting disconncet event*/
			#ifdef CONFIG_CFG80211
			printk("Reporting DISCONNECT event to CFG80211\n");
			trout_cfg80211_report_scan_done(g_mac_dev, 1);
			cfg80211_disconnected(g_mac_dev, 0, NULL, 0, GFP_KERNEL);
			#endif

		}
        break;
        // end add by Ke.Li at 2013-04-04 for fix refresh UI ap list

        // chenq add for post close event to os 2013-05-14
		case MAC_CLOSE:
		{   
			#ifdef IBSS_BSS_STATION_MODE
		    TROUT_DBG4("<=chenq send_mac_status: mac close \n");
			wrqu.ap_addr.sa_family = ARPHRD_ETHER;
			memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);
                     #ifdef CONFIG_TROUT_WEXT
	    	wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
                     #endif
			TROUT_DBG5("mac close.\n\r");
			TROUT_DBG5("=>chenq send_mac_status: mac close\n");
			netif_carrier_off(g_mac_dev);

#ifdef IBSS_BSS_STATION_MODE
			//add by chengwg for bt&wifi coex.
			TROUT_DBG4("=======%s: notify arm7 mac close======\n", __func__);
			host_notify_arm7_connect_status(BFALSE);
#endif /* IBSS_BSS_STATION_MODE */

			#else
			wrqu.ap_addr.sa_family = ARPHRD_ETHER;
			memset(wrqu.ap_addr.sa_data, 0, ETH_ALEN);
            #ifdef CONFIG_TROUT_WEXT
    		wireless_send_event(g_mac_dev, SIOCGIWAP, &wrqu, NULL);
            #endif
			netif_carrier_off(g_mac_dev);
			#endif
		}
		break;
		//chena add end

		case MAC_SCAN_CMP:
		{
			//printk("<=chenq send_mac_status: MAC_SCAN_CMP\n");
			#ifdef IBSS_BSS_STATION_MODE
			//copy_scan_ap_list();
			if(is_scanlist_report2ui)
			{
				printk("will report 2 ui \n");
				if(is_scanlist_report2ui == 1)
				{
					copy_scan_ap_list();
				}
				
				is_scanlist_report2ui = 0;
				
				wrqu.data.length = 0;
    			wrqu.data.flags = 0;
                #ifdef CONFIG_TROUT_WEXT
    			wireless_send_event(g_mac_dev, SIOCGIWSCAN, &wrqu, NULL);
                #endif
				/*leon liu added for cfg80211 event reporting*/
				#ifdef CONFIG_CFG80211
				trout_cfg80211_report_scan_done(g_mac_dev, 0);
				#endif
				//chenq add 2012-10-29
				if(itm_scan_task != NULL)
				{
					printk("chenq debug MAC_SCAN_CMP \n");
					wake_up_process(itm_scan_task);
					itm_scan_task = NULL;
				}	
			}
			#endif
    		TROUT_DBG5("MAC_SCAN_CMP.\n\r");
			TROUT_DBG5("=>chenq send_mac_status: MAC_SCAN_CMP\n");
		}
		break;

		case MAC_AP_UP:
		{
			#ifdef BSS_ACCESS_POINT_MODE
			#if WIRELESS_EXT > 17
			
			char data[] = "AP_UP";

			TROUT_DBG5("<=chenq send_mac_status: UP\n");
    		wrqu.data.length = sizeof(data);
    		wrqu.data.flags = 0;
            #ifdef CONFIG_TROUT_WEXT
    		wireless_send_event(g_mac_dev, IWEVCUSTOM, &wrqu, data);
		    #endif
			TROUT_DBG5("=>chenq send_mac_status: UP\n");
			#endif
			netif_carrier_on(g_mac_dev);
			#endif
		}
		break;

		case MAC_AP_DOWN:
		{
			#ifdef BSS_ACCESS_POINT_MODE
			#if WIRELESS_EXT > 17
			char data[] = "AP_DOWN";
			TROUT_DBG5("<=chenq send_mac_status: DOWN\n");

			wrqu.data.length = sizeof(data);
    		wrqu.data.flags = 0;
            #ifdef CONFIG_TROUT_WEXT
    		wireless_send_event(g_mac_dev, IWEVCUSTOM, &wrqu, data);
			#endif
			TROUT_DBG5("=>chenq send_mac_status: DOWN\n");
			#endif
			#endif
		}
		break;
		case MAC_CTL_OK:
		{
			#if WIRELESS_EXT > 17
			union iwreq_data wrqu;
			char data[] = "STARTED";

			TROUT_DBG5("<=chenq send_mac_status: MAC_CTL_OK\n");	
			wrqu.data.length = sizeof(data);
			wrqu.data.flags = 0;
            #ifdef CONFIG_TROUT_WEXT
			wireless_send_event(g_mac_dev, IWEVCUSTOM, &wrqu, data);
            #endif
			TROUT_DBG5("<=chenq send_mac_status: MAC_CTL_OK\n");
			
			#endif
		}
		break;
    }

#if 0
#ifdef SEND_MAC_STATUS
    UWORD8 *info_msg = NULL;
    UWORD8 *info_buf = NULL;

    /* Send Connection status always. This was done to fix the PMK Caching   */
    /* issue with WZC                                                        */
    {
        info_buf = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);

        if(info_buf == NULL)
        {
            return;
        }

        info_msg = info_buf +
                   get_config_pkt_hdr_len(get_host_config_if_type());

        info_msg[0] = 'I';
        info_msg[1] = g_info_id;

        /* Increment the id sequence counter by 1*/
        g_info_id++;

        info_msg[2] = (STATUS_MSG_LEN) & 0xFF;
        info_msg[3] = ((STATUS_MSG_LEN) & 0xFF00) >> 8;

        /* Set the WID_STATUS, Length to 1, Value to mac_status */
        info_msg[4] = WID_STATUS & 0xFF;
        info_msg[5] = (WID_STATUS & 0xFF00) >> 8;
        info_msg[6] = 1;
        info_msg[7] = mac_status;

        /* The response is sent back to the host */
        send_host_rsp(info_buf, STATUS_MSG_LEN, get_host_config_if_type());
        g_current_mac_status = mac_status;

        switch ( mac_status )
        {
            case MAC_CONNECTED:
            {
                PRINTD("Connected.\n\r");

            }
            break;

            case MAC_DISCONNECTED:
            {
                PRINTD("Disconnected.\n\r");

            }
            break;
        }
    }
#endif /* SEND_MAC_STATUS */
#endif
	TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_dev_spec_info                                       */
/*                                                                           */
/*  Description   : This function does initialization of device specific     */
/*                  information                                              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_info_id                                                */
/*                                                                           */
/*  Processing    : This function initializes the values for device info     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_dev_spec_info(void)
{
	TROUT_FUNC_ENTER;
#ifdef DEVICE_SPECIFIC_CONFIG
    set_dev_os_version(DEFAULT_DEV_OS_VERSION);
    set_manufacturer((UWORD8 *)g_default_manufacturer);
    set_model_name((UWORD8 *)g_default_model_name);
    set_model_num((UWORD8 *)g_default_model_num);
    set_dev_name((UWORD8 *)g_default_device_name);
#endif /* DEVICE_SPECIFIC_CONFIG */
	TROUT_FUNC_EXIT;
}

#ifdef INT_WPS_SUPP
/*****************************************************************************/
/*                                                                           */
/*  Function Name : send_wps_status                                          */
/*                                                                           */
/*  Description   : This function displays the WPS status (i.e. it sends the */
/*                  status message to the host)                              */
/*                                                                           */
/*  Inputs        : 1) WPS status code                                       */
/*                  2) Pointer to the WPS display message string             */
/*                  3) Length of the WPS message string                      */
/*                                                                           */
/*  Globals       : g_info_id                                                */
/*                                                                           */
/*  Processing    : This function prepares an Information type configuration */
/*                  packet with WID_WPS_STATUS that includes the WPS         */
/*                  status code and status message string. The same is then  */
/*                  sent to the host. Required buffers are allocated and     */
/*                  freed.                                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void send_wps_status(UWORD8 code, UWORD8* msg_ptr, UWORD16 msg_len)
{
	TROUT_FUNC_ENTER;
	/* Sending of Async messages is not supported for MWLAN */
#ifndef MWLAN
    UWORD16      wps_status_msg_len = 0;
    UWORD8       *info_msg          = 0;
    UWORD8       *info_buf          = 0;

    info_buf = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
    if(info_buf == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
    }

    info_msg = info_buf + get_config_pkt_hdr_len(get_host_config_if_type());

    /* If the incoming message length exceeds the limit, reset the same to   */
    /* 0 thus displaying only the status code                                */
    if(msg_len > MAX_WPS_STATUS_MSG_LEN)
        msg_len = 0;

    wps_status_msg_len = MSG_DATA_OFFSET + WID_VALUE_OFFSET + 2 + msg_len;

    /* Set the contents of the WPS Status message to be sent to the host */

    /* WID_WPS_STATUS (String WID) Message Format                            */
    /* --------------------------------------------------------------------- */
    /* | Message Type | ID | Length | WID Type | WID Length | WID Data     | */
    /* --------------------------------------------------------------------- */
    /* | 1            | 1  | 2      | 2        | 1          | 1 + msg_len  | */
    /* --------------------------------------------------------------------- */
    info_msg[0] = 'I';
    info_msg[1] = g_info_id++;
    info_msg[2] = (wps_status_msg_len) & 0xFF;
    info_msg[3] = ((wps_status_msg_len) & 0xFF00) >> 8;
    info_msg[4] = WID_WPS_STATUS & 0xFF;
    info_msg[5] = (WID_WPS_STATUS & 0xFF00) >> 8;
    info_msg[6] = (2 + msg_len);

#ifdef IBSS_BSS_STATION_MODE
    info_msg[7] = WPS_MODE_ENROLLEE;
#else  /* IBSS_BSS_STATION_MODE */
    info_msg[7] = WPS_MODE_REGISTRAR;
#endif /* IBSS_BSS_STATION_MODE */
    /* WID_WPS_STATUS Message Data Format        */
    /* ----------------------------------------- */
    /* | Status Code  | Status Message String  | */
    /* ----------------------------------------- */
    /* | 1            | msg_len                | */
    /* ----------------------------------------- */
    info_msg[8] = code;
    memcpy(&info_msg[9], msg_ptr, msg_len);

    /* Send the information message to the host */
    send_host_rsp(info_buf, wps_status_msg_len, get_host_config_if_type());
#endif /* MWLAN */
    TROUT_DBG4("WPS status: %02x\n", code);
    TROUT_FUNC_EXIT;
}
#endif /* INT_WPS_SUPP */

//chenq add for iw config
UWORD8 * parse_config_message_for_iw(mac_struct_t *mac, UWORD8* host_req,
                          UWORD8 *buffer_addr, UWORD8 host_if_type,
                          UWORD16* host_rsp_len)
{
    UWORD8  msg_type  = 0;
    UWORD8  msg_id    = 0;
    UWORD16 msg_len   = 0;
    BOOL_T  free_flag = BTRUE;
    UWORD8  offset    = get_config_pkt_hdr_len(host_if_type);
	struct trout_private *tp;

     //CHECK_MAC_RESET_IN_IW_HANDLER_RETURN_NULL(LPM_NO_ACCESS);	//add by chengwg, 2013.7.9
    /* Extract the Type, Length and ID of the incoming host message. The     */
    /* format of the message is:                                             */
    /* +-------------------------------------------------------------------+ */
    /* | Message Type | Message ID |  Message Length |Message body         | */
    /* +-------------------------------------------------------------------+ */
    /* |     1 Byte   |   1 Byte   |     2 Bytes     | Message Length      | */
    /* +-------------------------------------------------------------------+ */

    msg_type = host_req[0];
    msg_id   = host_req[1];
    msg_len  = (host_req[3] << 8) | host_req[2];
    msg_len -= MSG_HDR_LEN;

    /* The  valid types of incoming messages are 'Q' (Query) and 'W' (Write) */
    switch(msg_type)
    {
    	case 'Q':
	    {
	        UWORD16 rsp_len    = 0;
	        UWORD8  *query_rsp = 0;
	        UWORD8  *rsp_buff  = 0;

			//chenq add for g_reset_mac_in_progress mean not ready
			if(g_reset_mac_in_progress)
			{
				pkt_mem_free(buffer_addr);
				return NULL;
			}

	        /* To a Query message a Response message needs to be sent. This      */
	        /* message has a type 'R' and should have an ID equal to the         */
	        /* incoming Query message.                                           */

	        rsp_buff = (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);
	        if(rsp_buff == NULL)
	        {
	            pkt_mem_free(buffer_addr);
	            return NULL;
	        }

	        query_rsp = rsp_buff + offset;

	        /* Set the Message Type and ID. The Message Length is set after the  */
	        /* message contents are written. The length is known only after that.*/
	        query_rsp[0] = 'R';
	        query_rsp[1] = msg_id;

	        /* Process the Query message and set the contents as required. */
	        rsp_len = process_query(&host_req[MSG_DATA_OFFSET],
	                                &query_rsp[MSG_DATA_OFFSET], msg_len);

	        /* The length of the message returned is set in the Message length   */
	        /* field.                                                            */
	        query_rsp[2] = (rsp_len + MSG_HDR_LEN) & 0xFF;
	        query_rsp[3] = ((rsp_len + MSG_HDR_LEN) & 0xFF00) >> 8;

			if(free_flag == BTRUE)
	        	pkt_mem_free(buffer_addr);

	        /*set The response len. No MAC state changes occur */
	        /* on reception of a Query message.                         */
			*host_rsp_len = (UWORD16)(rsp_len + MSG_HDR_LEN);
			return query_rsp;
	    }

    	case 'W':
	    {	
			
			//chenq add for g_reset_mac_in_progress mean not ready
			if(g_reset_mac_in_progress)
			{
				pkt_mem_free(buffer_addr);
				return NULL;
			}
			
	        /* Check if any protocol disallows processing of this write request   */
	        if(BFALSE == process_wid_write_prot((host_req + MSG_DATA_OFFSET),
	                                            msg_len))
	        {
	            /* If the buffer is not freed yet, free it */
	            if(free_flag == BTRUE)
	               pkt_mem_free(buffer_addr);

				return NULL;
	        }

#ifdef BURST_TX_MODE
	        if(get_DesiredBSSType() != 4)
	        {
	            g_burst_tx_mode_enabled_earlier = BFALSE;
	        }
	        else
	        {
	            g_burst_tx_mode_enabled_earlier = BTRUE;
	        }
#endif /* BURST_TX_MODE */

	        /* This is done since some WPS specific WIDs can do an internal MAC  */
	        /* reset.                                                            */
	        lock_the_buffer(buffer_addr);

	        /* Indicate that WID Config Write is in progress */
	        indicate_config_write_prot(BTRUE);

	        /* Process the Write message and set the parameters with the given   */
	        /* values.                                                           */

	        process_write(host_req + MSG_DATA_OFFSET, msg_len);
	        /* Indicate that WID Config Write is in completed */
	        indicate_config_write_prot(BFALSE);
	        
	        /* Handle WPS related configuration update */
	        wps_handle_config_update();	
		
	        unlock_the_buffer(buffer_addr);
	        /* Change MAC states as required. If it is already in Enabled state  */
	        /* reset MAC and start again. The previous configuration values are  */
	        /* retained.                                                         */

	        if(g_reset_mac == BTRUE)
	        {
#ifndef MAC_HW_UNIT_TEST_MODE
	        	if((get_mac_state() != DISABLED) && (allow_mac_reset() == BTRUE))
#endif /* MAC_HW_UNIT_TEST_MODE */
		        {
#ifdef IBSS_BSS_STATION_MODE
#ifdef INT_WPS_SUPP
		            /* If there is reset because of config packet received from host */
		            /* then disable the WPS protocol. Please note WPS specific WIDs  */
		            /* must not require a reset                                      */
		            disable_wps_mode();
#endif /* INT_WPS_SUPP */
#endif /* IBSS_BSS_STATION_MODE */

//                    CHECK_MAC_RESET_IN_IW_HANDLER_RETURN_NULL;  //add by chengwg, 2013.7.9
//					//xuan yang, 2013-8-23, add wid mutex
//					tp = netdev_priv(g_mac_dev);
//					mutex_lock(&tp->rst_wid_mutex);

					g_reset_mac_in_progress    = BTRUE;

		            /* Free the buffer */
		            pkt_mem_free(buffer_addr);

		            free_flag = BFALSE;
		            /* Save the current configuration before going for Reset         */
		            save_wids();

		            /* Reset MAC - Bring down PHY and MAC H/W, disable MAC           */
		            /* interrupts and release all OS structures. Further, this       */
		            /* function restarts the MAC again from start.                   */
		            reset_mac(mac, BTRUE);

		            /* Restore the saved configuration before resetting              */
		            restore_wids();

		            g_reset_mac_in_progress    = BFALSE;
//					mutex_unlock(&tp->rst_wid_mutex);
#ifdef BURST_TX_MODE
		            /* Start scaning only when burst tx mode is disabled */
		            if(g_burst_tx_mode_enabled == BTRUE)
		            {
		               init_mac_fsm_bt(mac);
		            }
#endif /* BURST_TX_MODE */
		        }
	        }
			else
			{
				pkt_mem_free(buffer_addr);
			}

	        /* This function resolves all configuration related conflicts that */
	        /* might arise between various WIDs.                               */
	        resolve_wid_conflicts(mac);

#ifndef MAC_HW_UNIT_TEST_MODE
	        if((get_mac_state() == DISABLED) && (allow_mac_reset() == BTRUE))
#else /* MAC_HW_UNIT_TEST_MODE */
	        if(g_test_start == BTRUE)
#endif /* MAC_HW_UNIT_TEST_MODE */
	        {
//	            CHECK_MAC_RESET_IN_IW_HANDLER_RETURN_NULL;  //add by chengwg, 2013.7.9
	        	#ifdef IBSS_BSS_STATION_MODE
				//chenq add 2012-11-01
				if( (g_reset_mac == BTRUE) || (g_start_scan_req == BTRUE) )
	            {
					//chenq mask 2013-01-06
	            	//g_start_scan_req = BTRUE;

					start_mac_and_phy(mac);

					//chenq add 2013-01-06
					g_start_scan_req = BFALSE;
				}
				#else
				if(g_reset_mac == BTRUE)
					start_mac_and_phy(mac);
				#endif
	        }

	        /* To a Write message a Response message needs to be sent. This      */
	        /* message has a type 'R' and should have an ID equal to the         */
	        /* incoming Write message. The Message contents contain 'Y' to       */
	        /* indicate Success.                                                 */

			*host_rsp_len = 1;
			return NULL;
			#if 0
			if(free_flag == BFALSE)
	        {// Success
				*host_rsp_len = 1;
			}	
	        else
	        {//fail
	        	*host_rsp_len = 0;
	        }

			return NULL;
			#endif
	    }

	    default:
	    {
	        /* Unknown message type. Do nothing. */
	    }
    } /* end of switch(msg_type) */

    /* If the buffer is not freed yet, free it */
    if(free_flag == BTRUE)
        pkt_mem_free(buffer_addr);

	*host_rsp_len = 0;

    return NULL;	
}


UWORD8 * config_if_for_iw(mac_struct_t *mac,UWORD8 * host_req,UWORD16 host_req_len,
					      		char msgtype,UWORD16 * trout_rsp_len)
{
    UWORD16 msg_len = 0;
    UWORD8  *msg    = 0;
    UWORD8  *result = 0;	

//    CHECK_MAC_RESET_IN_IW_HANDLER_RETURN_NULL;  //add by chengwg, 2013-07-09.
    /* Allocate shared memory buffer for the configuration message */
    msg =  (UWORD8*)pkt_mem_alloc(MEM_PRI_CONFIG);

    if(msg == NULL)
    {
        /* Exception */
		*trout_rsp_len = -1;	
        return NULL;
    }

    /* Set the Message Type and ID. The Message Length is set after the      */
    /* message contents are written. The length is known only after that.    */
    msg[0] = msgtype;
    msg[1] = 0;

    msg_len = MSG_DATA_OFFSET;

    //PRINTD("Configuring device from iw ioctl \n\r");
    memcpy(&msg[msg_len], host_req, host_req_len);
    msg_len += host_req_len;

    /* The length of the message returned is set in the Message length       */
    /* field.                                                                */
    msg[2] = msg_len & 0xFF;
    msg[3] = (msg_len & 0xFF00) >> 8;

    /* The test configuration message is directly parsed for further action. */
    /* Note that no event is posted for test configuration.                  */
    /* Also, since this is not a user generated configuration message, no    */
    /* response is sent back to the host.                                    */
    result =  parse_config_message_for_iw(mac, msg, msg, INVALID_HOST_TYPE,trout_rsp_len);
    return result;	
}

//xuan yang, 2013-7-05, set and get the disconnected flag.
UWORD8 send_disconnect_flg(UWORD8 inp)
{
	static UWORD8 send_disc_flg = 0;
	UWORD8 tmp = send_disc_flg;

    send_disc_flg = inp; 
    printk("%s send_disc_flg from %d to %d\n", __FUNCTION__, tmp, send_disc_flg);
	return tmp;
}

