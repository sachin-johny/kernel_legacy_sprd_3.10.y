/*****************************************************************************/
/*                                                                           */
/*              Ittiam 802.11 MAC HARDWARE UNIT TEST CODE                    */
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
/*  File Name         : mh_test_config.h                                     */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to the H/w unit test configuration.          */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

#ifndef MH_TEST_CONFIG_H
#define MH_TEST_CONFIG_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "mh.h"

#ifdef MAC_WMM
#include "mib_11e.h"
#endif /* MAC_WMM */

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define NUM_RATES            12
#define MAX_HUT_NUM_INIT_PKT 10
#define MAX_PATTERN_LEN      1024
#define MAX_STATS_BUFFER_LEN 1500
#define DEFAULT_SQ_AVG_CNT   16

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/* PHY Test modes */
typedef enum {PHY_TEST_DISABLED             = 0,
              PHY_TEST_PER_TX_MODE          = 1,
              PHY_TEST_PER_RX_MODE          = 2,
              PHY_TEST_BURST_TX_MODE        = 3,
              PHY_TEST_CONTINUOUS_TX_MODE_0 = 4,
              PHY_TEST_CONTINUOUS_TX_MODE_1 = 5,
              PHY_TEST_CONTINUOUS_TX_MODE_2 = 6,
              PHY_TEST_CONTINUOUS_TX_MODE_3 = 7,
              PHY_TEST_NO_MODULATION_MODE   = 8,
              NUM_PHY_TEST_MODE
} PHY_TEST_MODE_T;

typedef enum
{
    VARY_FRAME_FORMAT = BIT0,
    VARY_FRAME_LENGTH = BIT1,
    VARY_FRAME_RATE   = BIT2
} VARY_VARIABLE_TYPES_T;

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    UWORD8  antset;
    UWORD8  bandwidth;
    UWORD8  dtim_period;
    UWORD8  erp_prot_type;
    UWORD8  ess;
    UWORD8  g_running_mode;
    UWORD8  ht_operating_mode;
    UWORD8  ht_prot_type;
    UWORD8  ht_coding;
    UWORD8  mac_mode;
    UWORD8  op_band;
    UWORD8  preamble;
    UWORD8  prot_type;
    UWORD8  rifs_mode;
    UWORD8  shortgi;
    UWORD8  smps_mode;
    UWORD8  smoothing_rec;
    UWORD8  sounding_pkt;
    UWORD8  stbc;
    UWORD8  tx_format;
    UWORD8  tx_power_11a;
    UWORD8  tx_power_11n;
    UWORD8  tx_power_11n40;
    UWORD8  tx_power_11b;
    UWORD8  tx_rate;
    UWORD8  tx_mcs;
    UWORD8  sec_chan_offset;
    UWORD8  machw_2040_coex;
    UWORD8  fc_txop_mod;
    UWORD8  fc_prot_type;
    UWORD8  sec_cca_assert;
    UWORD8  start_freq;
    WORD16  ac_vo_txop_limit;
    UWORD16 bcn_period;
    UWORD16 channel;
    UWORD16 frag_threshold;
    UWORD16 rts_threshold;
    UWORD8  bssid[6];
    UWORD8  mac_address[6];
} test_params_struct_t;

typedef struct
{
    UWORD8  txcomplete;
    UWORD8  rxcomplete;
    UWORD8  tbtt;
    UWORD8  cape;
    UWORD8  err;
    UWORD8  hprx_comp;
    UWORD8  radar_det;
    UWORD8  txsus;
    UWORD8  deauth;
} int_test_config_struct_t;

typedef struct
{
    UWORD16 tsf_test;
    UWORD16 bcn_swap_test;
    UWORD16 bcn_len;
    UWORD16 bcn_rate;
    UWORD32 next_bcn_addr;  /* Not user configurable */
    UWORD32 bcn_addr;       /* Not user configurable */
} tsf_test_config_struct_t;

typedef struct
{
    UWORD32 num_rx_buffers[2];
    UWORD8  ack_filter;
    UWORD8  cts_filter;
    UWORD8  rts_filter;
    UWORD8  bcn_filter;
    UWORD8  atim_filter;
    UWORD8  cf_end_filter;
    UWORD8  qcf_poll_filter;
    UWORD8  dup_filter;
    UWORD8  fcs_filter;
    UWORD8  non_dir_data_filter;
    UWORD8  non_dir_mgmt_filter;
    UWORD8  non_dir_ctrl_filter;
    UWORD8  bcmc_mgmt_obss_filter;
    UWORD8  bcmc_data_obss_filter;
    UWORD8  exp_ba_filter;
    UWORD8  unexp_ba_filter;
    UWORD8  deauth_filter;
    UWORD8  sec_chan_filter;
    UWORD8  disc_ibss_bcn_filter;
    UWORD8  hp_rxq_enable;
    UWORD8  hp_rxq_probereq;
    UWORD8  hp_rxq_probersp;
    UWORD8  hp_rxq_beacon;
    UWORD8  hp_rxq_atim;
    UWORD8  soft_crc_enable;
    UWORD8  soft_dupchk_enable;
    UWORD8  soft_integrity_check;
    UWORD8  tkip_mic_check;
    UWORD8  replay_detect;
} rx_test_config_struct_t;

typedef struct
{
    UWORD8 mode;
    UWORD8 rate_hi;
    UWORD8 rate_lo;
} phy_test_config_struct_t;

typedef struct
{
    UWORD8  init_num_pkt;
    UWORD8  priority;   /* Queue-ID */
    UWORD8  ack_policy;
    UWORD16 frame_len;
    UWORD32 frame_addr;
    UWORD32 bcst_frame_addr;
    UWORD16 num_submsdu;
    UWORD16 seq_num;
    UWORD16 min_frame_len; /* Not user configurable */
    UWORD16 cur_frame_len; /* Not user configurable */
} q_info_t;

typedef struct
{
    UWORD8  var_type;   /* Type of Variation required        */
    UWORD32 max_idx;    /* The Maximum index that is allowed */
    UWORD32 min_idx;    /* The Minimum index that is allowed */
    UWORD32 cur_idx;    /* The index used for current packet */
    UWORD32 num_val;    /* The Number of Valid Values        */
    UWORD8  *var_val;   /* Array of allowed values           */
} var_param_t;

typedef struct
{
    UWORD8      control;
    var_param_t txfmt;
    var_param_t rate_11b;
    var_param_t rate_11a;
    var_param_t rate_11g;
    var_param_t rate_11n;
    var_param_t length;
} test_var_param_t;

typedef struct
{
    UWORD8   tx_test;
    UWORD8   slot;
    UWORD8   da[6];
    UWORD32  num_pkts;
    UWORD32  test_time;
    UWORD8   frame_type;
    UWORD8   dls_prot;
    UWORD8   enable_retry_ar;
    UWORD8   bcst_percent;
    UWORD8   retry_rates[8];
    UWORD8   init_num_pkt; /* Not user configurable          */
    UWORD32  tx_dscr_addr; /* Not user configurable          */
    UWORD8   rtscts_txop;  /* Enable/Disable RTS/CTS in TXOP */
    test_var_param_t  vppp; /* Variable Per Pkt Parameter Struct */
#ifdef MAC_WMM
    q_info_t ac_bk;
    q_info_t ac_be;
    q_info_t ac_vo;
    q_info_t ac_vi;
#else /* MAC_WMM */
    q_info_t nor;
    q_info_t cf;
#endif /* MAC_WMM */

    q_info_t hp;

} tx_test_config_struct_t;

#ifdef MAC_802_11I

typedef struct
{
    UWORD8 ce_enable;
    UWORD8 ct;
    UWORD8 bcst_ct;
    UWORD8 key_orig;
    UWORD8 tkip_rx_mic_key[16]; /* Not user configurable */
    UWORD8 tkip_rx_bcst_mic_key[16]; /* Not user configurable */
} ce_test_config_struct_t;

#endif /* MAC_802_11I */

#ifdef MAC_802_11N

typedef struct
{
    UWORD8  dir;           /* Direction of Block-Ack            */
    UWORD8  tid;           /* TID for BACK session              */
    UWORD8  partial_state; /* 0 - Full state, 1 - Partial state */
    UWORD8  win_size;      /* Receive window size               */
    UWORD16 seq_num;       /* Starting sequence number          */
    UWORD8  addr[6];       /* MAC address of the transmitter    */
    UWORD32 bar_addr;      /* Not user configurable             */
} ba_lut_struct_t;

typedef struct
{
    UWORD8          ba_lut_prog_test;
    UWORD8          ba_enable;
    UWORD8          ba_num_sessions;
    ba_lut_struct_t ba_lut_entry[4];
} qos_test_config_struct_t;

typedef struct
{
    UWORD8 tid;
    UWORD8 maxnum; /* Maximum Number of Packets to Aggregate     */
    UWORD8 minmps; /* Minimum MPDU Start Spacing                 */
    UWORD8 maxraf; /* Maximum Rx A-MPDU Factor                   */
    UWORD8 numbuf; /* Number of Packets to Buffer before Queuing */
} ampdu_params_t;

typedef struct
{
    UWORD8              da_smps_mode;
    UWORD8              tx_htc_enable;
    UWORD8              amsdu_mode;
    UWORD8              ampdu_lut_prog_test;
    UWORD8              ampdu_num_sessions;
    UWORD16             amsdu_maxlen;
    UWORD16             amsdu_maxnum;
    ampdu_params_t ampdu_sess_params[4];
} ht_config_struct_t;

#endif /* MAC_802_11N */

typedef struct
{
    UWORD8 oper_mode;              /* Test S/w operating mode      */

    int_test_config_struct_t intr; /* Interrupt test configuration */
    tsf_test_config_struct_t tsf;  /* TSF test configuration       */
    rx_test_config_struct_t  rx;   /* Rx test configuration        */
    tx_test_config_struct_t  tx;   /* Tx test configuration        */
    phy_test_config_struct_t phy;   /* PHY Test configuration       */

#ifdef MAC_802_11I
    ce_test_config_struct_t  ce;   /* CE (11i) test configuration  */
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    qos_test_config_struct_t qos;  /* BA test configuration */ /* MERGE TBD - consider changing name */
    ht_config_struct_t       ht;   /* HT (11n) test configuration  */
#endif /* MAC_802_11N */

#ifdef TX_ABORT_FEATURE
    UWORD8 sw_trig_tx_abort_test;
#endif /* TX_ABORT_FEATURE */

} test_config_struct_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern test_config_struct_t  g_test_config;
extern test_params_struct_t  g_test_params;
extern UWORD8                g_tx_pattern[MAX_PATTERN_LEN];
extern UWORD16               g_tx_pattern_len;
extern UWORD8                g_disable_rxq_replenish;
extern UWORD8                g_test_id[256];
extern UWORD8                g_enable_frame_len_modification;
extern UWORD8                g_stats_buf[MAX_STATS_BUFFER_LEN];
extern UWORD16               g_sig_qual_avg;
extern UWORD16               g_sig_qual_avg_cnt;
extern UWORD8                g_last_pkt_tssi_value;

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void initialize_mh_test_config(void);
extern void initialize_mh_test_params(void);
extern void parse_vppp_config_frame(UWORD8 *val, UWORD8 len);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function controls FCS corrution of the transmitted frames. This is a */
/* test functionality used in H/w Unit test mode. Set mode to 1 to enable    */
/* corruption. Set to any other value to disable                             */
INLINE void set_fcs_corrupt_mode(UWORD8 mode)
{
    if(1 == mode)
        enable_machw_tx_fcs_corruption();
    else
        disable_machw_tx_fcs_corruption();
}

/* This function sets the number of frames to be transmitted during the test */
/* First the maximum allowable number of frames are initially queued. The    */
/* remaining are queued from the ISR.                                        */
INLINE void set_hut_num_tx_pkts(UWORD32 val)
{
    g_test_config.tx.init_num_pkt = MIN(val, MAX_HUT_NUM_INIT_PKT);

#ifdef MAC_802_11N
    if(g_test_config.ht.ampdu_num_sessions > 0)
    {
        g_test_config.tx.init_num_pkt = MIN(val,
                              2 * g_test_config.ht.ampdu_sess_params[0].numbuf);
    }
#endif /* MAC_802_11N */

#ifndef MAC_WMM
    g_test_config.tx.nor.init_num_pkt   = g_test_config.tx.init_num_pkt;
#else  /* MAC_WMM */
    g_test_config.tx.ac_vo.init_num_pkt = g_test_config.tx.init_num_pkt;
#endif /* MAC_WMM */

    g_test_config.tx.num_pkts = val - g_test_config.tx.init_num_pkt;

}

/* This function returns the number of frames transmitted during the test    */
INLINE UWORD32 get_hut_num_tx_pkts(void)
{
    return (g_test_config.tx.init_num_pkt + g_test_config.tx.num_pkts);
}

/* This function sets the length of the frame transmitted during the test    */
INLINE void set_hut_frame_len(UWORD16 val)
{
#ifndef MAC_WMM
/* Input Sanity Check */
    if(val > 1596)
        return;

    g_test_config.tx.nor.frame_len = val;
#else  /* MAC_WMM */
    /* Input Sanity Check */
    if(val > 7964)
        return;

    g_test_config.tx.ac_vo.frame_len = val;
#endif /* MAC_WMM */
}

/* This function returns the length of the frame transmitted during the test */
INLINE UWORD16 get_hut_frame_len(UWORD16 val)
{
#ifndef MAC_WMM
    return g_test_config.tx.nor.frame_len;
#else  /* MAC_WMM */
    return g_test_config.tx.ac_vo.frame_len;
#endif /* MAC_WMM */
}

/* This function sets the Transmit Packet Format */
INLINE void set_hut_tx_format(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 2)
        return;

    g_test_params.tx_format = val;

#ifdef MAC_802_11N
    if(g_test_params.tx_format == 0) /* Non-HT */
    {
        mset_HighThroughputOptionImplemented(TV_FALSE);
        mset_RIFSMode(TV_FALSE);
        mset_HTOperatingMode(0);
    }
    else /* HT Greenfield or HT Mixed */
    {
        mset_HighThroughputOptionImplemented(TV_TRUE);

        /* Set the S/w 11n operating mode based on the test transmit packet format */
        if(g_test_params.tx_format == 2)
            set_11n_op_type(NTYPE_MIXED); /* HT Mixed */
        else
            set_11n_op_type(NTYPE_HT_ONLY); /* HT Greenfield */
    }
#endif /* MAC_802_11N */
}

/* This function returns the Transmit Packet Format */
INLINE UWORD8 get_hut_tx_format(void)
{
    return g_test_params.tx_format;
}

/* This function sets the Channel Bandwidth */
INLINE void set_hut_bandwidth(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 3)
        return;

    g_test_params.bandwidth = val;
}

/* This function returns the  Channel Bandwidth */
INLINE UWORD8 get_hut_bandwidth(void)
{
    return g_test_params.bandwidth;
}

/* This function sets the Operating band */
INLINE void set_hut_op_band(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 3)
        return;

    g_test_params.op_band = val;
}

/* This function returns the Operating band */
INLINE UWORD8 get_hut_op_band(void)
{
    return g_test_params.op_band;
}

/* This function sets the STBC (difference between NSTS and NSS) */
INLINE void set_hut_stbc(UWORD8 val)
{
    g_test_params.stbc = val;
}

/* This function returns STBC (difference between NSTS and NSS) */
INLINE UWORD8 get_hut_stbc(void)
{
    return g_test_params.stbc;
}

/* This function sets the Number of Extension spatial streams */
INLINE void set_hut_ess(UWORD8 val)
{
    g_test_params.ess = val;
}

/* This function returns Number of Extension spatial streams */
INLINE UWORD8 get_hut_ess(void)
{
    return g_test_params.ess;
}

/* This function sets the Antenna Set */
INLINE void set_hut_antset(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 0xF)
        return;

    g_test_params.antset = val;
}

/* This function returns the Antenna Set */
INLINE UWORD8 get_hut_antset(void)
{
    return g_test_params.antset;
}

/* This function sets the  HT Operating mode */
INLINE void set_hut_op_mode(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 3)
        return;

    g_test_params.ht_operating_mode = val;
#ifdef MAC_802_11N
    mset_HTOperatingMode(g_test_params.ht_operating_mode);
#endif /* MAC_802_11N */
}

/* This function returns the  HT Operating mode */
INLINE UWORD8 get_hut_op_mode(void)
{
    return g_test_params.ht_operating_mode;
}

/* This function sets the RIFS mode */
INLINE void set_hut_rifs_mode(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_params.rifs_mode = val;
#ifdef MAC_802_11N
    if(g_test_params.rifs_mode == 1)
        mset_RIFSMode(TV_TRUE);
    else
        mset_RIFSMode(TV_FALSE);
#endif /* MAC_802_11N */
}

/* This function returns the RIFS mode */
INLINE UWORD8 get_hut_rifs_mode(void)
{
    return g_test_params.rifs_mode;
}

/* This function sets the Smoothing recommended mode */
INLINE void set_hut_smoothing_rec(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_params.smoothing_rec = val;
}

/* This function returns the Smoothing recommended mode */
INLINE UWORD8 get_hut_smoothing_rec(void)
{
    return g_test_params.smoothing_rec;
}

/* This function sets the sounding packet mode */
INLINE void set_hut_sounding_pkt(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_params.sounding_pkt = val;
}

/* This function returns the sounding packet mode */
INLINE UWORD8 get_hut_sounding_pkt(void)
{
    return g_test_params.sounding_pkt;
}

/* This function sets the HT Coding (BCC/LDPC) */
INLINE void set_hut_ht_coding(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_params.ht_coding = val;
}

/* This function returns the HT Coding (BCC/LDPC) */
INLINE UWORD8 get_hut_ht_coding(void)
{
    return g_test_params.ht_coding;
}

/* This function sets the test direction (Tx/Rx). Essentially it enables the */
/* Tx test as Rx is always enabled.                                          */
INLINE void set_hut_test_dir(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_config.tx.tx_test = val;
}

/* This function returns the test direction (Tx/Rx) */
INLINE UWORD8 get_hut_test_dir(void)
{
    return g_test_config.tx.tx_test;
}

/* This function sets the mode of PHY-Test */
INLINE void set_hut_phy_test_mode(UWORD8 val)
{
    /* Input Sanity Check */
    if(val >= NUM_PHY_TEST_MODE)
        return;

    g_test_config.phy.mode = val;
}

/* This function returns the mode of PHY-Test */
INLINE UWORD8 get_hut_phy_test_mode(void)
{
    return g_test_config.phy.mode;
}

/* This function sets upper bound for the range of rates to run the PHY-Test */
INLINE void set_hut_phy_test_rate_hi(UWORD8 val)
{
    g_test_config.phy.rate_hi = val;
}

/* This function gets upper bound for the range of rates to run the PHY-Test */
INLINE UWORD8 get_hut_phy_test_rate_hi(void)
{
    return g_test_config.phy.rate_hi;
}

/* This function sets upper bound for the range of rates to run the PHY-Test */
INLINE void set_hut_phy_test_rate_lo(UWORD8 val)
{
    g_test_config.phy.rate_lo = val;
}

/* This function gets upper bound for the range of rates to run the PHY-Test */
INLINE UWORD8 get_hut_phy_test_rate_lo(void)
{
    return g_test_config.phy.rate_lo;
}

/* This function sets the time for which the test should be run */
INLINE void set_hut_tx_test_time(UWORD32 val)
{
    g_test_config.tx.test_time = val;
    if(g_test_config.tx.test_time > 0)
    {
#ifndef MAC_WMM
    g_test_config.tx.nor.init_num_pkt   = MAX_HUT_NUM_INIT_PKT;
#else  /* MAC_WMM */
    g_test_config.tx.ac_vo.init_num_pkt = MAX_HUT_NUM_INIT_PKT;
#endif /* MAC_WMM */
    }
}

/* This function returns the total time for which the test is run */
INLINE UWORD32 get_hut_tx_test_time(UWORD32 val)
{
    return g_test_config.tx.test_time;
}

/* This function sets the TXOP limit for AC_VO queue */
INLINE void set_hut_txop_limit(UWORD16 val)
{
    g_test_params.ac_vo_txop_limit = val;
#ifdef  MAC_WMM
    if(val != (UWORD16)-1)
    {
        /* Both AP & STA MIB entries of AC_VO are updated since the Mode may */
        /* change after this parameter is set.                               */
        mset_EDCATableTXOPLimit(g_test_params.ac_vo_txop_limit, 3);
        mset_QAPEDCATableTXOPLimit(g_test_params.ac_vo_txop_limit, 3);
    }
#endif /* MAC_WMM */
}

/* This function returns the TXOP limit for AC_VO queue */
INLINE UWORD16 get_hut_txop_limit(void)
{
    if(g_test_params.ac_vo_txop_limit == -1)
        return mget_EDCATableTXOPLimit(3);
    else
        return g_test_params.ac_vo_txop_limit;
}

/* This function updates the Tx-pattern of the generated Tx-frames */
INLINE void set_hut_tx_pattern(UWORD8 *val)
{
    UWORD16 len = 0;

    /* Extract length */
    len = MAKE_WORD16(val[0], val[1]);

    /* The length of the pattern string is upper-bounded */
    g_tx_pattern_len = MIN(len, MAX_PATTERN_LEN);

    /* Update the pattern string */
    memcpy(g_tx_pattern, val + 2, g_tx_pattern_len);
}

/* This function sets capture mode of the rx packets */
INLINE void set_hut_disable_rxq_repl(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_disable_rxq_replenish = val;
}

/* This function returns the capture mode of the rx packets */
INLINE UWORD8 get_hut_disable_rxq_repl(void)
{
    return g_disable_rxq_replenish;
}

INLINE void hut_action_req(UWORD8 *req)
{
    if(req[1] == 7)
    {
#ifdef MAC_802_11N

        /* A-MSDU Setup Action Type */
        if(req[2] == 0)
        {
            g_test_config.ht.amsdu_mode    = req[3];
            g_test_config.ht.amsdu_maxlen  = MAKE_WORD16(req[4], req[5]);
            g_test_config.ht.amsdu_maxnum  = req[6];

            PRINTD("AMSDU Parameters updated:\n\r");
            PRINTD("Mode = %d MaxLen = %d MaxNum = %d\n\r",
                            g_test_config.ht.amsdu_mode,
                            g_test_config.ht.amsdu_maxlen,
                            g_test_config.ht.amsdu_maxnum);
        }
        /* A-MPDU-Setup Action Type */
        else if(req[2] == 2)
        {
            UWORD8 i           = 0;
            UWORD8 num         = req[3];
            UWORD8 *val        = 0;
            ampdu_params_t *le = NULL;

            g_test_config.ht.ampdu_num_sessions = num;

            for(i = 0; i < num; i++)
            {
                le  = &(g_test_config.ht.ampdu_sess_params[i]);
                val = req + 4 + i * 5;

                le->tid     = val[0];
                le->maxnum  = val[1];
                le->maxraf  = val[2];
                le->minmps  = val[3];
                le->numbuf  = val[4];

                /* Update the LUT */
                add_machw_ampdu_lut_entry(i + 1, le->maxnum, le->minmps, le->maxraf);

                PRINTD("Updated A-MPDU Params:\n\r");
                PRINTD("Num = %d TID=%d maxnum=%d maxraf=%d minmps=%d numbuf=%d\n\r",
                    num, le->tid, le->maxnum, le->maxraf, le->minmps, le->numbuf);
            }
        }
#endif /* MAC_802_11N */
    }
    else if(req[1] == 3)
    {
#ifdef MAC_802_11N
        UWORD8 i            = 0;
        UWORD8 num          = req[3];
        UWORD8 *val         = 0;
        ba_lut_struct_t *le = NULL;

        /* ADDBA-Request Action Type */
        if(req[2] == 0)
        {
            g_test_config.qos.ba_num_sessions = num;
            g_test_config.qos.ba_enable = (num > 0)? 1 : 0;

            for(i = 0; i < num; i ++)
            {
                le = &(g_test_config.qos.ba_lut_entry[i]);
                val = req + 4 + i * 12;

                le->tid             = val[0];
                le->dir             = val[1];
                le->seq_num         = MAKE_WORD16(val[2], val[3]);
                le->win_size        = val[4];
                le->addr[0]         = val[5];
                le->addr[1]         = val[6];
                le->addr[2]         = val[7];
                le->addr[3]         = val[8];
                le->addr[4]         = val[9];
                le->addr[5]         = val[10];
                le->partial_state   = val[11];

                PRINTD("TID=%d Dir=%d Seqno=%d WinSize=%d Addr=%x:%x:%x:%x:%x:%x PS=%d",
                le->tid, le->dir, le->seq_num, le->win_size,
                le->addr[0], le->addr[1], le->addr[2], le->addr[3], le->addr[4], le->addr[5],
                le->partial_state);

            }
        }
#endif /* MAC_802_11N */
    }
    else if(req[1] == 0x7F)
    {
        /* Process HUT-Mode Custom Action Frames */
        switch(req[2])
        {
            case 0: /* Variable Per Packet Parameters Configuration */
                /* The first element req[0] holds the length of the entire    */
                /* Action Frame. Length passed to the parsing function should */
                /* be adjusted for the header.                                */
                parse_vppp_config_frame(req+3, req[0]-3);
                break;
            default:
                PRINTD("Err: Undefined Action Frame\n\r");
                break;
        }
    }
}

/* This function updates the H/w AR table with the user supplied entries */
INLINE void set_hut_ar_table(UWORD8 *val)
{
    UWORD8 i   = 0;
    UWORD8 len = 0;

    /* Extract length (cannot be more than UWORD8) */
    len = MIN(val[0], 8);

    /* Zero length AR table switches off H/w Autorating */
    if(len == 0)
    {
        g_test_config.tx.enable_retry_ar = 0;
        return;
    }

    g_test_config.tx.enable_retry_ar = 1;

    /* Update the AR table with the user configured entries */
    for(i = 0; i < len; i++)
        g_test_config.tx.retry_rates[i] = val[i + 2];

    /* If the number of entries supplied by the user is less than 8, then the */
    /* remaining entries are set to the last entry supplied.                  */
    for(i = len; i < 8; i++)
        g_test_config.tx.retry_rates[i] = g_test_config.tx.retry_rates[len - 1];
}

/* This function returns the sounding packet mode */
INLINE UWORD8 get_hut_cip_type(void)
{
    return g_test_config.ce.ct;
}

/* This function sets the HT Coding (BCC/LDPC) */
INLINE void set_hut_cip_type(UWORD8 val)
{
    /* Input Sanity Check */
    switch(val)
    {
    case 3: /* NO_ENCRYP */
    case 0: /* WEP40     */
    case 4: /* WEP104    */
    case 1: /* TKIP      */
    case 2: /* CCMP      */
        g_test_config.ce.ct = val;
    break;

    default:
        return;
    }

    if(g_test_config.ce.ct == NO_ENCRYP)
        g_test_config.ce.ce_enable = 0;
    else
        g_test_config.ce.ce_enable = 1;
}

/* Set/Get Supplicant Password */
INLINE void set_hut_test_id(UWORD8 *val)
{
    strcpy((WORD8 *)g_test_id, (WORD8 *)val);
}

/* This function sets the origin of the CE key */
INLINE void set_hut_key_origin(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_config.ce.key_orig = val;
}

/* This function returns the origin of the CE key */
INLINE UWORD8 get_hut_key_origin(void)
{
    return g_test_config.ce.key_orig;
}

/* This function sets the origin of the CE key */
INLINE void set_hut_bcst_percent(UWORD8 val)
{
    g_test_config.tx.bcst_percent = MIN(val, 100);
}

/* This function returns the origin of the CE key */
INLINE UWORD8 get_hut_bcst_percent(void)
{
    return g_test_config.tx.bcst_percent;
}

/* This function sets the origin of the CE key */
INLINE void set_hut_bcst_ct(UWORD8 val)
{
    /* Input Sanity Check */
    switch(val)
    {
    case 3: /* NO_ENCRYP */
    case 0: /* WEP40     */
    case 4: /* WEP104    */
    case 1: /* TKIP      */
    case 2: /* CCMP      */
        g_test_config.ce.bcst_ct = val;
    break;

    default:
        return;
    }
}

/* This function returns the origin of the CE key */
INLINE UWORD8 get_hut_bcst_ct(void)
{
    return g_test_config.ce.bcst_ct;
}

/* This function sets the signal quality averaging count */
INLINE void set_hut_sig_qual_avg_cnt(UWORD16 val)
{
    if(val > 0)
        g_sig_qual_avg_cnt = val;
}

/* This function returns the signal quality averaging count */
INLINE UWORD16 get_hut_sig_qual_avg_cnt(void)
{
    return g_sig_qual_avg_cnt;
}

/* This function returns the signal quality average */
INLINE UWORD16 get_hut_sig_qual_avg(void)
{
    return g_sig_qual_avg;
}

/* This function returns the signal quality average */
INLINE UWORD16 get_last_pkt_tssi_value(void)
{
    return g_last_pkt_tssi_value;
}


/* This function sets the TSF Test Mode */
INLINE void set_hut_tsf_test_mode(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 3)
        return;

    g_test_config.tsf.tsf_test = val;
}

/* This function returns the TSF Test Mode */
INLINE UWORD8 get_hut_tsf_test_mode(void)
{
    return g_test_config.tsf.tsf_test;
}


/* This function sets capture mode of the rx packets */
INLINE void set_hut_short_slot(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_config.tx.slot = val;
}

/* This function returns the capture mode of the rx packets */
INLINE UWORD8 get_hut_short_slot(void)
{
    return g_test_config.tx.slot;
}

/* This function sets the TXOP Modification Mode during FC-Coexistence */
INLINE void set_hut_fc_txop_mod(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > (BIT0 | BIT1 | BIT2))
        return;

    g_test_params.fc_txop_mod = val;
}

/* This function returns the TXOP Modification Mode during FC-Coexistence */
INLINE UWORD8 get_hut_fc_txop_mod(void)
{
    return g_test_params.fc_txop_mod;
}

/* This function sets the FC-Protection type in MAC H/w */
INLINE void set_hut_fc_prot_type(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_params.fc_prot_type = val;
}

/* This function returns the FC-Protection type in MAC H/w */
INLINE UWORD8 get_hut_fc_prot_type(void)
{
    return g_test_params.fc_prot_type;
}

/* This function sets support for random assertion of Secondary CCA */
/* in PHY-Model.                                                    */
INLINE void set_hut_sec_cca_assert(UWORD8 val)
{
    /* Input Sanity Check */
    if(val > 1)
        return;

    g_test_params.sec_cca_assert = val;

    update_sec_cca_control_phy(val);
}

/* This function returns support for random assertion of Secondary CCA */
/* in PHY-Model.                                                       */
INLINE UWORD8 get_hut_sec_cca_assert(void)
{
    return g_test_params.sec_cca_assert;
}

#endif /* MH_TEST_CONFIG_H */

#endif /* MAC_HW_UNIT_TEST_MODE */
