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
/*  File Name         : mh_test.h                                            */
/*                                                                           */
/*  Description       : This file contains all declarations and functions    */
/*                      related to the H/w unit test.                        */
/*                                                                           */
/*  List of Functions : None                                                 */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

#ifndef MH_TEST_H
#define MH_TEST_H

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "imem_if.h"
#include "mh_test_config.h"
#include "proc_if.h"
#include "qmu_if.h"
#include "transmit.h"
#include "receive.h"
#include "prot_if.h"
#include "mh_test_utils.h"

#ifdef MAC_802_11I
#include "rsna_km.h"
#endif /* MAC_802_11I */

/* ITM-DEBUG */
#include "phy_prot_if.h"

#ifdef TX_ABORT_FEATURE
#ifdef TEST_TX_ABORT_FEATURE
#include "mh_test_txabort.h"
#endif /* TEST_TX_ABORT_FEATURE */
#endif /* TX_ABORT_FEATURE */

/*****************************************************************************/
/* Constants                                                                 */
/*****************************************************************************/

#define MAX_RX_BUFF_COUNT   20
#define INITFCS32           0xFFFFFFFF
#define INIT_TX_SEQNO       1
#define NUM_NONHT_LONG_SHORT_RATES  15
#define NUM_NON_HT_RATES            NUM_NONHT_LONG_SHORT_RATES
#define NUM_HT_RATES        32
#define MIN_TX_BUFFER_LEN   1500
#define TOTAL_RATES         (NUM_NON_HT_RATES + 32)

/* Constants for Variable Per Packet Parameter feature */
#define NUM_VALID_FRAME_FMT          18
#define MAX_VPPP_TX_FRAME_LEN        1500
#define MIN_VPPP_TX_FRAME_LEN        31
#define VARY_PARAM_ROUND_ROBIN       0
#define VARY_PARAM_RANDOM            1
#define VPPP_SHORT_GI_ENABLE_BIT     BIT6

/*****************************************************************************/
/* Structures                                                                */
/*****************************************************************************/

typedef struct
{
    /* PHY data rate structure */
    struct
    {
        UWORD32 dr1_l;  /* Long 1    */
        UWORD32 dr2_l;  /* Long 2    */
        UWORD32 dr5_l;  /* Long 5.5  */
        UWORD32 dr11_l; /* Long 11   */
        UWORD32 dr2_s;  /* Short 2   */
        UWORD32 dr5_s;  /* Short 5.5 */
        UWORD32 dr11_s; /* Short 11  */
        UWORD32 dr6;    /* 6         */
        UWORD32 dr9;    /* 9         */
        UWORD32 dr12;   /* 12        */
        UWORD32 dr18;   /* 18        */
        UWORD32 dr24;   /* 24        */
        UWORD32 dr36;   /* 36        */
        UWORD32 dr48;   /* 48        */
        UWORD32 dr54;   /* 54        */
        UWORD32 drht;   /* HT Rates  */
        UWORD32 dr0;    /* Reserved  */
    } phy_dr;

    /* Receive status structure */
    struct
    {
        UWORD32 irx0;        /* Invalid                     */
        UWORD32 irxf;        /* FCS failure                 */
        UWORD32 irxs;        /* Rx Successful               */
        UWORD32 irxd;        /* Duplicate Detected          */
        UWORD32 irxmf;       /* MIC failed                  */
        UWORD32 irxkf;       /* Key failed                  */
        UWORD32 irxicvf;     /* ICV failed                  */
        UWORD32 irxtkipicvf; /* ICV failed for TKIP         */
        UWORD32 irxtkipmf;   /* MIC failed for TKIP         */
        UWORD32 irxrf;       /* CCMP Replay Failure         */
        UWORD32 irxtkiprf;   /* TKIP Replay Failure         */
        UWORD32 irxdip;      /* Defragmentation in Progress */
        UWORD32 irxdf;       /* Defragmentation Failure     */
        UWORD32 irxEX;       /* Exception - Reserved value  */
    } st;

    /* Receive cipher type structure */
    struct
    {
        UWORD32 wep40;    /* WEP-40 encrypted  */
        UWORD32 wep104;   /* WEP-104 encrypted */
        UWORD32 tkip;     /* TKIP encrypted    */
        UWORD32 ccmp;     /* CCMP encrypted    */
        UWORD32 unencryp; /* Not encrypted     */
        UWORD32 ukencryp; /* Unknown encryption */
    } cip_type;

    /* MAC data rate structure */
    struct
    {
        UWORD32 dr1_l[2];  /* Long 1    */
        UWORD32 dr2_l[2];  /* Long 2    */
        UWORD32 dr5_l[2];  /* Long 5.5  */
        UWORD32 dr11_l[2]; /* Long 11   */
        UWORD32 dr2_s[2];  /* Short 2   */
        UWORD32 dr5_s[2];  /* Short 5.5 */
        UWORD32 dr11_s[2]; /* Short 11  */
        UWORD32 dr6[2];    /* 6         */
        UWORD32 dr9[2];    /* 9         */
        UWORD32 dr12[2];   /* 12        */
        UWORD32 dr18[2];   /* 18        */
        UWORD32 dr24[2];   /* 24        */
        UWORD32 dr36[2];   /* 36        */
        UWORD32 dr48[2];   /* 48        */
        UWORD32 dr54[2];   /* 54        */
        UWORD32 dr0[2];    /* Unknown   */
        UWORD32 mcs[MAX_MCS_SUPPORTED+1][2];
    } dr;

    /* MAC frame type structure */
    struct
    {
        UWORD32 bcn;            /* Beacon                 */
        UWORD32 rts;            /* RTS                    */
        UWORD32 cts;            /* CTS                    */
        UWORD32 ack;            /* ACK                    */
        UWORD32 probersp;       /* Probe response         */
        UWORD32 probereq;       /* Probe request          */
        UWORD32 atim;           /* ATIM                   */
        UWORD32 cfend;          /* CF-End                 */
        UWORD32 back;           /* Block-Ack              */
        UWORD32 bar;            /* Block-Ack Request      */
        UWORD32 mc_data;        /* Multicast data         */
        UWORD32 uc_data;        /* Unicast data           */
        UWORD32 oth_data;       /* Other data             */
        UWORD32 qos_uc_data;    /* Multicast QoS data     */
        UWORD32 qos_mc_data;    /* Unicast QoS data       */
        UWORD32 qos_oth_data;   /* Other QoS data         */
        UWORD32 amsdu_uc_data;  /* Multicast AMSDU data   */
        UWORD32 amsdu_mc_data;  /* Unicast AMSDU data     */
        UWORD32 amsdu_oth_data; /* Other AMSDU data       */
        UWORD32 oth;            /* Other type, not listed */
    } type;

    /* Frame Format Structure */
    struct
    {
        UWORD32 htgf;
        UWORD32 htmix;
        UWORD32 nht11bl;
        UWORD32 nht11bs;
        UWORD32 nht11a;
        UWORD32 resvd;
        UWORD32 mask20;
        UWORD32 mask40;
        UWORD32 maskresvd;
        UWORD32 bw40;
        UWORD32 bw20u;
        UWORD32 bw20l;
        UWORD32 bw20;
        UWORD32 shortgi;
    } ffmt;

    UWORD32 softrxf;            /* Soft CRC failures                        */
    UWORD32 soft_dupchk;        /* Soft duplicate check failures            */
    UWORD32 soft_intchkf;       /* Soft integrity check failures            */
    UWORD32 num_int_checks;
    UWORD32 amsdu_soft_intchkf; /* A-MSDU soft integrity check failures     */
    UWORD32 defrag_mpdus;       /* Number of defragmented frames received   */
    UWORD32 tkip_mic_diff;      /* Number of TKIP MIC status Differences    */
                                /* between S/w and H/w verifications.       */
    UWORD32 replay_det_diff;    /* Number of Replay Detection status diffs  */
                                /* between S/w and H/w verifications.       */
    UWORD32 ampdu;
    UWORD32 mpdus_in_ampdu;
    UWORD32 bytes_in_ampdu;
    UWORD32 smsdus_in_amsdu;    /* Number of Sub-MSDUs in AMSDU */
    UWORD32 bytes_in_amsdu;     /* Number of bytes in AMSDU     */
    UWORD32 rx_bufsz_exceeded;  /* Exception - Rx buffer size exceeded      */
    UWORD32 unexp_rx_bufend;    /* Exception - Unexpected end of Rx buffers */
    UWORD32 not_first_dscr;     /* Exception : Dscr from Interr. not First Buffer */
    UWORD32 dup_rx_ints;
    UWORD32 invalid_rx_int_cnt;
    WORD32  rssi_accum;
    UWORD32 rssi_pkt_count;
    UWORD32 avg_lna;
    UWORD32 avg_vga;
    UWORD32 agc_trig_stats;
    UWORD32 radar_det_cnt;
#ifdef DEBUG_REGS
    UWORD32 frame_det_11an;
    UWORD32 lp_detect_11an;
    UWORD32 upper_sigfield_pass_11an;
    UWORD32 lower_sigfield_pass_11an;
    UWORD32 forty_mhz_detect_11an;
    UWORD32 ht_signal_detect_11an;
    UWORD32 ht_sigfield_pass_11an;
    UWORD32 service_field_pass_11an;
    UWORD32 data_complete_11an;
    UWORD32 coarse_freq_avg_11an;
    UWORD32 fine_freq_avg_11an;
    UWORD32 freq_avg_11an;

#endif /* DEBUG_REGS */


} test_rxd_t;

typedef struct
{
    UWORD32 txc[16];     /* Tx complete for each queue                */
    UWORD32 bcst;    /* Total number of broadcast frames transmitted */
    UWORD32 data;        /* Total number of data frames transmitted   */
    UWORD32 oth;         /* Total number of non-Data frames txd.      */
    UWORD32 timeout;     /* Total number of Tx-Timeouts.              */
    UWORD32 numtxabrt;   /* Total number of Tx Aborts.                */
    UWORD32 selfctstxab; /* Total number of Self CTS sent on Tx Abort */
} test_txd_t;

typedef struct
{
    UWORD32 enpmsdu;    /* Unexpected Non_pending MPDU */
    UWORD32 erxqemp;    /* Unexpected Rx-Q Empty       */
    UWORD32 ehprxqemp;     /* Unexpected HP-Rx Q Empty    */
    UWORD32 emsaddr;    /* Unexpected MSDU Address     */
    UWORD32 etxsus1machang; /* Unexpected Tx Status 1-MAC  */
    UWORD32 etxsus1phyhang; /* Unexpected Tx Status 1-PHY  */
    UWORD32 ephyhang;       /* Unexpected Tx Status 1      */
    UWORD32 etxsus3;    /* Unexpected Tx Status 3      */
    UWORD32 ebus;       /* Bus Error                   */
    UWORD32 ebwrsig;    /* Bus Wrap Signal Error       */
    UWORD32 etxqempt;   /* Unexpected Tx-Q Empty       */
    UWORD32 edmanoerr;  /* DMA No Error                */
    UWORD32 etxcenr;    /* Tx CE Not Ready             */
    UWORD32 erxcenr;    /* Rx CE Not Ready             */
    UWORD32 esgaf;      /* Seqno Addr Fail             */
    UWORD32 etxfifo;    /* Tx-FIFO Overrun             */
    UWORD32 ehprxfifo;     /* HP-RX FIFO Overrun          */
    UWORD32 erxfifo;    /* Rx-FIFO Overrun             */
    UWORD32 ewdtofcsfailed; /* WDT expiry of FCS failed RX */
    UWORD32 ewdtofcspassed; /* WDT expiry of FCS passed RX */
    UWORD32 eother;     /* Other Error Interrupt       */
    UWORD32 deauth_rc[10]; /* Deauth Reason code          */
} test_exp_t;

typedef struct
{
    UWORD32    tbtti; /* TBTT interrupt        */
    UWORD32    hprxci;    /* Hi-Priority RX interrupt         */
    UWORD32    rxci;  /* Rx Complete interrupt */
    UWORD32    txci;  /* Tx Complete interrupt */
    UWORD32    deauthi;   /* Deauthentication frame interrupt */
    UWORD32    txsusi;    /* Tx suspend interrupt             */
    UWORD32    radardeti; /* Radar detect interrupt           */
    UWORD32    capei; /* CAP End interrupt     */
    UWORD32    erri;  /* Error interrupt       */
    UWORD32    uki;   /* Unknown interrupt     */
    test_rxd_t rxd;   /* Rx interrupt details  */
    test_txd_t txd;   /* Tx interrupt details  */
    test_exp_t exp;   /* Exception details     */
} test_stats_struct_t;

/* Tuple cache structure contains the cache entires for duplicate detection  */
/* and the next entry in the array. It is used as a static link-list.        */
typedef struct
{
    UWORD8  addr[6];    /* Receive Address */
    UWORD8  frag_num;   /* Fragment number */
    UWORD16 seq_num;    /* Sequence number */
    BOOL_T  in_use;     /* Entry valid     */
} tuple_cache_t;

/* ITM-DEBUG */
typedef struct
{
    UWORD32 last_seqno; /* Last sucessfully received Sequence Number */
    UWORD32 temp_cnt;   /* Variable to temporarily maintain some counts */
    UWORD32 numtx_st;   /* Total num of frames txd in Short Time Interval(STI)*/
    UWORD32 numrx_st;   /* Total num of frames rxd in STI.           */
    UWORD32 numsuc_st;  /* Total num of FCS passed frames rxd in STI */
    UWORD32 numtx_lt;   /* Total num of frames txd in Long Time Interval(LTI)*/
    UWORD32 numrx_lt;   /* Total num of frames rxd in LTI.           */
    UWORD32 numsuc_lt;  /* Total num of FCS passed frames rxd in LTI */
} rx_frame_stats_t;

typedef struct
{
    UWORD8 phy_rate; /* Indicates rate as required by PHY */
    UWORD8 mbps;     /* Corresponding rate for interfaces */
    UWORD8 preamble; /* 1:Long 0:Short preamble           */
} phy_test_rate_t;

/*****************************************************************************/
/* Extern Variable Declarations                                              */
/*****************************************************************************/

extern UWORD8               g_rx_buff_index;
extern UWORD8               g_cap_start;
extern UWORD8               g_phy_test_idx_hi;
extern UWORD8               g_phy_test_idx_lo;
extern UWORD8               g_phy_test_idx;
extern UWORD8               g_last_rx_int_seqno;
extern UWORD32              g_next_bcn_addr;
extern UWORD32              g_total_tx_time;
extern UWORD32              g_tx_start_time_lo;
extern UWORD32              g_tx_start_time_hi;
extern UWORD32              g_tx_end_time_lo;
extern UWORD32              g_tx_end_time_hi;
extern UWORD32              g_num_nonht_rates;
extern BOOL_T               g_test_start;
extern phy_test_rate_t      *g_tx_rate_table;
extern test_stats_struct_t  g_test_stats;
extern test_params_struct_t g_test_params;
extern rx_frame_stats_t     g_frm_stats[TOTAL_RATES];
extern rx_frame_stats_t     g_bcn_frm_stats[TOTAL_RATES];
extern rx_frame_stats_t     g_dat_frm_stats[TOTAL_RATES];
extern UWORD32              g_tx_frm_stats[TOTAL_RATES];
extern UWORD32              g_tx_seqno_table[TOTAL_RATES];
extern UWORD32              g_rxptr_dump[20];
extern UWORD32              g_num_frames_txqd;
#ifdef MAC_802_11I
extern UWORD8 g_pn_seqno[16][6];
extern UWORD8 g_pn_bcst_seqno[6];
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
extern q_head_t g_ampdu_buff_q[4];
extern UWORD8   g_ampdu_buff_num[4];
#endif /* MAC_802_11N */

extern phy_test_rate_t g_total_non_ht_gmixed_rates[NUM_NONHT_LONG_SHORT_RATES];
extern phy_test_rate_t g_total_non_ht_gonly_rates[NUM_DR_PHY_802_11G_ONLY];

/*****************************************************************************/
/* Extern Function Declarations                                              */
/*****************************************************************************/

extern void    start_test(void);
extern void    do_register_tests(void);
extern void    init_rx_buff(void);
extern void configure_mac(void);
extern void create_testmac_interrupts(void);
extern void initialize_testsw(mac_struct_t *mac);
extern void queue_packet(q_info_t *temp);
extern void alloc_set_dscr(void);
extern UWORD16 prepare_tx_frame(UWORD8 *mac_hdr, UWORD8 q_num, BOOL_T is_bcst);
extern UWORD16 prepare_beacon(UWORD8 *beacon, UWORD8 flag);
extern UWORD32 compute_crc32(UWORD32 fcs, UWORD8* data, UWORD16 len);

#ifdef MAC_802_11I
extern void check_lut_prog(void);
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
extern void check_ba_lut_prog(void);
extern void check_ampdu_lut_prog(void);
#endif /* MAC_802_11N */

extern UWORD8 check_and_send_ctrl_frames(q_info_t *qinfo);

#ifdef MAC_802_11I
extern void initialize_ce(void);
#endif /* MAC_802_11I */

extern void init_vppp_struct(void);
extern void update_vppp_txfmt_table(UWORD8 frmfmt, UWORD8 bwofst);
extern void update_vpp_params(q_info_t *q_info);
extern UWORD32 get_next_vppp_value(var_param_t *param);
extern void update_operating_channel_hut(void);

/*****************************************************************************/
/* Inline Functions                                                          */
/*****************************************************************************/

/* This function returns the queue information structure to be used for      */
/* queuing packets at the initialization stage.                              */
INLINE q_info_t *get_init_qinfo_struct(void)
{
    q_info_t *temp = 0;

#ifndef MAC_WMM
    if(g_test_config.tx.hp.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.hp);
        g_test_config.tx.hp.init_num_pkt--;
    }
    else if(g_test_config.tx.nor.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.nor);
        g_test_config.tx.nor.init_num_pkt--;
    }
    else if(g_test_config.tx.cf.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.cf);
        g_test_config.tx.cf.init_num_pkt--;
    }
#else /* MAC_WMM */

    UWORD8 stat = 0;

    /* Add frames per queue */
    if(g_test_config.tx.ac_be.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.ac_be);
        g_test_config.tx.ac_be.init_num_pkt--;
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_1 & 0x0F);
    }
    else if(g_test_config.tx.ac_bk.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.ac_bk);
        g_test_config.tx.ac_bk.init_num_pkt--;
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_0 & 0x0F);
    }
    else if(g_test_config.tx.ac_vo.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.ac_vo);
        g_test_config.tx.ac_vo.init_num_pkt--;
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_3 & 0x0F);
    }
    else if(g_test_config.tx.ac_vi.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.ac_vi);
        g_test_config.tx.ac_vi.init_num_pkt--;
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_2 & 0x0F);
    }
    else if(g_test_config.tx.hp.init_num_pkt > 0)
    {
        temp = &(g_test_config.tx.hp);
        g_test_config.tx.hp.init_num_pkt--;
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_3 & 0x0F);
    }

    set_tx_dscr_status((UWORD32 *)g_test_config.tx.tx_dscr_addr, stat);
#endif /* MAC_WMM */

    return temp;
}

/* This function returns the queue information structure to be used for      */
/* queuing packets at the Tx complete interrupt stage.                       */
INLINE q_info_t *get_txc_qinfo_struct(UWORD8 q_num)
{
    q_info_t *temp = 0;
#ifdef MAC_WMM
    UWORD8   stat = 0;
#endif /* MAC_WMM */

#ifdef MAC_WMM
    switch(q_num)
    {
    case HIGH_PRI_Q:
    {
        temp = &(g_test_config.tx.hp);
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_3 & 0x0F);
    }
    break;

    case AC_BK_Q:
    {
        temp = &(g_test_config.tx.ac_bk);
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_0 & 0x0F);
    }
    break;

    case AC_BE_Q:
    {
        temp = &(g_test_config.tx.ac_be);
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_1 & 0x0F);
    }
    break;

    case AC_VI_Q:
    {
        temp = &(g_test_config.tx.ac_vi);
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_2 & 0x0F);
    }
    break;

    case AC_VO_Q:
    {
        temp = &(g_test_config.tx.ac_vo);
        stat = ((PENDING << 5) & 0xE0) | BIT4 | (PRIORITY_3 & 0x0F);
    }
    break;
        }
    set_tx_dscr_status((UWORD32 *)g_test_config.tx.tx_dscr_addr, stat);
#else /* MAC_WMM */
    switch(q_num)
    {
    case HIGH_PRI_Q:
    {
        temp = &(g_test_config.tx.hp);
    }
    break;
    case NORMAL_PRI_Q:
    {
        temp = &(g_test_config.tx.nor);
    }
    break;
    case CF_PRI_Q:
    {
        temp = &(g_test_config.tx.cf);
    }
    break;
    }
#endif /* MAC_WMM */

    return temp;
}

/* This function maps Q-Number to TID */
INLINE UWORD8 test_qnum_to_tid(UWORD8 q_num)
{
    UWORD8 tid = 0;

    switch(q_num)
    {
    case HIGH_PRI_Q:
    {
        tid = PRIORITY_3;
    }
    break;

    case AC_VO_Q:
    {
        tid = PRIORITY_3;
    }
    break;

    case AC_VI_Q:
    {
        tid = PRIORITY_2;
    }
    break;

    case AC_BE_Q:
    {
        tid = PRIORITY_1;
    }
    break;

    case AC_BK_Q:
    {
        tid = PRIORITY_0;
    }
    break;
    }

    return tid;
}

/* This function updates the PHY data rate statistics at Rx Complete ISR */
INLINE void update_phy_dr_stats(void)
{
    UWORD8 phy_reg_val = get_phy_dr();

    switch(phy_reg_val)
    {
        case 11:/* 6 */
            g_test_stats.rxd.phy_dr.dr6 ++;
            break;
        case 15:/* 9 */
            g_test_stats.rxd.phy_dr.dr9 ++;
            break;
        case 10:/* 12 */
            g_test_stats.rxd.phy_dr.dr12 ++;
            break;
        case 14:/* 18 */
            g_test_stats.rxd.phy_dr.dr18 ++;
            break;
        case 9: /* 24 */
            g_test_stats.rxd.phy_dr.dr24 ++;
            break;
        case 13:/* 36 */
            g_test_stats.rxd.phy_dr.dr36 ++;
            break;
        case 8: /* 48 */
            g_test_stats.rxd.phy_dr.dr48 ++;
            break;
        case 12:/* 54 */
            g_test_stats.rxd.phy_dr.dr54 ++;
            break;
        case 1:/* 2-Short */
            g_test_stats.rxd.phy_dr.dr2_s++;
            break;
        case 2:/* 5.5-Short */
            g_test_stats.rxd.phy_dr.dr5_s++;
            break;
        case 3:/* 11-Short */
            g_test_stats.rxd.phy_dr.dr11_s++;
            break;
        case 4:/* 1-Long */
            g_test_stats.rxd.phy_dr.dr1_l++;
            break;
        case 5:/* 2-Long */
            g_test_stats.rxd.phy_dr.dr2_l++;
            break;
        case 6:/* 5.5 Long */
            g_test_stats.rxd.phy_dr.dr5_l++;
            break;
        case 7:/* 11 Long */
            g_test_stats.rxd.phy_dr.dr11_l++;
            break;
        case 128:
        case 129:
        case 130:
        case 131:
        case 132:
        case 133:
        case 134:
        case 135:
        case 136:
            g_test_stats.rxd.phy_dr.drht++;
            break;
        default:
            g_test_stats.rxd.phy_dr.dr0++;
            break;
    }
}

/* This function returns the index within the frame statistics table which */
/* corresponds to the required rate.                                       */
INLINE UWORD8 get_frm_stat_rx_idx(UWORD32 rate)
{
    UWORD8 idx = 0;

    /* Check whether the rate is a MCS */
    if(IS_RATE_MCS(rate) == BTRUE)
        return (GET_MCS(rate) + NUM_NON_HT_RATES);

    /* Search through the supported rate table for legacy rates */
    for(idx = 0; idx < NUM_NON_HT_RATES; idx++)
        if(rate == g_total_non_ht_gmixed_rates[idx].phy_rate)
            return idx;

    return 255;
}

/* This function updates the MAC statistics at Tx Complete ISR */
INLINE void update_mac_tx_stats(UWORD8 dr)
{
    UWORD8 idx = 0;

    idx = get_frm_stat_rx_idx(dr);
    if(255 != idx)
    {
        g_tx_frm_stats[idx]++;
    }
}

/* This function updates the MAC statistics at Rx Complete ISR */
INLINE void update_mac_rx_stats(UWORD8 *msa, UWORD8 status, CIPHER_T ct,
                                UWORD8 dr, UWORD16 frame_type, BOOL_T is_amsdu,
                                BOOL_T is_defrag)
{
    UWORD8 da[6] = {0};

    get_DA(msa, da);

    if(get_wep(msa))
    {
        switch(ct)
        {
        case WEP40:
            g_test_stats.rxd.cip_type.wep40++;
            break;
        case WEP104:
            g_test_stats.rxd.cip_type.wep104++;
            break;
#ifdef MAC_802_11I
        case TKIP:
            g_test_stats.rxd.cip_type.tkip++;
            break;
        case CCMP:
            g_test_stats.rxd.cip_type.ccmp++;
            break;
#endif /* MAC_802_11I */
        default:
            g_test_stats.rxd.cip_type.ukencryp++;
            break;
        }
    }
    else
    {
        g_test_stats.rxd.cip_type.unencryp++;
        ct = NO_ENCRYP;
    }

    switch(status)
    {
    case RX_NEW:
        g_test_stats.rxd.st.irx0++;
        break;
    case RX_SUCCESS: /* Rx Successful */
        g_test_stats.rxd.st.irxs++;
        if(is_group(da) == BTRUE)
            mincr_MulticastReceivedFrameCount();
        else
            mincr_ReceivedFragmentCount();
        led_off(1);
        break;
    case DUP_DETECTED: /* Duplicate Detected */
        g_test_stats.rxd.st.irxd++;
        mset_FrameDuplicateCount(mget_FrameDuplicateCount() + 1);
        break;
    case FCS_ERROR: /* FCS Fail */
        g_test_stats.rxd.st.irxf++;
        mset_FCSErrorCount(mget_FCSErrorCount() + 1);
        led_on(1);
        break;
    case KEY_SEARCH_FAILURE: /* Key search failed */
        g_test_stats.rxd.st.irxkf++;
        break;
#ifdef MAC_802_11I
    case MIC_FAILURE: /* MIC failure */
            g_test_stats.rxd.st.irxmf++;
        break;
    case ICV_FAILURE: /* ICV Failure */
        if(ct == TKIP)
            g_test_stats.rxd.st.irxtkipicvf++;
        else
            g_test_stats.rxd.st.irxicvf++;
        break;
    case TKIP_REPLAY_FAILURE: /* TKIP Replay Failure */
        g_test_stats.rxd.st.irxtkiprf++;
        break;
    case CCMP_REPLAY_FAILURE: /* CCMP Replay Failure */
        g_test_stats.rxd.st.irxrf++;
        break;
    case TKIP_MIC_FAILURE: /* TKIP MIC failure */
        g_test_stats.rxd.st.irxtkipmf++;
        break;
#endif /* MAC_802_11I */
    case DEFRAG_IN_PROGRESS: /* Deframentation in Progress */
        g_test_stats.rxd.st.irxdip++;
        break;
    case DEFRAG_ERROR: /*  Defragmenation Failure */
        g_test_stats.rxd.st.irxdf++;
        break;
    default: /* all other cases */
        g_test_stats.rxd.st.irxEX++;
        break;
    }

    switch(dr)
    {
    case 0: /* Reserved */
        g_test_stats.rxd.dr.dr0[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr0[1]++;
        break;
    case 1: /* 2Mbps Short */
        g_test_stats.rxd.dr.dr2_s[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr2_s[1]++;
        break;
    case 2: /* 5.5Mbps Short */
        g_test_stats.rxd.dr.dr5_s[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr5_s[1]++;
        break;
    case 3: /* 11Mbps Short */
        g_test_stats.rxd.dr.dr11_s[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr11_s[1]++;
        break;
    case 4: /* 1Mbps Long */
        g_test_stats.rxd.dr.dr1_l[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr1_l[1]++;
        break;
    case 5: /* 2Mbps Long */
        g_test_stats.rxd.dr.dr2_l[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr2_l[1]++;
        break;
    case 6: /* 5.5Mbps Long */
        g_test_stats.rxd.dr.dr5_l[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr5_l[1]++;
        break;
    case 7: /* 11Mbps Long */
        g_test_stats.rxd.dr.dr11_l[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr11_l[1]++;
        break;
    case 8: /* 48Mbps */
        g_test_stats.rxd.dr.dr48[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr48[1]++;
        break;
    case 9: /* 24Mbps */
        g_test_stats.rxd.dr.dr24[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr24[1]++;
        break;
    case 10: /* 12Mbps */
        g_test_stats.rxd.dr.dr12[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr12[1]++;
        break;
    case 11: /* 6Mbps */
        g_test_stats.rxd.dr.dr6[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr6[1]++;
        break;
    case 12: /* 54Mbps */
        g_test_stats.rxd.dr.dr54[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr54[1]++;
        break;
    case 13: /* 36Mbps */
        g_test_stats.rxd.dr.dr36[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr36[1]++;
        break;
    case 14: /* 18Mbps */
        g_test_stats.rxd.dr.dr18[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr18[1]++;
        break;
    case 15: /* 9Mbps */
        g_test_stats.rxd.dr.dr9[0]++;
        if(status == 1)
            g_test_stats.rxd.dr.dr9[1]++;
        break;
    default: /* Could be MCS rate */
        if(IS_RATE_MCS(dr))
        {
            UWORD8 num = GET_MCS(dr);

            if(num <= MAX_MCS_SUPPORTED)
            {
                g_test_stats.rxd.dr.mcs[num][0]++;
        if(status == 1)
                    g_test_stats.rxd.dr.mcs[num][1]++;
            }
        }
        break;
    }

    if(frame_type == BEACON)
    {
        UWORD8 idx = 0;

        g_test_stats.rxd.type.bcn++;

        /* Update beacon data rate statistics */
        idx = get_frm_stat_rx_idx(dr);
        if(255 != idx)
        {
            g_bcn_frm_stats[idx].numrx_lt++;
            if(status == 1)
                g_bcn_frm_stats[idx].numsuc_lt++;
        }
    }
    else if(frame_type == RTS)
    {
        g_test_stats.rxd.type.rts++;
    }
    else if(frame_type == CTS)
    {
        g_test_stats.rxd.type.cts++;
    }
    else if(frame_type == ACK)
    {
        g_test_stats.rxd.type.ack++;
    }
    else if(frame_type == PROBE_RSP)
    {
        g_test_stats.rxd.type.probersp++;
    }
    else if(frame_type == PROBE_REQ)
    {
        g_test_stats.rxd.type.probereq++;
    }
    else if(frame_type == ATIM)
    {
        g_test_stats.rxd.type.atim++;
    }
    else if(frame_type == CFEND)
    {
        g_test_stats.rxd.type.cfend++;
    }
    else if(frame_type == BLOCKACK)
    {
        g_test_stats.rxd.type.back++;
    }
    else if(frame_type == BLOCKACK_REQ)
    {
        g_test_stats.rxd.type.bar++;
    }
    else if(frame_type == DATA)
    {
        UWORD8 idx = 0;

        if(is_group(da) == BTRUE)
            g_test_stats.rxd.type.mc_data++;
        else if(mac_addr_cmp(da, mget_StationID()) == BTRUE)
            g_test_stats.rxd.type.uc_data++;
        else
            g_test_stats.rxd.type.oth_data++;

        /* Update data type data rate statistics */
        idx = get_frm_stat_rx_idx(dr);
        if(255 != idx)
        {
            g_dat_frm_stats[idx].numrx_lt++;
            if(status == 1)
                g_dat_frm_stats[idx].numsuc_lt++;
        }
    }
#ifdef MAC_WMM
    else if(frame_type == QOS_DATA)
    {
        UWORD8 idx = 0;

        if(is_group(da) == BTRUE)
        {
            g_test_stats.rxd.type.qos_mc_data++;
#ifdef MAC_802_11N
            if(BTRUE == is_amsdu)
                g_test_stats.rxd.type.amsdu_mc_data++;
#endif /* MAC_802_11N */
        }
        else if(mac_addr_cmp(da, mget_StationID()) == BTRUE)
        {
            g_test_stats.rxd.type.qos_uc_data++;
#ifdef MAC_802_11N
            if(BTRUE == is_amsdu)
                g_test_stats.rxd.type.amsdu_uc_data++;
#endif /* MAC_802_11N */
        }
        else
        {
            g_test_stats.rxd.type.qos_oth_data++;
#ifdef MAC_802_11N
            if(BTRUE == is_amsdu)
                g_test_stats.rxd.type.amsdu_oth_data++;
#endif /* MAC_802_11N */
        }

        /* Update qos data type data rate statistics */
        idx = get_frm_stat_rx_idx(dr);
        if(255 != idx)
        {
            g_dat_frm_stats[idx].numrx_lt++;
            if(status == 1)
                g_dat_frm_stats[idx].numsuc_lt++;
        }
    }
#endif /* MAC_WMM */
    else
    {
        g_test_stats.rxd.type.oth++;
    }

    /* Update the count of H/w defragmented frames received */
    if(is_defrag == BTRUE)
        g_test_stats.rxd.defrag_mpdus++;
}

/* This function can be used to filter out frames while collecting reception */
/* statistics                                                                */
INLINE BOOL_T filter_test_frame(UWORD8 *msa)
{
    if(msa != NULL)
    {
        UWORD8 rxd_sa[6] = {0};
        UWORD8 rqd_sa[6] = {0};

        /* FILTER-1: Only Data frames are passed */
        if(get_type(msa) != DATA_BASICTYPE)
            return BFALSE;

        /* FILTER-2: Only frames transmitted by the required station whose */
        /* address is given in g_test_config.tx.da are passed.             */
    mac_addr_cpy(rqd_sa, g_test_config.tx.da);
    get_SA(msa, rxd_sa);
    if(mac_addr_cmp(rqd_sa, rxd_sa) == BTRUE)
        return BTRUE;
    }

    return BFALSE;
}

/* This function verifies the TKIP MIC status indicated by H/w. Only frames */
/* received with TKIP Cipher Type should be passed to this function.        */
INLINE void verify_tkip_mic(UWORD8 *msa, UWORD8 status, UWORD16 frame_type,
                            UWORD16 pkt_len, UWORD8 *sa, UWORD8 *da)
{
#ifdef MAC_802_11I
    UWORD8        mac_hdr_len = 0;
    UWORD8        rx_mic[8]   = {0};
    UWORD8        cal_mic[8]  = {0};
    UWORD8        *mickey     = NULL;
    buffer_desc_t buff_desc   = {0};

#ifdef MAC_WMM
    if((frame_type != DATA) && (frame_type != QOS_DATA))
#else /* MAC_WMM */
    if(frame_type != DATA)
#endif /* MAC_WMM */
    {
        return;
    }

    if((status == RX_SUCCESS) || (status == TKIP_MIC_FAILURE))
    {
        mac_hdr_len = get_mac_hdr_len(msa);

        /* Copy the received mic to the local buffer */
        {
            UWORD8 *mic_ofst = msa + pkt_len - 16;
#ifdef MAC_WMM
            mic_ofst += 2;
#endif /* MAC_WMM */
            memcpy(rx_mic, mic_ofst, 8);
        }

        buff_desc.buff_hdl    = msa + 8 + mac_hdr_len;
#ifdef MAC_WMM
        buff_desc.buff_hdl += 2;
#endif /* MAC_WMM */
        buff_desc.data_offset = 0;
        buff_desc.data_length = pkt_len - mac_hdr_len - FCS_LEN - 20;

        /* Compute the mic using the received data */
        if(is_group(da) == BTRUE)
        {
            mickey = g_test_config.ce.tkip_rx_bcst_mic_key;
        }
        else
        {
            mickey = g_test_config.ce.tkip_rx_mic_key;

            /* For Unicast rxd frames, if the STA is the authenticator */
            /* then use the supplicant TKIP-MicKey                     */
            if(g_test_config.ce.key_orig == AUTH_KEY)
                mickey += 8;
        }


        compute_tkip_mic(da, sa, 0, &buff_desc, 1, buff_desc.data_length,
                         mickey, cal_mic);

        /* Compare the received mic and the computed mic */
        if(memcmp(cal_mic, rx_mic, 8) != 0)
{
            /* H/w says success and S/w says failure!! */
            if(status != TKIP_MIC_FAILURE)
                g_test_stats.rxd.tkip_mic_diff++;
        }
        else
    {
            /* H/w says failure and S/w says success!! */
            if((status != RX_SUCCESS))
                g_test_stats.rxd.tkip_mic_diff++;
    }
    }
#endif /* MAC_802_11I */
}

/* This function performs S/w duplicate detection check */
INLINE void soft_dup_check(UWORD8 *msa, UWORD8 *sa)
{
    if(1 == g_test_config.rx.soft_dupchk_enable)
    {
        UWORD8  ret        = 0;
        UWORD16 seq_num    = 0;
        UWORD8  frag_num   = 0;
        BOOL_T  dup_status = BFALSE;

        /* Extract the Source addess, sequence number and retry bit */
        seq_num  = get_sequence_number(msa);
        frag_num = get_fragment_number(msa);

        /* Duplicate detection is done for the received frame. The duplicate */
        /* status is updated with the status returned by the search. If no   */
        /* duplicate is found, the tuple cache is updated with the new entry.*/
        dup_status = search_and_update_tuple_cache(sa, seq_num, frag_num, ret);

        if(dup_status == BTRUE)
            g_test_stats.rxd.soft_dupchk++;
    }
}

/* This function performs S/w CRC-32 check */
INLINE void soft_crc_check(UWORD8 *msa, UWORD16 pkt_len)
{
    if(g_test_config.rx.soft_crc_enable == 1)
    {
        UWORD32 rx_crc   = 0;
        UWORD32 soft_crc = 0;

        soft_crc = compute_crc32(INITFCS32, msa, (pkt_len - 4));
        soft_crc = soft_crc ^ 0xFFFFFFFF;

        rx_crc = (msa[pkt_len - 1] << 24) | (msa[pkt_len - 2] << 16) |
                 (msa[pkt_len - 3] << 8)  | (msa[pkt_len - 4]);

        if(soft_crc != rx_crc)
            g_test_stats.rxd.softrxf++;
    }
}

#ifdef MAC_802_11I
/* This function performs 11i security related integrity check on the frames */
INLINE void perform_security_check(UWORD8 *msa, UWORD16 pkt_len, UWORD8 status,
                                   CIPHER_T ct)
{
    /* Security Checks performed only on selected frames. */
    if(BTRUE == filter_test_frame(msa))
        return;

    /* TKIP MIC Verification done here.                                      */
    /* This does not work for A-MSDU frames. This is okay since TKIP is not  */
    /* supposed to be used for 11n frames.                                   */
    if((g_test_config.rx.tkip_mic_check == 1) && (ct == TKIP))
    {
        UWORD8  sa[6]      = {0};
        UWORD8  da[6]      = {0};
        UWORD16 frame_type = 0;

        get_address1(msa, da);
        get_address2(msa, sa);

        frame_type = get_sub_type(msa);

        verify_tkip_mic(msa, status, frame_type, pkt_len, sa, da);
    }

    /* Replay detection done here. Only check frames which are touched by    */
    /* the replay detection logic                                            */
    if((g_test_config.rx.replay_detect == 1) &&
       ((ct == TKIP) || (ct == CCMP)) &&
       ((status == RX_SUCCESS) ||(status == TKIP_REPLAY_FAILURE) ||
        (status == CCMP_REPLAY_FAILURE)))
    {
        UWORD8 mac_hdr_len  = get_mac_hdr_len(msa);
        UWORD8 tid          = get_priority_value(msa);
        BOOL_T status       = BFALSE;
        UWORD8 rx_pn_val[6] = {0};
        UWORD8 da[6]        = {0};

        /* Extract the destination address from the frame */
        get_DA(msa, da);

#ifdef MAC_WMM
        mac_hdr_len += 2;
#endif /* MAC_WMM */
        get_pn_val(msa + mac_hdr_len, (UWORD8)ct, rx_pn_val);

        if(is_group(da) == BFALSE)
        {
            status = cmp_pn_val(g_pn_seqno[tid], rx_pn_val);
        }
        else
        {
            status = cmp_pn_val(g_pn_bcst_seqno, rx_pn_val);
        }

        if(status == BFALSE)
        {
            /* Replayed Frame */
            if(((ct == TKIP) && (status != TKIP_REPLAY_FAILURE)) ||
               ((ct == CCMP) && (status != CCMP_REPLAY_FAILURE)))
            {
                g_test_stats.rxd.replay_det_diff++;
            }
        }
        else
        {
            /* Clean, New Frame */
            if(status != RX_SUCCESS)
            {
                g_test_stats.rxd.replay_det_diff++;
            }
        }
    }
}
#endif /* MAC_802_11I*/

/* This function checks integrity of body of received  frames/sub-frames */
INLINE BOOL_T check_frame_integrity(UWORD8 *msdu, UWORD16 len)
{
    UWORD16 idx   = 0;
    UWORD16 p_idx = 0;

    if(g_test_config.rx.soft_integrity_check == 1)
    {
         g_test_stats.rxd.num_int_checks++;
    for(idx = 0; idx < len; idx++)
    {
        if(g_tx_pattern[p_idx++] != msdu[idx])
            return BFALSE;

        if(p_idx >= g_tx_pattern_len)
            p_idx = 0;
    }

    }

    return BTRUE;
}

/* This function returns the length of the MAC header depending upon the     */
/* protocol options enabled                                                  */
INLINE UWORD8 get_test_mac_hdr_len(void)
{
    UWORD8 mac_hdr_len = MAC_HDR_LEN;

#ifdef MAC_802_11N
    if(g_test_config.ht.tx_htc_enable == 1)
    {
        mac_hdr_len += HT_CTRL_FIELD_LEN;
    }
#endif /* MAC_802_11N */

#ifdef MAC_WMM
    mac_hdr_len += QOS_CTRL_FIELD_LEN;
#endif /* MAC_WMM */

    return mac_hdr_len;
}

/* This function checkes whether the received frame is part of an AMPDU */
INLINE BOOL_T is_ampdu(UWORD32 *rx_dscr)
{
    if(REGBIT9 == (*((UWORD32 *)rx_dscr + 2) & REGBIT9))
    {
        return BTRUE;
    }

    return BFALSE;
}

/* This function compares time1 with time2 and returns BTRUE if time1 is */
/* higher than time2.                                                    */
INLINE BOOL_T is_time_greater(UWORD32 time1_hi, UWORD32 time1_lo,
                              UWORD32 time2_hi, UWORD32 time2_lo)
{
    if((time1_hi < time2_hi) ||
       ((time1_hi == time2_hi) && (time1_lo < time2_lo)))
    {
        return BFALSE;
    }

    return BTRUE;
}

/* This function checks whether its time to end the test. */
INLINE BOOL_T is_tx_test_complete(void)
{
    /* Time check takes precedence over number check */
    if(g_test_config.tx.test_time != 0)
    {
        UWORD32 tsf_hi = 0;
        UWORD32 tsf_lo = 0;

        /* Get the current time */
        get_machw_tsf_timer(&tsf_hi, &tsf_lo);

        if(is_time_greater(tsf_hi, tsf_lo, g_tx_end_time_hi,
                           g_tx_end_time_lo) == BTRUE)
        {
            return BTRUE;
        }
        return BFALSE;
    }

    if(g_num_frames_txqd >= get_hut_num_tx_pkts())
        return BTRUE;

    return BFALSE;
}

INLINE void int_seqno_dup_chk(UWORD32 *rx_dscr)
{
    UWORD8 int_seqno = 0;

    int_seqno = get_rx_dscr_int_seq_num(rx_dscr);

    if(int_seqno == 0)
    {
        g_test_stats.rxd.invalid_rx_int_cnt++;

        return;
    }

    if((g_last_rx_int_seqno != 0) &&
       (dscr_seqno_lt(g_last_rx_int_seqno, int_seqno) != BTRUE))
    {
        g_test_stats.rxd.dup_rx_ints++;
    }

    g_last_rx_int_seqno = int_seqno;
}

/*****************************************************************************/
/* The following functions are provided for printing debug information in    */
/* MAC H/w unit test mode with Linux CSL.                                    */
/*****************************************************************************/

/* Receive debug log functions */

/* This function prints MAC receive statistics and can be printed at the end */
/* of the test or periodically.                                              */
INLINE void print_rx_log(void)
{
    PRINTK("\n====  Receive statistics log ===\n\r");
    PRINTK("RxC Interrupt    = %d\n\r", g_test_stats.rxci);
    PRINTK("Rx Success       = %d\n\r", g_test_stats.rxd.st.irxs);
    PRINTK("Rx FCS Failure        = %d\n\r", g_test_stats.rxd.st.irxf);
    PRINTK("Rx Duplicate Detected = %d\n\r", g_test_stats.rxd.st.irxd);
    PRINTK("Rx Status Invalid     = %d\n\r", g_test_stats.rxd.st.irx0);
    PRINTK("Rx CCMP MIC Failure   = %d\n\r", g_test_stats.rxd.st.irxmf);
    PRINTK("Rx Key Failure        = %d\n\r", g_test_stats.rxd.st.irxkf);
    PRINTK("Rx ICV Failure        = %d\n\r", g_test_stats.rxd.st.irxicvf);
    PRINTK("Rx TKIP-ICV Failure   = %d\n\r", g_test_stats.rxd.st.irxtkipicvf);
    PRINTK("Rx TKIP-MIC Failure   = %d\n\r", g_test_stats.rxd.st.irxtkipmf);
    PRINTK("Rx TKIP-Replay Failure= %d\n\r", g_test_stats.rxd.st.irxtkiprf);
    PRINTK("Rx CCMP-Replay Failure= %d\n\r", g_test_stats.rxd.st.irxrf);
    PRINTK("Rx Defrag In Progress = %d\n\r", g_test_stats.rxd.st.irxdip);
    PRINTK("Rx Defrag Failure     = %d\n\r", g_test_stats.rxd.st.irxdf);
    PRINTK("Rx Exception-Rsvd     = %d\n\r", g_test_stats.rxd.st.irxEX);
    PRINTK("\n\r");
    PRINTK("Rx# Beacon            = %d\n\r", g_test_stats.rxd.type.bcn);
    PRINTK("Rx# RTS               = %d\n\r", g_test_stats.rxd.type.rts);
    PRINTK("Rx# CTS               = %d\n\r", g_test_stats.rxd.type.cts);
    PRINTK("Rx# ACK               = %d\n\r", g_test_stats.rxd.type.ack);
    PRINTK("Rx# Probe Response    = %d\n\r", g_test_stats.rxd.type.probersp);
    PRINTK("Rx# Probe Request     = %d\n\r", g_test_stats.rxd.type.probereq);
    PRINTK("Rx# ATIM              = %d\n\r", g_test_stats.rxd.type.atim);
    PRINTK("Rx# CF End            = %d\n\r", g_test_stats.rxd.type.cfend);
    PRINTK("Rx# Block-Ack         = %d\n\r", g_test_stats.rxd.type.back);
    PRINTK("Rx# Block-Ack Request = %d\n\r", g_test_stats.rxd.type.bar);
    PRINTK("Rx# Mcst data         = %d\n\r", g_test_stats.rxd.type.mc_data);
    PRINTK("Rx# Ucst data         = %d\n\r", g_test_stats.rxd.type.uc_data);
    PRINTK("Rx# Othr data         = %d\n\r", g_test_stats.rxd.type.oth_data);
    PRINTK("Rx# Mcst QoS data     = %d\n\r", g_test_stats.rxd.type.qos_mc_data);
    PRINTK("Rx# Ucst QoS data     = %d\n\r", g_test_stats.rxd.type.qos_uc_data);
    PRINTK("Rx# Othr QoS data     = %d\n\r", g_test_stats.rxd.type.qos_oth_data);
    PRINTK("Rx# Mcst AMSDU data   = %d\n\r", g_test_stats.rxd.type.amsdu_mc_data);
    PRINTK("Rx# Ucst AMSDU data   = %d\n\r", g_test_stats.rxd.type.amsdu_uc_data);
    PRINTK("Rx# Othr AMSDU data   = %d\n\r", g_test_stats.rxd.type.amsdu_oth_data);
    PRINTK("NumSubMSDUsInAMSDU    = %d\n\r", g_test_stats.rxd.smsdus_in_amsdu);
    PRINTK("NumPayloadBytesInAMSDU= %d\n\r", g_test_stats.rxd.bytes_in_amsdu);
    PRINTK("Rx# Other Types       = %d\n\r", g_test_stats.rxd.type.oth);
    PRINTK("Rx# Defrag Frames     = %d\n\r", g_test_stats.rxd.defrag_mpdus);
    PRINTK("NumAMPDUs             = %d\n\r", g_test_stats.rxd.ampdu);
    PRINTK("NumMPDUsInAMPDU       = %d\n\r", g_test_stats.rxd.mpdus_in_ampdu);
    PRINTK("NumBytesInAMPDU       = %d\n\r", g_test_stats.rxd.bytes_in_ampdu);
    PRINTK("\n\r");
    PRINTK("Rx# WEP-40 Encryp     = %d\n\r", g_test_stats.rxd.cip_type.wep40);
    PRINTK("Rx# WEP-104 Encryp    = %d\n\r", g_test_stats.rxd.cip_type.wep104);
    PRINTK("Rx# TKIP Encryp       = %d\n\r", g_test_stats.rxd.cip_type.tkip);
    PRINTK("Rx# CCMP Encryp       = %d\n\r", g_test_stats.rxd.cip_type.ccmp);
    PRINTK("Rx# UnEncryp          = %d\n\r", g_test_stats.rxd.cip_type.unencryp);
    PRINTK("\n\r");
    PRINTK("Rx# 11b Long  Pream   = %d\n\r", g_test_stats.rxd.ffmt.nht11bl);
    PRINTK("Rx# 11b Short Pream   = %d\n\r", g_test_stats.rxd.ffmt.nht11bs);
    PRINTK("Rx# 11a Format        = %d\n\r", g_test_stats.rxd.ffmt.nht11a);
    PRINTK("Rx# HT-GF Format      = %d\n\r", g_test_stats.rxd.ffmt.htgf);
    PRINTK("Rx# HT-Mixed Format   = %d\n\r", g_test_stats.rxd.ffmt.htmix);
    PRINTK("Rx# Reserved Format   = %d\n\r", g_test_stats.rxd.ffmt.resvd);
    PRINTK("Rx# HT-MASK 20MHz     = %d\n\r", g_test_stats.rxd.ffmt.mask20);
    PRINTK("Rx# HT-MASK 40MHz     = %d\n\r", g_test_stats.rxd.ffmt.mask40);
    PRINTK("Rx# Reserved Mask     = %d\n\r", g_test_stats.rxd.ffmt.maskresvd);
    PRINTK("Rx# HT-BW 40MHz       = %d\n\r", g_test_stats.rxd.ffmt.bw40);
    PRINTK("Rx# HT-BW 20MHz       = %d\n\r", g_test_stats.rxd.ffmt.bw20);
    PRINTK("Rx# HT-BW 20MHz-Upper = %d\n\r", g_test_stats.rxd.ffmt.bw20u);
    PRINTK("Rx# HT-BW 20MHz-Lower = %d\n\r", g_test_stats.rxd.ffmt.bw20l);
    PRINTK("Rx# Short-GI          = %d\n\r", g_test_stats.rxd.ffmt.shortgi);
    PRINTK("\n\r");
    PRINTK("NumIntChks            = %d\n\r", g_test_stats.rxd.num_int_checks);

    PRINTK("SoftIntChkFail        = %d\n\r", g_test_stats.rxd.soft_intchkf +
                                           g_test_stats.rxd.amsdu_soft_intchkf);
    PRINTK("SoftCrcChkFail        = %d\n\r", g_test_stats.rxd.softrxf);
    PRINTK("SoftDupChkFail        = %d\n\r", g_test_stats.rxd.soft_dupchk);
    PRINTK("TkipMicDiff           = %d\n\r", g_test_stats.rxd.tkip_mic_diff);
    PRINTK("ReplayDetDiff         = %d\n\r", g_test_stats.rxd.replay_det_diff);
    PRINTK("RxBufferSizeExceed    = %d\n\r", g_test_stats.rxd.rx_bufsz_exceeded);
    PRINTK("UnexpRxBufEnd         = %d\n\r", g_test_stats.rxd.unexp_rx_bufend);
    PRINTK("RxNotFirstDscrErr     = %d\n\r", g_test_stats.rxd.not_first_dscr);
    PRINTK("DupRxIntrrCnt         = %d\n\r", g_test_stats.rxd.dup_rx_ints);
    PRINTK("InvalidRxIntrrCnt     = %d\n\r", g_test_stats.rxd.invalid_rx_int_cnt);
    PRINTK("================================\n\r");
    }

/* This function prints MAC receive data rate statistics and can be printed  */
/* at the end of the test or periodically.                                   */
INLINE void print_rx_dr_log(void)
{
    UWORD8 i = 0;

    PRINTK("\n=Reception Data Rate Statistics=\n\r");
    PRINTK("  Unknown (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr0[0]);
    PRINTK("  1L Mbps (TotalRx  = %d\n\r", g_test_stats.rxd.dr.dr1_l[0]);
    PRINTK("  1L Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr1_l[1]);
    PRINTK("  2S Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr2_s[0]);
    PRINTK("  2S Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr2_s[1]);
    PRINTK("  2L Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr2_l[0]);
    PRINTK("  2L Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr2_l[1]);
    PRINTK("  5S Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr5_s[0]);
    PRINTK("  5S Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr5_s[1]);
    PRINTK("  5L Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr5_l[0]);
    PRINTK("  5L Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr5_l[1]);
    PRINTK(" 11S Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr11_s[0]);
    PRINTK(" 11S Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr11_s[1]);
    PRINTK(" 11L Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr11_l[0]);
    PRINTK(" 11L Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr11_l[1]);
    PRINTK("   6 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr6[0]);
    PRINTK("   6 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr6[1]);
    PRINTK("   9 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr9[0]);
    PRINTK("   9 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr9[1]);
    PRINTK("  12 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr12[0]);
    PRINTK("  12 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr12[1]);
    PRINTK("  18 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr18[0]);
    PRINTK("  18 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr18[1]);
    PRINTK("  24 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr24[0]);
    PRINTK("  24 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr24[1]);
    PRINTK("  36 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr36[0]);
    PRINTK("  36 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr36[1]);
    PRINTK("  48 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr48[0]);
    PRINTK("  48 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr48[1]);
    PRINTK("  54 Mbps (TotalRx) = %d\n\r", g_test_stats.rxd.dr.dr54[0]);
    PRINTK("  54 Mbps (Success) = %d\n\r", g_test_stats.rxd.dr.dr54[1]);
    for(i = 0; i <= MAX_MCS_SUPPORTED; i++)
    {
        PRINTK(" HT MCS %2d (TotalRx) = %d\n\r", i,
                                                g_test_stats.rxd.dr.mcs[i][0]);
        PRINTK(" HT MCS %2d (Success) = %d\n\r", i,
                                                g_test_stats.rxd.dr.mcs[i][1]);
    }

    PRINTK("================================\n\r");
}

INLINE void print_erri_log(void)
{
    PRINTK("\n====  Error statistics log ===\n\r");
    PRINTK("Total Error Interrupts = %d\n\r",g_test_stats.erri);
    PRINTK("UnExp Non_pending MPDU = %d\n\r",g_test_stats.exp.enpmsdu);
    PRINTK("UnExp Rx Q Empty       = %d\n\r",g_test_stats.exp.erxqemp);
    PRINTK("UnExp HP-Rx Q Empty    = %d\n\r",g_test_stats.exp.ehprxqemp);
    PRINTK("UnExp MSDU Address     = %d\n\r",g_test_stats.exp.emsaddr);
    PRINTK("UnExp Tx Status 1-MAC  = %d\n\r",g_test_stats.exp.etxsus1machang);
    PRINTK("UnExp Tx Status 1-PHY  = %d\n\r",g_test_stats.exp.etxsus1phyhang);
    PRINTK("UnExp Tx Status 3      = %d\n\r",g_test_stats.exp.etxsus3);
    PRINTK("Bus-Error              = %d\n\r",g_test_stats.exp.ebus);
    PRINTK("Bus Wrap Signal Error  = %d\n\r",g_test_stats.exp.ebwrsig);
    PRINTK("UnExp Tx-Q Empty       = %d\n\r",g_test_stats.exp.etxqempt);
    PRINTK("DMA No Error           = %d\n\r",g_test_stats.exp.edmanoerr);
    PRINTK("Tx CE Not Ready        = %d\n\r",g_test_stats.exp.etxcenr);
    PRINTK("Rx CE Not Ready        = %d\n\r",g_test_stats.exp.erxcenr);
    PRINTK("Seqno Addr Fail        = %d\n\r",g_test_stats.exp.esgaf);
    PRINTK("Tx-FIFO Overrun        = %d\n\r",g_test_stats.exp.etxfifo);
    PRINTK("Rx-FIFO Overrun        = %d\n\r",g_test_stats.exp.erxfifo);
    PRINTK("HP-Rx-FIFO Overrun     = %d\n\r",g_test_stats.exp.ehprxfifo);
    PRINTK("WDT Expiry FCS Failed  = %d\n\r",g_test_stats.exp.ewdtofcsfailed);
    PRINTK("WDT Expiry FCS Passed  = %d\n\r",g_test_stats.exp.ewdtofcspassed);
    PRINTK("Other Error Interrupt  = %d\n\r",g_test_stats.exp.eother);
    PRINTK("================================\n\r");
}

INLINE void print_oth_log(void)
{
    PRINTK("\n====  Error statistics log ===\n\r");
    PRINTK("Total TBTT Intrpts     = %d\n\r",g_test_stats.tbtti);
    PRINTK("Total Unknown Intrpts  = %d\n\r",g_test_stats.uki);
    PRINTK("Total CAP End Intrpts  = %d\n\r",g_test_stats.capei);
    PRINTK("================================\n\r");
}

/* This function prints the details of a received frame. This can be printed */
/* for each frame or for some intermediate frames.                           */
/* Suggested use:                                                            */
/* Place : In fn 'rx_complete_isr' before 'update_mac_rx_stats' fn call      */
/* Call  : print_rx_frame_info(pkt_len, datarate, frame_type);               */
INLINE void print_rx_frame_info(UWORD16 len, UWORD16 dr, UWORD16 type)
{
    PRINTK("Rx frame info: Type %x, Length %d, Rate %d\n\r", type, len, dr);
}

/* This function prints the receive descriptor contents. This can be used    */
/* for each frame or for some intermediate frames.                           */
/* Suggested use:                                                            */
/* Place : In fn 'rx_complete_isr' before 'update_mac_rx_stats' fn call      */
/* Call  : print_rx_dscr_contents(rx_dscr);                                  */
INLINE void print_rx_dscr_contents(UWORD32 *rx_dscr)
{
    UWORD8 i = 0;

    PRINTK("Rx descriptor contents (in Little Endian format)\n\r");

    for(i = 0; i < RX_DSCR_NUM_WORDS; i++)
        PRINTK("Word %d = %08x\n\r", i, convert_to_le(rx_dscr[i]));

    PRINTK("\n\r");
}

/* Transmit debug log functions */

/* This function prints MAC transmit statistics and can be printed at end    */
/* of the test or periodically.                                              */
/* Suggested use:                                                            */
/* Place : In fn 'tx_complete_isr' at the end before returning               */
/* Call  : print_tx_log();                                                   */
INLINE void print_tx_log(void)
{
    UWORD16 i = 0;

    PRINTK("\n====  Transmit statistics log ====\n\r");
    PRINTK("TxC Interrupts          = %d\n\r", g_test_stats.txci);
    PRINTK("TBTT Interrupts         = %d\n\r", g_test_stats.tbtti);
    PRINTK("Tx Fragments            = %d\n\r", mget_TransmittedFragmentCount());
    PRINTK("Tx Frames               = %d\n\r", mget_TransmittedFrameCount());
    PRINTK("Tx Bcst Frame           = %d\n\r", g_test_stats.txd.bcst);
    PRINTK("ACK Failures            = %d\n\r", mget_ACKFailureCount());
    PRINTK("Failed Frames           = %d\n\r", mget_FailedCount());
    PRINTK("Retried Frames          = %d\n\r", mget_RetryCount());
    PRINTK("Multiple Retried Frames = %d\n\r", mget_MultipleRetryCount());
    PRINTK("Timed-out frames        = %d\n\r", g_test_stats.txd.timeout);

    for(i = 0; i < 16; i++)
        PRINTK("TxCI for Q#-%d          = %d\n\r", i, g_test_stats.txd.txc[i]);

    PRINTK("****FC-TX Statistics****\n\r");
    PRINTK("20MHz TXOP              = %d\n\r", get_machw_tx_num_20mhz_txop());
    PRINTK("40MHz TXOP              = %d\n\r", get_machw_tx_num_40mhz_txop());
    PRINTK("20MHz Frm in 40MHz TXOP = %d\n\r", get_machw_tx_num_20mhz_mpdu_in_40mhz_txop());
    PRINTK("Promoted MPDUs          = %d\n\r", get_machw_tx_num_promoted_mpdu());
    PRINTK("Demoted MPDUs           = %d\n\r", get_machw_tx_num_demoted_mpdu());
    PRINTK("Promoted Prot           = %d\n\r", get_machw_tx_num_prom_prot());
    PRINTK("FC Protected            = %d\n\r", get_machw_tx_num_fc_prot());
    PRINTK("TXOP Abort on Sec Busy  = %d\n\r", get_machw_tx_num_txop_abort_sec_busy());
#ifdef MWLAN
    for(i = 0; i < NUM_FC_ASSERT_COUNT_REG; i++)
        PRINTK("FcAssertRegister[%d]    = %d\n\r", i, get_machw_fc_assert_value(i));
#endif /* MWLAN */
    PRINTK("==================================\n\r");
}

/* This function prints the transmit data rate statistics */
INLINE void print_tx_dr_log(void)
{
    UWORD8 idx = 0;

    PRINTK("Transmit Data Rate Statistics\n\r");

    /* Print Non-HT Stats */
    PRINTK("Rate(Mbps):Preamble:TotalTxFrames\n\r");
    for(idx = 0; idx < NUM_NON_HT_RATES;idx++)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_tx_frm_stats[idx] > 0)
        {
            PRINTK("%2d %2d %8d\n\r", g_total_non_ht_gmixed_rates[idx].mbps,
                                    g_total_non_ht_gmixed_rates[idx].preamble,
                                    g_tx_frm_stats[idx]);
        }
    }

    /* Print HT Stats */
    PRINTK("MCS:TotalTxFrames:TotalTxFrames\n\r");
    while(idx < TOTAL_RATES)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_tx_frm_stats[idx] > 0)
        {
            PRINTK("%2d  %8d\n\r", idx - NUM_NON_HT_RATES, g_tx_frm_stats[idx]);
        }

        idx++;
    }
}

/* This function prints the details of a frame queued for transmission. This */
/* can be printed for each frame or for some intermediate frames.            */
/* Suggested use:                                                            */
/* Place : In fn 'queue_packet' before 'qmu_add_tx_packet' fn call           */
/* Call  : print_tx_frame_info(temp->frame_len, tx_rate, phy_tx_mode);       */
INLINE void print_tx_frame_info(UWORD16 len, UWORD16 dr, UWORD32 pm)
{
    PRINTK("Tx frame info: Length %d, Rate %d, PHY Tx Mode %x\n\r", len, dr, pm);
}

/* This function prints the transmit descriptor contents. This can be used   */
/* for each frame or for some intermediate frames.                           */
/* Suggested use:                                                            */
/* Place : In fn 'queue_packet' before 'qmu_add_tx_packet' fn call           */
/* Call  : print_tx_dscr_contents((UWORD32 *)g_test_config.tx.tx_dscr_addr); */
INLINE void print_tx_dscr_contents(UWORD32 *tx_dscr)
{
    UWORD8 i = 0;
    UWORD32 *sp = NULL;

    PRINTK("Tx descriptor contents (in Little Endian format)\n\r");

    for(i = 0; i < TX_DSCR_NUM_WORDS; i++)
        PRINTK("Word %d = %08x\n\r", i, convert_to_le(tx_dscr[i]));

    PRINTK("Tx Sub-MSDU Info Table:\n\r");
    sp = (UWORD32 *)get_tx_dscr_submsdu_info(tx_dscr);;
    for(i = 0; i < MAX_SUB_MSDU_TABLE_ENTRIES; i++)
      PRINTK("Word %d = %x\n\r", i, convert_to_le(sp[i]));

    PRINTK("\n\r");
}

/* This function prints the transmitted frame */
INLINE void print_tx_frame(UWORD32 *tx_dscr)
{
    UWORD16 len   = 0;
    UWORD8  *bufp = NULL;
    UWORD8 num_sm = get_tx_dscr_num_submsdu(tx_dscr);
    UWORD16 idx   = 0;
    UWORD8  fdx   = 0;
    buffer_desc_t buff_desc;

    len  = get_tx_dscr_mh_len(tx_dscr);
    bufp = (UWORD8 *)get_tx_dscr_mac_hdr_addr(tx_dscr);
    PRINTK("Header:\n\r");
    for(idx = 0; idx < len; idx++)
        PRINTK("%x ", bufp[idx]);
    PRINTK("Frame Body:\n\r");
    for(fdx = 0; fdx < num_sm; fdx++)
    {
        get_tx_dscr_submsdu_buff_info(tx_dscr, (UWORD32)&buff_desc, fdx);
        bufp = buff_desc.buff_hdl + buff_desc.data_offset;
        len  = buff_desc.data_length;
        for(idx = 0; idx < len; idx++)
                PRINTK("%x ", bufp[idx]);
    }
    PRINTK("\n\r");
}

/* This function creates the default pattern used for the Tx-Packets */
INLINE void prepare_default_tx_pattern(void)
{
    UWORD16 idx = 0;

    /* The default pattern is created only the first time. If there is */
    /* already a pattern in place, then it is not changed.             */
    if(g_tx_pattern_len == 0)
    {
        for(idx = 0; idx < 256; idx++)
            g_tx_pattern[idx] = (UWORD8)idx;

        g_tx_pattern_len = idx;
    }
}

/* This function inserts the supplied sequence number in the frame */
INLINE void set_hut_seqno(UWORD8 *buf, UWORD32 seqno)
{
    buf[0] = (seqno >> 0)  & 0xFF;
    buf[1] = (seqno >> 8)  & 0xFF;
    buf[2] = (seqno >> 16) & 0xFF;
    buf[3] = (seqno >> 24) & 0xFF;
}

/* This function returns the sequence number from the start of the buffer */
INLINE UWORD32 get_hut_seqno(UWORD8 *buf)
{
    UWORD32 seqno = 0;

    seqno = buf[3];
    seqno = (seqno << 8) | buf[2];
    seqno = (seqno << 8) | buf[1];
    seqno = (seqno << 8) | buf[0];

    return seqno;
}

/* This function returns the rate within the frame statistics table which */
/* corresponds to the required index                                      */
INLINE UWORD8 get_frm_stat_tx_rate(UWORD32 idx)
{
    /* Check whether the rate is a MCS */
    if(idx >= g_num_nonht_rates)
        return (idx - g_num_nonht_rates);

    return g_tx_rate_table[idx].mbps;
}

/* This function inserts the sequence number from the space identified by */
/* the rate index.                                                        */
INLINE void insert_hut_seqno_rate(UWORD8 *buf, UWORD8 idx)
{
    /* Update the sequence number in the frame body */
    set_hut_seqno(buf, g_tx_seqno_table[idx]);

    /* Increment the sequence number */
    g_tx_seqno_table[idx]++;
}

/* This function initializes the Tx-Sequence number table */
INLINE void init_hut_tx_seqno_table(void)
{
    UWORD32 idx = 0;

    for(idx = 0; idx < TOTAL_RATES; idx++)
        g_tx_seqno_table[idx] = INIT_TX_SEQNO;
}

/* This frame modifies the frame length of the transmit frames */
INLINE void update_tx_frame_length(q_info_t *q_info)
{
    /* Check whether frame length modification flag is enabled */
    if(g_enable_frame_len_modification == 1)
    {
        /* No Frame length modification is done if AMSDU is enabled */
        if(g_test_config.ht.amsdu_mode  == 0)
        {
            q_info->cur_frame_len = q_info->cur_frame_len - 1;

            if(q_info->cur_frame_len < q_info->min_frame_len)
                q_info->cur_frame_len = q_info->frame_len;
        }
    }
}

INLINE UWORD32 get_random_in_range(UWORD32 max)
{
     UWORD32  rand_no = get_random_byte() & 0x7F;

     return ((rand_no * max) >> 7);
}

INLINE BOOL_T is_tx_test_frame_bcst(void)
{
    UWORD32       rand_no  = 0;

    /* Generate a 7-bit Random number */
    rand_no =   get_random_byte() & 0x7F;

    if((100 * rand_no) < (UWORD32)(g_test_config.tx.bcst_percent * 128))
        return BTRUE;

    return BFALSE;

}
/* This function gathers some frame statistics from the received frame */
INLINE void update_frm_rx_stats(UWORD8 *msa, CIPHER_T ct, UWORD8 datarate,
                                UWORD8 status, UWORD16 len, UWORD32 ptm,
                                BOOL_T is_amsdu)
{
    UWORD16 idx = 0;
    UWORD32 ofst = get_mac_hdr_len(msa);

    /* Update frame format stats */
    switch((ptm & 0x7))
    {
    case 7:
        g_test_stats.rxd.ffmt.htgf++;
        break;
    case 6:
        g_test_stats.rxd.ffmt.htmix++;
        break;
    case 2:
        g_test_stats.rxd.ffmt.nht11a++;
        break;
    case 1:
        g_test_stats.rxd.ffmt.nht11bl++;
        break;
    case 0:
        g_test_stats.rxd.ffmt.nht11bs++;
        break;
    default:
        g_test_stats.rxd.ffmt.resvd++;
    }

    /* Update Channel Mask stats */
    switch(((ptm & 0x18) >> 3))
    {
    case 0:
        g_test_stats.rxd.ffmt.mask20++;
        break;
    case 1:
        g_test_stats.rxd.ffmt.mask40++;
        break;
    default:
        g_test_stats.rxd.ffmt.maskresvd++;
    }

    /* Update Channel Bandwidth stats */
    switch(((ptm & 0x60) >> 5))
    {
    case 0:
        g_test_stats.rxd.ffmt.bw40++;
        break;
    case 1:
        g_test_stats.rxd.ffmt.bw20u++;
        break;
    case 2:
        g_test_stats.rxd.ffmt.bw20++;
        break;
    default:
        g_test_stats.rxd.ffmt.bw20l++;
    }

    if(ptm & BIT11)
        g_test_stats.rxd.ffmt.shortgi++;

    /* Update frame length vs rate histogram. This is done only for the data */
    /* frames transmitted by the intended transmitter only.                  */
    if((filter_test_frame(msa) == BTRUE) &&
       (255 != (idx = get_frm_stat_rx_idx(datarate))))
    {
        /* Skip all frame headers and point to the body of the frame */
#ifdef MAC_WMM
        ofst += 2;
#endif /* MAC_WMM */

        /* Skip the Security header */
        if(ct != NO_ENCRYP)
            ofst += get_sec_header_len(ct);

        /* Skip the first Sub-MSDU header */
        if(is_amsdu == BTRUE)
            ofst += 14;

        /* Frame statistics are extracted from the sequence number only if */
        /* the frame is successfully received.                             */
        if(status == 1)
        {
            UWORD32 num_tx_frms = 1;

            /* It is assumed that the received frames carry valid sequence */
            /* numbers when the DUT is run in PER-Rx mode.                 */
            if(g_test_config.phy.mode == PHY_TEST_PER_RX_MODE)
            {
                UWORD32 seq_num = get_hut_seqno(msa + ofst);

                if((seq_num <= g_frm_stats[idx].last_seqno) ||
                   (g_frm_stats[idx].last_seqno == 0))
                {
                    /* Cond1: Either a frame reordering or a reset of the Txr.   */
                    /* Cond2: First frame received at this rate.                 */
                    /* Either case the new sequence number is adapted.           */
                    g_frm_stats[idx].last_seqno = seq_num - 1;
                    g_frm_stats[idx].temp_cnt = 0;

                }

                num_tx_frms = seq_num - g_frm_stats[idx].last_seqno;

                /* temp_cnt counts the number of frames received with FCS error  */
                /* between two correctly received frames. Since the sequence num */
                /* is used to determine the number of frames missed by the Rxr,  */
                /* temp_cnt is compensated from the difference. This works       */
                /* only when Txr does not retransmit failed frames. The if check */
                /* ensures some sanity when retransmissions happen.              */
                if(num_tx_frms > g_frm_stats[idx].temp_cnt)
                    num_tx_frms -= g_frm_stats[idx].temp_cnt;
                else
                    num_tx_frms  = 1;

                g_frm_stats[idx].last_seqno = seq_num;
                g_frm_stats[idx].temp_cnt   = 0;
            }

            g_frm_stats[idx].numtx_st += num_tx_frms;
            g_frm_stats[idx].numtx_lt += num_tx_frms;
            g_frm_stats[idx].numrx_st++;
            g_frm_stats[idx].numrx_lt++;
            g_frm_stats[idx].numsuc_st++;
            g_frm_stats[idx].numsuc_lt++;
        }
        else
        {
            g_frm_stats[idx].numtx_st++;
            g_frm_stats[idx].numtx_lt++;
            g_frm_stats[idx].numrx_st++;
            g_frm_stats[idx].numrx_lt++;

           if(g_test_config.phy.mode == PHY_TEST_PER_RX_MODE)
                g_frm_stats[idx].temp_cnt++;
        }
    }
}

/* This function prints the receive frame statistics */
INLINE void print_frm_stats(void)
{
    UWORD16 idx = 0;

    PRINTK("Directed Rx frame statistics\n\r");

    /* Print Non-HT Stats */
    PRINTK("Rate(Mbps): Preamble: TotalTxFrames: TotalRxFrames: NumSuccFrames\n\r");
    for(idx = 0; idx < NUM_NON_HT_RATES;idx++)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_frm_stats[idx].numtx_lt > 0)
        {
            PRINTK("%2d %2d %8d  %8d  %8d\n\r",g_total_non_ht_gmixed_rates[idx].mbps,
                   g_total_non_ht_gmixed_rates[idx].preamble,
                   g_frm_stats[idx].numtx_lt,
                   g_frm_stats[idx].numrx_lt,
                   g_frm_stats[idx].numsuc_lt);
        }
    }

    /* Print HT Stats */
    PRINTK("MCS:TotalTxFrames:TotalRxFrames:NumSuccFrames\n\r");
    while(idx < TOTAL_RATES)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_frm_stats[idx].numtx_lt > 0)
        {
            PRINTK("%2d  %8d  %8d  %8d\n\r",idx - NUM_NON_HT_RATES,
                   g_frm_stats[idx].numtx_lt, g_frm_stats[idx].numrx_lt,
                   g_frm_stats[idx].numsuc_lt);
        }

        idx++;
    }

    PRINTK("All Rx beacon frame statistics\n\r");

    /* Print Non-HT Stats */
    PRINTK("Rate(Mbps):TotalTxFrames:TotalRxFrames:NumSuccFrames\n\r");
    for(idx = 0; idx < NUM_NON_HT_RATES;idx++)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_bcn_frm_stats[idx].numrx_lt > 0)
        {
            /* Log statistics for rates for which at least one frame is received */
            PRINTK("%2d  %8d  %8d\n\r", g_total_non_ht_gmixed_rates[idx].mbps,
                                      g_bcn_frm_stats[idx].numrx_lt,
                                      g_bcn_frm_stats[idx].numsuc_lt);
        }
    }

    /* Print HT Stats */
    PRINTK("MCS:TotalTxFrames:TotalRxFrames:NumSuccFrames\n\r");
    while(idx < TOTAL_RATES)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_bcn_frm_stats[idx].numrx_lt > 0)
        {
            /* Log statistics for rates for which at least one frame is received */
            PRINTK("%2d  %8d  %8d\n\r", idx - NUM_NON_HT_RATES,
                                      g_bcn_frm_stats[idx].numrx_lt,
                                      g_bcn_frm_stats[idx].numsuc_lt);
        }
        idx++;
    }

    PRINTK("All Rx data/qos_data frame statistics\n\r");

    /* Print Non-HT Stats */
    PRINTK("Rate(Mbps):Preamble:TotalTxFrames:TotalRxFrames:NumSuccFrames\n\r");
    for(idx = 0; idx < NUM_NON_HT_RATES;idx++)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_dat_frm_stats[idx].numrx_lt > 0)
        {
            /* Log statistics for rates for which at least one frame is received */
            PRINTK("%2d %2d %8d  %8d\n\r", g_total_non_ht_gmixed_rates[idx].mbps,
                                      g_total_non_ht_gmixed_rates[idx].preamble,
                                      g_dat_frm_stats[idx].numrx_lt,
                                      g_dat_frm_stats[idx].numsuc_lt);
        }
    }

    /* Print HT Stats */
    PRINTK("MCS:TotalTxFrames:TotalRxFrames:NumSuccFrames\n\r");
    while(idx < TOTAL_RATES)
    {
        /* Log statistics for rates for which at least one frame is received */
        if(g_dat_frm_stats[idx].numrx_lt > 0)
        {
            /* Log statistics for rates for which at least one frame is received */
            PRINTK("%2d  %8d  %8d\n\r", idx - NUM_NON_HT_RATES,
                                      g_dat_frm_stats[idx].numrx_lt,
                                      g_dat_frm_stats[idx].numsuc_lt);
        }
        idx++;
    }
}

INLINE void reset_frm_stats(void)
{
    UWORD16 idx = 0;

    for(idx = 0; idx < TOTAL_RATES; idx++)
    {
        g_frm_stats[idx].last_seqno = 0;
        g_frm_stats[idx].temp_cnt  = 0;
        g_frm_stats[idx].numtx_lt  = 0;
        g_frm_stats[idx].numrx_lt  = 0;
        g_frm_stats[idx].numsuc_lt = 0;
        g_frm_stats[idx].numtx_st  = 0;
        g_frm_stats[idx].numrx_st  = 0;
        g_frm_stats[idx].numsuc_st = 0;

       g_dat_frm_stats[idx].numrx_lt  = 0;
       g_dat_frm_stats[idx].numsuc_lt = 0;
       g_bcn_frm_stats[idx].numrx_lt  = 0;
       g_bcn_frm_stats[idx].numsuc_lt = 0;
    }
}

INLINE void reset_st_frm_stats(void)
{
    UWORD16 idx = 0;

    for(idx = 0; idx < TOTAL_RATES; idx++)
    {
        g_frm_stats[idx].numtx_st  = 0;
        g_frm_stats[idx].numrx_st  = 0;
        g_frm_stats[idx].numsuc_st = 0;
    }
}

/* This function resets all test statistics */
INLINE void reset_test_stats(void)
{
    mem_set(&g_test_stats, 0, sizeof(test_stats_struct_t));
    g_last_rx_int_seqno = 0;
    g_num_frames_txqd   = 0;
    reset_frm_stats();
}

/* This function logs the frame statistics at the host. */
INLINE void log_st_frm_stats(void)
{
   UWORD16 idx = 0;

    for(idx = 0; idx < NUM_NON_HT_RATES;idx++)
    {
        /* Statistics are logged for rates at which at atlease one frame */
        /* is received.                                                  */
        if(g_frm_stats[idx].numtx_st > 0)
        {
            PRINTH("R%d-p%d,%d,%d,%d\n\r",g_total_non_ht_gmixed_rates[idx].mbps,
                   g_total_non_ht_gmixed_rates[idx].preamble,
                   g_frm_stats[idx].numtx_st,
                   g_frm_stats[idx].numrx_st,
                   g_frm_stats[idx].numsuc_st);
        }
    }

    while(idx < TOTAL_RATES)
    {
        /* Statistics are logged for MCSs for which at atleast one frame */
        /* is received.                                                  */
        if(g_frm_stats[idx].numtx_st > 0)
        {
            PRINTH("M%d,%d,%d,%d\n\r",(idx - NUM_NON_HT_RATES),
                   g_frm_stats[idx].numtx_st, g_frm_stats[idx].numrx_st,
                   g_frm_stats[idx].numsuc_st);
        }
        idx++;
    }

    PRINTH_END();

    reset_st_frm_stats();
}

/* This function prints different types of test statistics */
INLINE void print_test_stats(UWORD8 val)
{
	if(g_reset_mac_in_progress == BTRUE)
        return;

    /* Print the Testcase-ID before printing the actual test stats */
    PRINTK("############# %s #############\n\r",g_test_id);
    if((val == 0) || (val == 1))
    {
        print_tx_log();
        print_tx_dr_log();
    }

    if((val == 0) || (val == 2))
    {
        print_rx_log();
        print_rx_dr_log();
        print_frm_stats();
    }

    if((val == 0) || (val == 3))
        print_erri_log();

    if((val == 0) || (val == 4))
        print_oth_log();

    PRINTK("\n\r");
}

/* This function creates the response frame for the HUT log statistics */
/* requested by the host. Frame reception statistics are formatted in  */
/* a printable form and copied to a temporary buffer.                  */
INLINE UWORD8 *get_hut_log_stats(void)
{
   UWORD16 idx  = 0;
   UWORD16 ofst = 0;

    /* The first two bytes accomodate the length field */
    ofst = 2;

    for(idx = 0; idx < NUM_NON_HT_RATES;idx++)
    {
        /* Statistics are logged for rates at which at atleast one frame */
        /* is received.                                                  */
        ofst += PRINTS((WORD8 *)g_stats_buf+ofst, "R%d-p%d,%d,%d,%d\n\r",
               g_total_non_ht_gmixed_rates[idx].mbps,
               g_total_non_ht_gmixed_rates[idx].preamble,
               g_frm_stats[idx].numtx_st,
               g_frm_stats[idx].numrx_st,
               g_frm_stats[idx].numsuc_st);
    }

    while(idx < TOTAL_RATES)
    {
        /* Statistics are logged for MCSs for which at atleast one frame */
        /* is received.                                                  */
        ofst += PRINTS((WORD8 *)g_stats_buf+ofst, "M%d,%d,%d,%d\n\r",
               (idx - NUM_NON_HT_RATES),
               g_frm_stats[idx].numtx_st, g_frm_stats[idx].numrx_st,
               g_frm_stats[idx].numsuc_st);

        idx++;
    }

    /* Update the length field */
    g_stats_buf[0] = (ofst - 2) & 0x00FF;
    g_stats_buf[1] = ((ofst - 2) & 0xFF00) >> 8;

    /* Clear the Stats */
    reset_st_frm_stats();
    return  g_stats_buf;
}

/* This is the configuration interface function for clearing the per frame */
/* reception statistics.                                                   */
INLINE void set_hut_log_stats(UWORD8* val)
{
    reset_st_frm_stats();
}

/* This function extracts and updates the TSSI statistics from the last */
/* transmitted packet.                                                  */
INLINE void update_tssi_stats(UWORD32 *tx_dscr)
{
    g_last_pkt_tssi_value = get_tx_dscr_tssi_value(tx_dscr);
}

#endif /* MH_TEST_H */
#endif /* MAC_HW_UNIT_TEST_MODE */
