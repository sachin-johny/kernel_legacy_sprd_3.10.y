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
/*  File Name         : mh_test.c                                            */
/*                                                                           */
/*  Description       : This file contains all the functions for H/w unit    */
/*                      test.                                                */
/*                                                                           */
/*  List of Functions : start_test                                           */
/*                      configure_test_hw                                    */
/*                      configure_intr_test                                  */
/*                      configure_rx_test                                    */
/*                      configure_tx_test                                    */
/*                      configure_tsf_test                                   */
/*                      check_lut_prog                                       */
/*                      check_ampdu_lut_prog                                 */
/*                      check_ba_lut_prog                                    */
/*                      prepare_test_bar                                     */
/*                      send_test_bar                                        */
/*                      prepare_edca_frames                                  */
/*                      prepare_bcst_edca_frames                             */
/*                      prepare_abg_frames                                   */
/*                      alloc_set_dscr                                       */
/*                      queue_packet                                         */
/*                      check_and_send_ctrl_frames                           */
/*                      configure_test_mode                                  */
/*                      configure_phy_test                                   */
/*                      update_operating_channel_hut                         */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "mh_test.h"
#include "autorate.h"
#include "iconfig.h"

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

UWORD8              g_rx_buff_index               = 0;
UWORD8              g_cap_start                   = 0;
UWORD8              g_phy_test_idx_hi             = 0;
UWORD8              g_phy_test_idx_lo             = 0;
UWORD8              g_phy_test_idx                = 0;
UWORD32             g_next_bcn_addr               = 0;
UWORD32             g_total_tx_time               = 0;
UWORD32             g_tx_start_time_lo            = 0;
UWORD32             g_tx_start_time_hi            = 0;
UWORD32             g_tx_end_time_lo              = 0;
UWORD32             g_tx_end_time_hi              = 0;
UWORD32             g_num_tx_data_pkts            = 0;
UWORD32             g_num_nonht_rates             = NUM_NONHT_LONG_SHORT_RATES;
phy_test_rate_t     *g_tx_rate_table              = g_total_non_ht_gmixed_rates;
test_stats_struct_t g_test_stats                  = {0};
rx_frame_stats_t    g_frm_stats[TOTAL_RATES]      = {{0},};
rx_frame_stats_t    g_bcn_frm_stats[TOTAL_RATES]  = {{0},};
rx_frame_stats_t    g_dat_frm_stats[TOTAL_RATES]  = {{0},};
UWORD32             g_rxptr_dump[20]              = {0};
UWORD32             g_tx_frm_stats[TOTAL_RATES]   = {0};
UWORD32             g_tx_seqno_table[TOTAL_RATES] = {0};
UWORD32             g_num_frames_txqd             = 0;

#ifdef MAC_802_11I
/* PN Sequence Number of last frame of each TID from the transmitter */
UWORD8 g_pn_seqno[16][6];
UWORD8 g_pn_bcst_seqno[6];
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
q_head_t g_ampdu_buff_q[4];
UWORD8   g_ampdu_buff_num[4];
#endif /* MAC_802_11N */

/* Table of Total non HT mixed data rates */
phy_test_rate_t g_total_non_ht_gmixed_rates[NUM_NONHT_LONG_SHORT_RATES] =
{
    {0x04,  1, 1},  /*   1 Mbps Long  */
    {0x01,  2, 0},  /*   2 Mbps Short */
    {0x05,  2, 1},  /*   2 Mbps Long  */
    {0x02,  5, 0},  /* 5.5 Mbps Short */
    {0x06,  5, 1},  /* 5.5 Mbps Long  */
    {0x03, 11, 0},  /*  11 Mbps Short */
    {0x07, 11, 1},  /*  11 Mbps Long  */
    {0x0B,  6, 0},  /*   6 Mbps       */
    {0x0F,  9, 0},  /*   9 Mbps       */
    {0x0A, 12, 0},  /*  12 Mbps       */
    {0x0E, 18, 0},  /*  18 Mbps       */
    {0x09, 24, 0},  /*  24 Mbps       */
    {0x0D, 36, 0},  /*  36 Mbps       */
    {0x08, 48, 0},  /*  48 Mbps       */
    {0x0C, 54, 0}   /*  54 Mbps       */
};

/* Table of 11a data rates or G only mode data rates */
phy_test_rate_t g_total_non_ht_gonly_rates[NUM_DR_PHY_802_11G_ONLY] =
{
    {0x0B,  6, 0},  /*  6 Mbps */
    {0x0F,  9, 0},  /*  9 Mbps */
    {0x0A, 12, 0},  /* 12 Mbps */
    {0x0E, 18, 0},  /* 18 Mbps */
    {0x09, 24, 0},  /* 24 Mbps */
    {0x0D, 36, 0},  /* 36 Mbps */
    {0x08, 48, 0},  /* 48 Mbps */
    {0x0C, 54, 0}   /* 54 Mbps */
};

/* Table containing the list of all valid TX-Frame Formats */
/* Format: b6-b5[ChannelBW]:b4-b3[ChannelFilter]:b2-b0[PHYFrameFormat] */
UWORD8 g_vppp_allowed_txfmt[NUM_VALID_FRAME_FMT] = {
    0x40,  /*  20-20-11bSP */
    0x28,  /* 20U-40-11bSP */
    0x68,  /* 20L-40-11bSP */
    0x41,  /*  20-20-11bLP */
    0x29,  /* 20U-40-11bLP */
    0x69,  /* 20L-40-11bLP */
    0x42,  /*  20-20-11a   */
    0x0A,  /*  40-40-11a   */
    0x2A,  /* 20U-40-11a   */
    0x6A,  /* 20L-40-11a   */
    0x46,  /*  20-20-11nMF */
    0x0E,  /*  40-40-11nMF */
    0x2E,  /* 20U-40-11nMF */
    0x6E,  /* 20L-40-11nMF */
    0x47,  /*  20-20-11nGF */
    0x0F,  /*  40-40-11nGF */
    0x2F,  /* 20U-40-11nGF */
    0x6F   /* 20L-40-11nGF */
};

/* Table containing the list of all valid TX-Rate */
UWORD8 g_vppp_11b_rate_table[NUM_DR_PHY_802_11B] ={
     1,   /*   1 Mbps */
     2,   /*   2 Mbps */
     5,   /* 5.5 Mbps */
    11,   /*  11 Mbps */
};
/* Table containing the list of all valid TX-Rate */
UWORD8 g_vppp_11a_rate_table[NUM_DR_PHY_802_11A] ={
     6,   /*   6 Mbps */
     9,   /*   9 Mbps */
    12,   /*  12 Mbps */
    18,   /*  18 Mbps */
    24,   /*  24 Mbps */
    36,   /*  36 Mbps */
    48,   /*  48 Mbps */
    54,   /*  54 Mbps */
};
UWORD8  g_vppp_enabled_rate_table[NUM_DR_PHY_802_11G] = {0};
UWORD8  g_vppp_enabled_mcs_table[2*(MAX_MCS_SUPPORTED+1)] = {0};
BOOL_T  g_vppp_struct_init_done = BFALSE;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void configure_intr_test(void);
static void configure_rx_test(void);
static void configure_tsf_test(void);
static void configure_tx_test(void);
static void configure_test_mode(void);
static void configure_test_hw(void);
static void configure_phy_test(void);

#ifdef MAC_WMM
static void prepare_edca_frames(void);
static void prepare_bcst_edca_frames(void);
#else /* MAC_WMM */
static void prepare_abg_frames(void);
#endif /* MAC_WMM */

#ifdef MAC_802_11N
static UWORD32 prepare_test_bar(ba_lut_struct_t *ba);
#endif /* MAC_802_11N */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_test                                               */
/*                                                                           */
/*  Description   : This function starts the H/w unit test.                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function configures the H/w as per the test case    */
/*                  requirements.                                            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_test(void)
{
#ifdef BURST_TX_MODE
    /* Start other tests only when burst tx mode is disabled */
    if(g_burst_tx_mode_enabled == BTRUE)
    {
        unmask_machw_tx_comp_int();
        initiate_burst_tx_mode(&g_mac);
        return;
    }
    else
#endif /* BURST_TX_MODE */
    {
        /* Configure the hardware with any test settings specific to HUT */
        configure_test_hw();

        /* Configure the test mode and all tests */
        configure_test_mode();
        configure_rx_test();
        configure_tsf_test();
        configure_tx_test();
        configure_intr_test();
#ifdef TEST_TX_ABORT_FEATURE
        configure_txab_test();
#endif /* TEST_TX_ABORT_FEATURE */
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_test_hw                                        */
/*                                                                           */
/*  Description   : This function performs any specific H/w configuration    */
/*                  for the test.                                            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes all H/w modules which are      */
/*                  specifically required for the feature being tested.      */
/*                  These modules are not initialized in the regular h/w     */
/*                  initialization routine followed in system mode.          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_test_hw(void)
{

#ifdef MAC_802_11I
    /* Initialize the CE LUT */
    initialize_ce();
#endif /* MAC_802_11I */


    /* Control support for random Secondary CCA toggling in PHY */
    update_sec_cca_control_phy(g_test_params.sec_cca_assert);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_intr_test                                      */
/*                                                                           */
/*  Description   : This function configures the interrupts for the test.    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function unmasks required interrupts for the test.  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_intr_test(void)
{
    if(g_test_config.intr.rxcomplete == 1)
        unmask_machw_rx_comp_int();

    if(g_test_config.intr.tbtt == 1)
        unmask_machw_tbtt_int();

    if(g_test_config.intr.txcomplete == 1)
        unmask_machw_tx_comp_int();

    if(g_test_config.intr.cape == 1)
        unmask_machw_cap_end_int();

    if(g_test_config.intr.err == 1)
        unmask_machw_error_int();

    if(g_test_config.intr.hprx_comp == 1)
        unmask_machw_hprx_comp_int();

    if(g_test_config.intr.radar_det == 1)
        unmask_machw_radar_det_int();

    if(g_test_config.intr.txsus == 1)
        unmask_machw_txsus_int();

    if(g_test_config.intr.deauth == 1)
        unmask_machw_deauth_int();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_rx_test                                        */
/*                                                                           */
/*  Description   : This function configures tests related to receive path.  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function checks the test configuration and performs */
/*                  the required actions for that test.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_rx_test(void)
{
    UWORD32 temp = 0;

    /* Initialize the receive buffers */
    init_rx_buff();

/* 7          6            5          4        3       2       1       0     */
/* NonDirMgmt QCF_POLL     CF_END     ATIM     BCN     RTS     CTS     ACK   */
/* 14       13          12              11      10       9    8              */
/* SecChan  ExpectedBA  UnexpectedBA    Deauth  FCSFail  Dup  BcMcMgmtOBSS   */
/*        31-19     18                17          16          15             */
/*        Reserved  DiscardedIBSSBcn  NonDirCtrl  NonDirData  BcstDataOBSS   */
    temp  = (g_test_config.rx.ack_filter            << 0);
    temp |= (g_test_config.rx.cts_filter            << 1);
    temp |= (g_test_config.rx.rts_filter            << 2);
    temp |= (g_test_config.rx.bcn_filter            << 3);
    temp |= (g_test_config.rx.atim_filter           << 4);
    temp |= (g_test_config.rx.cf_end_filter         << 5);
    temp |= (g_test_config.rx.qcf_poll_filter       << 6);
    temp |= (g_test_config.rx.non_dir_mgmt_filter   << 7);
    temp |= (g_test_config.rx.bcmc_mgmt_obss_filter << 8);
    temp |= (g_test_config.rx.dup_filter            << 9);
    temp |= (g_test_config.rx.fcs_filter            << 10);
    temp |= (g_test_config.rx.deauth_filter         << 11);
    temp |= (g_test_config.rx.unexp_ba_filter       << 12);
    temp |= (g_test_config.rx.exp_ba_filter         << 13);
    temp |= (g_test_config.rx.sec_chan_filter       << 14);
    temp |= (g_test_config.rx.bcmc_data_obss_filter << 15);
    temp |= (g_test_config.rx.non_dir_data_filter   << 16);
    temp |= (g_test_config.rx.non_dir_ctrl_filter   << 17);
    temp |= (g_test_config.rx.disc_ibss_bcn_filter  << 18);

    set_machw_rx_frame_filter(temp);

    /* 31 - 5     4     3           2          1       0                   */
    /* Reserved  ATIM   Probe Rsp   Probe Req  Beacon  HighPriorityQEnable */
    temp  = (g_test_config.rx.hp_rxq_enable   << 0);
    temp |= (g_test_config.rx.hp_rxq_beacon   << 1);
    temp |= (g_test_config.rx.hp_rxq_probereq << 2);
    temp |= (g_test_config.rx.hp_rxq_probersp << 3);
    temp |= (g_test_config.rx.hp_rxq_atim     << 4);

    set_machw_hip_rxq_con(temp);

    if(1 == g_test_config.rx.soft_dupchk_enable)
        clear_tuple_cache();

#ifdef MAC_802_11N

    /* If Block ACK LUT programming test is enabled call a function to check */
    /* the same. The function adds maximum allowed entries to the LUT and    */
    /* then removes them. The values are read back into global arrays to be  */
    /* verified at the time of test.                                         */
    if(g_test_config.qos.ba_lut_prog_test == 1)
        check_ba_lut_prog();

    /* If Block ACK is enabled, update the Block ACK LUT with the details of */
    /* all the test Block ACK sessions.                                      */
    if(g_test_config.qos.ba_enable == 1)
    {
        UWORD8 i = 0;

        for(i = 0; i < g_test_config.qos.ba_num_sessions; i++)
        {
            ba_lut_struct_t le = g_test_config.qos.ba_lut_entry[i];

            /* Program the BA-LUT only when the DUT is the Recipient */
            if(le.dir == 0)
            {
                add_machw_ba_lut_entry(i, le.addr, le.tid, le.seq_num,
                                   le.win_size, le.partial_state);
                /* ITM-DEBUG */
                PRINTK("BA LUT_IDx=%d RA=%2x:%2x:%2x:%2x:%2x:%2x TID=%d Seq=%d WSize=%d\n\r",
                i,le.addr[0], le.addr[1],le.addr[2],le.addr[3],le.addr[4],
                le.addr[5],le.tid, le.seq_num,le.win_size);
            }
        }
    }

#endif /* MAC_802_11N */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_tx_test                                        */
/*                                                                           */
/*  Description   : This function configures tests related to transmit path. */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function checks the test configuration and performs */
/*                  the required actions for that test.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_tx_test(void)
{
    if(g_test_config.tx.slot == 1)
        set_machw_short_slot_select();
    else
        set_machw_long_slot_select();

    /* Configure RTS/CTS for TXOP with No ACK or Block ACK policy */
    if(g_test_config.tx.rtscts_txop == 1)
        disable_machw_rtscts_norsp();
    else
        enable_machw_rtscts_norsp();

    /* Prepare defualt Tx-frame pattern. If Tx-test is not enabled, then */
    /* this is used for frame integrity check.                           */
    prepare_default_tx_pattern();

    /* Initialize the table of sequence numbers maintained by MAC S/w */
    init_hut_tx_seqno_table();

    if(g_test_config.tx.tx_test == 1)
    {
        UWORD16 i = 0;

#ifdef MAC_WMM

        g_test_config.tx.init_num_pkt = g_test_config.tx.ac_be.init_num_pkt +
                                        g_test_config.tx.ac_bk.init_num_pkt +
                                        g_test_config.tx.ac_vo.init_num_pkt +
                                        g_test_config.tx.ac_vi.init_num_pkt +
                                        g_test_config.tx.hp.init_num_pkt;

#else /* MAC_WMM */

        g_test_config.tx.init_num_pkt = g_test_config.tx.nor.init_num_pkt +
                                        g_test_config.tx.cf.init_num_pkt  +
                                        g_test_config.tx.hp.init_num_pkt;
#endif /* MAC_WMM */


#ifndef MAC_WMM

        prepare_abg_frames();

#else /* MAC_WMM */

        /* Prepare all frames */
        prepare_edca_frames();

        /* Prepare all broadcast frames if required */
        if(g_test_config.tx.bcst_percent > 0)
            prepare_bcst_edca_frames();

        /* Other configurations */
        set_machw_stamacaddr(g_test_config.tx.da);
        set_machw_seq_num_index_update(0, BTRUE);
#endif /* MAC_WMM */

#ifdef MAC_802_11N
        /* If Block ACK is enabled, prepare the necessary control frames */
        if(g_test_config.qos.ba_enable == 1)
        {
            UWORD8 i = 0;

            for(i = 0; i < g_test_config.qos.ba_num_sessions; i++)
            {
                ba_lut_struct_t *le = &g_test_config.qos.ba_lut_entry[i];

                if(le->dir == 1)
                    prepare_test_bar(le);
                else
                    le->bar_addr = 0;
            }
        }
#endif /* MAC_802_11N */

        /* Queue packets */
        for(i = 0; i < g_test_config.tx.init_num_pkt; i++)
        {
            q_info_t *temp_qinfo = NULL;

            alloc_set_dscr();
            temp_qinfo = (q_info_t *)get_init_qinfo_struct();
            queue_packet(temp_qinfo);
            g_num_frames_txqd++;
            check_and_send_ctrl_frames(temp_qinfo);
        }

        /* Initialize the Tx-Time measurement parameters */
        g_total_tx_time = 0;
        get_machw_tsf_timer(&g_tx_start_time_hi, &g_tx_start_time_lo);
        g_num_tx_data_pkts = 0;

        if(g_test_config.tx.test_time != 0)
        {
            /* This computation introduces some error in the test end time   */
            /* which is acceptable since this is not a cruicial parameter.   */
            g_tx_end_time_hi = g_tx_start_time_hi +
                             ((g_test_config.tx.test_time >> 12) & 0x000FFFFF);
            g_tx_end_time_lo = g_tx_start_time_lo +
                             ((g_test_config.tx.test_time << 20));
            if(g_tx_end_time_lo < g_tx_start_time_lo)
                g_tx_end_time_hi++;
        }
    }

#ifdef MAC_802_11N
    /* If A-MPDU LUT programming test is enabled call a function to check    */
    /* the same. The function adds maximum allowed entries to the LUT and    */
    /* then removes them. The values are read back into global arrays to be  */
    /* verified at the time of test.                                         */
    if(g_test_config.ht.ampdu_lut_prog_test == 1)
        check_ampdu_lut_prog();

    /* Update the A-MPDU LUT with details of all the test A-MPDU sessions. */
    {
        UWORD8 i = 0;

        for(i = 0; i < g_test_config.ht.ampdu_num_sessions; i++)
        {
            ampdu_params_t le = g_test_config.ht.ampdu_sess_params[i];

            /* Index argument passed in the function is incremented as the   */
            /* entry at index 0 is invalid.                                  */
            add_machw_ampdu_lut_entry(i + 1, le.maxnum, le.minmps, le.maxraf);

            PRINTK("Programmed A-MPDU Params:\n\r");
            PRINTK("TID=%d maxnum=%d maxraf=%d minmps=%d numbuf=%d\n\r",
            le.tid, le.maxnum, le.maxraf, le.minmps, le.numbuf);

        }
    }
#endif /* MAC_802_11N */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_tsf_test                                       */
/*                                                                           */
/*  Description   : This function configures tests related to TSF.           */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function checks the test configuration and performs */
/*                  the required actions for that test.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_tsf_test(void)
{
    /* Create the Beacon Frame */
    g_test_config.tsf.bcn_addr = (UWORD32)mem_alloc(g_shared_pkt_mem_handle,
                                                    BEACON_LEN);

    if(0 == g_test_config.tsf.bcn_addr)
        return;

    prepare_beacon((UWORD8 *)(g_test_config.tsf.bcn_addr), 0);

    set_machw_beacon_pointer(g_test_config.tsf.bcn_addr);
    set_machw_beacon_tx_params(g_test_config.tsf.bcn_len,
                               get_phy_rate(g_test_config.tsf.bcn_rate));
    set_machw_beacon_ptm(get_reg_phy_tx_mode(g_test_config.tsf.bcn_rate, 1));

    /* If Beacon Swap Test is enabled, then create the alternate Beacon frame */
    if(1 == g_test_config.tsf.bcn_swap_test)
    {
        g_test_config.tsf.next_bcn_addr =
                       (UWORD32)mem_alloc(g_shared_pkt_mem_handle, BEACON_LEN);

        if(0 == g_test_config.tsf.next_bcn_addr)
            g_test_config.tsf.bcn_swap_test = 0;
        else
            prepare_beacon((UWORD8 *)(g_test_config.tsf.next_bcn_addr), 1);
    }

    switch(g_test_config.tsf.tsf_test)
    {
    case 0:
    {
        set_machw_tsf_disable();
    }
    break;
    case 1:
    {

        set_machw_tsf_join();
    }
    break;
    case 2:
    {
        set_machw_tsf_start();
    }
    break;
    case 3:
    {
        set_machw_tsf_start();
        set_machw_tsf_beacon_tx_suspend_enable();
    }
    break;
    default:
    {
        /* Do nothing */
    }
    break;
    }

    /* Disable Beacon transmission for PHY-SM test */
    if(g_test_config.phy.mode != PHY_TEST_DISABLED)
        set_machw_tsf_beacon_tx_suspend_enable();
}

#ifdef MAC_802_11I
/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_lut_prog                                           */
/*                                                                           */
/*  Description   : This function checks LUT programming.                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks LUT programming.                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 g_ce_exc[20] = {0xAA};

void check_lut_prog(void)
{
    UWORD8 key1[10][16] = {{0},};
    UWORD8 key2[16]     = {0};
    UWORD32 i, j;
    UWORD8 addr[6];
    UWORD8 mick[8]  = {0};

    addr[0] = 0;
    addr[1] = 0xDE;
    addr[2] = 0xFA;
    addr[3] = 0xCE;
    addr[4] = 0xBA;


    for(i = 0; i < 10; i ++)
    {
        for(j = 0; j < 4; j ++)
        {
            key1[i][4*j + 0] = (UWORD8)(i*j);
            key1[i][4*j + 1] = (UWORD8)(i*j + 1);
            key1[i][4*j + 2] = (UWORD8)(i*j + 2);
            key1[i][4*j + 3] = (UWORD8)(i*j + 3);
        }

    }

    for(i = 0 ; i < 10 ; i ++)
    {
        addr[5] = i*i;
        machw_ce_add_key(0, PTK_NUM, i, 0, 2, key1[i], addr, SUPP_KEY, mick);
    }

    for(i = 0 ; i < 10 ; i ++)
    {
        addr[5] = i*i;
        machw_ce_read_key(1, 0, i, key2);
        for(j = 0; j < 16; j++)
        {
            if(key1[i][j] != key2[j])
            {
                g_ce_exc[i] = 0xFF;
                break;
            }
        }
    }

    for(i = 0 ; i < 10 ; i ++)
    {
        machw_ce_del_key(i);
    }
}
#endif /* MAC_802_11I */

#ifdef MAC_802_11N
/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_ampdu_lut_prog                                     */
/*                                                                           */
/*  Description   : This function checks A-MPDU LUT programming.             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function programs A-MPDU LUT with maximum number of */
/*                  entries and checks if the programming is done correctly. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 g_ampdu_lut_arr0[MAX_AMPDU_LUT_SIZE];
UWORD32 g_ampdu_lut_arr1[MAX_AMPDU_LUT_SIZE];
UWORD32 g_ampdu_lut_arr2[MAX_AMPDU_LUT_SIZE];

void check_ampdu_lut_prog(void)
{
    UWORD8 i      = 0;
    UWORD8 maxnum = 0;
    UWORD8 minmps = 0;
    UWORD8 maxraf = 0;

    /* Add to the LUT for maximum size allowed varying last byte of address, */
    /* TID, sequence number and window size                                  */
    for(i = 0; i < MAX_AMPDU_LUT_SIZE; i ++)
    {
        maxnum++;
        if(maxnum > 8)
            maxnum = 0;

        minmps++;
        if(minmps > 7)
            minmps = 0;

        maxraf++;
        if(maxraf > 3)
            maxraf = 0;

        g_ampdu_lut_arr0[i] = BIT23                            |
                              (((UWORD32)maxnum & 0x3F) << 15) |
                              (((UWORD32)maxraf & 0x03) << 13) |
                              (((UWORD32)minmps & 0x07) << 10) |
                              (((UWORD32)i      & 0xFF) << 2);

        /* Add the entry to A-MPDU LUT */
        add_machw_ampdu_lut_entry(i , maxnum, minmps, maxraf);
    }

    /* Read back all the LUT entries. Need to check manually if the LUT was  */
    /* updated as required. The values are stored in a global array for this */
    /* purpose.                                                              */
    for(i = 0; i < MAX_AMPDU_LUT_SIZE; i ++)
    {
        UWORD32 temp_val = 0;

        read_machw_ampdu_lut_entry(i, &temp_val);
        g_ampdu_lut_arr1[i] = temp_val & ~(BIT0 | BIT1);
    }

    /* Remove all the LUT entries */
    for(i = 0 ; i < MAX_AMPDU_LUT_SIZE; i ++)
    {
        remove_machw_ampdu_lut_entry(i);
    }

    /* Read back all the LUT entries. Need to check manually if the LUT was  */
    /* updated as required. The values are stored in a global array for this */
    /* purpose.                                                              */
    for(i = 0; i < MAX_AMPDU_LUT_SIZE; i ++)
    {
        UWORD32 temp_val = 0;

        read_machw_ampdu_lut_entry(i, &temp_val);
        g_ampdu_lut_arr2[i] = temp_val & ~(BIT0 | BIT1);
    }

    /* Validate the set and report any inconsistencies */
    for(i = 0; i < MAX_AMPDU_LUT_SIZE; i ++)
    {
        if(g_ampdu_lut_arr1[i] != g_ampdu_lut_arr0[i])
        {
            PRINTK("LUT Addition failed for Index=%d, WR=%x RD=%x\n\r",
            i, g_ampdu_lut_arr0[i], g_ampdu_lut_arr1[i]);
        }
    }

    /* Validate the set and report any inconsistencies */
    for(i = 0; i < MAX_AMPDU_LUT_SIZE; i ++)
    {
        if(g_ampdu_lut_arr2[i] != (i << 2))
        {
            PRINTK("LUT Deletion failed for Index=%d. RD=%x\n\r",
                   i, g_ampdu_lut_arr2[i]);
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : check_ba_lut_prog                                        */
/*                                                                           */
/*  Description   : This function checks BA LUT programming.                 */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function programs the BA LUT with maximum number of */
/*                  entries and checks if the programming is done correctly. */
/*                  Note that currently only Compressed Block ACK is         */
/*                  supported. Hence all the entries are set and checked for */
/*                  only Compressed Block ACK.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 g_ba_lut_arr0[MAX_BA_LUT_SIZE];
UWORD32 g_ba_lut_arr1[MAX_BA_LUT_SIZE][3];
UWORD32 g_ba_lut_arr2[MAX_BA_LUT_SIZE][3];

void check_ba_lut_prog(void)
{
    UWORD8  i        = 0;
    UWORD8  win_size = 0;
    UWORD8  tid      = 0;
    UWORD8  par_st   = 0;
    UWORD16 seq_num  = 0;
    UWORD8  addr[6]  = {0};

    /* Initialize first 5 bytes of the address. The last byte will be varied */
    addr[1] = 0x89;
    addr[2] = 0xAB;
    addr[3] = 0xCD;
    addr[4] = 0xEF;

    /* Add to the LUT for maximum size allowed varying last byte of address, */
    /* TID, sequence number and window size                                  */
    for(i = 0; i < MAX_BA_LUT_SIZE; i ++)
    {
        /* Set the first and last byte of the address to some value based on i */
        addr[0] = i + 1;
        addr[5] = i;

        /* Increment TID and wrap-around */
        tid++;
        if(tid > 15)
            tid = 0;

        /* Increment sequence number by steps of 1024 and wrap-around */
        seq_num += 1024;
        if(seq_num > 4095)
            seq_num = 0;

        /* Increment window size by steps of 16 and wrap-around */
        win_size += 16;
        if(win_size > 64)
            win_size = 0;

        /* Alternate between partial and full state */
        par_st = 1 - par_st;

        g_ba_lut_arr0[i] =  ((tid & 0x0F) | ((seq_num & 0x0FFF) << 4) |
                             ((win_size & 0x3F) << 16) |
                             ((par_st & 0x01) << 24));

        /* Add the entry to BA LUT */
        add_machw_ba_lut_entry(i, addr, tid, seq_num, win_size, par_st);
    }

    /* Read back all the LUT entries. Need to check manually if the LUT was  */
    /* updated as required. The values are stored in a global array for this */
    /* purpose.                                                              */
    for(i = 0; i < MAX_BA_LUT_SIZE; i ++)
    {
        read_machw_ba_lut_entry(i);

        get_machw_ba_comp_info(&g_ba_lut_arr1[i][0], &g_ba_lut_arr1[i][1],
                               &g_ba_lut_arr1[i][2]);
    }

    /* Remove all the LUT entries */
    for(i = 0 ; i < MAX_BA_LUT_SIZE; i ++)
    {
        remove_machw_ba_lut_entry(i);
    }

    /* Read back all the LUT entries. Need to check manually if the LUT was  */
    /* updated as required. The values are stored in a global array for this */
    /* purpose.                                                              */
    for(i = 0; i < MAX_BA_LUT_SIZE; i ++)
    {
        read_machw_ba_lut_entry(i);

        get_machw_ba_comp_info(&g_ba_lut_arr2[i][0], &g_ba_lut_arr2[i][1],
                               &g_ba_lut_arr2[i][2]);
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_test_bar                                         */
/*                                                                           */
/*  Description   : This function prepares the Block ACK request frame for   */
/*                  unit test.                                               */
/*                                                                           */
/*  Inputs        : 1) Pointer to the Block ACK structure                    */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function prepares the Block ACK request frame for   */
/*                  unit test.                                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD32, length of Block ACK request frame               */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 prepare_test_bar(ba_lut_struct_t *ba)
{
    UWORD16 bar_ctl =0;
    UWORD8  *data   = NULL;

    /*************************************************************************/
    /*                     BlockAck Request Frame Format                     */
    /* --------------------------------------------------------------------  */
    /* |Frame Control|Duration|DA|SA|BAR Control|BlockAck Starting    |FCS|  */
    /* |             |        |  |  |           |Sequence number      |   |  */
    /* --------------------------------------------------------------------  */
    /* | 2           |2       |6 |6 |2          |2                    |4  |  */
    /* --------------------------------------------------------------------  */
    /*                                                                       */
    /*************************************************************************/

    data = (UWORD8 *)mem_alloc(g_shared_pkt_mem_handle, (20 + FCS_LEN));

    if(data == NULL)
    {
        ba->bar_addr = 0;

        return 0;
    }

    /*************************************************************************/
    /*                Set the fields in the frame header                     */
    /*************************************************************************/

    /* All the fields of the Frame Control Field are set to zero. Only the   */
    /* Type/Subtype field is set.                                            */
    set_frame_control(data, (UWORD16)BLOCKACK_REQ);

    /* Set DA to the address of the STA requesting authentication */
    set_address1(data, g_test_config.tx.da);

    /* Set SA to the dot11MacAddress */
    set_address2(data, mget_StationID());

    /* BAR Control field */
    bar_ctl = (ba->tid << 12);

    /* Only Compressed Block-Ack Policy supported */
    bar_ctl |= BIT2;

    /* BAR-Ack Policy is set to Normal Ack */
    bar_ctl &= ~BIT0;

    data[16] = bar_ctl & 0xFF;
    data[17] = (bar_ctl >> 8) & 0xFF;

    /* Sequence number */
    data[18] = 0;
    data[19] = 0;

    ba->bar_addr = (UWORD32)data;

    return 20 + FCS_LEN;
}

UWORD8 send_test_bar(ba_lut_struct_t *ba, UWORD8 q_num)
{
    UWORD8        idx          = 0;
    UWORD8        stat         = 0;
    UWORD8        tx_rate      = 0;
    UWORD8        pream        = 0;
    UWORD8        mac_hdr_len  = 0;
    UWORD16       frame_len    = 0;
    UWORD32       phy_tx_mode  = 0;
    UWORD32       *tx_dscr     = NULL;
    buffer_desc_t buffer_desc  = {0};
    UWORD32       retry_set[2] = {0};

    /* Create the Tx descriptor*/
    g_test_config.tx.tx_dscr_addr = (UWORD32)create_default_tx_dscr(0, 0, 0);

    tx_dscr = (UWORD32 *)g_test_config.tx.tx_dscr_addr;

    stat = ((PENDING << 5) & 0xE0) | BIT4 | (ba->tid & 0x0F);

    set_tx_dscr_status(tx_dscr, stat);

    set_tx_dscr_cipher_type(tx_dscr, NO_ENCRYP);

    tx_rate     = get_max_basic_rate();
    pream       = 1;
    phy_tx_mode = get_hut_phy_tx_mode(tx_rate, pream);

    /* Set the rate set words with the same rate as current transmit rate */
    update_retry_rate_set(0, tx_rate, 0, retry_set);

    set_tx_params((UWORD8 *)tx_dscr, tx_rate, pream, COMP_BAR, phy_tx_mode,
                  retry_set);
    set_tx_dscr_q_num(tx_dscr, ba->tid);

    /* Set the SMPS mode field in the Tx descriptor */
    set_tx_dscr_smps_mode(tx_dscr, g_test_config.ht.da_smps_mode);

    /* Set the mac header related parameters in the Tx descriptor */
    mac_hdr_len = 16;
    frame_len   = 24;

    set_tx_dscr_buffer_addr(tx_dscr, ba->bar_addr);
    set_tx_dscr_mh_len(tx_dscr, mac_hdr_len);
    set_tx_dscr_mh_offset((UWORD32 *)g_test_config.tx.tx_dscr_addr, 0);

    /* Create the Payload Buffer descriptor */
    buffer_desc.buff_hdl    = (UWORD8 *)(ba->bar_addr);
    buffer_desc.data_offset = mac_hdr_len;
    buffer_desc.data_length = frame_len - mac_hdr_len - FCS_LEN;

#ifndef DEBUG_KLUDGE
    /* This Kludge fixes the TX-Hang issue in MAC H/w */
    buffer_desc.data_length += 4;
#endif /* DEBUG_KLUDGE */

   /* Set the data buffer related information in the Tx-descriptor */
    set_tx_dscr_submsdu_buff_info(tx_dscr, (UWORD32)(&buffer_desc), 0);

    /* Set the frame length in the Tx Descriptor */
    set_tx_dscr_frame_len(tx_dscr, frame_len);
    set_tx_dscr_num_submsdu(tx_dscr, 1);
    set_tx_dscr_frag_len(tx_dscr, frame_len, 0);

    /* Set RA-LUT Index */
    for(idx = 0; idx < g_test_config.ht.ampdu_num_sessions; idx++)
        if(g_test_config.ht.ampdu_sess_params[idx].tid == ba->tid)
            break;

    if(idx == g_test_config.ht.ampdu_num_sessions)
        idx = 0;
    else
        idx++;

    set_tx_dscr_ra_lut_index(tx_dscr, idx);

    /* Update the TSF timestamp */
    update_tx_dscr_tsf_ts(tx_dscr);

    /* Set the MAC H/w queue pointers based on configuration */
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, q_num, (UWORD8 *)tx_dscr) != QMU_OK)
    {
        /* Exception. Do nothing. */
    }

    return 0;
}

#endif /* MAC_802_11N */

#ifdef MAC_WMM
/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_edca_frames                                      */
/*                                                                           */
/*  Description   : This function prepares all frames for EDCA.              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function prepares the frames for EDCA.              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_edca_frames(void )
{
    UWORD16 frame_len = 0;

    if(g_test_config.tx.ac_be.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_be.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_be.frame_addr = (UWORD32)mem_alloc(
                                            g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_be.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_be.frame_addr,
                             AC_BE_Q, BFALSE);
        }
        else
            g_test_config.tx.ac_be.init_num_pkt = 0;
    }

    if(g_test_config.tx.ac_bk.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_bk.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_bk.frame_addr = (UWORD32)mem_alloc(
                                            g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_bk.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_bk.frame_addr,
                             AC_BK_Q, BFALSE);
        }
        else
            g_test_config.tx.ac_bk.init_num_pkt = 0;
    }

    if(g_test_config.tx.ac_vo.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_vo.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_vo.frame_addr = (UWORD32)mem_alloc(
                                            g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_vo.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_vo.frame_addr,
                             AC_VO_Q, BFALSE);
        }
        else
            g_test_config.tx.ac_vo.init_num_pkt = 0;
    }

    if(g_test_config.tx.ac_vi.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_vi.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_vi.frame_addr = (UWORD32)mem_alloc(
                                            g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_vi.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_vi.frame_addr,
                             AC_VI_Q, BFALSE);
        }
        else
            g_test_config.tx.ac_vi.init_num_pkt = 0;
    }

    if(g_test_config.tx.hp.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.hp.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.hp.frame_addr = (UWORD32)mem_alloc(
                                         g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.hp.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.hp.frame_addr,
                             HIGH_PRI_Q, BFALSE);
        }
        else
            g_test_config.tx.hp.init_num_pkt = 0;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_bcst_edca_frames                                 */
/*                                                                           */
/*  Description   : This function prepares all broadcast frames for EDCA.    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function prepares broadcast frames for EDCA.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_bcst_edca_frames(void)
{
    UWORD16 frame_len = 0;

    if(g_test_config.tx.ac_be.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_be.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_be.bcst_frame_addr = (UWORD32)mem_alloc(
                                         g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_be.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_be.bcst_frame_addr,
                             AC_BE_Q, BTRUE);
        }
    }

    if(g_test_config.tx.ac_bk.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_bk.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_bk.bcst_frame_addr = (UWORD32)mem_alloc(
                                         g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_bk.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_bk.bcst_frame_addr,
                             AC_BK_Q, BTRUE);
        }
    }

    if(g_test_config.tx.ac_vo.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_vo.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_vo.bcst_frame_addr = (UWORD32)mem_alloc(
                                         g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_vo.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_vo.bcst_frame_addr,
                             AC_VO_Q, BTRUE);
        }
    }

    if(g_test_config.tx.ac_vi.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.ac_vi.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.ac_vi.bcst_frame_addr = (UWORD32)mem_alloc(
                                          g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.ac_vi.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.ac_vi.bcst_frame_addr,
                             AC_VI_Q, BTRUE);
        }
    }

    if(g_test_config.tx.hp.init_num_pkt > 0)
    {
        frame_len = MAX(g_test_config.tx.hp.frame_len, MIN_TX_BUFFER_LEN);

        g_test_config.tx.hp.bcst_frame_addr = (UWORD32)mem_alloc(
                                         g_shared_pkt_mem_handle, frame_len);
        if(g_test_config.tx.hp.frame_addr != 0)
        {
            prepare_tx_frame((UWORD8 *)g_test_config.tx.hp.bcst_frame_addr,
                             HIGH_PRI_Q, BTRUE);
        }
    }
}
#else /* MAC_WMM */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : prepare_abg_frames                                       */
/*                                                                           */
/*  Description   : This function prepares all frames for a/b/g mode.        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function prepares the frames for a/b/g mode.        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void prepare_abg_frames(void )
{
    UWORD16 frame_len = 0;

    frame_len = MAX(g_test_config.tx.hp.frame_len, MIN_TX_BUFFER_LEN);

    g_test_config.tx.hp.frame_addr = (UWORD32)mem_alloc(
                                     g_shared_pkt_mem_handle, frame_len);
    prepare_tx_frame((UWORD8 *)g_test_config.tx.hp.frame_addr, HIGH_PRI_Q);


    frame_len = MAX(g_test_config.tx.nor.frame_len, MIN_TX_BUFFER_LEN);

    g_test_config.tx.nor.frame_addr = (UWORD32)mem_alloc(
                                     g_shared_pkt_mem_handle, frame_len);
    prepare_tx_frame((UWORD8 *)g_test_config.tx.nor.frame_addr, NORMAL_PRI_Q);

    frame_len = MAX(g_test_config.tx.cf.frame_len, MIN_TX_BUFFER_LEN);

    g_test_config.tx.cf.frame_addr = (UWORD32)mem_alloc(
                                     g_shared_pkt_mem_handle, frame_len);
    prepare_tx_frame((UWORD8 *)g_test_config.tx.cf.frame_addr, CF_PRI_Q);
}

#endif /* MAC_WMM */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : alloc_set_dscr                                           */
/*                                                                           */
/*  Description   : This function allocates and sets the transmit descriptor */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function allocates memory for ans sets Tx Dscr.     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void alloc_set_dscr(void)
{
    UWORD8 is_amsdu = 0;

    if(g_test_config.ht.amsdu_mode != 0)
        is_amsdu = 1;

    /* Create the Tx descriptor*/
    g_test_config.tx.tx_dscr_addr = (UWORD32)create_default_tx_dscr(0, 0,
                                             is_amsdu);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : queue_packet                                             */
/*                                                                           */
/*  Description   : This function queues the packet as required.             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function queues the packet and updates globals.     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void queue_packet(q_info_t *temp)
{
    UWORD8        idx          = 0;
    UWORD8        tx_rate      = 0;
    UWORD8        pream        = 0;
    UWORD8        mac_hdr_len  = 0;
    UWORD8        tid          = 0;
    UWORD32       phy_tx_mode  = 0;
    UWORD32       retry_set[2] = {0};
    buffer_desc_t buffer_desc  = {0};
    BOOL_T        bcmc_type   = BFALSE;
    UWORD32       frame_addr  = 0;
    UWORD8        ack_policy  = 0;

    /* Insert broadcast frames randomly */
    if(is_tx_test_frame_bcst() == BTRUE)
    {
        bcmc_type   = BTRUE;
        g_test_stats.txd.bcst++;
    }

    /* Update the TX-Parameters if VPPP feature is enabled */
    update_vpp_params(temp);

    if(bcmc_type == BTRUE)
    {
        tx_rate     = get_hut_tx_rate();

#ifdef MAC_802_11I
        if(g_test_config.ce.ce_enable == 1)
        {
            set_tx_dscr_cipher_type((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                                    g_test_config.ce.bcst_ct);
            set_tx_dscr_key_type((UWORD32 *)g_test_config.tx.tx_dscr_addr, 0);
            set_tx_dscr_key_index((UWORD32 *)g_test_config.tx.tx_dscr_addr, 0);

            /* If the Cipher Type is other than CCMP for Bcst frames, then */
            /* they are sent as Non-HT frames at the configured rates (MCS */
            /* is not used)                                                */
            if(g_test_config.ce.bcst_ct != CCMP)
                tx_rate = get_curr_tx_rate();
        }
        else
            set_tx_dscr_cipher_type((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                                    NO_ENCRYP);
#endif /* MAC_802_11I */

        pream       = get_preamble(tx_rate);
        phy_tx_mode = get_hut_phy_tx_mode(tx_rate, pream);
        ack_policy  = BCAST_NO_ACK;
        frame_addr  = temp->bcst_frame_addr;
    }
    else
    {
        tx_rate     = get_hut_tx_rate();
        pream       = get_preamble(tx_rate);
        phy_tx_mode = get_hut_phy_tx_mode(tx_rate, pream);
        ack_policy  = temp->ack_policy;
        frame_addr  = temp->frame_addr;

#ifdef MAC_802_11I
        if(g_test_config.ce.ce_enable == 1)
        {
            set_tx_dscr_cipher_type((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                                    g_test_config.ce.ct);
            set_tx_dscr_key_type((UWORD32 *)g_test_config.tx.tx_dscr_addr, 1);
            set_tx_dscr_key_index((UWORD32 *)g_test_config.tx.tx_dscr_addr, 1);
        }
        else
            set_tx_dscr_cipher_type((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                                    NO_ENCRYP);
#endif /* MAC_802_11I */
    }

#ifndef MAC_802_11I

    set_tx_dscr_cipher_type((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                            NO_ENCRYP);
#endif /* MAC_802_11I */

    /* Update the retry auto rate table in the descriptor  */
    if(g_test_config.tx.enable_retry_ar == 1)
    {
        /* Update the rate set words with the rates from the configured test */
        /* retry rate table if retry auto rate is enabled                    */
        retry_set[0] = (g_test_config.tx.retry_rates[3] << 24) |
                       (g_test_config.tx.retry_rates[2] << 16) |
                       (g_test_config.tx.retry_rates[1] <<  8) |
                       (g_test_config.tx.retry_rates[0]);

        retry_set[1] = (g_test_config.tx.retry_rates[7] << 24) |
                       (g_test_config.tx.retry_rates[6] << 16) |
                       (g_test_config.tx.retry_rates[5] <<  8) |
                       (g_test_config.tx.retry_rates[4]);
    }
    else
    {
        /* Set the rate set words with the same rate as the current transmit */
        /* rate if retry auto rate is not enabled                            */
        update_retry_rate_set(0, tx_rate, 0, retry_set);
    }

    set_tx_params((UWORD8 *)g_test_config.tx.tx_dscr_addr, tx_rate, pream,
                 ack_policy, phy_tx_mode, retry_set);
    set_tx_dscr_q_num((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                      temp->priority);

    /* Enable TXOP protection for testing DLS protection */
    if(g_test_config.tx.dls_prot == 1)
        set_tx_dscr_op_flags((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                             TXOP_PROT_ENABLE_OP_FLAG);

#ifdef MAC_802_11N
    /* Set the SMPS mode field in the Tx descriptor */
    set_tx_dscr_smps_mode((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                          g_test_config.ht.da_smps_mode);
#else /* MAC_802_11N */
    /* Set the SMPS mode field in the Tx descriptor */
    set_tx_dscr_smps_mode((UWORD32 *)g_test_config.tx.tx_dscr_addr, 0);
#endif /* MAC_802_11N */

    /* Set the mac header related parameters in the Tx descriptor */
    mac_hdr_len = get_test_mac_hdr_len();
    set_tx_dscr_buffer_addr((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                            frame_addr);
    set_tx_dscr_mh_len((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                        mac_hdr_len);
    set_tx_dscr_mh_offset((UWORD32 *)g_test_config.tx.tx_dscr_addr, 0);

#ifdef MAC_802_11N
    /* Check whether multiple entries should be made in the sub-MSDU table */
    if(2 == g_test_config.ht.amsdu_mode)
    {
        UWORD16 offset      = 0;
        UWORD16 submsdu_idx = 0;

        buffer_desc.buff_hdl = (UWORD8 *)(frame_addr);
        offset               = mac_hdr_len;

        for(submsdu_idx = 0; submsdu_idx < temp->num_submsdu; submsdu_idx++)
        {
            if(submsdu_idx == temp->num_submsdu - 1)
            {
                buffer_desc.data_length = get_submsdu_len(
                                (UWORD8 *)(frame_addr + offset)) +
                                SUB_MSDU_HEADER_LENGTH;
#ifdef MAC_802_11I
     /* Update the length of the last Sub-MSDU for the Security trailer */
        if((g_test_config.ce.ce_enable == 1) && (g_test_config.ce.ct == TKIP))
             buffer_desc.data_length += 8;
#endif /* MAC_802_11I */

#ifndef DEBUG_KLUDGE
            /* This Kludge fixes the TX-Hang issue in MAC H/w */
            buffer_desc.data_length += 4;
#endif /* DEBUG_KLUDGE */

            }
            else
                buffer_desc.data_length = get_submsdu_frame_len(
                                                (UWORD8 *)(frame_addr + offset));


            buffer_desc.data_offset = offset;
            /* Set the data buffer related information in the Tx-descriptor */
            set_tx_dscr_submsdu_buff_info((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                                          (UWORD32)(&buffer_desc),
                                          submsdu_idx);

            offset += buffer_desc.data_length;
#ifdef MAC_802_11I
            /* For TKIP the statergy of inserting Sub-MSDU spacing corrupts */
            /* MIC computation.                                             */
            if(g_test_config.ce.ct != TKIP)
#endif /* MAC_802_11I */
                offset += SUBMSDU_SPACING;
        }
    }
    else
#endif /* MAC_802_11N */
    {
        /* Create the Payload Buffer descriptor */
        buffer_desc.buff_hdl    = (UWORD8 *)(frame_addr);
        buffer_desc.data_offset = mac_hdr_len;
        buffer_desc.data_length = temp->cur_frame_len - mac_hdr_len - FCS_LEN;

#ifndef DEBUG_KLUDGE
        /* This Kludge fixes the TX-Hang issue in MAC H/w */
        buffer_desc.data_length += 4;
#endif /* DEBUG_KLUDGE */

        /* Set the data buffer related information in the Tx-descriptor */
        set_tx_dscr_submsdu_buff_info((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                                      (UWORD32)(&buffer_desc),
                                      0);
    }

    /* Update the PHY-Test related paarmeters within the frame */
    /* If PHY PER-TX test enabled, then update the sequence number in the body */
    if(g_test_config.phy.mode == PHY_TEST_PER_TX_MODE)
    {
        UWORD8 *mbsa = (UWORD8 *)frame_addr + mac_hdr_len;

        if(g_test_config.ht.amsdu_mode != 0)
            mbsa += SUB_MSDU_HEADER_LENGTH;

        insert_hut_seqno_rate(mbsa, g_phy_test_idx);

        /* This is just one of the many configurations possible for PER-TX   */
        /* In this, the Txr cycles through the configured list of MCS values */
        /* and transmits one frame for each MCS.                             */
        if(++g_phy_test_idx > g_phy_test_idx_hi)
            g_phy_test_idx = g_phy_test_idx_lo;

        /* Update preamble according to Tx rate */
        set_preamble(g_tx_rate_table[g_phy_test_idx].preamble);
    }

    /* Set the frame length in the Tx Descriptor */
    set_tx_dscr_frame_len((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                          temp->cur_frame_len);

    set_tx_dscr_num_submsdu((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                            temp->num_submsdu);
    set_tx_dscr_frag_len((UWORD32 *)g_test_config.tx.tx_dscr_addr,
                         temp->cur_frame_len, 0);

    /* Set RA-LUT Index for Unicast frames */
    if(bcmc_type != BTRUE)
    {
        tid = test_qnum_to_tid(temp->priority);
        for(idx = 0; idx < g_test_config.ht.ampdu_num_sessions; idx++)
            if(g_test_config.ht.ampdu_sess_params[idx].tid == tid)
                break;

        if(idx == g_test_config.ht.ampdu_num_sessions)
            idx = 0;
        else
            idx++;
    }
    else
        idx = 0;

    set_tx_dscr_ra_lut_index((UWORD32 *)g_test_config.tx.tx_dscr_addr, idx);

    /* Update the TSF timestamp */
    update_tx_dscr_tsf_ts((UWORD32 *)g_test_config.tx.tx_dscr_addr);

    /* Set the MAC H/w queue pointers based on configuration */
    if(qmu_add_tx_packet(&g_q_handle.tx_handle, temp->priority,
                         (UWORD8 *)g_test_config.tx.tx_dscr_addr) != QMU_OK)
    {
        /* Exception. Do nothing. */
    }
    else
    {
        /* This way of S/w tracking the sequence number might not be */
        /* accurate. It is used since it is not considered crucial.  */
        if(bcmc_type != BTRUE)
            temp->seq_num++;
    }

    /* Update the frame length for the current MSDU */
    update_tx_frame_length(temp);
}


/* This function checks and transmits a control frame at the correct instance */
UWORD8 check_and_send_ctrl_frames(q_info_t *qinfo)
{
    UWORD8 retval = 0;
#ifdef MAC_802_11N
    UWORD8 tid          = 0;
    UWORD8 idx          = 0;
    ba_lut_struct_t *ba = NULL;

    if(g_test_config.qos.ba_enable == 0)
        return 0;

    tid = test_qnum_to_tid(qinfo->priority);

    /* Check whether a Tx Block-Ack Session is setup for this TID */
    for(idx = 0; idx < g_test_config.qos.ba_num_sessions; idx++)
        if((g_test_config.qos.ba_lut_entry[idx].tid == tid) &&
           (g_test_config.qos.ba_lut_entry[idx].dir == 1))
        {
            break;
        }

    if(idx == g_test_config.qos.ba_num_sessions)
        return 0;

    ba = &g_test_config.qos.ba_lut_entry[idx];

    /* Simply transmit BAR after every Win_Size Number of frames */
    if(qinfo->seq_num % ba->win_size == (ba->win_size - 1))
    {
        //UWORD16 ssn = (qinfo->seq_num - (ba->win_size - 1)) & 0x0FFF;

        //update_test_bar_ssn((UWORD8 *)ba->bar_addr, ssn);

        send_test_bar(ba, qinfo->priority);
    }
#endif /* MAC_802_11N */
    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_test_mode                                      */
/*                                                                           */
/*  Description   : This function does the necessary initializations for the */
/*                  chosen test mode.                                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_test_mode(void)
{
    if(g_test_config.oper_mode != 0)
    {
        /* If the operating mode is not DUT, then disable all tests */
        g_test_config.tx.tx_test   = 0;
        g_test_config.tsf.tsf_test = 0;

#ifdef MAC_802_11N
        g_test_config.qos.ba_enable        = 0;
        g_test_config.qos.ba_lut_prog_test = 0;
        g_test_config.ht.ampdu_lut_prog_test = 0;
#endif /* MAC_802_11N */

        g_test_config.phy.mode = PHY_TEST_DISABLED;
    }

    if(g_test_config.oper_mode == 1)
    {
        /* Configure for Frame Inject Mode - Enable all interrupts */
        g_test_config.intr.rxcomplete = 1;
        g_test_config.intr.txcomplete = 1;
        g_test_config.intr.tbtt       = 1;
        g_test_config.intr.cape       = 1;
        g_test_config.intr.hprx_comp  = 1;
        g_test_config.intr.radar_det  = 1;
        g_test_config.intr.deauth     = 1;
        g_test_config.intr.txsus      = 1;

        /* Disable Sequence Number insertion by h/w */
        disable_machw_seq_num_insertion();

        /* Initialize the TID to Q-num mapping table */
        init_hut_txq_pri_table();
    }
    else if(g_test_config.oper_mode == 2)
    {
        /* Configure for Monitor Mode - Clear all frame filters */
        g_test_config.rx.ack_filter            = 0;
        g_test_config.rx.cts_filter            = 0;
        g_test_config.rx.rts_filter            = 0;
        g_test_config.rx.bcn_filter            = 0;
        g_test_config.rx.atim_filter           = 0;
        g_test_config.rx.cf_end_filter         = 0;
        g_test_config.rx.qcf_poll_filter       = 0;
        g_test_config.rx.dup_filter            = 0;
        g_test_config.rx.fcs_filter            = 0;
        g_test_config.rx.non_dir_data_filter   = 0;
        g_test_config.rx.non_dir_mgmt_filter   = 0;
        g_test_config.rx.non_dir_ctrl_filter   = 0;
        g_test_config.rx.bcmc_mgmt_obss_filter = 0;
        g_test_config.rx.bcmc_data_obss_filter = 0;
        g_test_config.rx.exp_ba_filter         = 0;
        g_test_config.rx.unexp_ba_filter       = 0;
        g_test_config.rx.deauth_filter         = 0;
        g_test_config.rx.sec_chan_filter       = 0;
        g_test_config.rx.disc_ibss_bcn_filter  = 0;
    }

    /* PHY test is run when S/w is in DUT mode. Configure the same */
    configure_phy_test();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_phy_test                                       */
/*                                                                           */
/*  Description   : This function configures the PHY tests based on the PHY  */
/*                  test mode setting.                                       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_phy_test(void)
{
    UWORD8 mode = g_test_config.phy.mode;

    switch(mode)
    {
    case PHY_TEST_PER_TX_MODE:
    {
        /* In PER-TX mode, enqueue only Normal/AC-VO frames, one at a time */
        /* This is very crucial for Test Frame sequence number insertion   */
        /* logic to work.                                                  */
#ifndef MAC_WMM
        g_test_config.tx.nor.init_num_pkt = 1;
        g_test_config.tx.cf.init_num_pkt  = 0;
        g_test_config.tx.hp.init_num_pkt  = 0;
#else /* MAC_WMM */
        if(g_test_config.tx.ac_vo.init_num_pkt > 0)
            g_test_config.tx.num_pkts += g_test_config.tx.ac_vo.init_num_pkt - 1;

        g_test_config.tx.ac_be.init_num_pkt = 0;
        g_test_config.tx.ac_bk.init_num_pkt = 0;
        g_test_config.tx.ac_vo.init_num_pkt = 1;
        g_test_config.tx.ac_vi.init_num_pkt = 0;
        g_test_config.tx.hp.init_num_pkt    = 0;
#endif /* MAC_WMM */

        /* Select Non-HT rate table according to configured non-HT mode */
        if(get_running_mode() == G_ONLY_MODE)
        {
            g_tx_rate_table   = g_total_non_ht_gonly_rates;
            g_num_nonht_rates = NUM_DR_PHY_802_11G_ONLY;
        }
        else
        {
            g_tx_rate_table   = g_total_non_ht_gmixed_rates;
            g_num_nonht_rates = NUM_NONHT_LONG_SHORT_RATES;
        }

        /* Convert rate parameters to indices within sequence number table */
        if(g_test_params.tx_format == 0)
        {
            UWORD8 idx = 0;

            /* Initialize to default values */
            g_phy_test_idx_lo = 255;
            g_phy_test_idx_hi = 255;

            for(idx = 0; idx < g_num_nonht_rates; idx++)
            {
                UWORD8 rate = get_frm_stat_tx_rate(idx);

                if(rate == g_test_config.phy.rate_hi)
                    g_phy_test_idx_hi = idx;

                if((rate == g_test_config.phy.rate_lo) && (g_phy_test_idx_lo == 255))
                    g_phy_test_idx_lo = idx;
            }

            if(g_phy_test_idx_lo == 255)
                g_phy_test_idx_lo = 0;

            if(g_phy_test_idx_hi == 255)
                g_phy_test_idx_hi = 0;

            if(g_phy_test_idx_lo > g_phy_test_idx_hi)
            {
                g_phy_test_idx_lo = 0;
                g_phy_test_idx_hi = 0;
            }
        }
        else
        {
            /* Sanity check on the input values */
            if(g_test_config.phy.rate_hi > NUM_HT_RATES)
                g_test_config.phy.rate_hi = g_test_params.tx_mcs;

            if(g_test_config.phy.rate_lo > NUM_HT_RATES)
                g_test_config.phy.rate_lo = g_test_params.tx_mcs;

            g_phy_test_idx_hi = g_test_config.phy.rate_hi + g_num_nonht_rates;
            g_phy_test_idx_lo = g_test_config.phy.rate_lo + g_num_nonht_rates;

        }

        /* Sanity under erroneous input */
        if(g_phy_test_idx_hi < g_phy_test_idx_lo)
            g_phy_test_idx_hi = g_phy_test_idx_lo;

        g_phy_test_idx = g_phy_test_idx_lo;
    }
    break;
    case PHY_TEST_PER_RX_MODE:
    {
        /* Disable all transmissions in PER-Rx mode */
        g_test_config.tx.tx_test   = 0;
    }
    break;
    case PHY_TEST_BURST_TX_MODE:
    {
        UWORD32 init_num_pkts = 5;

        /* Settings to facilitate Spectral Mask measurement.         */
        /* This requires MAC to transmit frames with a low, constant */
        /* inter-packet delay.                                       */
        /* CWmin = 0, CWmax = 0, AIFSN = 1(PIFS Txn), TXOP-Limit = 0 */
#ifndef MAC_WMM
        /* Enqueue a substantial number of frames initially */
        g_test_config.tx.nor.init_num_pkt = 5;
        g_test_config.tx.cf.init_num_pkt  = 0;
        g_test_config.tx.hp.init_num_pkt  = 0;

        /* Set parameters to ensure continuous transmission of frames */
        /* with a low and constant delay.                             */
        set_machw_cw(0, 0);
        set_machw_aifsn();
#else /* MAC_WMM */
        /* Adjustments done to ensure that the configured number of */
        /* packets are correctly txd in burst TX Mode.              */
        if(g_test_config.tx.ac_vo.init_num_pkt >= init_num_pkts)
        {
            g_test_config.tx.num_pkts +=
                    g_test_config.tx.ac_vo.init_num_pkt - init_num_pkts;
        }
        else
        {
            if(g_test_config.tx.num_pkts >=
                    (init_num_pkts - g_test_config.tx.ac_vo.init_num_pkt))
            {
                g_test_config.tx.num_pkts -=
                     (init_num_pkts - g_test_config.tx.ac_vo.init_num_pkt);
            }
            else
                g_test_config.tx.num_pkts = 0;
        }

        g_test_config.tx.ac_be.init_num_pkt = 0;
        g_test_config.tx.ac_bk.init_num_pkt = 0;
        g_test_config.tx.ac_vo.init_num_pkt = init_num_pkts;
        g_test_config.tx.ac_vi.init_num_pkt = 0;
        g_test_config.tx.hp.init_num_pkt    = 0;

        /* Set parameters to ensure continuous transmission of frames */
        /* with a low and constant delay.                             */
        set_machw_cw_vo(0, 0);
        set_machw_aifsn_all_ac(1, 1, 1, 1);
        set_machw_txop_limit_bkbe(0, 0);
        set_machw_txop_limit_vovi(0, 0);

        /* Bypass CCA in order to enable continuous transmission required   */
        /* for spectral mask measurement                                    */
        bypass_cca();
#endif /* MAC_WMM */
    }
    break;
    case PHY_TEST_CONTINUOUS_TX_MODE_0:
    case PHY_TEST_CONTINUOUS_TX_MODE_1:
    case PHY_TEST_CONTINUOUS_TX_MODE_2:
    case PHY_TEST_CONTINUOUS_TX_MODE_3:
    {
    }
    break;
    case PHY_TEST_NO_MODULATION_MODE:
    {
    }
    break;
    default:
    {
    }
    break;
    }
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : init_vppp_struct                                         */
/*                                                                           */
/*  Description   : This function initializes the tables in the VPPP         */
/*                  Structures.                                              */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All elements of the VPPP Structure is initialized to the */
/*                  default values the first time. On subsequent resets,     */
/*                  only the Current Value Index is reset.                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void init_vppp_struct(void)
{
    var_param_t *param = NULL;
    UWORD8      mcs    = 0;

    if(g_vppp_struct_init_done == BFALSE)
    {
        g_vppp_struct_init_done = BTRUE;

        /* Initialize TX-Frame Format Table */
        param = &g_test_config.tx.vppp.txfmt;
        param->var_type = VARY_PARAM_ROUND_ROBIN;
        param->max_idx  = NUM_VALID_FRAME_FMT - 1;
        param->num_val  = NUM_VALID_FRAME_FMT;
        param->min_idx  = 0;
        param->cur_idx  = param->min_idx;
        param->var_val  = g_vppp_allowed_txfmt;

        /* Initialize TX-Rate/MCS Table */
        memcpy(g_vppp_enabled_rate_table, g_vppp_11b_rate_table, NUM_DR_PHY_802_11B);
        memcpy(g_vppp_enabled_rate_table + NUM_DR_PHY_802_11B,
               g_vppp_11a_rate_table,
               NUM_DR_PHY_802_11A);
        for(mcs = 0; mcs <= MAX_MCS_SUPPORTED; mcs++)
        {
            g_vppp_enabled_mcs_table[mcs] = mcs;
            /* BIT6 Set in MCS value is a custom method to differentiate */
            /* Short-GI from Long-GI. This is used only in VPPP tables   */
            g_vppp_enabled_mcs_table[mcs + MAX_MCS_SUPPORTED + 1] =
                                              (mcs | VPPP_SHORT_GI_ENABLE_BIT);
        }

        param = &g_test_config.tx.vppp.rate_11b;
        param->var_type = VARY_PARAM_ROUND_ROBIN;
        param->max_idx  = NUM_DR_PHY_802_11B - 1;
        param->num_val  = NUM_DR_PHY_802_11B;
        param->min_idx  = 0;
        param->cur_idx  = param->min_idx;
        param->var_val  = g_vppp_enabled_rate_table;

        param = &g_test_config.tx.vppp.rate_11a;
        param->var_type = VARY_PARAM_ROUND_ROBIN;
        param->max_idx  = NUM_DR_PHY_802_11A - 1;
        param->num_val  = NUM_DR_PHY_802_11A;
        param->min_idx  = 0;
        param->cur_idx  = param->min_idx;
        param->var_val  = g_vppp_enabled_rate_table + NUM_DR_PHY_802_11B;

        param = &g_test_config.tx.vppp.rate_11g;
        param->var_type = VARY_PARAM_ROUND_ROBIN;
        param->max_idx  = NUM_DR_PHY_802_11G - 1;
        param->num_val  = NUM_DR_PHY_802_11G;
        param->min_idx  = 0;
        param->cur_idx  = param->min_idx;
        param->var_val  = g_vppp_enabled_rate_table;

        param = &g_test_config.tx.vppp.rate_11n;
        param->var_type = VARY_PARAM_ROUND_ROBIN;
        param->max_idx  = 2*MAX_MCS_SUPPORTED + 1;
        param->num_val  = 2*MAX_MCS_SUPPORTED + 2;;
        param->min_idx  = 0;
        param->cur_idx  = param->min_idx;
        param->var_val  = g_vppp_enabled_mcs_table;

        /* Initialize TX-Length Table */
        param = &g_test_config.tx.vppp.length;
        param->var_type = VARY_PARAM_ROUND_ROBIN;
        param->max_idx  = MAX_VPPP_TX_FRAME_LEN;
        param->min_idx  = MIN_VPPP_TX_FRAME_LEN;
        param->num_val  = (MAX_VPPP_TX_FRAME_LEN - MIN_VPPP_TX_FRAME_LEN + 1);
        param->cur_idx  = param->min_idx;
        param->var_val  = NULL;
    }
    else
    {
        /* Reset TX-Frame Format Index */
        param = &g_test_config.tx.vppp.txfmt;
        param->cur_idx  = param->min_idx;

        /* Reset TX-Rate Index */
        param = &g_test_config.tx.vppp.rate_11b;
        param->cur_idx  = param->min_idx;
        param = &g_test_config.tx.vppp.rate_11a;
        param->cur_idx  = param->min_idx;
        param = &g_test_config.tx.vppp.rate_11g;
        param->cur_idx  = param->min_idx;
        param = &g_test_config.tx.vppp.rate_11n;
        param->cur_idx  = param->min_idx;

        /* Reset TX-Length Index */
        param = &g_test_config.tx.vppp.length;
        param->cur_idx  = param->min_idx;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_vppp_txfmt_table                                  */
/*                                                                           */
/*  Description   : This function generates tx-format elements and updates   */
/*                  the TX-FMT VPPP Structure based upon the formats chosen  */
/*                  by the user.                                             */
/*                                                                           */
/*  Inputs        : frmfmt - Frame Format for which the Tx-Format is         */
/*                           generated (11b,11a etc)                         */
/*                  bwofst - Code for the pair of PPDU Spectrum Mask Width   */
/*                           (20/40) and Channel Bandwidth i/p by user       */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_vppp_txfmt_table(UWORD8 frmfmt, UWORD8 bwofst)
{
    var_param_t *param = &g_test_config.tx.vppp.txfmt;

    if(bwofst & BIT0)
    {
        param->var_val[param->num_val] = (CH_OFF_NONE << 5) |
                                            (CH_BW_20MHZ << 3) | (frmfmt & 0x7);
        param->num_val++;
    }
    if(bwofst & BIT1)
    {
        param->var_val[param->num_val] = (CH_OFF_20U << 5) |
                                            (CH_BW_40MHZ << 3) | (frmfmt & 0x7);
        param->num_val++;
    }
    if(bwofst & BIT2)
    {
        param->var_val[param->num_val] = (CH_OFF_20L << 5) |
                                            (CH_BW_40MHZ << 3) | (frmfmt & 0x7);
        param->num_val++;
    }
    if(bwofst & BIT3)
    {
        param->var_val[param->num_val] = (CH_OFF_40 << 5) |
                                            (CH_BW_40MHZ << 3) | (frmfmt & 0x7);
        param->num_val++;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_vppp_txfmt_table                                  */
/*                                                                           */
/*  Description   : This function parses the Config Frame sent by the user   */
/*                  and updates the corresponding VPPP Structures            */
/*                                                                           */
/*  Inputs        : val - Pointer to start of VPPP config information        */
/*                  len - Length of VPPP Config information                  */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void parse_vppp_config_frame(UWORD8 *val, UWORD8 len)
{
    UWORD8      i     = 0;
    UWORD8      idx   = 0;
    UWORD8      plen  = 0;
    UWORD8      ptype = 0;
    UWORD8      num_11b_rates = 0;
    UWORD8      num_11a_rates = 0;
    var_param_t *param = NULL;
    g_test_config.tx.vppp.control= val[idx];
    idx++;

    while(idx < len)
    {
        ptype = val[idx];
        plen  = val[idx+1];
        idx  += 2;

        switch(ptype)
        {
            case VARY_FRAME_FORMAT:
                param = &g_test_config.tx.vppp.txfmt;
                param->min_idx = 0;
                param->num_val = 0;
                param->max_idx = 0;
                param->cur_idx = param->min_idx;
                param->var_val = g_vppp_allowed_txfmt;

                param->var_type = val[idx];
                /* 11b-Short Preamble */
                /* 40MHz Bandwidth is not valid in 11b mode */
                val[idx+1] &= ~ BIT3;
                update_vppp_txfmt_table(0, val[idx+1]);

                /* 11b-Long Preamble */
                /* 40MHz Bandwidth is not valid in 11b mode */
                val[idx+2] &= ~ BIT3;
                update_vppp_txfmt_table(1, val[idx+2]);

                /* 11a Mode */
                update_vppp_txfmt_table(2, val[idx+3]);

                /* 11n-Mixed Mode */
                update_vppp_txfmt_table(6, val[idx+4]);

                /* 11n-Greenfield Mode */
                update_vppp_txfmt_table(7, val[idx+5]);

                if(param->num_val == 0)
                {
                    /* No TX-Formats are enabled. Disable TX-Fmt Variation */
                    g_test_config.tx.vppp.control &= ~VARY_FRAME_FORMAT;
                }
                else
                {
                    param->max_idx = (param->num_val - 1);
                }
                break;
            case VARY_FRAME_LENGTH:
                param = &g_test_config.tx.vppp.length;

                param->var_type = val[idx];

                param->min_idx = MAKE_WORD16(val[idx+1], val[idx+2]);
                if(param->min_idx < MIN_VPPP_TX_FRAME_LEN)
                    param->min_idx = MIN_VPPP_TX_FRAME_LEN;

                param->max_idx = MAKE_WORD16(val[idx+3], val[idx+4]);
                if((param->max_idx > MAX_VPPP_TX_FRAME_LEN) ||
                   (param->max_idx < param->min_idx))
                {
                    param->max_idx  = MAX_VPPP_TX_FRAME_LEN;
                }

                param->cur_idx  = param->min_idx;
                param->num_val = (param->max_idx - param->min_idx + 1);
                break;
            case VARY_FRAME_RATE:
                /* Parse 11b Rates */
                param = &g_test_config.tx.vppp.rate_11b;
                param->var_type = val[idx];
                param->min_idx = 0;
                param->num_val = 0;
                param->cur_idx = param->min_idx;
                param->var_val = g_vppp_enabled_rate_table;

                for(i = 0; i < NUM_DR_PHY_802_11B; i++)
                {
                    if(val[idx+1] & (1 << i))
                    {
                        param->var_val[param->num_val] = g_vppp_11b_rate_table[i];
                        param->num_val++;
                    }
                }
                if(param->num_val)
                    param->max_idx = param->num_val - 1;
                num_11b_rates = param->num_val;

                /* Parse 11a Rate */
                param = &g_test_config.tx.vppp.rate_11a;
                param->var_type = val[idx];
                param->min_idx = 0;
                param->num_val = 0;
                param->cur_idx = param->min_idx;
                param->var_val = g_vppp_enabled_rate_table + num_11b_rates;

                for(i = 0; i < NUM_DR_PHY_802_11A; i++)
                {
                    if(val[idx+2] & (1 << i))
                    {
                        param->var_val[param->num_val] = g_vppp_11a_rate_table[i];
                        param->num_val++;
                    }
                }
                if(param->num_val)
                    param->max_idx = param->num_val - 1;
                num_11a_rates = param->num_val;

                /* Merge 11b & 11a Rates to generate 11g rates */
                param = &g_test_config.tx.vppp.rate_11g;
                param->var_type = val[idx];
                param->min_idx = 0;
                param->num_val = num_11a_rates + num_11b_rates;
                param->max_idx = param->num_val - 1;
                param->cur_idx = param->min_idx;
                param->var_val = g_vppp_enabled_rate_table;

                /* Parse 11n Rates */
                param = &g_test_config.tx.vppp.rate_11n;
                param->var_type = val[idx];
                param->min_idx = 0;
                param->num_val = 0;
                param->cur_idx = param->min_idx;
                param->var_val = g_vppp_enabled_mcs_table;

                /* Extract Long-GI MCS */
                for(i = 0; i <= MAX_MCS_SUPPORTED; i++)
                {
                    UWORD8 byt_ofst = (i >> 3);
                    UWORD8 bit_ofst = (1 << (7 - (i & 7)));

                    if(val[idx+3+byt_ofst] & bit_ofst)
                    {
                        param->var_val[param->num_val] = i;
                        param->num_val++;
                    }
                }
                /* Extract Short-GI MCS */
                for(i = 0; i <= MAX_MCS_SUPPORTED; i++)
                {
                    UWORD8 byt_ofst = (i >> 3);
                    UWORD8 bit_ofst = (1 << (7 - (i & 7)));

                    if(val[idx+7+byt_ofst] & bit_ofst)
                    {
                        param->var_val[param->num_val] =
                                                (i | VPPP_SHORT_GI_ENABLE_BIT);
                        param->num_val++;
                    }
                }
                if(param->num_val)
                    param->max_idx = param->num_val - 1;

                break;
            default:
                PRINTD("Err:Unrecognized VPPP-Type=%d\n\r",ptype);
                break;
        }

        idx += plen;
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_vppp_txfmt_table                                  */
/*                                                                           */
/*  Description   :  This function returns the next element from the VPPP    */
/*                   list. The value returned depends upon the type of       */
/*                   variation that is defined for the VPPP structure. If    */
/*                   the VPPP List is not defined, then the index is         */
/*                   returned.                                               */
/*                                                                           */
/*                                                                           */
/*  Inputs        : Pointer to VPPP Info Structure                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 get_next_vppp_value(var_param_t *param)
{
    UWORD32 retval = 0;
    UWORD32 idx    = 0;

    if(param->var_type == VARY_PARAM_RANDOM)
    {
        /* Generate a random index between Max_Idx & Min_Idx */
        idx  = get_random_in_range(param->num_val);
        idx += param->min_idx;
    }
    else
    {
        /* Default is VARY_PARAM_ROUND_ROBIN */
        idx = param->cur_idx;

        param->cur_idx++;
        if(param->cur_idx > param->max_idx)
            param->cur_idx = param->min_idx;
    }

    if(param->var_val == NULL)
    {
        retval = idx;
    }
    else
    {
        retval = param->var_val[idx];
    }

    return retval;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_vpp_params                                        */
/*                                                                           */
/*  Description   : This function updates the TX-Parameters which can be     */
/*                  varied on per-packet basis based on the settings in the  */
/*                  VPPP structure.                                          */
/*                                                                           */
/*  Inputs        : Pointer to Queue Info Structure                          */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The following elements can be varied depending upon the  */
/*                  settings in the VPPP structure:                          */
/*                  g_test_params.tx_format                                  */
/*                  g_test_params.bandwidth                                  */
/*                  g_test_params.op_band                                    */
/*                  g_test_params.shortgi                                    */
/*                  g_test_params.preamble                                   */
/*                  g_test_params.tx_mcs                                     */
/*                  g_test_params.tx_rate                                    */
/*                  q_info->cur_frame_len                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_vpp_params(q_info_t *q_info)
{
    UWORD8      txfmt = 0;
    UWORD8      frmfmt = 0;
    var_param_t *param = NULL;

    if(g_test_config.tx.vppp.control & VARY_FRAME_FORMAT)
    {
        txfmt = get_next_vppp_value(&g_test_config.tx.vppp.txfmt);

        /* Interpret the TX-Format and Update the relevant settings            */
        /* Format: b6-b5[ChannelBW]:b4-b3[ChannelFilter]:b2-b0[PHYFrameFormat] */
        frmfmt                  = (txfmt >> 0) & 0x7;
        g_test_params.bandwidth = (txfmt >> 3) & 0x3;
        g_test_params.op_band   = (txfmt >> 5) & 0x3;

        if(frmfmt == 0)
        {
            /* 11b Short Preamble */
            g_test_params.tx_format = 0;
            g_test_params.preamble  = 0;
            param = &g_test_config.tx.vppp.rate_11b;
            g_test_params.tx_rate = 11;
            g_test_params.tx_mcs  = INVALID_MCS_VALUE;
        }
        else if(frmfmt == 1)
        {
            /* 11b Long Preamble */
            g_test_params.tx_format = 0;
            g_test_params.preamble  = 1;
            param  = &g_test_config.tx.vppp.rate_11b;
            g_test_params.tx_rate = 11;
            g_test_params.tx_mcs  = INVALID_MCS_VALUE;
        }
        else if(frmfmt == 6)
        {
            /* HT Mixed Mode Frame */
            g_test_params.tx_format = 2;
            g_test_params.preamble  = 0;
            param  = &g_test_config.tx.vppp.rate_11n;
            g_test_params.tx_rate = 24;
            g_test_params.tx_mcs  = MAX_NUM_MCS_SUPPORTED-1;
        }
        else if(frmfmt == 7)
        {
            /* HT Green Field Frame */
            g_test_params.tx_format = 1;
            g_test_params.preamble  = 0;
            param = &g_test_config.tx.vppp.rate_11n;
            g_test_params.tx_rate = 24;
            g_test_params.tx_mcs  = MAX_NUM_MCS_SUPPORTED-1;
        }
        else
        {
            /* Default: 11a Frame Format */
            g_test_params.tx_format = 0;
            g_test_params.preamble  = 0;
            param  = &g_test_config.tx.vppp.rate_11a;
            g_test_params.tx_rate = 54;
            g_test_params.tx_mcs  = INVALID_MCS_VALUE;
        }

        if((g_test_config.tx.vppp.control & VARY_FRAME_RATE) == 0)
        {
            set_curr_tx_mcs(g_test_params.tx_mcs);
            set_curr_tx_rate(g_test_params.tx_rate);
        }

        set_preamble(g_test_params.preamble);
    }

    if(g_test_config.tx.vppp.control & VARY_FRAME_RATE)
    {
        /* The Valid Value interpretation is done based on whether a Rate or MCS */
        /* is required. This is decided based on the TX-Format.                  */
        if(g_test_params.tx_format == 0)
        {
            if(param == NULL)
            {
                param = &g_test_config.tx.vppp.rate_11g;
            }

            if(param->num_val > 0)
                g_test_params.tx_rate = get_next_vppp_value(param);

            set_curr_tx_rate(g_test_params.tx_rate);
        }
        else
        {
            if(param == NULL)
            {
                param = &g_test_config.tx.vppp.rate_11n;
            }

            if(param->num_val > 0)
            {
                UWORD8 mcs = get_next_vppp_value(param);
                g_test_params.shortgi = ((mcs & VPPP_SHORT_GI_ENABLE_BIT) > 0);
                g_test_params.tx_mcs  = (mcs & ~VPPP_SHORT_GI_ENABLE_BIT);
            }

            set_curr_tx_mcs(g_test_params.tx_mcs);
        }
    }

    if(g_test_config.tx.vppp.control & VARY_FRAME_LENGTH)
    {
        /* No Frame length modification is done if AMSDU is enabled */
        if(g_test_config.ht.amsdu_mode  == 0)
        {
            q_info->cur_frame_len =
                            get_next_vppp_value(&g_test_config.tx.vppp.length);

            if(q_info->cur_frame_len < q_info->min_frame_len)
                q_info->cur_frame_len = q_info->min_frame_len;
        }
    }
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_operating_channel_hut                             */
/*                                                                           */
/*  Description   : This function updates the Secondary Channel Setting      */
/*                  based upon the current  settings of Primary Channel,     */
/*                  Secondary Channel and applicable Regulatory Domain.      */
/*                                                                           */
/*                       *                                                   */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks whether the Primary Channel and the */
/*                  Secondary Channel Offset are consistent with each other. */
/*                  If not, it disables the 40MHz bandwidth usage.           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void update_operating_channel_hut(void)
{
}

#endif /* MAC_HW_UNIT_TEST_MODE */
