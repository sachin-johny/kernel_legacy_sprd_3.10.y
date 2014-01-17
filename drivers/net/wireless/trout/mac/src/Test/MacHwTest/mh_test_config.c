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
/*  File Name         : mh_test_config.c                                     */
/*                                                                           */
/*  Description       : This file contains all the functions for unit test   */
/*                      of MAC H/w.                                          */
/*                                                                           */
/*  List of Functions : initialize_mh_test_config                            */
/*                      initialize_mh_test_params                            */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "mh.h"
#include "mh_test_config.h"
#include "transmit.h"
#include "qif.h"
#include "phy_hw_if.h"
#include "cglobals.h"
#include "qmu_if.h"

#ifdef MAC_802_11I
#include "management_11i.h"
#endif /* MAC_802_11I */

#ifdef MAC_WMM
#include "management_11e.h"
#include "frame_11e.h"
#endif /* MAC_WMM */

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

test_params_struct_t     g_test_params;
test_config_struct_t     g_test_config;
BOOL_T                   g_test_start = BFALSE;
UWORD8                   g_tx_pattern[MAX_PATTERN_LEN];
UWORD16                  g_tx_pattern_len  = 0;
UWORD8                   g_disable_rxq_replenish = 0;
UWORD8                   g_phy_test_rate_hi = 0;
UWORD8                   g_phy_test_rate_lo = 0;
UWORD8                   g_test_id[256] = "Test Statistics";
UWORD8                   g_enable_frame_len_modification = 0;
UWORD8                   g_stats_buf[MAX_STATS_BUFFER_LEN];
UWORD16                  g_sig_qual_avg     = 0;
UWORD16                  g_sig_qual_avg_cnt = DEFAULT_SQ_AVG_CNT;
UWORD8                   g_last_pkt_tssi_value = 0;

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_mh_test_config                                */
/*                                                                           */
/*  Description   : This function initializes the test configuration. Note   */
/*                  that this function needs to be modified by the user to   */
/*                  configure the desired test case settings.                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function initializes the global test configuration  */
/*                  structure.                                               */
/*                                                                           */
/*  Outputs       : The global configuration structure is initialized.       */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_mh_test_config(void)
{
    /*************************************************************************/
    /*  MAC Test Software Operating Mode                                     */
    /*************************************************************************/

    /*  Operating Mode                                                       */
    /*  --------------------------------------------------------------       */
    /*  Configuration : 0           1          2                             */
    /*  Values to set : DUT  Frame-Injector  Monitor                         */
    /*  Note:                                                                */
    /*  0) In DUT Mode all the h/w unit tests are performed.                 */
    /*  1) In Frame-Injector Mode, all frames sent to the setup              */
    /*     through the host interface (except the config frames) are         */
    /*     transmitted on air.                                               */
    /*  2) In Monitor Mode, all received frames are passed to the host.      */
    /*  --------------------------------------------------------------       */
    g_test_config.oper_mode = 0;

    /*************************************************************************/
    /*  Interrupt configuration                                              */
    /*************************************************************************/

    /*  Interrupt Mask/Unmask                                                */
    /*  --------------------------------------------------------------       */
    /*  Configuration : 0           1                                        */
    /*  Values to set : Disable     Enable                                   */
    /*  --------------------------------------------------------------       */
    g_test_config.intr.rxcomplete = 1;
    g_test_config.intr.txcomplete = 1;
    g_test_config.intr.tbtt       = 1;
    g_test_config.intr.cape       = 1;
    g_test_config.intr.err        = 1;
    g_test_config.intr.hprx_comp  = 1;
    g_test_config.intr.radar_det  = 1;
    g_test_config.intr.deauth     = 1;
    g_test_config.intr.txsus      = 1;

    /*************************************************************************/
    /*  TSF Test configuration                                               */
    /*************************************************************************/

    /*  TSF Test type                                                        */
    /*  --------------------------------------------------------------       */
    /*  Configuration : 0           1           2          3                 */
    /*  Values to set : Disable     Join        Start      Beacon Suspend    */
    /*  --------------------------------------------------------------       */
    g_test_config.tsf.tsf_test = 0;

    /* Length of dummy beacon to be transmitted, rate of beacon transmission */
    g_test_config.tsf.bcn_len  = 74;
    g_test_config.tsf.bcn_rate = 6;

    /*  TSF Beacon swap test                                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration : 0           1                                        */
    /*  Values to set : Disable     Enable                                   */
    /*  --------------------------------------------------------------       */
    g_test_config.tsf.bcn_swap_test = 0;

#ifdef MWLAN
    /*************************************************************************/
    /*  RX Test configuration                                                */
    /*************************************************************************/

    /*  Number of receive buffers                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Linked list of receive buffers                      */
    /*  Values to set :  Number < 20                                         */
    /*  --------------------------------------------------------------       */
#ifndef PHY_TEST_MAX_PKT_RX
    /* For receiving maximum size of 64K A-MPDU number of buffers (each of   */
    /* size 4K) needs to be 16. Hence the queues is initialized to 32.       */
    g_test_config.rx.num_rx_buffers[NORMAL_PRI_RXQ] = 32;
#else /* PHY_TEST_MAX_PKT_RX */
    g_test_config.rx.num_rx_buffers[NORMAL_PRI_RXQ] = 10;
#endif /* PHY_TEST_MAX_PKT_RX */

    g_test_config.rx.num_rx_buffers[HIGH_PRI_RXQ]   = 5;

#endif /* MWLAN */


    /*  Frame filter test type                                               */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable     Disable  (frame types to filter)         */
    /*  Values to set :  1          0                                        */
    /*  --------------------------------------------------------------       */
    g_test_config.rx.ack_filter      = 0;
    g_test_config.rx.cts_filter            = 0;
    g_test_config.rx.rts_filter            = 0;
    g_test_config.rx.bcn_filter            = 0;
    g_test_config.rx.atim_filter     = 0;
    g_test_config.rx.cf_end_filter   = 0;
    g_test_config.rx.qcf_poll_filter       = 0;
    g_test_config.rx.dup_filter      = 0;
    g_test_config.rx.fcs_filter      = 0;
    g_test_config.rx.non_dir_data_filter   = 0;
    g_test_config.rx.non_dir_mgmt_filter   = 0;
    g_test_config.rx.non_dir_ctrl_filter   = 0;
    g_test_config.rx.bcmc_mgmt_obss_filter = 0;
    g_test_config.rx.bcmc_data_obss_filter = 0;
    g_test_config.rx.exp_ba_filter         = 0;
    g_test_config.rx.unexp_ba_filter       = 0;
    g_test_config.rx.deauth_filter   = 0;
    g_test_config.rx.sec_chan_filter       = 0;
    g_test_config.rx.disc_ibss_bcn_filter  = 0;

    /*  Hi Priority Rx Q test enable                                         */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable HP Rx Q    Disable HP Rx Q                   */
    /*  Values to set :  1                 0                                 */
    /*  --------------------------------------------------------------       */
    g_test_config.rx.hp_rxq_enable   = 0;

    /*  Hi Priority Rx Q test control                                        */
    /*  ------------------------------------------------------------------   */
    /*  Configuration :  Enable  Disable  (frame types to keep in HP Rx Q)   */
    /*  Values to set :  1       0                                           */
    /*  ------------------------------------------------------------------   */
    g_test_config.rx.hp_rxq_probereq = 0;
    g_test_config.rx.hp_rxq_probersp = 0;
    g_test_config.rx.hp_rxq_beacon   = 0;
    g_test_config.rx.hp_rxq_atim     = 0;

    /*  Soft CRC check test                                                  */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable Soft CRC check    Disable                    */
    /*  Values to set :  1                        0                          */
    /*  --------------------------------------------------------------       */
    g_test_config.rx.soft_crc_enable    = 0;

    /*  Soft Duplicate Frame Check Test                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable Soft Dup check    Disable                    */
    /*  Values to set :  1                        0                          */
    /*  --------------------------------------------------------------       */
    g_test_config.rx.soft_dupchk_enable = 0;

    /*  Soft Integrity Frame Check Test                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable Soft Int check    Disable                    */
    /*  Values to set :  1                         0                         */
    /*  --------------------------------------------------------------       */
    g_test_config.rx.soft_integrity_check = 0;

    /*  Soft TKIP MIC Verification                                           */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable TKIP-MIC Check     Disable                   */
    /*  Values to set :  1          0                                        */
    /*  --------------------------------------------------------------       */
    g_test_config.rx.tkip_mic_check       = 0;

    /*  Soft TKIP/CCMP Replay Detection                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable Replay Detection     Disable                 */
    /*  Values to set :  1                           0                       */
    /*  --------------------------------------------------------------       */
    g_test_config.rx.replay_detect       = 0;

    /*************************************************************************/
    /*  TX Test configuration                                                */
    /*************************************************************************/

    /*  TX test type                                                         */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.tx_test = 0;

    /*  Slot time                                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Long    Short                                       */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.slot = 1;

    /*  RTS/CTS exchange in TXOP with No ACK or Block ACK policy             */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Enable (default)   Disable                          */
    /*  Values to set :  0                  1                                */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.rtscts_txop = 0;

    /*  Frame type of the frame to be transmitted                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Frame type/subtype                                  */
    /*  Values to set :  DATA, AUTH, ASOC_REQ                                */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.frame_type   = DATA;

    if(g_reset_mac_in_progress == BFALSE)
    {
    /*  Enable retransmission auto rate                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.enable_retry_ar = 0;

    /*  Retransmission auto rate table                                       */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Rates to be used for retried frames                 */
    /*  Values to set :  Data rate for retry 1 to 8 in following format,     */
    /*                   00 - 1 Mbps                                         */
    /*                   01 - 2 Mbps                                         */
    /*                   02 - 5.5 Mbps                                       */
    /*                   03 - 11 Mbps                                        */
    /*                   0B - 6 Mbps                                         */
    /*                   0F - 9 Mbps                                         */
    /*                   0A - 12 Mbps                                        */
    /*                   0E - 18 Mbps                                        */
    /*                   09 - 24 Mbps                                        */
    /*                   0D - 36 Mbps                                        */
    /*                   08 - 48 Mbps                                        */
    /*                   0C - 54 Mbps                                        */
    /*                   8x - MCS x (x = 0 to F)                             */
    /*  Note that if retransmission auto rate is disabled (see above         */
    /*  configuration g_test_config.tx.enable_retry_ar) all the retry        */
    /*  rates will be set same as the transmission rate                      */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.retry_rates[0] = 0x00; /* Data Rate 1 */
    g_test_config.tx.retry_rates[1] = 0x00; /* Data Rate 2 */
    g_test_config.tx.retry_rates[2] = 0x00; /* Data Rate 3 */
    g_test_config.tx.retry_rates[3] = 0x00; /* Data Rate 4 */
    g_test_config.tx.retry_rates[4] = 0x00; /* Data Rate 5 */
    g_test_config.tx.retry_rates[5] = 0x00; /* Data Rate 6 */
    g_test_config.tx.retry_rates[6] = 0x00; /* Data Rate 7 */
    g_test_config.tx.retry_rates[7] = 0x00; /* Data Rate 8 */
    }

    /*  DLS protection for transmission                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable DLS protection  Enable DLS protection       */
    /*  Values to set :  0                       1                           */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.dls_prot = 0;

    /*  Destination address of the frame to be transmitted                   */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Destination address                                 */
    /*  Values to set :  48-bit MAC address                                  */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.da[0]     = 0x00;
    g_test_config.tx.da[1]     = 0x00;
    g_test_config.tx.da[2]     = 0x00;
    g_test_config.tx.da[3]     = 0x00;
    g_test_config.tx.da[4]     = 0x00;
    g_test_config.tx.da[5]     = 0x00;

    /*  Total Time for which the test be run                                 */
    /*  ----------------------------------------------------------------     */
    /*  Configuration :  Time (in secs) for which the test should be run     */
    /*                   num_pkts will be considered only if this is 0.      */
    /*  Values to set :  32-bit number                                       */
    /*  ----------------------------------------------------------------     */
    g_test_config.tx.test_time   = 0;

    /*  Number of packets to be transmitted from ISR                         */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Number of times transmission done from TxC          */
    /*                   Will take effect only when test_time = 0.           */
    /*  Values to set :  32-bit number                                       */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.num_pkts    = 0;

    /*  Percentage of Broadcast frames                                       */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Percentage of Broadcast frames among the total      */
    /*                   transmitted frames.                                 */
    /*  Values to set :  Any value between 0 (Only Unicast) to               */
    /*                   100 (Only Broadcast)                                */
    /*  --------------------------------------------------------------       */
    g_test_config.tx.bcst_percent   = 0;

    /*  Per Packet Parameter Variation. This feature varies the transmit     */
    /*  parameters of the packets for every frame.                           */
    /*  --------------------------------------------------------------       */
    /*  Config :  VARY_FRAME_FORMAT  VARY_FRAME_LENGTH  VARY_FRAME_RATE      */
    /*  Values :  Vary Frame Format  Vary Length        Vary Rate            */
    /*  --------------------------------------------------------------       */
    /* Note : Each of these can be independently enabled by ORing the values */
    if(g_reset_mac_in_progress == BFALSE)
        g_test_config.tx.vppp.control = 0;

#ifndef MAC_WMM

    /* Initially queued packets (MAX 5 per queue) */
    g_test_config.tx.nor.init_num_pkt = 0;
    g_test_config.tx.cf.init_num_pkt  = 0;
    g_test_config.tx.hp.init_num_pkt  = 0;

    /* Normal */
    g_test_config.tx.nor.frame_len  = 1500;
    g_test_config.tx.nor.priority   = NORMAL_PRI_Q;
    g_test_config.tx.nor.ack_policy = NORMAL_ACK;

    /* Contention Free */
    g_test_config.tx.cf.frame_len  = 1500;
    g_test_config.tx.cf.priority   = CF_PRI_Q;
    g_test_config.tx.cf.ack_policy = NORMAL_ACK;

    /* High Priority */
    g_test_config.tx.hp.frame_len  = 1500;
    g_test_config.tx.hp.priority   = HIGH_PRI_Q;
    g_test_config.tx.hp.ack_policy = NORMAL_ACK;

#else /* MAC_WMM */

    /* Initially queued packets (MAX 5 per queue) */
    g_test_config.tx.ac_be.init_num_pkt = 0;
    g_test_config.tx.ac_bk.init_num_pkt = 0;
    g_test_config.tx.ac_vo.init_num_pkt = 0;
    g_test_config.tx.ac_vi.init_num_pkt = 0;
    g_test_config.tx.hp.init_num_pkt    = 0;

    /* AC_BE */
    g_test_config.tx.ac_be.frame_len  = 1500;
    g_test_config.tx.ac_be.priority   = AC_BE_Q;
    g_test_config.tx.ac_be.ack_policy = NORMAL_ACK;

    /* AC_BK */
    g_test_config.tx.ac_bk.frame_len  = 1500;
    g_test_config.tx.ac_bk.priority   = AC_BK_Q;
    g_test_config.tx.ac_bk.ack_policy = NORMAL_ACK;

    /* AC_VO */
    g_test_config.tx.ac_vo.frame_len  = 1500;
    g_test_config.tx.ac_vo.priority   = AC_VO_Q;
    g_test_config.tx.ac_vo.ack_policy = NORMAL_ACK;

    /* AC_VI */
    g_test_config.tx.ac_vi.frame_len  = 1500;
    g_test_config.tx.ac_vi.priority   = AC_VI_Q;
    g_test_config.tx.ac_vi.ack_policy = NORMAL_ACK;

    /* HIGH_PRIORITY */
    g_test_config.tx.hp.frame_len  = 1500;
    g_test_config.tx.hp.priority   = HIGH_PRI_Q;
    g_test_config.tx.hp.ack_policy = NORMAL_ACK;

#endif /* MAC_WMM */

#ifdef MAC_802_11I
    /*************************************************************************/
    /* CE Test Configuration:                                                */
    /*************************************************************************/

    /*  CE Enable                                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.ce.ce_enable = 0;

    /*  Cipher Type                                                          */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  WEP40 TKIP CCMP WEP104 NO_ENCRYP                    */
    g_test_config.ce.ct        = NO_ENCRYP;

    /*  Broadcast Cipher Type                                                */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  WEP40 TKIP CCMP WEP104 NO_ENCRYP                    */
    g_test_config.ce.bcst_ct   = NO_ENCRYP;


    /*  KEY Origin (Valid in IBSS Mode Only)                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Authenticator  Supplicant                           */
    /*  Values to set :  AUTH_KEY       SUPP_KEY                             */
    /*  --------------------------------------------------------------       */
    g_test_config.ce.key_orig = AUTH_KEY;

#endif /* MAC_802_11I */

#ifdef MAC_802_11N
    /*************************************************************************/
    /* Block Ack Test Configuration                                          */
    /*************************************************************************/
    if(g_reset_mac_in_progress == BFALSE)
    {
    /*  BA LUT programming test Enable                                       */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.qos.ba_lut_prog_test = 0;

    /*  BA Enable                                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.qos.ba_enable   = 0;

    /*  BA Number of sessions                                                */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Number of test Block ACK sessions to set up         */
    /*  Values to set :  Maximum of 4                                        */
    /*  --------------------------------------------------------------       */
    g_test_config.qos.ba_num_sessions = 0;

    /*  BA LUT Entry 1 (NOTE: currently only compressed Block ACK supported) */
    g_test_config.qos.ba_lut_entry[0].dir           = 0;  /* 0=>Recipient 1=>Initiator */
    g_test_config.qos.ba_lut_entry[0].seq_num       = 0;
    g_test_config.qos.ba_lut_entry[0].win_size      = 0;
    g_test_config.qos.ba_lut_entry[0].tid           = 0;
    g_test_config.qos.ba_lut_entry[0].addr[0]       = 0x00;
    g_test_config.qos.ba_lut_entry[0].addr[1]       = 0x00;
    g_test_config.qos.ba_lut_entry[0].addr[2]       = 0x00;
    g_test_config.qos.ba_lut_entry[0].addr[3]       = 0x00;
    g_test_config.qos.ba_lut_entry[0].addr[4]       = 0x00;
    g_test_config.qos.ba_lut_entry[0].addr[5]       = 0x00;
    g_test_config.qos.ba_lut_entry[0].partial_state = 0;

    /*  BA LUT Entry 2 (NOTE: currently only compressed Block ACK supported) */
    g_test_config.qos.ba_lut_entry[1].dir           = 0;  /* 0=>Recipient 1=>Initiator */
    g_test_config.qos.ba_lut_entry[1].seq_num       = 0;
    g_test_config.qos.ba_lut_entry[1].win_size      = 0;
    g_test_config.qos.ba_lut_entry[1].tid           = 0;
    g_test_config.qos.ba_lut_entry[1].addr[0]       = 0x00;
    g_test_config.qos.ba_lut_entry[1].addr[1]       = 0x00;
    g_test_config.qos.ba_lut_entry[1].addr[2]       = 0x00;
    g_test_config.qos.ba_lut_entry[1].addr[3]       = 0x00;
    g_test_config.qos.ba_lut_entry[1].addr[4]       = 0x00;
    g_test_config.qos.ba_lut_entry[1].addr[5]       = 0x00;
    g_test_config.qos.ba_lut_entry[1].partial_state = 0;

    /*  BA LUT Entry 3 (NOTE: currently only compressed Block ACK supported) */
    g_test_config.qos.ba_lut_entry[2].dir           = 0;  /* 0=>Recipient 1=>Initiator */
    g_test_config.qos.ba_lut_entry[2].seq_num       = 0;
    g_test_config.qos.ba_lut_entry[2].win_size      = 0;
    g_test_config.qos.ba_lut_entry[2].tid           = 0;
    g_test_config.qos.ba_lut_entry[2].addr[0]       = 0x00;
    g_test_config.qos.ba_lut_entry[2].addr[1]       = 0x00;
    g_test_config.qos.ba_lut_entry[2].addr[2]       = 0x00;
    g_test_config.qos.ba_lut_entry[2].addr[3]       = 0x00;
    g_test_config.qos.ba_lut_entry[2].addr[4]       = 0x00;
    g_test_config.qos.ba_lut_entry[2].addr[5]       = 0x00;
    g_test_config.qos.ba_lut_entry[2].partial_state = 0;

    /*  BA LUT Entry 4 (NOTE: currently only compressed Block ACK supported) */
    g_test_config.qos.ba_lut_entry[3].dir           = 0;  /* 0=>Recipient 1=>Initiator */
    g_test_config.qos.ba_lut_entry[3].seq_num       = 0;
    g_test_config.qos.ba_lut_entry[3].win_size      = 0;
    g_test_config.qos.ba_lut_entry[3].tid           = 0;
    g_test_config.qos.ba_lut_entry[3].addr[0]       = 0x00;
    g_test_config.qos.ba_lut_entry[3].addr[1]       = 0x00;
    g_test_config.qos.ba_lut_entry[3].addr[2]       = 0x00;
    g_test_config.qos.ba_lut_entry[3].addr[3]       = 0x00;
    g_test_config.qos.ba_lut_entry[3].addr[4]       = 0x00;
    g_test_config.qos.ba_lut_entry[3].addr[5]       = 0x00;
    g_test_config.qos.ba_lut_entry[3].partial_state = 0;
    }

    /*  SMPS mode for 11n destination                                        */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Static   Dynamic   MIMO (Power Save Disabled)       */
    /*  Values to set :  1        2         3                                */
    /*  --------------------------------------------------------------       */
    g_test_config.ht.da_smps_mode = 3;

    /*************************************************************************/
    /* HT-Control Field Configuration                                        */
    /*************************************************************************/

    /*  Enable HT-Control Field in Tx-Frames.                                */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.ht.tx_htc_enable = 0;

    /*************************************************************************/
    /* A-MPDU Configuration                                                  */
    /*************************************************************************/

    /*  A-MPDU LUT programming test Enable                                   */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_config.ht.ampdu_lut_prog_test = 0;

    /* Don't reinitialize these parameters during a MAC Soft-Reset        */
    /* This is a temporary hack for facilitating remote setting of A-MPDU */
    /* test parameters.                                                   */
    if(g_reset_mac_in_progress == BFALSE)
    {
        /*  A-MPDU Number of sessions                                            */
        /*  --------------------------------------------------------------       */
        /*  Configuration :  Number of test A-MPDU sessions to set up            */
        /*  Values to set :  Maximum of 4                                        */
        /*  --------------------------------------------------------------       */
        g_test_config.ht.ampdu_num_sessions = 0;

        /* Note - various sessions are based only on TID, not RA. For different  */
        /* sessions enter unique TID.                                            */

        /*  A-MPDU LUT Entry 1 */
        g_test_config.ht.ampdu_sess_params[0].tid    = 0;
        g_test_config.ht.ampdu_sess_params[0].maxnum = 0;
        g_test_config.ht.ampdu_sess_params[0].maxraf = 0; /* 0 - 3 */
        g_test_config.ht.ampdu_sess_params[0].minmps = 0; /* 0 - 7 */
        g_test_config.ht.ampdu_sess_params[0].numbuf = 0;

        /*  A-MPDU LUT Entry 2 */
        g_test_config.ht.ampdu_sess_params[1].tid    = 0;
        g_test_config.ht.ampdu_sess_params[1].maxnum = 0;
        g_test_config.ht.ampdu_sess_params[1].maxraf = 0; /* 0 - 3 */
        g_test_config.ht.ampdu_sess_params[1].minmps = 0; /* 0 - 7 */
        g_test_config.ht.ampdu_sess_params[1].numbuf = 0;

        /*  A-MPDU LUT Entry 3 */
        g_test_config.ht.ampdu_sess_params[2].tid    = 0;
        g_test_config.ht.ampdu_sess_params[2].maxnum = 0;
        g_test_config.ht.ampdu_sess_params[2].maxraf = 0; /* 0 - 3 */
        g_test_config.ht.ampdu_sess_params[2].minmps = 0; /* 0 - 7 */
        g_test_config.ht.ampdu_sess_params[2].numbuf = 0;

        /*  A-MPDU LUT Entry 4 */
        g_test_config.ht.ampdu_sess_params[3].tid    = 0;
        g_test_config.ht.ampdu_sess_params[3].maxnum = 0;
        g_test_config.ht.ampdu_sess_params[3].maxraf = 0; /* 0 - 3 */
        g_test_config.ht.ampdu_sess_params[3].minmps = 0; /* 0 - 7 */
        g_test_config.ht.ampdu_sess_params[3].numbuf = 0;
    }

    if(g_reset_mac_in_progress == BFALSE)
    {
    /************************************************************************/
    /* AMSDU Test Configuration                                             */
    /************************************************************************/
    /* AMSDU Operating Mode.                                                */
    /* 0 => AMSDU Disable.                                                  */
    /* 1 => Single SubMSDU Info-table entry.                                */
    /* 2 => Multiple SubMSDUs Info-table entries                            */
    g_test_config.ht.amsdu_mode    = 0;

    /* If amsdu_mode = 1 then maxlen < 4096 */
    g_test_config.ht.amsdu_maxlen  = 0;

    /* If amsdu_mode = 2 then maxnum <= 16 */
    g_test_config.ht.amsdu_maxnum  = 0;
    }
#endif /* MAC_802_11N */

    /*************************************************************************/
    /*  PHY Test configuration                                               */
    /*************************************************************************/

    /*  -------------------------------------------------------------------- */
    /*  PHY Test Mode                                                        */
    /*  -------------------------------------------------------------------- */
    /*  Configuration :                                                      */
    /*    PHY_TEST_DISABLED             : Disable PHY tests                  */
    /*    PHY_TEST_PER_TX_MODE          : PER Tx Mode                        */
    /*    PHY_TEST_PER_RX_MODE          : PER Rx Mode                        */
    /*    PHY_TEST_BURST_TX_MODE        : Burst Tx Mode/Spectral measmnt     */
    /*    PHY_TEST_CONTINUOUS_TX_MODE_0 : Continuous Tx Mode (all zeros)     */
    /*    PHY_TEST_CONTINUOUS_TX_MODE_1 : Continuous Tx Mode (all ones)      */
    /*    PHY_TEST_CONTINUOUS_TX_MODE_2 : Continuous Tx Mode (alt 0x55/0xAA) */
    /*    PHY_TEST_CONTINUOUS_TX_MODE_3 : Continuous Tx Mode (pseudo-random) */
    /*    PHY_TEST_NO_MODULATION_MODE   : No modulation Mode/Arbit waveform  */
    /*  -------------------------------------------------------------------- */
    g_test_config.phy.mode = PHY_TEST_DISABLED;

    /*  PER-TX Test settings                                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  The range of Rate/MCS used during PER-TX test       */
    /*  Values to set :  Any valid rate or MCS                               */
    /*  --------------------------------------------------------------       */
    /* Note: If g_test_params.tx_format is set to 0 (non-HT) these fields    */
    /*       are interpreted as rates. Otherwise (Mixed-Mode or Greenfield)  */
    /*       they are interpreted as MCS values.                             */
    g_test_config.phy.rate_hi = 0;
    g_test_config.phy.rate_lo = 0;

#ifdef TX_ABORT_FEATURE
    /*  S/w triggered Tx Abort Test                                          */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable               Enable                        */
    /*  Values to set :  0                     1                             */
    /*  --------------------------------------------------------------       */
    g_test_config.sw_trig_tx_abort_test = 0;
#endif /* TX_ABORT_FEATURE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_mh_test_params                                */
/*                                                                           */
/*  Description   : This function initializes the test parameters. Note      */
/*                  that this function needs to be modified by the user to   */
/*                  configure MAC with the desired parameters for the test.  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_params                                            */
/*                                                                           */
/*  Processing    : This function initializes the global test parameters.    */
/*                  structure.                                               */
/*                                                                           */
/*  Outputs       : The global parameter structure is initialized.           */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_mh_test_params(void)
{
    UWORD8 test_mac_address[6] = {0x00, 0x49, 0x54, 0x54, 0x0B, 0x4E};

    /*  Station MAC Address */
    memcpy(g_test_params.mac_address, test_mac_address, 6);

    /*  BSS Type                                                             */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Infrastructure    Independent   Access Point        */
    /*  Values to set :         1               2            3               */
    /*  --------------------------------------------------------------       */
    g_test_params.mac_mode = 3;

    /*  BSS Identifier (BSSID) */
    if(g_test_params.mac_mode != 3)
    {
        g_test_params.bssid[0] = 0xBE;
        g_test_params.bssid[1] = 0xAC;
        g_test_params.bssid[2] = 0x23;
        g_test_params.bssid[3] = 0x39;
        g_test_params.bssid[4] = 0x76;
        g_test_params.bssid[5] = 0xCB;
    }
    else
    {
        memcpy(g_test_params.bssid, g_test_params.mac_address, 6);
    }

#ifdef BURST_TX_MODE
    if(g_test_params.mac_mode == 5)
    {
        g_burst_tx_mode_enabled = BTRUE;
    }
#endif /* BURST_TX_MODE */

    if(g_test_params.mac_mode == 4)
    {
        g_phy_continuous_tx_mode_enable = BTRUE;
    }

    /*  Beacon Period                                                        */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Any value within 16 bits                            */
    /*  Values to set :  Same value                                          */
    /*  --------------------------------------------------------------       */
    g_test_params.bcn_period = 100;

    /*  DTIM Period                                                          */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Any value between 1 to 255                         */
    /*  Values to set :   Same value. Default is 3                           */
    /*  --------------------------------------------------------------       */
    g_test_params.dtim_period = 3;

    /*  Channel                                                              */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  2.4GHz Band          5GHz Band                      */
    /*  Values to set :  RC_START_FREQ_2      RC_START_FREQ_5                */
    /*  --------------------------------------------------------------       */
    g_test_params.start_freq = RC_START_FREQ_2;

    /*  Channel                                                              */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  1  2  3  4  5  6  7  8  9  10  11  12  13  14       */
    /*  Values to set :  0  1  2  3  4  5  6  7  8   9  10  11  12  13       */
    /*  --------------------------------------------------------------       */
    g_test_params.channel = 10;

    /*  Secondary Channel Offset                                             */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  NoSecChan  SecChanAbove  SecChanBelow               */
    /*  Values to set :  SCN        SCA           SCB                        */
    /*  --------------------------------------------------------------       */
    g_test_params.sec_chan_offset = SCN;

    /*  Fragmentation Threshold                                              */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Any value between 256 to 3000                      */
    /*  Values to set :   Same value. Default is 3000                        */
    /*  --------------------------------------------------------------       */
    g_test_params.frag_threshold = 3000;

    /*  RTS Threshold                                                        */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Any value between 256 to 3000                      */
    /*  Values to set :   Same value. Default is 3000                        */
    /*  --------------------------------------------------------------       */
    g_test_params.rts_threshold = 3000;

    /*  Tx power level                                                       */
    /*  -------------------------------------------------------------------- */
    /*  Configuration : Sets TX Power in dBm                                 */
    /*  Values to set : Depends upon the RF being used                       */
    /*  -------------------------------------------------------------------- */
    g_test_params.tx_power_11a   = 12;
    g_test_params.tx_power_11n   = 12;
    g_test_params.tx_power_11n40 = 12;
    g_test_params.tx_power_11b   = 12;

    /*  11g operating mode                                                   */
    /*  ---------------------------------------------------------------      */
    /*  Configuration :                                                      */
    /*        G_ONLY_MODE           High Performance Mode                    */
    /*        G_MIXED_11B_1_MODE    Compatibility Mode - BasicRateSet 1      */
    /*        G_MIXED_11B_2_MODE    Compatibility Mode - BasicRateSet 2      */
    /*  ---------------------------------------------------------------      */
    g_test_params.g_running_mode = G_MIXED_11B_1_MODE;

    /*  Transmit Packet Format                                               */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Non-HT     HT Greenfield   HT Mixed                 */
    /*  Values to set :  0               1                 2                 */
    /*  --------------------------------------------------------------       */
    g_test_params.tx_format = 0;

    /*  Transmit Rate (valid only if Transmit Packet Format is 0)            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  1  2  5.5  11  6  9  12  18  24  36  48  54         */
    /*  Values to set :  1  2  5.5  11  6  9  12  18  24  36  48  54         */
    /*  --------------------------------------------------------------       */
    g_test_params.tx_rate = 54;

    /*  Preamble                                                             */
    /*  --------------------------------------------------------------       */
    /*  Configuration :   Short    Long                                      */
    /*  Values to set :       0       1                                      */
    /*  --------------------------------------------------------------       */
    g_test_params.preamble = 1;

    /*  Transmit MCS (ignored if Transmit Packet Format is 0)                */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  MCS 0 - 127                                         */
    /*  Values to set :  0 - 127                                             */
    /*  --------------------------------------------------------------       */
    g_test_params.tx_mcs = 0;

    /*  Short Guard Interval                                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Long GI  Short GI                                   */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_params.shortgi = 0;

    /*  Bandwidth                                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration    :  Values to set                                    */
    /*             20MHz :  CH_BW_20MHZ                                      */
    /*             40MHz :  CH_BW_40MHZ                                      */
    /*   DuplicateLegacy :  CH_BW_40DL                                       */
    /*       DuplicateHT :  CH_BW_40DH                                       */
    /*  --------------------------------------------------------------       */
    g_test_params.bandwidth = CH_BW_20MHZ;

    /*  Operating band                                                       */
    /*  --------------------------------------------------------------       */
    /*  Configuration    :  Values to set                                    */
    /*             40MHz :  CH_OFF_40                                        */
    /*        Upper20MHz :  CH_OFF_20U                                       */
    /*         Only20MHz :  CH_OFF_NONE                                      */
    /*        Lower20MHz :  CH_OFF_20L                                       */
    /*  --------------------------------------------------------------       */
    g_test_params.op_band = CH_OFF_NONE;

    /*  STBC (difference between NSTS and NSS)                               */
    /*  --------------------------------------------------------------       */
    /*  Configuration : No STBC (NSS = NSTS)                                 */
    /*  Values to set : 0                                                    */
    /*  --------------------------------------------------------------       */
    g_test_params.stbc = 0;

    /*  Number of Extension spatial streams                                  */
    /*  --------------------------------------------------------------       */
    /*  Configuration :                                                      */
    /*  Values to set :                                                      */
    /*  --------------------------------------------------------------       */
    g_test_params.ess = 0;

    /*  Antenna Set                                                          */
    /*  --------------------------------------------------------------       */
    /*  Configuration : Non-HT  MCS0-7  MCS8-15  MCS16-23  MCS24-31  Auto    */
    /*  Values to set : 0x01    0x01    0x03     0x07      0x0F      0x0     */
    /*  --------------------------------------------------------------       */
    g_test_params.antset = 0x01;

    /*  Smoothing Recommended                                                */
    /*  --------------------------------------------------------------       */
    /*  Configuration : Not recommended       Recommended                    */
    /*  Values to set : 0                     1                              */
    /*  --------------------------------------------------------------       */
    g_test_params.smoothing_rec = 0;

    /*  Sounding Packet                                                      */
    /*  --------------------------------------------------------------       */
    /*  Configuration : Not a sounding packet   A sounding packet            */
    /*  Values to set : 0                       1                            */
    /*  --------------------------------------------------------------       */
    g_test_params.sounding_pkt = 0;

    /*  Code                                                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration : BCC         LDPC                                     */
    /*  Values to set : 0           1                                        */
    /*  --------------------------------------------------------------       */
    g_test_params.ht_coding = 0;

    /*  HT Operating mode                                                    */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Standard 0     1       2        3                   */
    /*  Values to set :           0     1       2        3                   */
    /*  --------------------------------------------------------------       */
    g_test_params.ht_operating_mode = 0;

    /*  RIFS mode                                                            */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_params.rifs_mode = 0;

    /*  Protection mode for MAC                                              */
    /*  --------------------------------------------------------------       */
    /*  Configuration    :  Values to set                                    */
    /*     No protection :  NO_PROT                                          */
    /*               ERP :  ERP_PROT                                         */
    /*                HT :  HT_PROT                                          */
    /*                GF :  GF_PROT                                          */
    /*  --------------------------------------------------------------       */
    g_test_params.prot_type = NO_PROT;

    /*  ERP Protection type for MAC                                          */
    /*  --------------------------------------------------------------       */
    /*     Configuration :  Values to set                                    */
    /*          Self-CTS :  G_SELF_CTS_PROT                                  */
    /*           RTS-CTS :  G_RTS_CTS_PROT                                   */
    /*  --------------------------------------------------------------       */
    g_test_params.erp_prot_type = G_SELF_CTS_PROT;

    /*  11n HT Protection Type                                               */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  RTS-CTS   First Frame Exchange at non-HT-rate       */
    /*  Values to set :  0         1                                         */
    /*  Configuration :  LSIG TXOP First Frame Exchange in Mixed Fmt         */
    /*  Values to set :  2         3                                         */
    /*  --------------------------------------------------------------       */
    g_test_params.ht_prot_type = 0;

    /*  SMPS mode                                                            */
    /*  --------------------------------------------------------------       */
    /*      Configuration :  Values to set                                   */
    /*             Static :  STATIC_MODE                                     */
    /*            Dynamic :  DYNAMIC_MODE                                    */
    /*  MIMO(PS Disabled) :  MIMO_MODE                                       */
    /*  --------------------------------------------------------------       */
    g_test_params.smps_mode = MIMO_MODE;

    /*  TXOP Limit for AC_VO queue                                           */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Default MIB     User set TXOP limit                 */
    /*  Values to set :  -1              Any 16-bit value                    */
    /*  --------------------------------------------------------------       */
    g_test_params.ac_vo_txop_limit = -1;

    /*  20/40MHz Coexistence Support                                         */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_params.machw_2040_coex = 0;

    /*  FC TXOP Modification                                                 */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Promotion  DemotionInStart  DemotionInMiddle        */
    /*  Values to set :  BIT0       BIT1             BIT2                    */
    /*  --------------------------------------------------------------       */
    /*  Note: Each of these features can be independently controlled using   */
    /*        their respective bits.                                         */
    g_test_params.fc_txop_mod = 0;

    /*  FC Protection Type                                                   */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  RTS/CTS  Self-CTS                                   */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_params.fc_prot_type = 0;

    /*  Random Secondary Channel CCA Assertion (PHY-Model Only)              */
    /*  --------------------------------------------------------------       */
    /*  Configuration :  Disable  Enable                                     */
    /*  Values to set :  0        1                                          */
    /*  --------------------------------------------------------------       */
    g_test_params.sec_cca_assert = 0;
}

#endif /* MAC_HW_UNIT_TEST_MODE */
