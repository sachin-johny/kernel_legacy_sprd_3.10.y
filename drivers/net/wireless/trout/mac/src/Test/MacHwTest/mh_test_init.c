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
/*  File Name         : mh_test_init.c                                       */
/*                                                                           */
/*  Description       : This file contains all the functions to initialize   */
/*                      the variables and structures used in MAC.            */
/*                                                                           */
/*  List of Functions : test_main_function                                   */
/*                      initialize_testsw                                    */
/*                      create_testmac_interrupts                            */
/*                      configure_mac                                        */
/*                      initialize_ce                                        */
/*                      test_reinit_sw                                       */
/*                      initialize_test_case                                 */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

#ifdef MAC_HW_UNIT_TEST_MODE

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "mh_test.h"
#include "wep.h"
#include "iconfig.h"
#include "host_if.h"
#include "controller_mode_if.h"
#include "mh_test_case.h"
#include "csl_if.h"
#include "trout_wifi_rx.h"

#ifdef BURST_TX_MODE
#include "iconfig.h"
#endif /* BURST_TX_MODE */

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

static void initialize_test_case(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : test_main_function                                       */
/*                                                                           */
/*  Description   : This function is the main test function.                 */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the test parameters, test case */
/*                  configurations and MAC. It then starts the test.         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void test_main_function(void)
{
    /* Do processor specific initializations */
    init_proc();

    /* Reset PHY and MAC hardware */
    reset_phy_machw();

    /* Calibrate S/w delay loop */
    calibrate_delay_loop();

    /* Initialize the CSL Memory */
    csl_mem_init();

    /* reset QMU handle */
    mem_set(&g_q_handle, 0, sizeof(qmu_handle_t));

    /* Initialize MAC S/w as required for H/w unit test */
    initialize_testsw(&g_mac);

    /* Print the version information to the serial console */
    print_banner();

    /* Initialize the host interface */
    init_host_interface();

    /* Initialize the H/w unit test case (parameters and configuration) */
    initialize_test_case();

    /* Configure as per test settings - equivalent to user configuration     */
    /* with SME in MAC S/w                                                   */
    configure_mac();

    /* Initialize PHY, RF and MAC H/w. Note that the initializations need to */
    /* be done in this order exactly.                                        */
    /* Based on station mode, phy parameters  need to be updated */
    update_phy_mib(mget_DesiredBSSType());
    init_phy_curr_rates();
    initialize_machw();

    /* Read the Phy Version from the PHY Registers */
    read_phy_version();

    /* Initialize PHY H/w and RF Registers */
    initialize_phy();
    initialize_rf();
    perform_phy_calibration();

#ifdef PHY_CONTINUOUS_TX_MODE
    if(g_phy_continuous_tx_mode_enable == BTRUE)
    {
        set_phy_continuous_tx_mode();
    }
#endif /* PHY_CONTINUOUS_TX_MODE */

    /* Re-initialize Tx power with the saved values */
	// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
    set_tx_power_11a_after_spi_lock(g_curr_tx_power.pow_a.gaincode);
    set_tx_power_11n_after_spi_lock(g_curr_tx_power.pow_n.gaincode);
    set_tx_power_11b_after_spi_lock(g_curr_tx_power.pow_b.gaincode);
#else
    set_tx_power_11a_after_spi_lock(g_default_tx_power.pow_a.gaincode);
    set_tx_power_11n_after_spi_lock(g_default_tx_power.pow_n.gaincode);
    set_tx_power_11b_after_spi_lock(g_default_tx_power.pow_b.gaincode);
#endif

#ifndef OS_LINUX_CSL_TYPE
    /* The operation carried out will be dependent on the mode of    */
    /* operation, if AP then initiate start is called, otherwise     */
    /* scanning process is initiated.                                */
    enable_operation(&g_mac);

    /* Once all initializations are complete the PA control register */
    /* is programmed to enable PHY and MAC H/w.                      */
    enable_machw_phy_and_pa();

    /* Start MAC controller task */
    start_mac_controller();
#endif  /* OS_LINUX_CSL_TYPE */
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_testsw                                        */
/*                                                                           */
/*  Description   : This function initializes the software required for test.*/
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function calls appropriate initialization routines  */
/*                  to initialize all the globals used by MAC.               */
/*                                                                           */
/*  Outputs       : The given MAC library is initialized according to the    */
/*                  mode of operation. All globals are also initialized.     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_testsw(mac_struct_t *mac)
{
    /* Initialize the memory manager and the MAC memory pools */
    init_mac_mem_pools();

    /* Initialize the MAC library structure (FSM table) */
    init_mac_fsm(mac);

    /* Initialize PHY characteristics, and after that PHY mibs */
    init_phy_chars();

    /* Initialize the MIB parameters */
    init_mib();
    init_phy_mib();

    /* Initialize the globals */
    init_mac_globals();

   /* Queue interface table is initialized */
    init_qif_table();

    /* Initialize Queue Manager */
	init_mac_qmu();	//modify by chengwg.

    /* Initialize the chip support library. Note that the memory manager     */
    /* needs to be initialized before this.                                  */
    csl_init();

    /* Initialize event manager */
    init_event_manager();

    /* Initialize the supported channel list */
    init_supp_ch_list();

    /* Initialize supported Regulatory Domain tables */
    init_supp_reg_domain();
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_testmac_interrupts                                */
/*                                                                           */
/*  Description   : This function creates the MAC interrupts and attaches    */
/*                  the various service routines.                            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_isr_handle, g_mac_int_vector                       */
/*                                                                           */
/*  Processing    : Interrupts are created by calling the required chip      */
/*                  support library function.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void create_testmac_interrupts(void)
{
    ISR_T id = {0};

    id.isr = (void *)mac_isr;
    id.dsr = NULL;


    g_mac_isr_handle = create_interrupt(g_mac_int_vector,  /* Vector         */
                                        MAC_INTERRUPT_PRI, /* Priority       */
                                        MAC_INTERRUPT_DAT, /* Interrupt data */
                                        &id);              /* Service handler*/
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : configure_mac                                            */
/*                                                                           */
/*  Description   : This function configures MAC as per test settings.       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_params                                            */
/*                                                                           */
/*  Processing    : The MIB/MAC H/w registers are set as per the test        */
/*                  settings. Note that the MIB values are used to           */
/*                  initialize MAC H/w also.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void configure_mac(void)
{
    /* Initialize PHY characteristics, and after that PHY mibs */
    init_phy_chars();

    /* Initialize the MIB params */
    init_mib();
    init_phy_mib();

#ifdef MAC_WMM
    /* Enable QoS if 802.11e flag is present */
    mset_QoSOptionImplemented(TV_TRUE);
    set_wmm_enabled(BTRUE);
#endif /* MAC_WMM */

#ifdef MAC_802_11N

    mset_HighThroughputOptionImplemented(TV_TRUE);

    if(g_test_params.tx_format == 0) /* Non-HT */
    {
        mset_RIFSMode(TV_FALSE);
        mset_HTOperatingMode(0);
    }
    else /* HT Greenfield or HT Mixed */
    {
        /* Set the S/w 11n operating mode based on the test transmit packet format */
        if(g_test_params.tx_format == 2)
            set_11n_op_type(NTYPE_MIXED); /* HT Mixed */
        else
            set_11n_op_type(NTYPE_HT_ONLY); /* HT Greenfield */

        /* Set the RIFS and HTOperatingMode with the test parameter settings.*/
        /* Note that this overwrites the setting done by set_11n_op_type()   */
        if(g_test_params.rifs_mode == 1)
            mset_RIFSMode(TV_TRUE);
        else
            mset_RIFSMode(TV_FALSE);

        mset_HTOperatingMode(g_test_params.ht_operating_mode);
    }
#endif /* MAC_802_11N */

#ifdef BURST_TX_MODE
    set_DesiredBSSType(g_test_params.mac_mode - 1);
#else /* BURST_TX_MODE */
    mset_DesiredBSSType(g_test_params.mac_mode);
#endif /* BURST_TX_MODE */

    if(mget_DesiredBSSType() == 3)
    {
        memcpy(g_test_params.bssid, g_test_params.mac_address, 6);
    }

    mset_BeaconPeriod(g_test_params.bcn_period);
    mset_bssid(g_test_params.bssid);
    set_current_start_freq(g_test_params.start_freq);
    mset_CurrentChannel(g_test_params.channel);
    update_operating_channel_hut();
    mset_FragmentationThreshold(g_test_params.frag_threshold);
    mset_StationID(g_test_params.mac_address);
    mset_RTSThreshold(g_test_params.rts_threshold);
    set_running_mode(g_test_params.g_running_mode);
    mset_DTIMPeriod(g_test_params.dtim_period);
    /* Initialize the TX Power setting */
    set_user_control_enabled(1);
    set_tx_power_level_11a(g_test_params.tx_power_11a);
    set_tx_power_level_11b(g_test_params.tx_power_11b);
    set_tx_power_level_11n(g_test_params.tx_power_11n);
    set_tx_power_level_11n40(g_test_params.tx_power_11n40);

    /* Based on the test configuration set the AC_VO TXOP limit MIB */
    if(g_test_params.ac_vo_txop_limit != -1)
    {
        if(mget_DesiredBSSType() != ANY_BSS)
            mset_EDCATableTXOPLimit(g_test_params.ac_vo_txop_limit, AC_VO);
        else
            mset_QAPEDCATableTXOPLimit(g_test_params.ac_vo_txop_limit, AC_VO);
    }

#ifdef MAC_802_11N
    mset_MIMOPowerSave(g_test_params.smps_mode);
#endif /* MAC_802_11N */

    if(g_test_params.tx_rate != 0)
        set_curr_tx_rate(g_test_params.tx_rate);

    set_curr_tx_mcs(g_test_params.tx_mcs);

    set_preamble(g_test_params.preamble);

    /* Configure the protection and HT control parameters */
    set_protection(g_test_params.prot_type);
    set_erp_prot_type(g_test_params.erp_prot_type);
    set_11n_ht_prot_type(g_test_params.ht_prot_type);
}

#ifdef MAC_802_11I

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_ce                                            */
/*                                                                           */
/*  Description   : This function initializes the software required for CE   */
/*                  tests                                                    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_test_config                                            */
/*                                                                           */
/*  Processing    : This function calls appropriate initialization routines  */
/*                  to initialize all the globals used by MAC S/W and MAC H/W*/
/*                  CE.                                                      */
/*                                                                           */
/*  Outputs       : The given MAC library is initialized according to the    */
/*                  mode of operation. All globals are also initialized.     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_ce(void)
{
    UWORD8 key0[16] = {0};
    UWORD8 key1[16] = {0};
    UWORD8 key2[16] = {0};
    UWORD8 mick0[16]  = {0};
    UWORD8 mick1[16]  = {0};
    UWORD8 mick2[16]  = {0};
    UWORD8 i        = 0;
    UWORD8 ct       = 0;

    /* Clear the existing entries in the CE LUT */
    machw_ce_del_key(0);
    machw_ce_del_key(1);

    if(g_test_config.ce.ce_enable == 0)
    {
        disable_ce();
        return;
    }

    /* Enable CE */
    enable_ce();

    /* Set the correct Operating mode in the CE-Control register */
    if(g_test_params.mac_mode == 1)
        set_machw_ce_bss_sta_mode();
    else if(g_test_params.mac_mode == 2)
        set_machw_ce_ibss_sta_mode();
    else
        set_machw_ce_ap_mode();

    /* Update the CE MAC address register */
    set_machw_ce_macaddr(mget_StationID());

    /* Enable the other Cipher related functions */
    enable_machw_ce_replay_det();
    enable_machw_ce_tkip_mic();

    /* Initialize all the keys to be used for programming */
    for(i = 0 ; i < 16; i ++)
    {
        key0[i] = i;
        key1[i] = i + 1;
        key2[i] = 15 - i;
    }

    for(i = 0 ; i < 16; i ++)
    {
        mick0[i] = 15 - i;
        mick1[i] = i;
        mick2[i] = i + 1;
    }

    memcpy(g_test_config.ce.tkip_rx_mic_key, mick0, 16);
    memcpy(g_test_config.ce.tkip_rx_bcst_mic_key, mick1, 16);


    /* Program the CE LUT with the configured Unicast Cipher Type */
    ct = g_test_config.ce.ct;

    /* machw_ce_add_key(id, num, index, mask, key, addr, orig, mickey)       */
    /* id             = 1 (GTK1), 2 (GTK2), 0 (PTK)                          */
    /* num            = 0 (TxGTK), 1 (PTK), 2 (RxGTK)                        */
    /* index          = 0 (GTK), 1 (PTK)                                     */
    /* mask           = 0 (non-WEP), 1 (WEP)                                 */
    /* key            = Value of key                                         */
    /* addr           = Bcast Addr (TxGTK), SA (RxGTK), DA/SA (PTK)          */
    /* orig           = Supplicant (SUPP_KEY), Authenticator (AUTH_KEY)      */
    /* mickey         = Value of MIC key                                     */
    if((ct == CCMP) || (ct == TKIP))
    {
        if(g_test_params.mac_mode == 1) /* BSS-STA */
        {
            /* The BSS STA is always a Supplicant */
            g_test_config.ce.key_orig = SUPP_KEY;

            machw_ce_add_key(0, PTK_NUM,     1, 0, ct, key0,
                             g_test_config.tx.da, SUPP_KEY, mick0);
        }
        else if(g_test_params.mac_mode == 2)  /* IBSS-STA */
        {

            if(g_test_config.ce.key_orig == SUPP_KEY)
            {
                machw_ce_add_key(0, PTK_NUM,     1, 0, ct, key0,
                                 g_test_config.tx.da, SUPP_KEY, mick0);
            }
            else
            {
                machw_ce_add_key(0, PTK_NUM,     1, 0, ct, key0,
                                 g_test_config.tx.da, AUTH_KEY, mick0);
            }
        }
        else  /* AP */
        {
            /* The BSS AP is always an Authenticator */
            g_test_config.ce.key_orig = AUTH_KEY;

            machw_ce_add_key(0, PTK_NUM, 1, 0, ct, key0, g_test_config.tx.da,
                             AUTH_KEY, mick0);
        }
    }
    else
    {
        machw_ce_add_key(1, PTK_NUM, 1, 0x3, ct, key1, g_test_config.tx.da,
                         AUTH_KEY, NULL);
    }

    /* Program the CE LUT with the configured Broadcast Cipher Type */
    ct = g_test_config.ce.bcst_ct;

    if((ct == CCMP) || (ct == TKIP))
    {
        if(g_test_params.mac_mode == 1) /* BSS-STA */
    {
            set_machw_ce_grp_key_type(ct);

            machw_ce_add_key(1, RX_GTK0_NUM, 1, 0, ct, key1,
                             g_test_config.tx.da, SUPP_KEY, mick1);
            machw_ce_add_key(2, RX_GTK1_NUM, 1, 0, ct, key2,
                             g_test_config.tx.da, SUPP_KEY, mick2);
        }
        else if(g_test_params.mac_mode == 2)  /* IBSS-STA */
        {
            set_machw_ce_grp_key_type(ct);

            if(g_test_config.ce.key_orig == SUPP_KEY)
            {
                /* If the IBSS STA is the Supplicant then use the following keys */
                /* RX-GTK: Key1, Mic-Key1                                        */
                /* TX-GTK: Key2, Mic-Key2                                        */

                machw_ce_add_key(1, RX_GTK0_NUM, 1, 0, ct, key1,
                                 g_test_config.tx.da, SUPP_KEY, mick1);
                machw_ce_add_key(1, TX_GTK_NUM, 0, 0, ct, key2,
                                 mget_bcst_addr(), AUTH_KEY, mick2);

                memcpy(g_test_config.ce.tkip_rx_bcst_mic_key, mick1, 16);
        }
        else
        {
                /* If the IBSS STA is the Supplicant then use the following keys */
                /* RX-GTK: Key2, Mic-Key2                                        */
                /* TX-GTK: Key1, Mic-Key1                                        */

                machw_ce_add_key(1, RX_GTK0_NUM, 1, 0, ct, key2,
                                 g_test_config.tx.da, SUPP_KEY, mick2);
                machw_ce_add_key(1, TX_GTK_NUM, 0, 0, ct, key1,
                                 mget_bcst_addr(), AUTH_KEY, mick1);

                memcpy(g_test_config.ce.tkip_rx_bcst_mic_key, mick2, 16);

            }
        }
        else /* AP */
        {
            machw_ce_add_key(1, TX_GTK_NUM, 0, 0, ct, key1, mget_bcst_addr(),
                             AUTH_KEY, mick1);
        }
    }
    else
    {
        machw_ce_add_key(1, TX_GTK_NUM, 0, 0x3, ct, key2, mget_bcst_addr(),
                         AUTH_KEY, NULL);
    }
}

#endif /* MAC_802_11I */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : test_reinit_sw                                           */
/*                                                                           */
/*  Description   : This function performs some of the HUT mode specific     */
/*                  initializations on a MAC reset.                          */
/*                                                                           */
/*  Inputs        : MAC Context Structure                                    */
/*                                                                           */
/*  Globals       : g_test_start                                             */
/*                                                                           */
/*  Processing    : The test s/w is relinitialized here. Also any test       */
/*                  config parameter without a corresponding WID is set here.*/
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void test_reinit_sw(mac_struct_t *mac)
{
    /* Initialize MAC S/w as required for H/w unit test */
    initialize_testsw(mac);

    /* Any Test Configuration which does not have a corresponding WID can be */
    /* set here.                                                             */

    /* Software reset of the host interface for buffer re-allocation         */
    soft_reset_host_if();

    /* Reinitialize the test case */
    initialize_test_case();

    /* Reset all MAC test statistics */
    reset_test_stats();

    g_test_start = BTRUE;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_test_case                                     */
/*                                                                           */
/*  Description   : This function initializes the H/w unit test.             */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function initializes the test configuration and the */
/*                  test parameters.                                         */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_test_case(void)
{
    /* Initialize the global test configuration structure */
    initialize_mh_test_config();
    init_vppp_struct();

    /* Initialize the global test parameters structure */
    initialize_mh_test_params();

    /* Update the parameters related to the test case */
    initialize_mh_test_case();
}

#endif /* MAC_HW_UNIT_TEST_MODE */
