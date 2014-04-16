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
/*  File Name         : mh.c                                                 */
/*                                                                           */
/*  Description       : This file contains the functions required for the    */
/*                      MAC hardware interface including interrupt service   */
/*                      routines for MAC H/w generated interrupts.           */
/*                                                                           */
/*  List of Functions : get_random_seed                                      */
/*                      get_random_byte                                      */
/*                      reset_phy_machw                                      */
/*                      reset_phy                                            */
/*                      initialize_machw                                     */
/*                      mac_isr                                              */
/*                      tx_complete_isr                                      */
/*                      rx_complete_isr                                      */
/*                      tbtt_isr                                             */
/*                      deauth_isr                                           */
/*                      error_isr                                            */
/*                      write_dot11_phy_reg                                  */
/*                      read_dot11_phy_reg                                   */
/*                      update_machw_txq_chan_bw                             */
/*                      initialize_machw_2040                                */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "common.h"
#include "ce_lut.h"
#include "csl_if.h"
#include "qmu_if.h"
#include "autorate.h"
#include "core_mode_if.h"
#include "iconfig.h"
#include "phy_hw_if.h"
#include "phy_prot_if.h"
#include "receive.h"
#include "prot_if.h"
#ifdef MEASURE_PROCESSING_DELAY
#include "metrics.h"
#endif /* MEASURE_PROCESSING_DELAY */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#include "pm_sta.h"
#endif

#include "csl_linux.h"
#include "trout_wifi_rx.h"
#include "qmu_tx.h"

#ifdef MAC_WMM
#include "mib_11e.h"
#endif /* MAC_WMM */

/*****************************************************************************/
/* Static Global Variables                                                   */
/*****************************************************************************/
#ifndef MAC_HW_UNIT_TEST_MODE
static UWORD8 g_rxq_sync_loss[2];
static UWORD32 g_int_mask;	//add by chengwg for save trout int mask register value(reduce IO).
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
//volatile UWORD16 g_machw_clock_enable_flag = 0; // caisf add for power sleep, 0225
unsigned int arm2host_irq_cnt = 0;
#endif

extern struct task_struct *mac_int_task;
/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

#ifndef MAC_HW_UNIT_TEST_MODE
void tx_complete_isr(void);
#ifndef TROUT_WIFI
static void rx_complete_isr(UWORD8 q_num);
#endif
static void tbtt_isr(void);
static void error_isr(void);
static void deauth_isr(void);
static void radar_detect_isr(void);
#endif /* MAC_HW_UNIT_TEST_MODE */

extern BOOL_T is_all_machw_q_null(void);
extern void dump_allregs(unsigned long ix, unsigned long iy);


/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_machw_tsf_disable                                    */
/*                                                                           */
/*  Description   : Function to set TSF registers in the PA register bank    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The TSF Control register has various subfields           */
/*                  described below.                                         */
/*                                                                           */
/*                  31 - 2   1              0                                */
/*                  Reserved Start/Join~    TSF Enable                       */
/*                                                                           */
/*                  TSF Enable   0 - This function is disabled. TBTT is      */
/*                                   not generated.                          */
/*                               1 - This function is enabled.               */
/*                  Start/Join~  0 - After being enabled, the TSF waits      */
/*                                   to receive Beacon or Probe Response     */
/*                                   frame with matching BSSID, adopts the   */
/*                                   parameters and starts the TSF timer     */
/*                                   and TBTT generation.                    */
/*                               1 - After being enabled, the TSF            */
/*                                   immediately starts the TSF timer and    */
/*                                   TBTT generation. This bit is            */
/*                                   interpreted when TSF Enable is set to 1 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
struct timespec tv;		//add by chengwg.

void update_trout_int_mask(UWORD32 mask)
{
	g_int_mask = mask >> 2;
}

UWORD32 get_trout_int_mask(void)
{
	return g_int_mask;
}

void set_machw_tsf_disable(void)
{
    //chenq mod
    //rMAC_TSF_CON = ~REGBIT0;
	//chenq mod 2013-05-14
    //host_write_trout_reg( ~REGBIT0, (UWORD32)rMAC_TSF_CON );
	host_write_trout_reg( host_read_trout_reg( (UWORD32)rMAC_TSF_CON ) & (~REGBIT0) , (UWORD32)rMAC_TSF_CON );
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_machw_tsf_start                                      */
/*                                                                           */
/*  Description   : Function to set TSF registers in the PA register bank    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The TSF Control register has various subfields           */
/*                  described below.                                         */
/*                                                                           */
/*                  31 - 2   1              0                                */
/*                  Reserved Start/Join~    TSF Enable                       */
/*                                                                           */
/*                  TSF Enable   0 - This function is disabled. TBTT is      */
/*                                   not generated.                          */
/*                               1 - This function is enabled.               */
/*                  Start/Join~  0 - After being enabled, the TSF waits      */
/*                                   to receive Beacon or Probe Response     */
/*                                   frame with matching BSSID, adopts the   */
/*                                   parameters and starts the TSF timer     */
/*                                   and TBTT generation.                    */
/*                               1 - After being enabled, the TSF            */
/*                                   immediately starts the TSF timer and    */
/*                                   TBTT generation. This bit is            */
/*                                   interpreted when TSF Enable is set to 1 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_machw_tsf_start(void)
{
    //chenq mod
    //rMAC_TSF_CON = REGBIT1 | REGBIT0;
    host_write_trout_reg( REGBIT1 | REGBIT0, (UWORD32)rMAC_TSF_CON );
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : set_machw_tsf_join                                       */
/*                                                                           */
/*  Description   : Function to set TSF registers in the PA register bank    */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The TSF Control register has various subfields           */
/*                  described below.                                         */
/*                                                                           */
/*                  31 - 2   1              0                                */
/*                  Reserved Start/Join~    TSF Enable                       */
/*                                                                           */
/*                  TSF Enable   0 - This function is disabled. TBTT is      */
/*                                   not generated.                          */
/*                               1 - This function is enabled.               */
/*                  Start/Join~  0 - After being enabled, the TSF waits      */
/*                                   to receive Beacon or Probe Response     */
/*                                   frame with matching BSSID, adopts the   */
/*                                   parameters and starts the TSF timer     */
/*                                   and TBTT generation.                    */
/*                               1 - After being enabled, the TSF            */
/*                                   immediately starts the TSF timer and    */
/*                                   TBTT generation. This bit is            */
/*                                   interpreted when TSF Enable is set to 1 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void set_machw_tsf_join(void)
{
    //chenq mod
    //rMAC_TSF_CON = REGBIT0;
    host_write_trout_reg( REGBIT0, (UWORD32)rMAC_TSF_CON );
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_random_seed                                          */
/*                                                                           */
/*  Description   : This function gets the random seed.                      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function uses the MAC address to generate a random  */
/*                  value for initialization.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Random seed value                                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_random_seed(void)
{
    static UWORD8 sum   = 0;
    UWORD8        i     = 0;
    UWORD8        *addr = 0;

    addr = mget_StationID();

    for(i = 0; i < 6; i++)
        sum += addr[i];

    return sum;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : get_random_byte                                          */
/*                                                                           */
/*  Description   : This function gets a random byte from MAC hardware.      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function uses the MAC hardware PRBS generator to    */
/*                  obtain a random byte value.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, Random byte value                                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 get_random_byte(void)
{
	// 20120709 caisf add, merged ittiam mac v1.2 code
    //UWORD8 ret = 0;
    UWORD32 ret = 0;

	TROUT_FUNC_ENTER;
	// caisf temp mod for the reg rMAC_PRBS_READ_CTRL error!
	#ifdef TROUT_WIFI_EVB
    ret = get_random_seed();
    return ret;
	#endif
    //chenq mod
    //rMAC_PRBS_READ_CTRL |= REGBIT0;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PRBS_READ_CTRL ) | (REGBIT0) ,
        (UWORD32)rMAC_PRBS_READ_CTRL);

    //while(rMAC_PRBS_READ_CTRL & REGBIT0)
    while(host_read_trout_reg( (UWORD32)rMAC_PRBS_READ_CTRL ) & REGBIT0)
    {
		// ret = rMAC_PA_VER;
        //ret = host_read_trout_reg( (UWORD32)rMAC_PA_VER );
        //(*(volatile UWORD32 *)HOST_DELAY_FOR_TROUT_PHY);	//add by chengq.
        GET_TIME();
    }

	//ret = convert_to_le(rMAC_PRBS_READ_CTRL);
    ret = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PRBS_READ_CTRL ));
    ret = (ret >> 1) & 0xFF;
    TROUT_FUNC_EXIT;
    return ret;
}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
void enable_machw_clock(void)
{
    TROUT_DBG4("enable_machw_clock start \n");

	if(g_enable_machw_clock_flag){
        TROUT_DBG4("enable_machw_clock already enable \n");
		return;
    }
    
    //chenq mod
    //rMAC_PA_CON |= REGBIT25;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) | (REGBIT25) ,
        (UWORD32)rMAC_PA_CON );

	g_enable_machw_clock_flag = 1; 
    TROUT_DBG4("enable_machw_clock end \n");
}

void disable_machw_clock(void)
{
    TROUT_DBG4("disable_machw_clock start \n");

	if(!g_enable_machw_clock_flag){
        TROUT_DBG4("disable_machw_clock already disable \n");
		return;
    }        
    
    //chenq mod
    //rMAC_PA_CON &= ~REGBIT25;
    host_write_trout_reg(
        host_read_trout_reg( (UWORD32)rMAC_PA_CON ) & (~REGBIT25) ,
        (UWORD32)rMAC_PA_CON );

    g_enable_machw_clock_flag = 0;
    TROUT_DBG4("disable_machw_clock end \n");
}
#endif


/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_phy_machw                                          */
/*                                                                           */
/*  Description   : This function resets PHY and MAC hardware interface.     */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function resets PHY and MAC hardware by setting the */
/*                  required bits.                                           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void reset_phy_machw(void)
{
	TROUT_FUNC_ENTER;
#ifdef MWLAN
    /* Save the S/w debug register value for restoration after the reset */
    //chenq mod
    //UWORD32 swd_reg = rSW_HANG_DBG_REG;
//    UWORD32 swd_reg = host_read_trout_reg( (UWORD32)rSW_HANG_DBG_REG ); //Hugh
#endif /* MWLAN */


    /* Reset MAC H/W */

	//chenq mod 2012-11-08
	#if 0
    reset_machw();
	#else
    reset_pacore();
    reset_cecore();
	reset_tsf();
	reset_defrag_lut();
	reset_duplicate_det();
	#endif
	
    /* Delay added to let the MAC H/w come out of reset */
    add_calib_delay(10);

    /* Reset PHY */
    reset_phy();

#ifdef MWLAN
    //chenq mod
    //rSW_HANG_DBG_REG = swd_reg;
    //host_write_trout_reg( swd_reg, (UWORD32)rSW_HANG_DBG_REG ); //Hugh

#endif /* MWLAN */
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_machw                                         */
/*                                                                           */
/*  Description   : This function initializes MAC hardware interface.        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the registers to their initial value. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void initialize_machw(void)
{
    /*************************************************************************/
    /* Protocol Accelerator Register Initialization                          */
    /*************************************************************************/
	TROUT_FUNC_ENTER;
    /* Disable the MAC H/W */
    disable_machw_phy_and_pa();

    /*************************************************************************/
    /* General Registers                                                     */
    /*************************************************************************/
    //disable_machw_duplicate_detection();//dumy add 0814
    //disable_machw_selfcts_on_txab();//dumy add 0815
    //disable_machw_cts_trans();//dumy add 0815
    //disable_rx_buff_based_ack();//dumy add 0815
    
    /* Enable 8-bit PHY register addressing */
    enable_8bit_phy_addr();

    /* CCA based protocol timer */
    cca_based_prot(BTRUE);
    //cca_based_prot(BFALSE);//dumy add 0815
    
    /* Remove the RX buffer dependency                                       */
    disable_rxbuf_dependency();

    /* The PA version and status registers are Read Only. The MAC address,   */
    /* BSSID, Operation mode is initialized with MIB values. The PRBS seed   */
    /* value is initialized to a random value. The DMA burst size is set to  */
    /* an initial value. The required protocols in the PA control register   */
    /* are enabled as per the protocols in use (PCF, EDCA, HCCA, DCF).       */
    /* Note that the PA control register will be programmed to enable PHY    */
    /* and MAC H/w once all initializations are done.                        */

    set_machw_macaddr(mget_StationID());
    set_machw_bssid(mget_bssid());
    set_machw_op_mode(mget_DesiredBSSType());
    //chenq mod
    //rMAC_PA_DMA_BURST_SIZE =  convert_to_le(MAC_PA_DMA_BURST_SIZE_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_PA_DMA_BURST_SIZE_INIT_VALUE)
                            , (UWORD32)rMAC_PA_DMA_BURST_SIZE );

    

    enable_machw_clock();

#ifndef DISABLE_MACHW_DEFRAG
    enable_machw_defrag();
#endif /* DISABLE_MACHW_DEFRAG */

#ifdef AUTORATE_FEATURE
    enable_machw_autorate();
#endif /* AUTORATE_FEATURE */


    /* Enable Rx Buffer based Ack Sending */
    if(g_rx_buffer_based_ack == BTRUE)
    {
        enable_rx_buff_based_ack();
    }

    enable_msdu_lifetime_check();

#ifdef MAC_WMM

    if(get_wmm_enabled() == BTRUE)
        enable_machw_edca();
    else
        disable_machw_edca();
#endif /* MAC_WMM */

#ifdef MAC_802_11N
    if(get_ht_enable() == 1)
    {
        enable_machw_ht();
        enable_machw_ba();
#ifndef DISABLE_MACHW_DEAGGR
        enable_machw_deaggr();
#endif /* DISABLE_MACHW_DEAGGR */
    }
    else
    {
        disable_machw_ht();
        disable_machw_ba();
        disable_machw_deaggr();
    }
#endif /* MAC_802_11N */

    /* If autorate is enabled the NULL frame rate is set to the maximum      */
    /* basic rate. Otherwise it is set to the transmission rate selected by  */
    /* the user.                                                             */
    {
        UWORD8 tx_rate = 0;

        if(is_autorate_enabled() == BTRUE)
            tx_rate = get_max_basic_rate();
        else
            tx_rate = get_curr_tx_rate();

        set_machw_null_frame_rate(get_phy_rate(tx_rate));
        set_machw_null_frame_ptm(get_reg_phy_tx_mode(tx_rate, 0));
    }

    /*************************************************************************/
    /* EDCA Registers                                                        */
    /*************************************************************************/

#ifdef MAC_WMM

    if(get_wmm_enabled() == BTRUE)
    {
        /* Update EDCA Specific MAC H/w registers */
        update_edca_machw();

        set_machw_prng_seed_val_all_ac();
    }
    else
    {
        set_machw_aifsn();
        set_machw_cw(get_cwmax(), get_cwmin());
        set_machw_prng_seed_val(get_random_seed());
    }

#else /* MAC_WMM */

#ifdef BURST_TX_MODE
    if(g_burst_tx_mode_enabled == BTRUE)
    {
        set_machw_aifsn_burst_mode();
        /* For burst tx mode both CWmin & CWmax must be set to 0 */
        set_machw_cw(0, 0);
    }
    else
    {
        set_machw_aifsn();
        set_machw_cw(get_cwmax(), get_cwmin());
    }
#else /* BURST_TX_MODE */
    set_machw_aifsn();
    set_machw_cw(get_cwmax(), get_cwmin());
#endif /* BURST_TX_MODE */

    set_machw_prng_seed_val(get_random_seed());

#endif /* MAC_WMM */

    /* Initialize Tx Abort parameters */
    init_tx_abort_params();

    /*************************************************************************/
    /* Reception Registers                                                   */
    /*************************************************************************/

    /* The receive frame filter, frame control and receive buffer address    */
    /* are set to initial values. The max len filter is also set.            */

#ifdef TROUT_WIFI_NPI // zhuyeguang mod for trout rf test mode
    printk("MAC_RX_FRAME_FILTER_INIT_VALUE = 0x%x\n",MAC_RX_FRAME_FILTER_INIT_VALUE);
    printk("MAC_RX_FRAME_FILTER_INIT_VALUE & (~REGBIT16) = 0x%x\n",
		MAC_RX_FRAME_FILTER_INIT_VALUE & (~REGBIT16));
    host_write_trout_reg( convert_to_le(MAC_RX_FRAME_FILTER_INIT_VALUE & (~REGBIT16)), 
                          (UWORD32)rMAC_RX_FRAME_FILTER );
    printk("MAC_RX_FRAME_FILTER_INIT_VALUE read = 0x%x\n",
		convert_to_le(host_read_trout_reg((UWORD32)rMAC_RX_FRAME_FILTER)));
#else
    //chenq mod
    //rMAC_RX_FRAME_FILTER  = convert_to_le(MAC_RX_FRAME_FILTER_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_RX_FRAME_FILTER_INIT_VALUE), 
                          (UWORD32)rMAC_RX_FRAME_FILTER );
#endif    
    //rMAC_FRAME_CON        = convert_to_le(MAC_FRAME_CON_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_FRAME_CON_INIT_VALUE ), 
                          (UWORD32)rMAC_FRAME_CON );

#if 0	//shield by chengwg, this part is initialized in initialize_macsw(). 
    //rMAC_RX_BUFF_ADDR     = convert_to_le(virt_to_phy_addr(
                                            //MAC_RX_BUFF_ADDR_INIT_VALUE));                                          
    host_write_trout_reg( convert_to_le(virt_to_phy_addr(
                                            MAC_RX_BUFF_ADDR_INIT_VALUE)), 
                          (UWORD32)rMAC_RX_BUFF_ADDR );
    
    //rMAC_HIP_RX_BUFF_ADDR = convert_to_le(virt_to_phy_addr(
                                            //MAC_HIP_RX_BUFF_ADDR_INIT_VALUE));
    host_write_trout_reg( convert_to_le(virt_to_phy_addr(
                                            MAC_HIP_RX_BUFF_ADDR_INIT_VALUE)), 
                          (UWORD32)rMAC_HIP_RX_BUFF_ADDR );
#endif

    //rMAC_RXMAXLEN_FILT    = convert_to_le(MAC_RXMAXLENFILT_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_RXMAXLENFILT_INIT_VALUE),
                          (UWORD32)rMAC_RXMAXLEN_FILT );
    
    //rMAC_HIP_RXQ_CON      = convert_to_le(MAC_HIP_RXQ_CON_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_HIP_RXQ_CON_INIT_VALUE), 
                          (UWORD32)rMAC_HIP_RXQ_CON );

    
    set_machw_rx_lifetime(mget_MaxReceiveLifetime());
    set_machw_sub_msdu_gap(HOST_HEADER_GAP);
    set_machw_max_rx_buffer_len(RX_BUFFER_SIZE);

    /*************************************************************************/
    /* TSF Registers                                                         */
    /*************************************************************************/

    /* The TSF is initially disabled and will be set during Scan/Join/Start  */
    /* procedure as required. The TSF timer is Read Only. The beacon period  */
    /* and DTIM period is set with MIB values. The beacon pointer and beacon */
    /* transmit parameters will be set once S/w prepares and is ready to     */
    /* transmit a beacon.                                                    */
    set_machw_tsf_disable();
    //chenq add for reset tsf timer 2013-05-14
	set_machw_tsf_timer(0x00,0x00);
	
    set_machw_beacon_period(mget_BeaconPeriod());
    set_machw_dtim_period(mget_DTIMPeriod());

    /*************************************************************************/
    /* Protection And SIFS Response Registers                                */
    /*************************************************************************/

    /* The protection control register is set according to the PHY in use.   */
    /* It will also be set when the protection mode is set. RTS threshold is */
    /* set with MIB value. This will be updated if RTS threshold changes.    */
    /* The SIFS response table is set to initial values and need not be      */
    /* changed later.                                                        */
    set_machw_prot_control();
    set_machw_rts_thresh(mget_RTSThreshold());
    set_machw_min_txop_frag(mget_FragmentationThreshold());
    set_machw_frag_thresh(mget_FragmentationThreshold());
    set_default_machw_prot_params();
    enable_basic_rt_sifs_resp();

#ifdef MAC_802_11N
    set_machw_ht_control();
#endif /* MAC_802_11N */

    init_machw_ra_lut();

    /*************************************************************************/
    /* Channel Access Timer Management Registers                             */
    /*************************************************************************/

    /* The Slot, SIFS and EIFS time registers are set to initial values      */
    /* based on the PHY in use. The slot select bit may be updated for slot  */
    /* time adoption in AP mode for 802.11g.                                 */

    //chenq mod
    //rMAC_SIFS_TIME = convert_to_le(MAC_SIFS_TIME_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_SIFS_TIME_INIT_VALUE), 
                          (UWORD32)rMAC_SIFS_TIME );

    set_machw_slot_times(get_short_slot_time(), get_long_slot_time());

    if(get_short_slot_enabled() == 1)
        set_machw_short_slot_select();
    else
        set_machw_long_slot_select();

    //chenq mod
    //rMAC_EIFS_TIME     = convert_to_le(get_eifs_time());
    host_write_trout_reg( convert_to_le(get_eifs_time()), 
                          (UWORD32)rMAC_EIFS_TIME );
    
    //rMAC_PPDU_MAX_TIME = convert_to_le(get_ppdu_max_time() * 1024);
    host_write_trout_reg( convert_to_le(get_ppdu_max_time() * 1024), 
                          (UWORD32)rMAC_PPDU_MAX_TIME );

    //rMAC_SIFS_TIME2    = convert_to_le(MAC_SIFS_TIME2_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_SIFS_TIME2_INIT_VALUE), 
                          (UWORD32)rMAC_SIFS_TIME2 );

    //rMAC_RIFS_TIME_CONTROL_REG = convert_to_le(
                                 //MAC_RIFS_TIME_CONTROL_REG_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_RIFS_TIME_CONTROL_REG_INIT_VALUE),
                          (UWORD32)rMAC_RIFS_TIME_CONTROL_REG );                             

    if(get_current_start_freq() == RC_START_FREQ_2)
        //rMAC_SEC_CHAN_SLOT_COUNT = MAC_SEC_CHAN_SLOT_COUNT_INIT_VAL_FREQ_2;
        host_write_trout_reg( MAC_SEC_CHAN_SLOT_COUNT_INIT_VAL_FREQ_2, 
                              (UWORD32)rMAC_SEC_CHAN_SLOT_COUNT );
    else
        //rMAC_SEC_CHAN_SLOT_COUNT = MAC_SEC_CHAN_SLOT_COUNT_INIT_VAL_FREQ_5;
        host_write_trout_reg( MAC_SEC_CHAN_SLOT_COUNT_INIT_VAL_FREQ_5, 
                              (UWORD32)rMAC_SEC_CHAN_SLOT_COUNT );

    /*************************************************************************/
    /* Retry Registers                                                       */
    /*************************************************************************/

    /* The Retry limit register values are set to the MIB values at          */
    /* initialization. These need to be updated if the retry limits change.  */

    set_machw_lrl(mget_LongRetryLimit());
    set_machw_srl(mget_ShortRetryLimit());

    /*************************************************************************/
    /* Miscellaneous transmit registers                                      */
    /*************************************************************************/

    //chenq mod
    //rMAC_TX_MSDU_LIFETIME = convert_to_le(mget_MaxTransmitMSDULifetime());
    host_write_trout_reg( convert_to_le(mget_MaxTransmitMSDULifetime()), 
                          (UWORD32)rMAC_TX_MSDU_LIFETIME );

    /*************************************************************************/
    /* Sequence Number Registers                                             */
    /*************************************************************************/

    /* These registers are not initialized and will be set as required on    */
    /* packet transmissions.                                                 */

    /*************************************************************************/
    /* Interrupt Registers                                                   */
    /*************************************************************************/

    /* The Interrupt mask is set to initial value (mask all interrupts). The */
    /* required interrupts need to be unmasked as required. All other        */
    /* registers are Read Only.                                              */

    //chenq mod
    //rMAC_INT_MASK = convert_to_le(MAC_INT_MASK_INIT_VALUE);
    //rMAC_ERROR_MASK = convert_to_le(MAC_ERROR_MASK_INIT_VALUE);

    //chenq mod for fm int mask 2013-03-05
    //host_write_trout_reg(convert_to_le((MAC_INT_MASK_INIT_VALUE<<2)), (UWORD32)rCOMM_INT_MASK);
    host_write_trout_reg(convert_to_le(MAC_INT_MASK_INIT_VALUE), (UWORD32)rCOMM_INT_MASK);
    
    host_write_trout_reg(convert_to_le(MAC_ERROR_MASK_INIT_VALUE),  (UWORD32)rMAC_ERROR_MASK);

    /*************************************************************************/
    /* PHY Interface and Parameters Register                                 */
    /*************************************************************************/

    //chenq mod
    //rMAC_PHY_RF_REG_BASE_ADDR  = convert_to_le(
                                 //MAC_PHY_RF_REG_BASE_ADDR_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_PHY_RF_REG_BASE_ADDR_INIT_VALUE), 
					        (UWORD32)rMAC_PHY_RF_REG_BASE_ADDR );
    
    //rMAC_TXPLCP_DELAY          = convert_to_le(
                                 //MAC_TXPLCP_DELAY_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_TXPLCP_DELAY_INIT_VALUE), 
					        (UWORD32)rMAC_TXPLCP_DELAY );
    
    //rMAC_RXPLCP_DELAY          = convert_to_le(
                                 //MAC_RXPLCP_DELAY_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_RXPLCP_DELAY_INIT_VALUE), 
					        (UWORD32)rMAC_RXPLCP_DELAY );
    
    //rMAC_RXTXTURNAROUND_TIME   = convert_to_le(
                                 //MAC_RXTXTURNAROUND_TIME_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_RXTXTURNAROUND_TIME_INIT_VALUE), 
					        (UWORD32)rMAC_RXTXTURNAROUND_TIME );
    
    //rMAC_PHY_SERVICE_FIELD     = convert_to_le(MAC_PHY_SERVICE_FIELD_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_PHY_SERVICE_FIELD_INIT_VALUE), 
					        (UWORD32)rMAC_PHY_SERVICE_FIELD );

    enable_machw_rx_watchdog_timer(MAC_RX_WATCHDOG_TIMER_INIT_VALUE);

    //rMAC_PHY_CCA_DELAY         = convert_to_le(
                                 //MAC_PHY_CCA_DELAY_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_PHY_CCA_DELAY_INIT_VALUE), 
					        (UWORD32)rMAC_PHY_CCA_DELAY );

    //rMAC_TXPLCP_ADJUST_VAL     = convert_to_le(
                                 //MAC_TXPLCP_ADJUST_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_TXPLCP_ADJUST_INIT_VALUE), 
					        (UWORD32)rMAC_TXPLCP_ADJUST_VAL );

    //rMAC_RXPLCP_DELAY2         = convert_to_le(
                                 //MAC_RXPLCP_DELAY2_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_RXPLCP_DELAY2_INIT_VALUE), 
					        (UWORD32)rMAC_RXPLCP_DELAY2 );

    //rMAC_RXSTART_DELAY_REG     = convert_to_le(
                                 //MAC_RXSTART_DELAY_REG_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_RXSTART_DELAY_REG_INIT_VALUE), 
					        (UWORD32)rMAC_RXSTART_DELAY_REG );

// 20120830 caisf mod, merged ittiam mac v1.3 code
#if 0
    //rMAC_ANTENNA_SET           = convert_to_le(MAC_ANTENNA_SET_INIT_VALUE);
    host_write_trout_reg( convert_to_le(MAC_ANTENNA_SET_INIT_VALUE), 
					        (UWORD32)rMAC_ANTENNA_SET );
#else
    set_machw_ant_set(get_curr_tx_ant_set_prot());
#endif

// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef MWLAN
    set_mac_oddr_inv_phy_txclk();
    set_mac_oddr_inv_phyreg_ifclk();
#endif /* MWLAN */

    /*************************************************************************/
    /* Power Management Register Initialization                              */
    /*************************************************************************/

    set_machw_ps_active_mode();
    reset_machw_ps_pm_tx_bit();
    set_machw_offset_interval(MAC_OFFSET_INTERVAL_INIT_VALUE);

#ifdef MAC_802_11N
    set_machw_smps_rx_mode(mget_MIMOPowerSave());
#endif /* MAC_802_11N */


    /*************************************************************************/
    /* Miscellaneous Register Addresses (Arbiter, DMA, Host select etc)      */
    /*************************************************************************/


#ifndef GENERIC_PLATFORM
    /*************************************************************************/
    /* Arbiter Register Initialization                                       */
    /*************************************************************************/
	//masked by Hug: no such register.
    //chenq mod
    //rMAC_TRIG_ARBITER          = convert_to_le(
                                 //MAC_TRIG_ARBITER_CPU_PROG_INIT_VALUE);
    //host_write_trout_reg( convert_to_le(MAC_TRIG_ARBITER_CPU_PROG_INIT_VALUE), 
	//				        (UWORD32)rMAC_TRIG_ARBITER );
    
    //rMAC_PROG_ARBIT_POLICY     = convert_to_le(
                                 //MAC_PROG_ARBIT_POLICY_INIT_VALUE);
    //host_write_trout_reg( convert_to_le(MAC_PROG_ARBIT_POLICY_INIT_VALUE), 
	//				        (UWORD32)rMAC_PROG_ARBIT_POLICY );
    
    //rMAC_PROG_WEIGHT_RR        = convert_to_le(
                                 //MAC_PROG_WEIGHT_RR_INIT_VALUE);
   // host_write_trout_reg( convert_to_le(MAC_PROG_WEIGHT_RR_INIT_VALUE), 
	//				        (UWORD32)rMAC_PROG_WEIGHT_RR );
    
    //rMAC_PROG_PREEMPT          = convert_to_le(
                                 //MAC_PROG_PREEMPT_INIT_VALUE);
    //host_write_trout_reg( convert_to_le(MAC_PROG_PREEMPT_INIT_VALUE), 
	//				        (UWORD32)rMAC_PROG_PREEMPT );
    
    //rMAC_TRIG_ARBITER          = convert_to_le(
                                 //MAC_TRIG_ARBITER_INIT_VALUE);
    //host_write_trout_reg( convert_to_le(MAC_TRIG_ARBITER_INIT_VALUE), 
	//				        (UWORD32)rMAC_TRIG_ARBITER );
#endif /* GENERIC_PLATFORM */

#ifdef MWLAN
    /*************************************************************************/
    /* DMA Register Initialization                                           */
    /*************************************************************************/
    //rMAC_DMA_BURST_SIZE        = convert_to_le(MAC_DMA_BURST_SIZE_INIT_VALUE);
/*  //Hugh
    host_write_trout_reg( convert_to_le(MAC_DMA_BURST_SIZE_INIT_VALUE), 
					        (UWORD32)rMAC_DMA_BURST_SIZE );
*/
#endif /* MWLAN */

	// 20120709 caisf add, merged ittiam mac v1.2 code
	/*************************************************************************/
	/* P2P Register Initialization                                           */
	/*************************************************************************/
#ifdef MAC_P2P
    set_machw_p2p_eoa_offset(MAC_OFFSET_INTERVAL_INIT_VALUE);
#endif /* MAC_P2P */

    /* Queue interface table is initialized */
    init_qif_table();

    /* The LUT is initialized */
    initialize_lut();

    /*************************************************************************/
    /* Platform Dependent Initialization                                     */
    /*************************************************************************/
#ifdef MWLAN
    reset_pa_bus_clock_sync();
#endif /* MWLAN */
	TROUT_FUNC_EXIT;
}

#ifdef MAC_WMM
/*****************************************************************************/
/*                                                                           */
/*  Function Name : update_edca_machw                                        */
/*                                                                           */
/*  Description   : This function initializes MAC hardware interface related */
/*                  to EDCA protocol.                                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function sets the registers to their initial value. */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#ifdef IBSS_BSS_STATION_MODE
	extern volatile UWORD8 g_cmcc_test_mode;
#endif

void update_edca_machw(void)
{
	TROUT_FUNC_ENTER;
#ifndef BSS_ACCESS_POINT_MODE
    if(mget_DesiredBSSType() != ANY_BSS)
    {
        set_machw_aifsn_all_ac(mget_EDCATableAIFSN(AC_BK),
                               mget_EDCATableAIFSN(AC_BE),
                               mget_EDCATableAIFSN(AC_VI),
                               mget_EDCATableAIFSN(AC_VO));
/*
        set_machw_cw_bk(mget_EDCATableCWmax(AC_BK),
                                        mget_EDCATableCWmin(AC_BK));
        set_machw_cw_be(mget_EDCATableCWmax(AC_BE),
                                        mget_EDCATableCWmin(AC_BE));
        set_machw_cw_vi(mget_EDCATableCWmax(AC_VI),
                                        mget_EDCATableCWmin(AC_VI));
        set_machw_cw_vo(mget_EDCATableCWmax(AC_VO),
                                        mget_EDCATableCWmin(AC_VO));
                                        */
                                        
        set_machw_cw_bk(10,4);
        set_machw_cw_be(10,4);
        set_machw_cw_vi(4,3);
        set_machw_cw_vo(3,2);

        set_machw_txop_limit_bkbe(mget_EDCATableTXOPLimit(AC_BK),
                                  mget_EDCATableTXOPLimit(AC_BE));
        set_machw_txop_limit_vovi(mget_EDCATableTXOPLimit(AC_VO),
                                  mget_EDCATableTXOPLimit(AC_VI));

        set_machw_edca_bkbe_lifetime(mget_EDCATableMSDULifetime(AC_BK),
                                     mget_EDCATableMSDULifetime(AC_BE));
        set_machw_edca_vivo_lifetime(mget_EDCATableMSDULifetime(AC_VI),
                                     mget_EDCATableMSDULifetime(AC_VO));
#if 0
		if(g_cmcc_test_mode == 1)
		{
			set_machw_cw_vi(2, 2);
			set_machw_cw_vo(2, 2);
			set_machw_cw_be(2, 2);
			set_machw_cw_bk(2, 2);
		}
#endif
	}
    else
#endif /* BSS_ACCESS_POINT_MODE */
    {
        set_machw_aifsn_all_ac(mget_QAPEDCATableAIFSN(AC_BK),
                               mget_QAPEDCATableAIFSN(AC_BE),
                               mget_QAPEDCATableAIFSN(AC_VI),
                               mget_QAPEDCATableAIFSN(AC_VO));
/*
        set_machw_cw_bk(mget_QAPEDCATableCWmax(AC_BK),
                                        mget_QAPEDCATableCWmin(AC_BK));
        set_machw_cw_be(mget_QAPEDCATableCWmax(AC_BE),
                                        mget_QAPEDCATableCWmin(AC_BE));
        set_machw_cw_vi(mget_QAPEDCATableCWmax(AC_VI),
                                        mget_QAPEDCATableCWmin(AC_VI));
        set_machw_cw_vo(mget_QAPEDCATableCWmax(AC_VO),
                                        mget_QAPEDCATableCWmin(AC_VO));
*/
        set_machw_cw_bk(10,4);
        set_machw_cw_be(10,4);
        set_machw_cw_vi(4,3);
        set_machw_cw_vo(3,2);
        
        set_machw_txop_limit_bkbe(mget_QAPEDCATableTXOPLimit(AC_BK),
                                  mget_QAPEDCATableTXOPLimit(AC_BE));
        set_machw_txop_limit_vovi(mget_QAPEDCATableTXOPLimit(AC_VO),
                                  mget_QAPEDCATableTXOPLimit(AC_VI));

        set_machw_edca_bkbe_lifetime(mget_QAPEDCATableMSDULifetime(AC_BK),
                                     mget_QAPEDCATableMSDULifetime(AC_BE));
        set_machw_edca_vivo_lifetime(mget_QAPEDCATableMSDULifetime(AC_VI),
                                     mget_QAPEDCATableMSDULifetime(AC_VO));
    }
    TROUT_FUNC_EXIT;
}

#endif /* MAC_WMM */

#ifndef MAC_HW_UNIT_TEST_MODE

/*****************************************************************************/
/*                                                                           */
/*  Function Name : mac_isr                                                  */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the MAC H/w    */
/*                  interrupt.                                               */
/*                                                                           */
/*  Inputs        : 1) Interrupt vector                                      */
/*                  1) Interrupt data                                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the MAC H/w interrupt status and    */
/*                  mask registers to determine which interrupt has occurred */
/*                  and calls the required interrupt service routine. It     */
/*                  acknowledges the interrupt.                              */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD32 tx_count[4] = {0,0,0,0};   //0: invalid; 1: pending; 2: not pending; 3: tx time out.
UWORD32 rx_count[4] = {0,0,0,0};   //0: invalid; 1: rx ok; 2: duplicate detected; 3: fcs error.

#ifdef TROUT_TRACE_DBG    
#include <linux/gpio.h>
#endif	/* TROUT_TRACE_DBG */


#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
BOOL_T	host_awake_by_arm7 = BFALSE;
#endif
#endif

/*extern void show_tx_slots(void);*/
extern int buffer_frame_count;
irqreturn_t mac_isr_work(int irq, void *dev)
{    
	unsigned int ciso;
	unsigned int cints;
	unsigned int mints, v, cnt = 0;
	UWORD32 cmsk, statu = 0;
	UWORD32 arm2host = 0;
	int tx_flag = 0;
	struct trout_private *stp = netdev_priv(g_mac_dev);
	UWORD32 info3 = 0;
	if(reset_mac_trylock() == 0){
		return IRQ_HANDLED;
	}
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	mutex_lock(&stp->sm_mutex);
#ifdef WAKE_LOW_POWER_POLICY
	if(g_wifi_power_mode == WIFI_LOW_POWER_MODE)	//add by chwg.
	{
		exit_low_power_mode(BTRUE);

		info3 = host_read_trout_reg((UWORD32)rCOMM_ARM2HOST_INFO3);
		printk("%s: %x %s %s %s waiting for wifi resume done\n", __func__, info3,
								(((info3 & 0x10000) == 0x10000) ? "DTIM":"none"),
								(((info3 & 0x20000) == 0x20000) ? "TIM":"none"),
								(((info3 & 0x40000) == 0x40000) ? "ARM7":"none"));

		host_write_trout_reg(info3 & (~(BIT18 | BIT17 | BIT16)), (UWORD32)rCOMM_ARM2HOST_INFO3);
	}
 else if(g_wifi_suspend_status == wifi_suspend_early_suspending)
#else
      //if when suspending, the trout interrupt is coming, do nothing and exit. add by puyan
      if(g_wifi_suspend_status == wifi_suspend_early_suspending)
#endif
      {
             pr_info("%s: when suspending, do nothing and exit. \n", __func__);
             mutex_unlock(&stp->sm_mutex);
             reset_mac_unlock();
             return IRQ_HANDLED;
      }
	if (g_wifi_suspend_status == wifi_suspend_early_suspend) 
	{
		pr_info("%s: waiting for wifi resume\n", __func__);
		if (!wait_for_completion_timeout(&wifi_resume_completion, msecs_to_jiffies(500))) 
		{
			pr_info("******** ignore wrong INTR\n");
	             mutex_unlock(&stp->sm_mutex);
			reset_mac_unlock();
			return IRQ_HANDLED;
		}
		pr_info("%s: waiting for wifi resume done\n", __func__);
		info3 = host_read_trout_reg((UWORD32)rCOMM_ARM2HOST_INFO3);
		printk("%s: %x %s %s %s waiting for wifi resume done\n", __func__, info3,
								(((info3 & 0x10000) == 0x10000) ? "DTIM":"none"),
								(((info3 & 0x20000) == 0x20000) ? "TIM":"none"),
								(((info3 & 0x40000) == 0x40000) ? "ARM7":"none"));

		host_write_trout_reg(info3 & (~(BIT18 | BIT17 | BIT16)), (UWORD32)rCOMM_ARM2HOST_INFO3);
	}
#endif
#endif
	cints = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT));
	ciso = cints;
	cints = (cints >> 2) & 0x7FFFF;
	
	cmsk = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_MASK));
#if 0
#ifdef IBSS_BSS_STATION_MODE
	printk("\n======= mac_isr_work ========\n[%s] netif_queue_stopped = %d\n", __FUNCTION__, netif_queue_stopped(g_mac_dev));
	printk("[%s] rCOMM_INT_STAT=%#x, rCOMM_ARM2HOST_INFO3=%#x,  rCOMM_INT_MASK = %#x, g_wifi_bt_coex=%d\n", 
						__FUNCTION__,ciso, host_read_trout_reg((UWORD32)rCOMM_ARM2HOST_INFO3), cmsk, g_wifi_bt_coex);
	
	{
		UWORD32 tmp = 0;
		host_read_trout_ram(&tmp, (UWORD32*)TROUT_MEM_CFG_BEGIN, 4);
		printk("[%s] TROUT_MEM_CFG_BEGIN = %#x\n", __FUNCTION__, tmp);
	}
	{
		UWORD32 tmp32 = 0;		
		host_read_trout_ram((void *)&tmp32, (void *)(COEX_SELF_CTS_NULL_DATA_BEGIN-4), sizeof(UWORD32));
		printk("[%s] index from BT: %#x\n", __FUNCTION__, tmp32);
		//host_read_trout_ram((void *)&tmp32, (void *)(COEX_PS_NULL_DATA_BEGIN), sizeof(UWORD32));
		//printk("[%s] PS NULL data: %#x\n", __FUNCTION__, tmp32);
	}
	//print_qif_table_info();
	//tx_shareram_slot_stat();

#endif
#endif
	cmsk = (cmsk >> 2) & 0x7FFFF;
	
	mints = convert_to_le(host_read_trout_reg((UWORD32)rMAC_INT_STAT));
	//mmsk = convert_to_le(host_read_trout_reg((UWORD32)rMAC_INT_MASK));
	statu = mints & (~cmsk);

	mints &= 0x7FFFF;
	//mmsk &= 0x7FFFF; 

#ifdef IBSS_BSS_STATION_MODE
	if((cmsk & BIT1) && (!g_wifi_bt_coex))
	{
		printk("==== Enter WiFi/BT Coexist mode ====\n");
		g_wifi_bt_coex = BTRUE;
	}
	else if(g_wifi_bt_coex && (!(cmsk & BIT1)))
	{
		printk("==== Exit WiFi/BT Coexist mode ====\n");
		g_wifi_bt_coex = BFALSE;
		tx_complete_isr();
		exit_from_coexist_mode();
	}
#endif /* IBSS_BSS_STATION_MODE */

	//if(cmsk != mmsk || cints != mints)
	//	printk("CINTS: %X, CMSK:%X, MINTS:%X, MMSK:%X\n", cints, cmsk, mints,mmsk);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	if((ciso>>2) & BIT29)
	{
	arm2host_irq_cnt++;
#ifdef POWERSAVE_DEBUG
        printk("PS mac_isr: bit29 arm7 interrupt host !\n");
#endif
#ifdef IBSS_BSS_STATION_MODE	
		if(g_wifi_bt_coex)
		{
			arm2host = host_read_trout_reg((UWORD32)rCOMM_ARM2HOST_INFO3);
			if(arm2host & BIT0)	//tx int.
			{
				TROUT_DBG5("%s: arm7 notify host tx int occur(0x%x)!\n", __func__, arm2host);

				tx_flag = 1;
				tx_complete_isr();
				arm2host &= (~BIT0);	//clean tx int at this func end				
			}
		}
#endif	/* IBSS_BSS_STATION_MODE */

		arm2host_irq_cnt++;
#ifdef WIFI_SLEEP_POLICY
	if(host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM) == PS_MSG_ARM7_SIMULATE_HIGH_RX_INT_MAGIC)
	{
		printk("PS mac_isr: bit29 arm7 interrupt host! hacking\n");
		host_write_trout_reg(0x5A, (UWORD32)rSYSREG_INFO2_FROM_ARM);
		host_write_trout_reg(arm2host_irq_cnt, (UWORD32)rSYSREG_HOST2ARM_INFO2);
		wake_lock(&buffer_frame_lock);
		/*printk("sysreg_info2_from_arm = %x\n",host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM));*/
		/*pr_info("%s: %d OK! rSYSREG_INFO2_FROM_ARM = %x, arm2host_irq_cnt = %x\n", __func__, __LINE__, host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM),arm2host_irq_cnt);*/
		/*pr_info("resume ps_last_int_mask = %#x; \n",ps_last_int_mask);*/
		host_write_trout_reg(ps_last_int_mask , (UWORD32)rCOMM_INT_MASK);
		update_trout_int_mask(ps_last_int_mask);

		//host_awake_by_arm7 = BTRUE;
		//rx_complete_isr(HIGH_PRI_RXQ);
		/*pr_info("rMAC_PA_CON = %#x\n", root_host_read_trout_reg((UWORD32)rMAC_PA_CON));
		show_tx_slots();*/
		buffer_frame_count = 10;
		mod_timer(&buffer_frame_timer, jiffies + msecs_to_jiffies(30));
#ifdef POWERSAVE_DEBUG
		pr_info("get ready for TBTT!\n");
#endif
		/*tbtt_isr(); [>TBTT emulation<] */
	} else {
		pr_info("Get msg from arm7: %#x\n",host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM));
		host_write_trout_reg(0x5A, (UWORD32)rSYSREG_INFO2_FROM_ARM);
		host_write_trout_reg(arm2host_irq_cnt, (UWORD32)rSYSREG_HOST2ARM_INFO2);
	}
#else
		if(host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM) == PS_MSG_HOST_GET_READY_FOR_TBTT_MAGIC)
		{
	       	host_write_trout_reg(0x5A, (UWORD32)rSYSREG_INFO2_FROM_ARM);
			/*printk("sysreg_info2_from_arm = %x\n",host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM));*/
			/*pr_info("%s: %d OK! rSYSREG_INFO2_FROM_ARM = %x, arm2host_irq_cnt = %x\n", __func__, __LINE__, host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM),arm2host_irq_cnt);*/
			//  caisf add for unmask relate intterupt 0319                  
			/*pr_info("resume ps_last_int_mask = %#x; \n",ps_last_int_mask);*/
	        host_write_trout_reg(ps_last_int_mask , (UWORD32)rCOMM_INT_MASK);
	        update_trout_int_mask(ps_last_int_mask);

#ifdef POWERSAVE_DEBUG
			pr_info("get ready for TBTT!\n");
#endif
			//print_ps_reg();
			tbtt_isr(); /*TBTT emulation*/ 
	       	host_write_trout_reg(arm2host_irq_cnt, (UWORD32)rSYSREG_HOST2ARM_INFO2);
		}
    	else
	    {
	        //pr_info("Get msg from arm7!\n");
	        //pr_info("Get msg: %#x\n",host_read_trout_reg((UWORD32)rSYSREG_INFO2_FROM_ARM));

        	host_write_trout_reg(0x5A, (UWORD32)rSYSREG_INFO2_FROM_ARM);
        	host_write_trout_reg(arm2host_irq_cnt, (UWORD32)rSYSREG_HOST2ARM_INFO2);
    	}
#endif
	}
#endif	/* TROUT_WIFI_POWER_SLEEP_ENABLE */

	/* if these two regs not same, we must depend on the mac regs*/
	if(mints & BIT7){
	#ifdef DEBUG_MODE
		g_mac_stats.icapend++;
	#endif /* DEBUG_MODE */
	#ifdef TROUT_TRACE_DBG
		mac_isr_reg_count++;
	#endif
		reset_machw_cap_end_int();//just clear the comm regs, so we need clear mac regs for safe
		//mints &= ~BIT7;
	}

	if(mints & BIT0){
	#ifdef DEBUG_MODE
		g_mac_stats.irxc++;
	#endif /* DEBUG_MODE */
		rx_complete_isr(NORMAL_PRI_RXQ);
	#ifdef MEASURE_PROCESSING_DELAY
		g_delay_stats.rxcmpint++;
	#endif /* MEASURE_PROCESSING_DELAY */
	}

	if(mints & BIT10){
	#ifdef DEBUG_MODE
		g_mac_stats.ihprxc++;
	#endif /* DEBUG_MODE */
		rx_complete_isr(HIGH_PRI_RXQ);
	}

#ifdef IBSS_BSS_STATION_MODE
	if((!g_wifi_bt_coex) && (statu & BIT1))
#else
	if(mints & BIT1)
#endif	/* IBSS_BSS_STATION_MODE */
	{
	#ifdef DEBUG_MODE
		g_mac_stats.itxc++;
	#endif /* DEBUG_MODE */
		get_machw_tx_frame_pointer();	//read tx fifo to clear it.
		//printk("%s: tx int from host!\n", __func__);
		tx_complete_isr();
	#ifdef MEASURE_PROCESSING_DELAY
		g_delay_stats.txcmpint++;
	#endif /* MEASURE_PROCESSING_DELAY */
	}

	if(mints & BIT2){
	#ifdef DEBUG_MODE
		g_mac_stats.itbtt++;
	#endif /* DEBUG_MODE */
		
		tbtt_isr();
		reset_machw_tbtt_int();
	#ifdef TROUT_TRACE_DBG
		mac_isr_reg_count++;
	#endif                
		//mints &= ~BIT2;
	}

	if(mints & BIT6){
	#ifdef DEBUG_MODE
		g_mac_stats.ierr++;
	#endif /* DEBUG_MODE */
		error_isr();
		reset_machw_error_int();
	#ifdef TROUT_TRACE_DBG
		mac_isr_reg_count++;
	#endif                
		//mints &= ~BIT6;
	}

	#ifdef TX_ABORT_FEATURE
	if(mints & BIT11){
		g_machw_tx_aborted = BTRUE;
		tx_abort_start_isr();
		reset_tx_abort_start_int();
		mints &= ~BIT11;
	}
	if(mints & BIT12){
		g_machw_tx_aborted = BFALSE;
		reset_tx_abort_end_int();
		mints &= ~BIT12;
	}
	#endif /* TX_ABORT_FEATURE */

	if(mints & BIT13){
		radar_detect_isr();
		reset_machw_radar_det_int(); //just clear the commom reg
	#ifdef TROUT_TRACE_DBG
		mac_isr_reg_count++;
	#endif                
		//mints &= ~BIT13;
	}

	if(mints & BIT14){
		/* clear TX suspend, we must first set pa con, then wait for PA 
		 * status bit6 is 0, then clear the int status bit14
		 */
		v = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PA_CON));
		v = v & (~BIT12);
		host_write_trout_reg(convert_to_le(v), (UWORD32)rMAC_PA_CON);

		while((host_read_trout_reg( (UWORD32)rMAC_PA_STAT) & REGBIT6) != 0){
			if(++cnt > 1000)
				break;
		}
		if(cnt > 1000)
			printk("%s ..wait PA status BIT6 to 0 timeout!\n", __func__);		
	}
	if(mints & BIT15){
	#ifdef DEBUG_MODE
		g_mac_stats.ideauth++;
	#endif
		//deauth_isr();
		//reset_machw_deauth_int();//just clear the common reg
	#ifdef TROUT_TRACE_DBG
		mac_isr_reg_count++;
	#endif                
		//mints &= ~BIT15;
	}

	#ifdef IBSS_BSS_STATION_MODE
	if(mac_isr_prot(mints)){
	/* Do nothing */
	}
	#endif /* IBSS_BSS_STATION_MODE */

	/* for safe reason, we'd better to clear both mac and comm regs by zhao */
	host_write_trout_reg(mints, (UWORD32)rMAC_INT_STAT);
	host_write_trout_reg(ciso, (UWORD32)rCOMM_INT_CLEAR);

#ifdef IBSS_BSS_STATION_MODE
	if(tx_flag)
	{
		host_write_trout_reg(arm2host, (UWORD32)rCOMM_ARM2HOST_INFO3);	//clear tx int, must be here.
	}	
#endif
	
	TROUT_FUNC_EXIT;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
    if((ciso>>2) & BIT29)
    {
	     pr_info("******** %s: done\n", __func__);
    }
	mutex_unlock(&stp->sm_mutex);
#endif
#endif
      reset_mac_unlock();
	return IRQ_HANDLED;
}

void tx_complete_isr_simulate(void)
{
	int slot = 0;
	UWORD8 num_tx_dscr = 0;
	UWORD32  *tx_dscr_base = NULL;
	int cnt = 0;
	misc_event_msg_t *misc = 0;
	/* Wait on the transmission of this deauth */
       while((is_machw_q_null(HIGH_PRI_Q) == BFALSE) &&
               (cnt < DEAUTH_SEND_TIME_OUT_COUNT))
       {
            add_delay(0xFFF);
            cnt++;
       }
	get_machw_tx_frame_pointer();	//read tx fifo to clear it.
    	for(slot=0; slot<tx_shareram_slot_nr(); slot++){
		tx_dscr_base = NULL;
		num_tx_dscr = 0;
       	if(!tx_shareram_slot_busy(slot)){
            		continue;
       	}
        	mutex_lock(&g_q_handle.tx_handle.txq_lock);
	  	num_tx_dscr = tx_pkt_process(slot,TX_ISR_CALL,&tx_dscr_base);
	  	if( num_tx_dscr == 0 ){
	  		mutex_unlock(&g_q_handle.tx_handle.txq_lock);
			continue ;
	  	}
		mutex_unlock(&g_q_handle.tx_handle.txq_lock);
    	   	/* Allocate buffer for the miscellaneous event */
         	misc = (misc_event_msg_t*)_event_mem_alloc(MISC_EVENT_QID);

    		/* If allocation fails, then simply return doing nothing. Whenever a new */
    		/* is read out of the Tx-Q the sync operation will clear these packets.  */
    		if(misc == NULL){
			printk("TX can not alloc MISC mem\n");
#ifdef DEBUG_MODE
        		g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */
        		continue;
    		}

    		/* Create the Tx Complete miscellanous event with the pointer to the     */
    		/* first descriptor and the number of descriptors.                       */
    		misc->name = MISC_TX_COMP;
    		misc->info = num_tx_dscr;
    		misc->data = (UWORD8 *)tx_dscr_base;
		process_misc_event((UWORD32)misc);
		event_mem_free((void*)misc,MISC_EVENT_QID);
    }
}
#if 0
/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_complete_isr                                          */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the transmit   */
/*                  complete interrupt. This interrupt is raised when        */
/*                  1) a successful transmission occurs, 2) the retry limit  */
/*                  is reached or 3) packet lifetime expires. The status is  */
/*                  available in the Transmit Descriptor of the packet. When */
/*                  this interrupt is raised, a register is populated with   */
/*                  the address of the Transmit Descriptor.                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function reads the transmit descriptor from the MAC */
/*                  hardware register. It creates a MISC_TX_COMP event with  */
/*                  the message set to the pointer to the descriptor. This   */
/*                  event is posted to the event queue.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void tx_complete_isr(void)
{
    UWORD8           num_tx_dscr = 0;
    UWORD32          *tx_dscr    = NULL;
    misc_event_msg_t *misc       = 0;
    UWORD32 i;
    UWORD32 trout_descr, trout_next_descr, temp32[3];
    UWORD32 next_descr = 0;
    UWORD32  *tx_dscr_base = NULL;
    
    /* Read the hardware register with the Transmit descriptor address */
    trout_descr = get_machw_tx_frame_pointer();
    
    /* This interrupt can be raised for multiple transmitted frames. Read    */
    /* the hardware register with the number of frames.                      */
    num_tx_dscr = get_machw_num_tx_frames();

    //printk("trout(txisr): trout_dsc=%p, num=%d\n", (UWORD8 *)trout_descr, num_tx_dscr);
    mutex_lock(&g_q_handle.tx_handle.txq_lock);
    
    for(i=0; i<num_tx_dscr; i++)
    {
        if(trout_descr >= TX_MEM_END || trout_descr < TX_MEM_BEGIN)
        {
            mutex_unlock(&g_q_handle.tx_handle.txq_lock);
            printk("trout: bad trout tx descr:0x%x\n", trout_descr);
            return;
        }

        host_read_trout_ram(&tx_dscr, (UWORD32 *)trout_descr + WORD_OFFSET_35, 4);
        //printk("trout(txisr): trout_descr=0x%08x, tx_dscr=0x%p\n", trout_descr, tx_dscr);
		if(tx_dscr == NULL || validate_buffer(tx_dscr) == BFALSE || 
						(UWORD32)tx_dscr != get_tx_dscr_host_dscr_addr(tx_dscr))
        {
            mutex_unlock(&g_q_handle.tx_handle.txq_lock);
            printk("trout: bad host tx descr:%p\n", tx_dscr);
            return;
        }

        next_descr = *(tx_dscr + 4);
        host_read_trout_ram(tx_dscr, (UWORD32 *)trout_descr,  6 * 4);
        trout_next_descr = *(tx_dscr + 4);
        
        *(tx_dscr + 4) = next_descr;	//restore host next_descr value.
        
        host_read_trout_ram(tx_dscr + 26, (UWORD32 *)trout_descr + 26,  2*4);

        /* invalid trout dscr status bits */
        temp32[2] = 0;
        host_write_trout_ram((UWORD32 *)trout_descr, &temp32[2], 4);
   
        if(i == 0)
        {
            tx_dscr_base = tx_dscr;
        }

        trout_descr = trout_next_descr;
    }

    mutex_unlock(&g_q_handle.tx_handle.txq_lock);
    
#ifdef BURST_TX_MODE
    if(g_burst_tx_mode_enabled == BTRUE)
    {
        update_burst_mode_tx_dscr((UWORD32 *)tx_dscr);
        return;
    }
#endif /* BURST_TX_MODE */

    /* Allocate buffer for the miscellaneous event */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    /* If allocation fails, then simply return doing nothing. Whenever a new */
    /* is read out of the Tx-Q the sync operation will clear these packets.  */
    if(misc == NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */

        return;
    }

    /* Create the Tx Complete miscellanous event with the pointer to the     */
    /* first descriptor and the number of descriptors.                       */
    misc->name = MISC_TX_COMP;
    misc->info = num_tx_dscr;
    //misc->data = (UWORD8 *)tx_dscr;   //Hugh
    misc->data = (UWORD8 *)tx_dscr_base;

    /* Post the event */
    post_event((UWORD8*)misc, MISC_EVENT_QID);
#if 1
    mutex_lock(&g_q_handle.tx_handle.txq_lock);
    if(is_all_machw_q_null() == BTRUE)
    {
        trout_load_qmu();   //reload descr to trout.
    }
    
    mutex_unlock(&g_q_handle.tx_handle.txq_lock);
#endif
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.misctxcompevent++;
#endif /* MEASURE_PROCESSING_DELAY */

}
#else

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_complete_isr                                          */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the transmit   */
/*                  complete interrupt. This interrupt is raised when        */
/*                  1) a successful transmission occurs, 2) the retry limit  */
/*                  is reached or 3) packet lifetime expires. The status is  */
/*                  available in the Transmit Descriptor of the packet. When */
/*                  this interrupt is raised, a register is populated with   */
/*                  the address of the Transmit Descriptor.                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function reads the transmit descriptor from the MAC */
/*                  hardware register. It creates a MISC_TX_COMP event with  */
/*                  the message set to the pointer to the descriptor. This   */
/*                  event is posted to the event queue.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#if 0
void tx_pkt_process(void)
{
    UWORD8           num_tx_dscr = 0, qnum = 0, status;
    UWORD32          *tx_dscr    = NULL;
    misc_event_msg_t *misc       = 0;
    UWORD32 trout_descr, trout_next_descr, temp32[3];
    UWORD32 next_descr = 0;
    UWORD32  *tx_dscr_base = NULL;
#ifdef TROUT_TRACE_DBG
	UWORD32 id = 0;
	static int i = 0;
#endif

    TROUT_FUNC_ENTER;
    
	num_tx_dscr = 0;
	qnum = g_q_handle.tx_handle.tx_curr_qnum;
	trout_descr = g_q_handle.tx_handle.tx_header[qnum].trout_head;

    //printk("trout_head=%x\n", trout_descr);
	while(trout_descr < TX_MEM_END && trout_descr >= TX_MEM_BEGIN)
    {
#if 1 //dumy add for tx err 0823
        UWORD32 trout_dsc;
        host_read_trout_ram(&trout_dsc, (UWORD32 *)trout_descr, 4);
		status=get_tx_dscr_status(&trout_dsc);
    
    	if(status == INVALID)
		{
			TROUT_DBG2("err: first tx desc is invalid!\n");
			break;
		}
        
		if(status == PENDING)
		{
			//printk("first tx_end\n", status);
			break;
		}
#endif
        
        host_read_trout_ram(&tx_dscr, (UWORD32 *)trout_descr + WORD_OFFSET_35, 4);
        //printk("trout(txisr): trout_descr=0x%08x, tx_dscr=0x%p\n", trout_descr, tx_dscr);
		if(tx_dscr == NULL || validate_buffer(tx_dscr) == BFALSE || 
						(UWORD32)tx_dscr != get_tx_dscr_host_dscr_addr(tx_dscr))
        {
//            mutex_unlock(&g_q_handle.tx_handle.txq_lock);
            TROUT_DBG4("trout: bad host tx descr:%p\n", tx_dscr);
            TROUT_FUNC_EXIT;
            return;
        }
        
#ifdef TROUT_TRACE_DBG
		id = get_tx_dscr_identify_flag(tx_dscr);
		if(id == g_trace_count_buf[i])
		{
			TROUT_TX_DBG5("%s<%d>: id=%u\n", __func__, i, id);
			if(++i >= TJ_NUM)
				i = 0;
		}
#endif        
        next_descr = *(tx_dscr + 4);
        host_read_trout_ram(tx_dscr, (UWORD32 *)trout_descr,  6 * 4);
        trout_next_descr = *(tx_dscr + 4);
        
        *(tx_dscr + 4) = next_descr;	//restore host next_descr value.

		status = get_tx_dscr_status(tx_dscr);
        tx_count[status & 0x3]++;
		if(status == PENDING)
		{
			//printk("err: tx_uncompleted.\n");
			break;
		}
		else if(status == INVALID)
		{
			TROUT_DBG2("err: tx desc is invalid!\n");
			break;
		}

        host_read_trout_ram(tx_dscr + 26, (UWORD32 *)trout_descr + 26,  2*4);

        /* invalid trout dscr status bits */
        temp32[2] = 0;
        host_write_trout_ram((UWORD32 *)trout_descr, &temp32[2], 4);
        host_write_trout_ram((UWORD32 *)trout_descr + 4, &temp32[2], 4);
 
        if(num_tx_dscr == 0)
        {
            tx_dscr_base = tx_dscr;
        }

        //printk("trout_descr=%x, trout_next=%x\n", trout_descr, trout_next_descr);
        trout_descr = trout_next_descr;

        num_tx_dscr++;
    }

	g_q_handle.tx_handle.tx_header[qnum].trout_head = trout_descr;
//    mutex_unlock(&g_q_handle.tx_handle.txq_lock);
    
	if(tx_dscr_base == NULL || num_tx_dscr == 0)
	{
		TROUT_FUNC_EXIT;
		return;
    }
    
#ifdef BURST_TX_MODE
    if(g_burst_tx_mode_enabled == BTRUE)
    {
        update_burst_mode_tx_dscr((UWORD32 *)tx_dscr);
        TROUT_FUNC_EXIT;
        return;
    }
#endif /* BURST_TX_MODE */

    /* Allocate buffer for the miscellaneous event */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    /* If allocation fails, then simply return doing nothing. Whenever a new */
    /* is read out of the Tx-Q the sync operation will clear these packets.  */
    if(misc == NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }

    /* Create the Tx Complete miscellanous event with the pointer to the     */
    /* first descriptor and the number of descriptors.                       */
    misc->name = MISC_TX_COMP;
    misc->info = num_tx_dscr;
    misc->data = (UWORD8 *)tx_dscr_base;

    /* Post the event */
    post_event((UWORD8*)misc, MISC_EVENT_QID);
    TROUT_TX_DBG5("%s: post event 0x%x\n", __FUNCTION__, MISC_TX_COMP);
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.misctxcompevent++;
#endif /* MEASURE_PROCESSING_DELAY */

	TROUT_FUNC_EXIT;
}
//#else
void tx_pkt_process(UWORD8 slot, int call_flag)
{
    UWORD8           num_tx_dscr = 0, qnum = 0, status;
    UWORD32          *tx_dscr    = NULL;
    misc_event_msg_t *misc       = 0;
    UWORD32 trout_descr, trout_next_descr, temp32[3];
    UWORD32 next_descr = 0;
    UWORD32  *tx_dscr_base = NULL;
    UWORD32 tx_mem_begin = g_q_handle.tx_handle.tx_mem_start;
    UWORD32 tx_mem_end = g_q_handle.tx_handle.tx_mem_size + tx_mem_begin;

    TROUT_FUNC_ENTER;
	
    if(tx_shareram_slot_field_get(slot, &begin, &end, &qnum) < 0)
    {
        printk("Err: get shareram field fail.\n");
        return;
    }
    
	num_tx_dscr = 0;
	qnum = g_q_handle.tx_handle.tx_curr_qnum;
	trout_descr = g_q_handle.tx_handle.tx_header[qnum].trout_head;

    TX_INT_DBG("trout_head=0x%x(0x%x ~ 0x%x)\n", trout_descr, tx_mem_begin, tx_mem_end);
	//while(trout_descr < TX_MEM_END && trout_descr >= TX_MEM_BEGIN)
	while(trout_descr < tx_mem_end && trout_descr >= tx_mem_begin)
    {
    	UWORD32 h_dscr[TX_DSCR_LEN/4] = { 0 };
    	
		host_read_trout_ram((UWORD8 *)h_dscr, (UWORD8 *)trout_descr, sizeof(h_dscr));	//DMA!
		status=get_tx_dscr_status(h_dscr);
		tx_count[status & 0x3]++;
		if(status == INVALID)
		{
			TROUT_DBG4("err: first tx desc is invalid!\n");
			break;
		}
		if(status == PENDING)
		{
			TROUT_DBG5("warning: tx desc is not complete!\n");
			break;
		}

		tx_dscr = (UWORD32 *)(h_dscr[WORD_OFFSET_35]);
		if(tx_dscr == NULL || validate_buffer(tx_dscr) == BFALSE || 
						(UWORD32)tx_dscr != get_tx_dscr_host_dscr_addr(tx_dscr))
        {
            TROUT_DBG2("trout: bad host tx descr:%p\n", tx_dscr);
            TROUT_FUNC_EXIT;
            return;
        }
        
		next_descr = *(tx_dscr + 4);
		trout_next_descr = *(h_dscr + 4);

		memcpy((void *)tx_dscr, (void *)h_dscr, 6 * 4);
		*(tx_dscr + 4) = next_descr;	//restore host next_descr value.
		
		memcpy((void *)(tx_dscr+26), (void *)(h_dscr+26), 2 * 4);

//		temp32[2] = 0;
//		host_write_trout_ram((UWORD32 *)trout_descr, &temp32[2], 4);	//clear status.

        if(num_tx_dscr == 0)
        {
            tx_dscr_base = tx_dscr;
        }

        trout_descr = trout_next_descr;

        num_tx_dscr++;

        g_q_handle.tx_handle.tx_pkt_count--;
        g_q_handle.tx_handle.tx_header[qnum].handle_pkt--;	//chengwg add.
        tx_shareram_slot_packet_dec(slot);        
    }
	
	g_q_handle.tx_handle.tx_header[qnum].trout_head = trout_descr;
    
	if(tx_dscr_base == NULL || num_tx_dscr == 0)
	{
		TROUT_FUNC_EXIT;
		return;
    }
    
#ifdef BURST_TX_MODE
    if(g_burst_tx_mode_enabled == BTRUE)
    {
        update_burst_mode_tx_dscr((UWORD32 *)tx_dscr);
        TROUT_FUNC_EXIT;
        return;
    }
#endif /* BURST_TX_MODE */

    /* Allocate buffer for the miscellaneous event */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    /* If allocation fails, then simply return doing nothing. Whenever a new */
    /* is read out of the Tx-Q the sync operation will clear these packets.  */
    if(misc == NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.etxcexc++;
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }

    /* Create the Tx Complete miscellanous event with the pointer to the     */
    /* first descriptor and the number of descriptors.                       */
    misc->name = MISC_TX_COMP;
    misc->info = num_tx_dscr;
    misc->data = (UWORD8 *)tx_dscr_base;

    /* Post the event */
    post_event((UWORD8*)misc, MISC_EVENT_QID);
    TROUT_DBG5("%s: post event 0x%x, base: 0x%p, num: %d\n", 
    			__FUNCTION__, MISC_TX_COMP, tx_dscr_base, num_tx_dscr);
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.misctxcompevent++;
#endif /* MEASURE_PROCESSING_DELAY */

	TROUT_FUNC_EXIT;
}
#endif
void get_txrx_count(UWORD32 *tx_ok,
                       UWORD32 *tx_fail,
                       UWORD32 *rx_ok,
                       UWORD32 *rx_fail)
{
    if(tx_ok != NULL)
        *tx_ok = tx_count[NOT_PENDING];
    if(tx_fail != NULL)
        *tx_fail = tx_count[TX_TIMEOUT];
    
    if(rx_ok != NULL)
        *rx_ok = rx_count[RX_SUCCESS];
    if(rx_fail != NULL)
        *rx_fail = rx_count[RX_NEW]+ rx_count[DUP_DETECTED] + rx_count[FCS_ERROR];
}

EXPORT_SYMBOL(get_txrx_count);

#ifdef TROUT_WIFI_NPI
void qmu_cpy_npi_descr(int q_num);
extern int g_tx_flag;
//static int tx_num = 0;
extern struct trout_tx_shareram copy_ttr[2];  //two slot for NPI transfer
extern UWORD32 npi_addr[2];    //Queue start address inside trout
extern int copy_q_num;
extern int transfer_q_num;
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_complete_isr                                          */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the transmit   */
/*                  complete interrupt. This interrupt is raised when        */
/*                  1) a successful transmission occurs, 2) the retry limit  */
/*                  is reached or 3) packet lifetime expires. The status is  */
/*                  available in the Transmit Descriptor of the packet. When */
/*                  this interrupt is raised, a register is populated with   */
/*                  the address of the Transmit Descriptor.                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function reads the transmit descriptor from the MAC */
/*                  hardware register. It creates a MISC_TX_COMP event with  */
/*                  the message set to the pointer to the descriptor. This   */
/*                  event is posted to the event queue.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
void tx_complete_isr(void)
{
	UWORD8 slot;

	TROUT_FUNC_ENTER;

#ifdef TROUT_TRACE_DBG
	tx_isr_count += 1;
#endif

#ifdef TROUT_WIFI_NPI
     //printk("[libing] tx_complete_isr\n");
     if(g_tx_flag ){
#if 1
        if((transfer_q_num == HIGH_PRI_Q) && (copy_q_num == NORMAL_PRI_Q))
        {
            if(host_read_trout_reg(g_qif_table[HIGH_PRI_Q].addr) != 0)
            {
                //printk("Normal priority queue, Trout tx buffer is busy!\n");
                return;
            }
            else
            {
                //printk("+");
                host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)copy_ttr[NORMAL_PRI_Q].begin)), npi_addr[NORMAL_PRI_Q]);
                transfer_q_num = NORMAL_PRI_Q;
                qmu_cpy_npi_descr(HIGH_PRI_Q);
            }
        }
        else if((transfer_q_num == NORMAL_PRI_Q) && (copy_q_num == HIGH_PRI_Q))
        {
            if(host_read_trout_reg(g_qif_table[NORMAL_PRI_Q].addr) != 0)
            {
                //printk("Normal priority queue, Trout tx buffer is busy!\n");
                return;
            }
            else
            {
                //printk("=");
                host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)copy_ttr[HIGH_PRI_Q].begin)), npi_addr[HIGH_PRI_Q]);
                transfer_q_num = HIGH_PRI_Q;
                qmu_cpy_npi_descr(NORMAL_PRI_Q);
            }
        }
#else
	 tx_num += 1;
        if(tx_num<2) return ;
  	tx_num = 0;
#endif
        //printk("+ ");
        //host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)copy_ttr[q_num].begin)), npi_addr[q_num]);
        //qmu_cpy_npi_descr();
     		return ;
     }
#endif

	TX_PATH_DBG("%s: process\n", __func__);
    for(slot=0; slot<tx_shareram_slot_nr(); slot++)    //get_no_tx_queues() = 5;
	{
        if(!tx_shareram_slot_busy(slot))
        {
            continue;
        }
        
        mutex_lock(&g_q_handle.tx_handle.txq_lock);
        
        tx_pkt_process_new(slot, TX_ISR_CALL);
      
        mutex_unlock(&g_q_handle.tx_handle.txq_lock);
    }
	/* make sure no race condiction to start HW TX and add sync  by zhao */
    tx_shareram_wq_mount();
    
    if(tx_shareram_slot_valid()){
	    if(likely(reset_mac_trylock())){
		    trout_load_qmu();   //reload descr to trout.
		    reset_mac_unlock();
	    }
    }

	
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.misctxcompevent++;
#endif /* MEASURE_PROCESSING_DELAY */

	TROUT_FUNC_EXIT;
}

#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : rx_complete_isr                                          */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the receive    */
/*                  complete interrupt. This interrupt is raised when the    */
/*                  reception of a frame is complete and it has been saved   */
/*                  in memory with the Receive Descriptor. Reception of some */
/*                  frame types will not generate interrupt. This depends on */
/*                  the Receive frame filter register setting. This          */
/*                  interrupt is also raised when memory is not available    */
/*                  to store the received packet.                            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function creates and adds a new receive buffer to   */
/*                  the linked list of the receive packet descriptors. It    */
/*                  creates a WLAN_RX event with no message. This event is   */
/*                  posted to the event queue.                               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#ifndef TROUT_WIFI
void rx_complete_isr(UWORD8 q_num)
{
    UWORD8              num_dscr   = 0;
    UWORD8              *base_dscr = 0;
    wlan_rx_event_msg_t *wlan_rx   = 0;

    /* Read the  base descriptor pointer and the number of buffers */
    base_dscr = (UWORD8 *)get_machw_rx_frame_pointer(q_num);
    num_dscr  = get_machw_num_rx_frames(q_num);

    /* Allocate buffer for the WLAN receive event */
    wlan_rx = (wlan_rx_event_msg_t*)event_mem_alloc(WLAN_RX_EVENT_QID);

    /* Exception case 1: Event buffer allocation fails */
    if(wlan_rx == NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.erxcexc++;
#endif /* DEBUG_MODE */

        g_rxq_sync_loss[q_num] = 1;

        /* Do nothing and return */
        return;
    }

    /* Update the MIB statistics for FCS failed and Duplicate frames */
    mset_FCSErrorCount(get_fcs_count());
    mset_FrameDuplicateCount(get_dup_count());


    /* Update the WLAN Receive event buffer with the required information */
    wlan_rx->base_dscr = base_dscr;
    wlan_rx->num_dscr  = num_dscr;
    wlan_rx->rxq_num   = q_num;

    /* Set the synchronization required flag in the event if some sync loss  */
    /* has been detected and reset the sync loss detection global.           */
    if(g_rxq_sync_loss[q_num] == 1)
    {
        wlan_rx->sync_req      = 1;
        g_rxq_sync_loss[q_num] = 0;
    }
    else
    {
        wlan_rx->sync_req = 0;
    }

    /* Post the event */
    post_event((UWORD8 *)wlan_rx, WLAN_RX_EVENT_QID);

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.wlanrxevent++;
#endif /* MEASURE_PROCESSING_DELAY */
}

#endif	/* TROUT_WIFI */

/*****************************************************************************/
/*                                                                           */
/*  Function Name : tbtt_isr                                                 */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the TBTT       */
/*                  interrupt. This interrupt is raised at every TBTT.       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function creates a MISC_TBTT event with no message. */
/*                  This event is posted to the event queue.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
extern UWORD32 hw_txq_busy;

#ifdef IBSS_BSS_STATION_MODE
extern UWORD32 g_tbtt_cnt;
#endif

void tbtt_isr(void)
{
    misc_event_msg_t *misc = 0;
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	unsigned char *pp = NULL;
#endif
#endif


#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
	if(get_mac_state() == ENABLED){
		if(wake_lock_active(&deauth_err_lock)){
			pp = get_local_ipadd();
			if(pp){
				wake_unlock(&deauth_err_lock);
				printk("wake lock unlock---zz\n");
			}
		}
	}
#endif
#endif
// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef BSS_ACCESS_POINT_MODE
	/* No Beacon update will be allowed after TBTT */
	g_update_active_bcn = 0;
	
	if(get_mac_state() == ENABLED)	//add by chengwg.
	{
		raise_beacon_th();
		return;
	}
#endif /* BSS_ACCESS_POINT_MODE */
#ifdef IBSS_BSS_STATION_MODE
	// as a timer for power control 
	g_tbtt_cnt++;
// for coex state switch. wzl
    if(NULL != get_local_ipadd()){
        coex_state_switch(COEX_WIFI_CONNECTED);
    }

// wzl
#endif

    /* Create a MISC_TBTT event with no message and post it to the event     */
    /* queue.                                                                */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
    if(misc == NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.emiscexc++;
#endif /* DEBUG_MODE */
		TROUT_FUNC_EXIT;
        return;
    }

    misc->data = 0;
    misc->name = MISC_TBTT;
    post_event((UWORD8*)misc, MISC_EVENT_QID);
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : error_isr                                                */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the ERROR      */
/*                  interrupt. This interrupt is raised at every ERROR.      */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function creates a MISC_ERROR event with no message.*/
/*                  This event is posted to the event queue.                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

#if 0
void error_isr(void)
{
    UWORD32 err_stat = 0;
    UWORD8  report_error = 1;
    UWORD8  error_code   = 0;

#ifdef OLD_MACHW_ERROR_INT
    error_code   = get_machw_error_code();
    err_stat     = (1 << error_code);
#else /* OLD_MACHW_ERROR_INT */
    //chenq mod
    //err_stat = convert_to_le(rMAC_ERROR_STAT);
    err_stat = convert_to_le(host_read_trout_reg((UWORD32)rMAC_ERROR_STAT));
    printk("=========%s: err_stat=%08x===========\n", __FUNCTION__, err_stat);
#endif /* OLD_MACHW_ERROR_INT */

    if(err_stat & (1 << UNEXPECTED_RX_Q_EMPTY))
	{
        error_code   = UNEXPECTED_RX_Q_EMPTY;
        report_error = 0;

#ifndef OLD_MACHW_ERROR_INT
        /* Mask EC-12 to avoid this being raised repeatedly */
        mask_machw_error_int_code(UNEXPECTED_RX_Q_EMPTY);
#endif /* OLD_MACHW_ERROR_INT */

        /* Suspend MAC HW before disabling it */
        set_machw_tx_suspend();

        /* Disable MACHW & PHY */
        disable_machw_phy_and_pa();

#ifndef MAC_HW_UNIT_TEST_MODE
        /* Add more buffers to the RX-Q if possible */
//        replenish_rx_queue(&g_q_handle, NORMAL_PRI_RXQ);
#else /* MAC_HW_UNIT_TEST_MODE */
        /* Add more buffers to the RX-Q if possible */
        replenish_test_rx_queue(NORMAL_PRI_RXQ);
#endif /* MAC_HW_UNIT_TEST_MODE */

        /* Enabled MAC HW & PHY */
        enable_machw_phy_and_pa();

        /* Resume MAC HW after enabling it */
        set_machw_tx_resume();

#ifdef DEBUG_MODE
        g_mac_stats.erxqemp++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << UNEXPECTED_HIRX_Q_EMPTY))
    {
        error_code   = UNEXPECTED_HIRX_Q_EMPTY;
        report_error = 0;

#ifndef OLD_MACHW_ERROR_INT
        /* Mask EC-16 to avoid this being raised repeatedly */
        mask_machw_error_int_code(UNEXPECTED_HIRX_Q_EMPTY);
#endif /* OLD_MACHW_ERROR_INT */

        set_machw_tx_suspend();

        /* Disable MACHW & PHY */
        disable_machw_phy_and_pa();

#ifndef MAC_HW_UNIT_TEST_MODE
        /* Add more buffers to the RX-Q if possible */
//        replenish_rx_queue(&g_q_handle, HIGH_PRI_RXQ);
#else /* MAC_HW_UNIT_TEST_MODE */
        /* Add more buffers to the RX-Q if possible */
        replenish_test_rx_queue(HIGH_PRI_RXQ);
#endif /* MAC_HW_UNIT_TEST_MODE */

        /* Enabled MAC HW & PHY */
        enable_machw_phy_and_pa();

        /* Resume MAC HW after enabling it */
        set_machw_tx_resume();

#ifdef DEBUG_MODE
        g_mac_stats.ehprxqemp++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << TX_INTR_FIFO_OVERRUN))
    {
        error_code   = TX_INTR_FIFO_OVERRUN;
        report_error = 0;

#ifdef DEBUG_MODE
        g_mac_stats.etxfifo++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << RX_INTR_FIFO_OVERRUN))
    {
        error_code   = RX_INTR_FIFO_OVERRUN;
        report_error = 0;

        g_rxq_sync_loss[NORMAL_PRI_RXQ] = 1;

#ifdef DEBUG_MODE
        g_mac_stats.erxfifo++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << RX_PATH_WDT_TO_FCS_FAILED))
    {
        error_code   = RX_PATH_WDT_TO_FCS_FAILED;
        report_error = 0;

#ifdef DEBUG_MODE
        g_mac_stats.ewdtofcsfailed++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << RX_PATH_WDT_TO_FCS_PASSED))
    {
        error_code   = RX_PATH_WDT_TO_FCS_PASSED;
        report_error = 0;

#ifdef DEBUG_MODE
        g_mac_stats.ewdtofcspassed++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << HIRX_INTR_FIFO_OVERRUN))
    {
        error_code   = HIRX_INTR_FIFO_OVERRUN;
        report_error = 0;

        g_rxq_sync_loss[HIGH_PRI_RXQ]   = 1;

#ifdef DEBUG_MODE
        g_mac_stats.ehprxfifo++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << UNEXPECTED_MAC_TX_HANG))
    {
        error_code   = UNEXPECTED_MAC_TX_HANG;

        handle_machw_utxstat1_erri();

#ifdef DEBUG_MODE
        g_mac_stats.etxsus1machang++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << UNEXPECTED_PHY_TX_HANG))
    {
        error_code   = UNEXPECTED_PHY_TX_HANG;

        handle_machw_utxstat1_erri();

#ifdef DEBUG_MODE
        g_mac_stats.etxsus1phyhang++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << UNEXPECTED_MSDU_ADDR))
    {
        error_code   = UNEXPECTED_MSDU_ADDR;

#ifdef DEBUG_MODE
        g_mac_stats.emsaddr++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << UNEXPECTED_TX_Q_EMPTY))
    {
        error_code   = UNEXPECTED_TX_Q_EMPTY;

        /* Suspend MAC HW before disabling it */
        set_machw_tx_suspend();

        /* Disable MACHW & PHY */
        disable_machw_phy_and_pa();

        /* Enabled MAC HW & PHY */
        enable_machw_phy_and_pa();

        /* Resume MAC HW after enabling it */
        set_machw_tx_resume();

#ifdef DEBUG_MODE
        g_mac_stats.etxqempt++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << UNEXPECTED_TX_STATUS3))
    {
        error_code   = UNEXPECTED_TX_STATUS3;

#ifdef DEBUG_MODE
        g_mac_stats.etxsus3++;
#endif /* DEBUG_MODE */
    }
    else if(err_stat & (1 << BUS_ERROR))
    {
        error_code   = BUS_ERROR;

#ifdef DEBUG_MODE
            g_mac_stats.ebus++;
#endif /* DEBUG_MODE */
        }
    else if(err_stat & (1 << BUS_WRAP_SIG_ERROR))
        {
        error_code   = BUS_WRAP_SIG_ERROR;
#ifdef DEBUG_MODE
            g_mac_stats.ebwrsig++;
#endif /* DEBUG_MODE */
        }
    else if(err_stat & (1 << DMA_NO_ERROR))
        {
        error_code   = DMA_NO_ERROR;
#ifdef DEBUG_MODE
            g_mac_stats.edmanoerr++;
#endif /* DEBUG_MODE */
        }
    else if(err_stat & (1 << TX_CE_NOT_READY))
        {
        error_code   = TX_CE_NOT_READY;
#ifdef DEBUG_MODE
            g_mac_stats.etxcenr++;
#endif /* DEBUG_MODE */
        }
    else if(err_stat & (1 << RX_CE_NOT_READY))
        {
        error_code   = RX_CE_NOT_READY;
#ifdef DEBUG_MODE
            g_mac_stats.erxcenr++;
#endif /* DEBUG_MODE */
        }
    else if(err_stat & (1 << SEQNUM_GEN_ADDR_FAIL))
        {
        error_code   = SEQNUM_GEN_ADDR_FAIL;
#ifdef DEBUG_MODE
            g_mac_stats.esgaf++;
#endif /* DEBUG_MODE */
        }
    else if(err_stat & (1 << UNEXPECTED_NON_PENDING_MSDU))
    {
#if 0 /* Disabled for testing new HW F_00, No System reset is done */
#if 0 /* Temprorily disabled */
        UWORD8  i     = 0;
        UWORD32 dscr  = 0;
        UWORD16 num_q = get_no_tx_queues();
        UWORD8 status = 0;
#endif /* 0 */

        error_code   = UNEXPECTED_NON_PENDING_MSDU;

#ifndef OLD_MACHW_ERROR_INT
        /* Mask EC-10 to avoid this being raised repeatedly */
        mask_machw_error_int_code(UNEXPECTED_NON_PENDING_MSDU);
#endif /* OLD_MACHW_ERROR_INT */

#if 0 /* Temprorily disabled */
        for(i = 0; i < num_q; i++)
        {
            dscr = get_machw_q_ptr(i);

            /*If NULL dscr, go to next Queue*/
            if(dscr == 0)
                continue;

            status = get_tx_dscr_status((UWORD32*)dscr);

            if(status != PENDING)
            {
                /* Reset the transmit queue cache header  */
                set_machw_q_ptr((UWORD8)i, NULL);

                /* Delete all the existing packets in this Q */
                dscr = (UWORD32)(g_q_handle.tx_handle.tx_header[i].element_tail);
                if((UWORD8*)dscr != NULL)
                {
                    misc_event_msg_t *misc    = 0;
                    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

                    if(misc != NULL)
                    {
                        misc->data = (UWORD8 *)dscr;
                        misc->name = MISC_TX_COMP;
                        post_event((UWORD8*)misc, MISC_EVENT_QID);
                    }
                }
            }
        }
#endif /* 0 */
    /* Do not allow any system resets on EC-10 */
    report_error = 0;
    error_code   = UNEXPECTED_NON_PENDING_MSDU;
#else /* 0 */

#endif /*  0 */
#ifdef DEBUG_MODE
            g_mac_stats.enpmsdu++;
#endif /* DEBUG_MODE */
       }
    else
    {
#ifndef OLD_MACHW_ERROR_INT
         /* Reset UnAcked Error Interrupt */
        //chenq mod
        //rMAC_ERROR_STAT = rMAC_ERROR_STAT;
        host_write_trout_reg(
            host_read_trout_reg((UWORD32)rMAC_ERROR_STAT ),
            (UWORD32)rMAC_ERROR_STAT);
#endif /* OLD_MACHW_ERROR_INT */

        /* Handle Undefined/Unhandled Interrupts */
        report_error = 1;

#ifdef DEBUG_MODE
        PRINTD("HwEr: UKEI: err_stat = 0x%x\n\r",err_stat);
#endif /* DEBUG_MODE */
    }

	/* add by chengwg, need verify!!! */
	host_write_trout_reg(host_read_trout_reg((UWORD32)rMAC_ERROR_STAT ), (UWORD32)rMAC_ERROR_STAT);
	
#ifndef OLD_MACHW_ERROR_INT
    /* Acknowledge the Error Interrupt */
    reset_machw_error_int_code(error_code);
#endif /* OLD_MACHW_ERROR_INT */

#ifdef HANDLE_ERROR_INTR

    if(report_error == 1)
        raise_system_error(error_code);

#endif /* HANDLE_ERROR_INTR */

#ifdef DEBUG_MODE
    PRINTD2("HwEr:EC = %d\n\r",error_code);
#endif /* DEBUG_MODE */
}

#else

void error_isr(void)
{
    UWORD32 err_stat = 0;
    UWORD8  error_code   = 0;

	TROUT_FUNC_ENTER;
#ifdef TROUT_TRACE_DBG
	err_isr_count++;
#endif
	
#ifdef OLD_MACHW_ERROR_INT
    error_code   = get_machw_error_code();
    err_stat     = (1 << error_code);
#else
    err_stat = convert_to_le(host_read_trout_reg((UWORD32)rMAC_ERROR_STAT));
  	printk("%s: err_stat = %08x\n", __FUNCTION__, err_stat); //dumy mark for test 0731
#ifdef TROUT_TRACE_DBG
	err_isr_reg_count++;
#endif

#endif
    if(err_stat & (1 << UNEXPECTED_RX_Q_EMPTY))
	{
#ifndef OLD_MACHW_ERROR_INT
        /* Mask EC-12 to avoid this being raised repeatedly */
        mask_machw_error_int_code(UNEXPECTED_RX_Q_EMPTY);
#ifdef TROUT_TRACE_DBG
		err_isr_reg_count++;
#endif        
#endif /* OLD_MACHW_ERROR_INT */

        /* Suspend MAC HW before disabling it */
        set_machw_tx_suspend();

        /* Disable MACHW & PHY */
        disable_machw_phy_and_pa();

#ifndef MAC_HW_UNIT_TEST_MODE
        /* Add more buffers to the RX-Q if possible */
        //replenish_rx_queue(&g_q_handle, NORMAL_PRI_RXQ);
        TROUT_DBG5("normal pri rxq empty, replenish it...\n");
        rx_complete_isr(NORMAL_PRI_RXQ);	//test...
#else /* MAC_HW_UNIT_TEST_MODE */
        /* Add more buffers to the RX-Q if possible */
        replenish_test_rx_queue(NORMAL_PRI_RXQ);
#endif /* MAC_HW_UNIT_TEST_MODE */

        /* Enabled MAC HW & PHY */
        enable_machw_phy_and_pa();

        /* Resume MAC HW after enabling it */
        set_machw_tx_resume();

#ifdef DEBUG_MODE
        g_mac_stats.erxqemp++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << UNEXPECTED_HIRX_Q_EMPTY))
    {
#ifndef OLD_MACHW_ERROR_INT
        /* Mask EC-16 to avoid this being raised repeatedly */
        mask_machw_error_int_code(UNEXPECTED_HIRX_Q_EMPTY);
#endif /* OLD_MACHW_ERROR_INT */

        set_machw_tx_suspend();

        /* Disable MACHW & PHY */
        disable_machw_phy_and_pa();

#ifndef MAC_HW_UNIT_TEST_MODE
        /* Add more buffers to the RX-Q if possible */
        //replenish_rx_queue(&g_q_handle, HIGH_PRI_RXQ);
        TROUT_DBG5("high pri rxq empty, replenish it...\n");
        rx_complete_isr(HIGH_PRI_RXQ);	//test...
#else /* MAC_HW_UNIT_TEST_MODE */
        /* Add more buffers to the RX-Q if possible */
        replenish_test_rx_queue(HIGH_PRI_RXQ);
#endif /* MAC_HW_UNIT_TEST_MODE */

        /* Enabled MAC HW & PHY */
        enable_machw_phy_and_pa();

        /* Resume MAC HW after enabling it */
        set_machw_tx_resume();

#ifdef DEBUG_MODE
        g_mac_stats.ehprxqemp++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << TX_INTR_FIFO_OVERRUN))
    {
#ifdef DEBUG_MODE
        g_mac_stats.etxfifo++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << RX_INTR_FIFO_OVERRUN))
    {
        g_rxq_sync_loss[NORMAL_PRI_RXQ] = 1;

#ifdef DEBUG_MODE
        g_mac_stats.erxfifo++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << RX_PATH_WDT_TO_FCS_FAILED))
    {
    	mask_machw_error_int_code(RX_PATH_WDT_TO_FCS_FAILED);	//debug...
#ifdef DEBUG_MODE
        g_mac_stats.ewdtofcsfailed++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << RX_PATH_WDT_TO_FCS_PASSED))
    {
    	mask_machw_error_int_code(RX_PATH_WDT_TO_FCS_PASSED);	//debug...
#ifdef DEBUG_MODE
        g_mac_stats.ewdtofcspassed++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << HIRX_INTR_FIFO_OVERRUN))
    {
        g_rxq_sync_loss[HIGH_PRI_RXQ]   = 1;

#ifdef DEBUG_MODE
        g_mac_stats.ehprxfifo++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << UNEXPECTED_MAC_TX_HANG))
    {
	dump_allregs(1, 0);
        handle_machw_utxstat1_erri();
	dump_allregs(1, 1);
    printk("UNEX MAC_TX HANG\n");
	//BUG_ON(1);

#ifdef DEBUG_MODE
        g_mac_stats.etxsus1machang++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << UNEXPECTED_PHY_TX_HANG))
    {
	dump_allregs(19, 0);
        handle_machw_utxstat1_erri();
	dump_allregs(19, 1);
     printk("UNEX PHY_TX HANG\n");
	//BUG_ON(1);

#ifdef DEBUG_MODE
        g_mac_stats.etxsus1phyhang++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << UNEXPECTED_MSDU_ADDR))
    {
    //yangke, 2013-10-16, mask unexpected msdu addr address in NPI mode
#ifndef TROUT_WIFI_NPI
	dump_allregs(11, 0);
#ifdef DEBUG_MODE
        g_mac_stats.emsaddr++;
#endif /* DEBUG_MODE */
	dump_allregs(11, 1);
#endif
     printk("UNEX MSDU ADDR\n");
	//BUG_ON(1);
    }

    if(err_stat & (1 << UNEXPECTED_TX_Q_EMPTY))
    {
	dump_allregs(2, 0);
        /* Suspend MAC HW before disabling it */
        set_machw_tx_suspend();

        /* Disable MACHW & PHY */
        disable_machw_phy_and_pa();

        /* Enabled MAC HW & PHY */
        enable_machw_phy_and_pa();

        /* Resume MAC HW after enabling it */
        set_machw_tx_resume();

#ifdef DEBUG_MODE
        g_mac_stats.etxqempt++;
#endif /* DEBUG_MODE */
	dump_allregs(2, 1);
	printk("UNEX TX Q empty\n");
	//BUG_ON(1);
    }

    if(err_stat & (1 << UNEXPECTED_TX_STATUS3))
    {
	dump_allregs(3, 0);
#ifdef DEBUG_MODE
        g_mac_stats.etxsus3++;
#endif /* DEBUG_MODE */
	dump_allregs(3, 1);
	printk("UNEX TX status3\n");
	//BUG_ON(1);
    }

    if(err_stat & (1 << BUS_ERROR))
    {
	dump_allregs(4, 0);
#ifdef DEBUG_MODE
    	g_mac_stats.ebus++;
#endif /* DEBUG_MODE */
	dump_allregs(4, 1);
	printk("UNEX BUS_ERR\n");
	//BUG_ON(1);
    }

    if(err_stat & (1 << BUS_WRAP_SIG_ERROR))
    {
	dump_allregs(5, 0);
#ifdef DEBUG_MODE
        g_mac_stats.ebwrsig++;
#endif /* DEBUG_MODE */
	dump_allregs(5, 1);
	printk("UNEX BUS_WARP_SIG_ERROR\n");
	//BUG_ON(1);
    }

    if(err_stat & (1 << DMA_NO_ERROR))
    {
#ifdef DEBUG_MODE
        g_mac_stats.edmanoerr++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << TX_CE_NOT_READY))
    {
#ifdef DEBUG_MODE
        g_mac_stats.etxcenr++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << RX_CE_NOT_READY))
    {
#ifdef DEBUG_MODE
        g_mac_stats.erxcenr++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << SEQNUM_GEN_ADDR_FAIL))
    {
#ifdef DEBUG_MODE
        g_mac_stats.esgaf++;
#endif /* DEBUG_MODE */
    }

    if(err_stat & (1 << UNEXPECTED_NON_PENDING_MSDU))
    {
	dump_allregs(10, 0);
    	mask_machw_error_int_code(UNEXPECTED_NON_PENDING_MSDU);	//debug...
#ifdef DEBUG_MODE
       g_mac_stats.enpmsdu++;
#endif /* DEBUG_MODE */
	dump_allregs(10, 1);
	printk("UNEX non pending MSDU\n");
	//BUG_ON(1);
    }

	/* add by chengwg, need verify!!! */
//	PRINTD("HwEr: UKEI: err_stat = 0x%x\n\r",err_stat); //dumy mark for test 0731
	host_write_trout_reg(err_stat, (UWORD32)rMAC_ERROR_STAT);
#ifdef TROUT_TRACE_DBG
	err_isr_reg_count++;
	
#endif	
#ifndef OLD_MACHW_ERROR_INT
    /* Acknowledge the Error Interrupt */
    reset_machw_error_int_code(error_code);
#endif /* OLD_MACHW_ERROR_INT */

#ifdef DEBUG_MODE
//    PRINTD2("HwEr:EC = %d\n\r", error_code);//dumy mark for test 0731
#endif /* DEBUG_MODE */
	TROUT_FUNC_EXIT;
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : deauth_isr                                               */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the Deauth     */
/*                  interrupt.                                               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function raises a system error in BSS STA mode.     */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void deauth_isr(void)
{
	TROUT_FUNC_ENTER;
#ifdef IBSS_BSS_STATION_MODE
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        raise_system_error(DEAUTH_RCD);
    }
#endif /* IBSS_BSS_STATION_MODE */

    TROUT_DBG4("%s: RC=%x\n\r", __FUNCTION__, get_machw_deauth_reason_code());
	TROUT_FUNC_EXIT;
}
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef BURST_TX_MODE

void update_burst_mode_tx_dscr(UWORD8  *tx_dscr)
{
    UWORD8  status = 0;
    UWORD32 tsf_hi = 0;
    UWORD32 tsf_lo = 0;

    /* Get the TSF timer value */
    get_machw_tsf_timer(&tsf_hi, &tsf_lo);

    status = ((PENDING << 5) & 0xE0);
    status |= (0x02 & 0x0F); /* Set priority as high priority */
    set_tx_dscr_status((UWORD32 *)tx_dscr, status);

    /* Other than the first fragment, all other fragment's status should be  */
    /* set to 'not pending'.                                                 */
    (*((UWORD32 *)tx_dscr + 3)) = convert_to_le(0xFFFFFFFC);

    /* Set the current timestamp */
    set_tx_dscr_tsf_ts((UWORD32 *)tx_dscr, (UWORD16)((tsf_lo << 10) & 0xFFFF));
}
#endif /* BURST_TX_MODE */



/* duplicate write_phy function, but not hold mutex by zhao */
void internal_change_bank(UWORD8 ra, UWORD32 rd)
{
	    UWORD32 val   = 0;
	    UWORD32 delay = 0;

	    while((host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON) 
			& REGBIT0) == REGBIT0){
		add_calib_delay(1);
		delay++;
	       /* Wait for sometime for the CE-LUT update operation to complete */
		if(delay > PHY_REG_RW_TIMEOUT){
		    raise_system_error(PHY_HW_ERROR);
		    break;
		}
	    }

	    host_write_trout_reg( convert_to_le(rd), 
				  (UWORD32)rMAC_PHY_REG_RW_DATA );
 	    val  = ((UWORD8)ra << 2); /* Register address set in bits 2 - 9 */
	    val &= ~ BIT1;            /* Read/Write bit set to 0 for write */
	    val |= BIT0;              /* Access bit set to 1 */

	    host_write_trout_reg( convert_to_le(val), 
				  (UWORD32)rMAC_PHY_REG_ACCESS_CON );
}

//extern void set_default_tx_power_levels_ittiam(void);
/*****************************************************************************/
/*                                                                           */
/*  Function Name : write_dot11_phy_reg                                      */
/*                                                                           */
/*  Description   : This function writes to the Ittiam PHY register.         */
/*                                                                           */
/*  Inputs        : 1) Address of register                                   */
/*                  2) Value to be written                                   */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the lock bit and then writes the    */
/*                  PHY register by writing the address and value to the     */
/*                  serial control register.                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void write_dot11_phy_reg(UWORD8 ra, UWORD32 rd)
{
	    UWORD32 val   = 0;
	    UWORD32 delay = 0;
		UWORD32	v, swich = 0;
		unsigned long caller = __builtin_return_address(0);

	get_phy_mutex();
		/* handle the race condiction between initialize_phy and common
		 * write_dot11_phy_reg, and dest add of all the write operation is bank1
		 * so, if the current is bank2, so change it. by zhao 
		 */
		if(caller < initialize_phy_ittiam 
			&& caller >= set_default_tx_power_levels_ittiam){
			
			read_dot11_phy_reg(0xFF, &v);
			if(v){
				printk("@@@ WARN ct bank2, want write bank1 val%x\n", rd);
				/* it the current bank is not bank0, switch to bank0 by zhao*/
				internal_change_bank(0xff,  0x00);
				swich = 1;
			}
		}
	    //chenq mod
	    //while((rMAC_PHY_REG_ACCESS_CON & REGBIT0) == REGBIT0)
	    while((host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON) 
			& REGBIT0) == REGBIT0){
		add_calib_delay(1);
		delay++;

       /* Wait for sometime for the CE-LUT update operation to complete */
		if(delay > PHY_REG_RW_TIMEOUT){
#ifdef DEBUG_MODE
            PRINTD("HwEr:PhyRegRwFail write_dot11_phy_reg\n\r");
            g_mac_stats.phyregrwfail++;
#endif /* DEBUG_MODE */
            raise_system_error(PHY_HW_ERROR);
            break;
        }
    }

    /* Write the data to register */
    //chenq mod
    //rMAC_PHY_REG_RW_DATA = convert_to_le(rd);
    host_write_trout_reg( convert_to_le(rd), 
                          (UWORD32)rMAC_PHY_REG_RW_DATA );

    val  = ((UWORD8)ra << 2); /* Register address set in bits 2 - 9 */
    val &= ~ BIT1;            /* Read/Write bit set to 0 for write */
    val |= BIT0;              /* Access bit set to 1 */

    //chenq mod
    //rMAC_PHY_REG_ACCESS_CON = convert_to_le(val);
    host_write_trout_reg( convert_to_le(val), 
                          (UWORD32)rMAC_PHY_REG_ACCESS_CON );
		/* if we switched the bank, return to previous bank by zhao */
		if(swich)
			internal_change_bank(0xff,  v);
	put_phy_mutex();
}

EXPORT_SYMBOL(write_dot11_phy_reg);

/*zhq add for powersave*/
void root_write_dot11_phy_reg(UWORD8 ra, UWORD32 rd)
{
    UWORD32 val   = 0;
    UWORD32 delay = 0;

    //chenq mod
    //while((rMAC_PHY_REG_ACCESS_CON & REGBIT0) == REGBIT0)
    while((root_host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON) 
                & REGBIT0) == REGBIT0)
    {
        add_calib_delay(1);
        delay++;

       /* Wait for sometime for the CE-LUT update operation to complete */
        if(delay > PHY_REG_RW_TIMEOUT)
        {
#ifdef DEBUG_MODE
            PRINTD("HwEr:PhyRegRwFail write_dot11_phy_reg\n\r");
            g_mac_stats.phyregrwfail++;
#endif /* DEBUG_MODE */
            raise_system_error(PHY_HW_ERROR);
            break;
        }
    }

    /* Write the data to register */
    //chenq mod
    //rMAC_PHY_REG_RW_DATA = convert_to_le(rd);
    root_host_write_trout_reg( convert_to_le(rd), 
                          (UWORD32)rMAC_PHY_REG_RW_DATA );

    val  = ((UWORD8)ra << 2); /* Register address set in bits 2 - 9 */
    val &= ~ BIT1;            /* Read/Write bit set to 0 for write */
    val |= BIT0;              /* Access bit set to 1 */

    //chenq mod
    //rMAC_PHY_REG_ACCESS_CON = convert_to_le(val);
    root_host_write_trout_reg( convert_to_le(val), 
                          (UWORD32)rMAC_PHY_REG_ACCESS_CON );
}

EXPORT_SYMBOL(root_write_dot11_phy_reg);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : read_dot11_phy_reg                                       */
/*                                                                           */
/*  Description   : This function reads the Ittiam PHY register.             */
/*                                                                           */
/*  Inputs        : 1) Address of register                                   */
/*                  2) Pointer to value to be updated                        */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function checks the lock bit and then reads the     */
/*                  PHY register by writing the address to the serial        */
/*                  control register, polling the status bit and then        */
/*                  reading the register.                                    */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void read_dot11_phy_reg(UWORD8 ra, UWORD32 *rd)
{
    UWORD32 val = 0;
    UWORD32 delay = 0;

    //chenq mod
    //while((rMAC_PHY_REG_ACCESS_CON & REGBIT0) == REGBIT0)
    while((host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON)
                & REGBIT0) == REGBIT0)
    {
        add_calib_delay(1);
        delay++;

       /* Wait for sometime for the CE-LUT update operation to complete */
        if(delay > PHY_REG_RW_TIMEOUT)
        {
#ifdef DEBUG_MODE
            PRINTD("HwEr:PhyRegRwFail read_dot11_phy_reg\n\r");
            g_mac_stats.phyregrwfail++;
#endif /* DEBUG_MODE */

            raise_system_error(PHY_HW_ERROR);
            break;
        }
    }

    val  = ((UWORD8)ra << 2); /* Register address set in bits 2 - 9 */
    val |= BIT1;              /* Read/Write bit set to 1 for read */
    val |= BIT0;              /* Access bit set to 1 */

    //chenq mod
    //rMAC_PHY_REG_ACCESS_CON = convert_to_le(val);
    host_write_trout_reg( convert_to_le(val), 
                          (UWORD32)rMAC_PHY_REG_ACCESS_CON );

    //chenq mod
    //while((rMAC_PHY_REG_ACCESS_CON & REGBIT0) == REGBIT0)
    while((host_read_trout_reg( (UWORD32)rMAC_PHY_REG_ACCESS_CON)
                & REGBIT0) == REGBIT0)
    {
        add_calib_delay(1);
        delay++;

       /* Wait for sometime for the CE-LUT update operation to complete */
        if(delay > PHY_REG_RW_TIMEOUT)
        {
#ifdef DEBUG_MODE
            PRINTD("HwEr:PhyRegRwFail read_dot11_phy_reg 2\n\r");
            g_mac_stats.phyregrwfail++;
#endif /* DEBUG_MODE */

            raise_system_error(PHY_HW_ERROR);
            break;
        }
    }

    //chenq mod
    //*rd = convert_to_le(rMAC_PHY_REG_RW_DATA); /* Read data from register */
    *rd = convert_to_le(host_read_trout_reg( (UWORD32)rMAC_PHY_REG_RW_DATA)); /* Read data from register */
}

//EXPORT_SYMBOL(read_dot11_phy_reg);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : radar_detect_isr                                         */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the radar      */
/*                  detect interrupt.                                        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    :                                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void radar_detect_isr(void)
{
	TROUT_FUNC_ENTER;
    /* Reset PHY Radar detected  registers according to PHY interface */
    post_radar_detect_phy();

    /* Post an event for Radar detection */
    post_radar_detect_event();
    TROUT_FUNC_EXIT;
}

#ifdef TX_ABORT_FEATURE
/*****************************************************************************/
/*                                                                           */
/*  Function Name : tx_abort_start_isr                                       */
/*                                                                           */
/*  Description   : This is the interrupt service routine for the MAC H/w    */
/*                  interrupt indicating start of a H/w triggered TX-Abort.  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None.                                                    */
/*                                                                           */
/*  Processing    :                                                          */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void tx_abort_start_isr(void)
{
#ifdef ENABLE_MACHW_KLUDGE
	TROUT_FUNC_ENTER;
    disable_machw_phy_and_pa();
    add_calib_delay(2);
    reset_pacore();
    reset_cecore();
    enable_machw_phy_and_pa();
    TROUT_FUNC_EXIT;
#endif /* ENABLE_MACHW_KLUDGE */
}
#endif /* TX_ABORT_FEATURE */




