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
/*  File Name         : mac_init.c                                           */
/*                                                                           */
/*  Description       : This file contains all the functions to initialize   */
/*                      the variables and structures used in MAC.            */
/*                                                                           */
/*  List of Functions : main_function                                        */
/*                      initialize_macsw                                     */
/*                      create_mac_alarms                                    */
/*                      create_mac_interrupts                                */
/*                      delete_mac_interrupts                                */
/*                      reset_mac                                            */
/*                      enable_mac_interrupts                                */
/*                      disable_mac_interrupts                               */
/*                      restart_mac                                          */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "controller_mode_if.h"
#include "iconfig.h"
#include "core_mode_if.h"
#include "index_util.h"
#include "mac_init.h"
#include "qmu_if.h"
#include "phy_prot_if.h"
#include "csl_if.h"
#include "trout_wifi_rx.h"
#include "cglobals.h"
#include "qmu_tx.h"

#ifdef CONFIG_CFG80211
#include "trout_cfg80211.h"
#endif
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#include "ps_timer.h"
#endif


#include <linux/kthread.h>

#ifndef MAC_HW_UNIT_TEST_MODE
#ifdef DEFAULT_SME
#include "test_config.h"
#endif /* DEFAULT_SME */
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef BSS_ACCESS_POINT_MODE
struct task_struct *beacon_thread = NULL;
static unsigned int tbtt_rise = 0;
DECLARE_WAIT_QUEUE_HEAD(bfwq);
static struct 	mutex vbp_mutex;
static int	vbp_locked = 0;
static unsigned long	vbp_owner;
static struct task_struct	*vbp_tsk = NULL;
#endif

extern void mac_isr_work(struct work_struct *work);
extern BOOL_T is_all_machw_q_null(void);
extern void tx_complete_isr_simulate(void);


//chenq add 2012-11-01?
extern atomic_t g_event_cnt;

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

#ifdef BSS_ACCESS_POINT_MODE
/* since the beacon frame was prepared and modified in two threads, so need protection by zhao 6-21 2013 */
void get_vbp_mutex(unsigned long owner)
{
	if(vbp_locked && current == vbp_tsk){
		printk("@@@BUG!vbp mutex has locked by %pS\n", vbp_owner);
		dump_stack();
	}
	mutex_lock(&vbp_mutex);
	vbp_locked = 1;
	vbp_owner = owner;
	vbp_tsk = current;
}

void put_vbp_mutex(void)
{
	mutex_unlock(&vbp_mutex);
	vbp_locked = 0;
	vbp_owner = 0;
	vbp_tsk = NULL;
}
#endif
void delete_mac_interrupts(void);

/*****************************************************************************/
/*                                                                           */
/*  Function Name : main_function                                            */
/*                                                                           */
/*  Description   : This function is called first when the application       */
/*                  starts.                                                  */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac                                                    */
/*                                                                           */
/*  Processing    : This function initializes MAC S/w after initializing all */
/*                  processor related regitsers. It then starts the event    */
/*                  manager.                                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

int main_function(void)
{
	TROUT_FUNC_ENTER;
	//chenq add 2012-11-02
	//atomic_set( &g_mac_reset_done,(int)BFALSE );
	
	/* Print the version information to the serial console */
	print_banner();	//moved by chengwg.

    /* Check for consistency of build parameters */
    check_build_params();

    /* Initialize mode switch flag to BFALSE. This is set to BTRUE only if   */
    /* saved settings are found, indicating that the mode is being switched. */
    set_switch_in_progress(BFALSE);

    /* Do processor specific initializations */
    init_proc();

    /* Reset PHY and MAC hardware */
    reset_phy_machw();

    /* Initialize the CSL Memory */
    csl_mem_init();

    /* reset QMU handle */
    mem_set(&g_q_handle, 0, sizeof(qmu_handle_t));

#ifdef NO_ACTION_RESET
    /* Initialize the global saved action request list */
    init_save_action_req_list();
#endif /* NO_ACTION_RESET */

    /* Check if we have some saved WID's */
    get_wid_settings();

    /* Initialize MAC S/w (FSM, QMU, MM, Globals, MIB, event manager) */
    if(initialize_macsw(&g_mac) < 0)
    	return -1;

    /* Calibrate S/w delay loop */
    calibrate_delay_loop();

    /* Initialize the host interface */
    init_host_interface();

    /* After Host Interface is initialized   */
    /* Convey the current MAC status to Host */

	//chenq mask
    //send_mac_status(MAC_DISCONNECTED);

#ifndef OS_LINUX_CSL_TYPE
    /* For TEST SME mode call a function to add a dummy configuration HOST   */
    /* RX event to the event queue before starting MAC controller            */
#ifndef MAC_HW_UNIT_TEST_MODE

    if(is_dev_mode_host_managed() == BFALSE)
    {
        //config_op_params(&g_mac);
    }
	
	// 20120709 caisf add the "if", merged ittiam mac v1.2 code
    /* Initiate a Join with configuration saved in the device */
    /* Note : This check is added so that if P2P or Default SME has       */
    /* configured something then WPS should not overwrite the             */
    /* configuration.                                                     */
    if(DISABLED == get_mac_state())
    {
    	join_config_saved_in_device();
    }

#endif /* MAC_HW_UNIT_TEST_MODE */
    /* Start MAC controller task */
    start_mac_controller();
#endif /* OS_LINUX_CSL_TYPE */

	//chenq add 2012-11-02
	//atomic_set(&g_mac_reset_done,(int)BTRUE);

	TROUT_FUNC_EXIT;
	return 0;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : initialize_macsw                                         */
/*                                                                           */
/*  Description   : This function initializes the given MAC library          */
/*                  structure (state, FSM) and all required globals.         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                                                                           */
/*  Globals       : g_q_handle                                               */
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

int initialize_macsw(mac_struct_t *mac)
{
	TROUT_FUNC_ENTER;
	trout_share_ram_show();	//add by chengwg.
#ifdef WAKE_LOW_POWER_POLICY
	low_power_init();
#endif
#ifdef IBSS_BSS_STATION_MODE	
	wifi_bt_coexist_init();	//add by chengwg for wifi&bt coexist.
#endif	/* IBSS_BSS_STATION_MODE */

    /* Queue interface table is initialized by zhao*/
    init_qif_table();
	/* kmalloc first, then vmalloc by zhao*/
	if(init_mac_qmu() < 0)
		return -1;
	
	/* Initialize the memory manager and the MAC memory pools */
	if(init_mac_mem_pools() < 0){
		host_rx_queue_free(); //by zhao
#ifdef TX_PKT_USE_DMA
		tx_dma_buf_free();
#endif
		TROUT_DBG1("%s: mem pool init fail!\n", __func__);
		return -1;
    }

	/* Initialize the globals */
    init_mac_globals();	//moved by chengwg.

    /* Initialize the MAC library structure (FSM table) */
    init_mac_fsm(mac);

    /* Initialize PHY characteristics, and after that PHY mibs */
    init_phy_chars();

    /* Initialize the MIB parameters */
    init_mib();
    init_phy_mib();

    /* Initialize the supported channel list */
    init_supp_ch_list();

    /* Initialize supported Regulatory Domain tables */
    init_supp_reg_domain();

	/* move to head by zhao */
    /* Queue interface table is initialized */
    //init_qif_table();

    /* Initialize Queue Manager */
/*
	if(init_mac_qmu() < 0)	//modify by chengwg.
	{
		TROUT_DBG1("%s: init mac qmu failed!\n", __func__);
		free_mem_regions();
		return -1;
	}
*/
	tx_shareram_manage_init();

    /* Initialize the chip support library. Note that the memory manager     */
    /* needs to be initialized before this.                                  */
    csl_init();
#ifdef AUTORATE_FEATURE
    /* Initialize the autorate timer and initialize the auto rate table */
    create_autorate_timer();
    init_ar_table();
#endif /* AUTORATE_FEATURE */

    /* Initialize event manager */
    init_event_manager();

    /* Do protocol related system initialization */
    sys_init_prot();
    
    TROUT_FUNC_EXIT;
    return 0;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_mac_interrupts                                    */
/*                                                                           */
/*  Description   : This function creates the MAC interrupts and attaches    */
/*                  the various service routines.                            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_isr_handle                                         */
/*                  g_mac_int_vector                                         */
/*                                                                           */
/*  Processing    : Interrupts are created by calling the required chip      */
/*                  support library function.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void create_mac_interrupts(void)
{
    UWORD32 int_mask = 0;
    UWORD32 int_stat = 0;
    ISR_T id = {0};

    TROUT_FUNC_ENTER;
    id.isr = (ISR_FN_T *)mac_isr_work;
    id.dsr = NULL;    
	
	if(NULL == g_mac_isr_handle) //chenq add for re-create 0726
    {
    	g_mac_isr_handle = create_interrupt(g_mac_int_vector,  /* Vector         */
	                                        MAC_INTERRUPT_PRI, /* Priority       */
	                                        MAC_INTERRUPT_DAT, /* Interrupt data */
	                                        &id);          /* Service handler*/
	}

#ifndef ERROR_INT_ENABLE
	/* keep them same by zhao */
	//rCOMM_INT_MAS = 0x10008  umask tx,rx, tbtt, hi_rx
	host_write_trout_reg(0xFFFFEFE3, (UWORD32)rCOMM_INT_MASK);
	host_write_trout_reg(0xFFFFFBF8, (UWORD32)rMAC_INT_MASK);
#else
    /* enable error interrupt by zhao 6-25 2013 */
	host_write_trout_reg(0xFFFFEEE3, (UWORD32)rCOMM_INT_MASK);
	host_write_trout_reg(0xFFFFFBB8, (UWORD32)rMAC_INT_MASK);
	unmask_machw_error_int();
#endif
    /* Unmask the H/w interrupts once ISR is attached */
#ifndef TROUT_B2B_TEST_MODE
    unmask_machw_hprx_comp_int();
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
    if(mget_DesiredBSSType() == INFRASTRUCTURE)
    {
        unmask_machw_arm2host_int();
    }
#endif

    //int_mask = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_MASK)) >> 2;
    int_mask = host_read_trout_reg((UWORD32)rCOMM_INT_MASK);
    int_stat = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2;
	TROUT_DBG4("create isr, int mask: %#08x, int stat: %#08x\n", int_mask>>2, int_stat);
	
	update_trout_int_mask(int_mask);

/*
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
    // unmask TBTT ISR in wifi interrput,caisf add 0227 
    host_write_trout_reg(0xffffffff & (~BIT2), (UWORD32)rMAC_INT_MASK);
    TROUT_DBG4("wifi: wifi_int_mask = 0x%x; wifi_int_stat = 0x%x;\n", 
        convert_to_le(host_read_trout_reg((UWORD32)rMAC_INT_MASK)), 
        convert_to_le(host_read_trout_reg((UWORD32)rMAC_INT_STAT)));
#endif
*/
	{
		UWORD32 tmp32 = 0;		
		host_read_trout_ram((void *)&tmp32, (void *)(COEX_SELF_CTS_NULL_DATA_BEGIN-4), sizeof(UWORD32));
		printk("[%s] index from BT: %#x\n", __FUNCTION__, tmp32);
		//host_read_trout_ram((void *)&tmp32, (void *)(COEX_PS_NULL_DATA_BEGIN), sizeof(UWORD32));
		//printk("[%s] PS NULL data: %#x\n", __FUNCTION__, tmp32);
	}
	host_notify_arm7_coex_ready(BTRUE);
	host_notify_arm7_wifi_reset(BFALSE);
	{	
		UWORD32 tmp = 0;
		host_read_trout_ram(&tmp, (UWORD32*)TROUT_MEM_CFG_BEGIN, 4);
		printk("[%s] TROUT_MEM_CFG_BEGIN = %#x\n", __FUNCTION__, tmp);
	}

	TRACE_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : create_mac_alarms                                        */
/*                                                                           */
/*  Description   : This function creates all the alarms used by MAC.        */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mgmt_alarm_handle                                      */
/*                  g_diversity_timer                                        */
/*                                                                           */
/*  Processing    : Alarms are created by calling the required chip support  */
/*                  library function.                                        */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void create_mac_alarms(void)
{
    /* If the management alarm exists, stop and delete the same */
    if(g_mgmt_alarm_handle != 0)
    {
        stop_alarm(g_mgmt_alarm_handle);
        delete_alarm(&g_mgmt_alarm_handle);
    }
    g_mgmt_alarm_handle = create_alarm(mgmt_timeout_alarm_fn, 0, NULL);

}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : delete_mac_interrupts                                    */
/*                                                                           */
/*  Description   : This function deletes the MAC interrupts and attaches    */
/*                  the various service routines.                            */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_isr_handle                                         */
/*                                                                           */
/*  Processing    : Interrupts are deleted by calling the required chip      */
/*                  support library function.                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void delete_mac_interrupts(void)
{
    if(g_mac_isr_handle != NULL)
    {
        delete_interrupt(g_mac_isr_handle);
        g_mac_isr_handle = NULL;           //add by Hugh.
    }
}


#ifdef BSS_ACCESS_POINT_MODE	//add by zhaozhang.
void raise_beacon_th(void)
{
	tbtt_rise = 1;
	wake_up(&bfwq);
}

static int handle_beacon(void)
{
	UWORD32 beacon_len = 0;
	UWORD32 *beacon_buf = (UWORD32 *)BEACON_MEM_BEGIN;
	UWORD8  dtim_count  = 0;
	UWORD8  old_bcn_idx = 0;
	UWORD16 i           = 0;
	unsigned int v;
	int smart_type = MODE_START;
	struct timespec time;
	static unsigned int cnr = 0;    

	v = convert_to_le(host_read_trout_reg((UWORD32)rMAC_PA_STAT));
	while((v & 0x10) == 0 && !kthread_should_stop()){
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(10));
		// add by Ke.Li at 2012-02-11 for fix beacon frame Time count isn't be changed bug
        v = convert_to_le(host_read_trout_reg((UWORD32)rMAC_PA_STAT));
        // add by Ke.Li at 2012-02-11 end
	}
	/* by zhao, since we about to rmmod, so need do nothing here!\n */
	if(kthread_should_stop())
		return 0;
	if( reset_mac_trylock() == 0 ){
		return 0;
	}
	process_tbtt_ap_prot();
	old_bcn_idx  = (g_beacon_index + 1)%2;
	dtim_count = get_machw_dtim_count();

	/* If the beacon pointer has been updated to a new value, the free   */
	/* beacon buffer index is updated to the other buffer.               */
	if(dtim_count == 0)
	{
	    dtim_count = mget_DTIMPeriod() - 1;

	    /* The beacon transmitted at this TBTT is the DTIM. Requeue all  */
	    /* MC/BC packets to the high priority queue.                     */
	    /* Check if the beacon that is being transmitted has the MC bit  */
	    /* set                                                           */
	    if(BTRUE == get_mc_bit_bcn(old_bcn_idx))
		while(requeue_ps_packet(NULL, &g_mc_q, BTRUE, BFALSE) == PKT_REQUEUED);
	}
	else
	{
	    /* Do nothing */
	    dtim_count--;
	}

    /* virtual bit map should be protected by zhao 6-21 2013 */
	get_vbp_mutex(__builtin_return_address(0));
	g_vbmap[DTIM_CNT_OFFSET] = dtim_count;
	for(i = 0; i < g_vbmap[LENGTH_OFFSET] + 2; i++){
		g_beacon_frame[g_beacon_index][g_tim_element_index + i + SPI_SDIO_WRITE_RAM_CMD_WIDTH] =
						g_vbmap[i];
	}
	g_beacon_len = g_tim_element_index + g_tim_element_trailer_len +
				      g_vbmap[LENGTH_OFFSET] + 2 + FCS_LEN;

	/* The status of BC/MC packets queued for PS should be updated only  */
	/* in DTIM beacon dtim_count==0. For the rest of the becons reset    */
	/* the BC/MC bit in TIM                                              */
	if(dtim_count != 0)
        	reset_mc_bit_bcn(g_beacon_index);
	put_vbp_mutex();

	beacon_len = g_beacon_len;
	if(g_beacon_len % 4 != 0)
		beacon_len += 4 - (g_beacon_len % 4);
	if(beacon_len > BEACON_MEM_SIZE){
		printk("%s: beacon mem is too small(%d : %d)!\n", __func__, beacon_len, BEACON_MEM_SIZE);
		reset_mac_unlock();
		return -1;
	}
	host_write_trout_ram((void *)beacon_buf, g_beacon_frame[g_beacon_index] + SPI_SDIO_WRITE_RAM_CMD_WIDTH, beacon_len);
        /* Swap the beacon buffer used by MAC H/w */
	set_machw_beacon_tx_params(g_beacon_len, get_phy_rate(get_beacon_tx_rate()));
    set_machw_beacon_ptm(get_reg_phy_tx_mode(get_beacon_tx_rate(), 1));
    set_machw_beacon_pointer((UWORD32)beacon_buf);

	smart_type = critical_section_smart_start(0,1);
	if(dtim_count == 0)
		g_update_active_bcn = 1;
	critical_section_smart_end(smart_type);

	/* Copy the contents from the current beacon to the old beacon */
	for(i = 0; i < g_beacon_len; i++)
	g_beacon_frame[old_bcn_idx][i+SPI_SDIO_WRITE_RAM_CMD_WIDTH] = 
			g_beacon_frame[g_beacon_index][i+SPI_SDIO_WRITE_RAM_CMD_WIDTH];

	g_beacon_index = old_bcn_idx;
	handle_tbtt_chan_mgmt_ap(dtim_count);
	handle_tbtt_prot_ap();
	reset_mac_unlock();
	return 0;
}

static int beacon_th(void *dumy)
{
	while(1){
		if(kthread_should_stop())
			break;		
		if (wait_event_interruptible(bfwq, tbtt_rise == 1 || kthread_should_stop()))
			continue;
		handle_beacon();
		tbtt_rise = 0;
	}
	return 0;
}

int create_beacon_thread(void)
{
	int rc = 0;

	if(beacon_thread != NULL)
		return rc;
	
	mutex_init(&vbp_mutex);
	beacon_thread = kthread_run(beacon_th, NULL, "beacon thread");
	if(IS_ERR(beacon_thread))
	{
		rc = PTR_ERR(beacon_thread);
		TROUT_DBG4("create beacon thread error, errno=%d\n", rc);
		rc = -1;
	}

	return rc;
}

void delete_beacon_thread(void)
{
	if(beacon_thread)
	{
		kthread_stop(beacon_thread);
		beacon_thread = NULL;
	}
}
#endif	/* BSS_ACCESS_POINT_MODE */


/*****************************************************************************/
/*                                                                           */
/*  Function Name : reset_mac                                                */
/*                                                                           */
/*  Description   : This function resets all the modules in MAC and sets MAC */
/*                  state to DISABLED.                                       */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                                                                           */
/*  Globals       : g_mac_reset_done                                         */
/*                                                                           */
/*  Processing    : Call all the reset functions and at the end call MAC s/w */
/*                  initializing function.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
#if 0
void reset_mac(mac_struct_t *mac, BOOL_T init_mac_sw)
{
	TROUT_FUNC_ENTER;

	printk("%s: caller:", __func__);
	print_symbol("%s\n", (unsigned long)__builtin_return_address(0));
	
	//chenq add 2012-11-02
	atomic_set(&g_mac_reset_done, (int)BFALSE);
#ifdef DEBUG_MODE
    g_reset_stats.numresets++;
#endif /* DEBUG_MODE */

    /* Perform appropriate pre-reset tasks related to host */
	//chenq mask 2012-11-02
    //pre_reset_tasks_host(mac);

	netif_queue_stopped(g_mac_dev);		//chwg test-2013-01-11

    /* Perform appropriate pre-reset tasks related to wireless lan */
    pre_reset_tasks_wlan(mac);

	flush_all_rxq();	//add by chengwg.
	
	//chenq add drop all event 2012-11-02
	printk("chenq debug reset_mac 1\n");
	while(atomic_read(&g_event_cnt) > 0)
	{
		printk("chenq debug atomic_read(&g_event_cnt) = %d\n", atomic_read(&g_event_cnt));
		//chenq add drop all event 2012-11-02
		mac_event_schedule();
		msleep(1);
		//set_current_state(TASK_INTERRUPTIBLE);
		//schedule_timeout(jiffies+1);	//10 mecrosecond.
	}
	printk("chenq debug reset_mac 2\n");
	
    /* Memory manager, queue manager & host interface are going to be        */
    /* re-initialized/reset in this function. Interrupts coming during this  */
    /* phase may corrupt the buffers/HW. Hence, resetting of MAC is done     */
    /* inside critical section                                               */
    critical_section_start();

    /* Stop MAC HW and PHY HW first before going for reset */
    stop_mac_and_phy();

    /* Set the global to true (this is used to ensure that freeing is not    */
    /* done in the event manager (as memory is reinitialized here).          */
	//chenq move to end of this func 2012-11-02
	//g_mac_reset_done = BTRUE;

    /* Delete the MAC interrupts and alarms */
    delete_mac_interrupts();
    delete_mac_alarms();
    delete_phy_alarms();

    /* Delete all protocol related state information */
    delete_mac_prot();

    /* Free all beacon buffers used */
    free_beacon_buffers();

#ifndef MAC_HW_UNIT_TEST_MODE
    /* Stop the security associations */
    stop_sec_km();
#endif /* MAC_HW_UNIT_TEST_MODE */

    /* Memory pools used in CSL needs to be reset */
    csl_reset();

    destroy_mac_qmu(&g_q_handle);

    if(BTRUE == init_mac_sw)
    {
#ifndef MAC_HW_UNIT_TEST_MODE
        /* Initialize the MAC software again. Note that QMU, CSL and MM do not */
        /* need to be explicitly closed since they will be re-initialized here.*/
        initialize_macsw(mac);
#else /* MAC_HW_UNIT_TEST_MODE */
        /* Initialize MAC S/w */
        test_reinit_sw(mac);
#endif /* MAC_HW_UNIT_TEST_MODE */
    }
    else	//add by chengwg.
    {
		host_rx_queue_free();
#ifdef TX_PKT_USE_DMA		
		tx_dma_buf_free();
#endif		
		//chenq add 2012-10-29
		initialize_plus();
    }

    /* Software reset of the host interface for buffer re-allocation         */
    soft_reset_host_if();

    /* Enable interrupts */
    critical_section_end();
	
	//chenq move to end of this func 2012-11-02
	atomic_set(&g_mac_reset_done, (int)BTRUE);
	TROUT_DBG4("%s: reset OK!\n", __func__);
	
    TROUT_FUNC_EXIT;
}
#else

#include <linux/wait.h>
struct completion event_complete = COMPLETION_INITIALIZER(event_complete);

/* waiting for all packets transmitted, by zhao  */
void wait_for_tx_finsh(void)
{
	tx_barrier = 0x9;
	
	msleep(50);
	
	if(is_all_machw_q_null())
		tx_barrier &=  ~(1 << RST_BARRIER_TX);
loop:
	printk("RST_BARRIER: %x\n", tx_barrier);
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(10));
	if(tx_barrier != 0x8)
		goto loop;

	/* arrived here tx is finished safty*/
}

void clear_tx_barrier(void)
{
	tx_barrier = 0;
}


void reset_mac__lock(void)
{
	struct trout_private *tp = netdev_priv(g_mac_dev);
	if(tp == NULL) return 0;
	down_read(&tp->rst_semaphore);
	//printk("[libing]: reset_mac_trylock ret = %d,g_mac_reset_done = %d\n",ret,atomic_read(&g_mac_reset_done));
	//print_symbol("[libing] reset_mac_trylock:%s\n", (unsigned long)__builtin_return_address(0));
}

/*
 * trylock for reading -- returns 1 if successful, 0 if contention
 */
 
int reset_mac_trylock(void)
{
	int ret = 0;
	struct trout_private *tp = netdev_priv(g_mac_dev);
	if( tp == NULL || (BOOL_T)atomic_read(&g_mac_reset_done) == BFALSE ){
		return 0;
	}
	ret = down_read_trylock(&tp->rst_semaphore);
	//printk("[libing]: reset_mac_trylock ret = %d,g_mac_reset_done = %d\n",ret,atomic_read(&g_mac_reset_done));
	//print_symbol("[libing] reset_mac_trylock:%s\n", (unsigned long)__builtin_return_address(0));
	return ret;
}

void reset_mac_unlock(void)
{
	struct trout_private *tp = netdev_priv(g_mac_dev);
	if(tp == NULL) return;
	up_read(&tp->rst_semaphore);
	//print_symbol("[libing] reset_mac_unlock:%s\n", (unsigned long)__builtin_return_address(0));
}
/* judge write semaphore is locked */
int wsem_is_locked(void)	//chwg add.
{
	int ret = 1;
	unsigned long flags;
	struct trout_private *tp = netdev_priv(g_mac_dev);
	struct rw_semaphore *sem = &(tp->rst_semaphore);

	if(raw_spin_trylock_irqsave(&sem->wait_lock, flags)) 
	{
		ret = (sem->activity == -1);
		raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
	}
	return ret;
}
void reset_mac(mac_struct_t *mac, BOOL_T init_mac_sw)
{
	TROUT_FUNC_ENTER;
	struct trout_private *tp;
	

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	/*leon liu added, stop pstimer*/
	pstimer_stop(&pstimer);
#endif
	/* handle race condiction between some reset_macs by zhao  */
       
	tp = netdev_priv(g_mac_dev);
	host_write_trout_reg(convert_to_le(0xFFFFFFFF), (UWORD32)rCOMM_INT_MASK);
	printk("[reset_mac]: down_write >>>\n");
	down_write(&tp->rst_semaphore);
	printk("[reset_mac]: down_write <<<\n");
	atomic_set(&g_mac_reset_done, (int)BFALSE);

	wake_lock(&reset_mac_lock); /*Keep awake when resetting MAC, by keguang 20130609*/
	pr_info("[%s]: acquire wake_lock %s\n", __func__, reset_mac_lock.name);
	host_notify_arm7_connect_status(BFALSE);
#ifdef IBSS_BSS_STATION_MODE
	coex_state_switch(COEX_WIFI_IDLE);
	host_notify_arm7_wifi_reset(BTRUE);
	g_wifi_bt_coex = BFALSE;
#endif
	/*leon liu added cfg80211 report scan abort*/	
	
#ifdef  CONFIG_CFG80211
	trout_cfg80211_report_scan_done(g_mac_dev, 1);
#endif


#ifdef DEBUG_MODE
	printk("[%s]: reset=%d, caller:", __func__, init_mac_sw);
	print_symbol("%s\n", (unsigned long)__builtin_return_address(0));
	
    g_reset_stats.numresets++;
#endif /* DEBUG_MODE */

	//chenq add 2012-11-02
	//Begin:delete by wulei 2791 for bug 160423 on 2013-05-04
	//atomic_set(&g_mac_reset_done, (int)BFALSE);
	//End:delete by wulei 2791 for bug 160423 on 2013-05-04
    /* Perform appropriate pre-reset tasks related to host */
	//chenq mask 2012-11-02
    //pre_reset_tasks_host(mac);

	critical_section_start();

	printk("[reset_mac]\t1. stop netif queue!\n");
	netif_stop_queue(g_mac_dev);

    	/* Perform appropriate pre-reset tasks related to wireless lan */
    	pre_reset_tasks_wlan(mac);

	printk("[reset_mac]\t2. delete protocol alarm!\n");
	/* Delete all protocol related state information */
    	delete_mac_prot();
	/*leon liu masked tx_compelte*/
	printk("[reset_mac]\t3. skip tx_complete_isr_simulate!\n");
	/*tx_complete_isr_simulate();*/
	printk("[reset_mac]\t4. stop mac and phy!\n");
	/* Stop MAC HW and PHY HW first before going for reset */
    	stop_mac_and_phy();

	printk("[reset_mac]\t5. destroy mac txq & rxq!\n");
    	destroy_mac_qmu(&g_q_handle);

#ifdef BSS_ACCESS_POINT_MODE
	delete_beacon_thread();	//modify by chengwg.
#endif

    printk("[reset_mac]\t6. delete mac interrupt!\n");
    /* Delete the MAC interrupts and alarms */
    delete_mac_interrupts();
    printk("[reset_mac]\t7. delete mac alarm!\n");
    delete_mac_alarms();
    printk("[reset_mac]\t8. delete phy alarm!\n");
    delete_phy_alarms();

//    /* Delete all protocol related state information */
//    delete_mac_prot();

    /* Free all beacon buffers used */
    free_beacon_buffers();
    
	printk("[reset_mac]\tflush all pending event!\n");
    reset_all_event_q();	//add by chengwg, 2013.07.05
	printk("[reset_mac]\t9. wait for event complete!\n");
	mac_event_schedule();
	printk("[reset_mac]\t10. rcved event done msg!\n");

	printk("[reset_mac]\t11. flush rxq!\n");
	flush_all_rxq();	//add by chengwg.

#ifndef MAC_HW_UNIT_TEST_MODE
    /* Stop the security associations */
    stop_sec_km();
#endif /* MAC_HW_UNIT_TEST_MODE */

#ifdef IBSS_BSS_STATION_MODE
	//chenq add 2013-01-12
	//send_mac_status(MAC_SCAN_CMP);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	if(g_wifi_suspend_status != wifi_suspend_suspend)
		send_mac_status(MAC_DISCONNECTED);
	else
		pr_info("[%s] We won't send disconnect event!\n", __func__);
#else
	send_mac_status(MAC_DISCONNECTED);
#endif
#endif

    if(BTRUE == init_mac_sw)
    {
#ifndef MAC_HW_UNIT_TEST_MODE
        /* Initialize the MAC software again. Note that QMU, CSL and MM do not */
        /* need to be explicitly closed since they will be re-initialized here.*/
        initialize_macsw(mac);
#else /* MAC_HW_UNIT_TEST_MODE */
        /* Initialize MAC S/w */
        test_reinit_sw(mac);
#endif /* MAC_HW_UNIT_TEST_MODE */
    }

    /* Software reset of the host interface for buffer re-allocation         */
    soft_reset_host_if();

    /* Enable interrupts */
    critical_section_end();

#ifdef IBSS_BSS_STATION_MODE
// reset authentication and association retry time.
	g_auth_retry_cnt = 0;
	g_assoc_retry_cnt = 0;
	//host_write_trout_reg( host_read_trout_reg( (UWORD32)rSYSREG_HOST2ARM_INFO3 ) | BIT0 ,(UWORD32)rSYSREG_HOST2ARM_INFO3 );
#endif
	wake_unlock(&reset_mac_lock); /*Keep awake when resetting MAC, by keguang 20130609*/
	//chenq move to end of this func 2012-11-02
	atomic_set(&g_mac_reset_done, (int)BTRUE);
	/* handle race condiction between some reset_macs by zhao  */
	up_write(&tp->rst_semaphore);
#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
	/*leon liu added, start pstimer*/
	pstimer_start(&pstimer);
#endif
	pr_info("[%s]\t: release wake_lock %s\n", __func__, reset_mac_lock.name);
	TROUT_DBG4("[%s]\t: reset OK!\n", __func__);
    TROUT_FUNC_EXIT;
}
#endif

/*****************************************************************************/
/*                                                                           */
/*  Function Name : enable_mac_interrupts                                    */
/*                                                                           */
/*  Description   : This function enables all MAC interrupts.                */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_int_vector                                         */
/*                                                                           */
/*  Processing    : All the MAC interrupts are enabled. This function        */
/*                  needs to be called after creating the interrupts.        */
/*                                                                           */
/*  Outputs       : All interrupts are enabled. Interrupt processing         */
/*                  begins after enabling interrupts.                        */
/*                                                                           */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void enable_mac_interrupts(void)
{
    enable_interrupt(g_mac_int_vector);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : disable_mac_interrupts                                   */
/*                                                                           */
/*  Description   : This function disables all MAC interrupts.               */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : g_mac_int_vector                                         */
/*                                                                           */
/*  Processing    : All the MAC interrupts are disabled. This function is    */
/*                  needs to be called after reset command is issued by      */
/*                  the host.                                                */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void disable_mac_interrupts(void)
{
    disable_interrupt(g_mac_int_vector);
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : restart_mac                                              */
/*                                                                           */
/*  Description   : This function resets all the modules in MAC and restarts */
/*                  its opearations.                                         */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                  2) Duration in secs, for which mac must be held is reset */
/*                                                                           */
/*  Globals       : g_current_settings                                       */
/*                  g_current_len                                            */
/*                                                                           */
/*  Processing    : Call all the reset functions and at the end call MAC s/w */
/*                  initializing function.                                   */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void restart_mac(mac_struct_t *mac, UWORD32 delay)
{
	TROUT_FUNC_ENTER;
	struct trout_private *tp;

	//xuan yang, 2013-8-23, add wid mutex
	tp = netdev_priv(g_mac_dev);
	mutex_lock(&tp->rst_wid_mutex);
	
#ifdef DEBUG_MODE
	printk("%s", __FUNCTION__);
	print_symbol(" -> %s\n", (unsigned long)__builtin_return_address(0));
#endif
    g_reset_mac_in_progress    = BTRUE;

    /* Save the current configuration before resetting */
    save_wids();

    /* Reset MAC - Bring down PHY and MAC H/W, disable MAC interrupts and    */
    /* release all OS structures. Further, this function restarts the MAC    */
    /* again from start.                                                     */
    reset_mac(mac, BTRUE);

    led_display(0x00);

    /* Stay in reset condition for over given delay */
    if(delay != 0)
    {
        machw_poll_after_reset(delay);
    }

    led_display(0xFF);

    /* Restore the saved configuration before restting                       */
    restore_wids();

    /* This function resolves all configuration related conflicts that */
    /* might arise between various WIDs.                               */
    resolve_wid_conflicts(mac);

    g_reset_mac_in_progress       = BFALSE;

	mutex_unlock(&tp->rst_wid_mutex);

    start_mac_and_phy(mac);
    TROUT_FUNC_EXIT;
}

//chenq add 2012-12-26
void restart_mac_plus(mac_struct_t *mac, UWORD32 delay)
{
	TROUT_FUNC_ENTER;
	struct trout_private *tp;

	//xuan yang, 2013-8-23, add wid mutex
	tp = netdev_priv(g_mac_dev);
	printk("[%s]\t mutex_lock (rst_wid_mutex) ==>>", __FUNCTION__);
	mutex_lock(&tp->rst_wid_mutex);
	printk("[%s]\t mutex_lock (rst_wid_mutex) ==<<", __FUNCTION__);
	
#ifdef DEBUG_MODE
	printk("%s", __FUNCTION__);
	print_symbol(" -> %s\n", (unsigned long)__builtin_return_address(0));
#endif
    g_reset_mac_in_progress    = BTRUE;

    /* Save the current configuration before resetting */
    //save_wids();

    /* Reset MAC - Bring down PHY and MAC H/W, disable MAC interrupts and    */
    /* release all OS structures. Further, this function restarts the MAC    */
    /* again from start.                                                     */
//#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
//	if(g_wifi_suspend_status == wifi_suspend_suspend) /*by kelvin*/ 
//		reset_mac(mac, BFALSE);
//	else
//#endif
		reset_mac(mac, BTRUE);

    //led_display(0x00);

    /* Stay in reset condition for over given delay */
    if(delay != 0)
    {
        machw_poll_after_reset(delay);
    }

    //led_display(0xFF);

    /* Restore the saved configuration before restting                       */
    //restore_wids();

    /* This function resolves all configuration related conflicts that */
    /* might arise between various WIDs.                               */
    resolve_wid_conflicts(mac);

	g_reset_mac_in_progress       = BFALSE;

	mutex_unlock(&tp->rst_wid_mutex);
	printk("[%s]\texit", __FUNCTION__);
	TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : start_mac_and_phy                                        */
/*                                                                           */
/*  Description   : This function starts the MAC and PHY H/w operation.      */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The MAC and PHY H/w registers are initialized according  */
/*                  to the current settings and then their operations are    */
/*                  enabled.                                                 */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void start_mac_and_phy(mac_struct_t *mac)
{
	TROUT_FUNC_ENTER;
	
	/* avoid race condiction between rest_mac and start_ma_and_phy  by zhao */
        //mutex_lock(&tp->rst_mutex);
     /*jiangtao.yi changed reset_mac_trylock() to reset_mac__lock() for bug244758.*/
     reset_mac__lock();  

#ifdef DEBUG_MODE	
	printk("chenq_itm %s-%d: ", __FUNCTION__, __LINE__);    
	print_symbol("at %s\n", (unsigned long)__builtin_return_address(0));
#endif

#ifdef COMBO_SCAN	
	//chenq add 2013-04-05
	//calibrate_delay_loop_plus();
#endif

    /* Reset MAC and PHY H/w */
    reset_phy_machw();

    /* Initialize PHY, RF and MAC H/w. Note that the initializations need to */
    /* be done in this order exactly.                                        */
    update_phy_mib(mget_DesiredBSSType());
    init_phy_curr_rates();
    initialize_machw();
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

#ifndef HOST_LOOP_BACK_MODE
    /* The operation carried out will be dependent on the mode of operation, */
    /* if AP then initiate start is called, otherwise scanning process is    */
    /* initiated.                                                            */
    enable_operation(mac);

    //chenq add 2012-11-09
	if(BTRUE == is_machw_tx_suspended())
		set_machw_tx_resume();

	if(netif_queue_stopped(g_mac_dev))	//chwg test-2013-01-11
        netif_wake_queue(g_mac_dev);

    /* Once all initializations are complete the PA control register is      */
    /* programmed to enable PHY and MAC H/w.                                 */
    enable_machw_phy_and_pa();
#endif /* HOST_LOOP_BACK_MODE */
	/* avoid race condiction between rest_mac and start_ma_and_phy  by zhao */
	//mutex_unlock(&tp->rst_mutex);
     reset_mac_unlock();
	TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : stop_mac_and_phy                                         */
/*                                                                           */
/*  Description   : This function stops the MAC and PHY H/w operation.       */
/*                                                                           */
/*  Inputs        : None                                                     */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function stops the MAC & PHY H/w by following the   */
/*                  stop procedure.                                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void stop_mac_and_phy(void)
{
	TROUT_FUNC_ENTER;
    /* If MAC Hardware is enabled, then before disabling it put it in TX     */
    /* suspend mode                                                          */
    set_machw_tx_suspend();

    /* Disable MAC HW and PHY using PA control Register  */
    disable_machw_phy_and_pa();

    /* Clear all the buffers used by defragmentation by disabling it */
    disable_machw_defrag();

    /* Disable 20/40 operations */
    disable_2040_operation();

    /* Disable all CE features of MAC H/w before going for reset */
    flush_ce_lut();
    disable_machw_ce_features();

    /* Clear all the current pending interrupts in MAC HW */
    clear_machw_interrupts();
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : pre_reset_tasks_host                                     */
/*                                                                           */
/*  Description   : This function performs all host interface related tasks  */
/*                  before system reset.                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Currently nothing done here.                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void pre_reset_tasks_host(mac_struct_t *mac)
{
	TROUT_FUNC_ENTER;
	int smart_type = MODE_START;
    /* Wait for the host to be come idle */
    wait_for_host_idle();

    /* Before Reset, Informing the host is neccessary so that host doesn't send */
    /* any frames during the reset process */
    smart_type = critical_section_smart_start(0,1);

    /* Remove all the current pending packets to host  */
    remove_all_in_hifq();

    /* Send a packet (with format understood by host and device) to host informing */
    /* about the reset: TBD */
    /* Send MAC_DISCONNECTED status on every reset */
	//chenq mask
    //send_mac_status(MAC_DISCONNECTED);

    critical_section_smart_end(smart_type);

    /* Wait for this packet transmission */
    wait_for_host_idle();
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : pre_reset_tasks_wlan                                     */
/*                                                                           */
/*  Description   : This function performs all WLAN protocol related tasks   */
/*                  before system reset.                                     */
/*                                                                           */
/*  Inputs        : 1) Pointer to the MAC library structure                  */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : All pending frames are flushed out and all existing      */
/*                  associations are graciously broken.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void pre_reset_tasks_wlan(mac_struct_t *mac)
{
	struct trout_private *tp = netdev_priv(g_mac_dev); //jiangtao.yi
	TROUT_FUNC_ENTER;
#ifndef MAC_HW_UNIT_TEST_MODE
    /* If the device is an infrastrcure-device (i.e. AP or BSS-STA) and in      */
    /* connected state then send deauth frame to all the devices,connected to   */
    if((get_mac_state() == ENABLED) && (mget_DesiredBSSType() == INFRASTRUCTURE))
    {
		UWORD32 cnt = 0;
#ifdef IBSS_BSS_STATION_MODE
        UWORD8  null_bssid[6] = {0};
#endif /* IBSS_BSS_STATION_MODE  */

        /* Suspend the MAC HW For transmission                                   */
        /* It is assumed that after the suspension, MAC H/W Will not give any TX */
        /* complete INT                                                          */
		set_machw_tx_suspend();

		flush_all_qs();

        /* Resume MAC HW Tx */
		set_machw_tx_resume();

#ifdef IBSS_BSS_STATION_MODE
		/* If non-zero BSSID is present,then send de-auth frame */
		if(mac_addr_cmp(null_bssid, mget_bssid()) != BTRUE)
		{
		   send_deauth_frame(mget_bssid(), (UWORD16)UNSPEC_REASON);
		}
#endif /* IBSS_BSS_STATION_MODE  */

#ifdef BSS_ACCESS_POINT_MODE
		/* Inform all the associated stations and delete their enteries */
		unjoin_all_stations(UNSPEC_REASON);

		send_deauth_frame(mget_bcst_addr(), (UWORD16)UNSPEC_REASON);
#endif /* BSS_ACCESS_POINT_MODE */

        /* Wait on the transmission of this deauth */
        while((is_machw_q_null(HIGH_PRI_Q) == BFALSE) &&
               (cnt < DEAUTH_SEND_TIME_OUT_COUNT))
        {
            add_delay(0xFFF);
            cnt++;
        }
	if(cnt >= DEAUTH_SEND_TIME_OUT_COUNT)
		printk("@@@send deauth TIMEOUT\n");
    }
#endif /* MAC_HW_UNIT_TEST_MODE */
	TROUT_FUNC_EXIT;
}

extern atomic_t g_event_cnt;
void initialize_plus(void)
{
	init_mac_globals_plus();
	atomic_set(&g_event_cnt, 0);
}
