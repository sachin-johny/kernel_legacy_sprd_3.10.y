/*******************************************************************************
* Copyright (c) 2011-2012,Spreadtrum Corporation
* All rights reserved.
* Filename: trout_trace.c
* Abstract: 
* 
* Version: 1.0
* Revison Log: 
* 	2012/9/11, Chengwg: Create this file.
* CVS Log: 
* 	$Id$
*******************************************************************************/
#ifdef TROUT_TRACE_DBG
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/delay.h>
/*leon liu modified <unistd.h> to <linux/unistd.h>*/
#include <linux/unistd.h>
/*leon liu modified <time.h> to <linux/time.h>*/
#include <linux/time.h>

#include "spi_interface.h"
#include "csl_linux.h"

#include "iconfig.h"
#include "imem_if.h"
#include "qmu_if.h"
#include "mib.h"
#include "trout_trace.h"


UWORD32 misc_tx_comp_count = 0;
UWORD32 misc_tx_comp_time = 0;

UWORD32 misc_tbtt_count = 0;
UWORD32 misc_tbtt_time = 0;

UWORD32 process_all_events_count = 0;
UWORD32 process_all_events_time = 0;

UWORD32 process_wlan_event_count = 0;
UWORD32 process_host_event_count = 0;

UWORD32 process_host_rx_event_count = 0;
UWORD32 process_host_rx_event_time = 0;

UWORD32 process_wlan_rx_event_count = 0;
UWORD32 process_wlan_rx_event_time = 0;

UWORD32 process_misc_event_count = 0;
UWORD32 process_misc_event_time = 0;

UWORD32 process_host_tx_event_count = 0;
UWORD32 process_host_tx_event_time = 0;

UWORD32 pro_call_mac_xmit_count = 0;
UWORD32 rx_event_count = 0;
UWORD32 tx_data_count = 0;
UWORD32 tx_isr_count = 0;
UWORD32 is_all_machw_q_null_count = 0;

UWORD32 event_read_all_machw_q_null_count = 0;
UWORD32 tx_isr_read_all_machw_q_null_count = 0;

UWORD32 tx_isr_reg_count = 0;

UWORD32 send_cpy_to_trout_count = 0;
UWORD32 send_cpy_to_trout_cycle_count = 0;
UWORD32 tx_isr_cpy_to_trout_count = 0;
UWORD32 tx_isr_cpy_to_trout_cycle_count = 0;

UWORD32 tx_event_read_all_machw_q_null_reg_count = 0;
UWORD32 tx_isr_read_all_machw_q_null_reg_count = 0;

UWORD32 event_is_tx_data_empty_count  = 0;
UWORD32 tx_isr_is_tx_data_empty_count  = 0;

UWORD32 event_is_tx_data_empty_reg_count = 0;
UWORD32 tx_isr_is_tx_data_empty_reg_count = 0;

UWORD32 event_write_cpy_to_trout_reg_count = 0;
UWORD32 tx_isr_write_cpy_to_trout_reg_count = 0;

UWORD32 event_tx_pkt_process_count = 0;
UWORD32 tx_isr_pkt_process_count = 0;

UWORD32 event_tx_process_reg_count = 0;
UWORD32 tx_isr_process_reg_count = 0;

UWORD32 mac_isr_work_count = 0;
UWORD32 mac_isr_reg_count = 0;

UWORD32 rx_isr_reg_count = 0;
UWORD32 rx_isr_count = 0;

UWORD32 err_isr_count = 0;
UWORD32 err_isr_reg_count = 0;

UWORD32 reg_ram_timeout_count = 0;

UWORD32 io_cost_time = 0;
UWORD32 io_count = 0;

//rx pkt info...
int g_rx_dbg_flag = 0;	//rx debug switch.
UWORD32 rx_mgmt_frame_count[2] = {0, 0};
UWORD32 rx_ctrl_frame_count[2] = {0, 0};
UWORD32 rx_data_frame_count[2] = {0, 0};
UWORD32 rx_rsev_frame_count[2] = {0, 0};

volatile UWORD32 g_trout_dbg_mask = 0x1F;	//debug.
UWORD16 *g_trace_count_buf = NULL;

UWORD32 mac_xmit_seed = 1;

static struct task_struct *trace_thread = NULL;

extern volatile UWORD32 g_trout_last_tx_dscr;
extern BOOL_T is_all_machw_q_null_test(void);
void print_count_info(void)
{
	int i = 0;
	UWORD32 tempVal = 0;
	UWORD8 qnum;

	printk("mac_xmit:\n");
	printk("pro_call_mac_xmit_count = %u\n", pro_call_mac_xmit_count);
	printk("tx_data_frame_len = %u\n", g_mac_stats.tx_data_frame_len);
	printk("tx_data_frame_count = %u\n\n", g_mac_stats.tx_data_frame_count);

	printk("process_host_rx_event:\n");
	printk("rx_event_count = %u\n\n", rx_event_count);

	printk("sta_enabled_tx_data:\n");
	printk("tx_data_count = %u\n", tx_data_count);
	printk("tx_queue_full_count = %u\n\n", g_mac_stats.tx_queue_full_count);

	printk("qmu_add_tx_packet:\n");
	printk("g_mac_stats.qatxp = %u\n", g_mac_stats.qatxp);
	for(i = 0; i < NUM_TX_QUEUE; i++)
		printk("qa[%d] = %u\n", i, g_mac_stats.qa[i]);
	printk("\n");

	printk("txq full detail:\n");
	for(i = 0; i < NUM_TX_QUEUE; i++)
		printk("txq %d full = %u\n", i, g_mac_stats.qfull[i]);
	printk("\n");
		
	printk("is_all_machw_q_null:\n");
	printk("event_read_all_machw_q_null_count = %u\n", event_read_all_machw_q_null_count);
	printk("tx_isr_read_all_machw_q_null_count = %u\n", tx_isr_read_all_machw_q_null_count);
	printk("tx_event_read_all_machw_q_null_reg_count = %u\n", tx_event_read_all_machw_q_null_reg_count);
	printk("tx_isr_read_all_machw_q_null_reg_count = %u\n\n", tx_isr_read_all_machw_q_null_reg_count);

	printk("qmu_cpy_to_trout:\n");
	printk("send_cpy_to_trout_count = %u\n", send_cpy_to_trout_count);
	printk("tx_isr_cpy_to_trout_count = %u\n\n", tx_isr_cpy_to_trout_count);
	
	printk("send_cpy_to_trout_cycle_count = %u\n", send_cpy_to_trout_cycle_count);
	printk("tx_isr_cpy_to_trout_cycle_count = %u\n\n", tx_isr_cpy_to_trout_cycle_count);
	
	printk("event_write_cpy_to_trout_reg_count = %u\n", event_write_cpy_to_trout_reg_count);
	printk("tx_isr_write_cpy_to_trout_reg_count = %u\n\n", tx_isr_write_cpy_to_trout_reg_count);

	printk("is_trout_tx_data_empty:\n");
	printk("event_is_tx_data_empty_count = %u\n", event_is_tx_data_empty_count);
	printk("tx_isr_is_tx_data_empty_count = %u\n", tx_isr_is_tx_data_empty_count);
	printk("event_is_tx_data_empty_reg_count = %u\n", event_is_tx_data_empty_reg_count);
	printk("tx_isr_is_tx_data_empty_reg_count = %u\n\n", tx_isr_is_tx_data_empty_reg_count);
	
	printk("tx_pkt_process:\n");
	printk("event_tx_pkt_process_count = %u\n", event_tx_pkt_process_count);
	printk("tx_isr_pkt_process_count = %u\n", tx_isr_pkt_process_count);
	
	printk("event_tx_process_reg_count = %u\n", event_tx_process_reg_count);
	printk("tx_isr_process_reg_count = %u\n\n", tx_isr_process_reg_count);

	printk("mac_isr_work:\n");
	printk("mac_isr_work_count = %u\n", mac_isr_work_count);
	printk("mac_isr_reg_count = %u\n\n", mac_isr_reg_count);

	printk("rx_complete_isr:\n");
	printk("rx_isr_count = %u\n", rx_isr_count);
	printk("rx_isr_reg_count = %u\n\n", rx_isr_reg_count);

	printk("tx_complete_isr:\n");
	printk("tx_isr_count = %u\n", tx_isr_count);
	printk("tx_isr_reg_count = %u\n\n", tx_isr_reg_count);

	printk("error_isr:\n");
	printk("err_isr_count = %u\n", err_isr_count);
	printk("err_isr_reg_count = %u\n", err_isr_reg_count);

	printk("reg_ram_timeout_count = %u\n", reg_ram_timeout_count);

	printk("process_all_event:\n");
	printk("process_all_events_count = %u\n", process_all_events_count);
	printk("process_all_events_time = %u\n\n", process_all_events_time);
	
	printk("process_wlan_event_count = %u\n", process_wlan_event_count);
	printk("process_host_event_count = %u\n\n", process_host_event_count);

	printk("process_host_rx_event_count = %u\n", process_host_rx_event_count);
	printk("process_host_rx_event_time = %u\n\n", process_host_rx_event_time);
	
	printk("process_wlan_rx_event_count = %u\n", process_wlan_rx_event_count);
	printk("process_wlan_rx_event_time = %u\n\n", process_wlan_rx_event_time);
	
	printk("process_misc_event_count = %u\n", process_misc_event_count);
	printk("process_misc_event_time = %u\n\n", process_misc_event_time);
	
	printk("process_host_tx_event_count = %u\n", process_host_tx_event_count);
	printk("process_host_tx_event_time = %u\n\n", process_host_tx_event_time);

	printk("misc_tx_comp_count = %u\n", misc_tx_comp_count);
	printk("misc_tx_comp_time = %u\n\n", misc_tx_comp_time);
	
	printk("misc_tbtt_count = %u\n", misc_tbtt_count);
	printk("misc_tbtt_time = %u\n\n", misc_tbtt_time);
	printk("\n----------------------------------------------------\n");
	qnum = g_q_handle.tx_handle.tx_curr_qnum;
//	printk("tx_curr_qnum = %d, txq[%d].handle_pkt = %d, tx_pkt_count = %u\n", 
//		qnum, qnum, g_q_handle.tx_handle.tx_header[qnum].handle_pkt, g_q_handle.tx_handle.tx_pkt_count);
	printk("tx_list_count = %u\n", g_q_handle.tx_handle.tx_list_count);
	printk("tx_frame_pointer: 0x%08x\n", get_machw_tx_frame_pointer());
	printk("num_tx_frame: %d\n", get_machw_num_tx_frames());

	printk("PA Control: 0x%08x\n", host_read_trout_reg((UWORD32)rMAC_PA_CON));
	printk("PA status: 0x%08x\n", host_read_trout_reg((UWORD32)rMAC_PA_STAT));

	printk("int status: 0x%08x\n", convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2);
	printk("int mask: 0x%08x, local mask: 0x%08x\n", convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_MASK)) >> 2,
						get_trout_int_mask());
	
	is_all_machw_q_null_test();

	mutex_lock(&g_q_handle.tx_handle.txq_lock);
	if(g_trout_last_tx_dscr)
	{
		host_read_trout_ram(&tempVal, (UWORD32 *)g_trout_last_tx_dscr + WORD_OFFSET_0, 4);

		tempVal = (tempVal >> 29) & 0x3;
		printk("last trout dscr(0x%x) status: 0x%x\n", g_trout_last_tx_dscr, tempVal);
		printk("tx mem range: 0x%x ~ 0x%x\n", g_q_handle.tx_handle.tx_mem_start,
				g_q_handle.tx_handle.tx_mem_start+g_q_handle.tx_handle.tx_mem_size);
	}
	mutex_unlock(&g_q_handle.tx_handle.txq_lock);

	printk("io_cost_time: %u\n", io_cost_time);
	printk("io_count: %u\n", io_count);

	//printk("HZ = %d\n", HZ);
	//close_trace_log_file();		//define in mmc.
}

void clean_count_info(void)
{
	pro_call_mac_xmit_count = 0;
	g_mac_stats.tx_data_frame_len = 0;
	g_mac_stats.tx_data_frame_count = 0;

	rx_event_count = 0;
	
	tx_data_count = 0;
	g_mac_stats.tx_queue_full_count = 0;
	
	g_mac_stats.qatxp = 0;
	memset(g_mac_stats.qa, 0, sizeof(g_mac_stats.qa));

	memset(g_mac_stats.qfull, 0, sizeof(g_mac_stats.qfull));

	event_read_all_machw_q_null_count = 0;
	tx_isr_read_all_machw_q_null_count = 0;
	tx_event_read_all_machw_q_null_reg_count = 0;
	tx_isr_read_all_machw_q_null_reg_count = 0;

	send_cpy_to_trout_count = 0;
	tx_isr_cpy_to_trout_count = 0;

	send_cpy_to_trout_cycle_count = 0;
	tx_isr_cpy_to_trout_cycle_count = 0;

	event_write_cpy_to_trout_reg_count = 0;
	tx_isr_write_cpy_to_trout_reg_count = 0;

	event_is_tx_data_empty_count = 0;
	tx_isr_is_tx_data_empty_count = 0;
	event_is_tx_data_empty_reg_count = 0;
	tx_isr_is_tx_data_empty_reg_count = 0;
	
	tx_isr_count = 0;
	tx_isr_reg_count = 0;
	
	event_tx_pkt_process_count = 0;
	tx_isr_pkt_process_count = 0;
	
	event_tx_process_reg_count = 0;
	tx_isr_process_reg_count = 0;

	mac_isr_work_count = 0;
	mac_isr_reg_count = 0;

	rx_isr_count = 0;
	rx_isr_reg_count = 0;

	err_isr_count = 0;
	err_isr_reg_count = 0;

	reg_ram_timeout_count = 0;

	process_all_events_count = 0;
	process_all_events_time = 0;
	
	process_wlan_event_count = 0;
	process_host_event_count = 0;

	process_host_rx_event_count = 0;
	process_host_rx_event_time = 0;
	
	process_wlan_rx_event_count = 0;
	process_wlan_rx_event_time = 0;
	
	process_misc_event_count = 0;
	process_misc_event_time = 0;
	
	process_host_tx_event_count = 0;
	process_host_tx_event_time = 0;

	misc_tx_comp_count = 0;
	misc_tx_comp_time = 0;
	
	misc_tbtt_count = 0;
	misc_tbtt_time = 0;

	io_cost_time = 0;
	io_count = 0;

	//create_trace_log_file();	//clean trace log file.
}

void clear_rx_pkt_cnt_info(void)
{
	int i;

	for(i=0; i<2; i++)
	{
		rx_mgmt_frame_count[i] = 0;
		rx_ctrl_frame_count[i] = 0;
		rx_data_frame_count[i] = 0;
		rx_rsev_frame_count[i] = 0;
	}
}

void print_rx_pkt_cnt_info(void)
{
	int i;

	for(i=0; i<2; i++)
	{
		TROUT_DBG4("rx_mgmt_frame_count[%d] = %u\n", i, rx_mgmt_frame_count[i]);
		TROUT_DBG4("rx_ctrl_frame_count[%d] = %u\n", i, rx_ctrl_frame_count[i]);
		TROUT_DBG4("rx_data_frame_count[%d] = %u\n", i, rx_data_frame_count[i]);
		TROUT_DBG4("rx_rsev_frame_count[%d] = %u\n", i, rx_rsev_frame_count[i]);
	}
}

void mac_status_show(void)
{
	UWORD32 int_mask, int_stat;
	
	TROUT_DBG4("mac_state = %d\n", get_mac_state());

	int_mask = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_MASK)) >> 2;
	int_stat = convert_to_le(host_read_trout_reg((UWORD32)rCOMM_INT_STAT)) >> 2;
	TROUT_DBG4("int_mask=0x%x, int_stat=0x%x\n", int_mask, int_stat);

	int_mask = host_read_trout_reg((UWORD32)rMAC_HIP_RX_BUFF_ADDR);
	int_stat = host_read_trout_reg((UWORD32)rMAC_RX_BUFF_ADDR);
	TROUT_DBG4("rx high buff addr=0x%x, rx normal buff addr=0x%x\n", int_mask, int_stat);

	int_mask = host_read_trout_reg((UWORD32)rMAC_PA_CON);
	int_stat = host_read_trout_reg((UWORD32)rMAC_PA_STAT);
	TROUT_DBG4("pa con=0x%x, pa stat=0x%x\n", int_mask, int_stat);

	TROUT_DBG4("err_isr_count=%u\n", err_isr_count);
}

void print_mac_debug_reg_info(void)
{
	TROUT_DBG4("\n====  mac register statistics log ===\n");

	TROUT_PRINT("PA Ctrl reg value(0x04)",          get_machw_pa_reg_value());
	TROUT_PRINT("FCS Failures(0x8C)",               get_fcs_count());
    TROUT_PRINT("RX_frame_into_ram_count(0xC4)",    get_rx_ram_packet_count());
    TROUT_PRINT("RX_frame_filter_count(0x50)",      get_rx_frame_filter_count());
    TROUT_PRINT("RX_mac_header_filter_count(0x54)", get_rx_mac_header_filter_count());
    TROUT_PRINT("Rxq_full_filter_count(0x5C)",      get_rxq_full_filter_count());
	TROUT_PRINT("RX complete count(0x24)",          get_rx_complete_count());
    TROUT_PRINT("TX complete count(0x24)",          get_tx_complete_count());
    TROUT_PRINT("Duplicate frames(0x94)",           get_dup_count());
    TROUT_PRINT("RX-End Count(0x98)",               get_machw_rx_end_count());
    TROUT_PRINT("RX-End Error Count(0x9C)",         get_machw_rx_error_end_count());
	TROUT_PRINT("RX AMPDU Count(0xA0)",             get_machw_rxd_ampdu_count());
	TROUT_PRINT("MPDU in rcved AMPDU Count(0xA4)",  get_machw_rx_mpdus_in_ampdu_count());
	TROUT_PRINT("Rcved Octets in AMPDU Count(0xA8)",get_machw_rx_bytes_in_ampdu_count());
	TROUT_PRINT("AMPDU Delimiter CRC Err Count(0xAC)",get_machw_ampdu_dlmt_crc_error_count());
	TROUT_PRINT("Transmit AMPDU Count(0x2AC)",   get_machw_txd_ampdu_count());
	TROUT_PRINT("Transmit MPDUs in AMPDU Count(0x2B0)",get_machw_tx_mpdus_in_ampdu_count());
	TROUT_PRINT("Transmit Octets in AMPDU Count(0x2B4)",get_machw_tx_bytes_in_ampdu_count());
}


void reset_mac_debug_reg(void)
{
	set_fcs_count(0);
	set_rx_ram_packet_count(0);
	set_rx_frame_filter_count(0);
	set_rx_mac_header_filter_count(0);
	set_rxq_full_filter_count(0);
	set_dup_count(0);
	set_machw_rx_end_count(0);
	set_machw_rx_error_end_count(0);
}

void tx_procedure_log_show(void)
{
	TROUT_DBG4("\n===== tx procedure log info show =====\n");

	TROUT_PRINT("tx data frame length", g_mac_stats.tx_data_frame_len);
	TROUT_PRINT("tx data frame count", g_mac_stats.tx_data_frame_count);	
	TROUT_PRINT("tx mgmt frame length", g_mac_stats.tx_mgmt_frame_len);
	TROUT_PRINT("tx mgmt frame count", g_mac_stats.tx_mgmt_frame_count);
	TROUT_PRINT("tx pkt length exceeds max limit(1596)", g_mac_stats.ehrxexc1);
	TROUT_PRINT("tx queue full count", g_mac_stats.tx_queue_full_count);
	TROUT_PRINT("tx busy count", g_mac_stats.tx_busy_count);
	txq_handle_detail_show();
}

void rx_procedure_log_show(void)
{
	TROUT_DBG4("\n===== rx procedure log info show =====\n");
	
	TROUT_PRINT("total rcvd rx dscr num", g_mac_stats.rcv_rx_dscr_num);
	TROUT_PRINT("total rx drop pkt count", g_mac_stats.rx_drop_pkt_count);
	TROUT_PRINT("total host rxq full count", g_mac_stats.host_rx_queue_full);
	TROUT_PRINT("Rx frames exceed max acceptable length", g_mac_stats.pwrx_maxlenexc);
	TROUT_PRINT("rcvd Protocol Version Match Fail", g_mac_stats.rxfrmhdrprotverfail);
	TROUT_PRINT("rcvd pkt BSSID Match Fail", g_mac_stats.rxfrmhdrbssidchkfail);
	TROUT_PRINT("rcvd pkt Less than min accept length", g_mac_stats.pwrx_minlenexc);
	TROUT_PRINT("total rcvd ctrl frame count", g_mac_stats.rx_ctrl_frame_count);
	TROUT_PRINT("total rcvd ctrl frame length", g_mac_stats.rx_ctrl_frame_len);
	
	TROUT_PRINT("total rcvd beacon frame count", g_mac_stats.rcv_beacon_fram_count);
	TROUT_PRINT("total rcvd prob rsp frame count", g_mac_stats.rcv_prob_rsp_count);
	TROUT_PRINT("total rcvd data frame length", g_mac_stats.rx_data_frame_len);
	TROUT_PRINT("total rcvd data frame count", g_mac_stats.rx_data_frame_count);

	rx_share_ram_show();
	host_rx_buff_show();
}


void interrupt_log_show(void)
{
	TROUT_DBG4("\n===== interrupt log info show =====\n");
	
	TROUT_PRINT("total mac int count", g_mac_stats.imac);
    TROUT_PRINT("total tbtt int count", g_mac_stats.itbtt);
    TROUT_PRINT("total tx complete int count", g_mac_stats.itxc);
    TROUT_PRINT("total normal rx complete int count", g_mac_stats.irxc);
    TROUT_PRINT("total deauth int count", g_mac_stats.ideauth);
    TROUT_PRINT("total CAP End int count", g_mac_stats.icapend);	
}


int memory_log_show(void *data)
{
	TROUT_DBG4("\n===== memory log info show =====\n");
	
	trout_share_ram_show();
	mac_mem_detail_show();
	
	TROUT_PRINT("no event mem count", g_reset_stats.no_event_mem);
	TROUT_PRINT("no share mem count", g_reset_stats.no_shred_mem);
	TROUT_PRINT("no local mem count", g_reset_stats.no_local_mem);
	TROUT_PRINT("total txq error count", g_reset_stats.tx_q_error);
	TROUT_PRINT("no event mem count", g_reset_stats.rx_q_error);
	return 0;
}


int start_trace_info_show_thread(void)
{
	int err;
	
	trace_thread = kthread_create(memory_log_show, NULL, "trace_thread");
	if(IS_ERR(trace_thread))
	{
		printk("Unable to start trace info show thread.\n");
		err = PTR_ERR(trace_thread);
		trace_thread = NULL;
		return err;
	}
	else
		wake_up_process(trace_thread);

	return 0;
}


void trace_thread_exit(void)
{
	printk(KERN_ALERT "trace info show thread exit...\n");
	if(trace_thread)
	{
		kthread_stop(trace_thread);
		trace_thread = NULL;
	}
}


int trace_func_init(void)
{
	printk("trace func init...\n");
	if(g_trace_count_buf == NULL)
	{
		g_trace_count_buf = (UWORD16 *)kmalloc(TRACE_BUF_SIZE, GFP_KERNEL);
		if(g_trace_count_buf == NULL)
		{
			printk("alloc trace count buffer failed!\n");
			return -ENOMEM;
		}
	}

	memset((void *)g_trace_count_buf, 0xFF, TRACE_BUF_SIZE);
	
	return 0;
}


void trace_func_exit(void)
{
	printk("trace func exit...\n");
	if(g_trace_count_buf != NULL)
	{
		kfree(g_trace_count_buf);
		g_trace_count_buf = NULL;
	}
}

#endif	/* TROUT_TRACE_DBG */
