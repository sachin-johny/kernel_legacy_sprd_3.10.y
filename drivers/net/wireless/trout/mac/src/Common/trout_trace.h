/*******************************************************************************
* Copyright (c) 2011-2012,Spreadtrum Corporation
* All rights reserved.
* Filename: trout_trace.h
* Abstract: 
* 
* Version: 1.0
* Revison Log: 
* 	2012/9/11, Chengwg: Create this file.
* CVS Log: 
* 	$Id$
*******************************************************************************/

#ifndef TROUT_TRACE_H
#define TROUT_TRACE_H

#ifdef TROUT_TRACE_DBG

#include <linux/kallsyms.h>
#include <linux/timer.h>
#include "itypes.h"
#include "trace_log_fuc.h"


#undef TRACE_LOG_TO_FILE	//output log info to file '/mnt/asec/itm_trace.log'


extern UWORD32 io_cost_time;
extern UWORD32 io_count;

extern UWORD32 misc_tx_comp_count;
extern UWORD32 misc_tx_comp_time;

extern UWORD32 misc_tbtt_count;
extern UWORD32 misc_tbtt_time;

extern UWORD32 process_all_events_count;
extern UWORD32 process_all_events_time;

extern UWORD32 process_wlan_event_count;
extern UWORD32 process_host_event_count;
extern UWORD32 process_host_rx_event_count;
extern UWORD32 process_host_rx_event_time;

extern UWORD32 process_wlan_rx_event_count;
extern UWORD32 process_wlan_rx_event_time;

extern UWORD32 process_misc_event_count;
extern UWORD32 process_misc_event_time;

extern UWORD32 process_host_tx_event_count;
extern UWORD32 process_host_tx_event_time;

extern UWORD32 pro_call_mac_xmit_count;
extern UWORD32 rx_event_count;
extern UWORD32 tx_data_count;
extern UWORD32 tx_isr_count;
extern UWORD32 event_read_all_machw_q_null_count;
extern UWORD32 tx_isr_read_all_machw_q_null_count;

extern UWORD32 tx_isr_reg_count;

extern UWORD32 send_cpy_to_trout_count;
extern UWORD32 send_cpy_to_trout_cycle_count;
extern UWORD32 tx_isr_cpy_to_trout_count;
extern UWORD32 tx_isr_cpy_to_trout_cycle_count;

extern UWORD32 tx_event_read_all_machw_q_null_reg_count;
extern UWORD32 tx_isr_read_all_machw_q_null_reg_count;

extern UWORD32 event_is_tx_data_empty_count;
extern UWORD32 tx_isr_is_tx_data_empty_count;

extern UWORD32 event_is_tx_data_empty_reg_count;
extern UWORD32 tx_isr_is_tx_data_empty_reg_count;

extern UWORD32 event_write_cpy_to_trout_reg_count;
extern UWORD32 tx_isr_write_cpy_to_trout_reg_count;

extern UWORD32 event_tx_pkt_process_count;
extern UWORD32 tx_isr_pkt_process_count;

extern UWORD32 event_tx_process_reg_count;
extern UWORD32 tx_isr_process_reg_count;

extern UWORD32 mac_isr_work_count;
extern UWORD32 mac_isr_reg_count;

extern UWORD32 rx_isr_reg_count;
extern UWORD32 rx_isr_count;

extern UWORD32 err_isr_count;
extern UWORD32 err_isr_reg_count;

extern UWORD32 reg_ram_timeout_count;

//rx pkt info...
extern int g_rx_dbg_flag;
extern UWORD32 rx_mgmt_frame_count[];
extern UWORD32 rx_ctrl_frame_count[];
extern UWORD32 rx_data_frame_count[];
extern UWORD32 rx_rsev_frame_count[];

extern volatile UWORD32 g_trout_dbg_mask;
extern UWORD16 *g_trace_count_buf;
extern UWORD32 mac_xmit_seed;

#define TX_ISR_CALL		1
#define TX_SEND_CALL	2

#define PKT_TJ_THRESHOLD	1400	//统计长度大于1400的包
#define TJ_NUM				5000
#define TRACE_BUF_SIZE		(sizeof(UWORD16) * TJ_NUM)	//记录5000项数据
#define TX_PKT_INFO_OFFSET	25

#define TROUT_DBG(mask, fmt, arg...)	do{	\
											if(g_trout_dbg_mask & mask)	\
												printk(fmt, ##arg);	\
										}while(0)

#define TROUT_FUNC_ENTER	/*do{	\
								TROUT_DBG4("%s, line %d: enter! ", __FUNCTION__, __LINE__);	\
								print_symbol("-> %s\n", (unsigned long)__builtin_return_address(0));	\
							}while(0)*/
#define TROUT_FUNC_EXIT		/*TROUT_DBG4("%s, line %d: exit!\n", __FUNCTION__, __LINE__)*/

#define TRACE_FUNC_ENTER
#define TRACE_FUNC_EXIT

#if 0	//test.
extern int create_trace_log_file(void);
extern int close_trace_log_file(void);
extern int print_log(const char *fmt, ...);
#endif

INLINE void set_value(UWORD8 *buf, UWORD32 offset, UWORD16 value)
{
	*((UWORD16 *)(buf + offset)) = value;
}

INLINE UWORD16 get_value(UWORD8 *buf, UWORD32 offset)
{
	return *((UWORD16 *)(buf + offset));
}

INLINE UWORD32 trout_dbg_mask_get(void)
{
	return g_trout_dbg_mask;
}

INLINE void trout_dbg_mask_set(UWORD32 flag)
{
	g_trout_dbg_mask |= flag;
}

INLINE void trout_dbg_mask_clear(UWORD32 flag)
{
	g_trout_dbg_mask &= ~flag;
}

void clear_rx_pkt_cnt_info(void);
void print_rx_pkt_cnt_info(void);
void mac_status_show(void);

int start_trace_info_show_thread(void);
void trace_thread_exit(void);

int trace_func_init(void);
void trace_func_exit(void);

void print_count_info(void);
void clean_count_info(void);
#else	/* TROUT_TRACE_DBG */

#define TROUT_DBG(mask, fmt, arg...)

/*
#define TRACE_FUNC_ENTER	do{	\
								TROUT_DBG4("%s, line %d: enter! ", __FUNCTION__, __LINE__);	\
								print_symbol("-> %s\n", (unsigned long)__builtin_return_address(0));	\
							}while(0)
#define TRACE_FUNC_EXIT		TROUT_DBG4("%s, line %d: exit!\n", __FUNCTION__, __LINE__)
*/

#define TRACE_FUNC_ENTER
#define TRACE_FUNC_EXIT

INLINE void mem_using_count_inc(void)
{
	/* do nothing */
}

INLINE void mem_using_count_dec(void)
{
	/* do nothing */
}
#endif	/* TROUT_TRACE_DBG */


/******************************************************************************/
/*                    first log level defing                                  */
/******************************************************************************/
#define TROUT_DEBUG_MASK0		0x00000001		/* Reserved */
#define TROUT_DEBUG_MASK1		0x00000002		/* Fatal error */
#define TROUT_DEBUG_MASK2		0x00000004		/* Common error */
#define TROUT_DEBUG_MASK3		0x00000008		/* Warning message */
#define TROUT_DEBUG_MASK4		0x00000010		/* normal but significant message */
#define TROUT_DEBUG_MASK5		0x00000020		/* detail information */
#define TROUT_DEBUG_MASK6		0x00000040		/* Frequency function display --debug use!*/
#define TROUT_DEBUG_ALL			0xFFFFFFFF		/* all informations */

#define TROUT_DBG0(fmt, arg...)			TROUT_DBG(TROUT_DEBUG_MASK0, fmt, ##arg)
#define TROUT_DBG1(fmt, arg...)			TROUT_DBG(TROUT_DEBUG_MASK1, fmt, ##arg)
#define TROUT_DBG2(fmt, arg...)			TROUT_DBG(TROUT_DEBUG_MASK2, fmt, ##arg)
#define TROUT_DBG3(fmt, arg...)			TROUT_DBG(TROUT_DEBUG_MASK3, fmt, ##arg)
#define TROUT_DBG4(fmt, arg...)			TROUT_DBG(TROUT_DEBUG_MASK4, fmt, ##arg)
#define TROUT_DBG5(fmt, arg...)			TROUT_DBG(TROUT_DEBUG_MASK5, fmt, ##arg)
#define TROUT_DBG6(fmt, arg...)			TROUT_DBG(TROUT_DEBUG_MASK6, fmt, ##arg)
#define TROUT_DBG_ALL(fmt, arg...)		TROUT_DBG(TROUT_DEBUG_ALL, fmt, ##arg)

/******************************************************************************/
/*                    detail log level defing                                 */
/******************************************************************************/
/* TX debug level define */
#define TROUT_TX_DBG0  TROUT_DBG0
#define TROUT_TX_DBG1  TROUT_DBG1
#define TROUT_TX_DBG2  TROUT_DBG2
#define TROUT_TX_DBG3  TROUT_DBG3
#define TROUT_TX_DBG4  TROUT_DBG4
#define TROUT_TX_DBG5  TROUT_DBG5
#define TROUT_TX_DBG6  TROUT_DBG6

/* RX debug level define */
#define TROUT_RX_DBG0  TROUT_DBG0
#define TROUT_RX_DBG1  TROUT_DBG1
#define TROUT_RX_DBG2  TROUT_DBG2
#define TROUT_RX_DBG3  TROUT_DBG3
#define TROUT_RX_DBG4  TROUT_DBG4
#define TROUT_RX_DBG5  TROUT_DBG5
#define TROUT_RX_DBG6  TROUT_DBG6

/* INT debug level define */
#define TROUT_INT_DBG0  TROUT_DBG0
#define TROUT_INT_DBG1  TROUT_DBG1
#define TROUT_INT_DBG2  TROUT_DBG2
#define TROUT_INT_DBG3  TROUT_DBG3
#define TROUT_INT_DBG4  TROUT_DBG4
#define TROUT_INT_DBG5  TROUT_DBG5

#define TX_PATH_DBG		TROUT_DBG6
#define RX_PATH_DBG		TROUT_DBG5
#define TX_BA_DBG		TROUT_DBG6

#define TX_INT_DBG		TROUT_DBG6	//tx lose int debug!!!
#define TROUT_IO_DBG	TROUT_DBG6
#define TROUT_REG_DBG	TROUT_DBG6
#define TROUT_MEM_LEAK_DBG	TROUT_DBG6	//used for memory leak dbg.

#define TROUT_PRINT(str, val)	TROUT_DBG4("%-48s = 0x%X\n", str, val)


void dump_io_buf_info(int index);
int print_io_sta_info(void);
int get_index(UWORD32 addr);



#if 1
void trout_register_read_test(void);
void trout_register_write_test(void);
void trout_ram_read_test(void);
void trout_ram_write_test(void);
#endif

#endif	/* TROUT_TRACE_H */
