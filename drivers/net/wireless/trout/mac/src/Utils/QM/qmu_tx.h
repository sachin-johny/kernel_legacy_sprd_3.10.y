#ifndef QMU_TX_H
#define QMU_TX_H

#define TX_SHARERAM_IDLE  0
#define TX_SHARERAM_WAIT  1       //wait for data copy,and construction.
#define TX_SHARERAM_READY 2       //data is ready, waiting for hw tx.
#define TX_SHARERAM_BUSY  3       //data is transfering.
#define TX_SHARERAM_FULL  4       //data is ready and full, waiting for hw tx.

#define COEX_WIFI_IDLE 0         // wifi idle, need a little period in coexistence.
#define COEX_WIFI_ON_CONNECTING 1   // wifi is connecting, need more period in coexistence to guarantee connecting an AP.
#define COEX_WIFI_CONNECTED 2   // wifi has connected an AP.
#define COEX_WIFI_ON_SCANNING 3 // wifi is scanning.
#define COEX_WIFI_SCAN_OVER 4 // scan over
#define COEX_WIFI_8021X_DHCP 5

#define COEX_TRAFFIC_CHECK_PERIOD 1000  // 1s
#define TRAFFIC_LEVEL_1 1
#define TRAFFIC_LEVEL_2 2
#define TRAFFIC_LEVEL_3 3
#define TRAFFIC_LEVEL_4 4
#define TRAFFIC_LEVEL_ON_CONNECTING 5
#define TRAFFIC_LEVEL_ON_SCANNING 6

#define COEX_TRAFFIC_THRESHOLD 10
#define COEX_TRAFFIC_THRESHOLD_1 5
#define COEX_TRAFFIC_THRESHOLD_2 10
#define COEX_TRAFFIC_THRESHOLD_3 500
#define COEX_CNTNUE_INC_LVL_CNT_THRD 1
#define COEX_CNTNUE_DEC_LVL_CNT_THRD 5
#define COEX_WIFI_TX_PKG 0
#define COEX_WIFI_RX_PKG 1
#define TX_MAX_PKT_PER_SLOT 24    //single slot max pkt number.

struct trout_tx_shareram{
    UWORD32 begin;
    UWORD32 end;
    UWORD32 curr;
    UWORD32 desc_addr[TX_MAX_PKT_PER_SLOT];		 //Hugh: ugly! just for debug.
    UWORD8 pkt_num;
    WORD8 q_num;
    UWORD8 state;
    UWORD8 id;	//padding  by zhao
    void* slot_info;	//point to this slot info, used by arm7.
};

typedef struct coex_trffc_chk{
    UWORD32 coex_wifi_state;
    ALARM_HANDLE_T *coex_traffic_check_alarm_handle;
    UWORD8   coex_traffic_level;
    UWORD32 coex_tx_pkg_cnt;
    UWORD32 coex_rx_pkg_cnt;
}COEX_TRFFC_CHK_T;

extern struct trout_tx_shareram tx_shareram[];
extern UWORD32 hw_txq_busy;
extern BOOL_T g_wifi_bt_coex;
extern UWORD32 *g_trout_self_cts_null_data_buf;
extern UWORD32 *g_trout_ps_null_data_buf;
extern COEX_TRFFC_CHK_T g_coex_trffc_chk;

extern UWORD8 get_coex_traffic_level(void);
extern UWORD32 get_coex_state(void);
extern void coex_state_switch(UWORD8 state);
extern void coex_wifi_tx_rx_pkg_sum(UWORD8 tx_or_rx, UWORD32 pkg_num);
extern void create_coex_traffic_check_alarms(void);

#define HW_TXQ_IS_BUSY(q_num) (hw_txq_busy & (1UL << ((q_num) & 0x7)))
#define HW_TXQ_ALL_IDLE()	((hw_txq_busy)? 0: 1)

INLINE BOOL_T is_sub_msdu_table_in_tx_dscr(UWORD32 *tx_dscr)
{
	UWORD32 sub_msdu_info = get_tx_dscr_submsdu_info(tx_dscr);
	
	if((sub_msdu_info > (UWORD32)tx_dscr)
				&& ((sub_msdu_info - (UWORD32)tx_dscr) < TX_DSCR_BUFF_SZ))
	{
		return BTRUE;
	}
	
	return BFALSE;
}

INLINE void host_notify_arm7_wifi_on(BOOL_T flag)
{
	UWORD32 value;

	TROUT_DBG4("host notify arm7 wifi status: %s\n", (flag ? "open" : "close"));
	value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3);
/*	if((value & BIT15) == (flag << BIT15))
          		return;
  */
	value &= ~(BIT15);
	value |= (flag << 15);
	host_write_trout_reg(value,  (UWORD32)rSYSREG_HOST2ARM_INFO3);
//	printk("%s: sys_info3=0x%x\n", 
//			__func__, host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3));
}


INLINE void host_notify_arm7_coex_ready(BOOL_T flag)
{
	UWORD32 value;

	TROUT_DBG4("host notify arm7 coexist ready: <%s>\n", (flag ? "ready" : "not ready"));

	value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3);
	if((value & BIT0) == flag)
		return;
	
	value &= ~(BIT0);
	value |= flag;
	host_write_trout_reg(value, (UWORD32)rSYSREG_HOST2ARM_INFO3);
}

INLINE void host_notify_arm7_wifi_reset(BOOL_T flag)
{
	UWORD32 value;

	TROUT_DBG4("host notify arm7 wifi reset %s\n",  (flag ? "begin" : "end"));
	value = host_read_trout_reg((UWORD32)rCOMM_ARM2HOST_INFO3);
	if((value & BIT1) == (flag << 1))
		return;
		
	value &= ~(BIT1);
	value |= (flag << 1);
	host_write_trout_reg(value, (UWORD32)rCOMM_ARM2HOST_INFO3);
}

INLINE void host_notify_arm7_connect_status(BOOL_T flag)
{
	UWORD32 value;
		
	TROUT_DBG4("host notify arm7 link status: <%s>\n", 
							(flag ? "connected" : "disconnected"));
	value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3);
	
	if(flag)
		value |= BIT1;
	else
		value &= (~BIT1);
	host_write_trout_reg(value, (UWORD32)rSYSREG_HOST2ARM_INFO3);

      /*  #ifdef IBSS_BSS_STATION_MODE
      if(BFALSE == flag){
          coex_state_switch(COEX_WIFI_IDLE);
      }
      #endif*/
}

#ifdef IBSS_BSS_STATION_MODE
#include "autorate_sta.h"

#if 0
INLINE void host_notify_arm7_connect_status(void)
{
	UWORD32 value;
		
	TROUT_DBG4("host notify arm7 linked status!\n");
	
	value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3) | BIT1;
	host_write_trout_reg(value, (UWORD32)rSYSREG_HOST2ARM_INFO3);
}

INLINE void host_notify_arm7_discon_status(void)
{
	UWORD32 value;

	if(!g_wifi_bt_coex)
		return;
	
	/* use rSYSREG_HOST2ARM_INFO3 reg bit1 notify arm7 connect status           */
	value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3) & (~BIT1);	/* disconnected */
	host_write_trout_reg(value, (UWORD32)rSYSREG_HOST2ARM_INFO3);

	/* use rCOMM_ARM2HOST_INFO3 reg bit1 notify arm7 that wifi is reseted */
	value = host_read_trout_reg(rCOMM_ARM2HOST_INFO3) | BIT1;
	host_write_trout_reg(value, (UWORD32)rCOMM_ARM2HOST_INFO3);
    host_write_trout_reg((UWORD32)0x1, (UWORD32)rSYSREG_GEN_ISR_2_ARM7);	/*interrupt CP*/

    TROUT_DBG4("host notify arm7 disconnect status!\n");
}
#endif

INLINE void host_notify_arm7_con_ap_mode(void)
{
	UWORD32 value;
	LINK_MODE_T mode;

	value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3);
	
	/* notify arm7 wifi is in B only mode or not, use bit2 */
	mode = cur_rate_mode_sta();
	if(mode == B_ONLY_RATE_STA)
	{
		value |= BIT2;
		TROUT_DBG4("note: cur connected AP is in B only mode!\n");
	}
	else
	{
		value &= (~BIT2);
		TROUT_DBG4("note: cur connected AP mode is %d!\n", mode);
	}
	host_write_trout_reg(value, (UWORD32)rSYSREG_HOST2ARM_INFO3);
      coex_state_switch(COEX_WIFI_8021X_DHCP);      
}

INLINE void host_notify_arm7_traffic_level(UWORD32 level){
    UWORD32 value;


    value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3);
    /* notify arm7 traffic level, use bit3~5 */
    if((value & (UWORD32)(BIT3|BIT4|BIT5)) == level << 3){
          return;
    }
    else{
          value &= ~(BIT3|BIT4|BIT5);
          value |= level << 3;
    }
    host_write_trout_reg(value, (UWORD32)rSYSREG_HOST2ARM_INFO3);
}

INLINE void host_notify_arm7_wifi_state(UWORD32 state){
    UWORD32 value;

    value = host_read_trout_reg((UWORD32)rSYSREG_HOST2ARM_INFO3);
    /* notify arm7 wifi state, use bit6~10 */
    if((value & (UWORD32)(BIT6|BIT7|BIT8|BIT9|BIT10)) == state << 6){
          return;
    }
    else{
          value &= ~(BIT6|BIT7|BIT8|BIT9|BIT10);
          value |= state << 6;
    }
    host_write_trout_reg(value, (UWORD32)rSYSREG_HOST2ARM_INFO3);
}


extern void coex_null_data_init(void);
extern void  exit_from_coexist_mode(void);
extern void wifi_bt_coexist_init(void);

#endif	/* IBSS_BSS_STATION_MODE */

extern void tx_shareram_manage_init(void);
extern struct trout_tx_shareram *tx_shareram_slot_alloc(UWORD32 q_num);
extern void tx_shareram_slot_free(UWORD32 q_num);
extern int tx_shareram_slot_valid(void);
extern int tx_shareram_slot_busy(UWORD8 slot);
extern int tx_shareram_slot_ready(UWORD8 slot);
extern int dma_vmalloc_data(void *ta, void *ha, UWORD32 l);


#endif	/* QMU_TX_H */
