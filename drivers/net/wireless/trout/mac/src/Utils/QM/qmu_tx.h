#ifndef QMU_TX_H
#define QMU_TX_H

#define TX_SHARERAM_IDLE  0
#define TX_SHARERAM_WAIT  1       //wait for data copy,and construction.
#define TX_SHARERAM_READY 2       //data is ready, waiting for hw tx.
#define TX_SHARERAM_BUSY  3       //data is transfering.
#define TX_SHARERAM_FULL  4       //data is ready and full, waiting for hw tx.

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


extern struct trout_tx_shareram tx_shareram[];
extern UWORD32 hw_txq_busy;
extern BOOL_T g_wifi_bt_coex;
extern UWORD32 *g_trout_self_cts_null_data_buf;
extern UWORD32 *g_trout_ps_null_data_buf;

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

#ifdef IBSS_BSS_STATION_MODE
#include "autorate_sta.h"

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
