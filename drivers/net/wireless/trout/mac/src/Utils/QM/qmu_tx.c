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
/*  File Name         : qmu_tx.c                                             */
/*                                                                           */
/*  Description       : This file contains all the transmit queue related    */
/*                      functions.                                           */
/*                                                                           */
/*  List of Functions : qmu_add_tx_packet                                    */
/*                      qmu_add_tx_packet_list                               */
/*                      qmu_sync_tx_packet                                   */
/*                      qmu_del_tx_packet                                    */
/*                      qmu_find_pending_dscr                                */
/*                      qmu_update_txq_chan_bw                               */
/*                                                                           */
/*  Issues / Problems : None                                                 */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* File Includes                                                             */
/*****************************************************************************/

#include "imem_if.h"
#include "qmu_if.h"
#include "spi_interface.h"
#include "trout_share_mem.h"
#include "management_11n.h"

#include "qmu_tx.h"
#include "runmode.h"

extern void tx_complete_isr(void);


#ifdef IBSS_BSS_STATION_MODE
extern BOOL_T g_wifi_bt_coex;
extern COEX_TRFFC_CHK_T g_coex_trffc_chk;
#endif

BOOL_T switch_flag = BTRUE;

UWORD32 trout_tx_dscr_head[60] = { 0 };	//add by chengwg.
volatile UWORD32 g_trout_last_tx_dscr = 0;

UWORD32 g_tsf_time = 0;		//add by chengwg.

extern UWORD32 g_tx_pkt_count;
extern UWORD32 g_tx_times;

#ifdef IBSS_BSS_STATION_MODE  
extern void auto_set_tx_rx_power(void);
extern void set_tx_pwr_pa_sel_from_self_adaptive(UWORD32 new_tx_pwr_11b, UWORD32 new_tx_pwr_11g, UWORD32 pa_value);
#endif

#ifdef TROUT_B2B_TEST_MODE
//UWORD8 trout_b2b_descr_pkt[160+1600];
UWORD32 trout_b2b_descr_pkt[512];
#endif

/*****************************************************************************/
/* Static Function Declarations                                              */
/*****************************************************************************/

#ifdef LOCALMEM_TX_DSCR
static UWORD8 *transfer_tx_dscr_local_to_shared(UWORD8 *local_dscr);
#endif /* LOCALMEM_TX_DSCR */

extern BOOL_T is_all_machw_q_null(void);


#if 0
//add by chengwg.
static int add_tx_dscr_entry(UWORD32 tx_dscr)
{
	int i = 0, count = 0;

	count = sizeof(trout_tx_dscr_head)/sizeof(trout_tx_dscr_head[0]);
	while((trout_tx_dscr_head[i] != 0) && (i < count))
		i++;

	if(i >= count)
	{
		printk("add trout tx dscr head failed!\n");
		hex_dump("trout dscr", (UWORD8 *)trout_tx_dscr_head, sizeof(trout_tx_dscr_head));	//debug info.
		BUG();
		return -1;
	}

	//printk("chwg: add trout_tx_dscr=0x%08x, idx=%d\n", tx_dscr, i);
	trout_tx_dscr_head[i] = tx_dscr;
	return 0;
}
#endif

//add by chengwg.
void flush_tx_dscr_head(void)
{
	memset(trout_tx_dscr_head, 0, sizeof(trout_tx_dscr_head));
}

//add by chengwg.
#if 0
static int is_trout_tx_data_empty(void)
{
	UWORD32 trout_dscr, tempVal = PENDING;
	int i = 0, count = 0;
	int ret = 0;
	//UWORD32 trout_descr = 0, num_tx_dscr = 0;

	TROUT_FUNC_ENTER;
	count = sizeof(trout_tx_dscr_head)/sizeof(trout_tx_dscr_head[0]);
	for(i=0; i<count; i++)
	{
		trout_dscr = trout_tx_dscr_head[i];
		if(trout_dscr == 0 || ((trout_dscr >= TX_MEM_END) || (trout_dscr < TX_MEM_BEGIN)))
		{
			//printk("tx_dscr=0x%08x, tx mem: 0x%x ~ 0x%x\n", trout_dscr, TX_MEM_BEGIN, TX_MEM_END);
			break;
		}
		host_read_trout_ram(&tempVal, (UWORD32 *)trout_dscr + WORD_OFFSET_0,  4);
		tempVal = (tempVal >> 29) & 0x3;
		if(tempVal != INVALID)
		{
            if(tempVal != PENDING)
            {
                tx_pkt_process();
                continue;
            }
            else
            {
                TROUT_DBG4("unbelievable: tx busy at 0x%x, value = %u!\n", trout_dscr, tempVal);
				ret = -1;
            }
		    
			break;
		}
	}

	TROUT_FUNC_EXIT;
	return ret;
}

static BOOL_T is_trout_tx_mem_empty(void)
{
	UWORD8 qnum = g_q_handle.tx_handle.tx_curr_qnum;

	TROUT_FUNC_ENTER;
/*
	if(!g_trout_last_tx_dscr)
	{
		TROUT_FUNC_EXIT;
		return BTRUE;
	}
*/
	if(g_q_handle.tx_handle.tx_header[qnum].handle_pkt != 0)	//chwg test.
	{
		TROUT_DBG4("warning: handle_pkt = %d\n", g_q_handle.tx_handle.tx_header[qnum].handle_pkt);
		return BFALSE;
	}
	
	TROUT_FUNC_EXIT;
	return BTRUE;
}


#if 0
int qmu_cpy_to_trout(UWORD32 q_num)
{
	WORD32 ret = -1;
    UWORD32 trout_descr, trout_next_descr, trout_prev_descr, trout_buf, descr_len, data_len;
    UWORD32 next_descr, sub_msdu_info, temp32[3], trout_sub_msdu_table;
    UWORD32 *descr, *buff_ptr;
    UWORD32 i = 0, sub_buff_len, padding, nr;
    buffer_desc_t buff_list;
    UWORD8 sub_msdu_table[MAX_SUB_MSDU_TABLE_ENTRIES * SUB_MSDU_ENTRY_LEN];
    UWORD8 *sub_buff_ptr;
#ifdef TROUT_TRACE_DBG
	UWORD8 count = 0;
	UWORD32 send_len = 0;
#endif
	
	TROUT_FUNC_ENTER;

	descr = g_q_handle.tx_handle.tx_header[q_num].element_to_load;
	if(descr == NULL)
	{
		TROUT_FUNC_EXIT;
		return ret;
	}
	
	if(switch_flag)
	{
		if(is_trout_tx_data_empty() != 0)
		{
			TROUT_TX_DBG4("tx share mem busy!\n");
#ifdef DEBUG_MODE
			g_mac_stats.tx_busy_count++;
#endif
			TROUT_FUNC_EXIT;
			return ret;
		}
		
		flush_tx_dscr_head();
	}
	
    trout_descr = TX_MEM_BEGIN;
    trout_prev_descr = trout_descr;
	
	while(descr != NULL)
    {
        sub_msdu_info = get_tx_dscr_submsdu_info(descr);
        nr = get_tx_dscr_num_submsdu(descr);
		next_descr = get_tx_dscr_next_addr(descr);
		buff_ptr = (UWORD32 *)get_tx_dscr_buffer_addr(descr);
		descr_len = TX_DSCR_LEN + nr * SUB_MSDU_ENTRY_LEN;
		trout_buf = trout_descr + descr_len;
        data_len = 0;

        //get the pkts total data length.
        for(i = 0; i<nr; i++)
		{
			get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
            padding = 0;
            buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
            if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
            {
                padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
                buff_list.buff_hdl += padding;
            }
            	            
            data_len += buff_list.data_offset + buff_list.data_length + padding;
		}

		trout_next_descr = (trout_buf + data_len + 3U) & ~3U;
		if(trout_next_descr > TX_MEM_END)
    	{
			if(trout_prev_descr != trout_descr)  //prev->nextdescr = 0.
            {
            	temp32[2] = 0;
                host_write_trout_ram((UWORD32 *)trout_prev_descr + TX_DSCR_NEXT_ADDR_WORD_OFFSET,
                                      &temp32[2], 4);
            }
            break;
    	}

        trout_sub_msdu_table = trout_descr + TX_DSCR_LEN;   //get trout sub-msdu table address.
        set_tx_dscr_buffer_addr(descr, trout_buf);
		set_tx_dscr_host_dscr_addr(descr, (UWORD32)descr);
    	if(next_descr != 0)
        	set_tx_dscr_next_addr(descr, trout_next_descr);
        else
        	set_tx_dscr_next_addr(descr, 0);

        /* before change host sub msdu table's buffer address, backup the table */
        /* first, beacuse we need recover the value after write to trout        */
        memcpy((void *)sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
        	
        data_len = 0;
		if(is_sub_msdu_table_in_tx_dscr(descr))
		{
			if(nr > 1)	/* in this case, the sub-msdu number should be 1 */
			{
				TROUT_DBG3("Warning: sub-msdu table in tx dscr, but sub-msdu num(%d) is not match!\n", nr);
				TROUT_FUNC_EXIT;
				return ret;
			}

            get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);
			data_len = buff_list.data_offset + buff_list.data_length;
			buff_list.buff_hdl = (UWORD8 *)trout_buf;	//updata sub-msdu0 buffer address.

			set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);

			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);

            /* Set the current timestamp */
			update_tx_dscr_tsf_ts(descr);  //dumy add 0824
                        
        	//copy descr and buf data to trout
	        host_write_trout_ram((void *)trout_descr, descr, descr_len);    //descr+sub-msdu table.
	        host_write_trout_ram((void *)trout_buf, buff_ptr, data_len);

#ifdef TROUT_TRACE_DBG
			send_len += (data_len + descr_len);
#endif
		}
		else
		{
            /* copy nr sub msdu data to trout */
			for(i = 0; i<nr; i++)
	        {
				get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
	            sub_buff_ptr = buff_list.buff_hdl;
	            sub_buff_len = buff_list.data_offset + buff_list.data_length;
	            
	            /* change host sub MSDU addr to trout sub MSDU addr */
                padding = 0;
                buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
                if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
                {
                    padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
                    buff_list.buff_hdl += padding;
                }

	            set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);

				/* copy sub msdu data to trout */
				host_write_trout_ram((void *)buff_list.buff_hdl, (void *)sub_buff_ptr, sub_buff_len);
	            data_len += sub_buff_len + padding;
#ifdef TROUT_TRACE_DBG
				send_len += sub_buff_len;
#endif	            
	        }

            /* copy host sub msdu table to trout */
			host_write_trout_ram((void *)trout_sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
#ifdef TROUT_TRACE_DBG
			send_len += nr * SUB_MSDU_ENTRY_LEN;
#endif
			/* Set the current timestamp */
			update_tx_dscr_tsf_ts(descr);

			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);
			/* copy host dscr to trout */
	        ret = host_write_trout_ram((void *)trout_descr, descr, TX_DSCR_LEN);
#ifdef TROUT_TRACE_DBG
			send_len += TX_DSCR_LEN;
#endif	        
		}

        /* recover host sub msdu table info */
        memcpy((void *)sub_msdu_info, (void *)sub_msdu_table, nr * SUB_MSDU_ENTRY_LEN);

        //restore trout host value.
        set_tx_dscr_buffer_addr(descr, (UWORD32)buff_ptr);
        set_tx_dscr_next_addr(descr, next_descr);
        set_tx_dscr_submsdu_info(descr, sub_msdu_info);
		//printk("chwg: host_dscr=0x%p, index=%d, nr=%d, is_amsdu=%d\n", descr, index, nr, is_amsdu);

		if(switch_flag)
		{
			add_tx_dscr_entry(trout_descr);
		}
		
#ifdef TROUT_TRACE_DBG		
		count++;
#endif

#ifdef TROUT_B2B_TEST_MODE
#if 0
        if(get_tx_dscr_frame_len(descr) >= 500)
        {
            //host_read_trout_ram(trout_b2b_descr_pkt, trout_descr, sizeof(trout_b2b_descr_pkt));
            printk("back up pkt: trout_desc:0x%x.\n", trout_descr);
            host_read_trout_ram(&trout_b2b_descr_pkt[5], trout_descr, 2000);
            trout_b2b_descr_pkt[0] = trout_descr;
            trout_b2b_descr_pkt[1] = q_num;
            trout_b2b_descr_pkt[5+35] = 0;
        }
#endif
#endif
		trout_prev_descr = trout_descr;
        trout_descr = trout_next_descr;
        descr = (UWORD32 *)next_descr;
	}

	g_q_handle.tx_handle.tx_header[q_num].element_to_load = descr;
	g_q_handle.tx_handle.tx_header[q_num].trout_head = TX_MEM_BEGIN;
	g_q_handle.tx_handle.tx_curr_qnum = q_num;

	TROUT_TX_DBG5("%s<%d>: pkt=%d, len=%d\n", __FUNCTION__, q_num, count, send_len);

	//write trout 1st descr addr to txq register.
    if(trout_prev_descr != trout_descr)  //prev->nextdescr = 0.
    {
        set_machw_q_ptr(q_num, (UWORD32 *)TX_MEM_BEGIN);
    } 

	TROUT_FUNC_EXIT;
    return 0;
}
#else
int qmu_cpy_to_trout(UWORD32 q_num, UWORD8 caller)
{
	WORD32 ret = -1;
    UWORD32 trout_descr, trout_next_descr, trout_prev_descr, trout_buf, descr_len, data_len;
    UWORD32 next_descr, sub_msdu_info, temp32[3], trout_sub_msdu_table;
    UWORD32 *descr, *buff_ptr;
    UWORD32 i = 0, sub_buff_len, padding, nr;
    buffer_desc_t buff_list;
    UWORD8 sub_msdu_table[MAX_SUB_MSDU_TABLE_ENTRIES * SUB_MSDU_ENTRY_LEN];
    UWORD8 *sub_buff_ptr;
#ifdef TX_PKT_USE_DMA
    UWORD32 host_prev_descr = 0;
	UWORD8 *sbuf = NULL, *pcur = NULL, *pbak = NULL;
#endif	/* TX_PKT_USE_DMA */
	UWORD32 count = 0, tsf_lo = 0;
	struct timespec time;
	static UWORD32 stime = 0;
	UWORD32 etime;
	UWORD32 tx_mem_begin = g_q_handle.tx_handle.tx_mem_start;

	TROUT_FUNC_ENTER;
	
	descr = g_q_handle.tx_handle.tx_header[q_num].element_to_load;
	if(descr == NULL)
	{
		TROUT_FUNC_EXIT;
		return ret;
	}
#ifdef TX_PKT_USE_DMA
	if(g_tx_dma_handle.tx_dma_buf == NULL)
	{
		TROUT_TX_DBG1("tx dma buffer not initialized!\n");
		TROUT_FUNC_EXIT;
		return ret;
	}

	sbuf = (UWORD8 *)WORD_ALIGN((UWORD32)(g_tx_dma_handle.tx_dma_buf)) + SPI_SDIO_WRITE_RAM_CMD_WIDTH;
	pcur = sbuf;
#endif /* TX_PKT_USE_DMA */

#if 0
	if(switch_flag)
	{
		if(is_trout_tx_data_empty(call_flag) != 0)
		{
			TROUT_TX_DBG5("tx share mem busy!\n");
			return ret;
		}
		
		flush_tx_dscr_head();
	}
#else
	if(!is_trout_tx_mem_empty())
	{
		TROUT_TX_DBG5("tx busy!\n");
		TROUT_FUNC_EXIT
		return ret;
	}
	
	g_trout_last_tx_dscr = 0;
#endif

    //trout_descr = TX_MEM_BEGIN;
    trout_descr = tx_mem_begin;
    trout_prev_descr = trout_descr;

#ifdef TX_PKT_USE_DMA
	mutex_lock(&g_tx_dma_handle.tx_dma_lock);
#endif	/* TX_PKT_USE_DMA */

	while(descr != NULL)
    {
        sub_msdu_info = get_tx_dscr_submsdu_info(descr);
        nr = get_tx_dscr_num_submsdu(descr);
		next_descr = get_tx_dscr_next_addr(descr);
		buff_ptr = (UWORD32 *)get_tx_dscr_buffer_addr(descr);
		descr_len = TX_DSCR_LEN + nr * SUB_MSDU_ENTRY_LEN;
		trout_buf = trout_descr + descr_len;
        data_len = 0;

        //get the pkts total data length.
        for(i = 0; i<nr; i++)
		{
			get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
            padding = 0;
            buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
            if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
            {
                padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
                buff_list.buff_hdl += padding;
            }
           	//printk("%d: data_offset=0x%x, data_len=%d\n", i, buff_list.data_offset, buff_list.data_length); 
            data_len += buff_list.data_offset + buff_list.data_length + padding;
		}

		trout_next_descr = (trout_buf + data_len + 3U) & ~3U;
		
		//if(trout_next_descr > TX_MEM_END)
		if(trout_next_descr > tx_mem_begin + g_q_handle.tx_handle.tx_mem_size)
    	{
			if(trout_prev_descr != trout_descr)  //prev->nextdescr = 0.
            {
#ifdef TX_PKT_USE_DMA
				BUG_ON(!host_prev_descr);
				set_tx_dscr_next_addr((UWORD32 *)host_prev_descr, 0);   //notice!!!    
            	
#else                
                temp32[2] = 0;
                host_write_trout_ram((UWORD32 *)trout_prev_descr + TX_DSCR_NEXT_ADDR_WORD_OFFSET,
                                      &temp32[2], 4);
#endif	/* TX_PKT_USE_DMA */                
            }
            break;
    	}

        trout_sub_msdu_table = trout_descr + TX_DSCR_LEN;   //get trout sub-msdu table address.
        set_tx_dscr_buffer_addr(descr, trout_buf);
		set_tx_dscr_host_dscr_addr(descr, (UWORD32)descr);
    	if(next_descr != 0)
        	set_tx_dscr_next_addr(descr, trout_next_descr);
        else
        	set_tx_dscr_next_addr(descr, 0);
			
        /* before change host sub msdu table's buffer address, backup the table */
        /* first, beacuse we need recover the value after write to trout        */
        memcpy((void *)sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
        	
        data_len = 0;
		if(is_sub_msdu_table_in_tx_dscr(descr))
		{
			if(nr > 1)	/* in this case, the sub-msdu number should be 1 */
			{
				TROUT_DBG3("Warning: sub-msdu table in tx dscr, but sub-msdu num(%d) is not match!\n", nr);
#ifdef TX_PKT_USE_DMA
				mutex_unlock(&g_tx_dma_handle.tx_dma_lock);
#endif	/* TX_PKT_USE_DMA */	
				TROUT_FUNC_EXIT;
				return ret;
			}

            get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);
			data_len = buff_list.data_offset + buff_list.data_length;
			buff_list.buff_hdl = (UWORD8 *)trout_buf;	//updata sub-msdu0 buffer address.

			set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);

			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);
            
        	//copy descr and buf data to trout
        	if(!g_tsf_time)
        	{
        		getnstimeofday(&time);
				stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
        		g_tsf_time = get_machw_tsf_timer_lo();
        	}
        	else
        	{
        		getnstimeofday(&time);
				etime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
				if(etime - stime >= 10000000)	//10s
				{
					g_tsf_time = get_machw_tsf_timer_lo();	//reload tsf timer from reigster.
					getnstimeofday(&time);
					stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
					etime = stime;
				}
				g_tsf_time += etime - stime;
				stime = etime;	//update stime.
        	}
        	tsf_lo = g_tsf_time;

        	/* update the current timestamp */
#ifdef TX_PKT_USE_DMA
/*			if(count == 0)
			{
				tsf_lo = get_machw_tsf_timer_lo();	//read tsf time 1 time for reduce IO!
			}*/ 	
			simp_update_tx_dscr_tsf_ts(descr, tsf_lo);

			//pcur = sbuf + (trout_descr - TX_MEM_BEGIN);
			pcur = sbuf + (trout_descr - tx_mem_begin);
	        memcpy((void *)pcur, descr, descr_len);
	        host_prev_descr = (UWORD32)pcur;	//bakup!!!
	        pcur += descr_len;

			//pcur = sbuf + (trout_buf - TX_MEM_BEGIN);
	        pcur = sbuf + (trout_buf - tx_mem_begin);
	        memcpy((void *)pcur, buff_ptr, data_len);
	        pcur += data_len;
#else
			/* update the current timestamp */
            update_tx_dscr_tsf_ts(descr);
            
			host_write_trout_ram((void *)trout_descr, descr, descr_len);    //descr+sub-msdu table.
			host_write_trout_ram((void *)trout_buf, buff_ptr, data_len);
#endif	/* TX_PKT_USE_DMA */
		}
		else
		{
            /* copy nr sub msdu data to trout */
			for(i=0; i<nr; i++)
	        {
				get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
	            sub_buff_ptr = buff_list.buff_hdl;
	            sub_buff_len = buff_list.data_offset + buff_list.data_length;
	            
	            /* change host sub MSDU addr to trout sub MSDU addr */
                padding = 0;
                buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
                if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
                {
                    padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
                    buff_list.buff_hdl += padding;
                }

	            set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);

				/* copy sub msdu data to trout */
#ifdef TX_PKT_USE_DMA
				//pcur = sbuf + ((UWORD32)(buff_list.buff_hdl) - TX_MEM_BEGIN);
				pcur = sbuf + ((UWORD32)(buff_list.buff_hdl) - tx_mem_begin);
				memcpy((void *)pcur, (void *)sub_buff_ptr, sub_buff_len);
				pcur += sub_buff_len;
				pbak = pcur;	//record the last length!!!
#else
				host_write_trout_ram((void *)buff_list.buff_hdl, (void *)sub_buff_ptr, sub_buff_len);
#endif	/* TX_PKT_USE_DMA */
	            data_len += sub_buff_len + padding;
	        }

            /* copy host sub msdu table to trout */
#ifdef TX_PKT_USE_DMA
			//pcur = sbuf + (trout_sub_msdu_table - TX_MEM_BEGIN);
			pcur = sbuf + (trout_sub_msdu_table - tx_mem_begin);
			memcpy((void *)pcur, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
			pcur += nr * SUB_MSDU_ENTRY_LEN;

			/* Set the current timestamp */
			if(!g_tsf_time)
        	{
        		getnstimeofday(&time);
				stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
        		g_tsf_time = get_machw_tsf_timer_lo();
        	}
        	else
        	{
        		getnstimeofday(&time);
				etime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
				if(etime - stime >= 10000000)	//10s
				{
					g_tsf_time = get_machw_tsf_timer_lo();	//reload tsf timer from reigster.
					getnstimeofday(&time);
					stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
					etime = stime;
				}
				g_tsf_time += etime - stime;
				stime = etime;	//update stime.
        	}
        	tsf_lo = g_tsf_time;
        	
/*			if(count == 0)
			{
				tsf_lo = get_machw_tsf_timer_lo();	//read tsf time 1 time for reduce IO!
			}*/
			
			simp_update_tx_dscr_tsf_ts(descr, tsf_lo);
			
#else
			host_write_trout_ram((void *)trout_sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);

			/* Set the current timestamp */
			update_tx_dscr_tsf_ts(descr);
#endif	/* TX_PKT_USE_DMA */
			
			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);
			/* copy host dscr to trout */
#ifdef TX_PKT_USE_DMA
			//pcur = sbuf + (trout_descr - TX_MEM_BEGIN);
			pcur = sbuf + (trout_descr - tx_mem_begin);
	        memcpy((void *)pcur, descr, TX_DSCR_LEN);
	        host_prev_descr = (UWORD32)pcur;	//bakup!!!
	        pcur += TX_DSCR_LEN;
	        
	        pcur = pbak;	//get the last length!!!
#else
			ret = host_write_trout_ram((void *)trout_descr, descr, TX_DSCR_LEN);
#endif			
		}

        /* recover host sub msdu table info */
        memcpy((void *)sub_msdu_info, (void *)sub_msdu_table, nr * SUB_MSDU_ENTRY_LEN);

        //restore trout host value.
        set_tx_dscr_buffer_addr(descr, (UWORD32)buff_ptr);
        set_tx_dscr_next_addr(descr, next_descr);
        set_tx_dscr_submsdu_info(descr, sub_msdu_info);
		
		//printk("chwg: host_dscr=0x%p, nr=%d\n", descr, nr);
#if 0		
		if(switch_flag)
		{
			add_tx_dscr_entry(trout_descr);
		}
#endif		
		count++;
		
		trout_prev_descr = trout_descr;
        trout_descr = trout_next_descr;
        descr = (UWORD32 *)next_descr;
	}
	
#if 1
	g_trout_last_tx_dscr = trout_prev_descr;	//save last trout tx dscr.
#endif
	
#ifdef TX_PKT_USE_DMA
	//start dma write to trout.
	//host_write_trout_ram((void *)TX_MEM_BEGIN, sbuf, (UWORD32)(pcur-sbuf));
	host_write_trout_ram((void *)tx_mem_begin, sbuf, (UWORD32)(pcur-sbuf));
	TROUT_TX_DBG5("%s<%d>: count=%u, len=%u\n", __FUNCTION__, q_num, count, (UWORD32)(pcur-sbuf));	
#endif

	g_q_handle.tx_handle.tx_header[q_num].element_to_load = descr;
	//g_q_handle.tx_handle.tx_header[q_num].trout_head = TX_MEM_BEGIN;
	g_q_handle.tx_handle.tx_header[q_num].trout_head = tx_mem_begin;
	g_q_handle.tx_handle.tx_curr_qnum = q_num;

	if(trout_prev_descr != trout_descr)
		g_q_handle.tx_handle.tx_header[q_num].handle_pkt = count;	//chenwg add.
	else
		g_q_handle.tx_handle.tx_header[q_num].handle_pkt = 0;
		
	g_tx_pkt_count += count;
	g_tx_times++;
	
	//write trout 1st descr addr to txq register.
    if(trout_prev_descr != trout_descr)  //prev->nextdescr = 0.
    {
#if 0
        set_machw_q_ptr(q_num, (UWORD32 *)TX_MEM_BEGIN);
#else/*
		while(host_read_trout_reg((UWORD32)rMAC_PA_STAT) & g_qif_table[q_num].lock_bit)
		{;}*/
		host_write_trout_reg(convert_to_le(virt_to_phy_addr(tx_mem_begin)), 
					      					(UWORD32)(g_qif_table[q_num].addr));
#endif
    } 

    TX_INT_DBG("cpy_to_trout(caller: %s): pkt=%u, q_num=%d, last_dscr: 0x%x\n", 
    			((caller == TX_ISR_CALL) ? "isr" : "event"), count, q_num, g_trout_last_tx_dscr);

#ifdef TX_PKT_USE_DMA
	mutex_unlock(&g_tx_dma_handle.tx_dma_lock);
#endif

	TROUT_FUNC_EXIT;
    return 0;
}
#endif
#else
struct trout_tx_shareram tx_shareram[TX_SHARERAM_SLOTS];
UWORD32 hw_txq_busy = 0;

UWORD32 slot_pkt_nr[2] = {0, 0};
UWORD32 slot_mount_nr[2] = {0, 0};


void tx_shareram_manage_init(void)
{
    int i;
    struct trout_tx_shareram *ptr;
    UWORD32 tx_mem_begin = g_q_handle.tx_handle.tx_mem_start;
    UWORD32 tx_mem_size = g_q_handle.tx_handle.tx_mem_size;
    UWORD32 tx_mem_end = g_q_handle.tx_handle.tx_mem_end;
	UWORD32 tmp32[3];

	tmp32[2] = 0;
    for(i=0; i<TX_SHARERAM_SLOTS; i++)
    {
       ptr = &tx_shareram[i];
       ptr->begin = ((tx_mem_begin + i * (tx_mem_size / TX_SHARERAM_SLOTS)) + 3) & ~3UL;
       ptr->end = ptr->begin + (tx_mem_size/TX_SHARERAM_SLOTS) - 1;
       ptr->curr = ptr->begin;
       ptr->pkt_num = 0;
       ptr->q_num = -1;
       ptr->state = TX_SHARERAM_IDLE;
       ptr->id = i;
       ptr->slot_info = (void *)(tx_mem_end + i * sizeof(UWORD32));

#ifdef IBSS_BSS_STATION_MODE       
       host_write_trout_ram(ptr->slot_info, (void *)(&tmp32[2]), sizeof(UWORD32));	//init slot info region.
#endif	/* IBSS_BSS_STATION_MODE */

       printk("[slot%d] begin:0x%x, end:0x%x, slot_info: 0x%p\n",
       							ptr->id, ptr->begin, ptr->end, ptr->slot_info);
    }
    
    hw_txq_busy = 0;
}

struct trout_tx_shareram *tx_shareram_slot_alloc(UWORD32 q_num)
{
    int i;
	int smart_type = 0;
    struct trout_tx_shareram *ptr;

    for(i=0; i<TX_SHARERAM_SLOTS; i++)
    {
        ptr = &tx_shareram[i];
        smart_type = critical_section_smart_start(1,1);
        //critical_section_start1();
        
        if(ptr->state == TX_SHARERAM_IDLE)
        {
            ptr->state = TX_SHARERAM_WAIT;
            
            ptr->q_num = q_num;
            ptr->curr = ptr->begin;
            ptr->pkt_num = 0;
            critical_section_smart_end(smart_type);
           // critical_section_end1();
            return ptr;
        }
		
        critical_section_smart_end(smart_type);
        //critical_section_end1();
    }

    return NULL;
}

void tx_shareram_slot_free(UWORD32 q_num)
{
    int i;
    struct trout_tx_shareram *ptr;
	int smart_type = 0;

    for(i=0; i<TX_SHARERAM_SLOTS; i++)
    {
        ptr = &tx_shareram[i];
		smart_type = critical_section_smart_start(1,1);
        //critical_section_start1();

        if((ptr->q_num == q_num) && ((ptr->state == TX_SHARERAM_BUSY) 
        				|| (ptr->state == TX_SHARERAM_WAIT)))	//fix bug by chengwg.
        {
            ptr->state = TX_SHARERAM_IDLE;
            
            ptr->curr = ptr->begin;
            ptr->q_num = -1;
            ptr->pkt_num = 0;
            hw_txq_busy &= ~(1UL << ((q_num) & 0x7));
		critical_section_smart_end(smart_type);
            //critical_section_end1();
            break;
        }
        critical_section_smart_end(smart_type);
        //critical_section_end1();
    }
}


int tx_shareram_slot_valid(void)
{
    int i;
    struct trout_tx_shareram *ptr;

    for(i=0; i<TX_SHARERAM_SLOTS; i++)
    {
        ptr = &tx_shareram[i];
        if(ptr->state == TX_SHARERAM_IDLE)
        {
            return 1;
        }
    }

    return 0;
}

#if 0
void modify_tsf(struct trout_tx_shareram *txp, unsigned int nr)
{
	UWORD32 *tp, *tt, *tw;
	UWORD32 tsf_lo = 0, cnt = 0;
	UWORD32	nv[36];

	tp = (UWORD32 *)txp->begin;
		//printk("*******TX DSCR*******");
	while(tp){
		if((UWORD32)tp > txp->end || tp < txp->begin)
			break;
		memset((char *)&nv[0], 0, sizeof(UWORD32) * 36);
		host_read_trout_ram(&nv[0], &tp[0], sizeof(UWORD32) * 36);

		tsf_lo = get_machw_tsf_timer_lo();
		tsf_lo = (tsf_lo >> 10) & 0xffff;
		nv[3] &= 0xffff0000;
		nv[3] |= tsf_lo;
		host_write_trout_ram((void *)&tp[3], (void *)&nv[3], sizeof(UWORD32));
		tt = tp;
		tp = (UWORD32 *)nv[4];
		if(tt == tp || (UWORD32)tt & 0x3){
			printk("NWXT TXDSCR is SELF\n");
			BUG_ON(1);
		}
#if 0
		/* other checks */
		tw = nv[30];
		printk("@@@:%d-%08X,%08X,%08X,%08X,NR=%d\n", cnt, tt, tp, nv[35], tw, ((nv[32] & 0xff00) >> 8));
#endif
		cnt++;
	}
	if(cnt != nr)
		printk("@@@:BUG! the pkt nr is not equal to Trout\n");
}
#else
//Hugh: optimize for tx throughput.2013-10-12
void modify_tsf(struct trout_tx_shareram *txp, unsigned int nr)
{
#if 0
	UWORD32 *tp, *tt, *tw;
	UWORD32 tsf_lo = 0, cnt = 0;
	UWORD32	nv[36];

	/*
	for(cnt=0; cnt < txp->pkt_num; cnt++)
	{
		printk("0x%x->", txp->desc_addr[cnt]);
	}

	printk("\n");*/
	
	tsf_lo = get_machw_tsf_timer_lo();
	tsf_lo = (tsf_lo >> 10) & 0xffff;
	tp = (UWORD32 *)txp->begin;
		//printk("*******TX DSCR*******");

	cnt = 0;
	while(tp){
		if((UWORD32)tp > txp->end || tp < txp->begin)
			break;
		host_read_trout_ram(&nv[0], &tp[0], 20);
		nv[3] &= 0xffff0000;
		nv[3] |= tsf_lo;
		host_write_trout_ram((void *)&tp[3], (void *)&nv[3], sizeof(UWORD32));
		tt = tp;
		tp = (UWORD32 *)nv[4];
		//printk("tt=0x%x, tp=0x%x\n", tt, tp);
		if(tt == tp || (UWORD32)tt & 0x3){
			printk("NWXT TXDSCR is SELF\n");
			BUG_ON(1);
		}
		cnt++;
	}
	if(cnt != nr)
		printk("@@@:BUG! the pkt nr is not equal to Trout: act=%d, need=%d\n", cnt, nr);
#else
	UWORD32 tsf_lo, cnt;
	UWORD32 i, j;
	UWORD32 desc_word3_addr[TX_MAX_PKT_PER_SLOT];	
	UWORD32 desc_word3_value[BLOCK_WRITE_CAP] = {0};
	UWORD32 desc_word3_write_len[BLOCK_WRITE_CAP] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
	
	/*UWORD32 clear_len[BLOCK_WRITE_CAP] = 
							{4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};*/
	//printk("m_tsf: nr=%d, txp->nr=%d\n", nr, txp->pkt_num);
        if(txp->pkt_num > TX_MAX_PKT_PER_SLOT)
	{
               printk("pkt_num is error: %d\n", txp->pkt_num);
               return;
        }


	if(nr > txp->pkt_num)
	{
                nr = txp->pkt_num;
	}

	for(i=0; i<txp->pkt_num; i++)
	{
		desc_word3_addr[i] = txp->desc_addr[i] + 12;		//point to tsf field in tx desc.
	}

	j = 0;
	tsf_lo = get_machw_tsf_timer_lo();
	tsf_lo = (tsf_lo >> 10) & 0xffff;
	while(nr > 0)
	{
		cnt = nr < BLOCK_WRITE_CAP ? nr : BLOCK_WRITE_CAP;
		
		host_block_read_trout_ram(desc_word3_value, &desc_word3_addr[j], desc_word3_write_len, cnt);
		for(i=0; i<cnt; i++)
		{
			desc_word3_value[i] = (desc_word3_value[i] & 0xffff0000) | tsf_lo;  //update tsf.
		}
		host_block_write_trout_ram(&desc_word3_addr[j], (UWORD32)desc_word3_value, desc_word3_write_len, cnt);
		nr -= cnt;
		j += cnt;
	}

#endif	
}
#endif

/* select a slot to be transmitted, current just support
 * 2 slots policy, by zhao
 * note: this function must be called with txq_lock holed
 */
struct trout_tx_shareram *select_next_slot(void)
{
        int i, vr = 0, t;
        struct trout_tx_shareram *ptr;

        for(i = 0; i < TX_SHARERAM_SLOTS; i++){
                ptr = &tx_shareram[i];
                if(ptr->state == TX_SHARERAM_READY)
                        vr |= (1 << i);
        }
		if(!vr)
			return NULL;

        if(0x3 == vr){
                /* take turns  by zhao */
                t = g_q_handle.tx_handle.cur_slot ^ 0x1;
        }else{
                t = (vr == 1)? 0: 1;
        }
        return &tx_shareram[t];
}


#ifdef TROUT_TRACE_DBG
void tx_shareram_slot_stat(void)
{
	int i;
	//UWORD8 buf[152];
	UWORD32 tmp32;
	struct trout_tx_shareram *ptr;

	for(i = 0; i < TX_SHARERAM_SLOTS; i++)
	{
		ptr = &tx_shareram[i];
		printk("tx slot[%d] state: %d\n", i, ptr->state);
		if((ptr->state == TX_SHARERAM_READY) || (ptr->state == TX_SHARERAM_BUSY))
		{
			printk("begin=0x%x, end=0x%x\n", ptr->begin, ptr->end);
			printk("curr=0x%x, pkt_num=%u, q_num=%d\n", ptr->curr, ptr->pkt_num, ptr->q_num);

		}
		host_read_trout_ram((void *)(&tmp32), ptr->slot_info, sizeof(UWORD32));	//dbg.
        printk("slot_info is: 0x%x\n", tmp32);
	}
	printk("hw_tx_busy: 0x%x\n", (hw_txq_busy & 0x7));
}
#endif

#if 0
void print_slot_info(struct trout_tx_shareram *txp)	//debug.
{
	UWORD32 *tp, *tt, *tw;
	UWORD32	nv[36];
	UWORD32 count = 0;
	UWORD8 status;
				
	tp = (UWORD32 *)txp->begin;
	while(tp)
	{
		if((UWORD32)tp > txp->end || tp < txp->begin)
			break;
		memset((char *)&nv[0], 0, sizeof(UWORD32) * 36);
		host_read_trout_ram((void *)nv, &tp[0], sizeof(UWORD32) * 36);	//debug.

		status = (nv[0] >> 29) & 0x3;
		count = nv[24];
		
		printk("0x%x(%d:0x%x)->", tp, status, count);
		tt = tp;
		tp = (UWORD32 *)nv[4];
		if(tt == tp || (UWORD32)tt & 0x3)
		{
			printk("NWXT TXDSCR is SELF\n");
		}
	}
	printk("0\n");
}
#endif

/* add a mutex to make sure no race condiction to start TX, by zhao */
int tx_shareram_wq_mount(void)
{
    int i, num = 0;
    struct trout_tx_shareram *ptr = NULL;

#if 1
    mutex_lock(&g_q_handle.tx_handle.hwtx_mutex);

	ptr = select_next_slot();
	if(ptr && !hw_txq_busy)
	{
		UWORD32 tmp32[3];
		tmp32[2] = ptr->begin;
		
		#ifdef IBSS_BSS_STATION_MODE
		//chenq add for auto set tx rx power 2013-07-29
		#ifdef TROUT2_WIFI_IC
			//set_tx_pwr_from_self_adaptive();
			//auto_set_tx_rx_power();
		#endif
		#endif
		//chenq add end
		hw_txq_busy |= 1UL << (ptr->q_num & 0x7);
		ptr->state = TX_SHARERAM_BUSY;
#ifdef IBSS_BSS_STATION_MODE	
		if(!g_wifi_bt_coex)
#endif	/* IBSS_BSS_STATION_MODE */
		{
			modify_tsf(ptr, ptr->pkt_num);
			host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)ptr->begin)), 
									(UWORD32)(g_qif_table[ptr->q_num].addr));
			//printk("mtpkt%d:%d\n", ptr->id, ptr->pkt_num);

		}
#ifdef IBSS_BSS_STATION_MODE	
		/* This info is used for WiFi & BT coexist */	
		host_write_trout_ram(tx_shareram[0].slot_info, (void *)&tmp32[2], sizeof(UWORD32));
#endif	/* IBSS_BSS_STATION_MODE */

		slot_mount_nr[ptr->id]++;
		g_q_handle.tx_handle.tx_curr_qnum = ptr->q_num;
		g_q_handle.tx_handle.cur_slot = ((unsigned char *)ptr - (unsigned char *)&tx_shareram[0]) / sizeof(struct trout_tx_shareram);

		//printk("mount slot%d, st=%d, pkt=%d, hwbusy=%d\n", ptr->id, ptr->state, ptr->pkt_num, hw_txq_busy);
	}
    mutex_unlock(&g_q_handle.tx_handle.hwtx_mutex);

#else
    for(i=0; i<TX_SHARERAM_SLOTS; i++)
    {
        ptr = &tx_shareram[i];
        mutex_lock(&g_q_handle.tx_handle.hwtx_mutex);
	//critical_section_start1();
        if(ptr->state == TX_SHARERAM_IDLE)
        {
            num++;
        	mutex_unlock(&g_q_handle.tx_handle.hwtx_mutex);
            //critical_section_end1();
            continue;
        }
        
		/* by zhao */
        if((ptr->state == TX_SHARERAM_READY) 
		&& !hw_txq_busy){
/*            printk("mout: slot=%d, begin = 0x%x, pkt_num=%d, q_num = %d\n",
                   i,
                   ptr->begin,
                   ptr->pkt_num,
                   ptr->q_num
                  );
*/            
            hw_txq_busy |= 1UL << (ptr->q_num & 0x1F);
            ptr->state = TX_SHARERAM_BUSY;
	    modify_tsf(ptr, ptr->pkt_num);
            //critical_section_end1();
            host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)ptr->begin)), 
                                                (UWORD32)(g_qif_table[ptr->q_num].addr));
            slot_mount_nr[ptr->id]++;
        }
        else
        {
            //critical_section_end1();
        }
        mutex_unlock(&g_q_handle.tx_handle.hwtx_mutex);
    }
#endif
    return num;
}


void tx_shareram_slot_packet_dec(UWORD8 slot)
{
	int smart_type = 0;
    if(slot < TX_SHARERAM_SLOTS)
    {
    	smart_type = critical_section_smart_start(1,1);
        //critical_section_start1();
        tx_shareram[slot].pkt_num--;
        //printk("slot:%d dec\n", slot);
        if(tx_shareram[slot].pkt_num == 0)
        {
            //printk("free slot %d\n", slot);
            tx_shareram[slot].state = TX_SHARERAM_IDLE;
            hw_txq_busy &= ~(1UL << ((tx_shareram[slot].q_num) & 0x7));
            tx_shareram[slot].q_num = -1;
            tx_shareram[slot].pkt_num = 0;
            tx_shareram[slot].curr = tx_shareram[slot].begin;
        }
        critical_section_smart_end(smart_type);
        //critical_section_end1();
    }
}

int tx_shareram_slot_busy(UWORD8 slot)
{
    if(slot < TX_SHARERAM_SLOTS)
    {
        if(tx_shareram[slot].state == TX_SHARERAM_BUSY)
        {
            return 1;
        }
    }

    return 0;
}

void show_tx_slots(void)
{
	unsigned int i;
	struct trout_tx_shareram *ptr;

	for(i = 0; i < 2; i++){
		ptr = &tx_shareram[i];
		printk("TX-SLOT[%d] S:%d, q:%d, nr:%d\n", i, ptr->state,
				ptr->q_num, ptr->pkt_num);
	}
}

unsigned int txhw_idle(void)
{
	unsigned int i;
	struct trout_tx_shareram *ptr;

	for(i = 0; i < 2; i++){
		ptr = &tx_shareram[i];
		if(ptr->q_num != -1)
			return 0;
	}
	return 1;
}

/* dma function for vmalloced buffers  by zhao */
int dma_vmalloc_data(void *ta, void *ha, UWORD32 l)
{
	struct page *pg = NULL;
	char *hp = NULL;
	char *va = (char *)ha;
	char *da = (char *)ta;
	UWORD32 of = 0, dl = 0, left = l;
	
	printk("@@@ %s called\n", __func__);	
	while(left != 0){
		pg = vmalloc_to_page(va);
		if(!pg){
			printk("@@@BUG! vmalloc_to_page is NULL\n");
			return -1;
		}
		hp = lowmem_page_address(pg);
		of = ((UWORD32)va) & (PAGE_SIZE - 1);
		hp += of;
		if(of + left > PAGE_SIZE)
			dl = PAGE_SIZE - of;
		else
			dl = (left > PAGE_SIZE)? PAGE_SIZE: left;
		//dma_map_single(NULL, hp, dl, DMA_TO_DEVICE);
		host_write_trout_ram((void *)da, hp, dl);
		va += dl;
		da += dl;
		left -= dl;
	}
	return 0;
}

#ifdef TROUT_WIFI_NPI

struct npi_send_unit{
	UWORD8 copy_tx_descr[TX_DSCR_BUFF_SZ];
	UWORD32 npi_trout_descr ;
};

static struct npi_send_unit npi_send[20];
static int npi_send_unit_count = 0;
struct trout_tx_shareram copy_ttr[2];  //two slot for NPI transfer
UWORD32 npi_addr[2];    //Queue start address inside trout
int copy_q_num = -1;
int transfer_q_num = -1;

UWORD8 *  npi_update_pkt(UWORD8 * tx_descr);
void qmu_cpy_npi_descr(int q_num)
{
	int i = 0;
	UWORD8 tx_buf[TX_DSCR_BUFF_SZ]={0,};
	//UWORD16 num_tx_dscr  = get_machw_num_tx_frames();
      // UWORD32 tx_dscr      = get_machw_tx_frame_pointer();
	UWORD32 ret = 0;
       if(q_num == NORMAL_PRI_Q)
       {
        	for(i=0; i<NPI_TX_PKT_NUM; i++){
        		memcpy(tx_buf, npi_send[i].copy_tx_descr, TX_DSCR_BUFF_SZ);
        		npi_update_pkt(tx_buf);
        	      ret = host_write_trout_ram((void *)npi_send[i].npi_trout_descr, tx_buf, TX_DSCR_BUFF_SZ);
            }
            copy_q_num = NORMAL_PRI_Q;
        }
       else if(q_num == HIGH_PRI_Q)
       {
        	for(i=NPI_TX_PKT_NUM; i<npi_send_unit_count; i++){
		memcpy(tx_buf, npi_send[i].copy_tx_descr, TX_DSCR_BUFF_SZ);
		npi_update_pkt(tx_buf);
	      ret = host_write_trout_ram((void *)npi_send[i].npi_trout_descr, tx_buf, TX_DSCR_BUFF_SZ);
	}
              copy_q_num = HIGH_PRI_Q;
        }

        #if 0
        //printk("%s: %d, copy_q_num=%d, transfer_q_num=%d\n", __func__, __LINE__, copy_q_num, transfer_q_num);
        //yangke, 2013-10-16, in case of copy description is slower than transfer packet in trout
        //this function will call in interrupt isr, so no risk of above line
        if((copy_q_num == NORMAL_PRI_Q) && (transfer_q_num == HIGH_PRI_Q))
        {
                host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)copy_ttr[NORMAL_PRI_Q].begin)), npi_addr[NORMAL_PRI_Q]);
                transfer_q_num = NORMAL_PRI_Q;
        }
        else if((copy_q_num == HIGH_PRI_Q) && (transfer_q_num == NORMAL_PRI_Q))
        {
                host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)copy_ttr[HIGH_PRI_Q].begin)), npi_addr[HIGH_PRI_Q]);
                transfer_q_num = HIGH_PRI_Q;
        }
        #endif
}
#endif

#ifdef TROUT_WIFI_NPI
int qmu_cpy_to_trout_new(UWORD32 q_num, int call_flag)
{
	WORD32 ret = -1;
    UWORD32 trout_descr, trout_next_descr, trout_prev_descr, trout_buf, descr_len, data_len;
    UWORD32 next_descr, sub_msdu_info, temp32[3], trout_sub_msdu_table;
    UWORD32 *descr, *buff_ptr;
    UWORD32 i = 0, sub_buff_len, padding, nr;
    buffer_desc_t buff_list;
    UWORD8 sub_msdu_table[MAX_SUB_MSDU_TABLE_ENTRIES * SUB_MSDU_ENTRY_LEN];
    UWORD8 *sub_buff_ptr;
#ifdef TX_PKT_USE_DMA
    UWORD32 host_prev_descr = 0;
	UWORD8 *sbuf = NULL, *pcur = NULL, *pbak = NULL;
#endif	/* TX_PKT_USE_DMA */
	UWORD32 count = 0, tsf_lo = 0;
	struct timespec time;
	static UWORD32 stime = 0;
	UWORD32 etime;
    struct trout_tx_shareram *tx_shareram_ptr, *tts = NULL;

	//chwg debug
	misc_event_msg_t *misc = NULL;
	UWORD32 host_base_dscr = 0;

	/* if tx_barrier is set, no packet can be copy to trout ram by zhao */
	//Comment by zhao.zhang
	//if(tx_barrier)
	//	return;
    if((tx_shareram_ptr=tx_shareram_slot_alloc(q_num)) == NULL)
    {
        TROUT_TX_DBG3("Error: shareram is full!\n");
        TROUT_TX_DBG3("%s: exit1\n", __FUNCTION__);
        return ret;
    }
    
	descr = g_q_handle.tx_handle.tx_header[q_num].element_to_load;
	if(descr == NULL)
	{
        tx_shareram_slot_free(q_num);
        TROUT_TX_DBG3("%s: exit0\n", __FUNCTION__);
		TROUT_FUNC_EXIT;
		return ret;
	}

	host_base_dscr = (UWORD32)descr;

#ifdef TX_PKT_USE_DMA
	if(g_tx_dma_handle.tx_dma_buf == NULL)
	{
		TROUT_TX_DBG1("tx dma buffer not initialized!\n");
        tx_shareram_slot_free(q_num);
        TROUT_TX_DBG3("%s: exit2\n", __FUNCTION__);
		TROUT_FUNC_EXIT;
		return ret;
	}

	sbuf = (UWORD8 *)WORD_ALIGN((UWORD32)(g_tx_dma_handle.tx_dma_buf)) + SPI_SDIO_WRITE_RAM_CMD_WIDTH;
	pcur = sbuf;
#endif /* TX_PKT_USE_DMA */

	/*
	if(!is_trout_tx_mem_empty(call_flag))
	{
		TROUT_TX_DBG5("tx busy!\n");
        tx_shareram_slot_free(q_num);
        printk("%s: exit3\n", __FUNCTION__);
		TROUT_FUNC_EXIT
		return ret;
	}
	*/
	g_trout_last_tx_dscr = 0;

    trout_descr = tx_shareram_ptr->begin;
    trout_prev_descr = trout_descr;

#ifdef TX_PKT_USE_DMA
	mutex_lock(&g_tx_dma_handle.tx_dma_lock);
#endif	/* TX_PKT_USE_DMA */

	while(descr != NULL)
    {
        sub_msdu_info = get_tx_dscr_submsdu_info(descr);
        nr = get_tx_dscr_num_submsdu(descr);
		next_descr = get_tx_dscr_next_addr(descr);
		buff_ptr = (UWORD32 *)get_tx_dscr_buffer_addr(descr);
		descr_len = TX_DSCR_LEN + nr * SUB_MSDU_ENTRY_LEN;
		trout_buf = trout_descr + descr_len;
        data_len = 0;

        //get the pkts total data length.
        for(i = 0; i<nr; i++)
		{
			get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
            padding = 0;
            buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
            if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
            {
                padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
                buff_list.buff_hdl += padding;
            }
            	            
            data_len += buff_list.data_offset + buff_list.data_length + padding;
		}

		trout_next_descr = (trout_buf + data_len + 3U) & ~3U;
		
		if(trout_next_descr > tx_shareram_ptr->end)
    	{
			if(trout_prev_descr != trout_descr)  //prev->nextdescr = 0.
            {
#ifdef TX_PKT_USE_DMA
				BUG_ON(!host_prev_descr);
				set_tx_dscr_next_addr((UWORD32 *)host_prev_descr, 0);   //notice!!!    
            	
#else                
                temp32[2] = 0;
                host_write_trout_ram((UWORD32 *)trout_prev_descr + TX_DSCR_NEXT_ADDR_WORD_OFFSET,
                                      &temp32[2], 4);
#endif	/* TX_PKT_USE_DMA */                
            }
            
            break;
    	}

        trout_sub_msdu_table = trout_descr + TX_DSCR_LEN;   //get trout sub-msdu table address.
        set_tx_dscr_buffer_addr(descr, trout_buf);
		set_tx_dscr_host_dscr_addr(descr, (UWORD32)descr);
    	if(next_descr != 0)
        	set_tx_dscr_next_addr(descr, trout_next_descr);
        else
        	set_tx_dscr_next_addr(descr, 0);
			
        /* before change host sub msdu table's buffer address, backup the table */
        /* first, beacuse we need recover the value after write to trout        */
        memcpy((void *)sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
        	
        data_len = 0;
		if(is_sub_msdu_table_in_tx_dscr(descr))
		{
			if(nr > 1)	/* in this case, the sub-msdu number should be 1 */
			{
				TROUT_DBG3("Warning: sub-msdu table in tx dscr, but sub-msdu num(%d) is not match!\n", nr);
#ifdef TX_PKT_USE_DMA
				mutex_unlock(&g_tx_dma_handle.tx_dma_lock);
#endif	/* TX_PKT_USE_DMA */	
                tx_shareram_slot_free(q_num);
                printk("%s: exit4\n", __FUNCTION__);
				TROUT_FUNC_EXIT;
				return ret;
			}

            get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);
			data_len = buff_list.data_offset + buff_list.data_length;
			buff_list.buff_hdl = (UWORD8 *)trout_buf;	//updata sub-msdu0 buffer address.

			set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);

			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);
            
        	//copy descr and buf data to trout

			/* update the current timestamp */
        	if(!g_tsf_time)
        	{
        		getnstimeofday(&time);
				stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
        		g_tsf_time = get_machw_tsf_timer_lo();
        	}
        	else
        	{
        		getnstimeofday(&time);
				etime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
				if(etime - stime >= 10000000)	//10s
				{
					g_tsf_time = get_machw_tsf_timer_lo();	//reload tsf timer from reigster.
					getnstimeofday(&time);
					stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
					etime = stime;
				}
				g_tsf_time += etime - stime;
				stime = etime;	//update stime.
        	}
        	tsf_lo = g_tsf_time;
        	
#ifdef TX_PKT_USE_DMA
			if(1){
				if(count == 0){
					tsf_lo = get_machw_tsf_timer_lo();	//read tsf time 1 time for reduce IO!
				}
			}			
			simp_update_tx_dscr_tsf_ts(descr, tsf_lo);
			
			pcur = sbuf + (trout_descr - tx_shareram_ptr->begin);
			memcpy((void *)pcur, descr, descr_len);
			host_prev_descr = (UWORD32)pcur;	//bakup!!!
			pcur += descr_len;

			pcur = sbuf + (trout_buf - tx_shareram_ptr->begin);
			memcpy((void *)pcur, buff_ptr, data_len);
			pcur += data_len;
#else
			/* update the current timestamp */
            update_tx_dscr_tsf_ts(descr);
           		
				/* by zhao */
			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)descr, descr_len, DMA_TO_DEVICE);
				//tx_dscr_check(descr);
				host_write_trout_ram((void *)trout_descr, descr, descr_len);    //descr+sub-msdu table.
				dma_map_single(NULL, (void *)buff_ptr, data_len, DMA_TO_DEVICE);
				host_write_trout_ram((void *)trout_buf, buff_ptr, data_len);
			}else{
	#ifdef SYS_RAM_256M
				dma_vmalloc_data((void *)trout_descr, descr, descr_len);
				dma_vmalloc_data((void *)trout_buf, buff_ptr, data_len);
	#else
				host_write_trout_ram((void *)trout_descr, descr, descr_len);    //descr+sub-msdu table.
				host_write_trout_ram((void *)trout_buf, buff_ptr, data_len);
	#endif
			}
#endif	/* TX_PKT_USE_DMA */
		}
		else
		{
            /* copy nr sub msdu data to trout */
		for(i=0; i<nr; i++){
			get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
			sub_buff_ptr = buff_list.buff_hdl;
			sub_buff_len = buff_list.data_offset + buff_list.data_length;
			    
			    /* change host sub MSDU addr to trout sub MSDU addr */
			padding = 0;
			buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
			if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
			{
			    padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
			    buff_list.buff_hdl += padding;
			}

	            set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);

				/* copy sub msdu data to trout */
#ifdef TX_PKT_USE_DMA
				pcur = sbuf + ((UWORD32)(buff_list.buff_hdl) - tx_shareram_ptr->begin);
				memcpy((void *)pcur, (void *)sub_buff_ptr, sub_buff_len);
				pcur += sub_buff_len;
				pbak = pcur;	//record the last length!!!
#else
				
			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)sub_buff_ptr, sub_buff_len, DMA_TO_DEVICE);
				host_write_trout_ram((void *)buff_list.buff_hdl, (void *)sub_buff_ptr, sub_buff_len);
			}else{
		#ifdef SYS_RAM_256M
				dma_vmalloc_data((void *)buff_list.buff_hdl, sub_buff_ptr,  sub_buff_len);
		#else
				host_write_trout_ram((void *)buff_list.buff_hdl, (void *)sub_buff_ptr, sub_buff_len);
		#endif
			}
#endif	/* TX_PKT_USE_DMA */
	            data_len += sub_buff_len + padding;
	        }

            /* copy host sub msdu table to trout */
#ifdef TX_PKT_USE_DMA
			pcur = sbuf + (trout_sub_msdu_table - tx_shareram_ptr->begin);
			memcpy((void *)pcur, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
			pcur += nr * SUB_MSDU_ENTRY_LEN;

			/* Set the current timestamp */
			if(!g_tsf_time){
				getnstimeofday(&time);
					stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
				g_tsf_time = get_machw_tsf_timer_lo();
			}else{
				getnstimeofday(&time);
					etime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
					if(etime - stime >= 10000000)	//10s
					{
						g_tsf_time = get_machw_tsf_timer_lo();	//reload tsf timer from reigster.
						getnstimeofday(&time);
						stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
						etime = stime;
					}
					g_tsf_time += etime - stime;
					stime = etime;	//update stime.
			}
			tsf_lo = g_tsf_time;
			if(1){
				if(count == 0){
					tsf_lo = get_machw_tsf_timer_lo();	//read tsf time 1 time for reduce IO!
				}
			}
			simp_update_tx_dscr_tsf_ts(descr, tsf_lo);
#else

			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)sub_msdu_info,  nr * SUB_MSDU_ENTRY_LEN, DMA_TO_DEVICE);
				host_write_trout_ram((void *)trout_sub_msdu_table, 
					(void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
			}else{
	#ifdef SYS_RAM_256M
				dma_vmalloc_data((void *)trout_sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
	#else
				host_write_trout_ram((void *)trout_sub_msdu_table, 
					(void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
	#endif
			}
#endif	/* TX_PKT_USE_DMA */
			/* Set the current timestamp */
			update_tx_dscr_tsf_ts(descr);
			
			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);
			/* copy host dscr to trout */
#ifdef TX_PKT_USE_DMA
			pcur = sbuf + (trout_descr - tx_shareram_ptr->begin);
			memcpy((void *)pcur, descr, TX_DSCR_LEN);
			host_prev_descr = (UWORD32)pcur;	//bakup!!!
			pcur += TX_DSCR_LEN;
			
			pcur = pbak;	//get the last length!!!
#else
			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)descr,  TX_DSCR_LEN, DMA_TO_DEVICE);
				ret = host_write_trout_ram((void *)trout_descr, descr, TX_DSCR_LEN);
			}else{
	#ifdef SYS_RAM_256M
				ret = dma_vmalloc_data((void *)trout_descr, descr, TX_DSCR_LEN);
	#else
				ret = host_write_trout_ram((void *)trout_descr, descr, TX_DSCR_LEN);
	#endif
			}
#endif			
		}

#ifdef TROUT_WIFI_NPI
        if(npi_send_unit_count<20)
        {
            memcpy(npi_send[npi_send_unit_count].copy_tx_descr,descr,TX_DSCR_BUFF_SZ);
    		//memcpy(copy_tx_descr, descr, TX_DSCR_BUFF_SZ);
    		npi_send[npi_send_unit_count].npi_trout_descr =  trout_descr;
              printk("%s: npi_send_unit_count=%d, copy_tx_desc=%lu, npi_trout_desc=%lu.\n", __func__, 
                    npi_send_unit_count, npi_send[npi_send_unit_count].copy_tx_descr, npi_send[npi_send_unit_count].npi_trout_descr);
    		npi_send_unit_count++;
        }
#endif

        /* recover host sub msdu table info */
        memcpy((void *)sub_msdu_info, (void *)sub_msdu_table, nr * SUB_MSDU_ENTRY_LEN);

        //restore trout host value.
        set_tx_dscr_buffer_addr(descr, (UWORD32)buff_ptr);
        set_tx_dscr_next_addr(descr, next_descr);
        set_tx_dscr_submsdu_info(descr, sub_msdu_info);
		
		//printk("chwg: host_dscr=0x%p, nr=%d\n", descr, nr);
		count++;
		tx_shareram_ptr->pkt_num++;
		trout_prev_descr = trout_descr;
        trout_descr = trout_next_descr;
        descr = (UWORD32 *)next_descr;
	}

    slot_pkt_nr[tx_shareram_ptr->id] += tx_shareram_ptr->pkt_num;
    
#if 1
	g_trout_last_tx_dscr = trout_prev_descr;	//save last trout tx dscr.
#endif
	
#ifdef TX_PKT_USE_DMA
	//start dma write to trout.
	host_write_trout_ram((void *)tx_shareram_ptr->begin, sbuf, (UWORD32)(pcur-sbuf));
	TROUT_TX_DBG5("%s<%d>: count=%u, len=%u\n", __FUNCTION__, q_num, count, (UWORD32)(pcur-sbuf));	
#endif
    
	g_q_handle.tx_handle.tx_header[q_num].element_to_load = descr;
//	g_q_handle.tx_handle.tx_header[q_num].trout_head = tx_shareram_ptr->begin;
	//g_q_handle.tx_handle.tx_curr_qnum = q_num;

	//write trout 1st descr addr to txq register.
    if(trout_prev_descr != trout_descr)
    {
        tx_shareram_ptr->state = TX_SHARERAM_READY;
		mutex_lock(&g_q_handle.tx_handle.hwtx_mutex);
#if 1
	tts = select_next_slot();
#ifndef TROUT_WIFI_NPI
	if(tts && !hw_txq_busy){
#endif
		UWORD32 tmp32[3];
		tmp32[2] = tts->begin;
		
		#ifdef IBSS_BSS_STATION_MODE
		//chenq add for auto set tx rx power 2013-07-29
		#ifdef TROUT2_WIFI_IC 
			//set_tx_pwr_from_self_adaptive();
		//auto_set_tx_rx_power();
		#endif
		#endif
		//chenq add end
        tts->state = TX_SHARERAM_BUSY;
        hw_txq_busy |= 1UL << (tts->q_num & 0x7);
#ifdef IBSS_BSS_STATION_MODE
		if(!g_wifi_bt_coex)
#endif	/* IBSS_BSS_STATION_MODE */
		{
	    	modify_tsf(tts, tts->pkt_num);

#ifdef TROUT_WIFI_NPI
                npi_addr[q_num] = (UWORD32)(g_qif_table[tts->q_num].addr);
                memcpy(&copy_ttr[q_num],tts,sizeof(copy_ttr[q_num]));
                printk("%s: q_num=%d, tts->q_num=%d, npi_addr[q_num]=%lu,  copy_ttr[q_num].begin=%lu.\n", __func__, 
                        q_num, tts->q_num, npi_addr[q_num], copy_ttr[q_num].begin);
                
                //yangke, 2013-10-16, only the first time copy description to trout the same time is transmtting 
                if(q_num == HIGH_PRI_Q){
                    printk("%s: prepare to cpy NORMAL priority queue to trout\n", __func__);
                    qmu_cpy_npi_descr(NORMAL_PRI_Q);
                    //copy_q_num = NORMAL_PRI_Q;
                }
#endif
                host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)tts->begin)), 
					      					(UWORD32)(g_qif_table[tts->q_num].addr));
#ifdef TROUT_WIFI_NPI
                transfer_q_num = HIGH_PRI_Q;
#endif
		}

#ifdef IBSS_BSS_STATION_MODE
		/* This info is used for WiFi & BT coexist */
		host_write_trout_ram(tx_shareram[0].slot_info, (void *)&tmp32[2], sizeof(UWORD32));
#endif	/* IBSS_BSS_STATION_MODE */

        slot_mount_nr[tts->id]++;
	    g_q_handle.tx_handle.tx_curr_qnum = tts->q_num;
	    g_q_handle.tx_handle.cur_slot = ((unsigned char *)tts - (unsigned char *)&tx_shareram[0]) / sizeof(struct trout_tx_shareram);
#ifndef TROUT_WIFI_NPI
	}
#endif
#else
        TX_PATH_DBG("%s: q_num=%d, pkt_num=%d\n", __func__, q_num, tx_shareram_ptr->pkt_num);
        if(!hw_txq_busy){
            tx_shareram_ptr->state = TX_SHARERAM_BUSY;
            hw_txq_busy |= 1UL << (q_num & 0x7);
		    modify_tsf(tx_shareram_ptr, tx_shareram_ptr->pkt_num);
    		host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)tx_shareram_ptr->begin)), 
    					      					(UWORD32)(g_qif_table[q_num].addr));
            slot_mount_nr[tx_shareram_ptr->id]++;
	    g_q_handle.tx_handle.tx_curr_qnum = tx_shareram_ptr->q_num;
        }
#endif
 if( trout_get_runmode() == SINGLE_CPU_MODE ){
	misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);
	if(misc == NULL)
	{
		printk("%s: alloc misc event fail!\n", __func__);
		mutex_unlock(&g_q_handle.tx_handle.hwtx_mutex);
		return 0;
	}

	misc->name = MISC_TX_COMP;
	misc->info = tx_shareram_ptr->pkt_num;
	misc->data = (UWORD8 *)host_base_dscr;
	post_event((UWORD8*)misc, MISC_EVENT_QID);
 }

	mutex_unlock(&g_q_handle.tx_handle.hwtx_mutex);
    } 
    else
    {   //no pkt need to be send?
        printk("no pkt to send.\n");
        tx_shareram_slot_free(q_num);
    }

    TX_INT_DBG("cpy_to_trout(caller: %s): pkt=%u, q_num=%d, last_dscr: 0x%x\n", 
    			((call_flag == TX_ISR_CALL) ? "isr" : "event"), count, q_num, g_trout_last_tx_dscr);
    
#ifdef TX_PKT_USE_DMA
	mutex_unlock(&g_tx_dma_handle.tx_dma_lock);
#endif

	TROUT_FUNC_EXIT;
    return 0;
}

#else
int qmu_cpy_to_trout_new(UWORD32 q_num, int call_flag)
{
	WORD32 ret = -1;
    UWORD32 trout_descr, trout_next_descr, trout_prev_descr, trout_buf, descr_len, data_len;
    UWORD32 next_descr, sub_msdu_info, temp32[3], trout_sub_msdu_table;
    UWORD32 *descr, *buff_ptr;
    UWORD32 i = 0, sub_buff_len, padding, nr;
    buffer_desc_t buff_list;
    UWORD8 sub_msdu_table[MAX_SUB_MSDU_TABLE_ENTRIES * SUB_MSDU_ENTRY_LEN];
    UWORD8 *sub_buff_ptr;
#ifdef TX_PKT_USE_DMA
    UWORD32 host_prev_descr = 0;
	UWORD8 *sbuf = NULL, *pcur = NULL, *pbak = NULL;
#endif	/* TX_PKT_USE_DMA */
	UWORD32 count = 0, tsf_lo = 0;
	struct timespec time;
	static UWORD32 stime = 0;
	UWORD32 etime;
    struct trout_tx_shareram *tx_shareram_ptr, *tts = NULL;

	//chwg debug
	misc_event_msg_t *misc = NULL;
	UWORD32 host_base_dscr = 0;

	/* if tx_barrier is set, no packet can be copy to trout ram by zhao */
	//Comment by zhao.zhang
	//if(tx_barrier)
	//	return;
    if((tx_shareram_ptr=tx_shareram_slot_alloc(q_num)) == NULL)
    {
        TROUT_TX_DBG3("Error: shareram is full!\n");
        return ret;
    }
    
	descr = g_q_handle.tx_handle.tx_header[q_num].element_to_load;
	if(descr == NULL)
	{
        tx_shareram_slot_free(q_num);
        TROUT_TX_DBG3("%s: exit0\n", __FUNCTION__);
		TROUT_FUNC_EXIT;
		return ret;
	}

	host_base_dscr = (UWORD32)descr;

#ifdef TX_PKT_USE_DMA
	if(g_tx_dma_handle.tx_dma_buf == NULL)
	{
		TROUT_TX_DBG1("tx dma buffer not initialized!\n");
        tx_shareram_slot_free(q_num);
        TROUT_TX_DBG3("%s: exit2\n", __FUNCTION__);
		TROUT_FUNC_EXIT;
		return ret;
	}

	sbuf = (UWORD8 *)WORD_ALIGN((UWORD32)(g_tx_dma_handle.tx_dma_buf)) + SPI_SDIO_WRITE_RAM_CMD_WIDTH;
	pcur = sbuf;
#endif /* TX_PKT_USE_DMA */

	g_trout_last_tx_dscr = 0;

    trout_descr = tx_shareram_ptr->begin;
    trout_prev_descr = trout_descr;

#ifdef TX_PKT_USE_DMA
	mutex_lock(&g_tx_dma_handle.tx_dma_lock);
#endif	/* TX_PKT_USE_DMA */
	while(descr != NULL)
    {
        sub_msdu_info = get_tx_dscr_submsdu_info(descr);
        nr = get_tx_dscr_num_submsdu(descr);
		next_descr = get_tx_dscr_next_addr(descr);
		buff_ptr = (UWORD32 *)get_tx_dscr_buffer_addr(descr);
		descr_len = TX_DSCR_LEN + nr * SUB_MSDU_ENTRY_LEN;
		trout_buf = trout_descr + descr_len;
        data_len = 0;

        //get the pkts total data length.
        for(i = 0; i<nr; i++)
		{
			get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
            padding = 0;
            buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
            if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
            {
                padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
                buff_list.buff_hdl += padding;
            }
            	            
            data_len += buff_list.data_offset + buff_list.data_length + padding;
		}

		trout_next_descr = (trout_buf + data_len + 3U) & ~3U;
		
		if(trout_next_descr > tx_shareram_ptr->end || tx_shareram_ptr->pkt_num >= TX_MAX_PKT_PER_SLOT)
    	{
			if(trout_prev_descr != trout_descr)  //prev->nextdescr = 0.
            {
#ifdef TX_PKT_USE_DMA
				BUG_ON(!host_prev_descr);
				set_tx_dscr_next_addr((UWORD32 *)host_prev_descr, 0);   //notice!!!    
            	
#else                
                temp32[2] = 0;
                host_write_trout_ram((UWORD32 *)trout_prev_descr + TX_DSCR_NEXT_ADDR_WORD_OFFSET,
                                      &temp32[2], 4);
#endif	/* TX_PKT_USE_DMA */                
            }
            
            break;
    	}

        trout_sub_msdu_table = trout_descr + TX_DSCR_LEN;   //get trout sub-msdu table address.
        set_tx_dscr_buffer_addr(descr, trout_buf);
		set_tx_dscr_host_dscr_addr(descr, (UWORD32)descr);
    	if(next_descr != 0)
        	set_tx_dscr_next_addr(descr, trout_next_descr);
        else
        	set_tx_dscr_next_addr(descr, 0);
			
        /* before change host sub msdu table's buffer address, backup the table */
        /* first, beacuse we need recover the value after write to trout        */
        memcpy((void *)sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
        	
        data_len = 0;
		if(is_sub_msdu_table_in_tx_dscr(descr))
		{
			if(nr > 1)	/* in this case, the sub-msdu number should be 1 */
			{
				TROUT_DBG3("Warning: sub-msdu table in tx dscr, but sub-msdu num(%d) is not match!\n", nr);
#ifdef TX_PKT_USE_DMA
				mutex_unlock(&g_tx_dma_handle.tx_dma_lock);
#endif	/* TX_PKT_USE_DMA */	
                tx_shareram_slot_free(q_num);
                printk("%s: exit4\n", __FUNCTION__);
				TROUT_FUNC_EXIT;
				return ret;
			}

            get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);
			data_len = buff_list.data_offset + buff_list.data_length;
			buff_list.buff_hdl = (UWORD8 *)trout_buf;	//updata sub-msdu0 buffer address.

			set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), 0);

			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);
            
        	//copy descr and buf data to trout
        	
#ifdef TX_PKT_USE_DMA
			if(1){
				if(count == 0){
					tsf_lo = get_machw_tsf_timer_lo();	//read tsf time 1 time for reduce IO!
				}
			}			
			simp_update_tx_dscr_tsf_ts(descr, tsf_lo);
			
			pcur = sbuf + (trout_descr - tx_shareram_ptr->begin);
			memcpy((void *)pcur, descr, descr_len);
			host_prev_descr = (UWORD32)pcur;	//bakup!!!
			pcur += descr_len;

			pcur = sbuf + (trout_buf - tx_shareram_ptr->begin);
			memcpy((void *)pcur, buff_ptr, data_len);
			pcur += data_len;
#else
				/* by zhao */
			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)descr, descr_len, DMA_TO_DEVICE);
				//tx_dscr_check(descr);
				host_write_trout_ram((void *)trout_descr, descr, descr_len);    //descr+sub-msdu table.
				dma_map_single(NULL, (void *)buff_ptr, data_len, DMA_TO_DEVICE);
				host_write_trout_ram((void *)trout_buf, buff_ptr, data_len);
			}else{
	#ifdef SYS_RAM_256M
				dma_vmalloc_data((void *)trout_descr, descr, descr_len);
				dma_vmalloc_data((void *)trout_buf, buff_ptr, data_len);
	#else
				host_write_trout_ram((void *)trout_descr, descr, descr_len);    //descr+sub-msdu table.
				host_write_trout_ram((void *)trout_buf, buff_ptr, data_len);
	#endif
			}
#endif	/* TX_PKT_USE_DMA */
		}
		else
		{
            /* copy nr sub msdu data to trout */
		for(i=0; i<nr; i++){
			get_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);
			sub_buff_ptr = buff_list.buff_hdl;
			sub_buff_len = buff_list.data_offset + buff_list.data_length;
			    
			    /* change host sub MSDU addr to trout sub MSDU addr */
			padding = 0;
			buff_list.buff_hdl = (UWORD8 *)(trout_buf + data_len);
			if(((UWORD32)buff_list.buff_hdl & 0x3U) != 0)
			{
			    padding = 4 - ((UWORD32)buff_list.buff_hdl & 0x3U);
			    buff_list.buff_hdl += padding;
			}

	            set_tx_dscr_submsdu_buff_info(descr, (UWORD32)(&buff_list), i);

				/* copy sub msdu data to trout */
#ifdef TX_PKT_USE_DMA
				pcur = sbuf + ((UWORD32)(buff_list.buff_hdl) - tx_shareram_ptr->begin);
				memcpy((void *)pcur, (void *)sub_buff_ptr, sub_buff_len);
				pcur += sub_buff_len;
				pbak = pcur;	//record the last length!!!
#else
				
			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)sub_buff_ptr, sub_buff_len, DMA_TO_DEVICE);
				host_write_trout_ram((void *)buff_list.buff_hdl, (void *)sub_buff_ptr, sub_buff_len);
			}else{
		#ifdef SYS_RAM_256M
				dma_vmalloc_data((void *)buff_list.buff_hdl, sub_buff_ptr,  sub_buff_len);
		#else
				host_write_trout_ram((void *)buff_list.buff_hdl, (void *)sub_buff_ptr, sub_buff_len);
		#endif
			}
#endif	/* TX_PKT_USE_DMA */
	            data_len += sub_buff_len + padding;
	        }

            /* copy host sub msdu table to trout */
#ifdef TX_PKT_USE_DMA
			pcur = sbuf + (trout_sub_msdu_table - tx_shareram_ptr->begin);
			memcpy((void *)pcur, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
			pcur += nr * SUB_MSDU_ENTRY_LEN;

			/* Set the current timestamp */
			if(!g_tsf_time){
				getnstimeofday(&time);
					stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
				g_tsf_time = get_machw_tsf_timer_lo();
			}else{
				getnstimeofday(&time);
					etime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
					if(etime - stime >= 10000000)	//10s
					{
						g_tsf_time = get_machw_tsf_timer_lo();	//reload tsf timer from reigster.
						getnstimeofday(&time);
						stime = (UWORD32)(time.tv_sec * 1000 * 1000 + time.tv_nsec / 1000);
						etime = stime;
					}
					g_tsf_time += etime - stime;
					stime = etime;	//update stime.
			}
			tsf_lo = g_tsf_time;
			if(1){
				if(count == 0){
					tsf_lo = get_machw_tsf_timer_lo();	//read tsf time 1 time for reduce IO!
				}
			}
			simp_update_tx_dscr_tsf_ts(descr, tsf_lo);
#else

			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)sub_msdu_info,  nr * SUB_MSDU_ENTRY_LEN, DMA_TO_DEVICE);
				host_write_trout_ram((void *)trout_sub_msdu_table, 
					(void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
			}else{
	#ifdef SYS_RAM_256M
				dma_vmalloc_data((void *)trout_sub_msdu_table, (void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
	#else
				host_write_trout_ram((void *)trout_sub_msdu_table, 
					(void *)sub_msdu_info, nr * SUB_MSDU_ENTRY_LEN);
	#endif
			}
#endif	/* TX_PKT_USE_DMA */
			set_tx_dscr_submsdu_info(descr, trout_sub_msdu_table);
			/* copy host dscr to trout */
#ifdef TX_PKT_USE_DMA
			pcur = sbuf + (trout_descr - tx_shareram_ptr->begin);
			memcpy((void *)pcur, descr, TX_DSCR_LEN);
			host_prev_descr = (UWORD32)pcur;	//bakup!!!
			pcur += TX_DSCR_LEN;
			
			pcur = pbak;	//get the last length!!!
#else
			if(gsb->tx_start != NULL){
				dma_map_single(NULL, (void *)descr,  TX_DSCR_LEN, DMA_TO_DEVICE);
				ret = host_write_trout_ram((void *)trout_descr, descr, TX_DSCR_LEN);
			}else{
	#ifdef SYS_RAM_256M
				ret = dma_vmalloc_data((void *)trout_descr, descr, TX_DSCR_LEN);
	#else
				ret = host_write_trout_ram((void *)trout_descr, descr, TX_DSCR_LEN);
	#endif
			}
#endif			
		}

#ifdef TROUT_WIFI_NPI
        if(npi_send_unit_count<20)
        {
            memcpy(npi_send[npi_send_unit_count].copy_tx_descr,descr,TX_DSCR_BUFF_SZ);
    		//memcpy(copy_tx_descr, descr, TX_DSCR_BUFF_SZ);
    		npi_send[npi_send_unit_count].npi_trout_descr =  trout_descr;
    		npi_send_unit_count++;
        }
#endif

        /* recover host sub msdu table info */
        memcpy((void *)sub_msdu_info, (void *)sub_msdu_table, nr * SUB_MSDU_ENTRY_LEN);
		//printk("next_desc:%x\n", get_tx_dscr_next_addr(descr));

        //restore trout host value.
        set_tx_dscr_buffer_addr(descr, (UWORD32)buff_ptr);
        set_tx_dscr_next_addr(descr, next_descr);
        set_tx_dscr_submsdu_info(descr, sub_msdu_info);
		
		//printk("chwg: host_dscr=0x%p, nr=%d\n", descr, nr);
		//printk("c_desc:0x%x, n_desc:0x%x\n", trout_descr, trout_next_descr);
		count++;
		tx_shareram_ptr->desc_addr[tx_shareram_ptr->pkt_num] = trout_descr;
		tx_shareram_ptr->pkt_num++;
		trout_prev_descr = trout_descr;
        trout_descr = trout_next_descr;
        descr = (UWORD32 *)next_descr;
	}

	if(tx_shareram_ptr->pkt_num < TX_MAX_PKT_PER_SLOT)
        {
	        tx_shareram_ptr->desc_addr[tx_shareram_ptr->pkt_num] = trout_next_descr;
        }
	//printk("pkt: total=%d, ncpy =%d!\n", tx_shareram_ptr->pkt_num, count);  //Hugh: for debug.

    //slot_pkt_nr[tx_shareram_ptr->id] += tx_shareram_ptr->pkt_num;
    slot_pkt_nr[tx_shareram_ptr->id] += count;
    
#if 1
	g_trout_last_tx_dscr = trout_prev_descr;	//save last trout tx dscr.
#endif
	
#ifdef TX_PKT_USE_DMA
	//start dma write to trout.
	host_write_trout_ram((void *)tx_shareram_ptr->begin, sbuf, (UWORD32)(pcur-sbuf));
	TROUT_TX_DBG5("%s<%d>: count=%u, len=%u\n", __FUNCTION__, q_num, count, (UWORD32)(pcur-sbuf));	
#endif
    
	g_q_handle.tx_handle.tx_header[q_num].element_to_load = descr;
//	g_q_handle.tx_handle.tx_header[q_num].trout_head = tx_shareram_ptr->begin;
	//g_q_handle.tx_handle.tx_curr_qnum = q_num;

	//write trout 1st descr addr to txq register.
    if(trout_prev_descr != trout_descr)
    {
        tx_shareram_ptr->state = TX_SHARERAM_READY;
		mutex_lock(&g_q_handle.tx_handle.hwtx_mutex);
	if(!hw_txq_busy && (tts = select_next_slot())){
		UWORD32 tmp32[3];
		tmp32[2] = tts->begin;
		#ifdef IBSS_BSS_STATION_MODE
		//chenq add for auto set tx rx power 2013-07-29
		#ifdef TROUT2_WIFI_IC 
			//set_tx_pwr_from_self_adaptive();
		//auto_set_tx_rx_power();
		#endif
		#endif
		//chenq add end
        tts->state = TX_SHARERAM_BUSY;
        hw_txq_busy |= 1UL << (tts->q_num & 0x7);
		modify_tsf(tts, tts->pkt_num);
#ifdef IBSS_BSS_STATION_MODE
			if(!g_wifi_bt_coex)
#endif	/* IBSS_BSS_STATION_MODE */
			{
				host_write_trout_reg(convert_to_le(virt_to_phy_addr((UWORD32)tts->begin)), 
						      					(UWORD32)(g_qif_table[tts->q_num].addr));

				//printk("tpkt:%d, %d\n", tts->id, tts->pkt_num);

#ifdef TROUT_WIFI_NPI
				npi_addr = (UWORD32)(g_qif_table[tts->q_num].addr);
			    memcpy(&copy_ttr,tts,sizeof(copy_ttr));
#endif
			}

#ifdef IBSS_BSS_STATION_MODE
			/* This info is used for WiFi & BT coexist */
			host_write_trout_ram(tx_shareram[0].slot_info, (void *)&tmp32[2], sizeof(UWORD32));
#endif	/* IBSS_BSS_STATION_MODE */

	        slot_mount_nr[tts->id]++;
		    g_q_handle.tx_handle.tx_curr_qnum = tts->q_num;
		    g_q_handle.tx_handle.cur_slot = ((unsigned char *)tts - (unsigned char *)&tx_shareram[0]) / sizeof(struct trout_tx_shareram);
		}
		//else
		//	printk(" but hw busy:%d!\n", hw_txq_busy);
		
		mutex_unlock(&g_q_handle.tx_handle.hwtx_mutex);

	} 
	else
	{   //no pkt need to be send?
		printk("no pkt to send.\n");
		tx_shareram_slot_free(q_num);
	}

    TX_INT_DBG("cpy_to_trout(caller: %s): pkt=%u, q_num=%d, last_dscr: 0x%x\n", 
    			((call_flag == TX_ISR_CALL) ? "isr" : "event"), count, q_num, g_trout_last_tx_dscr);
    
#ifdef TX_PKT_USE_DMA
	mutex_unlock(&g_tx_dma_handle.tx_dma_lock);
#endif

	TROUT_FUNC_EXIT;
    return 0;
}
#endif

#endif


void tx_pkt_process_new_single(UWORD8 slot, int call_flag)
{
	//use to disable discription read back, to gain throughput
	//UWORD8 num_tx_dscr = 0, status;
	//UWORD32 *tx_dscr = NULL;
	//misc_event_msg_t *misc = 0;
	//UWORD32 trout_descr, trout_next_descr;
	//UWORD32 next_descr = 0;
	//UWORD32  *tx_dscr_base = NULL;
	struct trout_tx_shareram *tx_sram_ptr = &tx_shareram[slot];
	//UWORD32 h_dscr[TX_DSCR_LEN/4];
	//UWORD32 mac_head_len;
	//UWORD8 *trout_mac_head_addr, *host_mac_head_addr, *pcontent;
	//UWORD16 frame_len;
	int smart_type = 0;
	
	UWORD8 q_num = tx_sram_ptr->q_num;

	if(host_read_trout_reg(g_qif_table[q_num].addr) != 0)
	{
	//	printk("tx_process: slot%d is busy!\n", tx_sram_ptr->id);
		return;
	}

	//printk("tx_process: slot%d tx %d ok!\n", tx_sram_ptr->id, tx_sram_ptr->pkt_num);
     smart_type = critical_section_smart_start(1,1);
    //critical_section_start1();
    
    tx_shareram[slot].pkt_num = 0;
    tx_shareram[slot].state = TX_SHARERAM_IDLE;
    hw_txq_busy &= ~(1UL << ((tx_shareram[slot].q_num) & 0x7));
    tx_shareram[slot].q_num = -1;
    tx_shareram[slot].curr = tx_shareram[slot].begin;
    critical_section_smart_end(smart_type);
    //critical_section_end1();
}

UWORD8  tx_pkt_process(UWORD8 slot, int call_flag,UWORD32 **dscr_base)
{
	UWORD8 num_tx_dscr = 0, status;
	UWORD32 *tx_dscr = NULL;
	UWORD32 trout_descr, trout_next_descr;
	UWORD32 next_descr = 0;
	UWORD32  *tx_dscr_base = NULL;
	struct trout_tx_shareram *tx_sram_ptr = &tx_shareram[slot];
	UWORD32 h_dscr[TX_DSCR_LEN/4];
	UWORD32 mac_head_len;
	UWORD8 *trout_mac_head_addr, *host_mac_head_addr, *pcontent;
	UWORD16 frame_len;
	
    TROUT_FUNC_ENTER;
    
	num_tx_dscr = 0;
	trout_descr = tx_sram_ptr->curr;

	while(trout_descr >= tx_sram_ptr->begin && trout_descr < tx_sram_ptr->end){		
        memset(h_dscr, 0, TX_DSCR_LEN);
		host_read_trout_ram((UWORD8 *)h_dscr, (UWORD8 *)trout_descr, sizeof(h_dscr));	//DMA!
		status = get_tx_dscr_status(h_dscr);
		if(status == INVALID){
			TROUT_DBG4("err: first tx desc is invalid!\n");
			break;
		}
		if(status == PENDING){
			TROUT_DBG5("warning: tx desc is not complete!\n");
			break;
		}

		tx_dscr = (UWORD32 *)(h_dscr[WORD_OFFSET_35]);
		if(tx_dscr == NULL || validate_buffer(tx_dscr) == BFALSE || 
						(UWORD32)tx_dscr != get_tx_dscr_host_dscr_addr(tx_dscr)){
		    TROUT_DBG2("trout: bad host tx descr:%p\n", tx_dscr);
		    TROUT_FUNC_EXIT;
		    return 0;
		}

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
		if((null_frame_dscr != NULL) && ((UWORD32)null_frame_dscr != 0x1) && (tx_dscr == null_frame_dscr))
		{
			if(waitqueue_active(&null_frame_completion.wait))
			{
				if(status != TX_TIMEOUT
                                && (get_tx_dscr_frag_status_summary((UWORD32 *)h_dscr) == 0xFFFFFFFF))
				{
					null_frame_dscr = NULL;
                                        complete(&null_frame_completion);
					printk("==== %s:send complete msg to suspend progress ====\n", __func__);
				}
				else
				{
					null_frame_dscr = 0x01;
					printk("====== %s: send ps null data timeout =====\n", __func__);
				}
				
			}
			else
			{
			    if(status != TX_TIMEOUT
                            && (get_tx_dscr_frag_status_summary((UWORD32 *)h_dscr) == 0xFFFFFFFF)){
				    null_frame_dscr = NULL;
				}else{
				    null_frame_dscr = 0x01;
                                }
				printk("==== tx pkt process, status=%d ===\n", status);
			}
		}		
#endif
#endif
		
		//update tx_dscr without word4, word30, word31, word35!
		memcpy((void *)tx_dscr, (void *)h_dscr, 4 * 4);		//Word0~3.
        
		next_descr = *(tx_dscr + 4);
		trout_next_descr = *(h_dscr + 4);

		memcpy((void *)(tx_dscr+5), (void *)(h_dscr+5), 4 * 25);	//Word5~29.
		memcpy((void *)(tx_dscr+32), (void *)(h_dscr+32), 4 * 3);	//Word32~34.

#ifdef MAC_802_11N
		/* for block-ack, seq number in mac head is needed, so we need update */
		/* this region. in case the pkt is ADDBA-Req, after send complete, A- */
		/* DDBA-Req content is updated by H/W, we will using at process_tx_co-*/
		/*p_sta()  --add by chengwg                                           */
		if(g_tx_ba_setup && (get_ht_enable() == 1))
		{
			trout_mac_head_addr = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)h_dscr);
			host_mac_head_addr = (UWORD8 *)get_tx_dscr_mac_hdr_addr((UWORD32 *)tx_dscr);
			mac_head_len = get_tx_dscr_mh_len((UWORD32 *)tx_dscr);
			host_read_trout_ram(host_mac_head_addr, trout_mac_head_addr, 
											mac_head_len + BLOCK_REQ_DATA_LEN);
		}
#endif

        if(num_tx_dscr == 0)
        {
            tx_dscr_base = tx_dscr;
        }

        trout_descr = trout_next_descr;
        num_tx_dscr++;

//        g_q_handle.tx_handle.tx_header[qnum].handle_pkt--;	//chengwg add.
        tx_shareram_slot_packet_dec(slot);        
    }
	TX_PATH_DBG("%s: num_tx_dscr=%d\n", __func__, num_tx_dscr);
	tx_sram_ptr->curr = trout_descr;
	if(tx_dscr_base == NULL || num_tx_dscr == 0){
		TROUT_FUNC_EXIT;
		return 0;
	}
    
#ifdef BURST_TX_MODE
    if(g_burst_tx_mode_enabled == BTRUE)
    {
        update_burst_mode_tx_dscr((UWORD32 *)tx_dscr);
        TROUT_FUNC_EXIT;
        return 0;
    }
#endif /* BURST_TX_MODE */

	*dscr_base =  tx_dscr_base;
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.misctxcompevent++;
#endif /* MEASURE_PROCESSING_DELAY */
	return num_tx_dscr;
	TROUT_FUNC_EXIT;
}

void tx_pkt_process_new_smp(UWORD8 slot, int call_flag)	//cur use.
{
	UWORD8 num_tx_dscr = 0;
	UWORD32  *tx_dscr_base = NULL;
	misc_event_msg_t *misc = 0;
	
	num_tx_dscr = tx_pkt_process(slot,call_flag,&tx_dscr_base);
	
	if( num_tx_dscr == 0 || tx_dscr_base == NULL ){
		return ;
	}
    /* Allocate buffer for the miscellaneous event */
    misc = (misc_event_msg_t*)event_mem_alloc(MISC_EVENT_QID);

    /* If allocation fails, then simply return doing nothing. Whenever a new */
    /* is read out of the Tx-Q the sync operation will clear these packets.  */
    if(misc == NULL){
	printk("TX can not alloc MISC mem\n");
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
    TROUT_DBG6("%s: post event 0x%x, base: 0x%p, num: %d\n", 
    			__FUNCTION__, MISC_TX_COMP, tx_dscr_base, num_tx_dscr);
#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.misctxcompevent++;
#endif /* MEASURE_PROCESSING_DELAY */

	TROUT_FUNC_EXIT;
}

void tx_pkt_process_new(UWORD8 slot, int call_flag)
{
	int mode = trout_get_runmode();
	tx_pkt_process_new_smp(slot,call_flag);
}


#ifdef TROUT_B2B_TEST_MODE
int descpkt_cpy_to_trout(void)
{
    printk("2:restore txdesc at-> 0x%x\n", trout_b2b_descr_pkt[0]);

    /* Set the current timestamp */
    update_tx_dscr_tsf_ts(&trout_b2b_descr_pkt[5]);
    host_write_trout_ram((void *)trout_b2b_descr_pkt[0], &trout_b2b_descr_pkt[5], 2000);
    set_machw_q_ptr(trout_b2b_descr_pkt[1] & 0xFF, (UWORD32 *)trout_b2b_descr_pkt[0]);
    return 0;
}
#endif

void trout_load_qmu(void)
{
    WORD32 i;
    qmu_tx_handle_t *tx_handle = &g_q_handle.tx_handle;
    WORD32 q_nr = get_no_tx_queues();

	TROUT_FUNC_ENTER;
    for(i=0; i<q_nr; i++)
    {
        mutex_lock(&g_q_handle.tx_handle.txq_lock);
        if(tx_handle->tx_header[i].element_to_load != NULL
            && (tx_handle->tx_header[i].q_status != Q_SUSPENDED))
        {
        	TX_PATH_DBG("%s: reload pkt, qnum=%d\n", __func__, i);
            if(qmu_cpy_to_trout_new(i, TX_ISR_CALL) == 0)
            {
                mutex_unlock(&g_q_handle.tx_handle.txq_lock);
                break;
            }
        }
        mutex_unlock(&g_q_handle.tx_handle.txq_lock); 
    }
    
    TROUT_FUNC_EXIT;
}


/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_add_tx_packet                                        */
/*                                                                           */
/*  Description   : This function adds a packet to the specified Tx queue    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit queue handle                  */
/*                  2) Queue number                                          */
/*                  3) Pointer to the packet descriptor to be added          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The packet is added to the packet descriptor link list   */
/*                  and the queue structure is updated.                      */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, QMU_OK on success                                */
/*                          Failure code otherwise                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/
UWORD8 qmu_add_tx_packet(qmu_tx_handle_t *tx_handle, UWORD8 q_num, UWORD8 *tx_dscr)
{
    UWORD8     *dscr        = 0;
    UWORD32    *temp_ptr    = 0;
    q_struct_t *q_tx_struct = &(tx_handle->tx_header[q_num]);
	UWORD8 qnum;	//debug.

	TROUT_FUNC_ENTER;
#ifdef TX_ABORT_FEATURE
    if(is_machw_tx_aborted() == BTRUE)
    {
        if(BTRUE == is_this_txq_full(q_num))
        {
			TROUT_FUNC_EXIT;
            return QMU_Q_INACTIVE;
        }
    }
#endif /* TX_ABORT_FEATURE */

#ifdef LOCALMEM_TX_DSCR
    /* Check if the transmit descriptor belongs to local memory and in that  */
    /* case allocate a shared memory descriptor and copy the contents to the */
    /* same. Otherwise use same descriptor pointer for further processing    */
    if(get_mem_pool_hdl(tx_dscr) == g_local_dscr_mem_handle)
    {
		dscr = transfer_tx_dscr_local_to_shared(tx_dscr);
        if(NULL == dscr)
        {
        	TROUT_FUNC_EXIT;
            return QMU_SHARED_MALLOC_FAILED;
        }

		/* Free the TX descriptor buffer/s in local memory */
		free_tx_dscr_buffer(g_local_dscr_mem_handle, tx_dscr);
    }
    else
    {
        dscr = tx_dscr;
    }
#else /* LOCALMEM_TX_DSCR */
    dscr = tx_dscr;
#endif /* LOCALMEM_TX_DSCR */

#ifdef DEBUG_MODE
    g_mac_stats.qatxp++;
    g_mac_stats.qa[q_num]++;

    if(q_tx_struct->element_cnt == 0)
        g_mac_stats.qa0exc[q_num]++;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.txdscr2machwtxq++;
    g_delay_stats.submsdu2machwtxq +=
    (UWORD32) get_tx_dscr_num_submsdu((UWORD32 *)tx_dscr);
#endif /* MEASURE_PROCESSING_DELAY */

#ifdef IBSS_BSS_STATION_MODE
    if(BTRUE == g_wifi_bt_coex){
        coex_wifi_tx_rx_pkg_sum(COEX_WIFI_TX_PKG, 1);
    }
#endif
    /* Reset the next address in the transmit descriptor */
    set_tx_dscr_next_addr((UWORD32 *)dscr, 0);

	if(get_tx_dscr_status((UWORD32*)dscr) != PENDING)
	{
		TROUT_DBG3("%s:BUG! Tx status is not pending!\n", __func__);
		return QMU_Q_INACTIVE;
	}
	
    /* Critical section beginning */
    //critical_section_start();

    mutex_lock(&tx_handle->txq_lock); //Hugh
		
    /* Initialize a temporary pointer to the last element in the Tx Q */
    temp_ptr = q_tx_struct->element_tail;

    /* Increment the element count */
    q_tx_struct->element_cnt++;
    tx_handle->tx_list_count++;

	qnum = g_q_handle.tx_handle.tx_curr_qnum;

//	TX_PATH_DBG("%s: qnum=%d, tx_pkt_count=%d\n", __func__, q_num, tx_handle->tx_pkt_count);
//	TX_INT_DBG("event: txq[%d].handle_pkt=%d, tx_pkt_count = %u\n", 
//		qnum, g_q_handle.tx_handle.tx_header[qnum].handle_pkt, tx_handle->tx_pkt_count);
	
    /* If there are no packets in the current queue, update the head pointer */
    /* of the queue with the new packet descriptor.                          */
    if(q_tx_struct->element_head == NULL)
    {
        q_tx_struct->element_head = (UWORD32 *)dscr;
    }
    
    if(q_tx_struct->element_to_load == NULL)//add by Hugh
    {
        q_tx_struct->element_to_load = (UWORD32 *)dscr;    
    }
    
    /* If the last element is not NULL this is not the first packet. In this */
    /* case update the next element pointer of the previous descriptor with  */
    /* the new descriptor.                                                   */
    if(temp_ptr != NULL)
    {
        set_tx_dscr_next_addr(temp_ptr, (WORD32)dscr);
    }

    /* Update the last element pointer with the new descriptor */
    q_tx_struct->element_tail = (UWORD32 *)dscr;

    /* If the Tx Q header is empty update the MAC H/w Q header register */
    if((q_num < get_no_tx_queues()) && (HW_TXQ_ALL_IDLE())
       && (tx_handle->tx_header[q_num].q_status != Q_SUSPENDED))
    {
            qmu_cpy_to_trout_new(q_num, TX_SEND_CALL);
        }
    /* End of critical section */
    //critical_section_end();
    mutex_unlock(&tx_handle->txq_lock);  //Hugh
	TROUT_FUNC_EXIT;
    return QMU_OK;
}

#ifdef TROUT_WIFI_NPI
UWORD8 qmu_add_tx_packet_no_send(qmu_tx_handle_t *tx_handle, UWORD8 q_num, UWORD8 *tx_dscr)
{
    UWORD8     *dscr        = 0;
    UWORD32    *temp_ptr    = 0;
    q_struct_t *q_tx_struct = &(tx_handle->tx_header[q_num]);
	UWORD8 qnum;	//debug.

	TROUT_FUNC_ENTER;
#ifdef TX_ABORT_FEATURE
    if(is_machw_tx_aborted() == BTRUE)
    {
        if(BTRUE == is_this_txq_full(q_num))
        {
			TROUT_FUNC_EXIT;
            return QMU_Q_INACTIVE;
        }
    }
#endif /* TX_ABORT_FEATURE */

#ifdef LOCALMEM_TX_DSCR
    /* Check if the transmit descriptor belongs to local memory and in that  */
    /* case allocate a shared memory descriptor and copy the contents to the */
    /* same. Otherwise use same descriptor pointer for further processing    */
    if(get_mem_pool_hdl(tx_dscr) == g_local_dscr_mem_handle)
    {
		dscr = transfer_tx_dscr_local_to_shared(tx_dscr);
        if(NULL == dscr)
        {
        	TROUT_FUNC_EXIT;
            return QMU_SHARED_MALLOC_FAILED;
        }

		/* Free the TX descriptor buffer/s in local memory */
		free_tx_dscr_buffer(g_local_dscr_mem_handle, tx_dscr);
    }
    else
    {
        dscr = tx_dscr;
    }
#else /* LOCALMEM_TX_DSCR */
    dscr = tx_dscr;
#endif /* LOCALMEM_TX_DSCR */

#ifdef DEBUG_MODE
    g_mac_stats.qatxp++;
    g_mac_stats.qa[q_num]++;

    if(q_tx_struct->element_cnt == 0)
        g_mac_stats.qa0exc[q_num]++;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.txdscr2machwtxq++;
    g_delay_stats.submsdu2machwtxq +=
    (UWORD32) get_tx_dscr_num_submsdu((UWORD32 *)tx_dscr);
#endif /* MEASURE_PROCESSING_DELAY */

    /* Reset the next address in the transmit descriptor */
    set_tx_dscr_next_addr((UWORD32 *)dscr, 0);

    /* Critical section beginning */
    //critical_section_start();

    mutex_lock(&tx_handle->txq_lock); //Hugh
		
    /* Initialize a temporary pointer to the last element in the Tx Q */
    temp_ptr = q_tx_struct->element_tail;

    /* Increment the element count */
    q_tx_struct->element_cnt++;
    tx_handle->tx_list_count++;

	qnum = g_q_handle.tx_handle.tx_curr_qnum;
	
    /* If there are no packets in the current queue, update the head pointer */
    /* of the queue with the new packet descriptor.                          */
    if(q_tx_struct->element_head == NULL)
    {
        q_tx_struct->element_head = (UWORD32 *)dscr;
    }
    
    if(q_tx_struct->element_to_load == NULL)//add by Hugh
    {
        q_tx_struct->element_to_load = (UWORD32 *)dscr;    
    }
    
    /* If the last element is not NULL this is not the first packet. In this */
    /* case update the next element pointer of the previous descriptor with  */
    /* the new descriptor.                                                   */
    if(temp_ptr != NULL)
    {
        set_tx_dscr_next_addr(temp_ptr, (WORD32)dscr);
    }

    /* Update the last element pointer with the new descriptor */
    q_tx_struct->element_tail = (UWORD32 *)dscr;

    /* End of critical section */
    //critical_section_end();
    mutex_unlock(&tx_handle->txq_lock);  //Hugh
	TROUT_FUNC_EXIT;
    return QMU_OK;
}
#endif


/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_add_tx_packet_list                                   */
/*                                                                           */
/*  Description   : This function adds a list of packets to the specified Tx */
/*                  queue                                                    */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit queue handle                  */
/*                  2) Queue number                                          */
/*                  3) Double Pointer to the first descriptor to be added    */
/*                  4) Double Pointer to the last descriptor to be added     */
/*                  5) Pointer to the number of descriptors in list          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The list of packets are added to the packet descriptor   */
/*                  link list and the queue structure is updated.            */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8, QMU_OK on success                                */
/*                          Failure code otherwise                           */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 qmu_add_tx_packet_list(qmu_tx_handle_t *tx_handle, UWORD8 q_num,
                              UWORD8 **dscr_head, UWORD8 **dscr_tail,
                              UWORD8 *num_dscr)
{
    UWORD32    *lp_last     = (UWORD32 *)*dscr_head;
    UWORD32    *sp_head     = (UWORD32 *)*dscr_head;
    UWORD32    *sp_last     = (UWORD32 *)*dscr_head;
    UWORD32    *temp_ptr    = 0;
    UWORD32    dscr_cnt     = 0;
    q_struct_t *q_tx_struct = &(tx_handle->tx_header[q_num]);

    TROUT_FUNC_ENTER;

    TROUT_DBG5("%s: q_num=%d, num_dscr=%d\n", __func__, q_num, *num_dscr);
#ifdef LOCALMEM_TX_DSCR
    for(dscr_cnt = 0; dscr_cnt < *num_dscr; dscr_cnt++)
    {
        /* Check if the transmit descriptor belongs to local memory and in that  */
        /* case allocate a shared memory descriptor and copy the contents to the */
        /* same. Otherwise use same descriptor pointer for further processing    */
        if(get_mem_pool_hdl(lp_last) == g_local_dscr_mem_handle)
        {
			temp_ptr = transfer_tx_dscr_local_to_shared(lp_last);

            if(NULL == temp_ptr)
                break;

            if(dscr_cnt == 0)
            {
                sp_head = sp_last = temp_ptr;
            }
            else
            {
                set_tx_dscr_next_addr(sp_last, (UWORD32)temp_ptr);
                sp_last = temp_ptr;
            }

            /* Move to the next buffer in the link */
            temp_ptr = (UWORD32 *)get_tx_dscr_next_addr(lp_last);
			free_tx_dscr_buffer(g_local_dscr_mem_handle, lp_last);
            lp_last  = temp_ptr;
        }
        else
        {
            sp_last = lp_last;
            lp_last = (UWORD32 *)get_tx_dscr_next_addr(lp_last);
        }
    }
#else /* LOCALMEM_TX_DSCR */
    sp_last  = (UWORD32 *)*dscr_tail;
    lp_last  = NULL;
    dscr_cnt = *num_dscr;
#endif /* LOCALMEM_TX_DSCR */

    if(dscr_cnt == 0)
    {
		TROUT_FUNC_EXIT;
        return QMU_SHARED_MALLOC_FAILED;
	}
	
    *dscr_head = (UWORD8 *)lp_last;
    *num_dscr -= dscr_cnt;

    set_tx_dscr_next_addr(sp_last, 0);

#ifdef DEBUG_MODE
    g_mac_stats.qatxp += dscr_cnt;
    g_mac_stats.qa[q_num] += dscr_cnt;

    if(q_tx_struct->element_cnt == 0)
        g_mac_stats.qa0exc[q_num] += dscr_cnt;
#endif /* DEBUG_MODE */

#ifdef MEASURE_PROCESSING_DELAY
    g_delay_stats.txdscr2machwtxq += dscr_cnt;
    g_delay_stats.submsdu2machwtxq += dscr_cnt;
#endif /* MEASURE_PROCESSING_DELAY */

    /* Critical section beginning */
    //critical_section_start();
    mutex_lock(&tx_handle->txq_lock);   //Hugh

    /* Initialize a temporary pointer to the last element in the Tx Q */
    temp_ptr = q_tx_struct->element_tail;

    /* Increment the element count in the queue header */
    q_tx_struct->element_cnt += dscr_cnt;
    tx_handle->tx_list_count += dscr_cnt;

    /* If there are no packets in the current queue, update the head pointer */
    /* of the queue with the new packet descriptor.                          */
    if(q_tx_struct->element_head == NULL)
    {
        q_tx_struct->element_head = sp_head;
    }

    if(q_tx_struct->element_to_load == NULL)//add by Hugh
    {
        q_tx_struct->element_to_load = sp_head;    
    }

    /* If the last element is not NULL this is not the first packet. In this */
    /* case update the next element pointer of the previous descriptor with  */
    /* the new descriptor.                                                   */
    if(temp_ptr != NULL)
    {
        set_tx_dscr_next_addr(temp_ptr, (WORD32)sp_head);
    }

    /* Update the last element pointer with the last descriptor */
    q_tx_struct->element_tail = sp_last;


    /* If the Tx Q header is empty update the MAC H/w Q header register */
	    if((q_num < get_no_tx_queues()) && (HW_TXQ_ALL_IDLE()) &&
       //(is_all_machw_q_null() == BTRUE)  &&
       (tx_handle->tx_header[q_num].q_status != Q_SUSPENDED)){
        if(get_tx_dscr_status((UWORD32*)sp_head) == PENDING)
        {
            //set_machw_q_ptr(q_num, (UWORD32 *)sp_head);
            //qmu_cpy_to_trout(q_num, TX_SEND_CALL);  //Hugh
            qmu_cpy_to_trout_new(q_num, TX_SEND_CALL);  //Hugh
        }
    }

    /* End of critical section */
    //critical_section_end();
    mutex_unlock(&tx_handle->txq_lock);     //Hugh

    if(*num_dscr != 0)
    {
		TROUT_FUNC_EXIT;
        return QMU_SHARED_MALLOC_FAILED;
	}
	
	TROUT_FUNC_EXIT;
    return QMU_OK;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_sync_tx_packet                                       */
/*                                                                           */
/*  Description   : This function synchronizes the transmit queue with the   */
/*                  given transmit packet.                                   */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit queue handle                  */
/*                  2) Queue number                                          */
/*                  3) Pointer to the packet descriptor                      */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : Ideally the packet descriptor given at transmit complete */
/*                  should be at the head of the transmit queue. However if  */
/*                  synchronization is lost between S/w and H/w it is        */
/*                  possible that the given packet may not be at the head of */
/*                  the transmit queue. This function re-synchronizes with   */
/*                  H/w by locating the given packet in the specified queue  */
/*                  and deleting all the packets before the same. The queue  */
/*                  header is updated and the packet descriptor link list is */
/*                  modified so that the given packet is now the head of the */
/*                  transmit queue. Note that if the packet is not found the */
/*                  queue becomes empty and the head becomes NULL.           */
/*                                                                           */
/*  Outputs       : None                                                     */
/*  Returns       : None                                                     */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void qmu_sync_tx_packet(qmu_tx_handle_t* tx_handle, UWORD8 q_num, UWORD8 *dscr)
{
    UWORD32    *element_ptr   = 0;
    UWORD32    *next_elem_ptr = 0;
    q_struct_t *q_tx_struct   = &(tx_handle->tx_header[q_num]);
	TROUT_FUNC_ENTER;
#ifdef DEBUG_MODE
{
    UWORD32 temp = get_tx_dscr_frag_status_summary((UWORD32 *)dscr);
    if((temp & 0xFFFFFF00) != 0xFFFFFF00)
        g_mac_stats.qdexc++;

    if(q_tx_struct->element_cnt > 35)
        g_mac_stats.qn35exc[q_num]++;
    else if(q_tx_struct->element_cnt > 30)
        g_mac_stats.qn30exc[q_num]++;
    if(q_tx_struct->element_cnt > 25)
        g_mac_stats.qn25exc[q_num]++;
    else if(q_tx_struct->element_cnt > 20)
        g_mac_stats.qn20exc[q_num]++;
    else if(q_tx_struct->element_cnt > 15)
        g_mac_stats.qn15exc[q_num]++;
}
#endif /* DEBUG_MODE */

    /* Initialize the pointer to the transmit queue head */
    element_ptr = q_tx_struct->element_head;

    /* If the given packet is at the head of the transmit queue, as expected */
    /* no synchronization is required. Do nothing and return.                */
    if(element_ptr == (UWORD32 *)dscr)
    {
        //printk("trout: matched.\n");
        TROUT_FUNC_EXIT;
        return;
    }

    /* Critical section beginning */
    //critical_section_start();
    //printk("trout: sync:head=%p,dscr=%p.\n", element_ptr, dscr);
    mutex_lock(&tx_handle->txq_lock); //Hugh

    /* If the given packet is not at the head of the transmit queue, delete  */
    /* all packets from the head of the queue till the given packet is found */
    /* or the transmit queue becomes empty.                                  */
    while(element_ptr != NULL)
    {
#ifdef DEBUG_MODE
        g_mac_stats.qn_no_sync[q_num]++;
#endif /* DEBUG_MODE */

        /* Get the next element in the transmit queue */
        next_elem_ptr = (UWORD32 *)get_tx_dscr_next_addr(element_ptr);

        /* Decrement the element counter for this queue */
        q_tx_struct->element_cnt--;

        /* Free the transmit descriptor along with all associated buffers */
        free_tx_dscr((UWORD32 *)element_ptr);

        /* Proceed to the next packet in the queue */
        element_ptr = next_elem_ptr;

        /* If this is same as the given packet, all packets till the given   */
        /* packet have been deleted from the transmit queue. Break from loop.*/
        if(element_ptr == (UWORD32 *)dscr)
            break;

        /* Decrement count of total number of packets in all queues */
        tx_handle->tx_list_count--;
    }

    /* Update the head pointer of the queue to the given packet */
    q_tx_struct->element_head = element_ptr;

    /* If the element count for the transmit queue has become zero set the   */
    /* last element pointer to NULL. This is an excpetion that can happen if */
    /* the given packet was not found in the queue at all.                   */
    if(q_tx_struct->element_cnt == 0)
    {
#ifdef DEBUG_MODE
        g_mac_stats.qn_no_sync_pnf[q_num]++;
#endif /* DEBUG_MODE */

        q_tx_struct->element_tail = NULL;
    }

    /* End of critical section */
    //critical_section_end();
    mutex_unlock(&tx_handle->txq_lock);  //Hugh
    TROUT_FUNC_EXIT;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_del_tx_packet                                        */
/*                                                                           */
/*  Description   : This function deletes a packet from the head of the      */
/*                  specified transmit queue.                                */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit queue handle                  */
/*                  2) Queue number                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : The packet at the head of the queue is deleted and the   */
/*                  queue header is updated. The packet descriptor link list */
/*                  is also modified. The buffers associated with the head   */
/*                  descriptor are deleted as well.                          */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : QMU_OK on success                                        */
/*                  Failure code on failure                                  */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 qmu_del_tx_packet(qmu_tx_handle_t* tx_handle, UWORD8 q_num)
{
    UWORD32    *element_ptr   = 0;
    UWORD32    *next_elem_ptr = 0;
    q_struct_t *q_tx_struct   = &(tx_handle->tx_header[q_num]);

	TROUT_FUNC_ENTER;
    /* Initialize the element pointer to the head of the queue */
    element_ptr = q_tx_struct->element_head;

    /* If there is no element at the head return error code */
    if(element_ptr == NULL)
    {
		TROUT_FUNC_EXIT;
        return QMU_PACKET_NOT_FOUND;
	}
    /* Initialize the next element pointer to the next element in the queue */
    next_elem_ptr = (UWORD32 *)get_tx_dscr_next_addr(element_ptr);

    /* Critical section beginning */
    //critical_section_start();
    mutex_lock(&tx_handle->txq_lock);  //Hugh

    /* Decrement the element count for the transmit queue */
    q_tx_struct->element_cnt--;

    /* Decrement count of total number of packets in all queues */
    tx_handle->tx_list_count--;

    /* Set the transmit queue head to the next element in the queue */
    q_tx_struct->element_head = next_elem_ptr;

    /* If the element count for the transmit queue has become zero set the   */
    /* last element pointer to NULL                                          */
    if(q_tx_struct->element_cnt == 0)
    {
        q_tx_struct->element_tail = NULL;
        q_tx_struct->element_to_load = NULL;    //hugh
//        q_tx_struct->trout_head = 0;
    }

    /* End of critical section */
    //critical_section_end();
    mutex_unlock(&tx_handle->txq_lock);  //Hugh

    /* Free the buffers associated with the transmit descriptor */
    free_tx_dscr((UWORD32 *)element_ptr);

#ifdef DEBUG_MODE
    g_mac_stats.qdtxp++;
#endif /* DEBUG_MODE */

	TROUT_FUNC_EXIT;
    return QMU_OK;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_find_pending_dscr                                    */
/*                                                                           */
/*  Description   : This function returns the pointer to first descriptor    */
/*                  with status set to Pending .                             */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit queue handle                  */
/*                  2) Queue number                                          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses through the specified transmit      */
/*                  queue and returns the first descriptor with status set   */
/*                  to Pending .                                             */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : Pointer to first Pending TX-Dscr                         */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD32 *qmu_find_pending_dscr(qmu_tx_handle_t* tx_handle, UWORD8 q_num)
{
    UWORD32    *element_ptr   = 0;
    q_struct_t *q_tx_struct   = &(tx_handle->tx_header[q_num]);

    /* Initialize the pointer to the transmit queue head */
    element_ptr = q_tx_struct->element_head;

    if(element_ptr == (UWORD32 *)NULL)
        return NULL;

    /* If the given packet is not at the head of the transmit queue, delete  */
    /* all packets from the head of the queue till the given packet is found */
    /* or the transmit queue becomes empty.                                  */
    while(element_ptr != NULL)
    {
        if(get_tx_dscr_status((UWORD32*)element_ptr) == PENDING)
        {
            return element_ptr;
        }

        /* Get the next element in the transmit queue */
        element_ptr = (UWORD32 *)get_tx_dscr_next_addr(element_ptr);
    }

    return NULL;
}

/*****************************************************************************/
/*                                                                           */
/*  Function Name : qmu_update_txq_chan_bw                                   */
/*                                                                           */
/*  Description   : This function updates the Channel bandwidth and Channel  */
/*                  Mask fields in the PHY-TX mode field in the TX-Dscrs     */
/*                  enqueued in the specified TX-Queue.                      */
/*                  The MAC H/w should be suspended before this function is  */
/*                  called.                                                  */
/*                                                                           */
/*  Inputs        : 1) Pointer to the transmit queue handle                  */
/*                  2) Queue number                                          */
/*                  3) New Channel Mask                                      */
/*                  4) New Channel Bandwidth                                 */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function parses through the specified transmit      */
/*                  queue and updates the Channel bandwidth and Channel Mask */
/*                  fields in the PHY-TX mode field in the TX-Dscrs enqueued */
/*                  to the new values passed to this function.               */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : None                                                     */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

void qmu_update_txq_chan_bw(qmu_tx_handle_t* tx_handle, UWORD8 q_num,
                            UWORD8 ch_mask, UWORD8 ch_band)
{
    UWORD32    *element_ptr   = 0;
    q_struct_t *q_tx_struct   = &(tx_handle->tx_header[q_num]);

    /* Initialize the pointer to the transmit queue head */
    element_ptr = q_tx_struct->element_head;

    if(element_ptr == (UWORD32 *)NULL)
        return;

    /* If the given packet is not at the head of the transmit queue, delete  */
    /* all packets from the head of the queue till the given packet is found */
    /* or the transmit queue becomes empty.                                  */
    while(element_ptr != NULL)
    {
        update_tx_dscr_chan_bw(element_ptr, ch_mask, ch_band);

        /* Get the next element in the transmit queue */
        element_ptr = (UWORD32 *)get_tx_dscr_next_addr(element_ptr);
    }

    return;
}

#ifdef LOCALMEM_TX_DSCR

/*****************************************************************************/
/*                                                                           */
/*  Function Name : transfer_tx_dscr_local_to_shared                         */
/*                                                                           */
/*  Description   : This function transfers the given TX descriptor from     */
/*                  local memory to shared memory.                           */
/*                                                                           */
/*  Inputs        : 1) Pointer to the TX descriptor in Local memory          */
/*                                                                           */
/*  Globals       : None                                                     */
/*                                                                           */
/*  Processing    : This function allocates required TX descriptor buffers   */
/*                  in shared memory and transfers the required contents     */
/*                  from the given TX descriptor in local memory. Details of */
/*                  the contents to be transferred are given below.          */
/*                                                                           */
/*                  -------------------------------------------------------- */
/*                  | TX Descriptor buffer | LM: TX DSCR 0 | SM: TX DSCR 0 | */
/*                  | Sub-MSDU Info buffer | LM: TX DSCR 0 | SM: TX DSCR 0 | */
/*                  | Num-SubMSDU          | = 1           | = 1           | */
/*                  -------------------------------------------------------- */
/*                  Copy (TX_DSCR_LEN + MIN_SUBMSDU_TABLE_SIZE) bytes from   */
/*                  local memory TX DSCR 0 to shared memory TX DSCR 0        */
/*                                                                           */
/*                  -------------------------------------------------------- */
/*                  | TX Descriptor buffer | LM: TX DSCR 0 | SM: TX DSCR 0 | */
/*                  | Sub-MSDU Info buffer | LM: TX DSCR 1 | SM: TX DSCR 0 | */
/*                  | Num-SubMSDU          | = 1           | = 1           | */
/*                  -------------------------------------------------------- */
/*                  Copy (TX_DSCR_LEN + MIN_SUBMSDU_TABLE_SIZE) bytes from   */
/*                  local memory TX DSCR 0 to shared memory TX DSCR 0        */
/*                  and (SUB_MSDU_ENTRY_LEN) bytes from local memory         */
/*                  TX DSCR 1 to shared memory TX DSCR 0 + TX_DSCR_LEN       */
/*                                                                           */
/*                  -------------------------------------------------------- */
/*                  | TX Descriptor buffer | LM: TX DSCR 0 | SM: TX DSCR 0 | */
/*                  | Sub-MSDU Info buffer | LM: TX DSCR 1 | SM: TX DSCR 1 | */
/*                  | Num-SubMSDU          | > 1           | > 1           | */
/*                  -------------------------------------------------------- */
/*                  Copy (TX_DSCR_LEN + MIN_SUBMSDU_TABLE_SIZE) bytes from   */
/*                  local memory TX DSCR 0 to shared memory TX DSCR 0        */
/*                  and (SUB_MSDU_ENTRY_LEN * num_dscr) bytes from local     */
/*                  memory TX DSCR 1 to shared memory TX DSCR 1              */
/*                                                                           */
/*                  -------------------------------------------------------- */
/*                  | TX Descriptor buffer | LM: TX DSCR 0 | SM: TX DSCR 0 | */
/*                  | Sub-MSDU Info buffer | All other combinations are    | */
/*                  | Num-SubMSDU          | INVALID                       | */
/*                  -------------------------------------------------------- */
/*                  No processing required.                                  */
/*                                                                           */
/*  Outputs       : None                                                     */
/*                                                                           */
/*  Returns       : UWORD8 *; Pointer to the TX descriptor in Shared memory  */
/*                            with all fields set as required                */
/*                                                                           */
/*  Issues        : None                                                     */
/*                                                                           */
/*****************************************************************************/

UWORD8 *transfer_tx_dscr_local_to_shared(UWORD8 *local_dscr)
{
    UWORD8 *shared_dscr = NULL;
// 20120709 caisf add, merged ittiam mac v1.2 code
#ifdef LOCALMEM_TX_DSCR
    UWORD8  *buff_hdl = NULL;
#endif /* LOCALMEM_TX_DSCR */

	TROUT_FUNC_ENTER;
#ifdef SEPARATE_AMSDU_TX_DSCR_SUB_POOL
	UWORD16 tx_dscr_size = 0;
	UWORD32 num_sub_msdu = get_tx_dscr_num_submsdu((UWORD32 *)local_dscr);

	if(num_sub_msdu == 1)
		tx_dscr_size = TX_DSCR_LEN + MIN_SUBMSDU_TABLE_SIZE;
	else
		tx_dscr_size = TX_DSCR_LEN + MAX_SUBMSDU_TABLE_SIZE;

	shared_dscr = (UWORD8 *)mem_alloc(g_shared_dscr_mem_handle, tx_dscr_size);

	if(NULL == shared_dscr)
	{
		TROUT_FUNC_EXIT;
		return NULL;
	}

#ifdef USE_PROCESSOR_DMA
	mem_dma(shared_dscr, local_dscr, tx_dscr_size, LOCAL_TO_SHARED);
#else /* USE_PROCESSOR_DMA */
	memcpy(shared_dscr, local_dscr, tx_dscr_size);
#endif /* USE_PROCESSOR_DMA */

	/* Update the pointer to Sub-MSDU information table */
	set_tx_dscr_submsdu_info((UWORD32 *)shared_dscr,
	                         (UWORD32)shared_dscr + TX_DSCR_LEN);

#else /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */
	UWORD8  num_sub_msdu  = get_tx_dscr_num_submsdu((UWORD32 *)local_dscr);
	UWORD8  is_amsdu      = (1 == num_sub_msdu) ? 0 : 1;
    UWORD16 transfer_size = TX_DSCR_LEN + MIN_SUBMSDU_TABLE_SIZE;
    UWORD32 local_sub_msdu_info  = 0;
    UWORD32 shared_sub_msdu_info = 0;

	/* Allocate buffer for transmit descriptor and reset required fields */
	shared_dscr = allocate_tx_dscr(g_shared_dscr_mem_handle, is_amsdu);

	if(NULL == shared_dscr)
	{
		TROUT_FUNC_EXIT;
		return NULL;
	}

	/* Save sub-MSDU information table pointer from the shared descriptor */
	shared_sub_msdu_info = get_tx_dscr_submsdu_info((UWORD32 *)shared_dscr);

	/* Copy contents of the TX descriptor from local to shared memory */
#ifdef USE_PROCESSOR_DMA
	mem_dma(shared_dscr, local_dscr, transfer_size, LOCAL_TO_SHARED);
#else /* USE_PROCESSOR_DMA */
	memcpy(shared_dscr, local_dscr, transfer_size);
#endif /* USE_PROCESSOR_DMA */

	/* Restore sub-MSDU information table pointer value from saved info */
	set_tx_dscr_submsdu_info((UWORD32 *)shared_dscr, shared_sub_msdu_info);

    /* Get the sub-MSDU information table pointer from the local descriptor */
    local_sub_msdu_info = get_tx_dscr_submsdu_info((UWORD32 *)local_dscr);

    /* If the local sub-MSDU information table pointer indicates a separate  */
    /* buffer, contents must be copied to the appropriate shared sub-MSDU    */
    /* information table buffer.                                             */
    if(local_sub_msdu_info != ((UWORD32)(local_dscr + TX_DSCR_LEN)))
    {
	    /* Size of the sub-MSDU table to be transferred depends on the       */
		/* number of sub-MSDUs                                               */
		transfer_size = num_sub_msdu * SUB_MSDU_ENTRY_LEN;

		/* Copy sub-MSDU information table from local to shared memory */
#ifdef USE_PROCESSOR_DMA
		mem_dma((UWORD8 *)shared_sub_msdu_info, (UWORD8 *)local_sub_msdu_info,
		        transfer_size, LOCAL_TO_SHARED);
#else /* USE_PROCESSOR_DMA */
		memcpy((UWORD8 *)shared_sub_msdu_info, (UWORD8 *)local_sub_msdu_info,
		       transfer_size);
#endif /* USE_PROCESSOR_DMA */
	}

#endif /* SEPARATE_AMSDU_TX_DSCR_SUB_POOL */

	// 20120709 caisf add, merged ittiam mac v1.2 code
    /* Handle BAR Tx DSCR where the contents of the buffer are within the Tx */
    /* descriptor itself. This is required when Tx descriptor is created in  */
    /* local memory                                                          */
    buff_hdl = (UWORD8*)get_tx_dscr_buffer_addr((UWORD32*)local_dscr);
    if(BTRUE == is_buffer_in_tx_dscr(local_dscr, buff_hdl))
    {
        update_bar_buff_addr_txdscr((UWORD32 *)shared_dscr,
                                    (UWORD32 *)shared_sub_msdu_info);
    }
	
	TROUT_FUNC_EXIT;
	/* Return the shared descriptor */
	return shared_dscr;
}

#endif /* LOCALMEM_TX_DSCR */
