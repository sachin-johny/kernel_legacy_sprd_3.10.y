/*******************************************************************************
* Copyright (c) 2011-2012,Spreadtrum Corporation
* All rights reserved.
* Filename: trout_wifi_rx.c
* Abstract: 
* 
* Version: 1.0
* Revison Log: 
* 	2012/5/08, Chengwg: Create this file.
* CVS Log: 
* 	$Id$
*******************************************************************************/
#include <linux/spinlock.h>
#include <linux/netdevice.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include "maccontroller.h"
#include "proc_if.h"
#include "cglobals.h"
#include "mh.h"
#include "qmu_if.h"
#include "trout_wifi_rx.h"
#include "spi_interface.h"

#include "trout_trace.h"
#include "qmu_tx.h"

#ifdef OS_LINUX_CSL_TYPE
#include "csl_linux.h"
#endif /* OS_LINUX_CSL_TYPE */

#ifdef TROUT_WIFI

/* For High-throughput & security reasion, we shield this two macro, beacuse after */
/* open this two macro,  block-write 4*6 byte is slower than original ram write    */
//#define USE_NEW_CLEAR			1
//#define USE_BLOCK_WRITE			1

rx_queue_handle *g_rx_handle = NULL;

//UWORD16 g_normal_rxq_num = DFT_NUM_RX_BUFFS;
#ifdef SYS_RAM_256M
UWORD16 g_normal_rxq_num = 56;	//Hugh: 56 improve TX performance, 64 is original value. trout share ram 144kb, tx ram 32kb.
//UWORD16 g_normal_rxq_num = 34;	//trout share ram 80kb.
#else
UWORD16 g_normal_rxq_num = 34;	//test.  add buff by zhao
#endif
extern UWORD32 g_rx_pkt_count;
extern UWORD32 g_rx_times;

#ifdef IBSS_BSS_STATION_MODE
extern BOOL_T g_wifi_bt_coex;
extern COEX_TRFFC_CHK_T g_coex_trffc_chk;
#endif


static void rx_complete_work(struct work_struct *work);
#define MIN_RX_HOST_MEM		(72 * 1024)

#if 0 //dumy add for debug 0815
#undef TROUT_RX_DBG
#else
#define TROUT_RX_DBG 
#endif

/************************ debug info is defined here ***************************/
#ifdef TROUT_RX_DBG
void hexdump(UWORD8 *buf, UWORD32 len)
{
	UWORD32 i;

	for(i=0; i<len; i++)
	{
		printk("%02x ", buf[i]);
		if((i+1)%16 == 0)
			printk("\n");
	}
}


void print_trout_dscr_info(UWORD32 dscr)
{
	trout_rx_mem_struct *tmp_dscr = (trout_rx_mem_struct *)dscr;
	UWORD8 buf[40];

	host_read_trout_ram((void *)buf, (void *)tmp_dscr->trout_rx_dscr, 40);
	printk("trout dscr 0x%x info:\n", dscr);
	hexdump(buf, 40);
}

void print_trout_rx_dscr_cfg_info(int priority, UWORD32 *base_dscr)
{
	trout_rx_mem_struct *trout_mem = (trout_rx_mem_struct *)base_dscr;
	UWORD32 prev_dscr, next_dscr;
	UWORD32 num = 0, i;
	UWORD32 status, rst;
		
	if(priority == NORMAL_PRI_RXQ)
		num = g_normal_rxq_num;
	else if(priority == HIGH_PRI_RXQ)
		num = NUM_HIPR_RX_BUFFS;

	printk(KERN_INFO "trout pri%d queue cfg info:\n", priority);
	for(i=0; i<num; i++)
	{
		next_dscr = 0;
		prev_dscr = 0;
		
		host_read_trout_ram(&next_dscr, (void *)&trout_mem->trout_rx_dscr[RX_NEXT_DSCR_POINT_OFFSET * sizeof(UWORD32)], sizeof(UWORD32));
		host_read_trout_ram(&prev_dscr, (void *)&trout_mem->trout_rx_dscr[RX_PREV_DSCR_POINT_OFFSET * sizeof(UWORD32)], sizeof(UWORD32));
		host_read_trout_ram((void *)&status, (void *)trout_mem, 4);	//dbug.
		rst = get_rx_dscr_pkt_status((UWORD32 *)&status);
		printk(KERN_INFO "[%d]dscr 0x%08x: prev dscr = 0x%08x, next dscr = 0x%08x, status=%u\n", i, (UWORD32)trout_mem, prev_dscr, next_dscr, rst);

		if(next_dscr == 0 /*&& i < num-1*/)
		{
			printk(KERN_INFO "trout cfg display incomplete!\n");
			break;
		}

		trout_mem = (trout_rx_mem_struct *)next_dscr;
	}
}


void trout_rx_mem_show(void)
{
	int i;
	void *rx_desc_offset = (void *)NORMAL_RX_MEM_BEGIN;
	UWORD32 status, next, prev, rst;

	for(i=0; i<g_normal_rxq_num; i++)
    {
        host_read_trout_ram((void *)&status, (void *)rx_desc_offset, 4);
        rst = get_rx_dscr_pkt_status(&status);

        host_read_trout_ram(&next, &rx_desc_offset[32], sizeof(UWORD32));
		host_read_trout_ram(&prev, &rx_desc_offset[36], sizeof(UWORD32));

		printk("[%2d]dscr: 0x%p, prev: 0x%x, next: 0x%x, status: %u\n", i, rx_desc_offset, prev, next, rst);

        rx_desc_offset += sizeof(trout_rx_mem_struct);
    }

	rx_desc_offset = (void *)HIGH_RX_MEM_BEGIN;
	for(i=0; i < 5; i++)
    {
        host_read_trout_ram((void *)&status, (void *)rx_desc_offset, 4);
        rst = get_rx_dscr_pkt_status(&status);

        host_read_trout_ram(&next, &rx_desc_offset[32], sizeof(UWORD32));
		host_read_trout_ram(&prev, &rx_desc_offset[36], sizeof(UWORD32));

		printk("[%2d]dscr: 0x%p, prev: 0x%x, next: 0x%x, status: %u\n", i, rx_desc_offset, prev, next, rst);

        rx_desc_offset += sizeof(trout_rx_mem_struct);
    }



}

void print_trout_rx_queue(UWORD32 cur_dscr)
{
	trout_rx_mem_struct *trout_mem = (trout_rx_mem_struct *)cur_dscr;
	UWORD32 next_dscr = 0;

	if(trout_mem == NULL)
		return;
	
	printk("0x%x->", cur_dscr);
	do{
		host_read_trout_ram(&next_dscr, (void *)&trout_mem->trout_rx_dscr[RX_NEXT_DSCR_POINT_OFFSET * sizeof(UWORD32)], sizeof(UWORD32));

		if(next_dscr)
			printk("0x%x->", next_dscr);
		else
			printk("0x%x\n", next_dscr);

	}while((next_dscr != 0) && (trout_mem = (trout_rx_mem_struct *)next_dscr));
}


void print_host_rx_queue_info(rx_queue_struct *rx_ctrl)
{
	trout_rx_mem_struct *tmp_ptr = NULL;
	UWORD32 status = 0, *p;
	int i, v;
	
	printk("------------- host_rx_queue info start(%d) -------------------\n", rx_ctrl->queue_pri);
	tmp_ptr = (trout_rx_mem_struct *)rx_ctrl->queue_head;
	for(i=0; i<rx_ctrl->cell_num; i++)
	{
		p = (UWORD32 *)tmp_ptr->trout_rx_dscr;
		p += 10;
		v = *(volatile UWORD32 *)p;
		status = get_rx_dscr_pkt_status((UWORD32 *)(tmp_ptr->trout_rx_dscr));
		printk("%2d: %u-%X", i, status, v);
		if((i+1)%8 == 0)
			printk("\n");
			
		tmp_ptr++;
	}
	if((i+1)%10 != 0)
		printk("\n");
	printk("------------- host_rx_queue info end(%d) ----------------------\n", rx_ctrl->queue_pri);
}

void print_host_rx_dscr_and_buf_content(UWORD32 *base_dscr, UWORD8 num_dscr)
{
	trout_rx_mem_struct *cur_rx_mem = (trout_rx_mem_struct *)base_dscr;
	trout_rx_mem_struct *next_rx_mem;
	UWORD8 *buf = NULL;
	UWORD32 len, i;

	for(i=0; i<num_dscr; i++)
	{
		printk("==========================================================\n");
		buf = cur_rx_mem->trout_rx_dscr;
		len = RX_DSCR_LEN;
		printk("rx_dscr: 0x%p\n", buf);
		hexdump(buf, len);
		printk("\n----------------------------------------------------------\n");
		buf = cur_rx_mem->trout_rx_buf;
		len = get_rx_dscr_frame_len((UWORD32 *)cur_rx_mem);
		hexdump(buf, len);
		printk("\n");
		next_rx_mem = (trout_rx_mem_struct *)get_rx_dscr_next_dscr((UWORD32 *)cur_rx_mem->trout_rx_dscr);
		if(next_rx_mem == NULL)
			break;
		cur_rx_mem = next_rx_mem;
	}	
}

//be_af: before=False, after=True;
void print_host_rx_buf_content(UWORD32 *base_dscr, UWORD8 num_dscr, BOOL_T be_af)
{
	trout_rx_mem_struct *cur_rx_mem = (trout_rx_mem_struct *)base_dscr;
	trout_rx_mem_struct *next_rx_mem;
	UWORD8 *buf = NULL;
	UWORD32 len, i;

	UWORD8 *dst_addr, *bssid, *source;
	UWORD8 broad[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	BOOL_T is_amsdu, is_start_buf;
	UWORD8 num_buffers, num_msdus;	
	
	printk("base: 0x%p, num: %d\n", base_dscr, num_dscr);
	for(i=0; i<num_dscr;)
	{
		if(!be_af)
		{
			if(get_rx_dscr_pkt_status((UWORD32 *)cur_rx_mem) != RX_NEW)
				printk("rx dscr status is valid!\n");
		}
		
		if(be_af)
		{
			dst_addr = &cur_rx_mem->trout_rx_buf[4];
			bssid = &cur_rx_mem->trout_rx_buf[10];
			source = &cur_rx_mem->trout_rx_buf[16];

			is_amsdu = is_rx_frame_amsdu((UWORD32 *)cur_rx_mem);
			is_start_buf = is_rx_frame_start_buf((UWORD32 *)cur_rx_mem);
			num_buffers = get_rx_dscr_num_buffers((UWORD32 *)cur_rx_mem);
			num_msdus = get_rx_dscr_num_msdus((UWORD32 *)cur_rx_mem);

			if((memcmp(dst_addr, mget_StationID(), 6) == 0) || (memcmp(dst_addr, broad, 6) == 0)
					|| (memcmp(bssid, mget_bssid(), 6) == 0))
				goto next;
		
			printk("==========================================================\n");
			buf = cur_rx_mem->trout_rx_dscr;
			len = RX_DSCR_LEN;
			printk("rx_dscr: 0x%p\n", buf);
			printk("is_amsdu=%d, is_start_buf=%d, nun_buffers=%d, num_msdu=%d\n", 
							is_amsdu, is_start_buf, num_buffers, num_msdus);
			hexdump(buf, len);
			printk("\n----------------------------------------------------------\n");
			buf = cur_rx_mem->trout_rx_buf;
			len = get_rx_dscr_frame_len((UWORD32 *)cur_rx_mem);
			printk("rx_buffer: 0x%p, len=%d\n", buf, len);
			hexdump(buf, 1600);
			printk("\n");
		}
		
next:
		if(++i >= num_dscr)
			break;
		
		next_rx_mem = ++cur_rx_mem;
		
		cur_rx_mem = next_rx_mem;
	}	
}

int host_rx_pkt_analysis(void *wlan_rx)
{
	wlan_rx_event_msg_t *event  = (wlan_rx_event_msg_t*)wlan_rx;
	trout_rx_mem_struct *rx_str = (trout_rx_mem_struct *)event->base_dscr;
	UWORD8 num_dscr = event->num_dscr;
	UWORD8 q_num = event->rxq_num;
	UWORD8 rx_status = 0;
	UWORD8 *pkt = NULL;
	UWORD8 type, subtype;
	UWORD8 *dest, *src, *bssid;

	if(g_rx_dbg_flag == 3)
		printk("base_dscr=0x%p, num_dscr=%d, pri=%d\n", rx_str, num_dscr, q_num);
	
	while(num_dscr != 0)
	{
		rx_status = get_rx_dscr_pkt_status((UWORD32 *)(rx_str->trout_rx_dscr));
		if(rx_status != RX_SUCCESS)
		{
			printk("rx pkt status err: %d\n", rx_status);
			goto out;
		}

		pkt = rx_str->trout_rx_buf;
		if((pkt[0] & 0x03) != PROTOCOL_VERSION)
		{
			printk("protocol version err!\n");
			goto out;
		}

		type = (pkt[0] >> 0x02) & 0x03;
		subtype = (pkt[0] >> 0x0F) & 0x0F;

		if(type == 0x00)
			rx_mgmt_frame_count[q_num]++;
		else if(type == 0x01)
			rx_ctrl_frame_count[q_num]++;
		else if(type == 0x02)
			rx_data_frame_count[q_num]++;
		else
			rx_rsev_frame_count[q_num]++;
		
		dest = pkt + 4;
		src = pkt + 10;
		bssid = pkt + 16;

		if(g_rx_dbg_flag == 3)
		{
			printk("type=0x%x, subtype=0x%x\n", type, subtype);
			printk("dest:  %02X:%02X:%02X:%02X:%02X:%02X\n", dest[0], dest[1], dest[2], dest[3], dest[4], dest[5]);
			printk("src:   %02X:%02X:%02X:%02X:%02X:%02X\n", src[0], src[1], src[2], src[3], src[4], src[5]);
			printk("bssid: %02X:%02X:%02X:%02X:%02X:%02X\n", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
		}
out:
		num_dscr -= 1;
		rx_str = (trout_rx_mem_struct *)get_rx_dscr_next_dscr((UWORD32 *)rx_str);
	}

	return 0;
}

#endif

void create_trout_default_share_mem_cfg(void)
{
	void *trout_cfg_start = (void *)TROUT_MEM_CFG_BEGIN;
	UWORD8 cfg_buf[TROUT_MEM_CFG_SIZE];
	UWORD16 *data = (UWORD16 *)cfg_buf;
	int len = 0;
	
	memset(cfg_buf, '\0', sizeof(cfg_buf));
	
	*((UWORD32 *)cfg_buf) = TROUT_MEM_MAGIC;
	len += 4;

	data = (UWORD16 *)(&cfg_buf[4]);
	
	*data++ = RX_DSCR_LEN & 0xFFFF;
	len += 2;

	*data++ = TX_DSCR_LEN & 0xFFFF;
	len += 2;

	*data++ = RX_PACKET_SIZE & 0xFFFF;
	len += 2;

	*data++ = BEACON_MEM_SIZE & 0xFFFF;
	len += 2;

	*data++ = NUM_HIPR_RX_BUFFS & 0xFFFF;
	len += 2;

	*data++ = g_normal_rxq_num & 0xFFFF;
	len += 2;

	printk("save trout mem cfg...\n");
	host_write_trout_ram(trout_cfg_start, cfg_buf, len);
}

static trout_rx_mem_struct *move_to_next(rx_queue_struct *rx_ctrl, trout_rx_mem_struct *p)
{
	trout_rx_mem_struct *t = p;

	if((t > (UWORD32)rx_ctrl->trout_tail)
			|| (t < (UWORD32)rx_ctrl->trout_head)){
		printk("[libing]INVALID POINTER @ Func:%s..Line:%d\n", __func__, __LINE__);
		return (trout_rx_mem_struct *)rx_ctrl->trout_head;
	}

	t++;
	if(t > rx_ctrl->trout_tail)
		t = rx_ctrl->trout_head;
	return t;
}

static void set_rx_queue_struct(UWORD32 *host_base, UWORD32 *trout_base, UWORD8 num_dscr)
{
	trout_rx_mem_struct *host_mem = (trout_rx_mem_struct *)host_base;
	trout_rx_mem_struct *trout_mem = (trout_rx_mem_struct *)trout_base;
	UWORD32 *dscr_addr, trout_buf_addr, trout_prev_dscr_addr, trout_next_dscr_addr;
	UWORD8 i;

	for(i=0; i<num_dscr; i++)
	{
		dscr_addr = (UWORD32 *)host_mem[i].trout_rx_dscr;
		trout_buf_addr = (UWORD32)trout_mem[i].trout_rx_buf;
		reset_rx_dscr(dscr_addr);

		if(i == num_dscr - 1)	/* tail dscr's next dscr is null */
			trout_next_dscr_addr = 0;
		else
			trout_next_dscr_addr = (UWORD32)trout_mem[i+1].trout_rx_dscr;

		if(i == 0)	/* head dscr's previous dscr is null */
			trout_prev_dscr_addr = 0;
		else
			trout_prev_dscr_addr = (UWORD32)trout_mem[i-1].trout_rx_dscr;

		set_rx_dscr_buffer_ptr(dscr_addr, trout_buf_addr);		//set dscr->buf.
		set_rx_dscr_next_dscr(dscr_addr, trout_next_dscr_addr);	//set dscr->next_dscr
		set_rx_dscr_prev_dscr(dscr_addr, trout_prev_dscr_addr);	//set dscr->prev_dscr.
	}
}


static int init_trout_rx_queue(UWORD32 *host_mem, UWORD32 size)
{
	UWORD32 *t_mem_base, *h_mem_base;

	h_mem_base = host_mem;

	if((sizeof(trout_rx_mem_struct) * (NUM_HIPR_RX_BUFFS + g_normal_rxq_num)) > size)
	{
		TROUT_DBG1("init trout rxq fail: host mem not enough!\n");
		return -1;
	}
		
	/* initial trout high priority rx queue */
	t_mem_base = (UWORD32 *)HIGH_RX_MEM_BEGIN;
	set_rx_queue_struct(h_mem_base, t_mem_base, NUM_HIPR_RX_BUFFS);
	h_mem_base += sizeof(trout_rx_mem_struct) * NUM_HIPR_RX_BUFFS;

	/* initial trout normal priority rx queue */
	t_mem_base = (UWORD32 *)NORMAL_RX_MEM_BEGIN;
	set_rx_queue_struct(h_mem_base, t_mem_base, g_normal_rxq_num);
	
	return 0;
}


int init_trout_high_rx_queue(UWORD32 *h_rx_mem_s)
{
	UWORD32 *t_rx_mem_s, *h_tmp_mem;

	if(h_rx_mem_s == NULL)
	{
		TROUT_RX_DBG1("%s: invalid parameters!\n", __func__);
		return -1;
	}
	
	h_tmp_mem = h_rx_mem_s + 2;	//reserved 2word for spi cmd.
	t_rx_mem_s = (UWORD32 *)HIGH_RX_MEM_BEGIN;
	set_rx_queue_struct(h_tmp_mem, t_rx_mem_s, NUM_HIPR_RX_BUFFS);
	host_write_trout_ram((void *)t_rx_mem_s, (void *)h_tmp_mem, 
								(sizeof(trout_rx_mem_struct) * NUM_HIPR_RX_BUFFS));
	set_machw_rx_buff_addr((UWORD32)t_rx_mem_s, HIGH_PRI_RXQ);
	TROUT_RX_DBG1("high rxq init, rx buff addr: 0x%p\n", t_rx_mem_s);

	return 0;
}

int init_trout_normal_rx_queue(UWORD32 *h_rx_mem_s)
{
	UWORD32 *t_rx_mem_s, *h_tmp_mem;

	if(h_rx_mem_s == NULL)
	{
		TROUT_RX_DBG1("%s: invalid parameters!\n", __func__);
		return -1;
	}

	h_tmp_mem = h_rx_mem_s + 2;	//reserved 2word for spi cmd.
	t_rx_mem_s = (UWORD32 *)NORMAL_RX_MEM_BEGIN;
	set_rx_queue_struct(h_tmp_mem, t_rx_mem_s, g_normal_rxq_num);

	host_write_trout_ram((void *)t_rx_mem_s, (void *)h_tmp_mem, 
									(sizeof(trout_rx_mem_struct) * g_normal_rxq_num));
	
	set_machw_rx_buff_addr((UWORD32)t_rx_mem_s, NORMAL_PRI_RXQ);
	TROUT_RX_DBG1("normal rxq init, rx buff addr: 0x%p\n", t_rx_mem_s);

	return 0;
}

/* clear host rx queue dscr's status bits */
static void init_host_rx_queue(UWORD32 *dscr_base, UWORD8 num_dscr)
{
    trout_rx_mem_struct *tmp_dscr = (trout_rx_mem_struct *)dscr_base;
	UWORD8 i;
	UWORD32 *p;

	for(i=0; i<num_dscr; i++)
	{
		p = (UWORD32 *)tmp_dscr[i].trout_rx_dscr;
		clean_rx_dscr_pkt_status(p);
		p += 10;
		*p = 0;
	}
}

#if 0
int init_mac_rx_queue(rx_queue_handle *rx_q_handle)
{
	rx_queue_struct *normal_pri_q, *high_pri_q;
	UWORD32 trout_rx_base, size, *host_mem, *head_dscr;
	trout_rx_q_struct *trout_rx_q = NULL;
	
	TROUT_FUNC_ENTER;
	if(g_mac_dev == NULL)
	{
		TROUT_FUNC_EXIT;
		return (-EINVAL);
	}
	
	if(rx_q_handle == NULL)
	{
		rx_q_handle = (rx_queue_handle *)kmalloc(sizeof(rx_queue_handle), GFP_KERNEL);
		if(rx_q_handle == NULL)
		{
			TROUT_DBG2("%s: alloc rx_queue_handle space failed!\n", __FUNCTION__);
			TROUT_FUNC_EXIT;
			return (-ENOMEM);
		}

		memset(rx_q_handle, '\0', sizeof(rx_queue_handle));
		g_rx_handle = rx_q_handle;
	}

	normal_pri_q = &rx_q_handle->rx_q_info[NORMAL_PRI_RXQ];
	high_pri_q = &rx_q_handle->rx_q_info[HIGH_PRI_RXQ];

	if(normal_pri_q->queue_head == NULL)
	{
		size = sizeof(trout_rx_mem_struct) * NUM_RX_BUFFS * HOST_RX_Q_NUM;
		normal_pri_q->queue_head = (UWORD32 *)kmalloc(size, GFP_KERNEL);
		if(normal_pri_q->queue_head == NULL)
		{
			TROUT_DBG2("alloc host rx normal priority queue failed!\n");
			goto err0;
		}
	}

	if(high_pri_q->queue_head == NULL)
	{
		size = sizeof(trout_rx_mem_struct) * NUM_HIPR_RX_BUFFS * HOST_RX_Q_NUM;
		high_pri_q->queue_head = (UWORD32 *)kmalloc(size, GFP_KERNEL);
		if(high_pri_q->queue_head == NULL)
		{
			TROUT_DBG2("alloc host rx high priority queue failed!\n");
			goto err1;
		}
	}
	
	TROUT_DBG4("normal_rx_q: %p, high_rx_q: %p\n", normal_pri_q->queue_head, high_pri_q->queue_head);

	/* reserved 2 WORD for spi ram write cmd & dscr */
	host_mem = (UWORD32 *)(normal_pri_q->queue_head) + 2;
	
	trout_rx_base = RX_MEM_BEGIN;
	init_trout_rx_queue(host_mem, trout_rx_base);
	
	/* write configs to trout rx mem for initial trout rx dscr and buffer */
	host_write_trout_ram((void *)trout_rx_base, (void *)host_mem, sizeof(trout_rx_q_struct));

	trout_rx_q = (trout_rx_q_struct *)trout_rx_base;

	/* normal priority rx queue initial... */
	normal_pri_q->queue_pri = NORMAL_PRI_RXQ;
	normal_pri_q->cell_num = NUM_RX_BUFFS * HOST_RX_Q_NUM;
	normal_pri_q->queue_tail = (UWORD32 *)((trout_rx_mem_struct *)(normal_pri_q->queue_head) + 
												normal_pri_q->cell_num - 1);
	normal_pri_q->cur_ptr = normal_pri_q->queue_head;
	normal_pri_q->trout_head = (UWORD32 *)trout_rx_q->trout_normal_pri_rx_q;
    normal_pri_q->trout_tail = (UWORD32 *)(&trout_rx_q->trout_normal_pri_rx_q[NUM_RX_BUFFS - 1]);
    normal_pri_q->trout_cur_ptr = normal_pri_q->trout_head;
	normal_pri_q->trout_q_tail = (trout_rx_mem_struct *)normal_pri_q->trout_tail;
	
	INIT_LIST_HEAD(&normal_pri_q->rx_list);
	INIT_WORK(&normal_pri_q->work, rx_complete_work);
	spin_lock_init(&normal_pri_q->spin_lock);
	init_host_rx_queue(normal_pri_q->queue_head, normal_pri_q->cell_num);
	
	//high priority rx queue initial...	
	high_pri_q->queue_pri = HIGH_PRI_RXQ;
	high_pri_q->cell_num = NUM_HIPR_RX_BUFFS * HOST_RX_Q_NUM;
	high_pri_q->queue_tail = (UWORD32 *)((trout_rx_mem_struct *)(high_pri_q->queue_head) + 
													high_pri_q->cell_num - 1);
	high_pri_q->cur_ptr = high_pri_q->queue_head;
	high_pri_q->trout_head = (UWORD32 *)trout_rx_q->trout_high_pri_rx_q;
	high_pri_q->trout_tail = (UWORD32 *)(&trout_rx_q->trout_high_pri_rx_q[NUM_HIPR_RX_BUFFS - 1]);
	high_pri_q->trout_cur_ptr = high_pri_q->trout_head;
	high_pri_q->trout_q_tail = (trout_rx_mem_struct *)high_pri_q->trout_tail;

	INIT_LIST_HEAD(&high_pri_q->rx_list);
	INIT_WORK(&high_pri_q->work, rx_complete_work);
	spin_lock_init(&high_pri_q->spin_lock);
	init_host_rx_queue(high_pri_q->queue_head, high_pri_q->cell_num);;

	if(rx_q_handle->rx_work_q == NULL)
	{
		rx_q_handle->rx_work_q = create_singlethread_workqueue("rx_workqueue");
		if(rx_q_handle->rx_work_q == NULL)
			TROUT_DBG2("create work queue failed!\n");
	}

	head_dscr = (UWORD32 *)trout_rx_q->trout_normal_pri_rx_q->trout_rx_dscr;

	if(is_machw_rx_buff_null(NORMAL_PRI_RXQ) == BTRUE)
		set_machw_rx_buff_addr((UWORD32)head_dscr, NORMAL_PRI_RXQ);

	head_dscr = (UWORD32 *)trout_rx_q->trout_high_pri_rx_q->trout_rx_dscr;

	if(is_machw_rx_buff_null(HIGH_PRI_RXQ) == BTRUE)
		set_machw_rx_buff_addr((UWORD32)head_dscr, HIGH_PRI_RXQ);

	TROUT_FUNC_EXIT;
	return 0;
	
err1:
	if(normal_pri_q->queue_head != NULL)
	{
		kfree(normal_pri_q->queue_head);
		normal_pri_q->queue_head = NULL;
	}
	
err0:
	if(rx_q_handle != NULL)
	{
		kfree(rx_q_handle);
		rx_q_handle = NULL;
	}

	TROUT_FUNC_EXIT;
	return -1;
}
#endif

int init_mac_rx_queue(rx_queue_handle *rx_q_handle)
{
	rx_queue_struct *normal_pri_q, *high_pri_q;
	UWORD32 size;
	trout_rx_mem_struct *trout_rx_q = NULL;
	//UWORD32 host_mem_size = 0;
	rx_int_info *rx_info, *tmp;
	
	TROUT_FUNC_ENTER;
	
	if(g_mac_dev == NULL)
	{
		TROUT_FUNC_EXIT;
		return (-EINVAL);
	}
	
	if(rx_q_handle == NULL)
	{
		rx_q_handle = (rx_queue_handle *)kzalloc(sizeof(rx_queue_handle), GFP_KERNEL);
		if(rx_q_handle == NULL)
		{
			TROUT_DBG2("%s: alloc rx_queue_handle space failed!\n", __FUNCTION__);
			TROUT_FUNC_EXIT;
			return (-ENOMEM);
		}

		g_rx_handle = rx_q_handle;
	}

	if(rx_q_handle->rx_work_q == NULL)
	{
		rx_q_handle->rx_work_q = create_singlethread_workqueue("rx_workqueue");
		if(rx_q_handle->rx_work_q == NULL)
			TROUT_DBG2("create work queue failed!\n");
	}

	normal_pri_q = &rx_q_handle->rx_q_info[NORMAL_PRI_RXQ];
	high_pri_q = &rx_q_handle->rx_q_info[HIGH_PRI_RXQ];

	if(high_pri_q->queue_head == NULL)
	{
		size = sizeof(trout_rx_mem_struct) * NUM_HIPR_RX_BUFFS * HOST_RX_Q_NUM;
		high_pri_q->queue_head = (UWORD32 *)kmalloc(size, GFP_KERNEL);
		if(high_pri_q->queue_head == NULL)
		{
			TROUT_DBG2("alloc host rx high priority queue failed!\n");
			goto err0;
		}
		
		INIT_LIST_HEAD(&high_pri_q->rx_list);
		INIT_WORK(&high_pri_q->work, rx_complete_work);

		mutex_init(&high_pri_q->rx_mutex_lock);
	}

	if(normal_pri_q->queue_head == NULL)
	{
		/* Normal rx buffer protect and limit setting */
		if(g_normal_rxq_num * sizeof(trout_rx_mem_struct) > MAX_RX_MEM_SIZE)
		{
			g_normal_rxq_num = MAX_RX_MEM_SIZE / sizeof(trout_rx_mem_struct);
			TROUT_DBG2("Warning: rx mem is too large, auto adjust to %d!\n",
							g_normal_rxq_num);
		}
		
		size = sizeof(trout_rx_mem_struct) * g_normal_rxq_num * HOST_RX_Q_NUM;
		if(gsb->rx_start)
		{
			if(gsb->rx_size >= size)
				normal_pri_q->queue_head = gsb->rx_start;
			else if(gsb->rx_size >= MIN_RX_MEM_SIZE && gsb->rx_size < size)
			{
				g_normal_rxq_num = gsb->rx_size / (sizeof(trout_rx_mem_struct) * HOST_RX_Q_NUM);
				normal_pri_q->queue_head = gsb->rx_start;
			}
			else
			{
#ifdef  SYS_RAM_256M
				printk("Err: RX DMA buf not alloced!\n");
				goto err1;
#endif				
			}
			TROUT_DBG4("rx_start: 0x%x, size: 0x%x\n", gsb->rx_start, size);
		}
		else
		{
#ifdef  SYS_RAM_256M
			printk("Err: RX DMA buf not alloced!\n");
			goto err1;
#else
			normal_pri_q->queue_head = (UWORD32 *)kmalloc(size, GFP_KERNEL);
			if(normal_pri_q->queue_head == NULL)
			{
				TROUT_DBG2("alloc host rx normal priority queue failed!\n");
				goto err1;
			}
#endif
		}
		
		INIT_LIST_HEAD(&normal_pri_q->rx_list);
		INIT_WORK(&normal_pri_q->work, rx_complete_work);

		mutex_init(&normal_pri_q->rx_mutex_lock);
	}

	TROUT_DBG4("high_rxq_num = %d, normal_rxq_num = %d\n", NUM_HIPR_RX_BUFFS, g_normal_rxq_num);

	//high priority rx queue initial...	
	mutex_lock(&high_pri_q->rx_mutex_lock);
	
	trout_rx_q = (trout_rx_mem_struct *)HIGH_RX_MEM_BEGIN;
	
	high_pri_q->queue_pri = HIGH_PRI_RXQ;
	high_pri_q->cell_num = NUM_HIPR_RX_BUFFS * HOST_RX_Q_NUM;
	high_pri_q->queue_tail = (UWORD32 *)((trout_rx_mem_struct *)(high_pri_q->queue_head) + 
													high_pri_q->cell_num - 1);
	high_pri_q->cur_ptr = high_pri_q->queue_head;
	high_pri_q->trout_head = (UWORD32 *)trout_rx_q;
	high_pri_q->trout_tail = (UWORD32 *)(&trout_rx_q[NUM_HIPR_RX_BUFFS - 1]);
	high_pri_q->trout_cur_ptr = high_pri_q->trout_head;
	high_pri_q->trout_q_tail = (trout_rx_mem_struct *)high_pri_q->trout_tail;

	if(!list_empty(&high_pri_q->rx_list))	//if normal rx list is not empty, free it.
	{
		list_for_each_entry_safe(rx_info, tmp, &high_pri_q->rx_list, list)
		{		
			list_del(&rx_info->list);
			kfree(rx_info);
		}
	}
	
	if(init_trout_high_rx_queue(high_pri_q->queue_head) < 0)
	{
		TROUT_RX_DBG1("%s: init trout high rx queue failed!\n", __func__);
		mutex_unlock(&high_pri_q->rx_mutex_lock);
		goto err2;
	}
	init_host_rx_queue(high_pri_q->queue_head, high_pri_q->cell_num);
	mutex_unlock(&high_pri_q->rx_mutex_lock);

	/* normal priority rx queue initial... */
	mutex_lock(&normal_pri_q->rx_mutex_lock);

	trout_rx_q = (trout_rx_mem_struct *)NORMAL_RX_MEM_BEGIN;
	
	normal_pri_q->queue_pri = NORMAL_PRI_RXQ;
	normal_pri_q->cell_num = g_normal_rxq_num * HOST_RX_Q_NUM;
	normal_pri_q->queue_tail = (UWORD32 *)((trout_rx_mem_struct *)(normal_pri_q->queue_head) + 
						normal_pri_q->cell_num - 1);
	normal_pri_q->cur_ptr = normal_pri_q->queue_head;
	normal_pri_q->trout_head = (UWORD32 *)trout_rx_q;
	normal_pri_q->trout_tail = (UWORD32 *)(&trout_rx_q[g_normal_rxq_num - 1]);
	normal_pri_q->trout_cur_ptr = normal_pri_q->trout_head;
	normal_pri_q->trout_q_tail = (trout_rx_mem_struct *)normal_pri_q->trout_tail;

	if(!list_empty(&normal_pri_q->rx_list))	//if normal rx list is not empty, free it.
	{
		list_for_each_entry_safe(rx_info, tmp, &normal_pri_q->rx_list, list)
		{		
			list_del(&rx_info->list);
			kfree(rx_info);
		}
	}
	
	if(init_trout_normal_rx_queue(normal_pri_q->queue_head) < 0)
	{
		TROUT_RX_DBG1("%s: init trout normal rx queue failed!\n", __func__);
		mutex_unlock(&normal_pri_q->rx_mutex_lock);
		goto err2;
	}
	init_host_rx_queue(normal_pri_q->queue_head, normal_pri_q->cell_num);
	mutex_unlock(&normal_pri_q->rx_mutex_lock);
	
	TROUT_FUNC_EXIT;
	return 0;

err2:
	if(normal_pri_q->queue_head != NULL)
	{
		if(!gsb->rx_start)
			kfree(normal_pri_q->queue_head);
		normal_pri_q->queue_head = NULL;
	}
	
err1:
	if(high_pri_q->queue_head != NULL)
	{
		kfree(high_pri_q->queue_head);
		high_pri_q->queue_head = NULL;
	}
	
err0:
	if(rx_q_handle->rx_work_q)
	{
		destroy_workqueue(g_rx_handle->rx_work_q);
		g_rx_handle->rx_work_q = NULL;
	}
	
	if(rx_q_handle != NULL)
	{
		kfree(rx_q_handle);
		rx_q_handle = NULL;
	}

	TROUT_FUNC_EXIT;
	return -1;
}


void host_rx_queue_free(void)
{
    rx_queue_struct *rx_queue = NULL;
	rx_int_info *rx_info, *tmp;
	UWORD8 i;

    TROUT_FUNC_ENTER;
    
    if(g_rx_handle == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
	}
	
	rx_queue = g_rx_handle->rx_q_info;
	for(i=0; i<NUM_RX_Q; i++)
	{
		mutex_lock(&rx_queue->rx_mutex_lock);
		list_for_each_entry_safe(rx_info, tmp, &rx_queue->rx_list, list)
		{		
			list_del(&rx_info->list);
			kfree(rx_info);
		}

	    rx_queue->queue_pri = INVALID_PRI_RXQ;
	    rx_queue->cell_num = 0;
	    rx_queue->num_dscr = 0;
	    rx_queue->left_dscr_num = 0;
	    rx_queue->queue_tail = NULL;
	    rx_queue->trout_head = NULL;
	    rx_queue->trout_tail = NULL;
	    rx_queue->cur_ptr = NULL;
	    rx_queue->trout_cur_ptr = NULL;
	    rx_queue->trout_q_tail = NULL;

	    if(rx_queue->queue_head != NULL)
	    {
		if(i != NORMAL_PRI_RXQ){
			kfree(rx_queue->queue_head);
		}else{
			if(!gsb->rx_start)
				kfree(rx_queue->queue_head);
		}
	        rx_queue->queue_head = NULL;
	    }

	    mutex_unlock(&rx_queue->rx_mutex_lock);
	    rx_queue++;
	}
    
	if(g_rx_handle->rx_work_q != NULL)
	{
        flush_workqueue(g_rx_handle->rx_work_q);
		destroy_workqueue(g_rx_handle->rx_work_q);
		g_rx_handle->rx_work_q = NULL;
	}

	kfree(g_rx_handle);
	g_rx_handle = NULL;
	TROUT_FUNC_EXIT;
}

void flush_all_rxq(void)
{
	rx_queue_struct *rx_queue = NULL;
	rx_int_info *rx_info, *tmp;
	UWORD8 i;

    TROUT_FUNC_ENTER;
    if(g_rx_handle == NULL)
    {
		TROUT_FUNC_EXIT;
        return;
	}

	TROUT_RX_DBG4("flush all rxq -->");
	/*jiangtao.yi:moved from latter. 
          *For rxq, should flush all the work before reinitialize the rxq software structure.
          * in order to avoid kernel panic */
	if(g_rx_handle->rx_work_q != NULL)
	{
        flush_workqueue(g_rx_handle->rx_work_q);
	}

	rx_queue = g_rx_handle->rx_q_info;
	for(i=0; i<NUM_RX_Q; i++)
	{
		mutex_lock(&rx_queue->rx_mutex_lock);
		
		set_machw_rx_buff_addr(0, i);
		list_for_each_entry_safe(rx_info, tmp, &rx_queue->rx_list, list)
		{		
			list_del(&rx_info->list);
			kfree(rx_info);
		}

	    rx_queue->queue_pri = INVALID_PRI_RXQ;
	    rx_queue->cell_num = 0;
	    rx_queue->num_dscr = 0;
	    rx_queue->left_dscr_num = 0;
	    rx_queue->queue_tail = NULL;
	    rx_queue->trout_head = NULL;
	    rx_queue->trout_tail = NULL;
	    rx_queue->cur_ptr = NULL;
	    rx_queue->trout_cur_ptr = NULL;
	    rx_queue->trout_q_tail = NULL;

	    mutex_unlock(&rx_queue->rx_mutex_lock);
	    rx_queue++;
	}
    
        //jiangtao, have move adhead
	//if(g_rx_handle->rx_work_q != NULL)
	//{
        //flush_workqueue(g_rx_handle->rx_work_q);
	//}
	TROUT_RX_DBG4("[OK]\n");
}


#ifdef USE_NEW_CLEAR   /*by kelvin*/ 
void clear_trout_dscr_status(rx_queue_struct *rx_ctrl)
{
	trout_rx_mem_struct *tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->base_dscr;
	UWORD32 clear_addr[BLOCK_WRITE_CAP];
	UWORD32 clear_value[BLOCK_WRITE_CAP] = 
							{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	UWORD32 clear_len[BLOCK_WRITE_CAP] = 
							{
								4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6,
						   		4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6,
						   	};
	/*UWORD32 clear_len[BLOCK_WRITE_CAP] = 
							{4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};*/
	UWORD32 i, j;

	for(i = 0; i < rx_ctrl->num_dscr / BLOCK_WRITE_CAP; i++)
	{
		for(j = 0; j < BLOCK_WRITE_CAP; j++)
		{
			clear_addr[j] = (UWORD32)tmp_dscr;
			tmp_dscr++;
			if(tmp_dscr > (trout_rx_mem_struct *)rx_ctrl->trout_tail)
				tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->trout_head;
		}
		host_block_write_trout_ram(clear_addr, (UWORD32)clear_value, clear_len, j);
	}
	
	if(rx_ctrl->num_dscr % BLOCK_WRITE_CAP)
	{
		for(j = 0; j < rx_ctrl->num_dscr % BLOCK_WRITE_CAP; j++)
		{
			clear_addr[j] = (UWORD32)tmp_dscr;
			tmp_dscr++;
			if(tmp_dscr > (trout_rx_mem_struct *)rx_ctrl->trout_tail)
				tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->trout_head;
		}
		host_block_write_trout_ram(clear_addr, (unsigned int)clear_value, clear_len, j);
	}
}

#else

void clear_trout_dscr_status(rx_queue_struct *rx_ctrl)
{
	trout_rx_mem_struct *tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->base_dscr;
	UWORD32 clear_addr[BLOCK_WRITE_CAP]={0,};
	UWORD32 clear_value[BLOCK_WRITE_CAP] = {0,};
	UWORD32 clear_len[BLOCK_WRITE_CAP] = 
							{
								4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6,
						   		4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6,
						   	};
	UWORD32 i, j;

	for(i = 0; i < rx_ctrl->num_dscr / BLOCK_WRITE_CAP; i++)
	{
		for(j = 0; j < BLOCK_WRITE_CAP; j++)
		{
			clear_addr[j] = (UWORD32)tmp_dscr;
			tmp_dscr = move_to_next(rx_ctrl,tmp_dscr);
		}
		host_block_write_trout_ram(clear_addr, (UWORD32)clear_value, clear_len, BLOCK_WRITE_CAP);
	}
	
	if(rx_ctrl->num_dscr % BLOCK_WRITE_CAP)
	{
		for(j = 0; j < rx_ctrl->num_dscr % BLOCK_WRITE_CAP; j++)
		{
			clear_addr[j] = (UWORD32)tmp_dscr;
			tmp_dscr = move_to_next(rx_ctrl,tmp_dscr);
		}
		host_block_write_trout_ram(clear_addr, (unsigned int)clear_value, clear_len, j);
	}
}

#endif

INLINE void set_pkt_mem_in_using_flag(trout_rx_mem_struct *pkt_dscr)
{
	UWORD8 *ptr = &pkt_dscr->trout_rx_dscr[RX_MEM_MGMT_FLAG_OFFSET * sizeof(UWORD32)];
	
	*(UWORD32 *)ptr = MEM_IN_USE_FLAG;
}

INLINE void clear_pkt_mem_in_using_flag(trout_rx_mem_struct *pkt_dscr)
{
	UWORD8 *ptr = &pkt_dscr->trout_rx_dscr[RX_MEM_MGMT_FLAG_OFFSET * sizeof(UWORD32)];
	
	*(UWORD32 *)ptr = 0;
}

INLINE BOOL_T is_pkt_mem_busy(trout_rx_mem_struct *pkt_dscr)
{
	UWORD32 status;

	status = (*(volatile UWORD32 *)&pkt_dscr->trout_rx_dscr[RX_MEM_MGMT_FLAG_OFFSET * sizeof(UWORD32)]);
	if(status == MEM_IN_USE_FLAG)
		return BTRUE;
	else
		return BFALSE;
}


/* this function only clear chained list dscr's status and return next dscr */
UWORD32 *free_host_rx_dscr_list(UWORD32 *base_dscr, UWORD8 num_dscr)
{
	trout_rx_mem_struct *rx_dscr = (trout_rx_mem_struct *)base_dscr;
	UWORD32 *next_dscr = NULL;
	int flag;
	
	
	while(num_dscr-- > 0)
	{
		if(rx_dscr == NULL) 
		{			
			return NULL;
		}		
		next_dscr = (UWORD32 *)get_host_rxds_next_dscr((UWORD32 *)rx_dscr->trout_rx_dscr);

		flag = critical_section_smart_start(1,1);
		//xuanyang, 2013-10-31, decrease used flag 
		rx_mem_dec_user(rx_dscr);
		rx_mem_free(rx_dscr);
		critical_section_smart_end(flag);
		
		rx_dscr = (trout_rx_mem_struct *)next_dscr;
	}
	return (UWORD32 *)rx_dscr;
}


UWORD32 *free_host_rxds_when_err(UWORD32 *base_dscr, UWORD8 num_dscr)
{
	trout_rx_mem_struct *rx_dscr = (trout_rx_mem_struct *)base_dscr;
	UWORD32 *next_dscr = NULL;
	int flag;
	
	if(rx_dscr == NULL) 
		return NULL;
	
	while(num_dscr-- > 0)
	{
		flag = critical_section_smart_start(1,1);	
		//xuanyang, 2013-10-31, decrease used flag 
		rx_mem_dec_user(rx_dscr);
		rx_mem_free(rx_dscr);	
		critical_section_smart_end(flag);
		
		rx_dscr++;
	}
	
	
	return (UWORD32 *)rx_dscr;
}



#ifdef USE_BLOCK_WRITE
static void replenish_trout_rx_queue(rx_queue_struct *rx_ctrl)
{
	trout_rx_mem_struct *tmp_dscr = NULL;
	UWORD8 num_dscr, count, i = 0;
	UWORD32 trout_addr[BLOCK_WRITE_CAP];
	UWORD32 trout_value[BLOCK_WRITE_CAP] = {0, 0, 0, 0, 0, 0, 0, 0,
						     0, 0, 0, 0, 0, 0, 0, 0};
	UWORD32 trout_len[BLOCK_WRITE_CAP] = {4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6,
				     		   4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6, 4*6};
	//UWORD32 trout_len[BLOCK_WRITE_CAP] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};			     		   

	if((rx_ctrl->base_dscr > (UWORD32)rx_ctrl->trout_tail)
			|| (rx_ctrl->base_dscr < (UWORD32)rx_ctrl->trout_head))
	{
		TROUT_RX_DBG2("trout rx queue %d out of range!\n", rx_ctrl->queue_pri);
		return;
	}

	//mutex_lock(&rx_ctrl->rx_mutex_lock);
	/* before replenish trout rcved queue, clear dscr's status first */
	if(rx_ctrl->num_dscr > BLOCK_WRITE_CAP - 3)
		clear_trout_dscr_status(rx_ctrl);
	else
	{
		tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->base_dscr;
		for(i = 0; i < rx_ctrl->num_dscr; i++)
		{
			trout_addr[i] = (UWORD32)tmp_dscr;
			tmp_dscr++;
			if(tmp_dscr > (trout_rx_mem_struct *)rx_ctrl->trout_tail)
				tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->trout_head;
		}
	}

	// get new queue tail dscr.
	tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->base_dscr + (rx_ctrl->num_dscr - 1);
	if(tmp_dscr > (trout_rx_mem_struct *)rx_ctrl->trout_tail)
	{
		count = ((UWORD32)rx_ctrl->trout_tail - rx_ctrl->base_dscr) / sizeof(trout_rx_mem_struct) + 1;
		num_dscr = rx_ctrl->num_dscr - count;

		tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->trout_head + (num_dscr - 1);
	}

	/* use block write to replenish RX descriptors */  /*by kelvin*/ 
	trout_addr[i] = (UWORD32)&((UWORD32 *)tmp_dscr)[RX_NEXT_DSCR_POINT_OFFSET];	//must set new tail first!!!
	trout_addr[i+1] = (UWORD32)&((UWORD32 *)rx_ctrl->trout_q_tail->trout_rx_dscr)[RX_NEXT_DSCR_POINT_OFFSET];
	trout_addr[i+2] = (UWORD32)&((UWORD32 *)rx_ctrl->base_dscr)[RX_PREV_DSCR_POINT_OFFSET];
	trout_value[i] = 0;	//new tail value.
	trout_value[i+1] = rx_ctrl->base_dscr;
	trout_value[i+2] = (UWORD32)rx_ctrl->trout_q_tail->trout_rx_dscr;
	trout_len[i] = 4;
	trout_len[i+1] = 4;
	trout_len[i+2] = 4;
	
	host_block_write_trout_ram(trout_addr,(unsigned int)trout_value, trout_len, i+3);

/*	if(rx_ctrl->queue_pri == 0)	//debug..
		TROUT_RX_DBG4("replenish: tail(0x%x)->next(0x%x)...->tail(0x%x), %d\n",
						trout_value[i+2], trout_value[i+1], (UWORD32)tmp_dscr, rx_ctrl->num_dscr);	*/
	
	/* update trout_q_tail pointer, make it point to trout rx queue tail */
	rx_ctrl->trout_q_tail = tmp_dscr;
	
	if(is_machw_rx_buff_null(rx_ctrl->queue_pri) == BTRUE)
	{
		tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->base_dscr;
		set_machw_rx_buff_addr((UWORD32)tmp_dscr->trout_rx_dscr, rx_ctrl->queue_pri);

		TROUT_RX_DBG5("rxq%d: reset MAC rx buf register(0x%p, %d)!\n", 
						rx_ctrl->queue_pri, tmp_dscr->trout_rx_dscr, rx_ctrl->num_dscr);
		
		if(rx_ctrl->trout_cur_ptr == NULL)
		{
			rx_ctrl->trout_cur_ptr = (UWORD32 *)rx_ctrl->base_dscr;
			TROUT_RX_DBG5("reset trout current rx ptr(0x%x)!\n", rx_ctrl->base_dscr);
		}
	}
	//mutex_unlock(&rx_ctrl->rx_mutex_lock);
}

#else

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
trout_rx_mem_struct *move_to_prev(rx_queue_struct *rx_ctrl, trout_rx_mem_struct *p)
{
	trout_rx_mem_struct *t = p;

	if((t > (UWORD32)rx_ctrl->trout_tail)
			|| (t < (UWORD32)rx_ctrl->trout_head)){
		printk("INVALID POINTER @ Func:%s..Line:%d\n", __func__, __LINE__);
		return (trout_rx_mem_struct *)rx_ctrl->trout_head;
	}

	t--;
	if(t < rx_ctrl->trout_head)
		t = rx_ctrl->trout_tail;
	return t;
}
#endif
#endif

int to_idx(rx_queue_struct *rc, trout_rx_mem_struct *p)
{
	trout_rx_mem_struct *t = p;
	
	if((t > (UWORD32)rc->trout_tail)
			|| (t < (UWORD32)rc->trout_head)){
		printk("INVALID POINTER @ Func:%s..Line:%d\n", __func__, __LINE__);
		return (trout_rx_mem_struct *)rc->trout_head;
	}
	
	return ((char *)t - (char *)rc->trout_head) / sizeof(trout_rx_mem_struct);
}

static void replenish_trout_rx_queue(rx_queue_struct *rx_ctrl)
{
	trout_rx_mem_struct *tmp_dscr, *m = NULL;
	UWORD8 num_dscr, count;
	UWORD32 addr[3] = {0}, cs = 0, ps = 0,i;
	trout_rx_mem_struct *t = NULL;

	TROUT_FUNC_ENTER;
	
	if((rx_ctrl->base_dscr > (UWORD32)rx_ctrl->trout_tail)
			|| (rx_ctrl->base_dscr < (UWORD32)rx_ctrl->trout_head))
	{
		TROUT_RX_DBG2("trout rx queue %d out of range!\n", rx_ctrl->queue_pri);
		TROUT_FUNC_EXIT;
		return;
	}

	/* before replenish trout rcved queue, clear dscr's status first */
	clear_trout_dscr_status(rx_ctrl);

/*	if(rx_ctrl->queue_pri == 0)	//debug..
		printk("replenish1: reg_next=0x%x\n", get_machw_rx_buff_addr(rx_ctrl->queue_pri));	*/

	// get sub queue tail dscr.
	addr[0] = rx_ctrl->base_dscr + (rx_ctrl->num_dscr - 1) * sizeof(trout_rx_mem_struct);
	if(addr[0] > (UWORD32)rx_ctrl->trout_tail){
		count = ((UWORD32)rx_ctrl->trout_tail - rx_ctrl->base_dscr) / sizeof(trout_rx_mem_struct) + 1;
		num_dscr = rx_ctrl->num_dscr - count;

		addr[0] = (UWORD32)rx_ctrl->trout_head + (num_dscr - 1) * sizeof(trout_rx_mem_struct);
	}

	if(is_machw_rx_buff_null(rx_ctrl->queue_pri) == BTRUE){
			t = (trout_rx_mem_struct *)rx_ctrl->trout_q_tail;
			set_trout_next_dscr((UWORD32 *)rx_ctrl->trout_q_tail->trout_rx_dscr, rx_ctrl->base_dscr);
			set_trout_prev_dscr((UWORD32 *)rx_ctrl->base_dscr,(UWORD32)rx_ctrl->trout_q_tail->trout_rx_dscr);

			set_trout_next_dscr((UWORD32 *)addr[0], 0);	//new tail next -> NULL
			rx_ctrl->trout_q_tail = (trout_rx_mem_struct *)addr[0];
			m = move_to_next(rx_ctrl, t);
		
			host_read_trout_ram((void *)&ps, (void *)m, 4);
			ps = (ps >> 28) & 0xf;
			if(ps == 0){
				//still have buffer to receive
				udelay(5);
				addr[0] = (UWORD32)rx_ctrl->base_dscr;
				set_machw_rx_buff_addr(addr[0], rx_ctrl->queue_pri);
				
				if(rx_ctrl->trout_cur_ptr == NULL){
					rx_ctrl->trout_cur_ptr = (UWORD32 *)rx_ctrl->base_dscr;
					TROUT_RX_DBG2("reset trout current rx ptr!\n");
				}
				TROUT_RX_DBG2("reset MAC rx buf register(rxq%d, NS:%d, NR:%d)!\n", 
						rx_ctrl->queue_pri, to_idx(rx_ctrl, m), rx_ctrl->num_dscr);
			}else{
				if(rx_ctrl->queue_pri == NORMAL_PRI_RXQ){
				    for(i = 0; i < 0x2; i++){
					    printk("BUG---Q-next:%x, Base:%x, NR:%d\n", to_idx(rx_ctrl, m), 
							to_idx(rx_ctrl, rx_ctrl->base_dscr), rx_ctrl->num_dscr);
				    }
				}
				
			}
	}else{
		t = move_to_next(rx_ctrl, (trout_rx_mem_struct *)rx_ctrl->trout_q_tail);

		set_trout_next_dscr((UWORD32 *)addr[0], 0);	//new tail next -> NULL
		if(addr[0] != rx_ctrl->trout_q_tail){
			set_trout_next_dscr((UWORD32 *)rx_ctrl->trout_q_tail->trout_rx_dscr, rx_ctrl->base_dscr);
		    set_trout_prev_dscr((UWORD32 *)rx_ctrl->base_dscr,(UWORD32)rx_ctrl->trout_q_tail->trout_rx_dscr);
		}

		//t = (trout_rx_mem_struct *)rx_ctrl->trout_q_tail;
		rx_ctrl->trout_q_tail = (trout_rx_mem_struct *)addr[0];
		//if first check, HW is not stop, but here is stoped, so we check and restart it to receive later
		if(is_machw_rx_buff_null(rx_ctrl->queue_pri) == BTRUE){
			host_read_trout_ram((void *)&cs, (void *)t, 4);
			cs = cs >> 28;
			if(cs == 0){
				udelay(5);
				addr[0] = (UWORD32)rx_ctrl->base_dscr;
				set_machw_rx_buff_addr(addr[0], rx_ctrl->queue_pri);
				if(rx_ctrl->trout_cur_ptr == NULL){
					rx_ctrl->trout_cur_ptr = (UWORD32 *)rx_ctrl->base_dscr;
					TROUT_RX_DBG2("reset trout current rx ptr %d!\n", to_idx(rx_ctrl, t));
				}
			}
		}
	}

	TROUT_FUNC_EXIT;
}
#endif

UWORD8 get_machw_rx_num_dscr(rx_queue_struct *rx_ctrl, UWORD32 *cur_dscr)
{
	UWORD32 buff_addr, tmp_dscr = 0,stop_dscr=0;
	UWORD8 num_dscr = 0, max_dscr = 0;
	
	TROUT_FUNC_ENTER;
	
	if(rx_ctrl->queue_pri == HIGH_PRI_RXQ)
		max_dscr = NUM_HIPR_RX_BUFFS;
	else if(rx_ctrl->queue_pri == NORMAL_PRI_RXQ)
		max_dscr = g_normal_rxq_num;

	buff_addr = get_machw_rx_buff_addr(rx_ctrl->queue_pri);
	*cur_dscr = 0;
	
	if((rx_ctrl->trout_cur_ptr == NULL) || ((UWORD32)(rx_ctrl->trout_cur_ptr) == buff_addr))
	{
		TROUT_FUNC_EXIT;
		return 0;
	}

	if(buff_addr == NULL ){
		stop_dscr  = move_to_next(rx_ctrl,rx_ctrl->trout_q_tail);
	}else{
		stop_dscr = buff_addr;
	}

	*cur_dscr = (UWORD32)(rx_ctrl->trout_cur_ptr);	//get this time base_dscr.
	
	if(stop_dscr > (UWORD32)(rx_ctrl->trout_cur_ptr)){
		num_dscr = (stop_dscr - (UWORD32)(rx_ctrl->trout_cur_ptr))/sizeof(trout_rx_mem_struct);
	}else{
		num_dscr = ( (UWORD32)(rx_ctrl->trout_tail) -  (UWORD32)(rx_ctrl->trout_cur_ptr))/sizeof(trout_rx_mem_struct) +1 ;
		num_dscr += (stop_dscr - (UWORD32)rx_ctrl->trout_head)/sizeof(trout_rx_mem_struct);
	}

	if(num_dscr > max_dscr)
	{
		TROUT_RX_DBG2("ERR: get num of dscr failed(%d:%d)!\n", num_dscr, max_dscr);
		return 0;
	}

	//printk("[libing] : buff_addr =%u,trout_cur_ptr = %u,stop_dscr=%u,num_dscr= %d,trout_q_tail=%u\n",\
		//buff_addr,rx_ctrl->trout_cur_ptr,stop_dscr,num_dscr,rx_ctrl->trout_q_tail);

	rx_ctrl->trout_cur_ptr = buff_addr; 	//update trout_cur_ptr, make it point to next time rx dscr start position.
	
	TROUT_FUNC_EXIT;
	return num_dscr;
}


/* read the fifo, make it empty, is this reasonable?  */
void active_fifo_rx_frame_pointer(UWORD8 q_num)
{
	UWORD32 regAddr, temp[2];
	WORD8 fifo_depth = 3;
	UWORD32 ptr;
	rx_queue_handle *rx_handle = g_rx_handle;
	rx_queue_struct *rx_ctrl = &rx_handle->rx_q_info[q_num];	

	TROUT_FUNC_ENTER;
	
    if(q_num == HIGH_PRI_RXQ)
		regAddr = (UWORD32)rMAC_HIP_RX_FRAME_POINTER;
    else
    	regAddr = (UWORD32)rMAC_RX_FRAME_POINTER;

	temp[0] = host_read_trout_reg(regAddr);
	/* remove useless log by zhao */
/*
	if(temp[0] > 0xdc5c){
		trout_rx_mem_show();
		ptr = get_machw_rx_buff_addr(rx_ctrl->queue_pri);
		printk("rx_buff_addr: %x\n", ptr);
	}
*/
	do{
		temp[1] = host_read_trout_reg(regAddr);
	/* remove useless log by zhao */
/*
		if(temp[1] > 0xdc5c){
			trout_rx_mem_show();
			ptr = get_machw_rx_buff_addr(rx_ctrl->queue_pri);
			printk("rx_buff_addr: %x\n", ptr);
		}
*/
		if(temp[0] == temp[1])
			break;
		
		temp[0] = temp[1];
	}while(fifo_depth-- > 0);
	
	TROUT_FUNC_EXIT;
}




#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
void adjust_from_ps_state(rx_queue_struct *rx_ctrl)	//only apply to high pri rxq.
{
	trout_rx_mem_struct *rx_mem_s = NULL;
	UWORD32 *trout_dscr = NULL;
	UWORD32 next_dscr, status;
	BOOL_T cur_ptr_found = BFALSE, rxq_tail_found = BFALSE;
	int index;

	if(rx_ctrl->queue_pri != HIGH_PRI_RXQ)
	{
		TROUT_DBG4("invalid rxq %d\n", rx_ctrl->queue_pri);
		return;
	}

	TROUT_DBG4("before adjust: trout_cur_ptr=0x%p, trout_q_tail=0x%p\n",
								rx_ctrl->trout_cur_ptr, rx_ctrl->trout_q_tail);

	trout_dscr = rx_ctrl->trout_head;
	rx_mem_s = (trout_rx_mem_struct *)trout_dscr;
	for(index=0; index<NUM_HIPR_RX_BUFFS; index++)
	{
	/*
		if(!cur_ptr_found)
		{
			host_read_trout_ram((void *)&status, (void *)trout_dscr, sizeof(UWORD32));
			if(get_rx_dscr_pkt_status(&status) == PS_SUCCESS)
			{
				status = (status & 0x0FFFFFFF) | (RX_SUCCESS << 28);
				host_write_trout_ram((void *)trout_dscr, (void *)&status, sizeof(UWORD32));	//recover status.
				rx_ctrl->trout_cur_ptr = trout_dscr;
				cur_ptr_found = BTRUE;
			}
		}
	*/
		if(!rxq_tail_found)
		{
			next_dscr = get_trout_next_dscr(trout_dscr);
			if(next_dscr == 0)
			{
				rx_ctrl->trout_q_tail = (trout_rx_mem_struct *)trout_dscr;
				rxq_tail_found = BTRUE;
				trout_dscr = move_to_next(rx_ctrl, (trout_rx_mem_struct *)trout_dscr);
				rx_ctrl->trout_cur_ptr = trout_dscr;

				host_read_trout_ram((void *)&status, (void *)trout_dscr, sizeof(UWORD32));
				if(get_rx_dscr_pkt_status(&status) == PS_SUCCESS){
					status = (status & 0x0FFFFFFF) | (RX_SUCCESS << 28);
					host_write_trout_ram((void *)trout_dscr, (void *)&status, sizeof(UWORD32));	//recover status.
				}
				cur_ptr_found = BTRUE;
			}
		}

		if(cur_ptr_found && rxq_tail_found)
			break;

		trout_dscr = (UWORD32 *)(++rx_mem_s);
	}
	if(!rxq_tail_found || !cur_ptr_found)
		printk("BUG!!!!! the RX WR Pointer not SYNC\n");
	if(!rx_ctrl->trout_cur_ptr)
		printk("BUG!!! the cur_ptr is NULL!\n");

	TROUT_DBG4("after adjust: trout_cur_ptr=0x%p, trout_q_tail=0x%p\n",
								rx_ctrl->trout_cur_ptr, rx_ctrl->trout_q_tail);
}
extern BOOL_T host_awake_by_arm7;
#endif
#endif

void rx_complete_isr(UWORD8 q_num)
{
	rx_queue_handle *rx_handle = g_rx_handle;
	rx_queue_struct *rx_ctrl = &rx_handle->rx_q_info[q_num];	
	UWORD32 base_dscr = 0;
	UWORD8 num_dscr = 0;
	rx_int_info *rx_info, *tmp, *rx_info_record;
	UWORD32 trout_end;
	UWORD32 last_num = 0;


	TROUT_FUNC_ENTER;

	if(rx_handle == NULL || rx_ctrl == NULL || rx_ctrl->queue_pri == INVALID_PRI_RXQ)
	{
		get_machw_rx_frame_pointer(q_num);
		TROUT_RX_DBG2("rx queue %d isn't initialized!\n", q_num);
		TROUT_FUNC_EXIT;
		return;
	}

	mutex_lock(&rx_ctrl->rx_mutex_lock);
	
	//get_machw_rx_frame_pointer(q_num);
	//active_fifo_rx_frame_pointer(q_num);
#if 1
	active_fifo_rx_frame_pointer(q_num);	/* dummy read */

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
#if 0
        if((q_num == HIGH_PRI_RXQ) && (host_awake_by_arm7))     //maybe not correct, need verify, judge this is recover from ps mode!!!
        {
                //host_awake_by_arm7 = BFALSE;
                printk("AW by ARM7\n");
                //print_trout_rx_dscr_cfg_info(HIGH_PRI_RXQ, (UWORD32 *)HIGH_RX_MEM_BEGIN);
                adjust_from_ps_state(rx_ctrl);
        }
#endif
#endif
#endif
	/* Get the base descriptor pointer and the number of buffers */
	num_dscr = get_machw_rx_num_dscr(rx_ctrl, &base_dscr);
#else
	base_dscr = get_machw_rx_frame_pointer(q_num);
	num_dscr = get_machw_num_rx_frames(q_num);
#endif

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE
#ifdef WIFI_SLEEP_POLICY
#if 0
	if(q_num == HIGH_PRI_RXQ && host_awake_by_arm7){
		host_awake_by_arm7 = 0;
	}
#endif
#endif
#endif

	if(q_num == NORMAL_PRI_RXQ)
    	TROUT_RX_DBG5("%s: pri: %d, base_dscr: 0x%08x, num_dscr: %d\n", 
										__FUNCTION__, q_num, base_dscr, num_dscr);
#ifdef DEBUG_MODE
	g_mac_stats.rcv_rx_dscr_num += num_dscr;
#endif
    
#ifdef IBSS_BSS_STATION_MODE
        if(BTRUE == g_wifi_bt_coex){
            coex_wifi_tx_rx_pkg_sum(COEX_WIFI_RX_PKG, num_dscr);
        }
#endif

   	if(base_dscr == 0 || num_dscr == 0)
	{        
		TROUT_FUNC_EXIT;
		mutex_unlock(&rx_ctrl->rx_mutex_lock);
		return;
	}

	/* in this case, maybe need reinitial trout rx queue */
	if((base_dscr > (UWORD32)rx_ctrl->trout_tail) ||
			(base_dscr < (UWORD32)rx_ctrl->trout_head))
	{
		TROUT_RX_DBG2("ERR: trout rx%d queue is out of range(0x%08x ~ 0x%08x, 0x%08x)!\n",
					rx_ctrl->queue_pri, (UWORD32)rx_ctrl->trout_head, 
								(UWORD32)rx_ctrl->trout_tail, base_dscr);
		TROUT_FUNC_EXIT;
		mutex_unlock(&rx_ctrl->rx_mutex_lock);
		return;
	}

       /* add merge rx list node code */
	if(!list_empty(&rx_ctrl->rx_list))
	{
	       //foreach to find the last node	        
	       list_for_each_entry(rx_info_record, &rx_ctrl->rx_list, list)
	       {
	              if(rx_info_record->base_dscr != 0)
	              {
	                     rx_info = rx_info_record; 
	              }
	       }

		trout_end = rx_info->base_dscr + rx_info->num_dscr * sizeof(trout_rx_mem_struct);
		if(trout_end > ((UWORD32)rx_ctrl->trout_tail + sizeof(trout_rx_mem_struct)))
		{
		       last_num = ((UWORD32)rx_ctrl->trout_tail - rx_info->base_dscr) / sizeof(trout_rx_mem_struct) + 1;
		       trout_end = (UWORD32)rx_ctrl->trout_head + (rx_info->num_dscr - last_num) * sizeof(trout_rx_mem_struct);
	       }
	}		
	
	if(list_empty(&rx_ctrl->rx_list) || (trout_end != base_dscr))
	{
	       rx_info = (rx_int_info *)kmalloc(sizeof(rx_int_info), GFP_KERNEL);
		if(rx_info == NULL)
		{
			TROUT_RX_DBG2("%s: kmalloc failed!\n", __FUNCTION__);
			TROUT_FUNC_EXIT;
#ifdef DEBUG_MODE		
			g_mac_stats.rx_drop_pkt_count += num_dscr;
#endif
	        //caisf add 2013-02-15
	        if(g_mac_net_stats){
	            g_mac_net_stats->rx_dropped += num_dscr;
	        }

			mutex_unlock(&rx_ctrl->rx_mutex_lock);
			return;
		}
		
		rx_info->base_dscr = base_dscr;
		rx_info->num_dscr = num_dscr;
		
		list_add_tail(&rx_info->list, &rx_ctrl->rx_list);
	}   
	else if(trout_end == base_dscr)
	{
	       rx_info->num_dscr += num_dscr;
	}
	
	if(rx_handle->rx_work_q != NULL)
		queue_work_on(0, rx_handle->rx_work_q, &rx_ctrl->work);
	else
		schedule_work_on(0, &rx_ctrl->work);

    	mutex_unlock(&rx_ctrl->rx_mutex_lock);
	TROUT_FUNC_EXIT;
}

void *find_free_dscr_from_queue(rx_queue_struct *rx_ctrl, UWORD8 num_dscr)
{
	trout_rx_mem_struct *queue_head, *queue_tail, *tmp_ptr;
	void *host_mem_addr = NULL;
	UWORD8 node_num = 0;
	UWORD32 vv, vf, *p;

	TROUT_FUNC_ENTER;
	if(rx_ctrl == NULL || num_dscr == 0)
	{
		TROUT_DBG3("%s: invalid parameter!\n", __FUNCTION__);
		TROUT_FUNC_EXIT;
		return NULL;
	}
	
	queue_head = (trout_rx_mem_struct *)rx_ctrl->queue_head;
	queue_tail = (trout_rx_mem_struct *)rx_ctrl->queue_tail;
	
	tmp_ptr = (trout_rx_mem_struct *)rx_ctrl->cur_ptr;
	
	do{
		p = (UWORD32 *)&tmp_ptr->trout_rx_dscr[0];
		vv = *(volatile UWORD32 *)p;
		vv = vv >> 28;
		p += 10;
		vf = *(volatile UWORD32 *)p;
		if(vv == RX_NEW && vf == RX_NEW)
			node_num++;
		else
			node_num = 0;

		if(node_num == 1)
			host_mem_addr = (void *)tmp_ptr;
		
		tmp_ptr++;
		if(tmp_ptr > queue_tail)
		{
			tmp_ptr = queue_head;
			if(node_num < num_dscr)
				node_num = 0;
		}
	}while(node_num < num_dscr && tmp_ptr != (trout_rx_mem_struct *)rx_ctrl->cur_ptr);

    if(node_num == num_dscr)   //find ok.
        rx_ctrl->cur_ptr = (UWORD32 *)tmp_ptr;     //update cur_ptr.
    else
    	host_mem_addr = NULL;

    TROUT_FUNC_EXIT;
	return host_mem_addr;
}


/* when transfer data from trout to host memory, need to modify prev, next & data */
/* buffer address, make it point to host it's self memory space					  */
static UWORD32 transact_host_rx_queue(UWORD32 *base_dscr, UWORD8 num_dscr)
{
	trout_rx_mem_struct *tmp_dscr;
	UWORD32 dscr_buf, prev_dscr, next_dscr;
	UWORD32 *dscr_addr, *pt, pc, pn;
	UWORD8 i;

	tmp_dscr = (trout_rx_mem_struct *)base_dscr;
	for(i=0; i<num_dscr; i++)
	{
	       //xuanyang, 2013-10-31, init used flag = 1
		rx_mem_init_user(&tmp_dscr[i], 1);
		dscr_addr = (UWORD32 *)tmp_dscr[i].trout_rx_dscr;
		dscr_buf = (UWORD32)tmp_dscr[i].trout_rx_buf;

		if(i == num_dscr - 1)
			next_dscr = 0;
		else
			next_dscr = (UWORD32)tmp_dscr[i+1].trout_rx_dscr;

		if(i == 0)
			prev_dscr = 0;
		else
			prev_dscr = (UWORD32)tmp_dscr[i-1].trout_rx_dscr;

		set_host_rxds_buffer_ptr(dscr_addr, dscr_buf);
		set_host_rxds_next_dscr(dscr_addr, next_dscr);
		set_host_rxds_prev_dscr(dscr_addr, prev_dscr);
	}

	return (UWORD32)dscr_addr;
}


void do_rx_preprocess(rx_queue_struct *rx_ctrl)
{
	wlan_rx_event_msg_t *wlan_rx = NULL;
	void *host_rx_mem_addr = NULL;
	UWORD32 cur_num, tmp_addr, trout_end, trout_next_dscr, last_dscr_addr = 0;
	UWORD32 trout_base_dscr = rx_ctrl->base_dscr;
	int ret = 0;
	UWORD8 count = 0;
	UWORD32 *host_rx_mem_s_bak = NULL, dscr_num = 0;
#ifdef TROUT_TRACE_DBG
	UWORD32 rcv_len = 0;
#endif

#if 0
	//xuanyang.test
	UWORD32 *dscr_addr = NULL;
	trout_rx_mem_struct * tmp_dscr = NULL;
#endif

	TROUT_FUNC_ENTER;
	
	if(rx_ctrl->left_dscr_num == 0 || rx_ctrl->queue_pri == INVALID_PRI_RXQ)
		return;

	while(rx_ctrl->left_dscr_num > 0)
	{
		cur_num = rx_ctrl->left_dscr_num;
		trout_next_dscr = 0;
		
		trout_end = trout_base_dscr + rx_ctrl->left_dscr_num * sizeof(trout_rx_mem_struct);
		if(trout_end > ((UWORD32)rx_ctrl->trout_tail + sizeof(trout_rx_mem_struct)))
		{
			cur_num = ((UWORD32)rx_ctrl->trout_tail - trout_base_dscr) / sizeof(trout_rx_mem_struct) + 1;
			trout_next_dscr = (UWORD32)rx_ctrl->trout_head;
		}
		
		/* find need's continues free cell dscr from current dscr pointer. */
	    host_rx_mem_addr = find_free_dscr_from_queue(rx_ctrl, cur_num);
	    if(host_rx_mem_addr == NULL)
	    {
			TROUT_RX_DBG3("Warning: no free node, pkt droped(%d:%d) --pri:%d!\n", 
                                        	cur_num, rx_ctrl->left_dscr_num, rx_ctrl->queue_pri);
#ifdef DEBUG_MODE
			if(rx_ctrl->queue_pri == NORMAL_PRI_RXQ)
			{
				g_mac_stats.host_rx_queue_full++;
				g_mac_stats.rx_drop_pkt_count += rx_ctrl->left_dscr_num;

			}
			//print_host_rx_queue_info(rx_ctrl);
#endif

#if 0			
			//xuanyang.test
			tmp_dscr = (trout_rx_mem_struct *)rx_ctrl->queue_head;
			while(tmp_dscr <= rx_ctrl->queue_tail)
			{
				dscr_addr = (UWORD32*)tmp_dscr->trout_rx_dscr;
				printk("%s : dscr_addr:%p use_flag:%d\n", 
					__FUNCTION__,
					dscr_addr, dscr_addr[10]);
				tmp_dscr++;
			}
#endif

            //caisf add 2013-02-15
			if(rx_ctrl->queue_pri == NORMAL_PRI_RXQ)
			{
                		if(g_mac_net_stats){
                    			g_mac_net_stats->rx_dropped += rx_ctrl->left_dscr_num;
                		}
            		}

			if(host_rx_mem_s_bak != NULL && dscr_num > 0)
				free_host_rx_dscr_list(host_rx_mem_s_bak, dscr_num);
		 	if(wlan_rx != NULL)
		 	{
		 		event_mem_free((void *)wlan_rx, WLAN_RX_EVENT_QID);
				wlan_rx = NULL;
			}
			goto out;
	    }

		//printk("before DMA:\n");
	    //print_host_rx_buf_content((UWORD32 *)host_rx_mem_addr, cur_num, 0);
		//dma_map_single(NULL, host_rx_mem_addr, cur_num * sizeof(trout_rx_mem_struct),
		//		DMA_FROM_DEVICE);
		ret = host_read_trout_ram(host_rx_mem_addr, (void *)trout_base_dscr, 
				cur_num * sizeof(trout_rx_mem_struct));
	    if(ret < 0)
	    {
			TROUT_RX_DBG3("host read data from trout failed!\n");
        	free_host_rxds_when_err((UWORD32 *)host_rx_mem_addr, cur_num);
#ifdef DEBUG_MODE
			g_mac_stats.rx_drop_pkt_count += rx_ctrl->left_dscr_num;
#endif
            //caisf add 2013-02-15
            if(g_mac_net_stats)
            {
                g_mac_net_stats->rx_dropped += rx_ctrl->left_dscr_num;
            }

        	//goto err;
        	if(host_rx_mem_s_bak != NULL && dscr_num > 0)
				free_host_rx_dscr_list(host_rx_mem_s_bak, dscr_num);
		 	if(wlan_rx != NULL)
		 	{
		 		event_mem_free((void *)wlan_rx, WLAN_RX_EVENT_QID);
				wlan_rx = NULL;
			}
			goto out;
	    }
	    //printk("after DMA:\n");
	    //print_host_rx_buf_content((UWORD32 *)host_rx_mem_addr, cur_num, 1);

#ifdef TROUT_TRACE_DBG
		rcv_len += cur_num * sizeof(trout_rx_mem_struct);
#endif	    
	    /* transact rcved rx queue dscr, make it point to host rx buf and dscr. */
	    tmp_addr = transact_host_rx_queue((UWORD32 *)host_rx_mem_addr, cur_num);
	    
		//rx_frame_test(&g_mac, host_rx_mem_addr, cur_num);	//chengwg debug.
		
        rx_ctrl->left_dscr_num -= cur_num;
        if(rx_ctrl->left_dscr_num != 0) //if not last time, save current queue tail dscr addr.
            last_dscr_addr = tmp_addr;

        /* let separate sub host rx queue chaining to a single rx queue by connect */
        /* last_dscr_addr(last sub queue tail) and host_rx_mem_addr(current sub queue head),*/
        /* should skip first cycle time 										   */
        if(count != 0)
        {
			set_host_rxds_next_dscr((UWORD32 *)last_dscr_addr, (UWORD32)host_rx_mem_addr);
			set_host_rxds_prev_dscr((UWORD32 *)host_rx_mem_addr, last_dscr_addr);	
        }

		if(wlan_rx == NULL)
		{
			wlan_rx = (wlan_rx_event_msg_t *)event_mem_alloc(WLAN_RX_EVENT_QID);
			if(wlan_rx == NULL)
			{
				TROUT_RX_DBG4("alloc wlan_rx event failed!\n");
				//event_buf_detail_show(WLAN_RX_EVENT_QID);	//debug.
				free_host_rx_dscr_list((UWORD32 *)host_rx_mem_addr, cur_num);
#ifdef DEBUG_MODE
				g_mac_stats.event_no_mem_count++;
				g_mac_stats.rx_drop_pkt_count += rx_ctrl->num_dscr;
#endif
                //caisf add 2013-02-15
                if(g_mac_net_stats){
                    g_mac_net_stats->rx_dropped += rx_ctrl->num_dscr;
                }

				goto out;
			}

			host_rx_mem_s_bak = (UWORD32 *)host_rx_mem_addr;	//new add.
			dscr_num = cur_num;	//new add.
			wlan_rx->base_dscr = (void *)host_rx_mem_addr;
			wlan_rx->num_dscr = rx_ctrl->num_dscr;
			wlan_rx->rxq_num = rx_ctrl->queue_pri;
		}
		else
			dscr_num += cur_num;	//new add.
			
		/* update rx_ctrl->base_dscr, make it pointer to next sub rx queue head */			
		if((rx_ctrl->left_dscr_num != 0) && (trout_next_dscr == 0))
		{
			TROUT_RX_DBG3("Warning: transact incompatible, %d pkt droped!\n",
														rx_ctrl->left_dscr_num);
#ifdef DEBUG_MODE
			g_mac_stats.rx_drop_pkt_count += rx_ctrl->left_dscr_num;
#endif
            //caisf add 2013-02-15
            if(g_mac_net_stats){
                g_mac_net_stats->rx_dropped += rx_ctrl->left_dscr_num;
            }
			break;
		}
		
		trout_base_dscr = trout_next_dscr;
        count++;
	}

	g_rx_pkt_count += wlan_rx->num_dscr;
	g_rx_times++;
	TROUT_RX_DBG5("rx_process<%d>: pkt=%d, length=%u\n", wlan_rx->rxq_num, wlan_rx->num_dscr, rcv_len);

out:
	if(wlan_rx)
	{
		if(g_rx_dbg_flag != 0)
			host_rx_pkt_analysis(wlan_rx);	//debug!
		post_event((UWORD8 *)wlan_rx, WLAN_RX_EVENT_QID);
	}
	
	replenish_trout_rx_queue(rx_ctrl);
	//TROUT_FUNC_EXIT;
}

static void rx_complete_work(struct work_struct *work)
{
	rx_queue_struct *rx_ctrl = container_of(work, rx_queue_struct, work);
	rx_int_info *rx_info, *tmp;

	if( reset_mac_trylock() == 0 ){
		return;
	}
	mutex_lock(&rx_ctrl->rx_mutex_lock);
	list_for_each_entry_safe(rx_info, tmp, &rx_ctrl->rx_list, list)
	{		
		rx_ctrl->base_dscr = rx_info->base_dscr;
		rx_ctrl->num_dscr = rx_info->num_dscr;
		rx_ctrl->left_dscr_num = rx_info->num_dscr;

		do_rx_preprocess(rx_ctrl);
		list_del(&rx_info->list);
		kfree(rx_info);

		mutex_unlock(&rx_ctrl->rx_mutex_lock);
		
		mutex_lock(&rx_ctrl->rx_mutex_lock);
		//xuan.yang, 2013-09-25, prevent variable "tmp" from becoming illegal address
		if(list_empty(&rx_ctrl->rx_list))
		{
 			TROUT_RX_DBG5("%s rx_ctrl->rx_list is empty, return...\n", __FUNCTION__);
			break;
		}
	}
	mutex_unlock(&rx_ctrl->rx_mutex_lock);
	reset_mac_unlock();
}


void process_wlan_rx(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_event_msg_t *wlan_rx   = (wlan_rx_event_msg_t*)msg;
    UWORD8              num_dscr   = wlan_rx->num_dscr;
    UWORD32             *base_dscr = (UWORD32 *)(wlan_rx->base_dscr);
    UWORD8              num_buffs  = 0;
    UWORD8              num_mpdu   = 0, t = 0;
    UWORD8              rx_dscr_status = 0;

	TROUT_FUNC_ENTER;

    /* Loop till number of descriptors remaining to be processed becomes 0 */
    while(num_dscr != 0)
    {
        /* Exception case 1: Base descriptor becomes NULL before the number  */
        /* of descriptors indicated in the interrupt have been processed.    */
        if(base_dscr == NULL)
        {
#ifdef DEBUG_MODE
            PRINTD("HwEr:RxUnexpEnd\n");
            g_mac_stats.pwrx_unexp_end++;
#endif /* DEBUG_MODE */

            /* Do nothing and break */
            break;
        }

        /* Extract the number of buffers from the base descriptor */
        num_buffs = get_rx_dscr_num_buffers(base_dscr);
	 if(num_buffs == 0)
	 {
	 	TROUT_RX_DBG3("%s: num_buffs is empty!, status=%u\n", __func__, 
						get_rx_dscr_pkt_status(base_dscr));
		TROUT_RX_DBG4("%s: base_dscr = 0x%p, num_dscr = %d\n", 
						__FUNCTION__, (UWORD32 *)(wlan_rx->base_dscr), wlan_rx->num_dscr);
		raise_system_warning(RX_Q_ERROR);	//debug!
		free_host_rx_dscr_list(base_dscr, num_dscr);
            	break;
	}
		
    	/* Exception case 3: The number of buffers exceeds the remaining     */
    	/* number of descriptors                                             */
    	if(num_buffs > num_dscr)
    	{
		TROUT_RX_DBG3("%s: num_buffs(%d) bigger than num_dscr(%d)\n",
		__FUNCTION__, num_buffs, num_dscr);
#ifdef DEBUG_MODE
		PRINTD2("HwEr:RxNumExc\n\r");
		g_mac_stats.pwrx_numexceed++;
#endif /* DEBUG_MODE */
		/* Free all the remaining descriptors and break */
		free_host_rx_dscr_list(base_dscr, num_dscr);
		break;
    	}
		    			
	 /* Exception Case: The status of receive descriptor passed is not updated */
	 rx_dscr_status = get_rx_dscr_pkt_status(base_dscr);	
	 if(rx_dscr_status == RX_NEW)
	 {
		base_dscr = free_host_rx_dscr_list(base_dscr, num_buffs);
		num_dscr -= num_buffs;
#ifdef DEBUG_MODE
		PRINTD("HwEr:UnexpNewRxDscr\n");
		g_mac_stats.pwrx_unexp_newdscr++;
#endif /* DEBUG_MODE */
	      continue;
	  }		

        	if((is_rx_frame_amsdu(base_dscr) == BTRUE) &&
           		(is_rx_frame_start_buf(base_dscr) != BTRUE))
        	{
#ifdef DEBUG_MODE
            		PRINTD("HwEr:RxNotStart\n");
            		g_mac_stats.pwrx_notstart++;
#endif /* DEBUG_MODE */

            /* Free all the remaining descriptors and break */
            free_host_rx_dscr_list(base_dscr, num_dscr);
            break;
        }

		TROUT_RX_DBG5("%s: num_dscr = %d, num_buffs = %d\n", __FUNCTION__, num_dscr, num_buffs);
		
        /* Call function to process the receive frame */
        process_rx_frame(&g_mac, base_dscr, num_buffs);
        t = num_dscr;
			
        /* Update the number of remaining descriptors by decrementing the    */
        /* number of buffers processed                                       */
        num_dscr -= num_buffs;
	if( num_dscr > t)
		printk("@@@@@@@BUG! cut here, num_buffer > num_dscr!\n");

        /* Increment the number of MPDUs processed */
        num_mpdu++;

		/* Update required MIB for the aggregation of frames received */
    	update_aggr_rx_mib(num_mpdu);

		/* NOTE: this may lead to free dscr buffer twice times, need optimize!!! */
		//chenq mask 2012-11-02
		//if(g_mac_reset_done == BFALSE)
      	base_dscr = free_host_rx_dscr_list(base_dscr, num_buffs);
		/* node free in host_if.c host_frame_completion_fn(), line542 */
    }
    //TROUT_FUNC_EXIT;
}

#ifdef TROUT_TRACE_DBG
//add by Hugh for Rx debug.
void rx_share_ram_show(void)
{
    
}

void host_rx_buff_show(void)
{
	rx_queue_struct *rx_queue = NULL;
	trout_rx_mem_struct *rx_mem = NULL;
	UWORD8 used, free;
	int i, j;
	
	for(i=0; i<NUM_RX_Q; i++)
	{
		used = 0;
		free = 0;
		
		rx_queue = &g_rx_handle->rx_q_info[i];
		rx_mem = (trout_rx_mem_struct *)rx_queue->queue_head;
		TROUT_DBG5("host rxq(%d) detail: 1(used), 0(free)\n", rx_queue->queue_pri);
		for(j=0; j<rx_queue->cell_num; j++)
		{
			if(get_rx_dscr_pkt_status((UWORD32 *)(rx_mem->trout_rx_dscr)) != 0)
			{
				used++;
				TROUT_DBG5("1 ");
			}
			else
			{
				free++;
				TROUT_DBG5("0 ");
			}

			if((j+1)%16 == 0)
				TROUT_DBG5("\n");

			rx_mem++;
		}
		TROUT_DBG4("host rxq(%d): total = %d, used = %d, free = %d, used: %d%%\n", 
				rx_queue->queue_pri, rx_queue->cell_num, used, free, (int)((used*100)/rx_queue->cell_num));
	}
}


#endif	/* TROUT_TRACE_DBG */

#ifdef TROUT_B2B_TEST_MODE
extern UWORD32 rx_count[];

int trout_b2b_rx_low(mac_struct_t *mac, UWORD8 *msg)
{
    wlan_rx_event_msg_t *wlan_rx   = (wlan_rx_event_msg_t*)msg;
    UWORD8              num_dscr   = wlan_rx->num_dscr;
    UWORD32             *base_dscr = (UWORD32 *)(wlan_rx->base_dscr);
    UWORD8              num_buffs  = 0;
    UWORD8              rx_dscr_status = 0;
    UWORD8              num_mpdu   = 0;
    UWORD32  rx_len = 0;
    char *pkt_src = NULL;
    b2b_rx_pkt_s *rx_pkt = NULL;
	
    /* Exception case 0: Base descriptor becomes NULL before the number  */
    /* of descriptors indicated in the interrupt have been processed.    */


    /* Loop till number of descriptors remaining to be processed becomes 0 */
    while(num_dscr != 0)
    {
        if(base_dscr == NULL)
            return -1;

        rx_dscr_status = get_rx_dscr_pkt_status(base_dscr);

        /* Exception Case: The status of receive descriptor passed is not updated */
        rx_count[rx_dscr_status & 3]++;
        if(rx_dscr_status != RX_SUCCESS)
        {
            free_host_rx_dscr_list(base_dscr, num_dscr);
            printk("HwEr:UnexpNewRxDscr\n\r");
            g_mac_stats.pwrx_unexp_newdscr++;
            return -1;
        }
        
        /* of descriptors indicated in the interrupt have been processed.    */
        if(base_dscr == NULL)
        {
#ifdef DEBUG_MODE
            PRINTD2("HwEr:RxUnexpEnd\n\r");
            g_mac_stats.pwrx_unexp_end++;
#endif /* DEBUG_MODE */

            /* Do nothing and break */
            break;
        }

        pkt_src = (UWORD8*)get_host_rxds_buffer_ptr(base_dscr);
        if((((*pkt_src & 0x0C) >> 2) != 2) || get_rx_dscr_frame_len(base_dscr) < 1024)      //don't process non-data frame.
            return 0xAA;
        
        /* Extract the number of buffers from the base descriptor */
        num_buffs = get_rx_dscr_num_buffers(base_dscr);
		if(num_buffs == 0)
		{
			printk("%s: num_buffs is empty!\n", __FUNCTION__);
			BUG();
		}

        /* Exception case 3: The number of buffers exceeds the remaining     */
        /* number of descriptors                                             */
        if(num_buffs > num_dscr)
        {
#ifdef DEBUG_MODE
            PRINTD2("HwEr:RxNumExc\n\r");
            g_mac_stats.pwrx_numexceed++;
#endif /* DEBUG_MODE */

            /* Free all the remaining descriptors and break */
            free_host_rx_dscr_list(base_dscr, num_dscr);
            break;
        }

        /* Call function to process the receive frame */
        
        pkt_src = (UWORD8*)get_host_rxds_buffer_ptr(base_dscr);
        rx_len = get_rx_dscr_frame_len(base_dscr);
        if(rx_len > 1600)
            rx_len = 1600;

        //alloc mem for pkt node and pkt.
        rx_pkt = (b2b_rx_pkt_s *)kmalloc(sizeof(*rx_pkt) + rx_len, GFP_KERNEL);
        if(rx_pkt == NULL)
        {
            printk("malloc rx pkt mem fail.\n");
            
            /* Free all the remaining descriptors and break */
            free_host_rx_dscr_list(base_dscr, num_dscr);
            break;
        }

        rx_pkt->pkt = (char *)(rx_pkt + 1);
        rx_pkt->len = rx_len;
        memcpy(rx_pkt->pkt, pkt_src, rx_len);

        b2b_rx_pkt_add(rx_pkt);
        
        /* Update the number of remaining descriptors by decrementing the    */
        /* number of buffers processed                                       */
        num_dscr -= num_buffs;

        /* Increment the number of MPDUs processed */
        num_mpdu++;

		/* Update required MIB for the aggregation of frames received */
    	update_aggr_rx_mib(num_mpdu);

		/* NOTE: this may lead to free dscr buffer twice times, need optimize!!! */
		//chenq mask 2012-11-01
		//if(g_mac_reset_done == BFALSE)
      		base_dscr = free_host_rx_dscr_list(base_dscr, num_buffs);
		/* node free in host_if.c host_frame_completion_fn(), line542 */
    }

    return 0;
}

#endif

#endif	/* TROUT_WIFI */

