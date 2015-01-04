/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 *
 * Authors:<jinglong.chen@spreadtrum.com>
 * Owners:
 *      hua.chen jinglong.chen
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "wlan_event_q.h"
#include "wlan_common.h"

#define TCP_ACK_WIN_TIMEOUT      (500)
#define TCP_ACK_WIN_H_SIZE       (23168)
#define TCP_ACK_WIN_L_SIZE       (20272)
#define TCP_SESSION_TIMEOUT      (5)

int time_d_value(struct timeval *start, struct timeval *end)
{
	return (end->tv_sec - start->tv_sec)*1000000 + (end->tv_usec - start->tv_usec);
}

static bool is_tcp_data(unsigned char *frame, int len)
{
	int payload;
	if(len <= 54)
		return false;
	payload = len - (34 + (frame[0x2e]>>4)*4);
	if(payload <= 0)
		return false;
	if( !(0x08 == frame[12] && 0x0==frame[13]) )//IP
		return false;
	if( ! (0x06 == frame[23]) )//TCP
		return false;
	return true;
}

static bool is_tcp_ack(unsigned char *frame, int len)
{
	int payload;
	if(len < 54)
		return false;
	payload = len - (34 + (frame[0x2e]>>4)*4);
	if(0 < payload)
		return false;
	if( !(0x08 == frame[12] && 0x0==frame[13]) )
		return false;
	if( ! (0x06 == frame[23]) )
		return false;
	return true;
}

static bool ack_no_loss(unsigned char *frame, int len)
{
	if((54 == len) && (0x10 != frame[0x2f]) )
		return true;
	if(len > 54)
		return true;
	return false;
}

static unsigned int get_data_seq(unsigned char *frame, unsigned int len)
{
	unsigned int seq;
	unsigned char  seq1[4];
	seq1[3] = frame[38];
	seq1[2] = frame[39];	
	seq1[1] = frame[40];
	seq1[0] = frame[41];
	memcpy((char *)(&seq), &seq1[0], 4);
	CLEAR_BIT(seq, 31);
	return seq;
}

static unsigned int get_ack_seq(unsigned char *frame, unsigned int len)
{
	unsigned int seq;
	unsigned char  seq1[4];
	seq1[3] = frame[42];
	seq1[2] = frame[43];	
	seq1[1] = frame[44];
	seq1[0] = frame[45];
	memcpy((char *)(&seq), &seq1[0], 4);
	CLEAR_BIT(seq, 31);
	return seq;
}

static void tcp_session_del(wlan_tcp_session_t  *session)
{
	printkd("DEL TID:0x%x\n", session->tid);
	memset((char *)session, 0, sizeof(wlan_tcp_session_t) - sizeof(m_event_t) );
	return;
}

void tcp_session_updata(const unsigned char vif_id, unsigned char *frame, unsigned int len)
{
	unsigned int    src_ip,dst_ip,tid,seq;
	unsigned short  src_port,dst_port;
	int             i,id;
	wlan_vif_t     *vif;
	wlan_tcp_session_t  *session;
	struct timeval  cur_time;
	if(false == g_wlan.netif[vif_id].tcp_ack_suppress)
		return;
	if(false == is_tcp_data(frame, len) )
		return;
	
	memcpy((char *)(&src_ip),   &frame[0x1a], 4);
	memcpy((char *)(&dst_ip),   &frame[0x1e], 4);
	memcpy((char *)(&src_port), &frame[0x22], 2);
	memcpy((char *)(&dst_port), &frame[0x24], 2);

	tid = (src_ip ^ dst_ip^dst_port^src_port);
	seq = get_data_seq(frame, len);
	vif = &(g_wlan.netif[vif_id]);	
	do_gettimeofday(&cur_time);
	for(i=0, id = -1; i<MAX_TCP_SESSION; i++)
	{
		if( (1 == vif->tcp_session[i].active) && (tid ==vif->tcp_session[i].tid) )
		{
			id = i;
			break;
		}
		
	}
	if(-1 == id)
	{
		for(i=0; i<MAX_TCP_SESSION; i++)
		{
			if(1 == vif->tcp_session[i].active)
			{
				if( (cur_time.tv_sec - vif->tcp_session[i].data_time.tv_sec)  > TCP_SESSION_TIMEOUT)
				{
					tcp_session_del(&vif->tcp_session[i]);
					break;
				}
			}
			else
			{
				break;
			}
		}
		if(MAX_TCP_SESSION == i)
			return;
		vif->tcp_session[i].tid      = tid;
		vif->tcp_session[i].ack_seq  = seq;
		vif->tcp_session[i].ack_time = cur_time;
		vif->tcp_session[i].active   = 1;
		id = i;
		printkd("NEW TID:0x%x\n", tid);
	}
	else
	{
		if( (cur_time.tv_sec - vif->tcp_session[id].data_time.tv_sec)  > TCP_SESSION_TIMEOUT)
		{
			tcp_session_del(&vif->tcp_session[id]);
			return;
		}
	}
	vif->tcp_session[id].data_time = cur_time;
	vif->tcp_session[id].data_seq  = seq;
	return;
}

m_event_t *wlan_tcpack_q(wlan_vif_t *vif, unsigned char *frame, unsigned int len)
{
	unsigned int    src_ip,dst_ip,tid,i;
	unsigned short  src_port,dst_port;
	m_event_t      *event_q;
	
	event_q  = &(vif->event_q[1] );
	if(false == vif->tcp_ack_suppress)
		return event_q;
	if(false == is_tcp_ack(frame, len) )
		return event_q;
	
	memcpy((char *)(&dst_ip),   &frame[0x1a], 4);
	memcpy((char *)(&src_ip),   &frame[0x1e], 4);
	memcpy((char *)(&src_port), &frame[0x22], 2);
	memcpy((char *)(&dst_port), &frame[0x24], 2);
	tid = (src_ip ^ dst_ip^src_port^dst_port);
	for(i=0; i<MAX_TCP_SESSION; i++)
	{
		if( (1 == vif->tcp_session[i].active) && (tid == vif->tcp_session[i].tid) )
		{
			event_q = &(vif->tcp_session[i].event_q);
			break;
		}
	}
	return event_q;
}

int wlan_tcpack_tx(wlan_vif_t *vif, int *done)
{
	int            i,ack_cnt,usec,time_dt, seq_dt,index,retry,len, send_pkt;
	unsigned int   seq;
	tx_msg_t      *msg;
	struct timeval cur_time;
	wlan_tcp_session_t *session;
	m_event_t     *event_q;
	txfifo_t      *tx_fifo;
	unsigned char *frame;
	
	retry = *done = 0;
	tx_fifo = &(vif->txfifo);
	do_gettimeofday(&cur_time);

	for(index =0; index < MAX_TCP_SESSION; index++)
	{
		session = &(vif->tcp_session[index]);
		event_q = &(session->event_q);
		ack_cnt = event_q->event_cnt;
		if(0 == ack_cnt)
			continue;
		usec     = time_d_value(&(session->ack_time), &cur_time);
		time_dt  = (session->data_seq >= session->ack_seq)?(session->data_seq - session->ack_seq):0;	
		if((1 == session->active) && (time_dt < TCP_ACK_WIN_H_SIZE) && (usec < TCP_ACK_WIN_TIMEOUT) )
		{
			retry++;
			continue;
		}	
		for(i=0, send_pkt = 0; i<ack_cnt; i++)
		{
			msg = get_event(event_q);
			if(NULL == msg)
				break;
			frame  = msg->slice[0].data;
			len    = msg->slice[0].len;
			seq    = get_ack_seq(frame, len);
			seq_dt = seq - session->ack_seq;
			
			if( (i > (ack_cnt -2)) || (seq_dt >= TCP_ACK_WIN_L_SIZE ) || (0 == session->active) || (true == ack_no_loss(frame, len) )  )
			{
				if( TX_FIFO_FULL == tx_fifo_in(tx_fifo, msg) )
				{
					retry++;
					continue;
				}
				session->ack_seq = seq;
				do_gettimeofday( &(session->ack_time) );
				send_pkt++;
			}
			(*done)++;
			dev_kfree_skb((struct sk_buff  *)(msg->p));
			free_event((void *)msg, event_q );
		}
		for(i=0; i<send_pkt; i++)
		{
			trans_up();
		}
	}
	if(retry>0)
		return ERROR;
	return OK;	
}

int wlan_tcpack_buf_malloc(wlan_vif_t *vif)
{
	int index;
	m_event_conf_t q_conf = {0};
	q_conf.event_size  = sizeof(tx_msg_t);
	q_conf.max_events  = 40;
	q_conf.highThres   = 100;
	q_conf.lowThres    = 0;
	q_conf.weight      = 100;
	for(index=0; index < MAX_TCP_SESSION; index++)
	{
		event_q_init( &(vif->tcp_session[index].event_q), &q_conf);
		printkd("[event][%d][%d][0x%x]\n", vif->id,index, &(vif->tcp_session[index].event_q) );
	}
	return OK;
}

int wlan_tcpack_buf_free(wlan_vif_t *vif)
{
	int index;
	for(index=0; index < MAX_TCP_SESSION; index++)
	{
		event_q_deinit( &(vif->tcp_session[index].event_q) );
	}
	return OK;
}

int wlan_rx_buf_decode(unsigned char *buf, unsigned int max_len)
{
	unsigned int     p = 0;
	r_msg_hdr_t     *msg = NULL;
	unsigned char    vif_id;	
	unsigned char   *frame = NULL;
	unsigned short   len;
	unsigned char    event;
	if((NULL == buf) || (0 == max_len))
	{
		printke("[%s][ERROR]\n", __func__);
		return OK;
	}
	buf      = buf + 8;
	msg      = (r_msg_hdr_t *)(buf);
	max_len  = max_len - 8;
	while(p < max_len)
	{
		
		vif_id = msg->mode;
		frame  = (unsigned char *)(msg+1);
		len    = msg->len;
		if( (0xFF == msg->type) || (0xFF == msg->subtype) )
			break;
		if(HOST_SC2331_PKT == msg->type)
		{
			frame = frame + msg->subtype;
			len   = len   - msg->subtype;
			tcp_session_updata(vif_id, frame, len);
		}
		p = p + sizeof(t_msg_hdr_t) + ALIGN_4BYTE(msg->len);
		msg = (r_msg_hdr_t *)(buf+p);
	}
	return OK;	

}

int wlan_tx_buf_decode(unsigned char *buf, unsigned int max_len)
{
	int i,len;
	tx_big_hdr_t  *big_hdr = buf;
	t_msg_hdr_t   *hdr;
	unsigned char *frame;
	hdr = (t_msg_hdr_t  *)(big_hdr + 1);
	for(i=0; i<big_hdr->msg_num; i++)
	{
		if( (hdr->type > 2) || (hdr->subtype > 47) )
			break;
		if(HOST_SC2331_PKT == hdr->type)
		{
			frame = (char *)(hdr) + 38;
			len   = hdr->len;
		}
		hdr = TX_MSG_NEXT_MSG(hdr);
	}
	return 0;
}



