/*
 * Copyright (C) 2014 Spreadtrum Communications Inc.
 *
 * Authors:<jinglong.chen@spreadtrum.com>
 * Owner:
 *      jinglong.chen
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

#include "wlan_common.h"
#include "wlan_event_q.h"
#include "wlan_fifo.h"
#include "wlan_cmd.h"
#include "wlan_cfg80211.h"
#include "wlan_wapi.h"

wlan_info_t g_wlan = { 0 };
unsigned int g_dbg = 0xFFFFFFFF;

void core_up(void)
{
	up(&(g_wlan.wlan_core.sem));
}
void core_down(void)
{
	down(&(g_wlan.wlan_core.sem));
}
void trans_up(void )
{
	up(&(g_wlan.wlan_trans.sem));
}
void trans_down(void )
{
	down(&(g_wlan.wlan_trans.sem));
}

bool stop_net(unsigned char id )
{
	return false;
	if(NULL == g_wlan.netif[id].ndev)
		return false;	
	if( ! netif_queue_stopped(g_wlan.netif[id].ndev) )
	{
		netif_stop_queue(g_wlan.netif[id].ndev);
		return true;
	}
	return false;
}

bool wake_net(unsigned char id)
{
	return false;
	if( (NULL == g_wlan.netif[id].ndev) || (1 == g_wlan.sync.exit) )
		return false;
	if(netif_queue_stopped(g_wlan.netif[id].ndev) )
	{
		netif_wake_queue(g_wlan.netif[id].ndev);
		return true;
	}
	return false;
}

static int hw_rx(const unsigned short chn, unsigned char *buf, unsigned int *len)
{
	int ret;
	unsigned int sc2331_tx_cnt;
	unsigned int read_len = 0;	
	static unsigned int cnt = 0;
	if(NULL == buf)
		return ERROR;
#if 0
	ret = sdio_read_wlan(chn, buf, HW_RX_SIZE);
	*len = HW_RX_SIZE;
#else
	ret = sdio_dev_read(chn, buf, &read_len);
	*len = read_len;
#endif
	if(0 != ret)
	{
		printke("[chn %d][sdio_read] err:%d\n", chn, ret);
		return ERROR;
	}
	if(WLAN_HEX_DBG)
	{
		unsigned char str[64] = {0};
		printkd("[sdio_read][%d][%d][%d]\n", chn, 1024, cnt);
		sprintf(str, "[HEX-RX][%d]:", 1024);
		hex_dump(str, strlen(str), buf, 1024);
	}
	printkp("[rx][%d]\n", chn );
	cnt++;
	*len = read_len;
	g_wlan.hw.rx_cnt++;
	return OK;
}

static int hw_tx(const unsigned short chn, unsigned char *buf, unsigned int len)
{
	int ret;
	static unsigned int cnt = 0;
	static unsigned int skb = 0;
	tx_big_hdr_t *big_hdr;

	big_hdr = (tx_big_hdr_t *)buf;
	if ((PKT_AGGR_NUM < big_hdr->msg_num) || (0 == big_hdr->msg_num))
	{
		ASSERT();
		return ERROR;
	}	
	if(WLAN_HEX_DBG)
	{
		unsigned char str[64] = {0};
		sprintf(str, "[HEX-TX][%d]:", len);
		hex_dump(str, strlen(str), buf, 128);
	}
	big_hdr->tx_cnt = g_wlan.hw.tx_cnt;
	printkp("[tx][%d][%d]\n",big_hdr->tx_cnt, chn);
	len = (len+1023)&0xFC00;
	ret = sdio_dev_write(chn, buf, len);
	if(0 != ret)
	{
		ASSERT();
		return ERROR;
	}
	skb = skb + big_hdr->msg_num;
	cnt++;
	if(100 == cnt)
	{
		printkd("w%d\n", skb);
		cnt = 0;
		skb = 0;
	}
	g_wlan.hw.tx_cnt++;
	return OK;
}

static int wlan_rx_skb_process(const unsigned char vif_id, unsigned char *pData, unsigned short len)
{
	struct sk_buff *skb;
	struct net_device *ndev = g_wlan.netif[vif_id].ndev;
	if((NULL == pData) || (0 == len) || (NULL == ndev))
	{
		printkd("[%s][%d][err]\n", __func__, (int )vif_id);
		return ERROR;
	}
	skb = dev_alloc_skb(len + NET_IP_ALIGN);
	if(NULL == skb)
		return ERROR;
	skb_reserve(skb, NET_IP_ALIGN);
	memcpy(skb->data,   pData,  len);

	skb_put(skb, len);
	skb->dev = ndev;
	skb->protocol = eth_type_trans(skb, ndev);
	ndev->stats.rx_packets++;
	printkp("rx_skb:%d\n", (int)(ndev->stats.rx_packets) );
	ndev->stats.rx_bytes += skb->len;
	if ( in_interrupt() )
		netif_rx(skb);
	else
		netif_rx_ni(skb);
    return OK;
}

static int wlan_rx_wapi_process(const unsigned char vif_id, unsigned char *pData, unsigned short len)
{
	struct ieee80211_hdr_3addr *addr;
	int decryp_data_len = 0;
	struct sk_buff *skb;
	u8 snap_header[6] = { 0xaa, 0xaa, 0x03,0x00, 0x00, 0x00};
	wlan_vif_t   *vif;
	struct net_device *ndev;

	vif  = &(g_wlan.netif[vif_id]);
	ndev = vif->ndev;
	if((NULL == pData) || (0 == len) || (NULL == ndev))
	{
		printkd("[%s][%d][err]\n", __func__, (int )vif_id);
		return ERROR;
	}
	addr = (struct ieee80211_hdr_3addr *)pData;
	skb = dev_alloc_skb(len + NET_IP_ALIGN);
	if(NULL == skb)
		return ERROR;
	skb_reserve(skb, NET_IP_ALIGN);

	decryp_data_len = wlan_rx_wapi_decryption(vif, (unsigned char  *)addr, 24, (len -24),(skb->data + 12));
	if (decryp_data_len == 0)
	{
		dev_kfree_skb(skb);
		return ERROR;
	}
	if ( memcmp((skb->data + 12), snap_header, sizeof(snap_header)) == 0)
	{
		skb_reserve(skb, 6);
		memcpy(skb->data,addr->addr1, 6);
		memcpy(skb->data + 6,addr->addr2, 6);
		skb_put(skb, (decryp_data_len + 6));
	}
	else
	{
		/* copy eth header */
		memcpy(skb->data,addr->addr3, 6);
		memcpy(skb->data + 6, addr->addr2, 6);
		skb_put(skb, (decryp_data_len + 12) );
	}
	skb->dev = ndev;
	skb->protocol = eth_type_trans(skb, ndev);
	ndev->stats.rx_packets++;
	printkp("rx_skb:%d\n", (int)(ndev->stats.rx_packets) );
	ndev->stats.rx_bytes += skb->len;
	if ( in_interrupt() )
		netif_rx(skb);
	else
		netif_rx_ni(skb);
	return OK;
}

void wlan_rx_chn_isr(int chn)
{
	static unsigned int cnt = 1;
	printkp("[irq][%d]\n", cnt);
	cnt++;
	trans_up();
}

static unsigned char prio_to_q_id(unsigned char  *eth_hdr)
{
	unsigned short  eth_type;
	int             priority;
	unsigned char   q_id = EVENT_Q_ID_2;
	
	eth_type  = ( (eth_hdr[12]<<8) | eth_hdr[13] );
	if(IP_TYPE != eth_type)
		return q_id;
	priority = eth_hdr[15]&0xE0;
	switch(priority)
	{
		case 0x20:
		case 0x40:
			q_id = EVENT_Q_ID_1;
			break;
			
		case 0x80:
		case 0xA0:
			q_id = EVENT_Q_ID_3;
			break;
			
		case 0xc0:
		case 0xe0:
			q_id = EVENT_Q_ID_4;
			break;

		default:
			q_id = EVENT_Q_ID_2;
			break;
	}
	return q_id;
}

static int wlan_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned char q_id;
	int addr_len = 0;
	wlan_vif_t   *vif;
	tx_msg_t     *event;
	m_event_t    *event_q;
	struct sk_buff *wapi_skb;

	vif     = ndev_to_vif(dev);
	q_id    = EVENT_Q_ID_1;//q_id    = prio_to_q_id(skb->data);
	event_q = &(vif->event_q[q_id]);
	if(event_q->event_cnt > event_q->highThres)
	{
		if( stop_net(vif->id) )
			printke("[stop_net %d:%d][%d,%d]\n", vif->id, q_id, event_q->event_cnt, event_q->highThres);
	}
	event = alloc_event(event_q);
	if(NULL == event)
	{
		printkd("L-PKT\n");
		if( stop_net(vif->id) )
			printke("[stop_net %d:%d][%d,%d]\n", vif->id, q_id, event_q->event_cnt, event_q->highThres);			
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
#ifdef WIFI_DRV_WAPI
	if (vif->cfg80211.cipher_type == WAPI && vif->cfg80211.connect_status == ITM_CONNECTED &&
		vif->cfg80211.key_len[PAIRWISE][vif->cfg80211.key_index[PAIRWISE]] != 0 &&(*(u16 *)((u8 *)skb->data + ETH_PKT_TYPE_OFFSET) != 0xb488))
	{
		wapi_skb = dev_alloc_skb(skb->len+100+NET_IP_ALIGN);
		skb_reserve(wapi_skb, NET_IP_ALIGN);
		memcpy( wapi_skb->data, skb->data, ETHERNET_HDR_LEN );
		addr_len = wlan_tx_wapi_encryption(vif, skb->data, (skb->len - ETHERNET_HDR_LEN),  ( (unsigned char *)(wapi_skb->data) + ETHERNET_HDR_LEN)  );
		addr_len = addr_len + ETHERNET_HDR_LEN;
		skb_put(wapi_skb, addr_len);
		memset(event, 0, sizeof(tx_msg_t) );
		event->p = (void *)wapi_skb;
		event->slice[0].data = wapi_skb->data;
		event->slice[0].len  = wapi_skb->len;
		event->hdr.mode = vif->id;
		event->hdr.type = HOST_SC2331_PKT;
		event->hdr.subtype = 0;
		event->hdr.len = event->slice[0].len;
		vif->ndev->stats.tx_bytes += wapi_skb->len;
		vif->ndev->stats.tx_packets++;
		dev->trans_start = jiffies;
		dev_kfree_skb(skb);
	}
	else
#endif
	{
		event->p = (void *)skb;
		event->slice[0].data = skb->data;
		event->slice[0].len  = skb->len;
		event->hdr.mode = vif->id;
		event->hdr.type = HOST_SC2331_PKT;
		event->hdr.subtype = 0;
		event->hdr.len = skb->len;
		vif->ndev->stats.tx_bytes += skb->len;
		vif->ndev->stats.tx_packets++;
		dev->trans_start = jiffies;
	}

	post_event( (unsigned char *)event, event_q );
	core_up();
	return NETDEV_TX_OK;
}

static int wlan_rx_process(unsigned char *buf, unsigned int max_len)
{
	static unsigned int cnt = 0;
	static unsigned int skb = 0;
	unsigned int     p = 0;
	r_msg_hdr_t     *msg = NULL;
	unsigned char    vif_id;	
	unsigned char   *pData = NULL;
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
		pData  = (unsigned char *)(msg+1);
		len    = msg->len;
		
		if( (0xFF == msg->type) || (0xFF == msg->subtype) )
			break;
		if(HOST_SC2331_PKT == msg->type)
		{
			pData = pData + msg->subtype;
			len   = len   - msg->subtype;
			wlan_rx_skb_process(vif_id, pData, len);
		}
#ifdef WIFI_DRV_WAPI
		else if(HOST_SC2331_WAPI == msg->type)
		{
			wlan_rx_wapi_process(vif_id, pData, len);
		}
#endif
		else if(SC2331_HOST_RSP == msg->type)
		{
			wlan_rx_rsp_process(vif_id, msg);
		}
		else if(HOST_SC2331_CMD == msg->type)
		{
			event = msg->subtype;
			wlan_rx_event_process(vif_id, event, pData, len);
		}
		else
		{
			printke("[%s][RX DATA ERR]\n", __func__);
			break;
		}
		p = p + sizeof(t_msg_hdr_t) + ALIGN_4BYTE(msg->len);
		msg = (r_msg_hdr_t *)(buf+p);
		skb++;
	}
	cnt++;
	if(100 == cnt)
	{
		printkd("r%d\n", skb);
		cnt = 0;
		skb = 0;
	}
	return OK;
}

static int wlan_open(struct net_device *dev)
{
	printkd("%s open\n", dev->name );
	netif_start_queue(dev);
	return 0;
}
static int wlan_close(struct net_device *dev)
{
	printkd("%s %s enter\n", __func__, dev->name );
	netif_stop_queue(dev);
	printkd("%s %s ok\n", __func__, dev->name );
	return 0;
}
static struct net_device_stats *wlan_status(struct net_device *dev)
{
	return &(dev->stats);
}

static void wlan_tx_timeout(struct net_device *dev)
{
	wlan_vif_t   *vif;
	printke("%s tx timeout\n", dev->name);
	vif = ndev_to_vif(dev);
	
	if (!netif_carrier_ok(dev))
		netif_carrier_on(dev);
	dev->trans_start = jiffies;
	wake_net(vif->id);
	core_up();
	return;
}

static int wlan_ioctl(struct net_device *ndev, struct ifreq *req, int cmd)
{
	return 0;
}

struct net_device_ops wlan_ops = 
{
	.ndo_open                 = wlan_open,
	.ndo_stop                 = wlan_close,
	.ndo_start_xmit           = wlan_xmit,
	.ndo_get_stats            = wlan_status,
	.ndo_tx_timeout           = wlan_tx_timeout,
	.ndo_do_ioctl             = wlan_ioctl,
};

static void wlan_early_suspend(struct early_suspend *es)
{
	/* suspend clear bit0 of screen_on */
	if (atomic_read(&g_wlan.screen_on) & 0x1)
		atomic_dec(&g_wlan.screen_on);
	printkd("[%s]\n", __func__);
	wlan_cmd_sleep(1);
}

static void wlan_late_resume(struct early_suspend *es)
{
	/* resume set bit0 for resume,
	 * set bit1 for status change, used by wlan_wakeup_cp
	 */
	atomic_set(&g_wlan.screen_on, 0x3);
	printkd("[%s]\n", __func__);
	wlan_cmd_sleep(2);
}

int wlan_wakeup(void )
{
	int ret;
	if(0 != g_wlan.hw.wakeup)
		return OK;
	wake_lock(&g_wlan.hw.wlan_lock);
	printkd("time[1]\n");
	mod_timer(&(g_wlan.hw.wakeup_timer),  jiffies + msecs_to_jiffies(g_wlan.hw.wakeup_time) );
	g_wlan.hw.wakeup    = 1;
	g_wlan.hw.can_sleep = 0;
	return ret;
}

void wlan_sleep(void )
{
	if( (1 != g_wlan.hw.wakeup) || (0 == g_wlan.hw.can_sleep) )
		return;
	g_wlan.hw.wakeup = 0;
	printkd("time[4]\n");
	wake_unlock(&g_wlan.hw.wlan_lock);
	return;
}

void wakeup_timer_func(unsigned long data)
{
	if((g_wlan.wlan_trans.sem.count <= 0) && (0 == g_wlan.hw.can_sleep) )
	{
		printkd("timer[3]\n");
		g_wlan.hw.can_sleep = 1;
		trans_up();
		return;
	}
	printkd("timer[2]\n");
	mod_timer(&(g_wlan.hw.wakeup_timer),  jiffies + msecs_to_jiffies(g_wlan.hw.wakeup_time) );
}

int wlan_rx(rxfifo_t *rx_fifo, int cnt)
{
	int i,num,rx_cnt,ret;
	rx_cnt = 0;
	num = rx_fifo_used(rx_fifo);
	if(num > cnt)
		num = cnt;
	for(i=0; i < num ; i++)	
	{
		ret = rx_fifo_out(rx_fifo, wlan_rx_process);
		if(ERROR == ret)
			break;
		rx_cnt++;
	}
	return rx_cnt;
}

int wlan_tx(txfifo_t *tx_fifo, m_event_t *event_q, int cnt)
{
	int tx_cnt,i,ret;
	tx_msg_t  *event;
	tx_cnt = 0;
	for(i=0; i<cnt; i++)
	{
		event = get_event(event_q);
		if(NULL == event)
		{
			ret = TX_FIFO_EMPTY;
			break;
		}
		ret = tx_fifo_in(tx_fifo, event);
		if(TX_FIFO_FULL == ret)
		{
			break;
		}
		trans_up();
		if( HOST_SC2331_CMD == event->hdr.type) 
		{
			if((event->slice[0].len > 0) &&(NULL != event->slice[0].data))
				kfree(event->slice[0].data);
		}
		else if(HOST_SC2331_PKT == event->hdr.type)
		{
			if((NULL != event->p))
				dev_kfree_skb((struct sk_buff  *)(event->p));
		}
		else
		{}
		memset((unsigned char *)event,  0,  sizeof(tx_msg_t));
		free_event((void *)event, event_q );
		tx_cnt++;
	}
	return tx_cnt ? tx_cnt : ret;
}

int wmm_calc(wlan_vif_t *vif, unsigned short *q_event_num)
{
	int            i,weight;
	unsigned short event_q_array[EVENT_Q_MAX_ID][2];
	q_event_num[EVENT_Q_ID_0] = vif->event_q[EVENT_Q_ID_0].event_cnt;
	for(i=EVENT_Q_ID_1, weight = 0; i<EVENT_Q_MAX_ID; i++)
	{
		event_q_array[i][0] = vif->event_q[i].event_cnt;
		event_q_array[i][1] = vif->event_q[i].weight;
		if(0 == event_q_array[i][0])
			continue;
		weight = weight + event_q_array[i][1];
	}
	for(i=EVENT_Q_ID_1; i<EVENT_Q_MAX_ID; i++)
	{
		if(0 == weight)
		{
			q_event_num[i] = 0;
			continue;
		}
		q_event_num[i] = (event_q_array[i][0])*(event_q_array[i][1])/weight;
	}
	if( (0 == weight) &&  (0 == q_event_num[EVENT_Q_ID_0]) )
		return ERROR;
	return OK;
}

#define WLAN_CORE_SLEEP_TIME	600
void thread_sched_policy(wlan_thread_t *thread)
{
	int ret;
	struct sched_param param;
	param.sched_priority = thread->prio;
	ret = sched_setscheduler(current, SCHED_FIFO,   &param);
	printkd("sched_setscheduler, prio:%d,ret:%d\n", param.sched_priority, ret);
	return;
}

void thread_sleep_policy(wlan_thread_t *thread)
{
	if(thread->null_run > thread->max_null_run)
	{
		usleep_range(thread->idle_sleep - 50, thread->idle_sleep + 50);
		//thread->null_run = 0;
	}
	return;
}
static int wlan_core_thread(void *data)
{
	wlan_vif_t    *vif;	
	rxfifo_t      *rx_fifo;
	txfifo_t      *tx_fifo;
	m_event_t     *event_q;
	int            i,ret,retry,vif_id, done, sem_count,q_index,need_tx,tx_cnt,tx_pkt;
	int sleep_flag;
	unsigned long timeout;

	sleep_flag = timeout = 0;
	rx_fifo = &(g_wlan.rxfifo);
	sema_init(&g_wlan.wlan_core.sem, 0);
	printke("%s enter new\n", __func__);
	up(&(g_wlan.sync.sem));
	core_down();
	do
	{	
		retry = done = 0;
		sem_count = g_wlan.wlan_core.sem.count ;
		ret    = wlan_rx(rx_fifo, 1);
		if(1 == ret)
			done++;
		for(vif_id = NETIF_0_ID; vif_id < WLAN_MAX_ID; vif_id++)
		{
			vif     = &(g_wlan.netif[vif_id]);
			tx_fifo = &(vif->txfifo);			
			for( q_index=EVENT_Q_ID_0;  q_index <= EVENT_Q_ID_1;  q_index++ )
			{
				event_q      = &(vif->event_q[q_index]);
				need_tx      = vif->event_q[q_index].event_cnt;
				for(i=0; i<need_tx; i++)
				{
					ret = wlan_tx(tx_fifo, event_q, 1);
					if (TX_FIFO_FULL == ret) {
						if (0 == sleep_flag) {
							timeout = jiffies + msecs_to_jiffies(WLAN_CORE_SLEEP_TIME);
							sleep_flag = 1;
						} else {
							if (time_after(jiffies, timeout)) {
								printke("%s [TIMEOUT][%lu] jiffies:%lu\n",
										__func__, timeout, jiffies);
								msleep(300);
								sleep_flag = 0;
							}
						}
						retry++;
						continue;
					} else if (TX_FIFO_EMPTY == ret) {
						/* no problem as need_tx, excpt get_event erro */
						ASSERT();
						msleep(10);
						retry++;
						continue;
					}
					if (sleep_flag)
						sleep_flag = 0;
					done = done + ret;
					ret  = wlan_rx(rx_fifo, 1);
					done = done + ret;
				}
			}
		}
		if(g_wlan.sync.exit)
			break;
		if((0 == done) && (0 == retry) )
			done = (  (0 == sem_count)?(1):(sem_count) );
		for(i=0; i<done; i++)
		{
			core_down();
		}
	}while(!kthread_should_stop());
	
	printke("%s exit!\n", __func__);
	up(&(g_wlan.sync.sem));
	return 0;
}

static int check_valid_chn(int flag, unsigned short status, sdio_chn_t *chn_info)
{
	int i,index = -1;

	if ((status & 0xFF) == 0xFF)
		status &= 0xFF00;
	if ((status & 0xFF00) == 0xFF00)
		status &= 0xFF;
	if(1 == flag)
		status = ( status & (chn_info->bit_map) );	
	else
		status = ( (status & chn_info->bit_map) ^ (chn_info->bit_map) );
	if(0 == status)
		return -1;
	for(i=0; i < chn_info->num; i++)
	{
		if( status & (0x1 << chn_info->chn[i]) )
		{
			index = chn_info->chn[i];		
			break;
		}
	}
	return index;
}

#define WLAN_GPIO_PULL_CNT	8
static int wlan_wakeup_cp(unsigned long timeout)
{
	static int gpio_cnt = WLAN_GPIO_PULL_CNT;
	static unsigned long last_time;
	int screen;

	screen = atomic_read(&g_wlan.screen_on);
	/* suspend -> resume, reset gpio_cnt
	 * gpio_cnt used when suspend change to resume
	 * This make sure resume_cmd is recieved by CP
	 */
	if (screen & 0x2) {
		atomic_sub(0x2, &g_wlan.screen_on);
		gpio_cnt = WLAN_GPIO_PULL_CNT;
	}
	/* resume */
	if (screen & 0x1) {
		if (gpio_cnt) {
			gpio_cnt--;
			last_time = jiffies;
		} else if (time_after(jiffies, last_time + timeout))
			last_time = jiffies;
		else
			return 0;
	}
	/* wake up cp */
	screen = set_marlin_wakeup(0, 1);
	if (screen)
		gpio_cnt = WLAN_GPIO_PULL_CNT;

	return screen;
}

static int wlan_trans_thread(void *data)
{
	int i,vif_id,ret,done, retry,sem_count,send_pkt, index,pkt_num;
	rxfifo_t       *rx_fifo;
	txfifo_t       *tx_fifo;
	wlan_vif_t     *vif;	
	sdio_chn_t     *tx_chn;
	sdio_chn_t     *rx_chn;
	unsigned short  status;
	int             tx_retry_cnt;
	int             wake_flag;
	unsigned long   gpio_time;
	bool            tx_retry_flag = false;
	wlan_thread_t  *thread;
	
	thread = &(g_wlan.wlan_trans);
	sema_init(&g_wlan.wlan_trans.sem, 0);
	sdiodev_readchn_init(8, (void *)wlan_rx_chn_isr, 1);
	sdiodev_readchn_init(9, (void *)wlan_rx_chn_isr, 1);	
	rx_chn       = &(g_wlan.hw.sdio_rx_chn);
	tx_chn       = &(g_wlan.hw.sdio_tx_chn);
	rx_fifo      = &(g_wlan.rxfifo);
	up(&(g_wlan.sync.sem));
	printke("%s enter\n", __func__);

	g_wlan.wlan_trans.null_run     = 0;
	g_wlan.wlan_trans.max_null_run = 200;
	g_wlan.wlan_trans.idle_sleep   = 200;
	g_wlan.wlan_trans.prio         = 90;
	
	thread_sched_policy(thread);
	trans_down();
	
	wake_flag = 1;
	gpio_time = msecs_to_jiffies(1600);
	do
	{
		thread_sleep_policy(thread);
		send_pkt  = retry = done = 0;
		sem_count = g_wlan.wlan_trans.sem.count;
		if (wake_flag)
			wake_lock(&g_wlan.hw.wlan_lock);
		
RX:
		if(! gpio_get_value(SDIO_RX_GPIO) )
		{
#ifdef WLAN_THREAD_SLEPP_POLICE
			if(true == rx_chn->gpio_high)
			{
				rx_chn->gpio_high    = false;
				rx_chn->timeout_flag = false;
			}
#endif
			goto TX;
		}
		else
		{
#ifdef WLAN_THREAD_SLEPP_POLICE
			if(false == rx_chn->gpio_high)
			{
				rx_chn->gpio_high    = true;
			}
#endif
		}
		/* wlan_wakeup(); */
		ret   = sdio_chn_status( rx_chn->bit_map, &status);
		index = check_valid_chn(1, status, rx_chn);
		if(index < 0)
		{
#ifdef WLAN_THREAD_SLEPP_POLICE
			if(false == rx_chn->timeout_flag)
			{
				rx_chn->timeout_flag = true;
				rx_chn->timeout      = jiffies + msecs_to_jiffies(rx_chn->timeout_time); 
			}
			else
			{
				if ( time_after(jiffies, rx_chn->timeout) )
				{
					printke("[SDIO_RX_CHN][TIMEOUT][%lu] jiffies:%lu\n",
						rx_chn->timeout_time, jiffies);
					msleep(300);
					rx_chn->timeout_flag = false;
				}
			}
#endif		
			goto TX;
		}		
#ifdef	WLAN_THREAD_SLEPP_POLICE
		if(true == rx_chn->timeout_flag)
		{
			rx_chn->timeout_flag = false;
		}
#endif
		if(14 == index)
		{
			mdbg_sdio_read();
			goto TX;
		}
		if(11 == index)
		{
			mdbg_at_cmd_read();
			goto TX;
		}
		if(15 == index)
		{
			mdbg_loopcheck_read();
			goto TX;
		}
		ret = rx_fifo_in(index, rx_fifo, hw_rx);
		if(OK != ret )
		{
			retry++;
			goto TX;
		}
		core_up();

TX:	
		for(vif_id = NETIF_0_ID; vif_id < WLAN_MAX_ID; vif_id++ )
		{
			vif     = &(g_wlan.netif[vif_id]);
			tx_fifo = &(vif->txfifo);
			ret = tx_fifo_used(tx_fifo);
			if(0 == ret)
				continue;
			/* wlan_wakeup() */
			if (wlan_wakeup_cp(gpio_time)) {
				retry++;
				continue;
			}
			ret = sdio_chn_status(tx_chn->bit_map, &status);
			index = check_valid_chn(0, status, tx_chn);
			if(index < 0)
			{
#ifdef WLAN_THREAD_SLEPP_POLICE
				if(false == tx_chn->timeout_flag)
				{
					tx_chn->timeout_flag = true;
					tx_chn->timeout      = jiffies + msecs_to_jiffies(tx_chn->timeout_time); 
				}
				else
				{
					if ( time_after(jiffies, tx_chn->timeout) )
					{
						printke("[SDIO_TX_CHN][TIMEOUT][%lu] jiffies:%lu\n",
							tx_chn->timeout_time, jiffies);
						msleep(300);
						tx_chn->timeout_flag = false;
					}
				}
#endif				
				retry++;
				continue;
			}
#ifdef WLAN_THREAD_SLEPP_POLICE
			if(true == tx_chn->timeout_flag)
			{
				tx_chn->timeout_flag = false;
			}
#endif
			ret    = tx_fifo_out(vif_id, index, tx_fifo, hw_tx, &send_pkt);
			if(OK != ret)
			{
				if(HW_WRITE_ERROR == ret)
					retry++;
				continue;
			}
			done = done + send_pkt;
		}
		
		if (g_wlan.sync.exit) {
			wake_unlock(&g_wlan.hw.wlan_lock);
			break;
		}
		/* wlan_sleep(); */
		
		if(gpio_get_value(SDIO_RX_GPIO))
		{
			if(g_wlan.wlan_trans.sem.count - done <= 1)
			{
				done = (g_wlan.wlan_trans.sem.count > 0)?(g_wlan.wlan_trans.sem.count-1):(0); 
			}
		}
		else
		{
			if( (0 == done) && (0 == retry) )
				done = (  (0 == sem_count)?(1):(sem_count) );
		}
		if(done > 0)
			g_wlan.wlan_trans.null_run= 0;
		else
			g_wlan.wlan_trans.null_run++;		
		if (done >= g_wlan.wlan_trans.sem.count) {
			wake_flag = 1;
			wake_unlock(&g_wlan.hw.wlan_lock);
		} else
			wake_flag = 0;
		for(i=0; i<done; i++)
		{
			trans_down();
		}
	}while(!kthread_should_stop());
	sdiodev_readchn_uninit(8);
	sdiodev_readchn_uninit(9);
	mdbg_sdio_read();
	del_timer_sync(&(g_wlan.hw.wakeup_timer));
	printke("%s exit\n", __func__);
	up(&(g_wlan.sync.sem));
	return OK;
}

static int wlan_tx_buf_alloc(wlan_vif_t *vif)
{
	m_event_conf_t q_conf = {0};
	txfifo_conf_t  fifo_conf  = {0};
	int  ret,q_id;
	q_conf.event_size  = sizeof(tx_msg_t);
	q_conf.max_events  = 5;
	q_conf.highThres   = 100;
	q_conf.lowThres    = 0;
	q_conf.weight      = 100;
	ret = event_q_init(&(vif->event_q[EVENT_Q_ID_0]), &q_conf);
	if(ERROR == ret)
		return ERROR;

	q_conf.max_events  = 150;
	q_conf.highThres   = 130;
	q_conf.lowThres    = 10;
	for(q_id = EVENT_Q_ID_1; q_id < EVENT_Q_MAX_ID; q_id++)
	{
		ret = event_q_init(&(vif->event_q[q_id]), &q_conf);
		if(ERROR == ret)
			return ERROR;
	}
	vif->event_q[EVENT_Q_ID_1].weight = 40;
	vif->event_q[EVENT_Q_ID_2].weight = 30;
	vif->event_q[EVENT_Q_ID_3].weight = 20;
	vif->event_q[EVENT_Q_ID_4].weight = 10;
		
	fifo_conf.cp2_txRam   = HW_TX_SIZE - sizeof(tx_big_hdr_t);
	fifo_conf.max_msg_num = PKT_AGGR_NUM;
	fifo_conf.size        = 1024*256;
	ret =  tx_fifo_alloc(&(vif->txfifo), &fifo_conf);
	if(ERROR == ret)
		return ERROR;
	return OK;
}

static int wlan_tx_buf_free(wlan_vif_t *vif)
{
	int         q_id, ret,num,i;
	m_event_t  *event_q;
	tx_msg_t   *event;
	for(q_id=EVENT_Q_ID_0; q_id < EVENT_Q_MAX_ID; q_id++)
	{
		event_q = &(vif->event_q[q_id]);
		num = event_q->event_cnt;
		for(i=0; i < num; i++)
		{
			event = (tx_msg_t *)get_event(event_q);
			if(NULL == event)
				break;
			if( HOST_SC2331_CMD == event->hdr.type) 
			{
				if((event->slice[0].len > 0) &&(NULL != event->slice[0].data))
					kfree(event->slice[0].data);
			}
			else if(HOST_SC2331_PKT == event->hdr.type)
			{
				if((NULL != event->p))
					dev_kfree_skb((struct sk_buff  *)(event->p));
			}
			else
			{}
			memset((unsigned char *)event,  0,  sizeof(tx_msg_t) );
			free_event((void *)event, event_q);	
		}
		event_q_deinit(event_q);
	}
	tx_fifo_free(&(vif->txfifo));
	return OK;
}

static int wlan_rx_buf_alloc(void)
{
	int ret;
	rxfifo_t *rx_buf = &(g_wlan.rxfifo);
	ret = rx_fifo_alloc(rx_buf);
	return ret;
}

static int wlan_rx_buf_free(void )
{
	int ret;
	rxfifo_t *rx_buf = &(g_wlan.rxfifo);
	ret = rx_fifo_free(rx_buf);
	return ret;
}

static int wlan_hw_init(hw_info_t *hw)
{
	memset(hw, 0, sizeof(hw_info_t) );
	
	hw->sdio_tx_chn.num          = 3;
	hw->sdio_tx_chn.chn[0]       = 0;
	hw->sdio_tx_chn.chn[1]       = 1;
	hw->sdio_tx_chn.chn[2]       = 2;
	hw->sdio_tx_chn.bit_map      = 0x0007;
	hw->sdio_tx_chn.timeout_time = 600;
	hw->sdio_tx_chn.timeout_flag = false;

	hw->sdio_rx_chn.num          = 5;
	hw->sdio_rx_chn.chn[0]       = 8;
	hw->sdio_rx_chn.chn[1]       = 9;
	hw->sdio_rx_chn.chn[2]       = 14;
	hw->sdio_rx_chn.chn[3]       = 11;
	hw->sdio_rx_chn.chn[4]       = 15;
	hw->sdio_rx_chn.bit_map      = 0xcb00;
	hw->sdio_rx_chn.gpio_high    = false;	
	hw->sdio_rx_chn.timeout_time = 600;
	hw->sdio_rx_chn.timeout_flag = false;

	printke("[SDIO_TX_CHN][0x%x][0x%x]\n", hw->sdio_tx_chn.bit_map, HW_TX_SIZE);
	printke("[SDIO_RX_CHN][0x%x][0x%x]\n", hw->sdio_rx_chn.bit_map, HW_RX_SIZE);
	
	hw->wakeup = 0;
	spin_lock_init(&(hw->sdio_rx_chn.lock));
	wake_lock_init(&g_wlan.hw.wlan_lock, WAKE_LOCK_SUSPEND, "wlan_sc2331_lock");
	init_timer(&g_wlan.hw.wakeup_timer);
	g_wlan.hw.wakeup_timer.function = wakeup_timer_func;
	g_wlan.hw.wakeup_time  = 2000;
	return OK;
}
static int wlan_inetaddr_event(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	unsigned  char      vif_id;
	wlan_vif_t          *vif;
	struct net_device   *dev;
	printkd("inetaddr callback is comming in !\n");

	struct in_ifaddr *ifa = (struct in_ifaddr *)ptr;

	dev = ifa->ifa_dev ? ifa->ifa_dev->dev : NULL;

	if((dev != (id_to_vif(0)->ndev)) && (dev != (id_to_vif(1)->ndev)))
	{
		printkd("dev id not equal to 0 or 1!\n");
		goto done;
	}	

	if (dev == NULL)
		goto done;
	printkd(" inetaddr dev not equal to null !\n");

	vif = ndev_to_vif(dev);
	vif_id = vif->id;

	if (!vif)
		goto done;
	printkd("inetaddr vif not equal to null !\n");

	switch (event) {
	case NETDEV_UP:
	printkd("inetaddr UP event is comming in !\n");
		wlan_cmd_get_ip(vif_id, (u8 *) & ifa->ifa_address);
		break;
	case NETDEV_DOWN:
	printkd("inetaddr DOWN event is comming in  !\n");
		break;
	default:
	printkd("inetaddr defaut is comming in !\n");
		break;
	}

done:
	return NOTIFY_DONE;
}

static struct notifier_block itm_inetaddr_cb = {
	.notifier_call = wlan_inetaddr_event,
};

static ssize_t wlan_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	int cmd, value;
	char kbuf[32] = {0};
	if (copy_from_user(kbuf, buffer, ( (count <= 32)?(count):(32) ) )  )
		return -EFAULT;
	sscanf(kbuf, "%d %d\n", &cmd, &value);
	
	printkd("[%s][%d][%d]\n", __func__,cmd, value);
	switch (cmd)
	{
	case 1:
		SET_BIT(g_dbg, value);
		break;
	case 2:
		CLEAR_BIT(g_dbg, value);
		break;
	case 3:
		wlan_cmd_mac_open(NETIF_0_ID, value, &(g_wlan.netif[NETIF_0_ID].ndev->dev_addr[0] ) );
		break;
	case 4:
		wlan_cmd_mac_open(NETIF_1_ID, value, &(g_wlan.netif[NETIF_1_ID].ndev->dev_addr[0] ) );
		break;
	default:
		break;
	}
	return count;
}

static const struct file_operations wlan_proc_fops = 
{
	.owner		= THIS_MODULE,
	.write		= wlan_proc_write,
};

static const struct file_operations lte_concur_proc_fops = 
{
	.owner		= THIS_MODULE,
	.unlocked_ioctl  = lte_concur_proc_ioctl,
	.open       = lte_concur_proc_open,
	.release    = lte_concur_proc_release,
};

int wlan_module_init(struct device *dev)
{
	int ret;
	
	printke("[%s] [ version:0x22 ] [ time(%s %s) ]\n", __func__, __DATE__, __TIME__);
	if(NULL == dev)
		return -EPERM;
	ret = get_sdiohal_status();
	if(1 != ret)
	{
		printke("######## %s sdio is not ready  ##########\n", __func__);
		return -EPERM;
	}
	else
	{
		printke("sdio is ready !!!\n");
	}
	marlin_pa_enable(true);


	memset((unsigned char *)(&g_wlan), 0, sizeof(wlan_info_t));
	g_wlan.dev = dev;
	g_wlan.netif[NETIF_0_ID].id= NETIF_0_ID;
	g_wlan.netif[NETIF_1_ID].id= NETIF_1_ID;
	g_wlan.sync.exit = 0;
	atomic_set(&g_wlan.screen_on, 1);
	sema_init(&g_wlan.sync.sem, 0);
	wlan_hw_init(&(g_wlan.hw));
	
	ret = wlan_tx_buf_alloc(&(g_wlan.netif[NETIF_0_ID]));
	if(OK != ret)
		return -EPERM;
	ret = wlan_tx_buf_alloc(&(g_wlan.netif[NETIF_1_ID]));
	if(OK != ret)
		return -EPERM;
	ret = wlan_rx_buf_alloc();
	if(OK != ret)
		return -EPERM;
	
	wlan_cmd_init();
	
	g_wlan.wlan_core.task  = kthread_create(wlan_core_thread, (void *)(dev), "wlan_core");	
	if(NULL != g_wlan.wlan_core.task)
	{		
		wake_up_process(g_wlan.wlan_core.task);
	}
	down(&(g_wlan.sync.sem));
	
	g_wlan.wlan_trans.task = kthread_create(wlan_trans_thread, (void *)(dev), "wlan_trans");	
	if(NULL != g_wlan.wlan_trans.task)
	{		
		wake_up_process(g_wlan.wlan_trans.task);
	}
	down(&(g_wlan.sync.sem));
	
	ret = wlan_wiphy_new(&(g_wlan));
	if(OK != ret)
		return -EPERM;
	ret = wlan_vif_init(&(g_wlan.netif[NETIF_0_ID]), NL80211_IFTYPE_ADHOC,       "wlan0",    (void *)(&wlan_ops) );
	if(OK != ret)
		return -EPERM;
	ret = wlan_vif_init(&(g_wlan.netif[NETIF_1_ID]), NL80211_IFTYPE_P2P_DEVICE,  "p2p0",     (void *)(&wlan_ops) );
	if(OK != ret)
		return -EPERM;
	wlan_nl_init();
	
	g_wlan.hw.early_suspend.suspend  = wlan_early_suspend;
	g_wlan.hw.early_suspend.resume   = wlan_late_resume;
	g_wlan.hw.early_suspend.level    = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;	
	register_early_suspend(&g_wlan.hw.early_suspend);
	
	ret = register_inetaddr_notifier(&itm_inetaddr_cb);
	if (ret) {
		printke("Couldn't register inetaddr notifier \n");
	}
	if (!proc_create("wlan", 0666, NULL, &wlan_proc_fops)) 
	{
		printke("Couldn't create the /proc/wlan \n");
	}
	if (!proc_create("lte_concur", 0666, NULL, &lte_concur_proc_fops))
	{
		printke("Couldn't create the /proc/lte_concur \n");
	}	
	g_dbg = 0x0;
	SET_BIT(g_dbg,1);
	printke("%s ok!\n", __func__);
	return OK;
}
EXPORT_SYMBOL_GPL(wlan_module_init);

int wlan_module_exit(struct device *dev)
{
	printke("%s enter\n", __func__);
	unregister_inetaddr_notifier(&itm_inetaddr_cb);
	marlin_pa_enable(false);
	if(ITM_NONE_MODE != g_wlan.netif[NETIF_0_ID].mode)
		wlan_cmd_mac_close(NETIF_0_ID, g_wlan.netif[NETIF_0_ID].mode);
	if(ITM_NONE_MODE != g_wlan.netif[NETIF_1_ID].mode)
		wlan_cmd_mac_close(NETIF_1_ID, g_wlan.netif[NETIF_1_ID].mode);
	g_wlan.sync.exit = 1;
	core_up();
	down(&(g_wlan.sync.sem));
	trans_up();
	down(&(g_wlan.sync.sem));
	wlan_vif_free(&(g_wlan.netif[NETIF_0_ID]));
	wlan_vif_free(&(g_wlan.netif[NETIF_1_ID]));
	wlan_wiphy_free(&g_wlan);
	wlan_cmd_deinit();
	wlan_tx_buf_free(&(g_wlan.netif[NETIF_0_ID]));
	wlan_tx_buf_free(&(g_wlan.netif[NETIF_1_ID]));
	wlan_rx_buf_free();
	wlan_nl_deinit();

	remove_proc_entry("wlan", NULL);
	remove_proc_entry("lte_concur", NULL);
	unregister_early_suspend(&g_wlan.hw.early_suspend);
	wake_lock_destroy(&g_wlan.hw.wlan_lock);
	printke("%s ok!\n", __func__);
	return OK;
}
EXPORT_SYMBOL_GPL(wlan_module_exit);

static int  sprd_wlan_probe(struct platform_device *pdev)
{
	return wlan_module_init(&(pdev->dev));
}

static int  sprd_wlan_remove(struct platform_device *pdev)
{
	return wlan_module_exit(&(pdev->dev));
}

#define DEVICE_NAME "sc2331"
static struct platform_device *sprd_wlan_device;
static struct platform_driver  sprd_wlan_driver =
{
	.probe =   sprd_wlan_probe,
	.remove =  sprd_wlan_remove,
	.driver = 
	{
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};
static int  sprd_wlan_init(void)
{
	sprd_wlan_device = platform_device_register_simple(DEVICE_NAME, 0, NULL, 0);
	if (IS_ERR(sprd_wlan_device))
		return PTR_ERR(sprd_wlan_device);
	return platform_driver_register(&(sprd_wlan_driver));
}

static void  sprd_wlan_exit(void)
{
	platform_driver_unregister(&sprd_wlan_driver);
	platform_device_unregister(sprd_wlan_device);
	sprd_wlan_device = NULL;
}

module_init(sprd_wlan_init);
module_exit(sprd_wlan_exit);
MODULE_DESCRIPTION("SPRD sc2331 Wireless Network Adapter");
MODULE_AUTHOR("jinglong.chen");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");


