/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <asm/byteorder.h>
#include <linux/tty.h>
#include <linux/platform_device.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif

#include <linux/sipc.h>
#include <linux/seth.h>

/* debugging macros */
#define SETH_INFO(x...)		pr_info("SETH: " x)
#define SETH_DEBUG(x...)	pr_debug("SETH: " x)
#define SETH_ERR(x...)		pr_err("SETH: Error: " x)

#define SETH_BLOCK_SIZE	(ETH_HLEN + ETH_DATA_LEN + NET_IP_ALIGN)

#define DEV_ON 1
#define DEV_OFF 0

#define SETH_RESEND_MAX_NUM	10

#define SETH_NAPI
#define SETH_SHARKL

#ifdef SETH_NAPI
#define SETH_NAPI_WEIGHT 128
#define SETH_NAPI_FIFO_DEPTH 256
#define SETH_TX_WEIGHT 250
#define SETH_TX_FIFO_DEPTH 256

struct seth_rx_fifo {
	uint32_t rd_ptr; /* read pointer */
	uint32_t wr_ptr; /* write pointer */
	struct sk_buff *cache[SETH_NAPI_FIFO_DEPTH]; /* rx packet cache */
};

struct seth_tx_fifo {
	uint32_t rd_ptr; /* read pointer */
	uint32_t wr_ptr; /* write pointer */
	struct sk_buff *cache[SETH_TX_FIFO_DEPTH]; /* tx packet cache */
};
#endif

/*
 * Device instance data.
 */
typedef struct SEth {
	struct net_device_stats stats;	/* net statistics */
	struct net_device* netdev;	/* Linux net device */
	struct seth_init_data* pdata;	/* platform data */
	int state;			/* device state */
	int txstate;			/* device txstate */
	int stopped;		/* sblock indicator */
	int txrcnt;			/* seth tx resend count*/
#ifdef SETH_NAPI
	struct seth_rx_fifo *rx_fifo;
	struct seth_rx_fifo p_rx_fifos;
	struct napi_struct napi; /* napi instance */
	struct seth_tx_fifo *tx_fifo;
	struct seth_tx_fifo p_tx_fifos;
	struct timer_list seth_tx_timer;
	int txing;
#endif
} SEth;

#ifdef SETH_NAPI
static int seth_rx_poll_handler(struct napi_struct * napi, int budget)
{
	struct SEth *seth = container_of(napi, struct SEth, napi);
	volatile struct seth_rx_fifo *rx_fifo;
	struct sk_buff * skb;
	int skb_cnt = 0;
	int index;

	if (!seth) {
		SETH_ERR("seth_rx_poll_handler no seth device\n");
		return 0;
	}

	rx_fifo = (volatile struct seth_rx_fifo *)seth->rx_fifo;

	/* if the cache isn't empty, keep polling */
	while ((budget - skb_cnt) && (rx_fifo->rd_ptr != rx_fifo->wr_ptr)) {
		index = rx_fifo->rd_ptr & (SETH_NAPI_FIFO_DEPTH - 1);
		skb = rx_fifo->cache[index];
		if (!skb) {
			rx_fifo->rd_ptr += 1;
			SETH_ERR("seth_rx_poll_handler this skb is NULL, index %d\n", index);
			continue;
		}

		skb->protocol = eth_type_trans(skb, seth->netdev);
		skb->ip_summed = CHECKSUM_NONE;
		netif_receive_skb(skb);

		/* update fifo rd_ptr */
		rx_fifo->rd_ptr += 1;

		seth->stats.rx_bytes += skb->len;
		seth->stats.rx_packets++;

		/* update skb counter*/
		skb_cnt++;
	}

	SETH_DEBUG("napi polling done, budget %d, skb cnt %d, cpu %d\n",
				budget, skb_cnt, smp_processor_id());
	SETH_DEBUG("napi rx_fifo:rd_ptr 0x%x, wr_ptr 0x%x\n", rx_fifo->rd_ptr, rx_fifo->wr_ptr);

	if (skb_cnt >= 0 && budget > skb_cnt) {
		napi_complete(napi);
	}

	return skb_cnt;
}
#endif

/*
 * Tx_ready handler.
 */
static void
seth_tx_ready_handler (void* data)
{
	SEth* seth = (SEth*) data;

	printk(KERN_INFO "seth_tx_ready_handler state %0x\n", seth->state);
	if (seth->state != DEV_ON) {
		seth->state = DEV_ON;
		seth->txstate = DEV_ON;
		if (!netif_carrier_ok (seth->netdev)) {
			netif_carrier_on (seth->netdev);
		}
	}
	else {
		seth->state = DEV_OFF;
		seth->txstate = DEV_OFF;
		if (netif_carrier_ok (seth->netdev)) {
		netif_carrier_off (seth->netdev);
		/*
		netif_carrier_on (seth->netdev);
		*/
		}
	}
}

/*
 * Tx_open handler.
 */
static void
seth_tx_open_handler (void* data)
{
	SEth* seth = (SEth*) data;

	printk(KERN_INFO "seth_tx_open_handler state %0x\n", seth->state);
	if (seth->state != DEV_ON) {
		seth->state = DEV_ON;
		seth->txstate = DEV_ON;
		if (!netif_carrier_ok (seth->netdev)) {
			netif_carrier_on (seth->netdev);
		}
	}
}

/*
 * Tx_close handler.
 */
static void
seth_tx_close_handler (void* data)
{
	SEth* seth = (SEth*) data;

	printk(KERN_INFO "seth_tx_close_handler state %0x\n", seth->state);
	if (seth->state != DEV_OFF) {
		seth->state = DEV_OFF;
		seth->txstate = DEV_OFF;
		if (netif_carrier_ok (seth->netdev)) {
			netif_carrier_off (seth->netdev);
		}
	}
}

static void
seth_rx_handler (void* data)
{
	SEth* seth = (SEth *)data;
	struct seth_init_data *pdata = seth->pdata;
	struct sblock blk;
	struct sk_buff* skb;
	int ret,sblkret,count;

#ifdef SETH_NAPI
	volatile struct seth_rx_fifo *rx_fifo = NULL;
	int index;
	rx_fifo = (volatile struct seth_rx_fifo *)seth->rx_fifo;
#endif

	sblkret=0;
	count=0;
	if (seth->state != DEV_ON) {
		SETH_ERR ("rx_handler the state of %s is off!\n", seth->netdev->name);
		seth->stats.rx_errors++;
		ret = sblock_receive(pdata->dst, pdata->channel, &blk, -1);
		if (ret) {
			SETH_ERR ("receive sblock failed (%d)\n", ret);
			seth->stats.rx_errors++;
			return;
		}
		goto rx_failed;
	}
	while(!sblkret){
		sblkret=sblock_receive(pdata->dst, pdata->channel, &blk, 0);
		if (sblkret) {
#ifdef SETH_NAPI
			if(count>0){
				napi_schedule(&seth->napi);
				/* trigger a NET_RX_SOFTIRQ softirq directly */
				raise_softirq(NET_RX_SOFTIRQ);
				SETH_DEBUG ("SETH_RX:rx_fifo->wr_ptr = 0x%x, rx_fifo->rd_ptr = 0x%x ,napi_schedule\n",
				rx_fifo->wr_ptr,rx_fifo->rd_ptr);
			}
#endif
			SETH_DEBUG("receiving sblock is empty (%d)\n", sblkret);
			seth->stats.rx_errors++;
			goto rx_failed;
		}

#ifdef SETH_SHARKL
		skb = dev_alloc_skb (blk.length); //16 bytes align
		if (!skb) {
			SETH_ERR ("alloc skbuff failed!\n");
			seth->stats.rx_dropped++;
			ret = sblock_release(pdata->dst, pdata->channel, &blk);
			if (ret) {
				SETH_ERR ("release sblock failed (%d)\n", ret);
			}
			continue;
		}
		if((((uint32_t)(skb->data))&0x03) != 0){
			SETH_DEBUG ("SKBADDR:skb->data= 0x%x ,addr value is not 4x\n",
					skb->data);
		}
		memcpy(skb->data, blk.addr, blk.length);
		skb_reserve(skb, NET_IP_ALIGN);
		skb_put (skb, blk.length-2);
#else
		skb = dev_alloc_skb (blk.length + NET_IP_ALIGN); //16 bytes align
		if (!skb) {
			SETH_ERR ("alloc skbuff failed!\n");
			seth->stats.rx_dropped++;
			ret = sblock_release(pdata->dst, pdata->channel, &blk);
			if (ret) {
				SETH_ERR ("release sblock failed (%d)\n", ret);
			}
			continue;
		}
		skb_reserve(skb, NET_IP_ALIGN);
		memcpy(skb->data, blk.addr, blk.length);
		skb_put (skb, blk.length);
#endif

#ifdef SETH_NAPI
		/*if the fifo is full, drop the skb*/
		if ((int)(rx_fifo->wr_ptr - rx_fifo->rd_ptr) >= SETH_NAPI_FIFO_DEPTH) {
			SETH_ERR("napi rx_fifo is full, drop the skb.\n");
			SETH_ERR("napi rx_fifo:rd_ptr 0x%x, wr_ptr 0x%x\n",
				rx_fifo->rd_ptr, rx_fifo->wr_ptr);
			seth->stats.rx_dropped++;
			kfree(skb);
			goto rx_failed;
		}
		index = rx_fifo->wr_ptr & (SETH_NAPI_FIFO_DEPTH - 1);
		rx_fifo->cache[index] = skb;
		rx_fifo->wr_ptr += 1;
		count++;
		if(((uint32_t)(rx_fifo->wr_ptr - rx_fifo->rd_ptr) >= SETH_NAPI_WEIGHT)
			&&((count&0x3f)==0x00)) {		//>=128, trigger per 64
			napi_schedule(&seth->napi);
			/* trigger a NET_RX_SOFTIRQ softirq directly */
			raise_softirq(NET_RX_SOFTIRQ);
			SETH_DEBUG ("SETH_RX:rx_fifo->wr_ptr = 0x%x, rx_fifo->rd_ptr = 0x%x ,napi_schedule SETH_NAPI_WEIGHT\n",
					rx_fifo->wr_ptr,rx_fifo->rd_ptr);
		}
		ret = sblock_release(pdata->dst, pdata->channel, &blk);
		if (ret) {
			SETH_ERR ("release sblock failed (%d)\n", ret);
		}
#else
		skb->dev = seth->netdev;
		skb->protocol  = eth_type_trans (skb, seth->netdev);
		skb->ip_summed = CHECKSUM_NONE;

		seth->stats.rx_packets++;
		seth->stats.rx_bytes += skb->len;

		netif_rx (skb);

		seth->netdev->last_rx = jiffies;
#endif
	}

rx_failed:
	if (!sblkret) {
		ret = sblock_release(pdata->dst, pdata->channel, &blk);
		if (ret) {
			SETH_ERR ("release sblock failed (%d)\n", ret);
		}
	}
	return;
}

/*
 * Tx_close handler.
 */
static void
seth_tx_pre_handler (void* data)
{
	SEth* seth = (SEth*) data;

	if (seth->txstate != DEV_ON) {
		seth->txstate = DEV_ON;
		SETH_INFO ("seth_tx_ready_handler txstate %0x\n", seth->txstate );
		seth->netdev->trans_start = jiffies;
		netif_wake_queue(seth->netdev);
	}
}


static void
seth_handler (int event, void* data)
{
	SEth *seth = (SEth *)data;

	switch(event) {
		case SBLOCK_NOTIFY_GET:
			SETH_DEBUG ("SBLOCK_NOTIFY_GET is received\n");
			seth_tx_pre_handler(seth);
			break;
		case SBLOCK_NOTIFY_RECV:
			SETH_DEBUG ("SBLOCK_NOTIFY_RECV is received\n");
			seth_rx_handler(seth);
			break;
		case SBLOCK_NOTIFY_STATUS:
			SETH_DEBUG ("SBLOCK_NOTIFY_STATUS is received\n");
			seth_tx_ready_handler(seth);
			break;
		case SBLOCK_NOTIFY_OPEN:
			SETH_DEBUG ("SBLOCK_NOTIFY_OPEN is received\n");
			seth_tx_open_handler(seth);
			break;
		case SBLOCK_NOTIFY_CLOSE:
			SETH_DEBUG ("SBLOCK_NOTIFY_CLOSE is received\n");
			seth_tx_close_handler(seth);
			break;
		default:
			SETH_ERR ("Received event is invalid(event=%d)\n", event);
	}
}

#ifdef SETH_NAPI
static int
seth_tx_handler(void* data)
{
	struct sblock blk;
	SEth* seth    = netdev_priv (data);
	struct seth_init_data *pdata = seth->pdata;
	volatile struct seth_tx_fifo *tx_fifo;
	struct sk_buff* skb;
	int index,ret;

	if(seth->txing==1){
		return NETDEV_TX_OK;
	}else{
		seth->txing=1;
	}
	tx_fifo=seth->tx_fifo;
	while(tx_fifo->wr_ptr != tx_fifo->rd_ptr){
		index = tx_fifo->rd_ptr & (SETH_TX_FIFO_DEPTH - 1);
		skb = tx_fifo->cache[index];
		/*
		* Get a free sblock.
		*/
		ret = sblock_get(pdata->dst, pdata->channel, &blk, 0);
		if(ret) {
			SETH_INFO("Get free sblock failed(%d), drop data!\n", ret);
			seth->stats.tx_fifo_errors++;
			netif_stop_queue (data);
			seth->txstate = DEV_OFF;
			seth->txing=0;
			return NETDEV_TX_BUSY;
		}

		if(blk.length < skb->len) {
			SETH_ERR("The size of sblock is so tiny!\n");
			sblock_put(pdata->dst, pdata->channel, &blk);
			seth->stats.tx_fifo_errors++;
			seth->txrcnt = 0;
			seth->txing=0;
			return NETDEV_TX_OK;
		}

		blk.length = skb->len;
		memcpy (blk.addr, skb->data, skb->len);
		ret = sblock_send_prepare(pdata->dst, pdata->channel, &blk);
		if(ret) {
			SETH_INFO("send sblock failed(%d)!\n", ret);
			sblock_put(pdata->dst, pdata->channel, &blk);
			seth->stats.tx_fifo_errors++;
			if (seth->txrcnt > SETH_RESEND_MAX_NUM) {
				netif_stop_queue (data);
				seth->txstate = DEV_OFF;
			}
			seth->txrcnt ++;
			seth->txing=0;
			return NETDEV_TX_BUSY;
		}
		/*
		* Statistics.
		*/
		seth->stats.tx_bytes += skb->len;
		seth->stats.tx_packets++;
		seth->txrcnt = 0;

		dev_kfree_skb_any(skb);
		tx_fifo->rd_ptr += 1;
		//SETH_DEBUG ("SETH_TX:tx_fifo->wr_ptr = 0x%x, tx_fifo->rd_ptr = 0x%x\n",
				//tx_fifo->wr_ptr,tx_fifo->rd_ptr);
	}
	ret = sblock_send_finish(pdata->dst, pdata->channel);
	if(ret) {
		SETH_INFO("seth tx failed(%d)!\n", ret);
	}
	seth->txing=0;
	return NETDEV_TX_OK;
}

static int
is_skb_tcpdata(struct sk_buff* skb){
	SETH_DEBUG ("SKBdata: len = 0x%x,ip version = 0x%x\n",
		skb->len,((uint8_t)(*(skb->data+12))<<8)+(uint8_t)(*(skb->data+13)));
	if((uint8_t)(*(skb->data+12))==0x08 && (uint8_t)(*(skb->data+13))==0x00){	//ipv4
		SETH_DEBUG ("TCPdata: L4 type = 0x%x ,tcp flag = 0x%x\n",
						(uint8_t)(*(skb->data+23)),(uint8_t)(*(skb->data+47)));
		if(((uint8_t)(*(skb->data+23)))==0x06){		//tcp
			if((((uint8_t)(*(skb->data+47)))&0x12)==0x00
				|| ((((uint8_t)(*(skb->data+47)))&0x12)==0x10 && skb->len>200)){	//tcp flag
				return 1;
			}
		}
	}else if((uint8_t)(*(skb->data+12))==0x86 && (uint8_t)(*(skb->data+13))==0xdd){	//ipv6
		SETH_DEBUG ("TCPdata: L4 type = 0x%x ,tcp flag = 0x%x\n",
						(uint8_t)(*(skb->data+20)),(uint8_t)(*(skb->data+67)));
		if(((uint8_t)(*(skb->data+20)))==0x06){		//tcp
			if((((uint8_t)(*(skb->data+67)))&0x12)==0x00
				|| ((((uint8_t)(*(skb->data+67)))&0x12)==0x10 && skb->len>200)){	//tcp flag
				return 1;
			}
		}
	}
	return 0;
}
#endif

/*
 * Transmit interface
 */
static int
seth_start_xmit (struct sk_buff* skb, struct net_device* dev)
{
	SEth* seth = netdev_priv (dev);

#ifdef SETH_NAPI
	volatile struct seth_tx_fifo *tx_fifo = NULL;
	int index,retskb;
	tx_fifo = (volatile struct seth_tx_fifo *)seth->tx_fifo;
#endif

	if (seth->state != DEV_ON) {
		SETH_ERR ("xmit the state of %s is off\n", dev->name);
		netif_carrier_off (dev);
		seth->stats.tx_carrier_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

#ifdef SETH_NAPI
	/*if the fifo is full, drop the skb*/
	if ((int)(tx_fifo->wr_ptr - tx_fifo->rd_ptr) >= SETH_TX_FIFO_DEPTH) {
		SETH_ERR("napi tx_fifo is full, drop the skb.\n");
		SETH_ERR("napi tx_fifo:rd_ptr 0x%x, wr_ptr 0x%x\n",
			tx_fifo->rd_ptr, tx_fifo->wr_ptr);
		seth->stats.tx_dropped++;
		seth->stats.tx_fifo_errors++;
		netif_stop_queue (dev);
		seth->txstate = DEV_OFF;
		return NETDEV_TX_BUSY;
	}
	index = tx_fifo->wr_ptr & (SETH_TX_FIFO_DEPTH - 1);
	tx_fifo->cache[index] = skb;
	tx_fifo->wr_ptr += 1;
	retskb=is_skb_tcpdata(skb);
	if(((int)(tx_fifo->wr_ptr - tx_fifo->rd_ptr) >= SETH_TX_WEIGHT) || (retskb == 0)) {
		del_timer(&seth->seth_tx_timer);
		seth_tx_handler(dev);
		SETH_DEBUG ("seth_start_xmit:AT ONCE\n");
	}else{
		seth->seth_tx_timer.function = seth_tx_handler;
		seth->seth_tx_timer.expires = jiffies + 1;
		seth->seth_tx_timer.data = (unsigned long)dev;
		del_timer(&seth->seth_tx_timer);
		add_timer(&seth->seth_tx_timer);
		SETH_DEBUG ("seth_start_xmit:Timer\n");
	}
#endif
	dev->trans_start = jiffies;
	return NETDEV_TX_OK;
}

/*
 * Open interface
 */
static int seth_open (struct net_device *dev)
{
	SEth* seth = netdev_priv(dev);

	/* Reset stats */
	memset(&seth->stats, 0, sizeof(seth->stats));

	if (seth->state == DEV_ON && !netif_carrier_ok(seth->netdev)) {
		SETH_INFO("seth_open netif_carrier_on\n");
		netif_carrier_on(seth->netdev);
	}

#ifdef SETH_NAPI
	napi_enable(&seth->napi);
#endif

	seth->txstate = DEV_ON;

	netif_start_queue(dev);

	return 0;
}

/*
 * Close interface
 */
static int seth_close (struct net_device *dev)
{
	SEth* seth = netdev_priv(dev);

#ifdef SETH_NAPI
	napi_disable(&seth->napi);
#endif

	netif_stop_queue(dev);

	/*
	seth->state = DEV_OFF;
	*/

	seth->txstate = DEV_OFF;

	return 0;
}

static struct net_device_stats * seth_get_stats(struct net_device *dev)
{
	SEth * seth = netdev_priv(dev);
	return &(seth->stats);
}

static void seth_tx_timeout(struct net_device *dev)
{
	SEth * seth = netdev_priv(dev);

	SETH_INFO ("seth_tx_timeout()\n");
	if (seth->txstate != DEV_ON) {
		seth->txstate = DEV_ON;
		dev->trans_start = jiffies;
		netif_wake_queue(dev);
	}
}

static struct net_device_ops seth_ops = {
	.ndo_open = seth_open,
	.ndo_stop = seth_close,
	.ndo_start_xmit = seth_start_xmit,
	.ndo_get_stats = seth_get_stats,
	.ndo_tx_timeout = seth_tx_timeout,
};

static int seth_parse_dt(struct seth_init_data **init, struct device *dev)
{
#ifdef CONFIG_OF
	struct seth_init_data *pdata = NULL;
	struct device_node *np = dev->of_node;
	int ret;
	uint32_t data;

	pdata = kzalloc(sizeof(struct seth_init_data), GFP_KERNEL);
	if (!pdata) {
		return -ENOMEM;
	}

	ret = of_property_read_string(np, "sprd,name", (const char**)&pdata->name);
	if (ret) {
		goto error;
	}

	ret = of_property_read_u32(np, "sprd,dst", (uint32_t *)&data);
	if (ret) {
		goto error;
	}
	pdata->dst = (uint8_t)data;

	ret = of_property_read_u32(np, "sprd,channel", (uint32_t *)&data);
	if (ret) {
		goto error;
	}
	pdata->channel = (uint8_t)data;

	ret = of_property_read_u32(np, "sprd,blknum", (uint32_t *)&pdata->blocknum);
	if (ret) {
		goto error;
	}

	*init = pdata;
	return 0;
error:
	kfree(pdata);
	*init = NULL;
	return ret;
#else
	return -ENODEV;
#endif
}

static inline void seth_destroy_pdata(struct seth_init_data **init)
{
#ifdef CONFIG_OF
	struct seth_init_data *pdata = *init;

	if (pdata) {
		kfree(pdata);
	}
	*init = NULL;
#else
	return;
#endif
}

static int  seth_probe(struct platform_device *pdev)
{
	struct seth_init_data *pdata = pdev->dev.platform_data;
	struct net_device* netdev;
	SEth* seth;
	char ifname[IFNAMSIZ];
	int ret;

	if (pdev->dev.of_node && !pdata) {
		ret = seth_parse_dt(&pdata, &pdev->dev);
		if (ret) {
			printk(KERN_ERR "failed to parse seth device tree, ret=%d\n", ret);
			return ret;
		}
	}
	SETH_INFO("after parse device tree, name=%s, dst=%u, channel=%u, blocknum=%u\n",
		pdata->name, pdata->dst, pdata->channel, pdata->blocknum);

	if(pdata->name[0])
		strlcpy(ifname, pdata->name, IFNAMSIZ);
	else
		strcpy(ifname, "veth%d");

	netdev = alloc_netdev (sizeof (SEth), ifname, ether_setup);
	if (!netdev) {
		seth_destroy_pdata(&pdata);
		SETH_ERR ("alloc_netdev() failed.\n");
		return -ENOMEM;
	}
	seth = netdev_priv (netdev);
	seth->pdata = pdata;
	seth->netdev = netdev;
	seth->state = DEV_OFF;
	seth->stopped = 0;

#ifdef SETH_NAPI
	seth->rx_fifo = &(seth->p_rx_fifos);
	seth->tx_fifo = &(seth->p_tx_fifos);
	init_timer(&seth->seth_tx_timer);
#endif

	netdev->netdev_ops = &seth_ops;
	netdev->watchdog_timeo = 1*HZ;
	netdev->irq = 0;
	netdev->dma = 0;

	random_ether_addr(netdev->dev_addr);

#ifdef SETH_NAPI
	netif_napi_add(netdev, &seth->napi, seth_rx_poll_handler, SETH_NAPI_WEIGHT);
#endif

	ret = sblock_create(pdata->dst, pdata->channel,
		pdata->blocknum, SETH_BLOCK_SIZE,
		pdata->blocknum, SETH_BLOCK_SIZE);
	if (ret) {
		SETH_ERR ("create sblock failed (%d)\n", ret);
		free_netdev(netdev);
		seth_destroy_pdata(&pdata);
		return ret;
	}

	ret = sblock_register_notifier(pdata->dst, pdata->channel, seth_handler, seth);
	if (ret) {
		SETH_ERR ("regitster notifier failed (%d)\n", ret);
		free_netdev(netdev);
		sblock_destroy(pdata->dst, pdata->channel);
		return ret;
	}

	/* register new Ethernet interface */
	if ((ret = register_netdev (netdev))) {
		SETH_ERR ("register_netdev() failed (%d)\n", ret);
#ifdef SETH_NAPI
		netif_napi_del(&seth->napi);
#endif
		free_netdev(netdev);
		sblock_destroy(pdata->dst, pdata->channel);
		seth_destroy_pdata(&pdata);
		return ret;
	}

	/* set link as disconnected */
	netif_carrier_off (netdev);

	platform_set_drvdata(pdev, seth);
	return 0;
}

/*
 * Cleanup Ethernet device driver.
 */
static int  seth_remove (struct platform_device *pdev)
{
	struct SEth* seth = platform_get_drvdata(pdev);
	struct seth_init_data *pdata = seth->pdata;

#ifdef SETH_NAPI
	netif_napi_del(&seth->napi);
	del_timer(&seth->seth_tx_timer);
#endif

	sblock_destroy(pdata->dst, pdata->channel);
	seth_destroy_pdata(&pdata);
	unregister_netdev(seth->netdev);
	free_netdev(seth->netdev);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id seth_match_table[] = {
	{ .compatible = "sprd,seth", },
	{ },
};

static struct platform_driver seth_driver = {
	.probe = seth_probe,
	.remove = seth_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "seth",
		.of_match_table = seth_match_table,
	},
};

static int __init seth_init(void)
{
	return platform_driver_register(&seth_driver);
}

static void __exit seth_exit(void)
{
	platform_driver_unregister(&seth_driver);
}

module_init (seth_init);
module_exit (seth_exit);

MODULE_DESCRIPTION ("Spreadtrum Ethernet device driver");
MODULE_LICENSE ("GPL");


