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

#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>

#define GET_WIFI_MAC_ADDR_FROM_NV_ITEM	        1

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16
#define IFHWADDRLEN        	6

#define CUSTOMER_MAC_FILE "/data/wifimac.txt"

extern void * dhd_os_open_image(char *filename);
extern int  dhd_os_get_image_block(char *buf, int len, void *image);
extern void dhd_os_close_image(void *image);


static int bcm_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static int bcm_wifi_power_state;
static int bcm_wifi_reset_state;
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static struct regulator *wlan_regulator_18 = NULL;

int bcm_wifi_power(int on);
int bcm_wifi_reset(int on);
int bcm_wifi_set_carddetect(int on);

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

int __init bcm_init_wifi_mem(void)
{
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
				GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}

/* Control the BT_VDDIO and WLAN_VDDIO
Always power on  According to spec
*/
static int brcm_ldo_enable(void)
{
	int err;
	wlan_regulator_18 = regulator_get(NULL, REGU_NAME_WIFI);

	if (IS_ERR(wlan_regulator_18)) {
		pr_err("can't get wlan 1.8V regulator\n");
		return -1;
	}

	err = regulator_set_voltage(wlan_regulator_18,1800000,1800000);
	if (err){
		pr_err("can't set wlan to 1.8V.\n");
		return -1;
	}
	regulator_enable(wlan_regulator_18);
}

int bcm_wifi_power(int on)
{
	pr_info("%s:%d \n", __func__, on);

	if(on) {

		gpio_direction_output(GPIO_WIFI_SHUTDOWN, 0);
		mdelay(10);
		gpio_direction_output(GPIO_WIFI_SHUTDOWN, 1);
		mdelay(200);
	}
	else {
		gpio_direction_output(GPIO_WIFI_SHUTDOWN, 0);

	}
	bcm_wifi_power_state = on;
	return 0;
}

int bcm_wifi_reset(int on)
{
	pr_info("%s: do nothing\n", __func__);
	bcm_wifi_reset_state = on;
	return 0;
}


static int bcm_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int bcm_wifi_status(struct device *dev)
{
	return bcm_wifi_cd;
}

int bcm_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	bcm_wifi_cd = val;
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}



#ifdef GET_WIFI_MAC_ADDR_FROM_NV_ITEM
static unsigned char bcm_mac_addr[IFHWADDRLEN] = { 0x11,0x22,0x33,0x44,0x55,0x66 };

static unsigned char char2bin( char m)
{
	if (( m >= 'a') && (m <= 'f')) {
		return m - 'a' + 10;
	}
	if (( m >= 'A') && (m <= 'F')) {
		return m - 'A' + 10;
	}
	if (( m >= '0') && (m <= '9')) {
		return m - '0';
	}
}

static int bcm_wifi_get_mac_addr(unsigned char *buf)
{
	int rc = 0;

	char macaddr[20];
	int mac_len = 0;
	void *fp;

	if (!buf){
		pr_err("%s, null parameter !!\n", __func__);
		return -EFAULT;
	}
	fp = dhd_os_open_image(CUSTOMER_MAC_FILE);
	if (fp == NULL)
		return -EFAULT;
	mac_len = dhd_os_get_image_block(macaddr, 17, fp);
	if (mac_len < 17)
		return -EFAULT;


	bcm_mac_addr[0] = (unsigned char)((char2bin(macaddr[0]) << 4) | char2bin(macaddr[1]));
	bcm_mac_addr[1] = (unsigned char)((char2bin(macaddr[3]) << 4) | char2bin(macaddr[4]));
	bcm_mac_addr[2] = (unsigned char)((char2bin(macaddr[6]) << 4) | char2bin(macaddr[7]));
	bcm_mac_addr[3] = (unsigned char)((char2bin(macaddr[9]) << 4) | char2bin(macaddr[10]));
	bcm_mac_addr[4] = (unsigned char)((char2bin(macaddr[12]) << 4) | char2bin(macaddr[13]));
	bcm_mac_addr[5] = (unsigned char)((char2bin(macaddr[15]) << 4) | char2bin(macaddr[16]));

	memcpy(buf, bcm_mac_addr, IFHWADDRLEN);
	pr_info("wifi mac: %x:%x:%x:%x:%x:%x\n", bcm_mac_addr[0], bcm_mac_addr[1], bcm_mac_addr[2], bcm_mac_addr[3], bcm_mac_addr[4], bcm_mac_addr[5]);

	dhd_os_close_image(fp);

	return 0;

}
#endif

static struct wifi_platform_data bcm_wifi_control = {
	.set_power      = bcm_wifi_power,
	.set_reset      = bcm_wifi_reset,
	.set_carddetect = bcm_wifi_set_carddetect,
	.mem_prealloc	= bcm_wifi_mem_prealloc,
#ifdef GET_WIFI_MAC_ADDR_FROM_NV_ITEM
	.get_mac_addr	= bcm_wifi_get_mac_addr,
#endif
};
#ifdef CONFIG_WLAN_SDIO
static struct resource sdio_resources[] = {
	[0] = {
		.start  = SPRD_SDIO1_PHYS,
		.end    = SPRD_SDIO1_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.name = "bcmdhd_wlan_irq",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};
static struct platform_device sprd_sdio_controller_device = {
	.name   = "bcmdhd_wlan",
	.id     = 1,
	.dev    = {
		.platform_data = &bcm_wifi_control,
	},
	.resource	= sdio_resources,
	.num_resources	= ARRAY_SIZE(sdio_resources),
};

#else
static struct resource spi_resources[] = {
	[0] = {
		.start  = SPRD_SPI0_PHYS,
		.end    = SPRD_SPI0_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.name = "bcmdhd_wlan_irq",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};
static struct platform_device sprd_spi_controller_device = {
	.name   = "bcmdhd_wlan",
	.id     = 1,
	.dev    = {
		.platform_data = &bcm_wifi_control,
	},
	.resource	= spi_resources,
	.num_resources	= ARRAY_SIZE(spi_resources),
};
#endif
static int __init bcm_wifi_init(void)
{
	int ret;

	bcm_init_wifi_mem();
	brcm_ldo_enable();
	gpio_request(GPIO_WIFI_IRQ, "oob_irq");
	gpio_direction_input(GPIO_WIFI_IRQ);
#ifdef CONFIG_WLAN_SDIO
	sdio_resources[1].start = gpio_to_irq(GPIO_WIFI_IRQ);
	sdio_resources[1].end = gpio_to_irq(GPIO_WIFI_IRQ);
#else
	spi_resources[1].start = gpio_to_irq(GPIO_WIFI_IRQ);
	spi_resources[1].end = gpio_to_irq(GPIO_WIFI_IRQ);
#endif
	gpio_request(GPIO_WIFI_SHUTDOWN,"wifi_pwd");
#ifdef CONFIG_WLAN_SDIO
	ret = platform_device_register(&sprd_sdio_controller_device);
#else
	ret = platform_device_register(&sprd_spi_controller_device);
#endif

	return ret;
}

late_initcall(bcm_wifi_init);

MODULE_DESCRIPTION("Broadcomm wlan driver");
MODULE_LICENSE("GPL");

