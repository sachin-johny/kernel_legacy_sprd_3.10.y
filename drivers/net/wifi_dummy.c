/*
 * =====================================================================================
 *
 *       Filename:  wifi_dummy.c
 *
 *    Description:  wifi dummy
 *
 *        Version:  1.0
 *        Created:  02/19/2012 04:39:30 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Binary Yang <Binary.Yang@spreadtrum.com.cn>
 *        Company:  © Copyright 2010 Spreadtrum Communications Inc.
 *
 * =====================================================================================
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/gfp.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>
#include <linux/workqueue.h>
#include <mach/ldo.h>

/* Manufacturer id */
#define SDIO_MANF_ID_CSR              0x032a

/* Device id */
#define SDIO_CARD_ID_UNIFI_1          0x0001
#define SDIO_CARD_ID_UNIFI_2          0x0002
#define SDIO_CARD_ID_UNIFI_3          0x0007
#define SDIO_CARD_ID_UNIFI_4          0x0008

#define WIFI_DUMMY_DEBUG
#define DRIVER_NAME "WIFI_DUMMY"

#ifdef  WIFI_DUMMY_DEBUG
#define DBG(f, x...) \
	printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

extern int sprd_3rdparty_gpio_wifi_reset;
extern int sprd_3rdparty_gpio_wifi_pwd;

struct mmc_host *mmc;
static void wifi_dummy_notify_sdio(void *ptr);
/* 
 * SDIO ids *must* be statically declared, so we can't take
 * them from the list passed in csr_sdio_register_driver().
 */
static const struct sdio_device_id unifi_ids[] = {
    { SDIO_DEVICE(SDIO_MANF_ID_CSR,SDIO_CARD_ID_UNIFI_3) },
    { SDIO_DEVICE(SDIO_MANF_ID_CSR,SDIO_CARD_ID_UNIFI_4) },
    { /* end: all zeroes */				},
};

MODULE_DEVICE_TABLE(sdio, unifi_ids);

//static void wifi_dummy_notify_sdio(struct work_struct *work){
//static void wifi_dummy_notify_sdio(struct mmc_host *host){

static int wifi_dummy_probe(struct sdio_func *func,
			const struct sdio_device_id *id){

   mmc = func->card->host;
   DBG("%s\n", __func__);

   pid_t current_test_thread;
   current_test_thread = kernel_thread(wifi_dummy_notify_sdio, NULL, 0);
   if(current_test_thread<0)
       printk("!!!!! test_current_thread isn't allowed to be created !!!\n");
   
   return 0; 
}

static void wifi_dummy_remove(struct sdio_func *func){
   return;
}


static struct sdio_driver dummy_wifi_driver = {
    .name       = "wifi_dummy",
    .probe	= wifi_dummy_probe,
    .remove	= wifi_dummy_remove,
    .id_table	= unifi_ids,
};

static void wifi_dummy_notify_sdio( void *pdata){
 
   DBG("%s\n", __func__);
   
   msleep(20000);
//   mmc_power_save_host(mmc);  
   DBG("%s, power_save_host\n", __func__);
   gpio_direction_output(sprd_3rdparty_gpio_wifi_reset, 0);
   gpio_set_value(sprd_3rdparty_gpio_wifi_reset,0);
   gpio_direction_output(sprd_3rdparty_gpio_wifi_pwd,0);
   gpio_set_value(sprd_3rdparty_gpio_wifi_pwd,0);

   //LDO_TurnOffLDO(LDO_LDO_WIF1);

   mmc_power_off(mmc);  

   sdio_unregister_driver(&dummy_wifi_driver);
   return;
}

static int __init wifi_dummy_init(void)
{
	int r = 0;

    	r = sdio_register_driver(&dummy_wifi_driver);
   	if (r) {
        	printk(KERN_ERR "unifi_sdio: Failed to register UniFi SDIO driver: %d\n", r);
	}
	
	DBG("=================wifi dummy========================\n");

	return 0;
}

late_initcall(wifi_dummy_init); /* try to load after dma driver when built-in */
