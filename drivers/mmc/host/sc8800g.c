/* linux/drivers/mmc/host/sdhci-sprd.c
 *
 * Copyright 2010 Spreadtrum Inc.
 *      Yingchun Li <yingchun.li@spreadtrum.com>
 *      http://www.spreadtrum.com/
 *
 * SDHCI (HSMMC) support for Spreadtrum 8800H series.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>//wong

#include <linux/gpio.h>

#include <mach/regs_global.h>
#include <mach/regs_ahb.h>

#include "sdhci.h"
#include "sprdmci.h"



#define     SDIO_BASE_CLK_96M       96000000        // 96 MHz
#define     SDIO_BASE_CLK_80M       80000000        // 80 MHz
#define     SDIO_BASE_CLK_64M       64000000        // 64 MHz
#define     SDIO_BASE_CLK_50M       50000000        // 50 MHz   ,should cfg MPLL to 300/350/400???
#define     SDIO_BASE_CLK_48M       48000000        // 48 MHz
#define     SDIO_BASE_CLK_40M       40000000        // 40 MHz
#define     SDIO_BASE_CLK_32M       32000000        // 32 MHz
#define     SDIO_BASE_CLK_26M       26000000            // 26  MHz
#define     SDIO_BASE_CLK_25M       25000000            // 25  MHz
#define     SDIO_BASE_CLK_20M       20000000            // 20  MHz
#define     SDIO_BASE_CLK_16M       16000000        // 16 MHz
#define     SDIO_BASE_CLK_8M        8000000         // 8  MHz

#define SDIO_MAX_CLK  SDIO_BASE_CLK_96M  //96Mhz

/*
*   NOTE: sdhci_sdio1_set_power() is for CSR UNIFI6030, if wifi were not initialized at boot
* boot time, there will be 4mA current leakage;
*   For another devices on sdio1 bus, should not call this function if on strong reasons
*/
static void sdhci_sdio1_set_power( struct sdhci_host *host ){
 
   printk("%s\n", __func__);
   
   msleep(10000);

   mmc_power_off(host->mmc);  
   
   __gpio_set_value(140, 0);
   __gpio_set_value(137, 0);
   
   sdhci_bus_scan( );

   return;
}


/**
 * sdhci_sprd_get_max_clk - callback to get maximum clock frequency.
 * @host: The SDHCI host instance.
 *
 * Callback to return the maximum clock rate acheivable by the controller.
*/
static unsigned int sdhci_sprd_get_max_clk(struct sdhci_host *host)
{
	return SDIO_MAX_CLK;
}

/**
 * sdhci_sprd_set_base_clock - set base clock for SDIO module
 * @clock: The clock rate being requested.
 *
*/
static void sdhci_sprd_set_base_clock(unsigned int clock)
{
	unsigned long flags;

	/* don't bother if the clock is going off. */
	if (clock == 0)
		return;

	if (clock > SDIO_MAX_CLK)
		clock = SDIO_MAX_CLK;


	local_irq_save(flags);

    //Select the clk source of SDIO
	__raw_bits_and(~(BIT_17|BIT_18), GR_CLK_GEN5);

	if (clock >= SDIO_BASE_CLK_96M) {
		//default is 96M
		;
	} else if (clock >= SDIO_BASE_CLK_64M) {
		__raw_bits_or((1<<17), GR_CLK_GEN5);
	} else if (clock >= SDIO_BASE_CLK_48M) {
		__raw_bits_or((2<<17), GR_CLK_GEN5);
	} else {
		__raw_bits_or((3<<17), GR_CLK_GEN5);
	}

	local_irq_restore(flags);
	pr_info("after set sd clk, CLK_GEN5:%x\r\n",
		__raw_readl(GR_CLK_GEN5));
	return;
}

static void sdhci_sprd_set_ahb_clock(struct sdhci_host *host, unsigned int clock){
   unsigned int val = __raw_readl(AHB_CTL0);

   pr_debug("%s, set ahb clk:%u\n", __func__, clock);
   if(clock == 0){
      if(!strcmp(host->hw_name, "Spread SDIO host0")){
          printk("SDIO0: %s, set ahb clk:%u\n", __func__, clock);
         val &= ~AHB_CTL0_SDIO0_EN;
         __raw_writel(val, AHB_CTL0);
      }else if(!strcmp(host->hw_name, "Spread SDIO host1")){
          printk("SDIO1: %s, set ahb clk:%u\n", __func__, clock);
	  val &= ~AHB_CTL0_SDIO1_EN;
          __raw_writel(val, AHB_CTL0);
	  host->clock = 0;
      }	
   }else{
      if(!strcmp(host->hw_name, "Spread SDIO host0")){
          val |= AHB_CTL0_SDIO0_EN;
          __raw_writel(val, AHB_CTL0);
       }else if(!strcmp(host->hw_name, "Spread SDIO host1")){
          pr_debug("SDIO1: %s, set ahb clk:%u\n", __func__, clock);
	  val |= AHB_CTL0_SDIO1_EN;
          __raw_writel(val, AHB_CTL0);
       }
       msleep(3);
   }

   return;	  
}

/*
*   NOTE: sdhci_sdio1_set_power() is for CSR UNIFI6030, if wifi were not initialized at boot
* boot time, there will be 4mA current leakage;
*   For another devices on sdio1 bus, should not call this function if on strong reasons
*/
static void sdhci_sprd_set_power(struct sdhci_host *host, unsigned int power){
   printk("%s, entry\n", __func__ );
   if((!strcmp(host->hw_name, "Spread SDIO host1")) && (!power) ){
   
      pid_t sdio1_power_thread;
      sdio1_power_thread = kernel_thread(sdhci_sdio1_set_power, host, 0);
      if(sdio1_power_thread<0)
          printk("!!!!! sdio1_power_thread isn't allowed to be created !!!\n");
   
      printk("%s, done\n", __func__ );
   }
   return 0; 
}

static struct sdhci_ops sdhci_sprd_ops = {
	.get_max_clock		= sdhci_sprd_get_max_clk,
	.set_clock		= sdhci_sprd_set_ahb_clock,
//	.set_power      = sdhci_sprd_set_power,
};


static int __devinit sdhci_sprd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct resource *res;
	int sd_detect_gpio;
	int ret, irq, detect_irq;
	struct sprd_host_data *host_data;


	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(dev, sizeof(struct sprd_host_data));
	host_data = sdhci_priv(host);

	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	platform_set_drvdata(pdev, host);

	host->ioaddr = (void __iomem *)res->start;
#ifdef CONFIG_ARCH_SC8810
	if (0 == pdev->id)
			host->hw_name = "Spread SDIO host0";
	else
			host->hw_name = "Spread SDIO host1";
#else
	host->hw_name = "Spread SDIO host";
#endif
	host->ops = &sdhci_sprd_ops;
	/*
		SC8800G don't have timeout value and cann't find card
		insert, write protection...
		too sad
	*/
	host->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |\
		SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |\
		SDHCI_QUIRK_BROKEN_CARD_DETECTION|\
		SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	host->irq = irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	sd_detect_gpio = res ? res->start : -1;
#ifdef HOT_PLUG_SUPPORTED
	detect_irq = sprd_alloc_gpio_irq(sd_detect_gpio);
        if (detect_irq < 0){
		dev_err(dev, "cannot alloc detect irq\n");
                return -1;
	}
#endif
	host_data->detect_irq = detect_irq;

	sdhci_sprd_set_base_clock(SDIO_MAX_CLK);

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}
      
        host->mmc->pm_caps |= (MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ); 

	sdhci_sprd_set_power(host, 0);
    printk("sdhci_sprd_probe, done\n");
	return 0;

 err_add_host:
	sdhci_free_host(host);

	return ret;
}

static int __devexit sdhci_sprd_remove(struct platform_device *pdev)
{
	return 0;
}


#ifdef CONFIG_PM

static int sdhci_sprd_suspend(struct platform_device *dev, pm_message_t pm)
{
	int ret = 0; 
	struct sdhci_host *host = platform_get_drvdata(dev);
        
	ret = sdhci_suspend_host(host, pm);
        if(ret){
           printk("~wow, %s suspend error %d\n", host->hw_name, ret);
	   return ret;
	}
	return 0;
}

static int sdhci_sprd_resume(struct platform_device *dev)
{
	struct sdhci_host *host = platform_get_drvdata(dev);

    if(host->ops->set_clock){
	    host->ops->set_clock(host, 1); //enable ahb clock to restore registers
	}
        /*   VDD_SDIO must not be off when in deep-sleep, because of 
        *  host-wake-up.(correct?)
	    *    modification below is for CSR. CSR's chips need whole initialization  
	    *  even if the power is not off.So we clear MMC_PM_KEEP_POWER flag. before 
	    *  resume, and set it again after resume, or resume will 
	    *  failed. Other wifi manufacturers may not need this.
	    *    0x032a is vendor id of CSR. 
	    */
	if((host->mmc->card) && (host->mmc->card->cis.vendor ==  0x032a) ){
        printk("wifi: clear MMC_PM_KEEP_POWER flag\n");
		host->mmc->pm_flags &= ~MMC_PM_KEEP_POWER; 
	}
	
	sdhci_resume_host(host);
	
	if((host->mmc->card) && (host->mmc->card->cis.vendor ==  0x032a) ){		
        printk("wifi: set MMC_PM_KEEP_POWER flag\n");
        host->mmc->pm_flags |= MMC_PM_KEEP_POWER; 
	}
#if 0	
	if( !(host->mmc->pm_flags & MMC_PM_KEEP_POWER) ){
            if(host->ops->set_clock){
	      host->ops->set_clock(host, 0);
	    }
    }
#endif

	//disable sdio0(T-FLASH card) ahb clock  
	if(mmc_bus_manual_resume(host->mmc)) {
	   if(host->ops->set_clock){
	     host->ops->set_clock(host, 0);  
	   }
	}

	//disable sdio1(WIFI) ahb clock  
	if(!(host->mmc->card) ){
       if(host->ops->set_clock){
	     host->ops->set_clock(host, 0);  
	   }
	}

	return 0;
}

#else
#define sdhci_sprd_suspend NULL
#define sdhci_sprd_resume NULL
#endif

static struct platform_driver sdhci_sprd_driver = {
	.probe		= sdhci_sprd_probe,
	.remove		= __devexit_p(sdhci_sprd_remove),
	.suspend	= sdhci_sprd_suspend,
	.resume	        = sdhci_sprd_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "sprd-sdhci",
	},
};

static int __init sdhci_sprd_init(void)
{
	return platform_driver_register(&sdhci_sprd_driver);
}

static void __exit sdhci_sprd_exit(void)
{
	platform_driver_unregister(&sdhci_sprd_driver);
}

module_init(sdhci_sprd_init);
module_exit(sdhci_sprd_exit);

MODULE_DESCRIPTION("Spredtrum SDHCI glue");
MODULE_AUTHOR("Yingchun Li, <yingchun.li@spreadtrum.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sprd-sdhci");
