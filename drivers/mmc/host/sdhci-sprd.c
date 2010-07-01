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

#include <mach/regs_global.h>
#include "sdhci.h"

#define SDIO_MAX_CLK 40000000  //40Mhz

static unsigned int chip_get_mpll_clk(void)
{
	unsigned int pll_freq;
	unsigned int reg_value, M, N;

	reg_value = __raw_readl(GR_MPLL_MN);
	M = reg_value & 0x0FFF;
	N = (reg_value & 0x0FFF0000)>>16;

	pll_freq = 26*N/M;

	return (pll_freq*1000000);
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
 * sdhci_sprd_set_clock - callback on clock change
 * @host: The SDHCI host being changed
 * @clock: The clock rate being requested.
 *
 * When the card's clock is going to be changed, look at the new frequency
 * and find the best clock source to go with it.
*/
static void sdhci_sprd_set_clock(struct sdhci_host *host, unsigned int clock)
{
	unsigned long flags;
	volatile unsigned int clk_div,temp;

	pr_debug("set clock :%d", clock);
	/* don't bother if the clock is going off. */
	if (clock == 0)
		return;

	if (clock > SDIO_MAX_CLK)
		clock = SDIO_MAX_CLK;
	
	clk_div = chip_get_mpll_clk() / clock;

	if(0 != chip_get_mpll_clk() % clock)
	{
		clk_div++;
	}

	local_irq_save(flags);

	//Select the clk source of SDIO
	__raw_bits_and(~(BIT_17|BIT_18), GR_CLK_GEN5);
	
	//Configure the glabla dividor
	temp = __raw_readl(GR_CLK_GEN5);
	temp &= ~(BIT_5|BIT_6|BIT_7|BIT_8|BIT_9);
	temp |= ((clk_div-1) << 5);
	__raw_writel(temp, GR_CLK_GEN5);

	//Enable the SD clk
	__raw_bits_or(BIT_12, GR_CLK_EN);
	
	local_irq_restore(flags);
	return;
}

static u32 sdhci_sprd_readl(struct sdhci_host *host, int reg)
{
	return __raw_readl(host->ioaddr + reg);
}

static u16 sdhci_sprd_readw(struct sdhci_host *host, int reg)
{
	return __raw_readw(host->ioaddr + reg);
}

static u8 sdhci_sprd_readb(struct sdhci_host *host, int reg)
{
	return __raw_readb(host->ioaddr + reg);
}

static void sdhci_sprd_writel(struct sdhci_host *host, u32 val, int reg)
{
	__raw_writel(val, host->ioaddr + reg);
}

static void sdhci_sprd_writew(struct sdhci_host *host, u16 val, int reg)
{
	__raw_writew(val, host->ioaddr + reg);
}

static void sdhci_sprd_writeb(struct sdhci_host *host, u8 val, int reg)
{
	__raw_writeb(val, host->ioaddr + reg);
}
static struct sdhci_ops sdhci_sprd_ops = {
	.read_l	= sdhci_sprd_readl,
	.read_w	= sdhci_sprd_readw,
	.read_b	= sdhci_sprd_readb,
	.write_l	= sdhci_sprd_writel,
	.write_w	= sdhci_sprd_writew,
	.write_b	= sdhci_sprd_writeb,
	.get_max_clock		= sdhci_sprd_get_max_clk,
	.set_clock		= sdhci_sprd_set_clock,
};

static int __devinit sdhci_sprd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct resource *res;
	int ret, irq;


	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}
	pr_debug("sdio irq:%d \r\n", irq);
	//io_test();
	//timer_test();

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(dev, 0);
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	platform_set_drvdata(pdev, host);

	host->ioaddr = (void __iomem *)res->start;
	
	host->hw_name = "Spread SDIO host";
	host->ops = &sdhci_sprd_ops;
	host->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |\
		SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |\
		SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER;
	host->irq = irq;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}

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
	struct sdhci_host *host = platform_get_drvdata(dev);

	sdhci_suspend_host(host, pm);
	return 0;
}

static int sdhci_sprd_resume(struct platform_device *dev)
{
	struct sdhci_host *host = platform_get_drvdata(dev);

	sdhci_resume_host(host);
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
