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

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/globalregs.h>
#include <mach/regulator.h>
#include "sc8810g.h"


#define     SDIO0_SOFT_RESET        BIT(12)
#define     SDIO1_SOFT_RESET        BIT(16)

#define     SDIO_BASE_CLK_96M       96000000
#define     SDIO_BASE_CLK_64M       64000000
#define     SDIO_BASE_CLK_48M       48000000
#define     SDIO_BASE_CLK_26M       26000000
#define     SDIO_MAX_CLK            SDIO_BASE_CLK_96M

/* regulator use uv to set voltage */
#define     SDIO_VDD_VOLT_1V8       1800000
#define     SDIO_VDD_VOLT_3V0       3000000
#define     SDIO_VDD_VOLT_3V3       3300000

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
 * @host:  sdio host to be set.
 * @clock: The clock rate being requested.
*/
static void sdhci_sprd_set_base_clock(struct sdhci_host *host, unsigned int clock)
{
	struct clk *clk_parent;
	/* don't bother if the clock is going off. */
	if (clock == 0)
		return;

	if (clock > SDIO_MAX_CLK)
		clock = SDIO_MAX_CLK;

	/* Select the clk source of SDIO, default is 96MHz */
	if( !strcmp(host->hw_name, "Spread SDIO host0") ){
		host->clk = clk_get(NULL, "clk_sdio0");
	}else if( !strcmp(host->hw_name, "Spread SDIO host1") ){
		host->clk = clk_get(NULL, "clk_sdio1");
	}

	if (clock >= SDIO_BASE_CLK_96M) {
		clk_parent = clk_get(NULL, "clk_96m");
	} else if (clock >= SDIO_BASE_CLK_64M) {
		clk_parent = clk_get(NULL, "clk_64m");
	} else if (clock >= SDIO_BASE_CLK_48M) {
		clk_parent = clk_get(NULL, "clk_48m");
	} else {
		clk_parent = clk_get(NULL, "clk_24m");
	}
	clk_set_parent(host->clk, clk_parent);

	pr_debug("after set sd clk, CLK_GEN5:0x%x\n", sprd_greg_read(REG_TYPE_GLOBAL, GR_CLK_GEN5));

	return;
}

/**
 * sdhci_sprd_enable_clock - enable or disable sdio base clock
 * @host:  sdio host to be set.
 * @clock: The clock enable(clock>0) or disable(clock==0).
 *
*/
static void sdhci_sprd_enable_clock(struct sdhci_host *host, unsigned int clock)
{
	if(clock == 0){
		clk_disable(host->clk);
		host->clock = 0;
	}else{
		clk_enable(host->clk);
	}
	pr_debug("AHB_CTL0:0x%x\n", sprd_greg_read(REG_TYPE_AHB_GLOBAL, AHB_CTL0));
	return;
}

/*
*   The vdd_sdio is supplied by external LDO, power bit in register xxx is useless
*/
static void sdhci_sprd_set_power(struct sdhci_host *host, unsigned int power)
{
	unsigned int volt_level = 0;
	int ret;

	if(host->vmmc == NULL){
		/* in chip sc8810, sdio_vdd is not supplied by host,
		 * but regulator(LDO)
		*/
		if (!strcmp("Spread SDIO host0", host->hw_name)){
			host->vmmc = regulator_get(mmc_dev(host->mmc),
							REGU_NAME_SDHOST0);
			if (IS_ERR(host->vmmc)) {
				printk(KERN_ERR "%s: no vmmc regulator found\n",
							mmc_hostname(host->mmc));
				host->vmmc = NULL;
			}
		}else if (!strcmp("Spread SDIO host1", host->hw_name)){
			host->vmmc = regulator_get(mmc_dev(host->mmc),
							REGU_NAME_WIFIIO);
			if (IS_ERR(host->vmmc)) {
				printk(KERN_ERR "%s: no vmmc regulator found\n",
							mmc_hostname(host->mmc));
				host->vmmc = NULL;
			}
		}
	}

	switch(power){
	case SDHCI_POWER_180:
		volt_level = SDIO_VDD_VOLT_1V8;
		break;
	case SDHCI_POWER_300:
		volt_level = SDIO_VDD_VOLT_3V0;
		break;
	case SDHCI_POWER_330:
		volt_level = SDIO_VDD_VOLT_3V3;
		break;
	default:
		;
	}

	pr_debug("%s, power:%d, set regulator voltage:%d\n",
			mmc_hostname(host->mmc), power, volt_level);
	if(volt_level == 0){
		if (host->vmmc)
			ret = regulator_disable(host->vmmc);
	}else{
		if(host->vmmc){
			ret = regulator_set_voltage(host->vmmc, volt_level,
								volt_level);
			if(ret){
				printk(KERN_ERR "%s, set voltage error:%d\n",
					mmc_hostname(host->mmc), ret);
				return;
			}
			ret = regulator_enable(host->vmmc);
			if(ret){
				printk(KERN_ERR "%s, enabel regulator error:%d\n",
					mmc_hostname(host->mmc), ret);
				return;
			}
		}
	}

	return;
}

static struct sdhci_ops sdhci_sprd_ops = {
	.get_max_clock		= sdhci_sprd_get_max_clk,
	.set_clock		= sdhci_sprd_enable_clock,
	.set_power		= sdhci_sprd_set_power,
};

static void sdhci_module_init(struct sdhci_host* host)
{
	if(!strcmp(host->hw_name, "Spread SDIO host0")){
		/* Enable SDIO0 Module */
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_SDIO0_EN, AHB_CTL0);
		/* reset sdio0 module*/
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, SDIO0_SOFT_RESET, AHB_SOFT_RST);
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, SDIO0_SOFT_RESET, AHB_SOFT_RST);
	}
	if(!strcmp(host->hw_name, "Spread SDIO host1")){
		/* Enable SDIO1 Module */
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_SDIO1_EN, AHB_CTL0);
		/* reset sdio1 module*/
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, SDIO1_SOFT_RESET, AHB_SOFT_RST);
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, SDIO1_SOFT_RESET, AHB_SOFT_RST);
	}


	sdhci_sprd_set_base_clock(host, SDIO_MAX_CLK);
	sdhci_sprd_enable_clock(host, true);

}


static int __devinit sdhci_sprd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct resource *res;
	int ret, irq;
#ifdef HOT_PLUG_SUPPORTED
	int sd_detect_gpio;
	int detect_irq;
#endif
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
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}
	host_data = sdhci_priv(host);
	platform_set_drvdata(pdev, host);
	host->vmmc = NULL;
	host->ioaddr = (void __iomem *)res->start;
	pr_debug("sdio: host->ioaddr:0x%x\n", host->ioaddr);
	if (0 == pdev->id){
		host->hw_name = "Spread SDIO host0";
	}else{
		host->hw_name = "Spread SDIO host1";
	}
	host->ops = &sdhci_sprd_ops;
	/*
	 *   SC8810G don't have timeout value and cann't find card
	 *insert, write protection...
	 *   too sad
	 */
	host->quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |\
		SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |\
		SDHCI_QUIRK_BROKEN_CARD_DETECTION|\
		SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	host->irq = irq;
#ifdef HOT_PLUG_SUPPORTED
	/* hot plug is not supported yet, gpio name should be modifed*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	sd_detect_gpio = res ? res->start : -1;
	if (0 == pdev->id){
		ret = gpio_request(sd_detect_gpio, "sdio0_detect");
	}else{
		ret = gpio_request(sd_detect_gpio, "sdio1_detect");
	}
	if (ret) {
		dev_err(dev, "cannot request gpio\n");
		return -1;
	}
	detect_irq = gpio_to_irq(sd_detect_gpio);
	if (detect_irq < 0){
		dev_err(dev, "cannot alloc detect irq\n");
		return -1;
	}
	ret = gpio_direction_input(detect_irq);
	if (ret) {
		dev_err(dev, "gpio can not change to input\n");
		return -1;
	}
	host_data->detect_irq = detect_irq;
#endif
	host->clk = NULL;
	sdhci_module_init(host);

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}

	host->mmc->pm_caps |= (MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ);

	return 0;

err_add_host:
	sdhci_free_host(host);
	return ret;
}

/*
 * TODO: acommplish the funcion.
 * SDIO driver is a build-in module
 */
static int __devexit sdhci_sprd_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);

	sdhci_remove_host(host, 1);
	sdhci_free_host(host);
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

	/* enable ahb clock to restore registers */
	if(host->ops->set_clock){
		host->ops->set_clock(host, 1);
	}

	sdhci_resume_host(host);

	/* disable sdio0(T-FLASH card) ahb clock */
	if(mmc_bus_manual_resume(host->mmc)){
		if(host->ops->set_clock){
			host->ops->set_clock(host, 0);
		}
	}

	/* disable sdio1 ahb clock */
	if(!(host->mmc->card)){
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
	/* SDIO host is a build-in module
	 * .remove		= __devexit_p(sdhci_sprd_remove),
	 */
	.suspend	= sdhci_sprd_suspend,
	.resume		= sdhci_sprd_resume,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "sprd-sdhci",
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
