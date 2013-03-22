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
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/globalregs.h>
#include <mach/regulator.h>
#include <mach/hardware.h>
#include <linux/interrupt.h>

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

/* MMC_RESTORE_REGS depends on devices of 3rd parties, sdio registers
 *will be cleared after suspend. host wakeup only supported in sdio host1
 */
#define MMC_RESTORE_REGS

/* MMC_HOST_WAKEUP_SUPPORTED should be defined if sdio devices use data1
 *to wake the system up
 */

#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
#include <mach/pinmap.h>
static unsigned int sdio_wakeup_irq;
#define HOST_WAKEUP_GPIO     22
#endif

#ifdef CONFIG_MMC_BUS_SCAN
static struct sdhci_host *sdhci_host_g = NULL;
#endif

#ifdef MMC_RESTORE_REGS
static unsigned int host_addr = 0;
static unsigned int host_blk_size = 0;
static unsigned int host_blk_cnt = 0;
static unsigned int host_arg = 0;
static unsigned int host_tran_mode = 0;
static unsigned int host_ctrl = 0;
static unsigned int host_power = 0;
static unsigned int host_clk = 0;
#endif

extern void mmc_power_off(struct mmc_host* mmc);
extern void mmc_power_up(struct mmc_host* mmc);

/* we don't need to restore sdio0, because of CONFIG_MMC_BLOCK_DEFERED_RESUME=y */
#ifdef MMC_RESTORE_REGS
static void sdhci_save_regs(struct sdhci_host *host)
{
	if (!strcmp("Spread SDIO host1", host->hw_name)){
		host_addr = sdhci_readl(host, SDHCI_DMA_ADDRESS);
		host_blk_size = sdhci_readw(host, SDHCI_BLOCK_SIZE);
		host_blk_cnt = sdhci_readw(host, SDHCI_BLOCK_COUNT);
		host_arg = sdhci_readl(host, SDHCI_ARGUMENT);
		host_tran_mode = sdhci_readw(host, SDHCI_TRANSFER_MODE);
		host_ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		host_power = sdhci_readb(host, SDHCI_POWER_CONTROL);
		host_clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	}
}

static void sdhci_restore_regs(struct sdhci_host *host)
{
	if (!strcmp("Spread SDIO host1", host->hw_name)){
		sdhci_writel(host, host_addr, SDHCI_DMA_ADDRESS);
		sdhci_writew(host, host_blk_size, SDHCI_BLOCK_SIZE);
		sdhci_writew(host, host_blk_cnt, SDHCI_BLOCK_COUNT);
		sdhci_writel(host, host_arg, SDHCI_ARGUMENT);
		sdhci_writew(host, host_tran_mode, SDHCI_TRANSFER_MODE);
		sdhci_writeb(host, host_ctrl, SDHCI_HOST_CONTROL);
		sdhci_writeb(host, host_power, SDHCI_POWER_CONTROL);
		sdhci_writew(host, host_clk, SDHCI_CLOCK_CONTROL);
	}
}
#ifdef CONFIG_MMC_DEBUG
static void sdhci_dump_saved_regs(struct sdhci_host *host)
{
	if (!strcmp("Spread SDIO host1", host->hw_name)){
		printk("%s, host_addr:0x%x\n", host->hw_name, host_addr);
		printk("%s, host_blk_size:0x%x\n", host->hw_name, host_blk_size);
		printk("%s, host_blk_cnt:0x%x\n", host->hw_name, host_blk_cnt);
		printk("%s, host_arg:0x%x\n", host->hw_name, host_arg);
		printk("%s, host_tran_mode:0x%x\n", host->hw_name, host_tran_mode);
		printk("%s, host_ctrl:0x%x\n", host->hw_name, host_ctrl);
		printk("%s, host_power:0x%x\n", host->hw_name, host_power);
		printk("%s, host_clk:0x%x\n", host->hw_name, host_clk);
	}
}
#endif
#endif

#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
static irqreturn_t sdhci_wakeup_irq_handler(int irq, void *dev)
{
	struct sdhci_host *host = (struct sdhci_host *)dev;
	struct mmc_host *mmc = host->mmc;

	printk("sdhci_wakeup_irq_handler\n");
	/* Disable interrupt before calling handler */
	disable_irq_nosync(irq);

	wake_lock_timeout(&mmc->detect_wake_lock, HZ / 2);

	return IRQ_HANDLED;
}

void sdhci_set_data1_to_gpio(struct sdhci_host *host)
{
	unsigned int val;
	/* configurate sdio1 data1 to gpio when system in deep sleep */
	val = BITS_PIN_DS(1) | BITS_PIN_AF(3)  |
		BIT_PIN_WPU  | BIT_PIN_SLP_WPU |
		BIT_PIN_SLP_IE ;
	__raw_writel( val, CTL_PIN_BASE + REG_PIN_SD2_D1 );

	printk("%s, PIN_SD2_D1_REG:0x%x\n", __func__, __raw_readl(CTL_PIN_BASE + REG_PIN_SD2_D1));
	printk("sdhci_set_data1_to_gpio done\n");
}

void sdhci_set_gpio_to_data1(struct sdhci_host *host)
{
	unsigned int val;
	/* configurate sdio1 gpio to data1 when system wakeup */
	val = __raw_readl( CTL_PIN_BASE + REG_PIN_SD2_D1 );
	val = BITS_PIN_DS(1) | BITS_PIN_AF(0)  |
		BIT_PIN_WPU  | BIT_PIN_SLP_NUL |
		BIT_PIN_SLP_Z ;
	__raw_writel( val, CTL_PIN_BASE + REG_PIN_SD2_D1 );

	printk("%s, REG_PIN_SD2_D1:0x%x\n", __func__, __raw_readl(CTL_PIN_BASE + REG_PIN_SD2_D1));
	printk("sdhci_set_gpio_to_data1 done\n");
}


static void  sdhci_host_wakeup_set( struct sdhci_host *host )
{
	unsigned int val;
	int ret;


	if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
		sdhci_set_data1_to_gpio(host);
		gpio_request(HOST_WAKEUP_GPIO, "host_wakeup_irq");
		sdio_wakeup_irq = gpio_to_irq(HOST_WAKEUP_GPIO);
		gpio_direction_input(HOST_WAKEUP_GPIO);
		ret = request_threaded_irq(sdio_wakeup_irq, sdhci_wakeup_irq_handler, NULL,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "host_wakeup_irq", host);
		if(ret){
			printk(KERN_ERR "%s, request threaded irq error:%d\n",
				mmc_hostname(host->mmc), ret);
			return;
		}
		enable_irq_wake(sdio_wakeup_irq);
	}
	return;
}

static void  sdhci_host_wakeup_clear(struct sdhci_host *host)
{
       if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
		disable_irq_wake(sdio_wakeup_irq);
		free_irq(sdio_wakeup_irq, host);
		gpio_free(HOST_WAKEUP_GPIO);
		sdhci_set_gpio_to_data1(host);
        }
	return;
}
#endif

#ifdef CONFIG_MMC_BUS_SCAN
/*
 *  force card detection
 *  some sdio devices can use this API for detection
 */
void sdhci_bus_scan(void)
{
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		printk("%s, entry\n", __func__);
		if (sdhci_host_g->ops->set_clock) {
			sdhci_host_g->ops->set_clock(sdhci_host_g, 1);
		}

		sdhci_reinit(sdhci_host_g);
		mmc_detect_change(sdhci_host_g->mmc, 0);
	}

	return;
}
EXPORT_SYMBOL_GPL(sdhci_bus_scan);

unsigned int sdhci_wifi_detect_isbusy(void) {
	unsigned int busy = 0;
	#ifdef CONFIG_MMC_BUS_SCAN
	if(sdhci_host_g && sdhci_host_g->mmc) {
		busy = work_busy(&sdhci_host_g->mmc->detect.work);
	}
	#endif
	return busy;
}
EXPORT_SYMBOL_GPL(sdhci_wifi_detect_isbusy);
#endif

/*
 *   set indicator indicates that whether any devices attach on sdio bus.
 *   NOTE: devices must already attached on bus before calling this function.
 *   @ on: 0---deattach devices
 *         1---attach devices on bus
 *   @ return: 0---set indicator ok
 *             -1---no devices on sdio bus
 */
int sdhci_device_attach(int on)
{
	struct mmc_host *mmc = NULL;
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		mmc = sdhci_host_g->mmc;
		if(mmc->card){
			sdhci_host_g->dev_attached = on;
			if(!on){
				mmc_power_off(mmc);
			}else{
				mmc_power_up(mmc);
			}
		}else{
			/* no devices */
			sdhci_host_g->dev_attached = 0;
			return -1;
		}
		return 0;
	}
}
EXPORT_SYMBOL_GPL(sdhci_device_attach);

/*
 *   Slave start sdhci_bus_scan Ops then check SDIO card attach Bus status
 *
 *   @ return:  true--- SDIO device attach ready
 *              false---SDIO device attach not ready
 */
int sdhci_device_attached()
{
	struct mmc_host *mmc = NULL;
	if(sdhci_host_g && (sdhci_host_g->mmc)){
		mmc = sdhci_host_g->mmc;
		if(mmc->card){
			return true;
		}else{
			return false;
		}
	}
	return false;
}
EXPORT_SYMBOL_GPL(sdhci_device_attached);
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
		if (clock >= SDIO_BASE_CLK_96M) {
			clk_parent = clk_get(NULL, "clk_96m");
		} else if (clock >= SDIO_BASE_CLK_64M) {
			clk_parent = clk_get(NULL, "clk_64m");
		} else if (clock >= SDIO_BASE_CLK_48M) {
			clk_parent = clk_get(NULL, "clk_48m");
		} else {
			clk_parent = clk_get(NULL, "clk_26m");
		}
	}else if( !strcmp(host->hw_name, "Spread SDIO host1") ){
		host->clk = clk_get(NULL, "clk_sdio1");
		if (clock >= SDIO_BASE_CLK_96M) {
			clk_parent = clk_get(NULL, "clk_96m");
		} else if (clock >= SDIO_BASE_CLK_64M) {
			clk_parent = clk_get(NULL, "clk_64m");
		} else if (clock >= SDIO_BASE_CLK_48M) {
			clk_parent = clk_get(NULL, "clk_48m");
		} else {
			clk_parent = clk_get(NULL, "clk_26m");
		}
	}else  if( !strcmp(host->hw_name, "Spread SDIO host2") ){
		host->clk = clk_get(NULL, "clk_sdio2");
		clk_parent = clk_get(NULL, "clk_96m");
	}else if( !strcmp(host->hw_name, "Spread EMMC host0") ){
		host->clk = clk_get(NULL, "clk_emmc0");
		clk_parent = clk_get(NULL, "clk_26m");
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
		if(!host->clock){
			clk_enable(host->clk);
		}
	}
	pr_debug("clock:%d, host->clock:%d, AHB_CTL0:0x%x\n", clock,host->clock,
			sprd_greg_read(REG_TYPE_AHB_GLOBAL, AHB_CTL0));
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
		else if (!strcmp("Spread SDIO host2", host->hw_name)){
			host->vmmc = regulator_get(mmc_dev(host->mmc),
							REGU_NAME_WIFIIO);
			if (IS_ERR(host->vmmc)) {
				printk(KERN_ERR "%s: no vmmc regulator found\n",
							mmc_hostname(host->mmc));
				host->vmmc = NULL;
			}
		}
		else if (!strcmp("Spread EMMC host0", host->hw_name)){
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
		if (host->vmmc){
			ret = regulator_disable(host->vmmc);
			if(ret){
				printk(KERN_ERR "%s, disable regulator error:%d\n",
					mmc_hostname(host->mmc), ret);
			}
		}
	}else{
		if(host->vmmc){
			if(host->pwr) {
				ret = regulator_set_voltage(host->vmmc, volt_level,
										volt_level);
				if(ret){
					printk(KERN_ERR "%s, set voltage error:%d\n",
						mmc_hostname(host->mmc), ret);
					return;
				}
			}else{
				ret = regulator_enable(host->vmmc);
				if(ret){
					printk(KERN_ERR "%s, enabel regulator error:%d\n",
						mmc_hostname(host->mmc), ret);
					return;
				}
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
#ifdef CONFIG_ARCH_SC7710
	if(!strcmp(host->hw_name, "Spread SDIO host2")){
		/* Enable SDIO1 Module */
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, BIT_SDIO2_EB, AHB_CTL6);
		/* reset sdio1 module*/
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, BIT_SD2_SOFT_RST, AHB_SOFT2_RST);
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, BIT_SD2_SOFT_RST, AHB_SOFT2_RST);
	}
	if(!strcmp(host->hw_name, "Spread EMMC host0")){
		/* Enable SDIO1 Module */
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, BIT_EMMC_EB, AHB_CTL6);
		/* reset sdio1 module*/
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, BIT_EMMC_SOFT_RST, AHB_SOFT2_RST);
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, BIT_EMMC_SOFT_RST, AHB_SOFT2_RST);
	}
#endif
	sdhci_sprd_set_base_clock(host, SDIO_MAX_CLK);

}

extern struct class *sec_class;
static struct device *sd_detection_cmd_dev;
int gpio_detect;

static ssize_t sd_detection_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int	detect;
	if(gpio_detect != 777)
	{
		detect = gpio_get_value(gpio_detect);

		/* File Location : /sys/class/sec/sdcard/status */
		pr_info("%s : detect = %d.\n", __func__,  detect);
		detect = !detect;
		if (detect)
		{
			pr_info("sdhci: card inserted.\n");
			return sprintf(buf, "Insert\n");
		}
		else
		{
			pr_info("sdhci: card removed.\n");
			return sprintf(buf, "Remove\n");
		}
	}
	else
	{
		pr_info("sdhci: card removed.\n");
		return sprintf(buf, "Remove\n");
	}
}

static DEVICE_ATTR(status, 0444, sd_detection_cmd_show, NULL);

void sdhci_sprd_fix_controller_1p8v(struct sdhci_host *host) {
    local_irq_disable();
    host->mmc->ocr_avail |= MMC_VDD_165_195;
    host->mmc->ocr_avail_sdio |= MMC_VDD_165_195;
    host->mmc->ocr_avail_sd |= MMC_VDD_165_195;
    host->mmc->ocr_avail_mmc |= MMC_VDD_165_195;
    local_irq_enable();
}

static int __devinit sdhci_sprd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct resource *res;
	int ret, irq;
#ifdef CONFIG_MMC_CARD_HOTPLUG
	int sd_detect_gpio;
	int *sd_detect;
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
	pr_debug("sdio: host->ioaddr:0x%x\n", (unsigned int)host->ioaddr);
	if (0 == pdev->id){
		host->hw_name = "Spread SDIO host0";
	}else if(1 == pdev->id){
		host->hw_name = "Spread SDIO host1";
	}
	else if(2 == pdev->id){
		host->hw_name = "Spread SDIO host2";
	}
	else if(3 == pdev->id){
		host->hw_name = "Spread EMMC host0";
		host->mmc->caps |= MMC_CAP_8_BIT_DATA /*| MMC_CAP_1_8V_DDR*/;
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
#ifdef CONFIG_MMC_CARD_HOTPLUG
	gpio_detect = 777;
	sd_detect = dev_get_platdata(dev);
	if(sd_detect && (*sd_detect > 0)){
		sd_detect_gpio = *sd_detect;
		pr_info("%s, sd_detect_gpio:%d\n", __func__, sd_detect_gpio);

		if (0 == pdev->id){
			ret = gpio_request(sd_detect_gpio, "sdio0_detect");
		}else{
			ret = gpio_request(sd_detect_gpio, "sdio1_detect");
		}
		gpio_export( sd_detect_gpio, 1);

		if (ret) {
			dev_err(dev, "cannot request gpio\n");
			return -1;
		}

		ret = gpio_direction_input(sd_detect_gpio);
		if (ret) {
			dev_err(dev, "gpio can not change to input\n");
			return -1;
		}

		detect_irq = gpio_to_irq(sd_detect_gpio);
		if (detect_irq < 0){
			dev_err(dev, "cannot alloc detect irq\n");
			return -1;
		}
                gpio_detect = sd_detect_gpio;
		host_data->detect_irq = detect_irq;
	}else{
		printk("%s, sd_detect_gpio == 0 \n", __func__ );
	}
#endif
	if (sd_detection_cmd_dev == NULL)
	{
		sd_detection_cmd_dev = device_create(sec_class, NULL, 0, NULL, "sdcard");

		if (IS_ERR(sd_detection_cmd_dev))
			pr_err("Fail to create sysfs dev\n");

		if (device_create_file(sd_detection_cmd_dev,&dev_attr_status) < 0)
			pr_err("Fail to create sysfs file\n");
	}

	host->clk = NULL;
	sdhci_module_init(host);

#ifdef CONFIG_PM_RUNTIME
	ret = pm_runtime_set_active(&(pdev)->dev);
	if (ret < 0)
		pr_debug("%s: %s: failed with error %d", mmc_hostname(host->mmc),
				__func__, ret);
	/*
	 * There is no notion of suspend/resume for SD/MMC/SDIO
	 * cards. So host can be suspended/resumed without
	 * worrying about its children.
	 */
	pm_suspend_ignore_children(&(pdev)->dev, true);

	/*
	 * MMC/SD/SDIO bus suspend/resume operations are defined
	 * only for the slots that will be used for non-removable
	 * media or for all slots when CONFIG_MMC_UNSAFE_RESUME is
	 * defined. Otherwise, they simply become card removal and
	 * insertion events during suspend and resume respectively.
	 * Hence, enable run-time PM only for slots for which bus
	 * suspend/resume operations are defined.
	 */
#ifdef CONFIG_MMC_UNSAFE_RESUME
	/*
	 * If this capability is set, MMC core will enable/disable host
	 * for every claim/release operation on a host. We use this
	 * notification to increment/decrement runtime pm usage count.
	 */
	host->mmc->caps |= MMC_CAP_DISABLE;
	pm_runtime_enable(&(pdev)->dev);
#else
	if (host->mmc->caps & MMC_CAP_NONREMOVABLE) {
		host->mmc->caps |= MMC_CAP_DISABLE;
		pm_runtime_enable(&(pdev)->dev);
	}
#endif
	mmc_set_disable_delay(host->mmc, 500);
#endif
	host->mmc->caps |= MMC_CAP_HW_RESET;
	host->mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;
	host->mmc->pm_caps |= (MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ);
#ifdef CONFIG_MACH_KYLEW
	if(pdev->id == 1){
		host->mmc->pm_caps |= MMC_CAP_NONREMOVABLE;
		pm_runtime_disable(&(pdev)->dev); /* temprarily disable runtime case because of error arising when turn on*/
	} else if(pdev->id == 0) {
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
		pm_runtime_disable(&(pdev)->dev);
	}
#elif defined (CONFIG_ARCH_SC7710)
	if(pdev->id == 1){
		host->mmc->pm_caps |= MMC_CAP_NONREMOVABLE;
//		pm_runtime_disable(&(pdev)->dev);
	} else if(pdev->id == 2) {
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
//		pm_runtime_disable(&(pdev)->dev);
	}
	pm_runtime_disable(&(pdev)->dev); /*disable pm runtime temprarily in 7710*/
#elif defined (CONFIG_MACH_SP7702)
	if(pdev->id == 1){
		host->mmc->pm_flags |= MMC_PM_KEEP_POWER;
		pm_runtime_disable(&(pdev)->dev);
	}
#else
	if(pdev->id == 1){
		host->mmc->pm_caps |= MMC_CAP_NONREMOVABLE;

	}
#endif
	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed\n");
		goto err_add_host;
	}
#if defined(CONFIG_MACH_KYLEW) || defined(CONFIG_MACH_SP7702)
	sdhci_sprd_fix_controller_1p8v(host);
#endif
#ifdef CONFIG_MMC_BUS_SCAN
	if (pdev->id == 1)
		sdhci_host_g = host;
#endif
	return 0;

err_add_host:
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);
#endif
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

#ifdef CONFIG_PM_RUNTIME
	if (pm_runtime_suspended(&(pdev)->dev))
		pm_runtime_resume(&(pdev)->dev);
#endif
	sdhci_remove_host(host, 1);
	sdhci_free_host(host);

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);
#endif
	return 0;
}


#ifdef CONFIG_PM
static int sdhci_sprd_suspend(struct platform_device *dev, pm_message_t pm)
{
	int ret = 0;
	struct sdhci_host *host = platform_get_drvdata(dev);
	printk("%s, %s, start\n", mmc_hostname(host->mmc), __func__ );
#ifdef MMC_RESTORE_REGS
#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif
if(host->mmc && host->mmc->card)
	sdhci_save_regs(host);

#ifdef CONFIG_MMC_DEBUG
	sdhci_dump_saved_regs(host);
#endif
#endif
#ifdef CONFIG_PM_RUNTIME
	if (!pm_runtime_suspended(&dev->dev)) {
		ret = sdhci_suspend_host(host, pm);
	}else{
		printk("%s, %s, runtime suspended\n", mmc_hostname(host->mmc), __func__ );
	}
#else
	ret = sdhci_suspend_host(host, pm);
#endif
	if(ret){
		printk("~wow, %s suspend error %d\n", host->hw_name, ret);
		return ret;
	}

#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
	if( !strcmp(host->hw_name, "Spread SDIO host1") ) {
		sdhci_host_wakeup_set(host);
	}
#endif


	/* disable ahb clock */
	if(host->ops->set_clock){
		host->ops->set_clock(host, 0);
	}
	printk("%s, %s, done\n", mmc_hostname(host->mmc), __func__ );

	return 0;
}

static int sdhci_sprd_resume(struct platform_device *dev)
{
	struct sdhci_host *host = platform_get_drvdata(dev);
	printk("%s, %s, start\n", mmc_hostname(host->mmc), __func__ );

	/* enable ahb clock to restore registers */
	if(host->ops->set_clock){
		host->ops->set_clock(host, 1);
	}
#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
	if( !strcmp(host->hw_name, "Spread SDIO host1") ) {
		sdhci_host_wakeup_clear(host);
	}
#endif


#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif

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
	printk("%s, %s, done\n", mmc_hostname(host->mmc), __func__ );
#ifdef MMC_RESTORE_REGS

#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif
if(host->mmc && host->mmc->card)
	sdhci_restore_regs(host);
#endif

	return 0;
}

#else
#define sdhci_sprd_suspend NULL
#define sdhci_sprd_resume NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int sdhci_runtime_suspend(struct device *dev){
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;
	int rc = 0;
	if (mmc) {
		mmc->suspend_task = current;

		/*
		 * MMC core thinks that host is disabled by now since
		 * runtime suspend is scheduled after sdhci_disable()
		 * is called. Thus, MMC core will try to enable the host
		 * while suspending it. This results in a synchronous
		 * runtime resume request while in runtime suspending
		 * context and hence inorder to complete this resume
		 * requet, it will wait for suspend to be complete,
		 * but runtime suspend also can not proceed further
		 * until the host is resumed. Thus, it leads to a hang.
		 * Hence, increase the pm usage count before suspending
		 * the host so that any resume requests after this will
		 * simple become pm usage counter increment operations.
		 */
		pm_runtime_get_noresume(dev);
		/* If there is pending detect work abort runtime suspend */
		if (unlikely(work_busy(&mmc->detect.work)))
			rc = -EAGAIN;
		else{
			host->suspending = 1;
			rc = mmc_suspend_host(mmc);
		}
		pm_runtime_put_noidle(dev);

		host->suspending = 0;
		mmc->suspend_task = NULL;
	}
	return rc;
}

static int sdhci_runtime_resume(struct device *dev){
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;
	int err = 0;

	if (mmc) {
		(host->mmc)->bus_resume_flags &= ~MMC_BUSRESUME_MANUAL_RESUME;
		err = mmc_resume_host(mmc);
		if(err)
			printk("%s, runtime resume failed, err:%d\n", mmc_hostname(mmc), err);
		(host->mmc)->bus_resume_flags |= MMC_BUSRESUME_MANUAL_RESUME;
	}

	return err;
}

#define SPRD_MMC_IDLE_TIMEOUT 5000
static int sdhci_runtime_idle(struct device *dev){
	return 0;
}

static int sdhci_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct sdhci_host *host = dev_get_drvdata(dev);

#ifdef MMC_RESTORE_REGS
#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif
if(host->mmc && host->mmc->card)
	sdhci_save_regs(host);

#ifdef CONFIG_MMC_DEBUG
	sdhci_dump_saved_regs(host);
#endif
#endif

	if (!pm_runtime_suspended(dev)) {
		if (!(host->mmc->card && mmc_card_sdio(host->mmc->card))) {
			/*
			 * decrement power.usage_counter if it's
			 * not zero earlier
			 */
			pm_runtime_put_noidle(dev);
			ret = pm_runtime_suspend(dev);
		}

		/*
		 * if device runtime PM status is still not suspended
		 * then perform suspend here.
		 */
		if (!pm_runtime_suspended(dev))
			ret = sdhci_runtime_suspend(dev);
		/* device suspend here,can't use mmc_host_do_disable() function.
		 * we must change the host->is_resumed,host->mmc->enabled
		 * value,make sure the device can normal resume.
		*/
		if (host->is_resumed)
			host->is_resumed = 0;
		if (host->mmc->enabled)
			host->mmc->enabled = 0;
	}else{
		printk("%s, runtime_status:%d\n", mmc_hostname(host->mmc), dev->power.runtime_status);
	}

#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
	if( !strcmp(host->hw_name, "Spread SDIO host1") ) {
		sdhci_host_wakeup_set(host);
	}
#endif



	/* disable ahb clock */
	if(host->ops->set_clock){
		host->ops->set_clock(host, 0);
	}

	return 0;
}

static int sdhci_pm_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	unsigned long clock = 0;

	/* enable ahb clock to restore registers */
	if(host->ops->set_clock){
		host->ops->set_clock(host, 1);
	}
#ifdef CONFIG_MMC_HOST_WAKEUP_SUPPORTED
	if( !strcmp(host->hw_name, "Spread SDIO host1") ) {
		sdhci_host_wakeup_clear(host);
	}
#endif


	if (!pm_runtime_suspended(dev))
		sdhci_resume_host(host);
	else
		sdhci_reinit(host);//restore irq enable register

#if 0
	/* disable sdio0(T-FLASH card) ahb clock */
	if(mmc_bus_manual_resume(host->mmc)){
		if(host->ops->set_clock){
			host->ops->set_clock(host, 0);
		}
	}
#endif

	if(!(host->mmc->card) || !host->dev_attached){
		if(host->ops->set_clock){
			clock = host->clock;
			host->ops->set_clock(host, 0);
			if(clock != 0){
				host->clock = clock;
			}
		}
	}
#ifdef MMC_RESTORE_REGS

#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif
	if(host->mmc && host->mmc->card)
		sdhci_restore_regs(host);
#endif

#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif

#if !defined(CONFIG_MACH_SP7702) && !defined(CONFIG_MACH_KYLEW)
	if(host->ops->set_clock){
		clock = host->clock;
		host->ops->set_clock(host, 0);
	}
#endif
	return 0;
}

static const struct dev_pm_ops sdhci_dev_pm_ops = {
	.suspend	 = sdhci_pm_suspend,
	.resume		 = sdhci_pm_resume,
	.runtime_suspend = sdhci_runtime_suspend,
	.runtime_resume  = sdhci_runtime_resume,
	.runtime_idle    = sdhci_runtime_idle,
};
#else
static const struct dev_pm_ops sdhci_dev_pm_ops = {
	.suspend	 = NULL,
	.resume		 = NULL,
	.runtime_suspend = NULL,
	.runtime_resume  = NULL,
	.runtime_idle    = NULL,
};

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
#ifdef CONFIG_PM_RUNTIME
		.pm 		=  &sdhci_dev_pm_ops,
#endif
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
