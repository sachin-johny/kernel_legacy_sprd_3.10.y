/*
 *  linux/drivers/mmc/host/sdhci.c - Secure Digital Host Controller Interface driver
 *
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * Thanks to the following companies for their support:
 *
 *     - JMicron (hardware and technical support)
 */

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/pm.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/mmc.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <mach/ldo.h>


#include <linux/wakelock.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/regs_global.h>
#include <linux/bitops.h>
#include <mach/bits.h>

#include "sdhci.h"
#include "sprdmci.h"

#define SDHCI_BUS_SCAN
#define MMC_HOST_WAKEUP_SUPPORTED
#define INT_IRQ_EN				(SPRD_INTCV_BASE + 0x08)
#define HOST_WAKEUP_GPIO  22

#ifdef MMC_HOST_WAKEUP_SUPPORTED
#include <mach/pm_wakesource.h>
#include <mach/regs_cpc.h>
#include <mach/mfp.h>
static unsigned int sdio_wakeup_irq;
#endif

#define MMC_AUTO_SUSPEND 
#define DRIVER_NAME "sdhci"
#define KERN_DEBUG " "

//#define MMC_RESTORE_REGS

#define DBG(f, x...) \
	pr_debug(DRIVER_NAME " [%s()]: " f, __func__,## x)

//no led used in our host
#if 0
#if defined(CONFIG_LEDS_CLASS) || (defined(CONFIG_LEDS_CLASS_MODULE) && \
	defined(CONFIG_MMC_SDHCI_MODULE))
#define SDHCI_USE_LEDS_CLASS
#endif
#endif

#ifdef SDHCI_BUS_SCAN
static struct sdhci_host *sdhci_host_g = NULL;
#endif


#ifdef MMC_AUTO_SUSPEND
#define PM_AUTO_SUSPEND_TIMEOUT 20
#endif
static unsigned int debug_quirks = 0;

#ifdef HOT_PLUG_SUPPORTED	
static struct wake_lock sdhci_detect_lock;
#endif
static struct wake_lock sdhci_suspend_lock;

#ifdef MMC_HOST_WAKEUP_SUPPORTED
static struct wake_source sprd_host_wakeup;
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

static void sdhci_prepare_data(struct sdhci_host *, struct mmc_data *);
static void sdhci_finish_data(struct sdhci_host *);

static void sdhci_send_command(struct sdhci_host *, struct mmc_command *);
static void sdhci_finish_command(struct sdhci_host *);
static void sdhci_deselect_card(struct sdhci_host *host);
extern void mmc_power_off(struct mmc_host* mmc);
static void sdhci_auto_suspend_host(unsigned long data);

static void sdhci_dumpregs(struct sdhci_host *host)
{
	printk(KERN_DEBUG DRIVER_NAME ": ============== REGISTER DUMP ==============\n");

	printk(KERN_DEBUG DRIVER_NAME ": Sys addr: 0x%08x | Version:  0x%08x\n",
		sdhci_readl(host, SDHCI_DMA_ADDRESS),
		sdhci_readw(host, SDHCI_HOST_VERSION));
	printk(KERN_DEBUG DRIVER_NAME ": Blk size: 0x%08x | Blk cnt:  0x%08x\n",
		sdhci_readw(host, SDHCI_BLOCK_SIZE),
		sdhci_readw(host, SDHCI_BLOCK_COUNT));
	printk(KERN_DEBUG DRIVER_NAME ": Argument: 0x%08x | Trn mode: 0x%08x\n",
		sdhci_readl(host, SDHCI_ARGUMENT),
		sdhci_readw(host, SDHCI_TRANSFER_MODE));
	printk(KERN_DEBUG DRIVER_NAME ": Present:  0x%08x | Host ctl: 0x%08x\n",
		sdhci_readl(host, SDHCI_PRESENT_STATE),
		sdhci_readb(host, SDHCI_HOST_CONTROL));
	printk(KERN_DEBUG DRIVER_NAME ": Power:    0x%08x | Blk gap:  0x%08x\n",
		sdhci_readb(host, SDHCI_POWER_CONTROL),
		sdhci_readb(host, SDHCI_BLOCK_GAP_CONTROL));
	printk(KERN_DEBUG DRIVER_NAME ": Wake-up:  0x%08x | Clock:    0x%08x\n",
		sdhci_readb(host, SDHCI_WAKE_UP_CONTROL),
		sdhci_readw(host, SDHCI_CLOCK_CONTROL));
	printk(KERN_DEBUG DRIVER_NAME ": Timeout:  0x%08x | Int stat: 0x%08x\n",
		sdhci_readb(host, SDHCI_TIMEOUT_CONTROL),
		sdhci_readl(host, SDHCI_INT_STATUS));
	printk(KERN_DEBUG DRIVER_NAME ": Int enab: 0x%08x | Sig enab: 0x%08x\n",
		sdhci_readl(host, SDHCI_INT_ENABLE),
		sdhci_readl(host, SDHCI_SIGNAL_ENABLE));
	printk(KERN_DEBUG DRIVER_NAME ": AC12 err: 0x%08x | Slot int: 0x%08x\n",
		sdhci_readw(host, SDHCI_ACMD12_ERR),
		sdhci_readw(host, SDHCI_SLOT_INT_STATUS));
	printk(KERN_DEBUG DRIVER_NAME ": Caps:     0x%08x | Max curr: 0x%08x\n",
		sdhci_readl(host, SDHCI_CAPABILITIES),
		sdhci_readl(host, SDHCI_MAX_CURRENT));

	if (host->flags & SDHCI_USE_ADMA)
		printk(KERN_DEBUG DRIVER_NAME ": ADMA Err: 0x%08x | ADMA Ptr: 0x%08x\n",
		       readl(host->ioaddr + SDHCI_ADMA_ERROR),
		       readl(host->ioaddr + SDHCI_ADMA_ADDRESS));

	printk(KERN_DEBUG DRIVER_NAME ": ===========================================\n");
}


//we don't need to restore sdio0, because of CONFIG_MMC_BLOCK_DEFERED_RESUME=y
#ifdef MMC_RESTORE_REGS
static void sdhci_save_regs(struct sdhci_host *host){
    if (!strcmp("Spread SDIO host1", host->hw_name)){
			printk("%s, entry\n", __func__);
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

static void sdhci_restore_regs(struct sdhci_host *host){
    if (!strcmp("Spread SDIO host1", host->hw_name)){
			printk("%s, entry\n", __func__);
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

static void sdhci_dump_saved_regs(struct sdhci_host *host){
    if (!strcmp("Spread SDIO host1", host->hw_name)){
	printk("%s, entry\n", __func__);
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

/*****************************************************************************\
 *                                                                           *
 * Low level functions                                                       *
 *                                                                           *
\*****************************************************************************/

static void sdhci_clear_set_irqs(struct sdhci_host *host, u32 clear, u32 set)
{
	u32 ier;

	ier = sdhci_readl(host, SDHCI_INT_ENABLE);
	ier &= ~clear;
	ier |= set;
	sdhci_writel(host, ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, ier, SDHCI_SIGNAL_ENABLE);
}

static void sdhci_unmask_irqs(struct sdhci_host *host, u32 irqs)
{
	sdhci_clear_set_irqs(host, 0, irqs);
}

static void sdhci_mask_irqs(struct sdhci_host *host, u32 irqs)
{
	sdhci_clear_set_irqs(host, irqs, 0);
}

#ifdef HOT_PLUG_SUPPORTED
int sdcard_present(struct sdhci_host *host)
{
	int gpio;
	struct sprd_host_data *host_data;
	int irq;

        host_data = sdhci_priv(host);
        irq = host_data->detect_irq;
	gpio = irq_to_gpio(irq);

	if (gpio_get_value(gpio))
		return 0;
	else
		return 1;
}

static void sdhci_set_card_detection(struct sdhci_host *host, bool enable)
{
/*
	u32 irqs = SDHCI_INT_CARD_REMOVE | SDHCI_INT_CARD_INSERT;

	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)
		return;

	if (enable)
		sdhci_unmask_irqs(host, irqs);
	else
		sdhci_mask_irqs(host, irqs);
*/
	int irq;
	struct sprd_host_data *host_data;

	host_data = sdhci_priv(host);
	irq = host_data->detect_irq;
	if(!enable) {
		set_irq_type(irq,IRQF_TRIGGER_NONE);
		return;
	}
	if(sdcard_present(host))
		set_irq_type(irq,IRQF_TRIGGER_HIGH);
	 else
		set_irq_type(irq,IRQF_TRIGGER_LOW);
}

static void sdhci_enable_card_detection(struct sdhci_host *host)
{
	sdhci_set_card_detection(host, true);
}

static void sdhci_disable_card_detection(struct sdhci_host *host)
{
	sdhci_set_card_detection(host, false);
}
#endif

static void sdhci_reset(struct sdhci_host *host, u8 mask)
{
	unsigned long timeout;
	u32 uninitialized_var(ier);

	if (host->quirks & SDHCI_QUIRK_NO_CARD_NO_RESET) {
		if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) &
			SDHCI_CARD_PRESENT))
			return;
	}

	if (host->quirks & SDHCI_QUIRK_RESTORE_IRQS_AFTER_RESET)
		ier = sdhci_readl(host, SDHCI_INT_ENABLE);

	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);

	if (mask & SDHCI_RESET_ALL)
		host->clock = 0;

	/* Wait max 100 ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Reset 0x%x never completed.\n",
				mmc_hostname(host->mmc), (int)mask);
			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		mdelay(1);
	}
	
	if (host->quirks & SDHCI_QUIRK_RESTORE_IRQS_AFTER_RESET)
		sdhci_clear_set_irqs(host, SDHCI_INT_ALL_MASK, ier);
}

static void sdhci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios);

static void sdhci_init(struct sdhci_host *host, int soft)
{
	if (soft)
		sdhci_reset(host, SDHCI_RESET_CMD|SDHCI_RESET_DATA);
	else
		sdhci_reset(host, SDHCI_RESET_ALL);

	sdhci_clear_set_irqs(host, SDHCI_INT_ALL_MASK,
		SDHCI_INT_BUS_POWER | SDHCI_INT_DATA_END_BIT |
		SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
		SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
		SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE);

	if (soft) {
		/* force clock reconfiguration */
		host->clock = 0;
		sdhci_set_ios(host->mmc, &host->mmc->ios);
	}
}

static void sdhci_reinit(struct sdhci_host *host)
{
	sdhci_init(host, 0);
#ifdef HOT_PLUG_SUPPORTED	
	sdhci_enable_card_detection(host);
#endif
}

static void sdhci_activate_led(struct sdhci_host *host)
{
	u8 ctrl;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	ctrl |= SDHCI_CTRL_LED;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static void sdhci_deactivate_led(struct sdhci_host *host)
{
	u8 ctrl;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	ctrl &= ~SDHCI_CTRL_LED;
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

#ifdef SDHCI_USE_LEDS_CLASS
static void sdhci_led_control(struct led_classdev *led,
	enum led_brightness brightness)
{
	struct sdhci_host *host = container_of(led, struct sdhci_host, led);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	if (brightness == LED_OFF)
		sdhci_deactivate_led(host);
	else
		sdhci_activate_led(host);

	spin_unlock_irqrestore(&host->lock, flags);
}
#endif


#ifdef MMC_HOST_WAKEUP_SUPPORTED
static irqreturn_t sdhci_wakeup_irq_handler(int irq, void *dev){
    printk("sdhci_wakeup_irq_handler\n");
    /* Disable interrupt before calling handler */
     disable_irq_nosync(irq);
	 
	 return IRQ_HANDLED;

}



void sdhci_set_data1_to_gpio22(struct sdhci_host *host){
     
	 static unsigned long sdiod1_gpio_func_cfg[] = {
		MFP_CFG_X(SD2_D1,	 AF3,	 DS1,	 F_PULL_UP,  S_PULL_UP,  IO_IE), //gpio, input 
	 };
	 sprd_mfp_config(sdiod1_gpio_func_cfg, ARRAY_SIZE(sdiod1_gpio_func_cfg));
	 
	 printk("%s, PIN_SD2_D1_REG:0x%x\n", __func__, __raw_readl(PIN_SD2_D1_REG));
	 printk("sdhci_set_data1_to_gpio22 done\n");	 
     
		
}

void sdhci_set_gpio22_to_data1(struct sdhci_host *host){
	static unsigned long gpio_sdiod1_func_cfg[] = {
		MFP_CFG_X(SD2_D1,	AF0,	DS1,	F_PULL_UP,	S_PULL_UP,	IO_Z), //data[1] 
	};

	 sprd_mfp_config(gpio_sdiod1_func_cfg, ARRAY_SIZE(gpio_sdiod1_func_cfg));

	 printk("%s, PIN_SD2_D1_REG:0x%x\n", __func__, __raw_readl(PIN_SD2_D1_REG));	 
	 printk("sdhci_set_gpio22_to_data1 done\n");	 

}


static void  sdhci_host_wakeup_set( struct wake_source *src ){
    
	printk("%s\n", __func__);
#ifdef MMC_HOST_WAKEUP_SUPPORTED	
    struct  sdhci_host *host;
    host = (struct  sdhci_host *)(src->param);

	if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
	   
	    __raw_bits_or(BIT_6,SPRD_GPIO_BASE+0x18); //gpio22 irq enable
	
	    printk("set: SPRD_GPIO_BASE+0x18:0x%x\n", __raw_readl(SPRD_GPIO_BASE+0x18));
    }
#endif
    return;
}

static void  sdhci_host_wakeup_clear(struct wake_source *src){
	
	printk("%s\n", __func__);
#ifdef MMC_HOST_WAKEUP_SUPPORTED	
    struct  sdhci_host *host;
    host = (struct  sdhci_host *)(src->param);

    
	if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
	   
       __raw_bits_and(~BIT_6,SPRD_GPIO_BASE+0x18); //gpio22 irq disable
 
       printk("clr: SPRD_GPIO_BASE+0x18:0x%x\n", __raw_readl(SPRD_GPIO_BASE+0x18));
   }

#endif
   return;
}
#endif


#ifdef SDHCI_BUS_SCAN
//force card detection
 void sdhci_bus_scan(void){
    if(sdhci_host_g && (sdhci_host_g->mmc)){
	   printk("%s, entry\n", __func__);
	   if (sdhci_host_g->ops->set_clock) {
		 sdhci_host_g->ops->set_clock(sdhci_host_g, 1);
	   }

	   //sdhci_reset(sdhci_host_g, SDHCI_RESET_ALL);
	   sdhci_reinit(sdhci_host_g);
	   mmc_detect_change(sdhci_host_g->mmc, msecs_to_jiffies(200));
	}
	
	return;
}
EXPORT_SYMBOL_GPL(sdhci_bus_scan);
#endif


/*****************************************************************************\
 *                                                                           *
 * Core functions                                                            *
 *                                                                           *
\*****************************************************************************/

static void sdhci_read_block_pio(struct sdhci_host *host)
{
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 uninitialized_var(scratch);
	u8 *buf;

	DBG("PIO reading\n");

	blksize = host->data->blksz;
	chunk = 0;

	local_irq_save(flags);

	while (blksize) {
		if (!sg_miter_next(&host->sg_miter))
			BUG();

		len = min(host->sg_miter.length, blksize);

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = host->sg_miter.addr;

		while (len) {
			if (chunk == 0) {
				scratch = sdhci_readl(host, SDHCI_BUFFER);
				chunk = 4;
			}

			*buf = scratch & 0xFF;

			buf++;
			scratch >>= 8;
			chunk--;
			len--;
		}
	}

	sg_miter_stop(&host->sg_miter);

	local_irq_restore(flags);
}

static void sdhci_write_block_pio(struct sdhci_host *host)
{
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 scratch;
	u8 *buf;

	DBG("PIO writing\n");

	blksize = host->data->blksz;
	chunk = 0;
	scratch = 0;

	local_irq_save(flags);

	while (blksize) {
		if (!sg_miter_next(&host->sg_miter))
			BUG();

		len = min(host->sg_miter.length, blksize);

		blksize -= len;
		host->sg_miter.consumed = len;

		buf = host->sg_miter.addr;

		while (len) {
			scratch |= (u32)*buf << (chunk * 8);

			buf++;
			chunk++;
			len--;

			if ((chunk == 4) || ((len == 0) && (blksize == 0))) {
				sdhci_writel(host, scratch, SDHCI_BUFFER);
				chunk = 0;
				scratch = 0;
			}
		}
	}

	sg_miter_stop(&host->sg_miter);

	local_irq_restore(flags);
}

static void sdhci_transfer_pio(struct sdhci_host *host)
{
	u32 mask;

	BUG_ON(!host->data);

	if (host->blocks == 0)
		return;

	if (host->data->flags & MMC_DATA_READ)
		mask = SDHCI_DATA_AVAILABLE;
	else
		mask = SDHCI_SPACE_AVAILABLE;

	/*
	 * Some controllers (JMicron JMB38x) mess up the buffer bits
	 * for transfers < 4 bytes. As long as it is just one block,
	 * we can ignore the bits.
	 */
	if ((host->quirks & SDHCI_QUIRK_BROKEN_SMALL_PIO) &&
		(host->data->blocks == 1))
		mask = ~0;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (host->quirks & SDHCI_QUIRK_PIO_NEEDS_DELAY)
			udelay(100);

		if (host->data->flags & MMC_DATA_READ)
			sdhci_read_block_pio(host);
		else
			sdhci_write_block_pio(host);

		host->blocks--;
		if (host->blocks == 0)
			break;
	}

	DBG("PIO transfer complete.\n");
}

static char *sdhci_kmap_atomic(struct scatterlist *sg, unsigned long *flags)
{
	local_irq_save(*flags);
	return kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
}

static void sdhci_kunmap_atomic(void *buffer, unsigned long *flags)
{
	kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
	local_irq_restore(*flags);
}

static int sdhci_adma_table_pre(struct sdhci_host *host,
	struct mmc_data *data)
{
	int direction;

	u8 *desc;
	u8 *align;
	dma_addr_t addr;
	dma_addr_t align_addr;
	int len, offset;

	struct scatterlist *sg;
	int i;
	char *buffer;
	unsigned long flags;

	/*
	 * The spec does not specify endianness of descriptor table.
	 * We currently guess that it is LE.
	 */

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	/*
	 * The ADMA descriptor table is mapped further down as we
	 * need to fill it with data first.
	 */

	host->align_addr = dma_map_single(mmc_dev(host->mmc),
		host->align_buffer, 128 * 4, direction);
	if (dma_mapping_error(mmc_dev(host->mmc), host->align_addr))
		goto fail;
	BUG_ON(host->align_addr & 0x3);

	host->sg_count = dma_map_sg(mmc_dev(host->mmc),
		data->sg, data->sg_len, direction);
	if (host->sg_count == 0)
		goto unmap_align;

	desc = host->adma_desc;
	align = host->align_buffer;

	align_addr = host->align_addr;

	for_each_sg(data->sg, sg, host->sg_count, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);

		/*
		 * The SDHCI specification states that ADMA
		 * addresses must be 32-bit aligned. If they
		 * aren't, then we use a bounce buffer for
		 * the (up to three) bytes that screw up the
		 * alignment.
		 */
		offset = (4 - (addr & 0x3)) & 0x3;
		if (offset) {
			if (data->flags & MMC_DATA_WRITE) {
				buffer = sdhci_kmap_atomic(sg, &flags);
				WARN_ON(((long)buffer & PAGE_MASK) > (PAGE_SIZE - 3));
				memcpy(align, buffer, offset);
				sdhci_kunmap_atomic(buffer, &flags);
			}

			desc[7] = (align_addr >> 24) & 0xff;
			desc[6] = (align_addr >> 16) & 0xff;
			desc[5] = (align_addr >> 8) & 0xff;
			desc[4] = (align_addr >> 0) & 0xff;

			BUG_ON(offset > 65536);

			desc[3] = (offset >> 8) & 0xff;
			desc[2] = (offset >> 0) & 0xff;

			desc[1] = 0x00;
			desc[0] = 0x21; /* tran, valid */

			align += 4;
			align_addr += 4;

			desc += 8;

			addr += offset;
			len -= offset;
		}

		desc[7] = (addr >> 24) & 0xff;
		desc[6] = (addr >> 16) & 0xff;
		desc[5] = (addr >> 8) & 0xff;
		desc[4] = (addr >> 0) & 0xff;

		BUG_ON(len > 65536);

		desc[3] = (len >> 8) & 0xff;
		desc[2] = (len >> 0) & 0xff;

		desc[1] = 0x00;
		desc[0] = 0x21; /* tran, valid */

		desc += 8;

		/*
		 * If this triggers then we have a calculation bug
		 * somewhere. :/
		 */
		WARN_ON((desc - host->adma_desc) > (128 * 2 + 1) * 4);
	}

	/*
	 * Add a terminating entry.
	 */
	desc[7] = 0;
	desc[6] = 0;
	desc[5] = 0;
	desc[4] = 0;

	desc[3] = 0;
	desc[2] = 0;

	desc[1] = 0x00;
	desc[0] = 0x03; /* nop, end, valid */

	/*
	 * Resync align buffer as we might have changed it.
	 */
	if (data->flags & MMC_DATA_WRITE) {
		dma_sync_single_for_device(mmc_dev(host->mmc),
			host->align_addr, 128 * 4, direction);
	}

	host->adma_addr = dma_map_single(mmc_dev(host->mmc),
		host->adma_desc, (128 * 2 + 1) * 4, DMA_TO_DEVICE);
	if (dma_mapping_error(mmc_dev(host->mmc), host->adma_addr))
		goto unmap_entries;
	BUG_ON(host->adma_addr & 0x3);

	return 0;

unmap_entries:
	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
		data->sg_len, direction);
unmap_align:
	dma_unmap_single(mmc_dev(host->mmc), host->align_addr,
		128 * 4, direction);
fail:
	return -EINVAL;
}

static void sdhci_adma_table_post(struct sdhci_host *host,
	struct mmc_data *data)
{
	int direction;

	struct scatterlist *sg;
	int i, size;
	u8 *align;
	char *buffer;
	unsigned long flags;

	if (data->flags & MMC_DATA_READ)
		direction = DMA_FROM_DEVICE;
	else
		direction = DMA_TO_DEVICE;

	dma_unmap_single(mmc_dev(host->mmc), host->adma_addr,
		(128 * 2 + 1) * 4, DMA_TO_DEVICE);

	dma_unmap_single(mmc_dev(host->mmc), host->align_addr,
		128 * 4, direction);

	if (data->flags & MMC_DATA_READ) {
		dma_sync_sg_for_cpu(mmc_dev(host->mmc), data->sg,
			data->sg_len, direction);

		align = host->align_buffer;

		for_each_sg(data->sg, sg, host->sg_count, i) {
			if (sg_dma_address(sg) & 0x3) {
				size = 4 - (sg_dma_address(sg) & 0x3);

				buffer = sdhci_kmap_atomic(sg, &flags);
				WARN_ON(((long)buffer & PAGE_MASK) > (PAGE_SIZE - 3));
				memcpy(buffer, align, size);
				sdhci_kunmap_atomic(buffer, &flags);

				align += 4;
			}
		}
	}

	dma_unmap_sg(mmc_dev(host->mmc), data->sg,
		data->sg_len, direction);
}

static u8 sdhci_calc_timeout(struct sdhci_host *host, struct mmc_data *data)
{
	u8 count;
	unsigned target_timeout, current_timeout;

	/*
	 * If the host controller provides us with an incorrect timeout
	 * value, just skip the check and use 0xE.  The hardware may take
	 * longer to time out, but that's much better than having a too-short
	 * timeout value.
	 */
	if (host->quirks & SDHCI_QUIRK_BROKEN_TIMEOUT_VAL)
		return 0xE;

	/* timeout in us */
	target_timeout = data->timeout_ns / 1000 +
		data->timeout_clks / host->clock;

	if (host->quirks & SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK)
		host->timeout_clk = host->clock / 1000;

	/*
	 * Figure out needed cycles.
	 * We do this in steps in order to fit inside a 32 bit int.
	 * The first step is the minimum timeout, which will have a
	 * minimum resolution of 6 bits:
	 * (1) 2^13*1000 > 2^22,
	 * (2) host->timeout_clk < 2^16
	 *     =>
	 *     (1) / (2) > 2^6
	 */
	count = 0;
	current_timeout = (1 << 13) * 1000 / host->timeout_clk;
	while (current_timeout < target_timeout) {
		count++;
		current_timeout <<= 1;
		if (count >= 0xF)
			break;
	}

	if (count >= 0xF) {
		printk(KERN_WARNING "%s: Too large timeout requested!\n",
			mmc_hostname(host->mmc));
		count = 0xE;
	}

	return count;
}

static void sdhci_set_transfer_irqs(struct sdhci_host *host)
{
	u32 pio_irqs = SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL;
	u32 dma_irqs = SDHCI_INT_DMA_END | SDHCI_INT_ADMA_ERROR;

	if (host->flags & SDHCI_REQ_USE_DMA)
		sdhci_clear_set_irqs(host, pio_irqs, dma_irqs);
	else
		sdhci_clear_set_irqs(host, dma_irqs, pio_irqs);
}

static void sdhci_prepare_data(struct sdhci_host *host, struct mmc_data *data)
{
	u8 count;
	u8 ctrl;
	int ret;

	WARN_ON(host->data);

	if (data == NULL)
		return;

	/* Sanity checks */
	BUG_ON(data->blksz * data->blocks > 524288);
	BUG_ON(data->blksz > host->mmc->max_blk_size);
	BUG_ON(data->blocks > 65535);
	host->data = data;
	host->data_early = 0;

	count = sdhci_calc_timeout(host, data);
	sdhci_writeb(host, count, SDHCI_TIMEOUT_CONTROL);

	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA))
		host->flags |= SDHCI_REQ_USE_DMA;

	/*
	 * FIXME: This doesn't account for merging when mapping the
	 * scatterlist.
	 */
	if (host->flags & SDHCI_REQ_USE_DMA) {
		int broken, i;
		struct scatterlist *sg;

		broken = 0;
		if (host->flags & SDHCI_USE_ADMA) {
			if (host->quirks & SDHCI_QUIRK_32BIT_ADMA_SIZE)
				broken = 1;
		} else {
			if (host->quirks & SDHCI_QUIRK_32BIT_DMA_SIZE)
				broken = 1;
		}

		if (unlikely(broken)) {
			for_each_sg(data->sg, sg, data->sg_len, i) {
				if (sg->length & 0x3) {
					DBG("Reverting to PIO because of "
						"transfer size (%d)\n",
						sg->length);
					host->flags &= ~SDHCI_REQ_USE_DMA;
					break;
				}
			}
		}
	}

	/*
	 * The assumption here being that alignment is the same after
	 * translation to device address space.
	 */
	if (host->flags & SDHCI_REQ_USE_DMA) {
		int broken, i;
		struct scatterlist *sg;

		broken = 0;
		if (host->flags & SDHCI_USE_ADMA) {
			/*
			 * As we use 3 byte chunks to work around
			 * alignment problems, we need to check this
			 * quirk.
			 */
			if (host->quirks & SDHCI_QUIRK_32BIT_ADMA_SIZE)
				broken = 1;
		} else {
			if (host->quirks & SDHCI_QUIRK_32BIT_DMA_ADDR)
				broken = 1;
		}

		if (unlikely(broken)) {
			for_each_sg(data->sg, sg, data->sg_len, i) {
				if (sg->offset & 0x3) {
					DBG("Reverting to PIO because of "
						"bad alignment\n");
					host->flags &= ~SDHCI_REQ_USE_DMA;
					break;
				}
			}
		}
	}

	if (host->flags & SDHCI_REQ_USE_DMA) {
		if (host->flags & SDHCI_USE_ADMA) {
			ret = sdhci_adma_table_pre(host, data);
			if (ret) {
				/*
				 * This only happens when someone fed
				 * us an invalid request.
				 */
				WARN_ON(1);
				host->flags &= ~SDHCI_REQ_USE_DMA;
			} else {
				sdhci_writel(host, host->adma_addr,
					SDHCI_ADMA_ADDRESS);
			}
		} else {
			int sg_cnt;

			sg_cnt = dma_map_sg(mmc_dev(host->mmc),
					data->sg, data->sg_len,
					(data->flags & MMC_DATA_READ) ?
						DMA_FROM_DEVICE :
						DMA_TO_DEVICE);
			if (sg_cnt == 0) {
				/*
				 * This only happens when someone fed
				 * us an invalid request.
				 */
				WARN_ON(1);
				host->flags &= ~SDHCI_REQ_USE_DMA;
			} else {
				WARN_ON(sg_cnt != 1);
				sdhci_writel(host, sg_dma_address(data->sg),
					SDHCI_DMA_ADDRESS);
			}
		}
	}

	/*
	 * Always adjust the DMA selection as some controllers
	 * (e.g. JMicron) can't do PIO properly when the selection
	 * is ADMA.
	 */
	if (host->version >= SDHCI_SPEC_200) {
		ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
		ctrl &= ~SDHCI_CTRL_DMA_MASK;
		if ((host->flags & SDHCI_REQ_USE_DMA) &&
			(host->flags & SDHCI_USE_ADMA))
			ctrl |= SDHCI_CTRL_ADMA32;
		else
			ctrl |= SDHCI_CTRL_SDMA;
		sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	}

	if (!(host->flags & SDHCI_REQ_USE_DMA)) {
		int flags;

		flags = SG_MITER_ATOMIC;
		if (host->data->flags & MMC_DATA_READ)
			flags |= SG_MITER_TO_SG;
		else
			flags |= SG_MITER_FROM_SG;
		sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);
		host->blocks = data->blocks;
	}

	sdhci_set_transfer_irqs(host);

	/* We do not handle DMA boundaries, so set it to max (512 KiB) */
	sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, data->blksz), SDHCI_BLOCK_SIZE);
	sdhci_writew(host, data->blocks, SDHCI_BLOCK_COUNT);
}

static void sdhci_set_transfer_mode(struct sdhci_host *host,
	struct mmc_data *data)
{
	u16 mode;

	if (data == NULL)
		return;

	WARN_ON(!host->data);

	mode = SDHCI_TRNS_BLK_CNT_EN;
	if (data->blocks > 1)
		mode |= SDHCI_TRNS_MULTI;
	if (data->flags & MMC_DATA_READ)
		mode |= SDHCI_TRNS_READ;
	if (host->flags & SDHCI_REQ_USE_DMA)
		mode |= SDHCI_TRNS_DMA;

	sdhci_writew(host, mode, SDHCI_TRANSFER_MODE);
}

static void sdhci_finish_data(struct sdhci_host *host)
{
	struct mmc_data *data;

	BUG_ON(!host->data);

	data = host->data;
	host->data = NULL;

	if (host->flags & SDHCI_REQ_USE_DMA) {
		if (host->flags & SDHCI_USE_ADMA)
			sdhci_adma_table_post(host, data);
		else {
			dma_unmap_sg(mmc_dev(host->mmc), data->sg,
				data->sg_len, (data->flags & MMC_DATA_READ) ?
					DMA_FROM_DEVICE : DMA_TO_DEVICE);
		}
	}

	/*
	 * The specification states that the block count register must
	 * be updated, but it does not specify at what point in the
	 * data flow. That makes the register entirely useless to read
	 * back so we have to assume that nothing made it to the card
	 * in the event of an error.
	 */
	if (data->error)
		data->bytes_xfered = 0;
	else
		data->bytes_xfered = data->blksz * data->blocks;

	if (data->stop) {
		/*
		 * The controller needs a reset of internal state machines
		 * upon error conditions.
		 */
		if (data->error) {
			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);
		}

		sdhci_send_command(host, data->stop);
	} else
		tasklet_schedule(&host->finish_tasklet);
}

static void sdhci_send_command(struct sdhci_host *host, struct mmc_command *cmd)
{
	int flags;
	u32 mask;
	unsigned long timeout;

	WARN_ON(host->cmd);

	/* Wait max 10 ms */
	timeout = 10;

	mask = SDHCI_CMD_INHIBIT;
	if ((cmd->data != NULL) || (cmd->flags & MMC_RSP_BUSY))
		mask |= SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (host->mrq->data && (cmd == host->mrq->data->stop))
		mask &= ~SDHCI_DATA_INHIBIT;

	while (sdhci_readl(host, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Controller never released "
				"inhibit bit(s).\n", mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			cmd->error = -EIO;
			tasklet_schedule(&host->finish_tasklet);
			return;
		}
		timeout--;
		mdelay(1);
	}

        if(host->suspending){
	   mod_timer(&host->timer, jiffies + 2*HZ/10); 
        }else{
	   mod_timer(&host->timer, jiffies + 10 * HZ);
	}

	host->cmd = cmd;

	sdhci_prepare_data(host, cmd->data);

	sdhci_writel(host, cmd->arg, SDHCI_ARGUMENT);

	sdhci_set_transfer_mode(host, cmd->data);

	if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY)) {
		printk(KERN_ERR "%s: Unsupported response type!\n",
			mmc_hostname(host->mmc));
		cmd->error = -EINVAL;
		tasklet_schedule(&host->finish_tasklet);
		return;
	}

	if (!(cmd->flags & MMC_RSP_PRESENT))
		flags = SDHCI_CMD_RESP_NONE;
	else if (cmd->flags & MMC_RSP_136)
		flags = SDHCI_CMD_RESP_LONG;
	else if (cmd->flags & MMC_RSP_BUSY)
		flags = SDHCI_CMD_RESP_SHORT_BUSY;
	else
		flags = SDHCI_CMD_RESP_SHORT;

	if (cmd->flags & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->flags & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (cmd->data)
		flags |= SDHCI_CMD_DATA;
//	DBG("SDIO host send command:%d \n", cmd->opcode);
	DBG("%s send cmd:%d \n", mmc_hostname(host->mmc), cmd->opcode);
	sdhci_writew(host, SDHCI_MAKE_CMD(cmd->opcode, flags), SDHCI_COMMAND);
}

static void sdhci_finish_command(struct sdhci_host *host)
{
	int i;

	BUG_ON(host->cmd == NULL);

	if (host->cmd->flags & MMC_RSP_PRESENT) {
		if (host->cmd->flags & MMC_RSP_136) {
			/* CRC is stripped so we need to do some shifting. */
			for (i = 0;i < 4;i++) {
				host->cmd->resp[i] = sdhci_readl(host,
					SDHCI_RESPONSE + (3-i)*4) << 8;
				if (i != 3)
					host->cmd->resp[i] |=
						sdhci_readb(host,
						SDHCI_RESPONSE + (3-i)*4-1);
			}
		} else {
			host->cmd->resp[0] = sdhci_readl(host, SDHCI_RESPONSE);
		}
	}

	host->cmd->error = 0;

	if (host->data && host->data_early)
		sdhci_finish_data(host);

	if (!host->cmd->data)
		tasklet_schedule(&host->finish_tasklet);

	host->cmd = NULL;
}

static void sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	int div;
	u16 clk;
	unsigned long timeout;
	
	pr_debug("%s, %s, host->clock:%u, clock:%u\n", __func__, mmc_hostname(host->mmc), host->clock, clock);

	if (host->ops->set_clock) {
		host->ops->set_clock(host, clock);
		if (host->quirks & SDHCI_QUIRK_NONSTANDARD_CLOCK)
			return;
	}

        if (clock == host->clock)
		return;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
	if (clock == 0){
		goto out;
	}

	for (div = 1;div < 256;div *= 2) {
		if ((host->max_clk / div) <= clock)
			break;
	}
	div >>= 1;

	clk = div << SDHCI_DIVIDER_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Internal clock never "
				"stabilised.\n", mmc_hostname(host->mmc));
			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

out:
	host->clock = clock;
}

static void sdhci_set_power(struct sdhci_host *host, unsigned short power)
{
    pr_debug("%s, %s, power:%d\n", __func__, mmc_hostname(host->mmc), power);
	u8 pwr;
	int volt_level = LDO_VOLT_LEVEL1;
	
	if (power == (unsigned short)-1)
		pwr = 0;
	else {
		switch (1 << power) {
		case MMC_VDD_165_195:
			pwr = SDHCI_POWER_180;
			volt_level = LDO_VOLT_LEVEL3;
			break;
		case MMC_VDD_29_30:
		case MMC_VDD_30_31:
			pwr = SDHCI_POWER_300;
			volt_level = LDO_VOLT_LEVEL1;
			break;
			/*
		case MMC_VDD_32_33:
		case MMC_VDD_33_34:
			pwr = SDHCI_POWER_330;
			break;
			*/
		default:
			BUG();
		}
	}

	if (host->pwr == pwr)
		return;

	host->pwr = pwr;


	if (pwr == 0) {
#ifdef CONFIG_ARCH_SC8810
		if (!strcmp("Spread SDIO host0", host->hw_name)){
				LDO_TurnOffLDO(LDO_LDO_SDIO0);
		}
		else if (!strcmp("Spread SDIO host1", host->hw_name)){
				LDO_TurnOffLDO(LDO_LDO_SDIO1);
		}
#else
		LDO_TurnOffLDO(LDO_LDO_SDIO);
#endif
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
		return;
	}

	pwr |= SDHCI_POWER_ON;

#ifdef CONFIG_ARCH_SC8810
	if (!strcmp("Spread SDIO host0", host->hw_name)){
			LDO_SetVoltLevel (LDO_LDO_SDIO0, volt_level);
	}
	else if (!strcmp("Spread SDIO host1", host->hw_name)){
			LDO_SetVoltLevel (LDO_LDO_SDIO1, volt_level);
	}
	sdhci_writeb(host, SDHCI_POWER_ON, SDHCI_POWER_CONTROL);

	if (!strcmp("Spread SDIO host0", host->hw_name)){
			LDO_TurnOnLDO(LDO_LDO_SDIO0);
	}
	else if (!strcmp("Spread SDIO host1", host->hw_name)){
			LDO_TurnOnLDO(LDO_LDO_SDIO1);
	}
#else
	LDO_SetVoltLevel (LDO_LDO_SDIO, volt_level);
	sdhci_writeb(host, SDHCI_POWER_ON, SDHCI_POWER_CONTROL);
	LDO_TurnOnLDO(LDO_LDO_SDIO);
#endif
	/*
	 * Some controllers need an extra 10ms delay of 10ms before they
	 * can apply clock after applying power
	 */
	if (host->quirks & SDHCI_QUIRK_DELAY_AFTER_POWER)
		mdelay(10);
}

/*****************************************************************************\
 *                                                                           *
 * MMC callbacks                                                             *
 *                                                                           *
\*****************************************************************************/

static void sdhci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct sdhci_host *host;
	bool present;
	unsigned long flags;
	
	host = mmc_priv(mmc);
	wake_lock(&sdhci_suspend_lock);

	spin_lock_irqsave(&host->lock, flags);
//if card broken, send no commands,
	if( (mmc->card) && (mmc->card->removed) ){
	    printk("sdhci_request:: card was removed\n");
	    if(mrq->data){
	        mrq->data->error = -ETIMEDOUT;
	    }else if(mrq->cmd){
		mrq->cmd->error = -ETIMEDOUT;
	    }
	    host->mrq = NULL;
	    host->cmd = NULL;
	    host->data = NULL;
	    mmc_request_done(mmc, mrq);
#ifdef MMC_AUTO_SUSPEND
	    del_timer(&host->auto_suspend_timer);
#endif
	    wake_unlock(&sdhci_suspend_lock);
	    spin_unlock_irqrestore(&host->lock, flags);
	    return;
	}
//if card broken, send no commands, 
#ifdef MMC_AUTO_SUSPEND
	del_timer(&host->auto_suspend_timer);
#endif

	WARN_ON(host->mrq != NULL);

#ifdef SDHCI_USE_LEDS_CLASS
	sdhci_activate_led(host);
#endif

	host->mrq = mrq;

	/* If polling, assume that the card is always present. */
	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)
		present = true;
	else
		present = sdhci_readl(host, SDHCI_PRESENT_STATE) &
				SDHCI_CARD_PRESENT;

	if (!present || host->flags & SDHCI_DEVICE_DEAD) {
		host->mrq->cmd->error = -ENOMEDIUM;
		tasklet_schedule(&host->finish_tasklet);
	} else
		sdhci_send_command(host, mrq->cmd);

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct sdhci_host *host;
	unsigned long flags;
	u8 ctrl;

	host = mmc_priv(mmc);

	wake_lock(&sdhci_suspend_lock);

	spin_lock_irqsave(&host->lock, flags);

	if (host->flags & SDHCI_DEVICE_DEAD)
		goto out;

	/*
	 * Reset the chip on each power off.
	 * Should clear out any weird states.
	 */
//	if (ios->power_mode == MMC_POWER_OFF) {
//		sdhci_writel(host, 0, SDHCI_SIGNAL_ENABLE);
//		sdhci_reinit(host);
//	}
//      sdhci_set_clock(host, ios->clock);//ok case, original position
        
	if(ios->power_mode == MMC_POWER_UP){
		sdhci_reinit(host);
	}



	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;

	if (ios->timing == MMC_TIMING_SD_HS)
	        ctrl |= SDHCI_CTRL_HISPD;
	else
		ctrl &= ~SDHCI_CTRL_HISPD;

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	if (ios->power_mode == MMC_POWER_OFF){
		sdhci_set_power(host, -1);
		sdhci_reinit(host);
	}
	else if(ios->power_mode == MMC_POWER_UP)
		sdhci_set_power(host, ios->vdd);
	sdhci_set_clock(host, ios->clock);

	/*
	 * Some (ENE) controllers go apeshit on some ios operation,
	 * signalling timeout and CRC errors even on CMD0. Resetting
	 * it on each ios seems to solve the problem.
	 */
	if(host->quirks & SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS)
		sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

out:
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
        
	wake_unlock(&sdhci_suspend_lock);
}

static int sdhci_get_ro(struct mmc_host *mmc)
{
	struct sdhci_host *host;
	unsigned long flags;
	int present;

	host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	if (host->flags & SDHCI_DEVICE_DEAD)
		present = 0;
	else
		present = sdhci_readl(host, SDHCI_PRESENT_STATE);

	spin_unlock_irqrestore(&host->lock, flags);

	if (host->quirks & SDHCI_QUIRK_INVERTED_WRITE_PROTECT)
		return !!(present & SDHCI_WRITE_PROTECT);
	return !(present & SDHCI_WRITE_PROTECT);
}

static void sdhci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);

	if (host->flags & SDHCI_DEVICE_DEAD)
		goto out;

	if (enable)
		sdhci_unmask_irqs(host, SDHCI_INT_CARD_INT);
	else
		sdhci_mask_irqs(host, SDHCI_INT_CARD_INT);
out:
	mmiowb();

	spin_unlock_irqrestore(&host->lock, flags);
}

static int sdhci_auto_suspend(struct mmc_host* host_mmc){
	struct sdhci_host *host;
	host = mmc_priv(host_mmc);
	if(host_mmc->bus_resume_flags & MMC_BUSRESUME_NEEDS_RESUME){
	   return 0;
	}
	mod_timer(&host->auto_suspend_timer, jiffies + PM_AUTO_SUSPEND_TIMEOUT * HZ);
	return 0;
}
static const struct mmc_host_ops sdhci_ops = {
	.request	= sdhci_request,
	.set_ios	= sdhci_set_ios,
	.get_ro		= sdhci_get_ro,
	.enable_sdio_irq = sdhci_enable_sdio_irq,
	.auto_suspend   = sdhci_auto_suspend,
};

/*****************************************************************************\
 *                                                                           *
 * Tasklets                                                                  *
 *                                                                           *
\*****************************************************************************/

#ifdef HOT_PLUG_SUPPORTED	
static void sdhci_tasklet_card(unsigned long param)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = (struct sdhci_host*)param;

	spin_lock_irqsave(&host->lock, flags);

	//if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & SDHCI_CARD_PRESENT)) {
	if (!(sdcard_present(host))) {
		if (host->mrq) {
			printk(KERN_ERR "%s: Card removed during transfer!\n",
				mmc_hostname(host->mmc));
			printk(KERN_ERR "%s: Resetting controller.\n",
				mmc_hostname(host->mmc));

			sdhci_reset(host, SDHCI_RESET_CMD);
			sdhci_reset(host, SDHCI_RESET_DATA);

			host->mrq->cmd->error = -ENOMEDIUM;
			tasklet_schedule(&host->finish_tasklet);
		}
	}

	spin_unlock_irqrestore(&host->lock, flags);

	wake_lock_timeout(&sdhci_detect_lock, 5*HZ);

	mmc_detect_change(host->mmc, msecs_to_jiffies(200));
}
#endif

static void sdhci_tasklet_finish(unsigned long param)
{
	struct sdhci_host *host;
	unsigned long flags;
	struct mmc_request *mrq;

	host = (struct sdhci_host*)param;
	
	spin_lock_irqsave(&host->lock, flags);

	del_timer(&host->timer);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if(mrq){
	if (!(host->flags & SDHCI_DEVICE_DEAD) &&
		(mrq->cmd->error ||
		 (mrq->data && (mrq->data->error ||
		  (mrq->data->stop && mrq->data->stop->error))) ||
		   (host->quirks & SDHCI_QUIRK_RESET_AFTER_REQUEST))) {

		/* Some controllers need this kick or reset won't work here */
		if (host->quirks & SDHCI_QUIRK_CLOCK_BEFORE_RESET) {
			unsigned int clock;

			/* This is to force an update */
			clock = host->clock;
			host->clock = 0;
			sdhci_set_clock(host, clock);
		}

		/* Spec says we should do both at the same time, but Ricoh
		   controllers do not like that. */
		sdhci_reset(host, SDHCI_RESET_CMD);
		sdhci_reset(host, SDHCI_RESET_DATA);

	/*
	It's  a 8800G IC bug.
	if host issue a command error interrupt SDHCI_INT_TIMEOUT, after reset CMD,
	host will issue another interrupt SDHCI_INT_RESPONSE which should never come.
	so we delay a while and check the interrupt status, and remove it if we find it.

	so with the data error SDHCI_INT_DATA_TIMEOUT. After reset  DATA,  host will 
	issue SDHCI_INT_DATA_END, and we check  and remove it too.
	*/
#define WEIRD_TIMEOUT	10000 	//  10ms
		if (mrq->cmd && (mrq->cmd->error == -ETIMEDOUT)) {
			u32 count = WEIRD_TIMEOUT;
			while (!(sdhci_readl(host, SDHCI_INT_STATUS) & SDHCI_INT_RESPONSE)
				 && count){
				udelay(1);
				count--;
			}
		sdhci_writel(host, SDHCI_INT_RESPONSE, SDHCI_INT_STATUS);
		DBG("weired, after %dus, cmd complete irq come again\r\n",
			 WEIRD_TIMEOUT-count);
		}
		
		if (mrq->data && (mrq->data->error == -ETIMEDOUT)) {
			u32 count = WEIRD_TIMEOUT;
			while (!(sdhci_readl(host, SDHCI_INT_STATUS) & SDHCI_INT_DATA_END)
				 && count) {
				udelay(1);
				count--;
			}
		sdhci_writel(host, SDHCI_INT_DATA_END, SDHCI_INT_STATUS);
		DBG("weired, after reset %dus, data end irq come again\r\n",
			WEIRD_TIMEOUT-count);
		}
	}
        }
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

#ifdef SDHCI_USE_LEDS_CLASS
	sdhci_deactivate_led(host);
#endif

	sdhci_reset(host, SDHCI_RESET_CMD);
	sdhci_reset(host, SDHCI_RESET_DATA);
	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
	wake_unlock(&sdhci_suspend_lock);
}

static void sdhci_timeout_timer(unsigned long data)
{
	struct sdhci_host *host;
	unsigned long flags;

	host = (struct sdhci_host*)data;

	spin_lock_irqsave(&host->lock, flags);
//	if(host->mmc->card)
//           host->mmc->card->removed = 1;//from msm
        printk("!!!! %s: %s timeout !!!!\n", mmc_hostname(host->mmc), host->suspending?"supsend":"command");
	if (host->mrq) {
		printk(KERN_ERR "%s: Timeout waiting for hardware "
			"interrupt.\n", mmc_hostname(host->mmc));
		sdhci_dumpregs(host);

		if (host->data) {
			host->data->error = -ETIMEDOUT;
			sdhci_finish_data(host);
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;

			tasklet_schedule(&host->finish_tasklet);
		}
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}

void sdhci_deselect_card(struct sdhci_host *host){
       struct mmc_command cmd;
       memset(&cmd, 0, sizeof(struct mmc_command) );
       cmd.opcode = MMC_SELECT_CARD;
       cmd.arg = 0;
       cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;
       cmd.data = NULL;
       sdhci_writel(host, cmd.arg, SDHCI_ARGUMENT);
       sdhci_writew(host, SDHCI_MAKE_CMD(cmd.opcode, SDHCI_CMD_RESP_NONE), SDHCI_COMMAND);
}
#ifdef MMC_AUTO_SUSPEND
static void sdhci_auto_suspend_host(unsigned long data){
	struct sdhci_host *host;
	struct mmc_host *host_mmc;
	host = (struct sdhci_host*)data;
	host_mmc = host->mmc;
        printk("=== mmc: no requests in %ds, suspend host ===\n", PM_AUTO_SUSPEND_TIMEOUT);
	if(host_mmc->bus_resume_flags & MMC_BUSRESUME_NEEDS_RESUME){
	   printk("=== sdio host already been suspended ===\n");
	   return;
	}
        sdhci_deselect_card(host);
	if(host->mmc->card){
          host->mmc->card->state &= ~MMC_STATE_HIGHSPEED;
	}
	mmc_power_off(host_mmc);
	
        host_mmc->bus_resume_flags |= MMC_BUSRESUME_NEEDS_RESUME;
        del_timer(&host->auto_suspend_timer);
	printk("=== mmc: host auto-suspend done ===\n");
	return;
}
#endif
/*****************************************************************************\
 *                                                                           *
 * Interrupt handling                                                        *
 *                                                                           *
\*****************************************************************************/

static void sdhci_cmd_irq(struct sdhci_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (!host->cmd) {
		if(intmask != 1){
		printk(KERN_ERR "%s: Got command interrupt 0x%08x even "
			"though no command operation was in progress.\n",
			mmc_hostname(host->mmc), (unsigned)intmask);
		sdhci_dumpregs(host);
		}
		return;
	}

	if (intmask & SDHCI_INT_TIMEOUT)
		host->cmd->error = -ETIMEDOUT;
	else if (intmask & (SDHCI_INT_CRC | SDHCI_INT_END_BIT |
			SDHCI_INT_INDEX))
		host->cmd->error = -EILSEQ;

	if (host->cmd->error) {
	        if(host->mmc->card){
		  printk("%s: !!!!! error in sending cmd:%d, int:0x%x, err:%d \n", 
		  	     mmc_hostname(host->mmc), host->cmd->opcode, intmask, host->cmd->error);
		  //sdhci_dumpregs(host);
		}
		tasklet_schedule(&host->finish_tasklet);
		return;
	}

	/*
	 * The host can send and interrupt when the busy state has
	 * ended, allowing us to wait without wasting CPU cycles.
	 * Unfortunately this is overloaded on the "data complete"
	 * interrupt, so we need to take some care when handling
	 * it.
	 *
	 * Note: The 1.0 specification is a bit ambiguous about this
	 *       feature so there might be some problems with older
	 *       controllers.
	 */
	if (host->cmd->flags & MMC_RSP_BUSY) {
		if (host->cmd->data)
			DBG("Cannot wait for busy signal when also "
				"doing a data transfer");
		else if (!(host->quirks & SDHCI_QUIRK_NO_BUSY_IRQ))
			return;

		/* The controller does not support the end-of-busy IRQ,
		 * fall through and take the SDHCI_INT_RESPONSE */
	}

	if (intmask & SDHCI_INT_RESPONSE)
		sdhci_finish_command(host);
	
}

#ifdef DEBUG
static void sdhci_show_adma_error(struct sdhci_host *host)
{
	const char *name = mmc_hostname(host->mmc);
	u8 *desc = host->adma_desc;
	__le32 *dma;
	__le16 *len;
	u8 attr;

	sdhci_dumpregs(host);

	while (true) {
		dma = (__le32 *)(desc + 4);
		len = (__le16 *)(desc + 2);
		attr = *desc;

		DBG("%s: %p: DMA 0x%08x, LEN 0x%04x, Attr=0x%02x\n",
		    name, desc, le32_to_cpu(*dma), le16_to_cpu(*len), attr);

		desc += 8;

		if (attr & 2)
			break;
	}
}
#else
static void sdhci_show_adma_error(struct sdhci_host *host) { }
#endif

static void sdhci_data_irq(struct sdhci_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (!host->data) {
		/*
		 * The "data complete" interrupt is also used to
		 * indicate that a busy state has ended. See comment
		 * above in sdhci_cmd_irq().
		 */
		if (host->cmd && (host->cmd->flags & MMC_RSP_BUSY)) {
			if (intmask & SDHCI_INT_DATA_END) {
				sdhci_finish_command(host);
				return;
			}
		}

		printk("data irq, but no data request\n");
		//sdhci_dumpregs(host);

		return;
	}

	if (intmask & SDHCI_INT_DATA_TIMEOUT)
		host->data->error = -ETIMEDOUT;
	else if (intmask & (SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_END_BIT))
		host->data->error = -EILSEQ;
	else if (intmask & SDHCI_INT_ADMA_ERROR) {
		printk(KERN_ERR "%s: ADMA error\n", mmc_hostname(host->mmc));
		sdhci_show_adma_error(host);
		host->data->error = -EIO;
	}

	if (host->data->error){
		printk("%s: !!!!! error in sending data, int:0x%x\n", 
			   				mmc_hostname(host->mmc), intmask);
		sdhci_finish_data(host);
	}
	else {
		if (intmask & (SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL))
			sdhci_transfer_pio(host);

		/*
		 * We currently don't do anything fancy with DMA
		 * boundaries, but as we can't disable the feature
		 * we need to at least restart the transfer.
		 */
		if (intmask & SDHCI_INT_DMA_END)
			sdhci_writel(host, sdhci_readl(host, SDHCI_DMA_ADDRESS),
				SDHCI_DMA_ADDRESS);

		if (intmask & SDHCI_INT_DATA_END) {
			if (host->cmd) {
				/*
				 * Data managed to finish before the
				 * command completed. Make sure we do
				 * things in the proper order.
				 */
				host->data_early = 1;
			} else {
				sdhci_finish_data(host);
			}
		}
	}
}

static irqreturn_t sdhci_irq(int irq, void *dev_id)
{
	irqreturn_t result;
	struct sdhci_host* host = dev_id;
	u32 intmask;
	int cardint = 0;

	spin_lock(&host->lock);

	intmask = sdhci_readl(host, SDHCI_INT_STATUS);

	if (!intmask || intmask == 0xffffffff) {
		result = IRQ_NONE;
		goto out;
	}

	DBG("*** %s got interrupt: 0x%08x\n",
		mmc_hostname(host->mmc), intmask);

/*
	if (intmask & (SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE)) {
		sdhci_writel(host, intmask & (SDHCI_INT_CARD_INSERT |
			SDHCI_INT_CARD_REMOVE), SDHCI_INT_STATUS);
		tasklet_schedule(&host->card_tasklet);
	}
*/
	intmask &= ~(SDHCI_INT_CARD_INSERT | SDHCI_INT_CARD_REMOVE);

	if (intmask & SDHCI_INT_CMD_MASK) {
		sdhci_writel(host, intmask & SDHCI_INT_CMD_MASK,
			SDHCI_INT_STATUS);
		sdhci_cmd_irq(host, intmask & SDHCI_INT_CMD_MASK);
	}

	if (intmask & SDHCI_INT_DATA_MASK) {
		sdhci_writel(host, intmask & SDHCI_INT_DATA_MASK,
			SDHCI_INT_STATUS);
		sdhci_data_irq(host, intmask & SDHCI_INT_DATA_MASK);
	}

	intmask &= ~(SDHCI_INT_CMD_MASK | SDHCI_INT_DATA_MASK);

	intmask &= ~SDHCI_INT_ERROR;

	if (intmask & SDHCI_INT_BUS_POWER) {
		printk(KERN_ERR "%s: Card is consuming too much power!\n",
			mmc_hostname(host->mmc));
		sdhci_writel(host, SDHCI_INT_BUS_POWER, SDHCI_INT_STATUS);
	}

	intmask &= ~SDHCI_INT_BUS_POWER;

	if (intmask & SDHCI_INT_CARD_INT){
		cardint = 1;        
	}
	intmask &= ~SDHCI_INT_CARD_INT;

	if (intmask) {
		printk(KERN_ERR "%s: Unexpected interrupt 0x%08x.\n",
			mmc_hostname(host->mmc), intmask);
		sdhci_dumpregs(host);

		sdhci_writel(host, intmask, SDHCI_INT_STATUS);
	}

	result = IRQ_HANDLED;

	mmiowb();
out:
	spin_unlock(&host->lock);

	/*
	 * We have to delay this as it calls back into the driver.
	 */
	if (cardint)
		mmc_signal_sdio_irq(host->mmc);

	return result;
}

#ifdef HOT_PLUG_SUPPORTED	
static irqreturn_t sd_detect_irq(int irq, void *dev_id)
{
	struct sdhci_host* host = dev_id;
	/*
	deshaking for gpio stable
	*/
	msleep(200);

	if (sdcard_present(host))
		set_irq_type(irq,IRQF_TRIGGER_HIGH);
	else
		set_irq_type(irq,IRQF_TRIGGER_LOW);

	tasklet_schedule(&host->card_tasklet);
	return IRQ_HANDLED;
}
#endif
/*****************************************************************************\
 *                                                                           *
 * Suspend/resume                                                            *
 *                                                                           *
\*****************************************************************************/
#ifdef CONFIG_PM

int sdhci_suspend_host(struct sdhci_host *host, pm_message_t state)
{
   
   printk("%s, suspend_host, start\n", mmc_hostname(host->mmc));
	int ret;

#ifdef HOT_PLUG_SUPPORTED
	sdhci_disable_card_detection(host);
#endif

//#ifdef CONFIG_MMC_DEBUG    
	sdhci_dumpregs(host);
//#endif
        host->suspending = 1;//avoid dpm timeout
/*
    if(!mmc_bus_needs_resume(host->mmc)){
        if(!sdhci_readw(host, SDHCI_CLOCK_CONTROL)){
	   printk("!!! %s: no clock !!!\n", mmc_hostname(host->mmc));
           sdhci_set_clock(host, host->mmc->f_min);
        }
    }
*/	
	ret = mmc_suspend_host(host->mmc);
	if (ret){
		printk("=== wow~ %s suspend error:%d ===\n",mmc_hostname(host->mmc), ret);
		return ret;
    }
		
#ifdef MMC_HOST_WAKEUP_SUPPORTED
#if 0
	if( (host->mmc->card )			   &&
		mmc_card_sdio(host->mmc->card) && 
	   (host->mmc->pm_flags & MMC_PM_KEEP_POWER) ){
#endif

	if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){
	   sprd_host_wakeup.param = (void*)host;
	   sdhci_set_data1_to_gpio22(host);
	   gpio_request(HOST_WAKEUP_GPIO, "host_wakeup_irq");
	   gpio_direction_input(HOST_WAKEUP_GPIO);
	   sdio_wakeup_irq = sprd_alloc_gpio_irq(HOST_WAKEUP_GPIO);
	   request_threaded_irq(sdio_wakeup_irq, sdhci_wakeup_irq_handler, NULL, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "host_wakeup_irq", host);
    }
#endif

#ifdef MMC_RESTORE_REGS
#ifdef CONFIG_MMC_DEBUG    
	   sdhci_dumpregs(host);
#endif		

	   sdhci_save_regs(host);

#ifdef CONFIG_MMC_DEBUG 
       sdhci_dump_saved_regs(host);
#endif
#endif

	printk("%s, suspend_host, done\n", mmc_hostname(host->mmc));
	return 0;
}

EXPORT_SYMBOL_GPL(sdhci_suspend_host);

int sdhci_resume_host(struct sdhci_host *host)
{
    printk("%s sdhci_resume_host, start\n", mmc_hostname(host->mmc)); 
	int ret;
    host->suspending = 0;//clear indicator
#ifdef MMC_RESTORE_REGS

#ifdef CONFIG_MMC_DEBUG    
	sdhci_dumpregs(host);
#endif

    sdhci_restore_regs(host);
#endif

#ifdef MMC_HOST_WAKEUP_SUPPORTED    
#if 0
		if( (host->mmc->card )			   &&
			mmc_card_sdio(host->mmc->card) && 
		   (host->mmc->pm_flags & MMC_PM_KEEP_POWER) ){
#endif
	
	if( (host->mmc->card )&& mmc_card_sdio(host->mmc->card) ){

	   free_irq(sdio_wakeup_irq, host);	
	   gpio_free(HOST_WAKEUP_GPIO);	
	   sdhci_set_gpio22_to_data1(host);
    }
#endif	

	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA)) {
		if (host->ops->enable_dma)
			host->ops->enable_dma(host);
	}

	//ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED,
	//		  mmc_hostname(host->mmc), host);	
	//if (ret)
	//	return ret;

	sdhci_init(host, (host->mmc->pm_flags & MMC_PM_KEEP_POWER));
	mmiowb();
	
#ifdef CONFIG_MMC_DEBUG    
    sdhci_dumpregs(host);
#endif		

	ret = mmc_resume_host(host->mmc);
	if (ret){
		printk("=== %s resume error:%d ===\n", mmc_hostname(host->mmc), ret);
		return ret;
	}
#if 0      
	if(!(host->mmc->pm_flags & MMC_PM_KEEP_POWER)){ 
		if(!(host->mmc->card)){ 
			sdhci_set_power(host, -1);//power off ldo_sdio1
		}
	}
#endif     
     if(!(host->mmc->card)){ 
           sdhci_set_power(host, -1);//power off ldo_sdio1 if device is off
     }
     
#ifdef HOT_PLUG_SUPPORTED
	sdhci_enable_card_detection(host);
#endif	
        printk("%s sdhci_resume_host, done\n", mmc_hostname(host->mmc)); 
	return 0;
}

EXPORT_SYMBOL_GPL(sdhci_resume_host);

#endif /* CONFIG_PM */

/*****************************************************************************\
 *                                                                           *
 * Device allocation/registration                                            *
 *                                                                           *
\*****************************************************************************/

struct sdhci_host *sdhci_alloc_host(struct device *dev,
	size_t priv_size)
{
	struct mmc_host *mmc;
	struct sdhci_host *host;

	WARN_ON(dev == NULL);

	mmc = mmc_alloc_host(sizeof(struct sdhci_host) + priv_size, dev);
	if (!mmc)
		return ERR_PTR(-ENOMEM);

	host = mmc_priv(mmc);
	host->mmc = mmc;

	return host;
}

EXPORT_SYMBOL_GPL(sdhci_alloc_host);

int sdhci_add_host(struct sdhci_host *host)
{
	struct mmc_host *mmc;
	unsigned int caps;
	struct sprd_host_data *host_data;
#ifdef HOT_PLUG_SUPPORTED
	int detect_irq;
#endif
	int ret;

	WARN_ON(host == NULL);
	if (host == NULL)
		return -EINVAL;

	mmc = host->mmc;

	if (debug_quirks)
		host->quirks = debug_quirks;

	sdhci_reset(host, SDHCI_RESET_ALL);

	host->version = sdhci_readw(host, SDHCI_HOST_VERSION);
	host->version = (host->version & SDHCI_SPEC_VER_MASK)
				>> SDHCI_SPEC_VER_SHIFT;
	if (host->version > SDHCI_SPEC_200) {
		printk(KERN_ERR "%s: Unknown controller version (%d). "
			"You may experience problems.\n", mmc_hostname(mmc),
			host->version);
	}

	caps = sdhci_readl(host, SDHCI_CAPABILITIES);

	if (host->quirks & SDHCI_QUIRK_FORCE_DMA)
		host->flags |= SDHCI_USE_SDMA;
	else if (!(caps & SDHCI_CAN_DO_SDMA))
		DBG("Controller doesn't have SDMA capability\n");
	else
		host->flags |= SDHCI_USE_SDMA;

	if ((host->quirks & SDHCI_QUIRK_BROKEN_DMA) &&
		(host->flags & SDHCI_USE_SDMA)) {
		DBG("Disabling DMA as it is marked broken\n");
		host->flags &= ~SDHCI_USE_SDMA;
	}

	if ((host->version >= SDHCI_SPEC_200) && (caps & SDHCI_CAN_DO_ADMA2))
		host->flags |= SDHCI_USE_ADMA;

	if ((host->quirks & SDHCI_QUIRK_BROKEN_ADMA) &&
		(host->flags & SDHCI_USE_ADMA)) {
		DBG("Disabling ADMA as it is marked broken\n");
		host->flags &= ~SDHCI_USE_ADMA;
	}

	if (host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA)) {
		if (host->ops->enable_dma) {
			if (host->ops->enable_dma(host)) {
				printk(KERN_WARNING "%s: No suitable DMA "
					"available. Falling back to PIO.\n",
					mmc_hostname(mmc));
				host->flags &=
					~(SDHCI_USE_SDMA | SDHCI_USE_ADMA);
			}
		}
	}

	if (host->flags & SDHCI_USE_ADMA) {
		/*
		 * We need to allocate descriptors for all sg entries
		 * (128) and potentially one alignment transfer for
		 * each of those entries.
		 */
		host->adma_desc = kmalloc((128 * 2 + 1) * 4, GFP_KERNEL);
		host->align_buffer = kmalloc(128 * 4, GFP_KERNEL);
		if (!host->adma_desc || !host->align_buffer) {
			kfree(host->adma_desc);
			kfree(host->align_buffer);
			printk(KERN_WARNING "%s: Unable to allocate ADMA "
				"buffers. Falling back to standard DMA.\n",
				mmc_hostname(mmc));
			host->flags &= ~SDHCI_USE_ADMA;
		}
	}

	/*
	 * If we use DMA, then it's up to the caller to set the DMA
	 * mask, but PIO does not need the hw shim so we set a new
	 * mask here in that case.
	 */
	if (!(host->flags & (SDHCI_USE_SDMA | SDHCI_USE_ADMA))) {
		host->dma_mask = DMA_BIT_MASK(64);
		mmc_dev(host->mmc)->dma_mask = &host->dma_mask;
	}

	host->max_clk =
		(caps & SDHCI_CLOCK_BASE_MASK) >> SDHCI_CLOCK_BASE_SHIFT;
	host->max_clk *= 1000000;
	if (host->max_clk == 0) {
		if (!host->ops->get_max_clock) {
			printk(KERN_ERR
			       "%s: Hardware doesn't specify base clock "
			       "frequency.\n", mmc_hostname(mmc));
			return -ENODEV;
		}
		host->max_clk = host->ops->get_max_clock(host);
	}

	host->timeout_clk =
		(caps & SDHCI_TIMEOUT_CLK_MASK) >> SDHCI_TIMEOUT_CLK_SHIFT;
	if (host->timeout_clk == 0) {
		if (host->ops->get_timeout_clock) {
			host->timeout_clk = host->ops->get_timeout_clock(host);
		} else if (!(host->quirks &
				SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK)) {
			printk(KERN_ERR
			       "%s: Hardware doesn't specify timeout clock "
			       "frequency.\n", mmc_hostname(mmc));
			return -ENODEV;
		}
	}
	if (caps & SDHCI_TIMEOUT_CLK_UNIT)
		host->timeout_clk *= 1000;

	/*
	 * Set host parameters.
	 */
	mmc->ops = &sdhci_ops;
	if (host->quirks & SDHCI_QUIRK_NONSTANDARD_CLOCK &&
			host->ops->set_clock && host->ops->get_min_clock)
		mmc->f_min = host->ops->get_min_clock(host);
	else
		mmc->f_min = host->max_clk / 256;//375000Hz
	mmc->f_max = host->max_clk;
	
	mmc->caps = MMC_CAP_SDIO_IRQ;

	if (!(host->quirks & SDHCI_QUIRK_FORCE_1_BIT_DATA))
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if (caps & SDHCI_CAN_DO_HISPD)
		mmc->caps |= MMC_CAP_SD_HIGHSPEED;

	//it's a phone, we don't need polling;//sword
/*
	if (host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION)
		mmc->caps |= MMC_CAP_NEEDS_POLL;
*/
	mmc->ocr_avail = 0;
	if (caps & SDHCI_CAN_VDD_330)
		mmc->ocr_avail |= MMC_VDD_32_33|MMC_VDD_33_34;
	if (caps & SDHCI_CAN_VDD_300)
		mmc->ocr_avail |= MMC_VDD_29_30|MMC_VDD_30_31;
	if (caps & SDHCI_CAN_VDD_180)
		mmc->ocr_avail |= MMC_VDD_165_195;

	if (mmc->ocr_avail == 0) {
		printk(KERN_ERR "%s: Hardware doesn't report any "
			"support voltages.\n", mmc_hostname(mmc));
		return -ENODEV;
	}

	spin_lock_init(&host->lock);

	/*
	 * Maximum number of segments. Depends on if the hardware
	 * can do scatter/gather or not.
	 */
	if (host->flags & SDHCI_USE_ADMA)
		mmc->max_hw_segs = 128;
	else if (host->flags & SDHCI_USE_SDMA)
		mmc->max_hw_segs = 1;
	else /* PIO */
		mmc->max_hw_segs = 128;
	mmc->max_phys_segs = 128;

	/*
	 * Maximum number of sectors in one transfer. Limited by DMA boundary
	 * size (512KiB).
	 */
	mmc->max_req_size = 524288;

	/*
	 * Maximum segment size. Could be one segment with the maximum number
	 * of bytes. When doing hardware scatter/gather, each entry cannot
	 * be larger than 64 KiB though.
	 */
	if (host->flags & SDHCI_USE_ADMA)
		mmc->max_seg_size = 65536;
	else
		mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Maximum block size. This varies from controller to controller and
	 * is specified in the capabilities register.
	 */
	if (host->quirks & SDHCI_QUIRK_FORCE_BLK_SZ_2048) {
		mmc->max_blk_size = 2;
	} else {
		mmc->max_blk_size = (caps & SDHCI_MAX_BLOCK_MASK) >>
				SDHCI_MAX_BLOCK_SHIFT;
		if (mmc->max_blk_size >= 3) {
			printk(KERN_WARNING "%s: Invalid maximum block size, "
				"assuming 512 bytes\n", mmc_hostname(mmc));
			mmc->max_blk_size = 0;
		}
	}

	mmc->max_blk_size = 512 << mmc->max_blk_size;

	/*
	 * Maximum block count.
	 */
	mmc->max_blk_count = (host->quirks & SDHCI_QUIRK_NO_MULTIBLOCK) ? 1 : 65535;

	/*
	 * Init tasklets.
	 */
#ifdef HOT_PLUG_SUPPORTED
	tasklet_init(&host->card_tasklet,
		sdhci_tasklet_card, (unsigned long)host);
#endif
	tasklet_init(&host->finish_tasklet,
		sdhci_tasklet_finish, (unsigned long)host);

	setup_timer(&host->timer, sdhci_timeout_timer, (unsigned long)host);
#ifdef MMC_AUTO_SUSPEND	
	setup_timer(&host->auto_suspend_timer, sdhci_auto_suspend_host, (unsigned long)host);
#endif
	ret = request_irq(host->irq, sdhci_irq, IRQF_SHARED,
		mmc_hostname(mmc), host);
	if (ret)
		goto untasklet;

	host_data = sdhci_priv(host);
#ifdef HOT_PLUG_SUPPORTED	
	detect_irq = host_data->detect_irq;
	if (sdcard_present(host)){
		ret = request_threaded_irq(detect_irq, NULL, sd_detect_irq,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "sd card detect", host);
	} else {
		ret = request_threaded_irq(detect_irq, NULL, sd_detect_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "sd card detect", host);
	}
	if (ret)
		goto untasklet;
#endif
	sdhci_init(host, 0);

#ifdef CONFIG_MMC_DEBUG
	sdhci_dumpregs(host);
#endif

#ifdef SDHCI_USE_LEDS_CLASS
	snprintf(host->led_name, sizeof(host->led_name),
		"%s::", mmc_hostname(mmc));
	host->led.name = host->led_name;
	host->led.brightness = LED_OFF;
	host->led.default_trigger = mmc_hostname(mmc);
	host->led.brightness_set = sdhci_led_control;

	ret = led_classdev_register(mmc_dev(mmc), &host->led);
	if (ret)
		goto reset;
#endif

	mmiowb();
        
	mmc->pm_flags |= MMC_PM_IGNORE_PM_NOTIFY;

	mmc_add_host(mmc);

	printk(KERN_INFO "%s: SDHCI controller on %s [%s] using %s\n",
		mmc_hostname(mmc), host->hw_name, dev_name(mmc_dev(mmc)),
		(host->flags & SDHCI_USE_ADMA) ? "ADMA" :
		(host->flags & SDHCI_USE_SDMA) ? "DMA" : "PIO");

#ifdef HOT_PLUG_SUPPORTED
	sdhci_enable_card_detection(host);
#endif

#ifdef MMC_HOST_WAKEUP_SUPPORTED		   
	sprd_host_wakeup.param = (void*)host;
#endif

#ifdef SDHCI_BUS_SCAN
    sdhci_host_g = host;
#endif
        host->suspending = 0;//avoid dpm timeout
	return 0;

#ifdef SDHCI_USE_LEDS_CLASS
reset:
	sdhci_reset(host, SDHCI_RESET_ALL);
	free_irq(host->irq, host);
#ifdef HOT_PLUG_SUPPORTED
	free_irq(detect_irq, host);
#endif
#endif
untasklet:
#ifdef HOT_PLUG_SUPPORTED	
	tasklet_kill(&host->card_tasklet);
#endif
	tasklet_kill(&host->finish_tasklet);

	return ret;
}

EXPORT_SYMBOL_GPL(sdhci_add_host);

void sdhci_remove_host(struct sdhci_host *host, int dead)
{
	unsigned long flags;
	struct sprd_host_data *host_data;

	if (dead) {
		spin_lock_irqsave(&host->lock, flags);

		host->flags |= SDHCI_DEVICE_DEAD;

		if (host->mrq) {
			printk(KERN_ERR "%s: Controller removed during "
				" transfer!\n", mmc_hostname(host->mmc));

			host->mrq->cmd->error = -ENOMEDIUM;
			tasklet_schedule(&host->finish_tasklet);
		}

		spin_unlock_irqrestore(&host->lock, flags);
	}

#ifdef HOT_PLUG_SUPPORTED
	sdhci_disable_card_detection(host);
#endif
	mmc_remove_host(host->mmc);

#ifdef SDHCI_USE_LEDS_CLASS
	led_classdev_unregister(&host->led);
#endif

	if (!dead)
		sdhci_reset(host, SDHCI_RESET_ALL);

	free_irq(host->irq, host);

	host_data = sdhci_priv(host);
#ifdef HOT_PLUG_SUPPORTED
	free_irq(host_data->detect_irq, host);
#endif
	del_timer_sync(&host->timer);

#ifdef HOT_PLUG_SUPPORTED	
	tasklet_kill(&host->card_tasklet);
#endif	
	tasklet_kill(&host->finish_tasklet);

	kfree(host->adma_desc);
	kfree(host->align_buffer);

	host->adma_desc = NULL;
	host->align_buffer = NULL;
}

EXPORT_SYMBOL_GPL(sdhci_remove_host);

void sdhci_free_host(struct sdhci_host *host)
{
	mmc_free_host(host->mmc);
}

EXPORT_SYMBOL_GPL(sdhci_free_host);

/*****************************************************************************\
 *                                                                           *
 * Driver init/exit                                                          *
 *                                                                           *
\*****************************************************************************/

static int __init sdhci_drv_init(void)
{
	printk(KERN_INFO DRIVER_NAME
		": Secure Digital Host Controller Interface driver\n");
	printk(KERN_INFO DRIVER_NAME ": Copyright(c) Pierre Ossman\n");

#ifdef HOT_PLUG_SUPPORTED	
        wake_lock_init(&sdhci_detect_lock, WAKE_LOCK_SUSPEND, "mmc_pm_detect");
#endif
        wake_lock_init(&sdhci_suspend_lock, WAKE_LOCK_SUSPEND, "mmc_pm_suspend");

#ifdef MMC_HOST_WAKEUP_SUPPORTED
	sprd_host_wakeup.set = sdhci_host_wakeup_set;
    sprd_host_wakeup.clear = sdhci_host_wakeup_clear;
    register_wake_source(&sprd_host_wakeup);
#endif

	return 0;
}

static void __exit sdhci_drv_exit(void)
{ 
#ifdef HOT_PLUG_SUPPORTED	
        wake_lock_destroy(&sdhci_detect_lock);
#endif
        wake_lock_destroy(&sdhci_suspend_lock);

#ifdef MMC_HOST_WAKEUP_SUPPORTED
        unregister_wake_source(&sprd_host_wakeup);
#endif


}

module_init(sdhci_drv_init);
module_exit(sdhci_drv_exit);

module_param(debug_quirks, uint, 0444);

MODULE_AUTHOR("Pierre Ossman <pierre@ossman.eu>");
MODULE_DESCRIPTION("Secure Digital Host Controller Interface core driver");
MODULE_LICENSE("GPL");

MODULE_PARM_DESC(debug_quirks, "Force certain quirks.");
