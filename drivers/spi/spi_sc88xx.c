/*
 * Driver for SpreadTrum SC88XX Series SPI Controllers
 *
 * Copyright (C) 2010 SpreadTrum Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

#include <asm/io.h>
#include <mach/board.h>
#include <mach/gpio.h>

#include "spi_sc88xx.h"
#define SPRD_SPI_DEBUG  0
#define SPRD_SPI_CS_GPIO 1 /* you should also modify the spi_func_cfg[] */

// lock is held, spi irq is disabled
static int sprd_spi_do_transfer(struct sprd_spi_data *sprd_data)
{
    if (sprd_data->cspi_trans == 0) return -ENODATA;

    if (sprd_data->cspi_trans_num >= sprd_data->cspi_trans->len) {
        sprd_data->cspi_msg->actual_length += sprd_data->cspi_trans_num;
#if SPRD_SPI_DEBUG
        printk(KERN_WARNING "send ok\n");
#endif
        if (!sprd_data->cspi_msg->is_dma_mapped)
            sprd_spi_dma_unmap_transfer(sprd_data, sprd_data->cspi_trans);
        if (sprd_data->cspi_msg->transfers.prev != &sprd_data->cspi_trans->transfer_list) {
            grab_subsibling(sprd_data);
        } else {
            cs_deactivate(sprd_data, sprd_data->cspi); // all msg sibling data trans done
            list_del(&sprd_data->cspi_msg->queue);
            sprd_data->cspi_msg->status = 0; // msg tranfsered successfully
            spin_unlock(&sprd_data->lock);
            sprd_data->cspi_msg->complete(sprd_data->cspi_msg->context);
            spin_lock(&sprd_data->lock);
            sprd_data->cspi_trans = 0;
            if (list_empty(&sprd_data->queue)) {
                // no spi data on queue to transfer
                // cs_deactivate(sprd_data, sprd_data->cspi);
#if SPRD_SPI_DEBUG
                printk(KERN_WARNING "spi msg queue done.\n");
#endif
                return 0;
            }
            grab_sibling(sprd_data);
        }
    }
#if SPRD_SPI_DEBUG
    else {
        if (sprd_data->cspi_trans_num)
            printk(KERN_WARNING "Left data [%d]:[%d]\n", sprd_data->cspi_trans_num, sprd_data->cspi_trans->len);
    }
#endif

    if (sprd_data->cspi_trans == 0) return -ENOSR;

/*
    if (sprd_data->cspi != sprd_data->cspi_msg->spi) {
        cs_deactivate(sprd_data, sprd_data->cspi);
        cs_activate(sprd_data, sprd_data->cspi_msg->spi);
    }
*/
    cs_activate(sprd_data, sprd_data->cspi_msg->spi);
    sprd_data->cspi_trans_num += sprd_dma_update_spi(sprd_data->cspi_trans->tx_dma,
                                            sprd_data->cspi_trans->len,
                                            sprd_data->cspi_trans->rx_dma,
                                            sprd_data->cspi_trans->len,
                                            sprd_data->cspi,
                                            sprd_data);

    return 0;
}

static int sprd_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
    struct sprd_spi_data *sprd_data;
    struct spi_transfer *trans;
    unsigned long flags;
    int ret = 0;

    sprd_data = spi_master_get_devdata(spi->master);

	if (unlikely(list_empty(&msg->transfers)))
		return -EINVAL;

	if (sprd_data->stopping)
		return -ESHUTDOWN;

	list_for_each_entry(trans, &msg->transfers, transfer_list) {
		/* FIXME implement these protocol options!! */
		if (trans->bits_per_word || trans->speed_hz) {
			dev_dbg(&spi->dev, "no protocol options yet\n");
			return -ENOPROTOOPT;
		}

		if (!msg->is_dma_mapped) {
			if (sprd_spi_dma_map_transfer(sprd_data, trans) < 0)
				return -ENOMEM;
		}
#if SPRD_SPI_DEBUG
        else {
            printk(KERN_WARNING "spi dma msg\n");
        }
#endif

        if (!(trans->tx_dma || trans->rx_dma) || !trans->len)
            return -ENOMEM;
	}

	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

    spin_lock_irqsave(&sprd_data->lock, flags);
    list_add_tail(&msg->queue, &sprd_data->queue);
    if (sprd_data->cspi_trans == 0) {
        grab_sibling(sprd_data);
        ret = sprd_spi_do_transfer(sprd_data);
    }
    spin_unlock_irqrestore(&sprd_data->lock, flags);

    return ret;
}

#if SPRD_SPI_DMA_MODE
static void
#else
static irqreturn_t
#endif
sprd_spi_interrupt(int irq, void *dev_id)
{
    struct spi_master *master = dev_id;
    struct sprd_spi_data *sprd_data = spi_master_get_devdata(master);

    spin_lock(&sprd_data->lock);
#if SPRD_SPI_DEBUG
    // lprintf("spi irq [ %d ]\n", irq);
#endif
    if (sprd_spi_do_transfer(sprd_data) < 0)
        printk(KERN_ERR "error : %s\n", __func__);
    spin_unlock(&sprd_data->lock);

#if !SPRD_SPI_DMA_MODE
    return IRQ_HANDLED;
#endif
}

static inline void cs_activate(struct sprd_spi_data *sprd_data, struct spi_device *spi)
{
    if (sprd_data->cs_null) {
        struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;

        spi_writel(sprd_ctrl_data->spi_clkd, SPI_CLKD);
        spi_writel(sprd_ctrl_data->spi_ctl0, SPI_CTL0);
        spi_writel(sprd_ctrl_data->spi_ctl3, SPI_CTL3);

        __raw_bits_or((sprd_ctrl_data->clk_spi_and_div & 0xffff) << 21, GR_GEN2);
        __raw_bits_or((sprd_ctrl_data->clk_spi_and_div >> 16) << 26, GR_CLK_DLY);

#if SPRD_SPI_CS_GPIO
         __gpio_set_value(sprd_ctrl_data->cs_gpio, spi->mode & SPI_CS_HIGH);
#endif

        sprd_data->cspi = spi;

        sprd_data->cs_null = 0;
    }
}

static inline void cs_deactivate(struct sprd_spi_data *sprd_data, struct spi_device *spi)
{
#if SPRD_SPI_CS_GPIO
    if (spi) {
        struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
        u32 cs_gpio = sprd_ctrl_data->cs_gpio;
        __gpio_set_value(cs_gpio, !(spi->mode & SPI_CS_HIGH));
    }
#else
    spi_bits_or(0x0F << 8, SPI_CTL0);
#endif
    sprd_data->cs_null = 1;
}

/*
 * For DMA, tx_buf/tx_dma have the same relationship as rx_buf/rx_dma:
 *  - The buffer is either valid for CPU access, else NULL
 *  - If the buffer is valid, so is its DMA addresss
 *
 * This driver manages the dma addresss unless message->is_dma_mapped.
 */
static int sprd_spi_dma_map_transfer(struct sprd_spi_data *sprd_data, struct spi_transfer *trans)
{
    struct device *dev = &sprd_data->pdev->dev;

    trans->tx_dma = trans->rx_dma = 0;
    if (trans->tx_buf) {
        trans->tx_dma = dma_map_single(dev,
                (void *) trans->tx_buf, trans->len,
                DMA_TO_DEVICE);
        if (dma_mapping_error(dev, trans->tx_dma))
            return -ENOMEM;
    }
    if (trans->rx_buf) {
        trans->rx_dma = dma_map_single(dev,
                trans->rx_buf, trans->len,
                DMA_FROM_DEVICE);
        if (dma_mapping_error(dev, trans->rx_dma)) {
            if (trans->tx_buf)
                dma_unmap_single(dev,
                        trans->tx_dma, trans->len,
                        DMA_TO_DEVICE);
            return -ENOMEM;
        }
    }
    return 0;
}

static void sprd_spi_dma_unmap_transfer(struct sprd_spi_data *sprd_data, struct spi_transfer *trans)
{
    struct device *dev = &sprd_data->pdev->dev;

    if (trans->tx_dma)
        dma_unmap_single(dev, trans->tx_dma, trans->len, DMA_TO_DEVICE);

    if (trans->rx_dma)
        dma_unmap_single(dev, trans->rx_dma, trans->len, DMA_FROM_DEVICE);
}

static int sprd_spi_setup_dma(struct sprd_spi_data *sprd_data, struct spi_device *spi)
{
    int i, ch_id;
    u32 dsrc, ddst;
    int width;
    int autodma_src, autodma_dst;
    struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
    sprd_dma_ctrl ctrl;// = {.dma_desc = &sprd_ctrl_data->dma_desc};

    for (i = 0; i < 2; i++) {
        if (i == 0) ch_id = DMA_SPI_RX;
        else ch_id = DMA_SPI_TX;

        if (ch_id == DMA_SPI_RX) {
            ctrl.dma_desc = &sprd_ctrl_data->dma_desc_rx;
            autodma_src = DMA_NOCHANGE;
            autodma_dst = DMA_INCREASE;
        } else {
            ctrl.dma_desc = &sprd_ctrl_data->dma_desc_tx;
            autodma_src = DMA_INCREASE;
            autodma_dst = DMA_NOCHANGE;
        }

        width = (sprd_ctrl_data->spi_ctl0 >> 2) & 0x1f;
        if (width == 0) width = 32;

#if 0
        if (ch_id == DMA_SPI_RX) {
            dsrc = SPRD_SPI_PHYS + SPI_TXD;
            ddst = 0;
        } else {
            dsrc = 0;
            ddst = SPRD_SPI_PHYS + SPI_TXD;
        }
#endif

        dsrc = ddst = SPRD_SPI_PHYS + SPI_TXD;

        sprd_dma_setup_cfg(&ctrl,
                ch_id,
                DMA_NORMAL,
                TRANS_DONE_EN,
                autodma_src, autodma_dst,
                SRC_BURST_MODE_SINGLE, SRC_BURST_MODE_SINGLE,
                16, // 16 bytes DMA burst size
                width, width,
                dsrc, ddst, 0);
         sprd_dma_setup(&ctrl);
    }

    // 综合实验决定将SPI_CTL3的rx大小设置为1
    // SPI_CTL3的rx大小决定了DMA数据读取操作,如果非1,有时导致需要读取slave数据
    // 到一定个数之后才会满足DMA传输完毕的条件,才会触发dma irq[luther.ge]
//    spi_writel((width + 7) / 8/*0x0001*/, SPI_CTL3);
    sprd_ctrl_data->data_width = (width + 7) / 8;

    switch (sprd_ctrl_data->data_width) {
        case 1: sprd_ctrl_data->data_width_order = 0; break;
        case 2: sprd_ctrl_data->data_width_order = 1; break;
        case 4: sprd_ctrl_data->data_width_order = 2; break;
    }

    sprd_ctrl_data->spi_ctl3 = sprd_ctrl_data->data_width/*0x0001*/;
    sprd_ctrl_data->data_max = sprd_ctrl_data->data_width * SPRD_SPI_DMA_BLOCK_MAX;

    spi_write(SPI_CTL2,  6, 0x01, 0x01); // Enable SPI_DMA_EN

    return 0;
}

static inline int sprd_dma_update_spi(u32 sptr, u32 slen, u32 dptr, u32 dlen,
                        struct spi_device *spi, struct sprd_spi_data *sprd_data)
{
    struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
    sprd_dma_desc *dma_desc;
    int flag, len, offset;
    int blocks;

    offset = sprd_data->cspi_trans_num;

    if (unlikely(sptr && dptr))
        flag = 0xff;        // for both
    else if (likely(sptr))
        flag = 0x02;        // for tx only
    else {
        flag = 0x01;        // for rx only
    }

    if (flag & 0x01) {
        if (unlikely(flag & 0x02)) {
            // tx both
            len = dlen;
        } else {
            // only rx
            dlen -= offset;
            len = dlen > sprd_ctrl_data->data_max ? sprd_ctrl_data->data_max : dlen;
            if (likely(sprd_ctrl_data->data_width != 3)) {
                blocks = len >> sprd_ctrl_data->data_width_order;
            } else blocks = len / sprd_ctrl_data->data_width;
            // blocks = len / sprd_ctrl_data->data_width; // for 1byte 2bytes 3bytes 4bytes
        }
        dma_desc = &sprd_ctrl_data->dma_desc_rx;
        dma_desc->tlen = len;
        dma_desc->ddst = dptr + offset;

        sprd_dma_update(DMA_SPI_RX, dma_desc); // use const ch_id value to speed up code exec [luther.ge]
    }

    if (flag & 0x02) {
        len = slen;
        dma_desc = &sprd_ctrl_data->dma_desc_tx;
        dma_desc->tlen = len;
        dma_desc->dsrc = sptr + offset;

        sprd_dma_update(DMA_SPI_TX, dma_desc); // use const ch_id value to speed up code exec [luther.ge]
    }
#if SPRD_SPI_DEBUG
    printk(KERN_WARNING "sending[%02x][%d]...\n", flag, len);
#endif
    if (flag == 0xff)
        spi_start();
    else if (flag & 0x01)
        spi_start_rx(blocks);
    else spi_start_tx();

    return len;
}

static int sprd_spi_setup(struct spi_device *spi)
{
    struct sprd_spi_data *sprd_data;
    struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
    u8 bits = spi->bits_per_word;
    u32 clk_spi;
    u8  clk_spi_mode;
    u32 cs_gpio;
    u8 clk_spi_div;
    u32 spi_clkd;
    u32 spi_ctl0 = 0;
    int ret;

    sprd_data = spi_master_get_devdata(spi->master);

    if (sprd_data->stopping)
        return -ESHUTDOWN;
    if (spi->chip_select > spi->master->num_chipselect)
        return -EINVAL;
    if (bits > 32)
        return -EINVAL;
    if (bits == 32) bits = 0;

    clk_spi_mode = (__raw_readl(GR_CLK_DLY) >> 26) & 0x03;
    switch (clk_spi_mode) {
        case 0: clk_spi = 192   * 1000 * 1000; break;
        case 1: clk_spi = 153.6 * 1000 * 1000; break;
        case 2: clk_spi = 96    * 1000 * 1000; break;
        case 3: clk_spi = 26    * 1000 * 1000; break; 
    }
    clk_spi_div = (__raw_readl(GR_GEN2) >> 21) & 0x07;
    clk_spi /= (clk_spi_div + 1);

    // spi_clk = clk_spi / (2 * (spi_clkd + 1));
    if (spi->max_speed_hz) {
        spi_clkd = clk_spi / (2*spi->max_speed_hz) - 1;
    } else {
        spi_clkd = 0xffff;
    }
    if (spi_clkd < 0) spi_clkd = 0;

    /* chipselect must have been muxed as GPIO (e.g. in board setup) 
     * and we assume cs_gpio real gpio number not exceeding 0xffff
     */
    cs_gpio = (u32)spi->controller_data;
    if (cs_gpio < 0xffff) {
        sprd_ctrl_data = kzalloc(sizeof *sprd_ctrl_data, GFP_KERNEL);
        if (!sprd_ctrl_data)
            return -ENOMEM;

        ret = gpio_request(cs_gpio, dev_name(&spi->dev));
        if (ret) {
            printk(KERN_WARNING "%s[%s] cs_gpio %d is busy!", spi->modalias, dev_name(&spi->dev), cs_gpio);
            // kfree(sprd_ctrl_data);
            // return ret;
        }
        sprd_ctrl_data->cs_gpio = cs_gpio;
        spi->controller_data = sprd_ctrl_data;
        gpio_direction_output(cs_gpio, !(spi->mode & SPI_CS_HIGH));
    } else {
        unsigned long flags;
        spin_lock_irqsave(&sprd_data->lock, flags);
        if (sprd_data->cspi == spi)
            sprd_data->cspi = NULL;
        cs_deactivate(sprd_data, spi);
        spin_unlock_irqrestore(&sprd_data->lock, flags);
    }
    sprd_ctrl_data->clk_spi_and_div = clk_spi_div | (clk_spi_mode << 16);
    sprd_ctrl_data->spi_clkd = spi_clkd;

	if (spi->mode & SPI_CPHA)
         spi_ctl0 |= 0x01;
    else spi_ctl0 |= 0x02;

	if (spi->mode & SPI_CPOL)
        spi_ctl0 |= 1 << 13;
    else spi_ctl0 |= 0 << 13;

    spi_ctl0 |= bits << 2;
#if SPRD_SPI_CS_GPIO
    spi_ctl0 |= 0x0F << 8;
#else
    switch (spi->chip_select) {
        case 2:
        case 0: spi_ctl0 |= 0x0E << 8; break;
        case 3:
        case 1: spi_ctl0 |= 0x0D << 8; break;
        default: spi_ctl0 |= 0x0F << 8;break;
    }
#endif

    sprd_ctrl_data->spi_ctl0 = spi_ctl0;

    sprd_spi_setup_dma(sprd_data, spi);

#if SPRD_SPI_DEBUG
    lprintf("\n[%s]\n"
            "clk_spi_and_div = 0x%08x\n"
            "spi_clkd = 0x%08x\n"
            "spi_ctl0 = 0x%08x\n"
            "\n"
            ,spi->modalias
            ,sprd_ctrl_data->clk_spi_and_div
            ,sprd_ctrl_data->spi_clkd
            ,sprd_ctrl_data->spi_ctl0
            );
#if 0   //---------------------------
    {
        int i, data;
        cs_activate(sprd_data, spi);

        for (i = 0; i < 0xfffffff; i++) {
            // printk("[%d]=0x%08x\n", i, spi_readl(SPI_STS2));
            while (spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL);
            spi_writel(0x55, SPI_TXD);
            data = spi_readl(SPI_TXD) & 0xff;
            if (data != 0xff)
                printk("[in]0x%02x\n", data);
        }
    }
#elif 0 //---------------------------
    {
        sprd_dma_desc dma_desc;
        sprd_dma_ctrl ctrl = {.dma_desc = &dma_desc};
        int width;
        u32 tlen = 1; // SPRD_SPI_BUFFER_SIZE;
        // tlen should be width integer multiples, such as, 1,2,4bytes [luther.ge]
        // mode 4bits, 10bits, 25bits and so on, todo...
        u32 dsrc = sprd_data->buffer_dma;
        u32 ddst = SPRD_SPI_PHYS + SPI_TXD;
        u8 *data = sprd_data->buffer;;
        int i;
        memset(sprd_data->buffer, 0x15, SPRD_SPI_BUFFER_SIZE);

        width = (sprd_ctrl_data->spi_ctl0 >> 2) & 0x1f;
        if (width == 0) width = 32;

        printk("width=%d\n", width);

        sprd_dma_setup_cfg(&ctrl,
            DMA_SPI_TX,
            DMA_NORMAL,
            TRANS_DONE_EN,
            DMA_INCREASE, DMA_NOCHANGE,
            SRC_BURST_MODE_SINGLE, SRC_BURST_MODE_SINGLE,
            16, // 16 bytes DMA burst size
            width, width,
            0, ddst, 0);
        sprd_dma_setup(&ctrl);

        spi_write(SPI_CTL1, 12, 0x02, 0x03); // Only Enable SPI transmit mode
        spi_write(SPI_CTL2,  6, 0x01, 0x01); // Enable SPI_DMA_EN

        cs_activate(sprd_data, spi);

#if 1
        for (i = 0; i < 2; i++) {
#else
        for (i = 0; i < 0xfffffff; i++) {
#endif
            printk("enter\n");

#if 0
            data[1] = i & 0x01;
            data[0] = data[1] ? 0x0a:0xfa;
#endif
            sprd_dma_tlen(DMA_SPI_TX, tlen);
            sprd_dma_dsrc(DMA_SPI_TX, dsrc);

            sprd_dma_start(DMA_SPI_TX);
#if 0
            while (!(__raw_readl(DMA_INT_RAW) & (1 << DMA_SPI_TX)));
            __raw_writel(1 << DMA_SPI_TX, DMA_TRANSF_INT_CLR);
#else
            msleep(1);
#endif
        }
        lprintf("Done.\n");
    }
#elif 0  //---------------------------
    {
        int i, j, n, ni, no, flags, oflen;
        u32 optr = sprd_data->buffer_dma;
        u32 iptr = optr + 100;
        u8 *optrv= sprd_data->buffer;
        u8 *iptrv= optrv + 100;
        memset(sprd_data->buffer, 0x15, SPRD_SPI_BUFFER_SIZE);

        sprd_spi_setup_dma(sprd_data, spi);

        cs_activate(sprd_data, spi);

        oflen = 0;// 1023;
        flags = 7 | 0x80;

#if 1
        for (i = 0; i < 2; i++) {
#else
        for (i = 0; i < 0x7fffffff; i++) {
#endif

if (flags & 0x1) {
            no = (i % 5) + 1 + oflen;
            printk("enter only tx %d\n", no);
            memset(optrv, 0x15, no + 1);
            sprd_dma_update_spi(optr, no, 0, 0, spi, sprd_data);
            msleep(10);
}

if (flags & 0x2) {
            ni = (i % 5) + 1 + oflen;
            printk("enter only rx %d\n", ni);
            memset(iptrv, 0x88, ni + 1);
            n = sprd_dma_update_spi(0, no, iptr, ni, spi, sprd_data);
            msleep(10);

            for (j = 0; j <= n; j++) {
                printk("%02X ", iptrv[j]);
                if ((j & 0x1f) == 0x1f)
                    printk("\n");
            }
            if (j & 0x1f)
                printk("\n");
}

if (flags & 0x4) {
            no = ni = (i % 5) + 1 + oflen;
            printk("enter tx rx both %d\n", ni);
            memset(optrv, 0x15, no + 1);
            memset(iptrv, 0x88, ni + 1);
            n = sprd_dma_update_spi(optr, no, iptr, ni, spi, sprd_data);
            msleep(10);

            for (j = 0; j <= n; j++) {
                printk("%02X ", iptrv[j]);
                if ((j & 0x1f) == 0x1f)
                    printk("\n");
            }
            if (j & 0x1f)
                printk("\n");
}

if (flags & 0x80) {
            msleep(500);
}

        }
        lprintf("Done.\n");
    }
#elif 0  //---------------------------
    {
        struct spi_transfer t[1];
        struct spi_message m;
        u8 stack_data[64];
        u8 *kdata = kmalloc(512, GFP_KERNEL);
        u8 *dma_data = sprd_data->buffer;
        dma_addr_t dma_phy = sprd_data->buffer_dma;

    do {
        memset(stack_data, 0x05, sizeof stack_data);
        memset(kdata, 0x15, 512);
        memset(dma_data, 0x55, SPRD_SPI_BUFFER_SIZE);

        spi_message_init(&m);
        memset(t, 0, sizeof t);
        t[0].tx_buf = stack_data;
        t[0].len = sizeof stack_data;
        spi_message_add_tail(&t[0], &m);
        spi_sync(spi, &m);
        msleep(500);

        spi_message_init(&m);
        memset(t, 0, sizeof t);
        t[0].tx_buf = kdata;
        t[0].len = 512;
        spi_message_add_tail(&t[0], &m);
        spi_sync(spi, &m);
        msleep(500);

        spi_message_init(&m);
        m.is_dma_mapped = 1;
        memset(t, 0, sizeof t);
        t[0].tx_dma = dma_phy;
        // t[0].rx_dma = dma_phy;
        t[0].len = SPRD_SPI_BUFFER_SIZE;
        spi_message_add_tail(&t[0], &m);
        spi_sync(spi, &m);
        msleep(500);
    } while (1);
        kfree(kdata);
    }
#endif
#endif

    return 0;
}

static void sprd_spi_cleanup(struct spi_device *spi)
{
    struct sprd_spi_data *sprd_data = spi_master_get_devdata(spi->master);
    struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
	u32 cs_gpio = (u32)sprd_ctrl_data;
	unsigned long flags;

    if (cs_gpio < 0xffff)
        return;
    cs_gpio = sprd_ctrl_data->cs_gpio;

	spin_lock_irqsave(&sprd_data->lock, flags);
	if (sprd_data->cspi == spi) {
		sprd_data->cspi = NULL;
		cs_deactivate(sprd_data, spi);
	}
    spi->controller_data = (void*)cs_gpio;
	spin_unlock_irqrestore(&sprd_data->lock, flags);

	gpio_free(cs_gpio);
	kfree(sprd_ctrl_data);
}

static int __init sprd_spi_probe(struct platform_device *pdev)
{
    struct resource *regs;
	int irq, ret;
	struct spi_master *master;
	struct sprd_spi_data *sprd_data;

    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!regs)
        return -ENXIO;

    irq = platform_get_irq(pdev, 0);
    if (irq < 0)
        return irq;

    ret = -ENOMEM;
    master = spi_alloc_master(&pdev->dev, sizeof *sprd_data);
    if (!master)
        goto out_free;

    /* the spi->mode bits understood by this driver: */
    master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_3WIRE;

    master->bus_num = pdev->id;
    master->num_chipselect = 64;
    master->setup = sprd_spi_setup;
    master->transfer = sprd_spi_transfer;
    master->cleanup = sprd_spi_cleanup;
    platform_set_drvdata(pdev, master);

    sprd_data = spi_master_get_devdata(master);

    /*
     * Scratch buffer is used for throwaway rx and tx data.
     * It's coherent to minimize dcache pollution.
     */
    sprd_data->buffer = dma_alloc_coherent(&pdev->dev, SPRD_SPI_BUFFER_SIZE,
                                        &sprd_data->buffer_dma, GFP_KERNEL);
    if (!sprd_data->buffer)
        goto out_free;

    spin_lock_init(&sprd_data->lock);
    INIT_LIST_HEAD(&sprd_data->queue);
    sprd_data->pdev = pdev;
    sprd_data->regs = ioremap(regs->start, (regs->end - regs->start) + 1);
    if (!sprd_data->regs)
        goto out_free_buffer;
    sprd_data->irq = irq;

#if SPRD_SPI_DMA_MODE
    ret = sprd_request_dma(DMA_SPI_TX, sprd_spi_interrupt, master);
    ret = sprd_request_dma(DMA_SPI_RX, sprd_spi_interrupt, master);
#else
    ret = request_irq(irq, sprd_spi_interrupt, 0,
                dev_name(&pdev->dev), master);
#endif
    if (ret)
        goto out_unmap_regs;

    /* Initialize the hardware */
    __raw_bits_or(GEN0_SPI_EN, GR_GEN0); // Enable SPI module
    __raw_bits_or(SWRST_SPI_RST, GR_SOFT_RST);
    msleep(1);
    __raw_bits_and(~SWRST_SPI_RST, GR_SOFT_RST);
    spi_writel(0, SPI_INT_EN);

    sprd_data->cs_null = 1;

    ret = spi_register_master(master);
    if (ret)
        goto out_reset_hw;

    return 0;

out_reset_hw:
#if SPRD_SPI_DMA_MODE
    sprd_free_dma(DMA_SPI_TX);
    sprd_free_dma(DMA_SPI_RX);
#else
    free_irq(irq, master);
#endif
out_unmap_regs:
    iounmap(sprd_data->regs);
out_free_buffer:
    dma_free_coherent(&pdev->dev, SPRD_SPI_BUFFER_SIZE, sprd_data->buffer,
                    sprd_data->buffer_dma);
out_free:
    spi_master_put(master);
    return ret;
}

static int __exit sprd_spi_remove(struct platform_device *pdev)
{
    struct spi_master	*master = platform_get_drvdata(pdev);
    struct sprd_spi_data *sprd_data = spi_master_get_devdata(master);
	struct spi_message *msg;
    /* reset the hardware and block queue progress */
    spin_lock_irq(&sprd_data->lock);
    sprd_data->stopping = 1;
    spin_unlock_irq(&sprd_data->lock);

    /* Terminate remaining queued transfers */
    list_for_each_entry(msg, &sprd_data->queue, queue) {
        msg->status = -ESHUTDOWN;
        msg->complete(msg->context);
    }

    dma_free_coherent(&pdev->dev, SPRD_SPI_BUFFER_SIZE, sprd_data->buffer,
                    sprd_data->buffer_dma);
#if SPRD_SPI_DMA_MODE
    sprd_free_dma(DMA_SPI_TX);
    sprd_free_dma(DMA_SPI_RX);
#else
    free_irq(sprd_data->irq, master);
#endif
    iounmap(sprd_data->regs);

    spi_unregister_master(master);

    __raw_bits_and(~GEN0_SPI_EN, GR_GEN0); // Disable SPI module

    return 0;
}

#ifdef CONFIG_PM
static int sprd_spi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int sprd_spi_resume(struct platform_device *pdev)
{
    return 0;
}
#else
#define	sprd_spi_suspend NULL
#define	sprd_spi_resume NULL
#endif

static struct platform_driver sprd_spi_driver = {
    .driver		= {
        .name	= "sprd_spi",
        .owner	= THIS_MODULE,
    },
    .suspend	= sprd_spi_suspend,
    .resume		= sprd_spi_resume,
    .remove		= __exit_p(sprd_spi_remove),
};

static int __init sprd_spi_init(void)
{
    return platform_driver_probe(&sprd_spi_driver, sprd_spi_probe);
}
module_init(sprd_spi_init);

static void __exit sprd_spi_exit(void)
{
    platform_driver_unregister(&sprd_spi_driver);
}
module_exit(sprd_spi_exit);

MODULE_DESCRIPTION("SpreadTrum SC88XX Series SPI Controller driver");
MODULE_AUTHOR("Luther Ge <luther.ge@spreadtrum.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sprd_spi");
