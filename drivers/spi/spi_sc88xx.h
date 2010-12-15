/*
 * Header for SpreadTrum SC88XX Series SPI Controllers
 *
 * Copyright (C) 2010 SpreadTrum Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __SPI_SC88XX_H__
#define __SPI_SC88XX_H__

#include <mach/regs_global.h>
#include <mach/dma.h>

#define SPI_TXD                     0x0000
#define SPI_CLKD                    0x0004
#define SPI_CTL0                    0x0008
#define SPI_CTL1                    0x000c
#define SPI_CTL2                    0x0010
#define SPI_CTL3                    0x0014
#define SPI_CTL4                    0x0018
#define SPI_CTL5                    0x001c
#define SPI_INT_EN                  0x0020
#define SPI_INT_CLR                 0x0024
#define SPI_INT_RAW_STS             0x0028
#define SPI_INT_MASK_STS            0x002c
#define SPI_STS1                    0x0030
#define SPI_STS2                    0x0034
#define SPI_DSP_WAIT                0x0038
#define SPI_STS3                    0x003c
#define SPI_CTL6                    0x0040
#define SPI_STS4                    0x0044
#define SPI_FIFO_RST                0x0048

// Bit define for register STS2
#define SPI_RX_FIFO_FULL            BIT_0
#define SPI_RX_FIFO_EMPTY           BIT_1
#define SPI_TX_FIFO_FULL            BIT_2
#define SPI_TX_FIFO_EMPTY           BIT_3
#define SPI_RX_FIFO_REALLY_FULL     BIT_4
#define SPI_RX_FIFO_REALLY_EMPTY    BIT_5
#define SPI_TX_FIFO_REALLY_FULL     BIT_6
#define SPI_TX_FIFO_REALLY_EMPTY    BIT_7
#define SPI_TX_BUSY                 BIT_8
// Bit define for register ctr1
#define SPI_RX_MODE                 BIT_12
#define SPI_TX_MODE                 BIT_13
// Bit define for register ctr2
#define SPI_DMA_EN                  BIT_6
// Bit define for register ctr4
#define SPI_START_RX                BIT_9

#define spi_writel(value, reg) \
    __raw_writel(value, (volatile unsigned char __force *)sprd_data->regs + reg)

#define spi_readl(reg) \
    __raw_readl((volatile unsigned char __force *)sprd_data->regs + reg)

#define spi_bits_or(value, reg) \
    __raw_bits_or(value, (unsigned int)sprd_data->regs + reg)

#define spi_bits_and(value, reg) \
    __raw_bits_and(value, (unsigned int)sprd_data->regs + reg)

#define spi_write(reg, shift, val, mask) \
{ \
    unsigned long flags; \
    u32 tmp; \
    volatile void *regs = (volatile unsigned char __force *)sprd_data->regs + reg; \
    raw_local_irq_save(flags); \
    tmp = __raw_readl(regs); \
    tmp &= ~(mask<<shift); \
    tmp |= val << shift; \
    __raw_writel(tmp, regs); \
    raw_local_irq_restore(flags); \
}

#define spi_start() \
    do { \
        spi_write(SPI_CTL1, 12, 0x03, 0x03); /* Enable SPI transmit and receive both mode */\
        spi_writel(SPI_FIFO_RST, 1); /* spi rx功能使用的话,必须执行一次reset fifo,否则dma在rx传输中*/\
        spi_writel(SPI_FIFO_RST, 0); /* DMA会时常停止工作,通过读取1次SPI_TXD寄存器,DMA才可以恢复工作*/\
        sprd_dma_start2(DMA_SPI_TX, DMA_SPI_RX); \
    } while (0)

#define spi_start_tx() \
    do { \
        spi_write(SPI_CTL1, 12, 0x02, 0x03); /* Only Enable SPI transmit mode */\
        sprd_dma_start(DMA_SPI_TX); \
    } while (0)

#define spi_start_rx(blocks) \
    do { \
        /* 实验发现,SPI_CTL4的bit9,在每次置1之前,必须首先清0, 新一次的rx传输才会发生[luther.ge] */ \
        spi_writel(0x0000, SPI_CTL4); \
        spi_write(SPI_CTL1, 12, 0x01, 0x03); /* Only Enable SPI receive mode */\
        spi_writel(SPI_FIFO_RST, 1); /* spi rx功能使用的话,必须执行一次reset fifo,否则dma在rx传输中*/\
        spi_writel(SPI_FIFO_RST, 0); /* DMA会时常停止工作,通过读取1次SPI_TXD寄存器,DMA才可以恢复工作*/\
        sprd_dma_start(DMA_SPI_RX); \
        /* spi_write(SPI_CTL4,  9, 0x01, 0x01); */ /* start rx spiclk, must do it last */\
        spi_writel((1 << 9) | blocks, SPI_CTL4); \
    } while (0)

#if 1
#define lprintf(msg...) printk(KERN_EMERG "glx : %s() --> ", __func__);printk(msg)
#else
#define lprintf(msg...)
#endif

#define SPRD_SPI_DMA_MODE 1
struct sprd_spi_data {
#define SPRD_SPI_BUFFER_SIZE PAGE_SIZE
#define SPRD_SPI_DMA_BLOCK_MAX  (0x1ff) /* ctrl4-[0:8] */
    spinlock_t lock;
    struct list_head queue;
    struct spi_message *cspi_msg;
    struct spi_transfer	*cspi_trans;
    int cspi_trans_num;
    struct spi_device *cspi;
    void *buffer;
    dma_addr_t buffer_dma;

    int irq;
    struct platform_device *pdev;
    void __iomem *regs;

    u8 stopping;
    u8 cs_null;
};

struct sprd_spi_controller_data {
    u32 cs_gpio;
    u32 clk_spi_and_div;
    u32 spi_clkd;
    u32 spi_ctl0;
    u32 spi_ctl3;
    u32 data_width; /* 1bit,2bit,...,8bits,...,16bits,...,32bits */
    u32 data_width_order;
    u32 data_max;
    sprd_dma_desc dma_desc_rx;
    sprd_dma_desc dma_desc_tx;
};

#define grab_sibling(sprd_data) \
    sprd_data->cspi_msg = list_entry(sprd_data->queue.next, struct spi_message, queue); \
    /*if (unlikely(list_empty(&sprd_data->cspi_msg->transfers))) \
        goto __EINVAL;*/ \
    sprd_data->cspi_trans = list_entry(sprd_data->cspi_msg->transfers.next, \
                                struct spi_transfer, transfer_list); \
    sprd_data->cspi_trans_num = 0; \
/*__EINVAL:*/

#define grab_subsibling(sprd_data) \
    sprd_data->cspi_trans = list_entry(sprd_data->cspi_trans->transfer_list.next,\
                                struct spi_transfer, transfer_list);\
    sprd_data->cspi_trans_num = 0;

static int sprd_spi_do_transfer(struct sprd_spi_data *sprd_data);
static inline void cs_activate(struct sprd_spi_data *sprd_data, struct spi_device *spi);
static inline void cs_deactivate(struct sprd_spi_data *sprd_data, struct spi_device *spi);
static inline int sprd_dma_update_spi(u32 sptr, u32 slen, u32 dptr, u32 dlen,
                        struct spi_device *spi, struct sprd_spi_data *sprd_data);
static int sprd_spi_dma_map_transfer(struct sprd_spi_data *sprd_data, struct spi_transfer *trans);
static void sprd_spi_dma_unmap_transfer(struct sprd_spi_data *sprd_data, struct spi_transfer *trans);
#endif
