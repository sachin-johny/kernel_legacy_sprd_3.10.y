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
#include <linux/sched.h>
#include <linux/kthread.h>

#include <asm/io.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include "spi_sc8810.h"
#define SPRD_SPI_DEBUG  0
#define SPRD_SPI_CS_GPIO 1 /* you should also modify the spi_func_cfg[] */
#define ATH_HACK

#define _doSWAP32(x)	\
		((((x) & 0x000000FF) << 24) | \
		(((x) & 0x0000FF00) << 8)  | \
		(((x) & 0x00FF0000) >> 8)  | \
		(((x) & 0xFF000000) >> 24))
		
#define _doSWAP16(x)	\
		((((x) & 0x00FF) << 8) | \
		(((x) & 0xFF00) >> 8) )
// lock is held, spi irq is disabled
static int sprd_spi_do_transfer(struct sprd_spi_data *sprd_data)
{
    int len;
    unsigned long flags;

    sprd_data->dma_started = 0;

    if (sprd_data->cspi_trans == 0) return 0; /* when both tx & rx mode starts, first tx irq enters, 
                                               * then rx irq enters -- this time tx 
                                               * had set sprd_data->cspi_trans to 0, so return 0 [luther.ge]
                                               */

    spi_do_reset(sprd_data->cspi_trans->tx_dma,
                 sprd_data->cspi_trans->rx_dma); // spi hardwrare module has a bug, we must read busy bit twice to soft fix

    if (sprd_data->cspi_trans->rx_buf) {
        if (!sprd_data->cspi_msg->is_dma_mapped && sprd_data->cspi_trans_num) {
            if (sprd_data->cspi_trans_num == sprd_data->cspi_trans_len)
                sprd_data->rx_ptr = sprd_data->cspi_trans->rx_buf;

            memcpy(sprd_data->rx_ptr, sprd_data->rx_buffer, sprd_data->cspi_trans_len);

            if (sprd_data->cspi_trans_num < sprd_data->cspi_trans->len)
                sprd_data->rx_ptr += sprd_data->cspi_trans_len;
        }
    }

    if (sprd_data->cspi_trans_num >= sprd_data->cspi_trans->len) {
        sprd_data->cspi_msg->actual_length += sprd_data->cspi_trans_num;
#if SPRD_SPI_DEBUG
        printk(KERN_WARNING "send ok\n");
#endif
        if (!sprd_data->cspi_msg->is_dma_mapped)
            sprd_spi_dma_unmap_transfer(sprd_data, sprd_data->cspi_trans);

        local_irq_save(flags);

        if (sprd_data->cspi_msg->transfers.prev != &sprd_data->cspi_trans->transfer_list) {
            grab_subsibling(sprd_data);
        } else {
            cs_deactivate(sprd_data, sprd_data->cspi); // all msg sibling data trans done
            list_del(&sprd_data->cspi_msg->queue);
            sprd_data->cspi_msg->status = 0; // msg tranfsered successfully

            local_irq_restore(flags);

            spin_unlock(&sprd_data->lock);
            if (sprd_data->cspi_msg->complete) // spi_sync call
                sprd_data->cspi_msg->complete(sprd_data->cspi_msg->context);
            spin_lock(&sprd_data->lock);

            local_irq_save(flags);

            sprd_data->cspi_trans = 0;
            if (list_empty(&sprd_data->queue)) {
                // no spi data on queue to transfer
                // cs_deactivate(sprd_data, sprd_data->cspi);
#if SPRD_SPI_DEBUG
                printk(KERN_WARNING "spi msg queue done.\n");
#endif
                local_irq_restore(flags);
                return 0;
            }
            grab_sibling(sprd_data);
        }

        local_irq_restore(flags);
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
    len = sprd_dma_update_spi(sprd_data->cspi_trans->tx_dma,
                                            sprd_data->cspi_trans->len,
                                            sprd_data->cspi_trans->rx_dma,
                                            sprd_data->cspi_trans->len,
                                            sprd_data->cspi,
                                            sprd_data);
    if (!sprd_data->cspi_msg->is_dma_mapped) {
        if (sprd_data->cspi_trans->tx_dma
#if SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX
            && (sprd_data->cspi_trans->tx_buf != SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX_IGNORE_ADDR)
#endif
        ) {
            memcpy(sprd_data->tx_buffer, (u8 *)sprd_data->cspi_trans->tx_buf + sprd_data->cspi_trans_num, len);
        }
    }
#if SPRD_SPI_DEBUG
    else {
        printk("spi-dma-transfer=%d\n", sprd_data->cspi_trans->len);
    }
#endif

    sprd_data->cspi_trans_len = len;
    sprd_data->cspi_trans_num += len;
    sprd_data->dma_started = 1;
    spi_dma_start(); // Must Enable SPI_DMA_EN last

    return 0;
}

void dump_buffer(unsigned char *buffer, int length){

  int i=0;
  
  for(i=0; i< length&& i<32*32; i++){
    printk("%02X ",buffer[i]);
    if(i%8==7) printk(" ");
    if(i%32==31) printk("\n");
  }
  printk("\n\n");
}

#define CSPI_HACK

#ifdef CSPI_HACK
static int sprd_spi_direct_transfer_tx(void *data_in, const void *data_out, int len, void *cookie, void *cookie2)
{
  int i,timeout;

#define MYLOCAL_TIMEOUT 0xff0000
    struct sprd_spi_data *sprd_data = cookie;
    // struct sprd_spi_controller_data *sprd_ctrl_data = cookie2;
  /* Set tx only */
  spi_write_reg(SPI_CTL1, 12, 0x02, 0x03);

  for(i=0;i<len;++i){
    /* Wait for not full tx fifo */
    for(timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL) && timeout++ < MYLOCAL_TIMEOUT;);
    spi_writel(((u8*)data_out)[i],SPI_TXD);
  }

  return 0;
}
#endif

#ifdef CSPI_HACK
static int sprd_spi_direct_transfer_rx(void *data_in, const void *data_out, int len, void *cookie, void *cookie2)
{
  int i,j,timeout, block;
  int tlen = 0;
  int block_bytes = 128;
  unsigned char* data;

#define MYLOCAL_TIMEOUT 0xff0000
    struct sprd_spi_data *sprd_data = cookie;
    // struct sprd_spi_controller_data *sprd_ctrl_data = cookie2;

  data = (unsigned char *)data_in;
  /* Enable rx only */
  spi_write_reg(SPI_CTL1, 12, 0x01, 0x03); 

  for (i = 0, tlen = len; tlen;i++) {
    if (tlen > block_bytes) {
      block = block_bytes;
      tlen -= block_bytes;
    } else {
      block = tlen;
      tlen = 0;
    }
    
    spi_writel(0x0000, SPI_CTL4); /* stop only rx */
    spi_writel((1 << 9) | block, SPI_CTL4);
    
    for (j = 0; j < block; j++) {
      /* wait for rx fifo not empty */
      for (timeout = 0;(spi_readl(SPI_STS2) & SPI_RX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);
      /* Read the data */
      data[i*block_bytes+j] = spi_readl(SPI_TXD);
    }
  }
  return 0;
}
#endif

static int sprd_spi_direct_transfer_full_duplex(void *data_in, const void *data_out, int len, void *cookie, void *cookie2)
{
  int i,j,timeout,block,tlen;
#define MYLOCAL_TIMEOUT 0xff0000
#ifndef CSPI_HACK
#  define FIFO_SIZE 16
#else
#  define FIFO_SIZE 1
#endif

  struct sprd_spi_data *sprd_data = cookie;
  // struct sprd_spi_controller_data *sprd_ctrl_data = cookie2;

  /* Setup Full Duplex */
  spi_write_reg(SPI_CTL1, 12, 0x03, 0x03);

  j=0;

  /* Repeat until done */
  for (i = 0, tlen = len; tlen;i++){
    if (tlen > FIFO_SIZE) {
      block = FIFO_SIZE;
      tlen -= FIFO_SIZE;
    } else {
      block = tlen;
      tlen = 0;
    }
    ///* Wait until tx fifo buffer not full */
    //for(timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL) && timeout++ < MYLOCAL_TIMEOUT;);

    /* Wait until tx fifo empty*/
    for (timeout = 0;!(spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);

    /* Write byte to tx fifo buffer */
    for(j=0;j < block; ++j)
      spi_writel(((u8*)data_out)[i*FIFO_SIZE+j],SPI_TXD);

    /* Wait until tx fifo empty*/
    for (timeout = 0;!(spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);

    /* Wait until rx fifo not empty */
    for (timeout = 0;(spi_readl(SPI_STS2) & SPI_RX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);

    /* Read as many bytes as were written */
    for(j=0;j<block;++j)
      ((u8*)data_in)[i*FIFO_SIZE+j] = spi_readl(SPI_TXD);
  }

  return 0;
}

#ifdef CSPI_HACK
#  define CSPI_READ 0x10
#  define CSPI_WRITE 0x20
#  define CSPI_BURST 0x40
#  define CSPI_TYPE_MASK 0x70
#endif

static int sprd_spi_direct_transfer_main(void *data_in, const void *data_out, int len, void *cookie, void *cookie2)
{
  unsigned char cmd = *(unsigned char*)data_out;
  unsigned char *data_read = (unsigned char *)data_in;
  unsigned char *data_write = (unsigned char *)data_out;
  int fd_len=min(len,10);
  
#ifndef CSPI_HACK
  sprd_spi_direct_transfer_full_duplex(data_in, data_out, len, cookie, cookie2);
#else
  switch(cmd & CSPI_TYPE_MASK)
    {
    case CSPI_READ:
    case CSPI_READ|CSPI_BURST:
      sprd_spi_direct_transfer_full_duplex(data_in, data_out, fd_len, cookie, cookie2);
      if(fd_len!=len)
	sprd_spi_direct_transfer_rx(&data_read[fd_len], &data_write[fd_len], len - fd_len, cookie, cookie2);
      break;
    case CSPI_WRITE:
    case CSPI_WRITE|CSPI_BURST:
      sprd_spi_direct_transfer_full_duplex(data_in, data_out, fd_len, cookie, cookie2);
      if(fd_len!=len)
	sprd_spi_direct_transfer_tx(&data_read[fd_len], &data_write[fd_len], len - fd_len, cookie, cookie2);
      break;
    default:
      sprd_spi_direct_transfer_full_duplex(data_in, data_out, len, cookie, cookie2);
    }
#endif
  return 0;
}


static int sprd_spi_direct_transfer(void *data_in, const void *data_out, int len, void *cookie, void *cookie2)
{
  int i, timeout;
  u8 *data;
#ifdef CSR_CSPI
  unsigned char *cmd;
  int write_len=0;
  int read_len=0;
#endif

#define MYLOCAL_TIMEOUT 0xff0000
  struct sprd_spi_data *sprd_data = cookie;
  struct sprd_spi_controller_data *sprd_ctrl_data = cookie2;
    

  if (data_in) {
    int block, tlen, j, block_bytes;

    spi_write_reg(SPI_CTL1, 12, 0x01, 0x03); /* Only Enable SPI receive mode */

    if (likely(sprd_ctrl_data->data_width != 3)) {
      block_bytes = SPRD_SPI_DMA_BLOCK_MAX << sprd_ctrl_data->data_width_order;
    } else block_bytes = SPRD_SPI_DMA_BLOCK_MAX * 3;

    for (i = 0, tlen = len; tlen;) {
      if (tlen > block_bytes) {
	block = SPRD_SPI_DMA_BLOCK_MAX;
	tlen -= block_bytes;
      } else {
	if (likely(sprd_ctrl_data->data_width != 3))
	  block = tlen >> sprd_ctrl_data->data_width_order;
	else block = tlen / sprd_ctrl_data->data_width;
	tlen = 0;
      }

      spi_writel(0x0000, SPI_CTL4); /* stop only rx */
      spi_writel((1 << 9) | block, SPI_CTL4);

      for (j = 0; j < block; j++) {
	for (timeout = 0;(spi_readl(SPI_STS2) & SPI_RX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);
	if (timeout >= MYLOCAL_TIMEOUT){
	  printk("Timeout spi_readl(SPI_STS2) & SPI_RX_FIFO_REALLY_EMPTY)\n");
	  return  -ENOPROTOOPT;
	}
	data = (u8*)data_in + i;
	switch (sprd_ctrl_data->data_width) {
	case 1: ( (u8*)data)[0] = spi_readl(SPI_TXD); i += 1; break;
	case 2: ((u16*)data)[0] = spi_readl(SPI_TXD); i += 2; break;
	case 4: ((u32*)data)[0] = spi_readl(SPI_TXD); i += 4; break;
	}
      }
    }
  }

  if (data_out) {
    spi_write_reg(SPI_CTL1, 12, 0x02, 0x03); /* Only Enable SPI transmit mode */
    for (i = 0; i < len;) {
      for(timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL) && timeout++ < MYLOCAL_TIMEOUT;);
      if (timeout >= MYLOCAL_TIMEOUT){
	printk("Timeout spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL)\n");
	return  -ENOPROTOOPT;
      }
      data = (u8*)data_out + i;
      switch (sprd_ctrl_data->data_width) {
      case 1: spi_writel(( (u8*)data)[0], SPI_TXD); i += 1; break;
      case 2: spi_writel(((u16*)data)[0], SPI_TXD); i += 2; break;
      case 4: spi_writel(((u32*)data)[0], SPI_TXD); i += 4; break;
      }
    }
    for (timeout = 0;!(spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);
    if (timeout >= MYLOCAL_TIMEOUT){
      printk("Timeout spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY)\n");
      return -ENOPROTOOPT;
    }
    // for (i = 0; i < 5; i++);
    for (timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_BUSY) && timeout++ < MYLOCAL_TIMEOUT;);
    if (timeout >= MYLOCAL_TIMEOUT){
      printk("Timeout spi_readl(SPI_STS2) & SPI_TX_BUSY)\n");
      return -ENOPROTOOPT;
    }
  }

  return 0;
}

static int sprd_spi_direct_transfer_ath_spec(void *data_in, const void *data_out, int len, void *cookie, void *cookie2)
{
  int i, timeout;
  u8 *data;
  int data_width = 0;

#ifdef ATH_HACK
#define USE_RX_SWAP
#define MYLOCAL_TIMEOUT 0xff0000
  struct sprd_spi_data *sprd_data = cookie;
  struct sprd_spi_controller_data *sprd_ctrl_data = cookie2;
  u32    spi_ctl0 = sprd_ctrl_data->spi_ctl0;

  if(data_in){
#ifdef USE_RX_SWAP
      spi_ctl0 &= ~(0x1F << 2);
      if (len%4 == 0){
          spi_writel(spi_ctl0, SPI_CTL0);
          sprd_ctrl_data->data_width = 4;
          sprd_ctrl_data->data_width_order = 2;
      }else if ((len % 2) == 0){
          spi_ctl0 |= (0x10 << 2);
          spi_writel(spi_ctl0, SPI_CTL0);
          sprd_ctrl_data->data_width = 2;
          sprd_ctrl_data->data_width_order = 1;
      }
      else
#endif
     {
        spi_writel(sprd_ctrl_data->spi_ctl0, SPI_CTL0);
        sprd_ctrl_data->data_width = 1;
        sprd_ctrl_data->data_width_order = 0;
     }
  }
  
  if (data_in) {
    int block, tlen, j, block_bytes;
	u32 tmpData = 0;
 
    spi_write_reg(SPI_CTL1, 12, 0x01, 0x03); /* Only Enable SPI receive mode */

    if (likely(sprd_ctrl_data->data_width != 3)) {
      block_bytes = SPRD_SPI_DMA_BLOCK_MAX << sprd_ctrl_data->data_width_order;
    } else block_bytes = SPRD_SPI_DMA_BLOCK_MAX * 3;

    for (i = 0, tlen = len; tlen;) {
      if (tlen > block_bytes) {
    	block = SPRD_SPI_DMA_BLOCK_MAX;
    	tlen -= block_bytes;
      } else {
    	if (likely(sprd_ctrl_data->data_width != 3))
    	  block = tlen >> sprd_ctrl_data->data_width_order;
    	else 
          block = tlen / sprd_ctrl_data->data_width;
    	tlen = 0;
      }
	  
      spi_writel(0x0000, SPI_CTL4); /* stop only rx */
      spi_writel((1 << 9) | block, SPI_CTL4);

      for (j = 0; j < block; j++) {
		for (timeout = 0;(spi_readl(SPI_STS2) & SPI_RX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);
		if (timeout >= MYLOCAL_TIMEOUT){
		  printk("Timeout spi_readl(SPI_STS2) & SPI_RX_FIFO_REALLY_EMPTY)\n");
		  return  -ENOPROTOOPT;
		}
		data = (u8*)data_in + i;
		switch (sprd_ctrl_data->data_width) {
		case 1: ( (u8*)data)[0] = spi_readl(SPI_TXD); i += 1; break;
#ifndef USE_RX_SWAP
		case 2: ((u16*)data)[0] = (spi_readl(SPI_TXD)); i += 2; break;
		case 4: ((u32*)data)[0] = (spi_readl(SPI_TXD)); i += 4; break;
#else
		case 2: 
			{
				tmpData = spi_readl(SPI_TXD);
				((u16*)data)[0] = _doSWAP16(tmpData); 
				i += 2; 
				break;
			}
		case 4:
			{
				tmpData = spi_readl(SPI_TXD);
				((u32*)data)[0] = _doSWAP32(tmpData); 				 
				i += 4; 
				break;
			}
#endif
		}
      }
    }
  }

  if (data_out) {
    u32 orgLen = 0;
    u32 len2 = 0;
    u32 size = 0;

    spi_write_reg(SPI_CTL1, 12, 0x02, 0x03); 
	
    orgLen = 4 - (((u32)data_out) % 4);
    if (orgLen < 4 && orgLen > 0)
    {
        spi_writel(sprd_ctrl_data->spi_ctl0, SPI_CTL0);      
        for (i = 0; (i < orgLen) && (i < len);){
            for(timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL) && timeout++ < MYLOCAL_TIMEOUT;);
            if (timeout >= MYLOCAL_TIMEOUT){     
                printk("Timeout spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL)\n");
                return  -ENOPROTOOPT;
            }

            data = (u8*)data_out + i;
            spi_writel(( (u8*)data)[0], SPI_TXD); 
            i += 1;           
        }

        for (timeout = 0;!(spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);
        if (timeout >= MYLOCAL_TIMEOUT){
           printk("Timeout spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY)\n");
           return -ENOPROTOOPT;
        }
        
        for (timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_BUSY) && timeout++ < MYLOCAL_TIMEOUT;);
        if (timeout >= MYLOCAL_TIMEOUT){
           printk("Timeout spi_readl(SPI_STS2) & SPI_TX_BUSY)\n");
           return -ENOPROTOOPT;
        }

        data_out = (u8*)data_out + i;
        
        if (len >= orgLen){
            len -= orgLen;
        }else{
            len = 0;
        }    
    }

    size = len & 0x3;
    len2 = len - size;
    
    spi_ctl0 &= ~(0x1F << 2); 
    spi_writel(spi_ctl0, SPI_CTL0);
    
    for (i = 0; i < len2;) {
        for(timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL) && timeout++ < MYLOCAL_TIMEOUT;);
        if (timeout >= MYLOCAL_TIMEOUT){     
            printk("Timeout spi_readl(SPI_STS2) & SPI_TX_FIFO_FULL)\n");
            return  -ENOPROTOOPT;
        }
        data = (u8*)data_out + i;	                  
		spi_writel(_doSWAP32(((u32*)data)[0]), SPI_TXD); 
        i += 4; 
       
    }
    if (len2 > 0)
    {
        for (timeout = 0;!(spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);
        if (timeout >= MYLOCAL_TIMEOUT){
           printk("Timeout spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY)\n");
           return -ENOPROTOOPT;
        }
        for (timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_BUSY) && timeout++ < MYLOCAL_TIMEOUT;);
        if (timeout >= MYLOCAL_TIMEOUT){
           printk("Timeout spi_readl(SPI_STS2) & SPI_TX_BUSY)\n");
           return -ENOPROTOOPT;
        }    
    }    
    data_out = (u8*)data_out + i;
    if (size){
        spi_writel(sprd_ctrl_data->spi_ctl0, SPI_CTL0);
        
        for (i = 0; i < size;){
            data = (u8*)data_out + i;
            spi_writel(( (u8*)data)[0], SPI_TXD); 
            i += 1;           
        }

        for (timeout = 0;!(spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY) && timeout++ < MYLOCAL_TIMEOUT;);
        if (timeout >= MYLOCAL_TIMEOUT){
           printk("Timeout spi_readl(SPI_STS2) & SPI_TX_FIFO_REALLY_EMPTY)\n");
           return -ENOPROTOOPT;
        }
        
        for (timeout = 0;(spi_readl(SPI_STS2) & SPI_TX_BUSY) && timeout++ < MYLOCAL_TIMEOUT;);
        if (timeout >= MYLOCAL_TIMEOUT){
           printk("Timeout spi_readl(SPI_STS2) & SPI_TX_BUSY)\n");
           return -ENOPROTOOPT;
        }
    }
  }
#endif
  return 0;
}

static int sprd_spi_direct_transfer_compact(struct spi_device *spi, struct spi_message *msg)
{
    struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
    struct sprd_spi_data *sprd_data = spi_master_get_devdata(spi->master);
    struct spi_transfer *cspi_trans; // = list_entry(msg->transfers.next, struct spi_transfer, transfer_list);
    unsigned int cs_change = 1;

    down(&sprd_data->process_sem_direct);

    cspi_trans = list_entry(msg->transfers.next, struct spi_transfer, transfer_list);
    cs_activate(sprd_data, spi);

    do {
       switch (sprd_ctrl_data->tmod) {
            case SPI_TMOD_CSR:
                msg->status = sprd_spi_direct_transfer_main(
                        cspi_trans->rx_buf, cspi_trans->tx_buf,
                        cspi_trans->len, sprd_data, sprd_ctrl_data);
            break;
            case SPI_TMOD_DEMOD:
                if (cs_change) {
                    cs_activate(sprd_data, spi);
                    cs_change = cspi_trans->cs_change;
                }

                 msg->status = sprd_spi_direct_transfer(
                        cspi_trans->rx_buf, cspi_trans->tx_buf,
                        cspi_trans->len, sprd_data, sprd_ctrl_data);

                if (cs_change) {
                    cs_deactivate(sprd_data, spi);
                }
            break;
            case SPI_TMOD_ATH:
		msg->status = sprd_spi_direct_transfer_ath_spec(
                cspi_trans->rx_buf, cspi_trans->tx_buf,
                cspi_trans->len, sprd_data, sprd_ctrl_data);
		break;
            default:
                msg->status = sprd_spi_direct_transfer(
                        cspi_trans->rx_buf, cspi_trans->tx_buf,
                        cspi_trans->len, sprd_data, sprd_ctrl_data);
            break;
        }
       if (msg->status < 0) break;
        msg->actual_length += cspi_trans->len;
        if (msg->transfers.prev == &cspi_trans->transfer_list) break;
        cspi_trans = list_entry(cspi_trans->transfer_list.next, struct spi_transfer, transfer_list);
    } while (1);
    cs_deactivate(sprd_data, spi);

    up(&sprd_data->process_sem_direct);

    msg->complete(msg->context);

    return 0;
}

static int sprd_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
    struct sprd_spi_data *sprd_data;
    struct spi_transfer *trans;
    unsigned long flags;
    int ret = 0;

    // we use direct transfer function for linux kernel default spi api
#if 1    
    if (msg->complete != spi_complete2)
      return sprd_spi_direct_transfer_compact(spi, msg);
#endif

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
#if SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX
	  if (!trans->tx_dma && trans->rx_dma) {
            trans->tx_buf = SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX_IGNORE_ADDR;
            trans->tx_dma = sprd_data->tx_buffer_dma;
	  }
#endif
	  if (!(trans->tx_dma || trans->rx_dma) || !trans->len)
            return -ENOMEM;
	}
	
	msg->status = -EINPROGRESS;
	msg->actual_length = 0;

down(&sprd_data->process_sem_direct);

    spin_lock_irqsave(&sprd_data->lock, flags);
    list_add_tail(&msg->queue, &sprd_data->queue);
    if (sprd_data->cspi_trans == 0) {
        grab_sibling(sprd_data);
        up(&sprd_data->process_sem); // ret = sprd_spi_do_transfer(sprd_data);
    }
    spin_unlock_irqrestore(&sprd_data->lock, flags);

up(&sprd_data->process_sem_direct);

    return ret;
}

void sprd_spi_tmod(struct spi_device *spi, u32 transfer_mod)
{
    if (spi) {
        struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
        sprd_ctrl_data->tmod = transfer_mod;
    }
}
EXPORT_SYMBOL_GPL(sprd_spi_tmod);

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
    lprintf("spi irq [ %d ]\n", irq);
#endif
    if (sprd_data->cspi_trans &&
        sprd_data->cspi_trans->tx_dma && 
        sprd_data->cspi_trans->rx_dma) {
        sprd_dma_stop(irq);
        if (++sprd_data->tx_rx_finish < 2) {
            // printk("ignore tx_rx first irq\n");
            spin_unlock(&sprd_data->lock);
#if !SPRD_SPI_DMA_MODE
            return IRQ_HANDLED;
#endif
        }
        sprd_data->tx_rx_finish = 0;
        // printk("tx_rx irq ok\n");
    }
    up(&sprd_data->process_sem);
    up(&sprd_data->process_sem_direct);
/*
    if (sprd_spi_do_transfer(sprd_data) < 0)
        printk(KERN_ERR "error : %s\n", __func__);
*/
    spin_unlock(&sprd_data->lock);

#if !SPRD_SPI_DMA_MODE
    return IRQ_HANDLED;
#endif
}

extern int sprd_spi_cs_hook(int cs_gpio, int dir);
static inline void cs_activate(struct sprd_spi_data *sprd_data, struct spi_device *spi)
{
    if (sprd_data->cs_null) {
        struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;

        spi_writel(sprd_ctrl_data->spi_clkd, SPI_CLKD);
        spi_writel(sprd_ctrl_data->spi_ctl0, SPI_CTL0);

        __raw_bits_or((sprd_ctrl_data->clk_spi_and_div & 0xffff) << 21, GR_GEN2);
        __raw_bits_or((sprd_ctrl_data->clk_spi_and_div >> 16) << 26, GR_CLK_DLY);

#if SPRD_SPI_CS_GPIO
        __gpio_set_value(sprd_spi_cs_hook(sprd_ctrl_data->cs_gpio, 1), spi->mode & SPI_CS_HIGH);
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
        __gpio_set_value(sprd_spi_cs_hook(sprd_ctrl_data->cs_gpio, -1), !(spi->mode & SPI_CS_HIGH));
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
//  struct device *dev = &sprd_data->pdev->dev;

    if (trans->tx_buf) trans->tx_dma = sprd_data->tx_buffer_dma;
    if (trans->rx_buf) trans->rx_dma = sprd_data->rx_buffer_dma;
/*
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
*/
    return 0;
}

static void sprd_spi_dma_unmap_transfer(struct sprd_spi_data *sprd_data, struct spi_transfer *trans)
{
/*
    struct device *dev = &sprd_data->pdev->dev;

    if (trans->tx_dma 
#if SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX
        && (trans->tx_buf != SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX_IGNORE_ADDR)
#endif
       )
        dma_unmap_single(dev, trans->tx_dma, trans->len, DMA_TO_DEVICE);

    if (trans->rx_dma)
        dma_unmap_single(dev, trans->rx_dma, trans->len, DMA_FROM_DEVICE);
*/
}

static int sprd_spi_setup_dma(struct sprd_spi_data *sprd_data, struct spi_device *spi)
{
    int i, ch_id;
    u32 dsrc, ddst;
    int width;
    int autodma_src, autodma_dst, autodma_burst_mod_src, autodma_burst_mod_dst;
    struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
    sprd_dma_ctrl ctrl;// = {.dma_desc = &sprd_ctrl_data->dma_desc};

    for (i = 0; i < 2; i++) {
        if (i == 0) ch_id = DMA_SPI_RX;
        else ch_id = DMA_SPI_TX;

        if (ch_id == DMA_SPI_RX) {
            ctrl.dma_desc = &sprd_ctrl_data->dma_desc_rx;
            autodma_src = DMA_NOCHANGE;
            autodma_dst = DMA_INCREASE;
            autodma_burst_mod_src = SRC_BURST_MODE_SINGLE;
            autodma_burst_mod_dst = SRC_BURST_MODE_4;
        } else {
            ctrl.dma_desc = &sprd_ctrl_data->dma_desc_tx;
            autodma_src = DMA_INCREASE;
            autodma_dst = DMA_NOCHANGE;
            autodma_burst_mod_src = SRC_BURST_MODE_4;
            autodma_burst_mod_dst = SRC_BURST_MODE_SINGLE;
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

        dsrc = ddst = SPRD_SPI0_PHYS + SPI_TXD;

        sprd_dma_setup_cfg(&ctrl,
                ch_id,
                DMA_NORMAL,
                TRANS_DONE_EN,
                autodma_src, autodma_dst,
                autodma_burst_mod_src, autodma_burst_mod_dst, // SRC_BURST_MODE_SINGLE, SRC_BURST_MODE_SINGLE,
                SPRD_SPI_BURST_SIZE_DEFAULT, // 16 bytes DMA burst size
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

    sprd_ctrl_data->data_max = sprd_ctrl_data->data_width * SPRD_SPI_DMA_BLOCK_MAX;

    // only rx watermark for dma, 0x02 means slave device must send at least 2 bytes
    // when i set 0x01 to SPI_CTL3, Atheros 16 bits data read is wrong,
    // after i change 0x01 to 0x02, everything is ok
#if !SPRD_SPI_RX_WATERMARK_BUG_FIX
    spi_writel(0x02 | (0x00 << 8), SPI_CTL3);
#endif
    spi_writel(0x01 | (0x00 << 8), SPI_CTL6); // tx watermark for dma

    return 0;
}

static inline int sprd_dma_update_spi(u32 sptr, u32 slen, u32 dptr, u32 dlen,
                        struct spi_device *spi, struct sprd_spi_data *sprd_data)
{
    // static int chip_select = -1;
    struct sprd_spi_controller_data *sprd_ctrl_data = spi->controller_data;
    sprd_dma_desc *dma_desc;
    int flag, len, offset;
    int blocks = 1;

#if 1
{
    static int max_in, max_out;
    if (dptr && sprd_data->cspi_trans->len > max_in) {
        max_in = sprd_data->cspi_trans->len;
        printk("===================== %s --> max in =%d %s=====================\n", __func__, max_in, sprd_data->cspi_msg->is_dma_mapped ? "DMA":"");
    }
    if (sprd_data->cspi_trans->len > max_out) {
        max_out = sprd_data->cspi_trans->len;
        printk("===================== %s --> max out =%d %s=====================\n", __func__, max_out, sprd_data->cspi_msg->is_dma_mapped ? "DMA":"");
    }
}
#endif

    offset = sprd_data->cspi_trans_num;

    if (unlikely(sptr && dptr))
        flag = 0xff;        // for both
    else if (likely(sptr))
        flag = 0x02;        // for tx only
    else {
        flag = 0x01;        // for rx only
    }

    if (flag & 0x01) {
        // only rx or tx both
        dlen -= offset;
#if SPRD_SPI_RX_WATERMARK_BUG_FIX
        len = dlen;
        if (len > sprd_data->rt_max) len = sprd_data->rt_max;
    {
        int water_mark;
        water_mark = len > SPRD_SPI_RX_WATERMARK_MAX ? SPRD_SPI_RX_WATERMARK_MAX:len;
        spi_writel(water_mark | (water_mark << 8), SPI_CTL3);
    }
#else
        len = dlen > sprd_ctrl_data->data_max ? sprd_ctrl_data->data_max : dlen;
        if (likely(sprd_ctrl_data->data_width != 3)) {
            blocks = len >> sprd_ctrl_data->data_width_order;
        } else blocks = len / sprd_ctrl_data->data_width;
        // blocks = len / sprd_ctrl_data->data_width; // for 1byte 2bytes 3bytes 4bytes
#endif
        dma_desc = &sprd_ctrl_data->dma_desc_rx;
        dma_desc->tlen = len;
        dma_desc->ddst = dptr; // + offset;

        sprd_spi_update_burst_size();
        sprd_dma_update(DMA_SPI_RX, dma_desc); // use const ch_id value to speed up code exec [luther.ge]
    }

    if (flag & 0x02) {
        if ((flag & 0x01) == 0) len = slen; // only tx
        if (len > sprd_data->rt_max) len = sprd_data->rt_max;
        dma_desc = &sprd_ctrl_data->dma_desc_tx;
        dma_desc->tlen = len;
        dma_desc->dsrc = sptr; // + offset;
#if SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX
        if (sprd_data->cspi_trans->tx_buf == SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX_IGNORE_ADDR)
            dma_desc->dsrc = sptr;
#endif
        sprd_spi_update_burst_size();
        sprd_dma_update(DMA_SPI_TX, dma_desc); // use const ch_id value to speed up code exec [luther.ge]
    }
#if SPRD_SPI_DEBUG
    printk(KERN_WARNING "sending[%02x][%d]...\n", flag, len);
#endif

    /* if (spi->chip_select != chip_select) */ {
        /* when slot0,slot1,...,slotx,同时启动spi_dummy.c的8个测试线程的时,
         * dma传输会时常自动停止,必须在每次启动时加入如下300us延时
         * 8个测试线程才能正常的操作spclk分别为8M和500K的2个slot [luther.ge]
         * 但是如果单独启动slot0的4个线程或者单独启动slot1的4个线程都不会出现该问题
         * 只要8个线程同时启动就会出现上面的问题.
         *
         * 当打开SPRD_SPI_ONLY_RX_AND_TXRX_BUG_FIX之后,该udelay(300us)可以取消[luther.ge]
         *
         */
//        udelay(300);
        // chip_select = spi->chip_select;
    }

    if (flag == 0xff)
        spi_start();
    else if (flag & 0x01)
        spi_start_rx(blocks);
    else spi_start_tx();

//  spi_dma_start(); // Must Enable SPI_DMA_EN last

    return len;
}

static int spi_kthread(void *args)
{
    struct sprd_spi_data *sprd_data = args;

    while (!kthread_should_stop()) {
        down_interruptible(&sprd_data->process_sem);

        down(&sprd_data->process_sem_direct);

        if (sprd_spi_do_transfer(sprd_data) < 0)
            printk(KERN_ERR "error : %s\n", __func__);

        if (!sprd_data->dma_started)
            up(&sprd_data->process_sem_direct);
    }

    return 0;
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

    //clk_spi_mode = (__raw_readl(GR_CLK_DLY) >> 26) & 0x03;
	switch(spi->master->bus_num){
			case 0: clk_spi_mode = (__raw_readl(GR_CLK_DLY) >> 26) & 0x03; break;
			case 1: clk_spi_mode = (__raw_readl(GR_CLK_DLY) >> 30) & 0x03; break;
	}
    switch (clk_spi_mode) {
        case 0: clk_spi = 192   * 1000 * 1000; break;
        case 1: clk_spi = 153.6 * 1000 * 1000; break;
        case 2: clk_spi = 96    * 1000 * 1000; break;
        case 3: clk_spi = 26    * 1000 * 1000; break; 
    }
    //clk_spi_div = (__raw_readl(GR_GEN2) >> 21) & 0x07;
    	switch(spi->master->bus_num){
	    case 0: clk_spi_div = (__raw_readl(GR_GEN2) >> 21) & 0x07; break;
	    case 1: clk_spi_div = (__raw_readl(GR_GEN2) >> 11) & 0x07; break;
	}
    clk_spi /= (clk_spi_div + 1);

    // spi_clk = clk_spi / (2 * (spi_clkd + 1));
    if (spi->max_speed_hz) {
        spi_clkd = clk_spi / (2*spi->max_speed_hz) - 1;
    } else {
        spi_clkd = 0xffff;
    }
    if (spi_clkd < 0) {
        printk(KERN_WARNING "Warning: %s your spclk %d is so big!\n", __func__, spi->max_speed_hz);
        spi_clkd = 0;
    }

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

    return 0;
}

static void spi_complete2(void *arg)
{
    up(arg); //*/ *((int *)arg) = 1;
}

int spi_sync2(struct spi_device *spi, struct spi_message *message)
{
    struct semaphore sem; //*/ volatile int done = 0;
    int status;

    init_MUTEX_LOCKED(&sem); //*/
    message->complete = spi_complete2;
    message->context = (void*)&sem; //*/ (void*)&done;
    status = spi_async(spi, message);
    if (status == 0) {
        down(&sem); //*/ while (!done);
        status = message->status;
    }
    message->context = NULL;
    return status;
}
EXPORT_SYMBOL_GPL(spi_sync2);

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
    sprd_data->tx_buffer = (u8*)sprd_data->buffer;
    sprd_data->rx_buffer = (u8*)sprd_data->buffer + SPRD_SPI_BUFFER_SIZE/2;
    sprd_data->tx_buffer_dma = sprd_data->buffer_dma;
    sprd_data->rx_buffer_dma = sprd_data->buffer_dma + SPRD_SPI_BUFFER_SIZE/2;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++





    sprd_data->rt_max = SPRD_SPI_RX_WATERMARK_MAX; // SPRD_SPI_BUFFER_SIZE/2;




    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if (!sprd_data->buffer)
        goto out_free;

    memset(sprd_data->buffer, 0x55, SPRD_SPI_BUFFER_SIZE);

    spin_lock_init(&sprd_data->lock);
    INIT_LIST_HEAD(&sprd_data->queue);
    sprd_data->pdev = pdev;
    sprd_data->regs = ioremap(regs->start, (regs->end - regs->start) + 1);
    if (!sprd_data->regs)
        goto out_free_buffer;
    sprd_data->irq = irq;

#if SPRD_SPI_DMA_MODE
    //ret = sprd_request_dma(DMA_SPI_TX, sprd_spi_interrupt, master);
    //ret = sprd_request_dma(DMA_SPI_RX, sprd_spi_interrupt, master);
	switch(pdev->id){
			case 0: {
			ret = sprd_request_dma(DMA_SPI_TX, sprd_spi_interrupt, master);
			ret = sprd_request_dma(DMA_SPI_RX, sprd_spi_interrupt, master);
			}break;
			case 1: {
			/*to do*/
			ret = 0;
			}break;
	}    
#else
    ret = request_irq(irq, sprd_spi_interrupt, 0,
                dev_name(&pdev->dev), master);
#endif
    if (ret)
        goto out_unmap_regs;

    /* Initialize the hardware */
	      switch(pdev->id){
              case 0: {
                      __raw_bits_or(GEN0_SPI_EN, GR_GEN0); // Enable SPI module
                      __raw_bits_or(SWRST_SPI_RST, GR_SOFT_RST);
                      msleep(1);
                      __raw_bits_and(~SWRST_SPI_RST, GR_SOFT_RST);
                      spi_writel(0, SPI_INT_EN);
                      // clk source selected to 96M
                      __raw_bits_and(~(0x03 << 26), GR_CLK_DLY);
                      __raw_bits_or(2 << 26, GR_CLK_DLY);
                      /*
                       * clk_spi_div sets to 1, so clk_spi=96M/2=48M,
                       * and clk_spi is SPI_CTL5 interval base clock.[luther.ge]
                       */
                      __raw_bits_and(~(0x07 << 21), GR_GEN2);
                      // __raw_bits_or(1 << 21, GR_GEN2);
              }break;
              case 1: {
                      __raw_bits_or(GEN0_SPI1_EN, GR_GEN0); // Enable SPI module
                      __raw_bits_or(SWRST_SPI1_RST, GR_SOFT_RST);
                      msleep(1);
                      __raw_bits_and(~SWRST_SPI1_RST, GR_SOFT_RST);
                      spi_writel(0, SPI_INT_EN);
                      __raw_bits_and(~(0x03 << 30), GR_CLK_DLY);
                      __raw_bits_or(2 << 30, GR_CLK_DLY);
                      __raw_bits_and(~(0x07 << 11), GR_GEN2);
                      // __raw_bits_or(1 << 11, GR_GEN2);
                      /*turn on ldo for spi1*/
                      //LDO_TurnOnLDO(LDO_BPSDIO1);
              }break;
      }    

    sprd_data->cs_null = 1;

    ret = spi_register_master(master);
    if (ret)
        goto out_reset_hw;

    init_MUTEX(&sprd_data->process_sem_direct);
    init_MUTEX_LOCKED(&sprd_data->process_sem);
    //sprd_data->spi_kthread = kthread_create(spi_kthread, sprd_data, "spi_kthread");
    //if (IS_ERR(sprd_data->spi_kthread)) goto out_reset_hw;

    //wake_up_process(sprd_data->spi_kthread);

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

    //if (!IS_ERR(sprd_data->spi_kthread)) kthread_stop(sprd_data->spi_kthread);

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
