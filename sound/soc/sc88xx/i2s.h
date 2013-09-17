/*
 * sound/soc/sc88xx/i2s.h
 *
 * SPRD SoC CPU-DAI -- SpreadTrum SOC pure DAI.
 *
 * Copyright (C) 2012 SpreadTrum Ltd.
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
#ifndef __I2S_H
#define __I2S_H

#define I2S_VERSION	"i2s.r0p0"

/* bus_type */
#define I2S_BUS 0
#define PCM_BUS 1

/* byte_per_chan */
#define I2S_BPCH_8 0
#define I2S_BPCH_16 1
#define I2S_BPCH_32 3

/* mode */
#define I2S_MASTER 0
#define I2S_SLAVER 1

/* lsb */
#define I2S_MSB 0
#define I2S_LSB 1

/* rtx_mode */
#define I2S_RTX_DIS 0
#define I2S_RX_MODE 1
#define I2S_TX_MODE 2
#define I2S_RTX_MODE 3

/* sync_mode */
#define I2S_LRCK 0
#define I2S_SYNC 1

/* lrck_inv */
#define I2S_L_LEFT 0
#define I2S_L_RIGTH 1

/* clk_inv */
#define I2S_CLK_N 0
#define I2S_CLK_R 1

/* i2s_bus_mode */
#define I2S_MSBJUSTFIED 0
#define I2S_COMPATIBLE 1

/* pcm_bus_mode */
#define I2S_LONG_FRAME 0
#define I2S_SHORT_FRAME 1

struct i2s_config {
	u32 fs;
	u32 slave_timeout;
	u32 bus_type:1;
	u32 byte_per_chan:2;
	u32 mode:1;
	u32 lsb:1;
	u32 rtx_mode:2;
	u32 sync_mode:1;
	u32 lrck_inv:1;
	u32 clk_inv:2;
	u32 i2s_bus_mode:1;
	u32 pcm_bus_mode:1;
	u32 pcm_slot:3;
	u16 pcm_cycle;
	u16 tx_watermark;
	u16 rx_watermark;
};

struct i2s_from_fm_to_vbc_config {
	unsigned char *area;	/* virtual pointer */
	dma_addr_t addr;	/* physical address */
	size_t bytes;		/* buffer size in bytes */
	sprd_dma_desc *dma_desc;
	dma_addr_t dma_desc_phys;
	int i2s_source_clk; /* must set when i2s work on MASTER mode */
	struct i2s_config *config;
	void (*vbc_transfer_enable)(int on);
};

/* -------------------------- */

#define I2S_FIFO_DEPTH 32

int i2s_from_fm_to_vbc_config(int port, struct i2s_from_fm_to_vbc_config *cfg);
int i2s_from_fm_to_vbc_enable(int port);
int i2s_from_fm_to_vbc_disable(int port);

#endif /* __I2S_H */
