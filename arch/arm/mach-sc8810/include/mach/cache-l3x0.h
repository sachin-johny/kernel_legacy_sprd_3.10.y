/*
 * arch/arm/include/asm/hardware/cache-l2x0.h
 *
 * Copyright (C) 2007 ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __ASM_ARM_HARDWARE_L2X0_H
#define __ASM_ARM_HARDWARE_L2X0_H

#define L2X0_CACHE_ID			0x000
#define L2X0_CACHE_TYPE			0x004
#define L2X0_CTRL			0x100
#define L2X0_AUX_CTRL			0x104
#define L2X0_TAG_LATENCY_CTRL		0x108
#define L2X0_DATA_LATENCY_CTRL		0x10C
#define L2X0_EVENT_CNT_CTRL		0x200
#define L2X0_EVENT_CNT1_CFG		0x204
#define L2X0_EVENT_CNT0_CFG		0x208
#define L2X0_EVENT_CNT1_VAL		0x20C
#define L2X0_EVENT_CNT0_VAL		0x210
#define L2X0_INTR_MASK			0x214
#define L2X0_MASKED_INTR_STAT		0x218
#define L2X0_RAW_INTR_STAT		0x21C
#define L2X0_INTR_CLEAR			0x220
#define L2X0_CACHE_SYNC			0x730
#define L2X0_DUMMY_REG			0x740
#define L2X0_INV_LINE_PA		0x770
#define L2X0_INV_WAY			0x77C
#define L2X0_CLEAN_LINE_PA		0x7B0
#define L2X0_CLEAN_LINE_IDX		0x7B8
#define L2X0_CLEAN_WAY			0x7BC
#define L2X0_CLEAN_INV_LINE_PA		0x7F0
#define L2X0_CLEAN_INV_LINE_IDX		0x7F8
#define L2X0_CLEAN_INV_WAY		0x7FC
#define L2X0_LOCKDOWN_WAY_D		0x900
#define L2X0_LOCKDOWN_WAY_I		0x904
#define L2X0_TEST_OPERATION		0xF00
#define L2X0_LINE_DATA			0xF10
#define L2X0_LINE_TAG			0xF30
#define L2X0_DEBUG_CTRL			0xF40
#define L2X0_PREFETCH_CTRL		0xF60
#define L2X0_POWER_CTRL			0xF80
#define   L2X0_DYNAMIC_CLK_GATING_EN	(1 << 1)
#define   L2X0_STNDBY_MODE_EN		(1 << 0)

/* Registers shifts and masks */
#define L2X0_CACHE_ID_PART_MASK		(0xf << 6)
#define L2X0_CACHE_ID_PART_L210		(1 << 6)
#define L2X0_CACHE_ID_PART_L310		(3 << 6)

#define L2X0_AUX_CTRL_MASK			0xc0000fff
#define L2X0_AUX_CTRL_ASSOCIATIVITY_SHIFT	16
#define L2X0_AUX_CTRL_WAY_SIZE_SHIFT		17
#define L2X0_AUX_CTRL_WAY_SIZE_MASK		(0x3 << 17)
#define L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT	22
#define L2X0_AUX_CTRL_NS_LOCKDOWN_SHIFT		26
#define L2X0_AUX_CTRL_NS_INT_CTRL_SHIFT		27
#define L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT	28
#define L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT	29
#define L2X0_AUX_CTRL_EARLY_BRESP_SHIFT		30

#ifndef __ASSEMBLY__
extern int __init sp_init_l3x0(void);
#endif

//config begin
//config1
#define CACHE_EARLY_BRESP_ENABLE ((0 << 30)
#define CACHE_I_P_ENABLE ((0) << 29)
#define CACHE_D_P_ENABLE ((1) << 28)
#define CACHE_N_S_ACCESS ((1) << 27)
#define CACHE_N_L_ENABLE ((1) << 26)
#define CACHE_REPLACE_POLICY  ((1) << 25)//1:robin, 0:preudo
#define CACHE_FORCE_W_A  (((0) & 0x3) << 23)

#define CACHE_S_O_ENABLE ((1) << 22)
#define CACHE_PARITY_ENABLE ((0) << 21)
#define CACHE_EVENT_ENABLE ((1) << 20)
#define CACHE_WAY_SIZE (((2) & 0x3) << 17))
#define CACHE_ASSOCI  ((0) << 16)
#define CACHE_SHARED_INV_ENABLE  ((0) << 13)

#define CACHE_EC_CONFIG ((1) << 12)
#define CACHE_STORE_BUFFER_LIMI_ENABLE ((1) << 11)
#define CACHE_HIGH_PRIORITY_SO_DEV ((0) << 10)
#define CACHE_FULL_LINE_ZERO_ENABLE ((0)<<0)

//config2
#define TAG_RAM_WRITE_ACCESS_LATENCY (((0x1) & 0x7) << 8)
#define TAG_RAM_READ_ACCESS_LATENCY (((0x1) & 0x7) << 4)
#define TAG_RAM_SETUP_ACCESS_LATENCY (((0x1) & 0x7) << 0)

#define DATA_RAM_WRITE_ACCESS_LATENCY (((0x1) & 0x7) << 8)
#define DATA_RAM_READ_ACCESS_LATENCY (((0x1) & 0x7) << 4)
#define DATA_RAM_SETUP_ACCESS_LATENCY (((0x1) & 0x7) << 0)

#define AUX_VALUE (CACHE_EARLY_BRESP_ENABLE | CACHE_I_P_ENABLE | CACHE_D_P_ENABLE |CACHE_N_S_ACCESS \
	| CACHE_N_L_ENABLE | CACHE_REPLACE_POLICY | CACHE_FORCE_W_A | CACHE_S_O_ENABLE | \
	CACHE_PARITY_ENABLE | CACHE_EVENT_ENABLE | CACHE_WAY_SIZE | \
	CACHE_ASSOCI | CACHE_SHARED_INV_ENABLE | CACHE_EC_CONFIG | \
	CACHE_STORE_BUFFER_LIMI_ENABLE | CACHE_HIGH_PRIORITY_SO_DEV | CACHE_FULL_LINE_ZERO_ENABLE)

#define PL310_CACHE_AUX_VALUE (AUX_VALUE)
#define PL310_CACHE_AUX_MASK (0xffffffff)
#define PL310_DATA_RAM_LATENCY (DATA_RAM_WRITE_ACCESS_LATENCY | DATA_RAM_READ_ACCESS_LATENCY | DATA_RAM_SETUP_ACCESS_LATENCY)
#define PL310_TAG_RAM_LATENCY (TAG_RAM_WRITE_ACCESS_LATENCY | TAG_RAM_READ_ACCESS_LATENCY | TAG_RAM_SETUP_ACCESS_LATENCY)
#endif

