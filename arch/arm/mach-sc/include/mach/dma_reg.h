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

#ifndef __ASM_ARCH_SPRD_DMA_REG_H
#define __ASM_ARCH_SPRD_DMA_REG_H

#include <linux/interrupt.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/dma.h>

/**----------------------------------------------------------------------------*
**                               Micro Define                                 **
**----------------------------------------------------------------------------*/

/*dma r1p0 register offset define start*/
#ifdef DMA_VER_R1P0
#define DMA_REG_BASE                    SPRD_DMA0_BASE

/* 0X00 */
#define DMA_BLK_WAIT                    (DMA_REG_BASE + 0x0000)
#define DMA_CHN_EN_STATUS               (DMA_REG_BASE + 0x0004)
#define DMA_LINKLIST_EN                 (DMA_REG_BASE + 0x0008)
#define DMA_SOFTLINK_EN                 (DMA_REG_BASE + 0x000C)

/* 0X10 */
#define DMA_SOFTLIST_SIZE               (DMA_REG_BASE + 0x0010)
#define DMA_SOFTLIST_CMD                (DMA_REG_BASE + 0x0014)
#define DMA_SOFTLIST_STS                (DMA_REG_BASE + 0x0018)
#define DMA_SOFTLIST_BASEADDR           (DMA_REG_BASE + 0x001C)

/* 0X20 */
#define DMA_PRI_REG0                    (DMA_REG_BASE + 0x0020)
#define DMA_PRI_REG1                    (DMA_REG_BASE + 0x0024)

/* 0X30 */
#define DMA_INT_STS                     (DMA_REG_BASE + 0x0030)
#define DMA_INT_RAW                     (DMA_REG_BASE + 0x0034)

/* 0X40 */
#define DMA_LISTDONE_INT_EN             (DMA_REG_BASE + 0x0040)
#define DMA_BLOCK_INT_EN                (DMA_REG_BASE + 0x0044)
#define DMA_TRANSF_INT_EN               (DMA_REG_BASE + 0x0048)

/* 0X50 */
#define DMA_LISTDONE_INT_STS            (DMA_REG_BASE + 0x0050)
#define DMA_BURST_INT_STS               (DMA_REG_BASE + 0x0054)
#define DMA_TRANSF_INT_STS              (DMA_REG_BASE + 0x0058)

/* 0X60 */
#define DMA_LISTDONE_INT_RAW            (DMA_REG_BASE + 0x0060)
#define DMA_BURST_INT_RAW               (DMA_REG_BASE + 0x0064)
#define DMA_TRANSF_INT_RAW              (DMA_REG_BASE + 0x0068)

/* 0X70 */
#define DMA_LISTDONE_INT_CLR            (DMA_REG_BASE + 0x0070)
#define DMA_BURST_INT_CLR               (DMA_REG_BASE + 0x0074)
#define DMA_TRANSF_INT_CLR              (DMA_REG_BASE + 0x0078)

/* 0X80 */
#define DMA_SOFT_REQ                    (DMA_REG_BASE + 0x0080)
#define DMA_TRANS_STS                   (DMA_REG_BASE + 0x0084)
#define DMA_REQ_PEND                    (DMA_REG_BASE + 0x0088)

/* 0X90 */
#define DMA_WRAP_START                  (DMA_REG_BASE + 0x0090)
#define DMA_WRAP_END                    (DMA_REG_BASE + 0x0094)

#define DMA_CHN_UID_BASE                (DMA_REG_BASE + 0x0098)
#define DMA_CHN_UID0                    (DMA_REG_BASE + 0x0098)
#define DMA_CHN_UID1                    (DMA_REG_BASE + 0x009C)
#define DMA_CHN_UID2                    (DMA_REG_BASE + 0x00A0)
#define DMA_CHN_UID3                    (DMA_REG_BASE + 0x00A4)
#define DMA_CHN_UID4                    (DMA_REG_BASE + 0x00A8)
#define DMA_CHN_UID5                    (DMA_REG_BASE + 0x00AC)
#define DMA_CHN_UID6                    (DMA_REG_BASE + 0x00B0)
#define DMA_CHN_UID7                    (DMA_REG_BASE + 0x00B4)

#define DMA_CHx_EN                      (DMA_REG_BASE + 0x00C0)
#define DMA_CHx_DIS                     (DMA_REG_BASE + 0x00C4)

/* Chanel x dma contral regisers address offset*/
#define DMA_CH_CFG                      (0x0000)
#define DMA_CH_TOTAL_LEN                (0x0004)
#define DMA_CH_SRC_ADDR                 (0x0008)
#define DMA_CH_DEST_ADDR                (0x000c)
#define DMA_CH_LLPTR                    (0x0010)
#define DMA_CH_SDEP                     (0x0014)
#define DMA_CH_SBP                      (0x0018)
#define DMA_CH_DBP                      (0x001c)

/* Channel x dma contral regisers address */
#define DMA_CHx_CTL_BASE                (DMA_REG_BASE + 0x0400)
#define DMA_CHx_BASE(x)                 (DMA_CHx_CTL_BASE + 0x20 * x )
#define DMA_CHx_CFG(x)                  (DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_CFG)
#define DMA_CHx_TOTAL_LEN(x)		(DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_TOTAL_LEN)
#define DMA_CHx_SRC_ADDR(x)             (DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_SRC_ADDR)
#define DMA_CHx_DEST_ADDR(x)            (DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_DEST_ADDR)
#define DMA_CHx_LLPTR(x)                (DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_LLPTR)
#define DMA_CHx_SDEP(x)                 (DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_SDEP)
#define DMA_CHx_SBP(x)                  (DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_SBP)
#define DMA_CHx_DBP(x)                  (DMA_CHx_CTL_BASE + 0x20 * x + DMA_CH_DBP)

/*there is no different between full and std chn is DMA r1p0*/
struct sci_dma_reg {
	u32 cfg;
	u32 total_len;
	u32 src_addr;
	u32 des_addr;
	u32 llist_ptr;
	u32 elem_postm;
	u32 src_blk_postm;
	u32 des_blk_postm;
};

#endif
/*dma r1p0 register offset define end*/

/*dma r3p0 and r4p0 register offset define start*/
#ifdef DMA_VER_R4P0
#define DMA_REG_BASE	SPRD_DMA0_BASE

#define DMA_PAUSE	(DMA_REG_BASE + 0x0000)
#define DMA_FRAG_WAIT	(DMA_REG_BASE + 0x0004)
#define DMA_PEND0_EN	(DMA_REG_BASE + 0x0008)
#define DMA_PEND1_EN	(DMA_REG_BASE + 0x000C)
#define DMA_INT_RAW_STS	(DMA_REG_BASE + 0x0010)
#define DMA_INT_MSK_STS	(DMA_REG_BASE + 0x0014)
#define DMA_REQ_STS	(DMA_REG_BASE + 0x0018)
#define DMA_EN_STS	(DMA_REG_BASE + 0x001C)
#define DMA_DEGUG_STS	(DMA_REG_BASE + 0x0020)
#define DMA_ARB_SEL_STS	(DMA_REG_BASE + 0x0024)

#define DMA_CHx_BASE(x)	(DMA_REG_BASE + 0x1000 + 0x40 * (x))

#define DMA_CHN_PAUSE(x)	(DMA_CHx_BASE(x) + 0x0000)
#define DMA_CHN_REQ(x)		(DMA_CHx_BASE(x) + 0x0004)
#define DMA_CHN_CFG(x)		(DMA_CHx_BASE(x) + 0x0008)
#define DMA_CHN_INT(x)		(DMA_CHx_BASE(x) + 0x000C)
#define DMA_CHN_SRC_ADR(x)	(DMA_CHx_BASE(x) + 0x0010)
#define DMA_CHN_DES_ADR(x)	(DMA_CHx_BASE(x) + 0x0014)
#define DMA_CHN_FRAG_LEN(x)	(DMA_CHx_BASE(x) + 0x0018)
#define DMA_CHN_BLK_LEN(x)	(DMA_CHx_BASE(x) + 0x001C)

#define DMA_CHN_TRSC_LEN(x)	(DMA_CHx_BASE(x) + 0x0020)
#define DMA_CHN_TRSF_STEP(x)	(DMA_CHx_BASE(x) + 0x0024)
#define DMA_CHN_WRAP_PTR(x)	(DMA_CHx_BASE(x) + 0x0028)
#define DMA_CHN_WRAP_TO(x)	(DMA_CHx_BASE(x) + 0x002C)
#define DMA_CHN_LLIST_PRT(x)	(DMA_CHx_BASE(x) + 0x0030)
#define DMA_CHN_FRAP_STEP(x)	(DMA_CHx_BASE(x) + 0x0034)
#define DMA_CHN_SRC_BLK_STEP(x)	(DMA_CHx_BASE(x) + 0x0038)
#define DMA_CHN_DES_BLK_STEP(x)	(DMA_CHx_BASE(x) + 0x003C)
#define DMA_REQ_CID(uid)	(DMA_REG_BASE + 0x2000 + 0x4 * ((uid) -1))

struct sci_dma_reg {
	u32 pause;
	u32 req;
	u32 cfg;
	u32 intc;
	u32 src_addr;
	u32 des_addr;
	u32 frg_len;
	u32 blk_len;
	union {
		struct {
			u32 trs_len;
			u32 wrap_ptr;
			u32 wrap_to;
			u32 llist_ptr;
			u32 frg_step;
			u32 src_step;
			u32 des_step;
		};
		int dummy[0];
	};
};
#endif
/*dma r3p0 and r4p0 register offset define end*/

#define DMA_UID_MASK                    0x1f
#define DMA_UID_SHIFT_STP               8
#define DMA_UID_UNIT                    4

/* DMA_CFG */
#define DMA_PAUSE_REQ                  (1 << 8)

/* CH_CFG0 */
#define DMA_LLEND                      (1 << 31)
#define DMA_BIG_ENDIAN                 (1 << 28)
#define DMA_LIT_ENDIAN                 (0 << 28)	/* this bit not used by SC8800G2 */
#define DMA_SDATA_WIDTH8               (0 << 26)
#define DMA_SDATA_WIDTH16              (1 << 26)
#define DMA_SDATA_WIDTH32              (2 << 26)
#define DMA_SDATA_WIDTH_MASK           (0x03 << 26)
#define DMA_DDATA_WIDTH8               (0 << 24)
#define DMA_DDATA_WIDTH16              (1 << 24)
#define DMA_DDATA_WIDTH32              (2 << 24)
#define DMA_DDATA_WIDTH_MASK           (0x03 << 24)
#define DMA_REQMODE_NORMAL             (0 << 22)
#define DMA_REQMODE_TRANS              (1 << 22)
#define DMA_REQMODE_LIST               (2 << 22)
#define DMA_REQMODE_INFIINITE          (3 << 22)
#define DMA_SRC_WRAP_EN                (1 << 21)
#define DMA_DST_WRAP_EN                (1 << 20)
#define DMA_NO_AUTO_CLS                (1 << 17)

/* DMA_CH_SBP */
#define SRC_BURST_MODE_SINGLE          (0 << 28)
#define SRC_BURST_MODE_ANY             (1 << 28)
#define SRC_BURST_MODE_4               (3 << 28)
#define SRC_BURST_MODE_8               (5 << 28)
#define SRC_BURST_MODE_15              (7 << 28)
#define DMA_BURST_STEP_DIR_BIT         (1 << 25)
#define DMA_BURST_STEP_ABS_SIZE_MASK   (0x1FFFFFF)

#define DMA_UN_SWT_MODE                (0<<28)
#define DMA_FULL_SWT_MODE              (1<<28)
#define DMA_SWT_MODE0                  (2<<28)
#define DMA_SWT_MODE1                  (3<<28)
#define BURST_MODE_SINGLE              (0 << 28)
#define BURST_MODE_ANY                 (1 << 28)
#define BURST_MODE_4                   (3 << 28)
#define BURST_MODE_8                   (5 << 28)
#define BURST_MODE_16                  (7 << 28)

#define DMA_SOFT_WAITTIME              0x0f
#define DMA_HARD_WAITTIME              0x0f

#define DMA_INCREASE                   (0)
#define DMA_DECREASE                   (1)
#define DMA_NOCHANGE                   (0xff)

#define DMA_CFG_BLOCK_LEN_MAX           (0xffff)
#define SRC_ELEM_POSTM_SHIFT            16
#define CFG_BLK_LEN_MASK                0xffff
#define SRC_ELEM_POSTM_MASK             0xffff
#define DST_ELEM_POSTM_MASK             0xffff
#define SRC_BLK_POSTM_MASK              0x3ffffff
#define DST_BLK_POSTM_MASK              0x3ffffff
#define SOFTLIST_REQ_PTR_MASK           0xffff
#define SOFTLIST_REQ_PTR_SHIFT          16
#define SOFTLIST_CNT_MASK               0xffff

#define DMA_DEFAULT_PRIO DMA_PRI_1
#define DMA_DEFAULT_REQ_MODE FRAG_REQ_MODE

#endif
