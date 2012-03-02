/* arch/arm/mach-sc8800s/include/mach/dma.h
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_SPRD_DMA_H

#include <linux/list.h>
#include <mach/hardware.h>
#include <mach/regs_ahb.h>
#include <mach/regs_int.h>
#include <asm/io.h>
#include <linux/interrupt.h>

/**----------------------------------------------------------------------------*
**                               Micro Define                                 **
**----------------------------------------------------------------------------*/
#define DMA_REG_BASE                    SPRD_DMA_BASE

//0X00
#define DMA_CFG                         (DMA_REG_BASE + 0x0000)

#define DMA_CHx_EN                      (DMA_REG_BASE + 0x00C0)
#define DMA_CHx_DIS                     (DMA_REG_BASE + 0x00C4)

#define DMA_LINKLIST_EN                 (DMA_REG_BASE + 0x0008)
#define DMA_SOFTLINK_EN                 (DMA_REG_BASE + 0x000C)
#define DMA_SOFTLIST_SIZE               (DMA_REG_BASE + 0x0010)
#define DMA_SOFTLIST_CMD                (DMA_REG_BASE + 0x0014)
#define DMA_SOFTLIST_STS                (DMA_REG_BASE + 0x0018)
#define DMA_SOFTLIST_BASEADDR           (DMA_REG_BASE + 0x001C)
//0X20
#define DMA_PRI_REG0                    (DMA_REG_BASE + 0x0020)
#define DMA_PRI_REG1                    (DMA_REG_BASE + 0x0024)
//0X30
#define DMA_INT_STS                     (DMA_REG_BASE + 0x0030)
#define DMA_INT_RAW                     (DMA_REG_BASE + 0x0034)
//0X40
#define DMA_LISTDONE_INT_EN             (DMA_REG_BASE + 0x0040)
#define DMA_BURST_INT_EN                (DMA_REG_BASE + 0x0044)
#define DMA_TRANSF_INT_EN               (DMA_REG_BASE + 0x0048)
//0X50
#define DMA_LISTDONE_INT_STS            (DMA_REG_BASE + 0x0050)
#define DMA_BURST_INT_STS               (DMA_REG_BASE + 0x0054)
#define DMA_TRANSF_INT_STS              (DMA_REG_BASE + 0x0058)
//0X60
#define DMA_LISTDONE_INT_RAW            (DMA_REG_BASE + 0x0060)
#define DMA_BURST_INT_RAW               (DMA_REG_BASE + 0x0064)
#define DMA_TRANSF_INT_RAW              (DMA_REG_BASE + 0x0068)
//0X70
#define DMA_LISTDONE_INT_CLR            (DMA_REG_BASE + 0x0070)
#define DMA_BURST_INT_CLR               (DMA_REG_BASE + 0x0074)
#define DMA_TRANSF_INT_CLR              (DMA_REG_BASE + 0x0078)
//0X80
#define DMA_SOFT_REQ                    (DMA_REG_BASE + 0x0080)
#define DMA_TRANS_STS                   (DMA_REG_BASE + 0x0084)//for debug
#define DMA_REQ_PEND                    (DMA_REG_BASE + 0x0088)//for debug
//0X90
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

//Chanel x dma contral regisers base address ;
#define DMA_CHx_CTL_BASE                (DMA_REG_BASE + 0x0400)

#define DMA_SOFT0_BASE                  (DMA_REG_BASE + 0x0400)//UID:0x0
#define DMA_UART0_TX_BASE               (DMA_REG_BASE + 0x0420)//UID:0x1
#define DMA_UART0_RX_BASE               (DMA_REG_BASE + 0x0440)//UID:0x2
#define DMA_UART1_TX_BASE               (DMA_REG_BASE + 0x0460)//UID:0x3
#define DMA_UART1_RX_BASE               (DMA_REG_BASE + 0x0480)//UID:0x4
#define DMA_UART2_TX_BASE               (DMA_REG_BASE + 0x04a0)//UID:0x5
#define DMA_UART2_RX_BASE               (DMA_REG_BASE + 0x04c0)//UID:0x6
#define DMA_IIS_TX_BASE                 (DMA_REG_BASE + 0x04e0)//UID:0x7
#define DMA_IIS_RX_BASE                 (DMA_REG_BASE + 0x0500)//UID:0x8
#define DMA_EPT_IN_BASE                 (DMA_REG_BASE + 0x0520)//UID:0x9
#define DMA_EPT_OUT_BASE                (DMA_REG_BASE + 0x0540)//UID:0xA
#define DMA_VB_DA0_BASE                 (DMA_REG_BASE + 0x0560)//UID:0xB
#define DMA_VB_DA1_BASE                 (DMA_REG_BASE + 0x0580)//UID:0xC
#define DMA_VB_AD0_BASE                 (DMA_REG_BASE + 0x05A0)//UID:0xD
#define DMA_VB_AD1_BASE                 (DMA_REG_BASE + 0x05C0)//UID:0xE
#define DMA_SIM0_TX_BASE                (DMA_REG_BASE + 0x05E0)//UID:0xF
#define DMA_SIM0_RX_BASE                (DMA_REG_BASE + 0x0600)//UID:0x10
#define DMA_SIM1_TX_BASE                (DMA_REG_BASE + 0x0620)//UID:0x11
#define DMA_SIM1_RX_BASE                (DMA_REG_BASE + 0x0640)//UID:0x12
#define DMA_SPI_TX_BASE                 (DMA_REG_BASE + 0x0660)//UID:0x13
#define DMA_SPI_RX_BASE                 (DMA_REG_BASE + 0x0680)//UID:0x14
#define DMA_ROT_BASE                    (DMA_REG_BASE + 0x06A0)//UID:0x15
#define DMA_NLC_BASE                    (DMA_REG_BASE + 0x06C0)//UID:0x16
#define DMA_USB_EP0_IN_BASE             (DMA_REG_BASE + 0x06E0)//UID:0x17
#define DMA_USB_EP0_OUT_BASE            (DMA_REG_BASE + 0x0660)//UID:0x18
#define DMA_USB_EP1_BASE                (DMA_REG_BASE + 0x0680)//UID:0x19
#define DMA_USB_EP2_BASE                (DMA_REG_BASE + 0x06A0)//UID:0x1A
#define DMA_USB_EP3_BASE                (DMA_REG_BASE + 0x06C0)//UID:0x1B
#define DMA_USB_EP4_BASE                (DMA_REG_BASE + 0x06E0)//UID:0x1C

#define DMA_CH_NUM                      31//original, why 31?
#define DMA_UID_NUM                     29//original, why 29?
 
#define DMA_UID_MIN                 0 //wong
#define DMA_UID_MAX                 32 //wong
#define DMA_CHN_MIN                 0 //wong
#define DMA_CHN_MAX                 31 //wong
#define DMA_CHN_NUM                 32 //wong
#define DMA_MAX_PRI                     3//wong
#define DMA_MIN_PRI                     0//wong
#define ON                              1//wong
#define OFF                             0//wong
#define DMA_CHN_HARDWARE_START 1
#define DMA_CHN_HARDWARE_END   0x1c 
#define DMA_CHN_SOFTWARE_START 0x1d
#define DMA_CHN_SOFTWARE_END   0x1f
#define SRC_ELEM_POSTM_SHIFT  16
#define CFG_BLK_LEN_MASK      0xffff
#define SRC_ELEM_POSTM_MASK   0xffff
#define DST_ELEM_POSTM_MASK   0xffff
#define SRC_BLK_POSTM_MASK   0x3ffffff
#define DST_BLK_POSTM_MASK   0x3ffffff
#define SOFTLIST_REQ_PTR_MASK  0xffff
#define SOFTLIST_REQ_PTR_SHIFT 16
#define SOFTLIST_CNT_MASK      0xffff
#define DMA_UID_MASK           0x1f
#define DMA_UID_SHIFT_STP      8
#define DMA_UID_UNIT           4
//wong

//Chanel x dma contral regisers address ;
#define DMA_CH_CFG0                     (0x0000)
#define DMA_CH_CFG1                     (0x0004)
#define DMA_CH_SRC_ADDR                 (0x0008)
#define DMA_CH_DEST_ADDR                (0x000c)
#define DMA_CH_LLPTR                    (0x0010)
#define DMA_CH_SDEP                     (0x0014)
#define DMA_CH_SBP                      (0x0018)
#define DMA_CH_DBP                      (0x001c)


//Channel x dma contral regisers address ;
#define DMA_CHx_BASE(x)                 (DMA_CHx_CTL_BASE + 0x20 * x )
#define DMA_CHx_CFG0(x)                 (DMA_CHx_CTL_BASE + 0x20 * x + 0x0000)
#define DMA_CHx_CFG1(x)                 (DMA_CHx_CTL_BASE + 0x20 * x + 0x0004)
#define DMA_CHx_SRC_ADDR(x)             (DMA_CHx_CTL_BASE + 0x20 * x + 0x0008)
#define DMA_CHx_DEST_ADDR(x)            (DMA_CHx_CTL_BASE + 0x20 * x + 0x000c)
#define DMA_CHx_LLPTR(x)                (DMA_CHx_CTL_BASE + 0x20 * x + 0x0010)
#define DMA_CHx_SDEP(x)                 (DMA_CHx_CTL_BASE + 0x20 * x + 0x0014)
#define DMA_CHx_SBP(x)                  (DMA_CHx_CTL_BASE + 0x20 * x + 0x0018)
#define DMA_CHx_DBP(x)                  (DMA_CHx_CTL_BASE + 0x20 * x + 0x001c)


#define DMA_CTL_END                     (DMA_REG_BASE+SPRD_DMA_SIZE)

// DMA user id
#define DMA_SOFT0                       0x00
#define DMA_UID_SOFTWARE                0x00//wong
#define DMA_UART0_TX                    0x01
#define DMA_UART0_RX                    0x02
#define DMA_UART1_TX                    0x03
#define DMA_UART1_RX                    0x04
#define DMA_UART2_TX                    0x05
#define DMA_UART2_RX                    0x06
#define DMA_IIS_TX                      0x07
#define DMA_IIS_RX                      0x08
#define DMA_EPT_IN                      0x09
#define DMA_EPT_OUT                     0x0A
#define DMA_VB_DA0                      0x0B
#define DMA_VB_DA1                      0x0C
#define DMA_VB_AD0                      0x0D
#define DMA_VB_AD1                      0x0E
#define DMA_SIM0_TX                     0x0F
#define DMA_SIM0_RX                     0x10
#define DMA_SIM1_TX                     0x11
#define DMA_SIM1_RX                     0x12
#define DMA_SPI_TX                      0x13
#define DMA_SPI_RX                      0x14
#define DMA_ROT                         0x15
#define DMA_SPI1_TX                     0x16
#define DMA_SPI1_RX                     0x17
#define DMA_DRM_RAW                     0x1D
#define DMA_DRM_CPT                     0x1E
#define DMA_VB_AD                       DMA_VB_AD0
#define DMA_USB_EP1                     DMA_SOFT0
#define DMA_USB_EP3                     DMA_SOFT0

#define DMA_VB_DA0_BIT                  (1 << DMA_VB_DA0)
#define DMA_VB_DA1_BIT                  (1 << DMA_VB_DA1)
#define DMA_VB_AD0_BIT                  (1 << DMA_VB_AD0)
#define DMA_VB_AD1_BIT                  (1 << DMA_VB_AD1)

// DMA_LISTDONE_INT_EN, DMA_BURST_INT_EN, DMA_TRANSF_INT_EN
typedef enum{
    LINKLIST_DONE,
    BLOCK_DONE,
    TRANSACTION_DONE,
}dma_done_type;
//wong

enum {
    INT_NONE = 0x00,
    LLIST_DONE_EN = 0x01,
    BURST_DONE_EN = 0x02,
    TRANS_DONE_EN = 0x04,
};
//enum {
typedef enum {
    // For dma mode
    DMA_WRAP        = (1 << 0),
    DMA_NORMAL      = (1 << 1),
    DMA_LINKLIST    = (1 << 2),
    DMA_SOFTLIST    = (1 << 3),
//};    
}dma_work_mode;
// DMA_CFG
#define DMA_PAUSE_REQ   (1 << 8)
// CH_CFG0
#define DMA_LLEND       (1 << 31)
#define DMA_BIG_ENDIAN  (1 << 28)
#define DMA_LIT_ENDIAN  (0 << 28) // this bit not used by SC8800G2
#define DMA_SDATA_WIDTH8  (0 << 26)
#define DMA_SDATA_WIDTH16 (1 << 26)
#define DMA_SDATA_WIDTH32 (2 << 26)
#define DMA_SDATA_WIDTH_MASK (0x03 << 26)
#define DMA_DDATA_WIDTH8  (0 << 24)
#define DMA_DDATA_WIDTH16 (1 << 24)
#define DMA_DDATA_WIDTH32 (2 << 24)
#define DMA_DDATA_WIDTH_MASK (0x03 << 24)
#define DMA_REQMODE_NORMAL (0 << 22)
#define DMA_REQMODE_TRANS  (1 << 22)
#define DMA_REQMODE_LIST   (2 << 22)
#define DMA_REQMODE_INFIINITE (3 << 22)
#define DMA_SRC_WRAP_EN    (1 << 21)
#define DMA_DST_WRAP_EN    (1 << 20)
#define DMA_NO_AUTO_CLS    (1 << 17)
// DMA_CH_SBP
#define SRC_BURST_MODE_SINGLE (0 << 28)
#define SRC_BURST_MODE_ANY (1 << 28)
#define SRC_BURST_MODE_4   (3 << 28)
#define SRC_BURST_MODE_8   (5 << 28)
#define SRC_BURST_MODE_15  (7 << 28)
#define DMA_BURST_STEP_DIR_BIT (1 << 25)
#define DMA_BURST_STEP_ABS_SIZE_MASK (0x1FFFFFF)


//wong
#define DMA_UN_SWT_MODE (0<<28)
#define DMA_FULL_SWT_MODE (1<<28)
#define DMA_SWT_MODE0 (2<<28)
#define DMA_SWT_MODE1 (3<<28)
#define BURST_MODE_SINGLE (0 << 28)
#define BURST_MODE_ANY (1 << 28)
#define BURST_MODE_4   (3 << 28)
#define BURST_MODE_8   (5 << 28)
#define BURST_MODE_16  (7 << 28)
//wong

#define DMA_SOFT_WAITTIME   0x0f
#define DMA_HARD_WAITTIME   0x0f

#define DMA_INCREASE         ( 0 )
#define DMA_DECREASE         ( 1 )
#define DMA_NOCHANGE         ( 0xff )

#define DMA_CFG_BLOCK_LEN_MAX (0xffff)

typedef struct sprd_dma_desc {
	volatile u32 cfg;
	volatile u32 tlen; // Total transfer length, in bytes 
	volatile u32 dsrc; // Source address. This address value should align to the SRC_DATA_WIDTH 
	volatile u32 ddst; // Destination address. This address value should align to the DES_DATA_WIDTH.
    volatile u32 llptr;// Linked list pointer to the next node address 
    volatile u32 pmod; // POST MODE
    volatile u32 sbm;  // src burst mode
    volatile u32 dbm;  // dst burst mode
} sprd_dma_desc;

//wong
struct sprd_dma_channel_desc{
   u32 cfg_swt_mode_sel;//
   u32 cfg_src_data_width;
   u32 cfg_dst_data_width;
   u32 cfg_req_mode_sel;//request mode
   u32 cfg_src_wrap_en;
   u32 cfg_dst_wrap_en;
   u32 cfg_blk_len;
   u32 total_len;
   u32 src_addr;
   u32 dst_addr;
   u32 llist_ptr;//linklist mode only, must be 8 words boundary
   u32 src_elem_postm;
   u32 dst_elem_postm;
   u32 src_burst_mode;
   u32 src_blk_postm;
   u32 dst_burst_mode;
   u32 dst_blk_postm;
};

struct sprd_dma_softlist_desc{
   u32 softlist_size;
   u32 softlist_cnt_incr;
   u32 softlist_req_ptr;
   u32 softlist_cnt;
   u32 softlist_base_addr;
};

struct sprd_dma_wrap_addr{
   u32 wrap_start_addr;
   u32 wrap_end_addr;	
};
//wong

typedef struct sprd_dma_ctrl {
    int interrupt_type;
    int modes;
    int ch_id;
    sprd_dma_desc *dma_desc;
    dma_addr_t dma_desc_phy;
} sprd_dma_ctrl;

static inline void dma_reg_write(u32 reg, u8 shift, u32 val, u32 mask)
{
    unsigned long flags;
    u32 tmp;
    raw_local_irq_save(flags);
    tmp = __raw_readl(reg);
    tmp &= ~(mask<<shift);
    tmp |= val << shift;
    __raw_writel(tmp, reg);
    raw_local_irq_restore(flags);
}
#define sprd_dma_start(ch_id) \
    __raw_bits_or(1 << ch_id, DMA_CHx_EN) /* Enable DMA channel */

#define sprd_dma_start2(ch_id1, ch_id2) \
    __raw_bits_or((1 << ch_id1) | (1 << ch_id2), DMA_CHx_EN) /* Enable DMA channel */

#ifdef CONFIG_ARCH_SC8800S
#define sprd_dma_stop(ch_id) \
    __raw_bits_and(~(1 << ch_id), DMA_CHx_EN) /* Disable DMA channel */
#define sprd_dma_stop2(ch_id1, ch_id2) \
    __raw_bits_and(~((1 << ch_id1) | (1 << ch_id2)), DMA_CHx_EN) /* Disable DMA channel */
#elif defined(CONFIG_ARCH_SC8810)
#define sprd_dma_stop(ch_id) \
    __raw_bits_or(1 << ch_id, DMA_CHx_DIS) /* Disable DMA channel */
#define sprd_dma_stop2(ch_id1, ch_id2) \
    __raw_bits_or((1 << ch_id1) | (1 << ch_id2), DMA_CHx_DIS) /* Disable DMA channel */
#endif

#define sprd_dma_cfg(ch_id, cfg) \
    __raw_writel(cfg,  DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x00) /* cfg */

#define sprd_dma_tlen(ch_id, tlen) \
    __raw_writel(tlen, DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x04) /* tlen */

#define sprd_dma_dsrc(ch_id, dsrc) \
    __raw_writel(dsrc, DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x08) /* dsrc */

#define sprd_dma_ddst(ch_id, ddst) \
    __raw_writel(ddst, DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x0C) /* ddst */

#define sprd_dma_llptr(ch_id, llptr) \
    __raw_writel(llptr,DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x10) /* llptr */

#define sprd_dma_pmod(ch_id, pmod) \
    __raw_writel(pmod, DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x14) /* pmod */

#define sprd_dma_sbm(ch_id, sbm) \
    __raw_writel(sbm,  DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x18) /* sbm */

#define sprd_dma_dbm(ch_id, dbm) \
    __raw_writel(dbm,  DMA_CHx_CTL_BASE + (ch_id * 0x20) + 0x1C) /* dbm */

static inline void sprd_dma_update(const int ch_id, sprd_dma_desc *dma_desc)
{
    sprd_dma_cfg  (ch_id,dma_desc->cfg);
    sprd_dma_tlen (ch_id,dma_desc->tlen);
    sprd_dma_dsrc (ch_id,dma_desc->dsrc);
    sprd_dma_ddst (ch_id,dma_desc->ddst);
    sprd_dma_llptr(ch_id,dma_desc->llptr);
    sprd_dma_pmod (ch_id,dma_desc->pmod);
    sprd_dma_sbm  (ch_id,dma_desc->sbm);
    sprd_dma_dbm  (ch_id,dma_desc->dbm);
}

int sprd_request_dma(int ch_id, void (*irq_handler)(int, void *), void *data);
void sprd_free_dma(int ch_id);

void sprd_dma_setup(sprd_dma_ctrl *ctrl);
int sprd_irq_handler_ready(int ch_id);

int sprd_dma_request(u32 uid, void (*irq_handler)(int, void *), void *data);//wong
void sprd_dma_free(u32 uid);//wong
void sprd_dma_channel_config(u32 chn, dma_work_mode work_mode, struct sprd_dma_channel_desc *dma_cfg);//wong
void sprd_dma_softlist_config(struct sprd_dma_softlist_desc *softlist_desc);//wong
void sprd_dma_wrap_addr_config(struct sprd_dma_wrap_addr *wrap_addr);//wong
void sprd_dma_set_irq_type(u32 chn, dma_done_type irq_type, u32 on_off);//wong
void sprd_dma_set_chn_pri(u32 chn, u32 pri);//wong
void sprd_dma_channel_start(u32 chn_id);//wong
void sprd_dma_channel_stop(u32 chn_id);//wong
void sprd_dma_check_channel(void);//wong

void sprd_dma_setup_cfg(sprd_dma_ctrl *ctrl,
            int ch_id,
            int dma_modes,
            int interrupt_type,
            int autodma_src,
            int autodma_dst,
            int autodma_burst_mod_src,
            int autodma_burst_mod_dst,
            int burst_size,
            int src_data_width,
            int dst_data_width,
            u32 dsrc,
            u32 ddst,
            u32 tlen);

void sprd_dma_setup_cfg_ext(sprd_dma_ctrl *ctrl,
            int ch_id,
            int dma_modes,
            int interrupt_type,
            int autodma_src,
            int autodma_dst,
            int autodma_burst_mod_src,
            int autodma_burst_mod_dst,
            int burst_size,
            int src_data_width,
            int dst_data_width,
            u32 dsrc,
            u32 ddst,
            u32 tlen);

void sprd_dma_setup_cfg_pmod(sprd_dma_ctrl *ctrl,
            int ch_id,
            int dma_modes,
            int interrupt_type,
            int endian,
            int autodma_src,
            int autodma_dst,
            int src_ele_step,
            int des_ele_step,
            int autodma_burst_mod_src,
            int autodma_burst_mod_dst,
            int burst_size,
            int src_data_width,
            int dst_data_width,
            u32 dsrc,
            u32 ddst,
            u32 tlen);


#endif
