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
#define DMA_CHx_EN                      (DMA_REG_BASE + 0x0004)
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
#define DMA_UART3_TX_BASE               (DMA_REG_BASE + 0x04e0)//UID:0x7
#define DMA_UART3_RX_BASE               (DMA_REG_BASE + 0x0500)//UID:0x8
#define DMA_ENCRP_RAW_BASE              (DMA_REG_BASE + 0x0520)//UID:0x9
#define DMA_ENCRP_CPT_BASE              (DMA_REG_BASE + 0x0540)//UID:0xA
#define DMA_VB_DA0_BASE                 (DMA_REG_BASE + 0x0560)//UID:0xB
#define DMA_VB_DA1_BASE                 (DMA_REG_BASE + 0x0580)//UID:0xC
#define DMA_VB_AD0_BASE                 (DMA_REG_BASE + 0x05A0)//UID:0xD
#define DMA_VB_AD1_BASE                 (DMA_REG_BASE + 0x05C0)//UID:0xE
#define DMA_SIM0_TX_BASE                (DMA_REG_BASE + 0x05E0)//UID:0xF
#define DMA_SIM0_RX_BASE                (DMA_REG_BASE + 0x0600)//UID:0x10
#define DMA_SIM1_TX_BASE                (DMA_REG_BASE + 0x0620)//UID:0x11
#define DMA_SIM1_RX_BASE                (DMA_REG_BASE + 0x0640)//UID:0x12
#define DMA_DRM_RAW_BASE                (DMA_REG_BASE + 0x0660)//UID:0x13
#define DMA_DRM_CPT_BASE                (DMA_REG_BASE + 0x0680)//UID:0x14
#define DMA_ROT_BASE                    (DMA_REG_BASE + 0x06A0)//UID:0x15
#define DMA_NLC_BASE                    (DMA_REG_BASE + 0x06C0)//UID:0x16
#define DMA_USB_EP0_IN_BASE             (DMA_REG_BASE + 0x06E0)//UID:0x17
#define DMA_USB_EP0_OUT_BASE            (DMA_REG_BASE + 0x0660)//UID:0x18
#define DMA_USB_EP1_BASE                (DMA_REG_BASE + 0x0680)//UID:0x19
#define DMA_USB_EP2_BASE                (DMA_REG_BASE + 0x06A0)//UID:0x1A
#define DMA_USB_EP3_BASE                (DMA_REG_BASE + 0x06C0)//UID:0x1B
#define DMA_USB_EP4_BASE                (DMA_REG_BASE + 0x06E0)//UID:0x1C

#define DMA_CH_NUM                      31
#define DMA_UID_NUM                     29

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

#define DMA_SOFT0                       0
#define DMA_UART0_TX                    1
#define DMA_UART0_RX                    2
#define DMA_UART1_TX                    3
#define DMA_UART1_RX                    4
#define DMA_UART2_TX                    5
#define DMA_UART2_RX                    6
#define DMA_UART3_TX                    7
#define DMA_UART3_RX                    8
#define DMA_ENCRP_RAW                   9
#define DMA_ENCRP_CPT                   0xa
#define DMA_VB_DA0                      0xb
#define DMA_VB_DA1                      0xc
#define DMA_VB_AD0                      0xd
#define DMA_VB_AD1                      0xe
#define DMA_SIM0_TX                     0xf
#define DMA_SIM0_RX                     0x10
#define DMA_SIM1_TX                     0x11
#define DMA_SIM1_RX                     0x12
#define DMA_DRM_RAW                     0x13
#define DMA_DRM_CPT                     0x14
#define DMA_ROT                         0x15
#define DMA_NLC                         0x16
#define DMA_USB_EP0_IN                  0x17
#define DMA_USB_EP0_OUT                 0x18
#define DMA_USB_EP1                     0x19
#define DMA_USB_EP2                     0x1a
#define DMA_USB_EP3                     0x1b
#define DMA_USB_EP4                     0x1c


#define DMA_SOFT0_BIT                   BIT_0
#define DMA_UART0_TX_BIT                BIT_1
#define DMA_UART0_RX_BIT                BIT_2
#define DMA_UART1_TX_BIT                BIT_3
#define DMA_UART1_RX_BIT                BIT_4
#define DMA_UART2_TX_BIT                BIT_5
#define DMA_UART2_RX_BIT                BIT_6
#define DMA_UART3_TX_BIT                BIT_7
#define DMA_UART3_RX_BIT                BIT_8
#define DMA_ENCRP_RAW_BIT               BIT_9
#define DMA_ENCRP_CPT_BIT               BIT_10
#define DMA_VB_DA0_BIT                  BIT_11
#define DMA_VB_DA1_BIT                  BIT_12

#ifndef CHIP_VER_8800H5
#define DMA_VB_AD0_BIT                  BIT_13
#define DMA_VB_AD1_BIT                  BIT_14
#else
#define DMA_VB_AD                       BIT_13
#endif

#define DMA_SIM0_TX_BIT                 BIT_15
#define DMA_SIM0_RX_BIT                 BIT_16
#define DMA_SIM1_TX_BIT                 BIT_17
#define DMA_SIM2_RX_BIT                 BIT_18
#define DMA_DRM_RAW_BIT                 BIT_19
#define DMA_DRM_CPT_BIT                 BIT_20
#define DMA_ROT_BIT                     BIT_21
#define DMA_NLC_BIT                     BIT_22
#define DMA_USB_EP0_IN_BIT              BIT_23
#define DMA_USB_EP0_OUT_BIT             BIT_24
#define DMA_USB_EP1_BIT                 BIT_25
#define DMA_USB_EP2_BIT                 BIT_26
#define DMA_USB_EP3_BIT                 BIT_27
#define DMA_USB_EP4_BIT                 BIT_28

// DMA_LISTDONE_INT_EN, DMA_BURST_INT_EN, DMA_TRANSF_INT_EN
enum {
    INT_NONE = 0x00,
    LLIST_DONE_EN = 0x01,
    BURST_DONE_EN = 0x02,
    TRANS_DONE_EN = 0x04,
};
enum {
    // For dma mode
    DMA_WRAP        = (1 << 0),
    DMA_NORMAL      = (1 << 1),
    DMA_LINKLIST    = (1 << 2),
    DMA_SOFTLIST    = (1 << 3),
    
};
// DMA_CFG
#define DMA_PAUSE_REQ   (1 << 8)
// CH_CFG0
#define DMA_LLEND       (1 << 31)
#define DMA_BIG_ENDIAN  (0 << 30)
#define DMA_LIT_ENDIAN  (1 << 30)
#define DMA_SDATA_WIDTH8  (0 << 26)
#define DMA_SDATA_WIDTH16 (1 << 26)
#define DMA_SDATA_WIDTH32 (2 << 26)
#define DMA_DDATA_WIDTH8  (0 << 24)
#define DMA_DDATA_WIDTH16 (1 << 24)
#define DMA_DDATA_WIDTH32 (2 << 24)
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

#define DMA_SOFT_WAITTIME   0x0f
#define DMA_HARD_WAITTIME   0x0f

#define DMA_INCREASE         ( 0 )
#define DMA_DECREASE         ( 1 )
#define DMA_NOCHANGE         ( 0xff )

typedef struct sc88xx_dma_desc {
	volatile u32 cfg;
	volatile u32 tlen; // Total transfer length, in bytes 
	volatile u32 dsrc; // Source address. This address value should align to the SRC_DATA_WIDTH 
	volatile u32 ddst; // Destination address. This address value should align to the DES_DATA_WIDTH.
    volatile u32 llptr;// Linked list pointer to the next node address 
    volatile u32 pmod; // POST MODE
    volatile u32 sbm;  // src burst mode
    volatile u32 dbm;  // dst burst mode
} sc88xx_dma_desc;

typedef struct sc88xx_dma_ctrl {
    int interrupt_type;
    int modes;
    int ch_id;
    sc88xx_dma_desc *dma_desc;
    dma_addr_t dma_desc_phy;
} sc88xx_dma_ctrl;

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

int sc88xx_request_dma(int ch_id, void (*irq_handler)(int, void *), void *data);
void sc88xx_free_dma(int ch_id);
void sc88xx_dma_setup(sc88xx_dma_ctrl *ctrl);
int sc88xx_irq_handler_ready(int ch_id);

#endif
