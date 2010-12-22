/* linux/drivers/mtd/nand/sprd_nand.c
 *
 * Copyright (c) 2010 Spreadtrun.
 *
 * Spreadtrun NAND driver

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <mach/regs_ahb.h>
#include <mach/mfp.h>
#include <mach/dma.h>
#ifdef CONFIG_ARCH_SC8800G
#include "regs_nfc_sc8800g.h"
#endif
#ifdef CONFIG_ARCH_SC8800S
#include "regs_nfc_sc8800s.h"
#endif

#define NF_PARA_24M        	(0x7ac05)      //trwl = 0  trwh = 0
#define NF_PARA_48M        	(0x7ac15)      //trwl = 1  trwh = 0
#define NF_PARA_72M        	(0x7ac25)      //trwl = 2  trwh = 0
#define NF_PARA_96M        	(0x7ac25)      //trwl = 2  trwh = 0
#define NF_PARA_100M    	(0x7ad25)      //trwl = 2  trwh = 0
#define NF_PARA_DEFAULT		(0x7ad77)
#define ORIGINAL_NAND_TIMING	(0x7bd07)
//#define ORIGINAL_NAND_TIMING	(0x7ad05)
//#define ORIGINAL_NAND_TIMING	(0x5ad05)
//#define ORIGINAL_NAND_TIMING	(0x38d05)
#define NF_TIMEOUT_VAL 		(0x1000000)

#define PAGE_SIZE_S         512
#define SPARE_SIZE_S        16
#define PAGE_SIZE_L         2048
#define SPARE_SIZE_L        64

#define USE_DMA_MODE			1
#define DMA_NLC                         (0)
wait_queue_head_t	wait_queue;
sprd_dma_ctrl ctrl;
sprd_dma_desc dma_desc;
static unsigned long nand_func_cfg8[] = {
#ifdef CONFIG_ARCH_SC8800G
	MFP_CFG_X(NFWPN, AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFRB,  AF0, DS1, F_PULL_UP,   S_PULL_UP,   IO_Z),
	MFP_CFG_X(NFCLE, AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFALE, AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFCEN, AF0, DS1, F_PULL_NONE, S_PULL_UP,   IO_Z),
	MFP_CFG_X(NFWEN, AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFREN, AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD0,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD1,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD2,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD3,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD4,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD5,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD6,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD7,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
#endif
#ifdef CONFIG_ARCH_SC8800S
	MFP_CFG_X(NFWPN, AF0, DS3, PULL_NONE, IO_OE),
	MFP_CFG_X(NFRB,  AF0, DS3, PULL_UP,   IO_Z),
	MFP_CFG_X(NFCLE, AF0, DS3, PULL_NONE, IO_OE),
	MFP_CFG_X(NFALE, AF0, DS3, PULL_NONE, IO_OE),
	MFP_CFG_X(NFCEN, AF0, DS3, PULL_NONE, IO_OE),
	MFP_CFG_X(NFWEN, AF0, DS3, PULL_NONE, IO_OE),
	MFP_CFG_X(NFREN, AF0, DS3, PULL_NONE, IO_OE),
	MFP_CFG_X(NFD0,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(NFD1,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(NFD2,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(NFD3,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(NFD4,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(NFD5,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(NFD6,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(NFD7,  AF0, DS3, PULL_NONE, IO_IE),
#endif
};

static unsigned long nand_func_cfg16[] = {
#ifdef CONFIG_ARCH_SC8800G
	MFP_CFG_X(NFD8,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD9,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD10,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD11,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD12,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD13,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD14,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD15,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
#endif
#ifdef CONFIG_ARCH_SC8800S
	MFP_CFG_X(NFD8,  AF0, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(LCD_D9,  AF1, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(LCD_D10, AF1, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(LCD_D11, AF1, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(LCD_D12, AF1, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(LCD_D13, AF1, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(LCD_D14, AF1, DS3, PULL_NONE, IO_IE),
	MFP_CFG_X(LCD_D15, AF1, DS3, PULL_NONE, IO_IE),
#endif
};

static void sprd_config_nand_pins8(void)
{
	sprd_mfp_config(nand_func_cfg8, ARRAY_SIZE(nand_func_cfg8));	
}

static void sprd_config_nand_pins16(void)
{
	sprd_mfp_config(nand_func_cfg16, ARRAY_SIZE(nand_func_cfg16));
}

struct sprd_platform_nand {
	/* timing information for nand flash controller */
	int	acs;
	int 	ach;
	int	rwl;
	int	rwh;
	int	rr;
	int	acr;
	int	ceh;
};

struct sprd_nand_address {
	int column;
	int row;
	int colflag;
	int rowflag;
};

struct sprd_nand_info {
	unsigned long			phys_base;
	struct sprd_platform_nand	*platform;
	struct clk			*clk;
	struct mtd_info			mtd;
	struct platform_device		*pdev;
#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
	struct notifier_block	freq_policy;
#endif
};

typedef enum {
	NO_OP,
	WRITE_OP,
	READ_OP,
} sprd_nand_wr_mode_t;

typedef enum {
	NO_AREA,
	DATA_AREA,
	OOB_AREA,
	DATA_OOB_AREA,
} sprd_nand_area_mode_t;

#ifdef USE_DMA_MODE
typedef enum {
	NO_PORT,
	DATA_PORT,
	OOB_PORT,
	IDSTATUS_PORT,	/* for sprd_nand_read_byte */
	OOBWORD_PORT,	/*reserved for sprd_nand_read_word */
} sprd_nand_port_mode_t;
static sprd_nand_port_mode_t sprd_port_mode = NO_PORT;
#else
static unsigned char io_wr_port[NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE];
#endif

static struct mtd_info *sprd_mtd = NULL;
static unsigned long g_cmdsetting = 0;
static sprd_nand_wr_mode_t sprd_wr_mode = NO_OP;
static sprd_nand_area_mode_t sprd_area_mode = NO_AREA;
static unsigned long nand_flash_id = 0;
static struct sprd_nand_address sprd_colrow_addr = {0, 0, 0, 0};
static nand_ecc_modes_t sprd_ecc_mode = NAND_ECC_NONE;
#ifdef CONFIG_MACH_G2PHONE
static unsigned long g_buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
static unsigned long g_addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
#else
static unsigned long g_buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
static unsigned long g_addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
#endif

#ifdef CONFIG_CPU_FREQ
static int nand_freq_transition(struct notifier_block *nb, unsigned long val, void *data);
static int nand_freq_policy(struct notifier_block *nb, unsigned long val, void *data);
static int min_freq, max_freq;
typedef enum
{
	NFM_VOLTAGE_1800MV = 0,
	NFM_VOLTAGE_2650MV,
	NFM_VOLTAGE_2800MV,
	NFM_VOLTAGE_3000MV,
} nfm_voltage_e;
#endif

static struct sprd_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev->dev.platform_data;
}

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

#define NF_RESET                0xFF
#define NF_READ_STATUS  	0x70
#define NF_READ_ID              0x90

static int nfc_wait_command_finish(void)
{
	unsigned long nfc_cmd = REG_NFC_CMD;
	unsigned long counter = 0;
	
	while ((nfc_cmd & (0x1 << 31)) && (counter < NF_TIMEOUT_VAL)) {
		nfc_cmd = REG_NFC_CMD;
		counter ++;
	}
	
	if (NF_TIMEOUT_VAL == counter)
		return 2;
	
	return 0;
}

static void nfc_reset(void)
{
        unsigned long cmd = NF_RESET | (0x1 << 31);

        REG_NFC_CMD = cmd;
        nfc_wait_command_finish();
}

static void nfc_read_status(void)
{
	unsigned long status = 0;
        unsigned long cmd = NF_READ_STATUS | (0x1 << 31);

        REG_NFC_CMD = cmd;
        nfc_wait_command_finish();
        status = REG_NFC_IDSTATUS;
}

static unsigned long nfc_read_id(void)
{
        unsigned long id = 0;
        unsigned long cmd =  NF_READ_ID | (0x1 << 31);

        REG_NFC_CMD = cmd;
        nfc_wait_command_finish();
        id = REG_NFC_IDSTATUS;

	return id;
}

/* 
*  1 : I/O 16bit
*  0 : I/O 8bit
*/
static unsigned long nand_bit_width(unsigned long id)
{
	unsigned long bw = (id & 0x40000000) >> 30;

	return bw;
}

/* ahb_clk is frequency of ahb in MHz */
static void set_nfc_param(unsigned long ahb_clk)
{
	unsigned long flags, tmp, cycle;

	nfc_wait_command_finish();
	local_irq_save(flags);

	switch (ahb_clk) {
	case 24:
        	REG_NFC_PARA = NF_PARA_24M;
        break;

        case 48:
        	REG_NFC_PARA = NF_PARA_48M;
        break;

        case 72:
        	REG_NFC_PARA = NF_PARA_72M;
        break;

        case 96:
        	REG_NFC_PARA = NF_PARA_96M;
        break;

	/*case 100:
		REG_NFC_PARA = NF_PARA_100M;
	break;*/

        default:
		ahb_clk = ahb_clk * 1000000;
		tmp = (1000000000 >> 20) / (ahb_clk >> 20);
        	cycle = 50 / tmp; /* 10 */ /* 20 */ /* 50 */
		REG_NFC_PARA = ORIGINAL_NAND_TIMING | (cycle << 4);
    	}

	local_irq_restore(flags);	
}

static void nand_copy(unsigned char *src, unsigned char *dst, unsigned long len)
{
	unsigned long i;
	unsigned long *pDst_32, *pSrc_32;
	unsigned short *pDst_16, *pSrc_16;
	unsigned long flag = (unsigned long *)dst;
	
	flag = flag & 0x3;
	switch (flag) {
		case 0://word alignment
        		pDst_32 = (unsigned long *)dst;
                	pSrc_32 = (unsigned long *)src;
                	for (i = 0; i < (len / 4); i++) {
				*pDst_32 = *pSrc_32;
                    		pDst_32++;
                    		pSrc_32++;
			}
        	break;
        	case 2://half word alignment
                	pDst_16 = (unsigned short *)dst;
                	pSrc_16 = (unsigned short *)src;
                	for (i = 0; i < (len / 2); i++) {
                    		*pDst_16 = *pSrc_16;
                    		pDst_16++;
                    		pSrc_16++;
                	}
            	break;
        	default://byte alignment
                	for (i = 0; i < len; i++) {
                    		*dst = *src;
                    		dst++;
                    		src++;
                	}
            	break;
    	}	
}

#ifdef USE_DMA_MODE
void sprd_nand_dma_irq(int dma_ch, void *dev_id)
{
	wake_up_interruptible(&wait_queue);
}

void printk_dma_info()
{
	printk("\n--- dma register chn = %d ---\n", DMA_NLC);
	printk("DMA_CFG  : 0x%08x\n", __raw_readl(DMA_CFG));
	printk("DMA_CHx_EN  : 0x%08x\n", __raw_readl(DMA_CHx_EN));
	printk("DMA_CHx_DIS  : 0x%08x\n", __raw_readl(DMA_CHx_DIS));
	printk("DMA_LINKLIST_EN  : 0x%08x\n", __raw_readl(DMA_LINKLIST_EN));
	printk("DMA_SOFTLINK_EN  : 0x%08x\n", __raw_readl(DMA_SOFTLINK_EN));
	printk("DMA_SOFTLIST_SIZE  : 0x%08x\n", __raw_readl(DMA_SOFTLIST_SIZE));
	printk("DMA_SOFTLIST_CMD  : 0x%08x\n", __raw_readl(DMA_SOFTLIST_CMD));
	printk("DMA_SOFTLIST_STS  : 0x%08x\n", __raw_readl(DMA_SOFTLIST_STS));
	printk("DMA_PRI_REG0  : 0x%08x\n", __raw_readl(DMA_PRI_REG0));
	printk("DMA_PRI_REG1  : 0x%08x\n", __raw_readl(DMA_PRI_REG1));
	printk("DMA_INT_STS  : 0x%08x\n", __raw_readl(DMA_INT_STS));
	printk("DMA_INT_RAW  : 0x%08x\n", __raw_readl(DMA_INT_RAW));
	printk("DMA_LISTDONE_INT_EN  : 0x%08x\n", __raw_readl(DMA_LISTDONE_INT_EN));
	printk("DMA_BURST_INT_EN : 0x%08x\n", __raw_readl(DMA_BURST_INT_EN));
	printk("DMA_TRANSF_INT_EN  : 0x%08x\n", __raw_readl(DMA_TRANSF_INT_EN));
	printk("DMA_LISTDONE_INT_STS  : 0x%08x\n", __raw_readl(DMA_LISTDONE_INT_STS));
	printk("DMA_BURST_INT_STS  : 0x%08x\n", __raw_readl(DMA_BURST_INT_STS));
	printk("DMA_TRANSF_INT_STS  : 0x%08x\n", __raw_readl(DMA_TRANSF_INT_STS));
	printk("DMA_LISTDONE_INT_RAW  : 0x%08x\n", __raw_readl(DMA_LISTDONE_INT_RAW));
	printk("DMA_BURST_INT_RAW  : 0x%08x\n", __raw_readl(DMA_BURST_INT_RAW));
	printk("DMA_TRANSF_INT_RAW  : 0x%08x\n", __raw_readl(DMA_TRANSF_INT_RAW));
	printk("DMA_LISTDONE_INT_CLR  : 0x%08x\n", __raw_readl(DMA_LISTDONE_INT_CLR));
	printk("DMA_BURST_INT_CLR  : 0x%08x\n", __raw_readl(DMA_BURST_INT_CLR));
	printk("DMA_TRANSF_INT_CLR  : 0x%08x\n", __raw_readl(DMA_TRANSF_INT_CLR));
	printk("DMA_SOFT_REQ  : 0x%08x\n", __raw_readl(DMA_SOFT_REQ));
	printk("DMA_TRANS_STS  : 0x%08x\n", __raw_readl(DMA_TRANS_STS));
	printk("DMA_REQ_PEND  : 0x%08x\n", __raw_readl(DMA_REQ_PEND));
	printk("DMA_CHN_UID0  : 0x%08x\n", __raw_readl(DMA_CHN_UID0));
	printk("DMA_CHN_UID1  : 0x%08x\n", __raw_readl(DMA_CHN_UID1));
	printk("DMA_CHN_UID2  : 0x%08x\n", __raw_readl(DMA_CHN_UID2));
	printk("DMA_CHN_UID3  : 0x%08x\n", __raw_readl(DMA_CHN_UID3));
	printk("DMA_CHN_UID4  : 0x%08x\n", __raw_readl(DMA_CHN_UID4));
	printk("DMA_CHN_UID5  : 0x%08x\n", __raw_readl(DMA_CHN_UID5));
	printk("DMA_CHN_UID6  : 0x%08x\n", __raw_readl(DMA_CHN_UID6));
	printk("DMA_CHN_UID7  : 0x%08x\n", __raw_readl(DMA_CHN_UID7));
	printk("DMA_CHx_CFG0(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_CFG0(DMA_NLC)));
	printk("DMA_CHx_CFG1(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_CFG1(DMA_NLC)));
	printk("DMA_CHx_SRC_ADDR(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_SRC_ADDR(DMA_NLC)));
	printk("DMA_CHx_DEST_ADDR(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_DEST_ADDR(DMA_NLC)));
	printk("DMA_CHx_LLPTR(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_LLPTR(DMA_NLC)));
	printk("DMA_CHx_SDEP(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_SDEP(DMA_NLC)));
	printk("DMA_CHx_SBP(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_SBP(DMA_NLC)));
	printk("DMA_CHx_DBP(%d)  : 0x%08x\n", DMA_NLC, __raw_readl(DMA_CHx_DBP(DMA_NLC)));
}

/*
 * sprd_nand_dma_transfer: configer and start dma transfer
 * @mtd: MTD device structure
 * @addr: virtual address in RAM of source/destination
 * @len: number of data bytes to be transferred
 * @dir: flag for read/write operation
	DMA_TO_DEVICE is write to device
	DMA_FROM_DEVICE is read from device
 */
static int sprd_nand_dma_transfer(struct mtd_info *mtd, void *addr, unsigned int len, enum dma_data_direction dir)
{
	struct nand_chip *chip = mtd->priv;
	struct sprd_nand_info *info = chip->priv;

	dma_addr_t dma_addr;
   	u32 dsrc, ddst;
    	int width;
    	int autodma_src, autodma_dst;

	dma_addr = dma_map_single(&info->pdev->dev, addr, len, dir);
	if (dma_mapping_error(&info->pdev->dev, dma_addr)) {
		dev_err(&info->pdev->dev,
			"Couldn't DMA map a %d byte buffer\n", len);
		goto out_copy;
	}
	//printk("dma_addr = 0x%08x\n", dma_addr);
	memset(&ctrl, 0, sizeof(sprd_dma_ctrl));
	memset(&dma_desc, 0, sizeof(sprd_dma_desc));
	ctrl.dma_desc = &dma_desc;

	autodma_src = DMA_INCREASE;
        autodma_dst = DMA_INCREASE;
	width = 32;

	if (dir == DMA_TO_DEVICE) {
		dsrc = dma_addr;
		if (chip->IO_ADDR_W == NFC_MBUF)
			ddst = SPRD_NAND_PHYS;
		else if (chip->IO_ADDR_W == NFC_SBUF)
			ddst = SPRD_NAND_PHYS + 0x0C00;
	} else if (dir == DMA_FROM_DEVICE) {
		ddst = dma_addr;
		if (chip->IO_ADDR_R == NFC_MBUF)
			dsrc = SPRD_NAND_PHYS;
		else if (chip->IO_ADDR_R == NFC_SBUF)
			dsrc = SPRD_NAND_PHYS + 0x0C00;
	}

	sprd_dma_setup_cfg(&ctrl,
                DMA_NLC,
                DMA_NORMAL,
                TRANS_DONE_EN,
                autodma_src, autodma_dst,
                SRC_BURST_MODE_4, SRC_BURST_MODE_4,
                16,
                width, width,
                dsrc, ddst, len);

	sprd_dma_setup(&ctrl);
	sprd_dma_update(DMA_NLC, &dma_desc);
	__raw_bits_or(1 << DMA_NLC, DMA_CHx_EN);
	__raw_bits_or(1 << DMA_NLC, DMA_SOFT_REQ);
	interruptible_sleep_on(&wait_queue);
	dma_unmap_single(&info->pdev->dev, dma_addr, len, dir);
out_copy:

	return 0;
}

static void sprd_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;

	if (len == mtd->writesize)
		sprd_port_mode = DATA_PORT;
	else if (len == mtd->oobsize)
		sprd_port_mode = OOB_PORT;
	else
		printk("error W PORT\n");
		
	if (sprd_port_mode == DATA_PORT)
		chip->IO_ADDR_W = chip->IO_ADDR_R = NFC_MBUF;
	else if (sprd_port_mode == OOB_PORT)
		chip->IO_ADDR_W = chip->IO_ADDR_R = NFC_SBUF;

	//printk("buf = 0x%08x  ADDR_W = 0x%08x\n", buf, chip->IO_ADDR_W);	
	//sprd_nand_dma_transfer(mtd, buf, len, DMA_TO_DEVICE);
	memcpy(chip->IO_ADDR_W, buf, len);

}

static void sprd_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;

	if (len == mtd->writesize)
		sprd_port_mode = DATA_PORT;
	else if (len == mtd->oobsize)
		sprd_port_mode = OOB_PORT;
	else
		printk("error R PORT\n");

	if (sprd_port_mode == DATA_PORT)
		chip->IO_ADDR_W = chip->IO_ADDR_R = NFC_MBUF;
	else if (sprd_port_mode == OOB_PORT)
		chip->IO_ADDR_W = chip->IO_ADDR_R = NFC_SBUF;

	//printk("buf = 0x%08x  ADDR_R = 0x%08x\n", buf, chip->IO_ADDR_R);
	//sprd_nand_dma_transfer(mtd, buf, len, DMA_FROM_DEVICE);
	memcpy(buf, chip->IO_ADDR_R, len);

}

static uint8_t sprd_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;

	if (sprd_port_mode == OOBWORD_PORT)
		chip->IO_ADDR_R = chip->IO_ADDR_W = NFC_SBUF;
	else if (sprd_port_mode == IDSTATUS_PORT)
		chip->IO_ADDR_R = chip->IO_ADDR_W = NFC_IDSTATUS;

	if (g_buswidth == 1) {
		return (uint8_t)readw(chip->IO_ADDR_R);
	} else
		return readb(chip->IO_ADDR_R);
}

static u16 sprd_nand_read_word(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	
	if (sprd_port_mode == OOBWORD_PORT)
		chip->IO_ADDR_R = chip->IO_ADDR_W = NFC_SBUF;

	return readw(chip->IO_ADDR_R);
}
#endif

static int sprd_nand_inithw(struct sprd_nand_info *info, struct platform_device *pdev)
{
#if 0
	struct sprd_platform_nand *plat = to_nand_plat(pdev);
	unsigned long para = (plat->acs << 0) | 
				(plat->ach << 2) |
				(plat->rwl << 4) |
				(plat->rwh << 8) |
				(plat->rr << 10) |
				(plat->acr << 13) |
				(plat->ceh << 16);

 	writel(para, NFCPARAMADDR);
#endif
	
	return 0;
}

static void sprd_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	unsigned long phyblk, pageinblk, pageperblk;
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long chipsel = 0;
	unsigned long buswidth = g_buswidth;
	unsigned long addr_cycle = g_addr_cycle;
	//struct nand_chip *this = (struct nand_chip *)(mtd->priv);

	if (cmd == NAND_CMD_NONE)
		return;

	if (512 == mtd->writesize)
		pagetype = 0;
   	else
   	    	pagetype = 1;

	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 3;

	if (ctrl & NAND_CLE) {
		switch (cmd) {
			case NAND_CMD_RESET:
				REG_NFC_CMD = cmd | (0x1 << 31);
				nfc_wait_command_finish();
				mdelay(2);
			break;
			case NAND_CMD_STATUS:
				REG_NFC_CMD = cmd | (0x1 << 31);
				nfc_wait_command_finish();
#ifdef USE_DMA_MODE
				sprd_port_mode = IDSTATUS_PORT;
#else
				memset((unsigned char *)(this->IO_ADDR_R), 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
				nand_copy((unsigned char *)NFC_IDSTATUS, this->IO_ADDR_R, 4);
#endif
			break;
			case NAND_CMD_READID:
        			REG_NFC_CMD = cmd | (0x1 << 31);
        			nfc_wait_command_finish();
        			nand_flash_id = REG_NFC_IDSTATUS;
				//printk("%s  %s  %d   nand_flash_id=0x%08x\n", __FILE__, __FUNCTION__, __LINE__, nand_flash_id);
			break;
			case NAND_CMD_ERASE1:
				sprd_colrow_addr.column = 0;
				sprd_colrow_addr.row = 0;
				sprd_colrow_addr.colflag = 0;
				sprd_colrow_addr.rowflag = 0;
			break;
			case NAND_CMD_ERASE2:
				if ((0 == sprd_colrow_addr.colflag) && (0 == sprd_colrow_addr.rowflag)) {
					printk("erase address error!\n");
					return;
				} else {
					if (1 == sprd_colrow_addr.colflag) {
						sprd_colrow_addr.row = sprd_colrow_addr.column;
						sprd_colrow_addr.column = 0;
						sprd_colrow_addr.rowflag = 1;
						sprd_colrow_addr.colflag = 0;	
					}	
				}
				
				if ((0 == sprd_colrow_addr.colflag) && (1 == sprd_colrow_addr.rowflag)) {
					g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
							(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
					if (buswidth == 1)
        					REG_NFC_STR0 = sprd_colrow_addr.row * mtd->writesize;
					else
						REG_NFC_STR0 = sprd_colrow_addr.row * mtd->writesize * 2;
        				
        				REG_NFC_CMD = g_cmdsetting | NAND_CMD_ERASE1;
        				nfc_wait_command_finish();
				}
			break;
			case NAND_CMD_READ0:
				sprd_colrow_addr.column = 0;
				sprd_colrow_addr.row = 0;
				sprd_colrow_addr.colflag = 0;
				sprd_colrow_addr.rowflag = 0;
				sprd_wr_mode = READ_OP;
				sprd_area_mode = NO_AREA;
#ifdef USE_DMA_MODE
				sprd_port_mode = NO_PORT;
#else
				memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
#endif
			break;
			case NAND_CMD_READSTART:
				if (buswidth == 1) {
					if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
        					g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
								(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
        					REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        					nfc_wait_command_finish();
#ifdef USE_DMA_MODE
						sprd_port_mode = OOBWORD_PORT;
#else
						nand_copy((unsigned long *)NFC_SBUF, (unsigned long *)io_wr_port, mtd->oobsize);
#endif
					} else if (sprd_colrow_addr.column == 0) {
							if (sprd_area_mode == DATA_AREA)
								sprd_area_mode = DATA_OOB_AREA;

							if (sprd_area_mode == DATA_OOB_AREA) {
                                                		/* read data and spare area */
                                                		REG_NFC_END0 = 0xffffffff;
                                                		g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
										(1 << 21) | (buswidth << 19) | (pagetype << 18) | \
										(0 << 16) | (0x1 << 31);
                                                		REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
                                                		nfc_wait_command_finish();
#ifndef USE_DMA_MODE
								nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
#endif

                                        		} else if (sprd_colrow_addr.column == DATA_AREA) {
        							g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
										(buswidth << 19) | (pagetype << 18) | \
										(0 << 16) | (0x1 << 31);
        							REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        							nfc_wait_command_finish();
#ifndef USE_DMA_MODE				
								nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
#endif
						}
					} else
						printk("Operation !!! area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
				} else {  //if (buswidth == 1)
					if (sprd_colrow_addr.column == mtd->writesize) {
        					g_cmdsetting = (addr_cycle << 24) | (advance << 23) | (buswidth << 19) | \
								(pagetype << 18) | (0 << 16) | (0x1 << 31);
        					REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        					nfc_wait_command_finish();
#ifdef USE_DMA_MODE
						sprd_port_mode = OOBWORD_PORT;
#else
						nand_copy((unsigned char *)NFC_SBUF, io_wr_port, mtd->oobsize);
#endif
					} else if (sprd_colrow_addr.column == 0) {
						if (sprd_area_mode == DATA_AREA)
							sprd_area_mode = DATA_OOB_AREA;

						if (sprd_area_mode == DATA_OOB_AREA) {
                                                	/* read data and spare area, modify address */
                                                	REG_NFC_END0 = 0xffffffff;
                                                	g_cmdsetting = (addr_cycle << 24) | (advance << 23) | \
									(1 << 21) | (buswidth << 19) | (pagetype << 18) | \
									(0 << 16) | (0x1 << 31);

                                                	REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
                                                	nfc_wait_command_finish();
#ifndef USE_DMA_MODE
							nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
#endif
                                        	} else if (sprd_colrow_addr.column == DATA_AREA) {
        						g_cmdsetting = (addr_cycle << 24) | (advance << 23) | \
									(buswidth << 19) | (pagetype << 18) | \
									(0 << 16) | (0x1 << 31);
        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        						nfc_wait_command_finish();
#ifndef USE_DMA_MODE
							nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
#endif
						}
					} else
						printk("Operation !!! area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
				}
				sprd_wr_mode = NO_OP;
				sprd_area_mode = NO_AREA;
			break;
			case NAND_CMD_SEQIN:
				sprd_colrow_addr.column = 0;
				sprd_colrow_addr.row = 0;
				sprd_colrow_addr.colflag = 0;
				sprd_colrow_addr.rowflag = 0;
				sprd_wr_mode = WRITE_OP;
				sprd_area_mode = NO_AREA;
#ifdef USE_DMA_MODE
				sprd_port_mode = NO_PORT;
#else
				memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
#endif
			break;
			case NAND_CMD_PAGEPROG:
				if (buswidth == 1) {
					if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
						g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
								(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
#ifndef USE_DMA_MODE
						nand_copy((unsigned long *)io_wr_port, (unsigned long *)NFC_SBUF, mtd->oobsize);
#endif
        					REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        					nfc_wait_command_finish();
					} else if (sprd_colrow_addr.column == 0) {
						if (sprd_area_mode == DATA_OOB_AREA) {
							/* write data and spare area, modify address */
							REG_NFC_END0 = 0xffffffff;
							g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
								(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        						nfc_wait_command_finish();
						} else if (sprd_colrow_addr.column == DATA_AREA) {
        						g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
								(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
#ifndef USE_DMA_MODE
							nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, mtd->writesize);
#endif
        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        						nfc_wait_command_finish();
						}
					} else
						printk("Operation !!! area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);	
				} else {	//if (buswidth == 1)
					if (sprd_colrow_addr.column == mtd->writesize) {
        					g_cmdsetting = (addr_cycle << 24) | (advance << 23) | (buswidth << 19) | \
								(pagetype << 18) | (0 << 16) | (0x1 << 31);
#ifndef USE_DMA_MODE
						nand_copy(io_wr_port, (unsigned char *)NFC_SBUF, mtd->oobsize);
#endif
        					REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        					nfc_wait_command_finish();
					} else if (sprd_colrow_addr.column == 0) {
						if (sprd_area_mode == DATA_OOB_AREA) {
							/* write data and spare area, modify address */
							REG_NFC_END0 = 0xffffffff;
        						g_cmdsetting = (addr_cycle << 24) | (advance << 23) | (buswidth << 19) | \
									(pagetype << 18) | (0 << 16) | (0x1 << 31);
        			
        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        						nfc_wait_command_finish();
					
						} else if (sprd_colrow_addr.column == DATA_AREA) {
        						g_cmdsetting = (addr_cycle << 24) | (advance << 23) | (buswidth << 19) | \
									(pagetype << 18) | (0 << 16) | (0x1 << 31);
#ifndef USE_DMA_MODE
							nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, mtd->writesize);
#endif
        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        						nfc_wait_command_finish();
						}
					} else
						printk("Operation !!! area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
				}

				sprd_wr_mode = NO_OP;
				sprd_area_mode = NO_AREA;
#ifdef USE_DMA_MODE
				sprd_port_mode = NO_PORT;
#endif
			break;
			default:
			break;	
		}
	} else {
		if (0 == sprd_colrow_addr.colflag) {
			sprd_colrow_addr.colflag = 1;
			sprd_colrow_addr.column = cmd;
			//printk("%d  column = %d\n", __LINE__, cmd);
			return;
		}
		
		if (0 == sprd_colrow_addr.rowflag) {
			sprd_colrow_addr.rowflag = 1;
			sprd_colrow_addr.row = cmd;
			//printk("%d  row = %d\n",  __LINE__, cmd);
		}
		
		if ((1 == sprd_colrow_addr.colflag) && (1 == sprd_colrow_addr.rowflag)) {
			if (buswidth == 1) {
				if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
					pageperblk = mtd->erasesize / mtd->writesize;
					/*printk("pageperblk=%d  mtd->erasesize=%d  mtd->writesize=%d  mtd->oobsize=%d\n",
						 pageperblk, mtd->erasesize, mtd->writesize, mtd->oobsize);*/
					phyblk = sprd_colrow_addr.row / pageperblk;
        				pageinblk = sprd_colrow_addr.row % pageperblk;
					//printk("block = %d  page = %d  column = %d\n", phyblk, pageinblk, sprd_colrow_addr.column);

        				REG_NFC_STR0 = phyblk * pageperblk * mtd->writesize + 
								pageinblk * mtd->writesize + 
								sprd_colrow_addr.column;
        				REG_NFC_END0 = phyblk * pageperblk * mtd->writesize + 
								pageinblk * mtd->writesize + 
								sprd_colrow_addr.column + (mtd->oobsize >> 1) -1;
					sprd_area_mode = OOB_AREA;
					/*printk("Operation OOB area.  %s  %s  %d   row=0x%08x  column=0x%08x\n", 
						__FILE__, __FUNCTION__, __LINE__, sprd_colrow_addr.row, sprd_colrow_addr.column);*/

				} else if (sprd_colrow_addr.column == 0) {
					pageperblk = mtd->erasesize / mtd->writesize;
					phyblk = sprd_colrow_addr.row / pageperblk;
        				pageinblk = sprd_colrow_addr.row % pageperblk;
					/*printk("Line : %d  block = %d  page = %d  column = %d\n",
						__LINE__, phyblk, pageinblk, sprd_colrow_addr.column);*/
        				REG_NFC_STR0 = phyblk * pageperblk * mtd->writesize + 
								pageinblk * mtd->writesize + 
								sprd_colrow_addr.column;
        				REG_NFC_END0 = phyblk * pageperblk * mtd->writesize  + 
								pageinblk * mtd->writesize + 
								sprd_colrow_addr.column + (mtd->writesize >> 1) - 1;
					sprd_area_mode = DATA_AREA;	
					/*printk("Operation DATA area.  %s  %s  %d   row=0x%08x  column=0x%08x\n", 
						__FILE__, __FUNCTION__, __LINE__, sprd_colrow_addr.row, sprd_colrow_addr.column);*/
				} else
					printk("Operation ??? area for 16 Bits.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
			} else {   //if (buswidth == 0) {
				if (sprd_colrow_addr.column == mtd->writesize) {
					pageperblk = mtd->erasesize / mtd->writesize;
					phyblk = sprd_colrow_addr.row / pageperblk;
        				pageinblk = sprd_colrow_addr.row % pageperblk;

        				REG_NFC_STR0 = phyblk * pageperblk * mtd->writesize * 2 + 
								pageinblk * mtd->writesize * 2 + 
								sprd_colrow_addr.column;
        				REG_NFC_END0 = phyblk * pageperblk * mtd->writesize * 2 + 
								pageinblk * mtd->writesize * 2 + 
								sprd_colrow_addr.column + mtd->oobsize -1;
					sprd_area_mode = OOB_AREA;	
					/*printk("Operation OOB area.  %s  %s  %d   row=0x%08x  column=0x%08x\n", 
						__FILE__, __FUNCTION__, __LINE__, sprd_colrow_addr.row, sprd_colrow_addr.column);*/
				} else if (sprd_colrow_addr.column == 0) {				
					pageperblk = mtd->erasesize / mtd->writesize;
					phyblk = sprd_colrow_addr.row / pageperblk;
        				pageinblk = sprd_colrow_addr.row % pageperblk;

        				REG_NFC_STR0 = phyblk * pageperblk * mtd->writesize * 2 + 
								pageinblk * mtd->writesize * 2 + 
								sprd_colrow_addr.column;
        				REG_NFC_END0 = phyblk * pageperblk * mtd->writesize * 2 + 
								pageinblk * mtd->writesize * 2 + 
								sprd_colrow_addr.column + mtd->writesize - 1;
					sprd_area_mode = DATA_AREA;	
					/*printk("Operation DATA area.  %s  %s  %d   row=0x%08x  column=0x%08x\n", 
						__FILE__, __FUNCTION__, __LINE__, sprd_colrow_addr.row, sprd_colrow_addr.column);*/
				} else
					printk("Operation ??? area for 8 Bits.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
			}
		}
	}		
}

static unsigned long sprd_nand_readid(struct mtd_info *mtd)
{
	return(nand_flash_id);
}

static int sprd_nand_devready(struct mtd_info *mtd)
{
	unsigned long status = 0;
        unsigned long cmd = NAND_CMD_STATUS | (0x1 << 31);

        REG_NFC_CMD = cmd;
        nfc_wait_command_finish();
        status = REG_NFC_IDSTATUS;
   	if ((status & 0x1) != 0) 	
     		return -1; /* fail */
   	else if ((status & 0x20) == 0)
     		return 0; /* busy */
   	else 							
     		return 1; /* ready */
}

static void sprd_nand_select_chip(struct mtd_info *mtd, int chip)
{
	//struct nand_chip *this = mtd->priv;
	//struct sprd_nand_info *info = this->priv;

#if 0	
	if (chip != -1)
		clk_enable(info->clk);
	else
		clk_disable(info->clk);
#endif
}

#ifdef USE_DMA_MODE
static void sprd_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	sprd_ecc_mode = mode;
	if (sprd_ecc_mode == NAND_ECC_WRITE) {
		REG_NFC_ECCEN = 0x1;
	}
}
#else
static void sprd_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	sprd_ecc_mode = mode;
}
#endif

static unsigned long sprd_nand_wr_oob(struct mtd_info *mtd)
{
#ifndef USE_DMA_MODE
        /* copy io_wr_port into SBUF */
        nand_copy(io_wr_port, (unsigned char *)NFC_SBUF, mtd->oobsize);
#endif

	/* write oob area */
	if (sprd_area_mode == NO_AREA)
		sprd_area_mode = OOB_AREA;
	else if (sprd_area_mode == DATA_AREA)
		sprd_area_mode = DATA_OOB_AREA;

	return 0;
}
#ifdef USE_DMA_MODE
static int sprd_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	unsigned long *pecc_val;
	pecc_val=(unsigned long *)ecc_code;

	if (sprd_ecc_mode == NAND_ECC_WRITE) {
		pecc_val[0] = REG_NFC_PAGEECC0;
		pecc_val[1] = REG_NFC_PAGEECC1;
		pecc_val[2] = REG_NFC_PAGEECC2;
		pecc_val[3] = REG_NFC_PAGEECC3;
		REG_NFC_ECCEN = 0;
	} else  if (sprd_ecc_mode == NAND_ECC_READ) {
                /* large page */
		pecc_val[0] = REG_NFC_PAGEECC0;
		pecc_val[1] = REG_NFC_PAGEECC1;
		pecc_val[2] = REG_NFC_PAGEECC2;
		pecc_val[3] = REG_NFC_PAGEECC3;
	}

	sprd_ecc_mode = NAND_ECC_NONE;

	return 0;
}
#else
static int sprd_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	unsigned long *pecc_val;
	pecc_val=(unsigned long *)ecc_code;

	if (sprd_ecc_mode == NAND_ECC_WRITE) {
		REG_NFC_ECCEN = 0x1;
		/* copy io_wr_port into MBUF */
		nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, mtd->writesize);
		/* large page */
		pecc_val[0] = REG_NFC_PAGEECC0;
		pecc_val[1] = REG_NFC_PAGEECC1;
		pecc_val[2] = REG_NFC_PAGEECC2;
		pecc_val[3] = REG_NFC_PAGEECC3;
		REG_NFC_ECCEN = 0;
		memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);	
	} else if (sprd_ecc_mode == NAND_ECC_READ) {
                /* large page */
		pecc_val[0] = REG_NFC_PAGEECC0;
		pecc_val[1] = REG_NFC_PAGEECC1;
		pecc_val[2] = REG_NFC_PAGEECC2;
		pecc_val[3] = REG_NFC_PAGEECC3;
		memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
		nand_copy((unsigned char *)NFC_SBUF, io_wr_port, mtd->oobsize);
	}	
	sprd_ecc_mode = NAND_ECC_NONE;

	return 0;
}
#endif

static void ecc_trans(unsigned char *ecc)
{
	unsigned char trans;
	
	trans = ecc[0];
	ecc[0] = ecc[1];
	ecc[1] = ecc[2];
	ecc[2] = trans;
}

static int ECC_CompM(unsigned char *pEcc1, unsigned char *pEcc2, unsigned char *pBuf)
{
	unsigned long  nEccComp = 0, nEccSum = 0;
	unsigned long  nEBit    = 0;
	unsigned long  nEByte   = 0;
	unsigned long  nXorT1   = 0, nXorT2 = 0;
	unsigned long  nCnt;

	for (nCnt = 0; nCnt < 2; nCnt++) {
        	nXorT1 ^= (((*pEcc1) >> nCnt) & 0x01);
        	nXorT2 ^= (((*pEcc2) >> nCnt) & 0x01);
    	}

    	for (nCnt = 0; nCnt < 3; nCnt++) {
        	nEccComp |= ((~pEcc1[nCnt] ^ ~pEcc2[nCnt]) << (nCnt * 8));
    	}

    	for(nCnt = 0; nCnt < 24; nCnt++) {
        	nEccSum += ((nEccComp >> nCnt) & 0x01);
    	}

    	switch (nEccSum) {
	case 0 :
		//printk("No Error for Main\n");
		return 0;
	case 1 :
		//printk("ECC Error \n");
		return 1;
        case 12 :			
		if (nXorT1 != nXorT2) {
			nEByte  = ((nEccComp >>  7) & 0x100) +
				((nEccComp >>  6) & 0x80) + ((nEccComp >>  5) & 0x40) +
				((nEccComp >>  4) & 0x20) + ((nEccComp >>  3) & 0x10) +
				((nEccComp >>  2) & 0x08) + ((nEccComp >>  1) & 0x04) +
				(nEccComp & 0x02) + ((nEccComp >> 23) & 0x01);
			
			nEBit   = (unsigned char)(((nEccComp >> 19) & 0x04) +
				((nEccComp >> 18) & 0x02) + ((nEccComp >> 17) & 0x01));
			//printk("1ECC Position : %dth byte, %dth bit\n", nEByte, nEBit);
			if (pBuf != NULL) {
				//printf("Corrupted : 0x%02x \n", pBuf[nEByte]);
				pBuf[nEByte] = (unsigned char)(pBuf[nEByte] ^ (1 << nEBit));
				//printk("Corrected : 0x%02x \n", pBuf[nEByte]);
			}
			return 1;
		}
        default :
		//printk("Uncorrectable ECC Error Occurs for Main\n");
        	break;
    	}

	return -1;
}

static int correct(u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
#ifdef CONFIG_ARCH_SC8800G
	ecc_trans(read_ecc);
	ecc_trans(calc_ecc);
#endif
	return ECC_CompM(read_ecc, calc_ecc, dat);
}

static int sprd_nand_correct_data(struct mtd_info *mtd, uint8_t *dat,
				     uint8_t *read_ecc, uint8_t *calc_ecc)
{
	int retval = 0;
	int retval0, retval1, retval2, retval3;

	if (mtd->writesize > 512) {
		retval0 = correct(dat + 512 * 0, read_ecc + 4 * 0, calc_ecc + 4 * 0);
		retval1 = correct(dat + 512 * 1, read_ecc + 4 * 1, calc_ecc + 4 * 1);
		retval2 = correct(dat + 512 * 2, read_ecc + 4 * 2, calc_ecc + 4 * 2);
		retval3 = correct(dat + 512 * 3, read_ecc + 4 * 3, calc_ecc + 4 * 3);
		if ((retval0 == -1) || (retval1 == -1) || (retval2 == -1) || (retval3 == -1))
			retval = -1;
		else
			retval = retval0 + retval1 + retval2 + retval3;
	} else
		retval = correct(dat, read_ecc, calc_ecc);
	
	return retval;
}


//#define NAND_TEST_CODE 1

#ifdef NAND_TEST_CODE
#define NF_RESET                0xFF
#define NF_READ_STATUS  	0x70
#define NF_READ_ID              0x90
#define NF_READ_1ST             0x00
#define NF_BKERASE_ID           0x60
#define NF_WRITE_ID             0x80
int E[512];

#if 0
#define MTDPARTS_DEFAULT "mtdparts=sprd-nand:384k@256k(boot),256k(params),6m(kernel),6m(ramdisk),6m(recovery),70m(system),70m(userdata),70m(cache)"
#define CONFIG_BOOTARGS "mem=128M console=ttyS1,115200n8 initrd=0x3000000,4194304 init=/init root=/dev/ram0 rw "MTDPARTS_DEFAULT
#endif

static void nfc_ms_read_l(unsigned long page_no, unsigned char *buf)
{
#if 1
	/* test for 16 bit width */
	unsigned long cmd = NF_READ_1ST;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long chipsel = 0;
	unsigned long buswidth = g_buswidth;
	unsigned long addr_cycle = g_addr_cycle;
	unsigned long PAGEECC0, PAGEECC1, PAGEECC2, PAGEECC3;
	
	if (512 == PAGE_SIZE_L)
		pagetype = 0;
   	else
   	    	pagetype = 1;
	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 3;

  	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;

	printk("block = %d  page = %d\n", phyblk, pageinblk);

        REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L + pageinblk * PAGE_SIZE_L; 
        REG_NFC_END0 = 0xffffffff;

        g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(1 << 21) | (buswidth << 19) | (pagetype << 18) | \
			(0 << 16) | (0x1 << 31);

        REG_NFC_CMD = g_cmdsetting | cmd;
        nfc_wait_command_finish();
	nfc_read_status();
	/*printk("\nREADECC0 = 0x%08x  ", REG_NFC_PAGEECC0);
        printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);
        printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
        printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);*/

        nand_copy((unsigned char *)NFC_MBUF, buf, PAGE_SIZE_L);
        nand_copy((unsigned char *)NFC_SBUF, (buf + PAGE_SIZE_L), SPARE_SIZE_L);
#if 1	
	printk("\nRead\n");
        for (i = 0; i < (PAGE_SIZE_L + SPARE_SIZE_L); i++) {
		if ((i % 16) == 0)
			printk("\n");
		if (i == PAGE_SIZE_L)
			printk("ReadOob\n");
                printk("%02x ", buf[i]);
	}
#else
	printk("\nReadOob\n");
        for (i = PAGE_SIZE_L; i < (PAGE_SIZE_L + SPARE_SIZE_L); i++) {
		if ((i % 16) == 0)
			printk("\n");
                printk("%02x ", buf[i]);
	}
#endif
	printk("\n");
#else
	/* test for 8 bit width */
	unsigned long cmd = NF_READ_1ST;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
	unsigned long buswidth = g_buswidth;
	unsigned long addr_cycle = g_addr_cycle;
	unsigned long PAGEECC0, PAGEECC1, PAGEECC2, PAGEECC3;

	if (512 == PAGE_SIZE_L)
		pagetype = 0;
   	else
   	    	pagetype = 1;
	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 3;

  	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;

	printk("block = %d  page = %d\n", phyblk, pageinblk);
	REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L * 2 + pageinblk * PAGE_SIZE_L * 2;
        REG_NFC_END0 = 0xffffffff;

        g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(1 << 21) | (buswidth << 19) | (pagetype << 18) | \
			(0 << 16) | (0x1 << 31);

        REG_NFC_CMD = g_cmdsetting | cmd;
        nfc_wait_command_finish();

	printk("\nREADECC0 = 0x%08x  ", REG_NFC_PAGEECC0);
        printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);
        printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
        printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);

        nand_copy((unsigned char *)NFC_MBUF, buf, PAGE_SIZE_L);
        nand_copy((unsigned char *)NFC_SBUF, (buf + PAGE_SIZE_L), SPARE_SIZE_L);
	printk("\nRead\n");
        for (i = 0; i < (PAGE_SIZE_L + SPARE_SIZE_L); i++) {
		if ((i % 16) == 0)
			printk("\n");
		if (i == PAGE_SIZE_L)
			printk("ReadOob\n");
                printk("%02x ", buf[i]);
	}
	
	printk("\n");
#endif
}

static void nfc_ms_write_l(unsigned long page_no, unsigned char *buf)
{
#if 1
	/* test for 16 bit width */
	unsigned long cmd = NF_WRITE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long chipsel = 0;
	unsigned long buswidth = g_buswidth;
	unsigned long addr_cycle = g_addr_cycle;
	unsigned long PAGEECC0, PAGEECC1, PAGEECC2, PAGEECC3;

	if (512 == PAGE_SIZE_L)
		pagetype = 0;
   	else
   	    	pagetype = 1;
	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 3;

  	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;

	buf[21] = 0x35;
	printk("\nWrite\n");
	
        for (i = 0; i < (PAGE_SIZE_L + SPARE_SIZE_L); i++) {
		if ((i % 16) == 0)
			printk("\n");
		if (i == PAGE_SIZE_L)
			printk("WriteOob\n");
                printk("%02x ", buf[i]);
	}
	printk("\n");

 	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;

	printk("write block = %d  page = %d\n", phyblk, pageinblk);
	
        REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L + pageinblk * PAGE_SIZE_L;
        //REG_NFC_END0 = phyblk * pageperblk * PAGE_SIZE_L  + pageinblk * PAGE_SIZE_L + (PAGE_SIZE_L >> 1) - 1;	
	REG_NFC_END0 = 0xffffffff;
	//printk("%s  %s  %d  addr_cycle = %d  buswidth = %d\n", __FILE__, __FUNCTION__, __LINE__,addr_cycle, buswidth);
	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

	REG_NFC_ECCEN = 0x1;
	nand_copy(buf, (unsigned char *)NFC_MBUF, PAGE_SIZE_L);
	printk("\nWRITEECC0 = 0x%08x  ", REG_NFC_PAGEECC0);		
	printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);		
	printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
	printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
	REG_NFC_ECCEN = 0;	

        nand_copy((buf+PAGE_SIZE_L), (unsigned char *)NFC_SBUF, SPARE_SIZE_L);
        REG_NFC_CMD = g_cmdsetting | cmd;
	//printk("%s  %s  %d  addr_cycle = %d  buswidth = %d\n", __FILE__, __FUNCTION__, __LINE__,addr_cycle, buswidth);
        nfc_wait_command_finish();
	nfc_read_status();

#else
	/* test for 8 bit width */
	unsigned long cmd = NF_WRITE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
	unsigned long buswidth = g_buswidth;
	unsigned long addr_cycle = g_addr_cycle;
	unsigned long PAGEECC0, PAGEECC1, PAGEECC2, PAGEECC3;

	if (512 == PAGE_SIZE_L)
		pagetype = 0;
   	else
   	    	pagetype = 1;
	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 3;
	//buf[21] = 0x35;
	printk("\nWrite\n");
	
        for (i = 0; i < (PAGE_SIZE_L + SPARE_SIZE_L); i++) {
		if ((i % 16) == 0)
			printk("\n");
		if (i == PAGE_SIZE_L)
			printk("WriteOob\n");
                printk("%02x ", buf[i]);
	}
	printk("\n");

 	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;

	printk("write block = %d  page = %d\n", phyblk, pageinblk);
	REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L * 2 + pageinblk * PAGE_SIZE_L * 2;
	REG_NFC_END0 = 0xffffffff;

	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

	REG_NFC_ECCEN = 0x1;
	nand_copy(buf, (unsigned char *)NFC_MBUF, PAGE_SIZE_L);
	printk("\nWRITEECC0 = 0x%08x  ", REG_NFC_PAGEECC0);		
	printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);		
	printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
	printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
	REG_NFC_ECCEN = 0;

        nand_copy((buf+PAGE_SIZE_L), (unsigned char *)NFC_SBUF, SPARE_SIZE_L);
        REG_NFC_CMD = g_cmdsetting | cmd;
        nfc_wait_command_finish();
#endif
}

static void nfc_erase_block(unsigned long page_no)
{
#if 1
	/* test for 16 bit width */
	unsigned long cmd = NF_BKERASE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long chipsel = 0;
	unsigned long buswidth = g_buswidth;
	unsigned long addr_cycle = g_addr_cycle;

	if (512 == PAGE_SIZE_L)
		pagetype = 0;
   	else
   	    	pagetype = 1;
	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 3;

  	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;
	printk("erase block = %d\n", phyblk);
	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
				(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

        REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L;
        REG_NFC_CMD = g_cmdsetting | cmd;
	//printk("%s  %s  %d  addr_cycle = %d  buswidth = %d\n", __FILE__, __FUNCTION__, __LINE__,addr_cycle, buswidth);
        nfc_wait_command_finish();
	nfc_read_status();
#else
	/* test for 8 bit width */
	unsigned long cmd = NF_BKERASE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
	unsigned long buswidth = g_buswidth;
	unsigned long addr_cycle = g_addr_cycle;

	if (512 == PAGE_SIZE_L)
		pagetype = 0;
   	else
   	    	pagetype = 1;
	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 3;

  	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;
	printk("erase block = %d\n", phyblk);
	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
				(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

	REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L * 2;
        REG_NFC_CMD = g_cmdsetting | cmd;
        nfc_wait_command_finish();
	nfc_read_status();

#endif
}

#endif

/* driver device registration */
static int sprd_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct sprd_nand_info *info;
	struct sprd_platform_nand *plat = to_nand_plat(pdev);/* get timing */
	unsigned long id, type;

#ifdef NAND_TEST_CODE
	unsigned char buffer[PAGE_SIZE_L + SPARE_SIZE_L];
	unsigned long pageno;
	unsigned long idx;
#endif

#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
#endif

	/* set sprd_colrow_addr */
	sprd_colrow_addr.column = 0;
	sprd_colrow_addr.row = 0;
	sprd_colrow_addr.colflag = 0;
	sprd_colrow_addr.rowflag = 0;
	sprd_wr_mode = NO_OP;
	sprd_area_mode = NO_AREA;
	sprd_ecc_mode = NAND_ECC_NONE;
#ifdef USE_DMA_MODE
	sprd_port_mode = NO_PORT;
#endif

#ifdef CONFIG_ARCH_SC8800S
	//REG_AHB_CTL0 |= BIT_8 | BIT_9;//no BIT_9
	__raw_bits_or(BIT_8 | BIT_9 , AHB_CTL0);
	//REG_AHB_SOFT_RST |= BIT_5;
	__raw_bits_or(BIT_5, AHB_SOFT_RST);
	__raw_bits_and(~BIT_5, AHB_SOFT_RST);
#else
	/* CONFIG_ARCH_SC8800G */
	//REG_AHB_CTL0 |= BIT_8;
	__raw_bits_or(BIT_8, AHB_CTL0);
	//REG_AHB_SOFT_RST |= BIT_5; need test
	__raw_bits_or(BIT_5, AHB_SOFT_RST);
	__raw_bits_and(~BIT_5, AHB_SOFT_RST);
#endif

	REG_NFC_INTSRC |= BIT_0 | BIT_4 | BIT_5;
	/* 0x1 : WPN disable, and micron nand flash status is 0xeo 
 	   0x0 : WPN enable, and micron nand flash status is 0x60 */
	REG_NFC_WPN = 0x1;
	sprd_config_nand_pins8();
	set_nfc_param(100);
#ifndef USE_DMA_MODE
	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
#endif
	nfc_reset();
        mdelay(2);
        nfc_read_status();
        id = nfc_read_id();
	type = nand_bit_width(id);

#ifdef NAND_TEST_CODE
	/* run my test code */
	printk("\nRun nand test is starting\n");
	/* 16-bit bus width */
	if (type == 1) {
		sprd_config_nand_pins16();
		g_buswidth = 1;
		g_addr_cycle = 5;
		printk("is 16 Bit\n");	
	} else {
		g_buswidth = 0;
		g_addr_cycle = 4;
		printk("is 8 Bit\n");
	}

	pageno = 497 * 64;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);

	pageno = 497 * 64 + 24;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);

	/*memset(buffer, 0xff, PAGE_SIZE_L + SPARE_SIZE_L);
	for (idx = 0; idx < 256; idx++) {
		buffer[idx + 0] = idx;
		buffer[idx + 256] = idx;
		buffer[idx + 512] = idx;
		buffer[idx + 768] = idx;
		buffer[idx + 1024] = idx;
		buffer[idx + 1280] = idx;
		buffer[idx + 1536] = idx;
		buffer[idx + 1792] = idx;
	}
	for (idx = 1; idx < SPARE_SIZE_L; idx++)
		buffer[PAGE_SIZE_L + idx] = idx;

	pageno = 4000 * 64;
	nfc_ms_write_l(pageno, buffer);

	pageno = 4000 * 64 + 1;
	nfc_ms_write_l(pageno, buffer);

	pageno = 4000 * 64 + 1;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);

	pageno = 4000 * 64;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);*/
	
	printk("\nRun nand test is ended\n");	

	return 0;
#endif

	info = kmalloc(sizeof(*info), GFP_KERNEL);

	memset(info, 0 , sizeof(*info));
	platform_set_drvdata(pdev, info);/* platform_device.device.driver_data IS info */
	info->platform = plat; /* nand timing */
	//info->clk = clk_get(&pdev->dev, "nand"); /* nand clock */
	//clk_enable(info->clk);
	info->pdev = pdev;

	sprd_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	this = (struct nand_chip *)(&sprd_mtd[1]);
	memset((char *)sprd_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

	sprd_mtd->priv = this;
	
	/* set the timing for nand controller */
	sprd_nand_inithw(info, pdev);
	/* 16-bit bus width */
	if (type == 1) {
		sprd_config_nand_pins16();
		this->options |= NAND_BUSWIDTH_16;
		g_buswidth = 1;
		g_addr_cycle = 5;	
	} else {
		g_buswidth = 0;
		g_addr_cycle = 4;
	}
	this->cmd_ctrl = sprd_nand_hwcontrol;
	this->dev_ready = sprd_nand_devready;
	this->select_chip = sprd_nand_select_chip;
	this->nfc_readid = sprd_nand_readid;
	this->nfc_wr_oob = sprd_nand_wr_oob;
	this->ecc.calculate = sprd_nand_calculate_ecc;
	this->ecc.correct = sprd_nand_correct_data;
	this->ecc.hwctl = sprd_nand_enable_hwecc;
#ifdef USE_DMA_MODE
	this->write_buf = sprd_nand_write_buf;
	this->read_buf = sprd_nand_read_buf;
	this->read_word = sprd_nand_read_word;
	this->read_byte = sprd_nand_read_byte;
	sprd_request_dma(DMA_NLC, sprd_nand_dma_irq, info);
	init_waitqueue_head(&wait_queue);
#endif
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = 2048;//512;
	this->ecc.bytes = 16;//3
	this->chip_delay = 20;
	this->priv = info;
	/* scan to find existance of the device */
	nand_scan(sprd_mtd, 1);	
#ifdef CONFIG_MTD_CMDLINE_PARTS
	sprd_mtd->name = "sprd-nand";
	num_partitions = parse_mtd_partitions(sprd_mtd, part_probes, &partitions, 0);
#endif

	/*printk("num_partitons = %d\n", num_partitions);
	for (i = 0; i < num_partitions; i++) {
		printk("i=%d  name=%s  offset=0x%016Lx  size=0x%016Lx\n", i, partitions[i].name, 
			(unsigned long long)partitions[i].offset, (unsigned long long)partitions[i].size);
	}*/

	if ((!partitions) || (num_partitions == 0)) {
		printk("No parititions defined, or unsupported device.\n");
		goto release;
	}
	add_mtd_partitions(sprd_mtd, partitions, num_partitions);

#ifdef CONFIG_CPU_FREQ
	min_freq = 0;
	max_freq = 0;
	info->freq_transition.notifier_call = nand_freq_transition;
	info->freq_policy.notifier_call = nand_freq_policy;
	cpufreq_register_notifier(&info->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&info->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif
	return 0;
release:
	nand_release(sprd_mtd);
	return 0;
}

/* device management functions */
static int sprd_nand_remove(struct platform_device *pdev)
{
	struct sprd_nand_info *info = platform_get_drvdata(pdev);

	sprd_free_dma(DMA_NLC);
	platform_set_drvdata(pdev, NULL);
	if (info == NULL)
		return 0;

#ifdef CONFIG_CPU_FREQ
	cpufreq_unregister_notifier(&info->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_notifier(&info->freq_policy, CPUFREQ_TRANSITION_NOTIFIER);
#endif

	del_mtd_partitions(sprd_mtd);
	del_mtd_device(sprd_mtd);
	kfree(sprd_mtd);
	//clk_disable(info->clk);
#ifdef USE_DMA_MODE
	sprd_free_dma(DMA_NLC);
#endif
	kfree(info);	

	return 0;
}

/* CPU_FREQ Support */
#ifdef CONFIG_CPU_FREQ
static void set_nfm_voltage(nfm_voltage_e voltage)
{
	/* no implement */
}
/*
 * CPU clock speed change handler.  We need to adjust the NAND timing
 * parameters when the CPU clock is adjusted by the power management
 * subsystem.
 */
static int nand_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	//struct sprd_nand_info *info = container_of(nb, struct sprd_nand_info, freq_transition);
	struct cpufreq_freqs *freq = data;

	if (min_freq > freq->old)
		min_freq = freq->old;
	if (min_freq > freq->new)
		min_freq = freq->new;

	if (max_freq < freq->old)
		max_freq = freq->old;
	if (max_freq < freq->new)
		max_freq = freq->new;
	//printk("old = %d  new = %d  min_freq = %d   max_freq = %d\n", freq->old, freq->new, min_freq, max_freq);

	switch (val) {
	case CPUFREQ_PRECHANGE:
		if (freq->new == max_freq) {
			/* slow -> fastest, adjust NFM voltage */
			set_nfm_voltage(NFM_VOLTAGE_3000MV);
		} else if (freq->old == max_freq) {
			/* fastest -> slow, adjust NFC param */
			set_nfc_param(100);
		}
		break;

	case CPUFREQ_POSTCHANGE:
		if (freq->new == max_freq) {
			set_nfc_param(100);
		} else if (freq->old == max_freq) {
			set_nfm_voltage(NFM_VOLTAGE_2650MV);
		}		

		break;
	}

	return 0;
}

static int nand_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	//struct sprd_nand_info *info = container_of(nb, struct sprd_nand_info, freq_policy);
	struct cpufreq_policy *policy = data;

	//printk("min = %d  max = %d\n", policy->min, policy->max);
	min_freq = policy->min;
	max_freq = policy->max;

	switch (val) {
	case CPUFREQ_ADJUST:
		break;
	case CPUFREQ_INCOMPATIBLE:
		break;
	case CPUFREQ_NOTIFY:
		break;
	}

	return 0;
}
#endif

/* PM Support */
#ifdef CONFIG_PM
static int sprd_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
	//struct sprd_nand_info *info = platform_get_drvdata(dev);
#if 0
	if (info)
		clk_disable(info->clk);
#endif
	return 0;
}

static int sprd_nand_resume(struct platform_device *dev)
{
	struct sprd_nand_info *info = platform_get_drvdata(dev);

	if (info) {
		//clk_enable(info->clk);
		sprd_nand_inithw(info, dev);
	}

	return 0;
}

#else
#define sprd_nand_suspend NULL
#define sprd_nand_resume NULL
#endif

static struct platform_driver sprd_nand_driver = {
	.probe		= sprd_nand_probe,
	.remove		= sprd_nand_remove,
	.suspend	= sprd_nand_suspend,
	.resume		= sprd_nand_resume,
	.driver		= {
		.name	= "sprd_nand",
		.owner	= THIS_MODULE,
	},
};

static int __init sprd_nand_init(void)
{
	printk("\nSpreadtrum 8800 NAND Driver, (c) 2010 Spreadtrum\n");
	return platform_driver_register(&sprd_nand_driver);
}

static void __exit sprd_nand_exit(void)
{
	platform_driver_unregister(&sprd_nand_driver);
}

module_init(sprd_nand_init);
module_exit(sprd_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard Feng <Richard.Feng@spreadtrum.com>");
MODULE_DESCRIPTION("SPRD 8800 MTD NAND driver");
