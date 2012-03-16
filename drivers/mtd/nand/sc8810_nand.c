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
#include <mach/pm_devices.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <mach/regs_ahb.h>
#include <mach/mfp.h>
#include <mach/dma.h>
#if (defined CONFIG_ARCH_SC8810)
#include "sc8810_nfc.h"
#endif

#if 1
static unsigned long nand_base = 0x60000000;

struct sprd_nand_info {
	unsigned long			phys_base;
	struct sprd_platform_nand	*platform;
	struct clk			*clk;
	struct mtd_info			mtd;
	struct platform_device		*pdev;
};

static unsigned long nand_func_cfg8[] = {
#if ( defined CONFIG_ARCH_SC8810)
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
};

static unsigned long nand_func_cfg16[] = {
#if (defined CONFIG_ARCH_SC8810)
	MFP_CFG_X(NFD8,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD9,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD10,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD11,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD12,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD13,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD14,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
	MFP_CFG_X(NFD15,  AF0, DS1, F_PULL_NONE, S_PULL_DOWN, IO_Z),
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

#endif

/* Size of the block protected by one OOB (Spare Area in Samsung terminology) */
#define CONFIG_SYS_NAND_ECCSIZE	512
/* Number of ECC bytes per OOB - S3C6400 calculates 4 bytes ECC in 1-bit mode */
//#define CONFIG_SYS_NAND_ECCBYTES	4
#define CONFIG_SYS_NAND_ECCBYTES	4
/* Number of ECC-blocks per NAND page */
/* 2 bit correct, sc8810 support 1, 2, 4, 8, 12,14, 24 */
#define CONFIG_SYS_NAND_ECC_MODE	2
/* Size of a single OOB region */
/* Number of ECC bytes per page */
/* ECC byte positions */


#define	NFC_ECC_EVENT  		1
#define	NFC_DONE_EVENT		2
#define	NFC_TX_DMA_EVENT	4
#define	NFC_RX_DMA_EVENT	8
#define	NFC_ERR_EVENT		16
#define	NFC_TIMEOUT_EVENT	32
#define NFC_TIMEOUT_VAL		0x1000000

struct sc8810_nand_timing_param {
	u8 acs_time;
	u8 rwh_time;
	u8 rwl_time;
	u8 acr_time;
	u8 rr_time;
	u8 ceh_time;
};

struct sc8810_nand_info {
	struct clk	*clk;
	struct nand_chip *chip;
	unsigned int cfg0_setting;
	unsigned int ecc0_cfg_setting;
	unsigned int ecc1_cfg_setting;
	u8 	asy_cle; //address cycles, can be set 3, 4, 5
	u8	advance;// advance property, can be set 0, 1
	u8	bus_width; //bus width, can be 0 or 1
	u8	ecc_mode; // ecc mode can be 1, 2, 4, 8, 12, 16,24
	u8  	mc_ins_num; // micro instruction number
	u8	mc_addr_ins_num; //micro address instruction number
	u16	ecc_postion; //ecc postion
	u16 	b_pointer; // nfc buffer pointer
	u16 	addr_array[5];// the addrss of the flash to operation
	
};

struct sc8810_nand_page_oob {
	unsigned char m_c;
	unsigned char d_c;
	unsigned char cyc_3;
	unsigned char cyc_4;
	unsigned char cyc_5;
	int pagesize;
	int oobsize; /* total oob size */
	int eccsize; /* per ??? bytes data for ecc calcuate once time */
	int eccbit; /* ecc level per eccsize */
};

#define NF_MC_CMD_ID	(0xFD)
#define NF_MC_ADDR_ID	(0xF1)
#define NF_MC_WAIT_ID	(0xF2)
#define NF_MC_RWORD_ID	(0xF3)
#define NF_MC_RBLK_ID	(0xF4)
#define NF_MC_WWORD_ID	(0xF6)
#define NF_MC_WBLK_ID	(0xF7)
#define NF_MC_DEACTV_ID	(0xF9)
#define NF_MC_NOP_ID	(0xFA)
#define NF_PARA_20M        	0x7ac05      //trwl = 0  trwh = 0
#define NF_PARA_40M        	0x7ac15      //trwl = 1  trwh = 0
#define NF_PARA_53M        	0x7ad26      //trwl = 2  trwh = 1
#define NF_PARA_80M        	0x7ad37      //trwl = 3  trwh = 1
#define NF_PARA_DEFAULT    	0x7ad77      //trwl = 7  trwh = 1

#define REG_AHB_CTL0		       		(*((volatile unsigned int *)(AHB_CTL0)))
#define REG_AHB_SOFT_RST				(*((volatile unsigned int *)(AHB_SOFT_RST)))

#define REG_GR_NFC_MEM_DLY                      (*((volatile unsigned int *)(GR_NFC_MEM_DLY)))

/* only for 4kpage or 8kpage nand flash, no 2kpage */
static struct sc8810_nand_page_oob nand_config_table[] =
{
	{0xec, 0xbc, 0x00, 0x66, 0x56, 4096, 128, 512, 4},
	{0x2c, 0xb3, 0x90, 0x66, 0x64, 4096, 224, 512, 8},
	{0x2c, 0xbc, 0x90, 0x66, 0x54, 4096, 224, 512, 8}
};

static struct sc8810_nand_info g_info ={0};
static nand_ecc_modes_t sprd_ecc_mode = NAND_ECC_NONE;
static __attribute__((aligned(4))) unsigned char io_wr_port[NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE];

struct sc8810_nand_timing_param nand_timing =
{
	50,
	30,
	40,
	40,
	40,
	50
};
static void nfc_reg_write(unsigned int addr, unsigned int value)
{
	writel(value, addr);
}
static unsigned int nfc_reg_read(unsigned int addr)
{
	return readl(addr);
}
static void nand_copy(unsigned char *dst, unsigned char *src, unsigned long len)
{
	u32 i;
	for (i = 0; i < len; i++) {
		*dst = *src;
		dst++;
		src++;
	}
}
static void  nfc_mcr_inst_init(void)
{
	g_info.mc_ins_num = 0;
	g_info.b_pointer = 0;
	g_info.mc_addr_ins_num = 0;
}
void  nfc_mcr_inst_add(u32 ins, u32 mode)
{
	unsigned int offset;
	unsigned int high_flag;
	unsigned int reg_value;
	offset = g_info.mc_ins_num >> 1;
	high_flag = g_info.mc_ins_num & 0x1;
	if(NF_MC_ADDR_ID == mode)
	{
		g_info.addr_array[g_info.mc_addr_ins_num ++] = ins;
	}
	if(high_flag)
	{
		reg_value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		reg_value &= 0x0000ffff;
		reg_value |= ins << 24;
		reg_value |= mode << 16;
	}
	else
	{
		reg_value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		reg_value &= 0xffff0000;
		reg_value |= ins << 8;
		reg_value |= mode;		
	}
	nfc_reg_write(NFC_START_ADDR0 + (offset << 2), reg_value);
	g_info.mc_ins_num ++;
}
static unsigned int nfc_mcr_inst_exc(void)
{
	unsigned int value;
	value = nfc_reg_read(NFC_CFG0);
	if(g_info.chip->options & NAND_BUSWIDTH_16)
	{
		value |= NFC_BUS_WIDTH_16;
	}
	else
	{
		value &= ~NFC_BUS_WIDTH_16;
	}
	value |= (1 << NFC_CMD_SET_OFFSET);
	nfc_reg_write(NFC_CFG0, value);
	value = NFC_CMD_VALID | ((unsigned int)NF_MC_NOP_ID) | ((g_info.mc_ins_num - 1) << 16);
	nfc_reg_write(NFC_CMD, value);
	return 0;
}

static unsigned int nfc_mcr_inst_exc_for_id(void)
{
	unsigned int value;

	value = nfc_reg_read(NFC_CFG0);
	value &= ~NFC_BUS_WIDTH_16;
	value |= (1 << NFC_CMD_SET_OFFSET);

	nfc_reg_write(NFC_CFG0, value);
	value = NFC_CMD_VALID | ((unsigned int)NF_MC_NOP_ID) |((g_info.mc_ins_num - 1) << 16);
	nfc_reg_write(NFC_CMD, value);
	return 0;
}

static void sc8810_nand_wp_en(int en)
{
	unsigned int value;
	if(en)
	{
		value = nfc_reg_read(NFC_CFG0);
		value &= ~ NFC_WPN;
		nfc_reg_write(NFC_CFG0, value);
	}
	else
	{
		value = nfc_reg_read(NFC_CFG0);
		value |= NFC_WPN;
		nfc_reg_write(NFC_CFG0, value);		
	}
}

static int sc8810_nfc_wait_command_finish(unsigned int flag)
{
	unsigned int event = 0;
	unsigned int value;
	unsigned int counter = 0;
	while(((event & flag) != flag) && (counter < NFC_TIMEOUT_VAL/*time out*/))
	{
		value = nfc_reg_read(NFC_CLR_RAW);
		if(value & NFC_ECC_DONE_RAW)
		{
			event |= NFC_ECC_EVENT;
		}
		if(value & NFC_DONE_RAW)
		{
			event |= NFC_DONE_EVENT;
		}
		counter ++;
	}
	nfc_reg_write(NFC_CLR_RAW, 0xffff0000); //clear all interrupt status

	if(counter > NFC_TIMEOUT_VAL)
	{
		while (1);
		return -1;
	}
	return 0;
}
unsigned int ecc_mode_convert(u32 mode)
{
	u32 mode_m;
	switch(mode)
	{
	case 1:
		mode_m = 0;
		break;
	case 2:
		mode_m = 1;
		break;
	case 4:
		mode_m = 2;
		break;
	case 8:
		mode_m = 3;
		break;
	case 12:
		mode_m = 4;
		break;
	case 16:
		mode_m = 5;
		break;
	case 24:
		mode_m = 6;
		break;
	default:
		mode_m = 0;
		break;
	}
	return mode_m;
}
unsigned int sc8810_ecc_encode(struct sc8810_ecc_param *param)
{
	u32 reg;
	reg = (param->m_size - 1);
	nand_copy((void *)NFC_MBUF_ADDR, param->p_mbuf, param->m_size);
	nfc_reg_write(NFC_ECC_CFG1, reg);	
	reg = 0;
	reg = (ecc_mode_convert(param->mode)) << NFC_ECC_MODE_OFFSET;
	reg |= (param->ecc_pos << NFC_ECC_SP_POS_OFFSET) | ((param->sp_size - 1) << NFC_ECC_SP_SIZE_OFFSET) | ((param->ecc_num -1)<< NFC_ECC_NUM_OFFSET);
	reg |= NFC_ECC_ACTIVE;
	nfc_reg_write(NFC_ECC_CFG0, reg);
	sc8810_nfc_wait_command_finish(NFC_ECC_EVENT);
	nand_copy(param->p_sbuf, (u8 *)NFC_SBUF_ADDR,param->sp_size);

	return 0;
}
static u32 sc8810_get_decode_sts(void)
{
	u32 err;
	err = nfc_reg_read(NFC_ECC_STS0);
	err &= 0x1f;

	if(err == 0x1f)
		return -1;

	return err;
}

static u32 sc8810_ecc_decode(struct sc8810_ecc_param *param) 
{
	u32 reg;
	u32 ret = 0;
	s32 size = 0;

	nand_copy((void *)NFC_MBUF_ADDR, param->p_mbuf, param->m_size);
	nand_copy((void *)NFC_SBUF_ADDR, param->p_sbuf, param->sp_size);
	reg = (param->m_size - 1);
	nfc_reg_write(NFC_ECC_CFG1, reg);
	reg = 0;
	reg = (ecc_mode_convert(param->mode)) << NFC_ECC_MODE_OFFSET;
	reg |= (param->ecc_pos << NFC_ECC_SP_POS_OFFSET) | ((param->sp_size - 1) << NFC_ECC_SP_SIZE_OFFSET) | ((param->ecc_num - 1) << NFC_ECC_NUM_OFFSET);
	reg |= NFC_ECC_DECODE;
	reg |= NFC_ECC_ACTIVE;
	nfc_reg_write(NFC_ECC_CFG0, reg);
	sc8810_nfc_wait_command_finish(NFC_ECC_EVENT);
	ret = sc8810_get_decode_sts();
	
	if (ret != 0 && ret != -1) {
//		printk(KERN_INFO "sc8810_ecc_decode sts = %x\n",ret);
	}
	if (ret == -1) {
//		printk(KERN_INFO "(%x),(%x),(%x),(%x)\n",param->p_sbuf[0],param->p_sbuf[1],param->p_sbuf[2],param->p_sbuf[3]);

		//FIXME:
		size = param->sp_size;
		if (size > 0) {
			while (size--)
			{
				if (param->p_sbuf[size] != 0xff)
					break;
			}
			if (size < 0)
			{
				size = param->m_size;
				if (size > 0)
				{
					while (size--)
					{
						if (param->p_mbuf[size] != 0xff)
							break;
					}
					if (size < 0) {
						ret = 0;
					}
				}
			}
		}

	}

	if ((ret != -1) && (ret != 0))
	{
		nand_copy(param->p_mbuf, (void *)NFC_MBUF_ADDR, param->m_size);
		nand_copy(param->p_sbuf, (void *)NFC_SBUF_ADDR, param->sp_size);
		ret = 0;
	}

	return ret;
}

static void set_nfc_param(unsigned long nfc_clk)
{
	u32 value = 0;
	u32 cycles;
	cycles = nand_timing.acs_time / (1000000000 / nfc_clk);
	value |= (cycles << NFC_ACS_OFFSET);
	
	cycles = nand_timing.rwh_time / (1000000000 / nfc_clk);
	value |= (cycles << NFC_RWH_OFFSET);
	
	cycles = nand_timing.rwl_time / (1000000000 / nfc_clk);
	value |= (cycles << NFC_RWL_OFFSET);
	
	cycles = nand_timing.acr_time / (1000000000 / nfc_clk);
	value |= (cycles << NFC_ACR_OFFSET);
	
	cycles = nand_timing.rr_time / (1000000000 / nfc_clk);
	value |= (cycles << NFC_RR_OFFSET);
	
	cycles = nand_timing.ceh_time / (1000000000 / nfc_clk);
	value |= (cycles << NFC_CEH_OFFSET);
	nfc_reg_write(NFC_TIMING, value);

//	local_irq_restore(flags);	
}
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


static void sc8810_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	nand_copy(buf, g_info.b_pointer + io_wr_port,len);
	g_info.b_pointer += len;
}
static void sc8810_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				   int len)
{
	nand_copy(g_info.b_pointer + io_wr_port, (unsigned char*)buf,len);
	g_info.b_pointer += len;
}
static u_char sc8810_nand_read_byte(struct mtd_info *mtd)
{
	u_char ch;
	ch = io_wr_port[g_info.b_pointer ++];
	return ch;
}
static u16 sc8810_nand_read_word(struct mtd_info *mtd)
{
	u16 ch = 0;
	ch = io_wr_port[g_info.b_pointer ++];
	ch |= io_wr_port[g_info.b_pointer ++] << 8;

	return ch;
}

static void sc8810_nand_data_add(unsigned int bytes, unsigned int bus_width, unsigned int read)
{
	unsigned int word;
	unsigned int blk;
	if(!bus_width)
	{
		blk = bytes >> 8;
		word = bytes & 0xff;
	}
	else
	{
		blk = bytes >> 9;
		word = (bytes & 0x1ff) >> 1;
	}
	if(read)
	{
		if(blk)
		{
			nfc_mcr_inst_add(blk - 1, NF_MC_RBLK_ID);
		}
		if(word)
		{
			nfc_mcr_inst_add(word - 1, NF_MC_RWORD_ID);
		}
	}
	else
	{
		if(blk)
		{
			nfc_mcr_inst_add(blk - 1, NF_MC_WBLK_ID);
		}
		if(word)
		{
			nfc_mcr_inst_add(word - 1, NF_MC_WWORD_ID);
		}
	}
}

static void sc8810_nand_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	struct nand_chip *chip = (struct nand_chip *)(mtd->priv);
	u32 size = 0;
	if (ctrl & NAND_CLE) {
		switch (cmd) {
		case NAND_CMD_RESET:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT);
			break;
		case NAND_CMD_STATUS:
			nfc_mcr_inst_init();
			nfc_reg_write(NFC_CMD, 0x80000070);
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT);
			nand_copy(io_wr_port, (void *)NFC_ID_STS, 1);
			break;
		case NAND_CMD_READID:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0x00, NF_MC_ADDR_ID);
			nfc_mcr_inst_add(7, NF_MC_RWORD_ID);
			nfc_mcr_inst_exc_for_id();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT);
			nand_copy(io_wr_port, (void *)NFC_MBUF_ADDR, 5);
			break;					
		case NAND_CMD_ERASE1:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			break;
		case NAND_CMD_ERASE2:
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);	
			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT);
			break;	
		case NAND_CMD_READ0:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			break;	
		case NAND_CMD_READSTART:
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);			
			if((!g_info.addr_array[0]) && (!g_info.addr_array[1]) )//main part
			{
				size = mtd->writesize +mtd->oobsize;				
			}
			else
			{
				size = mtd->oobsize;
			}
			sc8810_nand_data_add(size, chip->options & NAND_BUSWIDTH_16, 1);

			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT);
			nand_copy(io_wr_port, (void *)NFC_MBUF_ADDR, size);
			break;	
		case NAND_CMD_SEQIN:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(NAND_CMD_SEQIN, NF_MC_CMD_ID);
			break;	
		case NAND_CMD_PAGEPROG:
			nand_copy((void *)NFC_MBUF_ADDR, io_wr_port, g_info.b_pointer);
			sc8810_nand_data_add(g_info.b_pointer, chip->options & NAND_BUSWIDTH_16, 0);
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);
			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT);
			break;	
		default :
		break;						
		}
	}
	else if(ctrl & NAND_ALE) {
		nfc_mcr_inst_add(cmd & 0xff, NF_MC_ADDR_ID);
	}
}
static int sc8810_nand_devready(struct mtd_info *mtd)
{
	unsigned long value = 0;

	value = nfc_reg_read(NFC_CMD);
	if ((value & NFC_CMD_VALID) != 0) 		
		return 0; 
	else
		return 1; /* ready */
}
static void sc8810_nand_select_chip(struct mtd_info *mtd, int chip)
{
	//struct nand_chip *this = mtd->priv;
	//struct sprd_nand_info *info = this->priv;
	/* clk_enable(info->clk) */
	int ik_cnt = 0;
	
	if (chip != -1) {
		REG_AHB_CTL0 |= BIT_8;//no BIT_9 /* enabel nfc clock */
		for(ik_cnt = 0; ik_cnt < 10000; ik_cnt ++);
	} else
		REG_AHB_CTL0 &= ~BIT_8; /* disabel nfc clock */
		
}
static int sc8810_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	struct sc8810_ecc_param param;
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	param.mode = g_info.ecc_mode;
	param.ecc_num = 1;
	param.sp_size = this->ecc.bytes;
	param.ecc_pos = 0;
	param.m_size = this->ecc.size;
	param.p_mbuf = (u8 *)dat;
	param.p_sbuf = ecc_code;	
	
	if (sprd_ecc_mode == NAND_ECC_WRITE) {
		sc8810_ecc_encode(&param);
		sprd_ecc_mode = NAND_ECC_NONE;
	}	
	return 0;
}
static void sc8810_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	sprd_ecc_mode = mode;
}
static int sc8810_nand_correct_data(struct mtd_info *mtd, uint8_t *dat,
				     uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct sc8810_ecc_param param;
	
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	int ret = 0;
	param.mode = g_info.ecc_mode;
	param.ecc_num = 1;
	param.sp_size = this->ecc.bytes;
	param.ecc_pos = 0;
	param.m_size = this->ecc.size;
	param.p_mbuf = dat;
	param.p_sbuf = read_ecc;
	ret = sc8810_ecc_decode(&param);

	return ret;	
}

static void sc8810_nand_hw_init(void)
{
	int ik_cnt = 0;

	REG_AHB_CTL0 |= BIT_8;//no BIT_9
	REG_AHB_SOFT_RST |= BIT_5;
	for(ik_cnt = 0; ik_cnt < 0xffff; ik_cnt++);
	REG_AHB_SOFT_RST &= ~BIT_5;

	sc8810_nand_wp_en(0);
	nfc_reg_write(NFC_TIMING, ((6 << 0) | (6 << 5) | (10 << 10) | (6 << 16) | (5 << 21) | (5 << 26)));	
	nfc_reg_write(NFC_TIMING+0X4, 0xffffffff);//TIMEOUT
//	set_nfc_param(1);//53MHz
}
static struct nand_ecclayout _nand_oob_128 = {
	.eccbytes = 56,
	.eccpos = {
		    72, 73, 74, 75, 76, 77, 78, 79,
		    80,  81,  82,  83,  84,  85,  86,  87,
		    88,  89,  90,  91,  92,  93,  94,  95,
		    96,  97,  98,  99, 100, 101, 102, 103,
		   104, 105, 106, 107, 108, 109, 110, 111,
		   112, 113, 114, 115, 116, 117, 118, 119,
		   120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = {
		{.offset = 2,
		 .length = 70}}
};

static struct nand_ecclayout _nand_oob_224 = {
	.eccbytes = 104,
	.eccpos = {
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132,
		133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145,
		146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158,
		159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171,
		172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184,
		185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197,
		198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
		211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223},
	.oobfree = {
		{.offset = 2,
		.length = 118}}
};

void nand_hardware_config(struct mtd_info *mtd, struct nand_chip *this, u8 id[8])
{
	int index;
	int array;
	
	for (index = 0; index < 5; index ++)
		printk(" %02x ", id[index]);
	printk("\n");

	array = sizeof(nand_config_table) / sizeof(struct sc8810_nand_page_oob);
	for (index = 0; index < array; index ++) {
		if ((nand_config_table[index].m_c == id[0]) && (nand_config_table[index].d_c == id[1]) && (nand_config_table[index].cyc_3 == id[2]) && (nand_config_table[index].cyc_4 == id[3]) && (nand_config_table[index].cyc_5 == id[4]))
			break;
	}

	if (index < array) {
		this->ecc.size = nand_config_table[index].eccsize;
		g_info.ecc_mode = nand_config_table[index].eccbit;
		/* 4 bit ecc, per 512 bytes can creat 13 * 4 = 52 bit , 52 / 8 = 7 bytes
		   8 bit ecc, per 512 bytes can creat 13 * 8 = 104 bit , 104 / 8 = 13 bytes */
		switch (g_info.ecc_mode) {
			case 4:
				/* 4 bit ecc, per 512 bytes can creat 13 * 4 = 52 bit , 52 / 8 = 7 bytes */
				this->ecc.bytes = 7;
				this->ecc.layout = &_nand_oob_128;
			break;
			case 8:
				/* 8 bit ecc, per 512 bytes can creat 13 * 8 = 104 bit , 104 / 8 = 13 bytes */
				this->ecc.bytes = 13;
				this->ecc.layout = &_nand_oob_224;
				mtd->oobsize = nand_config_table[index].oobsize;
			break;
		}
	} else 
		printk("The type of nand flash is not in table, so use default configuration!\n");
}

int board_nand_init(struct nand_chip *this)
{
	g_info.chip = this;
	g_info.ecc_mode = CONFIG_SYS_NAND_ECC_MODE;
	sc8810_nand_hw_init();

	this->IO_ADDR_R = this->IO_ADDR_W = (void __iomem*)NFC_MBUF_ADDR;
	this->cmd_ctrl = sc8810_nand_hwcontrol;
	this->dev_ready = sc8810_nand_devready;
	this->select_chip = sc8810_nand_select_chip;

	this->ecc.calculate = sc8810_nand_calculate_ecc;
	this->ecc.correct = sc8810_nand_correct_data;
	this->ecc.hwctl = sc8810_nand_enable_hwecc;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = CONFIG_SYS_NAND_ECCSIZE;//512;
	this->ecc.bytes = CONFIG_SYS_NAND_ECCBYTES;//3
	this->read_buf = sc8810_nand_read_buf;
	this->write_buf = sc8810_nand_write_buf;
	this->read_byte	= sc8810_nand_read_byte;
	this->read_word	= sc8810_nand_read_word;
	
	this->chip_delay = 20;
	this->priv = &g_info;
	this->options |= NAND_BUSWIDTH_16;
	return 0;
}
static struct sprd_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev->dev.platform_data;
}


//linux driver layout

/*device drive  registration */
static struct mtd_info *sprd_mtd = NULL;
#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

static int sprd_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct sprd_nand_info info;
	struct sprd_platform_nand *plat = pdev->dev.platform_data;/* get timing */
	struct resource *regs = NULL;

	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
	
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev,"resources unusable\n");
		goto Err;
	}
	nand_base = regs->start;

	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);

	memset(&info, 0 , sizeof(struct sprd_nand_info));

	platform_set_drvdata(pdev, &info);/* platform_device.device.driver_data IS info */
	info.platform = plat; /* nand timing */
	info.pdev = pdev;
	sprd_nand_inithw(&info, pdev);

	sprd_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	this = (struct nand_chip *)(&sprd_mtd[1]);
	memset((char *)sprd_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

	sprd_mtd->priv = this;

	if (1) {
		sprd_config_nand_pins16();
		this->options |= NAND_BUSWIDTH_16;
		this->options |= NAND_NO_READRDY;
	} else {		
		sprd_config_nand_pins8();
	}

	board_nand_init(this);

	/* scan to find existance of the device */
	nand_scan(sprd_mtd, 1);	

	sprd_mtd->name = "sprd-nand";
	num_partitions = parse_mtd_partitions(sprd_mtd, part_probes, &partitions, 0);

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
	
	REG_AHB_CTL0 &= ~BIT_8; /* disabel nfc clock */

	return 0;
release:
	nand_release(sprd_mtd);
Err:
	return 0;
}

/* device management functions */
static int sprd_nand_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	del_mtd_partitions(sprd_mtd);
	del_mtd_device(sprd_mtd);
	kfree(sprd_mtd);

	return 0;
}

/* PM Support */
#ifdef CONFIG_PM
static unsigned long nfc_reg_cfg0;
static int sprd_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
#if 0
	struct sprd_nand_info *info = platform_get_drvdata(dev);

	if (info)
		clk_disable(info->clk);
#else
	nfc_reg_cfg0 = nfc_reg_read(NFC_CFG0); /* save CS_SEL */
	nfc_reg_write(NFC_CFG0, (nfc_reg_cfg0 | BIT_6)); /* deset CS_SEL */
	REG_AHB_CTL0 &= ~BIT_8; /* disabel nfc clock */
#endif

	return 0;
}

static int sprd_nand_resume(struct platform_device *dev)
{
#if 0
	struct sprd_nand_info *info = platform_get_drvdata(dev);

	if (info) {
		clk_enable(info->clk);
		sprd_nand_inithw(info, dev);
	}
#else
	int ik_cnt = 0;

	REG_AHB_CTL0 |= BIT_8;//no BIT_9 /* enable nfc clock */
	REG_AHB_SOFT_RST |= BIT_5;
	for(ik_cnt = 0; ik_cnt < 0xffff; ik_cnt++);
	REG_AHB_SOFT_RST &= ~BIT_5;

	nfc_reg_write(NFC_CFG0, nfc_reg_cfg0); /* set CS_SEL */
	sc8810_nand_wp_en(0);
	nfc_reg_write(NFC_TIMING, ((6 << 0) | (6 << 5) | (10 << 10) | (6 << 16) | (5 << 21) | (5 << 26)));	
	nfc_reg_write(NFC_TIMING+0X4, 0xffffffff);//TIMEOUT
#endif

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
	printk("\nSpreadtrum NAND Driver, (c) 2011 Spreadtrum\n");
	return platform_driver_register(&sprd_nand_driver);
}

static void __exit sprd_nand_exit(void)
{
	platform_driver_unregister(&sprd_nand_driver);
}

module_init(sprd_nand_init);
module_exit(sprd_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("spreadtrum.com>");
MODULE_DESCRIPTION("SPRD 8810 MTD NAND driver");

