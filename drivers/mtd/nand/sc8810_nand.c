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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <mach/globalregs.h>
#include "sc8810_nand.h"

struct sprd_nand_info {
    unsigned long           phys_base;
	struct mtd_info		    mtd;
	struct platform_device	*pdev;
	struct nand_chip 	    *chip;
	u8                      ecc_mode;
	u8  			        mc_ins_num;
	u8			            mc_addr_ins_num;
	u16                     b_pointer;
	u16 			        addr_array[5];
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

static struct sprd_nand_info g_info;
static nand_ecc_modes_t sprd_ecc_mode = NAND_ECC_NONE;
static nfc_status_t nfc_cmd_result_status = NFC_CMD_OPER_OK;
static __attribute__((aligned(4))) unsigned char io_wr_port[NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE];

/* 4kB or 8KB PageSize nand flash config table
 * id(5 bytes), pagesize, oobsize, 512, eccbit
 * TODO : u-boot can transfer the setting here
 */
static struct sc8810_nand_page_oob nand_config_table[] =
{
	{0xec, 0xbc, 0x00, 0x66, 0x56, 4096, 128, 512, 4},
	{0x2c, 0xb3, 0x90, 0x66, 0x64, 4096, 224, 512, 8},
	{0x2c, 0xbc, 0x90, 0x66, 0x54, 4096, 224, 512, 8}
};

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

static void nfc_reg_write(unsigned int addr, unsigned int value)
{
	writel(value, addr);
}

static unsigned int nfc_reg_read(unsigned int addr)
{
	return readl(addr);
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

	if (NF_MC_ADDR_ID == mode)
		g_info.addr_array[g_info.mc_addr_ins_num ++] = ins;

	if (high_flag) {
		reg_value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		reg_value &= 0x0000ffff;
		reg_value |= ins << 24;
		reg_value |= mode << 16;
	} else {
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

	if (g_info.chip->options & NAND_BUSWIDTH_16)
		value |= NFC_BUS_WIDTH_16;
	else
		value &= ~NFC_BUS_WIDTH_16;

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
	value = NFC_CMD_VALID | ((unsigned int)NF_MC_NOP_ID) | ((g_info.mc_ins_num - 1) << 16);
	nfc_reg_write(NFC_CMD, value);

	return 0;
}

static void sc8810_nand_wp_en(int en)
{
	unsigned int value;

	if (en) {
		value = nfc_reg_read(NFC_CFG0);
		value &= ~ NFC_WPN;
		nfc_reg_write(NFC_CFG0, value);
	} else {
		value = nfc_reg_read(NFC_CFG0);
		value |= NFC_WPN;
		nfc_reg_write(NFC_CFG0, value);
	}
}

static void sc8810_nand_hw_init(void)
{
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, AHB_SOFT_NFC_RST, AHB_SOFT_RST);
	mdelay(2);
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, AHB_SOFT_NFC_RST, AHB_SOFT_RST);

	sc8810_nand_wp_en(0);
	nfc_reg_write(NFC_TIMING, NFC_DEFAULT_TIMING);
	nfc_reg_write(NFC_TIEOUT, 0xffffffff);


}

#if 0
static int sc8810_nfc_wait_command_finish(unsigned int flag)
{
	unsigned int event = 0;
	unsigned int value;
	unsigned int counter = 0;

	while (((event & flag) != flag) && (counter < NFC_TIMEOUT_VAL)) {
		value = nfc_reg_read(NFC_CLR_RAW);
		if(value & NFC_ECC_DONE_RAW)
			event |= NFC_ECC_EVENT;

		if(value & NFC_DONE_RAW)
			event |= NFC_DONE_EVENT;
		counter ++;
	}

	nfc_reg_write(NFC_CLR_RAW, 0xffff0000);
	if (counter > NFC_TIMEOUT_VAL)
		return -1;

	return 0;
}
#else
static int sc8810_nfc_wait_command_finish(unsigned int flag, int cmd)
{
	unsigned int event = 0;
	unsigned int value;
	unsigned int counter = 0;

	nfc_cmd_result_status = NFC_CMD_OPER_OK;
	while (((event & flag) != flag) && (counter < NFC_TIMEOUT_VAL)) {
		value = nfc_reg_read(NFC_CLR_RAW);

		if (value & NFC_ECC_DONE_RAW)
			event |= NFC_ECC_EVENT;

		if (value & NFC_DONE_RAW)
			event |= NFC_DONE_EVENT;
		counter ++;

		if ((cmd == NAND_CMD_ERASE2) && (counter >= NFC_ERASE_TIMEOUT)) {
			nfc_cmd_result_status = NFC_CMD_OPER_TIMEOUT;
			break;
		} else if ((cmd == NAND_CMD_READSTART) && (counter >= NFC_READ_TIMEOUT)) {
			nfc_cmd_result_status = NFC_CMD_OPER_TIMEOUT;
			break;
		} else if ((cmd == NAND_CMD_PAGEPROG) && (counter >= NFC_WRITE_TIMEOUT)) {
			nfc_cmd_result_status = NFC_CMD_OPER_TIMEOUT;
			break;
		}
	}

	nfc_reg_write(NFC_CLR_RAW, 0xffff0000);
	if (nfc_cmd_result_status == NFC_CMD_OPER_TIMEOUT) {
		printk("nfc cmd[0x%08x] timeout[0x%08x] and reset nand controller.\n", cmd, counter);
		sc8810_nand_hw_init();
	} else if (counter >= NFC_TIMEOUT_VAL)
		panic("nfc cmd timeout!!!");

	return 0;
}
#endif

unsigned int ecc_mode_convert(u32 mode)
{
	u32 mode_m;

	switch (mode) {
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
	memcpy((void *)NFC_MBUF_ADDR, param->p_mbuf, param->m_size);
	nfc_reg_write(NFC_ECC_CFG1, reg);
	reg = 0;
	reg = (ecc_mode_convert(param->mode)) << NFC_ECC_MODE_OFFSET;
	reg |= (param->ecc_pos << NFC_ECC_SP_POS_OFFSET) | ((param->sp_size - 1) << NFC_ECC_SP_SIZE_OFFSET) | ((param->ecc_num -1)<< NFC_ECC_NUM_OFFSET);
	reg |= NFC_ECC_ACTIVE;
	nfc_reg_write(NFC_ECC_CFG0, reg);
	sc8810_nfc_wait_command_finish(NFC_ECC_EVENT, -1);
	memcpy(param->p_sbuf, (u8 *)NFC_SBUF_ADDR,param->sp_size);

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

	memcpy((void *)NFC_MBUF_ADDR, param->p_mbuf, param->m_size);
	memcpy((void *)NFC_SBUF_ADDR, param->p_sbuf, param->sp_size);

	reg = (param->m_size - 1);
	nfc_reg_write(NFC_ECC_CFG1, reg);
	reg = 0;
	reg = (ecc_mode_convert(param->mode)) << NFC_ECC_MODE_OFFSET;
	reg |= (param->ecc_pos << NFC_ECC_SP_POS_OFFSET) | ((param->sp_size - 1) << NFC_ECC_SP_SIZE_OFFSET) | ((param->ecc_num - 1) << NFC_ECC_NUM_OFFSET);

	reg |= NFC_ECC_DECODE;
	reg |= NFC_ECC_ACTIVE;
	nfc_reg_write(NFC_ECC_CFG0, reg);
	sc8810_nfc_wait_command_finish(NFC_ECC_EVENT, -1);

	ret = sc8810_get_decode_sts();
	if (ret == -1) {
		size = param->sp_size;
		if (size > 0) {
			while (size --) {
				if (param->p_sbuf[size] != 0xff)
					break;
			}

			if (size < 0) {
				size = param->m_size;
				if (size > 0) {
					while (size --) {
						if (param->p_mbuf[size] != 0xff)
							break;
					}

					if (size < 0)
						ret = 0;
				}
			}
		}
	}

	if ((ret != -1) && (ret != 0)) {
		memcpy(param->p_mbuf, (void *)NFC_MBUF_ADDR, param->m_size);
		memcpy(param->p_sbuf, (void *)NFC_SBUF_ADDR, param->sp_size);
		ret = 0;
	}

	return ret;
}

static void sc8810_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	memcpy(buf, g_info.b_pointer + io_wr_port,len);
	g_info.b_pointer += len;
}

static void sc8810_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	memcpy(g_info.b_pointer + io_wr_port, (unsigned char*)buf,len);
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

	if (!bus_width) {
		blk = bytes >> 8;
		word = bytes & 0xff;
	} else {
		blk = bytes >> 9;
		word = (bytes & 0x1ff) >> 1;
	}

	if (read) {
		if(blk)
			nfc_mcr_inst_add(blk - 1, NF_MC_RBLK_ID);
		if(word)
			nfc_mcr_inst_add(word - 1, NF_MC_RWORD_ID);
	} else {
		if (blk)
			nfc_mcr_inst_add(blk - 1, NF_MC_WBLK_ID);
		if (word)
			nfc_mcr_inst_add(word - 1, NF_MC_WWORD_ID);
	}
}

static void sc8810_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = (struct nand_chip *)(mtd->priv);
	u32 size = 0;

	if (ctrl & NAND_CLE) {
		switch (cmd) {
		case NAND_CMD_RESET:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd);
			break;
		case NAND_CMD_STATUS:
			nfc_mcr_inst_init();
			nfc_reg_write(NFC_CMD, 0x80000070);
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd);
			memcpy(io_wr_port, (void *)NFC_ID_STS, 1);
			break;
		case NAND_CMD_READID:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0x00, NF_MC_ADDR_ID);
			nfc_mcr_inst_add(7, NF_MC_RWORD_ID);
			nfc_mcr_inst_exc_for_id();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd);
			memcpy(io_wr_port, (void *)NFC_MBUF_ADDR, 5);
			break;
		case NAND_CMD_ERASE1:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			break;
		case NAND_CMD_ERASE2:
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);
			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd);
			break;
		case NAND_CMD_READ0:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			break;
		case NAND_CMD_READSTART:
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);
			if((!g_info.addr_array[0]) && (!g_info.addr_array[1]) )
				size = mtd->writesize +mtd->oobsize;
			else
				size = mtd->oobsize;
			sc8810_nand_data_add(size, chip->options & NAND_BUSWIDTH_16, 1);

			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd);
			memcpy(io_wr_port, (void *)NFC_MBUF_ADDR, size);
			break;
		case NAND_CMD_SEQIN:
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(NAND_CMD_SEQIN, NF_MC_CMD_ID);
			break;
		case NAND_CMD_PAGEPROG:
			memcpy((void *)NFC_MBUF_ADDR, io_wr_port, g_info.b_pointer);
			sc8810_nand_data_add(g_info.b_pointer, chip->options & NAND_BUSWIDTH_16, 0);
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);
			nfc_mcr_inst_exc();
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd);
			break;
		}
	} else if (ctrl & NAND_ALE)
		nfc_mcr_inst_add(cmd & 0xff, NF_MC_ADDR_ID);
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
	if (chip != -1)
		sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);
	else
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);
}

static nfc_status_t sc8810_nfc_operation_status(struct mtd_info *mtd)
{
	return nfc_cmd_result_status;
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

static int sc8810_nand_correct_data(struct mtd_info *mtd, uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
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

void nand_hardware_config(struct mtd_info *mtd, struct nand_chip *this, u8 id[8])
{
	int index;
	int array;

	array = sizeof(nand_config_table) / sizeof(struct sc8810_nand_page_oob);
	for (index = 0; index < array; index ++) {
		if ((nand_config_table[index].m_c == id[0]) && (nand_config_table[index].d_c == id[1]) && (nand_config_table[index].cyc_3 == id[2]) && (nand_config_table[index].cyc_4 == id[3]) && (nand_config_table[index].cyc_5 == id[4]))
			break;
	}

	if (index < array) {
		this->ecc.size = nand_config_table[index].eccsize;
		g_info.ecc_mode = nand_config_table[index].eccbit;
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

	this->IO_ADDR_R = (void __iomem*)NFC_MBUF_ADDR;
	this->IO_ADDR_W = this->IO_ADDR_R;
	this->cmd_ctrl = sc8810_nand_hwcontrol;
	this->dev_ready = sc8810_nand_devready;
	this->select_chip = sc8810_nand_select_chip;

	this->ecc.calculate = sc8810_nand_calculate_ecc;
	this->ecc.correct = sc8810_nand_correct_data;
	this->ecc.hwctl = sc8810_nand_enable_hwecc;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = CONFIG_SYS_NAND_ECCSIZE;
	this->ecc.bytes = CONFIG_SYS_NAND_ECCBYTES;
	this->read_buf = sc8810_nand_read_buf;
	this->write_buf = sc8810_nand_write_buf;
	this->read_byte	= sc8810_nand_read_byte;
	this->read_word	= sc8810_nand_read_word;
	this->nfc_operation_status = sc8810_nfc_operation_status;

	this->chip_delay = 20;
	this->priv = &g_info;
	this->options |= NAND_BUSWIDTH_16;

	return 0;
}

static struct mtd_info *sprd_mtd = NULL;
#ifdef CONFIG_MTD_CMDLINE_PARTS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

static int sprd_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *this;
	struct resource *regs = NULL;
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev,"resources unusable\n");
		goto Err;
	}

	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
	memset(&g_info, 0 , sizeof(struct sprd_nand_info));

	platform_set_drvdata(pdev, &g_info);
	g_info.pdev = pdev;

	sprd_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	this = (struct nand_chip *)(&sprd_mtd[1]);
	memset((char *)sprd_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

	sprd_mtd->priv = this;

	this->options |= NAND_BUSWIDTH_16;
	this->options |= NAND_NO_READRDY;

	board_nand_init(this);
	nand_scan(sprd_mtd, 1);

	sprd_mtd->name = "sprd-nand";
#ifdef CONFIG_MTD_CMDLINE_PARTS
	num_partitions = parse_mtd_partitions(sprd_mtd, part_probes, &partitions, 0);
#endif

	if ((!partitions) || (num_partitions == 0)) {
		printk("No parititions defined, or unsupported device.\n");
		goto release;
	}
	mtd_device_register(sprd_mtd, partitions, num_partitions);
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);

	return 0;
release:
	nand_release(sprd_mtd);
Err:
	return 0;
}

static int sprd_nand_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	nand_release(sprd_mtd);
	kfree(sprd_mtd);

	return 0;
}

#ifdef CONFIG_PM
static unsigned long nfc_reg_cfg0;
static int sprd_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
	nfc_reg_cfg0 = nfc_reg_read(NFC_CFG0);
	nfc_reg_write(NFC_CFG0, (nfc_reg_cfg0 | NFC_CS1_SEL)); /* deset CS_SEL */
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);

	return 0;
}

static int sprd_nand_resume(struct platform_device *dev)
{
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);
	sprd_greg_set_bits(REG_TYPE_AHB_GLOBAL, AHB_SOFT_NFC_RST, AHB_SOFT_RST);
	mdelay(2);
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, AHB_SOFT_NFC_RST, AHB_SOFT_RST);

	nfc_reg_write(NFC_CFG0, nfc_reg_cfg0);
	sc8810_nand_wp_en(0);
	nfc_reg_write(NFC_TIMING, NFC_DEFAULT_TIMING);
	nfc_reg_write(NFC_TIEOUT, 0xffffffff);

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
		.name	= "sprd-nand",
		.owner	= THIS_MODULE,
	},
};

static int __init sprd_nand_init(void)
{
	return platform_driver_register(&sprd_nand_driver);
}

static void __exit sprd_nand_exit(void)
{
	platform_driver_unregister(&sprd_nand_driver);
}

module_init(sprd_nand_init);
module_exit(sprd_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("spreadtrum.com");
MODULE_DESCRIPTION("SPRD 8810 MTD NAND driver");
MODULE_ALIAS("platform:sprd-nand");
