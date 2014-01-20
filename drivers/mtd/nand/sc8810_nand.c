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
#include <mach/irqs.h>
#include "sc8810_nand.h"
#include <linux/string.h>
#include <linux/wakelock.h>
#include <linux/sched.h>

#define NAND_IRQ_EN
#ifdef  NAND_IRQ_EN
#include <linux/completion.h>
#define DRIVER_NAME "sc8810_nand"
#define IRQ_TIMEOUT  100//unit:ms,IRQ timeout value
#endif

static struct nand_spec_str *ptr_nand_spec = NULL;
static int have_no_nand = 0;
struct sprd_nand_info {
	unsigned long		phys_base;
	struct mtd_info		mtd;
	struct platform_device	*pdev;
	struct nand_chip	*chip;
	u8			ecc_mode;
	u8			mc_ins_num;
	u8			mc_addr_ins_num;
	u16			b_pointer;
	u16			addr_array[5];
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

struct sc8810_nand_timing_param {
	//u8 ancs_time;
	u8 rwh_time;
	u8 rwl_time;
	u8 acr_time;
	u8 rr_time;
	u8 ceh_time;
        u8 als_time;
        u8 cls_time;
        u8 rea_time;
};

struct nand_spec_str{
    u8          mid;
    u8          did;
    u8          id3;
    u8          id4;
    u8          id5;
    struct sc8810_nand_timing_param timing_cfg;
};

#ifdef  NAND_IRQ_EN
static struct completion  sc8810_op_completion;
static enum NAND_HANDLE_STATUS_S  handle_status=NAND_HANDLE_DONE;
#endif

static struct sprd_nand_info g_info;
static nand_ecc_modes_t sprd_ecc_mode = NAND_ECC_NONE;
static __attribute__((aligned(4))) unsigned char io_wr_port[NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE];
static unsigned char fix_timeout_id[8];
static unsigned long fix_timeout_reg[8];
static struct wake_lock nfc_wakelock;
static void set_nfc_timing(struct sc8810_nand_timing_param *nand_timing, u32 nfc_clk_MHz);

/* 4kB or 8KB PageSize nand flash config table
 * id(5 bytes), pagesize, oobsize, 512, eccbit
 * TODO : u-boot can transfer the setting here
 */
/* only for special 4kpage or 8kpage nand flash, no 2kpage or normal 4kpage or normal 8kpage */
static const struct sc8810_nand_page_oob nand_config_table[] =
{
	{0x98, 0xba, 0x90, 0x55, 0x76, 2048, 128, 512, 8}, //add for 6821
	{0xec, 0xbc, 0x00, 0x66, 0x56, 4096, 128, 512, 4},
	{0x2c, 0xb3, 0x90, 0x66, 0x64, 4096, 224, 512, 8},
	{0x2c, 0xbc, 0x90, 0x66, 0x54, 4096, 224, 512, 8},
	{0xec, 0xb3, 0x01, 0x66, 0x5a, 4096, 128, 512, 4},
	{0xec, 0xbc, 0x00, 0x6a, 0x56, 4096, 256, 512, 8},
	{0xad, 0xbc, 0x90, 0x55, 0x56, 2048, 64,  512, 4},
        {0xad, 0xbc, 0x90, 0x55, 0x54, 2048, 64,  512, 4},
        {0x2c, 0xbc, 0x90, 0x55, 0x56, 2048, 64,  512, 4},
        {0x2c, 0xb3, 0xd1, 0x55, 0x56, 2048, 64,  512, 4},
        {0xc8, 0xbc, 0x90, 0x55, 0x54, 2048, 64,  512, 4},
        {0xec, 0xbc, 0x01, 0x55, 0x48, 2048, 64,  512, 4},
        {0x98, 0xbc, 0x90, 0x66, 0x76, 4096, 224, 512, 8}
};

/* some nand id could not be calculated the pagesize by mtd, replace it with a known id which has the same format. */
static const unsigned char nand_id_replace_table[][10] =
{
    {0xad, 0xb3, 0x91, 0x11, 0x00, /*replace with*/ 0xec, 0xbc, 0x00, 0x66, 0x56},
    {0xad, 0xbc, 0x90, 0x11, 0x00, /*replace with*/ 0xec, 0xbc, 0x00, 0x66, 0x56}
};

static const struct nand_spec_str nand_spec_table[] = {
    {0x98, 0xba, 0x90, 0x55, 0x76, {15, 15, 10, 20, 50, 12, 12, 25}}, //add for 6821
    {0x2c, 0xb3, 0xd1, 0x55, 0x5a, {10, 12, 10, 20, 50, 10, 10, 25}},// MT29C8G96MAAFBACKD-5, MT29C4G96MAAHBACKD-5
    {0x2c, 0xb3, 0x90, 0x66, 0x64, {10, 15, 10, 20, 50, 10, 10, 25}},
    {0x2c, 0xba, 0x80, 0x55, 0x50, {10, 12, 10, 20, 50, 10, 10, 25}},// MT29C2G48MAKLCJA-5 IT
    {0x2c, 0xbc, 0x90, 0x55, 0x56, {25, 30, 10, 20, 50, 10, 10, 25}},// KTR0405AS-HHg1, KTR0403AS-HHg1, MT29C4G96MAZAPDJA-5 IT,mvr606,mvr593
    {0x2c, 0xbc, 0x90, 0x66, 0x54, {20, 25, 10, 20, 50, 10, 10, 25}}, //MT29F4G16ABBEA, mvr527

    {0x98, 0xac, 0x90, 0x15, 0x76, {10, 12, 10, 20, 50, 12, 12, 25}},// TYBC0A111392KC
    {0x98, 0xbc, 0x90, 0x55, 0x76, {15, 15, 10, 20, 50, 12, 12, 25}},// TYBC0A111430KC, KSLCBBL1FB4G3A, KSLCBBL1FB2G3A
    {0x98, 0xbc, 0x90, 0x66, 0x76, {35, 40, 10, 20, 50, 12, 12, 25}},// KSLCCBL1FB2G3A,mvr673,mvr594
    {0xad, 0xbc, 0x90, 0x11, 0x00, {15, 25, 10, 20, 50, 25, 25, 30}},// H9DA4VH4JJMMCR-4EMi, H9DA4VH2GJMMCR-4EM
    {0xad, 0xbc, 0x90, 0x55, 0x54, {15, 25, 10, 20, 50, 25, 25, 30}},//
    {0xad, 0xbc, 0x90, 0x55, 0x56, {25, 35, 10, 20, 50, 25, 25, 30}},//H9DA4GH2GJBMCR, mvr525, mvr543
    {0xec, 0xb3, 0x01, 0x66, 0x5a, {10, 21, 10, 20, 50, 21, 21, 30}},// KBY00U00VA-B450
    {0xec, 0xbc, 0x00, 0x55, 0x54, {10, 21, 10, 20, 50, 21, 21, 30}},// KA100O015M-AJTT
    {0xec, 0xbc, 0x00, 0x6a, 0x56, {10, 21, 10, 20, 50, 21, 21, 30}},// K524G2GACH-B050
    {0xec, 0xbc, 0x01, 0x55, 0x48, {15, 30, 10, 20, 30, 21, 21, 30}},// KBY00N00HM-A448,mvr645
    {0xec, 0xbc, 0x00, 0x66, 0x56, {15, 30, 10, 25, 30, 21, 21, 30}}, //K524G2GACJ-B050, mvr658

    {0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0}}
};

struct nand_ecclayout _nand_oob_64_4bit = {
	.eccbytes = 28,
	.eccpos = {
	               36, 37, 38, 39, 40, 41, 42, 43,
                   44, 45, 46, 47, 48, 49, 50, 51,
                   52, 53, 54, 55, 56, 57, 58, 59,
                   60, 61, 62, 63},
	.oobfree = {
		{.offset = 2,
		 .length = 34}}
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
	.eccbytes = 112,
	.eccpos = {
		112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124,
		125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137,
		138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150,
		151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163,
		164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176,
		177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
		190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202,
		203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215,
		216, 217, 218, 219, 220, 221, 222, 223},
	.oobfree = {
		{.offset = 2,
		.length = 110}}
};


static struct nand_ecclayout _nand_oob_256 = {
	.eccbytes = 112,
	.eccpos = {
		144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156,
		157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
		170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
		183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195,
		196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208,
		209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221,
		222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234,
		235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247,
		248, 249, 250, 251, 252, 253, 254, 255},
	.oobfree = {
		{.offset = 2,
		.length = 142}}
};

static void correct_invalid_id(unsigned char *buf);
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

static unsigned int nfc_mcr_inst_exc(int irq_en)
{
	unsigned int value;

	value = nfc_reg_read(NFC_CFG0);

	if (g_info.chip->options & NAND_BUSWIDTH_16)
		value |= NFC_BUS_WIDTH_16;
	else
		value &= ~NFC_BUS_WIDTH_16;

	value |= (1 << NFC_CMD_SET_OFFSET);
	nfc_reg_write(NFC_CFG0, value);
#ifdef  NAND_IRQ_EN
        nfc_reg_write(NFC_CLR_RAW, 0xffff0000);
        value = nfc_reg_read(NFC_STS_EN);
        if(irq_en){
            value |= (NFC_DONE_EN|NFC_TO_EN);
        }else{
            value &= ~(NFC_DONE_EN|NFC_TO_EN);
        }
        nfc_reg_write(NFC_STS_EN, value);
#endif
        value = NFC_CMD_VALID | ((unsigned int)NF_MC_NOP_ID) | ((g_info.mc_ins_num - 1) << 16);
	nfc_reg_write(NFC_CMD, value);

	return 0;
}

static unsigned int nfc_mcr_inst_exc_for_id(int irq_en)
{
	unsigned int value;

	value = nfc_reg_read(NFC_CFG0);
	value &= ~NFC_BUS_WIDTH_16;
	value |= (1 << NFC_CMD_SET_OFFSET);

	nfc_reg_write(NFC_CFG0, value);
	#ifdef  NAND_IRQ_EN
        nfc_reg_write(NFC_CLR_RAW, 0xffff0000);
        value = nfc_reg_read(NFC_STS_EN);
        if(irq_en){
            value |= (NFC_DONE_EN|NFC_TO_EN);
        }else{
            value &= ~(NFC_DONE_EN|NFC_TO_EN);
        }
        nfc_reg_write(NFC_STS_EN, value);
        #endif
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

void fixon_timeout_send_readid_command(void)
{
	u8 mc_ins_num = 0;
	u32 ins, mode;
	unsigned int offset, high_flag, value;

	/* nfc_mcr_inst_add(cmd, NF_MC_CMD_ID); */
	ins = NAND_CMD_READID;
	mode = NF_MC_CMD_ID;
	offset = mc_ins_num >> 1;
	high_flag = mc_ins_num & 0x1;
	if (high_flag) {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0x0000ffff;
		value |= ins << 24;
		value |= mode << 16;
	} else {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0xffff0000;
		value |= ins << 8;
		value |= mode;
	}
	nfc_reg_write(NFC_START_ADDR0 + (offset << 2), value);
	mc_ins_num ++;

	/* nfc_mcr_inst_add(0x00, NF_MC_ADDR_ID); */
	ins = 0x00;
	mode = NF_MC_ADDR_ID;
	offset = mc_ins_num >> 1;
	high_flag = mc_ins_num & 0x1;
	if (high_flag) {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0x0000ffff;
		value |= ins << 24;
		value |= mode << 16;
	} else {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0xffff0000;
		value |= ins << 8;
		value |= mode;
	}
	nfc_reg_write(NFC_START_ADDR0 + (offset << 2), value);
	mc_ins_num ++;

	/* nfc_mcr_inst_add(7, NF_MC_RWORD_ID); */
	ins = 7;
	mode = NF_MC_RWORD_ID;
	offset = mc_ins_num >> 1;
	high_flag = mc_ins_num & 0x1;
	if (high_flag) {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0x0000ffff;
		value |= ins << 24;
		value |= mode << 16;
	} else {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0xffff0000;
		value |= ins << 8;
		value |= mode;
	}
	nfc_reg_write(NFC_START_ADDR0 + (offset << 2), value);
	mc_ins_num ++;
	/* nfc_mcr_inst_exc_for_id(); */
	value = nfc_reg_read(NFC_CFG0);
	value &= ~NFC_BUS_WIDTH_16;
	value |= (1 << NFC_CMD_SET_OFFSET);

	nfc_reg_write(NFC_CFG0, value);
	value = NFC_CMD_VALID | ((unsigned int)NF_MC_NOP_ID) |((mc_ins_num - 1) << 16);
	nfc_reg_write(NFC_CMD, value);
}

unsigned long fixon_timeout_check_id(void)
{
	unsigned char id[5];

	memcpy(id, (void *)NFC_MBUF_ADDR, 5);

	printk("fix_timeout_id : 0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n", fix_timeout_id[0], fix_timeout_id[1], fix_timeout_id[2], fix_timeout_id[3], fix_timeout_id[4]);
	printk("newid : 0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n", id[0], id[1], id[2], id[3], id[4]);

	if ((fix_timeout_id[0] == id[0]) && (fix_timeout_id[1] == id[1]) && \
		(fix_timeout_id[2] == id[2]) && (fix_timeout_id[3] == id[3]) && (fix_timeout_id[4] == id[4]))
		return 1;
	return 0;
}

void fixon_timeout_send_reset_flash_command(void)
{
	u8 mc_ins_num = 0;
	u32 ins, mode;
	unsigned int offset, high_flag, value;

	/* nfc_mcr_inst_add(cmd, NF_MC_CMD_ID); */
	ins = NAND_CMD_RESET;
	mode = NF_MC_CMD_ID;
	offset = mc_ins_num >> 1;
	high_flag = mc_ins_num & 0x1;
	if (high_flag) {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0x0000ffff;
		value |= ins << 24;
		value |= mode << 16;
	} else {
		value = nfc_reg_read(NFC_START_ADDR0 + (offset << 2));
		value &= 0xffff0000;
		value |= ins << 8;
		value |= mode;
	}
	nfc_reg_write(NFC_START_ADDR0 + (offset << 2), value);
	mc_ins_num ++;

	/* nfc_mcr_inst_exc(); */
	value = nfc_reg_read(NFC_CFG0);
	value |= NFC_BUS_WIDTH_16;
	value |= (1 << NFC_CMD_SET_OFFSET);
	nfc_reg_write(NFC_CFG0, value);
	value = NFC_CMD_VALID | ((unsigned int)NF_MC_NOP_ID) | ((mc_ins_num - 1) << 16);
	nfc_reg_write(NFC_CMD, value);
}

void fixon_timeout_save_reg(void)
{
	unsigned long ii, total;

	for (ii = 0; ii < 8; ii ++)
		fix_timeout_reg[ii] = 0;

	total = g_info.mc_ins_num - 1;
	for (ii = 0; ii <= total; ii ++) {
		if ((ii % 2) != 0)
			continue;
		fix_timeout_reg[ii / 2] = nfc_reg_read(NFC_START_ADDR0 + ii * 2);
	}

	/*for (ii = 0; ii <= total; ii ++) {
		if ((ii % 2) != 0)
			continue;
		printk("reg[0x%08x] = 0x%08x --> timeoutreg[0x%08x] = 0x%08x\n", (NFC_START_ADDR0 + ii * 2), nfc_reg_read(NFC_START_ADDR0 + ii * 2), ii / 2, fix_timeout_reg[ii / 2]);
	}*/
}

void fixon_timeout_print_reg(void)
{
	unsigned long ii, total;
        unsigned long temp;   

	total = g_info.mc_ins_num - 1;
	for (ii = 0; ii <= total; ii ++) {
		if ((ii % 2) != 0)
			continue;
		temp = nfc_reg_read(NFC_START_ADDR0 + ii * 2);
		printk("reg[0x%08x] = 0x%08x --> timeoutreg[0x%08x] = 0x%08x\n", (NFC_START_ADDR0 + ii * 2), nfc_reg_read(NFC_START_ADDR0 + ii * 2), ii / 2, temp);
	}

	/*for (ii = 0; ii <= total; ii ++) {
		if ((ii % 2) != 0)
			continue;
		printk("reg[0x%08x] = 0x%08x --> timeoutreg[0x%08x] = 0x%08x\n", (NFC_START_ADDR0 + ii * 2), nfc_reg_read(NFC_START_ADDR0 + ii * 2), ii / 2, fix_timeout_reg[ii / 2]);
	}*/
}

void fixon_timeout_restore_reg(void)
{
	unsigned long ii, total;

	total = g_info.mc_ins_num - 1;
	for (ii = 0; ii <= total; ii ++) {
		if ((ii % 2) != 0)
			continue;
		nfc_reg_write((NFC_START_ADDR0 + ii * 2), fix_timeout_reg[ii / 2]);
	}

	/*for (ii = 0; ii <= total; ii ++) {
		if ((ii % 2) != 0)
			continue;
		printk("timeoutreg[0x%08x] = 0x%08x ---> reg[0x%08x] = 0x%08x\n", ii / 2, fix_timeout_reg[ii / 2], (NFC_START_ADDR0 + ii * 2), nfc_reg_read(NFC_START_ADDR0 + ii * 2));
	}*/
}

enum NAND_ERR_CORRECT_S  fixon_reset_function(void)
{
        unsigned int nfc_clr_raw, nfc_cmd;
        enum  NAND_ERR_CORRECT_S  ret;

        printk("%s enter\n", __func__);
	sc8810_nand_hw_init();
	if (ptr_nand_spec != NULL)
	    set_nfc_timing(&ptr_nand_spec->timing_cfg, 153);
        mdelay(2);
	printk("send reset flash command\n");
	fixon_timeout_send_reset_flash_command();
	mdelay(5);
	nfc_clr_raw = nfc_reg_read(NFC_CLR_RAW);
	nfc_cmd = nfc_reg_read(NFC_CMD);
	printk("3NFC_DONE_RAW bit[0x%08x],NFC_VALID bit[0x%08x]\n",nfc_clr_raw, nfc_cmd);
	nfc_reg_write(NFC_CLR_RAW, 0xffff0000); /* clear all interrupt status */
	if (((nfc_clr_raw & NFC_DONE_RAW) == 1) && ((nfc_cmd & 0x80000000) == 0)) {
		printk("3nfc/flash/hardware are all right, execute again\n");
		ret = NAND_ERR_NEED_RETRY;
	} else{
	        ret = NAND_FATAL_ERROR;
                printk("%s is wrong!!!\n", __func__);
	}

        return ret;
}

enum NAND_ERR_CORRECT_S fixon_timeout_function(unsigned int flag, unsigned int cmd_has_high, int retry_time)
{
	unsigned int nfc_cmd, nfc_clr_raw, value, nfc_clr_raw_2, nfc_cmd_2;
        enum NAND_ERR_CORRECT_S  ret= NAND_NO_ERROR;

        mdelay(5);
        //read command register
        nfc_cmd = nfc_reg_read(NFC_CMD);
        //clear the current command
        value = nfc_reg_read(NFC_CFG0);
        value |= 0x2;
        nfc_reg_write(NFC_CFG0, value);
        mdelay(5);
        //clear all interrupt status */
        nfc_reg_write(NFC_CLR_RAW, 0xffff0000);
        printk("send readid command\n");
        //check procedure after small reset
        fixon_timeout_send_readid_command();
        mdelay(5);
        nfc_clr_raw = nfc_reg_read(NFC_CLR_RAW);
        printk(" %s enter, flag=0x%x, cmd_has_high=0x%x, REG_NFC_CMD[0x%08x],  REG_NFC_CLR_RAW[0x%08x], retry_time=0x%x\n",\
                __func__,flag,  cmd_has_high, nfc_cmd, nfc_clr_raw, retry_time);
        printk("NFC_DONE_RAW bit[0x%08x]\n", nfc_clr_raw);
        //clear all interrupt status */
        nfc_reg_write(NFC_CLR_RAW, 0xffff0000);
        if ((nfc_clr_raw & NFC_DONE_RAW) && retry_time) { /* NFC_DONE_RAW is 1 */
                printk("NFC_DONE_RAW is 1\n");
                //check id
                value = fixon_timeout_check_id();
                if (value == 1) {
                        printk("Id is right, step 3\n");
                        printk("send reset flash command\n");
                        fixon_timeout_send_reset_flash_command();
                        mdelay(5);
                        nfc_clr_raw_2 = nfc_reg_read(NFC_CLR_RAW);
                        nfc_cmd_2 = nfc_reg_read(NFC_CMD);
                        printk("2NFC_DONE_RAW bit[0x%08x]  NFC_VALID bit[0x%08x]\n", nfc_clr_raw_2, nfc_cmd_2);
                        nfc_reg_write(NFC_CLR_RAW, 0xffff0000); /* clear all interrupt status */
                        if (((nfc_clr_raw_2 & NFC_DONE_RAW) == 1) && ((nfc_cmd_2 & 0x80000000) == 0)) {
                                printk("2nfc/flash/hardware are all right, execute again\n");
                                ret = NAND_ERR_NEED_RETRY;
                        } else {
                                printk("done status not come, reset nand controller\n");
                                ret = fixon_reset_function();
                        }
                } else{
                    printk("Id is wrong, reset nand controller\n");
                    ret = fixon_reset_function();
                }
        } else{
            printk("NFC_DONE_RAW is 0, nand controller reinit[0x%08x]\n", flag);
            ret = fixon_reset_function();
        }
        if((ret==NAND_ERR_NEED_RETRY) && !(nfc_cmd & 0x80000000) && cmd_has_high)
            ret = NAND_ERR_FIXED;
        return ret;
}

static enum NAND_ERR_CORRECT_S sc8810_nfc_wait_command_finish(unsigned int flag, int cmd, int retry_time)
{
	unsigned int event = 0;
	unsigned int value;
        unsigned int nfc_cmd;
        unsigned int counter=0;
	unsigned int is_timeout = 0;
        unsigned int cmd_has_high=0;

        enum NAND_ERR_CORRECT_S ret = NAND_NO_ERROR;

	while (((event & flag) != flag) && (counter< NFC_TIMEOUT_VAL)) {
        	value = nfc_reg_read(NFC_CLR_RAW);
                nfc_cmd = nfc_reg_read(NFC_CMD);

        	if (value & NFC_ECC_DONE_RAW)
			event |= NFC_ECC_EVENT;

		if (value & NFC_DONE_RAW)
			event |= NFC_DONE_EVENT;

                if (((nfc_cmd & 0x80000000) == 0x80000000) && !cmd_has_high)
                        cmd_has_high=1;

                counter++;
		if (flag == NFC_DONE_EVENT) {
			if ((cmd == NAND_CMD_RESET) && (counter >= NFC_RESET_TIMEOUT))
				is_timeout = 1;
			else if ((cmd == NAND_CMD_STATUS) && (counter >= NFC_STATUS_TIMEOUT))
                                    is_timeout = 1;
			else if ((cmd == NAND_CMD_READID) && (counter >= NFC_READID_TIMEOUT))
				is_timeout = 1;
			else if ((cmd == NAND_CMD_ERASE2) && (counter >= NFC_ERASE_TIMEOUT))
				is_timeout = 1;
			else if ((cmd == NAND_CMD_READSTART) && (counter >= NFC_READ_TIMEOUT))
				is_timeout = 1;
			else if ((cmd == NAND_CMD_PAGEPROG) && (counter >= NFC_WRITE_TIMEOUT))
				is_timeout = 1;
		} else if (flag == NFC_ECC_EVENT) {
			if ((cmd == NFC_CMD_ENCODE) && (counter >= NFC_ECCDECODE_TIMEOUT))
				is_timeout = 1;
			else if ((cmd == NFC_CMD_DECODE) && (counter >= NFC_ECCENCODE_TIMEOUT))
				is_timeout = 1;
		}

		if (is_timeout == 1) {
			printk("nfc cmd[0x%08x] timeout[0x%08x] cmd_has_high:%d and reset nand controller.\n", cmd, counter, cmd_has_high);
			break;
		}
	}
	/*if (((cmd == NFC_CMD_ENCODE)) || (cmd == NFC_CMD_DECODE)) {
		printk("2cmd = 0x%08x  counter = 0x%08x\n", cmd, counter);
	}*/

	if (is_timeout == 1) {
	        int i;
                ret = fixon_timeout_function(flag, cmd_has_high, retry_time);
		if (ret == NAND_FATAL_ERROR) {
			for (i=0; i<40; i+=4)
                        {
                            printk("\r\nnfc cmd reg addr:0x%x, value:0x%xx!\r\n", NFC_CMD+i, nfc_reg_read(NFC_CMD+i));
                        }
                        panic("nfc cmd timeout, check nfc, flash, hardware\n");
			while (1);
		}
        }

	nfc_reg_write(NFC_CLR_RAW, 0xffff0000); /* clear all interrupt status */

	if ( counter>= NFC_TIMEOUT_VAL) {
		panic("nfc cmd timeout!!!");
		while (1);
	}
        return ret;
}

#ifdef  NAND_IRQ_EN
static void sc8810_wait_op_done(int useirq)
{
        if (useirq) {
                //INIT_COMPLETION(sc8810_op_completion);
                if (!wait_for_completion_timeout(&sc8810_op_completion, msecs_to_jiffies(IRQ_TIMEOUT))) {
                    handle_status=NAND_HANDLE_ERR;
                    printk("%s, wait irq timeout\n", __func__);
                }
        }
}

static irqreturn_t sc8810_nfc_irq(int irq, void *dev_id)
{
	unsigned int value;
        value = nfc_reg_read(NFC_STS_EN);

        //printk("%s, STS_EN:0x%x\n", __func__, value);
        /*record handle status*/
        if(value & NFC_TO_STS){
            handle_status=NAND_HANDLE_TIMEOUT;
        }
        else if(value & NFC_DONE_STS){
            handle_status=NAND_HANDLE_DONE;
        }
        /*diable irq*/
        value = nfc_reg_read(NFC_STS_EN);
        value &= ~(NFC_DONE_EN|NFC_TO_EN);
        nfc_reg_write(NFC_STS_EN, value);
        /*clear irq status*/
        nfc_reg_write(NFC_CLR_RAW, 0xffff0000); /* clear all interrupt status */
        complete(&sc8810_op_completion);

	return IRQ_HANDLED;
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
	sc8810_nfc_wait_command_finish(NFC_ECC_EVENT, NFC_CMD_ENCODE, 1);
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
	sc8810_nfc_wait_command_finish(NFC_ECC_EVENT, NFC_CMD_DECODE, 1);

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

static struct nand_spec_str *get_nand_spec(u8 *nand_id)
{
	int i = 0;

	while (nand_spec_table[i].mid != 0) {
		if ((nand_id[0] == nand_spec_table[i].mid) && (nand_id[1] == nand_spec_table[i].did) && \
			(nand_id[2] == nand_spec_table[i].id3) && (nand_id[3] == nand_spec_table[i].id4) && \
			(nand_id[4] == nand_spec_table[i].id5)) {
			return (struct nand_spec_str *)(&(nand_spec_table[i]));
		}
		i++;
	}

	return (struct nand_spec_str *)NULL;
}

#define TIMEING_DIVIDER  1000
#define DELAY_NFC_TO_PAD 10
#define DELAY_PAD_TO_NFC 8
#define DELAY_RWL        (DELAY_NFC_TO_PAD + DELAY_PAD_TO_NFC)
#define DELAY_RWH        7

static void set_nfc_timing(struct sc8810_nand_timing_param *nand_timing, u32 nfc_clk_MHz)
{
	u32 value = 0;
	u32 cycles;
        u32 delta_als_rwl=0;
        u32 delta_cls_rwl=0;

        delta_als_rwl=(nand_timing->als_time > nand_timing->rwl_time?(nand_timing->als_time-nand_timing->rwl_time):\
                (nand_timing->rwl_time-nand_timing->als_time));
        delta_cls_rwl=(nand_timing->cls_time > nand_timing->rwl_time?(nand_timing->cls_time-nand_timing->rwl_time):\
                (nand_timing->rwl_time-nand_timing->cls_time));

        cycles = ((max(delta_als_rwl, delta_cls_rwl)\
                    + DELAY_PAD_TO_NFC)* nfc_clk_MHz + TIMEING_DIVIDER-1) / TIMEING_DIVIDER;
	value |= (min(cycles, 0x1F) << NFC_ACS_OFFSET);

	cycles = ((nand_timing->rwh_time + DELAY_RWH) * nfc_clk_MHz + TIMEING_DIVIDER-1) / TIMEING_DIVIDER;
	value |= (min(cycles, 0x1F) << NFC_RWH_OFFSET);

	cycles = ((max(nand_timing->rwl_time, nand_timing->rea_time) + DELAY_RWL) * nfc_clk_MHz + TIMEING_DIVIDER-1) / TIMEING_DIVIDER;
	value |= (min(cycles,  0x3F) << NFC_RWL_OFFSET);

	cycles = (nand_timing->acr_time * nfc_clk_MHz + TIMEING_DIVIDER-1) / TIMEING_DIVIDER;
	value |= (min(cycles,  0x1F) << NFC_ACR_OFFSET);

	cycles = (nand_timing->rr_time * nfc_clk_MHz + TIMEING_DIVIDER-1) / TIMEING_DIVIDER;
	value |= (min(cycles,  0x1F) << NFC_RR_OFFSET);

	cycles = (nand_timing->ceh_time * nfc_clk_MHz + TIMEING_DIVIDER-1) / TIMEING_DIVIDER;
	value |= (min(cycles,  0x3F) << NFC_CEH_OFFSET);

        nfc_reg_write(NFC_TIMING, value);
}

static void sc8810_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	memcpy(buf, (void *)(g_info.b_pointer + NFC_MBUF_ADDR),len);
	g_info.b_pointer += len;
}

static void sc8810_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = (struct nand_chip *)(mtd->priv);
	int eccsize = chip->ecc.size;
	memcpy((void *)(g_info.b_pointer + NFC_MBUF_ADDR), (unsigned char*)buf,len);
	if (g_info.b_pointer < eccsize)
		memcpy(io_wr_port, (unsigned char*)buf,len);
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
	unsigned char *port = (void *)NFC_MBUF_ADDR;

	ch = port[g_info.b_pointer ++];
	ch |= port[g_info.b_pointer ++] << 8;

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

#if 0
char nand_cmd_buff[40];
int nand_cmd_buff_index = 0;
char nand_cmd_comm[40][TASK_COMM_LEN+1];
static inline  nand_add_cmd(char cmd)
{
    nand_cmd_buff_index++;
    if(nand_cmd_buff_index == 40)
        nand_cmd_buff_index = 0;
    nand_cmd_buff[nand_cmd_buff_index] = cmd;
		memcpy(nand_cmd_comm[nand_cmd_buff_index],current->comm,TASK_COMM_LEN);
	nand_cmd_comm[nand_cmd_buff_index][TASK_COMM_LEN] = 0;
}
#endif

#define nand_add_cmd(x)

static void sc8810_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = (struct nand_chip *)(mtd->priv);
	u32 eccsize, size = 0;
        int time=2;
        enum NAND_ERR_CORRECT_S ret = NAND_NO_ERROR;


	if (ctrl & NAND_CLE) {
		switch (cmd) {
		case NAND_CMD_RESET:
			wake_lock(&nfc_wakelock);
                        nand_add_cmd(0x01);
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_exc(false);
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
			nand_add_cmd(0x11);
                        wake_unlock(&nfc_wakelock);
			break;
		case NAND_CMD_STATUS:
			wake_lock(&nfc_wakelock);
                        nand_add_cmd(0x02);
			nfc_mcr_inst_init();
			//nfc_reg_write(NFC_CMD, 0x80000070);
			//sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd);
			//memcpy(io_wr_port, (void *)NFC_ID_STS, 1);
			nfc_mcr_inst_add(0x70, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0x10, NF_MC_NOP_ID);//add nop clk for twrh timing param
			nfc_mcr_inst_add(3, NF_MC_RWORD_ID);
			fixon_timeout_save_reg();
                        nfc_mcr_inst_exc_for_id(false);
                        ret=sc8810_nfc_wait_command_finish(NFC_DONE_EVENT,cmd, time);
                        while((ret==NAND_ERR_NEED_RETRY)&&time--){
                                fixon_timeout_restore_reg();
				nfc_mcr_inst_exc_for_id(false);
				ret = sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
                                printk("\r\n %s, %d, cmd:%d retry status: %d\, retry_time: %d\n", __func__, __LINE__, cmd, ret, time);
                        }
                        if(ret==NAND_ERR_NEED_RETRY)
                            panic("%s, %d, retry error\n", __func__, __LINE__);
                        memcpy(io_wr_port, (void *)NFC_MBUF_ADDR, 1);
			//printf("read status3  0x%x, 0x%x\r\n", io_wr_port[0], nfc_reg_read(NFC_MBUF_ADDR));
			nand_add_cmd(0x22);
                        wake_unlock(&nfc_wakelock);
			break;
		case NAND_CMD_READID:
			wake_lock(&nfc_wakelock);
                        nand_add_cmd(0x03);
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0x00, NF_MC_ADDR_ID);
			nfc_mcr_inst_add(0x10, NF_MC_NOP_ID); /* add nop clk for twrh timing param */
			nfc_mcr_inst_add(7, NF_MC_RWORD_ID);
			nfc_mcr_inst_exc_for_id(false);
			sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
			correct_invalid_id(io_wr_port);
			if (fix_timeout_id[0] == 0) {
				fix_timeout_id[0] = io_wr_port[0];
				fix_timeout_id[1] = io_wr_port[1];
				fix_timeout_id[2] = io_wr_port[2];
				fix_timeout_id[3] = io_wr_port[3];
				fix_timeout_id[4] = io_wr_port[4];
			}
                        nand_add_cmd(0x33);
			wake_unlock(&nfc_wakelock);
			break;
		case NAND_CMD_ERASE1:
			wake_lock(&nfc_wakelock);
                        nand_add_cmd(0x04);
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nand_add_cmd(0x44);
                        break;
		case NAND_CMD_ERASE2:
                        nand_add_cmd(0x05);
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);
			fixon_timeout_save_reg();
			nfc_mcr_inst_exc(true);
                        #ifdef  NAND_IRQ_EN
                        sc8810_wait_op_done(true);
                        if(handle_status==NAND_HANDLE_DONE){
                            ret=NAND_NO_ERROR;
                        }else if(handle_status==NAND_HANDLE_TIMEOUT){
                            ret=NAND_ERR_NEED_RETRY;
                        }else if(handle_status==NAND_HANDLE_ERR){
                            ret=NAND_ERR_NEED_RETRY;
                        }
                        #else
			ret = sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
                        #endif
                        while((ret == NAND_ERR_NEED_RETRY)&& time--) {
                                fixon_timeout_restore_reg();
				nfc_mcr_inst_exc(false);
				ret=sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
                                printk("\r\n %s, %d, cmd:%d retry status: %d\, retry_time: %d\n", __func__, __LINE__, cmd, ret, time);
                        }

                        if(ret==NAND_ERR_NEED_RETRY)
                            panic("%s, %d, retry error\n", __func__, __LINE__);
                        nand_add_cmd(0x55);
			wake_unlock(&nfc_wakelock);
			break;
		case NAND_CMD_READ0:
			wake_lock(&nfc_wakelock);
			nand_add_cmd(0x06);
                        nfc_mcr_inst_init();
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nand_add_cmd(0x66);
                        break;
		case NAND_CMD_READSTART:
                        nand_add_cmd(0x07);
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0x10, NF_MC_NOP_ID); /* add nop clk */
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);
			if((!g_info.addr_array[0]) && (!g_info.addr_array[1]))
				size = mtd->writesize + mtd->oobsize;
			else
				size = mtd->oobsize;
			sc8810_nand_data_add(size, chip->options & NAND_BUSWIDTH_16, 1);
			fixon_timeout_save_reg();
			nfc_mcr_inst_exc(false);
			ret = sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
                        while((ret == NAND_ERR_NEED_RETRY)&& time--) {
				fixon_timeout_restore_reg();
				nfc_mcr_inst_exc(false);
				ret=sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
                                printk("\r\n %s, %d, cmd:%d retry status: %d\, retry_time: %d\n", __func__, __LINE__, cmd, ret, time);
                        }
                        if(ret==NAND_ERR_NEED_RETRY)
                            panic("%s, %d, retry error\n", __func__, __LINE__);
                        nand_add_cmd(0x77);
			wake_unlock(&nfc_wakelock);
			break;
		case NAND_CMD_SEQIN:
			wake_lock(&nfc_wakelock);
                        nand_add_cmd(0x08);
			nfc_mcr_inst_init();
			nfc_mcr_inst_add(NAND_CMD_SEQIN, NF_MC_CMD_ID);
			nand_add_cmd(0x88);
                        break;
		case NAND_CMD_PAGEPROG:
                        nand_add_cmd(0x09);
			eccsize = chip->ecc.size;
			memcpy((void *)NFC_MBUF_ADDR, io_wr_port, eccsize);
			nfc_mcr_inst_add(0x10, NF_MC_NOP_ID); /* add nop clk for twrh timing param */
			sc8810_nand_data_add(g_info.b_pointer, chip->options & NAND_BUSWIDTH_16, 0);
			nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
			nfc_mcr_inst_add(0, NF_MC_WAIT_ID);
			fixon_timeout_save_reg();
		        nfc_mcr_inst_exc(true);
                        #ifdef  NAND_IRQ_EN
                        sc8810_wait_op_done(true);
                        if(handle_status==NAND_HANDLE_DONE){
                            ret=NAND_NO_ERROR;
                        }else if(handle_status==NAND_HANDLE_TIMEOUT){
                            ret=NAND_ERR_NEED_RETRY;
                        }else if(handle_status==NAND_HANDLE_ERR){
                            ret=NAND_ERR_NEED_RETRY;
                        }
                        #else
			ret = sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
                        #endif
                        while((ret == NAND_ERR_NEED_RETRY)&& time--) {
				fixon_timeout_restore_reg();
				nfc_mcr_inst_exc(false);
				ret=sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, cmd, time);
                                printk("\r\n %s, %d, cmd:%d retry status: %d\, retry_time: %d\n", __func__, __LINE__, cmd, ret, time);
                        }

                        if(ret==NAND_ERR_NEED_RETRY)
                            panic("%s, %d, retry error\n", __func__, __LINE__);
                        nand_add_cmd(0x99);
			wake_unlock(&nfc_wakelock);
			break;
		default :
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

static void correct_invalid_id(unsigned char *buf)
{
	unsigned char id[5];
	int index;
	int num = sizeof(nand_id_replace_table) / sizeof(nand_id_replace_table[0]);

	memcpy(id, (void *)NFC_MBUF_ADDR, 5);

	for (index = 0; index < num; index++) {
		if (id[0] == nand_id_replace_table[index][0] && id[1] == nand_id_replace_table[index][1] && id[2] == nand_id_replace_table[index][2] && id[3] == nand_id_replace_table[index][3] && id[4] == nand_id_replace_table[index][4]) {
			break;
		}
	}

	if (index < num) {
		buf[0] = nand_id_replace_table[index][5];
		buf[1] = nand_id_replace_table[index][6];
		buf[2] = nand_id_replace_table[index][7];
		buf[3] = nand_id_replace_table[index][8];
		buf[4] = nand_id_replace_table[index][9];

		printk("nand id(%02x,%02x,%02x,%02x,%02x) is invalid, correct it by(%02x,%02x,%02x,%02x,%02x)\n", id[0], id[1], id[2], id[3], id[4], buf[0], buf[1], buf[2], buf[3], buf[4]);
	} else
		memcpy(buf, (void *)NFC_MBUF_ADDR, 5);
}

static void nand_hardware_config(struct mtd_info *mtd, struct nand_chip *this, u8 id[8])
{
	int index;
	int array;

	array = sizeof(nand_config_table) / sizeof(nand_config_table[0]);
	for (index = 0; index < array; index ++) {
		if ((nand_config_table[index].m_c == id[0])
		        && (nand_config_table[index].d_c == id[1])
		        && (nand_config_table[index].cyc_3 == id[2])
		        && (nand_config_table[index].cyc_4 == id[3])
		        && (nand_config_table[index].cyc_5 == id[4])
		    )
			break;
	}

	if (index < array) {
		this->ecc.size = nand_config_table[index].eccsize;
		g_info.ecc_mode = nand_config_table[index].eccbit;

		/* 4 bit ecc, per 512 bytes can creat 13 * 4 = 52 bit , 52 / 8 = 7 bytes
		   8 bit ecc, per 512 bytes can creat 13 * 8 = 104 bit , 104 / 8 = 14 bytes */
		switch (g_info.ecc_mode) {
			case 4:
				/* 4 bit ecc, per 512 bytes can creat 14 * 4 = 56 bit , 56 / 8 = 7 bytes */
				this->ecc.bytes = 7;
				if(nand_config_table[index].oobsize == 64)
				    this->ecc.layout = &_nand_oob_64_4bit;
				else
				 this->ecc.layout = &_nand_oob_128;
			break;
			case 8:
				/* 8 bit ecc, per 512 bytes can creat 13 * 8 = 104 bit , 104 / 8 = 14 bytes */
				this->ecc.bytes = 14;
				if (nand_config_table[index].oobsize == 224)
					this->ecc.layout = &_nand_oob_224;
				else if((nand_config_table[index].oobsize == 128) && (nand_config_table[index].eccbit == 8)){
                     			this->ecc.layout = &_nand_oob_128 ; //add for 6821
                		}
				else
					this->ecc.layout = &_nand_oob_256;
				mtd->oobsize = nand_config_table[index].oobsize;
			break;
		}
	}
}

void read_chip_id(void)
{
	int cmd = NAND_CMD_READID;

	nfc_mcr_inst_init();
	nfc_mcr_inst_add(cmd, NF_MC_CMD_ID);
	nfc_mcr_inst_add(0x00, NF_MC_ADDR_ID);
	nfc_mcr_inst_add(0x10, NF_MC_NOP_ID); /* add nop clk for twrh timing param */
	nfc_mcr_inst_add(7, NF_MC_RWORD_ID);
	nfc_mcr_inst_exc_for_id(false);
	sc8810_nfc_wait_command_finish(NFC_DONE_EVENT, NAND_CMD_READID, 1);
	correct_invalid_id(io_wr_port);
	if (fix_timeout_id[0] == 0) {
		fix_timeout_id[0] = io_wr_port[0];
		fix_timeout_id[1] = io_wr_port[1];
		fix_timeout_id[2] = io_wr_port[2];
		fix_timeout_id[3] = io_wr_port[3];
		fix_timeout_id[4] = io_wr_port[4];
	}
}

int board_nand_init(struct nand_chip *this)
{
	g_info.chip = this;
	g_info.ecc_mode = CONFIG_SYS_NAND_ECC_MODE;
	sc8810_nand_hw_init();
	read_chip_id();
	ptr_nand_spec = get_nand_spec(io_wr_port);
	this->IO_ADDR_R = (void __iomem*)NFC_MBUF_ADDR;
	this->IO_ADDR_W = this->IO_ADDR_R;
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
	this->nfc_hardware_config = nand_hardware_config;

	this->chip_delay = 20;
	this->priv = &g_info;
	this->options |= NAND_BUSWIDTH_16;

	/* The 4th id byte is the important one */

	if (ptr_nand_spec != NULL)
		set_nfc_timing(&ptr_nand_spec->timing_cfg, 153);

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
#ifdef  NAND_IRQ_EN
        int err = 0;
#endif
        regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev,"resources unusable\n");
		goto Err;
	}

	wake_lock_init(&nfc_wakelock, WAKE_LOCK_SUSPEND, "nfc_wakelock");
#ifdef  NAND_IRQ_EN
        init_completion(&sc8810_op_completion);
        err = request_irq(IRQ_NLC_INT, sc8810_nfc_irq, IRQF_DISABLED, DRIVER_NAME, NULL);
	if (err)
		goto Err;
#endif

	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
	memset(&g_info, 0 , sizeof(struct sprd_nand_info));
	memset(fix_timeout_id, 0x0, sizeof(fix_timeout_id));

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
	this->options |= NAND_USE_FLASH_BBT;
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
	have_no_nand = 1;
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);
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
	if (have_no_nand == 1)
		return 0;
	else {
		nfc_reg_cfg0 = nfc_reg_read(NFC_CFG0);
		nfc_reg_write(NFC_CFG0, (nfc_reg_cfg0 | NFC_CS1_SEL)); /* deset CS_SEL */
		sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, AHB_CTL0_NFC_EN, AHB_CTL0);

		return 0;
	}
}

static int sprd_nand_resume(struct platform_device *dev)
{
	if (have_no_nand == 1)
		return 0;
	else {
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
MODULE_AUTHOR("spreadtrum.com");
MODULE_DESCRIPTION("SPRD 8810 MTD NAND driver");
MODULE_ALIAS("platform:sprd-nand");
