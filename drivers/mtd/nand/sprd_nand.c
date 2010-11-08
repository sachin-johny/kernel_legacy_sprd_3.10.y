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
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <mach/regs_ahb.h>
#include <mach/regs_nfc.h>
#include <mach/regs_cpc.h>
#include <mach/regs_global.h>

#define NF_PARA_20M        	0x7ac05      //trwl = 0  trwh = 0
#define NF_PARA_40M        	0x7ac15      //trwl = 1  trwh = 0
#define NF_PARA_53M        	0x7ad26      //trwl = 2  trwh = 1
#define NF_PARA_80M        	0x7ad37      //trwl = 3  trwh = 1
#define NF_PARA_DEFAULT    	0x7ad77      //trwl = 7  trwh = 1
#define NF_TIMEOUT_VAL 		0x1000000

#define PAGE_SIZE_S         512
#define SPARE_SIZE_S        16
#define PAGE_SIZE_L         2048
#define SPARE_SIZE_L        64

#define REG_CPC_NFWPN				(*((volatile unsigned int *)(CPC_NFWPN_REG)))
#define REG_CPC_NFRB				(*((volatile unsigned int *)(CPC_NFRB_REG)))
#define REG_CPC_NFCLE                           (*((volatile unsigned int *)(CPC_NFCLE_REG)))
#define REG_CPC_NFALE				(*((volatile unsigned int *)(CPC_NFALE_REG)))
#define REG_CPC_NFCEN                           (*((volatile unsigned int *)(CPC_NFCEN_REG)))
#define REG_CPC_NFWEN                           (*((volatile unsigned int *)(CPC_NFWEN_REG)))
#define REG_CPC_NFREN                           (*((volatile unsigned int *)(CPC_NFREN_REG)))
#define REG_CPC_NFD0                            (*((volatile unsigned int *)(CPC_NFD0_REG)))
#define REG_CPC_NFD1                            (*((volatile unsigned int *)(CPC_NFD1_REG)))
#define REG_CPC_NFD2                            (*((volatile unsigned int *)(CPC_NFD2_REG)))
#define REG_CPC_NFD3                            (*((volatile unsigned int *)(CPC_NFD3_REG)))
#define REG_CPC_NFD4                            (*((volatile unsigned int *)(CPC_NFD4_REG)))
#define REG_CPC_NFD5                            (*((volatile unsigned int *)(CPC_NFD5_REG)))
#define REG_CPC_NFD6                            (*((volatile unsigned int *)(CPC_NFD6_REG)))
#define REG_CPC_NFD7                            (*((volatile unsigned int *)(CPC_NFD7_REG)))
#define REG_CPC_NFD8                            (*((volatile unsigned int *)(CPC_NFD8_REG)))

#define REG_CPC_NFD9                            (*((volatile unsigned int *)(CPC_NFD9_REG)))
#define REG_CPC_NFD10                           (*((volatile unsigned int *)(CPC_NFD10_REG)))
#define REG_CPC_NFD11                           (*((volatile unsigned int *)(CPC_NFD11_REG)))
#define REG_CPC_NFD12                           (*((volatile unsigned int *)(CPC_NFD12_REG)))
#define REG_CPC_NFD13                           (*((volatile unsigned int *)(CPC_NFD13_REG)))
#define REG_CPC_NFD14                           (*((volatile unsigned int *)(CPC_NFD14_REG)))
#define REG_CPC_NFD15                           (*((volatile unsigned int *)(CPC_NFD15_REG)))


#define REG_AHB_CTL0		       		(*((volatile unsigned int *)(AHB_CTL0)))
#define REG_AHB_SOFT_RST		       	(*((volatile unsigned int *)(AHB_SOFT_RST)))

#define REG_GR_NFC_MEM_DLY                      (*((volatile unsigned int *)(GR_NFC_MEM_DLY)))

/*
#define set_gpio_as_nand()	                        \
do {                                                    \
        REG_CPC_NFWPN = BIT_0 | BIT_4 | BIT_5;          \
        REG_CPC_NFWPN &= ~(BIT_6 | BIT_7);              \
        REG_CPC_NFRB = BIT_0 | BIT_3 | BIT_4 | BIT_5;   \
        REG_CPC_NFRB &= ~(BIT_6 | BIT_7);               \
	REG_CPC_NFCLE |= BIT_4 | BIT_5;			\
	REG_CPC_NFCLE &= ~(BIT_6 | BIT_7);		\
	REG_CPC_NFALE |= BIT_4 | BIT_5;                 \
        REG_CPC_NFALE &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFCEN |= BIT_4 | BIT_5;                 \
        REG_CPC_NFCEN &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFWEN |= BIT_4 | BIT_5;                 \
        REG_CPC_NFWEN &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFREN |= BIT_4 | BIT_5;                 \
        REG_CPC_NFREN &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD0 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD0 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD1 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD1 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD2 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD2 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD3 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD3 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD4 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD4 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD5 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD5 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD6 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD6 &= ~(BIT_6 | BIT_7);              \
        REG_CPC_NFD7 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD7 &= ~(BIT_6 | BIT_7);              \
       	REG_CPC_NFD8 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD8 &= ~(BIT_6 | BIT_7);              \
       	REG_CPC_NFD9 |= BIT_4 | BIT_5 | BIT_6;         \
        REG_CPC_NFD9 &= ~(BIT_7);                      \
       	REG_CPC_NFD10 |= BIT_4 | BIT_5 | BIT_6;        \
        REG_CPC_NFD10 &= ~(BIT_7);                     \
       	REG_CPC_NFD11 |= BIT_4 | BIT_5 | BIT_6;        \
        REG_CPC_NFD11 &= ~(BIT_7);                     \
       	REG_CPC_NFD12 |= BIT_4 | BIT_5 | BIT_6;        \
        REG_CPC_NFD12 &= ~(BIT_7);                     \
       	REG_CPC_NFD13 |= BIT_4 | BIT_5 | BIT_6;        \
        REG_CPC_NFD13 &= ~(BIT_7);                     \
       	REG_CPC_NFD14 |= BIT_4 | BIT_5 | BIT_6;        \
        REG_CPC_NFD14 &= ~(BIT_7);                     \
       	REG_CPC_NFD15 |= BIT_4 | BIT_5 | BIT_6;        \
        REG_CPC_NFD15 &= ~(BIT_7);                     \
} while (0)


#define set_gpio_as_nand()                              \
do {                                                    \
        REG_CPC_NFWPN = BIT_0 | BIT_4 | BIT_5;          \
        REG_CPC_NFWPN &= ~(BIT_6 | BIT_7);              \
        REG_CPC_NFRB = BIT_0 | BIT_3 | BIT_4 | BIT_5;   \
        REG_CPC_NFRB &= ~(BIT_6 | BIT_7);               \
	REG_CPC_NFCLE |= BIT_4 | BIT_5;			\
	REG_CPC_NFCLE &= ~(BIT_6 | BIT_7);		\
	REG_CPC_NFALE |= BIT_4 | BIT_5;                 \
        REG_CPC_NFALE &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFCEN |= BIT_4 | BIT_5;                 \
        REG_CPC_NFCEN &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFWEN |= BIT_4 | BIT_5;                 \
        REG_CPC_NFWEN &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFREN |= BIT_4 | BIT_5;                 \
        REG_CPC_NFREN &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD0 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD0 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD1 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD1 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD2 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD2 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD3 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD3 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD4 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD4 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD5 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD5 &= ~(BIT_6 | BIT_7);              \
	REG_CPC_NFD6 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD6 &= ~(BIT_6 | BIT_7);              \
        REG_CPC_NFD7 |= BIT_4 | BIT_5;                 \
        REG_CPC_NFD7 &= ~(BIT_6 | BIT_7);              \
} while (0)
*/

#define set_sc8800g_gpio_as_nand_8bit()                 \
do {                                                    \
        REG_CPC_NFWPN |= BIT_8 | BIT_9;          	\
        REG_CPC_NFWPN &= ~(BIT_4 | BIT_5);              \
        REG_CPC_NFRB |= BIT_8 | BIT_9;   		\
        REG_CPC_NFRB &= ~(BIT_4 | BIT_5);               \
	REG_CPC_NFCLE |= BIT_8 | BIT_9;			\
	REG_CPC_NFCLE &= ~(BIT_4 | BIT_5);		\
	REG_CPC_NFALE |= BIT_8 | BIT_9;                 \
        REG_CPC_NFALE &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFCEN |= BIT_8 | BIT_9;                 \
        REG_CPC_NFCEN &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFWEN |= BIT_8 | BIT_9;                 \
        REG_CPC_NFWEN &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFREN |= BIT_8 | BIT_9;                 \
        REG_CPC_NFREN &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFD0 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD0 &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFD1 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD1 &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFD2 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD2 &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFD3 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD3 &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFD4 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD4 &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFD5 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD5 &= ~(BIT_4 | BIT_5);              \
	REG_CPC_NFD6 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD6 &= ~(BIT_4 | BIT_5);              \
        REG_CPC_NFD7 |= BIT_8 | BIT_9;                 \
        REG_CPC_NFD7 &= ~(BIT_4 | BIT_5);              \
} while (0)

#define set_sc8800g_gpio_as_nand_16bit()	        \
do {                                                    \
       	REG_CPC_NFD8 |= BIT_8 | BIT_9;                 	\
        REG_CPC_NFD8 &= ~(BIT_4 | BIT_5);              	\
       	REG_CPC_NFD9 |= BIT_8 | BIT_9;         		\
        REG_CPC_NFD9 &= ~(BIT_4 | BIT_5);               \
       	REG_CPC_NFD10 |= BIT_8 | BIT_9;        		\
        REG_CPC_NFD10 &= ~(BIT_4 | BIT_5);              \
       	REG_CPC_NFD11 |= BIT_8 | BIT_9;        		\
        REG_CPC_NFD11 &= ~(BIT_4 | BIT_5);              \
       	REG_CPC_NFD12 |= BIT_8 | BIT_9;        		\
        REG_CPC_NFD12 &= ~(BIT_4 | BIT_5);              \
       	REG_CPC_NFD13 |= BIT_8 | BIT_9;        		\
        REG_CPC_NFD13 &= ~(BIT_4 | BIT_5);              \
       	REG_CPC_NFD14 |= BIT_8 | BIT_9;        		\
        REG_CPC_NFD14 &= ~(BIT_4 | BIT_5);              \
       	REG_CPC_NFD15 |= BIT_8 | BIT_9;        		\
        REG_CPC_NFD15 &= ~(BIT_4 | BIT_5);              \
} while (0)

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
	struct sprd_platform_nand	*platform;
	struct clk	*clk;
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

static struct mtd_info *sprd_mtd = NULL;
static unsigned long g_cmdsetting = 0;
static sprd_nand_wr_mode_t sprd_wr_mode = NO_OP;
static sprd_nand_area_mode_t sprd_area_mode = NO_AREA;
static unsigned long nand_flash_id = 0;
static struct sprd_nand_address sprd_colrow_addr = {0, 0, 0, 0};
static unsigned char io_wr_port[NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE];
static nand_ecc_modes_t sprd_ecc_mode = NAND_ECC_NONE;
#ifdef CONFIG_MACH_G2PHONE
static unsigned long buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
static unsigned long addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
#else
static unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
static unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
#endif
//static unsigned long writeaaa = 0;
//static unsigned long readaaa = 0;
//static unsigned long eccaaa = 0;

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

//static unsigned long g_CmdSetting;

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
		counter++;
	}
	
	if (NF_TIMEOUT_VAL == counter) {
		return 2;
	}
	
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
	//printk("%s  %d  teststatus = 0x%08x\n", __FUNCTION__, __LINE__, status);	
}

static unsigned long nfc_read_id(void)
{
        unsigned long id = 0;
        unsigned long cmd =  NF_READ_ID | (0x1 << 31);

        REG_NFC_CMD = cmd;
        nfc_wait_command_finish();

        id = REG_NFC_IDSTATUS;
        //printk("\n%s  %s  %d  id = 0x%08x\n", __FILE__, __FUNCTION__, __LINE__, id);
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
static void set_nfc_param(unsigned long ahb_clk)
{
	unsigned long flags;

	nfc_wait_command_finish();
	local_irq_save(flags);

	switch (ahb_clk) {
	case 20:
        	REG_NFC_PARA = NF_PARA_20M;
        break;

        case 40:
        	REG_NFC_PARA = NF_PARA_40M;
        break;

        case 53:
        	REG_NFC_PARA = NF_PARA_53M;
        break;

        case 80:
        	REG_NFC_PARA = NF_PARA_80M;
        break;

        default:
             	REG_NFC_PARA = NF_PARA_DEFAULT;    
    	}

	local_irq_restore(flags);	
}

static void nand_copy(unsigned char *src, unsigned char *dst, unsigned long len)
{
	unsigned long i;
	unsigned long *pDst_32, *pSrc_32;
	unsigned short *pDst_16, *pSrc_16;
	unsigned long flag = 0;
	
	flag = (unsigned long *)dst;
	//flag = (unsigned long)dst;
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
    	}//switch	
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

static void sprd_nand_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	unsigned long phyblk, pageinblk, pageperblk;
	unsigned long i;
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long chipsel = 0;

	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
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
				
				memset((unsigned char *)(this->IO_ADDR_R), 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
				nand_copy((unsigned char *)NFC_IDSTATUS, this->IO_ADDR_R, 4);
#if 0
				/* transfer to big endian */
				i = io_wr_port[3]; io_wr_port[3] = io_wr_port[0]; io_wr_port[0] = i;
				i = io_wr_port[2]; io_wr_port[2] = io_wr_port[1]; io_wr_port[1] = i;
#endif
        			/*for (i = 0; i < 4; i++)
                			printk("io_wr_port[%d] = 0x%02x\n", i, io_wr_port[i]);*/
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
					//printk("%d  row = %d\n", __LINE__, sprd_colrow_addr.row);
        				
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
				//memset((unsigned char *)(this->IO_ADDR_R), 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
				memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
			break;
			case NAND_CMD_READSTART:
				if (buswidth == 1) {
					if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
        					g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
								(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
        					REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        					nfc_wait_command_finish();
						//nand_copy((unsigned char *)NFC_SBUF, this->IO_ADDR_R, mtd->oobsize);
						//nand_copy((unsigned char *)NFC_SBUF, io_wr_port, mtd->oobsize);
						nand_copy((unsigned long *)NFC_SBUF, (unsigned long *)io_wr_port, mtd->oobsize);
#if 0
						if ((io_wr_port[0] != 0xff) || (io_wr_port[1] != 0xff)) {
							printk("\nrow = 0x%08x  column = 0x%08x\n", sprd_colrow_addr.row, sprd_colrow_addr.column);
							printk("Rport\n");
        						//for (i = 0; i < mtd->oobsize; i++)
							for (i = 0; i < 8; i++)
								printk("0x%02x,", io_wr_port[i]);
							printk("\n\n");
						}
#endif
					} else if (sprd_colrow_addr.column == 0) {
							//printk("%s  %s  %d  mode=%d\n", __FILE__, __FUNCTION__, __LINE__, sprd_area_mode);
							if (sprd_area_mode == DATA_AREA)
								sprd_area_mode = DATA_OOB_AREA;

							if (sprd_area_mode == DATA_OOB_AREA) {
                                                		/* read data and spare area */
								//printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
                                                		REG_NFC_END0 = 0xffffffff;
                                                		g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
										(1 << 21) | (buswidth << 19) | (pagetype << 18) | \
										(0 << 16) | (0x1 << 31);
                                                		REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
                                                		nfc_wait_command_finish();
								nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
								/*if (readaaa < 6) {							
									printk("\nREADPAGE\n");
									for (i = 0; i < mtd->writesize; i++)
										printk(",0x%02x", io_wr_port[i]);
									printk("\n\n");
									readaaa ++;
								}*/

                                        		} else if (sprd_colrow_addr.column == DATA_AREA) {
								//printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
        							g_cmdsetting = (addr_cycle << 24) | (advance << 23) | \
										(buswidth << 19) | (pagetype << 18) | \
										(0 << 16) | (0x1 << 31);
        							REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        							nfc_wait_command_finish();
				
								nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);

        							/*for (i = 0; i < mtd->writesize; i++)
                							printk(" Rport[%d]=%d ", i, io_wr_port[i]);*/
						}
					} else
						printk("Operation !!! area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
				} else {  //if (buswidth == 1)
					if (sprd_colrow_addr.column == mtd->writesize) {
        					g_cmdsetting = (addr_cycle << 24) | (advance << 23) | (buswidth << 19) | \
								(pagetype << 18) | (0 << 16) | (0x1 << 31);
        					REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        					nfc_wait_command_finish();
						nand_copy((unsigned char *)NFC_SBUF, io_wr_port, mtd->oobsize);

        					/*for (i = 0; i < mtd->oobsize; i++)
                					printk(" Rport[%d]=%d ", i, io_wr_port[i]);*/
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
							
							nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
							/*if (readaaa < 6) {							
									printk("\nREADPAGE\n");
									for (i = 0; i < mtd->writesize; i++)
										printk(",0x%02x", io_wr_port[i]);
									printk("\n\n");
									readaaa ++;
							}*/

                                        	} else if (sprd_colrow_addr.column == DATA_AREA) {
        						g_cmdsetting = (addr_cycle << 24) | (advance << 23) | \
									(buswidth << 19) | (pagetype << 18) | \
									(0 << 16) | (0x1 << 31);
        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        						nfc_wait_command_finish();
				
							nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
        						/*for (i = 0; i < mtd->writesize; i++)
                						printk(" Rport[%d]=%d ", i, io_wr_port[i]);*/
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
				//memset((unsigned char *)(this->IO_ADDR_W), 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
				memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
			break;
			case NAND_CMD_PAGEPROG:
				if (buswidth == 1) {
					if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
						g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
								(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
        					/*printk("\nWport\n");
        					for (i = 0; i < mtd->oobsize; i++)
                					//printk(" Wport[%d]=0x%02x ", i, io_wr_port[i]);
							printk("0x%02x,", io_wr_port[i]);
						printk("\n\n");*/
						//nand_copy(this->IO_ADDR_W, (unsigned char *)NFC_SBUF, mtd->oobsize);
						nand_copy((unsigned long *)io_wr_port, (unsigned long *)NFC_SBUF, mtd->oobsize);
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
							nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, mtd->writesize);
        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        						nfc_wait_command_finish();
						}
					} else
						printk("Operation !!! area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);	
				} else {	//if (buswidth == 1)
					if (sprd_colrow_addr.column == mtd->writesize) {
        					/*for (i = 0; i < mtd->oobsize; i++)
                					printk(" Wport[%d]=%d ", i, io_wr_port[i]);*/

        					g_cmdsetting = (addr_cycle << 24) | (advance << 23) | (buswidth << 19) | \
								(pagetype << 18) | (0 << 16) | (0x1 << 31);
						nand_copy(io_wr_port, (unsigned char *)NFC_SBUF, mtd->oobsize);
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
        			
							//nand_copy(this->IO_ADDR_W, (unsigned char *)NFC_MBUF, mtd->writesize);
							nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, mtd->writesize);
        						REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        						nfc_wait_command_finish();
						}
					} else
						printk("Operation !!! area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
				}

				sprd_wr_mode = NO_OP;
				sprd_area_mode = NO_AREA;
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
					//printk("pageperblk=%d  mtd->erasesize=%d  mtd->writesize=%d  mtd->oobsize=%d\n", pageperblk, mtd->erasesize, mtd->writesize, mtd->oobsize);
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

					//printk("Line : %d  block = %d  page = %d  column = %d\n", __LINE__, phyblk, pageinblk, sprd_colrow_addr.column);
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
	//printk("%s  %s  %d  status = 0x%08x\n", __FILE__, __FUNCTION__, __LINE__, status);	
   	if ((status & 0x1) != 0) 	
     		return -1; /* fail */
   	else if ((status & 0x20) == 0)
     		return 0; /* busy */
   	else 							
     		return 1; /* ready */
}

static void sprd_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *this = mtd->priv;
	struct sprd_nand_info *info = this->priv;

#if 0	
	if (chip != -1)
		clk_enable(info->clk);
	else
		clk_disable(info->clk);
#endif
}

/* ecc function */
static void sprd_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	sprd_ecc_mode = mode;
}

static unsigned long sprd_nand_wr_oob(struct mtd_info *mtd)
{
	unsigned int i;
        /* copy io_wr_port into SBUF */
	/*for (i = 0; i < mtd->oobsize; i++)
                	printk(" OOBWport[%d]=%d ", i, io_wr_port[i]);*/
        nand_copy(io_wr_port, (unsigned char *)NFC_SBUF, mtd->oobsize);

	/* write oob area */
	if (sprd_area_mode == NO_AREA)
		sprd_area_mode = OOB_AREA;
	else if (sprd_area_mode == DATA_AREA)
		sprd_area_mode = DATA_OOB_AREA;

	return 0;
}

void nand_ecc_trans(unsigned char *pEccIn, unsigned char *pEccOut, unsigned char nSct)
{
#if 1
	/* little endian */
        switch(nSct)
        {
           case 1:
                 pEccOut[0] = pEccIn[0];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[2];
                 break;
           case 2:
                 pEccOut[0] = pEccIn[0];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[2];
                 pEccOut[4] = pEccIn[4];
                 pEccOut[5] = pEccIn[5];
                 pEccOut[6] = pEccIn[6];
                 break;
           case 3:
                 pEccOut[0] = pEccIn[0];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[2];
                 pEccOut[4] = pEccIn[4];
                 pEccOut[5] = pEccIn[5];
                 pEccOut[6] = pEccIn[6];
                 pEccOut[8] = pEccIn[8];
                 pEccOut[9] = pEccIn[9];
                 pEccOut[10] = pEccIn[10];
                 break;
           case 4:
                 pEccOut[0] = pEccIn[0];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[2];
                 pEccOut[4] = pEccIn[4];
                 pEccOut[5] = pEccIn[5];
                 pEccOut[6] = pEccIn[6];
                 pEccOut[8] = pEccIn[8];
                 pEccOut[9] = pEccIn[9];
                 pEccOut[10] = pEccIn[10];
                 pEccOut[12] = pEccIn[12];
                 pEccOut[13] = pEccIn[13];
                 pEccOut[14] = pEccIn[14];
                 break;
           default:
                 break;     
        }
#else
	/* big endian */
        switch(nSct)
        {
           case 1:
                 pEccOut[0] = pEccIn[2];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[3];
                 break;
           case 2:
                 pEccOut[0] = pEccIn[2];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[3];
                 pEccOut[4] = pEccIn[6];
                 pEccOut[5] = pEccIn[5];
                 pEccOut[6] = pEccIn[7];
                 break;
           case 3:
                 pEccOut[0] = pEccIn[2];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[3];
                 pEccOut[4] = pEccIn[6];
                 pEccOut[5] = pEccIn[5];
                 pEccOut[6] = pEccIn[7];
                 pEccOut[8] = pEccIn[10];
                 pEccOut[9] = pEccIn[9];
                 pEccOut[10] = pEccIn[11];
                 break;
           case 4:
                 pEccOut[0] = pEccIn[2];
                 pEccOut[1] = pEccIn[1];
                 pEccOut[2] = pEccIn[3];
                 pEccOut[4] = pEccIn[6];
                 pEccOut[5] = pEccIn[5];
                 pEccOut[6] = pEccIn[7];
                 pEccOut[8] = pEccIn[10];
                 pEccOut[9] = pEccIn[9];
                 pEccOut[10] = pEccIn[11];
                 pEccOut[12] = pEccIn[14];
                 pEccOut[13] = pEccIn[13];
                 pEccOut[14] = pEccIn[15];
                 break;
           default:
                 break;     
        }
#endif
}

static int sprd_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	unsigned long *pecc_val;
	unsigned int i, j;
	pecc_val=(unsigned long *)ecc_code;

#if 0
	unsigned long testcnt = 0;
       	unsigned char mydata[512];
#endif

	if (sprd_ecc_mode == NAND_ECC_WRITE) {
		REG_NFC_ECCEN = 0x1;

#if 0
	//////////////////////////////////////////////
	for (testcnt = 0; testcnt < 256; testcnt ++) {
                mydata[testcnt + 0] = testcnt;
                mydata[testcnt + 256] = testcnt;
        }
	switch (writeaaa) {
	case 0:
		mydata[21] = 0x35;
		break;
	case 1:
		mydata[21] = 0x35;
		mydata[22] = 0x17;
		break;
	}
	for (testcnt = 0; testcnt < 512; testcnt ++)
		io_wr_port[testcnt] = mydata[testcnt];

		if (writeaaa < 2) {
			printk("\nWRITEPAGE  :  %d\n", writeaaa);
			printk("\n");			
			for (i = 0; i < mtd->writesize; i++) {
	               		printk(",0x%02x", io_wr_port[i]);
				if (i % 16 == 15)
					printk("\n");
			}
			/*printk("\n");
			for (i = 0; i < 512; i ++) {
				printk(",0x%02x", io_wr_port[i]);
				if (i % 16 == 15)
					printk("\n");
			}*/
			printk("\n\n");		
		}
	//////////////////////////////////////////////
#endif

		/* copy io_wr_port into MBUF */
		nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, mtd->writesize);
		/* large page */
		pecc_val[0] = REG_NFC_PAGEECC0;
		pecc_val[1] = REG_NFC_PAGEECC1;
		pecc_val[2] = REG_NFC_PAGEECC2;
		pecc_val[3] = REG_NFC_PAGEECC3;
#if 0		
		if (writeaaa < 2) {
			printk("\nWRITEECC0 = 0x%08x  ", REG_NFC_PAGEECC0);		
			printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);		
			printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);		
			printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
			writeaaa ++;
		}
#endif
		REG_NFC_ECCEN = 0;
		memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);	
	} else if (sprd_ecc_mode == NAND_ECC_READ) {
                /* large page */
		pecc_val[0] = REG_NFC_PAGEECC0;
		pecc_val[1] = REG_NFC_PAGEECC1;
		pecc_val[2] = REG_NFC_PAGEECC2;
		pecc_val[3] = REG_NFC_PAGEECC3;
		/*if (readaaa < 6) {
        		printk("\nREADECC0 = 0x%08x  ", REG_NFC_PAGEECC0);
                	printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);
                	printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
                	printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
		}*/
		memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
		nand_copy((unsigned char *)NFC_SBUF, io_wr_port, mtd->oobsize);
		/*if (readaaa < 6) {
			for (i = 0; i < mtd->oobsize; i++)
                		printk(" OOBRport[%d]=0x%02x ", i, io_wr_port[i]);
		}*/
	}	

	/*printk("ecc code :\n");
	for (j = 0; j < 16; j++)
		printk(" 0x%02x ", ecc_code[j]);*/
	sprd_ecc_mode = NAND_ECC_NONE;

	return 0;
}

static int countbits(unsigned long byte)
{
        int res = 0;

        for (;byte; byte >>= 1)
                res += byte & 0x01;
        return res;
}

static void ecc_trans(unsigned char *ecc)
{
	unsigned char trans;
	
	trans = ecc[0];
	ecc[0] = ecc[1];
	ecc[1] = ecc[2];
	ecc[2] = trans;
}

static int ECC_CompM(unsigned char *pEcc1, unsigned char *pEcc2, unsigned char *pBuf, unsigned char nBW)
{
	unsigned long  nEccComp = 0, nEccSum = 0;
	unsigned long  nEBit    = 0;
	unsigned long  nEByte   = 0;
	unsigned long  nXorT1   = 0, nXorT2 = 0;
	unsigned long  nCnt;
	unsigned i,ij;

#if 0
	if (eccaaa < 10) {
		printk("pBuf = \n");
		for (i = 0; i < 512; i ++) {
			printk("%02x ", *(pBuf + i));
			if (i % 16 == 15)
				printk("\n");
		}

		printk("pEcc1 = \n");
		for (i = 0; i < 4; i ++) {
			printk("%02x ", *(pEcc1 + i));
		}
		printk("\n");
		printk("pEcc2 = \n");
		for (i = 0; i < 4; i ++) {
			printk("%02x ", *(pEcc2 + i));
		}
		printk("\n");
		eccaaa ++;
	}
#endif

	for (nCnt = 0; nCnt < 2; nCnt++) {
        	nXorT1 ^= (((*pEcc1) >> nCnt) & 0x01);
        	nXorT2 ^= (((*pEcc2) >> nCnt) & 0x01);
    	}

    	for (nCnt = 0; nCnt < 3; nCnt++) {
        	nEccComp |= ((~pEcc1[nCnt] ^ ~pEcc2[nCnt]) << (nCnt * 8));
    	}
    	//printf("nEccComp = 0x%x\n", nEccComp);
    	for(nCnt = 0; nCnt < 24; nCnt++) {
        	nEccSum += ((nEccComp >> nCnt) & 0x01);
    	}

    	//printf("nEccSum = %d\n", nEccSum);
    	switch (nEccSum) {
	case 0 :
			//printk("No Error for Main\n");
			return 0;
	case 1 :
			//printk("ECC Error \n");
#if 0
			printk("\n%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
			printk("pBuf = \n");
			for (ij = 0; ij < 512; ij ++) {
				printk("%02x ", *(pBuf + ij));
				if (ij % 16 == 15)
					printk("\n");
			}

			printk("pEcc1 = \n");
			for (ij = 0; ij < 4; ij ++) {
				printk("%02x ", *(pEcc1 + ij));
			}
			printk("\n");
			printk("pEcc2 = \n");
			for (ij = 0; ij < 4; ij ++) {
				printk("%02x ", *(pEcc2 + ij));
			}
			printk("\n");
			printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
#endif
			return 1;
        case 12 :
			if (nXorT1 != nXorT2) {
				//printk("Correctable ECC Error Occurs for Main\n");
				ecc_trans(pEcc1);
				ecc_trans(pEcc2);
				/*if (nBW == 0) {
						nEByte  = ((nEccComp >>  9) & 0x100) +
								((nEccComp >>  8) & 0x80) + ((nEccComp >>  7) & 0x40) +
								((nEccComp >>  6) & 0x20) + ((nEccComp >>  5) & 0x10) +
								((nEccComp >>  4) & 0x08) + ((nEccComp >>  3) & 0x04) +
								((nEccComp >>  2) & 0x02) + ((nEccComp >>  1) & 0x01);
						nEBit   = ((nEccComp >> 21) & 0x04) +
								((nEccComp >> 20) & 0x02) + ((nEccComp >> 19) & 0x01);
				} else */{   /* (nBW == BW_X16) */
						nEByte  = ((nEccComp >>  7) & 0x100) +
								((nEccComp >>  6) & 0x80) + ((nEccComp >>  5) & 0x40) +
								((nEccComp >>  4) & 0x20) + ((nEccComp >>  3) & 0x10) +
								((nEccComp >>  2) & 0x08) + ((nEccComp >>  1) & 0x04) +
								(nEccComp & 0x02)         + ((nEccComp >> 23) & 0x01);
						nEBit   = (unsigned char)(((nEccComp >> 19) & 0x04) +
								((nEccComp >> 18) & 0x02) + ((nEccComp >> 17) & 0x01));
				}
				//printk("1ECC Position : %dth byte, %dth bit\n", nEByte, nEBit);

				if (pBuf != NULL) {
						//printf("Corrupted : 0x%02x \n", pBuf[nEByte]);
						pBuf[nEByte] = (unsigned char)(pBuf[nEByte] ^ (1 << nEBit));
						//printk("Corrected : 0x%02x \n", pBuf[nEByte]);
#if 0
						printk("\n%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
						printk("pBuf = \n");
						for (ij = 0; ij < 512; ij ++) {
							printk("%02x ", *(pBuf + ij));
							if (ij % 16 == 15)
								printk("\n");
						}

						printk("pEcc1 = \n");
						for (ij = 0; ij < 4; ij ++) {
							printk("%02x ", *(pEcc1 + ij));
						}
						printk("\n");
						printk("pEcc2 = \n");
						for (ij = 0; ij < 4; ij ++) {
							printk("%02x ", *(pEcc2 + ij));
						}
						printk("\n");
						printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
#endif
				}
				return 1;
			}
        default :
			//printk("Uncorrectable ECC Error Occurs for Main\n");
#if 0
			printk("\n%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
			printk("pBuf = \n");
			for (ij = 0; ij < 512; ij ++) {
				printk("%02x ", *(pBuf + ij));
				if (ij % 16 == 15)
					printk("\n");
			}

			printk("pEcc1 = \n");
			for (ij = 0; ij < 4; ij ++) {
				printk("%02x ", *(pEcc1 + ij));
			}
			printk("\n");
			printk("pEcc2 = \n");
			for (ij = 0; ij < 4; ij ++) {
				printk("%02x ", *(pEcc2 + ij));
			}
			printk("\n");
			printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
#endif
            break;
    	}
	return -1;
}

static int correct(u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	return ECC_CompM(read_ecc, calc_ecc, dat, buswidth);
}

static int sprd_nand_correct_data(struct mtd_info *mtd, uint8_t *dat,
				     uint8_t *read_ecc, uint8_t *calc_ecc)
{
	int i, retval = 0;
	int retval0, retval1, retval2, retval3;
#if 0
	printk("\nthe all data\n");
	for(i = 0; i < 2048; i++) {
		printk("%02x ", *(dat + i));
		if(i % 16 == 15)
		  printk("\n");
	}

	printk("\nthe read ecc\n");
	for(i = 0; i < 16; i ++) {
			printk("%02x ", *(read_ecc + i));
			if(i % 4 == 3)
				printk("\n");
	}

	printk("\nthe calc ecc\n");
	for(i = 0; i < 16; i ++) {
			printk("%02x ", *(calc_ecc + i));
			if(i % 4 == 3)
				printk("\n");
	}
#endif
	if (mtd->writesize > 512) {
#if 0
		for (i = 0; i < 4; i++) {
			if (correct(dat + 512 * i, read_ecc + 4 * i, calc_ecc + 4 * i) == -1) {				
				retval = -1;
			}
		}
#else
	retval0 = correct(dat + 512 * 0, read_ecc + 4 * 0, calc_ecc + 4 * 0);
	retval1 = correct(dat + 512 * 1, read_ecc + 4 * 1, calc_ecc + 4 * 1);
	retval2 = correct(dat + 512 * 2, read_ecc + 4 * 2, calc_ecc + 4 * 2);
	retval3 = correct(dat + 512 * 3, read_ecc + 4 * 3, calc_ecc + 4 * 3);
	if ((retval0 == -1) || (retval1 == -1) || (retval2 == -1) || (retval3 == -1))
		retval = -1;
	else
		retval = retval0 + retval1 + retval2 + retval3;
#endif
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

static int get_row_parity(int idx, int flag)
{
	int atmic, step, parity, ii, jj;

	atmic = idx / 8;
	step = 2 * atmic;
	
	parity = 0;
	if (flag == 2) {
		parity = E[1];
		//printf(" E[%d ", 1);
		for (ii = 2; ii <= atmic; ii++) {
			//printf(" %d ", ii);
			parity = parity ^ E[ii];
		}
		for (ii = step+1; ii <= 512-step+1; ii+=step)
			for (jj = 0; jj < atmic; jj++) {
				//printf(" %d ", ii+jj);
				parity = parity ^ E[ii+jj];
			}
	}
	
	if (flag == 1) {
		parity = E[1+atmic];
		//printf(" E[%d ", 1+atmic);
		for (ii = 2+atmic; ii <= 2*atmic; ii++) {
			//printf(" %d ", ii);
			parity = parity ^ E[ii];
		}
		for (ii = step+atmic+1; ii <= 512-atmic+1; ii+=step)
			for (jj = 0; jj < atmic; jj++) {
				//printf(" %d ", ii+jj);
				parity = parity ^ E[ii+jj];
			}
	}
	
	//printf("]\nparity = %d\n", parity);	
	return parity;
}

/* ecc for 512 bytes */
static void calculate(unsigned char *dat, unsigned char *ecc)
{
	int data[513];
	int D70;
	int cnt;
	int P1_1, P1_2, P2_1, P2_2, P4_1, P4_2;
	int P8_1, P8_2, P16_1, P16_2, P32_1, P32_2, P64_1, P64_2, P128_1, P128_2, P256_1, P256_2, P512_1, P512_2;
	int P1024_1, P1024_2, P2048_1, P2048_2;

	for (cnt = 1; cnt <= 512; cnt++)
		data[cnt] = dat[cnt-1];

	D70 = data[1];
	for (cnt = 2; cnt <= 512; cnt++) {
		D70 = D70 ^ data[cnt];
	}
	
	for (cnt = 0; cnt <= 512; cnt++)
		E[cnt] = 0;
	
	for (cnt = 1; cnt <= 512; cnt++) {
		E[cnt] = ((data[cnt] >> 0) & 0x1) ^ ((data[cnt] >> 1) & 0x1) ^ ((data[cnt] >> 2) & 0x1) \
			^ ((data[cnt] >> 3) & 0x1) ^ ((data[cnt] >> 4) & 0x1) ^ ((data[cnt] >> 5) & 0x1) \
			^ ((data[cnt] >> 6) & 0x1) ^ ((data[cnt] >> 7) & 0x1);
	}

	P1_1 = ((D70 >> 7) & 0x1) ^ ((D70 >> 5) & 0x1) ^ ((D70 >> 3) & 0x1) ^ ((D70 >> 1) & 0x1);	
	P1_2 = ((D70 >> 6) & 0x1) ^ ((D70 >> 4) & 0x1) ^ ((D70 >> 2) & 0x1) ^ ((D70 >> 0) & 0x1);
	
	P2_1 = ((D70 >> 7) & 0x1) ^ ((D70 >> 6) & 0x1) ^ ((D70 >> 3) & 0x1) ^ ((D70 >> 2) & 0x1);	
	P2_2 = ((D70 >> 5) & 0x1) ^ ((D70 >> 4) & 0x1) ^ ((D70 >> 1) & 0x1) ^ ((D70 >> 0) & 0x1);

	P4_1 = ((D70 >> 7) & 0x1) ^ ((D70 >> 6) & 0x1) ^ ((D70 >> 5) & 0x1) ^ ((D70 >> 4) & 0x1);
	P4_2 = ((D70 >> 3) & 0x1) ^ ((D70 >> 2) & 0x1) ^ ((D70 >> 1) & 0x1) ^ ((D70 >> 0) & 0x1);
	//printf("P1_1 = %d  P1_2 = 0x%d  P2_1 = %d  P2_2 = %d  P4_1 = %d  P4_2 = %d\n", P1_1, P1_2, P2_1, P2_2, P4_1, P4_2);
	
	P8_2 = get_row_parity(8, 2);
	//printf("P8_2 = %d  ", P8_2);

	P8_1 = get_row_parity(8, 1);
	//printf("P8_1 = %d\n", P8_1);

	P16_2 = get_row_parity(16, 2);
	//printf("P16_2 = %d  ", P16_2);

	P16_1 = get_row_parity(16, 1);
	//printf("P16_1 = %d\n", P16_1);
	
	P32_2 = get_row_parity(32, 2);
	//printf("P32_2 = %d  ", P32_2);
	P32_1 = get_row_parity(32, 1);
	//printf("P32_1 = %d\n", P32_1);
	
	P64_2 = get_row_parity(64, 2);
	//printf("P64_2 = %d  ", P64_2);
	P64_1 = get_row_parity(64, 1);
	//printf("P64_1 = %d\n", P64_1);
	
	P128_2 = get_row_parity(128, 2);
	//printf("P128_2 = %d  ", P128_2);
	P128_1 = get_row_parity(128, 1);
	//printf("P128_1 = %d\n", P128_1);
	
	P256_2 = get_row_parity(256, 2);
	//printf("P256_2 = %d  ", P256_2);
	P256_1 = get_row_parity(256, 1);
	//printf("P256_1 = %d\n", P256_1);

	P512_2 = get_row_parity(512, 2);
	//printf("P512_2 = %d  ", P512_2);
	P512_1 = get_row_parity(512, 1);
	//printf("P512_1 = %d\n", P512_1);
	
	P1024_2 = get_row_parity(1024, 2);
	//printf("P1024_2 = %d  ", P1024_2);
	P1024_1 = get_row_parity(1024, 1);
	//printf("P1024_1 = %d\n", P1024_1);
	
	P2048_2 = get_row_parity(2048, 2);
	//printf("P2048_2 = %d  ", P2048_2);
	P2048_1 = get_row_parity(2048, 1);
	//printf("P2048_1 = %d\n", P2048_1);
	
	ecc[0] = (P64_1 << 7) | (P64_2 << 6) | (P32_1 << 5) | (P32_2 << 4) | \
			(P16_1 << 3) | (P16_2 << 2) | (P8_1 << 1) | (P8_2 << 0);

	ecc[1] = (P1024_1 << 7) | (P1024_2 << 6) | (P512_1 << 5) | (P512_2 << 4) | \
			(P256_1 << 3) | (P256_2 << 2) | (P128_1 << 1) | (P128_2 << 0);

	ecc[2] = (P4_1 << 7) | (P4_2 << 6) | (P2_1 << 5) | (P2_2 << 4) | \
			(P1_1 << 3) | (P1_2 << 2) | (P2048_1 << 1) | (P2048_2 << 0);
}

static void nfc_ms_read_l(unsigned long page_no, unsigned char *buf)
{
#if 0
	unsigned long cmd = NF_READ_1ST;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
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

	//printk("block = %d  page = %d\n", phyblk, pageinblk);

        REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L + pageinblk * PAGE_SIZE_L; 
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

	/*good place
	PAGEECC0 = 0;
	PAGEECC1 = 0;
	PAGEECC2 = 0;
	PAGEECC3 = 0;
	calculate(buf, &PAGEECC0);
	calculate(buf+512, &PAGEECC1);
	calculate(buf+1024, &PAGEECC2);
	calculate(buf+1536, &PAGEECC3);
	printk("\nREADPAGEECC0 = 0x%08x  ", PAGEECC0);		
	printk("PAGEECC1 = 0x%08x   ", PAGEECC1);		
	printk("PAGEECC2 = 0x%08x   ", PAGEECC2);
	printk("PAGEECC3 = 0x%08x\n", PAGEECC3);*/

#else
	unsigned long cmd = NF_READ_1ST;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
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
#if 0
	unsigned long cmd = NF_WRITE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
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

	//printk("block = %d  page = %d\n", phyblk, pageinblk);
	/*good place 
	PAGEECC0 = 0;
	PAGEECC1 = 0;
	PAGEECC2 = 0;
	PAGEECC3 = 0;
	calculate(buf, &PAGEECC0);
	calculate(buf+512, &PAGEECC1);
	calculate(buf+1024, &PAGEECC2);
	calculate(buf+1536, &PAGEECC3);
	printk("\nWRITEPAGEECC0 = 0x%08x  ", PAGEECC0);		
	printk("PAGEECC1 = 0x%08x   ", PAGEECC1);		
	printk("PAGEECC2 = 0x%08x   ", PAGEECC2);
	printk("PAGEECC3 = 0x%08x\n", PAGEECC3);*/

        REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L + pageinblk * PAGE_SIZE_L;
        REG_NFC_END0 = phyblk * pageperblk * PAGE_SIZE_L  + pageinblk * PAGE_SIZE_L + (PAGE_SIZE_L >> 1) - 1;	

	//REG_NFC_END0 = 0xffffffff;
	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
#if 0
	/* no ecc */	
	nand_copy(buf, (unsigned char *)NFC_MBUF, PAGE_SIZE_L);
#else
	REG_NFC_ECCEN = 0x1;
	nand_copy(buf, (unsigned char *)NFC_MBUF, PAGE_SIZE_L);
	printk("\nWRITEECC0 = 0x%08x  ", REG_NFC_PAGEECC0);		
	printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);		
	printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
	printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
	REG_NFC_ECCEN = 0;	
#endif
        nand_copy((buf+PAGE_SIZE_L), (unsigned char *)NFC_SBUF, SPARE_SIZE_L);
        REG_NFC_CMD = g_cmdsetting | cmd;
        nfc_wait_command_finish();

#else

	unsigned long cmd = NF_WRITE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
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
#if 0
	/* no ecc */	
	nand_copy(buf, (unsigned char *)NFC_MBUF, PAGE_SIZE_L);
#else
	REG_NFC_ECCEN = 0x1;
	nand_copy(buf, (unsigned char *)NFC_MBUF, PAGE_SIZE_L);
	printk("\nWRITEECC0 = 0x%08x  ", REG_NFC_PAGEECC0);		
	printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);		
	printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
	printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
	REG_NFC_ECCEN = 0;
#endif
        nand_copy((buf+PAGE_SIZE_L), (unsigned char *)NFC_SBUF, SPARE_SIZE_L);
        REG_NFC_CMD = g_cmdsetting | cmd;
        nfc_wait_command_finish();
#endif
}

static int memlookfor(unsigned char *org, unsigned char *patten, unsigned long len)
{
	unsigned long idx, addr;
	unsigned int isfind = 0;

	for (idx = 0; idx <= (PAGE_SIZE_L + SPARE_SIZE_L - len); idx++) {
		if (org[idx] == patten[0]) {
			isfind = 0;
			for (addr = idx; addr < (idx + len); addr++) {
				if (org[addr] == patten[addr - idx]) {
					isfind ++;
				}			
			}
			if (isfind == len)
				return idx;
		}
		isfind = 0;
	}
	
	return 0;
}

static void nfc_erase_block(unsigned long page_no)
{
#if 0
	unsigned long cmd = NF_BKERASE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;

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

	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
				(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

        REG_NFC_STR0 = phyblk * pageperblk * PAGE_SIZE_L;
        REG_NFC_CMD = g_cmdsetting | cmd;
        nfc_wait_command_finish();
	nfc_read_status();
#else

	unsigned long cmd = NF_BKERASE_ID;
	unsigned long i, phyblk, pageinblk, pageperblk;
	unsigned long addr_cycle = 4; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 0; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;

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
	unsigned long i, id, type;

#ifdef NAND_TEST_CODE
	unsigned char searchdata[] = {0xe0,0x82,0x20,0x08,0xe0,0x83,0x30,0x08};
	unsigned long searchlen = 0;
	unsigned char buffer[PAGE_SIZE_L + SPARE_SIZE_L];
	unsigned long pageno;
	unsigned long isfind;
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

#ifdef CONFIG_ARCH_SC8800S
	REG_AHB_CTL0 |= BIT_8 | BIT_9;//no BIT_9
	//REG_AHB_SOFT_RST |= BIT_5;
#else
	/* CONFIG_ARCH_SC8800G */
	REG_AHB_CTL0 |= BIT_8;
	//REG_AHB_SOFT_RST |= BIT_5; need test
#endif
	REG_NFC_INTSRC |= BIT_0 | BIT_4 | BIT_5;
	/* 0x1 : WPN disable, and micron nand flash status is 0xeo 
 	   0x0 : WPN enable, and micron nand flash status is 0x60 */
	REG_NFC_WPN = 0x1;
	
	set_sc8800g_gpio_as_nand_8bit();
#ifdef CONFIG_ARCH_SC8800S
	REG_GR_NFC_MEM_DLY = 0x0;
#endif
	set_nfc_param(0);
	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
	nfc_reset();
        mdelay(2);
        nfc_read_status();
        id = nfc_read_id();
	type = nand_bit_width(id);


#ifdef NAND_TEST_CODE
	/* run my test code */
	printk("\nRun nand test is starting\n");

	pageno = 1000 * 64;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);
	pageno = 1000 * 64 + 1;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);
	nfc_erase_block(pageno);

	memset(buffer, 0xff, PAGE_SIZE_L + SPARE_SIZE_L);
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
	
	pageno = 1000 * 64;
	nfc_ms_write_l(pageno, buffer);
	pageno = 1000 * 64 + 1;
	nfc_ms_write_l(pageno, buffer);
	
	pageno = 1000 * 64 + 1;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);
	pageno = 1000 * 64;
	memset(buffer, 0, PAGE_SIZE_L + SPARE_SIZE_L);
	nfc_ms_read_l(pageno, buffer);

#if 0	
	searchlen = sizeof(searchdata);
	printk("searchlen = %d\n", searchlen);
	for (pageno = 0; pageno < (2048 * 64); pageno++) {
		memset(buffer, 0xff, PAGE_SIZE_L + SPARE_SIZE_L);
		//printk(".");
		nfc_ms_read_l(pageno, buffer);
		/*if (pageno == 100) {
			buffer[15] = 0x11;buffer[16] = 0x22;buffer[17] = 0xf3;
		}
		if (pageno == 131008) {
			buffer[279] = 0x11;buffer[280] = 0x22;buffer[281] = 0xf3;
		}*/
		isfind = memlookfor(buffer, searchdata, searchlen);
		if (isfind > 0) {
			printk("\n\nIs finded! pageno = 0x%08x   idx = %d\n\nbuffer :\n", pageno, isfind);
			for (idx = 0; idx < (PAGE_SIZE_L + SPARE_SIZE_L); idx++) {
				if ((idx % 16) == 0)
					printk("\n");				
				printk("0x%02x,", buffer[idx]);
			}
			printk("\n\n");
		}
	}
#endif
	
	printk("\nRun nand test is ended\n");	

	return 0;
#endif

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	//info->clk = clk_get(&pdev->dev, "nand"); /* nand clock */
	//clk_enable(info->clk);

	memset(info, 0 , sizeof(*info));
	platform_set_drvdata(pdev, info);/* platform_device.device.driver_data IS info */
	info->platform = plat; /* nand timing */

	sprd_mtd = kmalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip), GFP_KERNEL);
	this = (struct nand_chip *)(&sprd_mtd[1]);
	memset((char *)sprd_mtd, 0, sizeof(struct mtd_info));
	memset((char *)this, 0, sizeof(struct nand_chip));

	sprd_mtd->priv = this;
	
	/* set the timing for nand controller */
	sprd_nand_inithw(info, pdev);
	/* 16-bit bus width */
	if (type == 1) {
		this->options |= NAND_BUSWIDTH_16;
		buswidth = 1;
		addr_cycle = 5;
		printk("is 16 Bit\n");	
	} else {
		buswidth = 0;
		addr_cycle = 4;
		printk("is 8 Bit\n");
	}
	this->cmd_ctrl = sprd_nand_hwcontrol;
	this->dev_ready = sprd_nand_devready;
	this->select_chip = sprd_nand_select_chip;
	this->nfc_readid = sprd_nand_readid;
	this->nfc_wr_oob = sprd_nand_wr_oob;
	this->ecc.calculate = sprd_nand_calculate_ecc;
	this->ecc.correct = sprd_nand_correct_data;
	this->ecc.hwctl = sprd_nand_enable_hwecc;
	this->ecc.mode = NAND_ECC_HW;
	this->ecc.size = 2048;//512;
	this->ecc.bytes = 16;//3
	this->chip_delay = 20;
	this->IO_ADDR_W = this->IO_ADDR_R = io_wr_port;	
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
	struct sprd_nand_info *info = container_of(nb, struct sprd_nand_info, freq_transition);
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
			set_nfc_param(0);
		}
		break;

	case CPUFREQ_POSTCHANGE:
		if (freq->new == max_freq) {
			set_nfc_param(80);
		} else if (freq->old == max_freq) {
			set_nfm_voltage(NFM_VOLTAGE_2650MV);
		}		

		break;
	}

	return 0;
}

static int nand_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	struct sprd_nand_info *info = container_of(nb, struct sprd_nand_info, freq_policy);
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
	struct sprd_nand_info *info = platform_get_drvdata(dev);
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
