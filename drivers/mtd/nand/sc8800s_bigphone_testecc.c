/* linux/drivers/mtd/nand/sprd8800.c
 *
 * Copyright (c) 2010 Spreadtrun.
 *
 * Spreadtrun 8800 NAND driver

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


/* NandFlash command */
#define NF_READ_STATUS  	0x70

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

#define BLOCK_TOTAL         1024
#define PAGEPERBLOCK	    64

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
#define REG_CPC_NFD10                            (*((volatile unsigned int *)(CPC_NFD10_REG)))
#define REG_CPC_NFD11                           (*((volatile unsigned int *)(CPC_NFD11_REG)))
#define REG_CPC_NFD12                            (*((volatile unsigned int *)(CPC_NFD12_REG)))
#define REG_CPC_NFD13                            (*((volatile unsigned int *)(CPC_NFD13_REG)))
#define REG_CPC_NFD14                            (*((volatile unsigned int *)(CPC_NFD14_REG)))
#define REG_CPC_NFD15                            (*((volatile unsigned int *)(CPC_NFD15_REG)))


#define REG_AHB_CTL0		       		(*((volatile unsigned int *)(AHB_CTL0)))

#define REG_GR_NFC_MEM_DLY                      (*((volatile unsigned int *)(GR_NFC_MEM_DLY)))

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

static struct sprd_platform_nand *to_nand_plat(struct platform_device *dev)
{
	return dev->dev.platform_data;
}

//static unsigned long g_CmdSetting;

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

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

static void set_nfc_param(unsigned long ahb_clk)
{
	nfc_wait_command_finish();
	
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
}

static void nand_copy(unsigned char *src, unsigned char *dst, unsigned long len)
{
	unsigned long i;
	unsigned long *pDst_32, *pSrc_32;
	unsigned short *pDst_16, *pSrc_16;
	unsigned long flag = 0;
	
	//flag = (unsigned long *)dst;
	flag = (unsigned long)dst;
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

static void nand_copy16(unsigned short *src, unsigned short *dst, unsigned long len)
{
	unsigned long i;
	unsigned short *pDst_16, *pSrc_16;

        pDst_16 = (unsigned short *)dst;
        pSrc_16 = (unsigned short *)src;
        for (i = 0; i < (len / 2); i++) {
        	*pDst_16 = *pSrc_16;
                pDst_16++;
                pSrc_16++;
	}
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
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
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
   	    	addr_cycle = 1;

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
				/* transfer to litter end */
				i = io_wr_port[3]; io_wr_port[3] = io_wr_port[0]; io_wr_port[0] = i;
				i = io_wr_port[2]; io_wr_port[2] = io_wr_port[1]; io_wr_port[1] = i;
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

        				REG_NFC_STR0 = sprd_colrow_addr.row * mtd->writesize;
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
				if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
        				g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
							(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
        				REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        				nfc_wait_command_finish();
				
					//nand_copy((unsigned char *)NFC_SBUF, this->IO_ADDR_R, mtd->oobsize);
					//nand_copy((unsigned char *)NFC_SBUF, io_wr_port, mtd->oobsize);
					nand_copy((unsigned long *)NFC_SBUF, (unsigned long)io_wr_port, mtd->oobsize);

        				/*for (i = 0; i < mtd->oobsize; i++)
                				printk(" Rport[%d]=0x%02x ", i, io_wr_port[i]);*/
				} else if (sprd_colrow_addr.column == 0) {
						printk("%s  %s  %d  mode=%d\n", __FILE__, __FUNCTION__, __LINE__, sprd_area_mode);
						if (sprd_area_mode == DATA_AREA)
							sprd_area_mode = DATA_OOB_AREA;

						if (sprd_area_mode == DATA_OOB_AREA) {
                                                	/* read data and spare area */
							printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
                                                	REG_NFC_END0 = 0xffffffff;
                                                	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
									(1 << 21) | (buswidth << 19) | (pagetype << 18) | \
									(0 << 16) | (0x1 << 31);
                                                	REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
                                                	nfc_wait_command_finish();
							nand_copy((unsigned char *)NFC_MBUF, io_wr_port, mtd->writesize);
							
							for (i = 0; i < mtd->writesize; i+=100)
                						printk(" Rport[%d]=%d ", i, io_wr_port[i]);

                                        	} else if (sprd_colrow_addr.column == DATA_AREA) {
							printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
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
				if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
					printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
        				for (i = 0; i < mtd->oobsize; i++)
                				printk(" Wport[%d]=0x%02x ", i, io_wr_port[i]);

					g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
							(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
        			
					//nand_copy(this->IO_ADDR_W, (unsigned char *)NFC_SBUF, mtd->oobsize);
					nand_copy((unsigned long *)io_wr_port, (unsigned long *)NFC_SBUF, mtd->oobsize);
        				REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        				nfc_wait_command_finish();
				} else if (sprd_colrow_addr.column == 0) {
					printk("%s  %s  %d  \n", __FILE__, __FUNCTION__, __LINE__);
					if (sprd_area_mode == DATA_OOB_AREA) {
						/* write data and spare area, modify address */
        					/*REG_NFC_END0 = phyblk * pageperblk * mtd->writesize * 2 + 
								pageinblk * mtd->writesize * 2 + 
								mtd->writesize + mtd->writesize - 1;*/
						REG_NFC_END0 = 0xffffffff;
						g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
							(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

        			
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
			printk("%d  column = %d\n", __LINE__, cmd);
			return;
		}
		
		if (0 == sprd_colrow_addr.rowflag) {
			sprd_colrow_addr.rowflag = 1;
			sprd_colrow_addr.row = cmd;
			printk("%d  row = %d\n",  __LINE__, cmd);
		}
		
		if ((1 == sprd_colrow_addr.colflag) && (1 == sprd_colrow_addr.rowflag)) {
			if (sprd_colrow_addr.column == (mtd->writesize >> 1)) {
				pageperblk = mtd->erasesize / mtd->writesize;
				phyblk = sprd_colrow_addr.row / pageperblk;
        			pageinblk = sprd_colrow_addr.row % pageperblk;
				printk("block = %d  page = %d  column = %d\n", phyblk, pageinblk, sprd_colrow_addr.column);
        			REG_NFC_STR0 = phyblk * pageperblk * mtd->writesize + 
							pageinblk * mtd->writesize + 
							sprd_colrow_addr.column;
        			REG_NFC_END0 = phyblk * pageperblk * mtd->writesize + 
							pageinblk * mtd->writesize + 
							sprd_colrow_addr.column + (mtd->oobsize >> 1) -1;
				sprd_area_mode = OOB_AREA;	
				printk("Operation OOB area.  %s  %s  %d   row=0x%08x  column=0x%08x\n", 
					__FILE__, __FUNCTION__, __LINE__, sprd_colrow_addr.row, sprd_colrow_addr.column);
			} else if (sprd_colrow_addr.column == 0) {
				/*REG_NFC_STR0 = sprd_colrow_addr.row * mtd->writesize + sprd_colrow_addr.column;
				REG_NFC_END0 = sprd_colrow_addr.row * mtd->writesize + sprd_colrow_addr.column 
						+ mtd->writesize - 1;*/
				
				pageperblk = mtd->erasesize / mtd->writesize;
				phyblk = sprd_colrow_addr.row / pageperblk;
        			pageinblk = sprd_colrow_addr.row % pageperblk;

				printk("block = %d  page = %d  column = %d\n", phyblk, pageinblk, sprd_colrow_addr.column);

        			REG_NFC_STR0 = phyblk * pageperblk * mtd->writesize + 
							pageinblk * mtd->writesize + 
							sprd_colrow_addr.column;
        			REG_NFC_END0 = phyblk * pageperblk * mtd->writesize  + 
							pageinblk * mtd->writesize + 
							sprd_colrow_addr.column + (mtd->writesize >> 1) - 1;
				sprd_area_mode = DATA_AREA;	
				printk("Operation DATA area.  %s  %s  %d   row=0x%08x  column=0x%08x\n", 
					__FILE__, __FUNCTION__, __LINE__, sprd_colrow_addr.row, sprd_colrow_addr.column);
			} else
				printk("Operation ??? area.  %s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
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
        unsigned long cmd = NF_READ_STATUS | (0x1 << 31);

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
	for (i = 0; i < mtd->oobsize; i++)
                	printk(" OOBWport[%d]=%d ", i, io_wr_port[i]);
        nand_copy(io_wr_port, (unsigned char *)NFC_SBUF, mtd->oobsize);

	/* write oob area */
	if (sprd_area_mode == NO_AREA)
		sprd_area_mode = OOB_AREA;
	else if (sprd_area_mode == DATA_AREA)
		sprd_area_mode = DATA_OOB_AREA;

	return 0;
}

static int sprd_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	unsigned char ecc_val_in[16];
        unsigned long *pecc_val;
	unsigned int i;			

	if (sprd_ecc_mode == NAND_ECC_WRITE) {
		pecc_val = (unsigned long *)ecc_val_in;

		REG_NFC_ECCEN = 0x1;

		for (i = 0; i < mtd->writesize; i+=100)
	               printk(" Wport[%d]=%d ", i, io_wr_port[i]);

		/* copy io_wr_port into MBUF */
		nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, mtd->writesize);
		/* large page */
		pecc_val[0] = REG_NFC_PAGEECC0;
               	pecc_val[1] = REG_NFC_PAGEECC1;
               	pecc_val[2] = REG_NFC_PAGEECC2;
               	pecc_val[3] = REG_NFC_PAGEECC3;
		
		/*printk("\nECC0 = 0x%08x  ", REG_NFC_PAGEECC0);		
		printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);		
		printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);		
		printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
		for (i = 0; i < 16; i++)
			printk("0x%02x ", ecc_val_in[i]);*/		

#if 0
		/* little endian */
		ecc_code[0] = ecc_val_in[0];
		ecc_code[1] = ecc_val_in[1];
		ecc_code[2] = ecc_val_in[2];

		ecc_code[3] = ecc_val_in[4];
		ecc_code[4] = ecc_val_in[5];
		ecc_code[5] = ecc_val_in[6];

		ecc_code[6] = ecc_val_in[8];
		ecc_code[7] = ecc_val_in[9];
		ecc_code[8] = ecc_val_in[10];

		ecc_code[9] = ecc_val_in[12];
		ecc_code[10] = ecc_val_in[13];
		ecc_code[11] = ecc_val_in[14];
#else
		/* big endian */
		ecc_code[0] = ecc_val_in[3];
		ecc_code[1] = ecc_val_in[2];
		ecc_code[2] = ecc_val_in[1];

		ecc_code[3] = ecc_val_in[7];
		ecc_code[4] = ecc_val_in[6];
		ecc_code[5] = ecc_val_in[5];

		ecc_code[6] = ecc_val_in[11];
		ecc_code[7] = ecc_val_in[10];
		ecc_code[8] = ecc_val_in[9];

		ecc_code[9] = ecc_val_in[15];
		ecc_code[10] = ecc_val_in[14];
		ecc_code[11] = ecc_val_in[13];
#endif
		REG_NFC_ECCEN = 0;
		memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);	
	} else if (sprd_ecc_mode == NAND_ECC_READ) {
 		pecc_val = (unsigned long *)ecc_val_in;
                /* large page */
                pecc_val[0] = REG_NFC_PAGEECC0;
                pecc_val[1] = REG_NFC_PAGEECC1;
                pecc_val[2] = REG_NFC_PAGEECC2;
                pecc_val[3] = REG_NFC_PAGEECC3;

                /*printk("\nECC0 = 0x%08x  ", REG_NFC_PAGEECC0);
                printk("ECC1 = 0x%08x   ", REG_NFC_PAGEECC1);
                printk("ECC2 = 0x%08x   ", REG_NFC_PAGEECC2);
                printk("ECC3 = 0x%08x\n", REG_NFC_PAGEECC3);
		for (i = 0; i < 16; i++)
			printk("0x%02x ", ecc_val_in[i]);*/

#if 0
                /* little endian */
                ecc_code[0] = ecc_val_in[0];
                ecc_code[1] = ecc_val_in[1];
                ecc_code[2] = ecc_val_in[2];

                ecc_code[3] = ecc_val_in[4];
                ecc_code[4] = ecc_val_in[5];
                ecc_code[5] = ecc_val_in[6];

                ecc_code[6] = ecc_val_in[8];
                ecc_code[7] = ecc_val_in[9];
                ecc_code[8] = ecc_val_in[10];

                ecc_code[9] = ecc_val_in[12];
                ecc_code[10] = ecc_val_in[13];
                ecc_code[11] = ecc_val_in[14];
#else
                /* big endian */
                ecc_code[0] = ecc_val_in[3];
                ecc_code[1] = ecc_val_in[2];
                ecc_code[2] = ecc_val_in[1];

                ecc_code[3] = ecc_val_in[7];
                ecc_code[4] = ecc_val_in[6];
                ecc_code[5] = ecc_val_in[5];

                ecc_code[6] = ecc_val_in[11];
                ecc_code[7] = ecc_val_in[10];
                ecc_code[8] = ecc_val_in[9];

                ecc_code[9] = ecc_val_in[15];
                ecc_code[10] = ecc_val_in[14];
                ecc_code[11] = ecc_val_in[13];
#endif
                memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
		nand_copy((unsigned char *)NFC_SBUF, io_wr_port, mtd->oobsize);
		for (i = 0; i < mtd->oobsize; i++)
                	printk(" OOBRport[%d]=%d ", i, io_wr_port[i]);
	}	

	/*for (j = 0; j < 12; j++)
		printk(" ecc_code[%d]=%02x ", j, ecc_code[j]);*/
	sprd_ecc_mode = NAND_ECC_NONE;

	return 0;
}

static int correct(u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	unsigned int diff0, diff1, diff2;
	unsigned int bit, byte;

	diff0 = read_ecc[0] ^ calc_ecc[0];
	diff1 = read_ecc[1] ^ calc_ecc[1];
	diff2 = read_ecc[2] ^ calc_ecc[2];
	
	//printk("diff0=0x%08x  diff1=0x%08x  diff2=0x%08x\n", diff0, diff1, diff2);

	if (diff0 == 0 && diff1 == 0 && diff2 == 0) {
		return 0;
	}
	if (((diff0 ^ (diff0 >> 1)) & 0x55) == 0x55 &&
	    ((diff1 ^ (diff1 >> 1)) & 0x55) == 0x55 &&
	    ((diff2 ^ (diff2 >> 1)) & 0x55) == 0x55) {
		/* calculate the bit position of the error */
		bit  = (diff2 >> 2) & 1;
		bit |= (diff2 >> 3) & 2;
		bit |= (diff2 >> 4) & 4;
		/* calculate the byte position of the error */
		byte  = (diff1 << 1) & 0x80;
		byte |= (diff1 << 2) & 0x40;
		byte |= (diff1 << 3) & 0x20;
		byte |= (diff1 << 4) & 0x10;
		byte |= (diff0 >> 3) & 0x08;
		byte |= (diff0 >> 2) & 0x04;
		byte |= (diff0 >> 1) & 0x02;
		byte |= (diff0 >> 0) & 0x01;
		byte |= (diff2 << 8) & 0x100;
		dat[byte] ^= (1 << bit);
		return 1;
	}
	diff0 |= (diff1 << 8);
	diff0 |= (diff2 << 16);
	
	if ((diff0 & ~(1 << fls(diff0))) == 0) {
		return 1; /* ecc itself is wrong, data is right */
	}
	
	/* uncorrectable ecc error */
	return -1;
}

static int sprd_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	int i, retval = 0;
	printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
	if (mtd->writesize > 512) {
		for (i = 0; i < 4; i++) {
			if (correct(dat + 512 * i, read_ecc + 3 * i, calc_ecc + 3 * i) == -1) {
				printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);				
				retval = -1;
				return 0;
			}
		}
	} else
		retval = correct(dat, read_ecc, calc_ecc);
	
	return retval;
}

void nfc_reset(void)
{
	REG_NFC_CMD = NAND_CMD_RESET | (0x1 << 31);
	nfc_wait_command_finish();
	mdelay(2);
}

void nfc_read_status(void)
{
	unsigned long i;
	REG_NFC_CMD = NAND_CMD_STATUS | (0x1 << 31);
	nfc_wait_command_finish();
				
	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
	nand_copy((unsigned char *)NFC_IDSTATUS, io_wr_port, 4);
	/* transfer to litter end */
	i = io_wr_port[3]; io_wr_port[3] = io_wr_port[0]; io_wr_port[0] = i;
	i = io_wr_port[2]; io_wr_port[2] = io_wr_port[1]; io_wr_port[1] = i;
        for (i = 0; i < 4; i++)
                printk("io_wr_port[%d] = 0x%02x\n", i, io_wr_port[i]);
}

void nfc_read_id(void)
{
	REG_NFC_CMD = NAND_CMD_READID | (0x1 << 31);
        nfc_wait_command_finish();
        nand_flash_id = REG_NFC_IDSTATUS;
	printk("%s  %s  %d   nand_flash_id=0x%08x\n", __FILE__, __FUNCTION__, __LINE__, nand_flash_id);
}

void nfc_erase_block(unsigned long block_no)
{
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype = 1; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;

	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 1;


	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);

        REG_NFC_STR0 =  block_no * 64 * 2048;
       	REG_NFC_CMD = g_cmdsetting | NAND_CMD_ERASE1;
       	nfc_wait_command_finish();
}

void nand_ecc_trans(unsigned char *pEccIn, unsigned char *pEccOut, unsigned char nSct)
{
        //BUS_WIDTH = 16
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
}

void nfc_ms_write16_l(unsigned long page_no, unsigned char *buf, unsigned long ecc)
{
	unsigned char ecc_val_in[16];
	unsigned char ecc_val_out[16];
        unsigned long *pecc_val;
	unsigned int i;
	unsigned long pageperblk, phyblk, pageinblk, column;
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype = 1; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;

	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 1;

				
	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;
	column = 0;

	printk("block = %d  page = %d  column = %d\n", phyblk, pageinblk, column);

        REG_NFC_STR0 = phyblk * pageperblk * 2048 + 
			pageinblk * 2048 + 
			column;
        REG_NFC_END0 = phyblk * pageperblk * 2048  + 
			pageinblk * 2048 + 
			column + 1024 - 1;

	memset(ecc_val_in, 0xff, 16);
	memset(ecc_val_out, 0xff, 16);

	pecc_val = (unsigned long *)ecc_val_in;
	printk("\n\n");
	for (i = 0; i < 20; i++)
        	printk("%d,", io_wr_port[i]);
	printk("\n\n");
	REG_NFC_ECCEN = 0x1;
	/* copy io_wr_port into MBUF */
	//nand_copy(io_wr_port, (unsigned char *)NFC_MBUF, 2048);
	nand_copy16((unsigned short *)io_wr_port, (unsigned short *)NFC_MBUF, 2048);
	/* large page */
	pecc_val[0] = REG_NFC_PAGEECC0;
        pecc_val[1] = REG_NFC_PAGEECC1;
        pecc_val[2] = REG_NFC_PAGEECC2;
        pecc_val[3] = REG_NFC_PAGEECC3;
		
	printk("\nECC0 = 0x%08x  ", pecc_val[0]);		
	printk("ECC1 = 0x%08x   ", pecc_val[1]);		
	printk("ECC2 = 0x%08x   ", pecc_val[2]);		
	printk("ECC3 = 0x%08x\n", pecc_val[3]);
	
	/*printk("---------in1---------\n");
	for (i = 0; i < 16; i++)
		printk("0x%02x ", ecc_val_in[i]);*/
	nand_ecc_trans(ecc_val_in, ecc_val_out, 4);
	/*printk("\n---------out1---------\n");
	for (i = 0; i < 16; i++)
		printk("0x%02x ", ecc_val_out[i]);*/
	REG_NFC_ECCEN = 0;
	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
	io_wr_port[40] = ecc_val_out[0];
	io_wr_port[41] = ecc_val_out[1];
	io_wr_port[42] = ecc_val_out[2];

	io_wr_port[43] = ecc_val_out[4];
	io_wr_port[44] = ecc_val_out[5];
	io_wr_port[45] = ecc_val_out[6];

	io_wr_port[46] = ecc_val_out[8];
	io_wr_port[47] = ecc_val_out[9];
	io_wr_port[48] = ecc_val_out[10];

	io_wr_port[49] = ecc_val_out[12];
	io_wr_port[50] = ecc_val_out[13];
	io_wr_port[51] = ecc_val_out[14];

        nand_copy((unsigned long *)io_wr_port, (unsigned long *)NFC_SBUF, 64);

	REG_NFC_END0 = 0xffffffff;
	g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(buswidth << 19) | (pagetype << 18) | (0 << 16) | (0x1 << 31);
        			
        REG_NFC_CMD = g_cmdsetting | NAND_CMD_SEQIN;
        nfc_wait_command_finish();
	
}

void nfc_ms_read16_l(unsigned long page_no, unsigned char *buf, unsigned long ecc)
{
	unsigned char ecc_val_in[16];
	unsigned char ecc_val_out[16];
        unsigned long *pecc_val;
	unsigned int i;
	unsigned long pageperblk, phyblk, pageinblk, column;
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype = 1; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;

	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 1;

				
	pageperblk = 64;
	phyblk = page_no / pageperblk;
        pageinblk = page_no % pageperblk;
	column = 0;

	printk("block = %d  page = %d  column = %d\n", phyblk, pageinblk, column);

       	REG_NFC_STR0 = phyblk * pageperblk * 2048 + 
			pageinblk * 2048 + 
			column;
        /*REG_NFC_END0 = phyblk * pageperblk * 2048  + 
			pageinblk * 2048 + 
			column + 1024 - 1;*/

	memset(ecc_val_in, 0xff, 16);
	memset(ecc_val_out, 0xff, 16);

	pecc_val = (unsigned long *)ecc_val_in;
	REG_NFC_END0 = 0xffffffff;
        g_cmdsetting = (chipsel << 26) | (addr_cycle << 24) | (advance << 23) | \
			(1 << 21) | (buswidth << 19) | (pagetype << 18) | \
			(0 << 16) | (0x1 << 31);
                                             
	REG_NFC_CMD = g_cmdsetting | NAND_CMD_READ0;
        nfc_wait_command_finish();
	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
	nand_copy16((unsigned short *)NFC_MBUF, (unsigned short *)io_wr_port, 2048);
	printk("%s  %s  %d\n", __FILE__, __FUNCTION__, __LINE__);
	printk("\n\n");						
	for (i = 0; i < 20; i++)
                printk("%d,", io_wr_port[i]);
	printk("\n\n");	
	pecc_val = (unsigned long *)ecc_val_in;
        /* large page */
        pecc_val[0] = REG_NFC_PAGEECC0;
        pecc_val[1] = REG_NFC_PAGEECC1;
        pecc_val[2] = REG_NFC_PAGEECC2;
        pecc_val[3] = REG_NFC_PAGEECC3;

        printk("\nECC0 = 0x%08x  ", pecc_val[0]);
        printk("ECC1 = 0x%08x   ", pecc_val[1]);
        printk("ECC2 = 0x%08x   ", pecc_val[2]);
        printk("ECC3 = 0x%08x\n", pecc_val[3]);

	/*printk("---------in2--------\n");
	for (i = 0; i < 16; i++)
		printk("0x%02x ", ecc_val_in[i]);*/
	nand_ecc_trans(ecc_val_in, ecc_val_out, 4);
	/*printk("\n---------out2---------\n");
	for (i = 0; i < 16; i++)
		printk("0x%02x ", ecc_val_out[i]);*/

	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);

	nand_copy((unsigned long *)NFC_SBUF, (unsigned long *)io_wr_port, 64);
	/*for (i = 0; i < 64; i++)
                printk(" OOBRport[%d]=%d ", i, io_wr_port[i]);*/
}

/* driver device registration */
static int sprd_nand_probe(struct platform_device *pdev)
{
	unsigned long phyblk, pageinblk, pageperblk;
	unsigned long i, pageno;
	unsigned long addr_cycle = 5; /* advance 0 : can be set 3 or 4; advance 1: can be set 4 or 5 */
	unsigned long advance = 1; /* can be set 0 or 1 */
	unsigned long pagetype = 1; /* 0: small page; 1: large page*/
	unsigned long buswidth = 1; /* 0: X8 bus width 1: X16 bus width */
	unsigned long chipsel = 0;
	unsigned char datain[] = {222,111,129,115,75,251,226,251,84,246,189,223,124,28,225,135,1,191,49,222,
86,114,15,71,103,102,135,89,170,136,60,89,234,86,19,123,210,133,161,216,60,84,85,47,55,174,101,91,218,2,121,
152,204,227,26,118,142,95,217,153,143,31,63,54,238,67,120,77,13,250,190,166,218,228,134,142,220,41,109,78,255,86,
225,112,32,251,143,177,88,5,144,197,9,220,83,205,170,59,72,153,82,211,82,157,6,159,234,181,194,6,19,152,73,178,1,30,
172,50,136,49,156,82,70,149,113,54,143,87,246,57,29,22,250,136,116,245,152,124,23,92,65,187,109,113,142,15,112,89,199,
1,27,47,51,61,145,192,29,165,13,13,171,51,141,126,94,143,62,230,104,116,166,58,177,195,147,17,168,100,199,219,202,224,96,
225,243,191,9,0,103,162,227,37,160,33,49,135,213,98,197,168,79,126,46,9,107,148,159,176,109,169,158,90,11,70,112,128,182,
207,71,12,166,165,42,216,172,251,160,235,183,121,36,114,35,146,72,128,197,166,167,133,183,215,140,144,228,171,99,68,82,102,
227,156,51,37,249,94,170,186,115,96,93,75,113,126,190,169,140,87,25,113,195,202,94,229,42,51,172,136,81,102,161,123,117,103,100,
154,105,239,111,86,66,160,29,81,197,2,247,187,146,69,190,111,13,182,56,204,16,253,187,84,81,28,123,7,148,39,147,125,146,195,212,
198,165,97,81,1,56,56,167,191,241,4,13,21,155,128,31,131,213,164,105,136,124,159,182,1,218,147,23,69,139,18,178,2,51,92,80,214,
225,86,164,173,66,74,92,221,134,97,233,3,18,225,15,155,234,38,44,97,220,98,72,107,109,20,224,3,133,74,114,70,218,150,200,125,28,
209,5,62,229,146,112,67,95,108,3,5,179,235,179,32,53,77,126,102,80,1,54,192,51,225,15,201,56,46,233,41,25,79,94,177,209,73,139,
59,83,253,159,63,238,37,37,53,123,13,17,175,76,17,140,50,212,218,127,216,22,87,225,166,206,125,193,174,98,191,19,228,135,76,58,
193,179,12,89,153,71,88,90,189,120,124,186,80,1,237,27,234,138,73,136,238,214,20,133,171,176,44,222,53,147,17,45,1,28,215,40,67,
48,231,176,8,237,121,153,19,81,210,58,119,173,61,180,248,199,202,3,34,210,201,198,39,15,4,206,122,63,192,104,44,207,114,106,9,
194,66,0,114,94,65,52,248,150,105,63,189,58,88,145,139,225,204,162,177,146,221,119,161,53,254,243,75,188,177,227,55,17,13,199,
101,190,241,97,229,94,6,255,53,199,118,137,93,244,110,74,204,181,84,126,241,21,200,160,153,143,92,112,11,239,20,198,229,10,156,
25,180,29,76,206,86,6,220,66,17,37,231,150,111,15,33,61,223,249,87,71,13,223,43,106,252,119,141,213,233,217,249,181,224,235,114,
132,26,142,66,20,29,138,110,95,146,58,251,11,229,246,228,192,159,69,214,42,131,191,177,205,106,196,191,140,222,223,178,247,121,
247,96,87,252,59,61,123,46,203,156,65,123,39,165,227,72,88,21,7,23,224,185,133,95,99,168,246,41,18,67,0,106,219,238,100,36,82,
139,196,59,93,187,53,24,162,211,137,255,178,160,89,48,242,219,213,193,77,106,75,54,156,93,120,230,208,163,146,13,229,144,17,176,
134,15,65,52,128,166,137,189,233,47,120,71,13,80,149,135,27,191,227,127,148,55,54,228,111,57,56,47,12,131,58,133,223,81,188,72,
217,86,187,121,149,121,189,212,72,80,157,169,101,93,23,124,19,11,18,92,79,103,176,4,225,158,24,179,0,58,254,203,196,28,247,43,
80,56,126,78,187,19,197,32,195,254,61,164,48,15,228,71,10,228,82,1,122,23,129,49,128,128,95,53,90,45,21,204,176,34,21,45,128,
209,230,228,204,88,175,111,5,125,133,156,53,106,116,160,240,40,79,247,249,220,56,0,179,196,238,84,78,241,217,234,173,194,215,
235,25,36,196,86,168,139,203,84,107,175,112,88,90,7,89,254,0,6,223,161,230,24,89,186,193,91,35,252,91,30,112,48,66,26,212,208,
50,114,144,102,66,108,157,162,209,237,119,62,48,182,174,146,13,97,46,246,162,26,73,219,161,29,137,168,222,242,56,86,186,107,
171,202,83,90,83,246,109,19,129,174,31,165,252,74,61,215,69,1,137,228,164,0,152,246,251,77,134,100,70,95,89,172,245,121,54,47,
234,202,70,175,80,70,102,137,33,66,145,177,118,210,13,114,141,227,88,227,156,23,209,40,88,99,39,110,68,107,130,164,186,152,115,
250,187,255,156,26,118,242,31,41,153,98,200,124,91,251,249,26,70,253,89,246,197,219,60,233,113,150,208,113,28,216,13,44,153,208,
90,18,81,208,0,117,135,168,79,186,102,192,146,213,208,247,180,134,229,63,175,85,85,245,184,78,102,1,44,125,196,178,56,40,12,86,
75,207,23,156,61,228,7,171,60,74,18,254,123,144,17,6,153,234,199,125,209,243,242,140,231,37,20,156,206,20,254,252,25,109,33,55,
40,178,148,51,15,179,228,10,69,203,159,168,17,224,159,41,180,24,23,239,87,92,95,134,179,141,127,57,130,137,125,113,169,220,103,
208,34,70,31,17,171,241,233,158,48,111,182,238,249,117,46,165,148,89,127,105,128,77,232,133,158,89,4,64,88,26,215,251,142,60,154,
13,69,185,70,95,14,206,226,198,56,194,141,36,181,86,75,61,205,11,143,89,132,22,140,159,204,36,60,44,107,206,45,246,170,218,14,
100,195,55,253,169,8,183,142,228,211,138,155,249,49,126,206,45,77,248,239,131,158,177,238,218,208,50,176,195,115,13,154,36,102,
225,222,142,2,11,136,93,6,44,71,149,69,95,252,119,17,55,4,230,102,123,70,125,214,161,251,109,56,11,64,23,16,3,93,109,189,120,
211,9,101,118,39,10,161,103,113,178,231,11,163,192,187,57,154,142,149,83,230,235,145,138,90,182,217,215,82,63,210,180,199,93,
9,158,20,79,220,76,133,83,232,172,165,8,54,162,68,132,36,128,74,53,21,67,63,120,216,147,150,251,217,121,188,211,10,222,229,92,
143,199,145,212,44,82,224,183,111,112,155,216,157,96,254,68,93,239,71,214,38,113,255,154,106,125,11,226,127,108,113,42,82,144,
235,173,202,53,46,195,253,89,247,1,21,42,218,15,1,68,202,71,219,167,103,19,28,122,11,3,130,129,147,177,188,96,237,85,219,141,
102,39,121,22,177,120,167,24,182,143,152,251,32,68,14,110,165,94,136,38,20,174,40,86,32,232,102,237,238,68,119,146,96,216,123,
96,31,180,105,97,107,187,187,204,162,68,217,254,145,116,70,58,126,89,140,33,241,199,232,240,70,243,182,123,244,209,155,237,155,
45,116,61,207,139,1,111,167,192,81,143,4,77,237,110,161,126,196,27,223,71,218,32,78,217,175,130,251,7,112,118,124,92,224,227,
188,248,4,156,135,47,145,90,212,224,22,122,214,149,233,124,193,95,211,55,92,111,123,221,75,116,147,179,26,184,196,141,9,250,90,
10,154,9,175,149,219,37,89,23,116,21,19,124,111,8,108,236,202,44,49,198,190,16,155,92,205,186,57,113,142,136,156,115,56,199,196,
120,240,21,77,251,210,119,89,83,193,57,60,247,239,137,234,115,43,210,33,41,238,217,86,200,36,154,97,142,186,224,232,61,235,167,
139,222,75,49,212,57,144,234,221,15,35,253,190,30,107,178,189,210,212,142,53,203,161,41,66,19,119,205,50,27,165,211,171,122,52,
190,156,102,178,20,228,238,191,0,197,253,84,169,7,15,214,80,236,176,223,44,215,185,199,5,187,74,244,146,68,134,230,148,200,17,1,
175,236,75,25,11,22,73,192,174,150,151,79,151,147,176,181,155,183,58,2,1,154,2,178,220,240,186,186,43,113,116,84,177,139,221,139,
149,202,58,133,186,3,36,148,220,68,3,251,111,123,76,128,56,233,122,182,168,68,206,7,251,175,198,131,21,90,93,108,23,249,8,125,
196,230,109,254,151,21,225,137,160,187,168,153,34,190,236,216,238,219,121,37,126,153,62,103,208,241,132,20,8,186,235,128,197,214,
41,230,63,31,130,56,37,15,7,166,56,49,142,240,167,75,117,108,41,72,22,214,221,232,8,219,225,38,27,100,180,108,18,163,76,121,30,
222,246,14,31,254,242,92,154,214,202,45,120,53,118,212,132,170,49,214,162,26,25,85,208,3,137,64,222,141,55,60,237,85,12,81,169,
249,198,85,70,99,80,25,60,214,220,85,193,186,198,83,11,39,41,94,66,51,61,234,71,252,119,128,39,116,94,112,93,239,47,52,203,110,
48,166,119,169,212,226,6,222,148,249,249,92,136,90,169,206,199,1,3,72,132,93,4,19,229,2,243,57,162,19,98,207,98,109,226,5,214,
148,137,239,145,94,37,16,174,97,61,171,32,29,203,202,215,234,188,11,152,160,35,45,153,8,65,94,223,5,54,66,88,130,131,195,184,27,
71,155,20,139,53,163,63,216,88,217,232,64,134};

	if(addr_cycle == 3)
   		addr_cycle = 0;
   	else if((addr_cycle == 4) &&(advance == 1))
   	    	addr_cycle = 0;
   	else
   	    	addr_cycle = 1;

	REG_AHB_CTL0 |= BIT_8 | BIT_9;//no BIT_9
	REG_NFC_INTSRC |= BIT_0 | BIT_4 | BIT_5;
	/* 0x1 : WPN disable, and micron nand flash status is 0xeo 
 	   0x0 : WPN enable, and micron nand flash status is 0x60 */
	REG_NFC_WPN = 0x1;
	
	set_gpio_as_nand();
	REG_GR_NFC_MEM_DLY = 0x0;
	set_nfc_param(53);//53MHz
	memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);	

     	printk("\n\n\n   Test nand flash controller start ...\n\n\n");
        nfc_reset();
        mdelay(2);
        nfc_read_status();
        nfc_read_id();
	
	printk("io_wr_port is 0x%08x\n", io_wr_port);	

        pageno = 59392;
        nfc_erase_block(pageno / 64);
	nfc_read_status();

        memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
        for (i = 0; i < PAGE_SIZE_L; i++)
                io_wr_port[i] = datain[i];

        nfc_ms_write16_l(pageno, io_wr_port, 1);

	pageno = 59392;
        memset(io_wr_port, 0xff, NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE);
        nfc_ms_read16_l(pageno, io_wr_port, 1);

        printk("\n\n\n   Test nand flash controller end \n\n\n");

	return 0;
}

/* device management functions */

static int sprd_nand_remove(struct platform_device *pdev)
{
	struct sprd_nand_info *info;// = to_nand_info(pdev);

	platform_set_drvdata(pdev, NULL);
	if (info == NULL)
		return 0;
	
	del_mtd_partitions(sprd_mtd);
	del_mtd_device(sprd_mtd);
	kfree(sprd_mtd);
	//clk_disable(info->clk);
	kfree(info);	

	return 0;
}

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
