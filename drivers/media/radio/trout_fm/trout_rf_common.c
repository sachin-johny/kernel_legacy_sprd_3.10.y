#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "trout_fm_ctrl.h"
#include "trout_rf_common.h"

#ifdef USE_RF_2829
#include "trout_rf_2829.h"
#endif

#ifdef USE_RF_55
#include "trout_rf_55.h"
#endif

trout_reg_cfg_t fm_reg_init_des[]={
	{FM_REG_FM_CTRL	,		0x1011},
	//{FM_REG_FM_EN	,			0x36F6},
	//{FM_REG_FM_EN	,			0x36EE},
	{FM_REG_FM_EN	,			0x32EE},
	{FM_REG_CHAN	,			(860)*2},
	{FM_REG_AGC_TBL_CLK,		0},
	{FM_REG_SEEK_LOOP	,	(1080-800)},
	{FM_REG_FMCTL_STI	,		0},
	{FM_REG_BAND_LMT	,		(((1080-1)*2)<<16)|((860-1)*2)},
	{FM_REG_BAND_SPACE	,	2},
	//{FM_REG_SEEK_CH_TH	,	800},			//0x320
	{FM_REG_SEEK_CH_TH	,	2000},	/* Modified by Xuede, fix bug 156417 */	
	//{FM_REG_AGC_TBL_RFRSSI0,	0x200800},
	{FM_REG_AGC_TBL_RFRSSI0,	0x6f0800},
	{FM_REG_AGC_TBL_RFRSSI1,	(31 << 24) | (5 << 16) | (3000)},		//0x1f050bb8
	{FM_REG_AGC_TBL_WBRSSI,	0x00F60C12},
	{FM_REG_AGC_IDX_TH	,	0x00020407},
	{FM_REG_AGC_RSSI_TH	,	0x00F600D8},
	{FM_REG_SEEK_ADPS	,	0x0CD80000},
	{FM_REG_STER_PWR	,		0x000001A6},//
	{FM_REG_MONO_PWR	,	0x019C0192},//
	{FM_REG_AGC_CTRL	,		0x09040120},
	{FM_REG_AGC_ITV_TH	,	0x00FE00EE},
	{FM_REG_AGC_ADPS0	,	0xFDF70009},
	{FM_REG_AGC_ADPS1	,	0xFEF80003},
	{FM_REG_PDP_TH	,		400},
	{FM_REG_PDP_DEV	,		120},
	{FM_REG_ADP_ST_CONF	,	0x200501BA},
	{FM_REG_ADP_LPF_CONF	,	3},
	{FM_REG_DEPHA_SCAL	,	5},
	{FM_REG_HW_MUTE	,		0},
	{FM_REG_SW_MUTE	,		0x01A601A1},
	//{FM_REG_UPD_CTRL	,		0x01013356},
	{FM_REG_UPD_CTRL	,		0x0D013356}, /**/
	{FM_REG_AUD_BLD0	,		0x01AD01A1},
	{FM_REG_AUD_BLD1	,		0x01A901A5},	/*1a901a5*/	//bit_31???		//??? mode assign 1
	{FM_REG_AGC_HYS	,		0x00000202},

	{FM_REG_RF_CFG_DLY	,	52000},
	{FM_REG_SPUR_FREQ0    ,      0x084E081C},
	{FM_REG_SPUR_FREQ1    ,      0x06DC063C},

#ifdef USE_RF_55
#if 1   
	{FM_REG_RF_INIT_GAIN     	,	0x0007<<5},       //ZLF
	{FM_REG_AGC_BITS_TBL0	,	0x0003<<5}, //0110 0000
	{FM_REG_AGC_BITS_TBL1	,	0x0004<<5}, //1000 0000
	{FM_REG_AGC_BITS_TBL2	,	0x0005<<5}, //1010 0000
	{FM_REG_AGC_BITS_TBL3	,	0x0006<<5}, //1100 0000
	{FM_REG_AGC_BITS_TBL4	,	0x0007<<5}, //1110 0000
	{FM_REG_AGC_BITS_TBL5	,	0x0009<<5}, // 1 0110 0000
	{FM_REG_AGC_BITS_TBL6	,	0x000b<<5}, //10110 0000 
	{FM_REG_AGC_BITS_TBL7	,	0x000d<<5}, // 1 1010 0000
	{FM_REG_AGC_BITS_TBL8	,	0x0010<<5}, //10 0000 0000

#else
	{FM_REG_RF_INIT_GAIN	,	0x0003<<5},       //ZLF
	{FM_REG_AGC_BITS_TBL0	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL1	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL2	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL3	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL4	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL5	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL6	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL7	,	0x0003<<5},
	{FM_REG_AGC_BITS_TBL8	,	0x0003<<5},

#endif

#else
    {FM_REG_RF_INIT_GAIN	,	0x0302},
    {FM_REG_AGC_BITS_TBL0	,	0x0000},
    {FM_REG_AGC_BITS_TBL1	,	0x0100},
    {FM_REG_AGC_BITS_TBL2	,	0x0300},
    {FM_REG_AGC_BITS_TBL3	,	0x0301},
    {FM_REG_AGC_BITS_TBL4	,	0x0302},
    {FM_REG_AGC_BITS_TBL5	,	0x0312},
    {FM_REG_AGC_BITS_TBL6	,	0x0322},
    {FM_REG_AGC_BITS_TBL7	,	0x0332},
    {FM_REG_AGC_BITS_TBL8	,	0x0333},
#endif
    {FM_REG_AGC_BITS_TBL9	,  	0x0000},
    {FM_REG_AGC_BITS_TBL10, 	0x0100},
    {FM_REG_AGC_BITS_TBL11, 	0x0300},
    {FM_REG_AGC_BITS_TBL12, 	0x0301},
    {FM_REG_AGC_BITS_TBL13, 	0x0302},
    {FM_REG_AGC_BITS_TBL14, 	0x0312},
    {FM_REG_AGC_BITS_TBL15, 	0x0322},
    {FM_REG_AGC_BITS_TBL16, 	0x0332},
    {FM_REG_AGC_BITS_TBL17, 	0x0333},
#ifdef USE_RF_55
	{FM_REG_AGC_VAL_TBL0	,	38},//14
	{FM_REG_AGC_VAL_TBL1	,	44},//20
	{FM_REG_AGC_VAL_TBL2	,	49},//26
	{FM_REG_AGC_VAL_TBL3	,	55},//32
	{FM_REG_AGC_VAL_TBL4	,	61},//38
	{FM_REG_AGC_VAL_TBL5	,	70},//44
	{FM_REG_AGC_VAL_TBL6	,	76},//50
	{FM_REG_AGC_VAL_TBL7	,	81},//56
	{FM_REG_AGC_VAL_TBL8	,	93},//62
#else
    {FM_REG_AGC_VAL_TBL0	,	13},
    {FM_REG_AGC_VAL_TBL1	,	21},
    {FM_REG_AGC_VAL_TBL2	,	27},
    {FM_REG_AGC_VAL_TBL3	,	35},
    {FM_REG_AGC_VAL_TBL4	,	41},
    {FM_REG_AGC_VAL_TBL5	,	47},
    {FM_REG_AGC_VAL_TBL6	,	53},
    {FM_REG_AGC_VAL_TBL7	,	59},
    {FM_REG_AGC_VAL_TBL8	,	65},
#endif

	{FM_REG_AGC_BOND_TBL0,	-115UL},
	{FM_REG_AGC_BOND_TBL1,	-93UL},
	{FM_REG_AGC_BOND_TBL2,	-81UL},
	{FM_REG_AGC_BOND_TBL3,	-76UL},
	{FM_REG_AGC_BOND_TBL4,	-70UL},
	{FM_REG_AGC_BOND_TBL5,	-61UL},
	{FM_REG_AGC_BOND_TBL6,	-55UL},
	{FM_REG_AGC_BOND_TBL7,	-49UL},
	{FM_REG_AGC_BOND_TBL8,	-44UL},
	{FM_REG_AGC_BOND_TBL9,	-38UL}
};

int trout_fm_open_clock(void)
{
	unsigned int reg_data;
	unsigned int com_reg = 0x200E;
	unsigned int com_data = 0x4E4D4143;	//"NMAC"
	
	/* open clock*/
	READ_REG(SYS_REG_CLK_CTRL1, &reg_data);

	if(!(reg_data & BIT_8))
	{
		reg_data |= BIT_8;
		WRITE_REG(SYS_REG_CLK_CTRL1, reg_data);
	}

	READ_REG(com_reg, &reg_data);
	TROUT_PRINT("ID: 0x%x", reg_data);
	if(reg_data != com_data)
	{
		TROUT_PRINT("Read ID error! It should be 0x%x", com_data);
		return -1;
	}

	return 0;
}

int trout_fm_reg_cfg(void)
{
	u8 i = 0;
	u32 fm_reg_agc_db_tbl[FM_REG_AGC_DB_TBL_CNT] =
	{
		15848,	12589, 		10000, 	7943, 	6309,
		5011, 	3981,  		3162, 	2511,  	1995,
		1584, 	1258, 		1000,  	794,   	630,
		501,	398, 		316,  	251,  	199,
		158, 	125, 		100, 	79,  	63,
		50,  	39,  		31,  	25,   	19,
		15,  	12,  		10,  	7,   	6,
		5,   	3,   		3,   	2,   	1, 		1
	};

	if (write_fm_regs(fm_reg_init_des, 
				sizeof(fm_reg_init_des)/sizeof(fm_reg_init_des[0])) < 0)
		return -1;
	
	WRITE_REG(FM_REG_AGC_TBL_CLK, 0x00000001);
	for (i = 0; i < FM_REG_AGC_DB_TBL_CNT; i++)
	{
		WRITE_REG(FM_REG_AGC_DB_TBL_BEGIN+i,
				fm_reg_agc_db_tbl[i]);
	}
	
	WRITE_REG(FM_REG_AGC_TBL_CLK, 0x00000000);
	
	return 0;
}

int trout_fm_rf_write(u32 addr, u32 write_data)
{
	u32 read_data;

	if (trout_fm_rf_spi_write(addr, write_data) == -1)
	{
		TROUT_PRINT("%02x write error!\n", addr);
		return -1;
	}

	if(addr < 0x0200)
	{
		if (trout_fm_rf_spi_read(addr, &read_data) == -1)
		{
			TROUT_PRINT("%02x read error!\n", addr);
			return -1;
		}
		if (write_data == read_data)
			return 0;

		TROUT_PRINT("trout_fm_rf_write failed! addr(0x%x): r(0x%x) != w(0x%x)",
			addr, read_data, write_data);

		return -1;
	}
	
	return 0;
}

int trout_fm_rf_read(u32 addr, u32 *data)
{
	if (trout_fm_rf_spi_read(addr, data) == -1)
	{
		TROUT_PRINT("%02x read error!\n", addr);
		return -1;
	}

	return 0;
}

int trout_fm_iis_pin_cfg(void)
{//bit2,bit1,bit0 = 001 iis mode set
	unsigned int reg_data;

	READ_REG(SYS_REG_PAD_IISDO, &reg_data);
	reg_data |= 0x1;
	WRITE_REG(SYS_REG_PAD_IISDO, reg_data);

	READ_REG(SYS_REG_PAD_IISDI, &reg_data);
	reg_data |= 0x1;
	WRITE_REG(SYS_REG_PAD_IISDI, reg_data);

	READ_REG(SYS_REG_PAD_IISCLK, &reg_data);
	reg_data |= 0x1;
	WRITE_REG(SYS_REG_PAD_IISCLK, reg_data);

	READ_REG(SYS_REG_PAD_IISLRCK, &reg_data);
	reg_data |= 0x1;
	WRITE_REG(SYS_REG_PAD_IISLRCK, reg_data);

	return 0;
}

//chenq add a ioctl cmd for deal i2s work,in songkun mail,2013-01-17
int trout_fm_pcm_pin_cfg(void)
{//bit2,bit1,bit0 = 000 pcm mode set
	unsigned int reg_data;

	READ_REG(SYS_REG_PAD_IISDO, &reg_data);
	reg_data = (reg_data & 0xfffffff8);
	WRITE_REG(SYS_REG_PAD_IISDO, reg_data);

	READ_REG(SYS_REG_PAD_IISDI, &reg_data);
	reg_data = (reg_data & 0xfffffff8);
	WRITE_REG(SYS_REG_PAD_IISDI, reg_data);

	READ_REG(SYS_REG_PAD_IISCLK, &reg_data);
	reg_data = (reg_data & 0xfffffff8);
	WRITE_REG(SYS_REG_PAD_IISCLK, reg_data);

	READ_REG(SYS_REG_PAD_IISLRCK, &reg_data);
	reg_data = (reg_data & 0xfffffff8);
	WRITE_REG(SYS_REG_PAD_IISLRCK, reg_data);

	return 0;
}

int trout_rf_spi_mode(void)
{
	WRITE_REG(COM_REG_RF_SPI_CTRL, 0xd5);

	return 0;
}

int trout_rf_adi_mode(void)
{
#if 0
	u32 reg_data;

	READ_REG(COM_REG_RF_SPI_CTRL, &reg_data);

	if(reg_data & BIT_6)
	{
		reg_data &= ~BIT_6;
		
		WRITE_REG(COM_REG_RF_SPI_CTRL, reg_data);
	}
#endif
	WRITE_REG(COM_REG_RF_SPI_CTRL, 0x13);
	
	return 0;
}

