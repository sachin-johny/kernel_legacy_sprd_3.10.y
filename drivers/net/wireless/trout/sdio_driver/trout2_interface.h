/*
 * Trout2 public interface for SpreadTrum
 *
 * Copyright (C) 2013 SpreadTrum Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "mxd_castor_sdk.h" /* RF modules */
#include "bt_ram_code.h"
#include "trout2_sys_reg.h"
#include "trout2_com_reg.h"
/*====add for pinmax func begin===*/
#include <mach/globalregs.h>
#include <mach/hardware.h>
#include <mach/pinmap.h>
#include <linux/regulator/consumer.h>
/*====add for pinmax func end====*/

#define REG32(x)  (*((volatile unsigned long *)(x)))
#define GR_PLL_SRC              (SPRD_GREG_BASE + 0x70)

#if 1
#define BIT_0	0x1
#define BIT_1	0x2
#define BIT_2	0x4
#define BIT_3	0x8
#define BIT_4	0x10
#define BIT_5	0x20
#define BIT_6	0x40
#define BIT_7	0x80
#define BIT_8	0x100
#define BIT_9	0x200
#define BIT_10	0x400
#define BIT_11	0x800
#define BIT_12	0x1000
#define BIT_13	0x2000
#define BIT_14	0x4000
#define BIT_15	0x8000
#define BIT_16	0x10000
#define BIT_17	0x20000
#define BIT_18	0x40000
#define BIT_19	0x80000
#define BIT_20	0x100000
#define BIT_21	0x200000
#define BIT_22	0x400000
#define BIT_23	0x800000
#define BIT_24	0x1000000
#define BIT_25	0x2000000
#define BIT_26	0x4000000
#define BIT_27	0x8000000
#define BIT_28	0x10000000
#define BIT_29	0x20000000
#define BIT_30	0x40000000
#define BIT_31	0x80000000

//SC7710 does not have LDO_xxx macros in public headers
#define	LDO_REG_BASE		( SPRD_MISC_BASE+ 0x800	)

#define	ANA_LDO_PD_SET		( LDO_REG_BASE	+ 0x28 )
#define	ANA_LDO_PD_RST		( LDO_REG_BASE	+ 0x2c )
#define	ANA_LDO_PD_CTL0		( LDO_REG_BASE	+ 0x30 )
#define	ANA_LDO_PD_CTL1		( LDO_REG_BASE	+ 0x34 )
#define	ANA_LDO_VCTL0		( LDO_REG_BASE	+ 0x38 )
#define	ANA_LDO_SLP0		( LDO_REG_BASE	+ 0x50 )
//SC7710 does not have ANA_xxx macros in public headers
#ifndef ANA_REG_AND
#define ANA_REG_AND(reg_addr, value)	do{\
		unsigned long flags; \
		unsigned short adi_tmp_val;\
		local_irq_save(flags);\
		adi_tmp_val = sci_adi_read(reg_addr); \
		adi_tmp_val = sci_adi_read(reg_addr); \
		adi_tmp_val &= (unsigned short)(value); \
		sci_adi_raw_write(reg_addr, adi_tmp_val); \
		local_irq_restore(flags);\
}while(0)
#endif

#ifndef	ANA_REG_OR
#define	ANA_REG_OR(_r, _b)	sci_adi_write(_r, _b, 0)
#endif

#ifndef	ANA_REG_BIC
#define	ANA_REG_BIC(_r, _b)	sci_adi_write(_r, 0, _b)
#endif

#ifndef	ANA_REG_GET
#define	ANA_REG_GET(_r)		sci_adi_read(_r)
#endif

#ifndef	ANA_REG_SET
#define	ANA_REG_SET(_r, _v)	sci_adi_raw_write(_r, _v) 
#endif


#endif /*KERNEL 3.X*/
#ifndef BIT_0
#define	BIT_0		0x00000001
#endif
#ifndef BIT_25
#define	BIT_25		0x02000000
#endif

/*  */
#define TROUT_PIN_DS_0                0  
#define TROUT_PIN_DS_1                (BIT_5)
#define TROUT_PIN_DS_2                (BIT_6)
#define TROUT_PIN_DS_3                (BIT_6|BIT_5)
#define TROUT_PIN_DS_MSK	      TROUT_PIN_DS_3

#define TROUT_IIS_DS_0          BIT_0
#define TROUT_IIS_DS_1          BIT_1
#define TROUT_IIS_DS_2          BIT_2

//TROUT CLCK CONTROL
#define TROUT_WIFI_CLK_DIV_EN BIT_2
#define TROUT_BT_CLK_RTC_FORCE_EN BIT_3
#define TROUT_BT_CLK24M_FORCE_EN BIT_4
#define TROUT_FM_CLK_8          BIT_8
#define TROUT_FM_SLEEP          BIT_10
#define TROUT_SPI_XTLEN_AUTOPD_EN BIT_13
#define TROUT_WIFI_SLEEP_CLK_SEL_32K BIT_16

#define TROUT_CLK_CONFG_DEFAUT   (TROUT_BT_CLK_RTC_FORCE_EN| \
 								 TROUT_WIFI_CLK_DIV_EN|\
 								 TROUT_BT_CLK24M_FORCE_EN|\
 								 TROUT_FM_SLEEP|\
 								 TROUT_SPI_XTLEN_AUTOPD_EN| \
 								 TROUT_WIFI_SLEEP_CLK_SEL_32K) 
//end the configuate clck
#define TROUT_CONFIG_CLK_CTRL0  0x02E0B85E

#define TROUT_WIFI_RF_SEL  BIT_0

#define TROUT_RF_ADI_EN    0x13

#define ENABLE_DSP_LOG       (0x4b2)

#define PA_DS_0          BIT_0
#define PA_DS_25         BIT_25
#define BT_CLK_DS_4      BIT_4
#define BT_CLK_DS_7      BIT_7
//#define SYS_REG_PAD_SD_CMD      0xE
//#define SYS_REG_PAD_SD_D0 	    0xF
//#define SYS_REG_PAD_SD_D1 	    0x10
//#define SYS_REG_PAD_SD_D2 	    0x11
//#define SYS_REG_PAD_SD_D3 	    0x12

/* Wireless module Power state */
#define POWER_OFF			0

//#define TROUT_BOARD_EVB  // for EVB test power on/off
#define TROUT_BOARD_MOBILE

//zhouxw power test
#define rMAC_PA_CON                  0x00008004
#define TROUT_SYS_REG_BASE         0
#define REGBIT25  (1<<25)
#define REGBIT0   (1<<0)
#define M_CLK_CTRL1 0x44
#define M_POW_CTRL  0x45
//end
#define WIFI_OPEN_FLAG  1
#define WIFI_CLOSE_FLAG 0
#define PCM_MODE        0
#define IIS_MODE        1
#define SDIO_MODE       0
#define SPI_MODE        1
#define PMU_PD_INT_AUTO BIT_8
#define LDO_ANA_PD_AUTO BIT_10
#define WIFI_ADC_FORCE_EN BIT_1
#define BW_DAC_FORCE_EN   BIT_2
//baike xiong modified for sc7710
#ifdef	CONFIG_ARCH_SC7710
#define PIN_SD1_D3      83
#define TROUT2_RESET   151
#define TROUT2_PDN     148
#define WIFI_XLEN		9
#elif defined(CONFIG_ARCH_SC8825)
#define PIN_SD1_D3	99
#define TROUT2_RESET	190
#define TROUT2_PDN	210
#else
#define PIN_SD2_D3      24
#define TROUT2_RESET   140
#define TROUT2_PDN     137
#define PIN_SD1_D3 PIN_SD2_D3
#endif

typedef enum {
	NONE_MODE    = 0,
	WIFI_MODE    = 1,
	FM_MODE      = 2,
	BT_MODE      = 4,	
} MODE;

typedef enum {
	RF_DISABLE    = 0,
	RF_ENABLE    = 1,
} RF_STATUS;

#if 1
#define SPRD_GPIO_REG_WRITEL(val, reg)    do{__raw_writel(val, CTL_PIN_BASE + (reg));}while(0)
static unsigned long CEQ_close_cfg = (BITS_PIN_DS(1) | BITS_PIN_AF(3) | BIT_PIN_WPD | BIT_PIN_SLP_WPD | BIT_PIN_SLP_OE);
static unsigned long CEQ_open_cfg = (BITS_PIN_DS(1) | BITS_PIN_AF(2) | BIT_PIN_WPD | BIT_PIN_SLP_WPD | BIT_PIN_SLP_IE);
#else
static unsigned long CEQ_close_cfg[] = 
{                                                                                  
		MFP_CFG_X(IISMCK, AF1, DS1,F_PULL_DOWN,S_PULL_DOWN,IO_IE),
};
static unsigned long CEQ_open_cfg[] = 
{                                                                                 
		MFP_CFG_X(IISMCK, AF2, DS1,F_PULL_DOWN,S_PULL_DOWN,IO_IE),
};
#endif /*KERNEL 3.X*/

#ifdef TROUT_WIFI_POWER_SLEEP_ENABLE 
extern unsigned int current_rf_mode;
typedef enum
{
	WIFI_NORMAL_POWER_MODE = 0,
	WIFI_LOW_POWER_MODE    = 1,
}WIFI_POWER_MODE;
#endif


/*  */
void interface_init(void);

/* Public interface */
void Set_trout_pcm_iis_mode(bool mode);
unsigned int Get_trout_comm_int_stat(void);
void Set_trout_comm_int_clear(unsigned int bit);
void Set_trout_comm_int_mask(unsigned int bit);
bool Set_Trout_RF_Start(unsigned int mode);
bool Set_Trout_RF_Stop(unsigned int mode);
bool Set_Trout_Download_BT_Code(void);
bool Set_Trout_PowerOff(unsigned int MODE_ID);
bool Set_Trout_PowerOn( unsigned int  MODE_ID );
void Set_Trout2_Open(void);
void Trout_Print_Sys_Common_Reg(void);
void castorLcPllctrl(int on);
void get_phy_mutex(void);
void put_phy_mutex(void);
int GetTxPowerLevel(unsigned int* gnTxPower,
						unsigned int* bTxPower);
int get_tx_pwr_from_nv(unsigned int *b_tx_pwr, unsigned int *g_tx_pwr, unsigned int *n_tx_pwr, unsigned int* delta_0x57);



