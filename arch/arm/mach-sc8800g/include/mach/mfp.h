/*
 * arch/arm/mach-sprd/include/mach/pm.h
 *
 * Pin Map Definitions
 *
 * Copyright (C) 2010 Spreadtrum International Ltd.
 *
 * 2010-03-05: yingchun li <yingchun.li@spreadtrum.com>
 *            initial version
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MFP_H
#define __ASM_ARCH_MFP_H

/*
	This is identical to the order of pin's register map.
	NOTE: if the pin register is not countinus, you should
	intest a RESEVERED pin between them;
*/
enum {
	MFP_PIN_INVALID = -1,

	MFP_PIN_SIMCLK0 = 0,
	MFP_PIN_SIMDA0,
	MFP_PIN_SIMRST0,
	MFP_PIN_SIMCLK1,
	MFP_PIN_SIMDA1,
	MFP_PIN_SIMRST1,
	MFP_PIN_SD0_CLK,
	MFP_PIN_SD_CMD,
	MFP_PIN_SD_D0,
	MFP_PIN_SD_D1,
	MFP_PIN_SD_D2,
	MFP_PIN_SD_D3,
	MFP_PIN_SD1_CLK,
	MFP_PIN_KEYOUT0,
	MFP_PIN_KEYOUT1,
	MFP_PIN_KEYOUT2,
	MFP_PIN_KEYOUT3,
	MFP_PIN_KEYOUT4,
	MFP_PIN_KEYOUT5,
	MFP_PIN_KEYOUT6,
	MFP_PIN_KEYOUT7,
	MFP_PIN_KEYIN0,
	MFP_PIN_KEYIN1,
	MFP_PIN_KEYIN2,
	MFP_PIN_KEYIN3,
	MFP_PIN_KEYIN4,
	MFP_PIN_KEYIN5,
	MFP_PIN_KEYIN6,
	MFP_PIN_KEYIN7,
	MFP_PIN_SPI_DI,
	MFP_PIN_SPI_CLK,
	MFP_PIN_SPI_DO,
	MFP_PIN_SPI_CSN0,
	MFP_PIN_SPI_CSN1,
	MFP_PIN_MTDO,
	MFP_PIN_MTDI,
	MFP_PIN_MTCK,
	MFP_PIN_MTMS,
	MFP_PIN_MTRST_N,
	MFP_PIN_U0TXD,
	MFP_PIN_U0RXD,
	MFP_PIN_U0CTS,
	MFP_PIN_U0RTS,
	MFP_PIN_U1TXD,
	MFP_PIN_U1RXD,
	MFP_PIN_NFWPN,
	MFP_PIN_NFRB,
	MFP_PIN_NFCLE,
	MFP_PIN_NFALE,
	MFP_PIN_NFCEN,
	MFP_PIN_NFWEN,
	MFP_PIN_NFREN,
	MFP_PIN_NFD0,
	MFP_PIN_NFD1,
	MFP_PIN_NFD2,
	MFP_PIN_NFD3,
	MFP_PIN_NFD4,
	MFP_PIN_NFD5,
	MFP_PIN_NFD6,
	MFP_PIN_NFD7,
	MFP_PIN_NFD8,
	MFP_PIN_NFD9,
	MFP_PIN_NFD10,
	MFP_PIN_NFD11,
	MFP_PIN_NFD12,
	MFP_PIN_NFD13,
	MFP_PIN_NFD14,
	MFP_PIN_NFD15,
	MFP_PIN_EMRST_N,
	MFP_PIN_EMA0,
	MFP_PIN_EMA1,
	MFP_PIN_EMA2,
	MFP_PIN_EMA3,
	MFP_PIN_EMA4,
	MFP_PIN_EMA5,
	MFP_PIN_EMA6,
	MFP_PIN_EMA7,
	MFP_PIN_EMA8,
	MFP_PIN_EMA9,
	MFP_PIN_EMA10,
	MFP_PIN_EMA11,
	MFP_PIN_EMA12,
	MFP_PIN_EMA13,
	MFP_PIN_EMCKE1,
	MFP_PIN_EMD0,
	MFP_PIN_EMD1,
	MFP_PIN_EMD2,
	MFP_PIN_EMD3,
	MFP_PIN_EMD4,
	MFP_PIN_EMD5,
	MFP_PIN_EMD6,
	MFP_PIN_EMD7,
	MFP_PIN_EMDQM0,
	MFP_PIN_EMDQS0,
	MFP_PIN_EMD8,
	MFP_PIN_EMD9,
	MFP_PIN_EMD10,
	MFP_PIN_EMD11,
	MFP_PIN_EMD12,
	MFP_PIN_EMD13,
	MFP_PIN_EMD14,
	MFP_PIN_EMD15,
	MFP_PIN_EMDQM1,
	MFP_PIN_EMDQS1,
	MFP_PIN_EMD16,
	MFP_PIN_EMD17,
	MFP_PIN_EMD18,
	MFP_PIN_EMD19,
	MFP_PIN_EMD20,
	MFP_PIN_EMD21,
	MFP_PIN_EMD22,
	MFP_PIN_EMD23,
	MFP_PIN_EMDQM2,
	MFP_PIN_EMDQS2,
	MFP_PIN_EMD24,
	MFP_PIN_EMD25,
	MFP_PIN_EMD26,
	MFP_PIN_EMD27,
	MFP_PIN_EMD28,
	MFP_PIN_EMD29,
	MFP_PIN_EMD30,
	MFP_PIN_EMD31,
	MFP_PIN_EMDQM3,
	MFP_PIN_EMDQS3,
	MFP_PIN_CLKDPMEM,
	MFP_PIN_CLKDMMEM,
	MFP_PIN_EMRAS_N,
	MFP_PIN_EMCAS_N,
	MFP_PIN_EMWE_N,
	MFP_PIN_EMCS_N0,
	MFP_PIN_EMCS_N1,
	MFP_PIN_EMCS_N2,
	MFP_PIN_EMCS_N3,
	MFP_PIN_EMBA0,
	MFP_PIN_EMBA1,
	MFP_PIN_EMCKE0,
	MFP_PIN_LCD_CSN1,
	MFP_PIN_LCD_RSTN,
	MFP_PIN_LCD_CD,
	MFP_PIN_LCD_D0,
	MFP_PIN_LCD_D1,
	MFP_PIN_LCD_D2,
	MFP_PIN_LCD_D3,
	MFP_PIN_LCD_D4,
	MFP_PIN_LCD_D5,
	MFP_PIN_LCD_D6,
	MFP_PIN_LCD_D7,
	MFP_PIN_LCD_D8,
	MFP_PIN_LCD_WRN,
	MFP_PIN_LCD_RDN,
	MFP_PIN_LCD_CSN0,
	MFP_PIN_LCD_D9,
	MFP_PIN_LCD_D10,
	MFP_PIN_LCD_D11,
	MFP_PIN_LCD_D12,
	MFP_PIN_LCD_D13,
	MFP_PIN_LCD_D14,
	MFP_PIN_LCD_D15,
	MFP_PIN_LCD_D16,
	MFP_PIN_LCD_D17,
	MFP_PIN_LCD_FMARK,
	MFP_PIN_CCIRMCLK,
	MFP_PIN_CCIRCK,
	MFP_PIN_CCIRHS,
	MFP_PIN_CCIRVS,
	MFP_PIN_CCIRD0,
	MFP_PIN_CCIRD1,
	MFP_PIN_CCIRD2,
	MFP_PIN_CCIRD3,
	MFP_PIN_CCIRD4,
	MFP_PIN_CCIRD5,
	MFP_PIN_CCIRD6,
	MFP_PIN_CCIRD7,
	MFP_PIN_CCIRRST,
	MFP_PIN_CCIRPD1,
	MFP_PIN_CCIRPD0,
	MFP_PIN_SCL,
	MFP_PIN_SDA,
	MFP_PIN_CLK_AUX0,
	MFP_PIN_IISDI,
	MFP_PIN_IISDO,
	MFP_PIN_IISCLK,
	MFP_PIN_IISLRCK,
	MFP_PIN_IISMCK,
	MFP_PIN_RFSDA0,
	MFP_PIN_RFSCK0,
	MFP_PIN_RFSEN0,
	MFP_PIN_RFCTL0,
	MFP_PIN_RFCTL1,
	MFP_PIN_RFCTL2,
	MFP_PIN_RFCTL3,
	MFP_PIN_RFCTL4,
	MFP_PIN_RFCTL5,
	MFP_PIN_RFCTL6,
	MFP_PIN_RFCTL7,
	MFP_PIN_RFCTL8,
	MFP_PIN_RFCTL9,
	MFP_PIN_RFCTL10,
	MFP_PIN_RFCTL11,
	MFP_PIN_RFCTL12,
	MFP_PIN_RFCTL13,
	MFP_PIN_RFCTL14,
	MFP_PIN_RFCTL15,
	MFP_PIN_XTL_EN,
	MFP_PIN_PTEST,
	MFP_PIN_GPIO135,
	MFP_PIN_GPIO136,
	MFP_PIN_GPIO137,
	MFP_PIN_GPIO138,
	MFP_PIN_GPIO139,
	MFP_PIN_GPIO140,
	MFP_PIN_OPTION2,
	MFP_PIN_OPTION3,
	MFP_PIN_GPIO141,
	MFP_PIN_GPIO142,
	MFP_PIN_GPIO143,
	MFP_PIN_GPIO144,

/*----------Analog Die Pin Control Register----------*/
	MFP_ANA_PIN_START,
	MFP_ANA_PIN_CHIP_RSTN = MFP_ANA_PIN_START,
	MFP_PIN_RESERVE1,
	MFP_ANA_PIN_PBINT,
	MFP_ANA_PIN_TP_XL,
	MFP_ANA_PIN_TP_XR,
	MFP_ANA_PIN_TP_YU,
	MFP_ANA_PIN_TP_YD,


	/*
	.....
	*/
	MFP_PIN_MAX
};


#define MFP_PIN(x)	(((x) & 0xffff) << 16)
#define MFP_CFG_TO_PIN(x)  ((x) >> 16)

//special bit for setting, for the default value of is not same
//with all registers
#define MFP_IO_SET				(0x1 << 15)
#define MFP_S_PULL_SET 	(0x1 << 14)
#define MFP_AF_SET   			(0x1 << 13)
#define MFP_F_PULL_SET 		(0x1 << 12)
#define MFP_DS_SET 				(0x1 << 11)

/* Pinmap ctrl register Bit field value
--------------------------------------------------------------------------------------------------------------------------
|                 |                 |            |            |              |       |       |            |              |
| Reserved[31:10] | Drv str sel[9:8]| func PU[7] | func PD[6] | func sel[5:4]| PU[3] | PD[2] | input En[1]| output En[0] |
|                 |                 |            |            |              |       |       |            |              |
--------------------------------------------------------------------------------------------------------------------------
*/

/*
pin output/input enable.
NOTE, this is not applied to GPIO pins, GPIO pin's input/output direction have specific
	registers and specfic bits.
	BIT 0, 1
*/
#define MFP_IO_NONE  (0x0 << 0)
#define MFP_IO_Z		MFP_IO_NONE
#define MFP_IO_OE		(0x1	<< 0)
#define MFP_IO_IE		(0x2 << 0)
#define MFP_IO_BOTH   (0x3 << 0)
#define MFP_IO_MASK  MFP_IO_BOTH

/*
	pin weak pull up/down in sleep mode
	BIT 2, 3
*/
#define MFP_S_PULL_NONE	(0x0  << 2)
#define MFP_S_PULL_DOWN   	(0x1 << 2)
#define MFP_S_PULL_UP		(0x2  << 2)
#define MFP_S_PULL_BOTH	(0x3  << 2)
#define MFP_S_PULL_MASK 	MFP_S_PULL_BOTH

/*
	pin alternate function
	BIT 4, 5
*/
#define MFP_AF0			(0x0 << 4)
#define MFP_AF1			(0x1 << 4)
#define MFP_AF2			(0x2 << 4)
#define MFP_AF3			(0x3 << 4)
#define MFP_AF_MASK		(0x3 << 4)
#define MFP_GPIO  MFP_AF3

/*
	pin weak pull up/down in function mode
	BIT 6, 7
*/
#define MFP_F_PULL_NONE	(0x0  << 6)
#define MFP_F_PULL_DOWN   	(0x1 << 6)
#define MFP_F_PULL_UP		(0x2  << 6)
#define MFP_F_PULL_BOTH	(0x3  << 6)
#define MFP_F_PULL_MASK 	MFP_F_PULL_BOTH

/*
	pin driver strenth
	BIT 8, 9
*/

#define MFP_DS0		(0x0 << 8)
#define MFP_DS1		(0x1 << 8)
#define MFP_DS2		(0x2 << 9)
#define MFP_DS3		(0x3 << 9)
#define MFP_DS_MASK	(0x3 << 9)


#define MFP_CFG(pin, af)		\
	(MFP_AF_SET |\
	 (MFP_PIN(MFP_PIN_##pin) | MFP_##af))

#define MFP_CFG_DRV(pin, af, drv)	\
	((MFP_AF_SET |MFP_DS_SET) |\
	 (MFP_PIN(MFP_PIN_##pin) | MFP_##af | MFP_##drv))

#define MFP_CFG_SLEEP_UPDOWN(pin, af, updown)	\
	((MFP_AF_SET | MFP_S_PULL_SET) |\
	 (MFP_PIN(MFP_PIN_##pin) | MFP_##af | MFP_##updown))

#define MFP_CFG_IOE(pin, af, io)	\
	((MFP_AF_SET |MFP_IO_SET)  |\
	 (MFP_PIN(MFP_PIN_##pin) | MFP_##af | MFP_##io))

#define MFP_SET_ALL	\
	(MFP_AF_SET |MFP_IO_SET | MFP_S_PULL_SET | MFP_DS_SET | \
	MFP_F_PULL_SET)

#define MFP_CFG_X(pin, af, drv, func_updown, sleep_updown, io)	\
	(MFP_SET_ALL |\
	 (MFP_PIN(MFP_PIN_##pin) | MFP_##af | MFP_##drv |\
	 MFP_##func_updown  | MFP_##sleep_updown| MFP_##io))

#define MFP_ANA_CFG_X(pin, drv, func_updown, af, sleep_updown, io)	\
	(MFP_SET_ALL |\
	 (MFP_PIN(MFP_ANA_PIN_##pin) |  MFP_##af | MFP_##drv |\
	 MFP_##func_updown | MFP_##sleep_updown| MFP_##io))

extern unsigned long mfp_to_gpio(int pin);
extern void sprd_mfp_config(unsigned long *mfp_cfgs, int num);
#endif

