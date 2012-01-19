/*
  *     Deep sleep testing mode.
  *
  *     Download small piece of code into DSP and make it sleep for ever.
  *
  */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include <mach/regs_global.h>
#include <mach/regs_ahb.h>
#include <mach/regs_ana.h>
#include <mach/adi_hal_internal.h>
#include <mach/regs_emc.h>
#include <mach/regs_int.h>
#include <mach/regs_cpc.h>
#include <mach/regs_adi.h>

/* Pinmap ctrl register Bit field value
---------------------------------------------------------------------------------------------------
|        |            |            |              |           |           |          |           |
| DS[9:8]| func PU[7] | func PD[6] | func sel[5:4]| Slp PU[3] | slp PD[2] | input[1] | output[0] |
|        |            |            |              |           |           |          |           |
---------------------------------------------------------------------------------------------------
*/
const PM_PINFUNC_T pm_func[] = {
//  | Pin Register        |   DS    | Func PU/PD | Func Select  | Slp PU/PD | Sleep OE/IE | 
	{PIN_CTL_REG, (0x7E7FF00)},
	{PIN_SIMCLK0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	// SIM0
	{PIN_SIMDA0_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},	//  SIM0
	{PIN_SIMRST0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	//  SIM0
	{PIN_SIMCLK1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	// SIM1
	{PIN_SIMDA1_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},	// SIM1
	{PIN_SIMRST1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	// SIM1
	{PIN_SD0_CLK_REG, (PIN_DS_3 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// SD/SDIO   
	{PIN_SD_CMD_REG, (PIN_DS_3 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  SD/SDIO
	{PIN_SD_D0_REG, (PIN_DS_3 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  SD/SDIO
	{PIN_SD_D1_REG, (PIN_DS_3 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  SD/SDIO
	{PIN_SD_D2_REG, (PIN_DS_3 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  SD/SDIO
	{PIN_SD_D3_REG, (PIN_DS_3 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  SD/SDIO
	{PIN_SD1_CLK_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  SD/SDIO
	{PIN_KEYOUT0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	//  KEY OUT
	{PIN_KEYOUT1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	//  KEY OUT
	{PIN_KEYOUT2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  NC
	{PIN_KEYOUT3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  NC
	{PIN_KEYOUT4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  NC
	{PIN_KEYOUT5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  NC
	{PIN_KEYOUT6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPU_EN | PIN_Z_EN)},	// GPS RST,ldo wifi0
	{PIN_KEYOUT7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_Z_EN)},	// GPS PWD,set to spx_o when pm ready
	{PIN_KEYIN0_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},	//    KEY IN 
	{PIN_KEYIN1_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},	//    KEY IN 
	{PIN_KEYIN2_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},	//    NC 
	{PIN_KEYIN3_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//    NC
	{PIN_KEYIN4_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//    NC
	{PIN_KEYIN5_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_3 | PIN_SPU_EN | PIN_I_EN)},	//   PROX_INT,need wakeup mcu while lcd is working  
	{PIN_KEYIN6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPX_EN | PIN_Z_EN)},	//   G_INT1
	{PIN_KEYIN7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPX_EN | PIN_Z_EN)},	//   G_INT2
	{PIN_SPI_DI_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   CMMB_SPIDO,VDDRF1  
	{PIN_SPI_CLK_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   CMMB_SPICLK  
	{PIN_SPI_DO_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   CMMB_SPIDI  
	{PIN_SPI_CSN0_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   CMMB_CSN0
	{PIN_SPI_CSN1_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_3 | PIN_SPX_EN | PIN_Z_EN)},	//   NC
	{PIN_MTDO_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   JTAG  
	{PIN_MTDI_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   JTAG  
	{PIN_MTCK_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   JTAG  
	{PIN_MTMS_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   JTAG  
	{PIN_MTRST_N_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	// 0x188  JTAG  
	{PIN_U0TXD_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},	//   BT UART RX  
	{PIN_U0RXD_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//   BT UART TX  
	{PIN_U0CTS_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//   NC 
	{PIN_U0RTS_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//   NC 
	{PIN_U1TXD_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//   U1TXD,TESTPOINT  
	{PIN_U1RXD_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//   U1RXD,TESTPOINT
	{PIN_NFWPN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    NF WP = 0
	{PIN_NFRB_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//     NF RB
	{PIN_NFCLE_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    NF CLE
	{PIN_NFALE_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    NF ALE
	{PIN_NFCEN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//   NF CE = VCC
	{PIN_NFWEN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    NF WE
	{PIN_NFREN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    NF RE
	{PIN_NFD0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD8_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD9_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD10_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD11_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD12_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD13_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD14_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_NFD15_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     NF DATA
	{PIN_EMRST_N_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//   EMRST,NC
	{PIN_EMA0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA8_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA9_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA10_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA11_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA12_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMA13_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMCKE1_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	//  
	{PIN_EMD0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMD1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMD2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMD3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMD4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMD5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMD6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMD7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMDQM0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMDQS0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD8_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD9_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD10_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD11_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD12_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD13_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD14_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD15_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMDQM1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMDQS1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD16_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD17_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD18_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD19_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD20_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD21_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD22_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD23_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMDQM2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMDQS2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD24_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD25_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD26_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD27_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD28_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD29_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD30_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMD31_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMDQM3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMDQS3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_CLKDPMEM_REG, (PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_CLKDMMEM_REG, (PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMRAS_N_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    RAS = 0 
	{PIN_EMCAS_N_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    CAS = 0
	{PIN_EMWE_N_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//   WE  = 1
	{PIN_EMCS_N0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    CS = 0
	{PIN_EMCS_N1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     NC
	{PIN_EMGPRE_LOOP_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMGPST_LOOP_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    
	{PIN_EMBA0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMBA1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//     
	{PIN_EMCKE0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	//    CKE  = 0
	{PIN_LCD_CSN1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  NC 
	{PIN_LCD_RSTN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//    
	{PIN_LCD_CD_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_D8_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    
	{PIN_LCD_WRN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//     
	{PIN_LCD_RDN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//     
	{PIN_LCD_CSN0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D9_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D10_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D11_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D12_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D13_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D14_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D15_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//     
	{PIN_LCD_D16_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	// 
	{PIN_LCD_D17_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	// 
	{PIN_LCD_FMARK_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//INPUT   
	{PIN_CCIRMCLK_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  CAMERA will powerdown 
	{PIN_CCIRCK_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRHS_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRVS_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRD7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA will powerdown 
	{PIN_CCIRRST_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN)},	// CAMERA_Reset
	{PIN_CCIRPD1_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//CAMERA will powerdown 
	{PIN_CCIRPD0_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//CAMERA will powerdown 
	{PIN_SCL_REG,     (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//CAM SENSOR     
	{PIN_SDA_REG,     (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//CAM SENSOR     
	{PIN_CLK_AUX0_REG,(PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	//GPS_32K
	{PIN_IISDI_REG,   (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  BT PCM_IN,BT MASTER
	{PIN_IISDO_REG,   (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//  BT PCM_OUT
	{PIN_IISCLK_REG,  (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  BT PCM_CLK,INPUT 
	{PIN_IISLRCK_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//BT PCM_SYNC,INPUT
	{PIN_IISMCK_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_2    | PIN_SPD_EN | PIN_I_EN)},	//    BTXTLEN = 0 
	{PIN_RFSDA0_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF  | PIN_SPD_EN | PIN_Z_EN)},	//  QS3200 SPI SDA 
	{PIN_RFSCK0_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF  | PIN_SPD_EN | PIN_Z_EN)},	//  QS3200 SPI SCK 
	{PIN_RFSEN0_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF  | PIN_SPU_EN | PIN_Z_EN)},	//  QS3200 SPI LE 
	{PIN_RFCTL0_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_3    | PIN_SPX_EN | PIN_O_EN)},	//    GPIO90 BT RSTN /////PULL DOWN FOR TEST ONLY , BT DRAIN 4mA FROM VBAT
	{PIN_RFCTL1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPD_EN | PIN_Z_EN)},	//  TD PA MD1
	{PIN_RFCTL2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPD_EN | PIN_Z_EN)},	//  GSM PA MODE 
	{PIN_RFCTL3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPX_EN | PIN_Z_EN)},	//  NC
	{PIN_RFCTL4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPX_EN | PIN_Z_EN)},	//  NC
	{PIN_RFCTL5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPD_EN | PIN_Z_EN)},	//  TD PA EN
	{PIN_RFCTL6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3    | PIN_SPX_EN | PIN_Z_EN)},	//    MAG_INT
	{PIN_RFCTL7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3    | PIN_SPX_EN | PIN_Z_EN)},	//    MSENSOR_DRDY
	{PIN_RFCTL8_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPD_EN | PIN_Z_EN)},	//    TD PA VMODE
	{PIN_RFCTL9_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPX_EN | PIN_Z_EN)},	//    NC
	{PIN_RFCTL10_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//    GSM PA TXEN
	{PIN_RFCTL11_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//    NC
	{PIN_RFCTL12_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  NC 
	{PIN_RFCTL13_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//  NC
	{PIN_RFCTL14_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//  VC2
	{PIN_RFCTL15_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//  VC4  
	{PIN_XTL_EN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPX_EN | PIN_Z_EN)},	// NC   
	{PIN_PTEST_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF   | PIN_SPX_EN | PIN_Z_EN)},	//TIE TO GND    
	{PIN_GPIO135_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	  // FLASH_EN  
	{PIN_GPIO136_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC       
	{PIN_GPIO137_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN)},	// WIFI_1.8V_ON  ??         
	{PIN_GPIO138_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	// CMMB_RSTN               
	{PIN_GPIO139_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// CMMB_INT,don't need wakeup on 8800g,what about 8810?                   
	{PIN_GPIO140_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	// WIFI_RSTN    ??                   
	{PIN_GPIO141_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_I_EN)},	// HP_DET,EXT PULLUP                           
	{PIN_GPIO142_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// WIFI_WAKEUP(NC)
	{PIN_GPIO143_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_2   | PIN_SPX_EN | PIN_O_EN)},	// LCM_BL_EN,PWM                                   
	{PIN_GPIO144_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	// FLASH_LED,GPIO                                       
	{PIN_SD2_CLK_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//WIFI_SDIO,EXT PULLUP                                          
	{PIN_SD2_CMD_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//WIFI_SDIO,EXT PULLUP                                              
	{PIN_SD2_D0_REG,  (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//WIFI_SDIO,EXT PULLUP                                                  
	{PIN_SD2_D1_REG,  (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//WIFI_SDIO,EXT PULLUP                                                  
	{PIN_SD2_D2_REG,  (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//WIFI_SDIO,EXT PULLUP                                                  
	{PIN_SD2_D3_REG,  (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//WIFI_SDIO,EXT PULLUP                                                  
	{PIN_U2TXD_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// GPS UART
	{PIN_U2RXD_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// GPS UART    
	{PIN_NFCEN1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC
	{PIN_SCL0_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	// Proximity/G&MAG Sensor,vdd18,28    
	{PIN_SDA0_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN)},	// Proximity/G&MAG Sensor        
	{PIN_SCL2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC            
	{PIN_SDA2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC            
	{PIN_SCL3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC            
	{PIN_SDA3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC            
	{PIN_LCD_D18_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//                 
	{PIN_LCD_D19_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//                     
	{PIN_LCD_D20_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//                     
	{PIN_LCD_D21_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//                         
	{PIN_LCD_D22_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//                         
	{PIN_LCD_D23_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN)},	//                     
	{PIN_TRACECLK_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//                     
	{PIN_TRACECTRL_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                        
	{PIN_TRACEDAT0_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                            
	{PIN_TRACEDAT1_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                
	{PIN_TRACEDAT2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                
	{PIN_TRACEDAT3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                
	{PIN_TRACEDAT4_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                
	{PIN_TRACEDAT5_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                
	{PIN_TRACEDAT6_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                
	{PIN_TRACEDAT7_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                
	{PIN_SIMCLK2_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_1 | PIN_SPU_EN | PIN_Z_EN)},	// I2C2_SDA,CTP,LDOSIM2                                    
	{PIN_SIMDA2_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_1 | PIN_SPU_EN | PIN_Z_EN)},	// I2C2_SDL,CTP,LDOSIM2                                        
	{PIN_SIMRST2_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                            
	{PIN_SIMCLK3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_O_EN)},	// GPIO,CTP_RST                                            
	{PIN_SIMDA3_REG,  (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	// NC                                                
	{PIN_SIMRST3_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_I_EN)},	// GPIO,CTP_INT,EXT PULLUP                                                    
	{ANA_PIN_TESTRSTN_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//NC,NO USE!!
	{ANA_PIN_PBINT_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},	//0x18a    PBINT
	{ANA_PIN_TP_XL_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//0x100      NC
	{ANA_PIN_TP_XR_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//0x100      NC
	{ANA_PIN_TP_YU_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//0x100      NC
	{ANA_PIN_TP_YD_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},	//0x100      NC
	{0xffffffff, 0xffffffFF}
};

const PM_PINFUNC_T pm_default_global_map[] = {
	{0xffffffff, 0xffffffff}
};

#define ANA_IS_ANA_REG(addr) (SPRD_MISC_BASE == (addr & 0xfffff000))

/*
	functions for comparing pin config register.
 */
#define MAX_PIN_NUM 512
u32 pin_reg_val[MAX_PIN_NUM];

void sc8810_value_init(void)
{
	int i = 0;
	CHIP_REG_OR((GR_GEN0), (GEN0_PIN_EN));
	ANA_REG_OR(ANA_AGEN, (AGEN_PINREG_EN));	// enable gpio base romcode
	for (;;) {
		if (pm_func[i].addr == PM_INVALID_VAL) {
			break;
		}

		if (!ANA_IS_ANA_REG(pm_func[i].addr)) {
			pin_reg_val[i] = CHIP_REG_GET(pm_func[i].addr);
		} else {
			pin_reg_val[i] = ANA_REG_GET(pm_func[i].addr);
		}
		i++;
	}
	printk("####: sc8810_value_init() is done!\n");
}

void sc8810_value_compare(void)
{
	int i = 0;
	u32 val = 0;
	CHIP_REG_OR((GR_GEN0), (GEN0_PIN_EN));
	ANA_REG_OR(ANA_AGEN, (AGEN_PINREG_EN));	// enable gpio base romcode
	for (;;) {
		if (pm_func[i].addr == PM_INVALID_VAL) {
			break;
		}

		if (!ANA_IS_ANA_REG(pm_func[i].addr)) {
			val = CHIP_REG_GET(pm_func[i].addr);
			if (pin_reg_val[i] != val) {
				printk
				    ("##: Values are different[%08x]: want[%08x], but[%08x]!\n",
				     pm_func[i].addr, pin_reg_val[i], val);
				/*
				   CHIP_REG_SET (pm_func[i].addr ,pm_func[i].value);
				 */
			}
		} else {
			val = ANA_REG_GET(pm_func[i].addr);
			if (pin_reg_val[i] != val) {
				printk
				    ("##: Values are different[%08x]: want[%08x], but[%08x]!\n",
				     pm_func[i].addr, pin_reg_val[i], val);
				/*
				   ANA_REG_SET (pm_func[i].addr ,pm_func[i].value);
				 */
			}
		}
		i++;
	}
	/*
	   printk("####: sc8810_value_compare() is done!\n");
	 */
}

void sc8810_pin_map_init(void)
{
	int i = 0;

	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
	CHIP_REG_OR((GR_GEN0), (GEN0_PIN_EN));
	ANA_REG_OR(ANA_AGEN, (AGEN_PINREG_EN));	// enable gpio base romcode
	for (;;) {
		if (pm_func[i].addr == PM_INVALID_VAL) {
			break;
		}

		if (!ANA_IS_ANA_REG(pm_func[i].addr)) {
			CHIP_REG_SET(pm_func[i].addr, pm_func[i].value);
		}

		else {
			printk("##: write ANA register!\n");
			ANA_REG_SET(pm_func[i].addr, pm_func[i].value);
		}
		i++;
	}
	i = 0;
	for (;;) {
		if (pm_default_global_map[i].addr == PM_INVALID_VAL) {
			break;
		}

		CHIP_REG_SET(pm_default_global_map[i].addr,
			     pm_default_global_map[i].value);
		i++;
	}
	sc8810_value_init();
	printk("####: pin_map is done!\n");
}

EXPORT_SYMBOL_GPL(sc8810_pin_map_init);

/**
	functions for supsend.
**/

/* Pinmap ctrl register Bit field value
---------------------------------------------------------------------------------------------------
|        |            |            |              |           |           |          |           |
| DS[9:8]| func PU[7] | func PD[6] | func sel[5:4]| Slp PU[3] | slp PD[2] | input[1] | output[0] |
|        |            |            |              |           |           |          |           |
---------------------------------------------------------------------------------------------------
*/
const PM_PINFUNC_T pm_func_suspend[] = {
//  | Pin Register        |   DS    | Func PU/PD | Func Select  | Slp PU/PD | Sleep OE/IE | 
	{0xffffffff, 0xffffffFF}
};

const PM_PINFUNC_T pm_default_global_map_suspend[] = {
	{0xffffffff, 0xffffffff}
};

void gpio_for_suespend(void)
{
	int i = 0;
/*
	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
*/
	CHIP_REG_OR((GR_GEN0), (GEN0_PIN_EN));
	ANA_REG_OR(ANA_AGEN, (AGEN_PINREG_EN));	// enable gpio base romcode
	for (;;) {
		if (pm_func_suspend[i].addr == PM_INVALID_VAL) {
			break;
		}

		if (!ANA_IS_ANA_REG(pm_func_suspend[i].addr)) {
			CHIP_REG_SET(pm_func_suspend[i].addr,
				     pm_func_suspend[i].value);
		}

		else {
			printk("##: write ANA register!\n");
			ANA_REG_SET(pm_func_suspend[i].addr,
				    pm_func_suspend[i].value);
		}
		i++;
	}
	i = 0;
	for (;;) {
		if (pm_default_global_map_suspend[i].addr == PM_INVALID_VAL) {
			break;
		}

		CHIP_REG_SET(pm_default_global_map_suspend[i].addr,
			     pm_default_global_map_suspend[i].value);
		i++;
	}
	/* compare register's value. */
	sc8810_value_compare();

}

EXPORT_SYMBOL_GPL(gpio_for_suespend);

void gpio_dump_registers(void)
{
	int i = 0;
	u32 val = 0;
	CHIP_REG_OR((GR_GEN0), (GEN0_PIN_EN));
	ANA_REG_OR(ANA_AGEN, (AGEN_PINREG_EN));	// enable gpio base romcode
	for (;;) {
		if (pm_func[i].addr == PM_INVALID_VAL) {
			break;
		}

		if (!ANA_IS_ANA_REG(pm_func[i].addr)) {
			val = CHIP_REG_GET(pm_func[i].addr);
			printk("reg[%08x] = %08x.\n", pm_func[i].addr, val);
		} else {
			val = ANA_REG_GET(pm_func[i].addr);
			printk("reg[%08x] = %08x.\n", pm_func[i].addr, val);
		}
		i++;
	}
	printk("####: gpio_dump_registers() is done!\n");
}

EXPORT_SYMBOL_GPL(gpio_dump_registers);
