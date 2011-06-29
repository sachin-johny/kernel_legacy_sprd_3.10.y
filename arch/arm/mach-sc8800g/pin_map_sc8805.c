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
const PM_PINFUNC_T pm_func[]=
{
//  | Pin Register        |   DS    | Func PU/PD | Func Select  | Slp PU/PD | Sleep OE/IE | 
    {PIN_CTL_REG,        ( 0x1fff00) },
    {PIN_SIMCLK0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101    
    {PIN_SIMDA0_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18A    
    {PIN_SIMRST0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101    
    {PIN_SIMCLK1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101 NC   
    {PIN_SIMDA1_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18A NC   
    {PIN_SIMRST1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101 NC
    {PIN_SD0_CLK_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100 SD/SDIO   
    {PIN_SD_CMD_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D0_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D1_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D2_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D3_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD1_CLK_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100 NC
    {PIN_KEYOUT0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100    KEY OUT
    {PIN_KEYOUT1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100    KEY OUT
    {PIN_KEYOUT2_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100    KEY OUT
    {PIN_KEYOUT3_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100    KEY OUT
    {PIN_KEYOUT4_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100    KEY OUT
    {PIN_KEYOUT5_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100  NC 
    {PIN_KEYOUT6_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100  GPIO26 TEST POINT  
    {PIN_KEYOUT7_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_2   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  fm_32k   
    {PIN_KEYIN0_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180   KEY IN 
    {PIN_KEYIN1_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180   KEY IN 
    {PIN_KEYIN2_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180   KEY IN 
    {PIN_KEYIN3_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180   KEY IN 
    {PIN_KEYIN4_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180   KEY IN 
    {PIN_KEYIN5_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180  NC  
    {PIN_KEYIN6_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180  NC
    {PIN_KEYIN7_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180  NC 
    {PIN_SPI_DI_REG,     ( PIN_DS_3 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18A  CMMB WIFI SPIDO  
    {PIN_SPI_CLK_REG,    ( PIN_DS_3 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_O_EN ) },    // 0x106  SPICLK  
    {PIN_SPI_DO_REG,     ( PIN_DS_3 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_O_EN ) },    // 0x10A  CMMB WIFI SPIDI  
    {PIN_SPI_CSN0_REG,   ( PIN_DS_3 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_O_EN ) },    // 0x10A  SPI CS0: WIFI  
    {PIN_SPI_CSN1_REG,   ( PIN_DS_3 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x106  SPI CS1: CMMB  
    {PIN_MTDO_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x101  JTAG  
    {PIN_MTDI_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x18A  JTAG  
    {PIN_MTCK_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x18A  JTAG  
    {PIN_MTMS_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x18A  JTAG  
    {PIN_MTRST_N_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x146  JTAG  
    {PIN_U0TXD_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  BT UART RX  
    {PIN_U0RXD_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180  BT UART TX  
    {PIN_U0CTS_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180  GPIO41  
    {PIN_U0RTS_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  GPIO42  bb core dcdc en (NC)
    {PIN_U1TXD_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  U1TXD  
    {PIN_U1RXD_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x180  U1RXD  
    {PIN_NFWPN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   NF WP = 0
    {PIN_NFRB_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x180    NF RB
    {PIN_NFCLE_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF CLE
    {PIN_NFALE_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF ALE
    {PIN_NFCEN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x100   NF CE = VCC
    {PIN_NFWEN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF WE
    {PIN_NFREN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF RE
    {PIN_NFD0_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD1_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD2_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD3_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD4_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD5_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD6_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD7_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD8_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD9_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD10_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD11_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD12_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD13_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD14_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_NFD15_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    NF DATA
    {PIN_EMRST_N_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    EMRST(NC)
    {PIN_EMA0_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA1_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA2_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA3_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA4_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA5_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA6_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA7_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA8_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA9_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA10_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA11_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA12_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMA13_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMCKE1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD0_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD1_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD2_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD3_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD4_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD5_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD6_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD7_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQM0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQS0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD8_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD9_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD10_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD11_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD12_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD13_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD14_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD15_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQM1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQS1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD16_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD17_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD18_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD19_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD20_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD21_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD22_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD23_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQM2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQS2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD24_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD25_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD26_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD27_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD28_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD29_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD30_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMD31_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQM3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMDQS3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_CLKDPMEM_REG,   ( PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_CLKDMMEM_REG,   ( PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMRAS_N_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   RAS = 0 
    {PIN_EMCAS_N_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   CAS = 0
    {PIN_EMWE_N_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x100   WE  = 1
    {PIN_EMCS_N0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   CS = 0
    {PIN_EMCS_N1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    NC
    {PIN_EMCS_N2_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    NC
    {PIN_EMCS_N3_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    NC
    {PIN_EMBA0_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMBA1_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_EMCKE0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   CKE  = 0
    {PIN_LCD_CSN1_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  NC 
    {PIN_LCD_RSTN_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_CD_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D4_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D5_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D6_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D7_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D8_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_WRN_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_RDN_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_CSN0_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D9_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D10_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D11_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D12_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D13_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D14_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D15_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_LCD_D16_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_1   | PIN_SPU_EN | PIN_Z_EN ) },    // i2c SCL, port 2 (logic id = 1)
    {PIN_LCD_D17_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_1   | PIN_SPU_EN | PIN_Z_EN ) },    // i2c SDA
    {PIN_LCD_FMARK_REG,  ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x180    
    {PIN_CCIRMCLK_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRCK_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRHS_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRVS_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD4_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD5_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD6_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRD7_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_CCIRRST_REG ,   ( PIN_O_EN | PIN_SPX_EN | PIN_FUNC_3   | PIN_FPD_EN | PIN_DS_0) },    //  CAMERA_Reset
    {PIN_CCIRPD1_REG,    ( PIN_O_EN | PIN_SPX_EN | PIN_FUNC_3   | PIN_FPD_EN | PIN_DS_0) },    // 0x100    CAMERA PD
    {PIN_CCIRPD0_REG,    ( PIN_O_EN | PIN_SPX_EN | PIN_FUNC_3   | PIN_FPD_EN | PIN_DS_0) },    // 0x100    CAMERA PD
    {PIN_SCL_REG,        ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x180    
    {PIN_SDA_REG,        ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x180    
    {PIN_CLK_AUX0_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x106   CLKAUX0, 32KHz for FM
    {PIN_IISDI_REG,      ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x156   BT PCM_IN
    {PIN_IISDO_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x11A   BT PCM_OUT
    {PIN_IISCLK_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x15A   BT PCM_CLK 
    {PIN_IISLRCK_REG,    ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x15A   BT PCM_SYNC  
    {PIN_IISMCK_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_2   | PIN_SPD_EN | PIN_I_EN ) },    // 0x156    BTXTLEN = 0 
    {PIN_RFSDA0_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x140   QS3200 SPI SDA 
    {PIN_RFSCK0_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x140   QS3200 SPI SCK 
    {PIN_RFSEN0_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x180   QS3200 SPI LE 
    {PIN_RFCTL0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPU_EN | PIN_Z_EN ) },    // 0x100   GPIO90 BT RSTN /////PULL DOWN FOR TEST ONLY , BT DRAIN 4mA FROM VBAT
    {PIN_RFCTL1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   TD PA MD1
    {PIN_RFCTL2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   GSM PA MODE 
    {PIN_RFCTL3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPD_EN | PIN_I_EN ) },    // 0x100   GPIO93 CMMB_INT 
    {PIN_RFCTL4_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   GPIO94 CMMB_RSTN    
    {PIN_RFCTL5_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   TD PA EN
    {PIN_RFCTL6_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   SPK PA EN
    {PIN_RFCTL7_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   RF SWITCH VC3
    {PIN_RFCTL8_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   TD PA VMOD0
    {PIN_RFCTL9_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL10_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   GSM PA TXEN
    {PIN_RFCTL11_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   GPIO101 TEST POINT
    {PIN_RFCTL12_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   PA BANDSEL 
    {PIN_RFCTL13_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   RF SWITCH VC1
    {PIN_RFCTL14_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   RF SWITCH VC2 
    {PIN_RFCTL15_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x100   RF SWITCH VC4 
    {PIN_XTL_EN_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPU_EN | PIN_Z_EN ) },    // 0x101   GPIO106_WIFI/BT LOD EN
    {PIN_PTEST_REG,      ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    PULL DOWN TO GROUND EXTERNALLY

    {ANA_PIN_CHIP_RSTN_REG,(PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},     //           TEST POINT
    {ANA_PIN_PBINT_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},
    {ANA_PIN_TP_XL_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_Z_EN)},
    {ANA_PIN_TP_XR_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_Z_EN)},
    {ANA_PIN_TP_YU_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_Z_EN)},
    {ANA_PIN_TP_YD_REG, (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_Z_EN)},


    {PIN_GPIO135_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_O_EN ) },    // 0x100   CMMB Power   
    {PIN_GPIO136_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_GPIO137_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_GPIO138_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_GPIO139_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_GPIO140_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x100    
    {PIN_GPIO141_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_I_EN ) },    // 0x100    
    {PIN_GPIO142_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_GPIO143_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_GPIO144_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {0xffffffff, 0xffffffFF}              
};
 
   
    
const PM_PINFUNC_T pm_default_global_map[]=
{
    {0xffffffff, 0xffffffff}
};

const PM_GPIO_CTL_T  pm_gpio_default_map[]=
{
    {33,      0,     PM_OUTPUT,      PM_LEVEL    }, //Prevent CMMB current leakage 
    {163,     1,     PM_INPUT,       PM_LEVEL    }, //power button        
    {102,     1,     PM_OUTPUT,      PM_LEVEL    }, //LCD_RST
    {90,      0,     PM_OUTPUT,      PM_NO_INT   }, //BT_RESET
#ifdef DEMOD_HW_SIANO    
    {93,      0,     PM_INPUT,       PM_RISING_EDGE},//PM_RISING_EDGE    }, //sian2186 demod int
#endif
#ifdef DEMOD_HW_INNOFIDEI
    {93,      1,     PM_INPUT,       PM_FALLING_EDGE},//PM_RISING_EDGE    }, //if228 demod int
#endif
    {94,      0,     PM_OUTPUT,      PM_LEVEL    }, //if228 demod reset            
    {140,     0,     PM_OUTPUT,      PM_NO_INT   }, //WIFI RESET
	{106,     1,     PM_OUTPUT,      PM_NO_INT   }, //WIFI LDO POWER
    {135,     0,     PM_OUTPUT,      PM_LEVEL    }, //Add Temp For G1 ROMCODE Usb LDO Problem  
	{141,     1,     PM_INPUT,       PM_LEVEL	 }, //WIFI spi int
    {79,      1,     PM_OUTPUT,      PM_NO_INT	 }, //img sensor(postpositive sensor) powerdown
    {78,      1,     PM_OUTPUT,      PM_NO_INT	 }, //img sensor(preositive sensor) powerdown
    {77,      0,     PM_OUTPUT,      PM_NO_INT   },  // img sensor reset    
    {165,     1,     PM_INPUT,       PM_LEVEL	 }, //HEADSET_DETECT
    {164,     0,     PM_INPUT,       PM_LEVEL	 }, //HEADSET_BUTTON
    {145,     0,     PM_INPUT,       PM_NO_INT	 }, //GPIO_PROD_USB_DETECT_ID
    {146,     0,     PM_INPUT,       PM_NO_INT	 }, //USB_DP
    {162,     0,     PM_INPUT,       PM_LEVEL	 }, //CHARGE_PLUG_DETECT
   
    {0xffff,  0, 	 PM_INVALID_DIR, PM_INVALID_INT  }
};

#define ANA_IS_ANA_REG(addr) (SPRD_MISC_BASE == (addr & 0xfffff000))
void sc8800g_pin_map_init(void)
{
    int i = 0;

	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
    CHIP_REG_OR ( (GR_GEN0), (GEN0_PIN_EN));
    ANA_REG_OR (ANA_AGEN, (AGEN_PINREG_EN)); // enable gpio base romcode
    for(;;)
    {
        if (pm_func[i].addr == PM_INVALID_VAL)
        {
            break;
        }

        if (!ANA_IS_ANA_REG(pm_func[i].addr))
        {
            CHIP_REG_SET (pm_func[i].addr ,pm_func[i].value);
        }

        else
        {
		printk("##: write ANA register!\n");
            ANA_REG_SET (pm_func[i].addr ,pm_func[i].value);
        }
        i++;
    }
    i = 0;
    for(;;)
    {
        if (pm_default_global_map[i].addr == PM_INVALID_VAL)
        {
            break;
        }

        CHIP_REG_SET (pm_default_global_map[i].addr, pm_default_global_map[i].value);
        i++;
    }
    printk("####: pin_map is done!\n");
}
EXPORT_SYMBOL_GPL(sc8800g_pin_map_init);

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
const PM_PINFUNC_T pm_func_suspend[]=
{
//  | Pin Register        |   DS    | Func PU/PD | Func Select  | Slp PU/PD | Sleep OE/IE | 
    {PIN_CTL_REG,        ( 0x1fff00) },
    {0xffffffff, 0xffffffFF}              
};
    
    
const PM_PINFUNC_T pm_default_global_map_suspend[]=
{
    {0xffffffff, 0xffffffff}
};

const PM_GPIO_CTL_T  pm_gpio_default_map_suspend[]=
{
    {33,      0,     PM_OUTPUT,      PM_LEVEL    }, //Prevent CMMB current leakage 
    {163,     1,     PM_INPUT,       PM_LEVEL    }, //power button        
    {102,     1,     PM_OUTPUT,      PM_LEVEL    }, //LCD_RST
    {90,      0,     PM_OUTPUT,      PM_NO_INT   }, //BT_RESET
#ifdef DEMOD_HW_SIANO    
    {93,      0,     PM_INPUT,       PM_RISING_EDGE},//PM_RISING_EDGE    }, //sian2186 demod int
#endif
#ifdef DEMOD_HW_INNOFIDEI
    {93,      1,     PM_INPUT,       PM_FALLING_EDGE},//PM_RISING_EDGE    }, //if228 demod int
#endif
    {94,      0,     PM_OUTPUT,      PM_LEVEL    }, //if228 demod reset            
    {140,     0,     PM_OUTPUT,      PM_NO_INT   }, //WIFI RESET
	{106,     1,     PM_OUTPUT,      PM_NO_INT   }, //WIFI LDO POWER
    {135,     0,     PM_OUTPUT,      PM_LEVEL    }, //Add Temp For G1 ROMCODE Usb LDO Problem  
	{141,     1,     PM_INPUT,       PM_LEVEL	 }, //WIFI spi int
    {79,      1,     PM_OUTPUT,      PM_NO_INT	 }, //img sensor(postpositive sensor) powerdown
    {78,      1,     PM_OUTPUT,      PM_NO_INT	 }, //img sensor(preositive sensor) powerdown
    {77,      0,     PM_OUTPUT,      PM_NO_INT   },  // img sensor reset    
    {165,     1,     PM_INPUT,       PM_LEVEL	 }, //HEADSET_DETECT
    {164,     0,     PM_INPUT,       PM_LEVEL	 }, //HEADSET_BUTTON
    {145,     0,     PM_INPUT,       PM_NO_INT	 }, //GPIO_PROD_USB_DETECT_ID
    {146,     0,     PM_INPUT,       PM_NO_INT	 }, //USB_DP
    {162,     0,     PM_INPUT,       PM_LEVEL	 }, //CHARGE_PLUG_DETECT
   
    {0xffff,  0, 	 PM_INVALID_DIR, PM_INVALID_INT  }
};

void gpio_for_suespend(void)
{
    int i = 0;
/*
	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
	printk("##: Doing pin map!\n");
*/
    CHIP_REG_OR ( (GR_GEN0), (GEN0_PIN_EN));
    ANA_REG_OR (ANA_AGEN, (AGEN_PINREG_EN)); // enable gpio base romcode
    for(;;)
    {
        if (pm_func_suspend[i].addr == PM_INVALID_VAL)
        {
            break;
        }

        if (!ANA_IS_ANA_REG(pm_func_suspend[i].addr))
        {
            CHIP_REG_SET (pm_func_suspend[i].addr ,pm_func_suspend[i].value);
        }

        else
        {
		printk("##: write ANA register!\n");
            ANA_REG_SET (pm_func_suspend[i].addr ,pm_func_suspend[i].value);
        }
        i++;
    }
    i = 0;
    for(;;)
    {
        if (pm_default_global_map_suspend[i].addr == PM_INVALID_VAL)
        {
            break;
        }

        CHIP_REG_SET (pm_default_global_map_suspend[i].addr, pm_default_global_map_suspend[i].value);
        i++;
    }
    //printk("####: gpio_for_suespend() is done!\n");
}
EXPORT_SYMBOL_GPL(gpio_for_suespend);



