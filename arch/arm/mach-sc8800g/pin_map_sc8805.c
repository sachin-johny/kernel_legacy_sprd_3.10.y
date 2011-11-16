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
    {PIN_SIMCLK0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101 SIM0
    {PIN_SIMDA0_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18A SIM0
    {PIN_SIMRST0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101 SIM0
    {PIN_SIMCLK1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101 SIM1
    {PIN_SIMDA1_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18A SIM1
    {PIN_SIMRST1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101 SIM1
    {PIN_SD0_CLK_REG,    ( PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100 SD/SDIO   
    {PIN_SD_CMD_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D0_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D1_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D2_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD_D3_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180 SD/SDIO
    {PIN_SD1_CLK_REG,    ( PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100 NC
    {PIN_KEYOUT0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101    KEY OUT
    {PIN_KEYOUT1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101    KEY OUT
    {PIN_KEYOUT2_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101    NC
    {PIN_KEYOUT3_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101    NC
    {PIN_KEYOUT4_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101    NC
    {PIN_KEYOUT5_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPX_EN | PIN_O_EN ) },    // 0x131  GPS_PWON 
    {PIN_KEYOUT6_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_3 | PIN_SPU_EN | PIN_I_EN ) },    // 0x1ba  CTP_INT  
    {PIN_KEYOUT7_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_3 | PIN_SPU_EN | PIN_O_EN ) },    // 0x1b9  CTP_RST   
    {PIN_KEYIN0_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18a   KEY IN 
    {PIN_KEYIN1_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18a   KEY IN 
    {PIN_KEYIN2_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18a   NC
    {PIN_KEYIN3_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18a   NC
    {PIN_KEYIN4_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18a   NC
    {PIN_KEYIN5_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_3 | PIN_SPU_EN | PIN_I_EN ) },    // 0x1ba  PROX_INT  
    {PIN_KEYIN6_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_I_EN ) },    // 0x136  G_INT1
    {PIN_KEYIN7_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_I_EN ) },    // 0x136  G_INT2
    {PIN_SPI_DI_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x180  CMMB WIFI SPIDO  
    {PIN_SPI_CLK_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  SPICLK  
    {PIN_SPI_DO_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  CMMB WIFI SPIDI  
    {PIN_SPI_CSN0_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x130  SPI CS0: WIFI  
    {PIN_SPI_CSN1_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x130  SPI CS1: CMMB  
    {PIN_MTDO_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x108  JTAG  
    {PIN_MTDI_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188  JTAG  
    {PIN_MTCK_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188  JTAG  
    {PIN_MTMS_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188  JTAG  
    {PIN_MTRST_N_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188  JTAG  
    {PIN_U0TXD_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  BT UART RX  
    {PIN_U0RXD_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18a  BT UART TX  
    {PIN_U0CTS_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_1 | PIN_SPX_EN | PIN_Z_EN ) },    // 0x110  GPS UART TX  
    {PIN_U0RTS_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_1   | PIN_SPX_EN | PIN_Z_EN ) },  // 0x110  GPS UART RX 
    {PIN_U1TXD_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  U1TXD  
    {PIN_U1RXD_REG,      ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN ) },    // 0x18a  U1RXD  
    {PIN_NFWPN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   NF WP = 0
    {PIN_NFRB_REG,       ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188    NF RB
    {PIN_NFCLE_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF CLE
    {PIN_NFALE_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF ALE
    {PIN_NFCEN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x108   NF CE = VCC
    {PIN_NFWEN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF WE
    {PIN_NFREN_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF RE
    {PIN_NFD0_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD1_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD2_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD3_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD4_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD5_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD6_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD7_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD8_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD9_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD10_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD11_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD12_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD13_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD14_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_NFD15_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    NF DATA
    {PIN_EMRST_N_REG,    (PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_3 | PIN_SPX_EN | PIN_I_EN) },    // 0x100    EMRST(NC)
    {PIN_EMA0_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA1_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA2_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA3_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA4_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA5_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA6_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA7_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA8_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA9_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA10_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA11_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA12_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMA13_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMCKE1_REG,     (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_3 | PIN_SPD_EN | PIN_O_EN) },    // 0x100  NC
    {PIN_EMD0_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD1_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD2_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD3_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD4_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD5_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD6_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD7_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQM0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQS0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD8_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD9_REG,       ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD10_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD11_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD12_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD13_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD14_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD15_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQM1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQS1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD16_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD17_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD18_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD19_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD20_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD21_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD22_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD23_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQM2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQS2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD24_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD25_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD26_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD27_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD28_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD29_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD30_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMD31_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQM3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMDQS3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_CLKDPMEM_REG,   ( PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x204    
    {PIN_CLKDMMEM_REG,   ( PIN_DS_2 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x204    
    {PIN_EMRAS_N_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   RAS = 0 
    {PIN_EMCAS_N_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   CAS = 0
    {PIN_EMWE_N_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x108   WE  = 1
    {PIN_EMCS_N0_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   CS = 0
    {PIN_EMCS_N1_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    NC
    {PIN_EMCS_N2_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPX_EN | PIN_O_EN ) },    // 0x131   GPS_RST
    {PIN_EMCS_N3_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3 | PIN_SPX_EN | PIN_O_EN ) },    // 0x131   GPS_PWD
    {PIN_EMBA0_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMBA1_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_EMCKE0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   CKE  = 0
    {PIN_LCD_CSN1_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  NC 
    {PIN_LCD_RSTN_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x108    
    {PIN_LCD_CD_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D4_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D5_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D6_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D7_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D8_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_WRN_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x108    
    {PIN_LCD_RDN_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x108    
    {PIN_LCD_CSN0_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x108    
    {PIN_LCD_D9_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D10_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D11_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D12_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D13_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D14_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D15_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104    
    {PIN_LCD_D16_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  NC
    {PIN_LCD_D17_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100  NC
    {PIN_LCD_FMARK_REG,  ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
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
    {PIN_CCIRRST_REG ,   ( PIN_DS_1 | PIN_SPX_EN | PIN_FUNC_3   | PIN_FPX_EN | PIN_Z_EN) },    // 0x130 CAMERA_Reset
    //{PIN_CCIRPD1_REG,    ( PIN_DS_0 | PIN_SPX_EN | PIN_FUNC_3   | PIN_FPX_EN | PIN_O_EN) },    // 0x100    CAMERA PD
    {PIN_CCIRPD1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    //{PIN_CCIRPD0_REG,    ( PIN_DS_1 | PIN_SPX_EN | PIN_FUNC_DEF   | PIN_FPX_EN | PIN_Z_EN) },    // 0x100    CAMERA PD
    {PIN_CCIRPD0_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   CAMERA will powerdown 
    {PIN_SCL_REG,        ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188    
    {PIN_SDA_REG,        ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188    
    {PIN_CLK_AUX0_REG,   ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101   GPS_32K
    {PIN_IISDI_REG,      ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x144   BT PCM_IN
    {PIN_IISDO_REG,      ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   BT PCM_OUT
    {PIN_IISCLK_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x144   BT PCM_CLK 
    {PIN_IISLRCK_REG,    ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x144   BT PCM_SYNC  
    {PIN_IISMCK_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_2   | PIN_SPD_EN | PIN_I_EN ) },    // 0x166    BTXTLEN = 0 
    {PIN_RFSDA0_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x144   QS3200 SPI SDA 
    {PIN_RFSCK0_REG,     ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x144   QS3200 SPI SCK 
    {PIN_RFSEN0_REG,     ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_Z_EN ) },    // 0x188   QS3200 SPI LE 
    {PIN_RFCTL0_REG,     ( PIN_DS_1  | PIN_FPU_EN | PIN_FUNC_3   | PIN_SPU_EN | PIN_O_EN ) },    // 0x1b9   GPIO90 BT RSTN /////PULL DOWN FOR TEST ONLY , BT DRAIN 4mA FROM VBAT
    {PIN_RFCTL1_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   TD PA MD1
    {PIN_RFCTL2_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   GSM PA MODE 
    {PIN_RFCTL3_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_3   | PIN_SPX_EN | PIN_O_EN ) },    // 0x131   FLASH_LED 
    {PIN_RFCTL4_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC  
    {PIN_RFCTL5_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   TD PA EN
    {PIN_RFCTL6_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL7_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL8_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   TD PA VMOD0
    //origin {PIN_RFCTL9_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF   | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL9_REG,     ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL10_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   GSM PA TXEN
    {PIN_RFCTL11_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF  | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL12_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   PA BANDSEL1 
    {PIN_RFCTL13_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL14_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100   NC
    {PIN_RFCTL15_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPD_EN | PIN_Z_EN ) },    // 0x104   PA BANDSEL2 
    {PIN_XTL_EN_REG,     ( PIN_DS_1 | PIN_FPX_EN  | PIN_FUNC_3    | PIN_SPX_EN | PIN_O_EN ) },    // 0x131   GPIO106_WIFI/BT LDO EN
    {PIN_PTEST_REG,      ( PIN_DS_1 | PIN_FPD_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x140    PULL DOWN TO GROUND EXTERNALLY

    {ANA_PIN_CHIP_RSTN_REG,(PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},   //0x18a         TEST POINT
    {ANA_PIN_PBINT_REG, (PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPU_EN | PIN_I_EN)},      //0x18a    PBINT
    {ANA_PIN_TP_XL_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},      //0x100      NC
    {ANA_PIN_TP_XR_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},      //0x100      NC
    {ANA_PIN_TP_YU_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},      //0x100      NC
    {ANA_PIN_TP_YD_REG, (PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN)},      //0x100      NC


    {PIN_GPIO135_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x181   WIFI_RST
    {PIN_GPIO136_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_I_EN ) },    // 0x182 WIFI_SPI_INT
    {PIN_GPIO137_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101 CMMB_WIFI_SPI_SW
    {PIN_GPIO138_REG,    ( PIN_DS_1 | PIN_FPU_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x181 CMMB_RSTN
    {PIN_GPIO139_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_I_EN ) },    // 0x102  CMMB_INT
    {PIN_GPIO140_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_O_EN ) },    // 0x101  CMMB_PWRON  
    {PIN_GPIO141_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_I_EN ) },    // 0x102  HP_DET
    //origin {PIN_GPIO142_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100    
    {PIN_GPIO142_REG,    ( PIN_DS_1 | PIN_FPX_EN | PIN_FUNC_DEF | PIN_SPX_EN | PIN_Z_EN ) },    // 0x100 WIFI_WAKEUP(NU)
    {PIN_GPIO143_REG,    ( PIN_DS_3 | PIN_FPX_EN | PIN_FUNC_1 | PIN_SPX_EN | PIN_Z_EN ) },    // 0x310  SCL
    {PIN_GPIO144_REG,    ( PIN_DS_3 | PIN_FPX_EN | PIN_FUNC_1 | PIN_SPX_EN | PIN_Z_EN ) },    // 0x310  SDA 
    {0xffffffff, 0xffffffFF}              
};
 
   
    
const PM_PINFUNC_T pm_default_global_map[]=
{
    {0xffffffff, 0xffffffff}
};




#define ANA_IS_ANA_REG(addr) (SPRD_MISC_BASE == (addr & 0xfffff000))


/*
	functions for comparing pin config register.
 */
#define MAX_PIN_NUM 512
u32 pin_reg_val[MAX_PIN_NUM];

void sc8800g_value_init(void)
{
    int i = 0;
    CHIP_REG_OR ( (GR_GEN0), (GEN0_PIN_EN));
    ANA_REG_OR (ANA_AGEN, (AGEN_PINREG_EN)); // enable gpio base romcode
    for(;;) {
        if (pm_func[i].addr == PM_INVALID_VAL) {
            break;
        }

        if (!ANA_IS_ANA_REG(pm_func[i].addr)) {
		pin_reg_val[i] = CHIP_REG_GET (pm_func[i].addr);
        }
        else {
		pin_reg_val[i] = ANA_REG_GET (pm_func[i].addr);
        }
        i++;
    }
    printk("####: sc8800g_value_init() is done!\n");
}

void sc8800g_value_compare(void)
{
    int i = 0;
	u32 val = 0;
    CHIP_REG_OR ( (GR_GEN0), (GEN0_PIN_EN));
    ANA_REG_OR (ANA_AGEN, (AGEN_PINREG_EN)); // enable gpio base romcode
    for(;;) {
        if (pm_func[i].addr == PM_INVALID_VAL) {
            break;
        }

        if (!ANA_IS_ANA_REG(pm_func[i].addr)) {
		val = CHIP_REG_GET (pm_func[i].addr);
		if (pin_reg_val[i] != val) {
			printk("##: Values are different[%08x]: want[%08x], but[%08x]!\n", 
				pm_func[i].addr, pin_reg_val[i], val);
			/*
			CHIP_REG_SET (pm_func[i].addr ,pm_func[i].value);
			*/
		}
        }
        else {
		val = ANA_REG_GET (pm_func[i].addr);
		if (pin_reg_val[i] != val) {
			printk("##: Values are different[%08x]: want[%08x], but[%08x]!\n", 
				pm_func[i].addr, pin_reg_val[i], val);
			/*
			ANA_REG_SET (pm_func[i].addr ,pm_func[i].value);
			*/
		}
        }
        i++;
    }
    /*
    printk("####: sc8800g_value_compare() is done!\n");
    */
}





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
    sc8800g_value_init();
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
    {0xffffffff, 0xffffffFF}              
};
    
    
const PM_PINFUNC_T pm_default_global_map_suspend[]=
{
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
    /* compare register's value. */
    sc8800g_value_compare();

}
EXPORT_SYMBOL_GPL(gpio_for_suespend);


void gpio_dump_registers(void)
{
    int i = 0;
	u32 val = 0;
    CHIP_REG_OR ( (GR_GEN0), (GEN0_PIN_EN));
    ANA_REG_OR (ANA_AGEN, (AGEN_PINREG_EN)); // enable gpio base romcode
    for(;;) {
        if (pm_func[i].addr == PM_INVALID_VAL) {
            break;
        }

        if (!ANA_IS_ANA_REG(pm_func[i].addr)) {
		val = CHIP_REG_GET (pm_func[i].addr);
		printk("reg[%08x] = %08x.\n", pm_func[i].addr, val);
        }
        else {
		val = ANA_REG_GET (pm_func[i].addr);
		printk("reg[%08x] = %08x.\n", pm_func[i].addr, val);
        }
        i++;
    }
    printk("####: gpio_dump_registers() is done!\n");
}
EXPORT_SYMBOL_GPL(gpio_dump_registers);

