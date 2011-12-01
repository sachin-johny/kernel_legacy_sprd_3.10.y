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

/*Analog Die interrupt register*/
#define ANA_INT_STATUS             (SPRD_MISC_BASE +0x380+ 0x00)
#define ANA_INT_RAW                  (SPRD_MISC_BASE + 0x380 + 0x04)
#define ANA_INT_EN                	(SPRD_MISC_BASE + 0x380 + 0x08)
#define ANA_INT_STATUS_SYNC      (SPRD_MISC_BASE + 0x380 + 0x0C)



#include "dsp_data_no_wakeup_share_memory_0x0EC0_0000.h"

/* bit definitions. */
#define RUN_TEST         0
#define STOP_TEST        1
#define SYS_SLEEP_TEST   2
#define FORCE_SLEEP_TEST 3
#define PERIOD_TEST      4
#define GSM_PWR_DOMAIN_CLOSE (0x1UL << 16)
#define TD_PWR_DOMAIN_CLOSE  (0x1UL << 17)
#define CEVA_MEM_CLOSE  (0x1UL << 17)

#define 	SHARE_MEMORY_PHY_ADDR 	0x50000000
#define 	SHARE_MEMORY_SIZE 		0x00000100

#define 	DSP_MEMORY_PHY_ADDR 	0x0ec00000
#define 	DSP_MEMORY_SIZE 		0x00400000


extern void sc8800g_cpu_standby(void);

static int verify_dsp_deep_sleep(void)
{
	u32 val;
	int ret_val = 0;

	printk("####: check register: GR_STC_STATE for DSP\n");
	val = __raw_readl(GR_STC_STATE);
	printk("######: GR_STC_STATE =%08x\n", val);
	if (GR_DSP_STOP & val) {
		printk("#####: GR_STC_STATE[DSP_STOP] is set!\n");
	}
	else {
		printk("#####: GR_STC_STATE[DSP_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	printk("####: check register: GR_CLK_DLY for DSP\n");
	val = __raw_readl(GR_CLK_DLY);
	printk("######: GR_CLK_DLY =%08x\n", val);

	if (DSP_DEEP_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_DEEP_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_SYS_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_SYS_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_SYS_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_AHB_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_AHB_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_AHB_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (DSP_MTX_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_MTX_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_MTX_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_CORE_STOP & val) {
		printk("#####: GR_CLK_DLY[DSP_CORE_STOP] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[DSP_CORE_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (GR_EMC_STOP_CH5 & val) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH5] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH5] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (GR_EMC_STOP_CH4 & val) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH4] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH4] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (GR_EMC_STOP_CH3 & val) {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH3] is set!\n");
	}
	else {
		printk("#####: GR_CLK_DLY[GR_EMC_STOP_CH3] is NOT set!\n");
		ret_val = -EINVAL;
	}

	return ret_val;
}


static int verify_ahb_sts(void)
{
	u32 val;
	int ret_val = 0;

	printk("####: check register: AHB_STS for ARM\n");
	val = __raw_readl(AHB_STS);
	printk("######: AHB_STS =%08x\n", val);

	if (EMC_STOP_CH0 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH0] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH0] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (EMC_STOP_CH1 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH1] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH1] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH2 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH2] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH2] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH3 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH3] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH3] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH4 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH4] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH4] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH5 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH5] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH5] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH6 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH6] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH6] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH7 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH7] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH7] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (EMC_STOP_CH8 & val) {
		printk("#####: AHB_STS[EMC_STOP_CH8] is set!\n");
	}
	else {
		printk("#####: AHB_STS[EMC_STOP_CH8] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (ARMMTX_STOP_CH0 & val) {
		printk("#####: AHB_STS[ARMMTX_STOP_CH0] is set!\n");
	}
	else {
		printk("#####: AHB_STS[ARMMTX_STOP_CH0] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (ARMMTX_STOP_CH1 & val) {
		printk("#####: AHB_STS[ARMMTX_STOP_CH1] is set!\n");
	}
	else {
		printk("#####: AHB_STS[ARMMTX_STOP_CH1] is NOT set!\n");
		ret_val = -EINVAL;
	}
	if (ARMMTX_STOP_CH2 & val) {
		printk("#####: AHB_STS[ARMMTX_STOP_CH2] is set!\n");
	}
	else {
		printk("#####: AHB_STS[ARMMTX_STOP_CH2] is NOT set!\n");
		ret_val = -EINVAL;
	}


	if (AHB_STS_EMC_STOP & val) {
		printk("#####: AHB_STS[AHB_STS_EMC_STOP] is set!\n");
	}
	else {
		printk("#####: AHB_STS[AHB_STS_EMC_STOP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (AHB_STS_EMC_SLEEP & val) {
		printk("#####: AHB_STS[AHB_STS_EMC_SLEEP] is set!\n");
	}
	else {
		printk("#####: AHB_STS[AHB_STS_EMC_SLEEP] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DMA_BUSY & val) {
		printk("#####: AHB_STS[DMA_BUSY] is set!\n");
	}
	else {
		printk("#####: AHB_STS[DMA_BUSY] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (DSP_MAHB_SLEEP_EN & val) {
		printk("#####: AHB_STS[DSP_MAHB_SLEEP_EN] is set!\n");
	}
	else {
		printk("#####: AHB_STS[DSP_MAHB_SLEEP_EN] is NOT set!\n");
		ret_val = -EINVAL;
	}

	if (APB_PRI_EN & val) {
		printk("#####: AHB_STS[APB_PRI_EN] is set!\n");
	}
	else {
		printk("#####: AHB_STS[APB_PRI_EN] is NOT set!\n");
		ret_val = -EINVAL;
	}

	return ret_val;
}


int force_dsp_sleep(void)
{
	u32 i; 
	u32 temp;
	int times_try = 10;

	void __iomem *share_memory_base;
	void __iomem *dsp_memory_base;

	u32 val;

	verify_dsp_deep_sleep();

	printk("####: ioremap space for share memroy & dsp memory......\n");
	share_memory_base = ioremap(SHARE_MEMORY_PHY_ADDR,  SHARE_MEMORY_SIZE);
	if (!share_memory_base) {
		printk("####: Can't ioremap for share memroy!\n");
		return -ENOMEM;
	}


	dsp_memory_base = ioremap(DSP_MEMORY_PHY_ADDR,  DSP_MEMORY_SIZE);
	if (!dsp_memory_base) {
		printk("####: Can't ioremap for dsp memroy!\n");
		return -ENOMEM;
	}
	
	printk("#####: Let DSP enter deep sleep.....\n");

	/*    switch the ownership of UART1. */
	// *(volatile uint32 *)(GR_PCTL) |= BIT_8; 
	
    printk("For chip [SC8800G2]......\r\n");
	val = (SYS_SLEEP_TEST | TD_PWR_DOMAIN_CLOSE | TD_PWR_DOMAIN_CLOSE);
	__raw_writel(val, share_memory_base);
 
	printk("####: Downloading DSP firmware.......\r\n");

	for(i = 0; i < td_dsp_total_len; i+=4)
	{
    
    	temp = 0;
    	temp |= td_dsp_all_vec[i + 3];
    	temp |= td_dsp_all_vec[i + 2] << 8;
    	temp |= td_dsp_all_vec[i + 1] << 16;
    	temp |= td_dsp_all_vec[i + 0] << 24;
    	
    	__raw_writel(temp, dsp_memory_base + i);
    }
	
	for (i = 0; i < 32; i++) {
		printk("data[%d] = %02x\n", i, ((u8 *)dsp_memory_base)[i]);
	}
     /* reboot DSP */

	val = __raw_readl(DSP_BOOT_EN);
	val &= ~DSP_BOOT_ENABLE;
	__raw_writel(val, DSP_BOOT_EN);

	val = (DSP_MEMORY_PHY_ADDR & 0x0fffffff) | 0xc0000000;
	__raw_writel(val, DSP_BOOT_VEC);


	val = __raw_readl(DSP_BOOT_EN);
	val |= DSP_BOOT_ENABLE;
	__raw_writel(val, DSP_BOOT_EN);

	val = __raw_readl(DSP_RST);
	val |= DSP_RESET;
	__raw_writel(val, DSP_RST);

	while (verify_dsp_deep_sleep()) {
		if (!times_try) break;
		printk("#####: waiting DSP to enter deep sleep mode.....\n");
		mdelay(300);
	}
	if (!times_try) {
		printk("#####: timeout when waiting DSP to enter deep sleep mode!\n");
		return -EINVAL;
	}
	else {
		printk("#######: DSP is in deep sleep mode Now!\n");
	}
	return 0;
}

EXPORT_SYMBOL(force_dsp_sleep);
/*
 * configure pins for deep sleep, for example,
 * input & output enable in sleep mode.
 *
 */

static void pins_config(void)
{


}

static void adi_init(void)
{



}

int prepare_deep_sleep(void)
{
	u32 val;
	int i;
	u32 ahb_sts, gr_stc_sts, gr_clk_dly;
	u16 val_short;

	/* for saving global registers. */
	u32 ahb_ctl0, gen0, irq_flags;
	u16 backlight;

	/*
	irq_flags = __raw_readl(INT_IRQ_EN);
	*/

	/*
	__raw_writel(0xffffffff, INT_IRQ_DISABLE);
	__raw_writel(0xffffffff, INT_FIQ_DISABLE);
	*/

	/* disable all a-die interrupts. */
	/*
	ANA_REG_SET(ANA_INT_EN, 0x0);
	*/

	pins_config();
	adi_init();

	printk("#######: prepare system for deep sleep.......\n");
	/* disable voice band. */
	__raw_writel(0x0, GR_BUSCLK);

	//ANA_REG_OR(ANA_LDO_SLP, (FSM_RF0_BP_EN | FSM_RF1_BP_EN));
	ANA_REG_SET(ANA_LDO_SLP, 0xa7ff);

	/* vibrate power down. */
	ANA_REG_OR(ANA_ANA_CTL0, VIBR_PD);

	/* enable LDOs auto power down. */
	ANA_REG_OR(ANA_ANA_CTL0, FSM_AFCPD_EN);

	/* enable OTP. */
	ANA_REG_BIC(ANA_ANA_CTL0, OTP_ENABLE);

	/* shut down LDOs. */
	/*
	ANA_REG_SET(ANA_LDO_PD_CTL, 0x5555);
	*/


	backlight = ANA_REG_GET(ANA_LED_CTL);

	ANA_REG_SET(ANA_LED_CTL, 0xA081);//LED_CTRL

	ANA_REG_SET(ANA_PA_CTL, 0xC055); //PA_CTRL

	mdelay(100);

	val_short = ANA_REG_GET(ANA_LDO_SLP);
	printk("#####: ANA_LDO_SLP = %04x\n", val_short);

	val_short = ANA_REG_GET(ANA_ANA_CTL0);
	printk("#####: ANA_ANA_CTL0 = %04x\n", val_short);

	/* close all AHB devices. */
	val = __raw_readl(AHB_CTL0);
	ahb_ctl0 = val;
	val &= ~(AHB_CTL0_DCAM_EN | AHB_CTL0_CCIR_EN | AHB_CTL0_LCDC_EN |
			AHB_CTL0_SDIO_EN | AHB_CTL0_USBD_EN | AHB_CTL0_DMA_EN |
			AHB_CTL0_BM0_EN | AHB_CTL0_NFC_EN | AHB_CTL0_BM1_EN |
			AHB_CTL0_VSP_EN | AHB_CTL0_ROT_EN);
	__raw_writel(val, AHB_CTL0);

	/* setup EMC. */
	val = __raw_readl(EMC_CFG0);
	val |= RF_AUTO_SLEEP_ENABLE;
	__raw_writel(val, EMC_CFG0);

	for (i = 0; i < 6; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		val |= RF_AUTO_SLEEP_ENABLE_CHX;
		__raw_writel(val, EMC_CFG0_CHANNELS_BASE + i * 4);
	}

	for (i = 6; i < 9; i++) {
		val = __raw_readl(EMC_CFG0_CHANNELS_BASE + i * 4);
		val &= ~RF_AUTO_SLEEP_ENABLE_CHX;
		__raw_writel(val, EMC_CFG0_CHANNELS_BASE + i * 4);
	}


	val = __raw_readl(EMC_DCFG2);
	val &= ~DRF_AUTO_SLEEP_MODE;
	val |= DRF_REF_CNT_RST;
	__raw_writel(val, EMC_DCFG2);


	/* enable AHB devices's clock AUTO-GATE. */
	val = __raw_readl(AHB_CTL1);
	val |= (AHB_CTRL1_ARM_AUTO_GATE_EN | AHB_CTRL1_AHB_AUTO_GATE_EN |
			AHB_CTRL1_MCU_AUTO_GATE_EN | AHB_CTRL1_EMC_CH_AUTO_GATE_EN |
			AHB_CTRL1_EMC_AUTO_GATE_EN | AHB_CTRL1_ARMMTX_AUTO_GATE_EN |
			AHB_CTRL1_MSTMTX_AUTO_GATE_EN | AHB_CTRL1_ARM_DAHB_SLEEP_EN);
	__raw_writel(val, AHB_CTL1);

	/* enable XTL auto power down. */
	val = __raw_readl(GR_CLK_EN);
	val |= MCU_XTLEN_AUTOPD_EN;
	val &= ~BIT_17;		//must do, I don't know reason.
	__raw_writel(val, GR_CLK_EN);

	/* disable pwm[3:0]*/
	val = __raw_readl(GR_CLK_EN);
	val &= ~(CLK_PWM0_EN | CLK_PWM1_EN | CLK_PWM2_EN | CLK_PWM3_EN);
	__raw_writel(val, GR_CLK_EN);

	/*
	printk("####: check everything before disabling APB devices!\n");
	verify_dsp_deep_sleep();
	verify_ahb_sts();
	*/

	/*
	for (i = 0;i < 10; i++) {
		printk("####[%d]: IRQ_STS = %08x, FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_STS), __raw_readl(INT_FIQ_STS));

		printk("####[%d]: RAW_IRQ_STS = %08x, RAW_FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_RAW_STS), __raw_readl(INT_FIQ_RAW_STS));

		mdelay(500);
		val_short = ANA_REG_GET(ANA_INT_RAW);

		printk("#####: Analog die interrupt status: %04x\n", val_short);

	}
	mdelay(1000);
	*/

	/* disable all APB devices. */
	val = __raw_readl(GR_GEN0);
	gen0 = val;
	val &= ~(GEN0_TIMER_EN | GEN0_SIM0_EN | GEN0_I2C_EN  | GEN0_EFUSE_EN |
			GEN0_I2S_EN | GEN0_PIN_EN | GEN0_CCIR_MCLK_EN | GEN0_EPT_EN |
			GEN0_SIM1_EN | GEN0_SPI_EN | GEN0_UART0_EN | GEN0_UART1_EN  |
			GEN0_UART2_EN | GEN0_VB_EN);
	__raw_writel(val, GR_GEN0);

/*
	mdelay(10);
	ahb_sts = __raw_readl(AHB_STS);
	gr_stc_sts = __raw_readl(GR_STC_STATE);
	gr_clk_dly = __raw_readl(GR_CLK_DLY);

	val = __raw_readl(GR_GEN0);
	val |= GEN0_UART1_EN;
	__raw_writel(val, GR_GEN0);

	printk("####: after enabling UART1: ahb_sts = %08x, stc_sts = %08x, clk_dly = %08x\n",
				ahb_sts, gr_stc_sts, gr_clk_dly);
	mdelay(1000);


	val = __raw_readl(GR_GEN0);
	val &= ~GEN0_UART1_EN;
	__raw_writel(val, GR_GEN0);
	mdelay(100);
*/

#if 0
	/* set keypad as wakeup source. */
	val = __raw_readl(GR_GEN0);
	val |= (GEN0_KPD_EN | GEN0_KPD_RTC_EN);
	__raw_writel(val, GR_GEN0);

	/* enable interrupt of keypad. */
	val = BIT_10;
	__raw_writel(val, INT_IRQ_EN);
#endif

	val = __raw_readl(AHB_PAUSE);
	val |= (MCU_SYS_SLEEP_EN | MCU_DEEP_SLEEP_EN);
	__raw_writel(val, AHB_PAUSE);


	sc8800g_cpu_standby();

	mdelay(100);

	__raw_writel(ahb_ctl0, AHB_CTL0);
	__raw_writel(gen0, GR_GEN0);
//	__raw_writel(irq_flags, INT_IRQ_EN);

	mdelay(200);

	ANA_REG_SET(ANA_LED_CTL, backlight);


	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");
	printk("#####: return from sleep!!!!!\n");

	/*
	for (i = 0;i < 10; i++) {
		printk("####[%d]: IRQ_STS = %08x, FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_STS), __raw_readl(INT_FIQ_STS));

		printk("####[%d]: RAW_IRQ_STS = %08x, RAW_FIQ_STS = %08x\n", i,
				__raw_readl(INT_IRQ_RAW_STS), __raw_readl(INT_FIQ_RAW_STS));

		val_short = ANA_REG_GET(ANA_INT_RAW);

		printk("#####: Analog die interrupt status: %04x\n", val_short);

		mdelay(500);

	}
	*/
	/*

	val = __raw_readl(AHB_CTL0);
	val = (AHB_CTL0_DCAM_EN | AHB_CTL0_CCIR_EN | AHB_CTL0_LCDC_EN |
			AHB_CTL0_SDIO_EN | AHB_CTL0_USBD_EN | AHB_CTL0_DMA_EN |
			AHB_CTL0_BM0_EN | AHB_CTL0_NFC_EN | AHB_CTL0_BM1_EN |
			AHB_CTL0_VSP_EN | AHB_CTL0_ROT_EN);
	__raw_writel(val, AHB_CTL0);


	val = __raw_readl(GR_GEN0);
	val |= (GEN0_TIMER_EN | GEN0_SIM0_EN | GEN0_I2C_EN  | GEN0_EFUSE_EN |
			GEN0_I2S_EN | GEN0_PIN_EN | GEN0_CCIR_MCLK_EN | GEN0_EPT_EN |
			GEN0_SIM1_EN | GEN0_SPI_EN | GEN0_UART0_EN | GEN0_UART1_EN  |
			GEN0_UART2_EN | GEN0_VB_EN);
	__raw_writel(val, GR_GEN0);

	*/
/*
	val = __raw_readl(GR_GEN0);
	val |= (GEN0_TIMER_EN | GEN0_UART0_EN | GEN0_UART1_EN  | GEN0_UART2_EN);
	__raw_writel(val, GR_GEN0);

	printk("####: after waking up: ahb_sts = %08x, stc_sts = %08x, clk_dly = %08x\n",
				ahb_sts, gr_stc_sts, gr_clk_dly);
*/

	return 0;
}

EXPORT_SYMBOL(prepare_deep_sleep);


