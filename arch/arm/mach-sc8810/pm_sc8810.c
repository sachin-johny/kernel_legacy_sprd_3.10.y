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

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <mach/globalregs.h>
#include <asm/irqflags.h>
#include <clock_common.h>
#include "clock_sc8810.h"
#include <mach/adi.h>
#include <linux/io.h>

#define SLEEP_MODE_ARM_CORE 0
#define SLEEP_MODE_MCU 1
#define SLEEP_MODE_DEEP 2


#define hw_raw_irqs_disabled_flags(flags)	\
({										\
	(int)((flags) & PSR_I_BIT);				\
})

#define hw_irqs_disabled()				\
({										\
	unsigned long _flags;					\
	_flags = hw_local_save_flags();		\
	hw_raw_irqs_disabled_flags(_flags);	\
})

u32 __attribute__ ((naked)) sc8800g_read_cpsr(void)
{
	__asm__ __volatile__("mrs r0, cpsr\nbx lr");
}

/*FIXME:TODO
 *the code beblow just ensure adb ahb and audio is complete shut down.
 *but this should be done by drivers .
 */

#define GEN0_MASK ( GEN0_SIM0_EN | GEN0_I2C_EN | GEN0_GPIO_EN | 			\
			   GEN0_I2C0_EN|GEN0_I2C1_EN|GEN0_I2C2_EN|GEN0_I2C3_EN | 		\
			   GEN0_SPI0_EN|GEN0_SPI1_EN| GEN0_I2S0_EN | GEN0_I2S1_EN| 	\
	                GEN0_EFUSE_EN | GEN0_I2S_EN | GEN0_PIN_EN | 				\
	                GEN0_EPT_EN | GEN0_SIM1_EN | GEN0_SPI_EN | GEN0_UART0_EN | \
	                GEN0_UART1_EN | GEN0_UART2_EN)

#define CLK_EN_MASK (CLK_PWM0_EN | CLK_PWM1_EN | CLK_PWM2_EN | CLK_PWM3_EN)

#define BUSCLK_ALM_MASK (ARM_VB_MCLKON|ARM_VB_DA0ON|ARM_VB_DA1ON|ARM_VB_ADCON|ARM_VB_ANAON|ARM_VB_ACC)

#define AHB_CTL0_MASK   (AHB_CTL0_DCAM_EN|AHB_CTL0_CCIR_EN|AHB_CTL0_LCDC_EN|    \
                         AHB_CTL0_SDIO0_EN|AHB_CTL0_SDIO1_EN|AHB_CTL0_DMA_EN|     \
                         AHB_CTL0_BM0_EN |AHB_CTL0_NFC_EN|AHB_CTL0_BM1_EN|       \
                         AHB_CTL0_G2D_EN|AHB_CTL0_G3D_EN|	\
                         AHB_CTL0_AXIBUSMON0_EN|AHB_CTL0_AXIBUSMON1_EN|	\
                         AHB_CTL0_VSP_EN|AHB_CTL0_ROT_EN | AHB_CTL0_USBD_EN)

#define GR_CLK_EN_MASK CLK_EN_MASK
#define GR_GEN0_MASK GEN0_MASK

static void disable_audio_module(void)
{
	sprd_greg_clear_bits(REG_TYPE_GLOBAL, BUSCLK_ALM_MASK, GR_BUSCLK_ALM);
}

static void disable_ahb_module (void)
{
	sprd_greg_clear_bits(REG_TYPE_AHB_GLOBAL, GEN0_MASK, AHB_CTL0);
}


/*FIXME:TODO
 * the code beblow just save value of ahb apb audio
 */
u32 reg_gen_clk_en, reg_gen0_val, reg_busclk_alm, reg_ahb_ctl0_val;

/*register save*/
#define SAVE_GR_REG(_reg_save, _reg_type, _reg_addr, _reg_mask)  \
	{_reg_save = (sprd_greg_read(_reg_type, _reg_addr) & ((u32)_reg_mask));}

#define SAVE_GLOBAL_REG  do{ \
        SAVE_GR_REG(reg_gen_clk_en, REG_TYPE_GLOBAL, GR_CLK_EN, GR_CLK_EN_MASK); \
        SAVE_GR_REG(reg_gen0_val, REG_TYPE_GLOBAL, GR_GEN0, GR_GEN0_MASK);   \
        SAVE_GR_REG(reg_busclk_alm, REG_TYPE_GLOBAL, GR_BUSCLK_ALM, BUSCLK_ALM_MASK);    \
        SAVE_GR_REG(reg_ahb_ctl0_val, REG_TYPE_AHB_GLOBAL, AHB_CTL0, AHB_CTL0_MASK);\
    }while(0)

/*register restore*/
#define RESTORE_GR_REG(_reg_type, _reg_addr, _reg_mask, _reg_save)    do{ \
	sprd_greg_set_bits(_reg_type, _reg_mask & _reg_save, _reg_addr); \
    }while(0)

#define RESTORE_GLOBAL_REG   do{  \
        RESTORE_GR_REG(REG_TYPE_GLOBAL, GR_CLK_EN, GR_CLK_EN_MASK, reg_gen_clk_en);   \
        RESTORE_GR_REG(REG_TYPE_GLOBAL, GR_BUSCLK_ALM, BUSCLK_ALM_MASK, reg_busclk_alm); \
        RESTORE_GR_REG(REG_TYPE_GLOBAL, GR_GEN0, GR_GEN0_MASK, reg_gen0_val); \
        RESTORE_GR_REG(REG_TYPE_AHB_GLOBAL, AHB_CTL0, AHB_CTL0_MASK, reg_ahb_ctl0_val); \
    }while(0)

/*FIXME:TODO
 *the code beblow just print some registers info for debug
 *can not find them in headers, so just define here
 */
#define INT_REG(off) (SPRD_INTCV_BASE + (off))

#define INT_IRQ_STS            INT_REG(0x0000)
#define INT_IRQ_RAW           INT_REG(0x0004)
#define INT_IRQ_ENB           INT_REG(0x0008)
#define INT_IRQ_DIS            INT_REG(0x000c)
#define INT_FIQ_STS            INT_REG(0x0020)

#define INT_IRQ_MASK	(1<<3)

static void print_ahb(void)
{
	u32 val = sprd_greg_read(REG_TYPE_AHB_GLOBAL, AHB_CTL0);
	printk("##: AHB_CTL0 = %08x.\n", val);
	if (val & AHB_CTL0_DCAM_EN) printk("AHB_CTL0_DCAM_EN =1.\n");
	if (val & AHB_CTL0_CCIR_EN) printk("AHB_CTL0_CCIR_EN =1.\n");
	if (val & AHB_CTL0_LCDC_EN) printk("AHB_CTL0_LCDC_EN =1.\n");
	if (val & AHB_CTL0_SDIO0_EN) printk("AHB_CTL0_SDIO0_EN =1.\n");
	if (val & AHB_CTL0_SDIO1_EN) printk("AHB_CTL0_SDIO1_EN =1.\n");
	if (val & AHB_CTL0_DMA_EN) printk("AHB_CTL0_DMA_EN =1.\n");
	if (val & AHB_CTL0_BM0_EN) printk("AHB_CTL0_BM0_EN =1.\n");
	if (val & AHB_CTL0_NFC_EN) printk("AHB_CTL0_NFC_EN =1.\n");
	if (val & AHB_CTL0_BM1_EN) printk("AHB_CTL0_BM1_EN =1.\n");
	if (val & AHB_CTL0_G2D_EN) printk("AHB_CTL0_G2D_EN =1.\n");
	if (val & AHB_CTL0_G3D_EN) printk("AHB_CTL0_G3D_EN =1.\n");
	if (val & AHB_CTL0_AXIBUSMON0_EN) printk("AHB_CTL0_AXIBUSMON0_EN =1.\n");
	if (val & AHB_CTL0_AXIBUSMON1_EN) printk("AHB_CTL0_AXIBUSMON1_EN =1.\n");
	if (val & AHB_CTL0_VSP_EN) printk("AHB_CTL0_VSP_EN =1.\n");
	if (val & AHB_CTL0_ROT_EN) printk("AHB_CTL0_ROT_EN =1.\n");
	if (val & AHB_CTL0_USBD_EN) printk("AHB_CTL0_USBD_EN =1.\n");
}


static void print_gr(void)
{
	u32 val = sprd_greg_read(REG_TYPE_GLOBAL, GR_GEN0);
		printk("##: GR_GEN0 = %08x.\n", val);
		if (val & GEN0_SIM0_EN) printk("GEN0_SIM0_EN =1.\n");
		if (val & GEN0_I2C_EN) printk("GEN0_I2C_EN =1.\n");
		if (val & GEN0_GPIO_EN) printk("GEN0_GPIO_EN =1.\n");
		if (val & GEN0_I2C0_EN) printk("GEN0_I2C0_EN =1.\n");
		if (val & GEN0_I2C1_EN) printk("GEN0_I2C1_EN =1.\n");
		if (val & GEN0_I2C2_EN) printk("GEN0_I2C2_EN =1.\n");
		if (val & GEN0_I2C3_EN) printk("GEN0_I2C3_EN =1.\n");
		if (val & GEN0_SPI0_EN) printk("GEN0_SPI0_EN =1.\n");
		if (val & GEN0_SPI1_EN) printk("GEN0_SPI1_EN =1.\n");
		if (val & GEN0_I2S0_EN) printk("GEN0_I2S0_EN =1.\n");
		if (val & GEN0_I2S1_EN) printk("GEN0_I2S1_EN =1.\n");
		if (val & GEN0_EFUSE_EN) printk("GEN0_EFUSE_EN =1.\n");
		if (val & GEN0_I2S_EN) printk("GEN0_I2S_EN =1.\n");
		if (val & GEN0_PIN_EN) printk("GEN0_PIN_EN =1.\n");
		if (val & GEN0_EPT_EN) printk("GEN0_EPT_EN =1.\n");
		if (val & GEN0_SIM1_EN) printk("GEN0_SIM1_EN =1.\n");
		if (val & GEN0_SPI_EN) printk("GEN0_SPI_EN =1.\n");
		if (val & GEN0_UART0_EN) printk("GEN0_UART0_EN =1.\n");
		if (val & GEN0_UART1_EN) printk("GEN0_UART1_EN =1.\n");
		if (val & GEN0_UART2_EN) printk("GEN0_UART2_EN =1.\n");

}

static int print_info()
{
	print_ahb();
	print_gr();
	return 0;
}

void print_irqs(void){
	int irq_sts, fiq_sts;
	irq_sts = __raw_readl(INT_IRQ_STS);
	fiq_sts = __raw_readl(INT_FIQ_STS);
	printk("PM: irq_sts = %d  fiq_sts= %d\n", irq_sts, fiq_sts);
}

static void arm_sleep(void)
{
	cpu_do_idle();
}

static void mcu_sleep(void)
{
	SAVE_GLOBAL_REG;
	disable_audio_module();
	disable_ahb_module();
	cpu_do_idle();
	RESTORE_GLOBAL_REG;
}

static int deep_sleep(void)
{
	cpu_do_idle();
	return 0;
}

int sc8810_prepare_late(void)
{
	print_info();
	return 0;
}

int sc8810_deep_sleep(void)
{
	int status, ret = 0;
	unsigned long flags;

	if (!hw_irqs_disabled())  {
		flags = sc8800g_read_cpsr();
		printk("##: Error(%s): IRQ is enabled(%08lx)!\n",
			 "wakelock_suspend", flags);
	}

	status = sc8810_get_clock_status();

	/*sc8800g_save_pll();*/
	if (status & DEVICE_AHB)  {
		printk("## sleep[ARM_CORE].\n");
		arm_sleep();
	} else if (status & DEVICE_APB) {
		printk("## sleep[MCU].\n");
		mcu_sleep();
	} else {
		printk("## sleep[DEEP].\n");
		ret = deep_sleep();
	}
	/*sc8800g_restore_pll();*/
	return ret;
}


void sc8810_pm_init(void)
{
}


