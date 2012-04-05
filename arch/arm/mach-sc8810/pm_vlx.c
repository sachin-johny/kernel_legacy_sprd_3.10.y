/*
 * sc8800s Power Management Routines
 * * 
 * Copyright (c) 2010 Spreadtrum, Inc.
 *
 * created for sc8800g, 2010-09-07
 * Wang Liwei.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <mach/pm.h>

#include <asm/io.h>
#include <mach/regs_ahb.h>
#include <mach/test.h>
#include <mach/clock_common.h>
#include <mach/clock_sc8800g.h>
#include <mach/irqs.h>
#include <mach/pm_wakesource.h>


#define ANA_INT_STATUS             (SPRD_MISC_BASE +0x380+ 0x00)
#define ANA_INT_RAW                  (SPRD_MISC_BASE + 0x380 + 0x04)
#define ANA_INT_EN                     (SPRD_MISC_BASE + 0x380 + 0x08)
#define ANA_INT_STATUS_SYNC      (SPRD_MISC_BASE + 0x380 + 0x0C)
#if defined(CONFIG_ARCH_SC8810)
#define INT_IRQ_EN				(SPRD_INTCV_BASE + 0x08)

#define ANA_GPIO_DATA          (SPRD_MISC_BASE + 0x700 + 0x00)
#define ANA_GPIO_MASK          (SPRD_MISC_BASE + 0x700 + 0x04)
#define ANA_GPIO_IE            (SPRD_MISC_BASE + 0x700 + 0x18)
#define ANA_GPIO_IEV           (SPRD_MISC_BASE + 0x700 + 0x14)
#define ANA_GPIO_RIS           (SPRD_MISC_BASE + 0x700 + 0x1c)
#else
#define ANA_GPIO_IE            (SPRD_MISC_BASE + 0x600 + 0x18)
#define ANA_GPIO_IEV           (SPRD_MISC_BASE + 0x600 + 0x14)
#define ANA_GPIO_RIS           (SPRD_MISC_BASE + 0x600 + 0x1c)
#endif


#define WKAEUP_SRC_KEAPAD   BIT_10
#define WKAEUP_SRC_RX0      BIT_0
#define WAKEUP_SRC_PB		BIT_3
#define WAKEUP_SRC_CHG		BIT_2
#define SPRD_EICINT_BASE	(SPRD_EIC_BASE+0x80)

extern void sc8800g_cpu_standby(void);

extern int prepare_deep_sleep(void);


extern int prepare_deep_sleep(void);
extern int sc8800g_prepare_deep_sleep(void);
extern int sc8800g_enter_deepsleep(int);
int battery_updata(void);
void battery_sleep(void);

static u32 irq_enable = 0;
static u32 ana_gpio_irq_enable = 0;


int sc8800g_set_wakeup_src(void)
{
	u32 wakeup_src = 0;
	u32 val;
#ifdef CONFIG_MACH_SP6810A
	wakeup_src = (WKAEUP_SRC_KEAPAD |
				WKAEUP_SRC_RX0 |
				WAKEUP_SRC_PB |
				WAKEUP_SRC_CHG);
	if (WKAEUP_SRC_KEAPAD & wakeup_src) {
		val = __raw_readl(INT_IRQ_EN);
		irq_enable = val;
		val |= WKAEUP_SRC_KEAPAD;
		__raw_writel(val, INT_IRQ_EN);
	}
		 
	//enable UART0 Wake up Source  for  BT	
	if (WKAEUP_SRC_RX0 & wakeup_src) {
		val = __raw_readl(INT_IRQ_EN);
		val |= (BIT_0 | BIT_2);
		__raw_writel(val, INT_IRQ_EN);
		
		val = __raw_readl(INT_UINT_CTL);	
		val |= BIT_0 | BIT_16;
		__raw_writel(val, INT_UINT_CTL);
	}

	if (WAKEUP_SRC_PB & wakeup_src) {
		ana_gpio_irq_enable = ANA_REG_GET(ANA_GPIO_IE);
		ANA_REG_OR(ANA_GPIO_IE, WAKEUP_SRC_PB);
	}

	if (WAKEUP_SRC_CHG & wakeup_src) {
		ana_gpio_irq_enable = ANA_REG_GET(ANA_GPIO_IE);
		ANA_REG_OR(ANA_GPIO_IE, WAKEUP_SRC_CHG);
	}

#endif

#ifdef CONFIG_MACH_SP8805GA
	wakeup_src = (WKAEUP_SRC_KEAPAD | 
				WKAEUP_SRC_RX0 |
				 WAKEUP_SRC_CHG | 
				 WAKEUP_SRC_PB);
	if (WKAEUP_SRC_KEAPAD & wakeup_src) {
		val = __raw_readl(INT_IRQ_EN);
		irq_enable = val;
		val |= WKAEUP_SRC_KEAPAD;
		__raw_writel(val, INT_IRQ_EN);
	}

	//enable UART0 Wake up Source  for  BT	
	if (WKAEUP_SRC_RX0 & wakeup_src) {
		val = __raw_readl(INT_IRQ_EN);
		val |= (BIT_0 | BIT_2);
		__raw_writel(val, INT_IRQ_EN);
		
		val = __raw_readl(INT_UINT_CTL);	
		val |= BIT_0 | BIT_16;
		__raw_writel(val, INT_UINT_CTL);
	}
	if (WAKEUP_SRC_PB & wakeup_src) {
		ana_gpio_irq_enable = ANA_REG_GET(ANA_GPIO_IE);
		ANA_REG_OR(ANA_GPIO_IE, WAKEUP_SRC_PB);
	}

	if (WAKEUP_SRC_CHG & wakeup_src) {
		ana_gpio_irq_enable = ANA_REG_GET(ANA_GPIO_IE);
		ANA_REG_OR(ANA_GPIO_IE, WAKEUP_SRC_CHG);
	}

#endif

#ifdef CONFIG_ARCH_SC8810
	wakeup_src = (WKAEUP_SRC_KEAPAD |
				WKAEUP_SRC_RX0 |
				WAKEUP_SRC_CHG |
				 WAKEUP_SRC_PB);
	if (WKAEUP_SRC_KEAPAD & wakeup_src) {
		val = __raw_readl(INT_IRQ_EN);
		irq_enable = val;
		val |= WKAEUP_SRC_KEAPAD;
		__raw_writel(val, INT_IRQ_EN);
	}


	//enable UART0 Wake up Source  for  BT	
	if (WKAEUP_SRC_RX0 & wakeup_src) {
		//SIC interrupt enable
		val = __raw_readl(SPRD_EICINT_BASE+0x00);
		val |= BIT_0;
		__raw_writel(val, SPRD_EICINT_BASE+0x00);
		
		//SIC polarity 0
		val = __raw_readl(SPRD_EICINT_BASE+0x10);
		val |= BIT_0;
		__raw_writel(val, SPRD_EICINT_BASE+0x10);

	}

	if (WAKEUP_SRC_PB & wakeup_src) {
		ana_gpio_irq_enable = ANA_REG_GET(ANA_GPIO_IE);
		ANA_REG_OR(ANA_GPIO_IE, WAKEUP_SRC_PB);
	}

	if (WAKEUP_SRC_CHG & wakeup_src) {
		ana_gpio_irq_enable = ANA_REG_GET(ANA_GPIO_IE);
		ANA_REG_OR(ANA_GPIO_IE, WAKEUP_SRC_CHG);
	}
        
	printk("wake_source_set\n");
	wake_source_set( );
#endif
	return 0;
}

int sc8800g_unset_wakeup_src(void)
{
	/*
	__raw_writel(irq_enable, INT_IRQ_EN);
	*/
	wake_source_clr( );
	return 0;
}

#define BATTERY_CHECK_INTERVAL 30000
static int sprd_check_battery(void)
{
	int ret_val = 0;
	if (battery_updata()) {
		ret_val = 1;
	}
	return ret_val;
}
int in_calibration(void);
void my_mdelay(int ms)
{
	u32 suspend_start, suspend_end, suspend_time;
	suspend_start = suspend_end = get_sys_cnt();
	do {
		suspend_end = get_sys_cnt();
		suspend_time = suspend_end - suspend_start;
	}while(suspend_time < ms);
}

int sc8800g_pm_enter(suspend_state_t state)
{
	int ret_val = 0;
	u32 suspend_start, suspend_end, suspend_time;
	unsigned long flags;
    int calibration_enable = 0;
	/* for battery checking. */
	u32 battery_check_start;
    int status;

	/* make sure printk()  has finished. */
	my_mdelay(200);


	
    calibration_enable = in_calibration();
    /*
    if (calibration_enable) printk("##: In Calibration mode!\n");
    */
    status = sc8800g_get_clock_status();

    if (calibration_enable && (status & DEVICE_AHB)) return 0;
    
	suspend_start = suspend_end = get_sys_cnt();
	suspend_time = suspend_end - suspend_start;

	battery_check_start = suspend_start;
	sc8800g_set_wakeup_src();

	/*
	irq has been disabled,
	so we check irq status here safely.
	 */
	while((0 == sprd_suspend_interval) ||
		  (suspend_time < sprd_suspend_interval)) {
		hw_local_irq_disable();

#if defined(CONFIG_NKERNEL)
		local_irq_save(flags);
		if (raw_local_irq_pending()) {
			//printk("*******: pm_enter(), irq pending! *****\n");
			local_irq_restore(flags);
			hw_local_irq_enable();
			break;
		}
		local_irq_restore(flags);
		WARN_ONCE(!irqs_disabled(),
			"#####: Interrupts enabled in pm_enter()!\n");
	
		ret_val = os_ctx->idle(os_ctx);
		if (0 == ret_val) {
			sc8800g_enter_deepsleep(0);
			//printk("##: Return from deep sleep.\n");
			//mdelay(100);
		}
#else
		sc8800g_enter_deepsleep(0);
#endif
		suspend_end = get_sys_cnt();
		suspend_time = suspend_end - suspend_start;
		
		//printk("##: Going to enable hw irq.\n");
		//mdelay(200);

		hw_local_irq_enable();

		//printk("##: hw irq has been enabled.\n");
		//mdelay(300);

        /* check charger status. */
        battery_sleep();


		if ((suspend_end -  battery_check_start) > BATTERY_CHECK_INTERVAL) {
			battery_check_start = suspend_end;
			if (sprd_check_battery()) {
				printk("###: battery low!\n");
				break;
			}
		}
//		my_mdelay(100);
	}

	sc8800g_unset_wakeup_src();
	return ret_val;
}

EXPORT_SYMBOL_GPL(sc8800g_pm_enter);


static int sc8800g_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM || state == PM_SUSPEND_STANDBY;
}

int sc8800g_pm_prepare(void)
{
	int ret = 0;
	return ret;
}

void sc8800g_pm_finish(void)
{

}

static struct platform_suspend_ops sc8800g_pm_ops = {
	.valid		= sc8800g_pm_valid,
	.enter		= sc8800g_pm_enter,
	.prepare	= sc8800g_pm_prepare,
	.finish		= sc8800g_pm_finish,
};


/* APIs for cpuIdle. */

int sc8800g_enter_sleep(sc88xx_state_t state)
{
	return 0;
}

EXPORT_SYMBOL_GPL(sc8800g_enter_sleep);

static int __init sc8800g_pm_init(void)
{
/* prepare for deep sleep */
    sc8800g_prepare_deep_sleep();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&sc8800g_pm_ops);
#endif /* CONFIG_SUSPEND */

    return 0;
}

device_initcall(sc8800g_pm_init);

