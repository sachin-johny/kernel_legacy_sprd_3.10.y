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

#include <asm/irqflags.h>
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <mach/pm_debug.h>
#include <mach/sci_glb_regs.h>
#include <mach/sci.h>
#include <mach/adi.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>

struct dentry * dentry_debug_root = NULL;
static int is_print_sleep_mode = 0;
int is_print_linux_clock = 1;
int is_print_modem_clock = 1;
static int is_print_irq = 1;
static int is_print_wakeup = 1;
static int is_print_irq_runtime = 0;
static int is_print_time = 1;
static unsigned int core_time = 0;
static unsigned int mcu_time = 0;
static unsigned int lit_time = 0;
static unsigned int deep_time_successed = 0;
static unsigned int deep_time_failed = 0;
static unsigned int sleep_time = 0;
#define		SPRD_INTC_NUM			4
#define 	SPRD_HARD_INTERRUPT_NUM 128
#define 	SPRD_HARD_INT_NUM_EACH_INTC 32
#define		SPRD_IRQ_NUM			1024
static u32 sprd_hard_irq[SPRD_HARD_INTERRUPT_NUM]= {0, };
//static u32 sprd_irqs[SPRD_IRQ_NUM] = {0, };
static u32 sprd_irqs_sts[SPRD_INTC_NUM] = {0, };
static int is_wakeup = 0;
static int irq_status = 0;
//static int hard_irq_status[SPRD_INTC_NUM] = {0, };
static int sleep_mode = SLP_MODE_NON;
static char * sleep_mode_str[]  = {
	"[ARM]",
	"[MCU]",
	"[DEP]",
	"[NON]"
};
#define INT_REG(off)		(SPRD_INT_BASE + (off))
#define	INTC0_REG(off)		(SPRD_INTC0_BASE + (off))
#define	INTC1_REG(off)		(SPRD_INTC1_BASE + (off))
#define	INTC2_REG(off)		(SPRD_INTC2_BASE + (off))
#define	INTC3_REG(off)		(SPRD_INTC3_BASE + (off))
#define INT_IRQ_STS            INT_REG(0x0000)
#define INT_IRQ_RAW           INT_REG(0x0004)
#define INT_IRQ_ENB           INT_REG(0x0008)
#define INT_IRQ_DIS            INT_REG(0x000c)
#define INT_FIQ_STS            INT_REG(0x0020)
#define	INTCV0_IRQ_MSKSTS	INTC0_REG(0x0000)
#define	INTCV0_IRQ_RAW		INTC0_REG(0x0004)
#define	INTCV0_IRQ_EN		INTC0_REG(0x0008)
#define	INTCV0_IRQ_DIS		INTC0_REG(0x000C)
#define	INTCV0_FIQ_STS		INTC0_REG(0x0020)
#define	INTCV1_IRQ_MSKSTS	INTC1_REG(0x0000)
#define	INTCV1_IRQ_RAW		INTC1_REG(0x0004)
#define	INTCV1_IRQ_EN		INTC1_REG(0x0008)
#define	INTCV1_IRQ_DIS		INTC1_REG(0x000C)
#define	INTCV1_FIQ_STS		INTC1_REG(0x0020)
#define	INTCV2_IRQ_MSKSTS	INTC2_REG(0x0000)
#define	INTCV2_IRQ_RAW		INTC2_REG(0x0004)
#define	INTCV2_IRQ_EN		INTC2_REG(0x0008)
#define	INTCV2_IRQ_DIS		INTC2_REG(0x000C)
#define	INTCV2_FIQ_STS		INTC2_REG(0x0020)
#define	INTCV3_IRQ_MSKSTS	INTC3_REG(0x0000)
#define	INTCV3_IRQ_RAW		INTC3_REG(0x0004)
#define	INTCV3_IRQ_EN		INTC3_REG(0x0008)
#define	INTCV3_IRQ_DIS		INTC3_REG(0x000C)
#define	INTCV3_FIQ_STS		INTC3_REG(0x0020)
#define INT_IRQ_MASK	(1<<3)

void pm_debug_dump_ahb_glb_regs(void);

static void hard_irq_reset(void)
{
	int i = SPRD_HARD_INTERRUPT_NUM - 1;
	do{
		sprd_hard_irq[i] = 0;
	}while(--i >= 0);
}

static void parse_hard_irq(unsigned long val, unsigned long intc)
{
	int i;
	if(intc == 0){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[i]++;
		}
	}
	if(intc == 1){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[32+i]++;
		}
	}
	if(intc == 2){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[64+i]++;
		}
	}
	if(intc == 3){
		for (i = 0; i < SPRD_HARD_INT_NUM_EACH_INTC; i++) {
			if (test_and_clear_bit(i, &val)) sprd_hard_irq[96+i]++;
		}
	}
}

void hard_irq_set(void)
{
	sprd_irqs_sts[0] = __raw_readl(INT_IRQ_STS);
	sprd_irqs_sts[1] = __raw_readl(INT_FIQ_STS);
	irq_status = __raw_readl(INTCV0_IRQ_MSKSTS);
	parse_hard_irq(irq_status, 0);
	irq_status = __raw_readl(INTCV1_IRQ_MSKSTS);
	parse_hard_irq(irq_status, 1);
	irq_status = __raw_readl(INTCV2_IRQ_MSKSTS);
	parse_hard_irq(irq_status, 2);
	irq_status = __raw_readl(INTCV3_IRQ_MSKSTS);
	parse_hard_irq(irq_status, 3);
}

#define GPIO_GROUP_NUM		16
#define IRQ_EIC		(1<<14)
#define IRQ_BUSMON	(1<<13)
#define IRQ_WDG		(1<<11)
#define IRQ_CP2		(1<<10)
#define IRQ_CP1		(1<<9)
#define IRQ_CP0		(1<<8)
#define IRQ_CP_WDG	(1<<7)
#define IRQ_GPU		(1<<6)
#define IRQ_MM		(1<<5)
#define IRQ_4		(1<<4)
#define IRQ_3		(1<<3)
#define IRQ_2		(1<<2)

#define REG_GPIO_MIS            (0x0020)
#define ANA_REG_INT_MASK_STATUS (SPRD_MISC_BASE + 0x380 +0x0000)
void print_hard_irq_inloop(int ret)
{
	unsigned int i, j, val;
	unsigned int gpio_irq[GPIO_GROUP_NUM];

	if(sprd_irqs_sts[0] != 0)
		printk("%c#:INTC0: %08x\n", ret?'S':'F', sprd_irqs_sts[0]);
	if(sprd_irqs_sts[1] != 0)
		printk("%c#:INTC0 FIQ: %08x\n", ret?'S':'F', sprd_irqs_sts[1]);
	if(sprd_irqs_sts[0]&IRQ_EIC){
		printk("wake up by eic\n");
	}
	if(sprd_irqs_sts[0]&IRQ_BUSMON){
		printk("wake up by busmoniter\n");
	}
	if(sprd_irqs_sts[0]&IRQ_WDG){
		printk("wake up by ca7_wdg or ap_wdg\n");
	}
	if(sprd_irqs_sts[0]&IRQ_CP2){
		printk("wake up by cp2\n");
	}
	if(sprd_irqs_sts[0]&IRQ_CP1){
		printk("wake up by cp1\n");
	}
	if(sprd_irqs_sts[0]&IRQ_CP0){
		printk("wake up by cp0\n");
	}
	if(sprd_irqs_sts[0]&IRQ_GPU){
		printk("wake up by gpu\n");
	}
	if(sprd_irqs_sts[0]&IRQ_MM){
		printk("wake up by mm\n");
	}
	if(sprd_irqs_sts[0]&IRQ_4){
		if(sprd_hard_irq[34])
			printk("wake up by i2c\n");
		if(sprd_hard_irq[20])
			printk("wake up by audio\n");
		if(sprd_hard_irq[27])
			printk("wake up by fm\n");
		if(sprd_hard_irq[25]){
			printk("wake up by adi\n");
		}
		if(sprd_hard_irq[38])
			printk("wake up by ana\n");
		if(sprd_hard_irq[36])
			printk("wake up by kpd\n");
		if(sprd_hard_irq[35]){
			printk("wake up by gpio\n");
		}
	}
	if(sprd_irqs_sts[0]&IRQ_3){
		if(sprd_hard_irq[23])
			printk("wake up by vbc_ad01\n");
		if(sprd_hard_irq[24])
			printk("wake up by vbc_ad23\n");
		if(sprd_hard_irq[22])
			printk("wake up by vbc_da\n");
		if(sprd_hard_irq[21])
			printk("wake up by afifi_error\n");
	}
	if(sprd_irqs_sts[0]&IRQ_3){
		if(sprd_hard_irq[29]){
			printk("wake up by ap_tmr0\n");
		}
		if(sprd_hard_irq[118]){
			printk("wake up by ap_tmr1\n");
		}
		if(sprd_hard_irq[119]){
			printk("wake up by ap_tmr2\n");
		}
		if(sprd_hard_irq[120]){
			printk("wake up by ap_tmr3\n");
		}
		if(sprd_hard_irq[121]){
			printk("wake up by ap_tmr4\n");
		}
		if(sprd_hard_irq[31]){
			printk("wake up by ap_syst\n");
		}
		if(sprd_hard_irq[30]){
			printk("wake up by aon_syst\n");
		}
		if(sprd_hard_irq[28]){
			printk("wake up by aon_tmr\n");
		}
	}

	if(0){
		for(i=0; i<(GPIO_GROUP_NUM/2); i++){
			j = 2*i;
			gpio_irq[j]= __raw_readl(SPRD_GPIO_BASE + 0x100*i + REG_GPIO_MIS);
			gpio_irq[j+1]= __raw_readl(SPRD_GPIO_BASE + 0x100*i + 0x80 + REG_GPIO_MIS);
			printk("gpio_irq[%d]:0x%x, gpio_irq[%d]:0x%x \n", j, gpio_irq[j], j+1, gpio_irq[j+1]);
		}
		for(i=0; i<GPIO_GROUP_NUM; i++){
			if(gpio_irq[i] != 0){
				val = gpio_irq[i];
				while(val){
					j = __ffs(val);
					printk("gpio irq number : %d \n", (j+16*(i+1)) );
					val &= ~(1<<j);
				}
			}
		}
	}
	if(0){
		printk("adie, irq status:0x%x \n", sci_adi_read(ANA_REG_INT_MASK_STATUS));
	}

}

static void print_hard_irq(void)
{
	int i = SPRD_HARD_INTERRUPT_NUM -1;
	if(!is_print_irq)
		return;
	do{
		if(0 != sprd_hard_irq[i])
			printk("##: sprd_hard_irq[%d] = %d.\n",
                                i, sprd_hard_irq[i]);
	}while(--i >= 0);
}

#if 0
static void irq_reset(void)
{
	int i = SPRD_IRQ_NUM - 1;
	do{
		sprd_irqs[i] = 0;
	}while(--i >= 0);
}


void inc_irq(int irq)
{
	if(is_wakeup){
		if (irq >= SPRD_IRQ_NUM) {
			printk("## bad irq number %d.\n", irq);
			return;
		}
		sprd_irqs[irq]++;
		if(is_print_wakeup)
			printk("\n#####: wakeup irq = %d.\n", irq);
		is_wakeup = 0;
	}
}
EXPORT_SYMBOL(inc_irq);


static void print_irq(void)
{
	int i = SPRD_IRQ_NUM - 1;
	if(!is_print_irq)
		return;
	do{
		if(0 != sprd_irqs[i])
			printk("##: sprd_irqs[%d] = %d.\n",
                                i, sprd_irqs[i]);
	}while(--i >= 0);
}
#else
static void print_irq(void){}
#endif
void irq_wakeup_set(void)
{
	is_wakeup = 1;
}

void time_add(unsigned int time, int ret)
{
	switch(sleep_mode){
		case SLP_MODE_ARM:
			core_time += time;
			break;
		case SLP_MODE_MCU:
			mcu_time += time;
			break;
		case SLP_MODE_LIT:
			lit_time += time;
			break;
		case SLP_MODE_DEP:
			if(ret)
				deep_time_successed += time;
			else 
				deep_time_failed += time;
			break;
		default:
			break;
	}
}

void time_statisic_begin(void)
{
	core_time = 0;
	mcu_time = 0;
	lit_time = 0;
	deep_time_successed = 0;
	deep_time_failed = 0;
	sleep_time = get_sys_cnt();
	hard_irq_reset();
}

void time_statisic_end(void)
{
	sleep_time = get_sys_cnt() - sleep_time;
}

void print_time(void)
{
	if(!is_print_time)
		return;
	printk("time statisics : sleep_time=%d, core_time=%d, mcu_time=%d, lit_time=%d, deep_sus=%d, dep_fail=%d\n",
		sleep_time, core_time, mcu_time, lit_time, deep_time_successed, deep_time_failed);
}

void set_sleep_mode(int sm){
	int is_print = (sm == sleep_mode);
	sleep_mode = sm;
	if(is_print_sleep_mode == 0 || is_print )
		return;
	switch(sm){
		case SLP_MODE_ARM:
			printk("\n[ARM]\n");
			break;
		case SLP_MODE_MCU:
			printk("\n[MCU]\n");
			break;
		case SLP_MODE_LIT:
			printk("\n[LIT]\n");
			break;
		case SLP_MODE_DEP:
			printk("\n[DEP]\n");
			break;
		default:
			printk("\nNONE\n");
	}
}
void clr_sleep_mode(void)
{
	sleep_mode = SLP_MODE_NON;
}
void print_statisic(void)
{
	print_time();
	print_hard_irq();
	print_irq();
	pm_debug_dump_ahb_glb_regs();
	if(is_print_wakeup){
		printk("###wake up form %s : %08x\n",  sleep_mode_str[sleep_mode],  sprd_irqs_sts[0]);
		printk("###wake up form %s : %08x\n",  sleep_mode_str[sleep_mode],  sprd_irqs_sts[1]);
	}
}
#ifdef PM_PRINT_ENABLE
static struct wake_lock messages_wakelock;
#endif

#define PM_PRINT_ENABLE
static int print_thread(void * data)
{
	while(1){
		wake_lock(&messages_wakelock);
		has_wake_lock(WAKE_LOCK_SUSPEND);
		msleep(100);
		wake_unlock(&messages_wakelock);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(30 * HZ);
	}
	return 0;
}
static void debugfs_init(void)
{
	dentry_debug_root = debugfs_create_dir("power", NULL);
	if (IS_ERR(dentry_debug_root) || !dentry_debug_root) {
		printk("!!!powermanager Failed to create debugfs directory\n");
		dentry_debug_root = NULL;
		return;
	}
	debugfs_create_u32("print_sleep_mode", 0644, dentry_debug_root,
			   &is_print_sleep_mode);
	debugfs_create_u32("print_linux_clock", 0644, dentry_debug_root,
			   &is_print_linux_clock);
	debugfs_create_u32("print_modem_clock", 0644, dentry_debug_root,
			   &is_print_modem_clock);
	debugfs_create_u32("print_irq", 0644, dentry_debug_root,
			   &is_print_irq);
	debugfs_create_u32("print_wakeup", 0644, dentry_debug_root,
			   &is_print_wakeup);
	debugfs_create_u32("print_irq_runtime", 0644, dentry_debug_root,
			   &is_print_irq_runtime);
	debugfs_create_u32("print_time", 0644, dentry_debug_root,
			   &is_print_time);
}
static irqreturn_t sys_cnt_isr(int irq, void *dev_id)
{
	__raw_writel(8, SYSCNT_REG(0X8));
	return IRQ_HANDLED;
}
void pm_debug_init(void)
{
	struct task_struct * task;
	int ret;
	ret = request_irq(IRQ_APSYST_INT, sys_cnt_isr,
			    IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND,
			    "sys_cnt", NULL);
	if(ret){
		printk("sys cnt isr register failed\n");
		BUG();
	}
#ifdef PM_PRINT_ENABLE
	wake_lock_init(&messages_wakelock, WAKE_LOCK_SUSPEND,
			"pm_message_wakelock");
	task = kthread_create(print_thread, NULL, "pm_print");
	if (task == 0) {
		printk("Can't crate power manager print thread!\n");
	}else
		wake_up_process(task);
#endif
	debugfs_init();
}
void pm_debug_clr(void)
{
	if(dentry_debug_root != NULL)
		debugfs_remove_recursive(dentry_debug_root);
}
void pm_debug_dump_ahb_glb_regs(void){}
void pm_debug_save_ahb_glb_regs(void){}
void pm_debug_set_wakeup_timer(void)
{
	u32 val = get_sys_cnt();
	val = val + 30000;
	__raw_writel(val, SYSCNT_REG(0) );
	__raw_writel(1, SYSCNT_REG(0X8) );
}
