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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/debugfs.h>

#include <mach/hardware.h>
#ifdef CONFIG_SPRD_MAILBOX
#include <mach/mailbox.h>
#endif

#include <linux/sipc.h>
#include <linux/sipc_priv.h>
#ifndef CONFIG_OF

#define SMSG_TXBUF_ADDR		(0)
#define SMSG_TXBUF_SIZE		(SZ_1K)
#define SMSG_RXBUF_ADDR		(SMSG_TXBUF_SIZE)
#define SMSG_RXBUF_SIZE		(SZ_1K)

#define SMSG_RINGHDR		(SMSG_TXBUF_SIZE + SMSG_RXBUF_SIZE)
#define SMSG_TXBUF_RDPTR	(SMSG_RINGHDR + 0)
#define SMSG_TXBUF_WRPTR	(SMSG_RINGHDR + 4)
#define SMSG_RXBUF_RDPTR	(SMSG_RINGHDR + 8)
#define SMSG_RXBUF_WRPTR	(SMSG_RINGHDR + 12)

#ifdef CONFIG_SIPC_TD
extern uint32_t cpt_rxirq_status(void);
extern void cpt_rxirq_clear(void);
extern void cpt_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_cpt = {
	.name = "sipc-td",
	.dst = SIPC_ID_CPT,

	.irq = IRQ_SIPC_CPT,
	.rxirq_status = cpt_rxirq_status,
	.rxirq_clear = cpt_rxirq_clear,
	.txirq_trigger = cpt_txirq_trigger,
};

static int __init sipc_td_init(void)
{
	uint32_t base = (uint32_t)ioremap(CPT_RING_ADDR, CPT_RING_SIZE);

	smsg_ipc_cpt.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_cpt.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_cpt.txbuf_rdptr = base + SMSG_TXBUF_RDPTR;
	smsg_ipc_cpt.txbuf_wrptr = base + SMSG_TXBUF_WRPTR;

	smsg_ipc_cpt.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);;
	smsg_ipc_cpt.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_cpt.rxbuf_rdptr = base + SMSG_RXBUF_RDPTR;
	smsg_ipc_cpt.rxbuf_wrptr = base + SMSG_RXBUF_WRPTR;

	return smsg_ipc_create(SIPC_ID_CPT, &smsg_ipc_cpt);
}
#endif

#ifdef CONFIG_SIPC_WCDMA
extern uint32_t cpw_rxirq_status(void);
extern void cpw_rxirq_clear(void);
extern void cpw_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_cpw = {
	.name = "sipc-wcdma",
	.dst = SIPC_ID_CPW,

	.irq = IRQ_SIPC_CPW,
	.rxirq_status = cpw_rxirq_status,
	.rxirq_clear = cpw_rxirq_clear,
	.txirq_trigger = cpw_txirq_trigger,
};

static int __init sipc_wcdma_init(void)
{
	uint32_t base = (uint32_t)ioremap(CPW_RING_ADDR, CPW_RING_SIZE);

	smsg_ipc_cpw.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_cpw.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_cpw.txbuf_rdptr = base + SMSG_TXBUF_RDPTR;
	smsg_ipc_cpw.txbuf_wrptr = base + SMSG_TXBUF_WRPTR;

	smsg_ipc_cpw.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);;
	smsg_ipc_cpw.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_cpw.rxbuf_rdptr = base + SMSG_RXBUF_RDPTR;
	smsg_ipc_cpw.rxbuf_wrptr = base + SMSG_RXBUF_WRPTR;

	return smsg_ipc_create(SIPC_ID_CPW, &smsg_ipc_cpw);
}
#endif

#ifdef CONFIG_SIPC_WCN
extern uint32_t wcn_rxirq_status(void);
extern void wcn_rxirq_clear(void);
extern void wcn_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_wcn = {
	.name = "sipc-wcn",
	.dst = SIPC_ID_WCN,

	.irq = IRQ_SIPC_WCN,
	.rxirq_status = wcn_rxirq_status,
	.rxirq_clear = wcn_rxirq_clear,
	.txirq_trigger = wcn_txirq_trigger,
};

static int __init sipc_wcn_init(void)
{
	uint32_t base = (uint32_t)ioremap(WCN_RING_ADDR, WCN_RING_SIZE);

	smsg_ipc_wcn.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_wcn.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_wcn.txbuf_rdptr = base + SMSG_TXBUF_RDPTR;
	smsg_ipc_wcn.txbuf_wrptr = base + SMSG_TXBUF_WRPTR;

	smsg_ipc_wcn.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_wcn.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_wcn.rxbuf_rdptr = base + SMSG_RXBUF_RDPTR;
	smsg_ipc_wcn.rxbuf_wrptr = base + SMSG_RXBUF_WRPTR;

	return smsg_ipc_create(SIPC_ID_WCN, &smsg_ipc_wcn);
}
#endif

#ifdef CONFIG_SIPC_WCN
static int __init itm_sblock_init(void)
{
	int ret;

	ret = sblock_create(3, 7,
			    64, 1664,
			    128, 1664);
	if (ret) {
		printk(KERN_ERR "Failed to create data sblock (%d)\n", ret);
		return -ENOMEM;
	}

	ret = sblock_create(3, 8,
			    1, (10 * 1024),
			    1, (10 * 1024));
	if (ret) {
		printk(KERN_ERR "Failed to create event sblock (%d)\n", ret);
		sblock_destroy(3, 7);
		return -ENOMEM;
	}

	printk(KERN_ERR "create sblock successfully\n");
	return 0;
}
#endif
#ifdef CONFIG_SIPC_PMIC
extern uint32_t pmic_rxirq_status(void);
extern void pmic_rxirq_clear(void);
extern void pmic_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_pmic = {
	.name = "sipc-pmic",
	.dst = SIPC_ID_PMIC,
#ifdef CONFIG_SPRD_MAILBOX
	.core_id = ARM7,
#else
	.irq = IRQ_SIPC_PMIC,
#endif
	.rxirq_status = pmic_rxirq_status,
	.rxirq_clear = pmic_rxirq_clear,
	.txirq_trigger = pmic_txirq_trigger,
};

static int __init sipc_pmic_init(void)
{
	uint32_t base = (uint32_t)PMIC_SIPC_RING_ADDR;

	smsg_ipc_pmic.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_pmic.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_pmic.txbuf_rdptr =base+0x1c00+ 0;
	smsg_ipc_pmic.txbuf_wrptr = base+0x1c00+ 4;

	smsg_ipc_pmic.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_pmic.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_pmic.rxbuf_rdptr =  base+0x1c00+ 8;
	smsg_ipc_pmic.rxbuf_wrptr =  base+0x1c00+ 12;

	return smsg_ipc_create(SIPC_ID_PMIC, &smsg_ipc_pmic);
}
#endif

#ifdef CONFIG_SIPC_GGE
extern uint32_t gge_rxirq_status(void);
extern void gge_rxirq_clear(void);
extern void gge_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_gge = {
	.name = "sipc-gge",
	.dst = SIPC_ID_GGE,
#ifdef CONFIG_SPRD_MAILBOX
	.core_id = MBOX_CORE_GGE,
#else
	.irq = IRQ_SIPC_GGE,
#endif
	.rxirq_status = gge_rxirq_status,
	.rxirq_clear = gge_rxirq_clear,
	.txirq_trigger = gge_txirq_trigger,
};

static int __init sipc_gge_init(void)
{
	uint32_t base = (uint32_t)ioremap(GGE_RING_ADDR, GGE_RING_SIZE);

	smsg_ipc_gge.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_gge.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_gge.txbuf_rdptr = base + SMSG_TXBUF_RDPTR;
	smsg_ipc_gge.txbuf_wrptr = base + SMSG_TXBUF_WRPTR;

	smsg_ipc_gge.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_gge.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_gge.rxbuf_rdptr = base + SMSG_RXBUF_RDPTR;
	smsg_ipc_gge.rxbuf_wrptr = base + SMSG_RXBUF_WRPTR;

	return smsg_ipc_create(SIPC_ID_GGE, &smsg_ipc_gge);
}
#endif // end of CONFIG_SIPC_GGE

#ifdef CONFIG_SIPC_LTE
extern uint32_t lte_rxirq_status(void);
extern void lte_rxirq_clear(void);
extern void lte_txirq_trigger(void);

static struct smsg_ipc smsg_ipc_lte = {
	.name = "sipc-lte",
	.dst = SIPC_ID_LTE,

#ifdef CONFIG_SPRD_MAILBOX
	.core_id = MBOX_CORE_LTE,
#else
	.irq = IRQ_SIPC_LTE,
#endif
	.rxirq_status = lte_rxirq_status,
	.rxirq_clear = lte_rxirq_clear,
	.txirq_trigger = lte_txirq_trigger,
};

static int __init sipc_lte_init(void)
{
	uint32_t base = (uint32_t)ioremap(LTE_RING_ADDR, LTE_RING_SIZE);

	smsg_ipc_lte.txbuf_size = SMSG_TXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_lte.txbuf_addr = base + SMSG_TXBUF_ADDR;
	smsg_ipc_lte.txbuf_rdptr = base + SMSG_TXBUF_RDPTR;
	smsg_ipc_lte.txbuf_wrptr = base + SMSG_TXBUF_WRPTR;

	smsg_ipc_lte.rxbuf_size = SMSG_RXBUF_SIZE / sizeof(struct smsg);
	smsg_ipc_lte.rxbuf_addr = base + SMSG_RXBUF_ADDR;
	smsg_ipc_lte.rxbuf_rdptr = base + SMSG_RXBUF_RDPTR;
	smsg_ipc_lte.rxbuf_wrptr = base + SMSG_RXBUF_WRPTR;

	return smsg_ipc_create(SIPC_ID_LTE, &smsg_ipc_lte);
}
#endif // end of CONFIG_SIPC_LTE

static int __init sipc_init(void)
{
	uint32_t smem_size = 0;

	smsg_suspend_init();

#ifdef CONFIG_SIPC_TD
	smem_size += CPT_SMEM_SIZE;
	sipc_td_init();
#endif

#ifdef CONFIG_SIPC_WCDMA
	smem_size += CPW_SMEM_SIZE;
	sipc_wcdma_init();
#endif

#ifdef CONFIG_SIPC_WCN
	smem_size += WCN_SMEM_SIZE;
	sipc_wcn_init();
#endif

#ifdef CONFIG_SIPC_GGE
	smem_size += GGE_SMEM_SIZE;
	sipc_gge_init();
#endif

#ifdef CONFIG_SIPC_LTE
	smem_size += LTE_SMEM_SIZE;
	sipc_lte_init();
#endif
#ifdef CONFIG_SIPC_PMIC
	 sipc_pmic_init();
#endif


	smem_init(SIPC_SMEM_ADDR, smem_size);

#ifdef CONFIG_SIPC_WCN
	itm_sblock_init();
#endif
	return 0;
}

subsys_initcall(sipc_init);
//arch_initcall(sipc_init);

MODULE_AUTHOR("Chen Gaopeng");
MODULE_DESCRIPTION("SIPC module driver");
MODULE_LICENSE("GPL");

#endif /* end of CONFIG_OF */
