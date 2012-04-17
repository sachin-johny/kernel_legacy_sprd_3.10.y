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

#ifndef __SDHCI_SC8810G_H
#define __SDHCI_SC8810G_H

#include "sprdmci.h"

struct sprd_host_data {
	int detect_irq;
};

extern struct sdhci_host *sdhci_alloc_host(struct device *dev,
	size_t priv_size);
extern void sdhci_free_host(struct sdhci_host *host);
extern void sdhci_card_detect(struct sdhci_host *host);
extern int sdhci_add_host(struct sdhci_host *host);
extern void sdhci_remove_host(struct sdhci_host *host, int dead);

#ifdef CONFIG_PM
extern int sdhci_suspend_host(struct sdhci_host *host, pm_message_t state);
extern int sdhci_resume_host(struct sdhci_host *host);
extern void sdhci_enable_irq_wakeups(struct sdhci_host *host);
#endif


#endif /* __SDHCI_SC8810G_H */
