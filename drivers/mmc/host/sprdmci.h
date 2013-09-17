/*
 *  linux/drivers/mmc/host/sdhci.h - Secure Digital Host Controller Interface driver
 *
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#ifndef __SPRDMCI_H__
#define __SPRDMCI_H__

struct sprd_host_data {
	int detect_irq;
};

//add by chengwg.
#ifdef CONFIG_MMC_DEV_TROUT
struct trout_scan_info {
        int scan_rst;
        struct completion trout_scan_comp;
};

extern struct trout_scan_info trout_info;

#endif

#endif

