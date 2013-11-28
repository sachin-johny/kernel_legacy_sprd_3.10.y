/*
 * drivers/gpu/iommu/iommu.c *
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include"../sprd_iommu_common.h"

int sprd_iommu_mm_init(struct sprd_iommu_dev *dev, struct sprd_iommu_init_data *data)
{
	dev->mmu_mclock= clk_get(NULL,"clk_mm_i");
	dev->mmu_clock=clk_get(NULL,"clk_mmu");
	if((NULL==dev->mmu_mclock)||(NULL==dev->mmu_clock))
		return -1;
	clk_enable(dev->mmu_mclock);
	clk_enable(dev->mmu_clock);
	udelay(300);
	return sprd_iommu_init(dev,data);
}

int sprd_iommu_mm_exit(struct sprd_iommu_dev *dev)
{
	int err=-1;
	err=sprd_iommu_exit(dev);
	clk_disable(dev->mmu_clock);
	clk_disable(dev->mmu_mclock);
	return err;
}

unsigned long sprd_iommu_mm_iova_alloc(struct sprd_iommu_dev *dev, size_t iova_length)
{
	return sprd_iommu_iova_alloc(dev,iova_length);
}

void sprd_iommu_mm_iova_free(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length)
{
	sprd_iommu_iova_free(dev,iova,iova_length);
	return;
}

int sprd_iommu_mm_iova_map(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length, struct ion_buffer *handle)
{
	return sprd_iommu_iova_map(dev,iova,iova_length,handle);
}

int sprd_iommu_mm_iova_unmap(struct sprd_iommu_dev *dev, unsigned long iova, size_t iova_length, struct ion_buffer *handle)
{
	return sprd_iommu_iova_unmap(dev,iova,iova_length,handle);
}

int sprd_iommu_mm_backup(struct sprd_iommu_dev *dev)
{
	int err=-1;
	err=sprd_iommu_backup(dev);
	clk_disable(dev->mmu_clock);
	clk_disable(dev->mmu_mclock);
	return err;
}

int sprd_iommu_mm_restore(struct sprd_iommu_dev *dev)
{
	int err=-1;
	clk_enable(dev->mmu_mclock);
	clk_enable(dev->mmu_clock);
	udelay(300);
	err=sprd_iommu_restore(dev);
	return err;
}

struct sprd_iommu_ops iommu_mm_ops={
	.init=sprd_iommu_mm_init,
	.exit=sprd_iommu_mm_exit,
	.iova_alloc=sprd_iommu_mm_iova_alloc,
	.iova_free=sprd_iommu_mm_iova_free,
	.iova_map=sprd_iommu_mm_iova_map,
	.iova_unmap=sprd_iommu_mm_iova_unmap,
	.backup=sprd_iommu_mm_backup,
	.restore=sprd_iommu_mm_restore,
};

