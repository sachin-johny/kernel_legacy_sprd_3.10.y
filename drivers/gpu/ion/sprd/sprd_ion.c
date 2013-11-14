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

#include <linux/err.h>
#include <linux/ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <video/ion_sprd.h>
#include "../ion_priv.h"

#include <asm/cacheflush.h>

struct ion_device *idev;
int num_heaps;
struct ion_heap **heaps;
#if 1
static uint32_t user_va2pa(struct mm_struct *mm, uint32_t addr)
{
        pgd_t *pgd = pgd_offset(mm, addr);
        uint32_t pa = 0;

        if (!pgd_none(*pgd)) {
                pud_t *pud = pud_offset(pgd, addr);
                if (!pud_none(*pud)) {
                        pmd_t *pmd = pmd_offset(pud, addr);
                        if (!pmd_none(*pmd)) {
                                pte_t *ptep, pte;

                                ptep = pte_offset_map(pmd, addr);
                                pte = *ptep;
                                if (pte_present(pte))
                                        pa = pte_val(pte) & PAGE_MASK;
                                pte_unmap(ptep);
                        }
                }
        }

        return pa;
}
#endif

static long sprd_heap_ioctl(struct ion_client *client, unsigned int cmd,
				unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case ION_SPRD_CUSTOM_PHYS:
	{
		struct ion_phys_data data;
		struct ion_handle *handle;

		if (copy_from_user(&data, (void __user *)arg,
				sizeof(data))) {
			return -EFAULT;
		}

		handle = ion_import_dma_buf(client, data.fd_buffer);

		if (IS_ERR(handle))
			return PTR_ERR(handle);

		ret = ion_phys(client, handle, &data.phys, &data.size);
		ion_free(client, handle);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg,
				&data, sizeof(data))) {
			return -EFAULT;
		}
		break;
	}
	case ION_SPRD_CUSTOM_MSYNC:
	{
#if 0
		struct ion_msync_data data;
		void *kaddr;
		void *paddr;
		size_t size;
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(data))) {
			return -EFAULT;
		}
		kaddr = data.vaddr;
		paddr = data.paddr;	
		size = data.size;
		dmac_flush_range(kaddr, kaddr + size);
		outer_clean_range((phys_addr_t)paddr, (phys_addr_t)(paddr + size));

/*maybe open in future if support discrete page map so keep this code unremoved here*/
#else
		struct ion_msync_data data;
		void *v_addr;

		if (copy_from_user(&data, (void __user *)arg,
				sizeof(data))) {
			return -EFAULT;
		}

		if ((int)data.vaddr & (PAGE_SIZE - 1))
			return -EFAULT;

		dmac_flush_range(data.vaddr, data.vaddr + data.size);

		v_addr = data.vaddr;
		while (v_addr < data.vaddr + data.size) {
			uint32_t phy_addr = user_va2pa(current->mm, (uint32_t)v_addr);
			if (phy_addr) {
				outer_clean_range(phy_addr, phy_addr + PAGE_SIZE);
			}
			v_addr += PAGE_SIZE;
		}
#endif
		break;
	}
	default:
		return -ENOTTY;
	}

	return ret;
}


extern struct ion_heap *ion_cma_heap_create(struct ion_platform_heap *heap_data);
extern void ion_cma_heap_destroy(struct ion_heap *heap);


static struct ion_heap *__ion_heap_create(struct ion_platform_heap *heap_data)
{
	struct ion_heap *heap = NULL;

	switch ((int)heap_data->type) {
	case ION_HEAP_TYPE_CUSTOM:
		heap = ion_cma_heap_create(heap_data);
		break;
	default:
		return ion_heap_create(heap_data);
	}

	if (IS_ERR_OR_NULL(heap)) {
		pr_err("%s: error creating heap %s type %d base %lu size %u\n",
		       __func__, heap_data->name, heap_data->type,
		       heap_data->base, heap_data->size);
		return ERR_PTR(-EINVAL);
	}

	heap->name = heap_data->name;
	heap->id = heap_data->id;

	return heap;
}

static void __ion_heap_destroy(struct ion_heap *heap)
{
	if (!heap)
		return;

	switch ((int)heap->type) {
	case ION_HEAP_TYPE_CUSTOM:
		ion_cma_heap_destroy(heap);
		break;
	default:
		ion_heap_destroy(heap);
	}
}

int sprd_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	int err;
	int i;

	num_heaps = pdata->nr;

	heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);

	idev = ion_device_create(&sprd_heap_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < num_heaps; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

		heaps[i] = __ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);
	return 0;
err:
	for (i = 0; i < num_heaps; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}
	kfree(heaps);
	return err;
}

int sprd_ion_remove(struct platform_device *pdev)
{
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(idev);
	for (i = 0; i < num_heaps; i++)
		__ion_heap_destroy(heaps[i]);
	kfree(heaps);
	return 0;
}

static struct platform_driver ion_driver = {
	.probe = sprd_ion_probe,
	.remove = sprd_ion_remove,
	.driver = { .name = "ion-sprd" }
};

static int __init ion_init(void)
{
	return platform_driver_register(&ion_driver);
}

static void __exit ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

module_init(ion_init);
module_exit(ion_exit);

