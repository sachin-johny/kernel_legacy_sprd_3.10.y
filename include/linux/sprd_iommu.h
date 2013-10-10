#ifndef _SPRD_IOMMU_H
#define _SPRD_IOMMU_H

#include <linux/ion.h>

#define MMU_START_MB_ADDR(_x_)		( (_x_) << 20 & (BIT(20)|BIT(21)|BIT(22)|BIT(23)|BIT(24)|BIT(25)|BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)|BIT(31)) )
#define MMU_RAMCLK_DIV2_EN(_x_)		( (_x_) << 2 & (BIT(2)) )
#define MMU_TLB_EN(_x_)				( (_x_) << 1 & (BIT(1)) )
#define MMU_EN(_x_)					( (_x_) << 0 & (BIT(0)) )

struct sprd_iommu_init_data {
	int id;                        /* iommu id */
	char *name;                    /* iommu name */
	
	uint32_t iova_base;            /* io virtual address base */
	uint32_t iova_size;            /* io virtual address size */
	uint32_t pgt_base;             /* iommu page table base address */
	uint32_t pgt_size;             /* iommu page table array size */
};

struct sprd_iommu_dev {
	struct sprd_iommu_init_data *init_data;
	struct gen_pool *pool;
	struct sprd_iommu_ops *ops;
	uint32_t *pgt;
	struct mutex mutex_pgt;
	void *private;
};

struct sprd_iommu_ops {
	int (*init)(struct sprd_iommu_dev *dev, struct sprd_iommu_init_data *data);
	int (*exit)(struct sprd_iommu_dev *dev);
	uint32_t (*iova_alloc)(struct sprd_iommu_dev *dev, uint32_t size);
	void (*iova_free)(struct sprd_iommu_dev *dev, uint32_t addr, uint32_t size);
	uint32_t (*iova_map)(struct sprd_iommu_dev *dev,uint32_t addr, uint32_t size, struct ion_buffer *handle);
	uint32_t (*iova_unmap)(struct sprd_iommu_dev *dev, uint32_t addr, uint32_t size, struct ion_buffer *handle);
	int (*backup)(struct sprd_iommu_dev *dev);
	int (*restore)(struct sprd_iommu_dev *dev);
	int (*dump)(struct sprd_iommu_dev *dev, uint32_t addr, uint32_t size);
};

enum IOMMU_ID {
	IOMMU_GSP = 0,
	IOMMU_MM,
	IOMMU_MAX,
};

extern uint32_t sprd_iova_alloc(uint32_t iommu_id, uint32_t size);
extern void sprd_iova_free(uint32_t iommu_id,uint32_t iova, uint32_t size);
extern uint32_t sprd_iova_map(uint32_t iommu_id,uint32_t iova, struct ion_buffer *handle);
extern uint32_t sprd_iova_unmap(uint32_t iommu_id,uint32_t iova,  struct ion_buffer *handle);
#endif
