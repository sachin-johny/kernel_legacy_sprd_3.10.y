
#ifndef _SPRD_2713_FGU_H_
#define _SPRD_2713_FGU_H_

#include <linux/types.h>
#if defined(CONFIG_ARCH_SCX35)
#include "sprd_2713_charge.h"
#endif
int sprdfgu_init(struct platform_device *pdev);
uint32_t sprdfgu_read_capacity(void);
u32 sprdfgu_read_soc(void);
int sprdfgu_read_batcurrent(void);
uint32_t sprdfgu_read_vbat_vol(void);
uint32_t sprdfgu_read_vbat_ocv(void);
int sprdfgu_is_new_chip(void);
#endif

