

#ifndef __ARCH_ARM_MACH_SPRD_COMMON_H
#define __ARCH_ARM_MACH_SPRD_COMMON_H

#include <linux/i2c.h>



extern int sprd_register_i2c_bus(int bus_id,
				 struct i2c_board_info const *info,
				 unsigned len);


#endif /* __ARCH_ARM_MACH_SPRD_COMMON_H */
