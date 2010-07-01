#ifndef _SC8800H_REG_AHB_H_
#define _SC8800H_REG_AHB_H_

#include <mach/hardware.h>
#include <mach/bits.h>

#define AHB_REG_BASE                (SPRD_AHB_BASE+0x200)

#define AHB_CTL0                        (AHB_REG_BASE + 0x00)
#define AHB_CTL1                        (AHB_REG_BASE + 0x04)
#define AHB_CTL2                        (AHB_REG_BASE + 0x08)
#define AHB_RESERVED                    (AHB_REG_BASE + 0x0c)
#define AHB_SOFT_RST                    (AHB_REG_BASE + 0x10)
#define AHB_STOP_CTL                    (AHB_REG_BASE + 0x14)
#define AHB_REMAP                       (AHB_REG_BASE + 0x18)
#define AHB_INT_STS                     (AHB_REG_BASE + 0x1c)
#define AHB_INT_CLR                     (AHB_REG_BASE + 0x20)
#define AHB_AHB_ARM_CLK                 (AHB_REG_BASE + 0x24)
#define AHB_BOND_OPT                    (AHB_REG_BASE + 0x28)
#define AHB_TD_CLK                  (AHB_REG_BASE + 0x2c)
/* modified by zhengfei.xiao for SC8800H5 */
#ifdef CONFIG_CHIP_VER_8800H5
#define AHB_MISC                      (AHB_REG_BASE + 0x30)
#else
#define AHB_RFT_CLK                 (AHB_REG_BASE + 0x30)
#endif
#define AHB_DSP_WAKEUP              (AHB_REG_BASE + 0x80)
#define AHB_DSP_BOOT_EN             (AHB_REG_BASE + 0x84)
#define AHB_DSP_BOOT_VECTOR         (AHB_REG_BASE + 0x88)
#define AHB_DSP_RESET               (AHB_REG_BASE + 0x8C)
#define AHB_ARM_POWERDOWN_EN        (AHB_REG_BASE + 0x40)

#define AHB_DMA_SOFT_RST                BIT_0

#define CHIP_TYPE                   (AHB_REG_BASE + 0x03FC)    //0x209003FC

#endif

