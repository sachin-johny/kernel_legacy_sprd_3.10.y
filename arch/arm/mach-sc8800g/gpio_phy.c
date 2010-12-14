#include <asm/io.h>
#include <linux/kernel.h>

#include <mach/regs_gpio.h>
#include "gpio_phy.h"

const static GPIO_SECTION_T  s_gpio_section_table[] =
{
    {   (GPIO_BASE + 0*0x80),    0x8,    GPIO_SECTION_GPI    },
    {   (GPIO_BASE + 1*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 2*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 3*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 4*0x80),    0x10,    GPIO_SECTION_GPIO   },
    {   (GPIO_BASE + 5*0x80),    0x10,    GPIO_SECTION_GPIO   },
    {   (GPIO_BASE + 6*0x80),    0x10,    GPIO_SECTION_GPIO   },
    {   (GPIO_BASE + 7*0x80),    0x10,    GPIO_SECTION_GPIO   },
    {   (GPIO_BASE + 8*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 9*0x80),    0x10,    GPIO_SECTION_GPIO   },
    {   (0x82000600),           0x10,    GPIO_SECTION_GPI    },
    {   (0x82000680),             0x10,    GPIO_SECTION_GPIO  },
    {   (0x82000680 + 0x1*0x80),  0x10,    GPIO_SECTION_GPIO  },
};

GPIO_SECTION_T *Gpio_GetCfgSectionTable (u32 *pSize)
{
    *pSize = ARRAY_SIZE(s_gpio_section_table);

    return (GPIO_SECTION_T *) s_gpio_section_table;
}

