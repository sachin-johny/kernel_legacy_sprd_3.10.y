#include <asm/io.h>
#include <linux/kernel.h>

#include <mach/regs_gpio.h>
#include "gpio_phy.h"

const static struct gpio_section  s_gpio_section_table[] = {

    {   (GPIO_BASE + 0*0x80),    0x10,    GPIO_SECTION_GPIO    },//from 16
    {   (GPIO_BASE + 1*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 2*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 3*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 4*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 5*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 6*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 7*0x80),    0x10,    GPIO_SECTION_GPIO    },
    {   (GPIO_BASE + 8*0x80),    0x10,    GPIO_SECTION_GPIO    },//to 159

    {   (SPRD_MISC_BASE + 0x480),   0x10,    GPIO_SECTION_GPIO    },//from 176
    {   (SPRD_MISC_BASE + 0x4c0),   0xe,    GPIO_SECTION_GPIO       },//to 205
};

struct gpio_section * gpio_get_section_table (u32 *table_size)
{
    *table_size = ARRAY_SIZE(s_gpio_section_table);

    return (struct gpio_section *) s_gpio_section_table;
}

