#ifndef __GPIO_PHY_H__
#define __GPIO_PHY_H__

#include <mach/adi_hal_internal.h>

enum gpio_section_type {
    GPIO_SECTION_GPI = 0x0,
    GPIO_SECTION_GPO,
    GPIO_SECTION_GPIO,
    GPIO_SECTION_INVALID
};


typedef struct
{
    u32 gpxx_pagex_base;
    u32 gpxx_pagex_size;
    int gpxx_section_type;
} GPIO_SECTION_T;


#define ADI_GPIO_ADDR_MASK 0x08000000
static int gpio_is_d_die( u32 reg_addr)
{
	if ((reg_addr > GPIO_BASE) && (reg_addr < GPIO_BASE + 0x500))
		return 1;
	else
		return 0;
}
static __inline u32 GpioCfg_GetBaseAddr (u32 gpio_id)
{
    if (gpio_id >= 160)
    {
       // return ( (gpio_id - 160) >>4) * 0x80 + (unsigned int) 0x82000600;
       return ( (gpio_id - 160) >>4) * 0x80 + ANA_GPIO_BASE;
    }

    return (gpio_id>>4) * 0x80 + (u32) GPIO_BASE;
}

static __inline u32 GpioCfg_GetBitNum (u32 gpio_id)
{
    return (gpio_id & 0xF);
}
static __inline void gpio_chip_reg_set (u32 reg_addr, u32 value)
{
//	pr_info("set reg:%x, value %x\r\n", reg_addr, value);
    if (gpio_is_d_die(reg_addr))
    {
        //CHIP_REG_SET (reg_addr,value);
        __raw_writel(value, reg_addr);
    }
    else
    {
        ANA_REG_SET (reg_addr,value);
    }

    return;
}

static __inline u32 gpio_chip_reg32 (u32 reg_addr)
{
   // if (reg_addr & ADI_GPIO_ADDR_MASK)
 //  pr_info("read reg:%x\r\n", reg_addr);
   if (gpio_is_d_die(reg_addr))
    {
        return __raw_readl (reg_addr);
    }
    else
    {
        return ANA_REG_GET (reg_addr);
    }
}
static __inline void gpio_chip_reg_and (u32 reg_addr, u32 value)
{
   // if (reg_addr & ADI_GPIO_ADDR_MASK)
   if (gpio_is_d_die(reg_addr))
    {
        //CHIP_REG_AND (reg_addr,value);
        __raw_bits_and(value, reg_addr);
    }
    else
    {
        ANA_REG_AND (reg_addr,value);
    }
}
static __inline void gpio_chip_reg_or (u32 reg_addr, u32 value)
{
   // if (reg_addr & ADI_GPIO_ADDR_MASK)
   if (gpio_is_d_die(reg_addr))
    {
       //CHIP_REG_OR (reg_addr,value);
       __raw_bits_or(value, reg_addr);
    }
    else
    {
        ANA_REG_OR (reg_addr,value);
    }
}


#define GPIO_REG_SET(x,y) gpio_chip_reg_set(x,y)
#define GPIO_REG32(x) gpio_chip_reg32(x)
#define GPIO_REG_AND(reg_addr, value) gpio_chip_reg_and(reg_addr, value)
#define GPIO_REG_OR(reg_addr, value) gpio_chip_reg_or(reg_addr, value)

GPIO_SECTION_T *Gpio_GetCfgSectionTable (u32 *pSize);
#endif
