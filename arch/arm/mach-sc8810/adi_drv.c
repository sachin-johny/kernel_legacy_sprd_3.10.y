/*
 *  linux/arch/arm/mach-sc8810/adi_drv.c
 *
 *  Spreadtrum adi read write interface
 *
 *
 *  Author:	steve.zhan@spreadtrum.com
 *  Created:	Thu AUG 4, 2011
 *  Copyright:	Spreadtrum Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
*   the first version, only modified and checked ana ldo register.
*/

/**---------------------------------------------------------------------------*
 **                         Dependencies                                                                                                       *
 **---------------------------------------------------------------------------*/
#include <linux/irqflags.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <mach/regs_adi.h>
#include <mach/adi_hal_internal.h>

#define HAVA_ADI_SUPPORT	1
#define CHIP_REG_OR(reg_addr, value)   do{ \
                                             unsigned long flags;   \
                                             hw_local_irq_save(flags);  \
                                             (*(volatile unsigned int *)(reg_addr) |= (unsigned int)(value));   \
                                             hw_local_irq_restore(flags);   \
                                       }while(0)       
#define CHIP_REG_AND(reg_addr, value)   do{ \
                                             unsigned long flags;   \
                                             hw_local_irq_save(flags);  \
                                             (*(volatile unsigned int *)(reg_addr) &= (unsigned int)(value)); \
                                             hw_local_irq_restore(flags);   \
                                       }while(0)
#define CHIP_REG_GET(reg_addr)          (*(volatile unsigned int *)(reg_addr))
#define CHIP_REG_SET(reg_addr, value)   do{ \
                                             unsigned long flags;   \
                                             hw_local_irq_save(flags);  \
                                             (*(volatile unsigned int *)(reg_addr)  = (unsigned int)(value)) ;   \
                                             hw_local_irq_restore(flags);   \
                                       }while(0)

#define SCI_ASSERT(condition) BUG_ON(!(condition))  
#define SCI_PASSERT(condition, format...)  \
	do {		\
		if(!(condition)) { \
			pr_err("function :%s\r\n", __FUNCTION__);\
			BUG();	\
		} \
	}while(0)


/*****************************************************************************
 *  Description:    this function performs read operation to the analog die reg .   *
 *                      it will disable all interrupt and polling the ready bit,        *
 *              and return a half-word value after read complete.             *
 *  Global Resource Dependence:                                              *
 *  Author: Tim Luo                                                        *
 *  Note:   return register value                                               *
******************************************************************************/
unsigned short ADI_Analogdie_reg_read(unsigned int addr)
{
#if HAVA_ADI_SUPPORT
	unsigned int adi_rd_data;
	unsigned long flags;

	hw_local_irq_save(flags);
   // SCI_DisableIRQ();
   // SCI_DisableFIQ();

	//Set read command
	addr = __adi_virt_to_phys(addr);
	
	SCI_ASSERT((addr >= 0x82000040) && (addr <= 0x82000780));
	CHIP_REG_SET(ADI_ARM_RD_CMD, addr);

	//wait read operation complete, RD_data[31] will be cleared after the read operation complete
	do {
		adi_rd_data = CHIP_REG_GET(ADI_RD_DATA);
	} while (adi_rd_data & BIT_31);

	//rd_data high part should be the address of the last read operation
	SCI_ASSERT((adi_rd_data & 0xFFFF0000) == ((addr) << 16));

	//read operation complete
	hw_local_irq_restore(flags);

	return ((unsigned short)(adi_rd_data & 0x0000FFFF));
#else
	return 0;
#endif
}

/*****************************************************************************
 *  Description:    this function performs write operation to the analog die reg .   *
 *                      it will write the analog die register if the fifo is not full       *
 *              It will polling the fifo full status til it is not full                  *
 *  Global Resource Dependence:                                              *
 *  Author: Tim Luo                                                        *
 *  Note:                                                                      *
******************************************************************************/
void ADI_Analogdie_reg_write(unsigned int addr, unsigned short data)
{
#if HAVA_ADI_SUPPORT

	do			////ADI_wait_fifo_empty
	{
		if (((CHIP_REG_GET(ADI_FIFO_STS) &
		      ((unsigned int)ADI_FIFO_EMPTY)) != 0)) {
			break;
		}
	}
	while (1);		/*lint !e506 */

	CHIP_REG_SET(addr, data);
#else
#endif
}

/*****************************************************************************
 *  Description:    this function is used to init analog to digital module.   *
 *                      it will enable adi_acc and soft reset adi_module,        *
 *              and then config the priority of each channel.             *
 *  Global Resource Dependence:                                              *
 *  Author: Tim Luo                                                        *
 *  Note:                                                                                     *
******************************************************************************/
void ADI_init(void)
{
	//enable ADI_ACC to put the adi master to normal operation mode
	CHIP_REG_OR(GR_GEN0, GEN0_ADI_EN);

	//reset ADI module
	CHIP_REG_OR(GR_SOFT_RST, ADI_SOFT_RST);
	{
		unsigned int wait = 50;
		while (wait--) ;
	}
	CHIP_REG_AND(GR_SOFT_RST, (~ADI_SOFT_RST));

	//Please refer to Section 5. Program guide, SC8800G Analog-Digital Interface module Implementation Specifications.doc
	CHIP_REG_AND(ADI_CTL_REG, (~ARM_SERCLK_EN));

	//config channel priority
	CHIP_REG_SET(ADI_CHANNEL_PRI,
		     ((0 << INT_STEAL_PRI) | (1 << STC_WR_PRI) |
		      (0 << ARM_WR_PRI)
		      | (0 << ARM_RD_PRI) | (0 << DSP_WR_PRI) | (0 <<
								 DSP_RD_PRI)
		      | (1 << RFT_WR_PRI) | (1 << PD_WR_PRI)));

}

EXPORT_SYMBOL(ADI_Analogdie_reg_write);
EXPORT_SYMBOL(ADI_Analogdie_reg_read);
