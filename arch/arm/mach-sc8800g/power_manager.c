#include <linux/kernel.h>
#include <linux/io.h>
#include	<linux/irq.h>
#include <linux/bug.h>

#include <mach/bits.h>
#include <mach/regs_global.h>
#include <mach/regs_ahb.h>
#include <mach/power_manager.h>

void PWRMNG_ForcePowerOnVPll(void)
{
	uint32_t i;
	unsigned long flags;

	raw_local_irq_save(flags);

	if (0 == ((__raw_readl(GR_BUSCLK_ALM) & (1<<12)) >>12))
		__raw_bits_and( ~(1<<14), GR_BUSCLK_ALM);
	else
		printk(KERN_ERR "VPLL pd src ctl error!\n");

	raw_local_irq_restore(flags);

	for(i=0; i<5000; i++);
	 pr_info("power on vpll \r\n");
}

void PWRMNG_SetVpllPdSrc (int pdsrc)
{
	unsigned long flags;

	local_irq_save(flags);
	if (pdsrc == ARM_CTL)
	{
	    * (volatile unsigned int *) GR_BUSCLK_ALM &= ~BIT_12;
	}
	else
	{
	    * (volatile unsigned int *) GR_BUSCLK_ALM |= BIT_12;
	}

	local_irq_restore(flags);
	pr_info("set vpll source \r\n");
}

/*****************************************************************************/
// Description :    This function is used to change VPLL clock.
// Global resource dependence :
// Author :         Zhengjiang.Lei
// parameter: 
//					vpll_clk: unit is Hz, like 192000000
/*****************************************************************************/
void PWRMNG_SetVpll(unsigned int vpll_clk)
{
	unsigned long flags;
    // Change VPLL range according HW test result
	//SCI_PASSERT( ((vpll_clk>=112000000)&&(vpll_clk<=402000000)), ("vpll_clk=%d",vpll_clk)  );
    	if ((vpll_clk < 112000000) || (vpll_clk > 402000000)) {
		pr_err("wrong vpll clock %d \r\n", vpll_clk);
		BUG_ON(1);
	}
	local_irq_save(flags);
	
	//Enable change VPLL divider; 
   	* (volatile unsigned int * )GR_GEN1 |= BIT_20;

    if (*(volatile unsigned int *)GR_GEN1 & BIT_15)   // if 26MHz external crystal.
    {
        //Set V_PLL from 26MHz        
        * (volatile unsigned int * )GR_VPLL_MN = ( ((vpll_clk/1000000) << 16) | 0x1A );    
    }
    else
    {
        //Set V_PLL from 13MHz
        * (volatile unsigned int * )GR_VPLL_MN = ( ((vpll_clk/1000000) << 16) | 0xD );
    }
	//Disable change VPLL divider; 
    * (volatile unsigned int * )GR_GEN1 &= ~BIT_20;
    
	local_irq_restore(flags);
}

/* directly copied from chip.c */
unsigned int CHIP_GetVPllClk (void)
{
	uint32_t ext_clk_26M;
	uint32_t pll_freq;
	uint32_t reg_value, M, N;

	ext_clk_26M = (__raw_readl(GR_GEN1) >> 15) & 0x1;

	reg_value = __raw_readl(GR_VPLL_MN);
	M = reg_value & 0x0fff;
	N = (reg_value & 0x0fff0000)>>16;

	if(ext_clk_26M)
		pll_freq = 26*N/M;
	else
		pll_freq = 13*N/M;

	pr_debug("@fool2[%s] GR_VPLL_MN:0x%x, M:%d, N:%d, pll_freq: %d\n", __FUNCTION__, reg_value, M, N, pll_freq);

	return (pll_freq*1000000);
}


void PWRMNG_SetUsbClkSrc(void)
{
	/* Select usb clock source vpll*/
//	usb_clear(BIT_15 | BIT_16, GR_CLK_GEN5);
	__raw_bits_and(~(BIT_15 | BIT_16), GR_CLK_GEN5);
	
//	usb_set(BIT_15, GR_CLK_GEN5);
	__raw_bits_or(BIT_15, GR_CLK_GEN5);
	pr_info("GR_CLK_GEN5 :%x \r\n", __raw_readl(GR_CLK_GEN5));
}

void clk_12M_divider_set(unsigned int pll_clk)
{
	unsigned int  clk_divider;
	unsigned long flags;
	static int set = 0;
	
#define CLK_12M				12000000

	local_irq_save(flags);
	if (!set) {
		set = 1;
		clk_divider = __raw_readl(GR_GEN0);
		clk_divider   &= ~(0x3f<<25);
		clk_divider   |= ( (pll_clk / CLK_12M-1) << 25 );
		__raw_writel(clk_divider, GR_GEN0);
	}
	local_irq_restore(flags);
}
