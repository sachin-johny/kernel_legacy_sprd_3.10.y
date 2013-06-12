/******************************************************************************
 ** File Name:    sprdfb_chip_8825.c                                     *
 ** Author:       congfu.zhao                                           *
 ** DATE:         30/04/2013                                        *
 ** Copyright:    2013 Spreatrum, Incoporated. All Rights Reserved. *
 ** Description:                                                    *
 ******************************************************************************/
/******************************************************************************
 **                   Edit    History                               *
 **---------------------------------------------------------------------------*
 ** DATE          NAME            DESCRIPTION                       *

 ******************************************************************************/


#include "sprdfb_chip_8825.h"
#include "sprdfb_chip_common.h"


void dsi_enable(void)
{
	sci_glb_set(DSI_BIT_EB, DSI_REG_EB);  //enable dphy

}

void dispc_print_clk(void)
{
	printk("0x20900200 = 0x%x\n", __raw_readl(SPRD_AHB_BASE + 0x200));
	printk("0x20900208 = 0x%x\n", __raw_readl(SPRD_AHB_BASE + 0x208));
	printk("0x20900220 = 0x%x\n", __raw_readl(SPRD_AHB_BASE + 0x220));
}



