/******************************************************************************
 ** File Name:    sprdfb_chip_8830.c                                     *
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


#include "sprdfb_chip_8830.h"
#include "sprdfb_chip_common.h"


void dsi_enable(void)
{
	sci_glb_set(REG_AP_AHB_MISC_CKG_EN, BIT_DPHY_REF_CKG_EN);
	sci_glb_set(REG_AP_AHB_MISC_CKG_EN, BIT_DPHY_CFG_CKG_EN);
	sci_glb_set(DSI_REG_EB, DSI_BIT_EB);
	
//	__raw_writel(__raw_readl(REG_AP_AHB_MISC_CKG_EN) | (BIT_DPHY_REF_CKG_EN) | (BIT_DPHY_CFG_CKG_EN) , REG_AP_AHB_MISC_CKG_EN);
//	__raw_bits_or(DSI_BIT_EB, DSI_REG_EB);
	printk("REG_AP_AHB_MISC_CKG_EN:%x \n",__raw_readl(REG_AP_AHB_MISC_CKG_EN));
	printk("REG_AP_AHB_MISC_CKG_EN:%x \n",REG_AP_AHB_MISC_CKG_EN);
	printk("DSI_REG_EB:%x \n",__raw_readl(DSI_REG_EB));
	printk("DSI_REG_EB:%x \n",DSI_REG_EB);
	printk("BIT_DPHY_REF_CKG_EN:%x,BIT_DPHY_CFG_CKG_EN:%x \n",BIT_DPHY_REF_CKG_EN,BIT_DPHY_CFG_CKG_EN);
}

void dispc_print_clk(void)
{
	printk("sprdfb:0x402e0004 = 0x%x\n", __raw_readl((SPRD_AONAPB_BASE+0x4)));
	printk("sprdfb:0x20d00000 = 0x%x\n", __raw_readl(SPRD_AHB_BASE));
	printk("sprdfb:0x71300000 = 0x%x\n", __raw_readl(SPRD_APBREG_BASE));
	printk("sprdfb:0x71200034 = 0x%x\n", __raw_readl((SPRD_APBCKG_BASE+0x34)));
	printk("sprdfb:0x71200030 = 0x%x\n", __raw_readl((SPRD_APBCKG_BASE+0x30)));
	printk("sprdfb:0x7120002c = 0x%x\n", __raw_readl((SPRD_APBCKG_BASE+0x2c)));
}




