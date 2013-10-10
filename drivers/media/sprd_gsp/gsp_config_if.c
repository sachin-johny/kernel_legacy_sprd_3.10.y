/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**---------------------------------------------------------------------------*
**                         Dependencies                                      *
**---------------------------------------------------------------------------*/
//#include <linux/irq.h>

#include <mach/hardware.h>

#include "gsp_config_if.h"
uint32_t testregsegment[0x190]= {0};
extern struct clk	*g_gsp_emc_clk;

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                         Macro Definition                                  *
 **---------------------------------------------------------------------------*/


/**---------------------------------------------------------------------------*
 **                         Function Define                                   *
 **---------------------------------------------------------------------------*/

extern GSP_CONFIG_INFO_T s_gsp_cfg;


LOCAL void GSP_SetLayer0Parameter(void)
{
    if(!s_gsp_cfg.layer0_info.layer_en)
    {
        GSP_L0_ENABLE_SET(0);
        return ;
    }

    GSP_L0_ADDR_SET(s_gsp_cfg.layer0_info.src_addr);
    GSP_L0_PITCH_SET(s_gsp_cfg.layer0_info.pitch);
    GSP_L0_CLIPRECT_SET(s_gsp_cfg.layer0_info.clip_rect);
    GSP_L0_DESRECT_SET(s_gsp_cfg.layer0_info.des_rect);
    GSP_L0_GREY_SET(s_gsp_cfg.layer0_info.grey);
    GSP_L0_ENDIAN_SET(s_gsp_cfg.layer0_info.endian_mode);
    GSP_L0_ALPHA_SET(s_gsp_cfg.layer0_info.alpha);
    GSP_L0_COLORKEY_SET(s_gsp_cfg.layer0_info.colorkey);
    GSP_L0_IMGFORMAT_SET(s_gsp_cfg.layer0_info.img_format);
    GSP_L0_ROTMODE_SET(s_gsp_cfg.layer0_info.rot_angle);
    GSP_L0_COLORKEYENABLE_SET(s_gsp_cfg.layer0_info.colorkey_en);
    GSP_L0_PALLETENABLE_SET(s_gsp_cfg.layer0_info.pallet_en);
    //GSP_L0_SCALETAPMODE_SET(s_gsp_cfg.layer0_info.row_tap_mode,s_gsp_cfg.layer0_info.col_tap_mode);

}


LOCAL void GSP_SetLayer1Parameter(void)
{
    if(!s_gsp_cfg.layer1_info.layer_en)
    {
        GSP_L1_ENABLE_SET(0);
        return ;
    }

    GSP_L1_ADDR_SET(s_gsp_cfg.layer1_info.src_addr);
    GSP_L1_PITCH_SET(s_gsp_cfg.layer1_info.pitch);
    GSP_L1_CLIPRECT_SET(s_gsp_cfg.layer1_info.clip_rect);
    GSP_L1_DESPOS_SET(s_gsp_cfg.layer1_info.des_pos);
    GSP_L1_GREY_SET(s_gsp_cfg.layer1_info.grey);
    GSP_L1_ENDIAN_SET(s_gsp_cfg.layer1_info.endian_mode);
    GSP_L1_ALPHA_SET(s_gsp_cfg.layer1_info.alpha);
    GSP_L1_COLORKEY_SET(s_gsp_cfg.layer1_info.colorkey);
    GSP_L1_IMGFORMAT_SET(s_gsp_cfg.layer1_info.img_format);
    GSP_L1_ROTMODE_SET(s_gsp_cfg.layer1_info.rot_angle);
    GSP_L1_COLORKEYENABLE_SET(s_gsp_cfg.layer1_info.colorkey_en);
    GSP_L1_PALLETENABLE_SET(s_gsp_cfg.layer1_info.pallet_en);

}


LOCAL void GSP_SetLayerDesParameter(void)
{
    if(!s_gsp_cfg.layer0_info.layer_en && !s_gsp_cfg.layer1_info.layer_en)
    {
        return ;
    }

    GSP_Ld_ADDR_SET(s_gsp_cfg.layer_des_info.src_addr);
    GSP_Ld_PITCH_SET(s_gsp_cfg.layer_des_info.pitch);
    GSP_Ld_ENDIAN_SET(s_gsp_cfg.layer_des_info.endian_mode);
    GSP_Ld_IMGFORMAT_SET(s_gsp_cfg.layer_des_info.img_format);
    GSP_Ld_COMPRESSRGB888_SET(s_gsp_cfg.layer_des_info.compress_r8_en);
}

LOCAL void GSP_SetMiscParameter(void)
{
    if(!s_gsp_cfg.layer0_info.layer_en && !s_gsp_cfg.layer1_info.layer_en)
    {
        return ;
    }

    GSP_L0_ENABLE_SET(s_gsp_cfg.layer0_info.layer_en);
    GSP_L1_ENABLE_SET(s_gsp_cfg.layer1_info.layer_en);

    if(s_gsp_cfg.layer0_info.scaling_en == 1)
    {
        GSP_SCALESTATUS_RESET();
    }
    GSP_SCALE_ENABLE_SET(s_gsp_cfg.layer0_info.scaling_en);


    GSP_PMARGB_ENABLE_SET(s_gsp_cfg.layer0_info.pmargb_en||s_gsp_cfg.layer1_info.pmargb_en);
    GSP_L0_PMARGBMODE_SET(s_gsp_cfg.layer0_info.pmargb_mod);
    GSP_L1_PMARGBMODE_SET(s_gsp_cfg.layer1_info.pmargb_mod);


    GSP_DITHER_ENABLE_SET(s_gsp_cfg.misc_info.dithering_en);
    //GSP_AHB_CLOCK_SET(s_gsp_cfg.misc_info.ahb_clock);
    GSP_CLOCK_SET(s_gsp_cfg.misc_info.gsp_clock);
    GSP_EMC_GAP_SET(s_gsp_cfg.misc_info.gsp_gap);
}

PUBLIC void GSP_Init(void)
{
    int ret = 0;
    ret = clk_enable(g_gsp_emc_clk);
    if(ret) {
        printk(KERN_ERR "%s: enable emc clock failed!\n",__FUNCTION__);
        return;
    } else {
        pr_debug(KERN_INFO "%s: enable emc clock ok!\n",__FUNCTION__);
    }
    GSP_HWMODULE_ENABLE();
    GSP_HWMODULE_SOFTRESET();

    /* move to module init
        GSP_EMC_MATRIX_ENABLE();
        GSP_EMC_GAP_SET(0);
        GSP_CLOCK_SET(GSP_CLOCK_256M_BIT);
        GSP_AUTO_GATE_ENABLE();
        //GSP_AHB_CLOCK_SET(GSP_AHB_CLOCK_26M_BIT);
        *(volatile unsigned long *)(SPRD_PMU_BASE+0x1c) &= ~(1<<25);
        *(volatile unsigned long *)(SPRD_AONAPB_BASE) |= (1<<25);   //     MM enable
    */
    GSP_IRQMODE_SET(GSP_IRQ_MODE_LEVEL);
}
PUBLIC void GSP_Deinit(void)
{
    clk_disable(g_gsp_emc_clk);
    GSP_IRQSTATUS_CLEAR();
    GSP_IRQENABLE_SET(GSP_IRQ_TYPE_DISABLE);
    GSP_HWMODULE_DISABLE();//disable may not use the enable regiter
}

PUBLIC void GSP_ConfigLayer(GSP_MODULE_ID_E layer_id)
{
    switch(layer_id)
    {
    case GSP_MODULE_LAYER0:
        GSP_SetLayer0Parameter();
        break;

    case GSP_MODULE_LAYER1:
        GSP_SetLayer1Parameter();
        break;

    case GSP_MODULE_DST:
        GSP_SetLayerDesParameter();
        break;

    default:
        GSP_SetMiscParameter();
        break;
    }
}

PUBLIC void GSP_Wait_Finish(void)
{
    while(1)
    {
        if(GSP_WORKSTATUS_GET() == 0)
        {
            break;
        }
    }
}


PUBLIC uint32_t GSP_Trigger(void)
{
    if(GSP_ERRFLAG_GET())
    {
        //GSP_ASSERT();
        return GSP_ERRCODE_GET();
    }

    GSP_IRQENABLE_SET(GSP_IRQ_TYPE_ENABLE);
    GSP_ENGINE_TRIGGER();
    return 0;
}

