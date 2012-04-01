/*
* drivers/media/video/sprd_scale/scale_sc8810.c
 * Scale driver based on sc8810
 *
 * Copyright (C) 2010 Spreadtrum 
 * 
 * Author:Jianping wang <jianping.wang@spreadtrum.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <mach/clock_common.h>
#include <linux/clk.h>
#include <linux/err.h>

#include "scale_sc8800g2.h"
#include "scale_drv_sc8810.h"
#include "../sprd_dcam/sc8810/isp_control.h"


//#define ISP_DRV_SCALE_COEFF_DBG

#define ISP_PATH1 1 
#define ISP_PATH2 2
#define ISP_DRV_SCALE_COEFF_BUF_SIZE          (8*1024)
#define ISP_DRV_SCALE_COEFF_TMP_SIZE         (6*1024)
#define ISP_DRV_SCALE_COEFF_COEF_SIZE        (1*1024)
#define SCI_NULL 0 
#define SCI_MEMSET      memset
#define SCI_MEMCPY	memcpy


#include <mach/dma.h>
#include <linux/sched.h>

static int32_t _SCALE_DriverPath2Config(ISP_CFG_ID_E id, void* param);

typedef enum
{
	SCALE_CLK_128M = 0,
	SCALE_CLK_76M8,
	SCALE_CLK_64M,
	SCALE_CLK_48M
} SCALE_CLK_SEL_E;

#define SCALE_MINOR MISC_DYNAMIC_MINOR

static struct mutex *lock;

#define SA_SHIRQ IRQF_SHARED
#define SCALE_CHECK_PARAM_ZERO_POINTER(n)                do{if(0 == (int)(n)) return ISP_DRV_RTN_PARA_ERR;}while(0)
#define ISP_RTN_IF_ERR(n)                                                    if(ISP_DRV_RTN_SUCCESS != n)  return n
#define SCALE_PATH_ADDR_INVALIDE(addr)                  (NULL != (addr)) 
#define SCALE_PATH_YUV_ADDR_INVALIDE(y,u,v)         (SCALE_PATH_ADDR_INVALIDE(y) && SCALE_PATH_ADDR_INVALIDE(u) && SCALE_PATH_ADDR_INVALIDE(v))

//#define SCALE_DEBUG
#ifdef SCALE_DEBUG
#define SCALE_PRINT printk
#else
#define SCALE_PRINT(...)
#endif
#define SCALE_PRINT_ERR printk
static ISP_MODULE_T       s_scale_mod;
SCALE_MODE_E g_scale_mode = SCALE_MODE_SCALE;

typedef enum
{
	ISR_DONE = 0x0,
	CALL_HISR = 0x5a5
}ISR_EXE_T;

typedef struct zoom_dma_buf{
	uint32_t by_dma;
	uint32_t in_y_addr;
	uint32_t out_y_addr;
	uint32_t in_uv_addr;
	uint32_t out_uv_addr;	
	uint32_t width;
	uint32_t last_line_cnt;
}ZOOM_DMA_BUF;

#define SCALE_BUFFER_ALIGNED 0x4
#define ZOOM_BUF_ALIGNED(x) ((x + SCALE_BUFFER_ALIGNED - 1) & ~(SCALE_BUFFER_ALIGNED - 1))

#define IRQ_LINE_DCAM  27       //irq line number in system
#define NR_DCAM_ISRS    12
struct semaphore g_sem;

static int g_scale_num = 0;//store the time opened.
static uint32_t g_share_irq = 0xFF; //for share irq handler function
ZOOM_DMA_BUF g_zoom_dma_buf;	//use it when the zoom buf address is not aligned by 64 words.
struct clk *g_scale_clk = NULL; //for power manager

static int _SCALE_DriverSetMclk(SCALE_CLK_SEL_E clk_sel)
{
    	char *name_parent = NULL;
    	struct clk *clk_parent = NULL;
	int ret;

	switch(clk_sel)
	{
		case SCALE_CLK_128M:
			name_parent = "clk_128m";
			break;
		case SCALE_CLK_76M8:
			name_parent = "clk_76m800k";
			break;		
		case SCALE_CLK_64M:
			name_parent = "clk_64m";
			break;
		default:
			name_parent = "clk_48m";
			break;
	}
	
	clk_parent = clk_get_parent(g_scale_clk);
	SCALE_PRINT("SCALE:_SCALE_DriverSetMclk,clock[%s]: parent_name: %s.\n", g_scale_clk->name, clk_parent->name);
	if(strcmp(name_parent, clk_parent->name))
	{//need to wait the parent
		clk_parent = clk_get(NULL, name_parent);
		if(!clk_parent)
		{
			SCALE_PRINT_ERR("SCALE:_SCALE_DriverSetMclk,clock[%s]: failed to get parent [%s] by clk_get()!\n", g_scale_clk->name, name_parent);
			return -EINVAL;
		}

		ret = clk_set_parent(g_scale_clk, clk_parent);
		if(ret)
		{
			SCALE_PRINT_ERR("SCALE:_SCALE_DriverSetMclk,clock[%s]: clk_set_parent() failed!parent: %s, usecount: %d.\n", g_scale_clk->name, clk_parent->name, g_scale_clk->usecount);
			return -EINVAL;
		}		
	}
	ret = clk_enable(g_scale_clk);
	if(ret)
	{
		SCALE_PRINT_ERR("SCALE:_SCALE_DriverSetMclk,clock[%s]: clk_enable() failed!\n", g_scale_clk->name);
	}
	else
	{
		SCALE_PRINT("SCALE:_SCALE_DriverSetMclk, g_scale_clk clk_enable ok.\n");
	}	

	return 0;	
}
#ifdef SCALE_DEBUG //for debug
void get_scale_reg(void)
{
	uint32_t i, value;
	for(i = 0; i < 29; i++)
	{
		value = _pard(DCAM_REG_BASE + i * 4);
		SCALE_PRINT("SCALE reg:0x%x, 0x%x.\n", DCAM_REG_BASE + i * 4, value);
	}
	for(i = 0; i < 9; i++)
	{
		value = _pard(DCAM_REG_BASE + 0x0100 + i * 4);
		SCALE_PRINT("SCALE reg:0x%x, 0x%x.\n", DCAM_REG_BASE + 0x0100 + i * 4, value);
	}
}
#endif
static void _SCALE_DriverSetExtSrcFrameAddr(ISP_FRAME_T *p_frame) 
{
	ISP_REG_T  *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	
	p_isp_reg->frm_addr_0_u.dwValue = p_frame->yaddr;
	
	if(s_scale_mod.isp_path2.input_format == ISP_DATA_YUV422 ||       
	    s_scale_mod.isp_path2.input_format == ISP_DATA_YUV420 || 
	    s_scale_mod.isp_path2.input_format == ISP_DATA_YUV420_3FRAME) 
	{	
		p_isp_reg->frm_addr_1_u.dwValue = p_frame->uaddr;

		if(s_scale_mod.isp_path2.input_format == ISP_DATA_YUV420_3FRAME)
		{			
			p_isp_reg->frm_addr_2_u.dwValue = p_frame->vaddr;
		}
	}
}

static void _SCALE_DriverSetExtDstFrameAddr(ISP_FRAME_T *p_frame) 
{
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	
	p_isp_reg->frm_addr_4_u.dwValue = p_frame->yaddr;

	if(s_scale_mod.isp_path2.output_format == ISP_DATA_YUV422 || 	
	     s_scale_mod.isp_path2.output_format == ISP_DATA_YUV420) 
	{		
		p_isp_reg->frm_addr_5_u.dwValue = p_frame->uaddr;
	}
	SCALE_PRINT("SCALE:s_scale_mod.isp_path2.output_format=%d\n",	
		                    s_scale_mod.isp_path2.output_format );
	SCALE_PRINT("SCALE:_SCALE_DriverSetExtDstFrameAddr,0x%x,0x%x\n",
		                     p_isp_reg->frm_addr_4_u.dwValue,
		                     p_isp_reg->frm_addr_5_u.dwValue);	
}

static uint32_t _SCALE_DriverGetSubSampleFactor(uint32_t* src_width, 
												                                       uint32_t* src_height,
												                                       uint32_t dst_width,
												                                       uint32_t dst_height)
{
	if(*src_width > (dst_width * ISP_PATH_SC_COEFF_MAX) ||  *src_height > (dst_height * ISP_PATH_SC_COEFF_MAX) )
	{
		*src_width = *src_width >> 1;
		*src_height = *src_height >> 1;
		return _SCALE_DriverGetSubSampleFactor(src_width,src_height,dst_width,dst_height) + ISP_PATH_SUB_SAMPLE_FACTOR_BASE;
	}
	else
	{
		return ISP_DRV_RTN_SUCCESS;
	}
}
static int32_t _SCALE_DriverCalcSC2Size(void)
{
	uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
	ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;

	//printk("_SCALE_DriverCalcSC2Size,intput rect:%d,%d,%d,%d,output:%d,%d .\n",p_path->input_rect.x,p_path->input_rect.y,p_path->input_rect.w,p_path->input_rect.h,
	//	  p_path->output_size.w,p_path->output_size.h);

	if(p_path->input_rect.w * ISP_PATH_SC_COEFF_MAX < p_path->output_size.w ||		
	    p_path->input_rect.h * ISP_PATH_SC_COEFF_MAX < p_path->output_size.h) 
	{
		rtn = ISP_DRV_RTN_PARA_ERR;
	}
	else if(p_path->input_rect.w > p_path->output_size.w * ISP_PATH_SC_COEFF_MAX ||
		  p_path->input_rect.h > p_path->output_size.h * ISP_PATH_SC_COEFF_MAX)
	{
		p_path->sc_input_size.w = p_path->input_rect.w;
		p_path->sc_input_size.h = p_path->input_rect.h;		

		p_path->sub_sample_factor = _SCALE_DriverGetSubSampleFactor(&p_path->sc_input_size.w,
		                                                      &p_path->sc_input_size.h,
		                                                      p_path->output_size.w,
		                                                      p_path->output_size.h);

		if(((s_scale_mod.isp_mode == ISP_MODE_MPEG || ISP_MODE_PREVIEW_EX == s_scale_mod.isp_mode) &&
		     p_path->sub_sample_factor > ISP_PATH_SUB_SAMPLE_FACTOR_BASE ) ||  
		     p_path->sub_sample_factor > (ISP_PATH_SUB_SAMPLE_MAX + ISP_PATH_SUB_SAMPLE_FACTOR_BASE))
		{
			rtn = ISP_DRV_RTN_PARA_ERR;
		}
		else
		{
			p_path->sc_input_size.w = p_path->input_rect.w / ( 1 << p_path->sub_sample_factor );
			p_path->sc_input_size.h = p_path->input_rect.h / ( 1 << p_path->sub_sample_factor );			
			p_path->sub_sample_en = 1;
			p_path->sub_sample_mode = p_path->sub_sample_factor - ISP_PATH_SUB_SAMPLE_FACTOR_BASE;
		}
	}
	else
	{
		p_path->sc_input_size.w = p_path->input_rect.w;
		p_path->sc_input_size.h = p_path->input_rect.h;		
	}

	return rtn;
}


static int32_t _ISP_DriverGenScxCoeff(uint32_t idxScx)
{  
	ISP_PATH_DESCRIPTION_T *p_path = SCI_NULL;
	uint32_t i = 0;
	uint32_t HScaleAddr   = 0;
	uint32_t VScaleAddr   = 0;	
	uint32_t *pTmpBuf  = SCI_NULL;
	uint32_t *pHCoeff  = SCI_NULL;	
	uint32_t *pVCoeff   = SCI_NULL;
	ISP_REG_T   *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	if(ISP_PATH1 != idxScx && ISP_PATH2 != idxScx)
		return ISP_DRV_RTN_PARA_ERR;

#ifdef ISP_DRV_SCALE_COEFF_DBG
	SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff Entry");
#endif

	if(ISP_PATH1 == idxScx)
	{
		p_path = &s_scale_mod.isp_path1;
		HScaleAddr   = s_scale_mod.module_addr + ISP_SCALE1_H_TAB_OFFSET;
		VScaleAddr   = s_scale_mod.module_addr + ISP_SCALE1_V_TAB_OFFSET;        
	}
	else 
	{
		p_path = &s_scale_mod.isp_path2;        
		HScaleAddr   = s_scale_mod.module_addr + ISP_SCALE2_H_TAB_OFFSET;
		VScaleAddr   = s_scale_mod.module_addr + ISP_SCALE2_V_TAB_OFFSET;  
	}
    
	pTmpBuf = (uint32_t *)kmalloc(ISP_DRV_SCALE_COEFF_BUF_SIZE, GFP_KERNEL);
	if(SCI_NULL == pTmpBuf)
	{
		printk("SCALE ERROR:kmalloc is fail!");
		return ISP_DRV_RTN_KMALLOC_BUF_ERR;
	}
	
	SCI_MEMSET(pTmpBuf, 0, ISP_DRV_SCALE_COEFF_BUF_SIZE);

	pHCoeff = pTmpBuf;
	pVCoeff = pTmpBuf + (ISP_DRV_SCALE_COEFF_COEF_SIZE/4);

#ifdef ISP_DRV_SCALE_COEFF_DBG
	SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff i_w/i_h/o_w/o_h = {%d, %d, %d, %d,}",
				(int16_t)p_path->sc_input_size.w,
				(int16_t)p_path->sc_input_size.h,
				(int16_t)p_path->output_size.w,
				(int16_t)p_path->output_size.h
				);    
#endif

	if(!(GenScaleCoeff((int16_t)p_path->sc_input_size.w, 
	    	(int16_t)p_path->sc_input_size.h, 
	   	(int16_t)p_path->output_size.w,  
	    	(int16_t)p_path->output_size.h, 
		pHCoeff, 
		pVCoeff, 
		pTmpBuf + (ISP_DRV_SCALE_COEFF_COEF_SIZE/4*2), 
		ISP_DRV_SCALE_COEFF_TMP_SIZE)))
	{
		printk("SCALE ERROR: _ISP_DriverGenScxCoeff GenScaleCoeff error!");    
		kfree(pTmpBuf);
		pTmpBuf = SCI_NULL;
		return ISP_DRV_RTN_GEN_SCALECOEFF_ERR;
	}
	
	if(ISP_PATH1 == idxScx)
	{    
		do
		{   				  
		   	 p_isp_reg->dcam_cfg_u.mBits.path1_clock_switch = ISP_CLK_DOMAIN_AHB;
		}while(!(p_isp_reg->dcam_cfg_u.mBits.path1_clock_status));		 

#ifdef ISP_DRV_SCALE_COEFF_DBG
		SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff Domain = 0x%x", _pard(DCAM_CFG) >> 3) & 0x1);
#endif
	}
	else 
	{
		do
		{               
		    	p_isp_reg->dcam_cfg_u.mBits.path2_clock_switch = ISP_CLK_DOMAIN_AHB;
		}while(!(p_isp_reg->dcam_cfg_u.mBits.path2_clock_status));	

#ifdef ISP_DRV_SCALE_COEFF_DBG
		SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff Domain = 0x%x", _pard(DCAM_CFG) >> 4) & 0x1);
#endif
	}   
        
	for( i = 0; i < ISP_SCALE_COEFF_H_NUM; i++)
	{
		_pawd(HScaleAddr, *pHCoeff);

#ifdef ISP_DRV_SCALE_COEFF_DBG
		SCALE_PRINT("SCALE: Coeff H[%d] = 0x%x.\n", i, *pHCoeff); 
#endif
		HScaleAddr += 4;
		pHCoeff++;
	}    
    
	for( i=0 ;i < ISP_SCALE_COEFF_V_NUM; i++)
	{
		_pawd(VScaleAddr, *pVCoeff);        
#ifdef ISP_DRV_SCALE_COEFF_DBG
		SCALE_PRINT("SCALE: Coeff V[%d] = 0x%x.\n", i,  *pVCoeff);
#endif
		VScaleAddr += 4;
		pVCoeff++;
	}

	if(ISP_PATH1 == idxScx)
	{
		p_isp_reg->dcam_path_cfg_u.mBits.ver_down_tap =  (*pVCoeff) & 0x0F;
	}
	else
	{    	
		p_isp_reg->rev_path_cfg_u.mBits.ver_down_tap = (*pVCoeff) & 0x0F;
	}

#ifdef ISP_DRV_SCALE_COEFF_DBG
	SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff V[%d] = 0x%x", i,  (*pVCoeff) & 0x0F);
	SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff V[%d] = 0x%x", i,  p_isp_reg->rev_path_cfg_u.mBits.ver_down_tap);
#endif
    
	if(ISP_PATH1 == idxScx)        
	{      	
		p_isp_reg->dcam_cfg_u.mBits.path1_clock_switch = ISP_CLK_DOMAIN_DCAM;
#ifdef ISP_DRV_SCALE_COEFF_DBG
		SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff Domain = 0x%x", p_isp_reg->dcam_cfg_u.mBits.path1_clock_status);
#endif
	}
	else
	{	
		p_isp_reg->dcam_cfg_u.mBits.path2_clock_switch = ISP_CLK_DOMAIN_DCAM;
#ifdef ISP_DRV_SCALE_COEFF_DBG
		SCALE_PRINT("SCALE: _ISP_DriverGenScxCoeff Domain = 0x%x",  p_isp_reg->dcam_cfg_u.mBits.path2_clock_status);
#endif
	}
    
	kfree(pTmpBuf);
	pTmpBuf = SCI_NULL;
    
	return ISP_DRV_RTN_SUCCESS;
}

static int32_t _SCALE_DriverSetSC2Coeff(void)
{
	return _ISP_DriverGenScxCoeff( ISP_PATH2);	
}

static int32_t _SCALE_DriverPath2TrimAndScaling(void)
{
	uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
	ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
	ISP_REG_T  *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	
	/*trim config*/
	if(p_path->input_size.w != p_path->input_rect.w || 	p_path->input_size.h != p_path->input_rect.h)
	{
		p_isp_reg->rev_path_cfg_u.mBits.trim_eb = 1;
	}
	else
	{
		p_isp_reg->rev_path_cfg_u.mBits.trim_eb = 0;
	}

	/*scaling config*/
	rtn = _SCALE_DriverCalcSC2Size(); 
	ISP_RTN_IF_ERR(rtn);	
    
	if(p_path->sub_sample_en)
	{
		p_isp_reg->rev_path_cfg_u.mBits.sub_sample_eb = 1;			
		p_isp_reg->rev_path_cfg_u.mBits.sub_sample_mode = p_path->sub_sample_mode;				
	}
	else
	{
		p_isp_reg->rev_path_cfg_u.mBits.sub_sample_eb = 0;
	}
	
	if(p_path->sc_input_size.w != p_path->output_size.w ||p_path->sc_input_size.h != p_path->output_size.h)
	{
		p_isp_reg->rev_path_cfg_u.mBits.scale_bypass = 0;
		rtn = _SCALE_DriverSetSC2Coeff(); 
	}
	else
	{
		p_isp_reg->rev_path_cfg_u.mBits.scale_bypass = 1;
	}

	if(p_path->slice_en)
	{
		p_isp_reg->rev_path_cfg_u.mBits.scale_mode = 1;
	}
	else
	{
		p_isp_reg->rev_path_cfg_u.mBits.scale_mode = 0;		
	}
	return rtn;
}

static  void _SCALE_DriverIrqEnable(uint32_t mask)
{
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	p_isp_reg->dcam_int_mask_u.dwValue |= mask;
}

static void _SCALE_DriverForceCopy(void)
{
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	
	p_isp_reg->dcam_path_cfg_u.mBits.frc_copy_cap = 1;
	p_isp_reg->dcam_path_cfg_u.mBits.frc_copy_cap = 1;
	p_isp_reg->dcam_path_cfg_u.mBits.frc_copy_cap = 0;
}

static void _SCALE_DriverIramSwitch(uint32_t base_addr,uint32_t isp_or_arm)
{
	if(isp_or_arm == 0)    
	{
		_paad(base_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, ~BIT_0);
	}
	else
	{
		_paod(base_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, BIT_0);
	}
}

static  void   _SCALE_DrvierModuleReset(uint32_t base_addr)
{
	_paod(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET, BIT_1 | BIT_2);
	_paod(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET, BIT_1 | BIT_2);
	_paad(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET, ~(BIT_1 | BIT_2));
}

static int32_t _SCALE_DriverModuleEnable(uint32_t ahb_ctrl_addr)// must be AHB general control register base address
{
	ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;     

	_paod(ahb_ctrl_addr + ISP_AHB_CTRL_MOD_EN_OFFSET, BIT_1|BIT_2);
	_paad(ahb_ctrl_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, ~BIT_0); // switch memory to ISP

	_SCALE_DrvierModuleReset(ahb_ctrl_addr);

	return rtn;
}
static int32_t _SCALE_DriverModuleDisable(uint32_t ahb_ctrl_addr) // must be AHB general control register base address
{
	ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    

	_paod(ahb_ctrl_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, BIT_0); // switch memory to ARM    
	_paad(ahb_ctrl_addr + ISP_AHB_CTRL_MOD_EN_OFFSET,~(BIT_1|BIT_2));	

	return rtn;	
}

static  void   _SCALE_DriverScalingCoeffReset(void)
{
	s_scale_mod.isp_path1.h_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
	s_scale_mod.isp_path1.v_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
	s_scale_mod.isp_path2.h_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
	s_scale_mod.isp_path2.v_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;	    
}
static int32_t _SCALE_DriverSoftReset(uint32_t ahb_ctrl_addr)
{
	ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    

	_SCALE_DriverModuleEnable(ahb_ctrl_addr);

	_SCALE_DrvierModuleReset(ahb_ctrl_addr);    

	_SCALE_DriverScalingCoeffReset();

	return rtn;
}
#if 0
static int32_t _SCALE_DriverSetClk(uint32_t pll_src_addr,SCALE_CLK_SEL_E clk_sel)
{
	ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;	

	switch(clk_sel)
	{
		case SCALE_CLK_96M:
			_paad(pll_src_addr, ~(BIT_5 | BIT_4));
			break;
		case SCALE_CLK_64M:
			_paad(pll_src_addr, ~BIT_5);
			_paod(pll_src_addr, BIT_4);
			break;
		case SCALE_CLK_48M:
			_paad(pll_src_addr, ~BIT_4);
			_paod(pll_src_addr, BIT_5);
			break;
		default:
			_paod(pll_src_addr, BIT_4 | BIT_5);
			break;
	}
	return rtn;
    
}
#endif
uint32_t _SCALE_DriverInit(void)
{
	int32_t rtn_drv = ISP_DRV_RTN_SUCCESS;

	_SCALE_DriverIramSwitch(AHB_GLOBAL_REG_CTL0, 0); //switch IRAM to isp	

	rtn_drv = _SCALE_DriverSoftReset(AHB_GLOBAL_REG_CTL0);
	ISP_RTN_IF_ERR(rtn_drv);

//	rtn_drv = _SCALE_DriverSetClk(ARM_GLOBAL_PLL_SCR, SCALE_CLK_48M);
//	ISP_RTN_IF_ERR(rtn_drv);

	return rtn_drv;	
}

static void _SCALE_DriverDeinit(void)
{
	_SCALE_DriverModuleDisable(AHB_GLOBAL_REG_CTL0);
	_SCALE_DriverIramSwitch(AHB_GLOBAL_REG_CTL0, 1); //switch IRAM to ARM
}

static int32_t _SCALE_DriverStart(void)
{
	uint32_t rtn = ISP_DRV_RTN_SUCCESS;
	ISP_PATH_DESCRIPTION_T    *p_path2 = &s_scale_mod.isp_path2;
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	rtn = _SCALE_DriverPath2TrimAndScaling();            
	ISP_RTN_IF_ERR(rtn);

	_SCALE_DriverIrqEnable(ISP_IRQ_REVIEW_DONE_BIT);	

#if 0			
	if(ISP_DATA_RGB565 == p_path2->input_format)	
	{		
		endian_param.endian_y    = ISP_MASTER_ENDIANNESS_HALFBIG;		
		rtn = _SCALE_DriverPath2Config(ISP_PATH_INPUT_ENDIAN,(void*)&endian_param);
		ISP_RTN_IF_ERR(rtn);
	}
	else
	{
		endian_param.endian_y    = ISP_MASTER_ENDIANNESS_LITTLE;
		endian_param.endian_uv = ISP_MASTER_ENDIANNESS_LITTLE;		
		rtn = _SCALE_DriverPath2Config(ISP_PATH_INPUT_ENDIAN,(void*)&endian_param);
		ISP_RTN_IF_ERR(rtn);
	}
	
	SCALE_PRINT("SCALE: ISP_DriverStart , p_path2->input_format %d ,p_path2->output_format %d.\n",
	                             p_path2->input_format,p_path2->output_format);		

	if((ISP_DATA_RGB565 == p_path2->output_format) ||((ISP_DATA_YUV422 == p_path2->input_format) && (1 != p_path2->output_frame_endian.endian_y)))
	{
		endian_param.endian_y    = ISP_MASTER_ENDIANNESS_LITTLE;
		endian_param.endian_uv = ISP_MASTER_ENDIANNESS_LITTLE;		
		rtn = _SCALE_DriverPath2Config(ISP_PATH_OUTPUT_ENDIAN,(void*)&endian_param);
		ISP_RTN_IF_ERR(rtn);			
	}
	else
	{		 
		endian_param.endian_y    = ISP_MASTER_ENDIANNESS_LITTLE;
		endian_param.endian_uv = ISP_MASTER_ENDIANNESS_LITTLE;		
		rtn = _SCALE_DriverPath2Config(ISP_PATH_OUTPUT_ENDIAN,(void*)&endian_param);
		ISP_RTN_IF_ERR(rtn);
	} 
#endif

	if(1 == p_path2->slice_en)
	{				
		if(p_path2->slice_height < p_path2->output_size.h)
		{
			p_path2->is_last_slice = 0;
			p_isp_reg->slice_ver_cnt_u.mBits.last_slice = 0;			
		}
		else
		{
			p_path2->is_last_slice = 1;
			p_isp_reg->slice_ver_cnt_u.mBits.last_slice = 1;
		}
	}
            
	_SCALE_DriverForceCopy();
	
#ifdef SCALE_DEBUG
	get_scale_reg();
#endif

	p_isp_reg->rev_path_cfg_u.mBits.review_start = 1;
	SCALE_PRINT("SCALE: DriverStart is OK.\n"); 
	return rtn;
}

static  void _SCALE_DriverIrqClear(uint32_t mask)
{
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	p_isp_reg->dcam_int_clr_u.dwValue |= mask;	
}

static  void _SCALE_DriverIrqDisable(uint32_t mask)
{
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	p_isp_reg->dcam_int_mask_u.dwValue &= ~mask;
}

static int32_t _SCALE_DriverStop(void)
{
	uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
	ISP_REG_T  *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	dcam_dec_user_count();
	if( 0 == dcam_get_user_count())
	{
		_SCALE_DriverDeinit();
	}
	
	p_isp_reg->rev_path_cfg_u.mBits.review_start = 0;
	_SCALE_DriverIrqDisable(ISP_IRQ_SCL_LINE_MASK);
	_SCALE_DriverIrqClear(ISP_IRQ_SCL_LINE_MASK);

#if 0	
	endian_param.endian_y   = ISP_MASTER_ENDIANNESS_LITTLE;
	endian_param.endian_uv = ISP_MASTER_ENDIANNESS_LITTLE;
	rtn = _SCALE_DriverPath2Config(ISP_PATH_OUTPUT_ENDIAN,(void*)&endian_param);
	ISP_RTN_IF_ERR(rtn);	
#endif	
	//memset(&s_scale_mod, 0, sizeof(ISP_MODULE_T));	

	SCALE_PRINT("SCALE: DriverStop is OK.\n"); 

	return rtn;
}

static int32_t _SCALE_DriverSetMode(void)
{
	uint32_t    rtn = ISP_DRV_RTN_SUCCESS;
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	
	//review path enable
	p_isp_reg->dcam_cfg_u.mBits.review_path_eb = 1;
	p_isp_reg->dcam_cfg_u.mBits.cam_path2_eb   = 0;
	SCALE_PRINT("SCALE:_SCALE_DriverSetMode\n");
	return rtn;
}

void _SCALE_CopyTartgetData(void)
{
	uint32_t count, height;
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	
	if(0 == g_zoom_dma_buf.by_dma)
		return;

	count = p_isp_reg->slice_ver_cnt_u.mBits.slice_line_output;	
	height = count - g_zoom_dma_buf.last_line_cnt;
	SCALE_PRINT("SCALE: count: %d, hei: %d.\n", count, height);

	SCI_MEMCPY((void*)g_zoom_dma_buf.out_y_addr,(void*)g_zoom_dma_buf.in_y_addr,g_zoom_dma_buf.width*height);
	SCI_MEMCPY((void*)g_zoom_dma_buf.out_uv_addr,(void*)g_zoom_dma_buf.in_uv_addr,g_zoom_dma_buf.width*height);
}

uint32_t _SCALE_IsContinueSlice(void)
{
	ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
	uint32_t count = 0;
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	if(1 == p_path->slice_en)
	{		
		count = p_isp_reg->slice_ver_cnt_u.mBits.slice_line_output;
		p_path->slice_line_count = count;
		
		SCALE_PRINT("SCALE: current slice count: %d, line  output : %d, out_h: %d.\n", 
			                    p_path->slice_count,  p_path->slice_line_count, p_path->output_size.h);
		if(p_path->slice_line_count < p_path->output_size.h)
		{
			p_path->slice_count++;
			return 1;
		}
		else
		{
			p_path->slice_en = 0;
			g_zoom_dma_buf.by_dma = 0;
			return 0;
		}
	}
	else{
		return 0;		
	}
}


void _SCALE_ContinueSlice(long unsigned int data)
{
	ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
	ISP_FRAME_T next_frame;
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	ISP_RECT_T              rect = {0};
	int32_t                   rtn_drv = 0;

	//for YUV422
	//note: these addresses must be aligned by 64 words.	

	p_isp_reg->rev_path_cfg_u.mBits.review_start = 0;
	
          SCALE_PRINT("input_range:%d,%d,%d,%d \n",p_path->input_range.x,p_path->input_range.y,p_path->input_range.w,p_path->input_range.h);
	if((ISP_DATA_YUV422 == p_path->input_format)||(ISP_DATA_YUV420 == p_path->input_format))
	{
		next_frame.yaddr = p_path->input_frame.yaddr +(p_path->input_range.y+p_path->slice_count * p_path->slice_height )* p_path->input_size.w;
		if( ISP_DATA_YUV422 == p_path->input_format)
		{
			next_frame.uaddr = p_path->input_frame.uaddr +(p_path->input_range.y+p_path->slice_count * p_path->slice_height )* p_path->input_size.w;
		}
		else
		{
			next_frame.uaddr = p_path->input_frame.uaddr +( (p_path->input_range.y+p_path->slice_count * p_path->slice_height) * p_path->input_size.w)/2;
		}
	}
	else if( ISP_DATA_RGB888 == p_path->input_format)
	{
		next_frame.yaddr = p_path->input_frame.yaddr +(p_path->input_range.y+p_path->slice_count * p_path->slice_height )* p_path->input_size.w*4;
	}
	else
	{
		next_frame.yaddr = p_path->input_frame.yaddr +(p_path->input_range.y+p_path->slice_count * p_path->slice_height )* p_path->input_size.w*2;
	}
	next_frame.vaddr = next_frame.uaddr;
	_SCALE_DriverSetExtSrcFrameAddr(&next_frame);

	SCALE_PRINT("SCALE: input addr: y: 0x%x, u: 0x%x, in w: %d, slice hei: %d.\n", 
		                     p_path->input_frame.yaddr, 
		                     p_path->input_frame.uaddr, 
		                     p_path->input_size.w, 
		                     p_path->slice_height);
	SCALE_PRINT("SCALE: count: %d, next slice input buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", 
		                    p_path->slice_count, 
		                    next_frame.yaddr, 
		                    next_frame.uaddr, 
		                    next_frame.vaddr);

	next_frame.yaddr = p_path->output_frame.yaddr + p_path->slice_line_count * p_path->output_size.w;
	next_frame.uaddr = p_path->output_frame.uaddr + p_path->slice_line_count * p_path->output_size.w;

	next_frame.vaddr = next_frame.uaddr;
	_SCALE_DriverSetExtDstFrameAddr(&next_frame);
	SCALE_PRINT("SCALE: output addr: y: 0x%x, u: 0x%x, out w: %d.\n",
		                   p_path->output_frame.yaddr, 
		                   p_path->output_frame.uaddr, 
		                   p_path->output_size.w);
	SCALE_PRINT("SCALE: count: %d, next slice output buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", 
		                   p_path->slice_count, 
		                   next_frame.yaddr, 
		                   next_frame.uaddr, 
		                   next_frame.vaddr);

	rect.x = p_path->input_range.x;
	rect.y = 0;
	rect.w = p_path->input_range.w;
	rect.h = p_path->input_range.h;

	rtn_drv = _SCALE_DriverPath2Config(ISP_PATH_INPUT_RECT_PHY, (void*)&rect);
	ISP_RTN_IF_ERR(rtn_drv);

	if(p_path->slice_count == (p_path->input_range.h / p_path->slice_height))
	{
		p_path->slice_height = p_path->input_range.h - p_path->slice_count * p_path->slice_height;	        
		p_isp_reg->slice_ver_cnt_u.mBits.slice_line_input = p_path->slice_height;
		p_isp_reg->slice_ver_cnt_u.mBits.last_slice = 1;
		p_path->is_last_slice = 1;
		SCALE_PRINT("SCALE: the last slice height: %d.\n", p_path->slice_height);
	}
	else
	{
		p_isp_reg->slice_ver_cnt_u.dwValue = 0;
		p_isp_reg->slice_ver_cnt_u.mBits.slice_line_input = p_path->slice_height;	
	}
	
//	_SCALE_DriverPath2TrimAndScaling();   
//	_SCALE_DriverForceCopy();
	#ifdef SCALE_DEBUG
	//get_scale_reg();	
	#endif
	p_isp_reg->rev_path_cfg_u.mBits.review_start = 1;
	
}
static void  _SCALE_ISRPath2Done(void)
{  
	up(&g_sem);
	return ;
}

static int32_t _SCALE_DriverPath2Config(ISP_CFG_ID_E id, void* param)
{
	uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
	ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
   	ISP_REG_T        *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
        
	switch(id)
	{
		case ISP_PATH_MODE:
		{
			g_scale_mode = *(uint32_t*)param;
			break;
		}
		case ISP_PATH_INPUT_FORMAT:
		{
			uint32_t format = *(uint32_t*)param;

			if(format > ISP_DATA_RGB888)
			{
				rtn = ISP_DRV_RTN_PARA_ERR;     
			}
			else
			{
				p_path->input_format = format;
				if(format < ISP_DATA_RGB565) 
				{					
					p_isp_reg->rev_path_cfg_u.mBits.input_format = ISP_PATH_DATA_FORMAT_YUV;
					p_isp_reg->rev_path_cfg_u.mBits.yuv_input_format = format;
					SCALE_PRINT("SCALE: input format: %d, rev reg: %x.\n", format, p_isp_reg->rev_path_cfg_u.dwValue);
				}
				else
				{					   
					p_isp_reg->rev_path_cfg_u.mBits.input_format = ISP_PATH_DATA_FORMAT_RGB;
					p_isp_reg->rev_path_cfg_u.mBits.rgb_input_format = (format == ISP_DATA_RGB565) ? 1 : 0;
				}
			}
			break;
		}
		case ISP_PATH_INPUT_SIZE:
		{
			ISP_SIZE_T p_size;
			copy_from_user(&p_size, (ISP_SIZE_T *)param, sizeof(ISP_SIZE_T));			
			    	  
			if(p_size.w > ISP_PATH_FRAME_WIDTH_MAX ||p_size.h > ISP_PATH_FRAME_HEIGHT_MAX)
			{
				rtn = ISP_DRV_RTN_PARA_ERR;    
			}
			else
			{				
				p_isp_reg->rev_src_size_u.mBits.src_size_x = p_size.w;
				p_isp_reg->rev_src_size_u.mBits.src_size_y = p_size.h;
				p_path->input_size.w = p_size.w;
				p_path->input_size.h = p_size.h;				
			}
			break;
		}
		case ISP_PATH_INPUT_RECT:
		{
			ISP_RECT_T p_rect;
			copy_from_user(&p_rect, (ISP_RECT_T *)param, sizeof(ISP_RECT_T));			

			if(p_rect.x > ISP_PATH_FRAME_WIDTH_MAX ||
			   p_rect.y > ISP_PATH_FRAME_HEIGHT_MAX ||
			   p_rect.w > ISP_PATH_FRAME_WIDTH_MAX ||
			   p_rect.h > ISP_PATH_FRAME_HEIGHT_MAX )
			{
				rtn = ISP_DRV_RTN_PARA_ERR;    
			}
			else
			{				
				p_isp_reg->rev_trim_start_u.mBits.start_x = p_rect.x;
				p_isp_reg->rev_trim_start_u.mBits.start_y = p_rect.y;
				p_isp_reg->rev_trim_size_u.mBits.size_x   = p_rect.w;
				p_isp_reg->rev_trim_size_u.mBits.size_y   = p_rect.h;
				SCALE_MEMCPY((void*)&p_path->input_rect,&p_rect,sizeof(ISP_RECT_T));
			}
			break;
		}
		case ISP_PATH_INPUT_RECT_PHY:
		{
			ISP_RECT_T *p_rect = (ISP_RECT_T *)param;				

			if(p_rect->x > ISP_PATH_FRAME_WIDTH_MAX ||
			   p_rect->y > ISP_PATH_FRAME_HEIGHT_MAX ||
			   p_rect->w > ISP_PATH_FRAME_WIDTH_MAX ||
			   p_rect->h > ISP_PATH_FRAME_HEIGHT_MAX )
			{
				rtn = ISP_DRV_RTN_PARA_ERR;    
			}
			else
			{				
				p_isp_reg->rev_trim_start_u.mBits.start_x = p_rect->x;
				p_isp_reg->rev_trim_start_u.mBits.start_y = p_rect->y;
				p_isp_reg->rev_trim_size_u.mBits.size_x   = p_rect->w;
				p_isp_reg->rev_trim_size_u.mBits.size_y   = p_rect->h;
				SCALE_MEMCPY((void*)&p_path->input_rect,p_rect,sizeof(ISP_RECT_T));
			}
			break;
		}
		case ISP_PATH_INPUT_ADDR:
		{
			ISP_ADDRESS_T p_addr;

			SCALE_CHECK_PARAM_ZERO_POINTER(param);
			copy_from_user(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));			
				
			p_path->input_frame.yaddr = p_addr.yaddr;
			p_path->input_frame.uaddr = p_addr.uaddr;
			p_path->input_frame.vaddr = p_addr.vaddr;
			
			_SCALE_DriverSetExtSrcFrameAddr(&p_path->input_frame);  //for review, scale,slice scale
			SCALE_PRINT("SCALE: input buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", p_addr.yaddr, p_addr.uaddr, p_addr.vaddr);
			break;
		}
		case ISP_PATH_OUTPUT_SIZE:
		{
			ISP_SIZE_T p_size; 

			SCALE_CHECK_PARAM_ZERO_POINTER(param);
			copy_from_user(&p_size, (ISP_SIZE_T *)param, sizeof(ISP_SIZE_T));	
			
			if(p_size.w > ISP_PATH_FRAME_WIDTH_MAX ||p_size.h > ISP_PATH_FRAME_HEIGHT_MAX)
			{
				rtn = ISP_DRV_RTN_PARA_ERR;    
			}
			else
			{
				p_isp_reg->rev_des_size_u.mBits.des_size_x = p_size.w;
				p_isp_reg->rev_des_size_u.mBits.des_size_y = p_size.h;
				p_path->output_size.w = p_size.w;
				p_path->output_size.h = p_size.h;				
			}
			break;
		}  
		case ISP_PATH_OUTPUT_FORMAT:
		{
			uint32_t format = *(uint32_t*)param;
			SCALE_PRINT("SCALE:_SCALE_DriverPath2Config out format=%d",format);		

			if(format != ISP_DATA_YUV422 && 		
			    format != ISP_DATA_YUV420 && 
			    format != ISP_DATA_RGB565)
			{
				rtn = ISP_DRV_RTN_PARA_ERR;     
			}
			else
			{
				if(format == ISP_DATA_RGB565)
				{					
					p_isp_reg->rev_path_cfg_u.mBits.output_format = 2;
				}
				else if(format == ISP_DATA_YUV420)
				{
					p_isp_reg->rev_path_cfg_u.mBits.output_format = 1;
				}
				else
				{					
					p_isp_reg->rev_path_cfg_u.mBits.output_format = 0;
				}
				p_path->output_format = format;	
			}
			break;
		}    
		case ISP_PATH_OUTPUT_ADDR:
		{
			ISP_ADDRESS_T p_addr; 

			SCALE_CHECK_PARAM_ZERO_POINTER(param);
			copy_from_user(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));	
			{	
				p_path->output_frame.yaddr = p_addr.yaddr;
				p_path->output_frame.uaddr = p_addr.uaddr;
				p_path->output_frame.vaddr = p_addr.vaddr;
			}	
			SCALE_PRINT("SCALE: output addr: %x, %x, %x.\n", p_addr.yaddr, p_addr.uaddr, p_addr.vaddr);
			_SCALE_DriverSetExtDstFrameAddr(&p_path->output_frame);  //for review, scale,slice scale

			break;
		}   
		case ISP_PATH_OUTPUT_FRAME_FLAG:
		{
			SCALE_CHECK_PARAM_ZERO_POINTER(param);

			p_path->output_frame_flag = *(uint32_t*)param;
			break;
		}     
		case ISP_PATH_SUB_SAMPLE_EN:
		{
			SCALE_CHECK_PARAM_ZERO_POINTER(param);

			p_path->sub_sample_en = *(uint32_t*)param ? 1 : 0;			
			p_isp_reg->rev_path_cfg_u.mBits.sub_sample_eb = p_path->sub_sample_en;
			break;
		}      	        
		case ISP_PATH_SUB_SAMPLE_MOD:
		{
			uint32_t sub_sameple_mode = *(uint32_t*)param;

			SCALE_CHECK_PARAM_ZERO_POINTER(param);

			if(sub_sameple_mode > ISP_PATH_SUB_SAMPLE_MAX)
			{
				rtn = ISP_DRV_RTN_PARA_ERR;        
			}
			else
			{				
				p_isp_reg->rev_path_cfg_u.mBits.sub_sample_mode = sub_sameple_mode;
				p_path->sub_sample_factor = sub_sameple_mode;
			}	  	
			break;
		}        	
		case ISP_PATH_SWAP_BUFF:
		{
			ISP_ADDRESS_T p_addr;

			SCALE_CHECK_PARAM_ZERO_POINTER(param);
			memcpy(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));				
			p_isp_reg->frm_addr_2_u.dwValue = p_addr.yaddr;
			p_isp_reg->frm_addr_3_u.dwValue = p_addr.uaddr;
			p_isp_reg->frm_addr_6_u.dwValue = p_addr.vaddr;
			SCALE_PRINT("SCALE: swap and line buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", 
				                   p_addr.yaddr, p_addr.uaddr, p_addr.vaddr);
			break;
		}
		case ISP_PATH_SLICE_SCALE_EN:			
			p_isp_reg->rev_path_cfg_u.mBits.scale_mode = ISP_SCALE_SLICE;
			p_path->slice_en = 1;
			p_path->slice_line_count = 0;
			p_path->slice_count = 0;
			break;			
		case ISP_PATH_SLICE_SCALE_HEIGHT:
		{
			uint32_t slice_height;

			SCALE_CHECK_PARAM_ZERO_POINTER(param);

			slice_height = (*(uint32_t*)param) & ISP_PATH_SLICE_MASK;
			SCALE_PRINT("SCALE:SLICE_VER_CNT: 0x%x.\n",p_isp_reg->slice_ver_cnt_u.dwValue);			
			
			p_isp_reg->slice_ver_cnt_u.mBits.slice_line_input = slice_height;
			p_path->slice_en = 1;
			p_path->slice_height = slice_height;
			SCALE_PRINT("SCALE: slice height %d, SLICE_VER_CNT: 0x%x.\n",p_path->slice_height, p_isp_reg->slice_ver_cnt_u.dwValue);
			break;			
		}		
		case ISP_PATH_DITHER_EN:
			p_isp_reg->rev_path_cfg_u.mBits.dither_eb = 1;
			break;
		case ISP_PATH_OUTPUT_ENDIAN:
		{
			ISP_ENDIAN_T endian;
			SCALE_CHECK_PARAM_ZERO_POINTER(param);
			memcpy(&endian, (ISP_ENDIAN_T *)param, sizeof(ISP_ENDIAN_T));			
			p_path->output_frame_endian = endian;
			p_isp_reg->endian_sel_u.mBits.review_output_endian_y = endian.endian_y&0x3;
			p_isp_reg->endian_sel_u.mBits.review_output_endian_uv = endian.endian_uv&0x3;			
			break;
		}
		case ISP_PATH_INPUT_ENDIAN:
		{					
			ISP_ENDIAN_T endian;
			SCALE_CHECK_PARAM_ZERO_POINTER(param);
			memcpy(&endian, (ISP_ENDIAN_T *)param, sizeof(ISP_ENDIAN_T));				
			p_path->input_frame_endian = endian;
			p_isp_reg->endian_sel_u.mBits.review_input_endian_y = endian.endian_y&0x3 ;
			p_isp_reg->endian_sel_u.mBits.review_input_endian_uv = endian.endian_uv&0x3;
			break;
		}
		default:
			rtn = ISP_DRV_RTN_PARA_ERR;
			break;
    }	    

    return rtn;
	
}

int _SCALE_DriverIOPathConfig(SCALE_CFG_ID_E id, void* param)
{
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
#if 0	
    ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
           
    switch(id)
    {
    	case ISP_PATH_MODE:
	{
		g_scale_mode = *(uint32_t*)param;
		break;
    	}
        case ISP_PATH_INPUT_FORMAT:
        {
            uint32_t format = *(uint32_t*)param;
            
            if(format > ISP_DATA_RGB888)
            {
                rtn = ISP_DRV_RTN_PARA_ERR;     
            }
            else
            {
                p_path->input_format = format;
                if(format < ISP_DATA_RGB565) //YUV format
                {
                    _paad(REV_PATH_CFG, ~BIT_5);//ISP_PATH_DATA_FORMAT_YUV
		    _paad(REV_PATH_CFG, ~(BIT_11 | BIT_12));
		    _paod(REV_PATH_CFG, (format & 0x3) << 11);
		   SCALE_PRINT("###scale: input format: %d, rev reg: %x.\n", format, _pard(REV_PATH_CFG));
                }
                else
                {
                    _paod(REV_PATH_CFG, BIT_5);//ISP_PATH_DATA_FORMAT_RGB
                    _paad(REV_PATH_CFG, ~BIT_13);
		    _paod(REV_PATH_CFG, (format == ISP_DATA_RGB565 ? 1 : 0) << 13);		    
                }
            }
            break;
        }
        case ISP_PATH_INPUT_SIZE:
        {
            ISP_SIZE_T p_size;
	    memcpy(&p_size, (ISP_SIZE_T *)param, sizeof(ISP_SIZE_T));			
  
            if(p_size.w > ISP_PATH_FRAME_WIDTH_MAX ||
               p_size.h > ISP_PATH_FRAME_HEIGHT_MAX)
            {
                rtn = ISP_DRV_RTN_PARA_ERR;    
            }
            else
            {
		_pawd(REV_SRC_SIZE, (p_size.w & 0xFFF) | ((p_size.h & 0xFFF) << 16));
                p_path->input_size.w = p_size.w;
                p_path->input_size.h = p_size.h;				
            }
            break;
        }
        case ISP_PATH_INPUT_RECT:
        {
            ISP_RECT_T p_rect;
	    memcpy(&p_rect, (ISP_RECT_T *)param, sizeof(ISP_RECT_T));			
             
            if(p_rect.x > ISP_PATH_FRAME_WIDTH_MAX ||
               p_rect.y > ISP_PATH_FRAME_HEIGHT_MAX ||
               p_rect.w > ISP_PATH_FRAME_WIDTH_MAX ||
               p_rect.h > ISP_PATH_FRAME_HEIGHT_MAX )
            {
                rtn = ISP_DRV_RTN_PARA_ERR;    
            }
            else
            {
		_pawd(REV_TRIM_START, (p_rect.x & 0xFFF) | ((p_rect.y & 0xFFF) << 16));
		_pawd(REV_TRIM_SIZE, (p_rect.w & 0xFFF) | ((p_rect.h & 0xFFF) << 16));
                SCALE_MEMCPY((void*)&p_path->input_rect,
                            &p_rect,
                           sizeof(ISP_RECT_T));
            }
            break;
        }
        case ISP_PATH_INPUT_ADDR:
        {
            ISP_ADDRESS_T p_addr;
            
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
	    memcpy(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));			
            
            {	
                p_path->input_frame.yaddr = p_addr.yaddr;
                p_path->input_frame.uaddr = p_addr.uaddr;
                p_path->input_frame.vaddr = p_addr.vaddr;
            }
		
		_SCALE_DriverSetExtSrcFrameAddr(&p_path->input_frame);  //for review, scale,slice scale
            SCALE_PRINT("###SCALE: input buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", p_addr.yaddr, p_addr.uaddr, p_addr.vaddr);
            break;
        }
        case ISP_PATH_OUTPUT_SIZE:
        {
            ISP_SIZE_T p_size; 
          
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
	    memcpy(&p_size, (ISP_SIZE_T *)param, sizeof(ISP_SIZE_T));			
        	  
        	  
            if(p_size.w > ISP_PATH_FRAME_WIDTH_MAX ||
               p_size.h > ISP_PATH_FRAME_HEIGHT_MAX)
            {
                rtn = ISP_DRV_RTN_PARA_ERR;    
            }
            else
            {
		_pawd(REV_DES_SIZE, (p_size.w & 0xFFF) | ((p_size.h & 0xFFF) << 16));
                p_path->output_size.w = p_size.w;
                p_path->output_size.h = p_size.h;				
            }
            break;
        }  
        case ISP_PATH_OUTPUT_FORMAT:
        {
            uint32_t format = *(uint32_t*)param;
            
            if(format != ISP_DATA_YUV422 && 		
               format != ISP_DATA_YUV420 && 
               format != ISP_DATA_RGB565)
            {
               rtn = ISP_DRV_RTN_PARA_ERR;     
            }
            else
            {
                if(format == ISP_DATA_RGB565)
                {
                    _paod(REV_PATH_CFG, BIT_7);
		    _paad(REV_PATH_CFG, ~BIT_6);
                }
                else if(format == ISP_DATA_YUV420)
                {
                    _paad(REV_PATH_CFG, ~BIT_7);
		    _paod(REV_PATH_CFG, BIT_6);
                }
                else
                {
		    _paad(REV_PATH_CFG, ~(BIT_6 | BIT_7));
                }
                p_path->output_format = format;	
 
            }
            
            break;
        }    
        case ISP_PATH_OUTPUT_ADDR:
        {
            ISP_ADDRESS_T p_addr; 
	    ISP_FRAME_T p_frame;
        	
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
	    memcpy(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));			
            
                    p_frame.yaddr = p_addr.yaddr;
                    p_frame.uaddr = p_addr.uaddr;                    
                    p_frame.vaddr = p_addr.vaddr;
		SCALE_PRINT("###scale: output addr: %x, %x, %x.\n", p_frame.yaddr, p_frame.uaddr, p_frame.vaddr);
		_SCALE_DriverSetExtDstFrameAddr(&p_frame);  //for review, scale,slice scale
	
            break;
        }   
        case ISP_PATH_OUTPUT_FRAME_FLAG:
        {
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
			
            p_path->output_frame_flag = *(uint32_t*)param;
            break;
        }     
        case ISP_PATH_SUB_SAMPLE_EN:
        {
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
            
            p_path->sub_sample_en = *(uint32_t*)param ? 1 : 0;
            _paad(REV_PATH_CFG, ~BIT_2);
	    _paod(REV_PATH_CFG, (p_path->sub_sample_en & 0x1) << 2);
        	  	
            break;
        }      	
        
        case ISP_PATH_SUB_SAMPLE_MOD:
        {
            uint32_t sub_sameple_mode = *(uint32_t*)param;
            
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
            
            if(sub_sameple_mode > ISP_PATH_SUB_SAMPLE_MAX)
            {
                rtn = ISP_DRV_RTN_PARA_ERR;        
            }
            else
            {
            	_paad(REV_PATH_CFG, ~(BIT_9 | BIT_10));
	    	_paod(REV_PATH_CFG, (sub_sameple_mode & 0x3) << 9);
                p_path->sub_sample_factor = sub_sameple_mode;
            }	  	
            break;
           
        } 
        case ISP_PATH_SLICE_SCALE_EN:
            _paod(REV_PATH_CFG, BIT_4); //ISP_SCALE_SLICE
            p_path->slice_en = 1;
            break;		

        case ISP_PATH_SLICE_SCALE_HEIGHT:
        {
            uint32_t slice_height;
			
            SCALE_CHECK_PARAM_ZERO_POINTER(param);

            slice_height = (*(uint32_t*)param) & ISP_PATH_SLICE_MASK;
	    if(slice_height < p_path->input_rect.w){
			_paad(SLICE_VER_CNT, ~0x1FFF); 
			_paod(SLICE_VER_CNT, slice_height & 0xFFF);
			p_path->is_last_slice = 0;
	    }
	    else{
	    	_paad(SLICE_VER_CNT, ~0xFFF);
		_paod(SLICE_VER_CNT, (slice_height & 0xFFF) | BIT_12);
		p_path->is_last_slice = 1;
	    }			
            break;			
        }		
        case ISP_PATH_DITHER_EN:
            _paod(REV_PATH_CFG, BIT_8);
            break;
        default:
            rtn = ISP_DRV_RTN_PARA_ERR;
            break;
    }	    
#endif
    return rtn;
	
}

typedef void (*isr_func_t)(void);

void _SCALE_DriverEnableInt(void)
{
//	 _paod(SCL_INT_IRQ_EN, 1UL << 27);
}
void _SCALE_DriverDisableInt(void)
{
// 	_paod(SCL_INT_IRQ_DISABLE, 1UL << 27);
}
static  uint32_t _SCALE_DriverReadIrqLine(void)
{
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;

	return p_isp_reg->dcam_int_stat_u.dwValue;
}
typedef void (*ISP_ISR_PTR)(void);
static ISP_ISR_PTR s_isp_isr_list[ISP_IRQ_NUMBER]=
{
	NULL,
	NULL,		
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	_SCALE_ISRPath2Done
};

static void _SCALE_DriverISRRoot(void)
{
	uint32_t irq_line, irq_status;
	uint32_t  i;

	irq_line = ISP_IRQ_SCL_LINE_MASK & _SCALE_DriverReadIrqLine();
	irq_status = irq_line;

	for(i = 0; i < ISP_IRQ_NUMBER; i++)
	{
		if(irq_line & 1)
		{
			s_isp_isr_list[i]();
		}
		irq_line>>=1;
		if(!irq_line)
		break;
	}
	
	
	//_SCALE_ISRPath2Done();

	_SCALE_DriverIrqClear(irq_status);	
}

static ISR_EXE_T _SCALE_ISRSystemRoot(uint32_t param)
{
	_SCALE_DriverISRRoot();

	return ISR_DONE;
}

static irqreturn_t _SCALE_DriverISR(int irq, void *dev_id)
{
	ISP_REG_T *p_isp_reg = (ISP_REG_T*)s_scale_mod.module_addr;
	uint32_t value = p_isp_reg->dcam_int_stat_u.mBits.review_done;

	if(1 != value)
	{
		SCALE_PRINT("SCALE:irq exit.\n");
		return IRQ_NONE;
	}
	_SCALE_ISRSystemRoot(0);
	return IRQ_HANDLED;
}
int  _SCALE_DriverRegisterIRQ(void)
{
	uint32_t ret = 0;

	if(0 != (ret = request_irq(IRQ_LINE_DCAM, _SCALE_DriverISR, SA_SHIRQ, "SCALE", &g_share_irq)))
	SCALE_ASSERT(0);

	return ret;
}
void _SCALE_DriverUnRegisterIRQ(void)
{
  	free_irq(IRQ_LINE_DCAM, &g_share_irq);
}

int _SCALE_DriverIOInit(void)
{
	//down(&g_sem_cnt);
	isp_get_path2();
	if(0 < g_scale_num)
	{
		SCALE_PRINT_ERR("SCALE: fail to open device,g_scale_num=%d.\n",g_scale_num);
		return -1;
	}

	init_MUTEX(&g_sem);
	memset(&s_scale_mod, 0, sizeof(ISP_MODULE_T));
	g_scale_clk = clk_get(NULL, "clk_dcam");
	if(IS_ERR(g_scale_clk))
	{
		SCALE_PRINT_ERR("SCALE: _SCALE_DriverIOInit,Failed: Can't get clock [clk_dcam]!\n");
		SCALE_PRINT_ERR("SCALE: _SCALE_DriverIOInit,g_scale_clk = %p.\n", g_scale_clk);
	}
	else
	{
		SCALE_PRINT_ERR("SCALE:_SCALE_DriverIOInit, g_scale_clk clk_get ok.g_dcam_clk->parent->usecount: %d.\n", g_scale_clk->parent->usecount);
	}	
	s_scale_mod.module_addr = DCAM_REG_BASE;

	g_scale_num++;
	
	if( 0 == dcam_get_user_count())
     	{
     		_SCALE_DriverInit();
		SCALE_PRINT("SCALE:_SCALE_DriverInit.\n");
     	}
	if(0 != _SCALE_DriverSetMclk(SCALE_CLK_128M))
	{
		SCALE_PRINT_ERR("SCALE:_SCALE_DriverIOInit,Failed to _SCALE_DriverSetMclk!\n");
		return -1;
	}
	dcam_inc_user_count();

	return 0;
}

int SCALE_open (struct inode *node, struct file *pf)
{	
     	if(0 == _SCALE_DriverIOInit())
		return 0;
	else
		return -1;
}

int _SCALE_DriverIODeinit (void)
{
	_SCALE_DriverStop();
	_SCALE_DriverUnRegisterIRQ();
	g_scale_num--;

	//up(&g_sem_cnt);

	if(g_scale_clk)
	{
		clk_disable(g_scale_clk);
		SCALE_PRINT("SCALE:_SCALE_DriverIODeinit,clk_disable ok.\n");
		clk_put(g_scale_clk);
		SCALE_PRINT("SCALE: _SCALE_DriverIODeinit clk_put ok.\n");
		g_scale_clk = NULL;
	}
	isp_put_path2();

	return 0;
}

int SCALE_release (struct inode *node, struct file *pf)
{
	_SCALE_DriverIODeinit();

	return 0;
}
/* ------------------------------------------------------------------
	File operations for the device
   ------------------------------------------------------------------*/
int _SCALE_DriverIODone(void)
{
	int32_t ret = 0;
	_SCALE_DriverSetMode();
	_SCALE_DriverRegisterIRQ();
	ret = _SCALE_DriverStart();
	if(ISP_DRV_RTN_SUCCESS != ret)
	{
		SCALE_PRINT_ERR("_SCALE_DriverIODone: 0,ret=%d .\n ",ret);
	}
	down(&g_sem);
	down_interruptible(&g_sem);
	while(1){
		if(1 == _SCALE_IsContinueSlice()){
			_SCALE_ContinueSlice(0);
			down_interruptible(&g_sem);
		}
		else{			
			break;
		}
	}
	
	up(&g_sem);	
	
	return 0;	
}

static int SCALE_ioctl(struct inode *node, struct file *fl, unsigned int cmd, unsigned long param)
{	
	switch(cmd)
	{
		case SCALE_IOC_CONFIG:
		{			
			SCALE_CONFIG_T path2_config;			
			ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
			copy_from_user(&path2_config, (SCALE_CONFIG_T *)param, sizeof(SCALE_CONFIG_T));		
			if(ISP_PATH_INPUT_RECT == path2_config.id)
			{
				memcpy(&p_path->input_range,path2_config.param,sizeof(ISP_RECT_T));
			}
			_SCALE_DriverPath2Config(path2_config.id,  path2_config.param);
		}
		break;		
		case SCALE_IOC_DONE:
			_SCALE_DriverIODone();
			break;
		case SCALE_IOC_YUV422_YUV420:
		{			
		//	SCALE_YUV422_YUV420_T yuv_config;			
		//	copy_from_user(&yuv_config, (SCALE_YUV422_YUV420_T *)param, sizeof(SCALE_YUV422_YUV420_T));			
		//	_SCALE_DriverYUV422TYUV420(&yuv_config);
		}
			break;			
		default:
			break;
	}
	return 0;
}

/**************************************************************************/

static struct file_operations scale_fops = {
	.owner		= THIS_MODULE,
	.open           = SCALE_open,
	.ioctl          = SCALE_ioctl,
	.release	= SCALE_release,
};

/* -----------------------------------------------------------------
	Initialization and module stuff
   ------------------------------------------------------------------*/
static struct miscdevice scale_dev = {
	.minor   = SCALE_MINOR,
	.name   = "sc8800g_scale",
	.fops   = &scale_fops,
};

int scale_probe(struct platform_device *pdev)
{
	int ret;
	printk(KERN_ALERT"scale_probe called\n");

	ret = misc_register(&scale_dev);
	if (ret) {
		printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n",
				SCALE_MINOR, ret);
		return ret;
	}

	lock = (struct mutex *)kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (lock == NULL)
		return -1;

	mutex_init(lock);
	
	SCALE_PRINT("SCALE: init wait_queue_zoom.\n");

	printk(KERN_ALERT" scale_probe Success\n");

	return 0;
}

static int scale_remove(struct platform_device *dev)
{
	printk(KERN_INFO "scale_remove called !\n");

	misc_deregister(&scale_dev);

	printk(KERN_INFO "scale_remove Success !\n");
	return 0;
}

static struct platform_driver scale_driver = {
	.probe    = scale_probe,
	.remove   = scale_remove,
	.driver   = {
		.owner = THIS_MODULE,
		.name = "sc8800g_scale",
	},
};

int __init scale_init(void)
{
	if(platform_driver_register(&scale_driver) != 0) {
		printk("platform device register Failed \n");
		return -1;
	}

	isp_mutex_init();//init_MUTEX(&g_sem_cnt);	

	return 0;
}

void scale_exit(void)
{
	platform_driver_unregister(&scale_driver);
	mutex_destroy(lock);
	kfree(lock);//wxz20120118: free the mutex lock
	lock = NULL;
}

module_init(scale_init);
module_exit(scale_exit);

MODULE_DESCRIPTION("Scale Driver");
MODULE_LICENSE("GPL");
