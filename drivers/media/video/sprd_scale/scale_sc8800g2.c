/*
* drivers/media/video/sprd_scale/scale_sc8800g2.c
 * Scale driver based on sc8800g2
 *
 * Copyright (C) 2010 Spreadtrum 
 * 
 * Author: Xiaozhe wang <xiaozhe.wang@spreadtrum.com>
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
#include <mach/clock_common.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/dma.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "../sprd_dcam/dcam_power_sc8800g2.h"
#include "scale_sc8800g2.h"
#include "scale_reg_sc8800g2.h"

#ifdef ISP_DRV_SCALE_COEFF_TABLE_EN
#include "scale_sc8800g_scalecoeff_h.h"
#include "scale_sc8800g_scalecoeff_v.h"
#else
#include "gen_scale_coef.h"
//#define ISP_DRV_SCALE_COEFF_DBG
//#define ISP_DRV_SCALE_COEFF_TABLE_EN
#define ISP_PATH1 1 
#define ISP_PATH2 2
#define ISP_DRV_SCALE_COEFF_BUF_SIZE        (8*1024)
#define ISP_DRV_SCALE_COEFF_TMP_SIZE        (6*1024)
#define ISP_DRV_SCALE_COEFF_COEF_SIZE       (1*1024)
#define SCI_NULL 0 
#define SCI_MEMSET  memset
#define SCI_MEMCPY	memcpy
#define SCI_ASSERT(...) 
#define SCI_PASSERT(condition, format...)
#endif

static wait_queue_head_t	wait_queue;
static  int  condition; 
static wait_queue_head_t	wait_queue_zoom;
static  int  condition_zoom; 
static wait_queue_head_t	wait_queue_endian;
static  int  condition_endian; 

struct tasklet_struct my_tasklet;


typedef enum
{
    SCALE_CLK_96M = 0,
    SCALE_CLK_64M,
    SCALE_CLK_48M,
    SCALE_CLK_26M
} SCALE_CLK_SEL_E;

#define SCALE_MINOR MISC_DYNAMIC_MINOR

static struct mutex *lock;

#define SA_SHIRQ IRQF_SHARED
#define SCALE_CHECK_PARAM_ZERO_POINTER(n)                do{if(0 == (int)(n)) return ISP_DRV_RTN_PARA_ERR;}while(0)
#define ISP_RTN_IF_ERR(n)               if(ISP_DRV_RTN_SUCCESS != n)  return n
#define SCALE_PATH_INVALID_AHB_ADDR                      0x800 // invalide address if in low 2k bytes 
#define SCALE_PATH_ADDR_INVALIDE(addr)                   (NULL != (addr)) 
#define SCALE_PATH_YUV_ADDR_INVALIDE(y,u,v)              (SCALE_PATH_ADDR_INVALIDE(y) && SCALE_PATH_ADDR_INVALIDE(u) && SCALE_PATH_ADDR_INVALIDE(v))

//#define SCALE_DEBUG
#ifdef SCALE_DEBUG
#define SCALE_PRINT printk
#else
#define SCALE_PRINT(...)
#endif
#define SCALE_PRINT_ERR printk

ISP_MODULE_T           s_scale_mod;
uint32_t g_base_addr = ISP_AHB_SLAVE_ADDR;
SCALE_MODE_E g_scale_mode = SCALE_MODE_SCALE;

//wxz: from isr_drvapi.h
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

#define W64_ALIGNED 0x100
#define ZOOM_BUF_ALIGNED(x) ((x + W64_ALIGNED - 1) & ~(W64_ALIGNED - 1))

#define IRQ_LINE_DCAM 27 //26 //irq line number in system
#define NR_DCAM_ISRS 12
struct semaphore g_sem;
struct semaphore g_sem_cnt;
static int g_scale_num = 0;//store the time opened.
static uint32_t g_share_irq = 0xFF; //for share irq handler function
ZOOM_DMA_BUF g_zoom_dma_buf;	//use it when the zoom buf address is not aligned by 64 words.

struct clk *g_scale_clk = NULL; //for power manager


static int _SCALE_DriverSetMclk(void){
    	char *name_parent = NULL;
    	struct clk *clk_parent = NULL;
	int ret;

	//name_parent = "clk_64m";
	name_parent = "clk_96m";
	clk_parent = clk_get_parent(g_scale_clk);
	SCALE_PRINT("###scale:clock[%s]: parent_name: %s.\n", g_scale_clk->name, clk_parent->name);
	if(strcmp(name_parent, clk_parent->name)){//need to wait the parent
		clk_parent = clk_get(NULL, name_parent);
		if(!clk_parent){
			SCALE_PRINT_ERR("###scale:clock[%s]: failed to get parent [%s] by clk_get()!\n", g_scale_clk->name, name_parent);
			return -EINVAL;
		}

		ret = clk_set_parent(g_scale_clk, clk_parent);
		if(ret){
			SCALE_PRINT_ERR("###scale:clock[%s]: clk_set_parent() failed!parent: %s, usecount: %d.\n", g_scale_clk->name, clk_parent->name, g_scale_clk->usecount);
			return -EINVAL;
		}		
	}
	ret = clk_enable(g_scale_clk);
	if(ret){
		SCALE_PRINT_ERR("###scale:clock[%s]: clk_enable() failed!\n", g_scale_clk->name);
	}
	else{
		SCALE_PRINT("###scale g_scale_clk clk_enable ok.\n");
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
    printk("SCALE reg:0x%x, 0x%x.\n", DCAM_REG_BASE + i * 4, value);
   }
   for(i = 0; i < 9; i++)
   {
   	value = _pard(DCAM_REG_BASE + 0x0100 + i * 4);
        printk("SCALE reg:0x%x, 0x%x.\n", DCAM_REG_BASE + 0x0100 + i * 4, value);
   }
}
#endif
LOCAL void _SCALE_DriverSetExtSrcFrameAddr(ISP_FRAME_T *p_frame) //for review, scale,slice scale
{
    _pawd(FRM_ADDR_0, (p_frame->yaddr >> 8) & 0x3FFFF);
    _pawd(FRM_ADDR_H, (p_frame->yaddr>> (32 - ISP_PATH_FRAME_HIGH_BITS)) & 0x7F);
	
    if(s_scale_mod.isp_path2.input_format == ISP_DATA_YUV422 ||       
       s_scale_mod.isp_path2.input_format == ISP_DATA_YUV420 || 
       s_scale_mod.isp_path2.input_format == ISP_DATA_YUV420_3FRAME) 
    {
    	_pawd(FRM_ADDR_1, (p_frame->uaddr >> 8) & 0x3FFFF);
		
        if(s_scale_mod.isp_path2.input_format == ISP_DATA_YUV420_3FRAME)
        {
    		_pawd(FRM_ADDR_2, (p_frame->vaddr >> 8) & 0x3FFFF);

        }
    }
}

LOCAL void _SCALE_DriverSetExtDstFrameAddr(ISP_FRAME_T *p_frame)  //for review, scale,slice scale
{
    _pawd(FRM_ADDR_4, (p_frame->yaddr >> 8) & 0x3FFFF);
    _pawd(FRM_ADDR_H, (p_frame->yaddr>> (32 - ISP_PATH_FRAME_HIGH_BITS)) & 0x7F);
	
    if(s_scale_mod.isp_path2.output_format == ISP_DATA_YUV422 || 	
       s_scale_mod.isp_path2.output_format == ISP_DATA_YUV420) 
    {
    	_pawd(FRM_ADDR_5, (p_frame->uaddr >> 8) & 0x3FFFF);
    }
}
LOCAL uint32_t _SCALE_DriverGetSubSampleFactor(uint32_t* src_width, 
                                           uint32_t* src_height,
                                           uint32_t dst_width,
                                           uint32_t dst_height)
{
	if(*src_width > (dst_width * ISP_PATH_SC_COEFF_MAX) || 
       *src_height > (dst_height * ISP_PATH_SC_COEFF_MAX) )
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
LOCAL int32_t _SCALE_DriverCalcSC2Size(void)
{
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
	
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
            p_path->sub_sample_factor > ISP_PATH_SUB_SAMPLE_FACTOR_BASE ) ||  // in mpeg or preview_ex mode, path2 do deci by 1/2
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

#ifdef ISP_DRV_SCALE_COEFF_TABLE_EN
LOCAL uint32_t _SCALE_DriverCalcScaleCoeff(uint32_t input_width, 
                                       uint32_t input_height, 
                                       uint32_t output_width, 
                                       uint32_t output_height,
                                       uint32_t *p_h_coeff,
                                       uint32_t *p_v_coeff)
{
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
    uint32_t                    v_coeff = 0;
    uint32_t                    h_coeff= 0;    
	
    
    h_coeff = (output_width * ISP_PATH_SCALE_LEVEL + input_width - 1) / input_width;
    v_coeff = (output_height * ISP_PATH_SCALE_LEVEL + input_height -1) / input_height;

    if(( ISP_PATH_SCALE_LEVEL_MAX < h_coeff ) ||( ISP_PATH_SCALE_LEVEL_MIN > h_coeff ) ||
       ( ISP_PATH_SCALE_LEVEL_MAX < v_coeff ) ||( ISP_PATH_SCALE_LEVEL_MIN > v_coeff ) )
    {
        rtn = ISP_DRV_RTN_PARA_ERR;
    }
    else
    {
        if(ISP_PATH_SCALE_LEVEL < h_coeff)
        {
            h_coeff = ISP_PATH_SCALE_LEVEL - ISP_PATH_SCALE_LEVEL_MIN;
        }
        else
        {
            h_coeff = h_coeff - ISP_PATH_SCALE_LEVEL_MIN;
        }

        if(ISP_PATH_SCALE_LEVEL < v_coeff)
        {
            v_coeff = ISP_PATH_SCALE_LEVEL-ISP_PATH_SCALE_LEVEL_MIN;
        }
        else
        {
            v_coeff = v_coeff - ISP_PATH_SCALE_LEVEL_MIN;   
        }    

        *p_h_coeff = h_coeff;
        *p_v_coeff = v_coeff;  
	
    }
	
    return rtn;	
    
}
#endif //ISP_DRV_SCALE_COEFF_TABLE_EN

#ifndef ISP_DRV_SCALE_COEFF_TABLE_EN
static int32_t _ISP_DriverGenScxCoeff(uint32_t idxScx)
{  
    ISP_PATH_DESCRIPTION_T *p_path = SCI_NULL;
    uint32_t i = 0;
    uint32_t HScaleAddr   = 0;
    uint32_t VScaleAddr   = 0;	

    uint32_t *pTmpBuf     = SCI_NULL;
    uint32_t *pHCoeff     = SCI_NULL;	
    uint32_t *pVCoeff     = SCI_NULL;


	if(ISP_PATH1 != idxScx && ISP_PATH2 != idxScx)
	    return ISP_DRV_RTN_PARA_ERR;
    
#ifdef ISP_DRV_SCALE_COEFF_DBG
    SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff Entry");
#endif

	if(ISP_PATH1 == idxScx)
	{
	    p_path = &s_scale_mod.isp_path1;

        HScaleAddr   = g_base_addr + ISP_SCALE1_H_TAB_OFFSET;
        VScaleAddr   = g_base_addr + ISP_SCALE1_V_TAB_OFFSET;        
    }
    else 
    {
        p_path = &s_scale_mod.isp_path2;
        
        HScaleAddr   = g_base_addr + ISP_SCALE2_H_TAB_OFFSET;
        VScaleAddr   = g_base_addr + ISP_SCALE2_V_TAB_OFFSET;  
    }
    
    pTmpBuf = (uint32_t *)kmalloc(ISP_DRV_SCALE_COEFF_BUF_SIZE, GFP_KERNEL);
    SCI_ASSERT(SCI_NULL != pTmpBuf);
    SCI_MEMSET(pTmpBuf, 0, ISP_DRV_SCALE_COEFF_BUF_SIZE);
    
    pHCoeff = pTmpBuf;
    pVCoeff = pTmpBuf + (ISP_DRV_SCALE_COEFF_COEF_SIZE/4);

#ifdef ISP_DRV_SCALE_COEFF_DBG
    SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff i_w/i_h/o_w/o_h = {%d, %d, %d, %d,}",
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
        SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff GenScaleCoeff error!");    
        SCI_ASSERT(0);
    }
	
	if(ISP_PATH1 == idxScx)
	{    
        do
        {   
            //pIspReg->dcam_cfg_u.mBits.path1_clk_switch = ISP_CLK_DOMAIN_AHB;		
		_paod(DCAM_CFG, BIT_3);//ISP_CLK_DOMAIN_AHB
        //}while(!(pIspReg->dcam_cfg_u.mBits.path1_clk_status));
        }while(!((_pard(DCAM_CFG) >> 5) & 0x1));

#ifdef ISP_DRV_SCALE_COEFF_DBG
        SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
            //pIspReg->dcam_cfg_u.mBits.path1_clk_switch);
            _pard(DCAM_CFG) >> 3) & 0x1);
#endif
    }
    else 
    {
        do
        {
            //pIspReg->dcam_cfg_u.mBits.path2_clk_switch = ISP_CLK_DOMAIN_AHB;     
		_paod(DCAM_CFG, BIT_4);//ISP_CLK_DOMAIN_AHB
        //}while(!(pIspReg->dcam_cfg_u.mBits.path2_clk_status));
        }while(!((_pard(DCAM_CFG) >> 6) & 0x1));

#ifdef ISP_DRV_SCALE_COEFF_DBG
        SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
            //pIspReg->dcam_cfg_u.mBits.path1_clk_switch);
            _pard(DCAM_CFG) >> 4) & 0x1);
#endif
    }   
        
    for( i = 0; i < ISP_SCALE_COEFF_H_NUM; i++)
    {
       // *(volatile uint32_t*)HScaleAddr = *pHCoeff;
	_pawd(HScaleAddr, *pHCoeff);

#ifdef ISP_DRV_SCALE_COEFF_DBG
        SCALE_PRINT("ISP_DRV: Coeff H[%d] = 0x%x.\n", i, 
            *pHCoeff); 
#endif

        HScaleAddr += 4;
        pHCoeff++;
    }    
    
    for( i=0 ;i < ISP_SCALE_COEFF_V_NUM; i++)
    {
        //*(volatile uint32_t*)VScaleAddr = *pVCoeff;
        _pawd(VScaleAddr, *pVCoeff);
        
#ifdef ISP_DRV_SCALE_COEFF_DBG
        SCALE_PRINT("ISP_DRV: Coeff V[%d] = 0x%x.\n", i, 
            *pVCoeff);
#endif

        VScaleAddr += 4;
        pVCoeff++;
    }

    if(ISP_PATH1 == idxScx)
    {
    	//pIspReg->dcam_path_cfg_u.mBits.ver_down_tap = (*pVCoeff) & 0x0F;
    	_paad(DCAM_PATH_CFG, ~0xF0);
	_paod(DCAM_PATH_CFG, ( (*pVCoeff) & 0x0F) << 4);
    }
    else
    {
    	//pIspReg->review_path_cfg_u.mBits.ver_down_tap = (*pVCoeff) & 0x0F;
    	_paad(REV_PATH_CFG, ~(0xF << 14));
	_paod(REV_PATH_CFG, ((*pVCoeff) & 0x0F) << 14);
    }

#ifdef ISP_DRV_SCALE_COEFF_DBG
    SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff V[%d] = 0x%x", i, 
        (*pVCoeff) & 0x0F);
    SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff V[%d] = 0x%x", i, 
        //pIspReg->dcam_path_cfg_u.mBits.ver_down_tap);
        (_pard(DCAM_PATH_CFG) >> 4) & 0xF);
#endif
    
    if(ISP_PATH1 == idxScx)        
    {
       // pIspReg->dcam_cfg_u.mBits.path1_clk_switch = ISP_CLK_DOMAIN_DCAM;	
	_paad(DCAM_CFG, ~BIT_3);

#ifdef ISP_DRV_SCALE_COEFF_DBG
        SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
            //pIspReg->dcam_cfg_u.mBits.path1_clk_switch);
            (_pard(DCAM_CFG) >> 3) & 0x1);
#endif
    }
    else
    {
        //pIspReg->dcam_cfg_u.mBits.path2_clk_switch = ISP_CLK_DOMAIN_DCAM;	
        _paad(DCAM_CFG, ~BIT_4);

#ifdef ISP_DRV_SCALE_COEFF_DBG
        SCALE_PRINT("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
            //pIspReg->dcam_cfg_u.mBits.path2_clk_switch);
            (_pard(DCAM_CFG) >> 4) & 0x1);
#endif
    }
    
    //if(pTmpBuf)
    {
        kfree(pTmpBuf);
        pTmpBuf = SCI_NULL;
    }
    
    return ISP_DRV_RTN_SUCCESS;
}

#endif //not define ISP_DRV_SCALE_COEFF_TABLE_EN

LOCAL int32_t _SCALE_DriverSetSC2Coeff(void)
{
#ifdef ISP_DRV_SCALE_COEFF_TABLE_EN
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
    uint32_t                    v_coeff = 0,h_coeff= 0;
    uint32_t                    i = 0;
    uint32_t                    *p_v_coeff_ptr = SCALE_NULL;
    uint32_t                    *p_h_coeff_ptr = SCALE_NULL;	
    uint32_t                    scale_h_addr = g_base_addr + ISP_SCALE2_H_TAB_OFFSET;
    uint32_t                    scale_v_addr = g_base_addr + ISP_SCALE2_V_TAB_OFFSET;	
	
    rtn = _SCALE_DriverCalcScaleCoeff(p_path->sc_input_size.w,
                                    p_path->sc_input_size.h,
                                    p_path->output_size.w,
                                    p_path->output_size.h,
                                    &h_coeff,
                                    &v_coeff);
    if(ISP_DRV_RTN_SUCCESS != rtn)    return rtn;
    SCALE_PRINT("###scale: h_coeff: %d, v_coeff: %d.\n", h_coeff, v_coeff);

    _paod(DCAM_CFG, BIT_4);//ISP_CLK_DOMAIN_AHB
    if(p_path->h_scale_coeff != h_coeff)
    {    
        p_h_coeff_ptr = (uint32_t*)s_isp_h_scale_coeff[h_coeff];
        for( i = 0; i < ISP_SCALE_COEFF_H_NUM; i++)
        {
            _pawd(scale_h_addr, *p_h_coeff_ptr);
            scale_h_addr += 4;
            p_h_coeff_ptr++;
        }    
        p_path->h_scale_coeff = h_coeff; 
    }

    if(p_path->v_scale_coeff != v_coeff)
    {
        p_v_coeff_ptr = (uint32_t*)s_isp_v_scale_coeff[v_coeff];	
        for( i = 0; i < ISP_SCALE_COEFF_V_NUM; i++)
        {        	
	    _pawd(scale_v_addr, *p_v_coeff_ptr);
            scale_v_addr += 4;
            p_v_coeff_ptr++;
        }
        _paad(REV_PATH_CFG, ~(0xF << 14));
	_paod(REV_PATH_CFG, ((*p_v_coeff_ptr) & 0x0F) << 14);
        p_path->v_scale_coeff = v_coeff; 		
    }
    _paad(DCAM_CFG, ~BIT_4);//ISP_CLK_DOMAIN_DCAM
    return rtn;
#else
	return _ISP_DriverGenScxCoeff( ISP_PATH2);
#endif //ISP_DRV_SCALE_COEFF_TABLE_EN
	
}

LOCAL int32_t _SCALE_DriverPath2TrimAndScaling(void)
{
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
    /*trim config*/
    if(p_path->input_size.w != p_path->input_rect.w || 
       p_path->input_size.h != p_path->input_rect.h)
    {
        _paod(REV_PATH_CFG, BIT_1);
    }
    else
    {
        _paad(REV_PATH_CFG, ~BIT_1);
    }

    /*scaling config*/
    rtn = _SCALE_DriverCalcSC2Size(); 
    ISP_RTN_IF_ERR(rtn);	
 
    {
        if(p_path->sub_sample_en)
        {
            _paod(REV_PATH_CFG, BIT_2);
            _paad(REV_PATH_CFG, ~(BIT_9 | BIT_10));//wxz20110726: modify the sub sample mode bits.
	    	_paod(REV_PATH_CFG, (p_path->sub_sample_mode & 0x3) << 9);	
        }
        else
        {
            _paad(REV_PATH_CFG, ~BIT_2);
        }
    }
	
    if(p_path->sc_input_size.w != p_path->output_size.w ||
       p_path->sc_input_size.h != p_path->output_size.h)
    {
        _paad(REV_PATH_CFG, ~BIT_3);
        rtn = _SCALE_DriverSetSC2Coeff(); 
    }
    else
    {
        _paod(REV_PATH_CFG, BIT_3);
    }

    if(p_path->slice_en)
    {
	_paod(REV_PATH_CFG, BIT_4);
    }
    else
  {
    	_paad(REV_PATH_CFG, ~BIT_4);
  }
    return rtn;
}
LOCAL  void _SCALE_DriverIrqEnable(uint32_t mask)
{
    _paod(DCAM_INT_MASK, mask);
}
LOCAL BOOLEAN _SCALE_DrvierIsSysLittleEndian(void)
{
    uint32_t                    cell = 0x5A;
    
    return (*(uint8_t*)&cell == 0x5A);
}

LOCAL int32_t _SCALE_DriverSetMasterEndianness( 
                                           uint32_t master_sel,
                                           uint32_t is_rgb565)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    uint32_t                    endian_sel = ISP_MASTER_ENDIANNESS_LITTLE;
    
    if(master_sel >= ISP_MASTER_MAX)
        return ISP_DRV_RTN_MASTER_SEL_ERR;
        
    if(_SCALE_DrvierIsSysLittleEndian())
    {

        if(is_rgb565)
        {
            endian_sel = ISP_MASTER_ENDIANNESS_HALFBIG;    
        }
        else
        {
            endian_sel = ISP_MASTER_ENDIANNESS_LITTLE;    
        }
       

    }
    else
    {
        endian_sel = ISP_MASTER_ENDIANNESS_BIG;
    }

    if(master_sel == ISP_MASTER_READ)
    {
        _paad(ENDIAN_SEL, ~0x3);
	_paod(ENDIAN_SEL, endian_sel & 0x3);
    }
    else
    {
        _paad(ENDIAN_SEL, ~(0x3 << 2));
	_paod(ENDIAN_SEL, (endian_sel & 0x3) << 2);		
    }


    return rtn;

}


LOCAL void _SCALE_DriverForceCopy(void)
{
    _paod(DCAM_PATH_CFG, BIT_10);
    _paod(DCAM_PATH_CFG, BIT_10);
    _paad(DCAM_PATH_CFG, ~BIT_10);  
    
}
#if 0
LOCAL uint32_t _SCALE_DriverAutoCopy(void)
{
	_paod(DCAM_PATH_CFG, BIT_11);

    return ISP_DRV_RTN_SUCCESS;
}
#endif

LOCAL void _SCALE_DriverIramSwitch(uint32_t base_addr,uint32_t isp_or_arm)
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
LOCAL  void   _SCALE_DrvierModuleReset(uint32_t base_addr)
{
   _paod(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET, BIT_1 | BIT_2);
   _paod(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET, BIT_1 | BIT_2);
   _paad(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET, ~(BIT_1 | BIT_2));
}
LOCAL int32_t _SCALE_DriverModuleEnable(uint32_t ahb_ctrl_addr)// must be AHB general control register base address
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;     

    _paod(ahb_ctrl_addr + ISP_AHB_CTRL_MOD_EN_OFFSET, BIT_1|BIT_2);
   _paad(ahb_ctrl_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, ~BIT_0); // switch memory to ISP
   
    _SCALE_DrvierModuleReset(ahb_ctrl_addr);
	
    return rtn;
}
LOCAL int32_t _SCALE_DriverModuleDisable(uint32_t ahb_ctrl_addr) // must be AHB general control register base address
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    

    _paod(ahb_ctrl_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, BIT_0); // switch memory to ARM    
    _paad(ahb_ctrl_addr + ISP_AHB_CTRL_MOD_EN_OFFSET,~(BIT_1|BIT_2));	

    return rtn;
	
}

LOCAL  void   _SCALE_DriverScalingCoeffReset(void)
{
    s_scale_mod.isp_path1.h_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
    s_scale_mod.isp_path1.v_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
    s_scale_mod.isp_path2.h_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
    s_scale_mod.isp_path2.v_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;	    
}
LOCAL int32_t _SCALE_DriverSoftReset(uint32_t ahb_ctrl_addr)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    

    _SCALE_DriverModuleEnable(ahb_ctrl_addr);

    _SCALE_DrvierModuleReset(ahb_ctrl_addr);    

    _SCALE_DriverScalingCoeffReset();

    return rtn;
}
#if 0
LOCAL int32_t _SCALE_DriverSetClk(uint32_t pll_src_addr,SCALE_CLK_SEL_E clk_sel)
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
	
   // rtn_drv = _SCALE_DriverSetClk(ARM_GLOBAL_PLL_SCR, SCALE_CLK_48M);
    //ISP_RTN_IF_ERR(rtn_drv);

	return rtn_drv;
	
}

LOCAL void _SCALE_DriverDeinit(void)
{
	_SCALE_DriverModuleDisable(AHB_GLOBAL_REG_CTL0);
	_SCALE_DriverIramSwitch(AHB_GLOBAL_REG_CTL0, 1); //switch IRAM to ARM
}

LOCAL int32_t _SCALE_DriverStart(void)
{
    uint32_t rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path2 = &s_scale_mod.isp_path2;

            rtn = _SCALE_DriverPath2TrimAndScaling();            
	    ISP_RTN_IF_ERR(rtn);

            _SCALE_DriverIrqEnable(ISP_IRQ_REVIEW_DONE_BIT);	
            
	    //wxz20110107:the rgb565 endian type from android is ISP_MASTER_ENDIANNESS_HALFBIG	   
	    if((ISP_DATA_RGB565 == p_path2->input_format) || (1 == p_path2->input_frame_endian))
	             _SCALE_DriverSetMasterEndianness(ISP_MASTER_READ,1);
	    else
			_SCALE_DriverSetMasterEndianness(ISP_MASTER_READ,0);            
            SCALE_PRINT("ISP_DRV: ISP_DriverStart , p_path2->input_format %d ,p_path2->output_format %d.\n",
                          p_path2->input_format,
                          p_path2->output_format);
	    //wxz20110107:update the endian for LCDC display in copybit function. The blending needs the ISP_MASTER_ENDIANNESS_HALFBIG endian type of rgb565.     
	  if((ISP_DATA_RGB565 == p_path2->output_format) ||(1 == p_path2->output_frame_endian))
	             _SCALE_DriverSetMasterEndianness(ISP_MASTER_WRITE,1);		
	    else
  		     _SCALE_DriverSetMasterEndianness(ISP_MASTER_WRITE,0);  	 

	SCALE_PRINT("####ENDIAN_SEL: 0x%x, in : %d, out: %d.\n", _pard(ENDIAN_SEL), p_path2->input_frame_endian, p_path2->output_frame_endian);

	if(1 == p_path2->slice_en){
		if(p_path2->slice_height < p_path2->output_size.h){
			p_path2->is_last_slice = 0;
			_paad(SLICE_VER_CNT, ~BIT_12);
		}
		else{
			p_path2->is_last_slice = 1;
			_paod(SLICE_VER_CNT, BIT_12);
		}
	}
            
            _SCALE_DriverForceCopy();
#if 0 
	    get_scale_reg();
#endif

            _paod(REV_PATH_CFG, BIT_0);
   	
	SCALE_PRINT("###scale: DriverStart is OK.\n"); 
    return rtn;

}

LOCAL  void _SCALE_DriverIrqClear(uint32_t mask)
{
  _paod(DCAM_INT_CLR, mask);
  SCALE_PRINT("###scale: _SCALE_DriverIrqClear, DCAM_INT_CLR:0x%x,mask:0x%x.\n", _pard(DCAM_INT_CLR),mask); 
}

LOCAL  void _SCALE_DriverIrqDisable(uint32_t mask)
{
  _paad(DCAM_INT_MASK, ~mask);
}

LOCAL int32_t _SCALE_DriverStop(void)
{
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;

	dcam_dec_user_count();
     if( 0 == dcam_get_user_count())
     {
     	_SCALE_DriverDeinit();
     }

      _paad(REV_PATH_CFG, ~BIT_0);
	//msleep(20); //wait the review path stop.  //wxz:???
	memset(&s_scale_mod, 0, sizeof(ISP_MODULE_T));

    _SCALE_DriverIrqDisable(ISP_IRQ_SCL_LINE_MASK);
    _SCALE_DriverIrqClear(ISP_IRQ_SCL_LINE_MASK);	
	
	SCALE_PRINT("###scale: DriverStop is OK.\n"); 
	
    return rtn;
}

LOCAL int32_t _SCALE_DriverSetMode(void)
{
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;

   _paod(DCAM_CFG, BIT_2); //review path enable
   _paad(DCAM_CFG, ~BIT_1);
   
    return rtn;
}
#if 0
static void set_layer_cb(uint32_t base)
{
#define LCDC_Y2R_CONTRAST           (SPRD_LCDC_BASE + 0x0114)
#define LCDC_Y2R_SATURATION         (SPRD_LCDC_BASE + 0x0118)
#define LCDC_OSD1_CTRL                    (SPRD_LCDC_BASE + 0x0040)
#define LCDC_OSD1_ALPHA                     (SPRD_LCDC_BASE + 0x0058)
#define LCDC_IMG_CTRL                    (SPRD_LCDC_BASE + 0x0020)
#define LCDC_IMG_Y_BASE_ADDR         (SPRD_LCDC_BASE + 0x0024)
#define LCDC_IMG_UV_BASE_ADDR            (SPRD_LCDC_BASE + 0x0028)
#define LCDC_IMG_SIZE_XY             (SPRD_LCDC_BASE + 0x002c)
#define LCDC_IMG_PITCH                   (SPRD_LCDC_BASE + 0x0030)
#define LCDC_IMG_DISP_XY             (SPRD_LCDC_BASE + 0x0034)

       __raw_bits_or((1<<2), LCDC_OSD1_CTRL);//block alpha
        __raw_writel(0xF,LCDC_OSD1_ALPHA);

        //__raw_writel(0x23,LCDC_IMG_CTRL);
        //__raw_writel(0x14B,LCDC_IMG_CTRL);
        //__raw_writel(0xA1,LCDC_IMG_CTRL);
        __raw_writel(0x23,LCDC_IMG_CTRL);

        __raw_writel(64,LCDC_Y2R_CONTRAST);
        __raw_writel(64,LCDC_Y2R_SATURATION);

        __raw_writel(base >> 2,LCDC_IMG_Y_BASE_ADDR);
        __raw_writel((base+640*480)>>2,LCDC_IMG_UV_BASE_ADDR);
        __raw_writel((480<<16)|640,LCDC_IMG_SIZE_XY);
        __raw_writel(640,LCDC_IMG_PITCH);
        __raw_writel(0x0,LCDC_IMG_DISP_XY);
	printk("set_layer_cb: A1base : 0x%x.\n", base);
}
#endif
static void _SCALE_DriverDMAIrq(int dma_ch, void *dev_id)
{
        condition = 1;
	SCALE_PRINT("_SCALE_DriverDMAIrq() E.\n");
	wake_up_interruptible(&wait_queue);
	SCALE_PRINT("_SCALE_DriverDMAIrq() X.\n");
}
static void _SCALE_DriverDMAZoomIrq(int dma_ch, void *dev_id)
{
        condition_zoom = 1;
	SCALE_PRINT("_SCALE_DriverDMAZoomIrq() E.\n");	
	wake_up_interruptible(&wait_queue_zoom);
	SCALE_PRINT("_SCALE_DriverDMAZoomIrq() X.\n");
}
static void _SCALE_DriverDMAEndianIrq(int dma_ch, void *dev_id)
{
        condition_endian = 1;
	SCALE_PRINT("_SCALE_DriverDMAEndianIrq() E.\n");	
	wake_up_interruptible(&wait_queue_endian);
	SCALE_PRINT("_SCALE_DriverDMAEndianIrq() X.\n");
}
static int _SCALE_DriverZoomByDMA(uint32_t width, uint32_t height, uint32_t input_addr, uint32_t output_addr)
{
	sprd_dma_ctrl ctrl;
	sprd_dma_desc dma_desc;
	uint32_t byte_per_pixel = 1;
	uint32_t src_img_postm = 0;
	uint32_t dst_img_postm = 0;
	uint32_t src_addr = input_addr;
	uint32_t dst_addr = output_addr;
	uint32_t block_len = width * byte_per_pixel;
	uint32_t total_len = width * height * byte_per_pixel;
        int32_t ret = 0;

       SCALE_PRINT("_SCALE_DriverZoomByDMA [pid:%d]   %d,%d,%x,%x\n", current->pid,width,height,input_addr,output_addr);

	while(1){		
		ret  = sprd_request_dma(DMA_SOFT0, _SCALE_DriverDMAZoomIrq, &s_scale_mod);
        	if(ret){
        		printk("ZoomByDMA SCALE: request dma fail.ret : %d.\n", ret);
        		msleep(5);        		
        	}
		else{
			SCALE_PRINT("SCALE: request dma OK.\n");
			break;
		}			
	}
	condition_zoom = 0;
	memset(&ctrl, 0, sizeof(sprd_dma_ctrl));
	memset(&dma_desc, 0, sizeof(sprd_dma_desc));
	ctrl.dma_desc = &dma_desc;	
	sprd_dma_setup_cfg(&ctrl,
                DMA_SOFT0,
                DMA_NORMAL,
                TRANS_DONE_EN,
                DMA_INCREASE, DMA_INCREASE,
                SRC_BURST_MODE_8|src_img_postm, SRC_BURST_MODE_8|dst_img_postm,
                block_len,
                byte_per_pixel*8, byte_per_pixel*8,
                src_addr, dst_addr, total_len);
	ctrl.dma_desc->cfg |= (0x3 << 28);
	SCALE_PRINT("ZoomByDMA ctrl.dma_desc->cfg: 0x%x.\n", ctrl.dma_desc->cfg);
	sprd_dma_setup(&ctrl); 
	 sprd_dma_start(DMA_SOFT0);	 
	__raw_bits_or(1 << DMA_SOFT0, DMA_SOFT_REQ);
	 if(wait_event_interruptible(wait_queue_zoom, condition_zoom)){
	 	ret =  -EFAULT;
	 }
	 sprd_dma_stop(DMA_SOFT0);
	 sprd_free_dma(DMA_SOFT0);
	 return ret;
}
void _SCALE_ZoomDMA(void)
{
	uint32_t count, height;
	if(0 == g_zoom_dma_buf.by_dma)
		return;
	
	count = _pard(SLICE_VER_CNT);	
	count = (count >> 16)  & 0xFFF;
	height = count - g_zoom_dma_buf.last_line_cnt;
	SCALE_PRINT("###SCALE: count: %d, hei: %d.\n", count, height);
	_SCALE_DriverZoomByDMA(g_zoom_dma_buf.width, height, g_zoom_dma_buf.in_y_addr,  g_zoom_dma_buf.out_y_addr);
	_SCALE_DriverZoomByDMA(g_zoom_dma_buf.width, height, g_zoom_dma_buf.in_uv_addr,  g_zoom_dma_buf.out_uv_addr);	
}
uint32_t _SCALE_IsContinueSlice(void)
{
	ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
	uint32_t count = 0;

	if(1 == p_path->slice_en){
		_SCALE_ZoomDMA();
		count = _pard(SLICE_VER_CNT);		
		p_path->slice_line_count = (count >> 16)  & 0xFFF;
		SCALE_PRINT("###SCALE: current slice count: %d, line  output : %d, out_h: %d.\n",p_path->slice_count,  p_path->slice_line_count, p_path->output_size.h);
		if(p_path->slice_line_count < p_path->output_size.h){
			p_path->slice_count++;
			return 1;
		}
		else{
			p_path->slice_en = 0;
			g_zoom_dma_buf.by_dma = 0;
			return 0;
		}
	}
	else{
		return 0;		
	}
}


//int32_t _SCALE_ContinueSlice(void)
void _SCALE_ContinueSlice(long unsigned int data)
{
	ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;
	ISP_FRAME_T next_frame;

	_paad(REV_PATH_CFG, ~BIT_0);

	//for YUV422
	//note: these addresses must be aligned by 64 words.
	next_frame.yaddr = p_path->input_frame.yaddr + p_path->slice_count * p_path->slice_height * p_path->input_size.w;
	if( p_path->input_format ==ISP_DATA_YUV422)
	{
		next_frame.uaddr = p_path->input_frame.uaddr + p_path->slice_count * p_path->slice_height * p_path->input_size.w;
	}
	else
	{
		next_frame.uaddr = p_path->input_frame.uaddr + (p_path->slice_count * p_path->slice_height * p_path->input_size.w)/2;
	}
	next_frame.vaddr = next_frame.uaddr;
	_SCALE_DriverSetExtSrcFrameAddr(&next_frame);
	SCALE_PRINT("###SCALE: input addr: y: 0x%x, u: 0x%x, in w: %d, slice hei: %d.\n", p_path->input_frame.yaddr, p_path->input_frame.uaddr, p_path->input_size.w, p_path->slice_height);
	SCALE_PRINT("###SCALE: count: %d, next slice input buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", p_path->slice_count, next_frame.yaddr, next_frame.uaddr, next_frame.vaddr);
	next_frame.yaddr = p_path->output_frame.yaddr + p_path->slice_line_count * p_path->output_size.w;	
	next_frame.uaddr = p_path->output_frame.uaddr + p_path->slice_line_count * p_path->output_size.w;	
	//to check the address if they are aligned by 64 wods
	if((next_frame.yaddr & 0xFF) || (next_frame.uaddr & 0xFF))
	{
		g_zoom_dma_buf.by_dma = 1;
		g_zoom_dma_buf.out_y_addr = next_frame.yaddr;
		g_zoom_dma_buf.out_uv_addr = next_frame.uaddr;
		next_frame.yaddr = ZOOM_BUF_ALIGNED(next_frame.yaddr);
		next_frame.uaddr = ZOOM_BUF_ALIGNED(next_frame.uaddr);
		g_zoom_dma_buf.in_y_addr = next_frame.yaddr;
		g_zoom_dma_buf.in_uv_addr = next_frame.uaddr;	
		g_zoom_dma_buf.width = p_path->output_size.w;
		g_zoom_dma_buf.last_line_cnt = p_path->slice_line_count;		
	}
	else{
		g_zoom_dma_buf.by_dma = 0;
		g_zoom_dma_buf.last_line_cnt = 0;
	}
	next_frame.vaddr = next_frame.uaddr;
	_SCALE_DriverSetExtDstFrameAddr(&next_frame);
	SCALE_PRINT("###SCALE: output addr: y: 0x%x, u: 0x%x, out w: %d.\n", p_path->output_frame.yaddr, p_path->output_frame.uaddr, p_path->output_size.w);
	SCALE_PRINT("###SCALE: count: %d, next slice output buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", p_path->slice_count, next_frame.yaddr, next_frame.uaddr, next_frame.vaddr);


	if(p_path->slice_count == (p_path->input_rect.h / p_path->slice_height)){
		p_path->slice_height = p_path->input_rect.h - p_path->slice_count * p_path->slice_height;
	        _paad(SLICE_VER_CNT, ~0xFFF);
	    	_paod(SLICE_VER_CNT, (p_path->slice_height & 0xFFF) | BIT_12);
		p_path->is_last_slice = 1;
		SCALE_PRINT("###SCALE: the last slice height: %d.\n", p_path->slice_height);
	}
	//_SCALE_DriverIrqClear(BIT_9);
	_SCALE_DriverPath2TrimAndScaling();   
	_SCALE_DriverForceCopy();
//	get_scale_reg();
	SCALE_PRINT("####_SCALE_ContinueSlice ENDIAN_SEL: 0x%x.\n", _pard(ENDIAN_SEL));
	_paod(REV_PATH_CFG, BIT_0);	
	//return 0;
}
LOCAL void  _SCALE_ISRPath2Done(void)
{  
#if 0
	set_layer_cb(0xF800000);
	SCALE_PRINT("###_SCALE_ISRPath2Done: set_layer_cb.\n");
#endif
	up(&g_sem);
	/*if(0 == _SCALE_IsContinueSlice()){
		up(&g_sem);
	}
	else{
		_SCALE_ContinueSlice(0);
		//tasklet_schedule(&my_tasklet);
	}	*/
    SCALE_PRINT("###SCALE: path2 done IRQ.\n");
	
    return ;
}

static int32_t _SCALE_DriverPath2Config(ISP_CFG_ID_E id, void* param)
{
    uint32_t             rtn = ISP_DRV_RTN_SUCCESS;
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
            uint32_t format = (ISP_DATA_FORMAT_E)*(uint32_t*)param;
            
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
	    copy_from_user(&p_size, (ISP_SIZE_T *)param, sizeof(ISP_SIZE_T));			
      	        	  
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
            ISP_RECT_T p_rect;// = (ISP_RECT_T*)param;
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
            ISP_ADDRESS_T p_addr;// = (ISP_ADDRESS_T*)param;
            
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
	    copy_from_user(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));			
            
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
            ISP_SIZE_T p_size; // = (ISP_SIZE_T*)param;
          
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
	    copy_from_user(&p_size, (ISP_SIZE_T *)param, sizeof(ISP_SIZE_T));			
        	  
        	  
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
            uint32_t format = (ISP_DATA_FORMAT_E)*(uint32_t*)param;
            
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
            ISP_ADDRESS_T p_addr; // = (ISP_ADDRESS_T*)param;
        	
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
	    copy_from_user(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));	
            {	
                p_path->output_frame.yaddr = p_addr.yaddr;
                p_path->output_frame.uaddr = p_addr.uaddr;
                p_path->output_frame.vaddr = p_addr.vaddr;
            }		

		SCALE_PRINT("###scale: output addr: %x, %x, %x.\n", p_addr.yaddr, p_addr.uaddr, p_addr.vaddr);
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
        case ISP_PATH_SWAP_BUFF:
        {
            ISP_ADDRESS_T p_addr;
            
            SCALE_CHECK_PARAM_ZERO_POINTER(param);
	    memcpy(&p_addr, (ISP_ADDRESS_T *)param, sizeof(ISP_ADDRESS_T));	
	 	_pawd(FRM_ADDR_2, (p_addr.yaddr >> 8) & 0x3FFFF);
		_pawd(FRM_ADDR_3, (p_addr.uaddr >> 8) & 0x3FFFF);
		_pawd(FRM_ADDR_6, (p_addr.vaddr >> 8) & 0x3FFFF);
            SCALE_PRINT("###SCALE: swap and line buffer address: y: 0x%x, u: 0x%x, v: 0x%x.\n", p_addr.yaddr, p_addr.uaddr, p_addr.vaddr);
            break;
        }
        case ISP_PATH_SLICE_SCALE_EN:
            _paod(REV_PATH_CFG, BIT_4); //ISP_SCALE_SLICE
            p_path->slice_en = 1;
	    p_path->slice_line_count = 0;
	    p_path->slice_count = 0;
     	    tasklet_init(&my_tasklet, _SCALE_ContinueSlice, 0);
            break;		
        /*case ISP_PATH_SLICE_SCALE_HEIGHT:
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
		_paod(SLICE_VER_CNT, (slice_height & 0xFFF | BIT_12);
		p_path->is_last_slice = 1;
	    }			
            break;			
        }*/
       case ISP_PATH_SLICE_SCALE_HEIGHT:
        {
            uint32_t slice_height;
			
            SCALE_CHECK_PARAM_ZERO_POINTER(param);

            slice_height = (*(uint32_t*)param) & ISP_PATH_SLICE_MASK;
	    SCALE_PRINT("###scale:SLICE_VER_CNT: 0x%x.\n", _pard(SLICE_VER_CNT));
	    //_paad(SLICE_VER_CNT, ~0xFFF);
	    _paad(SLICE_VER_CNT, ~0xFFF0FFF);
	    _paod(SLICE_VER_CNT, slice_height & 0xFFF);
            p_path->slice_en = 1;
	    p_path->slice_height = slice_height;
	    SCALE_PRINT("###scale: slice height %d, SLICE_VER_CNT: 0x%x.\n",p_path->slice_height, _pard(SLICE_VER_CNT));
            break;			
        }		
        case ISP_PATH_DITHER_EN:
            _paod(REV_PATH_CFG, BIT_8);
            break;
	case ISP_PATH_OUTPUT_ENDIAN:
	{
		uint32_t endian = *(uint32_t*)param;
		p_path->output_frame_endian = endian;
		break;
	}
	case ISP_PATH_INPUT_ENDIAN:
	{
		uint32_t endian = *(uint32_t*)param;
		p_path->input_frame_endian = endian;
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
            uint32_t format = (ISP_DATA_FORMAT_E)*(uint32_t*)param;
            
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
            uint32_t format = (ISP_DATA_FORMAT_E)*(uint32_t*)param;
            
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
	case ISP_PATH_OUTPUT_ENDIAN:
	{
		uint32_t endian = *(uint32_t*)param;
		p_path->output_frame_endian = endian;
		break;
	}
	case ISP_PATH_INPUT_ENDIAN:
	{
		uint32_t endian = *(uint32_t*)param;
		p_path->input_frame_endian = endian;
		break;
	}				
        default:
            rtn = ISP_DRV_RTN_PARA_ERR;
            break;
    }	    

    return rtn;
	
}

typedef void (*isr_func_t)(void);

void _SCALE_DriverEnableInt(void)
{
  _paod(SCL_INT_IRQ_EN, 1UL << 27);
}
void _SCALE_DriverDisableInt(void)
{
  _paod(SCL_INT_IRQ_DISABLE, 1UL << 27);
}
LOCAL  uint32_t _SCALE_DriverReadIrqLine(void)
{
   return _pard(DCAM_INT_STS);
}
typedef void (*ISP_ISR_PTR)(void);
LOCAL ISP_ISR_PTR s_isp_isr_list[ISP_IRQ_NUMBER]=
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

LOCAL void _SCALE_DriverISRRoot(void)
{
    uint32_t                      irq_line, irq_status;
    uint32_t                      i;
	//ISP_PATH_DESCRIPTION_T    *p_path = &s_scale_mod.isp_path2;

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
	_SCALE_DriverIrqClear(irq_status);
	/*if(0 == p_path->slice_en){
	    	_SCALE_DriverIrqClear(irq_status);
	}
	else{
		irq_status &= ~BIT_9;
		_SCALE_DriverIrqClear(irq_status );
	}*/
	
}
LOCAL ISR_EXE_T _SCALE_ISRSystemRoot(uint32_t param)
{
    _SCALE_DriverISRRoot();

    return ISR_DONE;
}

static irqreturn_t _SCALE_DriverISR(int irq, void *dev_id)
{
	uint32_t value = (_pard(DCAM_INT_STS) >> 9) & 0x1;//*(uint32_t *)dev_id;
	if(1 != value)
	{
		//SCALE_PRINT("###scale:irq exit.\n");
		return IRQ_NONE;
	}
  _SCALE_ISRSystemRoot(0);
  return IRQ_HANDLED;
}
PUBLIC int _SCALE_DriverRegisterIRQ(void)
{
  uint32_t ret = 0;

  //enable dcam interrupt bit on global level.
//_SCALE_DriverEnableInt(); //wxz:???

  if(0 != (ret = request_irq(IRQ_LINE_DCAM, _SCALE_DriverISR, SA_SHIRQ, "SCALE", &g_share_irq)))
    SCALE_ASSERT(0);

  return ret;
}
PUBLIC void _SCALE_DriverUnRegisterIRQ(void)
{
  //disable dcam interrupt bit on global level.
  //_SCALE_DriverDisableInt();

  //unregister_interrupt(IRQ_LINE_DCAM);
  free_irq(IRQ_LINE_DCAM, &g_share_irq);
}

int _SCALE_DriverIOInit(void)
{
	down(&g_sem_cnt);
	if(0 < g_scale_num)
	{
		SCALE_PRINT_ERR("###scale: fail to open device.g_scale_num: %d.\n",g_scale_num);
		return -1;
	}

	init_MUTEX(&g_sem);
	memset(&s_scale_mod, 0, sizeof(ISP_MODULE_T));

	g_scale_num++;
	
	g_scale_clk = clk_get(NULL, "clk_dcam");
	if(IS_ERR(g_scale_clk)){
		SCALE_PRINT_ERR("###scale: Failed: Can't get clock [clk_dcam]!\n");
		SCALE_PRINT_ERR("###scale: g_scale_clk = %p.\n", g_scale_clk);
	}
	else{
		SCALE_PRINT("###scale g_scale_clk clk_get ok.g_dcam_clk->parent->usecount: %d.\n", g_scale_clk->parent->usecount);
	}	
	
	if( 0 == dcam_get_user_count())
     	{
     		_SCALE_DriverInit();
		//SCALE_PRINT_ERR("###scale:_SCALE_DriverInit.\n");
     	}
	if(0 != _SCALE_DriverSetMclk()){
		SCALE_PRINT_ERR("###scale  Failed to _SCALE_DriverSetMclk!\n");
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

	up(&g_sem_cnt);
	
	if(g_scale_clk){
		clk_disable(g_scale_clk);
		SCALE_PRINT("###scale g_scale_clk clk_disable ok.\n");
		clk_put(g_scale_clk);
		SCALE_PRINT("###scale g_dcam_clk clk_put ok.\n");
		g_scale_clk = NULL;
	}
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
	_SCALE_DriverSetMode();
	_SCALE_DriverRegisterIRQ();
	_SCALE_DriverStart();
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
void get_DMA_regs(void)
{
	uint32_t i, value;
	
	for(i = 0; i <= 0xC4; i+=4)
	{
		value = _pard(DMA_REG_BASE + i);
		SCALE_PRINT("DMA reg: 0x%x : 0x%x.\n", DMA_REG_BASE + i, value);
	}
	for(i = 0; i <= 0x1C; i+=4)
	{
		value = _pard(DMA_CHx_CTL_BASE + i);
		SCALE_PRINT("DMA reg: 0x%x : 0x%x.\n", DMA_CHx_CTL_BASE + i, value);
	}
}

static int _SCALE_DriverColorConvertByDMA(uint32_t width, uint32_t height, uint32_t input_addr, uint32_t output_addr)
{
	sprd_dma_ctrl ctrl;
	sprd_dma_desc dma_desc;
	uint32_t byte_per_pixel = 1;
	uint32_t src_img_postm = width * byte_per_pixel;
	uint32_t dst_img_postm = 0;
	uint32_t src_addr = input_addr;
	uint32_t dst_addr = output_addr;
	uint32_t block_len = width * byte_per_pixel;
	uint32_t total_len = width * height * byte_per_pixel / 2;
        int32_t ret = 0;

       SCALE_PRINT("[pid:%d]   %d,%d,%x,%x\n", current->pid,width,height,input_addr,output_addr);

	while(1){		
		ret  = sprd_request_dma(DMA_SOFT0, _SCALE_DriverDMAIrq, &g_scale_mode);
        	if(ret){
        		printk("ColorConvertByDMA SCALE: request dma fail.ret : %d.\n", ret);
        		msleep(5); 
        	}
		else{
			break;
		}			
	}
	condition = 0;
	memset(&ctrl, 0, sizeof(sprd_dma_ctrl));
	memset(&dma_desc, 0, sizeof(sprd_dma_desc));
	ctrl.dma_desc = &dma_desc;		
	sprd_dma_setup_cfg(&ctrl,
                DMA_SOFT0,
                DMA_NORMAL,
                TRANS_DONE_EN,
                DMA_INCREASE, DMA_INCREASE,
                SRC_BURST_MODE_8|src_img_postm, SRC_BURST_MODE_8|dst_img_postm,
                block_len,
                byte_per_pixel*8, byte_per_pixel*8,
                src_addr, dst_addr, total_len);		
	//ctrl.dma_desc->cfg |= (0x1 << 28);
	SCALE_PRINT("ColorConvertByDMA ctrl.dma_desc->cfg: 0x%x.\n", ctrl.dma_desc->cfg);
	 sprd_dma_setup(&ctrl); 
	 sprd_dma_start(DMA_SOFT0);	 
	__raw_bits_or(1 << DMA_SOFT0, DMA_SOFT_REQ);	
	 if(wait_event_interruptible(wait_queue, condition)){
	 	ret =  -EFAULT;
	 }
	 
	 sprd_dma_stop(DMA_SOFT0);
	 sprd_free_dma(DMA_SOFT0);
	 return ret;
}
static int _SCALE_DriverYUV422TYUV420(SCALE_YUV422_YUV420_T *yuv_config)
{
	uint32_t width = yuv_config->width;
	uint32_t height = yuv_config->height;
	uint32_t src_addr = yuv_config->src_addr;
	uint32_t dst_addr = yuv_config->dst_addr;

	_SCALE_DriverRegisterIRQ();

	return _SCALE_DriverColorConvertByDMA(width, height, src_addr, dst_addr);
}
static int _SCALE_DriverConvertEndianByDMA(uint32_t width, uint32_t height, uint32_t input_addr, uint32_t output_addr)
{
	sprd_dma_ctrl ctrl;
	sprd_dma_desc dma_desc;
	uint32_t byte_per_pixel = 1;
	uint32_t src_img_postm = 0;
	uint32_t dst_img_postm = 0;
	uint32_t src_addr = input_addr;
	uint32_t dst_addr = output_addr;
	uint32_t block_len = width * byte_per_pixel;
	uint32_t total_len = width * height * byte_per_pixel * 3 /2;
        int32_t ret = 0;

       SCALE_PRINT("convert endian [pid:%d]   %d,%d,%x,%x\n", current->pid,width,height,input_addr,output_addr);

	while(1){		
		ret  = sprd_request_dma(DMA_SOFT0, _SCALE_DriverDMAEndianIrq, &s_scale_mod);
        	if(ret){
        		printk("SCALE: convert endian request dma fail.ret : %d.\n", ret);
        		msleep(5); 
        	}
		else{
			SCALE_PRINT("SCALE: convert endian request dma OK.\n");
			break;
		}			
	}
	condition_endian = 0;
	memset(&ctrl, 0, sizeof(sprd_dma_ctrl));
	memset(&dma_desc, 0, sizeof(sprd_dma_desc));
	ctrl.dma_desc = &dma_desc;	
	sprd_dma_setup_cfg(&ctrl,
                DMA_SOFT0,
                DMA_NORMAL,
                TRANS_DONE_EN,
                DMA_INCREASE, DMA_INCREASE,
                SRC_BURST_MODE_8|src_img_postm, SRC_BURST_MODE_8|dst_img_postm,
                block_len,
                byte_per_pixel*8, byte_per_pixel*8,
                src_addr, dst_addr, total_len);
	ctrl.dma_desc->cfg |= (0x2 << 28);
	SCALE_PRINT("EndianByDMA ctrl.dma_desc->cfg: 0x%x.\n", ctrl.dma_desc->cfg);
	sprd_dma_setup(&ctrl); 
	 sprd_dma_start(DMA_SOFT0);	 
	__raw_bits_or(1 << DMA_SOFT0, DMA_SOFT_REQ);
	 if(wait_event_interruptible(wait_queue_endian, condition_endian)){
	 	ret =  -EFAULT;
	 }
	 sprd_dma_stop(DMA_SOFT0);
	 sprd_free_dma(DMA_SOFT0);
	 return ret;
}
static int _SCALE_DriverEndianHalf2Little(SCALE_YUV420_ENDIAN_T *yuv_config)
{
	uint32_t width = yuv_config->width;
	uint32_t height = yuv_config->height;
	uint32_t src_addr = yuv_config->src_addr;
	uint32_t dst_addr = yuv_config->dst_addr;

	_SCALE_DriverRegisterIRQ();

	return _SCALE_DriverConvertEndianByDMA(width, height, src_addr, dst_addr);
}

static int SCALE_ioctl(struct inode *node, struct file *fl, unsigned int cmd, unsigned long param)
{	
	switch(cmd)
	{
	case SCALE_IOC_CONFIG:
		{			
			SCALE_CONFIG_T path2_config;			
			copy_from_user(&path2_config, (SCALE_CONFIG_T *)param, sizeof(SCALE_CONFIG_T));			
			_SCALE_DriverPath2Config(path2_config.id,  path2_config.param);
		}
		break;		
	case SCALE_IOC_DONE:
		_SCALE_DriverIODone();
		break;
	case SCALE_IOC_YUV422_YUV420:
		{			
			SCALE_YUV422_YUV420_T yuv_config;			
			copy_from_user(&yuv_config, (SCALE_YUV422_YUV420_T *)param, sizeof(SCALE_YUV422_YUV420_T));			
			_SCALE_DriverYUV422TYUV420(&yuv_config);
		}
		break;	
	case SCALE_IOC_YUV420_ENDIAN:
		{			
			SCALE_YUV420_ENDIAN_T yuv_config;			
			copy_from_user(&yuv_config, (SCALE_YUV420_ENDIAN_T *)param, sizeof(SCALE_YUV420_ENDIAN_T));			
			_SCALE_DriverEndianHalf2Little(&yuv_config);
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
	init_waitqueue_head(&wait_queue);
	init_waitqueue_head(&wait_queue_zoom);
	init_waitqueue_head(&wait_queue_endian);
	SCALE_PRINT("###scale: init wait_queue_zoom.\n");

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

	init_MUTEX(&g_sem_cnt);	

	return 0;
}

void scale_exit(void)
{
	platform_driver_unregister(&scale_driver);
	mutex_destroy(lock);
}

module_init(scale_init);
module_exit(scale_exit);

MODULE_DESCRIPTION("Scale Driver");
MODULE_LICENSE("GPL");
