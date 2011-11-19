/*
* drivers/media/video/sprd_dcam/dcam_drv_sc8800g2.c
 * Dcam driver based on sc8800g2
 *
 * Copyright (C) 2011 Spreadtrum 
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
 
#include "dcam_drv_sc8800g2.h"
#include <linux/slab.h>

//#define ISP_DRV_SCALE_COEFF_TABLE_EN

#ifdef ISP_DRV_SCALE_COEFF_TABLE_EN
#include "dcam_sc8800g_scalecoeff_h.h"
#include "dcam_sc8800g_scalecoeff_v.h"
#else
#include "gen_scale_coef.h"
//#define ISP_DRV_SCALE_COEFF_DBG
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

#define SA_SHIRQ IRQF_SHARED
#define IRQ_LINE_DCAM 27 //irq line number in system
#define NR_DCAM_ISRS 12

typedef void (*ISP_ISR_PTR)(uint32_t base_addr);

#define ISP_SCALE1_H_TAB_OFFSET                        0x200
#define ISP_SCALE1_V_TAB_OFFSET                        0x2F0
#define ISP_SCALE2_H_TAB_OFFSET                        0x400
#define ISP_SCALE2_V_TAB_OFFSET                        0x4F0

#define ISP_IRQ_LINE_MASK                              0x1FFUL //0x000003FFUL
#define ISP_IRQ_SENSOR_SOF_BIT                         BIT_0
#define ISP_IRQ_SENSOR_EOF_BIT                         BIT_1
#define ISP_IRQ_CAP_SOF_BIT                            BIT_2
#define ISP_IRQ_CAP_EOF_BIT                            BIT_3
#define ISP_IRQ_CMR_DONE_BIT                           BIT_4
#define ISP_IRQ_CAP_BUF_OV_BIT                         BIT_5
#define ISP_IRQ_SENSOR_LE_BIT                          BIT_6
#define ISP_IRQ_SENSOR_FE_BIT                          BIT_7
#define ISP_IRQ_JPG_BUF_OV_BIT                         BIT_8
#define ISP_IRQ_REVIEW_DONE_BIT                        BIT_9

#define ISP_CAP_FRAME_SKIP_NUM_MAX                     0x40
#define ISP_CAP_FRAME_DECI_FACTOR_MAX                  0x10
#define ISP_CAP_X_DECI_FACTOR_MAX                      0x10
#define ISP_CAP_Y_DECI_FACTOR_MAX                      0x10
#define ISP_CAP_FRAME_WIDTH_MAX                        8192 //13bits for register
#define ISP_CAP_FRAME_HEIGHT_MAX                       4092
#define ISP_CAP_JPEG_DROP_NUM_MIN                      0x01 
#define ISP_CAP_JPEG_DROP_NUM_MAX                      0x10 
#define ISP_PATH_FRAME_HIGH_BITS                       6
#define ISP_PATH_FRAME_WIDTH_MAX                       4092
#define ISP_PATH_FRAME_HEIGHT_MAX                      4092
#define ISP_PATH_SUB_SAMPLE_MAX                        3  //0.....1/2, 1......1/4, 2......1/8, 3.......1/16
#define ISP_PATH_SUB_SAMPLE_FACTOR_BASE                1  // no subsample
#define ISP_PATH_SCALE_LEVEL                           64
#define ISP_PATH_SCALE_LEVEL_MAX                       256
#define ISP_PATH_SCALE_LEVEL_MIN                       16
#define ISP_PATH_SLICE_MASK                            0xFFF

#define ISP_CLK_DOMAIN_AHB                             1
#define ISP_CLK_DOMAIN_DCAM                            0

#define ISP_PATH_INVALID_AHB_ADDR                      0x800 // invalide address if in low 2k bytes 

#define ISP_PATH_ADDR_INVALIDE(addr)                 (SCI_NULL != (addr))
#define ISP_PATH_YUV_ADDR_INVALIDE(y,u,v)              (ISP_PATH_ADDR_INVALIDE(y) && ISP_PATH_ADDR_INVALIDE(u) && ISP_PATH_ADDR_INVALIDE(v))
	
#define ISP_DRV_RTN_IF_ERR                             if(rtn) return rtn
#ifndef SCI_NULL
#define SCI_NULL                                      0 
#endif

#define ISP_CHECK_PARAM_ZERO_POINTER(n)                do{if(SCI_NULL == (int)(n)) return ISP_DRV_RTN_PARA_ERR;}while(0)

#define ISP_DATA_CLEAR(a)                              do{memset((void *)(a),0,sizeof(*(a)));}while(0);
#define ISP_MEMCPY                                     memcpy

static uint32_t g_share_irq = 0x111;
enum
{

    ISP_IRQ_SENSOR_SOF = 0,
    ISP_IRQ_SENSOR_EOF,		
    ISP_IRQ_CAP_SOF,
    ISP_IRQ_CAP_EOF,    
    ISP_IRQ_PATH1_DONE, 
    ISP_IRQ_CAP_FIFO_OF,    
    ISP_IRQ_SENSOR_LINE_ERR,		
    ISP_IRQ_SENSOR_FRAME_ERR,		
    ISP_IRQ_JPEG_BUF_OF,		
    ISP_IRQ_PATH2_DONE,    
    ISP_IRQ_NUMBER
};

enum
{
    ISP_PATH_DATA_FORMAT_YUV = 0,
    ISP_PATH_DATA_FORMAT_RGB 
};

enum 
{
    DCAM_CAP_MODE_SINGLE=0,
    DCAM_CAP_MODE_MULTIPLE,
    DCAM_CAP__MODE_MAX
};

enum
{
    ISP_FRAME_UNLOCK = 0,
    ISP_FRAME_LOCK_WRITE = 0x10011001,
    ISP_FRAME_LOCK_READ = 0x01100110
};

enum 
{
    ISP_MASTER_READ,
    ISP_MASTER_WRITE,
    ISP_MASTER_MAX    
};

enum 
{
    ISP_MASTER_ENDIANNESS_BIG,
    ISP_MASTER_ENDIANNESS_LITTLE,
    ISP_MASTER_ENDIANNESS_HALFBIG,
    ISP_MASTER_ENDIANNESS_MAX        
};

	
typedef struct _isp_cap_desc_tag
{
    uint32_t                   input_format;
    uint32_t                   frame_deci_factor;
    uint32_t                   img_x_deci_factor;
    uint32_t                   img_y_deci_factor;
}ISP_CAP_DESCRIPTION_T;

typedef struct _isp_path_desc_tag
{
    ISP_SIZE_T               input_size;
    ISP_RECT_T               input_rect;
    ISP_SIZE_T               sc_input_size;
    ISP_SIZE_T               output_size;
    ISP_FRAME_T              input_frame;
    uint32_t                   input_format;
    ISP_FRAME_T              *p_output_frame_head;
    ISP_FRAME_T              *p_output_frame_cur;
    uint32_t                   output_frame_count;    
    uint32_t                   output_format;
    uint32_t                   output_frame_flag;   
    ISP_FRAME_T              swap_frame;
    ISP_FRAME_T              line_frame;	
    uint32_t                   scale_en;
    uint32_t                   sub_sample_en;
    uint32_t                   sub_sample_factor;
    uint32_t                   sub_sample_mode;	
    uint32_t                   slice_en;
    uint32_t                   h_scale_coeff;
    uint32_t                   v_scale_coeff;    
}ISP_PATH_DESCRIPTION_T;

typedef struct _isp_module_tagss
{
    ISP_MODE_E               isp_mode;
    uint32_t                   module_addr;
    ISP_CAP_DESCRIPTION_T    isp_cap;
    ISP_PATH_DESCRIPTION_T   isp_path1;
    ISP_PATH_DESCRIPTION_T   isp_path2;
    ISP_ISR_FUNC_PTR         user_func[ISP_IRQ_NUMBER];
  
}ISP_MODULE_T;

ISP_FRAME_T            s_path1_frame[ISP_PATH1_FRAME_COUNT_MAX];
ISP_FRAME_T            s_path2_frame[ISP_PATH2_FRAME_COUNT_MAX];
ISP_MODULE_T           s_isp_mod;

uint32_t g_is_stop = 0;

LOCAL void    _ISP_DrvierModuleReset(uint32_t base_addr);
LOCAL uint32_t  _ISP_DriverReadIrqLine(uint32_t base_addr);
LOCAL void    _ISP_DriverIrqClear(uint32_t base_addr,uint32_t mask);
LOCAL void    _ISP_DriverIrqDisable(uint32_t base_addr,uint32_t mask);
LOCAL void    _ISP_DriverIrqEnable(uint32_t base_addr,uint32_t mask);
LOCAL void    _ISP_ISRSensorStartOfFrame(uint32_t base_addr);
LOCAL void    _ISP_ISRSensorEndOfFrame(uint32_t base_addr);		
LOCAL void    _ISP_ISRCapStartOfFrame(uint32_t base_addr);
LOCAL void    _ISP_ISRCapEndOfFrame(uint32_t base_addr);
LOCAL void    _ISP_ISRPath1Done(uint32_t base_addr);
LOCAL void    _ISP_ISRCapFifoOverflow(uint32_t base_addr);
LOCAL void    _ISP_ISRSensorLineErr(uint32_t base_addr);
LOCAL void    _ISP_ISRSensorFrameErr(uint32_t base_addr);
LOCAL void    _ISP_ISRJpegBufOverflow(uint32_t base_addr);
LOCAL ISR_EXE_T    _ISP_ISRSystemRoot(uint32_t param);
LOCAL void    _ISP_DriverISRRoot(uint32_t base_addr);
LOCAL void    _ISP_DriverLinkFrames(void);
LOCAL void    _ISP_DriverAutoCopy(uint32_t base_addr);
LOCAL int32_t   _ISP_DriverCalcSC1Size(void);
LOCAL int32_t   _ISP_DriverSetSC1Coeff(uint32_t base_addr);
#if 0
LOCAL BOOLEAN _ISP_DrvierIsSysLittleEndian(void);
LOCAL void    _ISP_DriverForceCopy(uint32_t base_addr);
LOCAL int32_t   _ISP_DriverSetMasterEndianness(uint32_t base_addr, uint32_t master_sel, uint32_t is_rgb565);
LOCAL int32_t   _ISP_DriverSetPath1NextFrameAddr(uint32_t base_addr, BOOLEAN b_first_frame);
#endif
#ifdef ISP_DRV_SCALE_COEFF_TABLE_EN
LOCAL uint32_t  _ISP_DriverCalcScaleCoeff(uint32_t input_width, 
                                        uint32_t input_height, 
                                        uint32_t output_width, 
                                        uint32_t output_height,
                                        uint32_t *p_h_coeff,
                                        uint32_t *p_v_coeff);
#endif 
LOCAL int32_t   _ISP_DriverPath1TrimAndScaling(uint32_t base_addr);


LOCAL ISP_ISR_PTR s_isp_isr_list[ISP_IRQ_NUMBER]=
{
    _ISP_ISRSensorStartOfFrame,
    _ISP_ISRSensorEndOfFrame,		
    _ISP_ISRCapStartOfFrame,
    _ISP_ISRCapEndOfFrame,
    _ISP_ISRPath1Done,
    _ISP_ISRCapFifoOverflow,
    _ISP_ISRSensorLineErr,
    _ISP_ISRSensorFrameErr,
    _ISP_ISRJpegBufOverflow,
    NULL//_ISP_ISRPath2Done
};

#define TB_EnableINT(int_num) \
	*((volatile uint32_t *)INT_IRQ_EN) |= (1UL << int_num)
#define TB_DisableINT(int_num) \
	*((volatile uint32_t *)INT_IRQ_DISABLE) |= (1UL << int_num)
#define TB_DCAM_INT 0x1B


PUBLIC int32_t ISP_DriverModuleInit(uint32_t base_addr) 
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    
    
    ISP_DATA_CLEAR(&s_isp_mod);

    ISP_DriverScalingCoeffReset();
    //TB_EnableINT(TB_DCAM_INT);
    //ISR_RegHandler(TB_DCAM_INT,_ISP_ISRSystemRoot); 
    _ISP_DriverLinkFrames();	
    s_isp_mod.module_addr = base_addr;
	
    return rtn;
}

PUBLIC int32_t ISP_DriverModuleEnable(uint32_t ahb_ctrl_addr)// must be AHB general control register base address
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    
    
    ISP_CHECK_PARAM_ZERO_POINTER(ahb_ctrl_addr);

   *(volatile uint32_t *)(ahb_ctrl_addr + ISP_AHB_CTRL_MOD_EN_OFFSET) |=  (BIT_1|BIT_2);
    *(volatile uint32_t *)(ahb_ctrl_addr + ISP_AHB_CTRL_MEM_SW_OFFSET) &=  ~BIT_0; // switch memory to ISP
   
    _ISP_DrvierModuleReset(ahb_ctrl_addr);
	
    return rtn;
}
PUBLIC int32_t ISP_DriverModuleDisable(uint32_t ahb_ctrl_addr) // must be AHB general control register base address
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    

    *(volatile uint32_t *)(ahb_ctrl_addr + ISP_AHB_CTRL_MEM_SW_OFFSET) |= BIT_0; // switch memory to ARM
    
    *(volatile uint32_t *)(ahb_ctrl_addr + ISP_AHB_CTRL_MOD_EN_OFFSET) &=  ~(BIT_1|BIT_2);

    return rtn;
	
}
PUBLIC int32_t ISP_DriverSoftReset(uint32_t ahb_ctrl_addr)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;    

    ISP_DriverModuleEnable(ahb_ctrl_addr);

    _ISP_DrvierModuleReset(ahb_ctrl_addr);    

    //_ISP_DriverScalingCoeffReset();

    return rtn;
}

PUBLIC void ISP_DriverIramSwitch(uint32_t base_addr,uint32_t isp_or_arm)
{
    if(isp_or_arm == IRAM_FOR_ISP)    
    {
	  _paad(base_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, ~BIT_0);
    }
    else
    {
	_paod(base_addr + ISP_AHB_CTRL_MEM_SW_OFFSET, BIT_0);
    }
}

PUBLIC int32_t ISP_DriverSetClk(uint32_t pll_src_addr,ISP_CLK_SEL_E clk_sel)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;	

    ISP_CHECK_PARAM_ZERO_POINTER(pll_src_addr);

    switch(clk_sel)
    {
        case ISP_CLK_96M:
           _paad(pll_src_addr, ~(BIT_5 | BIT_4));
        break;
        case ISP_CLK_64M:
	    _paad(pll_src_addr, ~BIT_5);
	    _paod(pll_src_addr, BIT_4);
        break;
        case ISP_CLK_48M:
	  _paad(pll_src_addr, ~BIT_4);
	  _paod(pll_src_addr, BIT_5);
        break;
        default:
           _paod(pll_src_addr, BIT_4 | BIT_5); //default set 26M;
        break;
    }

    return rtn;
}
#if 0
LOCAL void _ISP_GetReg(void)
{
	printk("[DCAM:DCAM_CFG:0x%x]\n",_pard(DCAM_CFG));
	printk("[DCAM:DCAM_PATH_CFG:0x%x\n]",_pard(DCAM_PATH_CFG));
	printk("[DCAM:DCAM_SRC_SIZE:0x%x]\n",_pard(DCAM_SRC_SIZE));
	printk("[DCAM:DCAM_DES_SIZE:0x%x]\n",_pard(DCAM_DES_SIZE));
	printk("[DCAM:DCAM_TRIM_START:0x%x]\n",_pard(DCAM_TRIM_START));
	printk("[DCAM:DCAM_TRIM_SIZE:0x%x]\n",_pard(DCAM_TRIM_SIZE));
	printk("[DCAM:DCAM_INT_STS:0x%x]\n",_pard(DCAM_INT_STS));
	printk("[DCAM:DCAM_INT_MASK:0x%x]\n",_pard(DCAM_INT_MASK));
	printk("[DCAM:DCAM_INT_CLR:0x%x]\n",_pard(DCAM_INT_CLR));
	printk("[DCAM:DCAM_INT_RAW:0x%x]\n",_pard(DCAM_INT_RAW));
	printk("[DCAM:ENDIAN_SEL:0x%x]\n",_pard(ENDIAN_SEL));
	printk("[DCAM:DCAM_ADDR_7:0x%x]\n",_pard(DCAM_ADDR_7));
	printk("[DCAM:DCAM_ADDR_8:0x%x]\n",_pard(DCAM_ADDR_8));
//	printk("[DCAM:CAP_CTRL:0x%x]\n",_pard(CAP_CTRL));
	printk("[DCAM:CAP_FRM_CNT:0x%x]\n",_pard(CAP_FRM_CNT));
	printk("[DCAM:CAP_START:0x%x]\n",_pard(CAP_START));
	printk("[DCAM:CAP_END:0x%x]\n",_pard(CAP_END));
	printk("[DCAM:CAP_IMAGE_DECI:0x%x]\n",_pard(CAP_IMAGE_DECI));
//	printk("[DCAM:CAP_JPG_CTL:0x%x]\n",_pard(CAP_JPG_CTL));	
	printk("[DCAM:CAP_JPG_FRM_SIZE:0x%x]\n",_pard(CAP_JPG_FRM_SIZE));	
		
}
#endif

PUBLIC int32_t ISP_DriverStart(uint32_t base_addr)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;	

    g_is_stop = 0;
    
    ISP_CHECK_PARAM_ZERO_POINTER(base_addr);   

    DCAM_TRACE("###DCAM: isp mode: %x", s_isp_mod.isp_mode);

            rtn = _ISP_DriverPath1TrimAndScaling(base_addr);
            ISP_DRV_RTN_IF_ERR;
           DCAM_TRACE("###DCAM: _ISP_DriverPath1TrimAndScaling ok.\n");

	   _ISP_DriverIrqClear(base_addr,ISP_IRQ_LINE_MASK);
            _ISP_DriverIrqEnable(base_addr, ISP_IRQ_LINE_MASK);
    		DCAM_TRACE("###DCAM: int mask:%x", _pard(DCAM_INT_MASK));
	   //wxz20110602: set half word endian to dcam output endian. 
	    _paad(ENDIAN_SEL, ~(BIT_2 | BIT_3));	    
	    _paod(ENDIAN_SEL, 0x2 << 2);
         
            _ISP_DriverAutoCopy(base_addr);
            //_ISP_DriverForceCopy(base_addr);			
    //      _ISP_GetReg();
            _paod(DCAM_PATH_CFG, BIT_0);
		DCAM_TRACE("###dcam: DCAM_PATH_CFG: %x.\n", _pard(DCAM_PATH_CFG));

    return rtn;
}

PUBLIC int32_t ISP_DriverStop(uint32_t base_addr)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;

    if(1 == g_is_stop)
		return rtn;
    
    ISP_CHECK_PARAM_ZERO_POINTER(base_addr);   

	_ISP_DriverIrqDisable(base_addr,ISP_IRQ_LINE_MASK);
	
    switch(s_isp_mod.isp_mode)
    {
        case ISP_MODE_CAPTURE: 			
        case ISP_MODE_MPEG:
        case ISP_MODE_VT:
        case ISP_MODE_PREVIEW_EX:
            _paad(DCAM_PATH_CFG, ~BIT_0);
	    msleep(20);//wait the dcam stop
        break;  
	case ISP_MODE_PREVIEW:
	    {
		uint32_t count,value;		
		    _paad(DCAM_PATH_CFG, ~BIT_0);

		    g_is_stop = 1;
		    //msleep(20);//wait the dcam stop	
		    
		    for(count = 0; count < 100; count++)
		    	{
		    		value = _pard(DCAM_INT_RAW);
				printk("ISP_DriverStop wait the last interrupt.value: 0x%x, count: %d\n", value, count);
				if(value & 0x10){
					DCAM_TRACE("ISP_DriverStop wait the last interrupt.\n");					
					break;
				}				
				msleep(20);
		    	}			
	    }
	break;
        default:
            rtn = ISP_DRV_RTN_MODE_ERR; 
        break;
    }

    _ISP_DriverIrqClear(base_addr,ISP_IRQ_LINE_MASK);
    
    return rtn;
}

PUBLIC int32_t ISP_DriverCapConfig(uint32_t base_addr, ISP_CFG_ID_E isp_cfg_id, void* param)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    uint32_t value;
    
    ISP_CHECK_PARAM_ZERO_POINTER(base_addr);
        
    switch(isp_cfg_id)
    {
        case ISP_CAP_CCIR565_ENABLE:
            ISP_CHECK_PARAM_ZERO_POINTER(param);
        	  
            value = *(uint32_t*)param;
	   _paad(CAP_CNTRL, ~BIT_0);
	   _paod(CAP_CNTRL, value & 0x1); 
        break;

        case ISP_CAP_TV_FRAME_SEL:
        {
            uint32_t frame_sel = *(uint32_t*)param;
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            if(frame_sel < ISP_CAP_CCIR656_FRAME_SEL_MAX)
            {
                _paad(CAP_CNTRL, ~(BIT_1 | BIT_2));
		_paod(CAP_CNTRL, (frame_sel & 0x3) << 1);
            }
            else
            {
                rtn = ISP_DRV_RTN_CAP_FRAME_SEL_ERR;
            }
            break;
        }

        case ISP_CAP_SYNC_POL:        	  
        {
            ISP_CAP_SYNC_POL_T *p_sync_pol = (ISP_CAP_SYNC_POL_T*)param;  
                      
            ISP_CHECK_PARAM_ZERO_POINTER(param);

            
            if(p_sync_pol->vsync_pol > 1 || p_sync_pol->hsync_pol > 1)
            {
                rtn = ISP_DRV_RTN_CAP_SYNC_POL_ERR;
            }
            else
            {
                _paad(CAP_CNTRL, ~(BIT_3 | BIT_4));
		_paod(CAP_CNTRL, (p_sync_pol->hsync_pol & 0x1) << 3);
		_paod(CAP_CNTRL, (p_sync_pol->vsync_pol & 0x1) << 4);		
            }
            break;
            
        }

        case ISP_CAP_YUV_TYPE:
        {
            ISP_CAP_PATTERN_E cap_yun_pat = *(ISP_CAP_PATTERN_E*)param;
            ISP_CHECK_PARAM_ZERO_POINTER(param); 
        	  
            if(cap_yun_pat < ISP_PATTERN_MAX)
            {
                _paad(CAP_CNTRL, ~(BIT_7 | BIT_8));
		_paod(CAP_CNTRL, (cap_yun_pat & 0x3) << 7);                
            }
            else
            {
                rtn = ISP_DRV_RTN_CAP_INPUT_YUV_ERR;
            }
            break;
        }

        case ISP_CAP_FIFO_DATA_RATE:
        {
            ISP_CAP_FIFO_DATA_RATE_E cap_fifo_data_rate = *(ISP_CAP_FIFO_DATA_RATE_E*)param;
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            if((cap_fifo_data_rate < ISP_CAP_FIFO_DATA_RATE_MAX) && 
            	 (cap_fifo_data_rate != ISP_CAP_FIFO_DATA_RATE_RES))
            {
                value = *(uint32_t*)param;
		_paad(CAP_CNTRL, ~(BIT_9 | BIT_10));
		_paod(CAP_CNTRL, (value & 0x3) << 9);  
            }
            else
            {
                rtn = ISP_DRV_RTN_CAP_FIFO_DATA_RATE_ERR;
            }
            break;
        }

        case ISP_CAP_PRE_SKIP_CNT:
        {
            uint32_t skip_num = *(uint32_t*)param;
            ISP_CHECK_PARAM_ZERO_POINTER(param);

            if(skip_num < ISP_CAP_FRAME_SKIP_NUM_MAX)
            {
                _paad(CAP_FRM_CNT, ~0x3F);
		_paod(CAP_FRM_CNT, skip_num);
            }
            else
            {
                rtn = ISP_DRV_RTN_CAP_SKIP_FRAME_TOO_MANY;	
            }
            break;
        }

        case ISP_CAP_FRM_DECI:
        {
              uint32_t deci_factor = *(uint32_t*)param;

              ISP_CHECK_PARAM_ZERO_POINTER(param);
              
        	  if(deci_factor < ISP_CAP_FRAME_DECI_FACTOR_MAX)
        	  {
        	      _paad(CAP_FRM_CNT, ~0xF00);
		      _paod(CAP_FRM_CNT, deci_factor);
        	  }
        	  else
        	  {
        	     rtn = ISP_DRV_RTN_CAP_FRAME_DECI_FACTOR_ERR;	 
        	  }
        	  break;
        }

		case ISP_CAP_FRM_COUNT_CLR:
        	  _paod(CAP_FRM_CNT, BIT_22);
        break;

        case ISP_CAP_INPUT_RECT:
        {
            ISP_RECT_T *p_rect = (ISP_RECT_T*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            if(p_rect->x > (ISP_CAP_FRAME_WIDTH_MAX >> 1) ||
               p_rect->y > ISP_CAP_FRAME_HEIGHT_MAX ||
               p_rect->w > (ISP_CAP_FRAME_WIDTH_MAX >> 1) || 
               p_rect->h > ISP_CAP_FRAME_HEIGHT_MAX )
            {
                rtn = ISP_DRV_RTN_CAP_FRAME_SIZE_ERR;	  
            }    
            else
            {
		_pawd(CAP_START, ((p_rect->x << 1) & 0x1FFF) | (((p_rect->y) & 0xFFF) << 16) );
		_pawd(CAP_END, ((((p_rect->w + p_rect->x )<< 1) - 1) & 0x1FFF) | (((p_rect->y + p_rect->h - 1) & 0xFFF) << 16) );                
            }                	
            break;
        }

        case ISP_CAP_IMAGE_XY_DECI:
        {
            ISP_CAP_DEC_T *p_cap_dec = (ISP_CAP_DEC_T*)param;

            ISP_CHECK_PARAM_ZERO_POINTER(param);


            if(p_cap_dec->x_factor > ISP_CAP_X_DECI_FACTOR_MAX || 
               p_cap_dec->y_factor > ISP_CAP_X_DECI_FACTOR_MAX )
            {
                rtn = ISP_DRV_RTN_CAP_XY_DECI_FACTOR_ERR;	  
            }
            else
            {
		_pawd(CAP_IMAGE_DECI, ((p_cap_dec->x_factor) & 0xF) | (((p_cap_dec->y_factor) & 0xF) << 4) | (((p_cap_dec->x_mode) & 0x1) << 8) ); 
            }
            break;
        }

        case ISP_CAP_JPEG_DROP_NUM:
        {
            uint32_t jpeg_drop_num = *(uint32_t*)param; 

            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            if(ISP_CAP_JPEG_DROP_NUM_MIN > jpeg_drop_num || 
            	 ISP_CAP_JPEG_DROP_NUM_MAX < jpeg_drop_num)
            {
                rtn = ISP_DRV_RTN_CAP_JPEG_DROP_NUM_ERR;	  
            }
            else
            {
               _paad(CAP_JPG_FRM_CTL, ~0xF0);
		_paod(CAP_JPG_FRM_CTL, ((jpeg_drop_num) & 0xF) << 4);
            }
            break;
        }

        case ISP_CAP_INPUT_FORMAT:
        {
              
            ISP_CAP_INPUT_FORMAT_E cap_input_format = *(ISP_CAP_INPUT_FORMAT_E*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            if(cap_input_format >= ISP_CAP_INPUT_FORMAT_MAX)
            {
                rtn = ISP_DRV_RTN_CAP_INPUT_FORMAT_ERR;
            }
            else
            {
                _paad(DCAM_PATH_CFG, ~BIT_2);
		_paod(DCAM_PATH_CFG, (cap_input_format & 0x1) << 2);
            }
            s_isp_mod.isp_cap.input_format = cap_input_format;
            break;
        }            

        case ISP_CAP_JPEG_MEM_IN_16K:
        
            ISP_CHECK_PARAM_ZERO_POINTER(param);
        value = (*(uint32_t*)param) & 0x000000FF;
	_paad(CAP_JPG_FRM_CTL, 0xFF00);
	_paod(CAP_JPG_FRM_CTL, (value & 0xFF) << 8);
		
        break;

        default:
			
            rtn = ISP_DRV_RTN_IO_ID_UNSUPPORTED;
			
        break;
    }
    
    return rtn;
}

PUBLIC int32_t ISP_DriverCapGetInfo(uint32_t base_addr, ISP_CFG_ID_E id, void* param)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    
    ISP_CHECK_PARAM_ZERO_POINTER(base_addr);
    ISP_CHECK_PARAM_ZERO_POINTER(param);
    
    switch(id)
    {
        case ISP_CAP_FRM_COUNT_GET:
	    *(uint32_t*)param = (_pard(CAP_FRM_CNT) >> 16) & 0x3F;
            break;
        case ISP_CAP_JPEG_GET_NUM:
            *(uint32_t*)param = _pard(CAP_JPG_FRM_CTL) & 0x7;
            break;    
        case ISP_CAP_JPEG_GET_LENGTH:
	    *(uint32_t*)param = _pard(CAP_JPG_FRM_SIZE) & 0xFFFFFF;	
            break;
        default:
            rtn = ISP_DRV_RTN_IO_ID_UNSUPPORTED;
            break;
            
    }	
    return rtn;
}

PUBLIC int32_t ISP_DriverPath1Config(uint32_t base_addr, ISP_CFG_ID_E id, void* param)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;
    
    ISP_CHECK_PARAM_ZERO_POINTER(base_addr);
        
    switch(id)
    {
        case ISP_PATH_INPUT_FORMAT:
        {
            rtn = ISP_DRV_RTN_IO_ID_UNSUPPORTED; //DATA come from CAP
            break;
        }
        case ISP_PATH_INPUT_SIZE:
        {
            ISP_SIZE_T *p_size = (ISP_SIZE_T*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);

        	DCAM_TRACE("###dcam: ISP_PATH_INPUT_SIZE w: %d, h: %d.\n", p_size->w, p_size->h);  
            if(p_size->w > ISP_PATH_FRAME_WIDTH_MAX ||
               p_size->h > ISP_PATH_FRAME_HEIGHT_MAX)
            {
                rtn = ISP_DRV_RTN_PATH_SRC_SIZE_ERR;    
            }
            else
            {
		_pawd(DCAM_SRC_SIZE, ((p_size->w) & 0xFFF) | (((p_size->h) & 0xFFF) << 16));
                p_path->input_size.w = p_size->w;
                p_path->input_size.h = p_size->h;				
                
            }
            break;
        }
        case ISP_PATH_INPUT_RECT:
        {
            ISP_RECT_T *p_rect = (ISP_RECT_T*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            
            if(p_rect->x > ISP_PATH_FRAME_WIDTH_MAX ||
               p_rect->y > ISP_PATH_FRAME_HEIGHT_MAX ||
               p_rect->w > ISP_PATH_FRAME_WIDTH_MAX ||
               p_rect->h > ISP_PATH_FRAME_HEIGHT_MAX)
            {
                rtn = ISP_DRV_RTN_PATH_TRIM_SIZE_ERR;    
            }
            else
            {
		_pawd(DCAM_TRIM_START, ((p_rect->x) & 0xFFFF) | (((p_rect->y) & 0xFFFF) << 16));
		_pawd(DCAM_TRIM_SIZE, ((p_rect->w) & 0xFFFF) | (((p_rect->h) & 0xFFFF) << 16));
                ISP_MEMCPY((void*)&p_path->input_rect,
                                       (void*)p_rect,
                                       sizeof(ISP_RECT_T));
            }
            break;
        }
        case ISP_PATH_INPUT_ADDR:
        {
            ISP_ADDRESS_T *p_addr = (ISP_ADDRESS_T*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            
            if(ISP_PATH_YUV_ADDR_INVALIDE(p_addr->yaddr, p_addr->uaddr, p_addr->vaddr))
            {
                rtn = ISP_DRV_RTN_PATH_ADDR_INVALIDE;   
            }
            else
            {	
                ISP_MEMCPY((void*)&p_path->input_frame,
                           (void*)p_addr,
                           sizeof(ISP_ADDRESS_T));                      
            }
            
            break;
        }
        case ISP_PATH_OUTPUT_SIZE:
        {
            ISP_SIZE_T *p_size = (ISP_SIZE_T*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
        	  
        	  
            if(p_size->w > ISP_PATH_FRAME_WIDTH_MAX ||
               p_size->h > ISP_PATH_FRAME_HEIGHT_MAX)
            {
                rtn = ISP_DRV_RTN_PATH_DES_SIZE_ERR;    
            }
            else
            {
		_pawd(DCAM_DES_SIZE, ((p_size->w) & 0x3FF) | (((p_size->h) & 0x3FF) << 16));
                p_path->output_size.w = p_size->w;
                p_path->output_size.h = p_size->h;				
            }
		DCAM_TRACE("###dcam: output size w: %d, h: %d, reg DCAM_DES_SIZE: 0x %x.\n", p_size->w, p_size->h, _pard(DCAM_DES_SIZE));
            break;
        }  
        case ISP_PATH_OUTPUT_FORMAT:
            rtn = ISP_DRV_RTN_IO_ID_UNSUPPORTED; //only support YUV422
            break;
            
        case ISP_PATH_OUTPUT_ADDR:
        {
            ISP_ADDRESS_T *p_addr = (ISP_ADDRESS_T*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            
            if(ISP_PATH_YUV_ADDR_INVALIDE(p_addr->yaddr, p_addr->uaddr, p_addr->vaddr))
            {
                rtn = ISP_DRV_RTN_PATH_ADDR_INVALIDE;   
            }
            else
            {     
                if(p_path->output_frame_count > ISP_PATH1_FRAME_COUNT_MAX - 1)     	
                {
                    rtn = ISP_DRV_RTN_PATH_FRAME_TOO_MANY;  
                }
                else
                {
                    p_path->p_output_frame_cur->yaddr = p_addr->yaddr;
                    p_path->p_output_frame_cur->uaddr = p_addr->uaddr;                    
                    p_path->p_output_frame_cur->vaddr = p_addr->vaddr;
                    
                    p_path->p_output_frame_cur = p_path->p_output_frame_cur->next;                    
                    p_path->output_frame_count ++;
                }
            }
            break;
        } 
        case ISP_PATH_SUB_SAMPLE_EN:
        {
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            p_path->sub_sample_en = *(uint32_t*)param ? 1 : 0;
            //p_isp_reg->review_path_cfg_u.mBits.sub_sample_eb = p_path->sub_sample_en; //wxz:???
            DCAM_TRACE("###DCAM: errorp_isp_reg->review_path_cfg_u.mBits.sub_sample_eb.\n ");
        	  	
            break;
        }      	
        
        case ISP_PATH_SUB_SAMPLE_MOD:
        {
            uint32_t sub_sameple_mode = *(uint32_t*)param;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            if(sub_sameple_mode > ISP_PATH_SUB_SAMPLE_MAX)
            {
                rtn = ISP_DRV_RTN_PATH_SUB_SAMPLE_ERR;        
            }
            else
            {
                //p_isp_reg->review_path_cfg_u.mBits.sub_sample_eb = sub_sameple_mode;//wxz:???
                DCAM_TRACE("###DCAM:error p_isp_reg->review_path_cfg_u.mBits.sub_sample_eb.\n");
                p_path->sub_sample_factor = sub_sameple_mode;
            }
        	  	
            break;
           
        }	
        default:
            break;

    }	
    return rtn;
}

PUBLIC int32_t ISP_DriverPath1GetInfo(uint32_t base_addr, ISP_CFG_ID_E id, void* param)
{
 
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;
	
    ISP_CHECK_PARAM_ZERO_POINTER(base_addr);

    switch(id)       
    {
        case ISP_PATH_IS_IN_SCALE_RANGE:
        {
            ISP_SIZE_T *p_size_src = (ISP_SIZE_T*)param;
            ISP_SIZE_T *p_size_dst = ++p_size_src;
            
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            
            if(p_size_src->w > p_size_dst->w*ISP_PATH_SC_COEFF_MAX || 
               p_size_src->w*ISP_PATH_SC_COEFF_MAX < p_size_dst->w ||
               p_size_src->h > p_size_dst->h*ISP_PATH_SC_COEFF_MAX ||
               p_size_src->h*ISP_PATH_SC_COEFF_MAX < p_size_dst->h)
            {                
                rtn = ISP_DRV_RTN_PATH_SC_COEFF_ERR;
            }     
            break;

        }
        case ISP_PATH_IS_SCALE_EN:
        
            ISP_CHECK_PARAM_ZERO_POINTER(param);
            *(uint32_t*)param = p_path->scale_en;
       
        break;
					
        default:
            rtn = ISP_DRV_RTN_IO_ID_UNSUPPORTED;
        break;
    }

    return rtn;

}

PUBLIC int32_t ISP_DriverSetMode(uint32_t base_addr, ISP_MODE_E mode)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;

    ISP_CHECK_PARAM_ZERO_POINTER(base_addr);

    switch(mode)
    {
        case ISP_MODE_PREVIEW:
            _paod(DCAM_CFG, BIT_0);
            //_paad(DCAM_CFG, ~(BIT_1 | BIT_2)); 	  
            _paad(DCAM_CFG, ~BIT_1); 	  
	    _paod(DCAM_PATH_CFG, BIT_1);//DCAM_CAP_MODE_MULTIPLE
        break;
        case ISP_MODE_CAPTURE:
            _paod(DCAM_CFG, BIT_0);
            //_paad(DCAM_CFG, ~(BIT_1 | BIT_2));		
            _paad(DCAM_CFG, ~BIT_1); 	
            _paad(DCAM_PATH_CFG, ~BIT_1);//DCAM_CAP_MODE_SINGLE
            //_paod(DCAM_PATH_CFG, BIT_1);
            break;
        default:
            rtn = ISP_DRV_RTN_MODE_ERR;
        break;
    }
	
    if(!rtn)
    {
        s_isp_mod.isp_mode = mode;
    }
    return rtn;

}

PUBLIC int32_t ISP_DriverNoticeRegister(uint32_t base_addr,
                                      ISP_IRQ_NOTICE_ID_E notice_id,
                                      ISP_ISR_FUNC_PTR user_func)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;

    if(notice_id >= ISP_IRQ_NOTICE_NUMBER)
    {
        rtn = ISP_DRV_RTN_ISR_NOTICE_ID_ERR;
    }
    else
    {
        s_isp_mod.user_func[notice_id] = user_func;
    }
    return rtn;		
}

LOCAL  void   _ISP_DrvierModuleReset(uint32_t base_addr)
{
    *(volatile uint32_t *)(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET) |= BIT_1 | BIT_2;
    *(volatile uint32_t *)(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET) |= BIT_1 | BIT_2;
   *(volatile uint32_t *)(base_addr + ISP_AHB_CTRL_SOFT_RESET_OFFSET) &= ~(BIT_1 | BIT_2);
}

PUBLIC  void   ISP_DriverScalingCoeffReset(void)
{
    s_isp_mod.isp_path1.h_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
    s_isp_mod.isp_path1.v_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
    s_isp_mod.isp_path2.h_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;
    s_isp_mod.isp_path2.v_scale_coeff = ISP_PATH_SCALE_LEVEL_MAX + 1;	    
}

LOCAL  uint32_t _ISP_DriverReadIrqLine(uint32_t base_addr)
{
    return _pard(DCAM_INT_STS);
}

LOCAL  void _ISP_DriverIrqClear(uint32_t base_addr,uint32_t mask)
{
    _paod(DCAM_INT_CLR, mask);
    
}

LOCAL  void _ISP_DriverIrqDisable(uint32_t base_addr,uint32_t mask)
{
    _paad(DCAM_INT_MASK, ~mask);

}

LOCAL  void _ISP_DriverIrqEnable(uint32_t base_addr,uint32_t mask)
{
    _paod(DCAM_INT_MASK, mask);
}

LOCAL ISR_EXE_T _ISP_ISRSystemRoot(uint32_t param)
{
    _ISP_DriverISRRoot(s_isp_mod.module_addr);

    return ISR_DONE;
}
LOCAL void _ISP_DriverISRRoot(uint32_t base_addr)
{
    uint32_t                      irq_line, irq_status;
    uint32_t                      i;

    irq_line = ISP_IRQ_LINE_MASK & _ISP_DriverReadIrqLine(base_addr);
	
    irq_status = irq_line;
    for(i = 0; i < ISP_IRQ_NUMBER; i++)
    {
        if(irq_line & 1)
        {        
            s_isp_isr_list[i](base_addr);
        }
        irq_line>>=1;
        if(!irq_line)
            break;
    }
    
    	_ISP_DriverIrqClear(base_addr,irq_status);	
}


LOCAL void  _ISP_ISRSensorStartOfFrame(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR           user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_SENSOR_SOF];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
    return ;

}
LOCAL void  _ISP_ISRSensorEndOfFrame(uint32_t base_addr)		
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_SENSOR_EOF];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
    return ;

}
	
LOCAL void  _ISP_ISRCapStartOfFrame(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_CAP_SOF];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
    return ;

}

LOCAL void  _ISP_ISRCapEndOfFrame(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_CAP_EOF];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
    return ;

}
LOCAL void  _ISP_ISRPath1Done(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_PATH1_DONE];
    ISP_FRAME_T               *frame_curr = s_isp_mod.isp_path1.p_output_frame_cur->prev;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;
 
    frame_curr->width = p_path->output_size.w;
    frame_curr->height = p_path->output_size.h;        	
        
    if(user_func)
    {
        (*user_func)((void*)frame_curr);
    }
   
    return ;
}

LOCAL void  _ISP_ISRCapFifoOverflow(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_CAP_FIFO_OF];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
	
    return ;
}

LOCAL void  _ISP_ISRSensorLineErr(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_SENSOR_LINE_ERR];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
    return ;

}

LOCAL void  _ISP_ISRSensorFrameErr(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_SENSOR_FRAME_ERR];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
    return ;

}

LOCAL void  _ISP_ISRJpegBufOverflow(uint32_t base_addr)
{
    ISP_ISR_FUNC_PTR          user_func = s_isp_mod.user_func[ISP_IRQ_NOTICE_JPEG_BUF_OF];
	
    if(user_func)
    {
        (*user_func)(NULL);
    }
    return ;

}

LOCAL void _ISP_DriverLinkFrames(void)
{
    uint32_t                    i = 0;
    ISP_FRAME_T               *p_path1_frame = &s_path1_frame[0];
    ISP_FRAME_T               *p_path2_frame = &s_path2_frame[0];
    
    for(i = 0; i < ISP_PATH1_FRAME_COUNT_MAX; i++)
    {
        ISP_DATA_CLEAR(p_path1_frame + i);
        (p_path1_frame+i)->next = p_path1_frame + (i + 1) % ISP_PATH1_FRAME_COUNT_MAX;
        (p_path1_frame+i)->prev = p_path1_frame + (i -1 + ISP_PATH1_FRAME_COUNT_MAX) % ISP_PATH1_FRAME_COUNT_MAX;
    }
    
    for(i=0; i<ISP_PATH2_FRAME_COUNT_MAX; i++)
    {
        ISP_DATA_CLEAR(p_path2_frame + i);	
        (p_path2_frame+i)->next = p_path2_frame+(i + 1) % ISP_PATH2_FRAME_COUNT_MAX;
        (p_path2_frame+i)->prev = p_path2_frame+(i -1 + ISP_PATH2_FRAME_COUNT_MAX) % ISP_PATH2_FRAME_COUNT_MAX;
    }
	
    s_isp_mod.isp_path1.p_output_frame_head = p_path1_frame;
    s_isp_mod.isp_path2.p_output_frame_head = p_path2_frame;	
    s_isp_mod.isp_path1.p_output_frame_cur = p_path1_frame;
    s_isp_mod.isp_path2.p_output_frame_cur = p_path2_frame;	
	
}
#if 0
LOCAL int32_t _ISP_DriverSetPath1NextFrameAddr(uint32_t base_addr, BOOLEAN b_first_frame)
{
    ISP_FRAME_T               *p_frame = SCI_NULL;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
	
    if(b_first_frame)
    {
        if(p_path->output_frame_count < ISP_PATH1_FRAME_COUNT_MAX) // Not all round buffer given by app
        {
            p_frame = p_path->p_output_frame_cur->prev;
            p_frame->next = p_path->p_output_frame_head;
            p_path->p_output_frame_head->prev = p_frame;
            p_path->p_output_frame_cur = p_path->p_output_frame_head;
        }
    }

    if(p_path->p_output_frame_cur->lock == ISP_FRAME_UNLOCK)
    {
	_pawd(DCAM_ADDR_7, p_path->p_output_frame_cur->yaddr >> 8);
	_pawd(DCAM_ADDR_H, p_path->p_output_frame_cur->yaddr >> (32 - ISP_PATH_FRAME_HIGH_BITS));
		
        if(p_path->input_format == ISP_CAP_INPUT_FORMAT_YUV) // not JPEG
        {
     	    _pawd(DCAM_ADDR_7, p_path->p_output_frame_cur->uaddr >> 8);			
        }
        p_path->p_output_frame_cur = p_path->p_output_frame_cur->next;
    }
    else
    {
        rtn = ISP_DRV_RTN_PATH_FRAME_LOCKED;    
    }
    
	return rtn;
}

LOCAL void _ISP_DriverForceCopy(uint32_t base_addr)
{
	_paod(DCAM_PATH_CFG, BIT_10);  //wxz:??? why write 3 times
	_paod(DCAM_PATH_CFG, BIT_10);
	_paad(DCAM_PATH_CFG, ~BIT_10);    
}
#endif
LOCAL void _ISP_DriverAutoCopy(uint32_t base_addr)
{
	_paod(DCAM_PATH_CFG, BIT_11);
}
LOCAL int32_t _ISP_DriverPath1TrimAndScaling(uint32_t base_addr)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;

    /*trim config*/
    if(p_path->input_size.w != p_path->input_rect.w || 
       p_path->input_size.h != p_path->input_rect.h)
    {
		_paod(DCAM_PATH_CFG, BIT_8);
    }
    else
    {
		_paad(DCAM_PATH_CFG, ~BIT_8);
    }            

    /*scaling config*/
    rtn = _ISP_DriverCalcSC1Size();       
    if(rtn)    return rtn;
	
    if(p_path->sc_input_size.w != p_path->output_size.w ||
       p_path->sc_input_size.h != p_path->output_size.h)
    {
       _paad(DCAM_PATH_CFG, ~BIT_3);
        rtn = _ISP_DriverSetSC1Coeff(base_addr);
        p_path->scale_en = 1;
    }
    else
    {
	_paod(DCAM_PATH_CFG, BIT_3);
        p_path->scale_en = 0;

    }

    return rtn;
}

LOCAL int32_t _ISP_DriverCalcSC1Size(void)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;
		
    if(p_path->input_rect.w > p_path->output_size.w * ISP_PATH_SC_COEFF_MAX ||
       p_path->input_rect.h > p_path->output_size.h * ISP_PATH_SC_COEFF_MAX ||		
       p_path->input_rect.w * ISP_PATH_SC_COEFF_MAX < p_path->output_size.w ||		
       p_path->input_rect.h * ISP_PATH_SC_COEFF_MAX < p_path->output_size.h) 
    {
        rtn = ISP_DRV_RTN_PATH_SC_COEFF_ERR;
    }
    else
    {
        p_path->sc_input_size.w = p_path->input_rect.w;
        p_path->sc_input_size.h = p_path->input_rect.h;		
    }
    return rtn;
}

#ifndef ISP_DRV_SCALE_COEFF_TABLE_EN
static int32_t _ISP_DriverGenScxCoeff(uint32_t base_addr, uint32_t idxScx)
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
    DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff Entry");
#endif

	if(ISP_PATH1 == idxScx)
	{
	    p_path = &s_isp_mod.isp_path1;

        HScaleAddr   = base_addr + ISP_SCALE1_H_TAB_OFFSET;
        VScaleAddr   = base_addr + ISP_SCALE1_V_TAB_OFFSET;        
    }
    else 
    {
        p_path = &s_isp_mod.isp_path2;
        
        HScaleAddr   = base_addr + ISP_SCALE2_H_TAB_OFFSET;
        VScaleAddr   = base_addr + ISP_SCALE2_V_TAB_OFFSET;  
    }
    
    pTmpBuf = (uint32_t *)kmalloc(ISP_DRV_SCALE_COEFF_BUF_SIZE, GFP_KERNEL);
    SCI_ASSERT(SCI_NULL != pTmpBuf);
    SCI_MEMSET(pTmpBuf, 0, ISP_DRV_SCALE_COEFF_BUF_SIZE);
    
    pHCoeff = pTmpBuf;
    pVCoeff = pTmpBuf + (ISP_DRV_SCALE_COEFF_COEF_SIZE/4);

#ifdef ISP_DRV_SCALE_COEFF_DBG
    DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff i_w/i_h/o_w/o_h = {%d, %d, %d, %d,}",
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
        DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff GenScaleCoeff error!");    
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
        DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
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
        DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
            //pIspReg->dcam_cfg_u.mBits.path1_clk_switch);
            _pard(DCAM_CFG) >> 4) & 0x1);
#endif
    }   
        
    for( i = 0; i < ISP_SCALE_COEFF_H_NUM; i++)
    {
       // *(volatile uint32_t*)HScaleAddr = *pHCoeff;
	_pawd(HScaleAddr, *pHCoeff);

#ifdef ISP_DRV_SCALE_COEFF_DBG
        DCAM_TRACE("ISP_DRV: Coeff H[%d] = 0x%x.\n", i, 
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
        DCAM_TRACE("ISP_DRV: Coeff V[%d] = 0x%x.\n", i, 
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
    DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff V[%d] = 0x%x", i, 
        (*pVCoeff) & 0x0F);
    DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff V[%d] = 0x%x", i, 
        //pIspReg->dcam_path_cfg_u.mBits.ver_down_tap);
        (_pard(DCAM_PATH_CFG) >> 4) & 0xF);
#endif
    
    if(ISP_PATH1 == idxScx)        
    {
       // pIspReg->dcam_cfg_u.mBits.path1_clk_switch = ISP_CLK_DOMAIN_DCAM;	
	_paad(DCAM_CFG, ~BIT_3);

#ifdef ISP_DRV_SCALE_COEFF_DBG
        DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
            //pIspReg->dcam_cfg_u.mBits.path1_clk_switch);
            (_pard(DCAM_CFG) >> 3) & 0x1);
#endif
    }
    else
    {
        //pIspReg->dcam_cfg_u.mBits.path2_clk_switch = ISP_CLK_DOMAIN_DCAM;	
        _paad(DCAM_CFG, ~BIT_4);

#ifdef ISP_DRV_SCALE_COEFF_DBG
        DCAM_TRACE("ISP_DRV: _ISP_DriverGenScxCoeff Domain = 0x%x", 
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

LOCAL int32_t _ISP_DriverSetSC1Coeff(uint32_t base_addr)
{
#ifdef ISP_DRV_SCALE_COEFF_TABLE_EN
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;
    uint32_t                    v_coeff = 0,h_coeff= 0;
    uint32_t                    i = 0;
    uint32_t                    *p_v_coeff_ptr = SCI_NULL;
    uint32_t                    *p_h_coeff_ptr = SCI_NULL;	
    uint32_t                    scale_h_addr = base_addr + ISP_SCALE1_H_TAB_OFFSET;
    uint32_t                    scale_vaddr = base_addr + ISP_SCALE1_V_TAB_OFFSET;	
	
    rtn = _ISP_DriverCalcScaleCoeff(p_path->sc_input_size.w,
                                    p_path->sc_input_size.h,
                                    p_path->output_size.w,
                                    p_path->output_size.h,
                                    &h_coeff,
                                    &v_coeff);
    if(rtn)    return rtn;

    _paod(DCAM_CFG, BIT_3);

    if(p_path->h_scale_coeff != h_coeff)
    {
        p_h_coeff_ptr = (uint32_t*)s_dcam_h_scale_coeff[h_coeff];
        for( i = 0; i < ISP_SCALE_COEFF_H_NUM; i++)
        {
            *(volatile uint32_t*)scale_h_addr = *p_h_coeff_ptr;
            scale_h_addr += 4;
            p_h_coeff_ptr++;
        }    
        p_path->h_scale_coeff = h_coeff; 
    }

    if(p_path->v_scale_coeff != v_coeff)
    {
        p_v_coeff_ptr = (uint32_t*)s_dcam_v_scale_coeff[v_coeff];	

        for( i=0 ;i < ISP_SCALE_COEFF_V_NUM; i++)
        {
            *(volatile uint32_t*)scale_vaddr = *p_v_coeff_ptr;
            scale_vaddr += 4;
            p_v_coeff_ptr++;
        }
        p_path->v_scale_coeff = v_coeff; 
		_paad(DCAM_PATH_CFG, ~0xF0);
		_paod(DCAM_PATH_CFG, ( (*p_v_coeff_ptr) & 0x0F) << 4);
    }
    
    	_paad(DCAM_CFG, ~BIT_3);
    return rtn;
#else
	return _ISP_DriverGenScxCoeff(base_addr,  ISP_PATH1);
#endif
	
}
#if 0
LOCAL BOOLEAN _ISP_DrvierIsSysLittleEndian(void)
{
    uint32_t                    cell = 0x5A;
    
    return (*(uint8_t*)&cell == 0x5A);
}


LOCAL int32_t _ISP_DriverSetMasterEndianness(uint32_t base_addr, 
                                           uint32_t master_sel,
                                           uint32_t is_rgb565)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    uint32_t                    endian_sel = ISP_MASTER_ENDIANNESS_LITTLE;
    
    if(master_sel >= ISP_MASTER_MAX)
        return ISP_DRV_RTN_MASTER_SEL_ERR;
        
    if(_ISP_DrvierIsSysLittleEndian())
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
        _paad(ENDIAN_SEL, ~0xC);
	_paod(ENDIAN_SEL, (endian_sel & 0x3) << 2);		
    }


    return rtn;

}

#endif

#ifdef ISP_DRV_SCALE_COEFF_TABLE_EN
LOCAL uint32_t _ISP_DriverCalcScaleCoeff(uint32_t input_width, 
                                       uint32_t input_height, 
                                       uint32_t output_width, 
                                       uint32_t output_height,
                                       uint32_t *p_h_coeff,
                                       uint32_t *p_v_coeff)
{
    ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;
    uint32_t                    v_coeff = 0;
    uint32_t                    h_coeff= 0;    
	
    
    h_coeff = (output_width * ISP_PATH_SCALE_LEVEL + input_width - 1) / input_width;
    v_coeff = (output_height * ISP_PATH_SCALE_LEVEL + input_height -1) / input_height;

    if(( ISP_PATH_SCALE_LEVEL_MAX < h_coeff ) ||( ISP_PATH_SCALE_LEVEL_MIN > h_coeff ) ||
       ( ISP_PATH_SCALE_LEVEL_MAX < v_coeff ) ||( ISP_PATH_SCALE_LEVEL_MIN > v_coeff ) )
    {
        rtn = ISP_DRV_RTN_PATH_SC_COEFF_ERR;
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

//////////////////
typedef void (*isr_func_t)(void);

void _ISP_DriverEnableInt(void)
{
  _paod(INT_IRQ_EN, 1<<27);
}
void _ISP_DriverDisableInt(void)
{
  _paod(INT_IRQ_DISABLE, 1<<27);  
}

static irqreturn_t _ISP_DriverISR(int irq, void *dev_id)
{
	uint32_t value = _pard(DCAM_INT_STS) & 0x1FF;
	if(0 == value)
		return IRQ_NONE;
  _ISP_ISRSystemRoot(0);
  return IRQ_HANDLED;
}
PUBLIC int ISP_DriverRegisterIRQ(void)
{
  uint32_t ret = 0;

  //enable dcam interrupt bit on global level.
 _ISP_DriverEnableInt();

  if(0 != (ret = request_irq(IRQ_LINE_DCAM, _ISP_DriverISR, SA_SHIRQ, "DCAM", &g_share_irq)))
    DCAM_ASSERT(0);

  return ret;
}
PUBLIC void ISP_DriverUnRegisterIRQ(void)
{
  //disable dcam interrupt bit on global level.
  _ISP_DriverDisableInt();

  //unregister_interrupt(IRQ_LINE_DCAM);
  free_irq(IRQ_LINE_DCAM, &g_share_irq);
}

PUBLIC uint32_t ISP_DriverSetBufferAddress(uint32_t base_addr, uint32_t buf_addr)
{
    	ISP_PATH_DESCRIPTION_T    *p_path = &s_isp_mod.isp_path1;
    	ISP_DRV_RTN_E             rtn = ISP_DRV_RTN_SUCCESS;

		DCAM_TRACE("###DCAM_DRV:ISP_DriverSetBufferAddress  buff: %x, w: %d, h: %d.\n", buf_addr, p_path->output_size.w, p_path->output_size.h);

	_pawd(DCAM_ADDR_7, buf_addr >> 8);
	_pawd(DCAM_ADDR_H, buf_addr >> (32 - ISP_PATH_FRAME_HIGH_BITS));	
        	if(p_path->input_format == ISP_CAP_INPUT_FORMAT_YUV) // not JPEG
        	{
			_pawd(DCAM_ADDR_8, (buf_addr + p_path->output_size.w * p_path->output_size.h) >> 8);
       		 }        	
	_ISP_DriverAutoCopy(base_addr);

	return rtn;
}


