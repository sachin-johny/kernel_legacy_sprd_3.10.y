/*
* drivers/media/video/sprd_dcam/dcam_sc8800g2.c
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
#include "dcam_sc8800g2.h"
#include "dcam_power_sc8800g2.h"
#include "sensor_drv.h"

#define ISP_ALIGNED_PIXELS 4

typedef struct dcam_parameter
{
  DCAM_MODE_TYPE_E mode;
  DCAM_DATA_FORMAT_E format; 
  DCAM_YUV_PATTERN_E yuv_pattern; 
  RGB_TYPE_E display_rgb_type;
  DCAM_SIZE_T0 input_size;
  DCAM_POLARITY_T polarity;
  DCAM_RECT_T0 input_rect;
  DCAM_RECT_T0 display_rect;
  DCAM_RECT_T0 encoder_rect;  
  DCAM_ROTATION_E rotation;
  int skip_frame;
  uint32_t first_buf_addr;
}DCAM_PARAMETER_T;
DCAM_PARAMETER_T g_dcam_param;

typedef  int (*ISP_USER_FUNC_PTR)(void*);


void get_dcam_reg(void);

//service ID
#define ISP_SERVICE_IDLE                0x00
#define ISP_SERVICE_MPEG                0x01
#define ISP_SERVICE_JPEG                0x02
#define ISP_SERVICE_PREVIEW             0x03
#define ISP_SERVICE_SCALE               0x04
#define ISP_SERVICE_REVIEW              0x07
#define ISP_SERVICE_VP                  0x08
#define ISP_SERVICE_VP_ENCODE           0x09
#define ISP_SERVICE_VP_DECODE           0x10
#define ISP_SERVICE_VP_PLAYBACK         0x17

//machine state ID
//machine state ID
#define ISP_STATE_STOP                  1
#define ISP_STATE_PREVIEW               2
#define ISP_STATE_CAPTURE               3
#define ISP_STATE_PAUSE                 4
#define ISP_STATE_SCALE_DONE            7
#define ISP_STATE_MPEG                  8

//isp next to when jpeg mode
#define ISP_STATE_CAPTURE_DONE          9
#define ISP_STATE_CAPTURE_SCALE         10
#define ISP_STATE_CAPTURE_CFA           11
#define ISP_STATE_VP                    12
#define ISP_STATE_IDLETEMP              13
#define ISP_STATE_REVIEW_DONE           14
#define ISP_STATE_REVIEW_SLICE          15
#define ISP_STATE_SCALE_SLICE           16
#define ISP_STATE_SCALE                 17
#define ISP_STATE_REVIEW                18
#define ISP_STATE_CLOSED                0xFF

typedef struct _isp_display_block_t
{
    uint32_t                              lcd_id;
    uint32_t                              block_id;//block id
    ISP_RECT_T                          img_rect;
    ISP_RECT_T                          lcd_inv_rect;
    uint32_t                              img_fmt;
    uint32_t                              is_enable;
}ISP_DISP_BLOCK_T;

typedef struct _isp_service_t
{
    uint32_t                              service;//service id;
    uint32_t                              state;//machine state
    uint32_t                              module_addr;

    /*for cap*/
    uint32_t                              cap_input_image_format;
    uint32_t                              cap_input_image_pattern;
    uint32_t                              vsync_polarity;
    uint32_t                              hsync_polarity;
    uint32_t                              pclk_polarity;

    uint32_t                              is_first_frame;
    uint32_t                              preview_skip_frame_num;
    uint32_t                              preview_deci_frame_num;	
    uint32_t                              capture_skip_frame_num;
    ISP_SIZE_T                          cap_input_size;   
    ISP_RECT_T                          cap_input_range;
    ISP_CAP_DEC_T                       cap_img_dec;
    ISP_SIZE_T                          cap_output_size;	
	
    /*for review , scale , vt review */	
    ISP_SIZE_T                          input_size;   
    ISP_RECT_T                          input_range;	
    ISP_ADDRESS_T                       input_addr;	
    ISP_ADDRESS_T                       swap_buff;
    ISP_ADDRESS_T                       line_buff;
    ISP_ADDRESS_T                       output_addr;
    uint32_t                              is_slice;
    uint32_t                              slice_height;  
    uint32_t                              slice_out_height;      
    uint32_t                              total_scale_out;
    uint32_t                              total_slice_count;
    uint32_t                              vt_review_from; // 0 local, 1, remote
    uint32_t                              is_review_dc;
    
    /*local display(preview,review parameter , include block,round buffer,*/
    uint32_t                              local_review_enable;    
    uint32_t                              local_display_enable;
    ISP_DISP_BLOCK_T                    display_block; 
    ISP_RECT_T                          display_range;// preview, review,mpeg preview	
    ISP_ADDRESS_T                       display_addr[ISP_PATH1_FRAME_COUNT_MAX];
    ISP_FRAME_T                         display_frame;
    ISP_FRAME_T                         *display_frame_locked_ptr;
    ISP_ROTATION_E                      display_rotation;
    ISP_ADDRESS_T                       rotation_buf_addr[ISP_PATH1_FRAME_COUNT_MAX];
    ISP_ADDRESS_T                       local_addr[ISP_PATH1_FRAME_COUNT_MAX];
    
    /*remote display(vt review) parameter , include block,round buffer,*/
    ISP_SIZE_T                          remote_input_size;   
    ISP_RECT_T                          remote_input_range;	
    ISP_ADDRESS_T                       remote_input_addr;	
	
    uint32_t                              remote_deblock_enable;
    uint32_t                              remote_review_enable;    
    uint32_t                              remote_display_enable;
    ISP_DISP_BLOCK_T                    remote_display_block; 
    ISP_RECT_T                          remote_display_range;	
    ISP_ADDRESS_T                       remote_display_addr[ISP_PATH2_FRAME_COUNT_MAX];
    ISP_ROTATION_E                      remote_display_rotation; 
    ISP_ADDRESS_T                       remote_rotation_display_addr[ISP_PATH2_FRAME_COUNT_MAX];

    ISP_SIZE_T                          encoder_size;	//for jpeg capture ,mpeg capture, vt capture, scale out
    ISP_ADDRESS_T                       encoder_addr[ISP_PATH2_FRAME_COUNT_MAX];
    ISP_ADDRESS_T                       encoder_rot_addr[ISP_PATH2_FRAME_COUNT_MAX];
    ISP_ROTATION_E                      enc_rotation; 
	uint32_t								enc_stoped;
    uint32_t                              is_enc_first_frame;	
    uint32_t                              jpeg_capture_is_composed;
    ISP_FRAME_T                         *p_vt_enc;
    ISP_FRAME_T                         *p_vt_enc_cur;    
    uint32_t                              vt_local_encode_count;    
    uint32_t                              vt_local_display_count;    	
    uint32_t                              vt_review_remote_count;
    uint32_t                              vt_stop_arrived;

    /*control variable*/
    ISP_USER_FUNC_PTR                   display_user_cb;
    DCAM_MUTEX_PTR                       display_user_cb_ctrl;    
    ISP_USER_FUNC_PTR                   encode_user_cb;
    DCAM_MUTEX_PTR                       encode_user_cb_ctrl;
    uint32_t                              chng_freq_req_handler;
    get_data                            scale_user_cb;
    DCAM_SEMAPHORE_PTR                   review_path_sema;
    uint32_t                              curr_queue_available;
    uint32_t                              b_wait_for_vt_event;
    DCAM_TIMER_PTR                       timer_ptr;	
    uint32_t                              timer_mode;
    uint32_t                              sys_performance_level;
    uint32_t                              watchdog_feeded;
}ISP_SERVICE_T;

////////////////////

#define FREQ_INDEX_ISP_HIGH 7
#define FREQ_INDEX_ISP_LOW 6


#define ISP_RTN_IF_ERR(n)               if(n)  goto exit

LOCAL ISP_SERVICE_T          s_isp_service;// = {0};
CALLBACK_FUNC_PTR g_dcam_cb[DCAM_CB_NUMBER];


int dcam_parameter_init(DCAM_INIT_PARAM_T *init_param)
{
  //uint32 i;
  DCAM_TRACE("###dcam: dcam_parameter_init start. \n");

  g_dcam_param.mode = init_param->mode;
  g_dcam_param.format = init_param->format;
  g_dcam_param.yuv_pattern = init_param->yuv_pattern;
  g_dcam_param.display_rgb_type = init_param->display_rgb_type;
  g_dcam_param.input_size.w = init_param->input_size.w;
  g_dcam_param.input_size.h = init_param->input_size.h;
  g_dcam_param.polarity.hsync = init_param->polarity.hsync;
  g_dcam_param.polarity.vsync = init_param->polarity.vsync;
  g_dcam_param.polarity.pclk = init_param->polarity.pclk;
  g_dcam_param.input_rect.x = init_param->input_rect.x;
  g_dcam_param.input_rect.y = init_param->input_rect.y;
  g_dcam_param.input_rect.w = init_param->input_rect.w;
  g_dcam_param.input_rect.h = init_param->input_rect.h;
  g_dcam_param.display_rect.x = init_param->display_rect.x;
  g_dcam_param.display_rect.y = init_param->display_rect.y;
  g_dcam_param.display_rect.w = init_param->display_rect.w;
  g_dcam_param.display_rect.h = init_param->display_rect.h;
  g_dcam_param.encoder_rect.x = init_param->encoder_rect.x;
  g_dcam_param.encoder_rect.y = init_param->encoder_rect.y;
  g_dcam_param.encoder_rect.w = init_param->encoder_rect.w;
  g_dcam_param.encoder_rect.h = init_param->encoder_rect.h;
  g_dcam_param.rotation = init_param->rotation;  
  g_dcam_param.skip_frame = init_param->skip_frame;
  g_dcam_param.first_buf_addr = init_param->first_buf_addr;
  DCAM_TRACE("###dcam: dcam_parameter_init mode: %d, format: %d, yuv_pattern: %d. \n",g_dcam_param.mode,g_dcam_param.format,g_dcam_param.yuv_pattern);      
 DCAM_TRACE("###dcam: dcam_parameter_init disp w: %d, disp h: %d, input_rect:w: %d, h:%d\n",g_dcam_param.display_rect.w,g_dcam_param.display_rect.h,g_dcam_param.input_rect.w,g_dcam_param.input_rect.h);   
  DCAM_TRACE("###dcam: dcam_parameter_init end. \n");
  return 0;  
}

LOCAL void _ISP_ServiceInit(void)
{
    ISP_SERVICE_T           *s = &s_isp_service;
	
    s->module_addr = ISP_AHB_SLAVE_ADDR;
    s->service = ISP_SERVICE_IDLE;
    s->state = ISP_STATE_STOP;
    s->display_block.block_id = ISP_DISPLAY_NONE;
    s->remote_display_block.block_id = ISP_DISPLAY_NONE;	

    return ;
}
LOCAL void _ISP_ServiceDeInit(void)
{
    ISP_SERVICE_T           *s = &s_isp_service;   

    s->state = ISP_STATE_CLOSED;
    s->service = ISP_SERVICE_IDLE;

    return ;
}
static uint32_t g_dcam_user_count = 0;

inline void dcam_inc_user_count(void)
{
	g_dcam_user_count++;
	DCAM_TRACE("dcam_inc_user_count: count: %d.\n", g_dcam_user_count);
}
inline void dcam_dec_user_count(void)
{
	if(g_dcam_user_count >= 1)
		g_dcam_user_count--;
	else
		g_dcam_user_count = 0;
	DCAM_TRACE("dcam_dec_user_count: count: %d.\n", g_dcam_user_count);
}
inline uint32_t dcam_get_user_count(void)
{
	DCAM_TRACE("dcam_get_user_count: count: %d.\n", g_dcam_user_count);
	return g_dcam_user_count;
}

LOCAL void _ISP_ServiceOpen(void)
{
	_ISP_ServiceInit();

	if(0 == dcam_get_user_count())
	{
		ISP_DriverIramSwitch(AHB_GLOBAL_REG_CTL0, IRAM_FOR_ISP); //switch IRAM to isp
		ISP_DriverSoftReset(AHB_GLOBAL_REG_CTL0);  
		ISP_DriverSetClk(ARM_GLOBAL_PLL_SCR, ISP_CLK_48M);
	        DCAM_TRACE("###dcam: _ISP_ServiceOpen softreset and set clk.\n");
	}
	dcam_inc_user_count();
	ISP_DriverScalingCoeffReset();
}
LOCAL uint32_t _ISP_ServiceClose(void)
{
    ISP_SERVICE_T    *s = &s_isp_service;

    if(ISP_STATE_CLOSED == s->state)
    {
        return DCAM_SUCCESS;
    }
    DCAM_TRACE("ISP_SERVICE:ISP_ServiceClose, service = %d", s->service);
    
    ISP_DriverStop(s->module_addr);
	dcam_dec_user_count();
    	if(0 == dcam_get_user_count())
    	{
		ISP_DriverModuleDisable(AHB_GLOBAL_REG_CTL0);
		ISP_DriverIramSwitch(AHB_GLOBAL_REG_CTL0, IRAM_FOR_ARM); //switch IRAM to ARM	
		ISP_DriverSoftReset(AHB_GLOBAL_REG_CTL0);
		DCAM_TRACE("###dcam: _ISP_ServiceClose softreset and set clk.\n");
    	}
	_ISP_ServiceDeInit();
	    
    return DCAM_SUCCESS;
    
}

PUBLIC uint32_t dcam_callback_fun_register(DCAM_CB_ID_E cb_id,
                                      CALLBACK_FUNC_PTR user_func)
{
    uint32_t rtn = DCAM_SUCCESS;

    if(cb_id >= DCAM_CB_NUMBER)
    {
        rtn = DCAM_FAIL;
    }
    else
    {
        g_dcam_cb[cb_id] = user_func;
    }
    return rtn;		
}
LOCAL void _ISP_ServiceOnSensorSOF(void *p)
{
	CALLBACK_FUNC_PTR cb_fun = g_dcam_cb[DCAM_CB_SENSOR_SOF];
	if(cb_fun)
		(*cb_fun)();
}
LOCAL void _ISP_ServiceOnCAPEOF(void *p)
{
	CALLBACK_FUNC_PTR cb_fun = g_dcam_cb[DCAM_CB_CAP_EOF];
	if(cb_fun)
		(*cb_fun)();
}
LOCAL void _ISP_ServiceOnPath1(void *p)
{
	CALLBACK_FUNC_PTR cb_fun = g_dcam_cb[DCAM_CB_PATH1_DONE];
	if(cb_fun)
		(*cb_fun)();
#if 0    
    get_dcam_reg();
#endif
}



LOCAL void _ISP_ServiceStartPreview(void)
{
    ISP_SERVICE_T           *s = &s_isp_service;
    ISP_CAP_SYNC_POL_T      cap_sync = {0};
    int32_t                   rtn_drv = DCAM_SUCCESS;
    ISP_SIZE_T              disp_size = {0};
	
    DCAM_TRACE("ISP_SERVICE:_ISP_ServiceStartPreview E.\n");

    rtn_drv = ISP_DriverModuleInit(s->module_addr);
    ISP_RTN_IF_ERR(rtn_drv);
    DCAM_TRACE("ISP_SERVICE:ISP_DriverModuleInit.\n");
    
    rtn_drv = ISP_DriverSetClk(ARM_GLOBAL_PLL_SCR, ISP_CLK_48M);
    ISP_RTN_IF_ERR(rtn_drv);
    DCAM_TRACE("ISP_SERVICE:ISP_DriverSetClk.\n");
	
        rtn_drv = ISP_DriverSetMode(s->module_addr, ISP_MODE_PREVIEW);
		DCAM_TRACE("ISP_SERVICE:ISP_MODE_PREVIEW.\n");

    ISP_RTN_IF_ERR(rtn_drv);
	
    /*Set CAP*/
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_PRE_SKIP_CNT, 
                                  (void*)&s->preview_skip_frame_num);
    ISP_RTN_IF_ERR(rtn_drv);
		
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_FRM_DECI, 
                                  (void*)&s->preview_deci_frame_num);
    ISP_RTN_IF_ERR(rtn_drv);

    cap_sync.vsync_pol = s->vsync_polarity;
    cap_sync.hsync_pol = s->hsync_polarity;
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_SYNC_POL, 
                                  (void*)&cap_sync);
    ISP_RTN_IF_ERR(rtn_drv);

    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_INPUT_FORMAT, 
                                  (void*)&s->cap_input_image_format);
    ISP_RTN_IF_ERR(rtn_drv);
    
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_YUV_TYPE, 
                                  (void*)&s->cap_input_image_pattern);
    ISP_RTN_IF_ERR(rtn_drv);
	
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_INPUT_RECT, 
                                  (void*)&s->cap_input_range);

    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_FRM_COUNT_CLR, 
                                  NULL);
    ISP_RTN_IF_ERR(rtn_drv);
	
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_IMAGE_XY_DECI, 
                                  (void*)&s->cap_img_dec);
    ISP_RTN_IF_ERR(rtn_drv);
DCAM_TRACE("ISP_SERVICE:ISP_DriverCapConfig.\n");
    disp_size.w = s->display_range.w;
    disp_size.h = s->display_range.h; 
    

//    if(ISP_DATA_YUV422 == s->display_block.img_fmt)
    {
        /*Set Path1*/
        rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                        ISP_PATH_INPUT_SIZE, 
                                        (void*)&s->input_size);
        ISP_RTN_IF_ERR(rtn_drv);

        rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                        ISP_PATH_INPUT_RECT, 
                                        (void*)&s->input_range);
        ISP_RTN_IF_ERR(rtn_drv);

    
        rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                        ISP_PATH_OUTPUT_SIZE, 
                                        (void*)&disp_size);
        ISP_RTN_IF_ERR(rtn_drv);

		
        /*for(i = 0; i < ISP_PATH1_FRAME_COUNT_MAX; i++)
        {
            rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                            ISP_PATH_OUTPUT_ADDR, 
                                            (void*)&s->display_addr[i]);
            ISP_RTN_IF_ERR(rtn_drv);
        }*/

        /*Register ISR callback*/
        rtn_drv = ISP_DriverNoticeRegister(s->module_addr, 
                                           ISP_IRQ_NOTICE_SENSOR_SOF,
                                           _ISP_ServiceOnSensorSOF);
        ISP_RTN_IF_ERR(rtn_drv);
        rtn_drv = ISP_DriverNoticeRegister(s->module_addr, 
                                           ISP_IRQ_NOTICE_CAP_EOF,
                                           _ISP_ServiceOnCAPEOF);
        ISP_RTN_IF_ERR(rtn_drv);		
        rtn_drv = ISP_DriverNoticeRegister(s->module_addr, 
                                           ISP_IRQ_NOTICE_PATH1_DONE,
                                           _ISP_ServiceOnPath1);
        ISP_RTN_IF_ERR(rtn_drv);
    }
  
    ISP_DriverSetBufferAddress(s->module_addr, g_dcam_param.first_buf_addr);
#if 0    
    get_dcam_reg();
#endif  	
    rtn_drv = ISP_DriverStart(s->module_addr);
#if 0    
    get_dcam_reg();
#endif  
exit:

    if(rtn_drv)
    {
        DCAM_TRACE("ISP_SERVICE: preview: driver error code 0x%x",rtn_drv);	
    }
    else
    {
        s->state = ISP_STATE_PREVIEW;
    }
    DCAM_TRACE("ISP_SERVICE:_ISP_ServiceStartPreview X.\n");	
    return ;
	
}

LOCAL uint32_t ISP_ServiceStartPreview(void)
{
    ISP_SERVICE_T         *s = &s_isp_service;

    DCAM_TRACE("ISP_SERVICE:ISP_ServiceStartPreview");
    _ISP_ServiceStartPreview();
    s->is_first_frame = 1;
	
    return DCAM_SUCCESS;
}

void get_dcam_reg(void)
{
  uint32_t i, value;
  for(i = 0; i < 29; i++)
  {
    value = _pard(DCAM_REG_BASE + i * 4);
    DCAM_TRACE("DCAM reg:0x%x, 0x%x.\n", DCAM_REG_BASE + i * 4, value);
  }
  for(i = 0; i < 9; i++)
  {
    value = _pard(DCAM_REG_BASE + 0x0100 + i * 4);
    DCAM_TRACE("DCAM reg:0x%x, 0x%x.\n", DCAM_REG_BASE + 0x0100 + i * 4, value);
  }
  for(i = 0; i < 20; i++)
  {
    value = _pard(AHB_BASE + 0x200 + i * 4);
    DCAM_TRACE("DCAM AHB reg:0x%x, 0x%x.\n", AHB_BASE + 0x0200 + i * 4, value);
  }
}
int dcam_open(void)
{  
DCAM_TRACE("dcam_open E.\n");
	_ISP_ServiceOpen();	
	 //register dcam IRQ
	 ISP_DriverRegisterIRQ();
DCAM_TRACE("dcam_open X.\n");
	 return DCAM_SUCCESS;
}
int dcam_close(void)
{  
	 //unregister dcam IRQ
	 ISP_DriverUnRegisterIRQ();
	 _ISP_ServiceClose();

	 return DCAM_SUCCESS;
}

LOCAL void _ISP_ServiceStartJpeg(void)
{
    ISP_SERVICE_T           *s = &s_isp_service;
    ISP_CAP_SYNC_POL_T      cap_sync = {0};
    int32_t                   rtn_drv = DCAM_SUCCESS;
    ISP_SIZE_T              output_size = {0};
    
    DCAM_TRACE("ISP_SERVICE:_ISP_ServiceStartJpeg.\n");

    rtn_drv = ISP_DriverModuleInit(s->module_addr);
    ISP_RTN_IF_ERR(rtn_drv);
	

    rtn_drv = ISP_DriverSetClk(ARM_GLOBAL_PLL_SCR, ISP_CLK_64M);
    ISP_RTN_IF_ERR(rtn_drv);

    rtn_drv = ISP_DriverSetMode(s->module_addr,
                                ISP_MODE_CAPTURE);
    ISP_RTN_IF_ERR(rtn_drv);


    /*Set CAP*/
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_PRE_SKIP_CNT, 
                                  (void*)&s->capture_skip_frame_num);
    ISP_RTN_IF_ERR(rtn_drv);

    cap_sync.vsync_pol = s->vsync_polarity;
    cap_sync.hsync_pol = s->hsync_polarity;
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_SYNC_POL, 
                                  (void*)&cap_sync);
    ISP_RTN_IF_ERR(rtn_drv);

    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_INPUT_FORMAT, 
                                  (void*)&s->cap_input_image_format);
    ISP_RTN_IF_ERR(rtn_drv);
	
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_YUV_TYPE, 
                                  (void*)&s->cap_input_image_pattern);	
    ISP_RTN_IF_ERR(rtn_drv);

    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_INPUT_RECT, 
                                  (void*)&s->cap_input_range);

    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_FRM_COUNT_CLR, 
                                  NULL);
    ISP_RTN_IF_ERR(rtn_drv);

	
    rtn_drv = ISP_DriverCapConfig(s->module_addr, 
                                  ISP_CAP_IMAGE_XY_DECI, 
                                  (void*)&s->cap_img_dec);
    ISP_RTN_IF_ERR(rtn_drv);


    /*Set Path1*/
    rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                    ISP_PATH_INPUT_SIZE, 
                                    (void*)&s->input_size);
    ISP_RTN_IF_ERR(rtn_drv);

    rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                    ISP_PATH_INPUT_RECT, 
                                    (void*)&s->input_range);
    ISP_RTN_IF_ERR(rtn_drv);

    if(s->is_slice)
    {
        output_size.w = s->cap_output_size.w;
        output_size.h = s->cap_output_size.h;    	    	       
    }
    else
    {
        output_size.w = s->encoder_size.w;
        output_size.h = s->encoder_size.h;    	    	       
    }

    rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                    ISP_PATH_OUTPUT_SIZE, 
                                    (void*)&output_size);
    ISP_RTN_IF_ERR(rtn_drv);

		
    /*rtn_drv = ISP_DriverPath1Config(s->module_addr, 
                                    ISP_PATH_OUTPUT_ADDR, 
                                    (void*)&s->encoder_addr[0]);
    ISP_RTN_IF_ERR(rtn_drv);
    */


    /*Register ISR callback*/
    rtn_drv = ISP_DriverNoticeRegister(s->module_addr, 
                                       ISP_IRQ_NOTICE_PATH1_DONE,
                                       _ISP_ServiceOnPath1);
    ISP_RTN_IF_ERR(rtn_drv);

    ISP_DriverSetBufferAddress(s->module_addr, g_dcam_param.first_buf_addr); //wxz:???
#if 0    
    get_dcam_reg();
#endif    
    rtn_drv = ISP_DriverStart(s->module_addr);
    ISP_RTN_IF_ERR(rtn_drv);

exit:

    if(rtn_drv)
    {
        DCAM_TRACE("ISP_SERVICE:capture: driver error code 0x%x",rtn_drv);	
    }
    else
    {
        if(0 == s->is_slice)
        {
            s->state = ISP_STATE_CAPTURE_DONE;
        }
        else
        {
            s->state = ISP_STATE_CAPTURE_SCALE;
        }
    }
    return ;
}

LOCAL void ISP_ServiceStartCapture(void)
{
	_ISP_ServiceStartJpeg();
}

LOCAL uint32_t _ISP_ServiceGetXYDeciFactor(uint32_t *src_width, uint32_t *src_height, uint32_t dst_width, uint32_t dst_height)
{
	uint32_t i = 0;
	uint32_t width = 0;
	uint32_t height = 0;

	for(i = 1; i < ISP_CAP_DEC_XY_MAX + 1; i++)
	{
		width = *src_width / i;
		height = *src_height / i;
		if(width <= (dst_width * ISP_PATH_SC_COEFF_MAX) && 
		height <= (dst_height * ISP_PATH_SC_COEFF_MAX) && 
		(width % ISP_ALIGNED_PIXELS) == 0 && 
		(height % ISP_ALIGNED_PIXELS) == 0) 
			break;
	}
	*src_width = width;
	*src_height = height;

	return i;
}
LOCAL void ISP_ServiceSetParameters(void)
{
	ISP_SERVICE_T         *s = &s_isp_service;
	SENSOR_EXP_INFO_T sensor_info;
	ISP_SIZE_T dst_img_size;
	uint32_t coeff = 1; //wxz:???

	sensor_info = *Sensor_GetInfo();
	s->cap_input_image_format = sensor_info.image_format;
	s->cap_input_image_pattern = sensor_info.image_pattern;
	s->hsync_polarity = sensor_info.hsync_polarity;
	s->vsync_polarity = sensor_info.vsync_polarity;
	s->pclk_polarity = sensor_info.pclk_polarity;
	s->preview_skip_frame_num = sensor_info.preview_skip_num;
	s->preview_deci_frame_num = sensor_info.preview_deci_num;
	s->cap_input_size.w = g_dcam_param.input_size.w;
	s->cap_input_size.h = g_dcam_param.input_size.h;
	s->cap_input_range.x = g_dcam_param.input_rect.x;
	s->cap_input_range.y = g_dcam_param.input_rect.y;
	s->cap_input_range.w = g_dcam_param.input_rect.w;
	s->cap_input_range.h = g_dcam_param.input_rect.h;
	s->cap_output_size.w = s->cap_input_range.w;
	s->cap_output_size.h = s->cap_input_range.h;
	s->encoder_size.w = g_dcam_param.encoder_rect.w;
	s->encoder_size.h = g_dcam_param.encoder_rect.h;
	s->preview_deci_frame_num = 0;
	if((ISP_ROTATION_90 == g_dcam_param.rotation) || (ISP_ROTATION_270 == g_dcam_param.rotation))
	{
		dst_img_size.w = g_dcam_param.display_rect.h;
		dst_img_size.h = g_dcam_param.display_rect.w;
	}	
	else
	{
		dst_img_size.w = g_dcam_param.display_rect.w;
		dst_img_size.h = g_dcam_param.display_rect.h;
	}	

	s->cap_img_dec.x_factor = _ISP_ServiceGetXYDeciFactor(&s->cap_output_size.w, &s->cap_output_size.h,
								dst_img_size.w * coeff, dst_img_size.h * coeff);	
	s->cap_img_dec.y_factor = s->cap_img_dec.x_factor;
	s->cap_img_dec.x_mode = ISP_CAP_IMG_DEC_MODE_DIRECT;
	s->input_size.w = s->cap_output_size.w;
	s->input_size.h = s->cap_output_size.h;
	s->input_range.x = 0;
	s->input_range.y = 0;
	s->input_range.w = s->cap_output_size.w;
	s->input_range.h = s->cap_output_size.h;
	s->display_range.x = g_dcam_param.display_rect.x;
	s->display_range.y = g_dcam_param.display_rect.y;
	s->display_range.w = g_dcam_param.display_rect.w;
	s->display_range.h = g_dcam_param.display_rect.h;

	s->display_rotation = g_dcam_param.rotation;
}
int dcam_start(void)
{
   DCAM_TRACE("dcam: dcam_start start. \n"); 
   	ISP_ServiceSetParameters();
	if(DCAM_MODE_TYPE_PREVIEW == g_dcam_param.mode)
		 ISP_ServiceStartPreview();
	else if(DCAM_MODE_TYPE_CAPTURE == g_dcam_param.mode)
		 ISP_ServiceStartCapture();
   DCAM_TRACE("dcam: dcam_start end. \n"); 
   
  return DCAM_SUCCESS;
}

void ISP_ServiceStopPreview(void)
{
    ISP_SERVICE_T          *s = &s_isp_service;
	
    DCAM_TRACE("ISP_SERVICE:ISP_ServiceStopPreview E.\n");

    if(s->state != ISP_STATE_PREVIEW)    
	{		
    		DCAM_TRACE("ISP_SERVICE:ISP_ServiceStopPreview: state is not preview.\n");
        	return ;
	}
    
    	ISP_DriverStop(s->module_addr);   
    	s->state = ISP_STATE_STOP;   
		
    DCAM_TRACE("ISP_SERVICE:ISP_ServiceStopPreview X.\n");
    return ;
}

void ISP_ServiceStopCapture(void)
{
    ISP_SERVICE_T          *s = &s_isp_service;
	
    DCAM_TRACE("ISP_SERVICE:ISP_ServiceStopCapture");
    
    	ISP_DriverStop(s->module_addr); 
    	s->state = ISP_STATE_STOP;   
		
    return ;
}
int dcam_stop(void)
{
    ISP_SERVICE_T          *s = &s_isp_service;
   DCAM_TRACE("dcam: dcam_stop start. \n"); 

       	ISP_DriverStop(s->module_addr); 
    	s->state = ISP_STATE_STOP;  

   DCAM_TRACE("dcam: dcam_stop end. \n"); 
   
  return DCAM_SUCCESS;
}

PUBLIC uint32_t dcam_set_buffer_address(uint32_t address)
{
	 ISP_SERVICE_T           *s = &s_isp_service;

	 return ISP_DriverSetBufferAddress(s->module_addr, address);
}

