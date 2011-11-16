/******************************************************************************
** File Name:      aud_enha_exp.c                                            
** Author:         cherry.liu                                              
** DATE:           12/16/2010                                                
** Copyright:      2010 Spreadtrum, Incorporated. All Rights Reserved.         
** Description:    This file defines the basic operation interfaces 
**                 of audio enhanced express plugger.                   
******************************************************************************

******************************************************************************
**                        Edit History                                       
**  -----------------------------------------------------------------------  
** DATE           NAME             DESCRIPTION                               
** 12/16/2010      cherry.liu      Create.                                   
******************************************************************************/

/**---------------------------------------------------------------------------**
**                         Dependencies                                      **
**---------------------------------------------------------------------------**/



#include "aud_enha.h"

#include "filter_calc.h"
#include <linux/delay.h>
#include <linux/string.h>
#include "vbc-codec.h"


/**---------------------------------------------------------------------------**
**                         Compiler Flag                                      **
**----------------------------------------------------------------------------**/
#ifdef __cplusplus
extern   "C"
{
#endif


/**---------------------------------------------------------------------------**
 **                         MACRO Definations                                 **
 **---------------------------------------------------------------------------**/

/**---------------------------------------------------------------------------**
 **                         Data Structures                                   **
 **---------------------------------------------------------------------------**/
typedef struct 
{
//    AUDIO_ENHA_EQ_STRUCT_T eq_para_set[AUD_ENHA_EQPARA_MAX-1];
	AUDIO_ENHA_EQ_STRUCT_T eq_para_set;

    AUDIO_ENHA_TUNABLE_EQ_STRUCT_T tunable_eq_para_set[AUD_ENHA_TUNABLE_EQPARA_MAX-1]; 
}AUDIO_ENHA_EQ_NV_PARA_T;

/**---------------------------------------------------------------------------**
 **                         const data                                   **
 **---------------------------------------------------------------------------**/
 
/**---------------------------------------------------------------------------**
 **                         Global Variables                                  **
 **---------------------------------------------------------------------------**/
LOCAL AUDIO_ENHA_EQ_NV_PARA_T s_enha_eq_para_nv ={0};
LOCAL DG_CONTROL_PARAM_T      s_cur_dg_param     = {0};   //dg
LOCAL DAPATH_CONTROL_PARAM_T  s_cur_dapath_param = {0};   //da path
LOCAL ALC_CONTROL_PARAM_T     s_cur_alc_param    = {0};   //alc
LOCAL HPF_CONTROL_PARAM_T     s_cur_hpf_param    = {0};   //hpf





LOCAL uint32_t  s_cur_sample_rate         = 44100;
LOCAL uint32_t  s_cur_eq_para_set_index  = 0;
LOCAL uint32_t  s_cur_tunable_eq_para_set_index  = 1;

LOCAL uint32_t  s_cur_music_type  = 0; //music type:mp3
LOCAL AUD_ENHA_EQMODE_SEL_E  s_cur_eq_mode_sel       =  AUD_ENHA_EQMODE_SEL_OFF;

/**---------------------------------------------------------------------------**
 **                        local functions definition                               **
 **---------------------------------------------------------------------------**/

/*****************************************************************************/
//  Description:   get paras from eq para set 
//                   
//  Author:         Cherry.Liu
//  Note:           
//****************************************************************************/  
LOCAL int  AUDENHA_GetParaFromEqset(
    BOOLEAN is_eq_para_tunable,
	uint32_t eq_para_set_index,
	uint32_t tunable_eq_para_set_index	
    )
{
    uint16_t eq_mode_index = 0;
    uint16_t i =0;

    printk("aud_enha_exp.c,[AUDENHA_GetParaFromEqset]is_eq_para_tunable:%d,eq_para_set_index:%d,tunable_eq_para_set_index:%d,s_cur_eq_mode_sel:%d  \n",
        is_eq_para_tunable,eq_para_set_index,tunable_eq_para_set_index,s_cur_eq_mode_sel);
//	s_cur_eq_mode_sel=AUD_ENHA_EQMODE_SEL_REGULAR;   //test	
	//no eq
	if(AUD_ENHA_EQMODE_SEL_OFF == s_cur_eq_mode_sel)
    	{
		//(no eq effect)five band eqs are all-pass filters;dg on;alc on
		//s_cur_alc_param
		s_cur_hpf_param.eq_band_num    = HPF_S_GAIN_NUM-1;
		s_cur_alc_param.alc_ingain_Set = 4096;

		//s_cur_hpf_param
		for(i=0;i<EQ_BAND_MAX;i++)
		{
		    s_cur_hpf_param.band_sw[i] =  SCI_FALSE;
		}
		 printk("aud_enha_exp.c,[AUDENHA_GetParaFromEqset] no eq ! \n");
		 return AUDIO_NO_ERROR;
    	}

	// eq on
    if(0 == eq_para_set_index)
    {
        printk("aud_enha_exp.c,[AUDENHA_GetParaFromEqset] eq_para_set_index == 0! \n");
        return AUDIO_ERROR;
    }   
    if(AUD_ENHA_EQMODE_SEL_MMISET == s_cur_eq_mode_sel)
    {
        eq_mode_index = 0;   
    }
    else 
    {
        eq_mode_index = s_cur_eq_mode_sel-1; // mode set by nv
    }
        
    if(!is_eq_para_tunable)
    {
        AUDIO_ENHA_EQ_STRUCT_T* eq_para_ptr = SCI_NULL;
        eq_para_ptr = &s_enha_eq_para_nv.eq_para_set;

        //s_cur_alc_param
        s_cur_alc_param.alc_ingain_Set = eq_para_ptr->eq_modes[eq_mode_index].agc_in_gain;

        //s_cur_hpf_param
        if(eq_para_ptr->eq_control&0x8000)
        {
            s_cur_hpf_param.eq_band_num = 8;
        }
        else
        {
            s_cur_hpf_param.eq_band_num = 5;
        }  
        s_cur_hpf_param.low_shelve_on  = eq_para_ptr->eq_modes[eq_mode_index].band_control &0x1;//bit0
        s_cur_hpf_param.high_shelve_on = eq_para_ptr->eq_modes[eq_mode_index].band_control &0x2;//bit1
        
        for(i=0;i<s_cur_hpf_param.eq_band_num;i++)
        {
            s_cur_hpf_param.band_sw[i] = ((eq_para_ptr->eq_modes[eq_mode_index].band_control)&(1<<(15-i))) ? SCI_TRUE : SCI_FALSE;
        }
		
        memcpy(&s_cur_hpf_param.eq_band_para[0],&eq_para_ptr->eq_modes[eq_mode_index].eq_band[0],sizeof(EQ_BAND_INPUT_PARA_T)*s_cur_hpf_param.eq_band_num);
    }
    else 
    {
        AUDIO_ENHA_TUNABLE_EQ_STRUCT_T* tunable_eq_para_ptr = SCI_NULL;

        if(tunable_eq_para_set_index > AUD_ENHA_TUNABLE_EQPARA_COMMON)
        {
            tunable_eq_para_set_index = AUD_ENHA_TUNABLE_EQPARA_COMMON;
        }
        
        tunable_eq_para_ptr = &s_enha_eq_para_nv.tunable_eq_para_set[tunable_eq_para_set_index-1];

        //s_cur_alc_param
        s_cur_alc_param.alc_ingain_Set = tunable_eq_para_ptr->eq_modes[eq_mode_index].agc_in_gain;

        //s_cur_hpf_param
        if(tunable_eq_para_ptr->eq_control&0x8000)
        {
            s_cur_hpf_param.eq_band_num = 8;
        }
        else
        {
            s_cur_hpf_param.eq_band_num = 5;
        }  
        s_cur_hpf_param.low_shelve_on  = tunable_eq_para_ptr->eq_modes[eq_mode_index].band_control &0x1;//bit0
        s_cur_hpf_param.high_shelve_on = tunable_eq_para_ptr->eq_modes[eq_mode_index].band_control &0x2;//bit1
        
        for(i=0;i<s_cur_hpf_param.eq_band_num;i++)
        {
            s_cur_hpf_param.band_sw[i]              =  SCI_TRUE ;
            s_cur_hpf_param.eq_band_para[i].fo      = tunable_eq_para_ptr->fo_array[i];
            s_cur_hpf_param.eq_band_para[i].q       = tunable_eq_para_ptr->q_array[i];
            s_cur_hpf_param.eq_band_para[i].boostdB = tunable_eq_para_ptr->eq_modes[eq_mode_index].boostdB_current[i]*(tunable_eq_para_ptr->eq_control&0x3ff);
            s_cur_hpf_param.eq_band_para[i].gaindB  = 0;
        }

        printk("aud_enha_exp.c,[AUDENHA_GetParaFromEqset] tunable eq !current boost array:%d,%d,%d,%d,%d  \n",tunable_eq_para_ptr->eq_modes[eq_mode_index].boostdB_current[0],
            tunable_eq_para_ptr->eq_modes[eq_mode_index].boostdB_current[1],
            tunable_eq_para_ptr->eq_modes[eq_mode_index].boostdB_current[2],
            tunable_eq_para_ptr->eq_modes[eq_mode_index].boostdB_current[3],
            tunable_eq_para_ptr->eq_modes[eq_mode_index].boostdB_current[4]);
    }

    return AUDIO_NO_ERROR;

}


/*****************************************************************************/
//  Description:   get para from audio mode dev info 
//                   
//  Author:         Cherry.Liu
//  Note:           
//****************************************************************************/  
LOCAL int  AUDENHA_GetParaFromAudMode(
    AUDIO_NV_ARM_MODE_INFO_T* armAudioInfo
)
{
    uint16_t aud_proc_control[2];  

    //aud_proc_control
    aud_proc_control[0] = armAudioInfo->tAudioNvArmModeStruct.app_config_info_set.aud_proc_exp_control[0];//bit7:defined;bit3-bit0:defined;bit4-bit6:defined
    aud_proc_control[1] = armAudioInfo->tAudioNvArmModeStruct.app_config_info_set.aud_proc_exp_control[1];//bit15-bit8:agc sw;bit7-bit0:lcf sw


	printk("aud_enha_exp.c,[AUDENHA_GetParaFromAudMode] aud_proc_control[0]:0x%x;aud_proc_control[1]:0x%x  \n", aud_proc_control[0],aud_proc_control[1]); 
   
   
    //s_cur_alc_param
    s_cur_alc_param.alc_sw  = (aud_proc_control[1] & (1 << 8)) ? SCI_TRUE : SCI_FALSE;//bit 8
    s_cur_alc_param.alc_input_gain  =  armAudioInfo->tAudioNvArmModeStruct.app_config_info_set.app_config_info[s_cur_music_type].agc_input_gain[s_cur_music_type]; //s_cur_music_type=0:multimedia play(mp3)
    memcpy(&s_cur_alc_param.alc_Para,&armAudioInfo->tAudioNvArmModeStruct.reserve[7],sizeof(VBC_ALC_PARAS_T));

    printk("aud_enha_exp.c,[AUDENHA_GetParaFromAudMode] s_cur_alc_param.alc_sw:%d;s_cur_alc_param.alc_input_gain:%d  \n",s_cur_alc_param.alc_sw,s_cur_alc_param.alc_input_gain); 

    //s_cur_eq_mode_sel
    s_cur_eq_mode_sel = (AUD_ENHA_EQMODE_SEL_E)(armAudioInfo->tAudioNvArmModeStruct.app_config_info_set.app_config_info[s_cur_music_type].eq_switch&0xF);//BIT 3-BIT 0

    //s_cur_hpf_param
    s_cur_hpf_param.lcf_para.isFilterOn = (aud_proc_control[1] & 0x1) ? SCI_TRUE : SCI_FALSE;//bit 0
    s_cur_hpf_param.lcf_para.eLcfParaType = (FILTER_LCFPARA_TYPE_E)(armAudioInfo->tAudioNvArmModeStruct.reserve[0]&0x0700);   //BIT10  - BIT8
  
    if(FILTER_LCFPARA_F1F1 == s_cur_hpf_param.lcf_para.eLcfParaType)
    {
        s_cur_hpf_param.lcf_para.unlcfPara.lcfPara.f1_g0 = armAudioInfo->tAudioNvArmModeStruct.reserve[1];
        s_cur_hpf_param.lcf_para.unlcfPara.lcfPara.f1_g1 = armAudioInfo->tAudioNvArmModeStruct.reserve[2];
        s_cur_hpf_param.lcf_para.unlcfPara.lcfPara.f1_fp = armAudioInfo->tAudioNvArmModeStruct.reserve[3];
        s_cur_hpf_param.lcf_para.unlcfPara.lcfPara.f2_g0 = armAudioInfo->tAudioNvArmModeStruct.reserve[4];
        s_cur_hpf_param.lcf_para.unlcfPara.lcfPara.f2_g1 = armAudioInfo->tAudioNvArmModeStruct.reserve[5];
        s_cur_hpf_param.lcf_para.unlcfPara.lcfPara.f2_fp = armAudioInfo->tAudioNvArmModeStruct.reserve[6];
    }
    else
    {
        s_cur_hpf_param.lcf_para.unlcfPara.fp  = armAudioInfo->tAudioNvArmModeStruct.reserve[1];//fp
    }
    

    s_cur_hpf_param.r_limit    = (armAudioInfo->tAudioNvArmModeStruct.reserve[0] &0xff);

	
  
    //s_cur_dapath_param
    s_cur_dapath_param.left_fm_mix_mode  = (armAudioInfo->tAudioNvArmModeStruct.reserve[0])&0x3000;//BIT13-BIT12
    s_cur_dapath_param.right_fm_mix_mode = (armAudioInfo->tAudioNvArmModeStruct.reserve[0])&0xC000;//BIT15-BIT14
	printk("aud_enha_exp.c,[AUDENHA_GetParaFromAudMode] lcf_sw:%d;filter_type:%d r_limit:%d \n",s_cur_hpf_param.lcf_para.isFilterOn,s_cur_hpf_param.lcf_para.eLcfParaType,s_cur_hpf_param.r_limit); 

    return AUDIO_NO_ERROR;
}

/*****************************************************************************/
//  Description:    参数获取函数
//  Author:         Cherry.Liu
//  Note:           
//****************************************************************************/ 
LOCAL int  AUDENHA_GetPara(
	AUDIO_TOTAL_T *audio_param_ptr
    )
{
	AUDIO_NV_ARM_MODE_INFO_T *armAudioInfo = NULL;
	char *mode_name = NULL;
	BOOLEAN eq_para_tunable = SCI_FALSE;
	int32_t vol_index = 0;
	int16_t arm_vol = 0;
	
	s_cur_sample_rate=get_cur_sample_rate();
	vol_index = audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.app_config_info_set.app_config_info[s_cur_music_type].valid_volume_level_count;

	arm_vol=audio_param_ptr->audio_nv_arm_mode_info.tAudioNvArmModeStruct.app_config_info_set.app_config_info[s_cur_music_type].arm_volume[vol_index];
    s_cur_dg_param.left_gain = arm_vol >> 9;				//get s_cur_dg_param
    s_cur_dg_param.right_gain = arm_vol>>9;

	armAudioInfo = &audio_param_ptr->audio_nv_arm_mode_info;
	mode_name = (char *)armAudioInfo->ucModeName;
    if(NULL==mode_name)
    {
    	printk("aud_enha_exp.c AUDENHA_GetPara mode_name:%s.\n", mode_name);
    	return AUDIO_ERROR;
    }
	
	AUDENHA_GetParaFromAudMode(armAudioInfo);
	s_enha_eq_para_nv.eq_para_set= audio_param_ptr->audio_enha_eq;


	//s_cur_tunable_eq_para_set_index & s_cur_eq_para_set_index
    if(0 == memcmp((void*)mode_name, "Headset",7))
    {
        s_cur_eq_para_set_index = 1;
    }
    else if(0 == memcmp((void*)mode_name, "Headfree",8))
    {
        s_cur_eq_para_set_index = 2;
    }
    else if(0 == memcmp((void*)mode_name, "Handset",7))
    {
        s_cur_eq_para_set_index = 3;
    }
    else
    {
        s_cur_eq_para_set_index = 4;
    }

	printk("AUDENHA_GetPara mode_name:%s , s_cur_eq_para_set_index :%d \n",mode_name,s_cur_eq_para_set_index);
   if(armAudioInfo->tAudioNvArmModeStruct.reserve[0]&0x0800)
    {
        //EQ  Tunable
        eq_para_tunable = SCI_TRUE;
    }
    else
    {
        //EQ  unTunable
        eq_para_tunable = SCI_FALSE;
    }
		
	if( AUDIO_NO_ERROR != AUDENHA_GetParaFromEqset(SCI_FALSE,s_cur_eq_para_set_index,s_cur_tunable_eq_para_set_index) ) //pass eq
	{
		printk("aud_enha_exp.c AUDENHA_GetPara param error!\n");
		return AUDIO_ERROR;
	}

//s_cur_hpf_param.s_gain.s[0]~[6]
    s_cur_hpf_param.s_gain.s[0] = 4096;
    s_cur_hpf_param.s_gain.s[1] = 4096;
    s_cur_hpf_param.s_gain.s[2] = 4096;
    s_cur_hpf_param.s_gain.s[3] = 4096;
    s_cur_hpf_param.s_gain.s[4] = 4096;
    s_cur_hpf_param.s_gain.s[5] = 4096;
    s_cur_hpf_param.data_width = HPF_DATA_WIDTH_24;

    return AUDIO_NO_ERROR;
}

/*****************************************************************************/
//  Description:    参数设置函数
//  Author:         Cherry.Liu
//  Note:           
//****************************************************************************/ 
PUBLIC int  AUDENHA_SetPara(
	AUDIO_TOTAL_T *audio_param_ptr
    )
{       
   
	if(AUDIO_NO_ERROR != AUDENHA_GetPara(audio_param_ptr))
	{
		printk("aud_enha_exp.c AUDENHA_GetPara error!\n");
		return AUDIO_ERROR;
	}

    if(AUDIO_NO_ERROR != AUDENHA_Init(&s_cur_dg_param,   //dg
            &s_cur_dapath_param,      //da path
            &s_cur_alc_param,         //alc
            &s_cur_hpf_param,         //hpf
            s_cur_sample_rate  //fs
            ))
	{
		printk("aud_enha_exp.c AUDENHA_Init error! \n");
		return AUDIO_ERROR;
	}
	printk("aud_enha_exp.c AUDENHA_SetPara success!\n");

    return AUDIO_NO_ERROR;
}


/**---------------------------------------------------------------------------**
 **                         Compiler Flag                                     **
 **---------------------------------------------------------------------------**/ 
#ifdef __cplusplus
}
#endif //end of file aud_enha_exp.c





