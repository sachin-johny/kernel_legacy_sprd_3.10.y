/******************************************************************************
 ** File Name:      aud_enha.h                                                *
 ** Author:         Cherry.Liu                                             *
 ** DATE:           01/24/2011                                               *
 ** Copyright:      2011 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    
 ******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 01/24/2011       Cherry.Liu       Create.                                  *
 ******************************************************************************/

#ifndef _AUD_ENHA_H
#define _AUD_ENHA_H

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/
#include "vb_drvapi.h"
#include "filter_calc.h"
#include <linux/types.h>
#include "aud_enha_exp.h"

#ifdef __cplusplus
    extern   "C"
    {
#endif

//#include "os_api.h"

/**---------------------------------------------------------------------------*
 **                         MACRO Definations                                     *
 **---------------------------------------------------------------------------*/
#define AUDIO_NO_ERROR  0
#define AUDIO_ERROR  1
/**---------------------------------------------------------------------------*
 **                         Data Structures                                   *
 **---------------------------------------------------------------------------*/
//add for 5-bands eq support(eg.sc8800g)


typedef enum
{
	HPF_DATA_WIDTH_16 = 16,            
	HPF_DATA_WIDTH_24 = 24,                      
	HPF_DATA_WIDTH_MAX
}HPF_DATA_WIDTH_TYPE_E;

typedef struct 
{
    int16_t    left_gain;     
    int16_t    right_gain;     
}DG_CONTROL_PARAM_T; 

typedef struct 
{
    VBC_DA_MIX_MODE_E left_fm_mix_mode;
    VBC_DA_MIX_MODE_E right_fm_mix_mode;
}DAPATH_CONTROL_PARAM_T; 
 
typedef struct 
{
    BOOLEAN alc_sw;
    int16_t   alc_input_gain;   
    int16_t   alc_ingain_Set;    
    VBC_ALC_PARAS_T alc_Para;
}ALC_CONTROL_PARAM_T; 

typedef struct 
{
    int16_t s[HPF_S_GAIN_NUM];  //s0 ~ s5       
}HPF_GAIN_PARAM_T;  


typedef struct 
{
    HPF_DATA_WIDTH_TYPE_E  data_width;
    int16_t    r_limit; 
    HPF_GAIN_PARAM_T    s_gain;//s0~s6 
    FILTER_LCF_CALC_PARA_T lcf_para;//served for hpf-1
    uint32_t    eq_band_num;//0-8
    BOOLEAN  low_shelve_on;
    BOOLEAN  high_shelve_on;    
    BOOLEAN  band_sw[EQ_BAND_MAX]; 
    EQ_BAND_INPUT_PARA_T  eq_band_para[EQ_BAND_MAX];   
}HPF_CONTROL_PARAM_T; 



/**---------------------------------------------------------------------------*
 **                         Global Variables                                  *
 **---------------------------------------------------------------------------*/
 
/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 **                      public  Function Prototypes                               *
 **---------------------------------------------------------------------------*/

 PUBLIC int	AUDENHA_SetPara(
	 AUDIO_TOTAL_T *audio_param_ptr
);


 /*****************************************************************************/
//  Description:    init HPF filters:
//                  the first hpf is for lcf;
//                  the second to sixth hp are for 5-band eq;
//
//  Author:         Cherry.Liu
//  Note:           
//****************************************************************************/ 
PUBLIC int AUDENHA_InitHpf(
    HPF_CONTROL_PARAM_T* hpf_param_ptr,
    int32_t sample_rate
);

/*****************************************************************************/
//  Description:    set eq mode para for audio process modules, including lcf,alc and eq.
//  Author:         Cherry.Liu
//  Note:             
//****************************************************************************/  
PUBLIC int AUDENHA_SetEqMode(
    ALC_CONTROL_PARAM_T* alc_param_ptr,        //alc
    HPF_CONTROL_PARAM_T* hpf_param_ptr,        //hpf
    int32_t sample_rate                         //fs
);

/*****************************************************************************/
//  Description:    set digital gain 
//  Author:         Cherry.Liu
//  Note:           dg gain here is mapped to arm volume[i] bit15 ~ bit9
//****************************************************************************/  
PUBLIC BOOLEAN AUDENHA_SetDigiGain(
    DG_CONTROL_PARAM_T* dg_param_ptr
);

/*****************************************************************************/
//  Description:    init audio process modules including lcf,agc and eq.
//  Author:         Cherry.Liu
//  Note:           !attention! you should init it before music start. 
//****************************************************************************/  
PUBLIC int AUDENHA_Init(
    DG_CONTROL_PARAM_T* dg_param_ptr,          //dg
    DAPATH_CONTROL_PARAM_T * dapath_param_ptr, //da path
    ALC_CONTROL_PARAM_T* alc_param_ptr,        //alc
    HPF_CONTROL_PARAM_T* hpf_param_ptr,        //hpf
    int32_t sample_rate                         //fs
);

/*****************************************************************************/
//  Description:    deinti aud proc plugger
//  Author:         Cherry.Liu
//  Note:           
//****************************************************************************/  
PUBLIC BOOLEAN AUDENHA_DeInit(
    void
);


/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/    
#ifdef __cplusplus
}
#endif

#endif  // _AUD_AGC_H

// End of aud_agc.h



