/******************************************************************************
** File Name:      aud_enha_exp.h                                            
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

#ifndef _AUD_ENHA_EXP_H_
#define _AUD_ENHA_EXP_H_
/**---------------------------------------------------------------------------**
**                         Dependencies                                      **
**---------------------------------------------------------------------------**/
//#include "apm_codec.h"
//#include "asm.h"
//#include "audio_api.h"
#include "vb_drvapi.h"
#include "sci_types.h"
#include <linux/types.h>
#include "eng_audio.h"

//#include "eq_exp.h"
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

//#define NAME_LEN_MAX (16)
//#define EQ_BAND_MAX   8
//#define EQ_MODE_MAX   6
//#define AUD_ENHA_EXP_PARA_COUNT 2	
#define TOTAL_S_GAIN_NUM  7 //s0-s6
#define HPF_S_GAIN_NUM    6 //s0-s5
/**---------------------------------------------------------------------------**
 **                         Data Structures                                   **
 **---------------------------------------------------------------------------**/
typedef enum
{   
    AUD_ENHA_EQPARA_NULL = 0,
    AUD_ENHA_EQPARA_HEADSET ,// 1
    AUD_ENHA_EQPARA_HEADFREE,   // 2
    AUD_ENHA_EQPARA_HANDSET,   // 3
    AUD_ENHA_EQPARA_HANDSFREE,// 4
    AUD_ENHA_EQPARA_MAX// 5
}AUD_ENHA_EQPARA_SET_E;

typedef enum
{   
    AUD_ENHA_TUNABLE_EQPARA_NULL=0,//0
    AUD_ENHA_TUNABLE_EQPARA_COMMON,// 1
    AUD_ENHA_TUNABLE_EQPARA_MAX//  2
}AUD_ENHA_TUNABLE_EQPARA_SET_E;

typedef enum
{   
    AUD_ENHA_EQMODE_SEL_OFF = 0,     //0
    AUD_ENHA_EQMODE_SEL_REGULAR ,   // 1
    AUD_ENHA_EQMODE_SEL_CLASSIC,   // 2
    AUD_ENHA_EQMODE_SEL_ODEUM,     // 3
    AUD_ENHA_EQMODE_SEL_JAZZ,      // 4
    AUD_ENHA_EQMODE_SEL_ROCK,      // 5
    AUD_ENHA_EQMODE_SEL_SOFTROCK,// 6
    AUD_ENHA_EQMODE_SEL_MMISET =15,// 15
    AUD_ENHA_EQMODE_SEL_MAX
}AUD_ENHA_EQMODE_SEL_E;

//------------definition for getting params for this plugger ----------------//




//------------definition for setting params for this plugger ----------------//
typedef enum//  3  para  types
{   
    AUD_ENHA_PARA_EQ_MODE = 0,
    AUD_ENHA_PARA_DIGI_GAIN ,
    AUD_ENHA_PARA_EQ_SET ,
    AUD_ENHA_PARA_DEV_MODE,
    AUD_ENHA_PARA_BAND_BOOST,  
    AUD_ENHA_PARA_SET_DEFAULT,   
    AUD_ENHA_PARA_EQ_ALLBANDS_BOOST,
    AUD_ENHA_PARA_TUNABLE_EQ_SET,
    AUD_ENHA_PARA_MAX
}AUD_ENHA_PARA_TYPE_E;


typedef struct 
{
    uint32_t eqMode;
    int16_t  bandIndex;			  
	int16_t  bandBoost; 
}AUD_ENHA_EXP_BAND_INFO_T; 

typedef struct 
{
    uint32_t eqMode; 
    int16_t  bandNum;
	int16_t  bandBoost[EQ_BAND_MAX]; 
}AUD_ENHA_EXP_ALLBANDS_INFO_T;

typedef struct 
{
    uint32_t  eqSetIndex;		  
	uint32_t  eqMode; 
}AUD_ENHA_EXP_EQ_MODE_SET_T;

union Aud_enha_para_union
{
    uint32_t eqMode;
    uint32_t digitalGain;                
    AUD_ENHA_EXP_EQ_MODE_SET_T eqSetInfo;
    char *devModeName;
    AUD_ENHA_EXP_BAND_INFO_T bandInfo;
    AUD_ENHA_EXP_ALLBANDS_INFO_T modeInfo;
};

typedef struct 
{
    AUD_ENHA_PARA_TYPE_E eParaType;			  
	union Aud_enha_para_union unAudProcPara; 
}AUD_ENHA_EXP_PARA_T;

//---------------------------control params in nv ----------------------//
//untunable eq

//typedef struct 
//{
//    int16_t   fo ;  /*f0*/
//    int16_t   q;    /*q*/   
//    int16_t   boostdB;   /*boost */
//    int16_t   gaindB ;      /*gain*/
//}EQ_BAND_INPUT_PARA_T;

//typedef struct 
//{
 //   int16_t   agc_in_gain;  /*agc in gain set*/
 //   int16_t   band_control; /*bit15-bit8 :filter_sw_1~8 ; bit 1: high shelve;bit0:low shelve */
//    EQ_BAND_INPUT_PARA_T  eq_band[EQ_BAND_MAX];   
//}EQ_MODE_PARA_T;  

//typedef struct //PACKED  272 words
//{
//    uint8_t   para_name[NAME_LEN_MAX];/*struct name*/ 
//    uint16_t  eq_control;//bit15:8-bands-sw
//    EQ_MODE_PARA_T eq_modes[EQ_MODE_MAX];     /*eq mode para*/
//    int16_t externdArray[59]; /*reserved for future*/
//}AUDIO_ENHA_EQ_STRUCT_T;

//tunable eq
typedef struct 
{
    int16_t   agc_in_gain;  /*agc in gain set*/
    int16_t   band_control;
    int16_t   boostdB_default[EQ_BAND_MAX];  /*default boost dB for each band*/
    int16_t   boostdB_current[EQ_BAND_MAX];  /*current boost dB for each band;set by mmi*/
}TUNABLE_EQ_MODE_PARA_T;

typedef struct //PACKED 188 words
{
    uint8_t   para_name[NAME_LEN_MAX];/*struct name*/ 
    int16_t   eq_control;       /* bit15:8-bands-sw;bit9-bit0:level_step;*/
    int16_t   fo_array[EQ_BAND_MAX];
    int16_t   q_array[EQ_BAND_MAX];
    int16_t   level_n;
    TUNABLE_EQ_MODE_PARA_T eq_modes[EQ_MODE_MAX];     /*eq mode para*/
    int16_t externdArray[54]; /*reserved for future*/
}AUDIO_ENHA_TUNABLE_EQ_STRUCT_T;

/**---------------------------------------------------------------------------**
 **                         Global Variables                                  **
 **---------------------------------------------------------------------------**/

/**---------------------------------------------------------------------------**
 **                        Function Declare                               **
 **---------------------------------------------------------------------------**/
/**---------------------------------------------------------------------------**
 **                         Compiler Flag                                     **
 **---------------------------------------------------------------------------**/ 
#ifdef __cplusplus
}
#endif 



#endif //end of _AUD_ENHA_EXP_H_

//end of file



