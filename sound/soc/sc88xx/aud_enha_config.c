/******************************************************************************
 ** File Name:      aud_proc_config.c                                                     *
 ** Author:         Cherry.Liu                                                *
 ** DATE:           04/15/2010                                                *
 ** Copyright:      2010 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:    This file serves as audio process module.             * 
 **                                                                           *
 ******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 04/15/2010     Cherry.Liu       Create.                                   *
 ******************************************************************************/  
/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/ 

#include "filter_calc.h"
#include "aud_enha.h"
#include <linux/delay.h>
#include "sci_types.h"






/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif
/**---------------------------------------------------------------------------*
 **                         Macro Definition                                  *
 **---------------------------------------------------------------------------*/
#define AUD_EHA_DEBUG


#define HPF_FADE_OUT_TOTAL_TIME 10      //ms  range:25-50
#define HPF_FADE_OUT_GAIN_SET_TIMES 10
#define HPF_FADE_IN_TOTAL_TIME 10      //ms  range:25-50
#define HPF_FADE_IN_GAIN_SET_TIMES 10

#define BAND_FADE_OUT_TOTAL_TIME 8      //ms  range:25-50
#define BAND_FADE_OUT_GAIN_SET_TIMES 8
#define BAND_FADE_IN_TOTAL_TIME 8      //ms  range:25-50
#define BAND_FADE_IN_GAIN_SET_TIMES 8


/**---------------------------------------------------------------------------*
 **                         Local function definitions                       *
 **---------------------------------------------------------------------------*/
/*****************************************************************************/
//  Description:    fade out all of hpf filters to zeros
//  Author:         Cherry.Liu
//  Note:             
//****************************************************************************/  
LOCAL void AUDENHA_FadeOut(
    HPF_GAIN_PARAM_T* ori_gain_ptr,
    int32_t fade_out_total_time,//unit:ms
    int32_t fade_out_set_times
)
{
    int32_t i = 0,j = 0;
    int32_t step_value[HPF_S_GAIN_NUM] = {0} ,step_time = 0;

    //prepare step value and step time for each filter
    for(i=0;i<HPF_S_GAIN_NUM;i++)//S0  ~ S5
    {
        step_value[i]   = ori_gain_ptr->s[i]/fade_out_set_times;
    }
    step_time    = fade_out_total_time/fade_out_set_times;
    
    //fade out step by step
    for(j=1;j<=fade_out_set_times;j++)
    {
        for(i=0;i<HPF_S_GAIN_NUM;i++)//S0  ~ S5
        {
            VB_SetHpfGain(i, (ori_gain_ptr->s[i]-j*step_value[i]));  
        }

        msleep(step_time);  //ms
    }

    //set all gain to 0 finally
    for(i=0;i<HPF_S_GAIN_NUM;i++)//S0  ~ S5
    {
        VB_SetHpfGain(i,0);
    }

    return ;
}

/*****************************************************************************/
//  Description:    fade in hpf filters from zeros
//  Author:         Cherry.Liu
//  Note:            
//****************************************************************************/  
LOCAL void AUDENHA_FadeIn(
    HPF_GAIN_PARAM_T* dest_gain_ptr,
    int32_t fade_in_total_time,//unit:ms
    int32_t fade_in_set_times
)
{
    int32_t i = 0,j = 0;
    int32_t step_value[HPF_S_GAIN_NUM] = {0} ,step_time = 0;

    //prepare step value and step time for each filter
    for(i=0;i<HPF_S_GAIN_NUM;i++)//S0  ~ S5
    {
        step_value[i]   = dest_gain_ptr->s[i]/fade_in_set_times;
    }
    step_time    = fade_in_total_time/fade_in_set_times;
    
    for(j=1;j<=fade_in_set_times;j++)
    {     
        for(i=0;i<HPF_S_GAIN_NUM;i++)//S0  ~ S5
        {
            VB_SetHpfGain(i, (j*step_value[i]));  
        }
        
        msleep(step_time);
    }

    //set all gain to dest  gain finally
    for(i=0;i<HPF_S_GAIN_NUM;i++)//S0  ~ S5
    {
        VB_SetHpfGain(i,dest_gain_ptr->s[i]);
    }

    return ;
}

/*****************************************************************************/
//  Description:    set HPF filter A/B paras & gains down when the hpf is enabled:
//                  the first hpf is for lcf;
//                  the second to sixth hp are for 5-band eq;
//
//  Author:         Cherry.Liu
//  Note:            
//****************************************************************************/  
LOCAL BOOLEAN AUDENHA_SetHpf(
    HPF_CONTROL_PARAM_T* hpf_param_ptr,
    int32_t sample_rate
)
{
    uint32_t i  = 0;
    FILTER_LCF_CALC_PARA_T* lcf_param_ptr = &hpf_param_ptr->lcf_para;//in
    HPF_GAIN_PARAM_T*       hpf_gain_ptr  = &hpf_param_ptr->s_gain;
    FILTER_EQ_CALC_PARA_T  eq_input_para = {0};//in
    IIR_FILTER_PARA_T  lcf_filter_set = {0};//out
    IIR_FILTER_PARA_T  eq_filter_set[EQ_BAND_MAX] = {0};//out
    BOOLEAN  return_value = SCI_TRUE;
    HPF_GAIN_PARAM_T ori_hpf_gain = {0};

    
    //check input paras
    SCI_ASSERT(lcf_param_ptr != SCI_NULL);/*assert verified*/
    //SCI_ASSERT(eq_param_ptr != SCI_NULL);
    SCI_ASSERT(sample_rate  != 0);/*assert verified*/

    //the first hpf is for lcf;
    return_value = Filter_CalcLCF(lcf_param_ptr, 16384,
        sample_rate, 
        &lcf_filter_set,
        &hpf_gain_ptr->s[0]);

    if(!return_value)
    {
        printk("aud_eha_config.c,[AUDENHA_SetHpfs] encounters error when caculating filter para! \n");

        //if return error,this filter is set to all-pass filter
        hpf_gain_ptr->s[0]= 4096;
        lcf_filter_set.B0 = 16384;
        lcf_filter_set.B1 = 0;
        lcf_filter_set.B2 = 0;
        lcf_filter_set.A0 = 16384;
        lcf_filter_set.A1 = 0;
        lcf_filter_set.A2 = 0;
    }
    
#ifdef AUD_EHA_DEBUG
    //trace filter paras
    printk("aud_eha_config.c,[AUDENHA_SetHpfs]lcf:lcf_sw:%d  filter_type:%d\n",
        lcf_param_ptr->isFilterOn,lcf_param_ptr->eLcfParaType);
    printk("aud_eha_config.c,[AUDENHA_SetHpfs]lcf:f1_g0:%d  f1_g1:%d  f1_fp:%d  f2_g0:%d f2_g1:%d f2_fp:%d\n",
        lcf_param_ptr->unlcfPara.lcfPara.f1_g0,lcf_param_ptr->unlcfPara.lcfPara.f1_g1,lcf_param_ptr->unlcfPara.lcfPara.f1_fp,lcf_param_ptr->unlcfPara.lcfPara.f2_g0,lcf_param_ptr->unlcfPara.lcfPara.f2_g1,lcf_param_ptr->unlcfPara.lcfPara.f2_fp);
    printk("aud_eha_config.c,[AUDENHA_SetHpfs]lcf:S:%d,B0:%d B1:%d B2:%d; A0:%d A1:%d A2:%d;samplerate:%d\n",
        hpf_gain_ptr->s[0],lcf_filter_set.B0,lcf_filter_set.B1,lcf_filter_set.B2,
        lcf_filter_set.A0,(lcf_filter_set.A1),(lcf_filter_set.A2),sample_rate); 
#endif


    //calc the a/b parameters of each eq band
    for(i=0;i<hpf_param_ptr->eq_band_num;i++)//i :band index
    {
        eq_input_para.isFilterOn = hpf_param_ptr->band_sw[i];

        if((i==0)&&(hpf_param_ptr->low_shelve_on))//the first band
        {
            eq_input_para.eEqParaType      = FILTER_EQPARA_LOW_SHELVE;
            eq_input_para.unEqPara.fo_next = hpf_param_ptr->eq_band_para[i+1].fo;
        }
        else if((i==(hpf_param_ptr->eq_band_num-1))&&(hpf_param_ptr->high_shelve_on))
        {
            eq_input_para.eEqParaType      = FILTER_EQPARA_HIGH_SHELVE;
            eq_input_para.unEqPara.fo_last = hpf_param_ptr->eq_band_para[hpf_param_ptr->eq_band_num-2].fo;

        }
        else
        {
            eq_input_para.eEqParaType  = FILTER_EQPARA_NORMAL_EQ;  
            eq_input_para.unEqPara.q   = hpf_param_ptr->eq_band_para[i].q;
        }
        eq_input_para.fo         = hpf_param_ptr->eq_band_para[i].fo;
        eq_input_para.boostdB    = hpf_param_ptr->eq_band_para[i].boostdB;
        eq_input_para.basegaindB = hpf_param_ptr->eq_band_para[i].gaindB;
        
        return_value = Filter_CalcEQ(&eq_input_para,
            sample_rate, 
            &eq_filter_set[i],
            &hpf_gain_ptr->s[i+1]);

        if(!return_value)
        {
            printk("aud_eha_config.c,[AUDENHA_SetHpfs]band:%d encounters error when caculating filter para! \n",i);

            //if return error,this filter is set to all-pass filter
            hpf_gain_ptr->s[i+1]= 4096;
            eq_filter_set[i].B0 = 16384;
            eq_filter_set[i].B1 = 0;
            eq_filter_set[i].B2 = 0;
            eq_filter_set[i].A0 = 16384;
            eq_filter_set[i].A1 = 0;
            eq_filter_set[i].A2 = 0;
        }
        
#ifdef AUD_EHA_DEBUG
        printk("aud_eha_config.c,[AUDENHA_SetHpfs]band %d:sw:%d,fo:%d  q:%d  boost:%d  gain:%d\n",i,
            eq_input_para.isFilterOn,eq_input_para.fo,eq_input_para.unEqPara.q, eq_input_para.boostdB,eq_input_para.basegaindB );
        
        printk("aud_eha_config.c,[AUDPROC_Seteq]band %d:S:%d,B0:%d B1:%d B2:%d, A0:%d A1:%d A2:%d, samplerate:%d\n",i,
            hpf_gain_ptr->s[i+1],eq_filter_set[i].B0,eq_filter_set[i].B1,eq_filter_set[i].B2,
            eq_filter_set[i].A0,(eq_filter_set[i].A1),(eq_filter_set[i].A2),sample_rate);  
#endif

    }
    
    //****For future*****here if we add smooth algorithm,we need put the gain from smooth method to the S gain of HPF//

   //begin the eq paras setting process
    //fading out------
    for(i=0;i<HPF_S_GAIN_NUM;i++)//S0  ~ S5
    {
        ori_hpf_gain.s[i]= VB_GetHpfGain(i);
    }

    if(0 != ori_hpf_gain.s[HPF_S_GAIN_NUM-1])//S5
    {
        //the music is playing;the gain register is non-zero
        AUDENHA_FadeOut(&ori_hpf_gain,HPF_FADE_OUT_TOTAL_TIME,HPF_FADE_OUT_GAIN_SET_TIMES);
    }

#ifdef  AUD_EHA_DEBUG
    printk("aud_eha_config.c,[AUDENHA_SetHpf]:s0-s6 registers:%d  %d  %d  %d  %d  %d \n",
        ori_hpf_gain.s[0],ori_hpf_gain.s[1],ori_hpf_gain.s[2],ori_hpf_gain.s[3],ori_hpf_gain.s[4],ori_hpf_gain.s[5]);
#endif

    // clear eq  delay register by vbc
    VB_SetHpfMode(1);         

    //clean eq delay register by filter itself 
    for(i=1;i<=HPF_S_GAIN_NUM;i++)//hpf_1 ~ hpf_6
    {
        VB_SetHpfParas(i,0,0,0,0,0,0);
    }
    msleep(2);

    //set filter hpf_1 --- lcf
    VB_SetHpfParas(1,lcf_filter_set.B0, lcf_filter_set.B1, lcf_filter_set.B2, 
        16384, -lcf_filter_set.A1, -lcf_filter_set.A2);


    //set filter hpf_2 ~ hpf_6 --- 5-band eq
    for(i=0;i<hpf_param_ptr->eq_band_num;i++)
    {
        VB_SetHpfParas(i+2, eq_filter_set[i].B0,eq_filter_set[i].B1, eq_filter_set[i].B2, 
            16384,-eq_filter_set[i].A1, -eq_filter_set[i].A2);
    }
    
    // finish clear process ;and the fileters begin working normally
    VB_SetHpfMode(0);   
    
    //fading in------
    AUDENHA_FadeIn(hpf_gain_ptr,HPF_FADE_IN_TOTAL_TIME,HPF_FADE_IN_GAIN_SET_TIMES);

    return SCI_TRUE;   
}

/*****************************************************************************/
//  Description:    set input gain of alc (alc_ingain_Set)
//  Author:         Cherry.Liu
//  Note:           /*do we need fade out&in operation? cherry needs check here*/   
//****************************************************************************/  
LOCAL BOOLEAN AUDENHA_SetAlcIngain(
    ALC_CONTROL_PARAM_T* alc_param_ptr        //alc
)
{
    int32_t  temp = 0;
    int16_t  input_gain  = alc_param_ptr->alc_input_gain; 
    int16_t  in_gain_set = alc_param_ptr->alc_ingain_Set;

    //input gain
    temp = (input_gain * in_gain_set)>>12;
    if(temp<=32767)
    {
        input_gain   = temp;
    }
    else
    {
        input_gain   = 32767;
    }    

    //s6
    VB_SetHpfGain(6, input_gain); 

    return SCI_TRUE;
}

/*****************************************************************************/
//  Description:    enable DG Module
//  Author:         Cherry.Liu
//  Note:            
//****************************************************************************/  
LOCAL int AUDENHA_initDigiGain(
    DG_CONTROL_PARAM_T* dg_param_ptr
    )
{
    //DG gain set
    VB_SetDG (VBC_DA_LEFT,  dg_param_ptr->left_gain);
    VB_SetDG (VBC_DA_RIGHT,  dg_param_ptr->right_gain);


    //DG switch
    VB_DGSwitch(VBC_DA_LEFT, SCI_TRUE); 
    VB_DGSwitch(VBC_DA_RIGHT, SCI_TRUE);
	printk(KERN_DEBUG "CONFIG: AUDENHA_initDigiGain VBC_DA_LEFT=TRUE VBC_DA_RIGHT=SCI_TRUE \n");


    return AUDIO_NO_ERROR;
}

/*****************************************************************************/
//  Description:    init da path
//  Author:         Cherry.Liu
//  Note:            
//****************************************************************************/  
LOCAL BOOLEAN AUDENHA_initDAPath(
    DAPATH_CONTROL_PARAM_T* dapath_param_ptr
    )
{

    VB_SetFMMixMode(VBC_DA_LEFT, dapath_param_ptr->left_fm_mix_mode);
    VB_SetFMMixMode(VBC_DA_RIGHT, dapath_param_ptr->right_fm_mix_mode);

    return SCI_TRUE;
}


/*****************************************************************************/
//  Description:    int alc module and enable it
//  Author:         Cherry.Liu
//  Note:          
//****************************************************************************/  
LOCAL int AUDENHA_initAlc(
    ALC_CONTROL_PARAM_T* alc_param_ptr
)
{
    int32_t   temp = 0;
    int16_t   input_gain = 0; //4096 sacled
    BOOLEAN alc_sw = SCI_FALSE;

    //alc sw
    alc_sw      = alc_param_ptr->alc_sw;

    //calc input gain
    temp = (alc_param_ptr->alc_input_gain * alc_param_ptr->alc_ingain_Set)>>12;
    if(temp<=32767)
    {
        input_gain   = temp;
    }
    else
    {
        input_gain   = 32767;
    }    

    //set the input gain to s6
    VB_SetHpfGain((TOTAL_S_GAIN_NUM-1), input_gain); 


    if(alc_sw)
    {
        //alc para
        VB_SetALCParas(&alc_param_ptr->alc_Para);

        //enable alc
        VB_ALCSwitch(SCI_TRUE);
    }
    else
    {
        //disenable alc
        VB_ALCSwitch(SCI_FALSE);
    }
    
    return AUDIO_NO_ERROR;
}


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
)
{
    BOOLEAN return_value = SCI_FALSE;

    printk("aud_eha_config.c,[AUDENHA_InitHpf]  HPF is on! \n");

    VB_SetHpfLimit((int8_t)(hpf_param_ptr->r_limit)); //RLimit:bit7~0
    VB_SetHpfWidth(hpf_param_ptr->data_width);       //IIS_Bits_select  
    VB_SwitchHpf(SCI_TRUE);          //HPF ENABLE


    return_value = AUDENHA_SetHpf(hpf_param_ptr, 
        sample_rate);
    
    if(!return_value)
    {
        printk("aud_eha_config.c,[AUDENHA_InitHpf]  set eq failed! \n");
        return AUDIO_ERROR;   
    }

    return AUDIO_NO_ERROR;
}

/**---------------------------------------------------------------------------*
 **                         Public function definitions                       *
 **---------------------------------------------------------------------------*/
/*****************************************************************************/
//  Description:    set eq mode para for audio process modules, including lcf,alc and eq.
//  Author:         Cherry.Liu
//  Note:             
//****************************************************************************/  
PUBLIC int AUDENHA_SetEqMode(
    ALC_CONTROL_PARAM_T* alc_param_ptr,        //alc
    HPF_CONTROL_PARAM_T* hpf_param_ptr,        //hpf
    int32_t sample_rate                         //fs
)
{
    BOOLEAN set_return_value = SCI_FALSE;

    //set agc in gain set
    AUDENHA_SetAlcIngain(alc_param_ptr);

    //set lcf & eq (hpf_1 ~hpf_6)
    set_return_value = AUDENHA_SetHpf(hpf_param_ptr, 
        sample_rate);
    
    if(!set_return_value)
    {
        printk("aud_eha_config.c,[AUDENHA_SetEqMode]  set hpf failed! \n");
        return AUDIO_ERROR;   
    }

    return AUDIO_NO_ERROR;
    
}

/*****************************************************************************/
//  Description:    set digital gain 
//  Author:         Cherry.Liu
//  Note:           dg gain here is mapped to arm volume[i] bit15 ~ bit9
//****************************************************************************/  
PUBLIC BOOLEAN AUDENHA_SetDigiGain(
    DG_CONTROL_PARAM_T* dg_param_ptr
)
{

    VB_SetDG (VBC_DA_LEFT,dg_param_ptr->left_gain);
    VB_SetDG (VBC_DA_RIGHT,dg_param_ptr->right_gain);
    
    return SCI_TRUE;
}

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
    int32_t sample_rate                        //fs
)
{
    BOOLEAN init_return_value = SCI_FALSE;

    //digital gain
    AUDENHA_initDigiGain(dg_param_ptr);
	printk("AUDENHA_Init:AUDENHA_initDigiGain left_gain=%d right_gain=%d \n",dg_param_ptr->left_gain,dg_param_ptr->right_gain);
    //da path control
    AUDENHA_initDAPath(dapath_param_ptr);
	printk("AUDENHA_Init:AUDENHA_initDAPath left_fm_mix_mode=%d right_fm_mix_mode=%d \n",dapath_param_ptr->left_fm_mix_mode,dapath_param_ptr->right_fm_mix_mode);

    //alc init
    AUDENHA_initAlc(alc_param_ptr);
//	printk("AUDENHA_Init:AUDENHA_initAlc \n",alc_param_ptr->alc_sw,alc_param_ptr->alc_input_gain,alc_param_ptr->alc_ingain_Set,alc_param_ptr->alc_Para.);

    //hpf init
    init_return_value = AUDENHA_InitHpf(hpf_param_ptr,sample_rate);
    if(AUDIO_NO_ERROR != init_return_value)
    {
        printk("aud_eha_config.c,[AUDENHA_Init] Hpf init failed! \n");
        return AUDIO_ERROR;
    }

    return AUDIO_NO_ERROR;
}

/*****************************************************************************/
//  Description:    deinti aud proc plugger
//  Author:         Cherry.Liu
//  Note:           
//****************************************************************************/  
PUBLIC BOOLEAN AUDENHA_DeInit(
    void
)
{
   
    int32_t i=0;
    
    printk("aud_eha_config.c,[AUDENHA_InitHpf]  eq is off! \n");

    //set s0  ~ s6 to zero
    for(i=0;i<TOTAL_S_GAIN_NUM;i++)
    {
        VB_SetHpfGain(i, 0);
    }
    
    //clean delay register
    VB_SetHpfMode(SCI_TRUE);         // clear hpf delay register 
    msleep(SCI_TRUE);             // the clear process need at least 10us 
    VB_SetHpfMode(SCI_FALSE);         // finish clear process

    //shut down hpf(including lcf eq alc)
    VB_SwitchHpf(SCI_FALSE); 
    VB_ALCSwitch(SCI_FALSE);
    VB_DGSwitch(VBC_DA_LEFT, SCI_FALSE); 
    VB_DGSwitch(VBC_DA_RIGHT, SCI_FALSE);
    
    return SCI_TRUE;

}

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
    
#endif  // End of aud_enha_config.c


