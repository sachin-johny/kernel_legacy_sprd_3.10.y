/******************************************************************************
 ** File Name:      charge.c                                                 *
 ** Author:         Benjamin.Wang                                                   *
 ** DATE:           24/11/2004                                                *
 ** Copyright:      2003 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    This file defines the basic charging and power manage operation process.   *
*******************************************************************************

 ******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE                NAME                     DESCRIPTION                               *
 ** 24/11/2004      Benjamin.Wang       Create.                                   *
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 **                         Dependencies                                      *
 **---------------------------------------------------------------------------*/
//#include "sci_types.h"
//#include "sci_api.h"
////#include "os_api.h"
//#include "chip_plf_export.h"
//#include "tasks_id.h"
//#include "tx_api_thumb.h"
//#include "chg_drvapi.h"
//#include "dal_power.h"
//#include "adc_drvapi.h"
//#include "upm_api.h"
#include <mach/chg_drvapi.h>
#include <mach/adc_drvapi.h>
#include <linux/reboot.h>
#include <linux/string.h>

#define SCI_FALSE false
#define SCI_TRUE true
/**---------------------------------------------------------------------------*
 **                         Debugging Flag                                    *
 **---------------------------------------------------------------------------*/
#define DEBUG_CHGMNG

#ifdef DEBUG_CHGMNG
#define CHGMNG_PRINT( _format_string )   printk _format_string
#else
#define CHGMNG_PRINT( _format_string )
#endif

#define SCI_PASSERT( _fmt ) printk _fmt

/**---------------------------------------------------------------------------*
 **                         Global Variables                                  *
 **---------------------------------------------------------------------------*/




static CHGMNG_VBATTABLE_T vbat_table =         //vbat table. It can be calibrated by nveditor.
{
    CHG_VOLTAGE_2V8, // 2.8V
    CHG_VOLTAGE_2V9, // 2.9V
    CHG_VOLTAGE_3V0, // 3.0V
    CHG_VOLTAGE_3V1, // 3.1V
    CHG_VOLTAGE_3V2, // 3.2V
    CHG_VOLTAGE_3V3, // 3.3V
    CHG_VOLTAGE_3V4, // 3.4V
    CHG_VOLTAGE_3V5, // 3.5V
    CHG_VOLTAGE_3V6, // 3.6V
    CHG_VOLTAGE_3V7, // 3.7V
    CHG_VOLTAGE_3V8, // 3.8V
    CHG_VOLTAGE_3V9, // 3.9V
    CHG_VOLTAGE_4V0, // 4.0V
    CHG_VOLTAGE_4V1, // 4.1V
    CHG_VOLTAGE_4V2, // 4.2V
    0xFFFF // 0xffff
};

uint16_t voltage_capacity_table[16] =
{
    0,
    0,
    CHGMNG_VOLTAGE_20,
    20,
    CHGMNG_VOLTAGE_40,
    40,
    CHGMNG_VOLTAGE_60,
    60,
    CHGMNG_VOLTAGE_80,
    80,
    CHGMNG_VOLTAGE_100,
    100,
    0xffff,
    0xffff,
    0xffff,
    0xffff
};
/*CHGMNG_IDLE state use*/

/* recent_message_flag: Record the recent message which has been send before client registes.*/
/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/

extern bool PROD_SetChargeNVParam (CHG_SWITPOINT_E    point);

#ifdef CHARGER_OV_SUPPORT
static void CHGMNG_DetectHandler (uint32_t state);
#define VCHG_DETECT_TIMES        20
#define VCHG_DETECT_VALID_TIMES    10
static uint16_t g_Vcharging_detect_interval = 100;
static uint16_t g_Vcharging_detect_times = 0;
static SCI_TIMER_PTR g_Vcharging_timer = NULL;
static uint32_t g_charge_over_adc = 0;
static uint32_t g_charge_low_adc = 0;
#endif

//end modify by paul for charge manage
/**---------------------------------------------------------------------------*
 **                         Function Definitions                              *
 **---------------------------------------------------------------------------*/
/*****************************************************************************/
//  Description:    This function initialize the Charge manager. Before starting charge
//                      manager, you'd better set the essential parameters according to your need.
//                      Otherwise, it will use the default parameters.
//                      For example,
//                      {
//                          CHR_PARAM_T* chr_param = REFPARAM_GetChargeParam();
//
//                          CHGMNG_SetSwitchVoltage(chr_param->switch_voltage);
//                          CHGMNG_SetRechargeVoltage(chr_param->recharge_voltage);
//                          CHGMNG_SetChargeEndVoltage(chr_param->charge_end_voltage);
//
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_HCHARGING, chr_param->hcharging_endtime);
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_LCHARGING, chr_param->lcharging_endtime);
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_RECHARGING, chr_param->rcharging_endtime);
//
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_HCHARGING, chr_param->hcharging_detect_interval);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_LCHARGING, chr_param->lcharging_detect_interval);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_RECHARGING, chr_param->recharge_detect_interval);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_STOPPING, chr_param->scharging_detect_interval);
//
//                          CHGMNG_SetHWSwitchPoint((CHG_SWITPOINT_E)chr_param->hw_switch_point);
//
//                          CHGMNG_SetWarningVoltage(chr_param->voltage_warning);
//                          CHGMNG_SetShutdownVoltage(chr_param->voltage_shutdown);
//                          CHGMNG_SetWarningCount(chr_param->warning_count);
//
//                          CHGMNG_SetVBatTable((CHGMNG_VBATTABLE_T*)chr_param->adc_voltage_table);
//
//                          CHGMNG_SetChargeEndTime((CHGMNG_STATE_E)CHGMNG_PULSECHARGING, chr_param->pcharging_endtime);
//                          CHGMNG_SetTimerInterval((CHGMNG_STATE_E)CHGMNG_PULSECHARGING, chr_param->pcharge_detect_interval);
//                          CHGMNG_SetPulseVoltage(chr_param->pulse_charge_voltage);
//                          CHGMNG_SetPulseWide(chr_param->pulse_wide);
//                          CHGMNG_SetPulseWaitingTime(chr_param->pulse_waiting_time);
//
//                          CHGMNG_SetTestFlag(chr_param->charge_test_flag);
//                     }
//  Return:         CHGMNG_STARTSUCCESS - normal
//                      CHGMNG_STARTINCHARGING - t is charging when CHGMNG_Start
//  Author:         Benjamin.Wang
//  Note:           This module should be initialized after GPIO_Init.
/*****************************************************************************/

/*****************************************************************************/
//  Description:    This function is used to get the result of Vprog ADC.
//                  Return: the Vprog value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint32_t CHG_GetVprogADCResult (void)
{
    uint32_t   result = (uint32_t) (-1);

    result = ADC_GetValue(ADC_CHANNEL_PROG, SCI_FALSE);

    return result;
}

/*****************************************************************************/
//  Description:    This function is used to convert voltage value to ADC value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_VoltageToAdcvalue (uint16_t votage)
{
    uint32_t adcvalue = 0;

    if (votage < 2800)
    {
        SCI_PASSERT (("Voltage Setting is too low!"));
    }
    else if (votage <= 4200)
    {
        adcvalue = ( (uint16_t *) (&vbat_table)) [votage / 100 - 28] + (uint32_t) (votage % 100) * (vbat_table.vbat_4_2-vbat_table.vbat_2_8) /1400;
    }
    else
    {
        adcvalue = vbat_table.vbat_4_2 + (uint32_t) (votage - 4200) * (vbat_table.vbat_4_2-vbat_table.vbat_2_8) /1400;
    }

    return (uint16_t) (adcvalue);
}

/*****************************************************************************/
//  Description:    This function is used to convert ADC value to voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint16_t CHGMNG_AdcvalueToVoltage (uint32_t adcvalue)
{
    uint16_t i;
    uint16_t voltage = 0;
    i = 0;

    while ( ( (uint16_t *) (&vbat_table)) [i] != 0xFFFF)
    {
        if (adcvalue <= ( (uint16_t *) (&vbat_table)) [i])
        {
            voltage = 2800 + i * 100 - ( ( (uint16_t *) (&vbat_table)) [i] - adcvalue) * (4200-2800) / (vbat_table.vbat_4_2-vbat_table.vbat_2_8);
            break;
        }

        i++;
    }

    if (adcvalue > vbat_table.vbat_4_2)
    {
        voltage = 4200 + (adcvalue - vbat_table.vbat_4_2) * 100 / 20;
    }

    return voltage;
}

#define VOL_BUF_SIZE 10
uint32_t vol_buf[VOL_BUF_SIZE];
void put_vol_value(uint32_t voltage)
{
    int i;
    for(i=0;i<VOL_BUF_SIZE -1;i++){
        vol_buf[i] = vol_buf[i+1];
    }
    
    vol_buf[VOL_BUF_SIZE-1] = voltage;
}

uint32_t get_vol_value(void)
{
    unsigned long sum=0;
    int i;
    for(i=0; i < VOL_BUF_SIZE; i++)
      sum += vol_buf[i];

    return sum/VOL_BUF_SIZE;
}
/*****************************************************************************/
//  Description:    Convert ADCVoltage to percentrum.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
uint32_t CHGMNG_VoltageToPercentum (uint32_t voltage)
{
    /*-------------|---sL2--|---s24--|--------|---s24--|---s46--|--------|---s46--|---s68--|--------|---s68--|--s80---|--------|---s80--|--sG10--|-------------*/
    /*-------0%----|-----------------|---20%--|-----------------|---40%--|-----------------|---60%--|-----------------|--80%---|-----------------|----100%-----*/
    /*---------------------V2--------------------------V4-------------------------V6-------------------------V8-------------------------V10--------------------*/

    static uint32_t percentum = 0;
    uint16_t sl2,s24,s46,s68,s80,sg10;

    sl2 = 0;
    s24 = (3<= (voltage_capacity_table[4]-voltage_capacity_table[2])) ? ( (voltage_capacity_table[4]-voltage_capacity_table[2]) /3) :0;
    s46 = (3<= (voltage_capacity_table[6]-voltage_capacity_table[4])) ? ( (voltage_capacity_table[6]-voltage_capacity_table[4]) /3) :0;
    s68 = (3<= (voltage_capacity_table[8]-voltage_capacity_table[6])) ? ( (voltage_capacity_table[8]-voltage_capacity_table[6]) /3) :0;
    s80 = (3<= (voltage_capacity_table[10]-voltage_capacity_table[8])) ? ( (voltage_capacity_table[10]-voltage_capacity_table[8]) /3) :0;
    sg10 = 0;

    put_vol_value(voltage);
    voltage = get_vol_value();

    if (
        (
            ! (
                ( (voltage <voltage_capacity_table[10]+sg10) && (voltage >voltage_capacity_table[10]-s80))
                || ( (voltage <voltage_capacity_table[8]+s80) && (voltage >voltage_capacity_table[8]-s68))
                || ( (voltage <voltage_capacity_table[6]+s68) && (voltage >voltage_capacity_table[6]-s46))
                || ( (voltage <voltage_capacity_table[4]+s46) && (voltage >voltage_capacity_table[4]-s24))
                || ( (voltage <voltage_capacity_table[2]+s24) && (voltage >voltage_capacity_table[2]-sl2))
            )
        )
        || (0 == percentum)   //&& 0
    )
    {
        if (voltage >= voltage_capacity_table[10])
        {
            percentum = 100;
        }
        else if (voltage >= voltage_capacity_table[8])
        {
            percentum = 80;
        }
        else if (voltage >= voltage_capacity_table[6])
        {
            percentum = 60;
        }
        else if (voltage >= voltage_capacity_table[4])
        {
            percentum = 40;
        }
        else if (voltage >= voltage_capacity_table[2])
        {
            percentum = 20;
        }
        else
        {
            percentum = 0;
        }
    }

    /*
    CHGMNG_PRINT(("CHGMNG:%d,s24:%d",voltage_capacity_table[2],s24));
    CHGMNG_PRINT(("CHGMNG:%d,s46:%d",voltage_capacity_table[4],s46));
    CHGMNG_PRINT(("CHGMNG:%d,s68:%d",voltage_capacity_table[6],s68));
    CHGMNG_PRINT(("CHGMNG:%d,s80:%d",voltage_capacity_table[8],s80));
    CHGMNG_PRINT(("CHGMNG:%d",voltage_capacity_table[10]));
    CHGMNG_PRINT(("CHGMNG:voltage = %d,percentum = %d",voltage,percentum));
    */
    return percentum;
}

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}

#endif  // End of charge.c

