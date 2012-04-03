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
#include <mach/hardware.h>
#include <mach/regs_ahb.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <mach/bits.h>
#include <mach/board.h>
#include <mach/usb.h>

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

uint16_t adc_voltage_table[2][2] =
{
    {947, 4200},
    {811, 3600},
};
uint16_t charger_adc_voltage_table[2][2] =
{
    {0x198, 6500},
    {0x170, 5800},
};
uint16_t voltage_capacity_table[][2] = 
{
    {4150,  100},
    {4060,  90},
	{3980,  80},
	{3900,  70},
    {3840,  60},
	{3800,  50},
    {3760,  40},
	{3730,  30},
    {3700,  20},
	{3650,  15},
    {3600,  5},
    {3400,  0},
};

uint16_t ac_charging_voltage_capacity_table[][2]={
    {4210,  100},
	{4150,  70},
    {4010,  60},
	{3970,  50},
    {3930,  40},
	{3900,  30},
    {3870,  20},
    {3820,  15},
	{3770,  5},
    {3250,  0},
};

uint16_t usb_charging_voltage_capacity_table[][2]={
    {4200,  100},
	{4120,  90},
	{4080,  80},
	{4005,  70},
    {3965,  60},
	{3930,  50},
    {3890,  40},
	{3865,  30},
    {3830,  20},
    {3810,  15},
	{3730,  5},
    {3250,  0},
};

int32_t temp_adc_table[][2] = 
{
    {  900,      0x4E  },//mv
    {  850,      0x59  },
    {  800,      0x67  },
    {  750,      0x77  },
    {  700,      0x89  },
    {  650,      0x9E  },
    {  600,      0xB8  },
    {  550,      0xD4 },
    {  500,      0xF3  },
    {  450,      0x119},
    {  400,      0x13F},
    {  350,      0x16B},
    {  300,      0x199},
    {  250,      0x1CA},
    {  200,      0x1FD},
    {  150,      0x22F},
    {  100,      0x260},
    {   50,      0x291},
    {   0,      0x2BD},
    {  -50,      0x2E5},
    { -100,      0x308},
    { -150,      0x324},
    { -200,      0x33F},
    { -250,      0x354},
    { -300,      0x364}
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

uint32_t CHG_GetTempADCResult(void)
{
    uint32_t result = (uint32_t) (-1);
    result = ADC_GetValue(ADC_CHANNEL_TEMP, SCI_FALSE);
    return result;
}

int CHGMNG_AdcvalueToTemp(uint16_t adcvalue)
{
    int table_size = ARRAY_SIZE(temp_adc_table);
    int index;
    int result;
    int first, second;

    for(index = 0; index < table_size; index++){
        if(index == 0 && adcvalue < temp_adc_table[0][1])
          break;
        if(index == table_size - 1 && adcvalue>=temp_adc_table[index][1])
          break;

        if(adcvalue>=temp_adc_table[index][1] && adcvalue<temp_adc_table[index+1][1])
          break;
    }

    if(index == 0){
        first = 0;
        second = 1;
    }else if(index == table_size-1){
        first = table_size - 2;
        second = table_size -1;
    }else{
        first = index;
        second = index +1;
    }

    result = (adcvalue - temp_adc_table[first][1])*(temp_adc_table[first][0] - temp_adc_table[second][0])/(temp_adc_table[first][1] - temp_adc_table[second][1]) + temp_adc_table[first][0];
    return result;
}

uint32_t CHGMNG_AdcvalueToCurrent(uint16_t voltage, uint16_t cur_type)
{
    return (((uint32_t)voltage*cur_type*VOL_DIV_P1)/VOL_TO_CUR_PARAM)/VOL_DIV_P2;
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
#if 0
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
#else
uint16_t CHGMNG_AdcvalueToVoltage (uint16_t adcvalue)
{
    int32_t temp;
    temp = adc_voltage_table[0][1] - adc_voltage_table[1][1];
    temp = temp * (adcvalue - adc_voltage_table[0][0]);
    temp = temp / (adc_voltage_table[0][0] - adc_voltage_table[1][0]);
    return temp + adc_voltage_table[0][1];
}

uint16_t CHGMNG_ChargerAdcvalueToVoltage (uint16_t adcvalue)
{
    int32_t temp;
    temp = charger_adc_voltage_table[0][1] - charger_adc_voltage_table[1][1];
    temp = temp * (adcvalue - charger_adc_voltage_table[0][0]);
    temp = temp / (charger_adc_voltage_table[0][0] - charger_adc_voltage_table[1][0]);
    return temp + charger_adc_voltage_table[0][1];
}
#endif

/*****************************************************************************/
//  Description:    Convert ADCVoltage to percentrum.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
uint32_t CHGMNG_VoltageToPercentum (uint32_t voltage, int is_charging, int update, int is_usb)
{
    uint16_t percentum;
    int32_t temp;
    //find the position
    uint16_t table_size;
    int pos = 0;
    static uint16_t pre_percentum = 0xffff;

    if(update){
        pre_percentum = 0xffff;
        return 0;
    }

	if(is_charging){
		if(is_usb){
			table_size = ARRAY_SIZE(usb_charging_voltage_capacity_table);
			for(pos = table_size-1; pos > 0; pos--){
				if(voltage < usb_charging_voltage_capacity_table[pos][0])
					break;
			}
			if(pos == table_size-1) {
				percentum = 0;
			}else{
				temp = usb_charging_voltage_capacity_table[pos][1]-usb_charging_voltage_capacity_table[pos+1][1];
				temp = temp * (voltage - usb_charging_voltage_capacity_table[pos][0]);
				temp = temp / (usb_charging_voltage_capacity_table[pos][0] - usb_charging_voltage_capacity_table[pos+1][0]);
				temp = temp + usb_charging_voltage_capacity_table[pos][1];
				if(temp > 100)
					temp = 100;
				percentum = temp;
			}
		}else{
			table_size = ARRAY_SIZE(ac_charging_voltage_capacity_table);
			for(pos = table_size-1; pos > 0; pos--){
				if(voltage < ac_charging_voltage_capacity_table[pos][0])
					break;
			}
			if(pos == table_size-1) {
				percentum = 0;
			}else{
				temp = ac_charging_voltage_capacity_table[pos][1]-ac_charging_voltage_capacity_table[pos+1][1];
				temp = temp * (voltage - ac_charging_voltage_capacity_table[pos][0]);
				temp = temp / (ac_charging_voltage_capacity_table[pos][0] - ac_charging_voltage_capacity_table[pos+1][0]);
				temp = temp + ac_charging_voltage_capacity_table[pos][1];
				if(temp > 100)
					temp = 100;
				percentum = temp;
			}
		}

        if(pre_percentum == 0xffff)
          pre_percentum = percentum;
        else if(pre_percentum > percentum)
          percentum = pre_percentum;
        else
          pre_percentum = percentum;

    }else{
        table_size = ARRAY_SIZE(voltage_capacity_table);
        for(pos = 0; pos < table_size-1; pos++){
            if(voltage > voltage_capacity_table[pos][0])
              break;
        }
        if(pos == 0) {
          percentum = 100;
        }else{
            temp = voltage_capacity_table[pos][1]-voltage_capacity_table[pos-1][1];
            temp = temp*(voltage - voltage_capacity_table[pos][0]);
            temp = temp/(voltage_capacity_table[pos][0] - voltage_capacity_table[pos-1][0]);
            temp = temp + voltage_capacity_table[pos][1];
			if(temp < 0)
				temp = 0;
            percentum = temp;
        }

        if(pre_percentum == 0xffff)
          pre_percentum = percentum;
        else if(pre_percentum < percentum)
          percentum = pre_percentum;
        else
          pre_percentum = percentum;

    }

    return percentum;
}

#define CHIP_REG_OR(reg_addr, value)   do{ \
                                                 unsigned long flags;   \
                                                 hw_local_irq_save(flags);  \
                                                 (*(volatile unsigned int *)(reg_addr) |= (unsigned int)(value));   \
                                                 hw_local_irq_restore(flags);   \
                                           }while(0)       
#define CHIP_REG_AND(reg_addr, value)   do{ \
                                                 unsigned long flags;   \
                                                 hw_local_irq_save(flags);  \
                                                 (*(volatile unsigned int *)(reg_addr) &= (unsigned int)(value)); \
                                                 hw_local_irq_restore(flags);   \
                                           }while(0)
#define CHIP_REG_GET(reg_addr)          (*(volatile unsigned int *)(reg_addr))
#define CHIP_REG_SET(reg_addr, value)   do{ \
                                                 unsigned long flags;   \
                                                 hw_local_irq_save(flags);  \
                                                 (*(volatile unsigned int *)(reg_addr)  = (unsigned int)(value)) ;   \
                                                 hw_local_irq_restore(flags);   \
                                           }while(0)

int charger_is_adapter(void)
{
	uint32_t ret;
	volatile uint32_t i;
	unsigned long irq_flag=0;

	udc_enable();
	udc_phy_down();

	local_irq_save(irq_flag);

	CHIP_REG_AND(USB_PHY_CTRL,(~(USB_DM_PULLDOWN_BIT|USB_DP_PULLDOWN_BIT)));

	//Identify USB charger
	CHIP_REG_OR(USB_PHY_CTRL, USB_DM_PULLUP_BIT);
	for(i = 0;i < 200;i++){;}  ///wait
	ret = gpio_get_value(USB_DM_GPIO);   ///USB DM:GPIO145 in SC8800G2
	CHIP_REG_AND(USB_PHY_CTRL,(~USB_DM_PULLUP_BIT));

	//normal charger
	if(ret) // else usb host
	{
		///Identify standard adapter
		CHIP_REG_OR(USB_PHY_CTRL, USB_DM_PULLDOWN_BIT);
		for(i = 0;i < 200;i++){;}  ///wait
		if((gpio_get_value(USB_DM_GPIO)&BIT_1) && (gpio_get_value(USB_DP_GPIO)&BIT_2))
		{
			ret = 1; // adapter
		}
		else
		{
			ret = 1; //non standard adapter
		}
		CHIP_REG_AND(USB_PHY_CTRL,(~USB_DM_PULLDOWN_BIT));
	}

	local_irq_restore(irq_flag);
	udc_disable();
	return ret; 
}
#define VPROG_RESULT_NUM 10
#define VBAT_RESULT_DELAY 10
int32_t CHG_GetVirtualVprog (void)
{
    int i, temp;
    volatile int j;
    int32_t vprog_result[VPROG_RESULT_NUM];

    for (i = 0; i < VPROG_RESULT_NUM; )
    {
        vprog_result[i] =
            ADC_GetValue (ADC_CHANNEL_PROG, false);
        if(vprog_result[i]<0)
          continue;

        i++;
        //loop for delay
        for (j = VBAT_RESULT_DELAY - 1; j >= 0; j--)
        {
            ;
        }
    }

    for (j=1; j<=VPROG_RESULT_NUM-1; j++)
    {
        for (i=0; i<VPROG_RESULT_NUM -j; i++)
        {
            if (vprog_result[i] > vprog_result[i+1])
            {
                temp = vprog_result[i];
                vprog_result[i] = vprog_result[i+1];
                vprog_result[i+1] = temp;
            }
        }
    }

    return vprog_result[VPROG_RESULT_NUM/2];
}
/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}

#endif  // End of charge.c

