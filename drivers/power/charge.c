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
uint16_t adc_voltage_table[2][2] =
{
    {928, 4200},
    {796, 3600},
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

/*****************************************************************************/
//  Description:    This function is used to get the result of Vprog ADC.
//                  Return: the Vprog value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
 uint32_t CHG_GetVprogADCResult (void)
{
    uint32_t   result = (uint32_t) (-1);

    result = ADC_GetValue(ADC_CHANNEL_PROG, false);

    return result;
}

uint32_t CHG_GetTempADCResult(void)
{
    uint32_t result = (uint32_t) (-1);
    result = ADC_GetValue(ADC_CHANNEL_TEMP, false);
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

uint32_t CHGMNG_AdcvalueToCurrent(uint16_t vprog_adc, uint16_t cur_type)
{
    uint16_t voltage = CHGMNG_AdcvalueToVoltage(vprog_adc);
    return (((uint32_t)voltage*cur_type*VBAT_VOL_DIV_P1)/VOL_TO_CUR_PARAM)/VBAT_VOL_DIV_P2;
}

/*****************************************************************************/
//  Description:    This function is used to convert voltage value to ADC value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
uint16_t CHGMNG_VoltageToAdcvalue (uint16_t voltage)
{
    int32_t temp;
    temp = voltage - adc_voltage_table[0][1];
    temp = temp * (adc_voltage_table[1][0] - adc_voltage_table[0][0]);
    temp = temp / (adc_voltage_table[1][1] - adc_voltage_table[0][1]);
    temp = temp + adc_voltage_table[0][0];
    return temp;
}

/*****************************************************************************/
//  Description:    This function is used to convert ADC value to voltage value.
//  Author:         Benjamin.Wang
//  Note:
/*****************************************************************************/
uint16_t CHGMNG_AdcvalueToVoltage (uint16_t adcvalue)
{
    int32_t temp;
    temp = adc_voltage_table[0][1] - adc_voltage_table[1][1];
    temp = temp * (adcvalue - adc_voltage_table[0][0]);
    temp = temp / (adc_voltage_table[0][0] - adc_voltage_table[1][0]);
    return temp + adc_voltage_table[0][1];
}
uint16_t CHGMNG_ChargerAdcvalueToVoltage(uint16_t adc)
{
    uint32_t result;
    uint32_t vbat_vol = CHGMNG_AdcvalueToVoltage (adc);
    uint32_t m,n;
    
    ///v1 = vbat_vol*0.268 = vol_bat_m * r2 /(r1+r2)
    n = VBAT_VOL_DIV_P2*VCHG_DIV_P1;
    m = vbat_vol*VBAT_VOL_DIV_P1*(VCHG_DIV_P2);
    result = (m + n/2)/n;
    return result;
}

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

