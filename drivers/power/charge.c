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
#include <linux/delay.h>

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
extern int32_t get_battery_chg(void);
extern int32_t get_battery_current(int is_usb);

uint16_t  ac_charging_voltage_capacity_table[12][2]={0};
uint16_t usb_charging_voltage_capacity_table[12][2]={0};

uint16_t adc_voltage_table[2][2] ={
    {915, 4200},
    {783, 3600},
};

uint16_t voltage_capacity_table[][2] ={
    {4158,  100},
    {4059,  90},
	{3973,  80},
	{3899,  70},
    {3831,  60},
	{3777,  50},
    {3739,  40},
	{3712,  30},
    {3689,  20},
	{3669,  15},
    {3608,  5},
    {3400,  0},
};

int32_t temp_adc_table[][2] ={
    {  900,      0x4E  },//mv
    {  850,      0x59  },
    {  800,      0x67  },
    {  750,      0x77  },
    {  700,      0x89  },
    {  650,      0x9E  },
    {  600,      0xB8  },
    {  550,      0xD4  },
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
    {   0,       0x2BD},
    {  -50,      0x2E5},
    { -100,      0x308},
    { -150,      0x324},
    { -200,      0x33F},
    { -250,      0x354},
    { -300,      0x364}
};
/*****************************************************************************/
//  Description:    This function is used to set the usb_charging_voltage_capacity_table.
//  Add        :    yuelin.tang
//  Note
/*****************************************************************************/
void CHG_SetUsbAcChgTable(int is_usb ,int32_t vprog)
{
    int32_t Resistance;
    int32_t vol;
    int32_t current;
    uint16_t table_size;
    int pos = 0;
    Resistance = get_battery_chg(); 
    if(vprog){
	current = vprog ;
    }else{
    	current = get_battery_current(is_usb);
    }
    vol = (Resistance * current) / 1000;
    printk("############################################# vol=%d\n",vol);
    table_size = ARRAY_SIZE(voltage_capacity_table);
    for(pos=0 ; pos < table_size ; pos++){
	    if(is_usb){
	            usb_charging_voltage_capacity_table[pos][0] = voltage_capacity_table[pos][0] + vol ;
	            usb_charging_voltage_capacity_table[pos][1] = voltage_capacity_table[pos][1] ;
		    if(vprog && (pos >= 2)) break ;
	    }else{
	            ac_charging_voltage_capacity_table[pos][0]  = voltage_capacity_table[pos][0] + vol ;
	            ac_charging_voltage_capacity_table[pos][1]  = voltage_capacity_table[pos][1] ;
		    if(vprog && (pos >= 2)) break ;
	    }
	}
}
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

	udc_enable();
	mdelay(200);
	ret = gpio_get_value(USB_DM_GPIO);   ///USB DM:GPIO145 in SC8800G2

	udc_phy_down();

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

