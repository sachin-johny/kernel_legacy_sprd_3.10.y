#ifndef _ADC_DRVAPI_H_
#define _ADC_DRVAPI_H_

#define ADC_SCALE_3V       0 
#define ADC_SCALE_1V2   1

#define ADC_CHANNEL_INVALID  0xffff

enum adc_channel {
    ADIN_0 = 0,
    ADIN_1 = 1,
    ADIN_2 = 2,
    ADIN_3 = 3,
    ADIN_4 = 4,
    ADIN_5 = 5,
    ADIN_6 = 6,
    ADIN_7 = 7,
    ADIN_8 = 8,
    ADIN_9 = ADC_CHANNEL_INVALID,
    ADIN_10 = ADC_CHANNEL_INVALID,
    ADIN_11 = ADC_CHANNEL_INVALID,
    ADIN_12 = ADC_CHANNEL_INVALID,
    ADIN_13 = ADC_CHANNEL_INVALID,
    ADIN_14 = 14,
    ADIN_15 = 15,
    ADC_MAX = 16,
};

#ifdef CONFIG_MACH_SP6810A
#define ADC_CHANNEL_TEMP 0
#else
#define ADC_CHANNEL_TEMP 1
#endif
#define ADC_CHANNEL_VBAT 5
#define ADC_CHANNEL_PROG 4
#define ADC_CHANNEL_VCHG 6

void ADC_Init (void);
uint32_t ADC_GetValue(enum  adc_channel id, bool scale);


#endif


