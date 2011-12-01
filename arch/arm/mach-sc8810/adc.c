#include <mach/hardware.h>
#include <mach/regs_adc.h>
#include <mach/regs_adi.h>
#include <mach/regs_ana.h>
#include <mach/adc_drvapi.h>
#include <mach/adi_hal_internal.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <mach/test.h>
#include <linux/delay.h>
#include <linux/bug.h>

void ADC_Init(void)
{
//TODO::
	//ANA_REG_OR(ANA_AGEN, (BIT_5|BIT_13|BIT_14));
	ANA_REG_OR(ANA_AGEN, AGEN_ADC_EN | ACLK_CTL_AUXAD_EN | ACLK_CTL_AUXADC_EN);
	ANA_REG_OR(ADC_CTRL, ADC_EN_BIT);
}

static void adc_channel_valid(enum adc_channel id)
{
	BUG_ON (id >= ADC_MAX || id == ADC_CHANNEL_INVALID);		
}

void ADC_SetCs(enum adc_channel id)
{
	adc_channel_valid(id);
	ANA_REG_MSK_OR(ADC_CS, id, ADC_CS_BIT_MSK);
}

void ADC_SetScale(bool scale)
{
	if (ADC_SCALE_1V2 == scale) {
		ANA_REG_AND(ADC_CS, ~ADC_SCALE_BIT);
	} else if (ADC_SCALE_3V == scale) {
		ANA_REG_OR(ADC_CS, ADC_SCALE_BIT);
	} else
		pr_err("adc scale %d not support\n", scale);
}

void ADC_ConfigTPC(uint8_t x, uint8_t y)
{
	adc_channel_valid(x);
	adc_channel_valid(y);

	ANA_REG_MSK_OR(ADC_TPC_CH_CTRL, x | y << ADC_TPC_Y_CH_OFFSET,
		       ADC_TPC_X_CH_MSK | ADC_TPC_Y_CH_MSK);
}

uint32_t ADC_GetValue(enum  adc_channel id, bool scale)
{
	uint32_t result;
	unsigned long irq_flag;
	uint32_t count;
	hw_local_irq_save(irq_flag);

	// clear int 
	ANA_REG_OR(ADC_INT_CLR, ADC_IRQ_CLR_BIT);

	//choose channel
	ADC_SetCs(id);

	//set ADC scale
	ADC_SetScale(scale);

	//run ADC soft channel
	ANA_REG_OR(ADC_CTRL, SW_CH_ON_BIT);

    count = 12;

    //wait adc complete
    while(!(ANA_REG_GET(ADC_INT_SRC)&ADC_IRQ_RAW_BIT) && count){
        udelay(50);
        count--;
    }
    if (count == 0) {
        hw_local_irq_restore(irq_flag);
        pr_warning("WARNING: ADC_GetValue timeout....\n");
        return -EAGAIN;
    }

	result = ANA_REG_GET(ADC_DAT) & ADC_DATA_MSK;	// get adc value
	ANA_REG_AND(ADC_CTRL, ~SW_CH_ON_BIT);	// turn off adc soft channel
	ADC_SetCs(TPC_CHANNEL_X);	// set tpc channel x back
	ANA_REG_OR(ADC_INT_CLR, ADC_IRQ_CLR_BIT);	// clear irq of this time

    hw_local_irq_restore(irq_flag);
    return result;
}

EXPORT_SYMBOL_GPL(ADC_Init);
EXPORT_SYMBOL_GPL(ADC_GetValue);
